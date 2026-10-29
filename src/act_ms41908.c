// act_ms41908 — the Panasonic MS41908M (AN41908A clone) SPI lens stepper.
//
// The Xiongmai HI3516D_N81820 boards (HiSilicon Hi3516A V100) drive zoom/focus
// with an MS41908M wired straight to the SoC — SPI1 (/dev/spidev1.0) + PL061 GPIO
// (EN chip-select, VD_FZ move latch) — with no UART lens MCU. The bus/register
// map, SPI1 pinmux+clock, interrupt setup and per-move handshake are the ones the
// OpenIPC/motors `ms41908-lens` tool proved on hardware, ported here verbatim.
//
// Two things make a stepper fit behind motion.c's Pelco-shaped policy:
//
//   1. motion.c drives a lens as *continuous* travel (drive(+1) ... drive(0)). So
//      emit() does not step — it just sets a direction — and a stepping thread
//      issues micro-step bursts at a fixed cadence while a direction is held. Steps
//      then track time linearly. (The autofocus search on this lens is af3, which
//      steps exactly through focus_step(); see focus_steps below.)
//
//   2. The MS41908M reports no magnification (there is no MCU). But WE command the
//      zoom stepper, so we dead-reckon the zoom position and derive magnification
//      from it, pushing it through af_zoom_report() exactly where the UART reader
//      would.
//
// CALIBRATION: the zoom->magnification curve (ms_zoom_mag) and the focus mechanics
// below are best-effort for LENS_LH13_FHD_X16 and are finalised by an on-hardware
// pass. The soft travel limits are the measured libxmaf values and are exact.
//
// The motor only STEPS while the ISP pipeline is producing VD timing, so majestic
// must be streaming for motion — SPI register access (init) needs no streamer.

#include "act_ms41908.h"
#include "actuator.h"
#include "motion.h"

#include <majestic/af_plugin_abi.h>

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

#include <majestic/log.h>

Actuator act_ms41908;   // the vtable, at the end: ms_lens_from_config sets its focus travel

// musl's <sys/ioctl.h> does not expose _IOC_SIZEBITS, which <linux/spi/spidev.h>'s
// SPI_IOC_MESSAGE() references; the kernel value is 14. glibc/uClibc define it, so
// this only fills the musl gap (the OpenIPC target toolchain).
#ifndef _IOC_SIZEBITS
#define _IOC_SIZEBITS 14
#endif

// engine.c: the magnification sink. A UART lens reaches it via the reader thread;
// we reach it directly from the stepping thread as the zoom position changes.
void af_zoom_report(float mag);

// ---- device / register map (from the ms41908-lens tool) --------------------

#define MS_SPIDEV "/dev/spidev1.0"
#define MS_SPI_MODE (SPI_LSB_FIRST | SPI_CS_HIGH) /* 0x0C, matches libxmaf */
#define MS_SPI_HZ 5000000

#define GPIO_BASE(g) (0x20140000u + (unsigned)(g) * 0x10000u)
#define GPIO_DIR(g) (GPIO_BASE(g) + 0x400u)
#define EN_GRP 8
#define EN_PIN 7 /* xmspi_enable — HIGH during each SPI transfer */
#define VD_GRP 10
#define VD_PIN 5 /* VD_FZ move latch/trigger */
#define PIF_GRP 0
#define PIF_PIN 3 /* focus photo-interrupter (input) */
#define PIZ_GRP 0
#define PIZ_PIN 4 /* zoom photo-interrupter (input) */

#define INTR_STAT 0x20220414u
#define INTR_CLR 0x2022041Cu
#define ISR_ZOOM 1
#define ISR_FOCUS 0

#define CTRL_BASE 0x0400
#define REG_ZOOM 0x24
#define REG_FOCUS 0x29

// The stepping cadence (MS_STEP_BURST / MS_BURST_MS) and the derived focus
// mechanics (MS_TRAVEL_MS / MS_TRAVEL_MAX_MS / MS_BACKLASH_MS) live in
// act_ms41908.h so the unit test pins the derivation.

// The pure helpers (ms_verb_axis / ms_clamp_step / ms_zoom_mag) live in
// ms41908_calc.c, declared in act_ms41908.h, so the unit test can pin them
// without this file's SPI / /dev/mem / thread / log dependencies.

// ---- transport state -------------------------------------------------------

static pthread_mutex_t s_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_cv = PTHREAD_COND_INITIALIZER;
static pthread_t s_thread;
static bool s_thread_valid = false;
static volatile int s_run = 0;
static int s_focus_dir = 0;   // -1 near, +1 far, 0 stop
static int s_zoom_dir = 0;    // -1 wide, +1 tele, 0 stop
static int s_focus_pos = 0;   // microsteps from the near stop, [0, MS_FOCUS_MAX]
static int s_zoom_pos = 0;    // microsteps from the wide stop, [0, MS_ZOOM_MAX]
// The lens has no absolute position sensor, so a software counter seeded at 0 is
// NOT a physical reference until an axis has been driven onto its home (near/wide)
// stop. Until then the toward-0 soft limit is suppressed (so the homing seek runs)
// and zoom magnification is not published (its origin is unknown). s_*_run counts
// microsteps driven toward 0 in one continuous sweep; a full travel's worth means
// the axis is at its stop.
static bool s_focus_homed = false;
static bool s_zoom_homed = false;
static int s_focus_run = 0;
static int s_zoom_run = 0;
// A magnification restored from the last run (ms_seed_zoom), or <0 for none. The
// zoom stepper holds position unpowered, so this IS where the lens still sits: it
// seeds s_zoom_pos and marks zoom homed WITHOUT a physical seek, so the step
// thread reports magnification from the first move instead of leaving /zoom stale
// until the operator drives fully wide. Applied under s_mu on every open (the
// transport can reopen on a retry), and immediately if the thread is already up.
static float s_zoom_seed_mag = -1.0f;

// The lens on this chip, read at open (ms_lens_from_config): the LH13 16x lens's
// figures unless ptz.lens* say otherwise. s_swap puts zoom on the focus motor and
// focus on the zoom motor, for a lens wired the other way round to the chip.
static MsLens s_lens = MS_LENS_DEFAULT;
static bool s_swap = false;
// Homing at open (ptz.lensHome), for a lens whose photo-interrupters read nothing
// useful, so that no position is ever a guess: 1 = ramming zoom into its home
// stop, 2 = ramming focus into its near stop, 3 = taking zoom back to where the
// last run left it (s_zoom_target), 4 = then focus (s_focus_target), 0 = none or
// done. Focus goes back last because a zoom displaces it.
static int s_home_phase = 0;
static int s_zoom_target = -1;
static int s_focus_target = -1;
// The focus position the engine restored for this boot (ms_seed_focus), kept like
// s_zoom_seed_mag: the engine sends it once, and the lens may open only on a later
// retry, so every open takes it from here. <0 for none.
static long s_focus_seed = -1;
static bool s_lens_home = false;   // ptz.lensHome, as read at the last open
// The positions last logged, so a move's end is logged once (ms_log_positions).
static int s_logged_zoom = -1, s_logged_focus = -1;
static bool s_moved_manual = false;   // an operator's move stepped since the last log

// Synchronous precise-move channel, for af3's step-based focus search. emit() must never block
// (it is called under motion.c's policy lock), so the SPI work stays on the stepping thread: a
// caller (the engine worker running af3) posts a request here, signals the thread, and waits on
// s_done for the result — the thread owns the bus, the caller only blocks itself. kind: 0 = none,
// 1 = move exactly n focus microsteps in s_req_dir, 2 = home focus (ram to the near stop). abort
// is raised by any emit() so a manual/zoom command preempts a precise move promptly.
static pthread_cond_t s_done = PTHREAD_COND_INITIALIZER;
static int s_req_kind = 0;
static int s_req_dir = 0;
static int s_req_n = 0;
static int s_req_moved = 0;
static volatile int s_req_abort = 0;

static int spi_fd = -1;
static int mem_fd = -1;

// ---- /dev/mem MMIO (page-cached), from the tool ----------------------------

#define MAX_PAGES 16
static struct { uint32_t base; volatile uint8_t *p; } pages[MAX_PAGES];

static volatile uint32_t *reg(uint32_t phys) {
    uint32_t base = phys & ~0xFFFu;
    int i;
    for (i = 0; i < MAX_PAGES && pages[i].p; i++) {
        if (pages[i].base == base) {
            return (volatile uint32_t *)(pages[i].p + (phys & 0xFFF));
        }
    }
    if (i == MAX_PAGES) return NULL;
    // Issue SYS_mmap2 directly (offset in PAGES), NOT the libc mmap symbol.
    // majestic exports a uClibc-ABI mmap for the vendor MPP blobs — signature
    // `mmap(..., uint32_t off_bytes)` (src/libc/uclibc.c) — and when majestic
    // dlopens this .so, our <sys/mman.h> mmap (64-bit off_t) binds to THAT symbol.
    // The 64-bit offset is then read as the wrong 32-bit register, so the mapping
    // lands on a garbage page and register reads come back as RAM (observed:
    // GPIO8 0x201C0000 read junk while /dev/spidev worked). The raw syscall is
    // immune to which mmap symbol wins.
    // SYS_mmap2 (offset in 4 KiB pages) is the 32-bit ARM entry this camera runs;
    // off the target -- the native host build that only exists to catch compile
    // errors, never to touch hardware -- fall back to the libc mmap so the file
    // still compiles.
#ifdef SYS_mmap2
    void *m = (void *)syscall(SYS_mmap2, NULL, (size_t)0x1000,
                              PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, base >> 12);
#else
    void *m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd,
                   (off_t)base);
#endif
    if (m == MAP_FAILED) {
        log_e("ms41908: mmap %#x: %s", base, strerror(errno));
        return NULL;
    }
    pages[i].base = base;
    pages[i].p = m;
    return (volatile uint32_t *)(pages[i].p + (phys & 0xFFF));
}

static uint32_t mmio_r(uint32_t a) { volatile uint32_t *r = reg(a); return r ? *r : 0; }
static void mmio_w(uint32_t a, uint32_t v) { volatile uint32_t *r = reg(a); if (r) *r = v; }

static void gpio_set(int g, int pin, bool lvl) {
    mmio_w(GPIO_BASE(g) + (1u << (pin + 2)), lvl ? (1u << pin) : 0);
}
static void vd_fz_pulse(void) {
    gpio_set(VD_GRP, VD_PIN, true);
    usleep(1000);
    gpio_set(VD_GRP, VD_PIN, false);
}
static void clear_isr(int bit) { mmio_w(INTR_CLR, mmio_r(INTR_CLR) | (1u << bit)); }
static int poll_done(int bit, int timeout_us) {
    for (int w = 0; w < timeout_us; w += 2000) {
        if ((mmio_r(INTR_STAT) >> bit) & 1) return 1;
        usleep(2000);
    }
    return 0;
}

// Validate that every register page the backend needs actually mapped. reg()
// caches pages and returns NULL on a failed mmap, after which mmio_r() reads 0
// and mmio_w() silently drops the write — so without this a failed mapping would
// leave the VD pulse (or a GPIO direction) a no-op while every move still reported
// success and advanced the software position. Called at open; a page maps once
// and stays mapped until close, so a validated page cannot then fail at runtime.
static bool map_required(void) {
    static const uint32_t bases[] = {
        0x200F0000u,  // IOCONFIG pinmux
        0x20140000u,  // GPIO0 (focus/zoom PI)
        0x201C0000u,  // GPIO8 (EN chip-select) + SPI1 clock
        0x201E0000u,  // GPIO10 (VD_FZ)
        0x20220000u,  // motion-done interrupt block
    };
    for (unsigned i = 0; i < sizeof bases / sizeof *bases; i++) {
        if (!reg(bases[i])) {
            log_e("ms41908: cannot map register page %#x", bases[i]);
            return false;
        }
    }
    return true;
}

// SPI1 pad-function + clock + the EN chip-select GPIO. OpenIPC leaves the pads as
// GPIO, so this must run before any SPI or every read is 0 (libxmaf ms419_plsintr_init).
static void spi_pads_init(void) {
    mmio_w(0x200F0060, 1); mmio_w(0x200F0064, 1); mmio_w(0x200F0068, 1); /* SPI1 SCLK/SDO/SDI = func 1 */
    mmio_w(0x200F006C, 0);
    mmio_w(0x201C0000, mmio_r(0x201C0000) | 0x80u);                     /* SPI1 clock enable */
    mmio_w(GPIO_DIR(EN_GRP), mmio_r(GPIO_DIR(EN_GRP)) | (1u << EN_PIN)); /* EN output */
    gpio_set(EN_GRP, EN_PIN, false);                                    /* xmspi_disable */
}

static void plsintr_init(void) {
    spi_pads_init();
    mmio_w(0x200F0150, 1); mmio_w(0x200F014C, 1);
    for (uint32_t a = 0x20220400; a <= 0x20220410; a += 4) mmio_w(a, mmio_r(a) & ~3u);
    mmio_w(INTR_CLR, mmio_r(INTR_CLR) | 3u);
    mmio_w(0x200F00A4, 0);
    mmio_w(GPIO_DIR(VD_GRP), mmio_r(GPIO_DIR(VD_GRP)) | (1u << VD_PIN)); /* VD_FZ output */
    gpio_set(VD_GRP, VD_PIN, false);
    mmio_w(0x200F00E0, 0); mmio_w(0x200F00E4, 0);
    mmio_w(GPIO_DIR(PIF_GRP),
           mmio_r(GPIO_DIR(PIF_GRP)) & ~((1u << PIF_PIN) | (1u << PIZ_PIN))); /* PIs input */
}

// ---- SPI register access (mirrors xmspi_write) -----------------------------

static bool spi_xfer(uint8_t *tx, uint8_t *rx) {
    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx, .rx_buf = (unsigned long)rx,
        .len = 3, .speed_hz = MS_SPI_HZ, .bits_per_word = 8,
    };
    gpio_set(EN_GRP, EN_PIN, true);
    int rc = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);
    gpio_set(EN_GRP, EN_PIN, false);
    // SPI_IOC_MESSAGE returns the byte count on success. A short or zero transfer
    // means the command did not fully reach the controller, so it is a failure:
    // returning true here would advance the software position (and publish a
    // magnification) for a move the motor never got.
    if (rc != (int)tr.len) {
        log_e("ms41908: spi transfer %d of %u: %s", rc, (unsigned)tr.len,
              rc < 0 ? strerror(errno) : "short");
        return false;
    }
    return true;
}
static bool spi_write(uint8_t addr, uint16_t val) {
    uint8_t tx[3] = {addr, val & 0xff, val >> 8}, rx[3] = {0};
    return spi_xfer(tx, rx);
}
static uint16_t spi_read(uint8_t addr) {
    uint8_t tx[3] = {addr | 0x40, 0, 0}, rx[3] = {0};
    spi_xfer(tx, rx);
    return rx[1] | (rx[2] << 8);
}

static bool ms419_init(void) {
    bool w = true;
    w &= spi_write(0x20, 0x5C02); w &= spi_write(0x22, 0x0001); w &= spi_write(0x27, 0x0001);
    w &= spi_write(0x23, 0xD0D0); w &= spi_write(0x28, 0xD0D0);
    w &= spi_write(0x25, 0x0160); w &= spi_write(0x2A, 0x0160); /* zoom/focus PPS */
    w &= spi_write(0x0B, 0x8480); w &= spi_write(0x21, 0x0087);
    uint16_t r21 = spi_read(0x21), r20 = spi_read(0x20);
    bool ok = w && r21 == 0x0087 && r20 == 0x5C02;
    log_i("ms41908: init 0x21=%#06x %s 0x20=%#06x %s", r21, r21 == 0x0087 ? "OK" : "FAIL",
          r20, r20 == 0x5C02 ? "OK" : "FAIL");
    return ok;
}

static long mono_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000L + ts.tv_nsec / 1000L;
}

// The chip's channel for each axis: zoom on the zoom motor, unless the lens is
// wired the other way round (s_swap).
static uint8_t zoom_reg(void) { return s_swap ? REG_FOCUS : REG_ZOOM; }
static int zoom_isr(void) { return s_swap ? ISR_FOCUS : ISR_ZOOM; }
static uint8_t focus_reg(void) { return s_swap ? REG_ZOOM : REG_FOCUS; }
static int focus_isr(void) { return s_swap ? ISR_ZOOM : ISR_FOCUS; }

// One motor burst: write ctrl, clear the done ISR, pulse VD_FZ, then hold the
// burst to a FIXED MS_BURST_MS of wall time — wait for the done ISR up to the
// cadence, then sleep out the remainder. The done ISR does not fire on every
// board (the motor still steps), and an unbounded wait for it would make each
// burst take up to ~72 ms instead of the modelled cadence; af2 converts requested
// ms into focus travel with the mechanics derived from MS_BURST_MS, so a variable
// burst time would make its timed cold seek terminate a fraction of the way in and
// anchor a wrong end-stop. A constant cadence keeps steps proportional to time.
static bool move_axis(uint8_t rgn, int isr_bit, bool dir, int step) {
    if (step < 1) step = 1;
    if (step > 63) step = 63;
    uint16_t ctrl = (uint16_t)((4 * step) | CTRL_BASE | (dir ? 0x0100 : 0));
    if (!spi_write(rgn, ctrl)) return false;
    clear_isr(isr_bit);
    long t0 = mono_us();
    vd_fz_pulse();
    poll_done(isr_bit, (MS_BURST_MS - 2) * 1000);   // bounded by the cadence
    long budget = (long)MS_BURST_MS * 1000, spent = mono_us() - t0;
    if (spent < budget) usleep((useconds_t)(budget - spent));
    return true;
}

// ---- the stepping thread: emulate continuous drive as micro-step bursts -----

// Whether the core was last told the zoom, or the focus, is moving
// (sdk_ptz_motion). Only the step thread reads or writes them. Reported from
// here rather than from the verbs: a zoom held at its soft stop stopped moving
// the picture long before its verb's deadline, and a focus request at its stop
// moves nothing at all. Focus is told too, on its own axis: it does not move
// the view, but a pass blurs and sharpens the whole picture, and the core holds
// motion detection back for it (OpenIPC/majestic#347).
static bool s_told_zoom = false;
static bool s_told_focus = false;

// Tell the core the zoom has stopped, if it was told it moved. Drops s_mu
// around the report, so a caller that gets true reads the state again.
static bool zoom_still_locked(void) {
    if (!s_told_zoom) return false;
    s_told_zoom = false;
    pthread_mutex_unlock(&s_mu);
    ptz_motion_report(0, 2, -1);
    pthread_mutex_lock(&s_mu);
    return true;
}

// The same for focus.
static bool focus_still_locked(void) {
    if (!s_told_focus) return false;
    s_told_focus = false;
    pthread_mutex_unlock(&s_mu);
    ptz_motion_report(0, PTZ_AXIS_FOCUS, -1);
    pthread_mutex_lock(&s_mu);
    return true;
}

// Tell the core focus is moving, just before a burst that moves it. Returns with s_mu dropped,
// as the burst wants it; the caller takes it again after the burst.
static void focus_burst_unlock(void) {
    bool begin = !s_told_focus;
    if (begin) s_told_focus = true;
    pthread_mutex_unlock(&s_mu);
    if (begin) ptz_motion_report(1, PTZ_AXIS_FOCUS, -1);
}

// Where the axes stopped, once per stop, at info level: the one way an owner
// measuring a lens's travel (ptz.lensZoomSteps / lensFocusSteps) can read it off
// the camera. Only from a known origin. Under s_mu, on the stepping thread.
static void ms_log_positions(void) {
    // Only once an operator's move has stopped: an autofocus pass is dozens of
    // precise steps, and a line for each would fill the syslog ring.
    if (!s_moved_manual) return;
    s_moved_manual = false;
    int z = s_zoom_homed ? s_zoom_pos : -1, f = s_focus_homed ? s_focus_pos : -1;
    if (z == s_logged_zoom && f == s_logged_focus) return;
    s_logged_zoom = z;
    s_logged_focus = f;
    char zs[16], fs[16];
    if (z >= 0) snprintf(zs, sizeof zs, "%d", z); else snprintf(zs, sizeof zs, "unknown");
    if (f >= 0) snprintf(fs, sizeof fs, "%d", f); else snprintf(fs, sizeof fs, "unknown");
    log_i("ms41908: zoom at %s of %d, focus at %s of %d", zs, s_lens.zoom_max, fs,
          s_lens.focus_max);
}

// The chip steps only on the sensor's frame timing, so a move made before the sensor streams is
// counted and never happens. On the OpenIPC/firmware#2385 dome (32 MB, a reboot) the picture took
// ~40 s to come up and homing ran its whole ram-and-return inside that: the log said "homed ...
// focus at 1658" and the next autofocus started from 204, a lens nowhere near where the count
// put it. So nothing moves until the focus statistic reads: the ISP refuses it until it runs, and
// it reads 0 until frames come. Probed at most every 200 ms; once seen, kept for this open.
static bool s_streaming = false;
static bool s_stream_told = false;

static bool streaming_locked(void) {
    if (s_streaming) return true;
    pthread_mutex_unlock(&s_mu);
    unsigned fv = 0;
    bool live = sdk_get_focus_value(&fv) && fv > 0;
    pthread_mutex_lock(&s_mu);
    if (live) {
        s_streaming = true;
        if (s_stream_told) log_i("ms41908: the sensor is streaming, the lens moves now");
        return true;
    }
    if (!s_stream_told) {
        s_stream_told = true;
        log_i("ms41908: waiting for the sensor to stream before moving the lens");
    }
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_nsec += 200 * 1000000L;
    if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
    pthread_cond_timedwait(&s_cv, &s_mu, &ts);
    return false;
}

static void *step_thread(void *arg) {
    (void)arg;
    pthread_mutex_lock(&s_mu);
    while (s_run) {
        // Any move waits for the stream (streaming_locked) -- except a precise request already
        // aborted, which only has to be answered, so a caller blocked on it is let go.
        bool work = s_req_kind != 0 || s_focus_dir != 0 || s_zoom_dir != 0;
        if (work && !(s_req_kind != 0 && s_req_abort) && !streaming_locked()) continue;
        // A synchronous precise-move / home request (af3's focus search) takes priority and runs
        // right here on this thread, so the bus stays single-owner and emit() never blocks. s_mu
        // is released around each SPI burst; abort (set by any emit) stops it between bursts.
        if (s_req_kind != 0) {
            if (zoom_still_locked()) continue;
            int kind = s_req_kind, dir = s_req_dir, want = s_req_n, moved = 0;
            // Do NOT clear s_req_abort here: the submitter (ms_focus_step/home) already cleared it,
            // and a stop/manual that raced in AFTER that, but before we picked the request up, has
            // set it again to preempt us -- clearing it here would drop that preemption. The loop
            // also yields the moment a manual command has ARMED a direction (s_focus_dir/s_zoom_dir),
            // so a manual that claimed the lens between the caller's gate check and this point wins.
            const int max = s_lens.focus_max;
            while (!s_req_abort && s_focus_dir == 0 && s_zoom_dir == 0 && s_run) {
                int req, burst;
                if (kind == 2) {                         // HOME: ram to the near stop
                    if (s_focus_homed) break;
                    dir = -1; req = MS_STEP_BURST; burst = MS_STEP_BURST;
                } else {                                 // STEP: n microsteps, clamped when homed
                    req = want - moved;
                    if (req <= 0) break;
                    if (req > MS_STEP_BURST) req = MS_STEP_BURST;
                    burst = (!s_focus_homed && dir < 0) ? req
                              : ms_clamp_step(s_focus_pos, dir, req, max);
                    if (burst == 0) break;               // at the soft stop this way
                }
                focus_burst_unlock();
                bool ok = move_axis(focus_reg(), focus_isr(), dir > 0, burst);
                pthread_mutex_lock(&s_mu);
                if (!ok) break;
                if (s_focus_homed) {
                    s_focus_pos += dir * burst;
                } else if (dir < 0) {
                    s_focus_run += burst;
                    if (s_focus_run >= max) { s_focus_homed = true; s_focus_pos = 0; s_focus_run = 0; }
                } else {
                    s_focus_run = 0;
                    s_focus_pos += burst;
                }
                moved += burst;
                if (kind == 1 && burst < req) break;     // clamp gave less than asked → hit the stop
            }
            s_req_moved = moved;
            s_req_kind = 0;
            pthread_cond_broadcast(&s_done);
            continue;
        }
        int fdir = s_focus_dir, zdir = s_zoom_dir;
        if (fdir == 0 && zdir == 0) {
            if (zoom_still_locked()) continue;
            if (focus_still_locked()) continue;
            ms_log_positions();
            pthread_cond_wait(&s_cv, &s_mu);
            continue;
        }
        // Focus takes priority; emit() only ever leaves one axis armed anyway.
        bool is_zoom = (fdir == 0);
        int dir = is_zoom ? zdir : fdir;
        int *pos = is_zoom ? &s_zoom_pos : &s_focus_pos;
        bool *homed = is_zoom ? &s_zoom_homed : &s_focus_homed;
        int *run = is_zoom ? &s_zoom_run : &s_focus_run;
        int max = is_zoom ? s_lens.zoom_max : s_lens.focus_max;

        // Unhomed, the software position is not a physical reference, so the
        // toward-0 soft limit must NOT block the seek that homes the axis: ram full
        // bursts toward the home stop (the motor is current-limited, so riding it is
        // safe) and bound the toward-MAX direction only by the load origin. Homed,
        // the clamp applies both ways from a known 0.
        int step = (!*homed && dir < 0) ? MS_STEP_BURST
                                        : ms_clamp_step(*pos, dir, MS_STEP_BURST, max);
        if ((is_zoom && s_home_phase == 3) || (!is_zoom && s_home_phase == 4)) {
            // Back to where the last run left it, whichever way that now is (a seed
            // that arrives during this leg moves the target).
            int target = is_zoom ? s_zoom_target : s_focus_target;
            int left = dir > 0 ? target - *pos : *pos - target;
            if (left < step) step = left > 0 ? left : 0;
        }
        if (step == 0 && s_home_phase && *homed) {
            // A homing phase has reached its stop (or its target): the next one.
            if (s_home_phase == 1) {
                s_zoom_dir = 0;
                s_focus_dir = -1;
                s_home_phase = 2;
            } else if (s_home_phase == 2 && s_zoom_target > 0) {
                s_focus_dir = 0;
                s_zoom_dir = +1;
                s_home_phase = 3;
            } else if (s_home_phase <= 3 && s_focus_target > 0) {
                s_zoom_dir = 0;
                s_focus_dir = +1;   // focus is at its near stop (0): the target is FAR of it
                s_home_phase = 4;
            } else {
                s_zoom_dir = s_focus_dir = 0;
                s_home_phase = 0;
            }
            if (!s_home_phase) {
                log_i("ms41908: homed, zoom at %d of %d, focus at %d of %d", s_zoom_pos,
                      s_lens.zoom_max, s_focus_pos, s_lens.focus_max);
                // Nothing was reported while homing (see below); now the zoom is
                // somewhere known, say where, and let an af3 caller that waited go.
                float hmag = ms_lens_zoom_mag(&s_lens, s_zoom_pos);
                pthread_cond_broadcast(&s_done);
                pthread_mutex_unlock(&s_mu);
                af_zoom_report(hmag);
                pthread_mutex_lock(&s_mu);
            }
            continue;
        }
        if (step == 0) {
            if (zoom_still_locked()) continue;
            if (focus_still_locked()) continue;
            // At the stop in the commanded direction (e.g. af2's cold near-stop
            // seek once focus is homed): idle without spinning, staying responsive
            // to a stop or a teardown. af2's timed seek still elapses and re-anchors.
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 10 * 1000000L;
            if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
            pthread_cond_timedwait(&s_cv, &s_mu, &ts);
            continue;
        }
        if (!is_zoom && zoom_still_locked()) continue;   // focus took over
        if (is_zoom && focus_still_locked()) continue;   // zoom took over
        if (is_zoom) {
            bool begin = !s_told_zoom;
            if (begin) s_told_zoom = true;
            pthread_mutex_unlock(&s_mu);
            if (begin) ptz_motion_report(1, 2, -1);   // just before the first burst
        } else {
            focus_burst_unlock();
        }
        bool ok = move_axis(is_zoom ? zoom_reg() : focus_reg(),
                            is_zoom ? zoom_isr() : focus_isr(), dir > 0, step);
        float mag = 0.0f;
        bool report = false;
        pthread_mutex_lock(&s_mu);
        if (ok) {
            // Only this thread writes these, so the values are still current.
            if (*homed) {
                *pos += dir * step;   // ms_clamp_step kept it inside [0, max]
            } else if (dir < 0) {
                *run += step;
                if (*run >= max) { *homed = true; *pos = 0; *run = 0; }  // reached the stop
            } else {
                *run = 0;             // a forward move breaks the toward-0 sweep
                *pos += step;         // clamped to max from the load origin
            }
            // Only from a verified origin, and not while homing: the engine reads the
            // saved magnification after the lens opens to set where homing returns to,
            // and a report from the home stop would overwrite it first.
            if (is_zoom && s_zoom_homed && s_home_phase == 0) {
                mag = ms_lens_zoom_mag(&s_lens, s_zoom_pos);
                report = true;
            }
            if (s_home_phase == 0) s_moved_manual = true;
        }
        pthread_mutex_unlock(&s_mu);
        if (report) {
            af_zoom_report(mag);   // never holds s_mu — af_zoom_report takes its own
        }
        pthread_mutex_lock(&s_mu);
    }
    bool was_zooming = s_told_zoom, was_focusing = s_told_focus;
    s_told_zoom = false;
    s_told_focus = false;
    pthread_mutex_unlock(&s_mu);
    if (was_zooming) ptz_motion_report(0, 2, -1);   // shut down mid-zoom
    if (was_focusing) ptz_motion_report(0, PTZ_AXIS_FOCUS, -1);
    return NULL;
}

// ---- vtable ops ------------------------------------------------------------

static void unmap_pages(void) {
    for (int i = 0; i < MAX_PAGES && pages[i].p; i++) {
        munmap((void *)pages[i].p, 0x1000);
        pages[i].p = NULL;
        pages[i].base = 0;
    }
}

// The lens from ptz.lens*, each figure kept only inside a range a lens can
// have: a value the schema should have refused leaves that one at the LH13's.
static int lens_int(const char *key, int lo, int hi, int def) {
    int v = config_get_int("ptz", key);
    return v >= lo && v <= hi ? v : def;
}

static void ms_lens_from_config(void) {
    MsLens l = MS_LENS_DEFAULT;
    l.zoom_max = lens_int("lensZoomSteps", 200, 20000, l.zoom_max);
    l.focus_max = lens_int("lensFocusSteps", 200, 20000, l.focus_max);
    int mag = lens_int("lensZoomMax", 2, 60, (int)l.mag_max);
    l.mag_max = (float)mag;
    l.zoom_reversed = config_get_boolean("ptz", "lensZoomReversed");
    s_lens = l;
    s_swap = config_get_boolean("ptz", "lensSwap");
    // af3 sizes its search from the focus travel: the vtable carries this lens's. Not the
    // backlash, though: gear slack is the gear's, a fixed number of steps whatever the travel, and
    // scaling it with a longer lensFocusSteps made af3 overshoot the crest by that much more.
    act_ms41908.focus_steps = l.focus_max;
}

static bool ms_open(void) {
    pthread_mutex_lock(&s_mu);
    if (spi_fd >= 0) {
        pthread_mutex_unlock(&s_mu);
        return true;
    }
    int fd = open(MS_SPIDEV, O_RDWR);
    if (fd < 0) {
        pthread_mutex_unlock(&s_mu);
        return false;   // motion.c owns the rate-limited retry logging
    }
    uint8_t mode = MS_SPI_MODE, bits = 8;
    uint32_t hz = MS_SPI_HZ;
    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0 ||
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
        log_e("ms41908: SPI config: %s", strerror(errno));
        close(fd);
        pthread_mutex_unlock(&s_mu);
        return false;
    }
    int mfd = open("/dev/mem", O_RDWR | O_SYNC);
    if (mfd < 0) {
        log_e("ms41908: open /dev/mem: %s", strerror(errno));
        close(fd);
        pthread_mutex_unlock(&s_mu);
        return false;
    }
    spi_fd = fd;
    mem_fd = mfd;
    if (!map_required()) {
        // A required register page could not be mapped; without it a GPIO or the
        // VD pulse would silently no-op while moves still reported success.
        unmap_pages();
        close(spi_fd); spi_fd = -1;
        close(mem_fd); mem_fd = -1;
        pthread_mutex_unlock(&s_mu);
        return false;
    }
    plsintr_init();
    if (!ms419_init()) {
        // The chip is not answering on the SPI bus (pinmux/wiring, not a missing
        // stream — register reads need no VD). Report no motor and let motion.c
        // retry; a stepping thread that could never move is worse than none.
        unmap_pages();
        close(spi_fd); spi_fd = -1;
        close(mem_fd); mem_fd = -1;
        pthread_mutex_unlock(&s_mu);
        return false;
    }
    ms_lens_from_config();
    s_focus_dir = s_zoom_dir = 0;
    s_focus_pos = s_zoom_pos = 0;
    s_focus_run = s_zoom_run = 0;
    s_focus_homed = s_zoom_homed = false;   // no absolute reference until a home seek
    s_logged_zoom = s_logged_focus = -1;
    s_home_phase = 0;
    s_zoom_target = -1;
    s_focus_target = -1;
    // A restored magnification re-anchors zoom without a physical seek: the lens
    // did not move while the transport was down. Focus has no such restore here --
    // af2 re-homes it -- so only zoom is seeded.
    s_streaming = false;   // a restarted majestic brings the sensor up again
    s_stream_told = false;
    s_lens_home = config_get_boolean("ptz", "lensHome");
    if (s_lens_home) {
        // Ram both axes into their home stops before trusting any position, then
        // take the zoom back to where the last run left it. The seed below is then
        // only a target, not an origin: a lens that slipped since, or was never
        // where the saved figure said, comes out right either way.
        if (s_zoom_seed_mag >= MS_MAG_MIN)
            s_zoom_target = ms_lens_zoom_pos_for_mag(&s_lens, s_zoom_seed_mag);
        if (s_focus_seed >= 0 && s_focus_seed <= s_lens.focus_max)
            s_focus_target = (int)s_focus_seed;
        s_zoom_dir = -1;
        s_home_phase = 1;
        log_i("ms41908: homing the lens (zoom %d, focus %d steps)", s_lens.zoom_max,
              s_lens.focus_max);
    } else if (s_zoom_seed_mag >= MS_MAG_MIN) {
        s_zoom_pos = ms_lens_zoom_pos_for_mag(&s_lens, s_zoom_seed_mag);
        s_zoom_homed = true;
    }
    s_run = 1;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 0x10000);
    int rc = pthread_create(&s_thread, &attr, step_thread, NULL);
    pthread_attr_destroy(&attr);
    if (rc) {
        // No stepping thread means no motion; give the transport back so a later
        // retry does not take the already-open fast path and inherit a dead lens.
        log_e("ms41908: cannot start the stepping thread: %s", strerror(rc));
        s_run = 0;
        unmap_pages();
        close(spi_fd); spi_fd = -1;
        close(mem_fd); mem_fd = -1;
        pthread_mutex_unlock(&s_mu);
        return false;
    }
    s_thread_valid = true;   // JOINABLE: ms_close joins before dlclose
    pthread_mutex_unlock(&s_mu);
    return true;
}

static bool ms_emit(enum PtzVerb v, int speed) {
    (void)speed;   // the MS41908M runs zoom/focus at its own rate
    bool is_zoom;
    int dir;
    pthread_mutex_lock(&s_mu);
    if (s_home_phase) {
        // Homing: a stop is not the operator's to give this lens -- on a dome it is
        // the end of a pan, sent to both motors -- and stopping the seek would leave
        // the zoom with no origin for the rest of the run. A move waits until homing
        // is done: answered "unavailable", for the seconds it takes.
        pthread_mutex_unlock(&s_mu);
        return v == PTZ_STOP;
    }
    s_req_abort = 1;   // a manual/zoom command preempts any precise af3 move in flight
    if (v == PTZ_STOP) {
        s_focus_dir = 0;
        s_zoom_dir = 0;
    } else if (ms_lens_verb_axis(&s_lens, v, &is_zoom, &dir)) {
        // One axis at a time: arm this one, disarm the other.
        if (is_zoom) { s_zoom_dir = dir; s_focus_dir = 0; }
        else { s_focus_dir = dir; s_zoom_dir = 0; }
    } else {
        pthread_mutex_unlock(&s_mu);
        return false;   // a verb this lens does not carry
    }
    pthread_cond_signal(&s_cv);   // wake the stepping thread; never blocks
    pthread_mutex_unlock(&s_mu);
    return true;
}

static bool ms_has(enum PtzVerb v) {
    return v == PTZ_STOP || v == PTZ_NEAR || v == PTZ_FAR || v == PTZ_TELE || v == PTZ_WIDE;
}

static const char *ms_proto_name(void) { return "ms41908"; }
static bool ms_wake_noop(void) { return true; }   // no MCU handshake on a stepper
static int ms_fd(void) { return -1; }             // no UART: engine's reader stays off

// Seed the zoom origin from a restored magnification (see s_zoom_seed_mag and the
// seed_mag vtable note). Remembered so every open re-anchors, and applied at once
// when the thread is already running so a seed that arrives after open still takes.
static bool ms_seed_zoom(float mag) {
    if (mag < MS_MAG_MIN) return false;
    pthread_mutex_lock(&s_mu);
    s_zoom_seed_mag = mag;
    if (s_run && s_lens_home) {
        // Homing finds the origin itself: a saved figure is only where to take the
        // zoom back to afterwards, and anchoring on it mid-seek would stop the ram
        // short of the stop it is looking for.
        if (s_home_phase == 1 || s_home_phase == 2) {
            s_zoom_target = ms_lens_zoom_pos_for_mag(&s_lens, mag);
        } else if (s_home_phase == 3) {
            s_zoom_target = ms_lens_zoom_pos_for_mag(&s_lens, mag);
            s_zoom_dir = s_zoom_target > s_zoom_pos ? +1 : s_zoom_target < s_zoom_pos ? -1 : 0;
            pthread_cond_signal(&s_cv);
        }
    } else if (s_run) {   // already open: re-anchor now; only this and the step thread write these
        s_zoom_pos = ms_lens_zoom_pos_for_mag(&s_lens, mag);
        s_zoom_homed = true;
        s_zoom_run = 0;
    }
    pthread_mutex_unlock(&s_mu);
    return true;
}

// The focus position the engine restored for this boot (Actuator.seed_focus).
// Only a lens that homes at open has a use for it: homing leaves focus on its near
// stop, and this is where to take it afterwards. Without homing the backend never
// anchored focus on a saved figure, and does not start now.
static bool ms_seed_focus(long pos) {
    if (pos < 0) return false;
    pthread_mutex_lock(&s_mu);
    s_focus_seed = pos;   // for an open still to come (ms_open applies it)
    bool used = false;
    if (s_run && s_lens_home && pos <= s_lens.focus_max) {
        if (s_home_phase >= 1 && s_home_phase <= 3) {
            s_focus_target = (int)pos;
            used = true;
        } else if (s_home_phase == 0 && s_focus_homed && s_focus_pos == pos) {
            used = true;   // already there: homing at open took it back (an earlier seed)
        } else if (s_home_phase == 4 ||
                   (s_home_phase == 0 && s_focus_homed && s_focus_pos == 0 &&
                    s_focus_dir == 0 && s_zoom_dir == 0 && s_req_kind == 0)) {
            // Mid-return, or homing already over with focus still on the near stop it
            // left it at (nothing has moved it since): go there now.
            s_focus_target = (int)pos;
            s_focus_dir = pos > s_focus_pos ? +1 : pos < s_focus_pos ? -1 : 0;
            if (s_focus_dir) s_home_phase = 4;
            pthread_cond_signal(&s_cv);
            used = true;
        }
    }
    pthread_mutex_unlock(&s_mu);
    return used;
}

// Homing at every open (ptz.lensHome) makes focus positions boot-independent: each is
// a step count from a near stop this backend has just found again (Actuator.homes_focus).
static bool ms_homes_focus(void) {
    pthread_mutex_lock(&s_mu);
    bool h = s_run && s_lens_home;
    pthread_mutex_unlock(&s_mu);
    return h;
}

// --- af3's step-based focus interface (synchronous; runs on the stepping thread) ----------

// Move up to n focus microsteps in dir; returns the microsteps the dead reckoning advanced
// (less at a soft stop). Blocks the CALLER on s_done while the stepping thread does the SPI —
// emit() and motion.c never wait on this.
static int ms_focus_step(int dir, int n) {
    if (n <= 0) return 0;
    pthread_mutex_lock(&s_mu);
    while (s_home_phase && s_run) pthread_cond_wait(&s_done, &s_mu);   // homing first
    if (spi_fd < 0 || !s_run) { pthread_mutex_unlock(&s_mu); return 0; }
    s_req_dir = dir < 0 ? -1 : 1;
    s_req_n = n;
    s_req_abort = 0;
    s_req_kind = 1;
    pthread_cond_signal(&s_cv);
    while (s_req_kind != 0 && s_run) pthread_cond_wait(&s_done, &s_mu);
    int moved = s_req_moved;
    pthread_mutex_unlock(&s_mu);
    return moved;
}

// Ram focus to the near stop and take it as 0 (no working PI on these units). One-time reference
// for a cold af3 pass; a no-op once homed. Blocks the caller until the axis is homed.
static bool ms_focus_home(void) {
    pthread_mutex_lock(&s_mu);
    while (s_home_phase && s_run) pthread_cond_wait(&s_done, &s_mu);   // homing first
    if (spi_fd < 0 || !s_run) { pthread_mutex_unlock(&s_mu); return false; }
    if (!s_focus_homed) {
        s_req_abort = 0;
        s_req_kind = 2;
        pthread_cond_signal(&s_cv);
        while (s_req_kind != 0 && s_run) pthread_cond_wait(&s_done, &s_mu);
    }
    bool homed = s_focus_homed;
    pthread_mutex_unlock(&s_mu);
    return homed;
}

// Current dead-reckoned focus position in microsteps, or -1 if not homed (unknown origin).
static int ms_focus_position(void) {
    pthread_mutex_lock(&s_mu);
    int p = s_focus_homed ? s_focus_pos : -1;
    pthread_mutex_unlock(&s_mu);
    return p;
}

static void ms_close(void) {
    pthread_mutex_lock(&s_mu);
    s_focus_dir = s_zoom_dir = 0;
    s_run = 0;
    s_home_phase = 0;
    pthread_cond_signal(&s_cv);
    pthread_cond_broadcast(&s_done);   // an af3 caller waiting out homing goes too
    bool join = s_thread_valid;
    pthread_t t = s_thread;
    s_thread_valid = false;
    pthread_mutex_unlock(&s_mu);
    if (join) {
        pthread_join(t, NULL);   // no hardware access after this
    }
    pthread_mutex_lock(&s_mu);
    unmap_pages();
    if (spi_fd >= 0) { close(spi_fd); spi_fd = -1; }
    if (mem_fd >= 0) { close(mem_fd); mem_fd = -1; }
    pthread_mutex_unlock(&s_mu);
}

// Not const: focus_steps is this lens's, set at open
// (ms_lens_from_config) under motion.c's lock, which is where they are read.
Actuator act_ms41908 = {
    .name = "ms41908",
    .reports_motion = true,
    .proto_name = ms_proto_name,
    .open = ms_open,
    .close = ms_close,
    .emit = ms_emit,
    .has = ms_has,
    .wake_blob = ms_wake_noop,
    .wake = ms_wake_noop,
    .fd = ms_fd,
    .derives_mag = true,   // pushed from the stepping thread via af_zoom_report
    .seed_mag = ms_seed_zoom,   // restore the zoom origin without a physical seek
    .seed_focus = ms_seed_focus,   // with lensHome: where focus goes back to after homing
    .homes_focus = ms_homes_focus,   // with lensHome: a saved focus outlives a reboot
    .backlash_ms = MS_BACKLASH_MS,
    .zoom_settle_ms = 0,   // the SoC drives both steppers itself: nothing moves after a stop
    .zoom_late_ms = 0,
    // Step-based focus for af3. focus_steps > 0 selects af3 over af2. The backlash estimate is
    // the timed placeholder converted to steps (MS_BACKLASH_MS of the MS_TRAVEL_MS travel); it
    // only sizes af3's slack-priming move, and af3's landing is FV-guided, so a rough value is fine.
    .focus_steps = MS_FOCUS_MAX,
    .focus_backlash_steps = (MS_FOCUS_MAX * MS_BACKLASH_MS) / MS_TRAVEL_MS,
    .focus_step = ms_focus_step,
    .focus_home = ms_focus_home,
    .focus_pos = ms_focus_position,
};
