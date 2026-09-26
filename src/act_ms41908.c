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
//   1. af2 dead-reckons focus position as milliseconds of *continuous* travel
//      (drive(+1) ... drive(0), integrating wall-clock time). So emit() does not
//      step — it just sets a direction — and a stepping thread issues micro-step
//      bursts at a fixed cadence while a direction is held. Steps then track time
//      linearly, and af2 needs no change; the lens mechanics it wants (travel_ms,
//      backlash) are computed from the cadence and handed over through the vtable.
//
//   2. The MS41908M reports no magnification (there is no MCU). But WE command the
//      zoom stepper, so we dead-reckon the zoom position and derive magnification
//      from it, pushing it through af_zoom_report() exactly where the UART reader
//      would — which is what lets af2's parfocal tracking work here at all.
//
// CALIBRATION: the zoom->magnification curve (ms_zoom_mag) and the focus mechanics
// below are best-effort for LENS_LH13_FHD_X16 and are finalised by an on-hardware
// pass. The soft travel limits are the measured libxmaf values and are exact.
//
// The motor only STEPS while the ISP pipeline is producing VD timing, so majestic
// must be streaming for motion — SPI register access (init) needs no streamer.

#include "act_ms41908.h"
#include "actuator.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <pthread.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include <majestic/log.h>

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

// Stepping cadence. A move_axis burst is a VD pulse plus a bounded wait for the
// motion-done ISR (~12 ms fallback where it does not fire), so ~13 ms per burst;
// MS_STEP_BURST microsteps per burst sets the focus resolution.
#define MS_STEP_BURST 8
#define MS_BURST_MS 13

// Focus mechanics handed to af2, in ms of travel — derived from the cadence so
// af2's dead-reckoning lands on real step counts. backlash is a placeholder.
#define MS_TRAVEL_MS ((MS_FOCUS_MAX / MS_STEP_BURST) * MS_BURST_MS)
#define MS_TRAVEL_MAX_MS (MS_TRAVEL_MS + MS_TRAVEL_MS / 4)
#define MS_BACKLASH_MS 150

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
    void *m = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, base);
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
    if (rc < 0) { log_e("ms41908: spi ioctl: %s", strerror(errno)); return false; }
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

// One motor burst: write ctrl, clear the done ISR, pulse VD_FZ, wait for done.
static bool move_axis(uint8_t rgn, int isr_bit, bool dir, int step) {
    if (step < 1) step = 1;
    if (step > 63) step = 63;
    uint16_t ctrl = (uint16_t)((4 * step) | CTRL_BASE | (dir ? 0x0100 : 0));
    if (!spi_write(rgn, ctrl)) return false;
    clear_isr(isr_bit);
    vd_fz_pulse();
    // The done ISR does not fire on every board (the motor still steps), so a poll
    // timeout is "no done-signal", not a failed move — settle and return ok.
    if (!poll_done(isr_bit, 60000)) usleep(12000);
    return true;
}

// ---- the stepping thread: emulate continuous drive as micro-step bursts -----

static void *step_thread(void *arg) {
    (void)arg;
    pthread_mutex_lock(&s_mu);
    while (s_run) {
        int fdir = s_focus_dir, zdir = s_zoom_dir;
        if (fdir == 0 && zdir == 0) {
            pthread_cond_wait(&s_cv, &s_mu);
            continue;
        }
        // Focus takes priority; emit() only ever leaves one axis armed anyway.
        bool is_zoom = (fdir == 0);
        int dir = is_zoom ? zdir : fdir;
        int pos = is_zoom ? s_zoom_pos : s_focus_pos;
        int max = is_zoom ? MS_ZOOM_MAX : MS_FOCUS_MAX;
        int step = ms_clamp_step(pos, dir, MS_STEP_BURST, max);
        if (step == 0) {
            // At the stop in the commanded direction (e.g. af2's cold near-stop
            // seek): idle without spinning, staying responsive to a stop or a
            // teardown. af2's timed seek still elapses and re-anchors its position.
            struct timespec ts;
            clock_gettime(CLOCK_REALTIME, &ts);
            ts.tv_nsec += 10 * 1000000L;
            if (ts.tv_nsec >= 1000000000L) { ts.tv_sec++; ts.tv_nsec -= 1000000000L; }
            pthread_cond_timedwait(&s_cv, &s_mu, &ts);
            continue;
        }
        pthread_mutex_unlock(&s_mu);
        bool ok = move_axis(is_zoom ? REG_ZOOM : REG_FOCUS,
                            is_zoom ? ISR_ZOOM : ISR_FOCUS, dir > 0, step);
        float mag = 0.0f;
        bool report = false;
        pthread_mutex_lock(&s_mu);
        if (ok) {
            // Only this thread writes the positions, so `pos` is still current.
            int np = pos + dir * step;
            if (is_zoom) {
                s_zoom_pos = np;
                mag = ms_zoom_mag(np);
                report = true;
            } else {
                s_focus_pos = np;
            }
        }
        pthread_mutex_unlock(&s_mu);
        if (report) {
            af_zoom_report(mag);   // never holds s_mu — af_zoom_report takes its own
        }
        pthread_mutex_lock(&s_mu);
    }
    pthread_mutex_unlock(&s_mu);
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
    s_focus_dir = s_zoom_dir = 0;
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
    if (v == PTZ_STOP) {
        s_focus_dir = 0;
        s_zoom_dir = 0;
    } else if (ms_verb_axis(v, &is_zoom, &dir)) {
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

static void ms_close(void) {
    pthread_mutex_lock(&s_mu);
    s_focus_dir = s_zoom_dir = 0;
    s_run = 0;
    pthread_cond_signal(&s_cv);
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

const Actuator act_ms41908 = {
    .name = "ms41908",
    .proto_name = ms_proto_name,
    .open = ms_open,
    .close = ms_close,
    .emit = ms_emit,
    .has = ms_has,
    .wake_blob = ms_wake_noop,
    .wake = ms_wake_noop,
    .fd = ms_fd,
    .derives_mag = true,   // pushed from the stepping thread via af_zoom_report
    .travel_ms = MS_TRAVEL_MS,
    .travel_max_ms = MS_TRAVEL_MAX_MS,
    .backlash_ms = MS_BACKLASH_MS,
};
