// Contrast autofocus: drive the focus motor while watching the ISP's
// per-frame focus statistic, stop on the peak. The statistic comes through
// sdk_get_focus_value() (vendor-neutral; HiSilicon gen4 implements it from
// the BE AF zone grid), the motor through a UART protocol chosen at runtime
// from isp.autofocus.actuator — the XiongMai near-Pelco variant btzoom-xm
// speaks, or standard Pelco-D. Checksums are computed over the bytes actually
// sent (majestic-webui#258).
//
// The search itself lives in af2.c: a LOCAL momentum hunt from wherever the
// lens is. The 85H50AI's focus travel is long (~38 s near<->far, measured) and
// its reversal backlash small (~1 s), so a full-range strategy (seek a stop,
// scan, replay) costs 2-3 traversals and blows past the WebUI's ~60 s poll —
// which is exactly the "never converges" the field saw. Instead af2 drives one
// way tracking the focus statistic's trend, finds the single unimodal crest
// (the FV has a gradient everywhere, so it can always tell which way is up),
// and reverses back onto it closed-loop. Fast when near focus, bounded by one
// traversal when deep-defocused. The scene's achievable maximum is not known
// in advance (the ceiling swings ~20x between daylight FV ~4000 and dusk ~250
// on the same view), so nothing here compares against an absolute target.
//
// Serialisation: the whole pass holds the same mkdir lock btzoom/btzoom-xm
// take (/tmp/btzoom.lock), so a pad press and the engine never interleave
// frames on the UART.

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <majestic/af.h>
#include <majestic/af2.h>
#include <majestic/af_plugin_abi.h>   // HAL seams imported from the core: sdk_get_focus_value,
                                       // sdk_set_zoom_mag, config_get_*
#include <majestic/log.h>

#define AF_LOCK "/tmp/btzoom.lock"
// Timing budget of a pass. Guarded so the offline model harness (tests) can
// compress a full pass into milliseconds without patching the source; the
// values below are the production ones and are unchanged when nothing overrides
// them.
#ifndef AF_TOTAL_BUDGET_MS
// A cold pass always seeks the near stop (~40 s worst case) then drives the curve (up to ~27 s
// for tele) and trims, so the budget must cover a full-travel seek plus a full-range drive.
#define AF_TOTAL_BUDGET_MS 90000
#endif
#ifndef AF_SAMPLE_MS
#define AF_SAMPLE_MS 60
#endif
#ifndef AF_SETTLE_MS
#define AF_SETTLE_MS 160
#endif
// One zoom pulse: command, hold, stop — the timed pulse btzoom-xm sends (the MCU ends its own
// move). majestic drives zoom on the SAME UART it uses for focus, so owning it here (instead of
// the external btzoom-xm) removes the cross-process lock contention that made a zoom wait on a
// running focus pass.
#ifndef AF_ZOOM_PULSE_MS
#define AF_ZOOM_PULSE_MS 500
#endif
// Focus-follows-zoom seed. Measured 2026-09-02: after any zoom the focus element overshoots
// the parfocal peak toward FAR by a roughly constant ~6800 ms, independent of the start
// position and the zoom distance. So a pass after a zoom seeds the position at
// peak+overshoot and drives NEAR onto the peak (a short move + trim) instead of a ~40 s
// re-home. If the seed lands in the floor (a one-jump to tele that clamped at the far stop,
// or a transition whose coupling differs), the pass falls back to a cold re-home.
#ifndef AF_ZOOM_OVERSHOOT_MS
#define AF_ZOOM_OVERSHOOT_MS 6800
#endif
// FV below this is the flat contrast floor (peaks run ~1000..11000 in daylight, the floor is
// integer ~2..9): a seeded pass whose best FV stays here missed, and re-homes.
#ifndef AF_FLOOR_FV
#define AF_FLOOR_FV 40
#endif

// Actuator wire protocol: the five command frames and their length. The command
// BITS are identical across the XiongMai near-Pelco variant and standard Pelco-D
// (focus near = cmd1 0x01, far = cmd2 0x80, zoom tele = cmd2 0x20, wide = 0x40);
// only the wrapper differs — see the two tables in the actuator section. Chosen
// at open() from isp.autofocus.actuator.
typedef struct {
    const char *name;
    int len;    // 8 (XM: 0xC5 sync + 0x5C terminator) or 7 (standard Pelco-D)
    unsigned char near[8], far[8], stop[8], tele[8], wide[8];
} ActuatorProto;

typedef struct {
    int fd;
    const ActuatorProto *proto;
} Actuator;

enum AfDir { AF_NEAR, AF_FAR };

static pthread_mutex_t af_mu = PTHREAD_MUTEX_INITIALIZER;
static bool af_running_flag = false;
static bool af_settle_first = false;
// Preemption: a fresh zoom while a pass runs sets af_cancel (the pass abandons its now-stale
// moves and drops the UART lock so the zoom can proceed) and requests one more pass, settled,
// for the new magnification. All three are written under af_mu; af_cancel is read lock-free by
// the running pass through the af2 cancel hook.
static volatile int af_cancel = 0;
static bool af_restart_pending = false;
static bool af_restart_settle = false;
// Pending zoom pulses the worker owes the wire: +N tele, -N wide. A zoom request appends here
// and preempts the running focus (af_cancel), so the worker drains the pulses first, then
// re-focuses. Written under af_mu.
static int af_zoom_req = 0;
static char af_result[96] = "idle";

// Plugin lifecycle: the worker (transient, one per pass) and the magnification
// reader (persistent) are JOINABLE, and af_engine_stop() joins both before the
// core dlclose()s this .so — a detached thread outliving the unmap would fault.
// af_shutdown refuses new work during teardown; af_reader_stop ends the reader's
// poll loop. All reset by af_zoom_start() on (re)load.
static pthread_t af_worker;
static bool af_worker_valid = false;
static pthread_t af_reader;
static bool af_reader_valid = false;
static volatile int af_reader_stop = 0;
static volatile int af_shutdown = 0;

// Dead-reckoned focus position (ms of FAR travel from the near stop), carried across passes.
// The lens has no focus-position sensor, so the engine tracks position by integrating every
// focus command from a near-stop reference; this lets a re-AF at the SAME zoom drive to the
// ABSOLUTE parfocal position instead of re-homing. Touched only by the single running pass,
// so no lock. < 0 = unknown (forces a cold end-stop seek to re-anchor).
//
// IMPORTANT (measured 2026-09-02): zooming mechanically DISPLACES the focus element by an
// amount no focus command accounts for, so this dead-reckoned position is invalidated by any
// zoom. A pass therefore re-homes (cold) whenever the magnification changed since the last
// pass, and only trusts the carried position for a same-zoom re-AF. af_last_mag tracks that.
static long af_focus_pos = -1;
static float af_last_mag = -1.0f;

bool af_available(void) {
    return config_get_boolean("isp.autofocus", "enabled");
}

static void af_set_result(const char *s) {
    pthread_mutex_lock(&af_mu);
    snprintf(af_result, sizeof(af_result), "%s", s);
    pthread_mutex_unlock(&af_mu);
}

const char *af_status(void) {
    // The running flag flips before/after the result is written, and the
    // string itself is guarded — good enough for a diagnostic endpoint.
    if (af_running_flag) {
        return "running";
    }
    return af_result;
}

static long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void msleep(long ms) { usleep(ms * 1000); }

// btzoom's lock discipline, in C: one holder of the UART at a time, a dead
// holder must not wedge the camera, a caller that cannot have the port
// within ~2s gives up.
static bool af_lock_take(void) {
    for (int i = 0; i < 40; i++) {
        if (mkdir(AF_LOCK, 0755) == 0) {
            return true;
        }
        struct stat st;
        if (stat(AF_LOCK, &st) == 0 && time(NULL) - st.st_mtime > 60) {
            rmdir(AF_LOCK);
            continue;
        }
        msleep(50);
    }
    return false;
}

static void af_lock_drop(void) { rmdir(AF_LOCK); }

// --- UART actuator ---------------------------------------------------------
// Pelco-style timed-pulse motor control. Two protocols share the same command
// bits (see ActuatorProto) and differ only in framing. The checksum in each
// frame is precomputed — sum of the addr/command/data bytes, mod 100 for the XM
// variant (the historical btzoom-xm form) and mod 256 for standard Pelco-D.
//   pelco-xm  XiongMai near-Pelco (btzoom-xm): 0xC5 sync, 0x5C terminator, 8 bytes.
//             The 85H50AI MCU validates this checksum and rejects the older
//             constant-1 form.
//   pelco-d   standard Pelco-D: 0xFF sync, 7 bytes, no terminator.

static const ActuatorProto PROTO_PELCO_XM = {
    .name = "pelco-xm", .len = 8,
    .near = {0xc5, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02, 0x5c},
    .far  = {0xc5, 0x01, 0x00, 0x80, 0x00, 0x00, 0x1d, 0x5c},
    .stop = {0xc5, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x5c},
    .tele = {0xc5, 0x01, 0x00, 0x20, 0x00, 0x00, 0x21, 0x5c},
    .wide = {0xc5, 0x01, 0x00, 0x40, 0x00, 0x00, 0x41, 0x5c},
};

static const ActuatorProto PROTO_PELCO_D = {
    .name = "pelco-d", .len = 7,
    .near = {0xff, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02},   // sum 0x02
    .far  = {0xff, 0x01, 0x00, 0x80, 0x00, 0x00, 0x81},   // sum 0x81
    .stop = {0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01},   // sum 0x01
    .tele = {0xff, 0x01, 0x00, 0x20, 0x00, 0x00, 0x21},   // sum 0x21
    .wide = {0xff, 0x01, 0x00, 0x40, 0x00, 0x00, 0x41},   // sum 0x41
};

static const ActuatorProto *const ACTUATOR_PROTOS[] = {
    &PROTO_PELCO_XM,
    &PROTO_PELCO_D,
};

// Pick the protocol named by isp.autofocus.actuator; default to pelco-xm (the
// only backend shipped before this key was honoured, and the lab lens's protocol).
static const ActuatorProto *actuator_proto(void) {
    const char *name = config_get_string("isp.autofocus", "actuator");
    if (name && *name) {
        for (unsigned i = 0; i < sizeof ACTUATOR_PROTOS / sizeof *ACTUATOR_PROTOS; i++) {
            if (!strcmp(name, ACTUATOR_PROTOS[i]->name)) {
                return ACTUATOR_PROTOS[i];
            }
        }
        log_w("autofocus: unknown actuator '%s', using %s", name, PROTO_PELCO_XM.name);
    }
    return &PROTO_PELCO_XM;
}

static speed_t baud_const(int baud) {
    switch (baud) {
    case 1200: return B1200;
    case 2400: return B2400;
    case 4800: return B4800;
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    default: return B115200;
    }
}

static bool act_open(Actuator *a) {
    const char *port = config_get_string("isp.autofocus", "port");
    int speed = config_get_int("isp.autofocus", "speed");
    a->proto = actuator_proto();
    a->fd = open(port && *port ? port : "/dev/ttyAMA0", O_WRONLY | O_NOCTTY);
    if (a->fd < 0) {
        log_e("autofocus: cannot open %s: %s", port, strerror(errno));
        return false;
    }
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(a->fd, &tio) == 0) {
        cfmakeraw(&tio);
        cfsetspeed(&tio, baud_const(speed));
        tcsetattr(a->fd, TCSANOW, &tio);
    }
    return true;
}

static void act_frame(Actuator *a, const unsigned char *f) {
    if (write(a->fd, f, a->proto->len) != a->proto->len) {
        log_e("autofocus: short UART write: %s", strerror(errno));
    }
}

static void act_drive(Actuator *a, enum AfDir dir) {
    act_frame(a, dir == AF_NEAR ? a->proto->near : a->proto->far);
}

static void act_stop(Actuator *a) { act_frame(a, a->proto->stop); }

// Zoom (dir: +1 tele, -1 wide) — same wire and framing as the focus commands.
static void act_zoom(Actuator *a, int dir) {
    act_frame(a, dir > 0 ? a->proto->tele : a->proto->wide);
}

static void act_close(Actuator *a) {
    if (a->fd >= 0) {
        close(a->fd);
        a->fd = -1;
    }
}

// --- zoom magnification reader ----------------------------------------------
// The XiongMai lens MCU reports its absolute zoom magnification upstream on the
// focus UART's RX line as ASCII "X<ratio> " (e.g. "X3.2 "), but only while the
// zoom motor is moving. af.c drives focus write-only and btzoom-xm drives zoom,
// so nothing in-process reads that stream. This thread opens the port read-only
// (no UART lock — it never writes, so it can sit alongside a focus pass or a pad
// zoom) and caches the latest value for the OSD `%@` token. Value persists between
// zooms (the lens stays where it was); -1 until the first report after boot.

static pthread_mutex_t af_zoom_mu = PTHREAD_MUTEX_INITIALIZER;
static float af_zoom_value = -1.0f;
static long af_zoom_ts = 0;   // now_ms() of the last magnification update (0 = never)

float af_zoom_mag(void) {
    pthread_mutex_lock(&af_zoom_mu);
    float v = af_zoom_value;
    pthread_mutex_unlock(&af_zoom_mu);
    return v;
}

// Milliseconds since the magnification last changed, or a large number if never seen. The
// MCU only reports while zoom moves, so a fresh value means a zoom just happened; a stale
// value is still the correct current zoom (the lens has not moved).
long af_zoom_age(void) {
    pthread_mutex_lock(&af_zoom_mu);
    long ts = af_zoom_ts;
    pthread_mutex_unlock(&af_zoom_mu);
    return ts ? now_ms() - ts : 1 << 30;
}

static void af_zoom_set(float v) {
    pthread_mutex_lock(&af_zoom_mu);
    af_zoom_value = v;
    af_zoom_ts = now_ms();
    pthread_mutex_unlock(&af_zoom_mu);
    // Push to the CORE's cache too (sdk_set_zoom_mag is the imported seam), so the
    // OSD "%@" token and /zoom (GET) — which read from the core — reflect the
    // magnification this plugin's reader parsed. The local cache above is what
    // this plugin's own passes read through af_zoom_mag() for the parfocal target.
    sdk_set_zoom_mag(v);
}

static void *af_zoom_thread(void *arg) {
    (void)arg;
    const char *port = config_get_string("isp.autofocus", "port");
    int speed = config_get_int("isp.autofocus", "speed");
    int fd = open(port && *port ? port : "/dev/ttyAMA0", O_RDONLY | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        log_e("autofocus: zoom reader cannot open %s: %s", port, strerror(errno));
        return NULL;
    }
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(fd, &tio) == 0) {
        cfmakeraw(&tio);
        cfsetspeed(&tio, baud_const(speed));
        tio.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(fd, TCSANOW, &tio);
    }
    tcflush(fd, TCIFLUSH);

    // Accumulate an "X<ratio>" token: reset on 'X', append printable bytes, and parse when a
    // space/NUL/newline closes it. A malformed run just resets. Each wake DRAINS the port so
    // the latest report always wins even if several arrived during one fast zoom.
    unsigned char buf[256], acc[16];
    int accn = 0;
    // poll's 1 s timeout doubles as the teardown check interval: af_engine_stop()
    // sets af_reader_stop and joins, so the reader exits within ~1 s of a reload.
    while (!af_reader_stop) {
        struct pollfd p = {.fd = fd, .events = POLLIN};
        if (poll(&p, 1, 1000) <= 0 || !(p.revents & POLLIN)) {
            continue;
        }
        int n;
        while ((n = read(fd, buf, sizeof(buf))) > 0) {   // drain everything available
        for (int i = 0; i < n; i++) {
            unsigned char c = buf[i];
            if (c == 'X') {
                acc[0] = 'X';
                accn = 1;
            } else if (accn > 0) {
                if (c == ' ' || c == '\0' || c == '\r' || c == '\n') {
                    acc[accn] = 0;
                    float v = atof((char *)acc + 1);
                    if (v > 0.5f && v < 40.0f) {
                        af_zoom_set(v);
                    }
                    accn = 0;
                } else if (((c >= '0' && c <= '9') || c == '.') && accn < (int)sizeof(acc) - 1) {
                    acc[accn++] = c;
                } else {
                    accn = 0; // not a magnification token
                }
            }
        }
        }
    }
    close(fd);
    return NULL;
}

// Start the magnification reader. Called from the plugin's constructor at load,
// and again after a reload; resets the teardown flags so a reused image starts
// clean. JOINABLE — af_engine_stop() joins it before dlclose.
void af_zoom_start(void) {
    if (!af_available()) {
        return; // no focus motor declared: nothing reports magnification here
    }
    af_reader_stop = 0;
    af_shutdown = 0;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 0x10000);
    if (pthread_create(&af_reader, &attr, af_zoom_thread, NULL)) {
        log_e("autofocus: cannot start zoom reader");
    } else {
        af_reader_valid = true;
    }
    pthread_attr_destroy(&attr);
}

// --- the pass ---------------------------------------------------------------

static bool fv_sample(unsigned *fv) {
    unsigned v, sum = 0, n = 0;
    for (int i = 0; i < 2; i++) {
        if (sdk_get_focus_value(&v)) {
            sum += v;
            n++;
        }
        msleep(AF_SAMPLE_MS / 2);
    }
    if (!n) {
        return false;
    }
    *fv = sum / n;
    return true;
}

// The focus search itself lives in af2.c (a local momentum hunt with a closed-loop
// landing). af.c owns the actuator, the focus statistic, the lock and the thread; it
// hands af2 those four operations through an AfIO. The earlier full-range seek/scan/
// replay took 2-3 traversals of the ~38 s travel — past the WebUI's ~60 s poll, so the
// button appeared to never converge; the hunt lands within budget from any start
// (offline model of the 85H50AI: ~92 % of an extreme-corner sweep within 90 % of peak,
// ~95 % mean sharpness, avg ~26 s, on the measured travel/backlash/curve).
static void af_io_drive(void *ctx, int dir) {
    Actuator *a = ctx;
    if (dir < 0) {
        act_drive(a, AF_NEAR);
    } else if (dir > 0) {
        act_drive(a, AF_FAR);
    } else {
        act_stop(a);
    }
}
static unsigned af_io_fv(void *ctx) {
    // A single raw read: af2 medians several of these per sweep point (fv_samples),
    // which rejects transient bright frames far better than averaging two here would.
    (void)ctx;
    unsigned v;
    return sdk_get_focus_value(&v) ? v : 0;
}
static long af_io_now(void *ctx) {
    (void)ctx;
    return now_ms();
}
static void af_io_sleep(void *ctx, long ms) {
    (void)ctx;
    msleep(ms);
}

// Wait for the focus port to have been quiet — the pad lock absent — for a
// continuous window. A held zoom button strings pulses with only tens of
// milliseconds between them, so the window never elapses until the operator
// lets go; the after-zoom trigger therefore starts its pass exactly once,
// after the zooming is over.
static void af_wait_quiet(void) {
    long deadline = now_ms() + 8000;
    long quiet_since = now_ms();
    while (now_ms() < deadline) {
        struct stat st;
        if (stat(AF_LOCK, &st) == 0) {
            quiet_since = 0; // busy right now
        } else if (!quiet_since) {
            quiet_since = now_ms();
        } else if (now_ms() - quiet_since >= 700) {
            return;
        }
        msleep(100);
    }
}

// Trajectory trace sink (see the AfParams.trace hook): one "pos,fv" line per measurement.
static void af_trace(void *ctx, long pos, unsigned fv) {
    FILE *f = ctx;
    if (f) fprintf(f, "%ld,%u\n", pos, fv);
}

// One autofocus pass: wait for any in-flight zoom to settle, take the UART lock, run the
// engine, land, release. Cancellable at any move via af_cancel — a fresh zoom preempts it
// (af_trigger), and cancelling drops the lock so that zoom can proceed.
static void af_run_one_pass(bool settle) {
    char line[96];

    if (settle) {
        af_wait_quiet();
    }
    if (af_cancel) {
        return;                       // preempted before we even took the port
    }

    if (!af_lock_take()) {
        af_set_result("failed: focus port is busy");
        return;
    }

    Actuator a = {.fd = -1};
    if (!act_open(&a)) {
        af_lock_drop();
        af_set_result("failed: cannot open focus port");
        return;
    }

    unsigned before = 0;
    if (!fv_sample(&before)) {
        af_set_result("failed: no focus statistic");
        goto out;
    }

    AfIO io = {.drive = af_io_drive,
               .fv = af_io_fv,
               .now_ms = af_io_now,
               .sleep_ms = af_io_sleep,
               .ctx = &a};
    // Optional per-pass FV(position) trace for offline diagnosis: enabled by touching
    // /tmp/af_trace.on, written to /tmp/af_trace.csv. Off (and zero cost) otherwise.
    FILE *trf = access("/tmp/af_trace.on", F_OK) == 0 ? fopen("/tmp/af_trace.csv", "w") : NULL;
    if (trf) fprintf(trf, "pos,fv\n");
    // Pick the starting position (see af2.h). Three cases, from the measured mechanics:
    //  - zoom changed since the last pass: the zoom displaced focus to ~peak+overshoot, so SEED
    //    there and let af2 sweep NEAR onto the peak (fast) instead of a ~40 s re-home.
    //  - same zoom as last pass: the dead-reckoned position is still valid (no zoom to disturb
    //    it) — af2 backs off to the far side and sweeps in.
    //  - no magnification yet, or no prior pass: cold-seek the near stop to re-anchor.
    float mag_now = af_zoom_mag();
    float dmag = mag_now - af_last_mag;
    if (dmag < 0) dmag = -dmag;
    bool zoomed = af_last_mag >= 1.0f && dmag > 0.05f;
    long in_pos;
    if (mag_now < 1.0f || af_last_mag < 0) {
        in_pos = -1;                                             // cold re-home
    } else if (zoomed) {
        in_pos = af2_parfocal_foc(mag_now) + AF_ZOOM_OVERSHOOT_MS;   // seed at the zoom overshoot
    } else {
        in_pos = af_focus_pos;                                  // same zoom: position still holds
    }
    AfParams p = {// Mechanics measured on the 85H50AI: ~400 ms reversal backlash, full focus
                  // travel ~38 s (cap the cold seek a little above it).
                  .backlash_ms = 400,
                  .travel_max_ms = 42000,
                  .settle_ms = AF_SETTLE_MS,
                  .budget_ms = AF_TOTAL_BUDGET_MS,
                  // Median several frames per measurement so a rain glint or a passing
                  // light can't be mistaken for sharpness in a dynamic night scene.
                  .fv_samples = 5,
                  .fv_frame_ms = 40,
                  .travel_ms = 38000,
                  // Live magnification the lens MCU reports picks the parfocal target; the
                  // dead-reckoned focus position lets the pass drive to it absolutely. A
                  // position of -1 (fresh boot) makes the pass cold-seek the near stop first.
                  .mag_now = mag_now,
                  .in_focus_pos = in_pos,
                  .trace = trf ? af_trace : NULL,
                  .trace_ctx = trf,
                  .cancel = &af_cancel};
    unsigned final = af2_run(&io, &p);
    // A seeded (non-cold) pass that never rose above the contrast floor missed the peak — the
    // seed overshoot was wrong for this transition. Re-home reliably (unless we were preempted).
    if (in_pos >= 0 && p.out_peak_seen < AF_FLOOR_FV && !af_cancel) {
        p.in_focus_pos = -1;
        final = af2_run(&io, &p);
    }
    if (trf) fclose(trf);
    if (af_cancel) {
        // A fresh zoom preempted this pass mid-drive: the dead-reckoned position is no longer
        // known, and the pass was chasing the old magnification anyway. Drop it cleanly; the
        // caller will run another for the new position.
        af_focus_pos = -1;
        af_set_result("preempted");
        goto out;
    }
    unsigned peak = p.out_peak_seen;
    if (peak == 0) {
        af_set_result("failed: lens does not respond");
        goto out;
    }
    af_focus_pos = p.out_focus_pos;   // carry the dead-reckoned position to the next pass
    af_last_mag = mag_now;            // remember the zoom, to detect a change next pass

    snprintf(
        line, sizeof(line), "done fv=%u peak=%u start=%u mag=%.1f pos=%ld steps=%d path=%d",
        final, peak, before, (double)p.out_mag, p.out_focus_pos, p.out_steps, p.out_path);
    af_set_result(line);
    log_i("autofocus: %s", line);

out:
    act_stop(&a);
    act_close(&a);
    af_lock_drop();
}

// Drive `n` zoom pulses in one direction (dir: +1 tele, -1 wide), each command/hold/stop, under
// the UART lock. Quick (~0.6 s each); a fresh focus pass runs afterwards from the worker.
static void af_do_zoom(int dir, int n) {
    if (n <= 0 || !af_lock_take()) {
        return;
    }
    Actuator a = {.fd = -1};
    if (act_open(&a)) {
        for (int i = 0; i < n; i++) {
            act_zoom(&a, dir);
            msleep(AF_ZOOM_PULSE_MS);
            act_stop(&a);
            if (i < n - 1) msleep(60);   // brief gap so the MCU registers separate steps
        }
        act_close(&a);
    }
    af_lock_drop();
}

static void *af_thread(void *arg) {
    (void)arg;
    bool settle = af_settle_first;
    for (;;) {
        // Any pending zoom pulses come first: they preempted whatever focus was running, so the
        // lens must move before we focus. Clearing af_cancel here (under af_mu) pairs with the
        // set in af_zoom_pulse/af_trigger, so a zoom that arrives after this point re-arms
        // instead of being lost.
        pthread_mutex_lock(&af_mu);
        int z = af_zoom_req;
        af_zoom_req = 0;
        af_cancel = 0;
        pthread_mutex_unlock(&af_mu);
        if (z != 0) {
            af_do_zoom(z > 0 ? 1 : -1, z > 0 ? z : -z);
            settle = true;
        }

        af_run_one_pass(settle);

        pthread_mutex_lock(&af_mu);
        if (af_shutdown || (af_zoom_req == 0 && !af_restart_pending)) {
            af_running_flag = false;
            pthread_mutex_unlock(&af_mu);
            return NULL;
        }
        af_restart_pending = false;
        settle = af_restart_settle;
        pthread_mutex_unlock(&af_mu);
    }
}

// Spawn the worker. Caller holds no lock and has already set af_running_flag under af_mu.
static bool af_spawn(void) {
    // A prior worker runs exactly one convergence and returns; af_spawn is only
    // reached when no worker is running, so the last one has exited — join it
    // before starting the next so joinable workers never accumulate.
    // af_engine_stop() joins the live one at teardown, before dlclose.
    if (af_worker_valid) {
        pthread_join(af_worker, NULL);
        af_worker_valid = false;
    }
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 0x10000);
    if (pthread_create(&af_worker, &attr, af_thread, NULL)) {   // JOINABLE, not detached
        pthread_attr_destroy(&attr);
        pthread_mutex_lock(&af_mu);
        af_running_flag = false;
        pthread_mutex_unlock(&af_mu);
        return false;
    }
    af_worker_valid = true;
    pthread_attr_destroy(&attr);
    return true;
}

int af_trigger(bool settle) {
    if (!af_available()) {
        return -1;
    }
    pthread_mutex_lock(&af_mu);
    if (af_shutdown) {   // tearing down: refuse new work so the joined worker stays joined
        pthread_mutex_unlock(&af_mu);
        return -1;
    }
    if (af_running_flag) {
        // A pass is already running for the previous position — a fresh trigger arrived.
        // Preempt it: cancel the current pass (it is chasing a now-stale magnification, and
        // cancelling releases the UART lock) and ask it to run once more, settled, for the new
        // one. Rapid triggers just keep re-arming this, so the focus runs once after they stop.
        af_cancel = 1;
        af_restart_pending = true;
        af_restart_settle = settle;
        pthread_mutex_unlock(&af_mu);
        return 2;
    }
    af_cancel = 0;
    af_restart_pending = false;
    af_running_flag = true;
    af_settle_first = settle;
    pthread_mutex_unlock(&af_mu);
    return af_spawn() ? 0 : -1;
}

int af_zoom_pulse(int dir) {
    if (!af_available()) {
        return -1;
    }
    bool start;
    pthread_mutex_lock(&af_mu);
    if (af_shutdown) {   // tearing down: refuse new work
        pthread_mutex_unlock(&af_mu);
        return -1;
    }
    af_zoom_req += dir > 0 ? 1 : -1;   // append the pulse; the worker drains it before focusing
    af_cancel = 1;                     // preempt any running focus so it yields the wire at once
    start = !af_running_flag;
    if (start) {
        af_running_flag = true;
        af_settle_first = true;
    } else {
        af_restart_pending = true;     // make the running worker loop back and drain the zoom
        af_restart_settle = true;
    }
    pthread_mutex_unlock(&af_mu);
    return start ? (af_spawn() ? 0 : -1) : 0;
}

// Teardown for the plugin's af_plugin_exit(): stop accepting work, cancel any
// pass in flight, and JOIN both threads so no code in this .so is still executing
// when the core dlclose()s it (a detached thread outliving the unmap would fault).
// Idempotent — safe when nothing was started.
void af_engine_stop(void) {
    pthread_mutex_lock(&af_mu);
    af_shutdown = 1;
    af_cancel = 1;   // a running pass abandons its moves, drops the UART lock, and returns
    pthread_mutex_unlock(&af_mu);

    if (af_worker_valid) {
        pthread_join(af_worker, NULL);
        af_worker_valid = false;
    }
    af_reader_stop = 1;
    if (af_reader_valid) {
        pthread_join(af_reader, NULL);
        af_reader_valid = false;
    }
}
