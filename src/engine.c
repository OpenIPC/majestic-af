// The engine reads the ISP focus metric and runs the selected AF algorithm.
// Motor access goes through af_motor and libmotors. The algorithm does not know
// which driver or transport the motor service uses.
//
#include <pthread.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "af_motor.h"

#include <majestic/af.h>
#include <majestic/af_algorithm.h>
#include <majestic/af_blind_seek.h>
#include <majestic/af2.h>
#include <majestic/af_plugin_abi.h>   // HAL seams imported from the core: sdk_get_focus_value,
                                       // sdk_set_zoom_mag, config_get_*
#include <majestic/log.h>

#define AF_CANCEL_REQUEST "/tmp/majestic-af.cancel"
#define AF_METRIC_STREAM_REQUEST "/tmp/af_metric_stream.on"
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
// Duration of one zoom request from the Majestic plugin interface.
#ifndef AF_ZOOM_PULSE_MS
#define AF_ZOOM_PULSE_MS 500
#endif
// After the quiet window identifies the end of a zoom sequence, give the P035
// controller more time to finish its focus matching before autofocus moves the lens.
#ifndef AF_ZOOM_QUIET_MS
#define AF_ZOOM_QUIET_MS 1200
#endif
#ifndef AF_ZOOM_POST_SETTLE_MS
#define AF_ZOOM_POST_SETTLE_MS 150
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

static pthread_mutex_t af_mu = PTHREAD_MUTEX_INITIALIZER;
static bool af_running_flag = false;
static bool af_settle_first = false;
// Preemption: a fresh zoom while a pass runs sets af_cancel (the pass abandons its now-stale
// moves and releases its motor lease) and requests one more pass, settled,
// for the new magnification. All three are written under af_mu; af_cancel is read lock-free by
// the running pass through the af2 cancel hook.
static volatile int af_cancel = 0;
static bool af_restart_pending = false;
static bool af_restart_settle = false;
// Pending zoom pulses: +N tele, -N wide. A zoom request appends here
// and preempts the running focus (af_cancel), so the worker drains the pulses first, then
// re-focuses. Written under af_mu.
static int af_zoom_req = 0;
static char af_result[96] = "idle";
// The HTTP caller needs a stable string after af_status() releases af_mu. Each caller gets its
// own copy while the worker continues to publish measurements into af_result.
static __thread char af_status_copy[160];

static pthread_once_t af_algorithm_once = PTHREAD_ONCE_INIT;
static enum AfAlgorithm af_algorithm = AF_ALGORITHM_INVALID;
static long now_ms(void);

static void af_load_algorithm(void) {
    const char *name = config_get_string("isp.autofocus", "algorithm");
    af_algorithm = af_algorithm_parse(name);
    if (af_algorithm == AF_ALGORITHM_INVALID)
        log_e("autofocus: isp.autofocus.algorithm must be blind_seek or af2");
}

/* This marker supports the current WebUI during migration to motorsd. The AF
 * worker also receives preemption events from motorsd through libmotors. */
static void af_poll_cancel_request(void) {
    if (unlink(AF_CANCEL_REQUEST) != 0) return;

    pthread_mutex_lock(&af_mu);
    af_cancel = 1;
    af_zoom_req = 0;
    af_restart_pending = false;
    af_restart_settle = false;
    if (af_running_flag) snprintf(af_result, sizeof(af_result), "cancelling");
    pthread_mutex_unlock(&af_mu);
}

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
    if (!config_get_boolean("isp.autofocus", "enabled")) return false;
    pthread_once(&af_algorithm_once, af_load_algorithm);
    return af_algorithm != AF_ALGORITHM_INVALID;
}

static void af_set_result(const char *s) {
    pthread_mutex_lock(&af_mu);
    snprintf(af_result, sizeof(af_result), "%s", s);
    pthread_mutex_unlock(&af_mu);
}

static void af_set_progress(const char *step, unsigned fv, unsigned peak) {
    pthread_mutex_lock(&af_mu);
    snprintf(af_result, sizeof(af_result), "running step=%s fv=%u peak=%u", step, fv, peak);
    pthread_mutex_unlock(&af_mu);
}

const char *af_status(void) {
    /* A local marker changes the status response into a raw metric stream.
     * The lens characterization tool uses this mode while AF is idle. */
    if (access(AF_METRIC_STREAM_REQUEST, F_OK) == 0) {
        unsigned fv = 0;
        if (!sdk_get_focus_value(&fv)) return "metric unavailable";
        snprintf(af_status_copy, sizeof(af_status_copy),
                 "metric t_mono_ms=%ld fv=%u", now_ms(), fv);
        return af_status_copy;
    }
    pthread_mutex_lock(&af_mu);
    snprintf(af_status_copy, sizeof(af_status_copy), "%s", af_result);
    pthread_mutex_unlock(&af_mu);

    /* Keep the state at the start of the response for existing clients. Add
     * one current sample so read-only users, such as the WebUI graph, do not
     * need the characterization marker or a second API. */
    unsigned fv = 0;
    if (sdk_get_focus_value(&fv)) {
        size_t used = strlen(af_status_copy);
        snprintf(af_status_copy + used, sizeof(af_status_copy) - used,
                 " metric_fv=%u t_mono_ms=%ld", fv, now_ms());
    }
    return af_status_copy;
}

static long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void msleep(long ms) { usleep(ms * 1000); }

// --- zoom magnification reader ----------------------------------------------
// The motor service provides optional magnification telemetry. This engine
// caches each received value for AF, the OSD `%@` token, and `/zoom` status.

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

static void af_zoom_set(void *ctx, float v) {
    (void)ctx;
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
    af_motor_read_magnification(&af_reader_stop, af_zoom_set, NULL);
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

// The engine owns the focus metric and worker. The motor adapter maps AF
// requests to service operations. AfIO gives both algorithms the same motor
// and metric interface.
static void af_io_drive(void *ctx, int dir) {
    AfMotor *motor = ctx;
    if (dir < 0) {
        af_motor_focus(motor, AF_MOTOR_FOCUS_NEAR);
    } else if (dir > 0) {
        af_motor_focus(motor, AF_MOTOR_FOCUS_FAR);
    } else {
        af_motor_stop(motor);
    }
}
static bool af_io_pulse(void *ctx, int dir, long ms) {
    if (ms <= 0 || ms > INT_MAX) return false;
    AfMotor *motor = ctx;
    enum AfMotorFocusDirection direction = dir < 0 ? AF_MOTOR_FOCUS_NEAR
                                                    : AF_MOTOR_FOCUS_FAR;
    bool completed = af_motor_focus_timed(motor, direction, (unsigned)ms);
    if (!completed) af_cancel = 1;
    return completed;
}
static unsigned af_io_fv(void *ctx) {
    // A single raw read: af2 medians several of these per sweep point (fv_samples),
    // which rejects transient bright frames far better than averaging two here would.
    (void)ctx;
    unsigned v;
    return sdk_get_focus_value(&v) ? v : 0;
}
static long af_io_now(void *ctx) {
    if (af_motor_poll(ctx)) af_cancel = 1;
    af_poll_cancel_request();
    return now_ms();
}
static void af_io_sleep(void *ctx, long ms) {
    if (af_motor_poll(ctx)) af_cancel = 1;
    af_poll_cancel_request();
    msleep(ms);
    if (af_motor_poll(ctx)) af_cancel = 1;
    af_poll_cancel_request();
}
static void af_io_progress(void *ctx, const char *step, unsigned fv, unsigned peak) {
    (void)ctx;
    af_set_progress(step, fv, peak);
}

static bool af_motor_cancelled(void *ctx) {
    (void)ctx;
    af_poll_cancel_request();
    return af_cancel != 0;
}

// Wait after the final requested zoom pulse. The service event interface will
// replace this fixed window when external zoom clients also use motorsd.
static void af_wait_quiet(void) {
    long deadline = now_ms() + AF_ZOOM_QUIET_MS;
    while (now_ms() < deadline && !af_cancel) {
        af_poll_cancel_request();
        msleep(100);
    }
}

// Trajectory trace sink (see the AfParams.trace hook): one "pos,fv" line per measurement.
static void af_trace(void *ctx, long pos, unsigned fv) {
    FILE *f = ctx;
    if (f) fprintf(f, "%ld,%u\n", pos, fv);
}

static void af_trace_sample(void *ctx, const char *phase, long started,
                            long ended, long pos, unsigned fv,
                            int direction, long pulse_ms) {
    fprintf(ctx, "%ld,%u,%s,%ld,%ld,%d,%ld\n", pos, fv, phase,
            started, ended, direction, pulse_ms);
}

// One autofocus pass: wait for zoom to settle, acquire the focus lease, run the
// engine, land, and release. A fresh zoom or service event cancels the pass.
static void af_run_one_pass(bool settle) {
    char line[96];

    if (settle) {
        af_set_progress("settle", 0, 0);
        af_wait_quiet();
        // This delay starts after the zoom wait. It is separate from the delay
        // after each focus pulse, which lets the ISP focus metric catch up with lens movement.
        msleep(AF_ZOOM_POST_SETTLE_MS);
    }
    if (af_cancel) {
        return;                       // preempted before we even took the port
    }

    float mag_now = af_zoom_mag();
    if (af_algorithm == AF_ALGORITHM_AF2 && mag_now < 1.0f) {
        af_set_result("failed: af2 needs zoom magnification");
        return;
    }

    AfMotor motor;
    enum AfMotorOpenResult open_result =
        af_motor_open(&motor, AF_MOTOR_AXIS_FOCUS, af_motor_cancelled, NULL);
    if (open_result != AF_MOTOR_OPEN_OK) {
        af_set_result(open_result == AF_MOTOR_OPEN_BUSY
                          ? "failed: focus port is busy"
                          : "failed: cannot open focus port");
        return;
    }

    unsigned before = 0;
    if (!fv_sample(&before)) {
        af_set_result("failed: no focus statistic");
        goto out;
    }

    AfIO io = {.drive = af_io_drive,
               .pulse_ms = af_io_pulse,
               .fv = af_io_fv,
               .now_ms = af_io_now,
               .sleep_ms = af_io_sleep,
               .progress = af_io_progress,
               .ctx = &motor};
    // Optional per-pass trace for offline diagnosis: enabled by touching /tmp/af_trace.on and
    // written to /tmp/af_trace.csv. The first column is the estimated focus position.
    FILE *trf = access("/tmp/af_trace.on", F_OK) == 0 ? fopen("/tmp/af_trace.csv", "w") : NULL;
    // A controller without a magnification report cannot use the calibrated curve. Follow the
    // live focus metric from the position where its own focus matching left the lens.
    if (trf) fprintf(trf, af_algorithm == AF_ALGORITHM_BLIND_SEEK
                             ? "pos,fv,phase,sample_start_ms,sample_end_ms,last_direction,last_pulse_ms\n"
                             : "pos,fv\n");
    unsigned final, peak;
    bool converged = true;
    unsigned start_ref = before;
    long final_pos;
    int steps, path;
    if (af_algorithm == AF_ALGORITHM_BLIND_SEEK) {
        AfBlindSeekParams lp = {.micro_ms = 40,
                            .nudge_ms = 70,
                            .recovery_ms = 160,
                            .settle_ms = 300,
                            .budget_ms = 30000,
                            .fv_samples = 5,
                            .fv_frame_ms = 40,
                            .direction_ms = 5000,
                            .sweep_ms = 12000,
                            .live_sample_ms = 40,
                            .min_improve_percent = 2,
                            .drop_percent = 12,
                            .trace_sample = trf ? af_trace_sample : NULL,
                            .trace_ctx = trf,
                            .cancel = &af_cancel};
        final = af_blind_seek_run(&io, &lp);
        converged = lp.out_converged;
        start_ref = lp.out_start_fv;
        peak = lp.out_peak_seen;
        final_pos = lp.out_focus_pos;
        steps = lp.out_steps;
        path = 3;
        af_focus_pos = -1;
        af_last_mag = -1.0f;
    } else {
        // Pick the starting position (see af2.h). Two cases, from the measured mechanics:
        //  - zoom changed since the last pass: the zoom displaced focus to ~peak+overshoot, so
        //    SEED there and let af2 sweep NEAR onto the peak instead of a ~40 s re-home.
        //  - same zoom as last pass: the dead-reckoned position is still valid (no zoom to
        //    disturb it) — af2 backs off to the far side and sweeps in.
        float dmag = mag_now - af_last_mag;
        if (dmag < 0) dmag = -dmag;
        bool zoomed = af_last_mag >= 1.0f && dmag > 0.05f;
        long in_pos;
        if (af_last_mag < 0) {
            in_pos = -1;                                        // cold re-home
        } else if (zoomed) {
            in_pos = af2_parfocal_foc(mag_now) + AF_ZOOM_OVERSHOOT_MS;
        } else {
            in_pos = af_focus_pos;                              // same zoom: position still holds
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
        final = af2_run(&io, &p);
        // A seeded pass that stays on the contrast floor missed the peak. Re-home unless a
        // new request preempted it.
        if (in_pos >= 0 && p.out_peak_seen < AF_FLOOR_FV && !af_cancel) {
            p.in_focus_pos = -1;
            final = af2_run(&io, &p);
        }
        peak = p.out_peak_seen;
        final_pos = p.out_focus_pos;
        steps = p.out_steps;
        path = p.out_path;
        af_focus_pos = p.out_focus_pos;
        af_last_mag = mag_now;
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
    if (peak == 0) {
        af_set_result("failed: lens does not respond");
        goto out;
    }
    if (path == 3 && (!converged || (unsigned long long)final * 100 <
                         (unsigned long long)start_ref * 95)) {
        snprintf(line, sizeof(line), "incomplete: start=%u final=%u peak=%u", start_ref,
                 final, peak);
        af_set_result(line);
        log_w("autofocus: %s", line);
        goto out;
    }

    snprintf(
        line, sizeof(line), "done fv=%u peak=%u start=%u mag=%.1f pos=%ld steps=%d path=%d",
        final, peak, before, (double)mag_now, final_pos, steps, path);
    af_set_result(line);
    log_i("autofocus: %s", line);

out:
    af_motor_stop(&motor);
    af_motor_close(&motor);
}

// Drive `n` zoom pulses in one direction (dir: +1 tele, -1 wide). The worker
// holds one zoom lease and starts a fresh focus pass after the pulses.
static void af_do_zoom(int dir, int n) {
    if (n <= 0) {
        return;
    }
    AfMotor motor;
    if (af_motor_open(&motor, AF_MOTOR_AXIS_ZOOM,
                      af_motor_cancelled, NULL) != AF_MOTOR_OPEN_OK) {
        return;
    }
    for (int i = 0; i < n; i++) {
        af_motor_zoom(&motor, dir);
        msleep(AF_ZOOM_PULSE_MS);
        af_motor_stop(&motor);
        if (i < n - 1) msleep(60);   // brief gap so the MCU registers separate steps
    }
    af_motor_close(&motor);
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
    unlink(AF_CANCEL_REQUEST);
    pthread_mutex_lock(&af_mu);
    if (af_shutdown) {   // tearing down: refuse new work so the joined worker stays joined
        pthread_mutex_unlock(&af_mu);
        return -1;
    }
    if (af_running_flag) {
        // A pass is already running for the previous position — a fresh trigger arrived.
        // Preempt it: cancel the current pass (it is chasing a now-stale magnification, and
        // cancelling releases the motor lease) and ask it to run once more, settled, for the new
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
    snprintf(af_result, sizeof(af_result), "running step=queued fv=0 peak=0");
    pthread_mutex_unlock(&af_mu);
    return af_spawn() ? 0 : -1;
}

int af_cancel_pass(void) {
    if (!af_available()) {
        return -1;
    }

    pthread_mutex_lock(&af_mu);
    if (af_shutdown) {
        pthread_mutex_unlock(&af_mu);
        return -1;
    }

    bool active = af_running_flag;
    af_cancel = 1;
    af_zoom_req = 0;
    af_restart_pending = false;
    af_restart_settle = false;
    if (active) snprintf(af_result, sizeof(af_result), "cancelling");
    pthread_mutex_unlock(&af_mu);
    return active ? 0 : 1;
}

void af_note_manual_focus(void) {
    pthread_mutex_lock(&af_mu);
    af_cancel = 1;
    af_zoom_req = 0;
    af_restart_pending = false;
    af_restart_settle = false;
    af_focus_pos = -1;
    af_last_mag = -1.0f;
    if (af_running_flag) snprintf(af_result, sizeof(af_result), "cancelling");
    pthread_mutex_unlock(&af_mu);
}

int af_zoom_pulse(int dir) {
    if (!af_available()) {
        return -1;
    }
    unlink(AF_CANCEL_REQUEST);
    bool start;
    pthread_mutex_lock(&af_mu);
    if (af_shutdown) {   // tearing down: refuse new work
        pthread_mutex_unlock(&af_mu);
        return -1;
    }
    af_zoom_req += dir > 0 ? 1 : -1;   // append the pulse; the worker drains it before focusing
    af_cancel = 1;                     // preempt any running focus and release its lease
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
    af_cancel = 1;   // a running pass abandons its moves, releases its lease, and returns
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
