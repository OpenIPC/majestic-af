// Contrast autofocus: drive the focus motor while watching the ISP's
// per-frame focus statistic, stop on the peak. The statistic comes through
// sdk_get_focus_value() (vendor-neutral; HiSilicon gen4 implements it from
// the BE AF zone grid), the motor through a UART protocol chosen at runtime
// from isp.autofocus.actuator — the XiongMai near-Pelco variant or standard
// Pelco-D. The wire itself is proto.c; the port and its arbitration, motion.c.
//
// The search itself lives in af2.c (a continuous UART lens) or af3.c (a
// microstep lens). The 85H50AI's lens MCU tracks focus through a zoom by
// itself and leaves it close to the crest, so af2 searches a short window
// around wherever the lens is, after the MCU has finished (af2.h). The scene's
// achievable maximum is not known in advance (the ceiling swings ~20x between
// daylight FV ~4000 and dusk ~250 on the same view), so nothing here compares
// against an absolute target.
//
// Serialisation: there is nothing to serialise against any more. motion.c owns
// the port outright — this pass drives the motor through motion_engine_drive(),
// which stands aside the instant an operator touches the pad. The WebUI's
// btzoom/btzoom-xm scripts, and the /tmp/btzoom.lock they were arbitrated with,
// are gone; a lock could never have made a three-step movement (drive, wait,
// stop) atomic against a second writer anyway.

#include <poll.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// The restoring magnification sink is referenced WEAKLY, and the pragma has to
// precede the ABI header so the attribute lands on the first declaration of the
// symbol in this file. It is the first seam added since the contract was
// frozen, and the core and the plugin are not always updated in one step: a
// strong reference would make RTLD_NOW refuse this .so against a core that
// predates it, so a camera whose lens worked would lose the motor driver
// outright in exchange for an accurate age_ms. Weak keeps that trade the right
// way up — where the core is older the pointer is null and af_zoom_restore()
// falls back to sdk_set_zoom_mag(), which is exactly today's behaviour.
#pragma weak sdk_set_zoom_mag_restored

#include <majestic/af.h>
#include <majestic/af2.h>
#include <majestic/af3.h>
#include <majestic/af_plugin_abi.h>   // HAL seams imported from the core: sdk_get_focus_value,
                                       // sdk_set_zoom_mag, config_get_*
#include <majestic/log.h>

#include "motion.h"
#include "proto.h"

// Timing budget of a pass. Guarded so the offline model harness (tests) can
// compress a full pass into milliseconds without patching the source; the
// values below are the production ones and are unchanged when nothing overrides
// them.
#ifndef AF_TOTAL_BUDGET_MS
// A hard cap, not a target. af3's full-travel sweep on a step lens takes tens of seconds; af2's
// search around the board's own focus takes ~5 s, ~15 s when it has to widen.
#define AF_TOTAL_BUDGET_MS 90000
#endif
#ifndef AF_SAMPLE_MS
#define AF_SAMPLE_MS 60
#endif
#ifndef AF_SETTLE_MS
#define AF_SETTLE_MS 160
#endif
// af3 (step lens) timing: one sensor frame, and how many frames to let the lens settle after a
// move before the focus statistic can be trusted (the ~300 ms post-move transient is gone by ~7
// frames at 25 fps; 5 balances that against pass time). Overridable so a host test can shrink them.
#ifndef AF_FRAME_MS
#define AF_FRAME_MS 40
#endif
#ifndef AF_SETTLE_FRAMES
#define AF_SETTLE_FRAMES 5
#endif
// af2's search window and the lens's reversal slack; 0 takes af2's measured defaults (af2.h).
// Guarded like the timings above so the host test can run a whole pass in milliseconds.
#ifndef AF_WINDOW_MS
#define AF_WINDOW_MS 0
#endif
#ifndef AF_WIDE_MS
#define AF_WIDE_MS 0
#endif
#ifndef AF_BACKLASH_MS
#define AF_BACKLASH_MS 0
#endif


static pthread_mutex_t af_mu = PTHREAD_MUTEX_INITIALIZER;
static bool af_running_flag = false;
// The worker has finished its pass and is only watching for the lens MCU's late focus move
// (af_watch_late_move): the status endpoint reports the pass's result, not "running".
static volatile bool af_watching = false;
static bool af_settle_first = false;
// Preemption: a fresh zoom while a pass runs sets af_cancel (the pass abandons its now-stale
// moves and drops the UART lock so the zoom can proceed) and requests one more pass, settled,
// for the new magnification. All three are written under af_mu; af_cancel is read lock-free by
// the running pass through the af2 cancel hook.
static volatile int af_cancel = 0;
static bool af_restart_pending = false;
static bool af_restart_settle = false;
static char af_result[96] = "idle";

// Plugin lifecycle: the worker (transient, one per pass) and the magnification
// reader (persistent) are JOINABLE, and af_engine_stop() joins both before the
// core dlclose()s this .so — a detached thread outliving the unmap would fault.
// af_shutdown refuses new work during teardown; af_reader_stop ends the reader's
// poll loop. All reset by af_engine_start() on (re)load.
static pthread_t af_worker;
static bool af_worker_valid = false;
static pthread_t af_reader;
static bool af_reader_valid = false;
static volatile int af_reader_stop = 0;
static volatile int af_shutdown = 0;

// The focus position the last pass landed at, and the magnification it was measured at. Only a
// step lens (af3) has one — its microstep count from the near stop; af2 keeps none. Touched only
// by the single running pass, so no lock. < 0 = unknown. af_last_mag also tells a pass whether
// the zoom has moved since the last one.
static long af_focus_pos = -1;
static float af_last_mag = -1.0f;
// Last step-lens (af3) pass's peak focus value, so a warm re-AF can tell "still in focus, hold" from
// "defocused, search" without moving the lens off a razor-thin crest. 0 = no trusted focus yet.
static unsigned af3_last_peak = 0;

// The after-zoom booking lives HERE, under af_mu, rather than beside the
// motion state it is timed from. It has to: motion.c would have to drop its
// own lock before calling in, and a manual focus arriving in that gap could
// clear a booking that had already been handed over and was about to run --
// which is exactly the pass this change exists to stop from running. Owning
// the flag and the decision to act on it in one critical section leaves no
// such gap. 0 = nothing booked.
static long af_book_at = 0;
// Bumped by every manual focus. A booking is taken at one value and refused if
// it has moved since: the watchdog reads it, decides, and calls in without
// holding its own lock, and that gap is long enough for an operator to set
// focus by hand between the two.
static unsigned af_focus_seq = 0;

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
    if (af_running_flag && !af_watching) {
        return "running";
    }
    // A shut port is not idleness. Focus and zoom share one descriptor, so while
    // it is closed there is no autofocus at all -- and this endpoint was the last
    // place that said otherwise: `idle` is what it answered on a camera that
    // could not move its lens, with only the word `closed` inside the /ptz
    // capability line to contradict it. Reported with the string a pass that
    // hits the same condition already uses, so every reader that words one
    // words the other.
    if (af_available() && !motion_is_open()) {
        return "failed: focus port is not open";
    }
    return af_result;
}

static long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void msleep(long ms) { usleep(ms * 1000); }


// --- zoom magnification reader ----------------------------------------------
// The XiongMai lens MCU reports its absolute zoom magnification upstream on the
// focus UART's RX line as ASCII "X<ratio> " (e.g. "X3.2 "), but only while the
// zoom motor is moving. This thread reads that stream on the descriptor motion.c
// owns and caches the latest value for the OSD `%@` token. The value persists
// between zooms (the lens stays where it was); -1 until the first report after
// boot.

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

// Where the zoom position is kept across a restart.
//
// The lens reports its magnification perfectly well -- it is what the OSD's
// "x2.7" shows -- but only WHILE the zoom motor is turning, so the value is
// learned on movement and never polled. Holding it in memory alone therefore
// throws away a fact about the hardware that did not change: restart majestic
// and the camera no longer knows where its own lens is, until somebody happens
// to zoom.
//
// That is not a cosmetic loss: the OSD and /zoom show it, and a pass compares
// against it to tell whether the zoom has moved since the last focus.
// Overridable so the host test can point it at a temp file, the way the AF_*_MS
// timing constants are overridable for the offline model.
#ifndef AF_ZOOM_STATE
#define AF_ZOOM_STATE "/etc/majestic-af.zoom"
#endif
// Written only once the lens has been still this long, so a zoom that sweeps
// through a dozen reported ratios costs one write and not a dozen. /etc is the
// jffs2 overlay on these boards, and an operator zooms a handful of times a day.
#define AF_ZOOM_SETTLE_MS 3000

// Waking the lens MCU is not a one-shot.
//
// After a cold power-up the MCU accepts NO Pelco command until it has seen the
// vendor wake blob -- measured on an 85H50AI, twelve zoom pulses moved nothing
// and reported nothing, and the same twelve moved the lens two pulses after the
// blob. It is not receptive to the blob immediately either: one sent when the
// plugin loads, ~12 s into the boot, is ignored, and the MCU first answers
// between 45 and 60 s of uptime. So the blob is re-sent until the lens speaks.
//
// "The lens has spoken" is af_zoom_ts: the MCU reports its magnification while
// the zoom motor turns, and nothing else stamps that this run. Bounded, because
// a protocol whose MCU never reports at all (pelco-d) would otherwise re-send
// for ever; the cap covers the measured window several times over.
#ifndef AF_WAKE_RETRY_MS
#define AF_WAKE_RETRY_MS 2000
#endif
#define AF_WAKE_TRIES 60

static long af_wake_next = 0;
static int af_wake_left = 0;
static bool af_wake_said = false;

static void af_wake_retry(void) {
    if (af_wake_left <= 0) {
        return;
    }
    pthread_mutex_lock(&af_zoom_mu);
    long spoke = af_zoom_ts;
    pthread_mutex_unlock(&af_zoom_mu);
    if (spoke) {
        af_wake_left = 0;   // the lens has answered; it is awake
        return;
    }
    long t = now_ms();
    if (t < af_wake_next) {
        return;
    }
    af_wake_next = t + AF_WAKE_RETRY_MS;
    if (!af_wake_said) {
        af_wake_said = true;
        log_i("autofocus: waking the lens MCU");
    }
    if (--af_wake_left == 0) {
        // Not necessarily broken: a protocol whose MCU only ever listens, or an
        // operator who has not touched the zoom, looks exactly like this.
        log_w("autofocus: the lens has not reported after %d wake attempts",
              AF_WAKE_TRIES);
    }
    motion_wake_blob();
}

static float af_zoom_saved = -1.0f;   // last value actually written out
static float af_zoom_failed = -1.0f;  // value whose write failed, so it is not retried
static long af_zoom_dirty_at = 0;     // when the live value last moved (0 = clean)

// The focus position is kept for the same reason the magnification is: a lens
// with no focus-position sensor knows it only by dead reckoning, and a restart
// throws that away. (Only a step lens has one to keep; af2 carries none.)
//
// Published here by the worker at the end of a pass, so the reader thread can
// write it out without reaching into the pass's own lockless bookkeeping.
static pthread_mutex_t af_fpos_mu = PTHREAD_MUTEX_INITIALIZER;
static long af_fpos_pub = -1;         // position to persist (< 0 = nothing to say)
static float af_fmag_pub = -1.0f;     // the magnification it was measured at
static bool af_fpos_dirty = false;

// A position is only meaningful with the magnification it was taken at: zooming
// mechanically displaces the focus element, so a carried position is trusted
// only for a same-zoom re-AF (see af_last_mag).
static void af_focus_publish(long pos, float mag) {
    pthread_mutex_lock(&af_fpos_mu);
    if (pos != af_fpos_pub || mag != af_fmag_pub) {
        af_fpos_pub = pos;
        af_fmag_pub = mag;
        af_fpos_dirty = true;
    }
    pthread_mutex_unlock(&af_fpos_mu);
}

// The position we held is no longer true of the lens. Unlike the publish above
// this ALWAYS marks the record stale, because the in-memory pair is often
// already (-1, -1) -- a boot that withheld the saved position leaves it that way
// -- while the FILE still holds a real one, eligible for a later restore within
// the same boot. Nothing else would ever rewrite it.
static void af_focus_invalidate(void) {
    pthread_mutex_lock(&af_fpos_mu);
    af_fpos_pub = -1;
    af_fmag_pub = -1.0f;
    af_fpos_dirty = true;
    pthread_mutex_unlock(&af_fpos_mu);
}

// Does the saved focus position belong to THIS boot?
//
// It has to, because the lens MCU exercises both motors at power-up, before
// Linux userspace exists. Measured on an 85H50AI they come back to where they
// were -- which is why the saved MAGNIFICATION is trusted unconditionally (a
// zoom parked mid-range at 2.6 read 2.6 again after a power cycle). The focus
// half of that measurement rests on a contrast statistic that a night scene
// renders nearly blind, so it is the weaker claim, and a wrong seed sends the
// search off from a fiction.
//
// Uptime cannot answer this. The plugin is loaded whenever isp.autofocus.enabled
// turns on or the pipeline is rebuilt, which is routinely long after boot, so a
// camera that power-cycled and had its lens enabled an hour later would look
// exactly like a restart. The kernel's per-boot UUID does answer it: it is
// regenerated on every boot and survives any number of majestic restarts within
// one. Unreadable, absent, or written by a build that did not record it -> the
// position is not trusted.
#ifndef AF_BOOTID_PATH
#define AF_BOOTID_PATH "/proc/sys/kernel/random/boot_id"
#endif
#define AF_BOOTID_MAX 48

static void af_boot_id(char *out, size_t n) {
    out[0] = 0;
    FILE *f = fopen(AF_BOOTID_PATH, "r");
    if (!f) {
        return;
    }
    if (!fgets(out, (int)n, f)) {
        out[0] = 0;
    }
    fclose(f);
    for (size_t i = strlen(out); i > 0 && (out[i - 1] == '\n' || out[i - 1] == '\r');) {
        out[--i] = 0;
    }
}

// `force` skips the settle wait: teardown has to take whatever is dirty now,
// because there is no later.
static void af_zoom_persist(bool force) {
    pthread_mutex_lock(&af_zoom_mu);
    float v = af_zoom_value;
    long dirty = af_zoom_dirty_at;
    pthread_mutex_unlock(&af_zoom_mu);

    pthread_mutex_lock(&af_fpos_mu);
    long fpos = af_fpos_pub;
    float fmag = af_fmag_pub;
    bool fdirty = af_fpos_dirty;
    pthread_mutex_unlock(&af_fpos_mu);

    // Either half can be what makes the file stale. The zoom half keeps its
    // settle wait (a sweep reports a dozen ratios and must cost one write); the
    // focus half is published once per pass and needs none.
    bool zoom_new = dirty && v > 0.0f && v != af_zoom_saved && v != af_zoom_failed;
    if (!zoom_new && !fdirty) {
        return;
    }
    if (zoom_new && !force && now_ms() - dirty < AF_ZOOM_SETTLE_MS) {
        return;   // still moving; wait for the lens to come to rest
    }
    if (v <= 0.0f) {
        return;   // no magnification yet, and a position without one says nothing
    }
    // A write that failed is not a write that happened. Marking it saved would
    // leave the file holding an older position and nothing ever correcting it,
    // which is worse than having no file at all: the next start would restore a
    // magnification the lens has since left. The failed value is remembered
    // instead, so a read-only rootfs is not retried once a second forever while
    // a NEW reading still gets its chance.
    FILE *f = fopen(AF_ZOOM_STATE, "w");
    if (!f) {
        af_zoom_failed = v;
        log_w("autofocus: cannot write %s; zoom position will not survive a restart",
              AF_ZOOM_STATE);
        return;
    }
    // Line 1 is the magnification, alone, exactly as it always was: a plugin
    // that predates the second line reads it with the same fscanf("%f") and
    // ignores the rest, so a downgrade loses the focus position and nothing else.
    // Line 2 carries the boot the position was measured in, so a later run can
    // tell a restart from a power cycle.
    char boot[AF_BOOTID_MAX];
    af_boot_id(boot, sizeof boot);
    bool ok = fprintf(f, "%.2f\n%ld %.2f %s\n", (double)v, fpos, (double)fmag,
                      boot[0] ? boot : "-") > 0;
    if (fclose(f) != 0) {
        ok = false;   // buffered content can fail at the flush, not at the write
    }
    if (!ok) {
        af_zoom_failed = v;
        log_w("autofocus: failed to save the zoom position to %s", AF_ZOOM_STATE);
        return;
    }
    af_zoom_saved = v;
    af_zoom_failed = -1.0f;
    pthread_mutex_lock(&af_zoom_mu);
    if (af_zoom_dirty_at == dirty) {
        af_zoom_dirty_at = 0;   // a report that landed mid-write stays dirty
    }
    pthread_mutex_unlock(&af_zoom_mu);
    pthread_mutex_lock(&af_fpos_mu);
    if (af_fpos_pub == fpos && af_fmag_pub == fmag) {
        af_fpos_dirty = false;   // a pass that landed mid-write stays dirty
    }
    pthread_mutex_unlock(&af_fpos_mu);
}

// Seed both caches from the last run. Not a measurement, so it deliberately
// does NOT stamp af_zoom_ts: the age stays "never seen this run", which is the
// truth, and /zoom goes on reporting a large age until the lens reports again.
static void af_zoom_restore(void) {
    FILE *f = fopen(AF_ZOOM_STATE, "r");
    if (!f) {
        return;
    }
    float v = 0.0f;
    long fpos = -1;
    float fmag = -1.0f;
    char saved_boot[AF_BOOTID_MAX] = "";
    int got = fscanf(f, "%f", &v);
    int got_focus = fscanf(f, "%ld %f %47s", &fpos, &fmag, saved_boot);
    fclose(f);
    if (got != 1 || !(v > 0.5f && v < 40.0f)) {
        return;   // the same range the reader trusts; a corrupt file is no value
    }
    // The focus position, if this run may believe it, paired with the
    // magnification it was measured at -- a position without one is not usable.
    char boot[AF_BOOTID_MAX];
    af_boot_id(boot, sizeof boot);
    if (got_focus == 3 && fpos >= 0 && fpos <= 42000 && fmag >= 1.0f &&
        fmag < 40.0f && boot[0] && !strcmp(boot, saved_boot)) {
        af_focus_pos = fpos;
        af_last_mag = fmag;
        af_focus_publish(fpos, fmag);
        pthread_mutex_lock(&af_fpos_mu);
        af_fpos_dirty = false;   // restored, not new: nothing to write back
        pthread_mutex_unlock(&af_fpos_mu);
        log_i("autofocus: focus position restored at %ld (x%.1f)",
              fpos, (double)fmag);
    }
    pthread_mutex_lock(&af_zoom_mu);
    af_zoom_value = v;
    pthread_mutex_unlock(&af_zoom_mu);
    af_zoom_saved = v;
    // Restored, not measured: the core must take the value without stamping it
    // as a fresh report, or /zoom would claim the lens had just spoken. Older
    // cores do not export that seam (the reference is weak, see the pragma at
    // the top), and there the value still lands — only its age is overstated.
    if (sdk_set_zoom_mag_restored) {
        sdk_set_zoom_mag_restored(v);
    } else {
        sdk_set_zoom_mag(v);
    }
    log_i("autofocus: zoom position restored as x%.1f", (double)v);
}

static void af_zoom_set(float v) {
    pthread_mutex_lock(&af_zoom_mu);
    af_zoom_value = v;
    af_zoom_ts = now_ms();
    af_zoom_dirty_at = af_zoom_ts;
    pthread_mutex_unlock(&af_zoom_mu);
    // Push to the CORE's cache too (sdk_set_zoom_mag is the imported seam), so the
    // OSD "%@" token and /zoom (GET) — which read from the core — reflect the
    // magnification this plugin's reader parsed. The local cache above is what
    // this plugin's own passes read through af_zoom_mag().
    sdk_set_zoom_mag(v);
}

// A backend that commands the zoom motor itself (act_ms41908) has no UART report
// to parse, so it derives magnification from its dead-reckoned zoom position and
// pushes it here — the same sink af_zoom_thread feeds. Stamping it as a fresh
// report is correct: the zoom just moved. (motion_fd() is -1 for such a backend,
// so af_zoom_thread and af_wake_retry never run and cannot contend with this.)
void af_zoom_report(float mag) {
    af_zoom_set(mag);
}

static void *af_zoom_thread(void *arg) {
    (void)arg;
    // The port belongs to motion.c, which opened it O_RDWR and set the line up
    // once. Reading from the same descriptor the writer uses is what keeps a
    // second tcsetattr from reconfiguring the line behind it mid-move.
    int fd = motion_fd();
    if (fd < 0) {
        return NULL;
    }

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
            // The 1 s timeout is also when a settled value gets written out.
            // Here rather than in af_zoom_set() so the file write stays off the
            // parsing path, and off this thread's hot loop while a zoom is
            // actually reporting.
            af_zoom_persist(false);
            af_wake_retry();
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
    return NULL;   // the descriptor is motion.c's; it closes it
}

// Bring the motor up: open the port (motion.c) and start the magnification
// reader on it. Called from the plugin's constructor at load and again after a
// reload, so it resets the teardown flags. The reader is JOINABLE —
// af_engine_stop() joins it before the core dlclose()s this .so.
bool af_alive(void) {
    return !af_shutdown;
}

// Spawn the magnification reader, once. Guarded by af_mu because motion_ready()
// can reach it from an HTTP thread on a late open while the constructor path is
// doing the same thing, and af_reader_valid is what teardown reads to decide
// whether there is anything to join.
void af_reader_ensure(void) {
    pthread_mutex_lock(&af_mu);
    if (af_reader_valid || af_shutdown || motion_fd() < 0) {
        pthread_mutex_unlock(&af_mu);
        return;
    }
    af_reader_stop = 0;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 0x10000);
    if (pthread_create(&af_reader, &attr, af_zoom_thread, NULL)) {
        log_e("autofocus: cannot start zoom reader");
    } else {
        af_reader_valid = true;
    }
    pthread_attr_destroy(&attr);
    pthread_mutex_unlock(&af_mu);
}

void af_engine_start(void) {
    if (!af_available()) {
        return;   // no focus motor declared: nothing to own and nothing to read
    }
    af_reader_stop = 0;
    af_shutdown = 0;
    motion_reset();   // a previous teardown latched the port shut
    af_wake_left = AF_WAKE_TRIES;
    af_wake_next = 0;
    af_wake_said = false;
    // Before anything can ask: put back what the last run measured. The lens
    // has not moved since, so this is current rather than stale.
    af_zoom_restore();
    // motion_ready(), not motion_start(): it sends the wake blob and starts the
    // reader on success, and -- the point -- a failure here is no longer the
    // verdict for the whole run. Every verb asks again.
    motion_ready();
    // Hand the restored magnification to a dead-reckoning backend so it re-anchors
    // its zoom origin (the stepper held position with no power) and reports from
    // the first move, instead of leaving /zoom on the stale restored value until
    // the operator drives fully wide. A no-op for the UART MCU, which is absolute.
    // Harmless if the port has not opened yet: the backend remembers the seed and
    // applies it on open.
    float restored = af_zoom_mag();
    if (restored >= 1.0f) {
        motion_seed_zoom(restored);
    }
}

// Revive the engine if a prior af_engine_stop() tore it down but this .so was
// never actually unmapped. The core keeps a second, RTLD_GLOBAL handle on this
// library to resolve the HAL seams, so when a pipeline rebuild reloads the
// plugin the dlclose() does not unmap it and the follow-up dlopen() returns the
// SAME mapping WITHOUT re-running the constructor -- af_engine_start() (which the
// constructor calls) never re-runs, motion_reset() never clears the teardown
// latch, and the motor stays shut with /ptz answering "unavailable" until a full
// restart. Symptom on the MS41908M board: change any isp.autofocus or pipeline
// setting in the WebUI and the PTZ pad goes dead.
//
// So bring it back here, driven by the first af_plugin_call after the reload --
// the pad polls /autofocus/status at mount, and every verb comes through too.
// Idempotent while the engine is up (the common case, a cheap flag read), and
// the claim under af_mu keeps two racing HTTP threads from both reviving it.
void af_engine_ensure(void) {
    pthread_mutex_lock(&af_mu);
    bool revive = af_shutdown;
    if (revive) {
        af_shutdown = 0;   // claim it; a racing caller now sees the engine as up
    }
    pthread_mutex_unlock(&af_mu);
    if (revive) {
        af_engine_start();
    }
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

// The focus search itself lives in af2.c. af.c owns the actuator, the focus statistic,
// the lock and the thread; it hands af2 those four operations through an AfIO.
static void af_io_drive(void *ctx, int dir) {
    (void)ctx;
    if (!motion_engine_drive(dir)) {
        // The wire belongs to an operator, or the write failed. Either way this
        // move did not happen, and af2 must not go on sampling and
        // dead-reckoning as though it had -- the rest of the pass would be
        // fiction, and the lens would start obeying it again the moment the
        // operator let go. Abandon it; a zoom still dirty books another.
        af_preempt_always();
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
// af3's step hook (microstep lens). Returns microsteps advanced; a refusal (a human took the
// wire) abandons the pass, exactly as af_io_drive does for af2 — af3 dead-reckons from what it
// commanded, so a silently dropped move would make the rest of the pass fiction.
static int af_io_step(void *ctx, int dir, int n) {
    (void)ctx;
    int moved = motion_focus_step(dir, n);
    if (moved < 0) {
        af_preempt_always();
        return 0;
    }
    return moved;
}

// Wait until the operator has stopped driving. motion.c knows this in-process
// now; it used to be inferred by watching a lock directory appear and vanish.
// A held button re-arms its deadline continuously, so the quiet window elapses
// only once the finger is off — the after-zoom pass therefore runs exactly
// once, after the zooming is over. Bounded, so a stuck caller cannot park a
// worker here for ever.
static void af_wait_settled(void) {
    long deadline = now_ms() + 8000;
    while (now_ms() < deadline && !af_cancel) {
        if (motion_idle_ms() >= 700) {
            return;
        }
        msleep(50);
    }
}

// Trajectory trace sink (see the AfParams.trace hook): one "pos,fv" line per measurement.
static void af_trace(void *ctx, long pos, unsigned fv) {
    FILE *f = ctx;
    if (f) fprintf(f, "%ld,%u\n", pos, fv);
}

// After a pass that followed a zoom: the lens MCU may still make one last focus move of its own
// (Actuator.zoom_late_ms), back onto its own curve, undoing the pass. Watch the picture until
// that window closes, and for a few reads in any case (the pass itself may have run past the
// window, and the move may have come during it); if the focus statistic falls clearly below the
// crest and stays there, the move came -- run one more pass. The reference is what the pass
// landed on, but never less than 90 % of the crest it saw: a move that came mid-pass leaves the
// landing itself low. Cheap: a statistic read every ~200 ms. A fresh zoom or a hand on the pad
// ends the watch (af_cancel), as it would end a pass. Returns 1 if it ran a second pass, with its
// outcome in *p and the FV that set it off in *trigger.
static int af_watch_late_move(AfIO *io, AfParams *p, unsigned landed, unsigned *trigger) {
    if (landed == 0) {
        return 0;
    }
    unsigned ref = landed;
    unsigned crest = (unsigned)((unsigned long long)p->out_peak_seen * 90 / 100);
    if (ref < crest) {
        ref = crest;
    }
    af_watching = true;
    int low = 0, reads = 0, redone = 0;
    while (!af_cancel && (motion_zoom_late_ms() > 0 || reads < 3)) {
        unsigned v;
        if (!fv_sample(&v)) {
            break;
        }
        reads++;
        low = (unsigned long long)v * 100 < (unsigned long long)ref * 80 ? low + 1 : 0;
        if (low >= 3) {
            log_i("autofocus: the lens moved focus after the pass (fv %u, landed %u, crest %u); "
                  "refocusing", v, landed, p->out_peak_seen);
            *trigger = v;
            af_watching = false;
            af2_run(io, p);
            redone = 1;
            break;
        }
        msleep(150);
    }
    af_watching = false;
    return redone;
}

// Publish an af2 pass's outcome. Returns 1 if it found focus.
static int af2_publish(const AfParams *p, unsigned final, unsigned before, float mag, bool late) {
    char line[112];
    if (af_cancel) {
        // A fresh zoom preempted this pass mid-drive: it was searching around the focus the
        // old zoom left. Drop it cleanly; the caller will run another after the new zoom.
        af_set_result("preempted");
        return 0;
    }
    unsigned peak = p->out_peak_seen;
    if (peak == 0) {
        af_set_result("failed: lens does not respond");
        return 0;
    }
    // No crest: nothing rose AND fell inside either window, so af2 has put the lens
    // back where the board's own tracking left it. That is a guess, not a
    // measurement, and calling it `done` would put a number on a pass that found
    // nothing -- `done fv=25 peak=27` was an 85H50AI at x1.0 reporting exactly this,
    // and a bright blank wall can hold the statistic well above any absolute floor.
    // The crest test is relative to the floor it found, so a dim scene that HAS a
    // crest still reports its result.
    if (!p->out_found_crest) {
        snprintf(line, sizeof(line), "failed: no contrast to focus on (peak=%u mag=%.1f)",
                 peak, (double)mag);
        af_set_result(line);
        log_i("autofocus: %s", line);
        return 0;
    }
    af_last_mag = mag;                // remember the zoom, to detect a change next pass
    snprintf(line, sizeof(line), "done fv=%u peak=%u start=%u mag=%.1f moved=%ld steps=%d window=%d%s",
             final, peak, before, (double)mag, p->out_landed_pos, p->out_steps, p->out_window,
             late ? " late=1" : "");
    af_set_result(line);
    log_i("autofocus: %s", line);
    return 1;
}

// One autofocus pass: wait for any in-flight manual move to settle, run the engine,
// land. Cancellable at any move via af_cancel — a fresh zoom or a pad press preempts
// it, and every frame it writes goes through motion_engine_drive(), which refuses
// while a human holds the wire. So a cancelled pass cannot fight the operator even
// in the moments before it notices.
static void af_run_one_pass(bool settle) {
    char line[96];

    if (settle) {
        af_wait_settled();
    }
    // Every pass, booked or started by hand, waits until the lens MCU has usually finished
    // moving focus after the last zoom (zoom_settle_ms): it goes on doing so for a few seconds
    // after the stop, to a point on its own curve, and would undo a pass that ran first. Its
    // rare later move (the 85H50AI: up to ~10 s, zoom_late_ms) is caught after the pass by
    // af_watch_late_move. A zoom arriving meanwhile cancels this pass (af_cancel) and books
    // its own.
    for (long w; (w = motion_zoom_settle_ms()) > 0 && !af_cancel;) {
        msleep(w < 100 ? w : 100);
    }
    if (af_cancel) {
        return;                       // preempted before we even took the port
    }

    if (!motion_ready()) {
        af_set_result("failed: focus port is not open");
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
               .ctx = NULL};   // the actuator is motion.c's, not a handle we carry
    // Optional per-pass FV(position) trace for offline diagnosis: enabled by touching
    // /tmp/af_trace.on, written to /tmp/af_trace.csv. Off (and zero cost) otherwise.
    FILE *trf = access("/tmp/af_trace.on", F_OK) == 0 ? fopen("/tmp/af_trace.csv", "w") : NULL;
    if (trf) fprintf(trf, "pos,fv\n");
    float mag_now = af_zoom_mag();
    float dmag = mag_now - af_last_mag;
    if (dmag < 0) dmag = -dmag;
    bool zoomed = af_last_mag >= 1.0f && dmag > 0.05f;

    // A microstep lens (the MS41908M) runs the vendor bracket-and-return search (af3) instead of
    // af2's timed sweep: step, let the lens SETTLE, read the statistic stopped, record the best and
    // drive back to it, bounded so it converges and falls silent rather than hunting. Home once for
    // an absolute reference (a no-op after the first pass); the search is FV-guided, so it tolerates
    // the dead reckoning drifting when a zoom displaces focus. Separate from the af2 path below,
    // which stays exactly as-is for the continuous (UART) lens it was tuned on.
    long f_steps = 0, f_backlash = 0;
    if (motion_focus_stepper(&f_steps, &f_backlash)) {
        // Warm iff a prior pass already homed the axis AND no zoom has displaced focus since: only
        // then may the lens already be on the crest. A cold pass (not yet homed) or a post-zoom pass
        // must sweep, never short-circuit on the "already focused?" test.
        int warm = motion_focus_pos() >= 0 && !zoomed;
        if (!motion_focus_home()) {                // absolute reference; rams once, then a no-op
            // Homing failed (the SPI lens is not answering). Without an anchor a later stationary
            // reading could still pass the checks and publish a guessed, unanchored position.
            af_focus_pos = -1;
            af3_last_peak = 0;
            af_focus_invalidate();
            af_set_result("failed: lens does not respond");
            if (trf) fclose(trf);
            goto out;
        }
        Af3IO io3 = {.step = af_io_step, .fv = af_io_fv, .now_ms = af_io_now,
                     .sleep_ms = af_io_sleep, .ctx = NULL};
        Af3Params p3 = {.focus_steps = f_steps,
                        .backlash_steps = f_backlash,
                        .frame_ms = AF_FRAME_MS,
                        .settle_frames = AF_SETTLE_FRAMES,
                        .fv_samples = 5,
                        .budget_ms = AF_TOTAL_BUDGET_MS,
                        .cancel = &af_cancel,
                        .in_pos = motion_focus_pos(),
                        .warm = warm,
                        // Hold if a warm re-AF finds the image already at least half as sharp as the
                        // last focus — enough to know we are on the crest, and not so high that a
                        // little scene change forces a needless (focus-disturbing) re-search.
                        .hold_fv = warm ? af3_last_peak / 2 : 0,
                        .trace = trf ? af_trace : NULL,
                        .trace_ctx = trf};
        unsigned final3 = af3_run(&io3, &p3);
        if (trf) fclose(trf);
        if (af_cancel) {
            af_focus_pos = -1;
            af_focus_invalidate();
            af_set_result("preempted");
            goto out;
        }
        unsigned peak3 = p3.out_peak_seen;
        if (peak3 == 0) {
            af_focus_pos = -1;
            af3_last_peak = 0;
            af_focus_invalidate();
            af_set_result("failed: lens does not respond");
            goto out;
        }
        // af3's crest flag is authoritative: it sets it only when FV rose to a real peak and fell
        // again. No crest means no focus was found -- a flat scene whose statistic merely sits above
        // a fixed floor must NOT be published as focused on the strength of that absolute value.
        if (!p3.out_found_crest) {
            af_focus_pos = -1;
            af3_last_peak = 0;
            af_focus_invalidate();
            snprintf(line, sizeof(line), "failed: no contrast to focus on (peak=%u mag=%.1f)",
                     peak3, (double)mag_now);
            af_set_result(line);
            log_i("autofocus: %s", line);
            goto out;
        }
        af_focus_pos = p3.out_pos;               // microsteps (this lens always uses af3)
        af3_last_peak = peak3;                    // the sharpness a warm re-AF holds against
        af_last_mag = mag_now;
        af_focus_publish(af_focus_pos, af_last_mag);
        snprintf(line, sizeof(line),
                 "done fv=%u peak=%u start=%u mag=%.1f pos=%ld steps=%d rev=%d",
                 final3, peak3, before, (double)mag_now, p3.out_pos, p3.out_steps, p3.out_reversals);
        af_set_result(line);
        log_i("autofocus: %s", line);
        goto out;
    }

    // A continuous (UART) lens: af2 searches around wherever the lens board left focus (af2.h).
    // Its reversal slack is the actuator's own if it knows it, else af2's measured default.
    long backlash_ms = AF_BACKLASH_MS;
    motion_actuator_backlash(&backlash_ms);
    AfParams p = {.backlash_ms = backlash_ms,
                  .settle_ms = AF_SETTLE_MS,
                  .budget_ms = AF_TOTAL_BUDGET_MS,
                  // Median several frames per measurement so a rain glint or a passing
                  // light can't be mistaken for sharpness in a dynamic night scene.
                  .fv_samples = 5,
                  .fv_frame_ms = 40,
                  .window_ms = AF_WINDOW_MS,
                  .wide_ms = AF_WIDE_MS,
                  .trace = trf ? af_trace : NULL,
                  .trace_ctx = trf,
                  .cancel = &af_cancel};
    // Whether this pass follows a zoom whose board may still move focus on its own.
    bool watch = motion_zoom_late_ms() > 0;
    unsigned final = af2_run(&io, &p);
    if (trf) fclose(trf);
    p.trace = NULL;            // the late-move pass below must not write to the closed file
    p.trace_ctx = NULL;
    // af2 carries no position (af2.h), and af2_run has already driven the motor,
    // so any position still held -- in memory AND in the state file -- describes a
    // place the lens has left. Drop it whichever way the pass ends.
    af_focus_pos = -1;
    af_focus_invalidate();   // and the copy that would outlive this process
    if (!af2_publish(&p, final, before, mag_now, false)) {
        goto out;
    }
    unsigned trigger = 0;
    if (watch && af_watch_late_move(&io, &p, final, &trigger)) {
        af2_publish(&p, p.out_peak_fv, trigger, mag_now, true);
    }

out:
    motion_engine_drive(0);
}

static void *af_thread(void *arg) {
    (void)arg;
    bool settle = af_settle_first;
    for (;;) {
        // af_cancel is NOT cleared here. Whoever asked for this pass cleared it
        // in the same critical section that set af_running_flag, so every
        // preemption raised after that point belongs to this pass and must
        // survive to be seen. Clearing it at the top of the loop opened a
        // window between the spawn and the first iteration in which a pad
        // press was silently discarded -- and the pass it should have stopped
        // then went on to undo the operator's focus, which is the whole defect.
        af_run_one_pass(settle);

        pthread_mutex_lock(&af_mu);
        if (af_shutdown || !af_restart_pending) {
            af_running_flag = false;
            pthread_mutex_unlock(&af_mu);
            return NULL;
        }
        // A freshly requested pass starts uncancelled; this is the only other
        // place a pass is asked for, so it is the only other place that clears.
        af_restart_pending = false;
        af_cancel = 0;
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
    //
    // Under af_mu throughout, including across the pthread_create: af_worker
    // and af_worker_valid are what teardown reads to decide whether there is
    // anything to join, and setting them after the thread exists but outside
    // the lock leaves a moment where a live worker looks like no worker. A
    // teardown arriving mid-create now waits here rather than walking past it.
    pthread_mutex_lock(&af_mu);
    if (af_worker_valid) {
        pthread_mutex_unlock(&af_mu);
        pthread_join(af_worker, NULL);   // the old one has exited; never blocks long
        pthread_mutex_lock(&af_mu);
        af_worker_valid = false;
    }
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 0x10000);
    bool ok = pthread_create(&af_worker, &attr, af_thread, NULL) == 0;
    pthread_attr_destroy(&attr);
    af_worker_valid = ok;
    if (!ok) {
        af_running_flag = false;
    }
    pthread_mutex_unlock(&af_mu);
    return ok;
}

int af_trigger(bool settle) {
    if (!af_available()) {
        return AF_TRIGGER_UNAVAILABLE;
    }
    // Answer the request the camera can actually perform. Without the port there
    // is no focus motor, and accepting the trigger only spawned a worker that
    // reached the same conclusion a moment later and left `started` standing as
    // the reply -- so the page was told a pass had begun on a camera with no
    // lens. motion_ready() also gives a port that failed to open at load its
    // chance here, which is often the press that fixes the camera.
    if (!motion_ready()) {
        return AF_TRIGGER_UNAVAILABLE;
    }
    pthread_mutex_lock(&af_mu);
    if (af_shutdown) {   // tearing down: refuse new work so the joined worker stays joined
        pthread_mutex_unlock(&af_mu);
        return AF_TRIGGER_UNAVAILABLE;
    }
    if (af_running_flag) {
        // A pass is already running for the previous position — a fresh trigger arrived.
        // Preempt it: cancel the current pass (it is chasing a now-stale magnification)
        // and ask it to run once more, settled, for the new one. Rapid triggers just keep
        // re-arming this, so the focus runs once after they stop.
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
    return af_ptz_move(dir > 0 ? PTZ_TELE : PTZ_WIDE, 0) ? 0 : -1;
}

// Every manual verb lands here. The move itself is motion.c's; what belongs to
// the engine is the consequence — a pass in flight is chasing a position the
// operator is currently changing, so it is cancelled, and motion.c books the
// follow-up focus once a zoom settles.
bool af_ptz_move(enum PtzVerb v, int ms) {
    if (!af_available() || af_shutdown) {
        return false;
    }
    if (v == PTZ_STOP) {
        return motion_halt();
    }
    return motion_move(v, ms);
}

// Cancel a running pass and ask for nothing in its place. Called by motion.c
// before a manual move takes the wire.
void af_preempt(void) {
    pthread_mutex_lock(&af_mu);
    if (af_running_flag) {
        af_cancel = 1;
    }
    // The restart an earlier trigger queued goes with it. Leaving it armed let
    // the worker clear the cancel at its restart point and start driving while
    // the operator still held the wire. A zoom that needs a follow-up focus
    // books one through af_book_after_zoom() once the pad goes quiet; it does
    // not need this queue.
    af_restart_pending = false;
    pthread_mutex_unlock(&af_mu);
}

// Cancel whatever is running, whether or not the flag says so. Used by the
// actuator hook when a move it was asked for did not reach the wire.
void af_preempt_always(void) {
    pthread_mutex_lock(&af_mu);
    af_cancel = 1;
    af_restart_pending = false;
    pthread_mutex_unlock(&af_mu);
}

// The operator moved focus by hand. Three things stop being true: the
// dead-reckoned position (they moved the element by an amount nothing
// counted), any restart a preempted pass had queued, and any booking an
// earlier zoom left behind. The cancel is raised here too, so this is enough
// on its own -- a caller does not have to remember to preempt as well.
void af_note_manual_focus(void) {
    // The operator moved the element by an amount nothing counted, so the saved
    // position is now a lie about the hardware -- and unlike the in-memory copy
    // it would outlive the process and seed the next run's first pass.
    af_focus_invalidate();
    pthread_mutex_lock(&af_mu);
    af_focus_seq++;
    af_focus_pos = -1;
    // The hand move left the image at an unknown sharpness, so the last pass's peak can no longer
    // vouch that we are focused: drop it, or a step-lens re-AF could take the no-motion warm-hold
    // path and republish "focused" without actually searching.
    af3_last_peak = 0;
    af_restart_pending = false;
    af_book_at = 0;
    if (af_running_flag) {
        af_cancel = 1;
    }
    pthread_mutex_unlock(&af_mu);
}

unsigned af_focus_gen(void) {
    pthread_mutex_lock(&af_mu);
    unsigned g = af_focus_seq;
    pthread_mutex_unlock(&af_mu);
    return g;
}

// Does the operator want the camera refocusing on its own?
//
// isp.autofocus.mode, in the vocabulary every other IP camera uses minus the
// value we do not have:
//
//   semi    (default) refocus after a zoom, and at no other time
//   manual  never refocus unasked; the /autofocus trigger still works
//
// There is no continuous AUTO, here or in config: this search is one-shot and
// moves the lens through the crest and back, so a mode that followed a scene
// would pump focus visibly every time it looked.
//
// Read per booking rather than cached at load, so saving the key takes effect
// on the next zoom instead of at the next restart -- config_get_string reads
// the tree the core has already reloaded. An unset or unrecognised value is
// `semi`, which is what every camera did before this key existed: a value this
// build does not know must not silently stop the camera focusing.
static bool af_refocus_after_zoom(void) {
    const char *m = config_get_string("isp.autofocus", "mode");
    return !(m && !strcmp(m, "manual"));
}

// A zoom has finished moving: run a focus pass at `at_ms` unless the operator
// touches focus first. Re-booking simply pushes the moment out.
void af_book_after_zoom(long at_ms, unsigned gen) {
    if (!af_refocus_after_zoom()) {
        return;
    }
    pthread_mutex_lock(&af_mu);
    if (gen == af_focus_seq) {
        af_book_at = at_ms;
    }
    // else: the operator set focus by hand after this booking was decided.
    // Their focus stands; arming it here would restore exactly the pass
    // af_note_manual_focus() had just cancelled.
    pthread_mutex_unlock(&af_mu);
}

// Called on every watchdog tick. Returns true when it started the booked pass.
// The check, the clear and the decision to run all happen in one critical
// section, so a manual focus either lands before it (and the booking is gone)
// or after it (and af_cancel stops the pass at its first check) -- there is no
// moment where a booking is in flight and no longer revocable.
bool af_book_tick(long now) {
    bool start = false;
    pthread_mutex_lock(&af_mu);
    if (af_book_at && now >= af_book_at) {
        af_book_at = 0;
        if (!af_shutdown && !af_running_flag) {
            af_cancel = 0;
            af_restart_pending = false;
            af_running_flag = true;
            af_settle_first = false;
            start = true;
        }
        // A pass already running is the follow-up focus this booking wanted.
    }
    pthread_mutex_unlock(&af_mu);
    if (start && !af_spawn()) {
        return false;
    }
    return start;
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

    // The watchdog goes FIRST. It can start an autofocus pass (af_book_tick),
    // so joining the worker before it is stopped leaves a window in which it
    // spawns one into a shutdown that has already decided there was nothing to
    // join — and that thread then outlives the dlclose.
    motion_stop_watchdog();

    if (af_worker_valid) {
        pthread_join(af_worker, NULL);
        af_worker_valid = false;
    }
    af_reader_stop = 1;
    if (af_reader_valid) {
        pthread_join(af_reader, NULL);
        af_reader_valid = false;
    }
    // The reader is the only thing that writes this file, and it has just been
    // joined, so this is the one moment a flush cannot race it. Forced, because
    // a zoom that reported inside the settle window is still dirty and the
    // reader exited before its next wake — without this, a restart within three
    // seconds of a zoom restores the position the lens was at BEFORE it.
    af_zoom_persist(true);
    // Last, because the reader polls this descriptor and the watchdog writes to
    // it: all three are joined by now, so nothing is left to touch a closed fd.
    motion_close();
}
