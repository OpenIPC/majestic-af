#include "motion.h"

#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <majestic/af.h>              // af_zoom_mag: its lock is a leaf, safe under mo_mu
#include <majestic/af_plugin_abi.h>   // config_get_* — the core's seams
#include <majestic/log.h>

#include "actuator.h"

// Auto-stop window when the caller names none, and the bounds a caller may ask
// for. The default is the 500 ms the btzoom scripts used for one press, so a
// tap moves the lens exactly as far as it always did; a held button re-arms it
// every ~250 ms and the motion is continuous instead of stuttering.
#ifndef MOTION_DEFAULT_MS
#define MOTION_DEFAULT_MS 500
#endif
#define MOTION_MIN_MS 50
#define MOTION_MAX_MS 3000

// How long the wire must be quiet after a manual zoom before the follow-up
// focus pass starts. A held button re-arms the deadline every tick, so this
// elapses only once the operator has let go. A lens that goes on moving focus
// after the stop (Actuator.zoom_settle_ms) makes the wait that long instead.
#define MOTION_BOOK_QUIET_MS 700

// Watchdog tick. Deadlines land within a tick of where they were asked for,
// which is far finer than the motor's own response.
#define MOTION_TICK_MS 20

// How often a REPEATED open failure may be logged. The first one always is.
#define MOTION_OPEN_LOG_MS 10000

static pthread_mutex_t mo_mu = PTHREAD_MUTEX_INITIALIZER;
// The transport, behind the vtable: how a verb becomes motion and where (if
// anywhere) magnification comes from. Everything below is the arbitration that is
// the same whatever the wire — it reaches the wire ONLY through mo_act.
static const Actuator *mo_act;
static bool mo_open = false;              // the transport is open and owned
// The pan/tilt head beside the lens (ptz.head; actuator_select_head), NULL on
// every camera that has one motor. It takes the pan/tilt verbs and nothing else:
// focus, zoom, the wake, the magnification reader and the ICR stay with mo_act,
// the lens, so a dome's base never sees a lens MCU's frames. The lens is still
// what makes the camera open (mo_open); a head that will not open leaves zoom
// and focus working and is tried again on the next request.
static const Actuator *mo_head;
static bool mo_head_open = false;
// The pan/tilt motors turn the other way from the picture (ptz.panInvert,
// ptz.tiltInvert): a dome whose sensor sits upside down to its base has every
// arrow reversed even once the picture is set upright, because the core turns
// each direction by the picture's mirror and flip, and those say nothing about
// how the base is mounted (OpenIPC/firmware#2385). Read at open, applied to
// every pan/tilt verb before it is routed.
static bool mo_pan_invert = false;
static bool mo_tilt_invert = false;
// A go-to-preset is on the wire (motion_preset): the head drives itself to a stored
// position for as long as that takes, so the move's deadline (ptz.presetMs) ends the
// move WITHOUT a stop -- a stop there would halt the head short of the preset. Any
// other verb, or an operator's stop, ends it the ordinary way.
static bool mo_preset_running = false;
static bool mo_head_failed = false;       // as mo_open_failed, for the head
static long mo_head_last_fail_log;
static enum PtzVerb mo_verb = PTZ_STOP;   // the manual move now running
static bool mo_reported_moving = false;   // what sdk_ptz_motion was last told
// Focus moving with no verb on the wire, as the core was last told it (on the
// focus axis): the engine's pass driving it (motion_engine_drive, which goes
// past mo_verb, so the verb report never sees it), or the lens MCU still moving
// it by itself after a zoom stop (until mo_zoom_settle_until). One flag for
// both, so neither ends the other's report.
static bool mo_engine_driving = false;
static bool mo_reported_focus = false;
static long mo_deadline;                  // when to stop it
static bool mo_counted = false;           // mo_verb is a counted move (motion_step)
static int mo_axes = 0;                   // the axes its deadline covers: 1 pan, 2 tilt
static long mo_idle_since;                // when manual motion last ended
// The lens MCU goes on moving focus by itself after a zoom stops (Actuator.zoom_settle_ms).
// A zoom verb is running or has run in this move (mo_zoom_moving), and until when the lens
// may still be moving focus after it (mo_zoom_settle_until, now_ms() time; 0 = never).
static bool mo_zoom_moving;
static long mo_zoom_settle_until;
static long mo_zoom_late_until;   /* until when the lens may still make a last move (zoom_late_ms) */
// A zoom-out's end bounce (Actuator.zoom_out_bounce_ms): 1 = carrying the zoom-out on past the
// stop, 2 = zooming back in, 3 = a stop that missed the wire, being retried (no new bounce);
// 0 = none. Any new verb abandons it.
static int mo_bounce;
static float mo_bounce_mag;    // the reported magnification where the zoom-out stopped
static bool mo_bounce_guided;  // the outward leg reached the wide stop: come back to mo_bounce_mag
// A zoom moved the focus element and nothing has re-focused since. This
// outlives the verb that set it: an operator who zooms and then pans still
// wants the follow-up focus, and reading it off the last verb alone lost it
// the moment any other command arrived. Cleared when the pass runs, or when
// the operator sets focus by hand and makes it their business instead.
static bool mo_zoom_dirty;
static bool mo_rebook;                    // tell the engine to (re)arm the booking
static pthread_t mo_thread;
static bool mo_thread_valid = false;
static volatile int mo_run = 0;
// An open has failed and has not succeeded since. Drives the rate-limited log
// above and the "opened on retry" line that closes it out.
static bool mo_open_failed = false;
static long mo_last_fail_log;
// Teardown has begun: refuse to (re)open. Set under mo_mu by
// motion_stop_watchdog() BEFORE it joins, and the watchdog is created under the
// same lock, so a late motion_ready() either creates a thread teardown will
// still see or is refused. Without both halves a request arriving in that window
// starts a watchdog after the only join has run, and it outlives the dlclose.
static bool mo_down = false;

static long now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void msleep(long ms) { usleep(ms * 1000); }

int motion_default_ms(void) {
    int ms = config_get_int("ptz", "pulse");
    if (ms < MOTION_MIN_MS || ms > MOTION_MAX_MS) {
        return MOTION_DEFAULT_MS;   // unset, or a value the schema should have refused
    }
    return ms;
}

// Which actuator carries v: the head for pan/tilt when there is one (nobody
// while it is not open), the lens for everything else. Under mo_mu.
static const Actuator *owner(enum PtzVerb v) {
    if (mo_head && ptz_verb_is_pantilt(v)) {
        return mo_head_open ? mo_head : NULL;
    }
    return mo_act;
}

// One verb onto the wire, through the actuator that carries it. mo_mu is held;
// emit() is non-blocking and serialises on the backend's own lock (never mo_mu).
// A stop goes to the head too, but whether it reached the wire is the answer of
// the one carrying the move being stopped: a base that has gone quiet must not
// leave a lens stop retried forever, nor the other way round.
static bool emit_locked(enum PtzVerb v) {
    if (!mo_open || !mo_act) {
        return false;
    }
    if (v == PTZ_STOP && mo_head) {
        bool lens_ok = mo_act->emit(PTZ_STOP, 0);
        bool head_ok = mo_head_open && mo_head->emit(PTZ_STOP, 0);
        return mo_verb != PTZ_STOP && owner(mo_verb) == mo_head ? head_ok : lens_ok;
    }
    const Actuator *a = owner(v);
    return a && a->emit(v, 0);
}

// The lens alone, for the engine's focus drive: its trailing stop is no business
// of the head's.
static bool emit_lens_locked(enum PtzVerb v) {
    return mo_open && mo_act && mo_act->emit(v, 0);
}

#pragma weak sdk_ptz_motion

void ptz_motion_report(int moving, int axis, long upper_ms) {
    if (sdk_ptz_motion) {
        sdk_ptz_motion(moving, axis, upper_ms);
    }
}

static int verb_axis(enum PtzVerb v) {
    switch (v) {
    case PTZ_LEFT: case PTZ_RIGHT: return 0;
    case PTZ_UP: case PTZ_DOWN: return 1;
    case PTZ_TELE: case PTZ_WIDE: return 2;
    // Focus is reported too: a pass blurs and sharpens the whole picture, and
    // the core holds motion detection back for it as it does for a turn
    // (OpenIPC/majestic#347) -- without capping the exposure the pass measures.
    case PTZ_NEAR: case PTZ_FAR: return PTZ_AXIS_FOCUS;
    default: return -1;
    }
}

// Report a moving/still transition of the verb now on the wire, for a backend
// that cannot see its motor itself (the UART lenses: the verb is all we know).
// Called under mo_mu after every change of mo_verb; sdk_ptz_motion never
// blocks or calls back, so that is safe.
static void report_motion_locked(void) {
    if (!mo_act || (!mo_head && mo_act->reports_motion)) {
        return;
    }
    // A manual command on the wire has taken the lens from the engine: its
    // pass is over, and its trailing stop will be refused (motion_engine_drive),
    // so the drive must not go on being reported once the operator lets go.
    if (mo_verb != PTZ_STOP && !mo_act->reports_motion)
        mo_engine_driving = false;
    int axis = verb_axis(mo_verb);
    // With a head, the two motors may report differently: a lens that sees its
    // motor reports its own moves, and only the other's are reported from here.
    const Actuator *o = mo_verb != PTZ_STOP ? owner(mo_verb) : NULL;
    bool self = o && o->reports_motion;
    // A diagonal moves pan and tilt both, which no single axis names: it is
    // reported moving on axis -1, the ABI's "unknown".
    bool moving = !self && (axis >= 0 || ptz_verb_is_pantilt(mo_verb));
    if (moving == mo_reported_moving) {
        return;
    }
    mo_reported_moving = moving;
    // A stop on axis -1 ends every axis at the core, a focus drive the
    // operator interrupted among them.
    if (!moving)
        mo_reported_focus = false;
    long left = moving ? mo_deadline - now_ms() : -1;
    ptz_motion_report(moving, axis, left > 0 ? left : -1);
}

// Tell the core when focus starts or stops moving on its own (mo_reported_focus).
// Under mo_mu; called wherever either half of it may have changed, and on every
// watchdog tick for the settle running out.
static void report_focus_locked(void) {
    if (!mo_act || mo_act->reports_motion) {   // focus is the lens's alone
        return;
    }
    bool busy = mo_engine_driving || (mo_verb == PTZ_STOP && now_ms() < mo_zoom_settle_until);
    if (busy == mo_reported_focus) {
        return;
    }
    mo_reported_focus = busy;
    ptz_motion_report(busy, PTZ_AXIS_FOCUS, -1);
}

// End the manual move: stop the motor, remember when the wire went quiet, and
// ask for the follow-up focus if a zoom is still waiting for one. Every move
// that ends re-arms it, so the pass waits out a whole session at the pad
// rather than the last verb of it. A manual focus move never leaves one armed:
// the operator set the focus by hand and a pass would simply undo it, which is
// the defect this replaced.
// `bounce` false: stop now, whatever (a stop asked for during a zoom-out's end bounce).
static void end_move_locked(bool bounce_ok) {
    long bounce = bounce_ok && mo_act && mo_act->zoom_out_bounce_ms ? mo_act->zoom_out_bounce_ms() : 0;
    // Not into the wide stop: there the zoom cannot be carried on, the bounce would only leave
    // the lens short of the widest view (X1.1-1.2 measured), and the after-zoom pass sees to
    // focus. The board reports X1.0 there.
    float mag = af_zoom_mag();
    bool at_wide = mag > 0.0f && mag < 1.05f;
    if (bounce > 0 && mo_verb == PTZ_WIDE && mo_bounce == 0 && !at_wide) {
        mo_bounce = 1;                           // keep zooming out a little past the stop
        mo_bounce_mag = mag;
        mo_bounce_guided = false;
        mo_deadline = now_ms() + bounce;
        return;
    }
    if (bounce > 0 && mo_bounce == 1) {   // (3, retrying a stop, falls through to the stop)
        if (emit_locked(PTZ_TELE)) {             // then back in by as much
            mo_verb = PTZ_TELE;
            mo_bounce = 2;
            // If the outward leg ran into the wide stop it moved the zoom less than its time:
            // come back by the reported magnification instead, to where the zoom-out stopped.
            mo_bounce_guided = at_wide && mo_bounce_mag > 0.0f;
            mo_deadline = now_ms() + (mo_bounce_guided ? 2 * bounce : bounce);
            return;
        }
        // The zoom-in did not reach the wire: just stop.
    }
    mo_bounce_guided = false;
    if (mo_preset_running) {
        mo_preset_running = false;   // the head stops at the preset by itself
    } else if (!emit_locked(PTZ_STOP)) {
        mo_bounce = 3;   // the retry is a stop, not the start of a new bounce
        // The stop did not reach the wire. Saying the move ended would retire
        // the only thing that will try again, while the motor keeps driving.
        // Leave it running and let the next tick have another go.
        mo_deadline = now_ms() + MOTION_TICK_MS;
        return;
    }
    mo_bounce = 0;
    mo_verb = PTZ_STOP;
    mo_idle_since = now_ms();
    report_motion_locked();
    if (mo_zoom_moving) {
        mo_zoom_moving = false;
        mo_zoom_settle_until = mo_idle_since + (mo_act ? mo_act->zoom_settle_ms : 0);
        mo_zoom_late_until = mo_idle_since + (mo_act ? mo_act->zoom_late_ms : 0);
        // The lens MCU goes on pulling focus along its curve after the stop:
        // still the camera changing its own picture, for as long as it does.
        report_focus_locked();
    }
    if (mo_zoom_dirty) {
        mo_rebook = true;   // the watchdog arms it outside this lock
    }
}

static void *motion_thread(void *arg) {
    (void)arg;
    while (mo_run) {
        bool rebook;
        // Read the focus generation BEFORE taking mo_mu, so the booking below
        // carries the number it was decided at. A manual focus that lands any
        // time after this makes the number stale and the engine refuses the
        // booking; one that lands later still clears af_book_at directly. The
        // two together leave no window where a booking survives the operator
        // setting focus by hand. (Read outside mo_mu because it takes af_mu,
        // and nothing here may hold both.)
        unsigned gen = af_focus_gen();
        pthread_mutex_lock(&mo_mu);
        long t = now_ms();
        if (mo_verb != PTZ_STOP &&
            (t >= mo_deadline ||
             (mo_bounce == 2 && mo_bounce_guided && af_zoom_mag() >= mo_bounce_mag - 0.001f))) {
            end_move_locked(true);
        }
        report_focus_locked();   // a zoom's settle running out
        rebook = mo_rebook;
        mo_rebook = false;
        long quiet = mo_act && mo_act->zoom_settle_ms > MOTION_BOOK_QUIET_MS
                         ? mo_act->zoom_settle_ms
                         : MOTION_BOOK_QUIET_MS;
        pthread_mutex_unlock(&mo_mu);

        // Outside mo_mu, always: the engine's calls take af_mu and may spawn a
        // worker that calls straight back into motion_engine_drive(). Nothing
        // here holds both locks, in either order.
        if (rebook) {
            af_book_after_zoom(t + quiet, gen);
        }
        if (af_book_tick(t)) {
            pthread_mutex_lock(&mo_mu);
            mo_zoom_dirty = false;   // the follow-up focus is running
            pthread_mutex_unlock(&mo_mu);
        }
        msleep(MOTION_TICK_MS);
    }
    return NULL;
}

// Open the head, logging a failure the way the lens's is (once, then at most
// one line per MOTION_OPEN_LOG_MS: every request retries it). Under mo_mu, with
// the lens open.
static void head_open_locked(void) {
    if (!mo_head || mo_head_open) {
        return;
    }
    if (!mo_head->open()) {
        long t = now_ms();
        if (!mo_head_failed || t - mo_head_last_fail_log >= MOTION_OPEN_LOG_MS) {
            log_e("ptz: cannot open the %s pan/tilt head", mo_head->name);
            mo_head_last_fail_log = t;
        }
        mo_head_failed = true;
        return;
    }
    if (mo_head_failed) {
        log_i("ptz: %s pan/tilt head opened on retry", mo_head->name);
        mo_head_failed = false;
    }
    mo_head_open = true;
}

static void head_close_locked(void) {
    if (mo_head && mo_head_open) {
        mo_head->close();
    }
    mo_head_open = false;
    mo_head = NULL;
}

bool motion_start(void) {
    pthread_mutex_lock(&mo_mu);
    if (mo_down) {
        pthread_mutex_unlock(&mo_mu);
        return false;   // tearing down; the port stays shut and unowned
    }
    if (mo_open) {
        pthread_mutex_unlock(&mo_mu);
        return true;
    }
    const char *name = config_get_string("ptz", "actuator");
    mo_act = actuator_select(name);

    if (!mo_act->open()) {
        // The first failure is always logged; after that at most one line per
        // MOTION_OPEN_LOG_MS. Every verb retries the open now (motion_ready), and
        // a held button re-sends its verb every ~250 ms, so an unopenable port
        // would otherwise write four lines a second into a syslog ring that is
        // the only record of anything else going wrong.
        long t = now_ms();
        if (!mo_open_failed || t - mo_last_fail_log >= MOTION_OPEN_LOG_MS) {
            log_e("ptz: cannot open the %s actuator", mo_act->name);
            mo_last_fail_log = t;
        }
        mo_open_failed = true;
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    if (mo_open_failed) {
        // The condition cleared. Say so: the failure above was logged, and an
        // operator who saw it has no other way to learn that the lens came back.
        log_i("ptz: %s actuator opened on retry", mo_act->name);
        mo_open_failed = false;
    }
    mo_open = true;

    // The head, if this camera has one, is selected against the lens just
    // opened: a lens that pans, or one on the same transport, takes none.
    const char *why = NULL;
    const char *head = config_get_string("ptz", "head");
    mo_head = actuator_select_head(head, mo_act, &why);
    if (why) {
        log_w("ptz: ignoring the %s head: %s", head, why);
    }
    mo_head_open = false;
    head_open_locked();
    mo_pan_invert = config_get_boolean("ptz", "panInvert");
    mo_tilt_invert = config_get_boolean("ptz", "tiltInvert");

    mo_verb = PTZ_STOP;
    mo_idle_since = now_ms();
    mo_zoom_dirty = false;
    mo_zoom_moving = false;
    mo_zoom_settle_until = 0;
    mo_zoom_late_until = 0;
    mo_bounce = 0;
    mo_rebook = false;
    mo_run = 1;

    // Under mo_mu across the create, and mo_thread_valid published inside it --
    // the same reason af_spawn() holds af_mu across its own: mo_thread_valid is
    // what teardown reads to decide whether there is anything to join, and
    // setting it after the thread exists but outside the lock leaves a moment
    // where a live watchdog looks like no watchdog.
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 0x10000);
    int rc = pthread_create(&mo_thread, &attr, motion_thread, NULL);
    pthread_attr_destroy(&attr);
    if (!rc) {
        mo_thread_valid = true;   // JOINABLE: motion_stop joins before dlclose
    }
    pthread_mutex_unlock(&mo_mu);
    if (rc) {
        // Without the watchdog nothing enforces a deadline, so a move would
        // start a motor that nothing ever stops. Give the port back and report
        // failure: a camera with no PTZ is a great deal better than one whose
        // lens drives into its end stop and stays there, and leaving the
        // transport open would also make every later retry take the already-open
        // fast path and inherit the same state.
        log_e("ptz: cannot start the motion watchdog: %s", strerror(rc));
        pthread_mutex_lock(&mo_mu);
        mo_run = 0;
        if (mo_open) {
            head_close_locked();
            mo_act->close();
            mo_open = false;
        }
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    return true;
}

void motion_reset(void) {
    pthread_mutex_lock(&mo_mu);
    mo_down = false;
    pthread_mutex_unlock(&mo_mu);
}

bool motion_ready(void) {
    pthread_mutex_lock(&mo_mu);
    bool open_ = mo_open;
    pthread_mutex_unlock(&mo_mu);
    if (open_) {
        // A head that would not open is tried again here, as a lens is below:
        // a base powered after the camera must not need a restart to be driven.
        pthread_mutex_lock(&mo_mu);
        if (mo_open && !mo_down) {
            head_open_locked();
        }
        pthread_mutex_unlock(&mo_mu);
        // Idempotent, and asked EVERY time rather than only on the transition:
        // if the reader's pthread_create failed once, the transport stays open
        // and this fast path would otherwise be the only one ever taken again --
        // leaving no magnification reader and, with it, nothing to drive the
        // wake retry that gets an asleep MCU listening.
        af_reader_ensure();
        return true;
    }
    if (!af_alive()) {
        return false;   // tearing down; the port must stay shut
    }
    if (!motion_start()) {
        return false;
    }
    // The MCU accepts nothing until it has seen the wake blob. A failed write is
    // worth saying out loud, but it does NOT make the port unready: the reader's
    // retry sends it again, and refusing readiness here would disable the very
    // thing that recovers it. A wire that is genuinely broken surfaces where it
    // should -- the next move reports "unavailable", because its frame fails too.
    if (!motion_wake_blob()) {
        log_w("ptz: could not send the lens wake sequence");
    }
    af_reader_ensure();
    return true;
}

bool motion_wake_blob(void) {
    pthread_mutex_lock(&mo_mu);
    bool ok = mo_open && mo_act && mo_act->wake_blob();
    pthread_mutex_unlock(&mo_mu);
    return ok;
}

void motion_stop_watchdog(void) {
    pthread_mutex_lock(&mo_mu);
    mo_down = true;   // before the join, so a late open cannot slip past it
    mo_run = 0;
    bool join = mo_thread_valid;
    pthread_t t = mo_thread;
    mo_thread_valid = false;
    pthread_mutex_unlock(&mo_mu);
    if (join) {
        pthread_join(t, NULL);
    }
}

void motion_close(void) {
    pthread_mutex_lock(&mo_mu);
    if (mo_open) {
        if (mo_verb != PTZ_STOP && mo_preset_running) {
            mo_preset_running = false;   // the head finishes the go-to by itself
            mo_verb = PTZ_STOP;
            report_motion_locked();
        }
        if (mo_verb != PTZ_STOP) {
            emit_locked(PTZ_STOP);   // never leave a motor running behind us
            mo_verb = PTZ_STOP;
            report_motion_locked();
        }
        // A focus drive still running when the port goes: the motor stops
        // with it, and the core must not go on holding detection back for it.
        if (mo_engine_driving) {
            emit_locked(PTZ_STOP);
        }
        mo_engine_driving = false;
        mo_zoom_settle_until = 0;
        report_focus_locked();
        head_close_locked();
        mo_act->close();   // releases the transport LAST, joining its own threads
        mo_open = false;
    }
    mo_zoom_dirty = false;
    mo_rebook = false;
    pthread_mutex_unlock(&mo_mu);
}

int motion_fd(void) {
    pthread_mutex_lock(&mo_mu);
    int fd = (mo_open && mo_act) ? mo_act->fd() : -1;
    pthread_mutex_unlock(&mo_mu);
    return fd;
}

// Transport readiness, independent of motion_fd(): a backend can be open and
// driving with no descriptor to share (the MS41908M SPI stepper, whose fd() is -1
// precisely so no magnification reader starts). No side effects — it does not open.
bool motion_is_open(void) {
    pthread_mutex_lock(&mo_mu);
    bool o = mo_open;
    pthread_mutex_unlock(&mo_mu);
    return o;
}

bool motion_actuator_derives_mag(void) {
    pthread_mutex_lock(&mo_mu);
    bool d = mo_act && mo_act->derives_mag;
    pthread_mutex_unlock(&mo_mu);
    return d;
}

// Hand a dead-reckoning backend the magnification restored from the last run, so
// it can re-anchor its zoom origin without a physical seek. A no-op (false) for a
// backend with no seed_mag (the UART MCU reports absolute position on its own).
bool motion_seed_zoom(float mag) {
    pthread_mutex_lock(&mo_mu);
    bool (*seed)(float) = mo_act ? mo_act->seed_mag : NULL;
    pthread_mutex_unlock(&mo_mu);
    return seed ? seed(mag) : false;
}

bool motion_seed_focus(long pos) {
    pthread_mutex_lock(&mo_mu);
    bool (*seed)(long) = mo_act ? mo_act->seed_focus : NULL;
    pthread_mutex_unlock(&mo_mu);
    return seed ? seed(pos) : false;
}

bool motion_focus_rehomes(void) {
    pthread_mutex_lock(&mo_mu);
    bool (*h)(void) = mo_open && mo_act ? mo_act->homes_focus : NULL;
    pthread_mutex_unlock(&mo_mu);
    return h ? h() : false;
}

bool motion_actuator_backlash(long *backlash_ms) {
    pthread_mutex_lock(&mo_mu);
    const Actuator *a = mo_act;
    bool have = a && a->backlash_ms > 0;
    if (have && backlash_ms) *backlash_ms = a->backlash_ms;
    pthread_mutex_unlock(&mo_mu);
    return have;
}

// --- af3 step-based focus bridge --------------------------------------------------------------
// These mirror motion_engine_drive() for a MICROSTEP actuator: the engine worker drives the af3
// search through them. focus_step/focus_home BLOCK the caller while the backend's own thread does
// the SPI, so — unlike emit — they must NOT hold mo_mu across the call (it would stall the policy
// thread and every verb for the move's duration). The gate is checked and the actuator pointer
// grabbed under mo_mu, then released before the blocking call; the pointer is stable for the pass
// (teardown joins the engine before closing the actuator).

// Whether the open actuator drives focus by microsteps (→ the engine runs af3, not af2), and if
// so its travel and backlash in microsteps.
bool motion_can_focus(void) {
    motion_ready();
    pthread_mutex_lock(&mo_mu);
    bool can = mo_open && mo_act && mo_act->has(PTZ_NEAR) && mo_act->has(PTZ_FAR);
    pthread_mutex_unlock(&mo_mu);
    return can;
}

bool motion_focus_stepper(long *steps, long *backlash_steps) {
    pthread_mutex_lock(&mo_mu);
    const Actuator *a = mo_act;
    bool cap = mo_open && a && a->focus_steps > 0 && a->focus_step;
    if (cap) {
        if (steps) *steps = a->focus_steps;
        if (backlash_steps) *backlash_steps = a->focus_backlash_steps;
    }
    pthread_mutex_unlock(&mo_mu);
    return cap;
}

// Move exactly n focus microsteps in dir; returns microsteps advanced, or -1 if the wire is not
// ours (a human is driving, or it is not open) — the caller then abandons the pass, as with drive.
int motion_focus_step(int dir, int n) {
    pthread_mutex_lock(&mo_mu);
    const Actuator *a = mo_act;
    bool ok = mo_open && a && a->focus_step && mo_verb == PTZ_STOP;
    pthread_mutex_unlock(&mo_mu);
    if (!ok) return -1;
    return a->focus_step(dir, n);
}

// Take an absolute focus reference (ram to the near stop). false if unavailable or refused.
bool motion_focus_home(void) {
    pthread_mutex_lock(&mo_mu);
    const Actuator *a = mo_act;
    bool ok = mo_open && a && a->focus_home && mo_verb == PTZ_STOP;
    pthread_mutex_unlock(&mo_mu);
    return ok && a->focus_home();
}

// Dead-reckoned focus position in microsteps, or -1 if unknown (not homed / no such actuator).
int motion_focus_pos(void) {
    pthread_mutex_lock(&mo_mu);
    const Actuator *a = mo_act;
    int p = (mo_open && a && a->focus_pos) ? a->focus_pos() : -1;
    pthread_mutex_unlock(&mo_mu);
    return p;
}

bool motion_move(enum PtzVerb v, int ms) { return motion_move_at(v, ms, 0); }

// A percentage of the top rate as the actuator's 1..63, rounded up so that 1%
// is still a move; 0 (none asked for) stays 0, the configured rate.
static int speed_units(int pct) {
    if (pct <= 0) {
        return 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    return (pct * 63 + 99) / 100;
}

// A step move ends by itself; its deadline is only the watchdog's safety net.
// It allows half as long again as the actuator says the steps take (step_ms),
// or 100 ms a step where it does not say, and a second more. A move cut short
// here would leave the head off its target.
#define MOTION_STEP_MS_PER_STEP 100
#define MOTION_STEP_SLACK_MS 1000

// A verb still running on the other motor -- a zoom when a pan arrives, or the
// other way round -- is stopped before the new one is sent. Only ever true with
// a head. Its stop is reported here, if the verbs were reporting it, rather than
// by the next report_motion_locked(), which would name the new verb's axis.
//
// False when that stop did not reach the wire. The new move is then refused, so
// the running one stays the watchdog's -- its deadline is what retries the stop.
// Replacing it would leave a motor turning that nothing tracks any more.
static bool stop_other_locked(const Actuator *a) {
    if (mo_verb == PTZ_STOP) {
        return true;
    }
    const Actuator *o = owner(mo_verb);
    if (!o || o == a) {
        return true;
    }
    if (!o->emit(PTZ_STOP, 0)) {
        return false;
    }
    if (mo_reported_moving) {
        int axis = verb_axis(mo_verb);
        mo_reported_moving = false;
        if (axis < 0)
            mo_reported_focus = false;   // -1 ends every axis at the core
        ptz_motion_report(0, axis, -1);
    }
    return true;
}

static bool move(enum PtzVerb v, int ms, int pct, int steps);

bool motion_move_at(enum PtzVerb v, int ms, int pct) { return move(v, ms, pct, 0); }

bool motion_step(enum PtzVerb v, int steps, int pct) {
    if (steps <= 0 || !ptz_verb_is_pantilt(v)) {
        return false;
    }
    return move(v, steps * MOTION_STEP_MS_PER_STEP + MOTION_STEP_SLACK_MS, pct, steps);
}

// One move, timed (`steps` 0: runs until `ms` elapses without a repeat) or
// counted (`steps` > 0: the actuator moves that far and stops; `ms` is the
// safety deadline).
static bool move(enum PtzVerb v, int ms, int pct, int steps) {
    if (v == PTZ_STOP) {
        return motion_halt();
    }
    pthread_mutex_lock(&mo_mu);
    v = ptz_verb_mirror(v, mo_pan_invert, mo_tilt_invert);
    pthread_mutex_unlock(&mo_mu);
    if (steps > 0) {
        // the deadline computed for the count stands as given
    } else if (ms < MOTION_MIN_MS) {
        ms = motion_default_ms();
    } else if (ms > MOTION_MAX_MS) {
        ms = MOTION_MAX_MS;
    }
    motion_ready();   // an open that failed at load is not a verdict for the run
    // Can this camera send it at all? Ask first. A verb the actuator does not
    // carry -- day and night on the XiongMai wire, which only ever reports
    // them; or anything but zoom/focus on the MS41908M -- used to cancel a
    // running autofocus pass and clear the focus bookkeeping on its way to being
    // refused, which is a rejected request with side effects.
    pthread_mutex_lock(&mo_mu);
    const Actuator *a = mo_open ? owner(v) : NULL;
    bool can = a && a->has(v) && (steps == 0 || a->move_steps);
    pthread_mutex_unlock(&mo_mu);
    if (!can) {
        return false;
    }

    // Outside the lock: the pass must be told to abandon its moves before we
    // take the wire, and these take the engine's own mutex.
    if (ptz_verb_is_focus(v)) {
        af_note_manual_focus();   // raises the cancel itself
    } else {
        af_preempt();
    }

    pthread_mutex_lock(&mo_mu);
    a = mo_open ? owner(v) : NULL;
    if (!a) {
        pthread_mutex_unlock(&mo_mu);   // closed under us between the two locks
        return false;
    }
    // The other motor's move ends before this one starts: one verb on the wire
    // at a time, as on a single Pelco wire, so nothing else here learns that
    // there are two. Reported first, so that its stop cannot land at the core
    // after the start of a move the lens reports itself.
    if (!stop_other_locked(a)) {
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    // Re-send even when this verb is already running: a repeating command is
    // what a Pelco decoder expects, and it covers a frame lost on the wire --
    // and a repeat is how a new speed reaches a move already under way.
    int speed = a->honours_speed ? speed_units(pct) : 0;
    if (steps > 0 && a->step_ms) {
        long per = a->step_ms(v, speed);
        if (per > 0) {
            ms = (int)(steps * per * 3 / 2 + MOTION_STEP_SLACK_MS);
        }
    }
    bool sent = steps > 0 ? a->move_steps && a->move_steps(v, steps, speed)
                          : a->emit(v, speed);
    if (!sent) {
        // Nothing reached the lens. Arming the state anyway would leave the
        // watchdog minding a move that never started, and -- the part an
        // operator sees -- the endpoint answering "moving <verb>" for a lens
        // that did not budge.
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    // A counted move while a count still runs on the other axis joins it --
    // the actuator runs both (act_gpiostep's arm), taking turns between the
    // axes -- so the safety net is what is left of the first plus all of the
    // second, never cutting either count short. A count on the same axis
    // replaces the one there, and its deadline is its own.
    int pan = 0, tilt = 0;
    ptz_verb_dirs(v, &pan, &tilt);
    int axes = (pan ? 1 : 0) | (tilt ? 2 : 0);
    long now = now_ms();
    long until = now + ms;
    bool joins = steps > 0 && mo_counted && ptz_verb_is_pantilt(mo_verb) &&
                 mo_deadline > now && (mo_axes & ~axes) != 0;
    if (joins) {
        until = mo_deadline + (ms - MOTION_STEP_SLACK_MS);
    }
    mo_axes = joins ? (mo_axes | axes) : (steps > 0 ? axes : 0);
    mo_preset_running = false;   // a manual move replaces a go-to (the frame does that)
    mo_verb = v;
    mo_deadline = until;
    mo_counted = steps > 0;
    report_motion_locked();
    mo_bounce = 0;   // a new verb, even the same zoom-out held on, ends any bounce in progress
    if (ptz_verb_is_zoom(v)) {
        // Marked as the zoom STARTS, not as it ends. A pan arriving before the
        // zoom's deadline overwrites mo_verb, and reading the flag off the
        // verb that happened to be last lost the follow-up focus entirely.
        mo_zoom_dirty = true;
        mo_zoom_moving = true;
    } else if (mo_zoom_moving) {
        // Any other verb replaces the zoom on the wire: the zoom stopped here, and the board's
        // settle and late window count from now, not from the end of whatever replaced it. The
        // follow-up pass still waits out the whole session at the pad (mo_rebook).
        mo_zoom_moving = false;
        mo_zoom_settle_until = now_ms() + (mo_act ? mo_act->zoom_settle_ms : 0);
        mo_zoom_late_until = now_ms() + (mo_act ? mo_act->zoom_late_ms : 0);
    }
    if (ptz_verb_is_focus(v)) {
        // The operator is setting focus by hand; the zoom that displaced it no
        // longer has a claim on the lens.
        mo_zoom_dirty = false;
        mo_rebook = false;
    }
    pthread_mutex_unlock(&mo_mu);
    return true;
}

// The head's stored positions, 1..ptz.presetMax (default 32: many bases reserve 33-99
// as menu and command calls), on whichever motor carries pan/tilt -- the head beside a
// lens, or the actuator itself. Set and clear store or forget the position the head is
// at and move nothing. A go-to is a move of the head's own making, of unknown length:
// it takes the wire like any pan (the other motor's move stops first, a running pass is
// abandoned), is reported moving on axis -1, and is held for ptz.presetMs (default 4 s)
// so that nothing else claims the wire while the head travels -- ending WITHOUT a stop.
#define MOTION_PRESET_MAX_DEFAULT 32
#define MOTION_PRESET_MS_DEFAULT 4000

int motion_preset_max(void) {
    int n = config_get_int("ptz", "presetMax");
    return n >= 0 && n <= 255 ? n : MOTION_PRESET_MAX_DEFAULT;
}

static int motion_preset_ms(void) {
    int ms = config_get_int("ptz", "presetMs");
    return ms >= 500 && ms <= 30000 ? ms : MOTION_PRESET_MS_DEFAULT;
}

bool motion_preset(enum PtzPresetOp op, int n) {
    if (n < 1 || n > motion_preset_max()) {
        return false;
    }
    motion_ready();
    pthread_mutex_lock(&mo_mu);
    const Actuator *a = mo_open ? owner(PTZ_LEFT) : NULL;
    bool can = a && a->preset && a->has_presets && a->has_presets();
    pthread_mutex_unlock(&mo_mu);
    if (!can) {
        return false;
    }
    if (op != PTZ_PRESET_GO) {
        pthread_mutex_lock(&mo_mu);
        a = mo_open ? owner(PTZ_LEFT) : NULL;
        bool ok = a && a->preset && a->preset(op, n);
        pthread_mutex_unlock(&mo_mu);
        return ok;
    }
    af_preempt();   // the picture is about to swing: a pass measuring it is void
    pthread_mutex_lock(&mo_mu);
    a = mo_open ? owner(PTZ_LEFT) : NULL;
    if (!a || !a->preset || !stop_other_locked(a) || !a->preset(PTZ_PRESET_GO, n)) {
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    long now = now_ms();
    mo_axes = 0;
    mo_counted = false;
    mo_bounce = 0;
    // A sentinel pan/tilt verb, so routing and the report treat the go-to as the head's
    // move: a diagonal is reported on axis -1, "both", which is what a go-to does.
    mo_verb = PTZ_UPLEFT;
    mo_preset_running = true;
    mo_deadline = now + motion_preset_ms();
    report_motion_locked();
    if (mo_zoom_moving) {   // a zoom it replaced has stopped here (stop_other_locked)
        mo_zoom_moving = false;
        mo_zoom_settle_until = now + (mo_act ? mo_act->zoom_settle_ms : 0);
        mo_zoom_late_until = now + (mo_act ? mo_act->zoom_late_ms : 0);
    }
    pthread_mutex_unlock(&mo_mu);
    return true;
}

bool motion_halt(void) {
    motion_ready();
    af_preempt();
    pthread_mutex_lock(&mo_mu);
    if (!mo_open) {
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    bool ok = true;
    mo_preset_running = false;   // an operator's stop stops a go-to as well
    if (mo_verb != PTZ_STOP) {
        // A stop during a zoom-out's end bounce stops now (the after-zoom pass sees to focus);
        // a stop of the zoom-out itself starts the bounce.
        end_move_locked(mo_bounce == 0);
    } else {
        ok = emit_locked(PTZ_STOP);   // a stop that missed the wire is not a stop
    }
    pthread_mutex_unlock(&mo_mu);
    return ok;
}

bool motion_wake(void) {
    if (!motion_ready()) {
        return false;
    }
    pthread_mutex_lock(&mo_mu);
    const Actuator *a = mo_open ? mo_act : NULL;
    pthread_mutex_unlock(&mo_mu);
    // The full vendor wake sequence, on the caller's thread (it takes a few
    // seconds and is why this verb is not on the pad). The backend serialises its
    // own writes; a stepper has no sequence and returns true.
    return a ? a->wake() : false;
}

bool motion_engine_drive(int dir) {
    pthread_mutex_lock(&mo_mu);
    bool drove = false;
    if (mo_open && mo_verb == PTZ_STOP) {
        drove = emit_lens_locked(dir < 0 ? PTZ_NEAR : dir > 0 ? PTZ_FAR : PTZ_STOP);
        // A focus pass blurs and sharpens the whole picture: the core holds
        // motion detection back while it drives (OpenIPC/majestic#347).
        mo_engine_driving = drove && dir != 0;
        report_focus_locked();
    }
    // else: a human is driving, and the answer is false. Letting the pass's
    // trailing stop through here is the interleaving that made the operator's
    // presses vanish -- but swallowing it and saying nothing is its own bug:
    // af2 would go on sampling and dead-reckoning a move that never happened.
    // The caller abandons the pass instead.
    pthread_mutex_unlock(&mo_mu);
    return drove;
}

bool motion_manual_active(void) {
    pthread_mutex_lock(&mo_mu);
    bool a = mo_verb != PTZ_STOP;
    pthread_mutex_unlock(&mo_mu);
    return a;
}

long motion_zoom_settle_ms(void) {
    pthread_mutex_lock(&mo_mu);
    long r;
    if (mo_zoom_moving) {
        r = mo_act ? mo_act->zoom_settle_ms : 0;   // still zooming: the whole settle is ahead
    } else {
        r = mo_zoom_settle_until - now_ms();
    }
    pthread_mutex_unlock(&mo_mu);
    return r > 0 ? r : 0;
}

long motion_zoom_late_ms(void) {
    pthread_mutex_lock(&mo_mu);
    long r = mo_zoom_moving ? (mo_act ? mo_act->zoom_late_ms : 0) : mo_zoom_late_until - now_ms();
    pthread_mutex_unlock(&mo_mu);
    return r > 0 ? r : 0;
}

long motion_idle_ms(void) {
    pthread_mutex_lock(&mo_mu);
    long r = (mo_verb != PTZ_STOP || !mo_idle_since) ? 0 : now_ms() - mo_idle_since;
    pthread_mutex_unlock(&mo_mu);
    return r;
}

const char *motion_describe(char *buf, size_t n) {
    // The capability line is where `state=closed` is reported, so it is also the
    // one place an operator can see the port is shut -- and the WebUI asks it on
    // every page load. Retry here too, so the line says what is true now rather
    // than what was true at load, and so simply opening the pad heals a camera
    // whose port came back.
    motion_ready();
    pthread_mutex_lock(&mo_mu);
    const Actuator *a = mo_act;
    bool open_ = mo_open;
    // With a head, pan/tilt is the head's: the speeds, the counted steps and the
    // frame below describe pan/tilt, so they are its, and the verbs are each
    // motor's own. Without one, `pt` is the lens and every line is as it was.
    const Actuator *head = mo_head;
    bool head_open = mo_head_open;
    const Actuator *pt = head ? head : a;
    // Only the UART family has a port and a line speed to report; a stepper
    // (MS41908M, gpiostep) is named by its actuator alone. With a head it may be
    // either motor that is on the tty.
    const Actuator *u = a && !strcmp(a->name, "pelco") ? a
                        : head && !strcmp(head->name, "pelco") ? head : NULL;
    bool u_open = u == head ? head_open : open_;
    bool uart = u != NULL;
    bool has[PTZ_VERB_COUNT];
    for (int v = 0; v < PTZ_VERB_COUNT; v++) {
        // Closed, the lens names what it would carry, as before; so does the head.
        const Actuator *o = open_ ? owner((enum PtzVerb)v)
                            : head && ptz_verb_is_pantilt((enum PtzVerb)v) ? head : a;
        has[v] = o && o->has((enum PtzVerb)v);
    }
    pthread_mutex_unlock(&mo_mu);

    char verbs[128];
    size_t used = 0;
    verbs[0] = 0;
    for (int v = 0; v < PTZ_VERB_COUNT; v++) {
        if (!has[v]) {
            continue;
        }
        const char *nm = ptz_verb_name((enum PtzVerb)v);
        int w = snprintf(verbs + used, sizeof verbs - used, "%s%s",
                         used ? "," : "", nm);
        if (w < 0 || (size_t)w >= sizeof verbs - used) {
            break;
        }
        used += (size_t)w;
    }
    const char *pname = a && a->proto_name ? a->proto_name() : (a ? a->name : "");
    if (uart) {
        // The line the lens is being spoken to at, while there is one. The
        // config's port and speed are read only at open: after a save they
        // say what the next open will use, and reporting them for the port
        // still open at the old rate told an operator correcting a baud that
        // it had taken when it had not. Closed, they are what the next open
        // will use, which is the truth to tell then.
        char lport[128];
        int baud = 0, addr = PTZ_ADDR_DEFAULT;
        const char *port;
        if (u_open && u->line && u->line(lport, sizeof lport, &baud, &addr)) {
            port = lport;
        } else {
            port = config_get_string("ptz", "port");
            port = port && *port ? port : "/dev/ttyAMA0";
            baud = config_get_int("ptz", "baud");
            addr = ptz_addr(config_get_int("ptz", "address"), NULL);
        }
        snprintf(buf, n, "actuator=%s port=%.128s speed=%d addr=%d pulse=%d state=%s verbs=%s%s",
                 pname, port, baud, addr, motion_default_ms(),
                 open_ ? "ready" : "closed", verbs,
                 pt->honours_speed ? " speeds=1-100" : "");
        if (pt->move_steps) {
            size_t used_ = strlen(buf);
            snprintf(buf + used_, n > used_ ? n - used_ : 0, " steps=1");
        }
    } else {
        snprintf(buf, n, "actuator=%s pulse=%d state=%s verbs=%s%s%s",
                 pname, motion_default_ms(), open_ ? "ready" : "closed", verbs,
                 pt && pt->honours_speed ? " speeds=1-100" : "",
                 pt && pt->move_steps ? " steps=1" : "");
    }
    int fw = 0, fh = 0;
    if (pt && pt->move_steps && pt->frame_steps && pt->frame_steps(&fw, &fh)) {
        size_t used_ = strlen(buf);
        snprintf(buf + used_, n > used_ ? n - used_ : 0, " frame=%dx%d", fw, fh);
    }
    // Stored positions, where the pan/tilt motor has them (motion_preset).
    int pmax = motion_preset_max();
    if (pt && pt->has_presets && pt->has_presets() && pmax > 0) {
        size_t used_ = strlen(buf);
        snprintf(buf + used_, n > used_ ? n - used_ : 0, " presets=1-%d", pmax);
    }
    if (head) {
        // Last, so every token a core already reads stays where it was.
        const char *hname = head->proto_name ? head->proto_name() : head->name;
        size_t used_ = strlen(buf);
        snprintf(buf + used_, n > used_ ? n - used_ : 0, " head=%s headstate=%s",
                 hname && *hname ? hname : head->name, head_open ? "ready" : "closed");
    }
    return buf;
}
