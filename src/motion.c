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
static enum PtzVerb mo_verb = PTZ_STOP;   // the manual move now running
static bool mo_reported_moving = false;   // what sdk_ptz_motion was last told
static long mo_deadline;                  // when to stop it
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
    int ms = config_get_int("isp.autofocus", "pulse");
    if (ms < MOTION_MIN_MS || ms > MOTION_MAX_MS) {
        return MOTION_DEFAULT_MS;   // unset, or a value the schema should have refused
    }
    return ms;
}

// One verb onto the wire, through the actuator. mo_mu is held; emit() is
// non-blocking and serialises on the backend's own lock (never mo_mu).
static bool emit_locked(enum PtzVerb v) {
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
    default: return -1;
    }
}

// Report a moving/still transition of the verb now on the wire, for a backend
// that cannot see its motor itself (the UART lenses: the verb is all we know).
// Called under mo_mu after every change of mo_verb; sdk_ptz_motion never
// blocks or calls back, so that is safe.
static void report_motion_locked(void) {
    if (!mo_act || mo_act->reports_motion) {
        return;
    }
    int axis = verb_axis(mo_verb);
    bool moving = axis >= 0;
    if (moving == mo_reported_moving) {
        return;
    }
    mo_reported_moving = moving;
    long left = moving ? mo_deadline - now_ms() : -1;
    ptz_motion_report(moving, axis, left > 0 ? left : -1);
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
    if (!emit_locked(PTZ_STOP)) {
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
    const char *name = config_get_string("isp.autofocus", "actuator");
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
        if (mo_verb != PTZ_STOP) {
            emit_locked(PTZ_STOP);   // never leave a motor running behind us
            mo_verb = PTZ_STOP;
            report_motion_locked();
        }
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

bool motion_move_at(enum PtzVerb v, int ms, int pct) {
    if (v == PTZ_STOP) {
        return motion_halt();
    }
    if (ms < MOTION_MIN_MS) {
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
    bool can = mo_open && mo_act->has(v);
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
    if (!mo_open) {
        pthread_mutex_unlock(&mo_mu);   // closed under us between the two locks
        return false;
    }
    // Re-send even when this verb is already running: a repeating command is
    // what a Pelco decoder expects, and it covers a frame lost on the wire --
    // and a repeat is how a new speed reaches a move already under way.
    int speed = mo_act->honours_speed ? speed_units(pct) : 0;
    if (!(mo_open && mo_act && mo_act->emit(v, speed))) {
        // Nothing reached the lens. Arming the state anyway would leave the
        // watchdog minding a move that never started, and -- the part an
        // operator sees -- the endpoint answering "moving <verb>" for a lens
        // that did not budge.
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    mo_verb = v;
    mo_deadline = now_ms() + ms;
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

bool motion_halt(void) {
    motion_ready();
    af_preempt();
    pthread_mutex_lock(&mo_mu);
    if (!mo_open) {
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    bool ok = true;
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
        drove = emit_locked(dir < 0 ? PTZ_NEAR : dir > 0 ? PTZ_FAR : PTZ_STOP);
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
    // Only the UART family has a port and a line speed to report; a stepper
    // (MS41908M, gpiostep) is named by its actuator alone.
    bool uart = a && !strcmp(a->name, "pelco");
    pthread_mutex_unlock(&mo_mu);

    char verbs[128];
    size_t used = 0;
    verbs[0] = 0;
    for (int v = 0; v < PTZ_VERB_COUNT; v++) {
        if (!a || !a->has((enum PtzVerb)v)) {
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
        const char *port = config_get_string("isp.autofocus", "port");
        snprintf(buf, n, "actuator=%s port=%s speed=%d pulse=%d state=%s verbs=%s%s",
                 pname, port && *port ? port : "/dev/ttyAMA0",
                 config_get_int("isp.autofocus", "speed"), motion_default_ms(),
                 open_ ? "ready" : "closed", verbs,
                 a->honours_speed ? " speeds=1-100" : "");
    } else {
        snprintf(buf, n, "actuator=%s pulse=%d state=%s verbs=%s%s",
                 pname, motion_default_ms(), open_ ? "ready" : "closed", verbs,
                 a && a->honours_speed ? " speeds=1-100" : "");
    }
    return buf;
}
