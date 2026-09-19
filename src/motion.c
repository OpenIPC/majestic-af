#include "motion.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include <majestic/af_plugin_abi.h>   // config_get_* — the core's seams
#include <majestic/log.h>

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
// elapses only once the operator has let go.
#define MOTION_BOOK_QUIET_MS 700

// Watchdog tick. Deadlines land within a tick of where they were asked for,
// which is far finer than the motor's own response.
#define MOTION_TICK_MS 20

// How often a REPEATED open failure may be logged. The first one always is.
#define MOTION_OPEN_LOG_MS 10000

static pthread_mutex_t mo_mu = PTHREAD_MUTEX_INITIALIZER;
static int mo_fd = -1;
static const PtzProto *mo_proto;
static enum PtzVerb mo_verb = PTZ_STOP;   // the manual move now running
static long mo_deadline;                  // when to stop it
static long mo_idle_since;                // when manual motion last ended
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

int motion_default_ms(void) {
    int ms = config_get_int("isp.autofocus", "pulse");
    if (ms < MOTION_MIN_MS || ms > MOTION_MAX_MS) {
        return MOTION_DEFAULT_MS;   // unset, or a value the schema should have refused
    }
    return ms;
}

// Everything below writes through here, with mo_mu held.
//
// The descriptor is non-blocking (see motion_start), so a write can come back
// short or with EAGAIN even for eight bytes -- rare on an idle 115200 line,
// but a half-written Pelco frame is a frame the lens will not act on. Finish
// it, bounded, rather than log it and move on.
static bool write_locked(const unsigned char *buf, size_t len) {
    if (mo_fd < 0 || !len) {
        return false;
    }
    size_t done = 0;
    for (int tries = 0; done < len && tries < 20; tries++) {
        ssize_t n = write(mo_fd, buf + done, len - done);
        if (n > 0) {
            done += (size_t)n;
        } else if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
            usleep(1000);
        } else {
            break;
        }
    }
    if (done != len) {
        log_e("ptz: short UART write (%u of %u): %s", (unsigned)done,
              (unsigned)len, strerror(errno));
        return false;
    }
    return true;
}

static bool frame_locked(enum PtzVerb v, int speed) {
    unsigned char f[PTZ_FRAME_MAX];
    int n = ptz_frame(mo_proto, v, speed, f);
    return write_locked(f, (size_t)n);
}

// End the manual move: stop the motor, remember when the wire went quiet, and
// ask for the follow-up focus if a zoom is still waiting for one. Every move
// that ends re-arms it, so the pass waits out a whole session at the pad
// rather than the last verb of it. A manual focus move never leaves one armed:
// the operator set the focus by hand and a pass would simply undo it, which is
// the defect this replaced.
static void end_move_locked(void) {
    if (!frame_locked(PTZ_STOP, 0)) {
        // The stop did not reach the wire. Saying the move ended would retire
        // the only thing that will try again, while the motor keeps driving.
        // Leave it running and let the next tick have another go.
        mo_deadline = now_ms() + MOTION_TICK_MS;
        return;
    }
    mo_verb = PTZ_STOP;
    mo_idle_since = now_ms();
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
        if (mo_verb != PTZ_STOP && t >= mo_deadline) {
            end_move_locked();
        }
        rebook = mo_rebook;
        mo_rebook = false;
        pthread_mutex_unlock(&mo_mu);

        // Outside mo_mu, always: the engine's calls take af_mu and may spawn a
        // worker that calls straight back into motion_engine_drive(). Nothing
        // here holds both locks, in either order.
        if (rebook) {
            af_book_after_zoom(t + MOTION_BOOK_QUIET_MS, gen);
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
    if (mo_fd >= 0) {
        pthread_mutex_unlock(&mo_mu);
        return true;
    }
    const char *port = config_get_string("isp.autofocus", "port");
    const char *name = config_get_string("isp.autofocus", "actuator");
    int speed = config_get_int("isp.autofocus", "speed");
    mo_proto = ptz_proto(name);
    if (name && *name && strcmp(name, ptz_proto_name(mo_proto))) {
        log_w("ptz: unknown actuator '%s', using %s", name,
              ptz_proto_name(mo_proto));
    }

    // Read AND write on one descriptor: the magnification reader shares it
    // through motion_fd(). Two opens meant two independent tcsetattr calls on
    // the same tty, the writer's without CLOCAL|CREAD.
    //
    // O_NONBLOCK is load-bearing twice over, and both were learned the hard
    // way. Opening a tty without it blocks until carrier is asserted, which on
    // a UART with no modem lines need never happen -- and the port is opened
    // here before CLOCAL is set, so there is nothing yet to say the line has
    // no carrier to wait for. And the reader drains with
    // `while ((n = read(fd, ...)) > 0)`, which terminates on EAGAIN: on a
    // blocking descriptor that loop parks in read() for ever, never looks at
    // af_reader_stop again, and the join in af_engine_stop() hangs the whole
    // process on the way down -- a camera whose event loop is gone but whose
    // pid is still there.
    mo_fd = open(port && *port ? port : "/dev/ttyAMA0",
                 O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (mo_fd < 0) {
        // The first failure is always logged; after that at most one line per
        // MOTION_OPEN_LOG_MS. Every verb retries the open now (motion_ready), and
        // a held button re-sends its verb every ~250 ms, so an unopenable port
        // would otherwise write four lines a second into a syslog ring that is
        // the only record of anything else going wrong.
        long t = now_ms();
        if (!mo_open_failed || t - mo_last_fail_log >= MOTION_OPEN_LOG_MS) {
            log_e("ptz: cannot open %s: %s", port ? port : "/dev/ttyAMA0",
                  strerror(errno));
            mo_last_fail_log = t;
        }
        mo_open_failed = true;
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    if (mo_open_failed) {
        // The condition cleared. Say so: the failure above was logged, and an
        // operator who saw it has no other way to learn that the lens came back.
        log_i("ptz: %s opened on retry", port && *port ? port : "/dev/ttyAMA0");
        mo_open_failed = false;
    }
    struct termios tio;
    memset(&tio, 0, sizeof(tio));
    if (tcgetattr(mo_fd, &tio) == 0) {
        cfmakeraw(&tio);
        cfsetspeed(&tio, baud_const(speed));
        tio.c_cflag |= (CLOCAL | CREAD);
        tcsetattr(mo_fd, TCSANOW, &tio);
    }
    tcflush(mo_fd, TCIFLUSH);

    mo_verb = PTZ_STOP;
    mo_idle_since = now_ms();
    mo_zoom_dirty = false;
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
        // descriptor open would also make every later retry take the
        // already-open fast path and inherit the same state.
        log_e("ptz: cannot start the motion watchdog: %s", strerror(rc));
        pthread_mutex_lock(&mo_mu);
        mo_run = 0;
        if (mo_fd >= 0) {
            close(mo_fd);
            mo_fd = -1;
        }
        mo_proto = NULL;
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
    if (motion_fd() >= 0) {
        // Idempotent, and asked EVERY time rather than only on the transition:
        // if the reader's pthread_create failed once, the descriptor stays open
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
    size_t wlen = 0;
    pthread_mutex_lock(&mo_mu);
    const unsigned char *w = ptz_wake_blob(mo_proto, &wlen);
    // No blob for this protocol is nothing to send, not a failure; only a short
    // or failed write is, and the caller can tell the two apart.
    bool ok = mo_fd >= 0 && (!w || write_locked(w, wlen));
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
    if (mo_fd >= 0) {
        if (mo_verb != PTZ_STOP) {
            frame_locked(PTZ_STOP, 0);   // never leave a motor running behind us
            mo_verb = PTZ_STOP;
        }
        close(mo_fd);
        mo_fd = -1;
    }
    mo_zoom_dirty = false;
    mo_rebook = false;
    pthread_mutex_unlock(&mo_mu);
}

int motion_fd(void) {
    pthread_mutex_lock(&mo_mu);
    int fd = mo_fd;
    pthread_mutex_unlock(&mo_mu);
    return fd;
}

bool motion_move(enum PtzVerb v, int ms) {
    if (v == PTZ_STOP) {
        return motion_halt();
    }
    if (ms < MOTION_MIN_MS) {
        ms = motion_default_ms();
    } else if (ms > MOTION_MAX_MS) {
        ms = MOTION_MAX_MS;
    }
    motion_ready();   // an open that failed at load is not a verdict for the run
    // Can this camera send it at all? Ask first. A verb the protocol does not
    // carry -- day and night on the XiongMai wire, which only ever reports
    // them -- used to cancel a running autofocus pass and clear the focus
    // bookkeeping on its way to being refused, which is a rejected request
    // with side effects.
    pthread_mutex_lock(&mo_mu);
    bool can = mo_fd >= 0 && ptz_proto_has(mo_proto, v);
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
    if (mo_fd < 0) {
        pthread_mutex_unlock(&mo_mu);   // closed under us between the two locks
        return false;
    }
    // Re-send even when this verb is already running: a repeating command is
    // what a Pelco decoder expects, and it covers a frame lost on the wire.
    if (!frame_locked(v, 0)) {
        // Nothing reached the lens. Arming the state anyway would leave the
        // watchdog minding a move that never started, and -- the part an
        // operator sees -- the endpoint answering "moving <verb>" for a lens
        // that did not budge.
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    mo_verb = v;
    mo_deadline = now_ms() + ms;
    if (ptz_verb_is_zoom(v)) {
        // Marked as the zoom STARTS, not as it ends. A pan arriving before the
        // zoom's deadline overwrites mo_verb, and reading the flag off the
        // verb that happened to be last lost the follow-up focus entirely.
        mo_zoom_dirty = true;
    } else if (ptz_verb_is_focus(v)) {
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
    if (mo_fd < 0) {
        pthread_mutex_unlock(&mo_mu);
        return false;
    }
    bool ok = true;
    if (mo_verb != PTZ_STOP) {
        end_move_locked();
    } else {
        ok = frame_locked(PTZ_STOP, 0);   // a stop that missed the wire is not a stop
    }
    pthread_mutex_unlock(&mo_mu);
    return ok;
}

bool motion_wake(void) {
    unsigned char f[PTZ_FRAME_MAX];

    if (!motion_ready() || !motion_wake_blob()) {
        return false;
    }
    pthread_mutex_lock(&mo_mu);
    bool has_presets = ptz_preset_frame(mo_proto, 0x53, f) > 0;
    pthread_mutex_unlock(&mo_mu);
    if (!has_presets) {
        return true;   // the XiongMai variant has only the blob
    }

    // The Pelco-D lens tool's wakeup: two vendor presets, then an ICR exercise.
    // Slow on purpose — the lens needs the time — and run on the caller's
    // thread, which is why this verb is not on the pad.
    static const int steps_ms[] = {500, 500};
    const int presets[] = {0x53, 0x52};
    for (int i = 0; i < 2; i++) {
        msleep(steps_ms[i]);
        pthread_mutex_lock(&mo_mu);
        int n = ptz_preset_frame(mo_proto, presets[i], f);
        write_locked(f, (size_t)n);
        pthread_mutex_unlock(&mo_mu);
    }
    msleep(3000);
    pthread_mutex_lock(&mo_mu);
    frame_locked(PTZ_NIGHT, 0);
    pthread_mutex_unlock(&mo_mu);
    msleep(3000);
    pthread_mutex_lock(&mo_mu);
    frame_locked(PTZ_DAY, 0);
    pthread_mutex_unlock(&mo_mu);
    return true;
}

bool motion_engine_drive(int dir) {
    pthread_mutex_lock(&mo_mu);
    bool drove = false;
    if (mo_fd >= 0 && mo_verb == PTZ_STOP) {
        drove = frame_locked(dir < 0 ? PTZ_NEAR : dir > 0 ? PTZ_FAR : PTZ_STOP, 0);
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
    const PtzProto *p = mo_proto;
    bool open_ = mo_fd >= 0;
    pthread_mutex_unlock(&mo_mu);

    const char *port = config_get_string("isp.autofocus", "port");
    char verbs[128];
    size_t used = 0;
    verbs[0] = 0;
    for (int v = 0; v < PTZ_VERB_COUNT; v++) {
        if (!ptz_proto_has(p, (enum PtzVerb)v)) {
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
    snprintf(buf, n, "actuator=%s port=%s speed=%d pulse=%d state=%s verbs=%s",
             ptz_proto_name(p), port && *port ? port : "/dev/ttyAMA0",
             config_get_int("isp.autofocus", "speed"), motion_default_ms(),
             open_ ? "ready" : "closed", verbs);
    return buf;
}
