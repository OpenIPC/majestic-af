// act_gpiostep — pan/tilt heads whose two 4-wire steppers hang straight off GPIO.
//
// The Goke GK7205V510 PTZ cameras (the Zenointel SD-2N-4G, an NC-IPTC2XXX_DL_4G
// board) have no motor MCU and no lens driver for the head: the SoC energises the
// coils itself. OpenIPC/firmware's gpiostep.ko does that from kernel context and
// exposes one ioctl, "move the pan coil by N steps and the tilt coil by M"; this
// backend turns motion.c's continuous verbs into those moves.
//
// The shape is act_ms41908's. emit() only records a direction and never blocks; a
// stepping thread issues short moves (gs_run_chunk(), at most ~100 ms) while a
// direction is held, so a stop lands within one chunk. The head carries left,
// right, up and down and nothing else: no zoom, no focus, no ICR. That is why
// gs_has() is the whole capability list, and why /autofocus answers "unavailable"
// on such a camera (engine.c asks motion_can_focus()).
//
// What the head looks like is the board's to say, in GS_CONF_PATH: the travel of
// each axis, which raw direction is left and which is up, and the step rate. With
// a travel known, the position is dead-reckoned from a homing seek into both
// gearbox stops, done once per boot, and every move is clamped short of the stops.
// A majestic restart within the same boot keeps the position (GS_POS_PATH lives on
// tmpfs) instead of sweeping the head again.

#include "act_gpiostep.h"
#include "actuator.h"
#include "motion.h"

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <majestic/log.h>

// The module's ABI (OpenIPC/firmware general/package/gpiostep-openipc/src/gpiostep.h),
// restated: that header is the kernel side's, and this plugin builds without a
// kernel tree.
struct gpiostep_move {
    int pan;        // pan steps (sign = direction)
    int tilt;       // tilt steps (sign = direction)
    int delay_us;   // per-microstep delay
};
#define GPIOSTEP_MOVE _IOW('g', 1, struct gpiostep_move)
#define GS_DEV "/dev/motorDev"

// Where the dead-reckoned position survives a majestic restart. tmpfs, so it dies
// with the boot -- exactly when the position stops being true -- and the next boot
// homes again.
#define GS_POS_PATH "/tmp/gpiostep.pos"

static pthread_mutex_t s_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_cv = PTHREAD_COND_INITIALIZER;
static pthread_t s_thread;
static bool s_thread_valid = false;
static int s_run = 0;
static int s_fd = -1;
static GsConfig s_cfg;
static int s_axis = GS_PAN;   // the axis armed by the last verb
static int s_dir = 0;         // its raw direction, 0 = stopped
static int s_speed = 0;       // its speed, 1..63, or 0 for the configured rate
static int s_pos[GS_AXES];
// Per axis, all under s_mu. An axis with a travel is "homed" once its s_pos is
// a physical position; one whose seek did not finish is "lost", and is refused
// moves rather than driven without the limits the board asked for, until a
// majestic restart homes it again.
static bool s_homed[GS_AXES];
static bool s_lost[GS_AXES];
static bool s_homing = false;       // the seek is running: directional verbs are refused
static bool s_home_cancel = false;  // a stop arrived during the seek
static bool s_dirty = false;        // s_pos moved since it was last saved
static bool s_pos_saved = false;    // GS_POS_PATH describes s_pos now
static bool s_limit_said[GS_AXES][2];

static void load_config(void) {
    gs_config_defaults(&s_cfg);
    FILE *f = fopen(GS_CONF_PATH, "r");
    if (!f) {
        log_w("gpiostep: no %s, so no travel limits and no homing", GS_CONF_PATH);
        return;
    }
    char line[128];
    int n = 0;
    while (fgets(line, sizeof line, f)) {
        n++;
        line[strcspn(line, "\r\n")] = 0;
        if (!gs_config_line(&s_cfg, line)) {
            log_w("gpiostep: %s:%d: ignoring '%s'", GS_CONF_PATH, n, line);
        }
    }
    fclose(f);
}

static bool limited(int axis) { return s_cfg.travel[axis] > 0; }

static bool restore_pos(void) {
    FILE *f = fopen(GS_POS_PATH, "r");
    if (!f) {
        return false;
    }
    char line[64];
    bool read = fgets(line, sizeof line, f) != NULL;
    fclose(f);
    if (!read) {
        return false;
    }
    int pos[GS_AXES];
    bool known[GS_AXES];
    gs_parse_pos(&s_cfg, line, pos, known);
    for (int a = 0; a < GS_AXES; a++) {
        if (limited(a) && !known[a]) {
            return false;   // half a position is not one: home both
        }
    }
    for (int a = 0; a < GS_AXES; a++) {
        if (known[a]) {
            s_pos[a] = pos[a];
            s_homed[a] = true;
        }
    }
    s_pos_saved = true;
    return true;
}

static void save_pos(void) {
    FILE *f = fopen(GS_POS_PATH, "w");
    if (f) {
        fprintf(f, "%d %d\n", s_pos[GS_PAN], s_pos[GS_TILT]);
        fclose(f);
        s_pos_saved = true;
    }
}

// The saved position goes the moment the head starts to move, and comes back
// when it stops: a majestic killed mid-move then leaves no file, and the next
// one homes, rather than trusting a position the head has since left.
static void forget_saved_pos(void) {
    if (s_pos_saved) {
        unlink(GS_POS_PATH);
        s_pos_saved = false;
    }
}

// One move on the coils. Blocks for as long as the module steps; called only from
// the stepping thread, with s_mu released.
static bool coil_move(int axis, int steps, int delay_us) {
    struct gpiostep_move m = {
        .pan = axis == GS_PAN ? steps : 0,
        .tilt = axis == GS_TILT ? steps : 0,
        .delay_us = delay_us,
    };
    if (ioctl(s_fd, GPIOSTEP_MOVE, &m) < 0) {
        log_e("gpiostep: move: %s", strerror(errno));
        return false;
    }
    return true;
}


static bool seek_wanted(void) {
    pthread_mutex_lock(&s_mu);
    bool r = s_run != 0 && !s_home_cancel;
    pthread_mutex_unlock(&s_mu);
    return r;
}

// `steps` on one axis in gs_home_chunk() pieces, giving up if the plugin is being
// unloaded or an operator sent a stop. Returns false when it was interrupted.
static bool home_move(int axis, int steps) {
    int dir = steps < 0 ? -1 : 1;
    int delay = gs_home_delay(&s_cfg, axis);
    int chunk = gs_home_chunk(delay);
    for (int left = steps * dir; left > 0; left -= chunk) {
        if (!seek_wanted()) {
            return false;
        }
        int n = left < chunk ? left : chunk;
        if (!coil_move(axis, dir * n, delay)) {
            return false;
        }
    }
    return true;
}

// Drive each axis with a travel into its stop at 0, then to the middle of its
// travel. The overshoot past the travel makes sure the head reaches the stop
// wherever it was left; the steppers slip against it without harm, the way the
// vendor firmware's own self-check does at boot. An axis with no travel is left
// alone: there is nothing to home it against. Called without s_mu; marks each
// axis homed as it is centred, and every axis it did not finish lost.
static void home(void) {
    log_i("gpiostep: homing pan %d / tilt %d steps", s_cfg.travel[GS_PAN],
          s_cfg.travel[GS_TILT]);
    bool ok = true;
    for (int a = 0; a < GS_AXES && ok; a++) {
        int t = s_cfg.travel[a];
        if (limited(a)) {
            ok = home_move(a, -(t + t / 8 + 8));
        }
    }
    for (int a = 0; a < GS_AXES && ok; a++) {
        if (!limited(a)) {
            continue;
        }
        ok = home_move(a, s_cfg.travel[a] / 2);
        if (ok) {
            pthread_mutex_lock(&s_mu);
            s_pos[a] = s_cfg.travel[a] / 2;
            s_homed[a] = true;
            pthread_mutex_unlock(&s_mu);
        }
    }
    pthread_mutex_lock(&s_mu);
    for (int a = 0; a < GS_AXES; a++) {
        if (limited(a) && !s_homed[a]) {
            s_lost[a] = true;
            log_w("gpiostep: %s did not finish homing%s; it stays still until "
                  "majestic restarts and homes it again",
                  a == GS_PAN ? "pan" : "tilt", s_home_cancel ? " (stopped)" : "");
        }
    }
    if (ok) {
        save_pos();
        log_i("gpiostep: homed, centred at %d/%d", s_pos[GS_PAN], s_pos[GS_TILT]);
    }
    pthread_mutex_unlock(&s_mu);
}

// Whether the core was last told the coils are turning (sdk_ptz_motion). Only
// the step thread reads or writes it. Reported from here rather than from the
// verbs: this is the one place that knows when the coils really start and
// stop, whatever stopped them (a stop, the deadline, a soft limit, a failed
// ioctl), and a seek at boot moves the picture as much as a pan does.
static bool s_told_moving = false;
// The axis that moved last while s_told_moving: a later verb may have armed
// another one (s_axis) without any of its coils turning yet.
static int s_told_axis = GS_PAN;

static void *step_thread(void *arg) {
    (void)arg;
    pthread_mutex_lock(&s_mu);
    if (limited(GS_PAN) || limited(GS_TILT)) {
        if (restore_pos()) {
            log_i("gpiostep: position %d/%d kept from this boot", s_pos[GS_PAN],
                  s_pos[GS_TILT]);
        } else if (s_cfg.home) {
            unlink(GS_POS_PATH);   // whatever it said, the seek replaces it
            s_homing = true;
            s_home_cancel = false;
            pthread_mutex_unlock(&s_mu);
            ptz_motion_report(1, -1, -1);
            home();
            ptz_motion_report(0, -1, -1);
            pthread_mutex_lock(&s_mu);
            s_homing = false;
            s_dir = 0;   // gs_emit refused directions during the seek; a stop is spent
        }
    }
    while (s_run) {
        if (s_dir == 0) {
            if (s_told_moving) {
                // The coils have stopped. Said outside s_mu, then the loop
                // looks again: a new verb may have arrived meanwhile.
                s_told_moving = false;
                int ax = s_told_axis;
                pthread_mutex_unlock(&s_mu);
                ptz_motion_report(0, ax, -1);
                pthread_mutex_lock(&s_mu);
                continue;
            }
            if (s_dirty) {
                save_pos();
                s_dirty = false;
            }
            pthread_cond_wait(&s_cv, &s_mu);
            continue;
        }
        int axis = s_axis, dir = s_dir;
        int delay = gs_speed_delay(&s_cfg, axis, s_speed);
        int travel = s_homed[axis] ? s_cfg.travel[axis] : 0;
        int n = gs_clamp_step(s_pos[axis], dir, gs_run_chunk(delay), travel);
        if (n == 0) {
            bool *said = &s_limit_said[axis][dir > 0];
            if (!*said) {
                log_i("gpiostep: %s at its limit", axis == GS_PAN ? "pan" : "tilt");
                *said = true;
            }
            s_dir = 0;
            continue;
        }
        s_limit_said[axis][dir < 0] = false;
        forget_saved_pos();
        bool begin = !s_told_moving;
        s_told_moving = true;
        s_told_axis = axis;
        pthread_mutex_unlock(&s_mu);
        if (begin) {
            ptz_motion_report(1, axis, -1);   // just before the first coils turn
        }
        bool ok = coil_move(axis, dir * n, delay);
        pthread_mutex_lock(&s_mu);
        if (!ok) {
            s_dir = 0;
            continue;
        }
        if (s_homed[axis]) {
            s_pos[axis] += dir * n;
            s_dirty = true;
        }
    }
    if (s_dirty) {
        save_pos();
    }
    bool was_moving = s_told_moving;
    s_told_moving = false;
    int ax = s_told_axis;
    pthread_mutex_unlock(&s_mu);
    if (was_moving) {
        ptz_motion_report(0, ax, -1);   // shut down mid-move: the coils stop with the thread
    }
    return NULL;
}

static bool gs_open(void) {
    pthread_mutex_lock(&s_mu);
    if (s_fd >= 0) {
        pthread_mutex_unlock(&s_mu);
        return true;
    }
    int fd = open(GS_DEV, O_RDWR);
    if (fd < 0) {
        pthread_mutex_unlock(&s_mu);
        return false;   // motion.c owns the rate-limited retry logging
    }
    s_fd = fd;
    load_config();
    s_dir = 0;
    s_run = 1;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 0x10000);
    int rc = pthread_create(&s_thread, &attr, step_thread, NULL);
    pthread_attr_destroy(&attr);
    if (rc) {
        log_e("gpiostep: cannot start the stepping thread: %s", strerror(rc));
        s_run = 0;
        close(s_fd);
        s_fd = -1;
        pthread_mutex_unlock(&s_mu);
        return false;
    }
    s_thread_valid = true;
    pthread_mutex_unlock(&s_mu);
    return true;
}

static void gs_close(void) {
    pthread_mutex_lock(&s_mu);
    s_dir = 0;
    s_run = 0;
    pthread_cond_signal(&s_cv);
    bool join = s_thread_valid;
    pthread_t t = s_thread;
    s_thread_valid = false;
    pthread_mutex_unlock(&s_mu);
    if (join) {
        pthread_join(t, NULL);   // the thread finishes its last chunk, then saves
    }
    pthread_mutex_lock(&s_mu);
    if (s_fd >= 0) {
        close(s_fd);
        s_fd = -1;
    }
    pthread_mutex_unlock(&s_mu);
}

static bool gs_emit(enum PtzVerb v, int speed) {
    pthread_mutex_lock(&s_mu);
    if (v == PTZ_STOP) {
        s_dir = 0;
        if (s_homing) {
            s_home_cancel = true;   // a stop stops the seek too
        }
    } else {
        int axis, dir;
        // Refused rather than accepted and dropped: while the head seeks its
        // stops a direction has nowhere to go, and an axis whose seek did not
        // finish has no position to keep it off them. Either way the caller
        // hears that nothing moved.
        if (!gs_verb_axis(&s_cfg, v, &axis, &dir) || s_homing || s_lost[axis]) {
            pthread_mutex_unlock(&s_mu);
            return false;
        }
        s_axis = axis;
        s_dir = dir;
        s_speed = speed;
    }
    pthread_cond_signal(&s_cv);   // wake the stepping thread; never blocks
    pthread_mutex_unlock(&s_mu);
    return true;
}

static bool gs_has(enum PtzVerb v) {
    return v == PTZ_STOP || v == PTZ_LEFT || v == PTZ_RIGHT || v == PTZ_UP ||
           v == PTZ_DOWN;
}

static const char *gs_proto_name(void) { return "gpiostep"; }
static bool gs_wake_noop(void) { return true; }   // nothing to wake
static int gs_fd_none(void) { return -1; }        // no UART: the zoom reader stays off

const Actuator act_gpiostep = {
    .name = "gpiostep",
    .reports_motion = true,
    .honours_speed = true,
    .proto_name = gs_proto_name,
    .open = gs_open,
    .close = gs_close,
    .emit = gs_emit,
    .has = gs_has,
    .wake_blob = gs_wake_noop,
    .wake = gs_wake_noop,
    .fd = gs_fd_none,
    .derives_mag = false,   // no zoom, so no magnification from anywhere
};
