// act_gpiostep — pan/tilt heads whose two 4-wire steppers hang straight off GPIO.
//
// The Goke GK7205V510 PTZ cameras (the Zenointel SD-2N-4G, an NC-IPTC2XXX_DL_4G
// board) have no motor MCU and no lens driver for the head: the SoC energises the
// coils itself. OpenIPC/firmware's gpiostep.ko does that from kernel context and
// exposes one ioctl, "move the pan coil by N steps and the tilt coil by M"; this
// backend turns motion.c's continuous verbs into those moves.
//
// The shape is act_ms41908's. emit() only records a direction and never blocks; a
// stepping thread issues short moves (GS_CHUNK steps, a few tens of ms) while a
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

// Steps per move while a direction is held: 2 x 8 microsteps at 2-3 ms is 32-48 ms,
// which is how late a stop can land.
#define GS_CHUNK 2

static pthread_mutex_t s_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t s_cv = PTHREAD_COND_INITIALIZER;
static pthread_t s_thread;
static bool s_thread_valid = false;
static int s_run = 0;
static int s_fd = -1;
static GsConfig s_cfg;
static int s_axis = GS_PAN;   // the axis armed by the last verb
static int s_dir = 0;         // its raw direction, 0 = stopped
static int s_pos[GS_AXES];
static bool s_homed = false;   // s_pos is a physical position
static bool s_dirty = false;   // s_pos moved since it was last saved
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

static bool limits_known(void) {
    return s_cfg.travel[GS_PAN] > 0 && s_cfg.travel[GS_TILT] > 0;
}

static bool restore_pos(void) {
    FILE *f = fopen(GS_POS_PATH, "r");
    if (!f) {
        return false;
    }
    int p, t;
    bool ok = fscanf(f, "%d %d", &p, &t) == 2 && p >= 0 && t >= 0 &&
              p <= s_cfg.travel[GS_PAN] && t <= s_cfg.travel[GS_TILT];
    fclose(f);
    if (ok) {
        s_pos[GS_PAN] = p;
        s_pos[GS_TILT] = t;
    }
    return ok;
}

static void save_pos(void) {
    FILE *f = fopen(GS_POS_PATH, "w");
    if (f) {
        fprintf(f, "%d %d\n", s_pos[GS_PAN], s_pos[GS_TILT]);
        fclose(f);
    }
}

// One move on the coils. Blocks for as long as the module steps; called only from
// the stepping thread, with s_mu released.
static bool coil_move(int axis, int steps) {
    struct gpiostep_move m = {
        .pan = axis == GS_PAN ? steps : 0,
        .tilt = axis == GS_TILT ? steps : 0,
        .delay_us = s_cfg.delay_us[axis],
    };
    if (ioctl(s_fd, GPIOSTEP_MOVE, &m) < 0) {
        log_e("gpiostep: move: %s", strerror(errno));
        return false;
    }
    return true;
}

// Steps per move while homing: long enough not to waste time on ioctls, short
// enough (~0.3-0.4 s) that a majestic stop during the seek is not held up by it.
#define GS_HOME_CHUNK 20

static bool still_running(void) {
    pthread_mutex_lock(&s_mu);
    bool r = s_run != 0;
    pthread_mutex_unlock(&s_mu);
    return r;
}

// `steps` on one axis in GS_HOME_CHUNK pieces, giving up if the plugin is being
// unloaded. Returns false when it was interrupted.
static bool home_move(int axis, int steps) {
    int dir = steps < 0 ? -1 : 1;
    for (int left = steps * dir; left > 0; left -= GS_HOME_CHUNK) {
        if (!still_running()) {
            return false;
        }
        int n = left < GS_HOME_CHUNK ? left : GS_HOME_CHUNK;
        if (!coil_move(axis, dir * n)) {
            return false;
        }
    }
    return true;
}

// Drive both axes into their stops at 0, then to the middle of their travel. The
// overshoot past the travel makes sure the head reaches the stop wherever it was
// left; the steppers slip against it without harm, the way the vendor firmware's
// own self-check does at boot. Called without s_mu. Returns false if interrupted,
// leaving the position unknown.
static bool home(void) {
    log_i("gpiostep: homing pan %d / tilt %d steps", s_cfg.travel[GS_PAN],
          s_cfg.travel[GS_TILT]);
    for (int a = 0; a < GS_AXES; a++) {
        int t = s_cfg.travel[a];
        if (!home_move(a, -(t + t / 8 + 8))) {
            return false;
        }
    }
    for (int a = 0; a < GS_AXES; a++) {
        if (!home_move(a, s_cfg.travel[a] / 2)) {
            return false;
        }
        s_pos[a] = s_cfg.travel[a] / 2;
    }
    return true;
}

static void *step_thread(void *arg) {
    (void)arg;
    pthread_mutex_lock(&s_mu);
    if (limits_known() && !s_homed) {
        if (restore_pos()) {
            log_i("gpiostep: position %d/%d kept from this boot", s_pos[GS_PAN],
                  s_pos[GS_TILT]);
            s_homed = true;
        } else if (s_cfg.home) {
            pthread_mutex_unlock(&s_mu);
            bool done = home();
            pthread_mutex_lock(&s_mu);
            if (done) {
                s_homed = true;
                save_pos();
                log_i("gpiostep: homed, centred at %d/%d", s_pos[GS_PAN],
                      s_pos[GS_TILT]);
            }
            s_dir = 0;   // a verb sent during the seek is stale now
        }
    }
    while (s_run) {
        if (s_dir == 0) {
            if (s_dirty) {
                save_pos();
                s_dirty = false;
            }
            pthread_cond_wait(&s_cv, &s_mu);
            continue;
        }
        int axis = s_axis, dir = s_dir;
        int travel = s_homed ? s_cfg.travel[axis] : 0;
        int n = gs_clamp_step(s_pos[axis], dir, GS_CHUNK, travel);
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
        pthread_mutex_unlock(&s_mu);
        bool ok = coil_move(axis, dir * n);
        pthread_mutex_lock(&s_mu);
        if (!ok) {
            s_dir = 0;
            continue;
        }
        if (s_homed) {
            s_pos[axis] += dir * n;
            s_dirty = true;
        }
    }
    if (s_dirty) {
        save_pos();
    }
    pthread_mutex_unlock(&s_mu);
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
    (void)speed;   // the coils step at the board's configured rate
    pthread_mutex_lock(&s_mu);
    if (v == PTZ_STOP) {
        s_dir = 0;
    } else {
        int axis, dir;
        if (!gs_verb_axis(&s_cfg, v, &axis, &dir)) {
            pthread_mutex_unlock(&s_mu);
            return false;
        }
        s_axis = axis;
        s_dir = dir;
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
