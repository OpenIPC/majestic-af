// act_motorsd.c — motorsd backend behind the majestic-af Actuator interface.
//
// Owns two Unix socket connections to motorsd (/run/motorsd.sock by default):
//   - a command client driven exclusively by md_cmd_thread (so emit() and
//     move_steps() stay strictly non-blocking under motion.c's mo_mu);
//   - a subscribed event client driven by md_evt_thread to forward live
//     zoom_magnification updates into engine.c via af_zoom_report().

#include "actuator.h"
#include "motion.h"

#include <libmotors.h>
#include <majestic/af_plugin_abi.h>
#include <majestic/log.h>

#include <ctype.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define MD_DEFAULT_SOCKET "/run/motorsd.sock"
#define MD_STEP_MS 10L
#define MD_QUEUE_CAP 16
#define MD_FOCUS_SLICE_MS 25
#define MD_CONTINUOUS_LEASE_MS 15000U
#define MD_MIN_STEP_LEASE_MS 1500U
#define MD_BACKLASH_MS 240L
#define MD_ZOOM_SETTLE_MS 500L

enum MdPendingKind { MD_CMD_NONE = 0, MD_CMD_MOVE, MD_CMD_PRESET };

struct MdCmd {
    enum MdPendingKind kind;
    enum PtzVerb verb;
    int steps;
    int speed;
    enum PtzPresetOp preset_op;
    int preset_num;
};

static pthread_mutex_t md_mu = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t md_cv = PTHREAD_COND_INITIALIZER;
static pthread_cond_t md_done_cv = PTHREAD_COND_INITIALIZER;

static char md_socket[256] = MD_DEFAULT_SOCKET;
static struct motors_capabilities md_caps;
static char md_proto[80] = "motorsd";
static int md_frame_w = 0;
static int md_frame_h = 0;
static float md_zoom_mag = 0.0f;

static struct motors_client *md_cmd_client = NULL;
static struct motors_client *md_evt_client = NULL;

static bool md_open_flag = false;
static bool md_transport_ok = false;
static bool md_stop = false;
static bool md_cmd_valid = false;
static pthread_t md_cmd_thread_id;
static bool md_evt_valid = false;
static pthread_t md_evt_thread_id;

static struct MdCmd md_queue[MD_QUEUE_CAP];
static int md_q_head = 0;
static int md_q_count = 0;
static bool md_cmd_busy = false;
static unsigned md_focus_seq = 0;
static bool md_focus_abort = false;

static long md_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void md_record_zoom(float mag) {
    if (mag >= 0.5f && mag <= 100.0f) {
        pthread_mutex_lock(&md_mu);
        md_zoom_mag = mag;
        pthread_mutex_unlock(&md_mu);
        af_zoom_report(mag);
    }
}

static bool md_seed_mag(float mag) {
    if (mag < 0.5f || mag > 100.0f) {
        return false;
    }
    pthread_mutex_lock(&md_mu);
    bool seeded = false;
    if (md_zoom_mag <= 0.0f) {
        md_zoom_mag = mag;
        seeded = true;
    }
    pthread_mutex_unlock(&md_mu);
    return seeded;
}

static const char *resolve_socket_path(void) {
    const char *port = config_get_string("ptz", "port");
    if (port && port[0] == '/' && !strstr(port, "/tty")) {
        return port;
    }
    return MD_DEFAULT_SOCKET;
}

static void parse_initial_zoom(struct motors_client *client) {
    char response[4096];
    char error[128] = "";
    if (motors_request(client, "{\"version\":1,\"id\":\"init\",\"op\":\"status\"}",
                       response, sizeof response, error, sizeof error) != 0) {
        return;
    }
    const char *p = strstr(response, "\"zoom_magnification\"");
    if (!p) {
        return;
    }
    p = strchr(p, ':');
    if (!p) {
        return;
    }
    p++;
    while (*p && isspace((unsigned char)*p)) {
        p++;
    }
    char *end = NULL;
    double mag = strtod(p, &end);
    if (end != p) {
        md_record_zoom((float)mag);
    }
}

static bool verb_to_single(enum PtzVerb v, enum motors_client_axis *axis,
                           enum motors_direction *dir) {
    switch (v) {
    case PTZ_LEFT:  *axis = MOTORS_PAN;   *dir = MOTORS_LEFT;  return true;
    case PTZ_RIGHT: *axis = MOTORS_PAN;   *dir = MOTORS_RIGHT; return true;
    case PTZ_UP:    *axis = MOTORS_TILT;  *dir = MOTORS_UP;    return true;
    case PTZ_DOWN:  *axis = MOTORS_TILT;  *dir = MOTORS_DOWN;  return true;
    case PTZ_TELE:  *axis = MOTORS_ZOOM;  *dir = MOTORS_TELE;  return true;
    case PTZ_WIDE:  *axis = MOTORS_ZOOM;  *dir = MOTORS_WIDE;  return true;
    case PTZ_NEAR:  *axis = MOTORS_FOCUS; *dir = MOTORS_NEAR;  return true;
    case PTZ_FAR:   *axis = MOTORS_FOCUS; *dir = MOTORS_FAR;   return true;
    default:        return false;
    }
}

static bool verb_to_pantilt(enum PtzVerb v, enum motors_direction *pan_dir,
                            enum motors_direction *tilt_dir) {
    switch (v) {
    case PTZ_UPLEFT:
        *pan_dir = MOTORS_LEFT;  *tilt_dir = MOTORS_UP;   return true;
    case PTZ_UPRIGHT:
        *pan_dir = MOTORS_RIGHT; *tilt_dir = MOTORS_UP;   return true;
    case PTZ_DOWNLEFT:
        *pan_dir = MOTORS_LEFT;  *tilt_dir = MOTORS_DOWN; return true;
    case PTZ_DOWNRIGHT:
        *pan_dir = MOTORS_RIGHT; *tilt_dir = MOTORS_DOWN; return true;
    default:
        return false;
    }
}

static bool ensure_cmd_client(const char *sock_path) {
    if (md_cmd_client) {
        return true;
    }
    char err[160] = "";
    if (motors_open(&md_cmd_client, sock_path, err, sizeof err) != 0) {
        md_cmd_client = NULL;
        log_w("act_motorsd: reconnect failed: %s", err[0] ? err : "unknown error");
        return false;
    }
    return true;
}

static void reset_cmd_client(void) {
    if (md_cmd_client) {
        motors_close(md_cmd_client);
        md_cmd_client = NULL;
    }
}

static const char *preset_cmd_name(enum PtzPresetOp op) {
    switch (op) {
    case PTZ_PRESET_SET:   return "preset.set";
    case PTZ_PRESET_GO:    return "preset.call";
    case PTZ_PRESET_CLEAR: return "preset.clear";
    default:               return NULL;
    }
}

static bool acquire_move_leases(const char *sock_path, enum PtzVerb v, unsigned lease,
                                enum motors_client_axis *axis,
                                enum motors_direction *dir,
                                enum motors_direction *pdir,
                                enum motors_direction *tdir,
                                bool *is_pantilt, char *err, size_t err_sz) {
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!ensure_cmd_client(sock_path)) {
            return false;
        }
        bool ok = false;
        if (verb_to_pantilt(v, pdir, tdir)) {
            *is_pantilt = true;
            ok = motors_acquire(md_cmd_client, MOTORS_MANUAL, MOTORS_PAN, lease, err, err_sz) == 0 &&
                 motors_acquire(md_cmd_client, MOTORS_MANUAL, MOTORS_TILT, lease, err, err_sz) == 0;
        } else if (verb_to_single(v, axis, dir)) {
            *is_pantilt = false;
            ok = motors_acquire(md_cmd_client, MOTORS_MANUAL, *axis, lease, err, err_sz) == 0;
        } else {
            return false;
        }
        if (ok) {
            return true;
        }
        reset_cmd_client();
    }
    return false;
}

static bool probe_cmd_client(const char *sock_path, char *err, size_t err_sz) {
    for (int attempt = 0; attempt < 2; attempt++) {
        if (!ensure_cmd_client(sock_path)) {
            return false;
        }
        struct motors_capabilities probe = {0};
        if (motors_get_capabilities(md_cmd_client, &probe, err, err_sz) == 0) {
            return true;
        }
        reset_cmd_client();
    }
    return false;
}

static bool execute_pending(const char *sock_path, const struct MdCmd *cmd) {
    char err[160] = "";
    if (cmd->kind == MD_CMD_PRESET) {
        const char *cname = preset_cmd_name(cmd->preset_op);
        if (!cname || cmd->preset_num < 1 || cmd->preset_num > 255) {
            return true;
        }
        if (!probe_cmd_client(sock_path, err, sizeof err)) {
            return false;
        }
        char val[16];
        snprintf(val, sizeof val, "%d", cmd->preset_num);
        if (motors_command(md_cmd_client, cname, val, err, sizeof err) != 0) {
            /* Do not blindly repeat a potentially applied preset command. */
            reset_cmd_client();
            return false;
        }
        return true;
    }

    if (cmd->verb == PTZ_STOP) {
        for (int attempt = 0; attempt < 2; attempt++) {
            if (!ensure_cmd_client(sock_path)) {
                return false;
            }
            if (motors_stop_all(md_cmd_client, err, sizeof err) == 0) {
                (void)motors_release(md_cmd_client, err, sizeof err);
                return true;
            }
            reset_cmd_client();
        }
        return false;
    }

    unsigned dur = cmd->steps > 0 ? (unsigned)cmd->steps * (unsigned)MD_STEP_MS : 0U;
    unsigned lease = dur > 0 ? (dur + MD_MIN_STEP_LEASE_MS) : MD_CONTINUOUS_LEASE_MS;
    enum motors_client_axis axis = MOTORS_PAN;
    enum motors_direction dir = MOTORS_LEFT;
    enum motors_direction pdir = MOTORS_LEFT, tdir = MOTORS_UP;
    bool is_pantilt = false;

    if (!acquire_move_leases(sock_path, cmd->verb, lease, &axis, &dir, &pdir, &tdir,
                             &is_pantilt, err, sizeof err)) {
        return false;
    }

    int rc = is_pantilt
        ? motors_move_pantilt(md_cmd_client, pdir, tdir, (unsigned)cmd->speed, dur, err, sizeof err)
        : motors_move_speed(md_cmd_client, axis, dir, (unsigned)cmd->speed, dur, err, sizeof err);
    if (rc != 0) {
        /* Never reissue a non-idempotent counted/continuous move after send. */
        reset_cmd_client();
        return false;
    }
    return true;
}

static void *md_cmd_worker(void *arg) {
    (void)arg;
    char sock_path[sizeof md_socket];
    pthread_mutex_lock(&md_mu);
    snprintf(sock_path, sizeof sock_path, "%s", md_socket);
    for (;;) {
        while (md_q_count == 0 && !md_stop) {
            pthread_cond_wait(&md_cv, &md_mu);
        }
        if (md_stop) {
            break;
        }
        struct MdCmd cmd = md_queue[md_q_head];
        md_q_head = (md_q_head + 1) % MD_QUEUE_CAP;
        md_q_count--;
        md_cmd_busy = true;
        pthread_mutex_unlock(&md_mu);

        bool ok = execute_pending(sock_path, &cmd);

        pthread_mutex_lock(&md_mu);
        md_transport_ok = ok || md_cmd_client != NULL;
        md_cmd_busy = false;
        pthread_cond_broadcast(&md_done_cv);
    }
    md_cmd_busy = false;
    pthread_cond_broadcast(&md_done_cv);
    if (md_cmd_client) {
        char err[128] = "";
        (void)motors_stop_all(md_cmd_client, err, sizeof err);
        (void)motors_release(md_cmd_client, err, sizeof err);
        motors_close(md_cmd_client);
        md_cmd_client = NULL;
    }
    pthread_mutex_unlock(&md_mu);
    return NULL;
}

static void *md_evt_worker(void *arg) {
    (void)arg;
    char sock_path[sizeof md_socket];
    pthread_mutex_lock(&md_mu);
    snprintf(sock_path, sizeof sock_path, "%s", md_socket);
    struct motors_client *evt = md_evt_client;
    md_evt_client = NULL;
    pthread_mutex_unlock(&md_mu);

    for (;;) {
        pthread_mutex_lock(&md_mu);
        bool stop = md_stop;
        pthread_mutex_unlock(&md_mu);
        if (stop) {
            break;
        }

        char err[128] = "";
        if (!evt) {
            if (motors_open(&evt, sock_path, err, sizeof err) != 0 ||
                motors_subscribe(evt, err, sizeof err) != 0) {
                if (evt) {
                    motors_close(evt);
                    evt = NULL;
                }
                pthread_mutex_lock(&md_mu);
                md_transport_ok = false;
                pthread_mutex_unlock(&md_mu);
                usleep(200000);
                continue;
            }
            pthread_mutex_lock(&md_mu);
            md_transport_ok = true;
            pthread_mutex_unlock(&md_mu);
            parse_initial_zoom(evt);
        }

        int rc = motors_poll(evt, 200, err, sizeof err);
        if (rc < 0) {
            motors_close(evt);
            evt = NULL;
            pthread_mutex_lock(&md_mu);
            md_transport_ok = false;
            pthread_mutex_unlock(&md_mu);
            usleep(200000);
            continue;
        }
        if (rc > 0) {
            float mag = 0.0f;
            if (motors_zoom_magnification(evt, &mag, NULL)) {
                md_record_zoom(mag);
            }
        }
    }

    if (evt) {
        motors_close(evt);
    }
    return NULL;
}

static bool md_open(void) {
    const char *sock = resolve_socket_path();
    char err[160] = "";

    struct motors_client *cmd = NULL;
    if (motors_open(&cmd, sock, err, sizeof err) != 0) {
        log_e("act_motorsd: connect %s failed: %s", sock, err[0] ? err : "unknown");
        return false;
    }

    struct motors_capabilities caps = {0};
    if (motors_get_capabilities(cmd, &caps, err, sizeof err) != 0 || !caps.available) {
        log_e("act_motorsd: driver not available on %s: %s", sock,
              err[0] ? err : "unavailable");
        motors_close(cmd);
        return false;
    }

    struct motors_client *evt = NULL;
    if (motors_open(&evt, sock, err, sizeof err) == 0) {
        if (motors_subscribe(evt, err, sizeof err) != 0) {
            motors_close(evt);
            evt = NULL;
        }
    }

    pthread_mutex_lock(&md_mu);
    md_zoom_mag = 0.0f;
    pthread_mutex_unlock(&md_mu);
    parse_initial_zoom(cmd);

    int fw = config_get_int("ptz", "frameWidth");
    int fh = config_get_int("ptz", "frameHeight");

    pthread_mutex_lock(&md_mu);
    snprintf(md_socket, sizeof md_socket, "%s", sock);
    md_caps = caps;
    if (caps.driver[0]) {
        snprintf(md_proto, sizeof md_proto, "motorsd:%s", caps.driver);
    } else {
        snprintf(md_proto, sizeof md_proto, "motorsd");
    }
    md_frame_w = fw > 0 ? fw : 0;
    md_frame_h = fh > 0 ? fh : 0;
    md_cmd_client = cmd;
    md_evt_client = evt;
    md_stop = false;
    md_q_head = 0;
    md_q_count = 0;
    md_focus_abort = false;
    md_transport_ok = true;
    md_open_flag = true;

    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setstacksize(&attr, 0x20000);
    if (pthread_create(&md_cmd_thread_id, &attr, md_cmd_worker, NULL) != 0) {
        pthread_attr_destroy(&attr);
        md_open_flag = false;
        md_cmd_client = NULL;
        md_evt_client = NULL;
        pthread_mutex_unlock(&md_mu);
        motors_close(cmd);
        if (evt) motors_close(evt);
        log_e("act_motorsd: cannot create command thread");
        return false;
    }
    md_cmd_valid = true;

    if (pthread_create(&md_evt_thread_id, &attr, md_evt_worker, NULL) != 0) {
        md_evt_valid = false;
        if (md_evt_client) {
            motors_close(md_evt_client);
            md_evt_client = NULL;
        }
    } else {
        md_evt_valid = true;
    }
    pthread_attr_destroy(&attr);
    pthread_mutex_unlock(&md_mu);

    log_i("act_motorsd: ready on %s (driver=%s axes=0x%x)",
          sock, caps.driver, (unsigned)caps.axes);
    return true;
}

static void md_close(void) {
    pthread_mutex_lock(&md_mu);
    if (!md_open_flag && !md_cmd_valid && !md_evt_valid) {
        pthread_mutex_unlock(&md_mu);
        return;
    }
    md_stop = true;
    md_open_flag = false;
    md_transport_ok = false;
    md_focus_abort = true;
    md_focus_seq++;
    md_q_head = 0;
    md_q_count = 0;
    pthread_cond_broadcast(&md_cv);
    pthread_cond_broadcast(&md_done_cv);
    bool join_cmd = md_cmd_valid;
    pthread_t cmd_t = md_cmd_thread_id;
    md_cmd_valid = false;
    bool join_evt = md_evt_valid;
    pthread_t evt_t = md_evt_thread_id;
    md_evt_valid = false;
    pthread_mutex_unlock(&md_mu);

    if (join_cmd) {
        pthread_join(cmd_t, NULL);
    }
    if (join_evt) {
        pthread_join(evt_t, NULL);
    }
}

static bool md_has_locked(enum PtzVerb v) {
    if (v == PTZ_STOP) {
        return true;
    }
    enum motors_direction pdir, tdir;
    if (verb_to_pantilt(v, &pdir, &tdir)) {
        return (md_caps.axes & MOTORS_AXIS_MASK(MOTORS_PAN)) &&
               (md_caps.axes & MOTORS_AXIS_MASK(MOTORS_TILT));
    }
    enum motors_client_axis axis;
    enum motors_direction dir;
    if (verb_to_single(v, &axis, &dir)) {
        return (md_caps.axes & MOTORS_AXIS_MASK(axis)) != 0;
    }
    return false;
}

static bool md_has(enum PtzVerb v) {
    pthread_mutex_lock(&md_mu);
    bool ok = md_open_flag && md_has_locked(v);
    pthread_mutex_unlock(&md_mu);
    return ok;
}

static bool md_has_presets_locked(void) {
    if (!md_open_flag) {
        return false;
    }
    if (!(md_caps.axes & (MOTORS_AXIS_MASK(MOTORS_PAN) | MOTORS_AXIS_MASK(MOTORS_TILT)))) {
        return false;
    }
    return strcmp(md_caps.driver, "anjoy") == 0 ||
           strcmp(md_caps.driver, "pelcod") == 0 ||
           strcmp(md_caps.driver, "mock") == 0;
}

static bool md_has_presets(void) {
    pthread_mutex_lock(&md_mu);
    bool ok = md_has_presets_locked();
    pthread_mutex_unlock(&md_mu);
    return ok;
}

static bool md_push_locked(const struct MdCmd *cmd) {
    if (!md_open_flag || (!md_transport_ok && !md_cmd_client)) {
        return false;
    }
    md_focus_abort = true;
    md_focus_seq++;
    if (md_q_count >= MD_QUEUE_CAP) {
        if (cmd->kind == MD_CMD_MOVE && cmd->verb == PTZ_STOP) {
            int last = (md_q_head + md_q_count - 1) % MD_QUEUE_CAP;
            md_queue[last] = *cmd;
            pthread_cond_signal(&md_cv);
            return true;
        }
        return false;
    }
    int tail = (md_q_head + md_q_count) % MD_QUEUE_CAP;
    md_queue[tail] = *cmd;
    md_q_count++;
    pthread_cond_signal(&md_cv);
    return true;
}

static bool md_enqueue(enum PtzVerb v, int steps, int speed) {
    pthread_mutex_lock(&md_mu);
    if (!md_has_locked(v)) {
        pthread_mutex_unlock(&md_mu);
        return false;
    }
    struct MdCmd cmd = {
        .kind = MD_CMD_MOVE,
        .verb = v,
        .steps = steps,
        .speed = speed,
    };
    bool ok = md_push_locked(&cmd);
    pthread_mutex_unlock(&md_mu);
    return ok;
}

static bool md_emit(enum PtzVerb v, int speed) {
    return md_enqueue(v, 0, speed);
}

static bool md_move_steps(enum PtzVerb v, int steps, int speed) {
    if (steps <= 0 || !ptz_verb_is_pantilt(v)) {
        return false;
    }
    return md_enqueue(v, steps, speed);
}

static long md_step_ms(enum PtzVerb v, int speed) {
    (void)v;
    (void)speed;
    return MD_STEP_MS;
}

static bool md_frame_steps(int *w, int *h) {
    pthread_mutex_lock(&md_mu);
    int fw = md_frame_w;
    int fh = md_frame_h;
    bool has_zoom = (md_caps.axes & MOTORS_AXIS_MASK(MOTORS_ZOOM)) != 0;
    float mag = md_zoom_mag;
    pthread_mutex_unlock(&md_mu);
    if (fw <= 0 || fh <= 0) {
        return false;
    }
    if (has_zoom) {
        if (mag <= 0.0f) {
            return false;
        }
        fw = (int)((double)fw / (double)mag + 0.5);
        fh = (int)((double)fh / (double)mag + 0.5);
        if (fw < 1) fw = 1;
        if (fh < 1) fh = 1;
    }
    *w = fw;
    *h = fh;
    return true;
}

static bool md_preset(enum PtzPresetOp op, int n) {
    if (!preset_cmd_name(op) || n < 1 || n > 255) {
        return false;
    }
    pthread_mutex_lock(&md_mu);
    if (!md_has_presets_locked()) {
        pthread_mutex_unlock(&md_mu);
        return false;
    }
    struct MdCmd cmd = {
        .kind = MD_CMD_PRESET,
        .preset_op = op,
        .preset_num = n,
    };
    bool ok = md_push_locked(&cmd);
    pthread_mutex_unlock(&md_mu);
    return ok;
}

static const char *md_proto_name(void) {
    pthread_mutex_lock(&md_mu);
    const char *p = md_proto;
    pthread_mutex_unlock(&md_mu);
    return p;
}

static bool md_wake_noop(void) { return true; }
static int md_fd_none(void) { return -1; }

static long md_focus_ms(int dir, unsigned ms) {
    if (dir == 0 || ms == 0) {
        return 0;
    }
    char sock_path[sizeof md_socket];
    pthread_mutex_lock(&md_mu);
    while ((md_q_count > 0 || md_cmd_busy) && !md_stop) {
        pthread_cond_wait(&md_done_cv, &md_mu);
    }
    if (!md_open_flag || md_stop || (!md_transport_ok && !md_cmd_client) ||
        !(md_caps.axes & MOTORS_AXIS_MASK(MOTORS_FOCUS))) {
        pthread_mutex_unlock(&md_mu);
        return -1;
    }
    md_focus_abort = false;
    unsigned my_seq = md_focus_seq;
    snprintf(sock_path, sizeof sock_path, "%s", md_socket);
    pthread_mutex_unlock(&md_mu);

    struct motors_client *cl = NULL;
    char err[128] = "";
    if (motors_open(&cl, sock_path, err, sizeof err) != 0) {
        return -1;
    }
    long done = -1;
    if (motors_subscribe(cl, err, sizeof err) == 0 &&
        motors_acquire(cl, MOTORS_AF, MOTORS_FOCUS, ms + 2000U, err, sizeof err) == 0 &&
        motors_move(cl, MOTORS_FOCUS, dir < 0 ? MOTORS_NEAR : MOTORS_FAR, ms, err, sizeof err) == 0) {
        long deadline = md_now_ms() + (long)ms + 2000L;
        for (;;) {
            pthread_mutex_lock(&md_mu);
            bool preempted = md_stop || !md_open_flag || md_focus_abort || md_focus_seq != my_seq;
            pthread_mutex_unlock(&md_mu);
            if (preempted) {
                (void)motors_stop(cl, MOTORS_FOCUS, err, sizeof err);
                break;
            }
            long rem = deadline - md_now_ms();
            if (rem <= 0) {
                break;
            }
            int slice = rem > MD_FOCUS_SLICE_MS ? MD_FOCUS_SLICE_MS : (int)rem;
            err[0] = '\0';
            if (motors_wait_movement(cl, MOTORS_FOCUS, slice, NULL, err, sizeof err) == 0) {
                pthread_mutex_lock(&md_mu);
                bool still_valid = !md_stop && md_open_flag && !md_focus_abort && md_focus_seq == my_seq;
                pthread_mutex_unlock(&md_mu);
                if (still_valid && !motors_lease_revoked(cl, MOTORS_FOCUS)) {
                    done = (long)ms;
                }
                break;
            }
            if (motors_lease_revoked(cl, MOTORS_FOCUS) || strstr(err, "timed out") == NULL) {
                break;
            }
        }
    }
    if (!motors_lease_revoked(cl, MOTORS_FOCUS)) {
        (void)motors_release(cl, err, sizeof err);
    }
    motors_close(cl);
    return done;
}

const Actuator act_motorsd = {
    .name = "motorsd",
    .reports_motion = false,
    .honours_speed = true,
    .proto_name = md_proto_name,
    .open = md_open,
    .close = md_close,
    .emit = md_emit,
    .move_steps = md_move_steps,
    .step_ms = md_step_ms,
    .frame_steps = md_frame_steps,
    .position = NULL,
    .preset = md_preset,
    .has_presets = md_has_presets,
    .has = md_has,
    .wake_blob = md_wake_noop,
    .wake = md_wake_noop,
    .fd = md_fd_none,
    .derives_mag = true,
    .seed_mag = md_seed_mag,
    .backlash_ms = MD_BACKLASH_MS,
    .zoom_settle_ms = MD_ZOOM_SETTLE_MS,
    .zoom_late_ms = 0,
    .focus_ms = md_focus_ms,
};
