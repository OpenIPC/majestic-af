// majestic-af — out-of-core autofocus plugin for majestic (OpenIPC).
//
// The ABI boundary (include/majestic/af_plugin_abi.h): majestic dlsym's
// af_plugin_call / af_plugin_exit here and calls them from its /autofocus,
// /zoom, and /ptz handlers; this plugin resolves the core HAL seams (sdk_get_focus_value,
// sdk_set_zoom_mag, config_get_*) against the executable at dlopen.
//
// The AF engine and search live in engine.c and the af*.c search modules. Motor
// access lives behind the interface in af_motor.c. This file is only the thin
// adapter from the two-token command ABI to the AF engine.

#include <majestic/af.h>
#include <majestic/af_plugin_abi.h>

#include <errno.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <libmotors.h>

// engine.c: synchronous teardown — joins the worker + reader before we return.
void af_engine_stop(void);

static const char *map_trigger(int r) {
    // af_trigger: 0 started, 2 preempted-and-rearmed, else busy (-1 shouldn't
    // happen — the plugin only loaded because autofocus is enabled).
    return r == 0 ? "started" : r == 2 ? "restarted" : "busy";
}

static pthread_mutex_t ptz_mu = PTHREAD_MUTEX_INITIALIZER;
static struct motors_client *ptz_client;
static unsigned ptz_dirty_axes;
static char ptz_reply[256];

static bool ptz_should_autofocus(unsigned dirty_axes) {
    if (!dirty_axes) return false;
    const char *mode = config_get_string("isp.autofocus", "mode");
    if (mode && *mode && !strcmp(mode, "manual")) return false;

    char trig[32] = "ptz";
    FILE *f = fopen("/etc/af_triggers.conf", "r");
    if (f) {
        char line[128];
        while (fgets(line, sizeof(line), f)) {
            if (!strncmp(line, "trigger_mode=", 13)) {
                char *val = line + 13;
                while (*val == ' ' || *val == '\t') val++;
                char *end = val + strlen(val);
                while (end > val && (end[-1] == ' ' || end[-1] == '\t' ||
                                     end[-1] == '\r' || end[-1] == '\n')) {
                    *--end = '\0';
                }
                if (*val) {
                    snprintf(trig, sizeof(trig), "%.31s", val);
                }
                break;
            }
        }
        fclose(f);
    }

    if (!strcmp(trig, "manual")) return false;
    if (!strcmp(trig, "zoom_only"))
        return (dirty_axes & MOTORS_AXIS_MASK(MOTORS_ZOOM)) != 0;
    if (!strcmp(trig, "pan_tilt_only"))
        return (dirty_axes & (MOTORS_AXIS_MASK(MOTORS_PAN) | MOTORS_AXIS_MASK(MOTORS_TILT))) != 0;
    return (dirty_axes & (MOTORS_AXIS_MASK(MOTORS_ZOOM) |
                          MOTORS_AXIS_MASK(MOTORS_PAN) |
                          MOTORS_AXIS_MASK(MOTORS_TILT))) != 0;
}

static void ptz_disconnect(void) {
    if (!ptz_client) return;
    motors_close(ptz_client);
    ptz_client = NULL;
}

static bool ptz_connect(char *error, size_t error_size) {
    if (ptz_client) return true;
    return motors_open(&ptz_client, "/run/motorsd.sock", error, error_size) == 0;
}

static bool ptz_parse(const char *name, enum motors_client_axis *axis,
                      enum motors_direction *direction) {
    struct mapping {
        const char *name;
        enum motors_client_axis axis;
        enum motors_direction direction;
    };
    static const struct mapping mappings[] = {
        {"left", MOTORS_PAN, MOTORS_LEFT}, {"right", MOTORS_PAN, MOTORS_RIGHT},
        {"up", MOTORS_TILT, MOTORS_UP}, {"down", MOTORS_TILT, MOTORS_DOWN},
        {"tele", MOTORS_ZOOM, MOTORS_TELE}, {"wide", MOTORS_ZOOM, MOTORS_WIDE},
        {"near", MOTORS_FOCUS, MOTORS_NEAR}, {"far", MOTORS_FOCUS, MOTORS_FAR},
        {"open", MOTORS_IRIS, MOTORS_OPEN}, {"close", MOTORS_IRIS, MOTORS_CLOSE},
    };
    for (size_t i = 0; i < sizeof(mappings) / sizeof(mappings[0]); ++i) {
        if (!strcmp(name, mappings[i].name)) {
            *axis = mappings[i].axis;
            *direction = mappings[i].direction;
            return true;
        }
    }
    return false;
}

static const char *ptz_describe(void) {
    char error[160] = "";
    pthread_mutex_lock(&ptz_mu);
    if (!ptz_connect(error, sizeof(error))) {
        pthread_mutex_unlock(&ptz_mu);
        return "unavailable";
    }
    struct motors_capabilities caps = {0};
    if (motors_get_capabilities(ptz_client, &caps, error, sizeof(error)) != 0 ||
        !caps.available) {
        ptz_disconnect();
        pthread_mutex_unlock(&ptz_mu);
        return "unavailable";
    }
    char verbs[96] = "stop";
    if (caps.axes & MOTORS_AXIS_MASK(MOTORS_TILT)) strcat(verbs, ",up,down");
    if (caps.axes & MOTORS_AXIS_MASK(MOTORS_PAN)) strcat(verbs, ",left,right");
    if (caps.axes & MOTORS_AXIS_MASK(MOTORS_ZOOM)) strcat(verbs, ",wide,tele");
    if (caps.axes & MOTORS_AXIS_MASK(MOTORS_FOCUS)) strcat(verbs, ",near,far");
    if (caps.axes & MOTORS_AXIS_MASK(MOTORS_IRIS)) strcat(verbs, ",open,close");
    snprintf(ptz_reply, sizeof(ptz_reply),
             "driver=%s state=ready verbs=%s axes=%s%s%s%s%s", caps.driver,
             verbs,
             caps.axes & MOTORS_AXIS_MASK(MOTORS_PAN) ? "pan " : "",
             caps.axes & MOTORS_AXIS_MASK(MOTORS_TILT) ? "tilt " : "",
             caps.axes & MOTORS_AXIS_MASK(MOTORS_ZOOM) ? "zoom " : "",
             caps.axes & MOTORS_AXIS_MASK(MOTORS_FOCUS) ? "focus " : "",
             caps.axes & MOTORS_AXIS_MASK(MOTORS_IRIS) ? "iris" : "");
    pthread_mutex_unlock(&ptz_mu);
    return ptz_reply;
}

static struct {
    bool active;
    int dp, dt, df;
    uint64_t start_ms;
    uint64_t end_ms;
    unsigned gen;
} ptz_track;

static uint64_t ptz_pos_now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)(ts.tv_nsec / 1000000L);
}

static void ptz_pos_read(long *pan, long *tilt, long *focus) {
    *pan = 0;
    *tilt = 0;
    *focus = 0;
    FILE *f = fopen("/tmp/ptz_pos.conf", "r");
    if (!f) f = fopen("/etc/ptz_pos.conf", "r");
    if (!f) return;
    char line[128];
    while (fgets(line, sizeof(line), f)) {
        long v = 0;
        if (sscanf(line, "PAN_MS=%ld", &v) == 1) *pan = v;
        else if (sscanf(line, "TILT_MS=%ld", &v) == 1) *tilt = v;
        else if (sscanf(line, "FOCUS_MS=%ld", &v) == 1) *focus = v;
    }
    fclose(f);
}

static void ptz_pos_write(long pan, long tilt, long focus, bool persist) {
    char buf[128];
    int n = snprintf(buf, sizeof(buf), "PAN_MS=%ld\nTILT_MS=%ld\nFOCUS_MS=%ld\n",
                     pan, tilt, focus);
    if (n <= 0 || (size_t)n >= sizeof(buf)) return;
    FILE *f = fopen("/tmp/ptz_pos.conf.tmp", "w");
    if (f) {
        fwrite(buf, 1, (size_t)n, f);
        fclose(f);
        rename("/tmp/ptz_pos.conf.tmp", "/tmp/ptz_pos.conf");
    }
    if (!persist) return;
    char cur[128] = "";
    FILE *fe = fopen("/etc/ptz_pos.conf", "r");
    if (fe) {
        size_t r = fread(cur, 1, sizeof(cur) - 1, fe);
        cur[r] = '\0';
        fclose(fe);
    }
    if (strcmp(cur, buf) != 0) {
        FILE *fw = fopen("/etc/ptz_pos.conf.tmp", "w");
        if (fw) {
            fwrite(buf, 1, (size_t)n, fw);
            fclose(fw);
            if (rename("/etc/ptz_pos.conf.tmp", "/etc/ptz_pos.conf") != 0)
                unlink("/etc/ptz_pos.conf.tmp");
        }
    }
}

static void ptz_write_live_locked(long base_pan, long base_tilt, long base_focus) {
    if (!ptz_track.active) {
        unlink("/tmp/ptz_live.conf");
        return;
    }
    FILE *f = fopen("/tmp/ptz_live.conf.tmp", "w");
    if (!f) return;
    fprintf(f,
            "BASE_PAN=%ld\nBASE_TILT=%ld\nBASE_FOCUS=%ld\n"
            "DP=%d\nDT=%d\nDF=%d\n"
            "START_MS=%llu\nEND_MS=%llu\nMOVING=1\n",
            base_pan, base_tilt, base_focus,
            ptz_track.dp, ptz_track.dt, ptz_track.df,
            (unsigned long long)ptz_track.start_ms,
            (unsigned long long)ptz_track.end_ms);
    fclose(f);
    rename("/tmp/ptz_live.conf.tmp", "/tmp/ptz_live.conf");
}

static void ptz_commit_active_locked(uint64_t now_ms, bool persist) {
    if (!ptz_track.active) return;
    uint64_t clamped = now_ms < ptz_track.end_ms ? now_ms : ptz_track.end_ms;
    long elapsed = clamped > ptz_track.start_ms ? (long)(clamped - ptz_track.start_ms) : 0;
    long pan = 0, tilt = 0, focus = 0;
    ptz_pos_read(&pan, &tilt, &focus);
    pan += (long)ptz_track.dp * elapsed;
    tilt += (long)ptz_track.dt * elapsed;
    focus += (long)ptz_track.df * elapsed;
    ptz_track.active = false;
    ptz_pos_write(pan, tilt, focus, persist);
    unlink("/tmp/ptz_live.conf");
}

struct ptz_expire_arg {
    unsigned duration;
    unsigned gen;
};

static void *ptz_expire_thread(void *raw) {
    struct ptz_expire_arg *arg = (struct ptz_expire_arg *)raw;
    unsigned duration = arg->duration;
    unsigned gen = arg->gen;
    free(arg);
    struct timespec ts = {
        .tv_sec = (time_t)((duration + 25U) / 1000U),
        .tv_nsec = (long)(((duration + 25U) % 1000U) * 1000000L),
    };
    nanosleep(&ts, NULL);
    pthread_mutex_lock(&ptz_mu);
    if (ptz_track.active && ptz_track.gen == gen)
        ptz_commit_active_locked(ptz_track.end_ms, true);
    pthread_mutex_unlock(&ptz_mu);
    return NULL;
}

static const char *ptz_move(const char *value) {
    if (!*value) return ptz_describe();
    char name[16];
    unsigned duration = 0;
    const char *colon = strchr(value, ':');
    size_t length = colon ? (size_t)(colon - value) : strlen(value);
    if (!length || length >= sizeof(name)) return NULL;
    memcpy(name, value, length);
    name[length] = '\0';
    if (colon) {
        char *end = NULL;
        errno = 0;
        unsigned long parsed = strtoul(colon + 1, &end, 10);
        if (errno || !end || end == colon + 1 || *end || parsed > 5000)
            return NULL;
        duration = (unsigned)parsed;
    }

    char error[160] = "";
    pthread_mutex_lock(&ptz_mu);
    if (!ptz_connect(error, sizeof(error))) {
        pthread_mutex_unlock(&ptz_mu);
        return "unavailable";
    }
    if (!strcmp(name, "stop")) {
        ptz_commit_active_locked(ptz_pos_now_ms(), true);
        int result = motors_stop_all(ptz_client, error, sizeof(error));
        if (result == 0) result = motors_release(ptz_client, error, sizeof(error));
        bool run_af = (result == 0 && ptz_should_autofocus(ptz_dirty_axes));
        ptz_dirty_axes = 0;
        if (result != 0) ptz_disconnect();
        pthread_mutex_unlock(&ptz_mu);
        (void)af_cancel_pass();
        if (run_af) (void)af_trigger(true);
        return result == 0 ? "stopped" : "unavailable";
    }

    enum motors_client_axis axis;
    enum motors_direction direction;
    if (!ptz_parse(name, &axis, &direction)) {
        pthread_mutex_unlock(&ptz_mu);
        return NULL;
    }
    if (!duration) {
        int configured = config_get_int("isp.autofocus", "pulse");
        duration = configured >= 50 && configured <= 3000
                       ? (unsigned)configured : 500U;
    }
    uint64_t now_ms = ptz_pos_now_ms();
    ptz_commit_active_locked(now_ms, false);
    int result = motors_acquire(ptz_client, MOTORS_MANUAL, axis,
                                duration + 1000U, error, sizeof(error));
    if (result == 0)
        result = motors_move(ptz_client, axis, direction, duration,
                             error, sizeof(error));
    bool trigger_af = false;
    if (result == 0) {
        int dp = (axis == MOTORS_PAN) ? (direction == MOTORS_RIGHT ? 1 : -1) : 0;
        int dt = (axis == MOTORS_TILT) ? (direction == MOTORS_UP ? 1 : -1) : 0;
        int df = (axis == MOTORS_FOCUS) ? (direction == MOTORS_FAR ? 1 : -1) : 0;
        if (dp || dt || df) {
            long pan = 0, tilt = 0, focus = 0;
            ptz_pos_read(&pan, &tilt, &focus);
            ptz_track.active = true;
            ptz_track.dp = dp;
            ptz_track.dt = dt;
            ptz_track.df = df;
            ptz_track.start_ms = now_ms;
            ptz_track.end_ms = now_ms + duration;
            ptz_track.gen++;
            ptz_write_live_locked(pan, tilt, focus);
            struct ptz_expire_arg *arg = malloc(sizeof(*arg));
            if (arg) {
                arg->duration = duration;
                arg->gen = ptz_track.gen;
                pthread_t th;
                pthread_attr_t attr;
                pthread_attr_init(&attr);
                pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
                if (pthread_create(&th, &attr, ptz_expire_thread, arg) != 0)
                    free(arg);
                pthread_attr_destroy(&attr);
            }
        }
        if (axis == MOTORS_FOCUS) {
            af_note_manual_focus();
        } else {
            (void)af_cancel_pass();
            if (axis == MOTORS_ZOOM || axis == MOTORS_PAN || axis == MOTORS_TILT) {
                ptz_dirty_axes |= MOTORS_AXIS_MASK(axis);
                trigger_af = ptz_should_autofocus(ptz_dirty_axes);
            }
        }
    } else {
        ptz_disconnect();
    }
    pthread_mutex_unlock(&ptz_mu);
    if (trigger_af) af_trigger_after(duration + 1200U);
    if (result != 0) return "unavailable";
    snprintf(ptz_reply, sizeof(ptz_reply), "moving %s", name);
    return ptz_reply;
}

const char *af_plugin_call(const char *cmd, const char *val) {
    if (!cmd || !val) {
        return NULL;
    }

    if (!strcmp(cmd, "autofocus")) {
        if (!strcmp(val, "status")) {
            return af_status();               // "idle" | "running" | "done fv=… …"
        }
        if (!strcmp(val, "run")) {
            return map_trigger(af_trigger(false));
        }
        if (!strcmp(val, "settle")) {
            return map_trigger(af_trigger(true));
        }
        if (!strcmp(val, "cancel")) {
            int r = af_cancel_pass();
            return r == 0 ? "cancelled" : r == 1 ? "idle" : "unavailable";
        }
        return NULL;
    }

    if (!strcmp(cmd, "ptz")) return ptz_move(val);

    if (!strcmp(cmd, "zoom")) {
        if (!strcmp(val, "tele") || !strcmp(val, "wide")) {
            const char *res = ptz_move(val);
            if (res && !strncmp(res, "moving ", 7)) return "zooming";
            return res;
        }
        if (!strcmp(val, "stop")) {
            return ptz_move("stop");
        }
        return NULL;
    }

    return NULL;                              // unknown command -> core falls back
}

void af_plugin_exit(void) {
    pthread_mutex_lock(&ptz_mu);
    if (ptz_client) {
        char error[160] = "";
        (void)motors_stop_all(ptz_client, error, sizeof(error));
        ptz_disconnect();
    }
    pthread_mutex_unlock(&ptz_mu);
    af_engine_stop();
}

// Start the magnification reader the moment the .so is loaded (the core dlopen's
// it in af_plugin_init, after config is loaded), so the OSD %@ token and /zoom
// (GET) have a value without waiting for the first AF. Torn down in
// af_plugin_exit / af_engine_stop before dlclose.
__attribute__((constructor)) static void af_plugin_load(void) {
    af_zoom_start();
}
