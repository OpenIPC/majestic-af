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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
static bool ptz_zoom_dirty;
static char ptz_reply[160];

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
    snprintf(ptz_reply, sizeof(ptz_reply),
             "driver=%s axes=%s%s%s%s", caps.driver,
             caps.axes & MOTORS_AXIS_MASK(MOTORS_PAN) ? "pan " : "",
             caps.axes & MOTORS_AXIS_MASK(MOTORS_TILT) ? "tilt " : "",
             caps.axes & MOTORS_AXIS_MASK(MOTORS_ZOOM) ? "zoom " : "",
             caps.axes & MOTORS_AXIS_MASK(MOTORS_FOCUS) ? "focus" : "");
    pthread_mutex_unlock(&ptz_mu);
    return ptz_reply;
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
        int result = motors_stop_all(ptz_client, error, sizeof(error));
        if (result == 0) result = motors_release(ptz_client, error, sizeof(error));
        bool run_af = result == 0 && ptz_zoom_dirty;
        ptz_zoom_dirty = false;
        if (result != 0) ptz_disconnect();
        pthread_mutex_unlock(&ptz_mu);
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
    if (axis == MOTORS_FOCUS) af_note_manual_focus();
    else (void)af_cancel_pass();
    int result = motors_acquire(ptz_client, MOTORS_MANUAL, axis,
                                duration + 1000U, error, sizeof(error));
    if (result == 0)
        result = motors_move(ptz_client, axis, direction, duration,
                             error, sizeof(error));
    if (result == 0 && axis == MOTORS_ZOOM) ptz_zoom_dirty = true;
    if (result != 0) ptz_disconnect();
    pthread_mutex_unlock(&ptz_mu);
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
        if (!strcmp(val, "tele")) {
            return ptz_move("tele");
        }
        if (!strcmp(val, "wide")) {
            return ptz_move("wide");
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
