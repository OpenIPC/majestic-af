// majestic-af — out-of-core autofocus and PTZ plugin for majestic (OpenIPC).
//
// The ABI boundary (include/majestic/af_plugin_abi.h): majestic dlsym's
// af_plugin_call / af_plugin_exit here and calls them from its /autofocus,
// /zoom and /ptz handlers; this plugin resolves the core HAL seams
// (sdk_get_focus_value, sdk_set_zoom_mag, config_get_*) against the executable
// at dlopen.
//
// Everything below the ABI lives in three files: proto.c builds the wire
// frames, motion.c owns the port and is the only thing that writes to it, and
// engine.c + af2.c run the contrast search through motion.c's actuator hook.
// This file is only the adapter from the two-token command ABI to those.

#include <majestic/af.h>
#include <majestic/af_plugin_abi.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion.h"
#include "proto.h"

// engine.c: synchronous teardown — joins the worker + reader and closes the
// port before we return.
void af_engine_stop(void);

static const char *map_trigger(int r) {
    // af_trigger: 0 started, 2 preempted-and-rearmed, else busy (-1 shouldn't
    // happen — the plugin only loaded because autofocus is enabled).
    return r == 0 ? "started" : r == 2 ? "restarted" : "busy";
}

// Answers that outlive the call. The ABI promises the returned pointer stays
// valid until the next call; these two buffers are only ever written from the
// calling thread (the core's HTTP loop), which is the same thread that reads
// them back out.
static char ptz_reply[64];
static char ptz_caps[192];

// "<verb>" or "<verb>:<ms>". The verb is matched against the closed list in
// proto.c, so no request token ever reaches the wire unvalidated; a duration
// outside the accepted range falls back to the configured default rather than
// failing the move.
static const char *do_ptz(const char *val) {
    if (!*val) {
        return motion_describe(ptz_caps, sizeof ptz_caps);
    }

    char name[16];
    long ms = 0;
    const char *colon = strchr(val, ':');
    size_t n = colon ? (size_t)(colon - val) : strlen(val);
    if (n == 0 || n >= sizeof name) {
        return NULL;
    }
    memcpy(name, val, n);
    name[n] = 0;
    if (colon) {
        // strtol with the end checked, not atoi: "500junk" must not become a
        // 500 ms move and "abc" must not quietly become the default. A
        // duration that is not a plain number is a bad request, and the core
        // turns a NULL from here into a 400 rather than putting a frame on the
        // wire with a length nobody asked for.
        char *end = NULL;
        errno = 0;
        ms = strtol(colon + 1, &end, 10);
        if (errno || !end || end == colon + 1 || *end || ms < 0 || ms > 100000) {
            return NULL;
        }
    }

    enum PtzVerb v;
    if (!ptz_verb_parse(name, &v)) {
        return NULL;   // the core turns this into a 400
    }
    if (v == PTZ_STOP) {
        return af_ptz_move(PTZ_STOP, 0) ? "stopped" : "unavailable";
    }
    if (!af_ptz_move(v, (int)ms)) {
        return "unavailable";
    }
    snprintf(ptz_reply, sizeof ptz_reply, "moving %s", ptz_verb_name(v));
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
        return NULL;
    }

    if (!strcmp(cmd, "ptz")) {
        return do_ptz(val);
    }

    // The original spelling of two of the ptz verbs, kept because majestic's
    // /zoom endpoint and anything driving the TCP command server still say it.
    if (!strcmp(cmd, "zoom")) {
        if (!strcmp(val, "tele")) {
            return af_ptz_move(PTZ_TELE, 0) ? "zooming" : "unavailable";
        }
        if (!strcmp(val, "wide")) {
            return af_ptz_move(PTZ_WIDE, 0) ? "zooming" : "unavailable";
        }
        if (!strcmp(val, "stop")) {
            // This used to be a lie: it answered "stopped" without writing a
            // single byte, because the pulses it was stopping ended by
            // themselves. A held button now runs the motor until told
            // otherwise, so the stop has to reach the wire.
            return af_ptz_move(PTZ_STOP, 0) ? "stopped" : "unavailable";
        }
        return NULL;
    }

    return NULL;                              // unknown command -> core reports it
}

void af_plugin_exit(void) {
    af_engine_stop();
}

// Open the port and start the magnification reader the moment the .so is
// loaded (the core dlopen's it in af_plugin_init, after config is loaded), so
// the OSD %@ token and /zoom (GET) have a value without waiting for the first
// AF, and the first pad press does not pay for the open. Torn down in
// af_plugin_exit / af_engine_stop before dlclose.
__attribute__((constructor)) static void af_plugin_load(void) {
    af_engine_start();
}
