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
// engine.c: bring the motor back if a reload left this .so mapped but its
// constructor un-re-run (see af_engine_ensure). A no-op while the engine is up.
void af_engine_ensure(void);

static const char *map_trigger(int r) {
    // af_trigger: started, preempted-and-rearmed, refused because there is no
    // motor to drive, or could-not-start-one-now. `unavailable` is the same word
    // the ptz and zoom verbs answer in that state, and the WebUI already words it.
    switch (r) {
    case AF_TRIGGER_STARTED:     return "started";
    case AF_TRIGGER_RESTARTED:   return "restarted";
    case AF_TRIGGER_UNAVAILABLE: return "unavailable";
    default:                     return "busy";
    }
}

// Answers that outlive the call. The ABI promises the returned pointer stays
// valid until the next call; these two buffers are only ever written from the
// calling thread (the core's HTTP loop), which is the same thread that reads
// them back out.
static char ptz_reply[64];
static char ptz_caps[PTZ_CAPS_MAX];

// "<verb>[:<ms>[:<speed>]]" (ptz_command_parse). The verb is matched against
// the closed list in proto.c, so no request token ever reaches the wire
// unvalidated, and a field that is not a plain number in range makes the whole
// command a bad request: the core turns a NULL from here into a 400 rather than
// putting a frame on the wire with a length or a speed nobody asked for. A
// duration of 0 falls back to the configured default. The speed field is sent
// only by a core that has read `speeds=` on the capability line.
static const char *do_ptz(const char *val) {
    if (!*val) {
        return motion_describe(ptz_caps, sizeof ptz_caps);
    }

    PtzCommand c;
    if (!ptz_command_parse(val, &c)) {
        return NULL;
    }
    enum PtzVerb v = c.verb;
    if (v == PTZ_STOP) {
        return af_ptz_move(PTZ_STOP, 0) ? "stopped" : "unavailable";
    }
    if (!af_ptz_move_at(v, (int)c.ms, c.speed)) {
        return "unavailable";
    }
    snprintf(ptz_reply, sizeof ptz_reply, "moving %s", ptz_verb_name(v));
    return ptz_reply;
}

const char *af_plugin_call(const char *cmd, const char *val) {
    if (!cmd || !val) {
        return NULL;
    }

    // A pipeline rebuild reloads this plugin without unmapping it, so the
    // constructor that starts the engine does not re-run. Revive it on the first
    // call after such a reload -- the pad's /autofocus/status poll or any verb --
    // so the motor does not stay dead until a restart. No-op while it is up.
    af_engine_ensure();

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
