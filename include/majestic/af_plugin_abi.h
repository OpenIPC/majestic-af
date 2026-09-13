// ABI between majestic core (commercial) and the out-of-core autofocus plugin
// (majestic-af.so, OpenIPC OSS). This header is vendored BYTE-IDENTICAL in both
// repositories — it is the single frozen contract between them.
//
// Only C functions with scalar/pointer arguments cross this boundary; no structs.
// The engine's AfIO/AfParams never leave the plugin, so the ABI is immune to
// struct-layout drift between the firmware toolchain and the plugin toolchain.
//
// Direction of each symbol:
//   * af_plugin_* : DEFINED by the plugin, dlsym'd and called by the core.
//   * sdk_* / config_get_* : DEFINED by the core, exported via
//     cmake/dynamic-list.txt (built with WITH_PLUGINS_SUPPORT=ON), and resolved
//     by the plugin against the executable at dlopen.

#ifndef MAJESTIC_AF_PLUGIN_ABI_H
#define MAJESTIC_AF_PLUGIN_ABI_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ---- Plugin exports (core -> plugin) --------------------------------------

// One command per call. The two tokens mirror the legacy plugin_call shape so
// the same entry can be driven over the TCP:4000 command server unchanged.
//   cmd = "autofocus", val in { "run", "settle", "status" }
//       run     -> "started" | "restarted" | "busy"
//       settle  -> same, run only after the pipeline is quiet
//       status  -> "idle" | "running" | "done fv=... peak=... mag=... pos=..."
//   cmd = "ptz", val = "<verb>" | "<verb>:<ms>" | ""
//       verb in { up, down, left, right, tele, wide, near, far, stop,
//                 day, night }  -- day/night only where the protocol has them
//       ""        -> "actuator=... port=... speed=... pulse=... state=...
//                     verbs=..."  (the capability line)
//       <verb>    -> "moving <verb>" | "stopped" | "unavailable"
//       An unrecognised verb returns NULL, which the core answers as 400.
//       A move runs until <ms> (default isp.autofocus.pulse) elapses without
//       another command for it, so a held button repeats the same request and
//       a release sends "stop". The plugin stops the motor on that deadline,
//       which is what keeps a lost release from driving a lens into its stop.
//       A manual verb preempts a running autofocus pass; a manual FOCUS verb
//       additionally cancels the pass an earlier zoom booked, because the
//       operator has just set the focus by hand.
//   cmd = "zoom", val in { "tele", "wide", "stop" }
//       The original spelling of three of the ptz verbs, unchanged.
//       tele/wide -> "zooming" | "unavailable"
//       stop      -> "stopped"
// Returns a pointer to storage that stays valid until the next call (static,
// mutex-guarded inside the plugin); the caller must NOT free it. Returns NULL
// for an unrecognised command.
const char *af_plugin_call(const char *cmd, const char *val);

// Stop the motor, stop the worker, magnification-reader and motion threads,
// close the UART, and RETURN before the core dlclose()s the plugin. Threads
// must be joinable and joined here — a detached thread that outlives dlclose
// runs unmapped code. Idempotent; safe to call when nothing is running.
void af_plugin_exit(void);

// ---- Core HAL seams (plugin -> core) --------------------------------------

// The vendor focus statistic: one scalar, bigger = sharper. This is the single
// thing only the core can produce (it holds the vendor ISP handles). Returns
// false on a vendor with no AF statistic (the weak default), in which case the
// plugin's passes report "lens does not respond".
bool sdk_get_focus_value(unsigned *fv);

// Publish the lens magnification the plugin's UART reader parsed into the core's
// cache, which the OSD "%@" token and the /zoom (GET) handler read. Lets the
// board-specific "X<ratio>" parser live in the plugin while a single core cache
// survives plugin reloads.
void sdk_set_zoom_mag(float mag);

// Config accessors so the plugin reads its own isp.autofocus.* keys
// (actuator/port/speed/pulse) — the exact calls the in-core engine makes today.
const char *config_get_string(const char *path, const char *param_name);
int config_get_int(const char *path, const char *param_name);
bool config_get_boolean(const char *path, const char *param_name);

#ifdef __cplusplus
}
#endif

#endif // MAJESTIC_AF_PLUGIN_ABI_H
