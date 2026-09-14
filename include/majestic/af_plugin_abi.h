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
//   cmd = "autofocus", val in { "run", "settle", "cancel", "status" }
//       run     -> "started" | "restarted" | "busy"
//       settle  -> same, run only after the pipeline is quiet
//       status  -> state text followed by "metric_fv=... t_mono_ms=..."
// The metric suffix contains a current ISP sample for read-only clients. The
// state text stays first so clients can continue to match idle, running, or done.
//   cmd = "zoom", val in { "tele", "wide", "stop" }
//       tele/wide -> "zooming" | "unavailable"
//       stop      -> "stopped"
//   cmd = "ptz", val = "" or "ACTION[:DURATION_MS]"
//       empty     -> driver name and available axes
//       action    -> left, right, up, down, tele, wide, near, far, or stop
// Returns a pointer to storage that stays valid until the next call (static,
// mutex-guarded inside the plugin); the caller must NOT free it. Returns NULL
// for an unrecognised command.
const char *af_plugin_call(const char *cmd, const char *val);

// Cancel AF work and join the worker and event-reader threads before the core
// unloads the plugin. A detached thread can run unmapped code after dlclose().
// This function is idempotent and is safe when no AF job is active.
void af_plugin_exit(void);

// ---- Core HAL seams (plugin -> core) --------------------------------------

// The vendor focus statistic: one scalar, bigger = sharper. This is the single
// thing only the core can produce (it holds the vendor ISP handles). Returns
// false on a vendor with no AF statistic (the weak default), in which case the
// plugin's passes report "lens does not respond".
bool sdk_get_focus_value(unsigned *fv);

// Publish zoom magnification from the selected motor driver into the core cache.
// The OSD "%@" token and the /zoom (GET) handler read this cache.
void sdk_set_zoom_mag(float mag);

// Configuration accessors for the plugin's isp.autofocus.* keys.
const char *config_get_string(const char *path, const char *param_name);
int config_get_int(const char *path, const char *param_name);
bool config_get_boolean(const char *path, const char *param_name);

#ifdef __cplusplus
}
#endif

#endif // MAJESTIC_AF_PLUGIN_ABI_H
