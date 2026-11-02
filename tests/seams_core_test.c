// The real majestic-af.so, dlopen()ed with RTLD_NOW the way majestic loads it,
// by a stand-in for the core. Built twice (CMakeLists.txt):
//
// - CORE_EXPORTS_SEAMS: this core defines every seam and exports it, as a
//   majestic built WITH_PLUGINS_SUPPORT does. The plugin carries definitions of
//   the same names (src/seams_fallback.c), and its calls must still land here:
//   that is what keeps the fallback from changing anything on such a camera.
// - without: this core exports none, as the infinity6 lite build. The plugin
//   must load anyway; before the fallback, RTLD_NOW refused it.
//
// The plugin is loaded once for the whole run: musl's dlclose never unmaps, so
// a second dlopen in the process would neither run the constructor again nor
// start from a fresh engine.
//
// Nothing here may touch hardware. The exporting core says ptz.enabled is false,
// so the constructor starts nothing, and names the gpiostep actuator, whose
// /dev/motorDev no host has, so the capability line's open fails and is logged.
// The bare core only asks the plugin for its schema, which reads no config and
// does not touch the engine (with no seams exported, the constructor reads
// /etc/majestic.yaml, which a host does not have).

#define _GNU_SOURCE
#include <greatest.h>

#include <dlfcn.h>
#include <stdio.h>
#include <string.h>

#include <majestic/af_plugin_abi.h>
#include <majestic/log.h>

static const char *const SEAMS[] = {
    "config_get_string", "config_get_int", "config_get_boolean",
    "log_log", "sdk_get_focus_value", "sdk_set_zoom_mag",
};

#ifdef CORE_EXPORTS_SEAMS
// ---- the core's seams, and what the plugin asked of them ----------------

static int asked_string, asked_int, asked_enabled, logged;

const char *config_get_string(const char *path, const char *param_name) {
    asked_string++;
    return !strcmp(path, "ptz") && !strcmp(param_name, "actuator") ? "gpiostep" : NULL;
}

int config_get_int(const char *path, const char *param_name) {
    asked_int++;
    return !strcmp(path, "ptz") && !strcmp(param_name, "pulse") ? 432 : -1;
}

bool config_get_boolean(const char *path, const char *param_name) {
    if (!strcmp(path, "ptz") && !strcmp(param_name, "enabled")) {
        asked_enabled++;
    }
    return false;
}

void log_log(enum LogType level, const char *file, int line, const char *func,
             const char *fmt, ...) {
    (void)level; (void)file; (void)line; (void)func; (void)fmt;
    logged++;
}

bool sdk_get_focus_value(unsigned *fv) {
    *fv = 0;
    return false;
}

void sdk_set_zoom_mag(float mag) {
    (void)mag;
}

static void *core_seam(const char *name) {
    if (!strcmp(name, "config_get_string")) return (void *)config_get_string;
    if (!strcmp(name, "config_get_int")) return (void *)config_get_int;
    if (!strcmp(name, "config_get_boolean")) return (void *)config_get_boolean;
    if (!strcmp(name, "log_log")) return (void *)log_log;
    if (!strcmp(name, "sdk_get_focus_value")) return (void *)sdk_get_focus_value;
    return (void *)sdk_set_zoom_mag;
}
#endif

typedef const char *(*PluginCall)(const char *, const char *);
typedef void (*PluginExit)(void);

static void *plugin;
static char load_error[512];

TEST the_plugin_loads(void) {
    ASSERTm(load_error, plugin);
    PASS();
}

// Each seam is defined in the plugin itself, so it loads without the core's,
// and answers its schema there.
TEST the_plugin_carries_every_seam(void) {
    ASSERT(plugin);
    for (size_t i = 0; i < sizeof SEAMS / sizeof *SEAMS; i++) {
        void *own = dlsym(plugin, SEAMS[i]);
        Dl_info where;
        ASSERTm(SEAMS[i], own && dladdr(own, &where) && where.dli_fname &&
                              strstr(where.dli_fname, "majestic-af.so"));
#ifdef CORE_EXPORTS_SEAMS
        ASSERTm(SEAMS[i], own != core_seam(SEAMS[i]));
#endif
    }
    PluginCall call = (PluginCall)dlsym(plugin, "af_plugin_call");
    ASSERT(call);
    const char *schema = call("schema", "");
    ASSERT(schema && strstr(schema, "\"gpiostep\""));
    PASS();
}

#ifdef CORE_EXPORTS_SEAMS
// The plugin's calls reach the core's seams, not its own fallbacks: the
// constructor asked the core whether ptz is enabled, and the capability line
// carries the core's actuator and pulse and logs the failed open through it.
TEST an_exporting_core_keeps_its_seams(void) {
    ASSERT(plugin);
    ASSERT_EQ(1, asked_enabled);
    asked_string = asked_int = logged = 0;
    PluginCall call = (PluginCall)dlsym(plugin, "af_plugin_call");
    ASSERT(call);
    const char *caps = call("ptz", "");
    ASSERT(caps);
    ASSERTm(caps, strstr(caps, "actuator=gpiostep"));
    ASSERTm(caps, strstr(caps, "pulse=432"));
    ASSERTm(caps, strstr(caps, "state=closed"));
    ASSERT(asked_string > 0);
    ASSERT(asked_int > 0);
    ASSERT(logged > 0);
    PASS();
}
#endif

SUITE(seams_core_suite) {
    plugin = dlopen(MAJESTIC_AF_SO, RTLD_NOW | RTLD_LOCAL);
    if (!plugin) {
        snprintf(load_error, sizeof load_error, "%s", dlerror());
    }
    RUN_TEST(the_plugin_loads);
    RUN_TEST(the_plugin_carries_every_seam);
#ifdef CORE_EXPORTS_SEAMS
    RUN_TEST(an_exporting_core_keeps_its_seams);
#endif
    if (plugin) {
        PluginExit stop = (PluginExit)dlsym(plugin, "af_plugin_exit");
        if (stop) {
            stop();
        }
        dlclose(plugin);
    }
}

GREATEST_MAIN_DEFS();

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(seams_core_suite);
    GREATEST_MAIN_END();
}
