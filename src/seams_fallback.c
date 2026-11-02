// The core's HAL seams, for a majestic that does not export them.
//
// The plugin leaves sdk_get_focus_value, sdk_set_zoom_mag, config_get_* and
// log_log undefined and resolves them against the majestic executable at
// dlopen (af_plugin_abi.h). A majestic built without WITH_PLUGINS_SUPPORT
// exports none of them -- the infinity6 (SSC325) lite build's dynamic symbol
// table holds a few libc functions and the stack protector, nothing else --
// and still loads /usr/lib/majestic-af.so, which RTLD_NOW then refuses on the
// first seam it cannot find ("sdk_set_zoom_mag: symbol not found"). /ptz
// answers that there is no motor driver, and a pan/tilt head that works
// with this plugin cannot be driven.
//
// These stand in for the core's there, and only there. They are ordinary
// default-visibility definitions in a .so linked without -Bsymbolic, so the
// plugin's own calls to them still go through the dynamic linker, which
// searches the executable before the plugin: a majestic that exports a seam
// keeps it, and nothing changes on its cameras. tests/seams_core_test.c pins
// that against the real .so. Keep it so: hidden visibility, -Bsymbolic, or
// an LTO that binds these calls inside the .so would quietly route every
// camera to the definitions below.
//
// - config_get_*: the section of /etc/majestic.yaml named by `path`, read
//   from the file on every call, so a saved setting counts from the next call
//   on. A missing key is unset: NULL, false, and -1 for an int, which the
//   plugin's range checks take for their defaults.
// - log_log: syslog.
// - sdk_get_focus_value: false, a core with no AF statistic, so a pass reports
//   that the lens does not respond. sdk_set_zoom_mag: nothing; there is no
//   core cache to publish to.
// The weakly referenced seams (sdk_set_zoom_mag_restored, sdk_ptz_motion) stay
// undefined: the plugin already runs without them.

#include <majestic/af_plugin_abi.h>
#include <majestic/log.h>

#include <errno.h>
#include <limits.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <syslog.h>

// Overridden by the tests, the way engine.c's state file is.
#ifndef MAJESTIC_YAML
#define MAJESTIC_YAML "/etc/majestic.yaml"
#endif

// Longer than any line majestic writes. A longer one is skipped whole.
#define CFG_LINE 512

static char *rtrim(char *s) {
    char *e = s + strlen(s);
    while (e > s && (e[-1] == ' ' || e[-1] == '\t' || e[-1] == '\n' || e[-1] == '\r')) {
        *--e = '\0';
    }
    return s;
}

// The value after "key:", or NULL where there is none ("key:" alone opens a
// map). A quoted scalar loses its quotes (no escapes are read), and a plain
// one ends at a comment, '#' after a blank.
static const char *scalar(char *s) {
    while (*s == ' ' || *s == '\t') {
        s++;
    }
    if (*s == '"' || *s == '\'') {
        char *end = strchr(s + 1, *s);
        if (!end) {
            return NULL;
        }
        *end = '\0';
        return s + 1;
    }
    for (char *p = s; *p; p++) {
        if (*p == '#' && (p == s || p[-1] == ' ' || p[-1] == '\t')) {
            *p = '\0';
            break;
        }
    }
    rtrim(s);
    return *s ? s : NULL;
}

// <path>.<name> from the file, copied into out. majestic writes it two levels
// deep, "section:" at the margin and "  key: value" under it; keys nested
// deeper than the section's own are not its keys and are skipped.
static bool cfg_find(const char *path, const char *name, char *out, size_t n) {
    if (!path || !name) {
        return false;
    }
    FILE *f = fopen(MAJESTIC_YAML, "r");
    if (!f) {
        return false;
    }
    char line[CFG_LINE];
    bool in = false, whole = true, found = false;
    int indent = -1;   // the column of the section's own keys
    while (!found && fgets(line, sizeof line, f)) {
        size_t len = strlen(line);
        bool starts = whole;
        whole = len && line[len - 1] == '\n';
        // Neither the head of an over-long line nor its tail, which would
        // otherwise read as a line of its own at the margin.
        if (!starts || (!whole && !feof(f))) {
            continue;
        }
        char *s = line;
        while (*s == ' ' || *s == '\t') {
            s++;
        }
        if (*s == '\0' || *s == '\n' || *s == '\r' || *s == '#') {
            continue;
        }
        char *colon = strchr(s, ':');
        if (s == line) {
            if (colon) {
                *colon = '\0';
            }
            in = colon && !strcmp(rtrim(s), path);
            indent = -1;
            continue;
        }
        if (!in || !colon) {
            continue;
        }
        if (indent < 0) {
            indent = (int)(s - line);
        }
        if (s - line != indent) {
            continue;
        }
        *colon = '\0';
        if (strcmp(rtrim(s), name)) {
            continue;
        }
        const char *v = scalar(colon + 1);
        if (v) {
            snprintf(out, n, "%s", v);
            found = true;
        }
    }
    fclose(f);
    return found;
}

// config_get_string hands out a pointer its callers use without copying,
// while another thread may be reading the file again, and nothing tells this
// file when they are done with it. So a value is never freed: each distinct
// one is stored once, and reading it again, after any number of saves, hands
// back the same pointer.
typedef struct Interned {
    struct Interned *next;
    char s[];
} Interned;

static pthread_mutex_t cfg_mu = PTHREAD_MUTEX_INITIALIZER;
static Interned *cfg_values;

static const char *intern(const char *s) {
    pthread_mutex_lock(&cfg_mu);
    Interned *i;
    for (i = cfg_values; i; i = i->next) {
        if (!strcmp(i->s, s)) {
            break;
        }
    }
    if (!i) {
        size_t n = strlen(s) + 1;
        i = malloc(sizeof *i + n);
        if (i) {
            memcpy(i->s, s, n);
            i->next = cfg_values;
            cfg_values = i;
        }
    }
    pthread_mutex_unlock(&cfg_mu);
    return i ? i->s : NULL;
}

const char *config_get_string(const char *path, const char *param_name) {
    char v[CFG_LINE];
    return cfg_find(path, param_name, v, sizeof v) ? intern(v) : NULL;
}

int config_get_int(const char *path, const char *param_name) {
    char v[CFG_LINE], *end;
    if (!cfg_find(path, param_name, v, sizeof v)) {
        return -1;
    }
    errno = 0;
    long n = strtol(v, &end, 10);
    if (end == v || *end || errno || n < INT_MIN || n > INT_MAX) {
        return -1;
    }
    return (int)n;
}

bool config_get_boolean(const char *path, const char *param_name) {
    char v[CFG_LINE];
    if (!cfg_find(path, param_name, v, sizeof v)) {
        return false;
    }
    return !strcasecmp(v, "true") || !strcasecmp(v, "yes") || !strcasecmp(v, "on");
}

void log_log(enum LogType level, const char *file, int line, const char *func,
             const char *fmt, ...) {
    // log.h's levels in order; L_TRACE exists only in a build without NDEBUG.
    static const int prio[] = {
#ifndef NDEBUG
        LOG_DEBUG,
#endif
        LOG_DEBUG, LOG_DEBUG, LOG_INFO, LOG_WARNING, LOG_ERR, LOG_CRIT,
    };
    int l = (int)level;
    char msg[256];
    va_list ap;

    (void)line;
    (void)func;
    va_start(ap, fmt);
    vsnprintf(msg, sizeof msg, fmt, ap);
    va_end(ap);
    syslog(prio[l >= 0 && l < (int)(sizeof prio / sizeof *prio) ? l : L_ERROR],
           "%s: %s", file ? file : "majestic-af", msg);
}

bool sdk_get_focus_value(unsigned *fv) {
    (void)fv;
    return false;
}

void sdk_set_zoom_mag(float mag) {
    (void)mag;
}
