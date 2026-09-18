/* engine.c's state file and its two honesty rules, against stubbed HAL seams.
 *
 * engine.c had no host harness: it imports the six symbols majestic exports and
 * talks to motion.c, so nothing here was reachable without hardware. The seams
 * are narrow enough to stub, and what they buy is coverage of three things that
 * are otherwise only testable on a camera -- the focus position surviving a
 * restart, the status endpoint refusing to call a shut port "idle", and the
 * trigger refusing a pass the camera cannot perform.
 *
 * The state file and /proc/uptime are redirected at compile time (AF_ZOOM_STATE,
 * AF_UPTIME_PATH), the same way the offline model overrides af2's timing.
 */
#include <greatest.h>

#include <majestic/af.h>
#include <majestic/af2.h>
#include <majestic/log.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion.h"
#include "proto.h"

/* ---- the core's seams ---------------------------------------------------- */
bool g_af_enabled = true;
const char *config_get_string(const char *path, const char *name) {
    (void)path;
    if (!strcmp(name, "actuator")) return "pelco-xm";
    if (!strcmp(name, "port")) return "/dev/null";
    if (!strcmp(name, "mode")) return "semi";
    return NULL;
}
int config_get_int(const char *path, const char *name) {
    (void)path;
    if (!strcmp(name, "speed")) return 115200;
    if (!strcmp(name, "pulse")) return 500;
    return 0;
}
bool config_get_boolean(const char *path, const char *name) {
    (void)path; (void)name; return g_af_enabled;
}
bool sdk_get_focus_value(unsigned *fv) { (void)fv; return false; }
void sdk_set_zoom_mag(float m) { (void)m; }
/* engine.c announces a restored focus position and nothing else does, so the
 * log is the only observable for it -- there is no accessor, and adding one to
 * production code for a test's benefit would be the wrong trade. */
static char g_log[4096];
static void log_reset(void) { g_log[0] = 0; }
static int log_saw(const char *needle) { return strstr(g_log, needle) != NULL; }
void log_log(enum LogType l, const char *f, int ln, const char *fn,
             const char *fmt, ...) {
    (void)l; (void)f; (void)ln; (void)fn;
    char line[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(line, sizeof line, fmt, ap);
    va_end(ap);
    size_t used = strlen(g_log);
    snprintf(g_log + used, sizeof g_log - used, "%s\n", line);
}

/* ---- motion.c, stubbed: the port is a switch the tests flip -------------- */
int g_port_open = 0;
int g_wake_blobs = 0;
int motion_fd(void) { return g_port_open ? 7 : -1; }
bool motion_ready(void) {
    if (g_port_open) { af_reader_ensure(); return true; }
    return false;
}
bool motion_wake_blob(void) { g_wake_blobs++; return true; }
bool motion_start(void) { return g_port_open != 0; }
void motion_stop_watchdog(void) {}
void motion_close(void) { g_port_open = 0; }
bool motion_engine_drive(int d) { (void)d; return false; }
long motion_idle_ms(void) { return 10000; }
bool motion_manual_active(void) { return false; }
bool motion_move(enum PtzVerb v, int ms) { (void)v; (void)ms; return g_port_open != 0; }
bool motion_halt(void) { return g_port_open != 0; }
int motion_default_ms(void) { return 500; }
const char *motion_describe(char *b, size_t n) { snprintf(b, n, "stub"); return b; }

/* engine.c's own exports we drive directly */
void af_engine_stop(void);

static void write_uptime(double secs) {
    FILE *f = fopen(AF_UPTIME_PATH, "w");
    if (f) { fprintf(f, "%.2f 0.00\n", secs); fclose(f); }
}
static void rm_state(void) { remove(AF_ZOOM_STATE); }

/* A shut port is not idleness: focus and zoom share the one descriptor, so
 * while it is closed there is no autofocus and the endpoint must say so. */
TEST status_never_calls_a_shut_port_idle(void) {
    g_af_enabled = true;
    g_port_open = 0;
    ASSERT_STR_EQ("failed: focus port is not open", af_status());
    g_port_open = 1;
    ASSERT_STR_EQ("idle", af_status());
    PASS();
}

/* A pass the camera cannot perform is refused, not accepted and failed later. */
TEST trigger_refuses_when_there_is_no_motor(void) {
    g_af_enabled = true;
    g_port_open = 0;
    ASSERT_EQ(AF_TRIGGER_UNAVAILABLE, af_trigger(false));
    g_af_enabled = false;
    ASSERT_EQ(AF_TRIGGER_UNAVAILABLE, af_trigger(false));
    g_af_enabled = true;
    PASS();
}

/* The focus position survives a majestic restart, so the next pass TRACKs
 * instead of paying a ~42 s seek to the near stop. */
TEST focus_position_survives_a_restart(void) {
    rm_state();
    write_uptime(100000.0);          /* long up: a restart, not a fresh boot */
    g_port_open = 1;
    g_af_enabled = true;

    FILE *f = fopen(AF_ZOOM_STATE, "w");
    ASSERT(f != NULL);
    fprintf(f, "2.60\n18830 2.60\n");
    fclose(f);

    log_reset();
    af_engine_start();
    ASSERT_EQ_FMT(2.6f, af_zoom_mag(), "%.2f");
    ASSERTm("the focus position was not restored across a restart",
            log_saw("focus position restored at 18830"));
    af_engine_stop();
    PASS();
}

/* ...but NOT across a power cut, where the MCU exercises both motors before
 * Linux userspace exists. The magnification is still restored -- that half is
 * measured -- only the position is withheld. */
TEST fresh_boot_withholds_the_focus_position(void) {
    rm_state();
    write_uptime(12.0);              /* the plugin loads ~12 s into a boot */
    g_port_open = 1;

    FILE *f = fopen(AF_ZOOM_STATE, "w");
    ASSERT(f != NULL);
    fprintf(f, "2.60\n18830 2.60\n");
    fclose(f);

    log_reset();
    af_engine_start();
    ASSERT_EQ_FMT(2.6f, af_zoom_mag(), "%.2f");   /* zoom: still trusted */
    ASSERTm("a fresh boot must not trust the saved focus position",
            !log_saw("focus position restored"));
    af_engine_stop();
    PASS();
}

/* Line 1 stays the bare magnification, so a plugin that predates the second
 * line reads it with the same fscanf and loses only the focus position. */
TEST state_file_first_line_stays_backward_compatible(void) {
    rm_state();
    write_uptime(100000.0);
    g_port_open = 1;
    FILE *f = fopen(AF_ZOOM_STATE, "w");
    ASSERT(f != NULL);
    fprintf(f, "3.40\n9000 3.40\n");
    fclose(f);

    /* what an OLD plugin does with the new file */
    f = fopen(AF_ZOOM_STATE, "r");
    ASSERT(f != NULL);
    float v = 0.0f;
    int got = fscanf(f, "%f", &v);
    fclose(f);
    ASSERT_EQ(1, got);
    ASSERT_EQ_FMT(3.4f, v, "%.2f");
    PASS();
}

SUITE(engine_state_suite) {
    RUN_TEST(status_never_calls_a_shut_port_idle);
    RUN_TEST(trigger_refuses_when_there_is_no_motor);
    RUN_TEST(focus_position_survives_a_restart);
    RUN_TEST(fresh_boot_withholds_the_focus_position);
    RUN_TEST(state_file_first_line_stays_backward_compatible);
}
