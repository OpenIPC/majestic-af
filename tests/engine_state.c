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
#include <unistd.h>

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
int g_fv_ok = 0;
unsigned g_fv = 0;
bool sdk_get_focus_value(unsigned *fv) {
    if (!g_fv_ok) return false;
    *fv = g_fv;
    return true;
}
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
static int g_pipe[2] = {-1, -1};
int motion_fd(void) {
    if (!g_port_open) return -1;
    if (g_pipe[0] < 0 && pipe(g_pipe) != 0) return -1;
    return g_pipe[0];   /* never written: poll() times out, as on an idle UART */
}
bool motion_ready(void) { return g_port_open != 0; }
bool motion_wake_blob(void) { g_wake_blobs++; return true; }
bool motion_start(void) { return g_port_open != 0; }
void motion_stop_watchdog(void) {}
void motion_reset(void) {}
void motion_close(void) { g_port_open = 0; }
bool motion_engine_drive(int d) { (void)d; return g_port_open != 0; }
long motion_idle_ms(void) { return 10000; }
bool motion_manual_active(void) { return false; }
bool motion_move(enum PtzVerb v, int ms) { (void)v; (void)ms; return g_port_open != 0; }
bool motion_halt(void) { return g_port_open != 0; }
int motion_default_ms(void) { return 500; }
const char *motion_describe(char *b, size_t n) { snprintf(b, n, "stub"); return b; }

/* engine.c's own exports we drive directly */
void af_engine_stop(void);

/* The kernel's per-boot UUID, redirected into the build tree. Writing a
 * DIFFERENT one is how a test says "the camera has been power-cycled since". */
static void set_boot_id(const char *id) {
    FILE *f = fopen(AF_BOOTID_PATH, "w");
    if (f) { fprintf(f, "%s\n", id); fclose(f); }
}
static const char *BOOT_A = "061de083-cf77-4ff0-9ae1-25eef7d7bec2";
static const char *BOOT_B = "9f2c41aa-0e15-42d7-badc-71b0a4c0f0e9";
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
    set_boot_id(BOOT_A);
    g_port_open = 1;
    g_af_enabled = true;

    FILE *f = fopen(AF_ZOOM_STATE, "w");
    ASSERT(f != NULL);
    fprintf(f, "2.60\n18830 2.60 %s\n", BOOT_A);   /* same boot */
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
 * measured -- only the position is withheld.
 *
 * Note there is no uptime here. The plugin is loaded whenever the lens setting
 * turns on or the pipeline is rebuilt, which is routinely long after boot, so a
 * camera that power-cycled and was enabled an hour later has a large uptime and
 * a stale position -- the case an uptime threshold gets wrong and the boot id
 * gets right. */
TEST fresh_boot_withholds_the_focus_position(void) {
    rm_state();
    set_boot_id(BOOT_B);             /* the camera has rebooted since */
    g_port_open = 1;

    FILE *f = fopen(AF_ZOOM_STATE, "w");
    ASSERT(f != NULL);
    fprintf(f, "2.60\n18830 2.60 %s\n", BOOT_A);
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
    set_boot_id(BOOT_A);
    g_port_open = 1;
    FILE *f = fopen(AF_ZOOM_STATE, "w");
    ASSERT(f != NULL);
    fprintf(f, "3.40\n9000 3.40 %s\n", BOOT_A);
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

/* A second line written before the boot id existed carries no way to tell which
 * boot it belongs to, so it is not trusted -- one cold pass, not a bad one. */
TEST a_position_without_a_boot_id_is_not_trusted(void) {
    rm_state();
    set_boot_id(BOOT_A);
    g_port_open = 1;
    FILE *f = fopen(AF_ZOOM_STATE, "w");
    ASSERT(f != NULL);
    fprintf(f, "2.60\n18830 2.60\n");          /* no third field */
    fclose(f);

    log_reset();
    af_engine_start();
    ASSERT_EQ_FMT(2.6f, af_zoom_mag(), "%.2f");
    ASSERTm("a position with no boot id must not be restored",
            !log_saw("focus position restored"));
    af_engine_stop();
    PASS();
}

/* A manual focus must invalidate the SAVED position too, not just the one in
 * memory. After a boot that withheld the saved position the in-memory pair is
 * already (-1,-1), so an invalidation that only fires on a change leaves the
 * old line on disk -- eligible for a restore later in the same boot. */
TEST manual_focus_clears_the_saved_position(void) {
    rm_state();
    set_boot_id(BOOT_A);
    g_port_open = 1;
    FILE *f = fopen(AF_ZOOM_STATE, "w");
    ASSERT(f != NULL);
    fprintf(f, "2.60\n18830 2.60 %s\n", BOOT_A);
    fclose(f);

    af_engine_start();          /* restores 18830, and starts the reader */
    af_note_manual_focus();     /* the operator moved the element by hand */
    sleep(2);                   /* the reader's idle tick writes the file out */
    af_engine_stop();

    long pos = 0; float mag = 0.0f; float v = 0.0f;
    f = fopen(AF_ZOOM_STATE, "r");
    ASSERT(f != NULL);
    fscanf(f, "%f", &v);
    int got = fscanf(f, "%ld %f", &pos, &mag);
    fclose(f);
    ASSERT_EQm("the saved position must be invalidated by a manual focus", 2, got);
    ASSERTm("a manual focus left the old position on disk", pos < 0);
    PASS();
}

/* A pass that MOVED the lens and then failed must not leave the old position
 * standing -- in memory or on disk. af2_run has already driven the motor by the
 * time the contrast verdict is read, so the pair we were carrying describes a
 * place the lens has left; another same-zoom pass, or a restart, would start
 * from it. An unknown position costs one cold re-home, which is the right answer
 * after a pass that could not measure anything. */
TEST a_failed_pass_clears_the_saved_position(void) {
    rm_state();
    set_boot_id(BOOT_A);
    g_port_open = 1;
    g_af_enabled = true;
    FILE *f = fopen(AF_ZOOM_STATE, "w");
    ASSERT(f != NULL);
    fprintf(f, "2.60\n18830 2.60 %s\n", BOOT_A);
    fclose(f);

    af_engine_start();
    g_fv_ok = 1;
    g_fv = 0;                       /* the lens never answers: peak stays 0 */
    ASSERT_EQ(AF_TRIGGER_STARTED, af_trigger(false));
    for (int i = 0; i < 600 && !strncmp(af_status(), "running", 7); i++) {
        usleep(20000);
    }
    if (strncmp(af_status(), "failed:", 7)) {
        static char m[160];
        snprintf(m, sizeof m, "expected a failure, got: %s", af_status());
        FAILm(m);
    }
    sleep(2);                       /* the reader's idle tick writes it out */
    af_engine_stop();
    g_fv_ok = 0;

    long pos = 0; float mag = 0.0f; float v = 0.0f;
    f = fopen(AF_ZOOM_STATE, "r");
    ASSERT(f != NULL);
    fscanf(f, "%f", &v);
    int got = fscanf(f, "%ld %f", &pos, &mag);
    fclose(f);
    ASSERT_EQ(2, got);
    ASSERTm("a failed pass left the superseded position on disk", pos < 0);
    PASS();
}

SUITE(engine_state_suite) {
    RUN_TEST(status_never_calls_a_shut_port_idle);
    RUN_TEST(trigger_refuses_when_there_is_no_motor);
    RUN_TEST(focus_position_survives_a_restart);
    RUN_TEST(fresh_boot_withholds_the_focus_position);
    RUN_TEST(state_file_first_line_stays_backward_compatible);
    RUN_TEST(a_position_without_a_boot_id_is_not_trusted);
    RUN_TEST(manual_focus_clears_the_saved_position);
    RUN_TEST(a_failed_pass_clears_the_saved_position);
}
