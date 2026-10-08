// motion.c's report of motion to the core (sdk_ptz_motion), for a backend that
// cannot see its motor and leaves the report to the verbs.
//
// Links the real src/motion.c against a fake actuator and stubs for the
// engine and the core's seams, and records every sdk_ptz_motion call. The
// watchdog thread runs for real, so a move that ends on its deadline ends the
// way it does on a camera.

#include <greatest.h>

#include <pthread.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "actuator.h"
#include "motion.h"
#include <majestic/af.h>
#include <majestic/af_plugin_abi.h>
#include <majestic/log.h>

// ---- what sdk_ptz_motion was told ---------------------------------------

static pthread_mutex_t rep_mu = PTHREAD_MUTEX_INITIALIZER;
static struct { int moving, axis; long upper_ms; } rep[16];
static int rep_n;

void sdk_ptz_motion(int moving, int axis, long upper_ms) {
    pthread_mutex_lock(&rep_mu);
    if (rep_n < 16) {
        rep[rep_n].moving = moving;
        rep[rep_n].axis = axis;
        rep[rep_n].upper_ms = upper_ms;
        rep_n++;
    }
    pthread_mutex_unlock(&rep_mu);
}

static int reports(void) {
    pthread_mutex_lock(&rep_mu);
    int n = rep_n;
    pthread_mutex_unlock(&rep_mu);
    return n;
}

// ---- a fake motor --------------------------------------------------------

static bool fake_open(void) { return true; }
static void fake_close(void) {}
static int last_speed = -1;
static bool fake_emit(enum PtzVerb v, int speed) {
    (void)v;
    last_speed = speed;
    return true;
}
static bool fake_has(enum PtzVerb v) { (void)v; return true; }

static Actuator fake = {
    .name = "fake",
    .open = fake_open,
    .close = fake_close,
    .emit = fake_emit,
    .has = fake_has,
};

const Actuator *actuator_select(const char *name) { (void)name; return &fake; }

// ---- the engine and the core, stubbed ------------------------------------

float af_zoom_mag(void) { return 0.0f; }
void af_reader_ensure(void) {}
void af_preempt(void) {}
void af_note_manual_focus(void) {}
unsigned af_focus_gen(void) { return 0; }
void af_book_after_zoom(long at_ms, unsigned gen) { (void)at_ms; (void)gen; }
bool af_book_tick(long now) { (void)now; return false; }
bool af_alive(void) { return false; }
static const char *port_name = "fake";
const char *config_get_string(const char *path, const char *param) {
    (void)path;
    return param && !strcmp(param, "port") ? port_name : "fake";
}
int config_get_int(const char *path, const char *param) { (void)path; (void)param; return 0; }
bool config_get_boolean(const char *path, const char *param) { (void)path; (void)param; return false; }
void log_log(enum LogType level, const char *file, int line, const char *func, const char *fmt, ...) {
    (void)level; (void)file; (void)line; (void)func; (void)fmt;
}

static void sleep_ms(long ms) {
    struct timespec ts = {ms / 1000, (ms % 1000) * 1000000L};
    nanosleep(&ts, NULL);
}

static void start(bool reports_motion) {
    fake.reports_motion = reports_motion;
    fake.honours_speed = false;
    last_speed = -1;
    pthread_mutex_lock(&rep_mu);
    rep_n = 0;
    pthread_mutex_unlock(&rep_mu);
    motion_reset();
    motion_start();
}

static void stop(void) {
    motion_stop_watchdog();
    motion_close();
}

TEST a_move_is_reported_once_and_its_deadline_ends_it(void) {
    start(false);
    ASSERT(motion_move(PTZ_LEFT, 200));
    ASSERT(motion_move(PTZ_LEFT, 200));   // the pad's repeat: still one move
    ASSERT_EQ(1, reports());
    ASSERT_EQ(1, rep[0].moving);
    ASSERT_EQ(0, rep[0].axis);
    ASSERT(rep[0].upper_ms > 0 && rep[0].upper_ms <= 200);
    sleep_ms(400);   // past the deadline: the watchdog ends it
    ASSERT_EQ(2, reports());
    ASSERT_EQ(0, rep[1].moving);
    stop();
    PASS();
}

TEST a_stop_ends_it_and_a_new_axis_is_one_move(void) {
    start(false);
    ASSERT(motion_move(PTZ_UP, 1000));
    ASSERT_EQ(1, rep[0].axis);
    ASSERT(motion_move(PTZ_RIGHT, 1000));   // tilt into pan without a stop
    ASSERT_EQ(1, reports());
    ASSERT(motion_halt());
    ASSERT_EQ(2, reports());
    ASSERT_EQ(0, rep[1].moving);
    stop();
    PASS();
}

TEST a_backend_that_reports_itself_is_left_to_it(void) {
    start(true);
    ASSERT(motion_move(PTZ_LEFT, 200));
    ASSERT(motion_halt());
    ASSERT_EQ(0, reports());
    stop();
    PASS();
}

// A speed reaches an actuator that keeps it, as 1..63, and only that one; the
// capability line offers speeds only then, so a core never asks a motor that
// would ignore it.
TEST a_speed_reaches_only_a_motor_that_keeps_it(void) {
    char caps[192];
    start(true);
    ASSERT(motion_move_at(PTZ_LEFT, 500, 50));
    ASSERT_EQ(0, last_speed);
    ASSERT_EQ(NULL, strstr(motion_describe(caps, sizeof caps), "speeds="));
    fake.honours_speed = true;
    ASSERT(strstr(motion_describe(caps, sizeof caps), " speeds=1-100"));
    ASSERT(motion_move_at(PTZ_LEFT, 500, 50));
    ASSERT_EQ(32, last_speed);
    ASSERT(motion_move_at(PTZ_LEFT, 500, 100));
    ASSERT_EQ(63, last_speed);
    ASSERT(motion_move_at(PTZ_LEFT, 500, 1));
    ASSERT_EQ(1, last_speed);              // 1% is still a move
    ASSERT(motion_move(PTZ_LEFT, 500));
    ASSERT_EQ(0, last_speed);              // none asked for: the configured rate
    ASSERT(motion_halt());
    stop();
    PASS();
}

// The serial lenses' line carries the port path the operator typed, and a
// long one must not push " speeds=" off the end of the plugin's buffer: the
// core reads the token to decide whether a speed may go out at all.
TEST a_long_port_path_keeps_the_speed_offer(void) {
    static char longport[201];
    memset(longport, 'p', 200);
    longport[0] = '/';
    port_name = longport;
    char caps[PTZ_CAPS_MAX];
    start(true);
    fake.name = "pelco";
    fake.honours_speed = true;
    motion_describe(caps, sizeof caps);
    ASSERT(strstr(caps, " speeds=1-100"));
    ASSERT(strstr(caps, " verbs="));
    fake.name = "fake";
    port_name = "fake";
    stop();
    PASS();
}

SUITE(motion_report_suite) {
    RUN_TEST(a_move_is_reported_once_and_its_deadline_ends_it);
    RUN_TEST(a_stop_ends_it_and_a_new_axis_is_one_move);
    RUN_TEST(a_backend_that_reports_itself_is_left_to_it);
    RUN_TEST(a_speed_reaches_only_a_motor_that_keeps_it);
    RUN_TEST(a_long_port_path_keeps_the_speed_offer);
}

GREATEST_MAIN_DEFS();

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(motion_report_suite);
    GREATEST_MAIN_END();
}
