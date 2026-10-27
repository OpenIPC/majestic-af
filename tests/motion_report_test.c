// motion.c's report of motion to the core (sdk_ptz_motion), for a backend that
// cannot see its motor and leaves the report to the verbs.
//
// Links the real src/motion.c against a fake actuator and stubs for the
// engine and the core's seams, and records every sdk_ptz_motion call. The
// watchdog thread runs for real, so a move that ends on its deadline ends the
// way it does on a camera.

#include <greatest.h>

#include <limits.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
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

// What reached each motor, in order: 'L' the lens (fake), 'H' the head; a
// close is logged as PTZ_VERB_COUNT.
static struct { char who; int v; } ev[64];
static int ev_n;
static void note(char who, int v) {
    if (ev_n < 64) {
        ev[ev_n].who = who;
        ev[ev_n].v = v;
        ev_n++;
    }
}
#define CLOSED PTZ_VERB_COUNT

static bool fake_open(void) { return true; }
static void fake_close(void) { note('L', CLOSED); }
static int last_speed = -1;
static bool fake_emit(enum PtzVerb v, int speed) {
    note('L', v);
    last_speed = speed;
    return true;
}
static bool fake_has(enum PtzVerb v) { (void)v; return true; }
static int last_steps = -1;
static bool fake_move_steps(enum PtzVerb v, int steps, int speed) {
    (void)v;
    last_steps = steps;
    last_speed = speed;
    return true;
}

static int step_ms_speed = -1;
static long fake_step_ms(enum PtzVerb v, int speed) {
    (void)v;
    step_ms_speed = speed;
    return 800;   // the slowest a gpiostep head can be configured: 8 x 100 ms
}

static Actuator fake = {
    .name = "fake",
    .open = fake_open,
    .close = fake_close,
    .emit = fake_emit,
    .has = fake_has,
};

const Actuator *actuator_select(const char *name) { (void)name; return &fake; }

// ---- a pan/tilt head beside it (ptz.head) --------------------------------

static bool use_head = false;
static bool head_opens = true;
static bool head_open(void) { return head_opens; }
static void head_close(void) { note('H', CLOSED); }
static int head_speed = -1;
static bool head_stop_fails = false;
static bool head_emit(enum PtzVerb v, int speed) {
    if (v == PTZ_STOP && head_stop_fails) {
        return false;
    }
    note('H', v);
    head_speed = speed;
    return true;
}
static bool head_has(enum PtzVerb v) { return v == PTZ_STOP || ptz_verb_is_pantilt(v); }
static const char *head_proto(void) { return "pelco-d"; }
static bool head_line(char *port, size_t n, int *baud, int *addr) {
    snprintf(port, n, "/dev/ttyAMA1");
    *baud = 9600;
    *addr = 1;
    return true;
}

static int head_presets = 1;   /* the head stores positions (a Pelco-D base) */
static bool head_preset(enum PtzPresetOp op, int n) { note('P', (int)op * 1000 + n); return true; }
static bool head_has_presets(void) { return head_presets != 0; }

static Actuator head = {
    .name = "pelco",
    .honours_speed = true,
    .proto_name = head_proto,
    .open = head_open,
    .close = head_close,
    .emit = head_emit,
    .has = head_has,
    .line = head_line,
    .preset = head_preset,
    .has_presets = head_has_presets,
};

const Actuator *actuator_select_head(const char *name, const Actuator *lens,
                                     const char **why) {
    (void)name;
    (void)lens;
    if (why) {
        *why = NULL;
    }
    return use_head ? &head : NULL;
}

// ---- the engine and the core, stubbed ------------------------------------

float af_zoom_mag(void) { return 0.0f; }
void af_reader_ensure(void) {}
void af_preempt(void) {}
void af_note_manual_focus(void) {}
unsigned af_focus_gen(void) { return 0; }
void af_book_after_zoom(long at_ms, unsigned gen) { (void)at_ms; (void)gen; }
static volatile int book_ticks = 0;   /* watchdog ticks that offered a booked pass a start */
bool af_book_tick(long now) { (void)now; book_ticks++; return false; }
bool af_alive(void) { return false; }
static const char *port_name = "fake";
const char *config_get_string(const char *path, const char *param) {
    (void)path;
    return param && !strcmp(param, "port") ? port_name : "fake";
}
static int cfg_preset_max = 32, cfg_preset_ms = 300;   /* a short travel, for the tests */
int config_get_int(const char *path, const char *param) {
    (void)path;
    if (param && !strcmp(param, "presetMax")) return cfg_preset_max;
    if (param && !strcmp(param, "presetMs")) return cfg_preset_ms < 500 ? 500 : cfg_preset_ms;
    return param && !strcmp(param, "address") ? INT_MIN : 0;
}
static bool cfg_pan_invert = false, cfg_tilt_invert = false;
bool config_get_boolean(const char *path, const char *param) {
    (void)path;
    if (param && !strcmp(param, "panInvert")) return cfg_pan_invert;
    if (param && !strcmp(param, "tiltInvert")) return cfg_tilt_invert;
    return false;
}
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
    ev_n = 0;
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

// A focus nudge changes the whole picture, as a turn does: it is reported, on
// the focus axis, so the core can hold motion detection back for it -- the
// pass after a zoom raised a motion alarm of its own (OpenIPC/majestic#347).
TEST a_focus_nudge_is_reported_on_the_focus_axis(void) {
    start(false);
    ASSERT(motion_move(PTZ_NEAR, 1000));
    ASSERT_EQ(1, reports());
    ASSERT_EQ(1, rep[0].moving);
    ASSERT_EQ(PTZ_AXIS_FOCUS, rep[0].axis);
    ASSERT(motion_halt());
    ASSERT_EQ(2, reports());
    ASSERT_EQ(0, rep[1].moving);
    stop();
    PASS();
}

// The autofocus pass drives the wire itself, past the verbs: its drive is told
// on the focus axis too, once at the start and once at the stop.
TEST an_autofocus_drive_is_reported_on_the_focus_axis(void) {
    start(false);
    ASSERT(motion_engine_drive(-1));
    ASSERT(motion_engine_drive(-1));   // the pass's next step: still one drive
    ASSERT_EQ(1, reports());
    ASSERT_EQ(1, rep[0].moving);
    ASSERT_EQ(PTZ_AXIS_FOCUS, rep[0].axis);
    ASSERT(motion_engine_drive(0));
    ASSERT_EQ(2, reports());
    ASSERT_EQ(0, rep[1].moving);
    ASSERT_EQ(PTZ_AXIS_FOCUS, rep[1].axis);
    stop();
    PASS();
}

// A lens MCU that goes on pulling focus after a zoom stop (zoom_settle_ms) is
// still changing the picture: the stop is followed by focus moving, until the
// settle runs out. Measured on an 85H50AI, the picture went on changing for
// about three seconds after the zoom stop was reported.
TEST a_lens_settling_after_a_zoom_is_focus_moving(void) {
    start(false);
    fake.zoom_settle_ms = 300;
    ASSERT(motion_move(PTZ_TELE, 100));
    sleep_ms(200);   // the deadline stops the zoom
    ASSERT_EQ(3, reports());
    ASSERT_EQ(2, rep[0].axis);
    ASSERT_EQ(0, rep[1].moving);
    ASSERT_EQ(1, rep[2].moving);
    ASSERT_EQ(PTZ_AXIS_FOCUS, rep[2].axis);
    sleep_ms(400);   // past the settle
    ASSERT_EQ(4, reports());
    ASSERT_EQ(0, rep[3].moving);
    ASSERT_EQ(PTZ_AXIS_FOCUS, rep[3].axis);
    fake.zoom_settle_ms = 0;
    stop();
    PASS();
}

// An operator taking the lens in the middle of a pass ends the pass: its
// trailing stop is refused while their command holds the wire, and nothing may
// go on reporting focus moving once they let go.
TEST a_takeover_mid_pass_leaves_no_focus_report_behind(void) {
    start(false);
    ASSERT(motion_engine_drive(-1));
    ASSERT(motion_move(PTZ_LEFT, 1000));   // the operator pans
    ASSERT_FALSE(motion_engine_drive(0));  // the pass's stop is refused
    ASSERT(motion_halt());
    const int n = reports();
    ASSERT_EQ(0, rep[n - 1].moving);
    sleep_ms(200);   // watchdog ticks
    ASSERT_EQ(n, reports());
    stop();
    PASS();
}

// Closing the port in the middle of a drive ends it at the core as well.
TEST a_drive_cut_short_by_closing_is_ended(void) {
    start(false);
    ASSERT(motion_engine_drive(1));
    stop();
    ASSERT_EQ(2, reports());
    ASSERT_EQ(0, rep[1].moving);
    ASSERT_EQ(PTZ_AXIS_FOCUS, rep[1].axis);
    PASS();
}

// A diagonal turns both axes: it is a move, said on axis -1 (no one axis names
// it), and swinging from it onto one axis is still the same move.
TEST a_diagonal_is_a_move_on_both_axes(void) {
    start(false);
    ASSERT(motion_move(PTZ_UPLEFT, 1000));
    ASSERT_EQ(1, reports());
    ASSERT_EQ(1, rep[0].moving);
    ASSERT_EQ(-1, rep[0].axis);
    ASSERT(motion_move(PTZ_LEFT, 1000));
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

// The serial line is reported as it was opened, not as the config now says
// it should be: the config is read only at open, so after a save the two
// differ until the port is opened again, and an operator correcting a baud
// reads this line to learn whether the lens is being spoken to at it.
static bool fake_line(char *port, size_t n, int *baud, int *addr) {
    snprintf(port, n, "/dev/ttyOPENED");
    *baud = 9600;
    *addr = 7;
    return true;
}

TEST the_line_is_reported_as_opened_not_as_configured(void) {
    char caps[PTZ_CAPS_MAX];
    start(true);
    fake.name = "pelco";
    motion_describe(caps, sizeof caps);   // no line(): the config's
    ASSERT(strstr(caps, " port=fake speed=0 addr=1 "));
    fake.line = fake_line;
    motion_describe(caps, sizeof caps);
    ASSERT(strstr(caps, " port=/dev/ttyOPENED speed=9600 addr=7 "));
    ASSERT(strstr(caps, " state=ready "));
    fake.line = NULL;
    fake.name = "fake";
    stop();
    PASS();
}

// A counted move reaches only an actuator that can count, with its speed; the
// capability line offers steps= only then; zoom/focus are not counted.
TEST a_step_move_reaches_only_a_motor_that_counts(void) {
    char caps[PTZ_CAPS_MAX];
    start(true);
    fake.honours_speed = true;
    ASSERT_FALSE(motion_step(PTZ_LEFT, 20, 50));   // no move_steps op
    ASSERT_EQ(NULL, strstr(motion_describe(caps, sizeof caps), " steps="));
    fake.move_steps = fake_move_steps;
    ASSERT(strstr(motion_describe(caps, sizeof caps), " steps=1"));
    ASSERT(motion_step(PTZ_LEFT, 20, 50));
    ASSERT_EQ(20, last_steps);
    ASSERT_EQ(32, last_speed);
    ASSERT_FALSE(motion_step(PTZ_TELE, 20, 50));   // nothing to count on a zoom
    ASSERT_FALSE(motion_step(PTZ_LEFT, 0, 50));
    ASSERT(motion_halt());
    fake.move_steps = NULL;
    stop();
    PASS();
}

// A counted move's safety deadline covers what the actuator says its steps
// take, not a fixed bound a slow head would overrun, and without step_ms the
// fixed bound stands.
TEST a_step_deadline_follows_the_actuators_step_time(void) {
    start(false);
    fake.honours_speed = true;
    fake.move_steps = fake_move_steps;
    ASSERT(motion_step(PTZ_LEFT, 2, 50));
    ASSERT_EQ(1, reports());
    ASSERT_EQ(2 * 100 + 1000, rep[0].upper_ms);
    ASSERT(motion_halt());
    fake.step_ms = fake_step_ms;
    ASSERT(motion_step(PTZ_RIGHT, 2, 50));
    ASSERT_EQ(32, step_ms_speed);
    ASSERT_EQ(3, reports());
    ASSERT_EQ(2 * 800 * 3 / 2 + 1000, rep[2].upper_ms);
    ASSERT(motion_halt());
    fake.move_steps = NULL;
    fake.step_ms = NULL;
    stop();
    PASS();
}

// A pan count sent while a tilt count runs joins it (the actuator runs both):
// the safety net stays the tilt's, not the short pan's, which would stop the
// tilt short of its target.
TEST a_count_on_the_other_axis_keeps_the_longer_deadline(void) {
    start(false);
    fake.move_steps = fake_move_steps;
    ASSERT(motion_step(PTZ_UP, 30, 0));      // 30 x 100 + 1000 ms
    ASSERT(motion_step(PTZ_RIGHT, 1, 0));    // 1 x 100 + 1000 ms
    ASSERT_EQ(1, reports());
    sleep_ms(1500);                          // past the pan's own deadline
    ASSERT_EQ(1, reports());                 // still moving: not stopped
    ASSERT(motion_halt());
    ASSERT_EQ(2, reports());
    fake.move_steps = NULL;
    stop();
    PASS();
}

// Two long counts together take about as long as both added up -- the head
// takes turns between the axes -- so the net covers the sum, not the longer.
TEST two_counts_together_get_the_time_of_both(void) {
    start(false);
    fake.move_steps = fake_move_steps;
    ASSERT(motion_step(PTZ_UP, 30, 0));      // 30 x 100 + 1000 ms
    ASSERT(motion_step(PTZ_RIGHT, 30, 0));   // and 30 x 100 more
    sleep_ms(4500);                          // past either one's own deadline
    ASSERT_EQ(1, reports());                 // still moving
    ASSERT(motion_halt());
    fake.move_steps = NULL;
    stop();
    PASS();
}

// A count on the same axis replaces the one there: its deadline is its own,
// not the longer one it replaced.
TEST a_count_replacing_one_on_its_axis_has_its_own_deadline(void) {
    start(false);
    fake.move_steps = fake_move_steps;
    ASSERT(motion_step(PTZ_RIGHT, 30, 0));   // 4000 ms
    ASSERT(motion_step(PTZ_LEFT, 1, 0));     // 1100 ms, replacing it
    sleep_ms(1600);
    ASSERT_EQ(2, reports());                 // stopped on the short one's deadline
    ASSERT_EQ(0, rep[1].moving);
    fake.move_steps = NULL;
    stop();
    PASS();
}

static bool fake_frame(int *w, int *h) { *w = 134; *h = 62; return true; }
static bool no_frame(int *w, int *h) { *w = *h = 0; return false; }

// frame= is offered only by a motor that counts its steps and knows how many
// make a frame.
TEST the_frame_is_offered_only_where_it_is_known(void) {
    char caps[PTZ_CAPS_MAX];
    start(true);
    fake.move_steps = NULL;   // whatever a failed test before left
    fake.frame_steps = fake_frame;
    ASSERT_EQ(NULL, strstr(motion_describe(caps, sizeof caps), " frame="));   // no move_steps
    fake.move_steps = fake_move_steps;
    ASSERT(strstr(motion_describe(caps, sizeof caps), " steps=1 frame=134x62"));
    fake.frame_steps = no_frame;
    ASSERT_EQ(NULL, strstr(motion_describe(caps, sizeof caps), " frame="));
    fake.frame_steps = NULL;
    ASSERT_EQ(NULL, strstr(motion_describe(caps, sizeof caps), " frame="));
    fake.move_steps = NULL;
    stop();
    PASS();
}

// ---- a lens and a head together -------------------------------------------

static void start_with_head(bool lens_reports) {
    use_head = true;
    head_opens = true;
    start(lens_reports);
}

static void stop_with_head(void) {
    head_stop_fails = false;
    stop();
    use_head = false;
    head_opens = true;
}

static bool event(int i, char who, int v) { return i < ev_n && ev[i].who == who && ev[i].v == v; }

// Pan and tilt go to the head, zoom and focus to the lens; nothing of the
// lens's reaches the head, and the other way round.
TEST pan_goes_to_the_head_and_zoom_to_the_lens(void) {
    start_with_head(false);
    ASSERT(motion_move(PTZ_LEFT, 1000));
    ASSERT(event(0, 'H', PTZ_LEFT));
    ASSERT(motion_move(PTZ_UPRIGHT, 1000));
    ASSERT(event(1, 'H', PTZ_UPRIGHT));
    ASSERT(motion_halt());
    ASSERT(motion_move(PTZ_NEAR, 1000));
    ASSERT(motion_halt());
    for (int i = 0; i < ev_n; i++) {
        if (ev[i].who == 'H') {
            ASSERT(ev[i].v == PTZ_STOP || ptz_verb_is_pantilt((enum PtzVerb)ev[i].v));
        }
    }
    int near = -1;
    for (int i = 0; i < ev_n; i++) {
        if (ev[i].v == PTZ_NEAR) {
            near = i;
        }
    }
    ASSERT(near >= 0 && ev[near].who == 'L');
    stop_with_head();
    PASS();
}

// One verb on the wire at a time, across both motors: a pan arriving during a
// zoom stops the lens first, and a zoom during a pan stops the head first.
TEST the_other_motor_is_stopped_before_this_one_starts(void) {
    start_with_head(false);
    ASSERT(motion_move(PTZ_TELE, 1000));
    ASSERT(event(0, 'L', PTZ_TELE));
    ASSERT(motion_move(PTZ_LEFT, 1000));
    ASSERT(event(1, 'L', PTZ_STOP));
    ASSERT(event(2, 'H', PTZ_LEFT));
    ASSERT(motion_move(PTZ_WIDE, 1000));
    ASSERT(event(3, 'H', PTZ_STOP));
    ASSERT(event(4, 'L', PTZ_WIDE));
    stop_with_head();
    PASS();
}

// A stop stops both, and so does closing; the head is released before the
// lens, which goes last as it always has.
TEST a_stop_reaches_both_and_the_head_closes_first(void) {
    start_with_head(false);
    ASSERT(motion_move(PTZ_LEFT, 1000));
    ev_n = 0;
    ASSERT(motion_halt());
    ASSERT(event(0, 'L', PTZ_STOP));
    ASSERT(event(1, 'H', PTZ_STOP));
    ev_n = 0;
    stop_with_head();
    int h = -1, l = -1;
    for (int i = 0; i < ev_n; i++) {
        if (ev[i].v == CLOSED) {
            if (ev[i].who == 'H') h = i;
            if (ev[i].who == 'L') l = i;
        }
    }
    ASSERT(h >= 0 && l > h);
    PASS();
}

// The autofocus pass drives the lens alone: not even its trailing stop goes to
// the head.
TEST the_focus_pass_never_touches_the_head(void) {
    start_with_head(false);
    ASSERT(motion_engine_drive(-1));
    ASSERT(motion_engine_drive(0));
    ASSERT_EQ(2, ev_n);
    ASSERT(event(0, 'L', PTZ_NEAR));
    ASSERT(event(1, 'L', PTZ_STOP));
    stop_with_head();
    PASS();
}

// The speed is the head's to keep: it reaches the head's pan at 1..63 even
// though the lens keeps none.
TEST the_heads_speed_is_its_own(void) {
    start_with_head(false);
    ASSERT(motion_move_at(PTZ_LEFT, 500, 50));
    ASSERT_EQ(32, head_speed);
    ASSERT(motion_move_at(PTZ_TELE, 500, 50));
    ASSERT_EQ(0, last_speed);   // the lens does not keep one
    stop_with_head();
    PASS();
}

// One capability line: every verb either motor carries, the head's serial
// line and speeds, and the head named at the end, after every token a core
// already reads.
TEST the_capability_line_describes_both(void) {
    char caps[PTZ_CAPS_MAX];
    start_with_head(true);
    fake.name = "ms41908";
    motion_describe(caps, sizeof caps);
    ASSERT(strstr(caps, "actuator=ms41908 "));
    ASSERT(strstr(caps, " port=/dev/ttyAMA1 speed=9600 addr=1 "));
    ASSERT(strstr(caps, " state=ready "));
    ASSERT(strstr(caps, "left"));
    ASSERT(strstr(caps, "tele"));
    ASSERT(strstr(caps, " speeds=1-100"));
    size_t len = strlen(caps);
    const char *tail = " head=pelco-d headstate=ready";
    ASSERT(len > strlen(tail) && !strcmp(caps + len - strlen(tail), tail));
    fake.name = "fake";
    stop_with_head();
    PASS();
}

// The lens reports its own motor (an MS41908M) and the head is reported from
// its verbs (a Pelco-D base): the pan is reported, the zoom is not, and the
// pan's stop is told before the zoom starts, never after.
TEST each_motor_is_reported_the_way_it_can_be(void) {
    start_with_head(true);
    ASSERT(motion_move(PTZ_LEFT, 1000));
    ASSERT_EQ(1, reports());
    ASSERT_EQ(1, rep[0].moving);
    ASSERT_EQ(0, rep[0].axis);
    ASSERT(motion_move(PTZ_TELE, 1000));
    ASSERT_EQ(2, reports());
    ASSERT_EQ(0, rep[1].moving);
    ASSERT_EQ(0, rep[1].axis);   // the pan's axis, not the zoom's
    ASSERT(motion_halt());
    ASSERT_EQ(2, reports());     // the lens tells its own stop
    stop_with_head();
    PASS();
}

// A head that will not open leaves the lens working and says it is closed;
// the next pan tries it again and, once it opens, drives it.
TEST a_head_that_will_not_open_leaves_the_lens_working(void) {
    char caps[PTZ_CAPS_MAX];
    use_head = true;
    head_opens = false;
    start(false);
    ASSERT_FALSE(motion_move(PTZ_LEFT, 500));
    ASSERT(motion_move(PTZ_TELE, 500));
    motion_describe(caps, sizeof caps);
    ASSERT(strstr(caps, " headstate=closed"));
    ASSERT_EQ(NULL, strstr(caps, "left"));
    ASSERT(motion_halt());
    head_opens = true;
    ASSERT(motion_move(PTZ_LEFT, 500));
    motion_describe(caps, sizeof caps);
    ASSERT(strstr(caps, " headstate=ready"));
    ASSERT(strstr(caps, "left"));
    stop_with_head();
    PASS();
}

// A base whose stop does not reach the wire is not left turning untracked: the
// zoom that would replace its pan is refused, and the pan stays the watchdog's,
// whose deadline goes on asking it to stop.
TEST a_head_that_misses_its_stop_keeps_its_move(void) {
    start_with_head(false);
    ASSERT(motion_move(PTZ_LEFT, 300));
    head_stop_fails = true;
    ASSERT_FALSE(motion_move(PTZ_TELE, 500));
    ASSERT(motion_manual_active());   // the pan is still the move on the wire
    for (int i = 0; i < ev_n; i++) {
        ASSERT(ev[i].v != PTZ_TELE);   // the lens was never sent the zoom
    }
    head_stop_fails = false;
    sleep_ms(500);                     // its deadline stops it, now that it can
    ASSERT_FALSE(motion_manual_active());
    stop_with_head();
    PASS();
}

// A head mounted the other way round: ptz.panInvert and ptz.tiltInvert turn
// each pan/tilt verb before it reaches the motor, diagonals included, and leave
// zoom alone. Off, the verb goes out as asked.
TEST an_inverted_head_turns_the_other_way(void) {
    cfg_pan_invert = true;
    cfg_tilt_invert = true;
    start_with_head(false);
    ASSERT(motion_move(PTZ_LEFT, 500));
    ASSERT(event(0, 'H', PTZ_RIGHT));
    ASSERT(motion_move(PTZ_UPLEFT, 500));
    ASSERT(event(1, 'H', PTZ_DOWNRIGHT));
    ASSERT(motion_move(PTZ_TELE, 500));
    ASSERT(event(3, 'L', PTZ_TELE));      // after the head's stop
    stop_with_head();
    cfg_pan_invert = false;
    start_with_head(false);
    ASSERT(motion_move(PTZ_LEFT, 500));
    ASSERT(event(0, 'H', PTZ_LEFT));      // pan as asked
    ASSERT(motion_move(PTZ_UP, 500));
    ASSERT(event(1, 'H', PTZ_DOWN));      // tilt still inverted
    stop_with_head();
    cfg_tilt_invert = false;
    PASS();
}

// ---- stored positions (presets) --------------------------------------------

static int preset_event(int op, int n) { return op * 1000 + n; }

// A go-to reaches the head, stops a zoom the lens is running first, and is the
// wire's move: reported moving, manual-active, and ended at its travel time WITHOUT
// a stop -- a stop there would halt the base short of the preset.
TEST a_go_to_is_the_heads_move_and_ends_without_a_stop(void) {
    start_with_head(false);
    ASSERT(motion_move(PTZ_TELE, 1000));
    ASSERT(motion_preset(PTZ_PRESET_GO, 2));
    ASSERT(event(1, 'L', PTZ_STOP));                       // the zoom stopped first
    ASSERT(event(2, 'P', preset_event(PTZ_PRESET_GO, 2)));
    ASSERT(motion_manual_active());
    int before = ev_n;
    sleep_ms(800);                                         // past presetMs (500 here)
    ASSERT_FALSE(motion_manual_active());
    for (int i = before; i < ev_n; i++) {
        ASSERT(ev[i].v != PTZ_STOP);                       // nothing stopped the head
    }
    stop_with_head();
    PASS();
}

// An operator's stop, or any manual move, ends a go-to the ordinary way.
TEST a_go_to_yields_to_the_operator(void) {
    start_with_head(false);
    ASSERT(motion_preset(PTZ_PRESET_GO, 3));
    ASSERT(motion_halt());
    ASSERT(event(1, 'L', PTZ_STOP) || event(1, 'H', PTZ_STOP));
    int stops = 0;
    for (int i = 0; i < ev_n; i++) stops += ev[i].who == 'H' && ev[i].v == PTZ_STOP;
    ASSERT(stops >= 1);                                    // the head was told to stop
    ASSERT(motion_preset(PTZ_PRESET_GO, 3));
    ASSERT(motion_move(PTZ_LEFT, 300));                    // a pan replaces the go-to
    ASSERT(event(ev_n - 1, 'H', PTZ_LEFT));
    sleep_ms(500);
    ASSERT_FALSE(motion_manual_active());                  // and ends as a pan does
    ASSERT(event(ev_n - 1, 'H', PTZ_STOP) || event(ev_n - 2, 'H', PTZ_STOP));
    stop_with_head();
    PASS();
}

// Set and clear store and forget, and move nothing: the wire stays idle.
TEST set_and_clear_move_nothing(void) {
    start_with_head(false);
    ASSERT(motion_preset(PTZ_PRESET_SET, 5));
    ASSERT(motion_preset(PTZ_PRESET_CLEAR, 5));
    ASSERT_EQ(2, ev_n);
    ASSERT(event(0, 'P', preset_event(PTZ_PRESET_SET, 5)));
    ASSERT(event(1, 'P', preset_event(PTZ_PRESET_CLEAR, 5)));
    ASSERT_FALSE(motion_manual_active());
    stop_with_head();
    PASS();
}

// Only 1..presetMax, and only where the pan/tilt motor has presets; the capability
// line says so, and says nothing where there are none.
TEST presets_are_offered_only_where_they_exist(void) {
    char caps[PTZ_CAPS_MAX];
    start_with_head(false);
    ASSERT(strstr(motion_describe(caps, sizeof caps), " presets=1-32"));
    ASSERT_FALSE(motion_preset(PTZ_PRESET_GO, 0));
    ASSERT_FALSE(motion_preset(PTZ_PRESET_GO, 33));
    cfg_preset_max = 0;
    ASSERT_EQ(NULL, strstr(motion_describe(caps, sizeof caps), "presets="));
    ASSERT_FALSE(motion_preset(PTZ_PRESET_GO, 1));
    cfg_preset_max = 32;
    head_presets = 0;
    ASSERT_EQ(NULL, strstr(motion_describe(caps, sizeof caps), "presets="));
    ASSERT_FALSE(motion_preset(PTZ_PRESET_SET, 1));
    head_presets = 1;
    stop_with_head();
    start(false);                                          // no head: the fake lens has none
    ASSERT_EQ(NULL, strstr(motion_describe(caps, sizeof caps), "presets="));
    ASSERT_FALSE(motion_preset(PTZ_PRESET_GO, 1));
    stop();
    PASS();
}

// A focus pass driving the lens is stopped before the head moves -- a go-to or a
// pan alike: its own trailing stop is refused while the head holds the wire, so
// nothing else would stop the focus motor.
TEST the_head_moving_stops_a_focus_drive_first(void) {
    start_with_head(false);
    ASSERT(motion_engine_drive(-1));
    ASSERT(event(0, 'L', PTZ_NEAR));
    ASSERT(motion_preset(PTZ_PRESET_GO, 2));
    ASSERT(event(1, 'L', PTZ_STOP));
    ASSERT(event(2, 'P', preset_event(PTZ_PRESET_GO, 2)));
    stop_with_head();
    start_with_head(false);
    ASSERT(motion_engine_drive(+1));
    ASSERT(motion_move(PTZ_LEFT, 300));
    ASSERT(event(1, 'L', PTZ_STOP));
    ASSERT(event(2, 'H', PTZ_LEFT));
    stop_with_head();
    PASS();
}

// A booked pass waits while a move holds the wire -- it could not drive focus
// until the move ends -- and is offered its start again once the wire is quiet.
TEST a_booked_pass_waits_out_a_go_to(void) {
    start_with_head(false);
    ASSERT(motion_preset(PTZ_PRESET_GO, 2));
    sleep_ms(60);
    int during = book_ticks;
    sleep_ms(200);
    ASSERT_EQ(during, book_ticks);        // held while the head travels
    sleep_ms(500);                        // past presetMs (500 here)
    ASSERT(book_ticks > during);          // offered again once it is quiet
    stop_with_head();
    PASS();
}

// Without a head the line carries no head token: a one-motor camera's line is
// exactly what it was.
TEST no_head_no_head_token(void) {
    char caps[PTZ_CAPS_MAX];
    start(false);
    ASSERT_EQ(NULL, strstr(motion_describe(caps, sizeof caps), "head"));
    stop();
    PASS();
}

SUITE(motion_report_suite) {
    RUN_TEST(a_move_is_reported_once_and_its_deadline_ends_it);
    RUN_TEST(a_stop_ends_it_and_a_new_axis_is_one_move);
    RUN_TEST(a_focus_nudge_is_reported_on_the_focus_axis);
    RUN_TEST(an_autofocus_drive_is_reported_on_the_focus_axis);
    RUN_TEST(a_drive_cut_short_by_closing_is_ended);
    RUN_TEST(a_takeover_mid_pass_leaves_no_focus_report_behind);
    RUN_TEST(a_lens_settling_after_a_zoom_is_focus_moving);
    RUN_TEST(a_diagonal_is_a_move_on_both_axes);
    RUN_TEST(a_backend_that_reports_itself_is_left_to_it);
    RUN_TEST(a_speed_reaches_only_a_motor_that_keeps_it);
    RUN_TEST(a_long_port_path_keeps_the_speed_offer);
    RUN_TEST(the_line_is_reported_as_opened_not_as_configured);
    RUN_TEST(a_step_move_reaches_only_a_motor_that_counts);
    RUN_TEST(a_count_on_the_other_axis_keeps_the_longer_deadline);
    RUN_TEST(two_counts_together_get_the_time_of_both);
    RUN_TEST(a_count_replacing_one_on_its_axis_has_its_own_deadline);
    RUN_TEST(the_frame_is_offered_only_where_it_is_known);
    RUN_TEST(a_step_deadline_follows_the_actuators_step_time);
    RUN_TEST(no_head_no_head_token);
    RUN_TEST(pan_goes_to_the_head_and_zoom_to_the_lens);
    RUN_TEST(the_other_motor_is_stopped_before_this_one_starts);
    RUN_TEST(a_stop_reaches_both_and_the_head_closes_first);
    RUN_TEST(the_focus_pass_never_touches_the_head);
    RUN_TEST(the_heads_speed_is_its_own);
    RUN_TEST(the_capability_line_describes_both);
    RUN_TEST(each_motor_is_reported_the_way_it_can_be);
    RUN_TEST(a_head_that_will_not_open_leaves_the_lens_working);
    RUN_TEST(a_head_that_misses_its_stop_keeps_its_move);
    RUN_TEST(an_inverted_head_turns_the_other_way);
    RUN_TEST(a_go_to_is_the_heads_move_and_ends_without_a_stop);
    RUN_TEST(a_go_to_yields_to_the_operator);
    RUN_TEST(set_and_clear_move_nothing);
    RUN_TEST(presets_are_offered_only_where_they_exist);
    RUN_TEST(the_head_moving_stops_a_focus_drive_first);
    RUN_TEST(a_booked_pass_waits_out_a_go_to);
}

GREATEST_MAIN_DEFS();

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(motion_report_suite);
    GREATEST_MAIN_END();
}
