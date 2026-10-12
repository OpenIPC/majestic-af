// The gpiostep backend's pure logic, pinned: the board config it reads from
// /etc/gpiostep.conf, the verb->axis mapping with the board's direction signs, and
// the clamp that keeps the head off its gearbox stops.

#include <greatest.h>

#include "act_gpiostep.h"

// No file: the motors run, but nothing is known about the head.
TEST defaults_know_no_limits(void) {
    GsConfig c;
    gs_config_defaults(&c);
    ASSERT_EQ(0, c.travel[GS_PAN]);
    ASSERT_EQ(0, c.travel[GS_TILT]);
    ASSERT_EQ(1, c.left_sign);
    ASSERT_EQ(1, c.up_sign);
    ASSERT_FALSE(c.home);
    PASS();
}

// A board file, line by line (with a negative pan sign, so that parses too).
TEST config_reads_a_board(void) {
    GsConfig c;
    gs_config_defaults(&c);
    ASSERT(gs_config_line(&c, "# Zenointel SD-2N-4G"));
    ASSERT(gs_config_line(&c, ""));
    ASSERT(gs_config_line(&c, "pan_travel=580"));
    ASSERT(gs_config_line(&c, " tilt_travel = 170 "));
    ASSERT(gs_config_line(&c, "pan_left=-1"));
    ASSERT(gs_config_line(&c, "tilt_up=1"));
    ASSERT(gs_config_line(&c, "pan_delay_us=2000"));
    ASSERT(gs_config_line(&c, "tilt_delay_us=3000"));
    ASSERT(gs_config_line(&c, "home=1"));
    ASSERT_EQ(580, c.travel[GS_PAN]);
    ASSERT_EQ(170, c.travel[GS_TILT]);
    ASSERT_EQ(-1, c.left_sign);
    ASSERT_EQ(1, c.up_sign);
    ASSERT_EQ(2000, c.delay_us[GS_PAN]);
    ASSERT_EQ(3000, c.delay_us[GS_TILT]);
    ASSERT(c.home);
    PASS();
}

// A bad line is refused and leaves the config as it was.
// Homing has a rate of its own, and is never faster than the running rate:
// a head configured to run fast still seeks its stops slowly.
TEST homing_runs_slow_whatever_the_running_rate(void) {
    GsConfig c;
    gs_config_defaults(&c);
    ASSERT_EQ(2000, gs_home_delay(&c, GS_PAN));    // defaults: as every head has homed
    ASSERT_EQ(3000, gs_home_delay(&c, GS_TILT));
    ASSERT(gs_config_line(&c, "pan_delay_us=833"));
    ASSERT(gs_config_line(&c, "tilt_delay_us=833"));
    ASSERT_EQ(2000, gs_home_delay(&c, GS_PAN));    // running faster homes no faster
    ASSERT_EQ(3000, gs_home_delay(&c, GS_TILT));
    ASSERT(gs_config_line(&c, "pan_home_delay_us=4000"));
    ASSERT_EQ(4000, gs_home_delay(&c, GS_PAN));    // a board may home slower still
    ASSERT(gs_config_line(&c, "tilt_delay_us=5000"));
    ASSERT_EQ(5000, gs_home_delay(&c, GS_TILT));   // and never faster than it runs
    ASSERT_FALSE(gs_config_line(&c, "tilt_home_delay_us=10"));
    ASSERT_EQ(5000, gs_home_delay(&c, GS_TILT));
    PASS();
}

// A homing ioctl lasts at most ~0.4 s, so a stop or shutdown during the seek
// waits no longer than that -- down to one step at the slowest delay accepted.
TEST homing_chunks_stay_short_at_any_rate(void) {
    ASSERT_EQ(20, gs_home_chunk(2000));     // 0.32 s: as homing always chunked
    ASSERT_EQ(16, gs_home_chunk(3000));     // 0.38 s
    ASSERT_EQ(20, gs_home_chunk(200));      // never more than 20 steps
    ASSERT_EQ(1, gs_home_chunk(100000));    // 0.8 s: one step, the least there is
    for (int d = 200; d <= 100000; d += 100) {
        int n = gs_home_chunk(d);
        ASSERT(n >= 1 && n <= 20);
        ASSERT(n == 1 || 8L * d * n <= GS_HOME_CHUNK_US);
    }
    PASS();
}

// A speed below the top stretches the delay by 63/speed, so the step rate
// scales with it; none asked for, or the top, is the configured rate.
TEST speed_scales_the_step_rate(void) {
    GsConfig c;
    gs_config_defaults(&c);
    ASSERT(gs_config_line(&c, "pan_delay_us=833"));
    ASSERT_EQ(833, gs_speed_delay(&c, GS_PAN, 0));
    ASSERT_EQ(833, gs_speed_delay(&c, GS_PAN, 63));
    ASSERT_EQ(833 * 63 / 32, gs_speed_delay(&c, GS_PAN, 32));
    ASSERT_EQ(833 * 63 / 8, gs_speed_delay(&c, GS_PAN, 8));
    ASSERT_EQ(3000 * 63 / 16, gs_speed_delay(&c, GS_TILT, 16));
    ASSERT_EQ(GS_RUN_CHUNK_US / 8, gs_speed_delay(&c, GS_TILT, 2));   // floored: one step <= 100 ms
    ASSERT_EQ(GS_RUN_CHUNK_US / 8, gs_speed_delay(&c, GS_PAN, 1));   // one step fits a running ioctl
    ASSERT(gs_config_line(&c, "tilt_delay_us=20000"));
    ASSERT_EQ(20000, gs_speed_delay(&c, GS_TILT, 1));   // a board configured slower keeps its own rate
    PASS();
}

// A running ioctl lasts at most GS_RUN_CHUNK_US at any speed, so a stop, a
// deadline or a shutdown lands within it -- down to one step where the board
// itself is configured slower than that.
TEST running_chunks_stay_short_at_any_speed(void) {
    ASSERT_EQ(GS_CHUNK_MAX, gs_run_chunk(833));
    ASSERT_EQ(GS_CHUNK_MAX, gs_run_chunk(3000));
    ASSERT_EQ(1, gs_run_chunk(GS_RUN_CHUNK_US / 8));
    ASSERT_EQ(1, gs_run_chunk(100000));
    GsConfig c;
    gs_config_defaults(&c);
    for (int d = 200; d <= 5000; d += 50) {
        c.delay_us[GS_PAN] = d;
        for (int sp = 0; sp <= 63; sp++) {
            int delay = gs_speed_delay(&c, GS_PAN, sp);
            ASSERT(8L * delay * gs_run_chunk(delay) <= GS_RUN_CHUNK_US);
        }
    }
    PASS();
}

TEST config_refuses_nonsense(void) {
    GsConfig c;
    gs_config_defaults(&c);
    ASSERT_FALSE(gs_config_line(&c, "pan_travel"));          // no '='
    ASSERT_FALSE(gs_config_line(&c, "pan_travel=fast"));     // not a number
    ASSERT_FALSE(gs_config_line(&c, "pan_travel=-5"));       // out of range
    ASSERT_FALSE(gs_config_line(&c, "pan_left=0"));          // a sign is never 0
    ASSERT_FALSE(gs_config_line(&c, "tilt_delay_us=10"));    // faster than a coil follows
    ASSERT_FALSE(gs_config_line(&c, "zoom_travel=100"));     // unknown key
    ASSERT_FALSE(gs_config_line(&c, "pan_travelx=100"));     // not a prefix match
    ASSERT_EQ(0, c.travel[GS_PAN]);
    ASSERT_EQ(1, c.left_sign);
    ASSERT_EQ(3000, c.delay_us[GS_TILT]);
    PASS();
}

#define DIRS_EQ(c, v, pan, tilt)                                                \
    do {                                                                        \
        int d_[GS_AXES] = {9, 9};                                               \
        ASSERT(gs_verb_dirs((c), (v), d_));                                     \
        ASSERT_EQ_FMT((pan), d_[GS_PAN], "%d");                                 \
        ASSERT_EQ_FMT((tilt), d_[GS_TILT], "%d");                               \
    } while (0)

// left/right and up/down are opposite, the board's sign decides which is +, and
// an axis a verb leaves alone is 0.
TEST verbs_follow_the_board_signs(void) {
    GsConfig c;
    gs_config_defaults(&c);
    DIRS_EQ(&c, PTZ_LEFT, 1, 0);
    DIRS_EQ(&c, PTZ_RIGHT, -1, 0);
    DIRS_EQ(&c, PTZ_UP, 0, 1);
    DIRS_EQ(&c, PTZ_DOWN, 0, -1);
    c.left_sign = -1;
    c.up_sign = -1;
    DIRS_EQ(&c, PTZ_LEFT, -1, 0);
    DIRS_EQ(&c, PTZ_DOWN, 0, 1);
    PASS();
}

// A diagonal turns both axes, each the way its own verb would.
TEST diagonals_turn_both_axes(void) {
    GsConfig c;
    gs_config_defaults(&c);
    DIRS_EQ(&c, PTZ_UPLEFT, 1, 1);
    DIRS_EQ(&c, PTZ_UPRIGHT, -1, 1);
    DIRS_EQ(&c, PTZ_DOWNLEFT, 1, -1);
    DIRS_EQ(&c, PTZ_DOWNRIGHT, -1, -1);
    c.left_sign = -1;
    c.up_sign = -1;
    DIRS_EQ(&c, PTZ_UPLEFT, -1, -1);
    DIRS_EQ(&c, PTZ_DOWNRIGHT, 1, 1);
    PASS();
}

// Two armed axes take turns, pan first after a tilt step; one armed axis steps
// alone whatever stepped last; none, nothing.
TEST axes_take_turns_on_a_diagonal(void) {
    int both[GS_AXES] = {1, -1}, pan[GS_AXES] = {-1, 0}, tilt[GS_AXES] = {0, 1};
    int still[GS_AXES] = {0, 0};
    ASSERT_EQ(GS_PAN, gs_next_axis(both, GS_TILT));
    ASSERT_EQ(GS_TILT, gs_next_axis(both, GS_PAN));
    ASSERT_EQ(GS_PAN, gs_next_axis(pan, GS_PAN));
    ASSERT_EQ(GS_PAN, gs_next_axis(pan, GS_TILT));
    ASSERT_EQ(GS_TILT, gs_next_axis(tilt, GS_TILT));
    ASSERT_EQ(GS_TILT, gs_next_axis(tilt, GS_PAN));
    ASSERT_EQ(-1, gs_next_axis(still, GS_PAN));
    // Ten steps of a diagonal: five on each axis, alternating.
    int last = GS_TILT, n[GS_AXES] = {0, 0};
    for (int i = 0; i < 10; i++) {
        int a = gs_next_axis(both, last);
        ASSERT(a != last);
        n[a]++;
        last = a;
    }
    ASSERT_EQ(5, n[GS_PAN]);
    ASSERT_EQ(5, n[GS_TILT]);
    PASS();
}

// A pan count then a tilt count: both run, each its own. A held direction is
// replaced by a count, a count by a held direction, and a count on the same
// axis by the new count.
TEST counts_on_two_axes_run_together(void) {
    int cur[GS_AXES] = {0, 0}, left[GS_AXES] = {-1, -1}, sp[GS_AXES] = {0, 0};
    bool armed[GS_AXES];
    int pan[GS_AXES] = {1, 0}, tilt[GS_AXES] = {0, -1}, both[GS_AXES] = {-1, 1};
    gs_arm_axes(pan, 40, 63, cur, left, sp, armed);
    ASSERT(armed[GS_PAN] && !armed[GS_TILT]);
    gs_arm_axes(tilt, 12, 20, cur, left, sp, armed);
    ASSERT(!armed[GS_PAN] && armed[GS_TILT]);
    ASSERT_EQ(63, sp[GS_PAN]);    // the pan count keeps its own speed
    ASSERT_EQ(20, sp[GS_TILT]);
    ASSERT_EQ(1, cur[GS_PAN]);   ASSERT_EQ(40, left[GS_PAN]);
    ASSERT_EQ(-1, cur[GS_TILT]); ASSERT_EQ(12, left[GS_TILT]);
    // a new pan count replaces the pan count, the tilt one goes on
    int back[GS_AXES] = {-1, 0};
    gs_arm_axes(back, 5, 30, cur, left, sp, armed);
    ASSERT_EQ(-1, cur[GS_PAN]);  ASSERT_EQ(5, left[GS_PAN]);
    ASSERT_EQ(-1, cur[GS_TILT]); ASSERT_EQ(12, left[GS_TILT]);
    // held until stopped: replaces both
    gs_arm_axes(pan, -1, 30, cur, left, sp, armed);
    ASSERT_EQ(1, cur[GS_PAN]);   ASSERT_EQ(-1, left[GS_PAN]);
    ASSERT_EQ(0, cur[GS_TILT]);
    // a count over a held direction on the other axis stops it
    gs_arm_axes(tilt, 7, 30, cur, left, sp, armed);
    ASSERT_EQ(0, cur[GS_PAN]);
    ASSERT_EQ(-1, cur[GS_TILT]); ASSERT_EQ(7, left[GS_TILT]);
    // a diagonal count arms both
    gs_arm_axes(both, 3, 30, cur, left, sp, armed);
    ASSERT(armed[GS_PAN] && armed[GS_TILT]);
    ASSERT_EQ(3, left[GS_PAN]); ASSERT_EQ(3, left[GS_TILT]);
    PASS();
}

// The steps a whole frame takes are the board's, and only the board's.
TEST frame_steps_come_from_the_board(void) {
    GsConfig c;
    gs_config_defaults(&c);
    ASSERT_EQ(0, c.frame_steps[GS_PAN]);
    ASSERT_EQ(0, c.frame_steps[GS_TILT]);
    ASSERT(gs_config_line(&c, "pan_frame_steps=134"));
    ASSERT(gs_config_line(&c, "tilt_frame_steps = 62"));
    ASSERT_EQ(134, c.frame_steps[GS_PAN]);
    ASSERT_EQ(62, c.frame_steps[GS_TILT]);
    ASSERT_FALSE(gs_config_line(&c, "pan_frame_steps=-3"));
    ASSERT_FALSE(gs_config_line(&c, "tilt_frame_steps=lots"));
    ASSERT_FALSE(gs_config_line(&c, "pan_frame_steps=2001"));   // half of it must fit one counted move
    ASSERT(gs_config_line(&c, "pan_frame_steps=2000"));
    ASSERT_EQ(62, c.frame_steps[GS_TILT]);
    ASSERT_EQ(2000, c.frame_steps[GS_PAN]);
    PASS();
}

// A pan/tilt head carries no zoom, focus or ICR verb, and stop is not a move.
TEST verbs_a_head_does_not_carry(void) {
    GsConfig c;
    gs_config_defaults(&c);
    int d[GS_AXES];
    ASSERT_FALSE(gs_verb_dirs(&c, PTZ_STOP, d));
    ASSERT_FALSE(gs_verb_dirs(&c, PTZ_NEAR, d));
    ASSERT_FALSE(gs_verb_dirs(&c, PTZ_FAR, d));
    ASSERT_FALSE(gs_verb_dirs(&c, PTZ_TELE, d));
    ASSERT_FALSE(gs_verb_dirs(&c, PTZ_WIDE, d));
    ASSERT_FALSE(gs_verb_dirs(&c, PTZ_DAY, d));
    ASSERT_FALSE(gs_verb_dirs(&c, PTZ_NIGHT, d));
    PASS();
}

// The clamp never carries the head past a stop, from either end.
TEST clamp_stops_short_of_the_stops(void) {
    ASSERT_EQ(2, gs_clamp_step(100, 1, 2, 580));   // mid-travel: the whole chunk
    ASSERT_EQ(1, gs_clamp_step(579, 1, 2, 580));   // the remainder at the edge
    ASSERT_EQ(0, gs_clamp_step(580, 1, 2, 580));   // nothing AT the edge
    ASSERT_EQ(2, gs_clamp_step(100, -1, 2, 580));
    ASSERT_EQ(1, gs_clamp_step(1, -1, 2, 580));
    ASSERT_EQ(0, gs_clamp_step(0, -1, 2, 580));
    ASSERT_EQ(0, gs_clamp_step(50, 1, 0, 580));    // no move asked
    PASS();
}

// Travel unknown: no limit at all, in either direction.
TEST clamp_without_travel_allows_all(void) {
    ASSERT_EQ(2, gs_clamp_step(0, -1, 2, 0));
    ASSERT_EQ(2, gs_clamp_step(100000, 1, 2, 0));
    PASS();
}

// A saved position is trusted axis by axis: one axis without a travel, or a
// value past its travel, does not cost the other its limits.
TEST saved_position_is_per_axis(void) {
    GsConfig c;
    gs_config_defaults(&c);
    c.travel[GS_PAN] = 580;
    c.travel[GS_TILT] = 170;
    int pos[GS_AXES] = {-1, -1};
    bool known[GS_AXES];

    ASSERT_EQ(2, gs_parse_pos(&c, "290 85\n", pos, known));
    ASSERT(known[GS_PAN] && known[GS_TILT]);
    ASSERT_EQ(290, pos[GS_PAN]);
    ASSERT_EQ(85, pos[GS_TILT]);

    // A tilt past its stop is not a position; the pan still is.
    ASSERT_EQ(1, gs_parse_pos(&c, "300 171", pos, known));
    ASSERT(known[GS_PAN] && !known[GS_TILT]);
    ASSERT_EQ(300, pos[GS_PAN]);

    // A head whose tilt travel nobody measured: the pan keeps its position.
    c.travel[GS_TILT] = 0;
    ASSERT_EQ(1, gs_parse_pos(&c, "10 85", pos, known));
    ASSERT(known[GS_PAN] && !known[GS_TILT]);

    ASSERT_EQ(0, gs_parse_pos(&c, "", pos, known));
    ASSERT_EQ(0, gs_parse_pos(&c, "moving", pos, known));
    ASSERT_EQ(0, gs_parse_pos(&c, "-1 5", pos, known));
    ASSERT(!known[GS_PAN]);
    PASS();
}

SUITE(gpiostep_suite) {
    RUN_TEST(defaults_know_no_limits);
    RUN_TEST(config_reads_a_board);
    RUN_TEST(homing_runs_slow_whatever_the_running_rate);
    RUN_TEST(homing_chunks_stay_short_at_any_rate);
    RUN_TEST(speed_scales_the_step_rate);
    RUN_TEST(running_chunks_stay_short_at_any_speed);
    RUN_TEST(config_refuses_nonsense);
    RUN_TEST(verbs_follow_the_board_signs);
    RUN_TEST(diagonals_turn_both_axes);
    RUN_TEST(axes_take_turns_on_a_diagonal);
    RUN_TEST(counts_on_two_axes_run_together);
    RUN_TEST(frame_steps_come_from_the_board);
    RUN_TEST(verbs_a_head_does_not_carry);
    RUN_TEST(clamp_stops_short_of_the_stops);
    RUN_TEST(clamp_without_travel_allows_all);
    RUN_TEST(saved_position_is_per_axis);
}
