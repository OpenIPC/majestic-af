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

// left/right and up/down are opposite, and the board's sign decides which is +.
TEST verbs_follow_the_board_signs(void) {
    GsConfig c;
    gs_config_defaults(&c);
    int a, d;
    ASSERT(gs_verb_axis(&c, PTZ_LEFT, &a, &d));  ASSERT_EQ(GS_PAN, a);  ASSERT_EQ(1, d);
    ASSERT(gs_verb_axis(&c, PTZ_RIGHT, &a, &d)); ASSERT_EQ(GS_PAN, a);  ASSERT_EQ(-1, d);
    ASSERT(gs_verb_axis(&c, PTZ_UP, &a, &d));    ASSERT_EQ(GS_TILT, a); ASSERT_EQ(1, d);
    ASSERT(gs_verb_axis(&c, PTZ_DOWN, &a, &d));  ASSERT_EQ(GS_TILT, a); ASSERT_EQ(-1, d);
    c.left_sign = -1;
    c.up_sign = -1;
    ASSERT(gs_verb_axis(&c, PTZ_LEFT, &a, &d));  ASSERT_EQ(-1, d);
    ASSERT(gs_verb_axis(&c, PTZ_DOWN, &a, &d));  ASSERT_EQ(1, d);
    PASS();
}

// A pan/tilt head carries no zoom, focus or ICR verb, and stop is not a move.
TEST verbs_a_head_does_not_carry(void) {
    GsConfig c;
    gs_config_defaults(&c);
    int a, d;
    ASSERT_FALSE(gs_verb_axis(&c, PTZ_STOP, &a, &d));
    ASSERT_FALSE(gs_verb_axis(&c, PTZ_NEAR, &a, &d));
    ASSERT_FALSE(gs_verb_axis(&c, PTZ_FAR, &a, &d));
    ASSERT_FALSE(gs_verb_axis(&c, PTZ_TELE, &a, &d));
    ASSERT_FALSE(gs_verb_axis(&c, PTZ_WIDE, &a, &d));
    ASSERT_FALSE(gs_verb_axis(&c, PTZ_DAY, &a, &d));
    ASSERT_FALSE(gs_verb_axis(&c, PTZ_NIGHT, &a, &d));
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
    RUN_TEST(config_refuses_nonsense);
    RUN_TEST(verbs_follow_the_board_signs);
    RUN_TEST(verbs_a_head_does_not_carry);
    RUN_TEST(clamp_stops_short_of_the_stops);
    RUN_TEST(clamp_without_travel_allows_all);
    RUN_TEST(saved_position_is_per_axis);
}
