// The MS41908M backend's pure logic, pinned: the verb->axis mapping the stepping
// thread reads, the soft-limit clamp that keeps the motor off the end stops, and
// the zoom-position->magnification curve that feeds af2's parfocal target. No
// hardware — this is the SPI backend's proto_test.

#include <greatest.h>

#include "act_ms41908.h"

// near/far drive focus, tele/wide drive zoom, with the right sign.
TEST verb_axis_maps_focus_and_zoom(void) {
    bool z = true;
    int d = 0;
    ASSERT(ms_verb_axis(PTZ_NEAR, &z, &d)); ASSERT_FALSE(z); ASSERT_EQ(-1, d);
    ASSERT(ms_verb_axis(PTZ_FAR, &z, &d));  ASSERT_FALSE(z); ASSERT_EQ(1, d);
    // TELE zooms IN toward the home (max-zoom) stop, so dir < 0; WIDE zooms OUT.
    ASSERT(ms_verb_axis(PTZ_TELE, &z, &d)); ASSERT(z);       ASSERT_EQ(-1, d);
    ASSERT(ms_verb_axis(PTZ_WIDE, &z, &d)); ASSERT(z);       ASSERT_EQ(1, d);
    PASS();
}

// stop and the verbs this lens has no motor for are not motion.
TEST verb_axis_rejects_nonmotion(void) {
    bool z;
    int d;
    ASSERT_FALSE(ms_verb_axis(PTZ_STOP, &z, &d));
    ASSERT_FALSE(ms_verb_axis(PTZ_UP, &z, &d));
    ASSERT_FALSE(ms_verb_axis(PTZ_DOWN, &z, &d));
    ASSERT_FALSE(ms_verb_axis(PTZ_LEFT, &z, &d));
    ASSERT_FALSE(ms_verb_axis(PTZ_RIGHT, &z, &d));
    ASSERT_FALSE(ms_verb_axis(PTZ_DAY, &z, &d));
    ASSERT_FALSE(ms_verb_axis(PTZ_NIGHT, &z, &d));
    PASS();
}

// The clamp never lets a burst carry the position past a stop, from either end.
TEST clamp_never_passes_a_stop(void) {
    // Driving toward FAR/TELE (the top): full burst mid-travel, the remainder at
    // the edge, zero AT the edge.
    ASSERT_EQ(8, ms_clamp_step(100, 1, 8, MS_FOCUS_MAX));
    ASSERT_EQ(5, ms_clamp_step(MS_FOCUS_MAX - 5, 1, 8, MS_FOCUS_MAX));
    ASSERT_EQ(0, ms_clamp_step(MS_FOCUS_MAX, 1, 8, MS_FOCUS_MAX));
    // Driving toward NEAR/WIDE (zero): symmetric.
    ASSERT_EQ(8, ms_clamp_step(100, -1, 8, MS_ZOOM_MAX));
    ASSERT_EQ(3, ms_clamp_step(3, -1, 8, MS_ZOOM_MAX));
    ASSERT_EQ(0, ms_clamp_step(0, -1, 8, MS_ZOOM_MAX));
    // A stop verb (dir 0) never steps.
    ASSERT_EQ(0, ms_clamp_step(100, 0, 8, MS_FOCUS_MAX));
    PASS();
}

// The span clamp keeps MS_STOP_CLEAR off the home stop, and lets an axis just homed onto
// the stop (below the bar) move away from it but never further in.
TEST clamp_span_keeps_clear_of_the_home_stop(void) {
    ASSERT_EQ(40, ms_clamp_span(500, -1, 40, MS_STOP_CLEAR, 1240));
    ASSERT_EQ(20, ms_clamp_span(MS_STOP_CLEAR + 20, -1, 40, MS_STOP_CLEAR, 1240));
    ASSERT_EQ(0, ms_clamp_span(MS_STOP_CLEAR, -1, 40, MS_STOP_CLEAR, 1240));
    ASSERT_EQ(0, ms_clamp_span(0, -1, 40, MS_STOP_CLEAR, 1240));      // on the stop
    ASSERT_EQ(40, ms_clamp_span(0, 1, 40, MS_STOP_CLEAR, 1240));      // ... leaves it
    ASSERT_EQ(10, ms_clamp_span(1230, 1, 40, MS_STOP_CLEAR, 1240));
    ASSERT_EQ(0, ms_clamp_span(1240, 1, 40, MS_STOP_CLEAR, 1240));
    PASS();
}

// A seek at open with a saved position stops a margin past it, never past a whole
// travel; with none (or a figure off the lens) it is the whole travel.
TEST home_seek_is_bounded_by_the_saved_position(void) {
    ASSERT_EQ(3000, ms_home_seek(-1, 3000));
    ASSERT_EQ(3000, ms_home_seek(3001, 3000));
    ASSERT_EQ(300, ms_home_seek(0, 3000));            // on the stop: a tenth of travel
    ASSERT_EQ(375 + 93 + 300, ms_home_seek(375, 3000));
    ASSERT_EQ(3000, ms_home_seek(2500, 3000));        // capped at the travel
    for (int k = 0; k <= 3000; k += 50) {
        int s = ms_home_seek(k, 3000);
        ASSERT(s >= k && s <= 3000);                   // reaches the stop from k
    }
    PASS();
}

// With no origin an axis moves toward its home stop only.
TEST unhomed_moves_toward_home_only(void) {
    ASSERT_EQ(40, ms_unhomed_step(-1, 40));
    ASSERT_EQ(0, ms_unhomed_step(1, 40));
    ASSERT_EQ(0, ms_unhomed_step(0, 40));
    ASSERT_EQ(0, ms_unhomed_step(-1, 0));
    PASS();
}

// The application note's own example: OSCIN 27 MHz, a 60 Hz VD, PSUM 53 -> INTCT 354.
TEST burst_intct_fills_the_period(void) {
    ASSERT_EQ(353, ms_burst_intct(53, 27000000u, 1000000L / 60));   // 353.7, floored
    // A burst sized for the cadence turns for MS_ROT_US, whatever its size.
    for (int n = 1; n <= MS_STEP_BURST; n++) {
        int psum = 4 * n, intct = ms_burst_intct(psum, 37125000u, MS_ROT_US);
        long rot_us = (long)(24ull * intct * psum * 1000000ull / 37125000u);
        ASSERT(rot_us <= MS_ROT_US && rot_us > MS_ROT_US - 2000);
    }
    ASSERT_EQ(65535, ms_burst_intct(1, MS_OSCIN_MAX_HZ, MS_ROT_US));   // clamped
    ASSERT_EQ(1, ms_burst_intct(0, 27000000u, MS_ROT_US));
    PASS();
}

// The clock read back from a timed rotation; and an assumed clock below the real one only
// ever shortens a rotation, so it can never overrun the period and be cancelled.
TEST oscin_from_rotation_round_trips(void) {
    int intct = ms_burst_intct(160, 27000000u, MS_ROT_US);
    unsigned hz = ms_oscin_from_rotation(intct, 160, MS_ROT_US);
    ASSERT(hz > 26900000u && hz <= 27000000u);
    ASSERT_EQ(0u, ms_oscin_from_rotation(intct, 160, 1000));      // a start edge, not an end
    ASSERT_EQ(0u, ms_oscin_from_rotation(intct, 160, 10000000));  // far too slow
    unsigned real[] = {24000000u, 27000000u, 37125000u, 74250000u};
    for (unsigned i = 0; i < sizeof real / sizeof *real; i++) {
        int c = ms_burst_intct(160, MS_OSCIN_MIN_HZ, MS_ROT_US);
        long rot_us = (long)(24ull * c * 160 * 1000000ull / real[i]);
        ASSERT(rot_us <= MS_ROT_US);
    }
    PASS();
}

// The lens record: only a lens at rest, of this travel and wiring, inside it, is trusted.
TEST lens_state_round_trips_and_trusts_only_rest(void) {
    MsLens l = {3320, 3000, 20.0f, false};
    char buf[96];
    int z = -1, f = -1;
    ms_lens_state_format(buf, sizeof buf, 3079, 375, &l, false, true);
    ASSERT_STR_EQ("3079 375 3320 3000 0 rest\n", buf);
    ASSERT(ms_lens_state_parse(buf, &l, false, &z, &f));
    ASSERT_EQ(3079, z);
    ASSERT_EQ(375, f);
    ms_lens_state_format(buf, sizeof buf, 3079, 375, &l, false, false);
    ASSERT_FALSE(ms_lens_state_parse(buf, &l, false, &z, &f));           // mid-move
    ASSERT_FALSE(ms_lens_state_parse("3079 375 3320 3000 0 rest", &l, true, &z, &f));  // rewired
    MsLens other = {3320, 1280, 20.0f, false};
    ASSERT_FALSE(ms_lens_state_parse("3079 375 3320 3000 0 rest", &other, false, &z, &f));
    ASSERT_FALSE(ms_lens_state_parse("3079 3001 3320 3000 0 rest", &l, false, &z, &f));
    ASSERT_FALSE(ms_lens_state_parse("-1 375 3320 3000 0 rest", &l, false, &z, &f));
    ASSERT_FALSE(ms_lens_state_parse("garbage", &l, false, &z, &f));
    ASSERT_FALSE(ms_lens_state_parse(NULL, &l, false, &z, &f));
    PASS();
}

// The zoom->mag curve hits its endpoints, clamps beyond them, and only falls:
// pos 0 is the max-zoom (TELE) home stop, pos MS_ZOOM_MAX is fully WIDE.
TEST zoom_mag_endpoints_and_monotonic(void) {
    ASSERT_IN_RANGE(MS_MAG_MAX, ms_zoom_mag(0), 0.001f);
    ASSERT_IN_RANGE(MS_MAG_MIN, ms_zoom_mag(MS_ZOOM_MAX), 0.001f);
    ASSERT_IN_RANGE(MS_MAG_MAX, ms_zoom_mag(-50), 0.001f);              // clamped low
    ASSERT_IN_RANGE(MS_MAG_MIN, ms_zoom_mag(MS_ZOOM_MAX + 50), 0.001f); // clamped high

    float prev = ms_zoom_mag(0);
    for (int p = 50; p <= MS_ZOOM_MAX; p += 50) {
        float m = ms_zoom_mag(p);
        ASSERT(m <= prev);
        prev = m;
    }
    float mid = ms_zoom_mag(MS_ZOOM_MAX / 2);
    ASSERT(mid > MS_MAG_MIN && mid < MS_MAG_MAX);

    // Round-trips through the inverse (used to seed the position from a restored
    // magnification): the ends map to the right stops, and a mid value returns.
    ASSERT_EQ(0, ms_zoom_pos_for_mag(MS_MAG_MAX));
    ASSERT_EQ(MS_ZOOM_MAX, ms_zoom_pos_for_mag(MS_MAG_MIN));
    int p = ms_zoom_pos_for_mag(mid);
    ASSERT(p > 0 && p < MS_ZOOM_MAX);
    PASS();
}

// The focus mechanics af2 is given must be DERIVED from the burst cadence, not a
// separate hand-set number: a 13 ms model against a slower real burst is exactly
// what would make af2's timed cold seek stop short of the near stop and anchor a
// wrong position. Pin the derivation so the two cannot drift apart.
TEST mechanics_are_derived_from_the_cadence(void) {
    ASSERT(MS_STEP_BURST > 0);
    ASSERT(MS_BURST_MS > 0);
    ASSERT_EQ((MS_FOCUS_MAX / MS_STEP_BURST) * MS_BURST_MS, (int)MS_TRAVEL_MS);
    ASSERT(MS_TRAVEL_MAX_MS > MS_TRAVEL_MS);   // the cold-seek cap sits above the travel
    PASS();
}

// Another lens on the same chip (the 20x speed dome): a longer travel and a
// higher ratio scale the curve, and a reversed lens has WIDE at the home stop.
TEST a_lens_of_its_own_scales_and_reverses(void) {
    MsLens l = {5000, 2000, 20.0f, false};
    ASSERT_IN_RANGE(20.0f, ms_lens_zoom_mag(&l, 0), 0.001f);
    ASSERT_IN_RANGE(MS_MAG_MIN, ms_lens_zoom_mag(&l, 5000), 0.001f);
    ASSERT_EQ(0, ms_lens_zoom_pos_for_mag(&l, 20.0f));
    ASSERT_EQ(5000, ms_lens_zoom_pos_for_mag(&l, MS_MAG_MIN));
    bool z;
    int d;
    ASSERT(ms_lens_verb_axis(&l, PTZ_TELE, &z, &d) && z && d == -1);
    l.zoom_reversed = true;
    ASSERT_IN_RANGE(MS_MAG_MIN, ms_lens_zoom_mag(&l, 0), 0.001f);   // home is wide
    ASSERT_IN_RANGE(20.0f, ms_lens_zoom_mag(&l, 5000), 0.001f);
    ASSERT_EQ(5000, ms_lens_zoom_pos_for_mag(&l, 20.0f));
    ASSERT(ms_lens_verb_axis(&l, PTZ_TELE, &z, &d) && z && d == +1);
    ASSERT(ms_lens_verb_axis(&l, PTZ_WIDE, &z, &d) && z && d == -1);
    ASSERT(ms_lens_verb_axis(&l, PTZ_NEAR, &z, &d) && !z && d == -1);   // focus as before
    // Round trip at a middle ratio.
    int p = ms_lens_zoom_pos_for_mag(&l, 10.0f);
    ASSERT_IN_RANGE(10.0f, ms_lens_zoom_mag(&l, p), 0.01f);
    PASS();
}

// The default lens through the new helpers is the old one exactly.
TEST the_default_lens_is_the_lh13(void) {
    MsLens l = MS_LENS_DEFAULT;
    for (int p = 0; p <= MS_ZOOM_MAX; p += 87) {
        ASSERT_IN_RANGE(ms_zoom_mag(p), ms_lens_zoom_mag(&l, p), 0.0001f);
    }
    ASSERT_EQ(ms_zoom_pos_for_mag(5.5f), ms_lens_zoom_pos_for_mag(&l, 5.5f));
    PASS();
}

SUITE(actuator_suite) {
    RUN_TEST(a_lens_of_its_own_scales_and_reverses);
    RUN_TEST(the_default_lens_is_the_lh13);
    RUN_TEST(verb_axis_maps_focus_and_zoom);
    RUN_TEST(verb_axis_rejects_nonmotion);
    RUN_TEST(clamp_never_passes_a_stop);
    RUN_TEST(clamp_span_keeps_clear_of_the_home_stop);
    RUN_TEST(home_seek_is_bounded_by_the_saved_position);
    RUN_TEST(unhomed_moves_toward_home_only);
    RUN_TEST(burst_intct_fills_the_period);
    RUN_TEST(oscin_from_rotation_round_trips);
    RUN_TEST(lens_state_round_trips_and_trusts_only_rest);
    RUN_TEST(zoom_mag_endpoints_and_monotonic);
    RUN_TEST(mechanics_are_derived_from_the_cadence);
}
