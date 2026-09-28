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

SUITE(actuator_suite) {
    RUN_TEST(verb_axis_maps_focus_and_zoom);
    RUN_TEST(verb_axis_rejects_nonmotion);
    RUN_TEST(clamp_never_passes_a_stop);
    RUN_TEST(zoom_mag_endpoints_and_monotonic);
    RUN_TEST(mechanics_are_derived_from_the_cadence);
}
