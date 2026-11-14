// The VISCA backend's pure logic, pinned: the bytes for each verb (as the stock
// N51813L firmware sends them, captured from Sofia's own writes), the reply
// parser, and the magnification curve measured on that zoom block.

#include <greatest.h>
#include <string.h>

#include "act_visca.h"

#define EXPECT_BYTES(exp, got, n)                                     \
    do {                                                              \
        ASSERT_EQ((int)sizeof(exp), (n));                             \
        ASSERT_MEM_EQ((exp), (got), (size_t)(n));                     \
    } while (0)

TEST zoom_from_auto_is_a_plain_move(void) {
    unsigned char f[VISCA_FRAME_MAX];
    bool manual = true;
    static const unsigned char tele[] = {0x81, 0x01, 0x04, 0x07, 0x25, 0xff};
    int n = visca_frame(PTZ_TELE, 5, false, &manual, f);
    EXPECT_BYTES(tele, f, n);
    ASSERT_FALSE(manual);
    static const unsigned char wide[] = {0x81, 0x01, 0x04, 0x07, 0x35, 0xff};
    n = visca_frame(PTZ_WIDE, 5, false, &manual, f);
    EXPECT_BYTES(wide, f, n);
    PASS();
}

// After a manual focus the stock firmware hands focus back to the lens first.
TEST zoom_from_manual_restores_autofocus(void) {
    unsigned char f[VISCA_FRAME_MAX];
    bool manual = true;
    static const unsigned char exp[] = {0x81, 0x01, 0x04, 0x38, 0x02, 0xff,
                                        0x81, 0x01, 0x04, 0x07, 0x35, 0xff};
    int n = visca_frame(PTZ_WIDE, 5, true, &manual, f);
    EXPECT_BYTES(exp, f, n);
    ASSERT_FALSE(manual);
    PASS();
}

TEST focus_goes_manual_once(void) {
    unsigned char f[VISCA_FRAME_MAX];
    bool manual = false;
    static const unsigned char near1[] = {0x81, 0x01, 0x04, 0x38, 0x03, 0xff,
                                          0x81, 0x01, 0x04, 0x08, 0x35, 0xff};
    int n = visca_frame(PTZ_NEAR, 5, false, &manual, f);
    EXPECT_BYTES(near1, f, n);
    ASSERT(manual);
    static const unsigned char far2[] = {0x81, 0x01, 0x04, 0x08, 0x25, 0xff};
    n = visca_frame(PTZ_FAR, 5, true, &manual, f);
    EXPECT_BYTES(far2, f, n);
    ASSERT(manual);
    PASS();
}

TEST stop_halts_both_drives(void) {
    unsigned char f[VISCA_FRAME_MAX];
    bool manual = true;
    static const unsigned char exp[] = {0x81, 0x01, 0x04, 0x07, 0x00, 0xff,
                                        0x81, 0x01, 0x04, 0x08, 0x00, 0xff};
    int n = visca_frame(PTZ_STOP, 5, true, &manual, f);
    EXPECT_BYTES(exp, f, n);
    ASSERT(manual);   // a stop leaves the focus mode alone
    PASS();
}

TEST pan_tilt_and_ir_are_not_carried(void) {
    unsigned char f[VISCA_FRAME_MAX];
    ASSERT_EQ(0, visca_frame(PTZ_LEFT, 5, false, NULL, f));
    ASSERT_EQ(0, visca_frame(PTZ_DAY, 5, false, NULL, f));
    ASSERT_FALSE(visca_has(PTZ_UP));
    ASSERT(visca_has(PTZ_TELE));
    PASS();
}

TEST speed_out_of_range_falls_back(void) {
    unsigned char f[VISCA_FRAME_MAX];
    int n = visca_frame(PTZ_TELE, 40, false, NULL, f);
    ASSERT_EQ(6, n);
    ASSERT_EQ(0x20 | VISCA_SPEED_DEFAULT, f[4]);
    PASS();
}

TEST parses_zoom_position_reply(void) {
    static const unsigned char r[] = {0x90, 0x50, 0x01, 0x0f, 0x0f, 0x07, 0xff};
    int pos = -1, mag = -1;
    ASSERT_EQ(1, visca_parse(r, sizeof r, &pos, &mag));
    ASSERT_EQ(0x1ff7, pos);
    PASS();
}

TEST parses_mcu_status_magnification(void) {
    static const unsigned char s[] = {0xbb, 0x82, 0x00, 0x37, 0x03, 0xde, 0x00,
                                      0x12, 0x00, 0x64, 0x00, 0x03, 0xff};
    int pos = -1, mag = -1;
    ASSERT_EQ(2, visca_parse(s, sizeof s, &pos, &mag));
    ASSERT_EQ(18, mag);
    PASS();
}

TEST ignores_acks_and_noise(void) {
    static const unsigned char ack[] = {0x90, 0x41, 0xff};
    static const unsigned char done[] = {0x90, 0x51, 0xff};
    static const unsigned char junk[] = {0x12, 0x34};
    ASSERT_EQ(0, visca_parse(ack, sizeof ack, NULL, NULL));
    ASSERT_EQ(0, visca_parse(done, sizeof done, NULL, NULL));
    ASSERT_EQ(0, visca_parse(junk, sizeof junk, NULL, NULL));
    PASS();
}

TEST magnification_curve(void) {
    ASSERT_IN_RANGE(1.0f, visca_pos_to_mag(0), 0.001f);
    ASSERT_IN_RANGE(9.0f, visca_pos_to_mag(0x1ff7), 0.001f);
    ASSERT_IN_RANGE(18.0f, visca_pos_to_mag(0x4000), 0.001f);   // past the last point
    float mid = visca_pos_to_mag(0x2400);
    ASSERT(mid > 9.0f && mid < 12.0f);
    float prev = 0;
    for (int p = 0; p <= 0x4000; p += 0x200) {   // monotone over the travel
        float m = visca_pos_to_mag(p);
        ASSERT(m >= prev);
        prev = m;
    }
    PASS();
}

TEST status_carries_the_mcu_gain_and_shutter(void) {
    static const unsigned char s[] = {0xbb, 0x82, 0x01, 0x3e, 0x03, 0xde, 0x00,
                                      0x01, 0x00, 0x64, 0x00, 0x03, 0xff};
    int gain = -1, shutter = -1;
    ASSERT(visca_status(s, sizeof s, &gain, &shutter));
    ASSERT_EQ(0x13e, gain);
    ASSERT_EQ(0x3de, shutter);
    static const unsigned char ack[] = {0x90, 0x41, 0xff};
    ASSERT_FALSE(visca_status(ack, sizeof ack, &gain, &shutter));
    PASS();
}

TEST gain_word_maps_to_a_rising_iso(void) {
    ASSERT_EQ(100u, visca_gain_to_iso(0));
    ASSERT_EQ(1000u, visca_gain_to_iso(130));  // a decade
    // The calibration points: stock's ISP at 800 by word 98, ~4000 by 209.
    ASSERT(visca_gain_to_iso(98) > 500 && visca_gain_to_iso(98) < 800);
    ASSERT(visca_gain_to_iso(209) > 3000 && visca_gain_to_iso(209) < 5000);
    ASSERT_EQ(visca_gain_to_iso(391), visca_gain_to_iso(600));   // capped
    unsigned prev = 0;
    for (int g = 0; g <= 0x200; g += 8) {
        unsigned iso = visca_gain_to_iso(g);
        ASSERT(iso >= prev);
        prev = iso;
    }
    PASS();
}

SUITE(visca_suite) {
    RUN_TEST(zoom_from_auto_is_a_plain_move);
    RUN_TEST(zoom_from_manual_restores_autofocus);
    RUN_TEST(focus_goes_manual_once);
    RUN_TEST(stop_halts_both_drives);
    RUN_TEST(pan_tilt_and_ir_are_not_carried);
    RUN_TEST(speed_out_of_range_falls_back);
    RUN_TEST(parses_zoom_position_reply);
    RUN_TEST(parses_mcu_status_magnification);
    RUN_TEST(ignores_acks_and_noise);
    RUN_TEST(magnification_curve);
    RUN_TEST(status_carries_the_mcu_gain_and_shutter);
    RUN_TEST(gain_word_maps_to_a_rising_iso);
}
