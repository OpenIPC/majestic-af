// The wire, pinned.
//
// Every frame here is a literal transcript of what the WebUI's btzoom and
// btzoom-xm shell scripts put on the UART before majestic took the port over
// (majestic-webui bin/btzoom, bin/btzoom-xm, deleted in the same change that
// added this file). Those scripts are the reference: if these bytes match, a
// camera that worked before works after, and the deletion changed nothing a
// lens can see.
//
// It is also the check that was missing when the frames were a hand-typed
// table. `far` is the one verb whose byte sum passes 100, so it is the only
// frame where the mod-100 and mod-256 checksum rules disagree — every other
// frame is identical under both. A wrong rule was therefore invisible on
// eight frames out of nine, which is exactly how long it took to notice.

#include <greatest.h>

#include "proto.h"

static const PtzProto *XM;
static const PtzProto *D;

// Assert a built frame equals the bytes the script sent.
#define FRAME_EQ(proto, verb, ...)                                             \
    do {                                                                       \
        const unsigned char want[] = {__VA_ARGS__};                            \
        unsigned char got[PTZ_FRAME_MAX];                                      \
        int n = ptz_frame((proto), (verb), 0, got);                            \
        ASSERT_EQ_FMT((int)sizeof want, n, "%d");                              \
        ASSERT_MEM_EQ(want, got, sizeof want);                                 \
    } while (0)

TEST xm_frames_match_btzoom_xm(void) {
    // bin/btzoom-xm: xm_stop, xm_near, xm_far, xm_tele, xm_wide.
    FRAME_EQ(XM, PTZ_STOP, 0xc5, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x5c);
    FRAME_EQ(XM, PTZ_NEAR, 0xc5, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02, 0x5c);
    FRAME_EQ(XM, PTZ_FAR, 0xc5, 0x01, 0x00, 0x80, 0x00, 0x00, 0x1d, 0x5c);
    FRAME_EQ(XM, PTZ_TELE, 0xc5, 0x01, 0x00, 0x20, 0x00, 0x00, 0x21, 0x5c);
    FRAME_EQ(XM, PTZ_WIDE, 0xc5, 0x01, 0x00, 0x40, 0x00, 0x00, 0x41, 0x5c);
    PASS();
}

TEST xm_pan_tilt_keep_the_scripts_speed(void) {
    // xm_send scaled its ±100 argument to 63 and put it in the pan or tilt
    // slot. The verb NAMES are the standard's here and the script's were
    // inverted for pan, so btzoom-xm's xm_left is this xm_right — the bytes
    // are what is being pinned, not the spelling.
    FRAME_EQ(XM, PTZ_RIGHT, 0xc5, 0x01, 0x00, 0x02, 0x3f, 0x00, 0x42, 0x5c);
    FRAME_EQ(XM, PTZ_LEFT, 0xc5, 0x01, 0x00, 0x04, 0x3f, 0x00, 0x44, 0x5c);
    FRAME_EQ(XM, PTZ_UP, 0xc5, 0x01, 0x00, 0x08, 0x00, 0x3f, 0x48, 0x5c);
    FRAME_EQ(XM, PTZ_DOWN, 0xc5, 0x01, 0x00, 0x10, 0x00, 0x3f, 0x50, 0x5c);
    PASS();
}

TEST pelco_d_frames_match_btzoom(void) {
    // bin/btzoom: every pelcoD_* verb, byte for byte.
    FRAME_EQ(D, PTZ_STOP, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01);
    FRAME_EQ(D, PTZ_NEAR, 0xff, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02);
    FRAME_EQ(D, PTZ_FAR, 0xff, 0x01, 0x00, 0x80, 0x00, 0x00, 0x81);
    FRAME_EQ(D, PTZ_TELE, 0xff, 0x01, 0x00, 0x20, 0x00, 0x00, 0x21);
    FRAME_EQ(D, PTZ_WIDE, 0xff, 0x01, 0x00, 0x40, 0x00, 0x00, 0x41);
    FRAME_EQ(D, PTZ_UP, 0xff, 0x01, 0x00, 0x08, 0x00, 0x00, 0x09);
    FRAME_EQ(D, PTZ_DOWN, 0xff, 0x01, 0x00, 0x10, 0x00, 0x00, 0x11);
    FRAME_EQ(D, PTZ_RIGHT, 0xff, 0x01, 0x00, 0x02, 0x00, 0x00, 0x03);
    FRAME_EQ(D, PTZ_LEFT, 0xff, 0x01, 0x00, 0x04, 0x00, 0x00, 0x05);
    FRAME_EQ(D, PTZ_NIGHT, 0xff, 0x01, 0x00, 0x09, 0x00, 0x01, 0x0b);
    FRAME_EQ(D, PTZ_DAY, 0xff, 0x01, 0x00, 0x0b, 0x00, 0x01, 0x0d);
    PASS();
}

TEST far_is_the_only_frame_the_two_rules_disagree_on(void) {
    // The whole reason the checksum mistake survived: sum the address, command
    // and data bytes of every verb and only `far` (0x01 + 0x80 = 129) passes
    // 100. So mod-100 and mod-256 produce the same byte everywhere else, and a
    // camera driven with the wrong rule misbehaves on exactly one button.
    for (int v = 0; v < PTZ_VERB_COUNT; v++) {
        unsigned char xm[PTZ_FRAME_MAX], d[PTZ_FRAME_MAX];
        if (!ptz_proto_has(XM, (enum PtzVerb)v)) {
            continue;
        }
        ptz_frame(XM, (enum PtzVerb)v, 0, xm);
        ptz_frame(D, (enum PtzVerb)v, 0, d);
        // Compare the command/data bytes' checksum only; the wrappers differ.
        int sum = d[1] + d[2] + d[3] + d[4] + d[5];
        if (v == PTZ_FAR) {
            ASSERT_EQ_FMT(129, sum, "%d");
            ASSERT(xm[6] != d[6]);
        } else if (d[4] == 0 && d[5] == 0 && xm[4] == 0 && xm[5] == 0) {
            // Same command bytes, no speed byte on either side: the two rules
            // must agree. (Pan and tilt are skipped — the two scripts sent
            // different speeds, so their sums differ for a reason that has
            // nothing to do with the checksum rule.)
            ASSERT(sum < 100);
            ASSERT_EQ_FMT((int)d[6], (int)xm[6], "%d");
        }
    }
    PASS();
}

TEST speed_lands_in_the_right_slot(void) {
    unsigned char f[PTZ_FRAME_MAX];
    ptz_frame(D, PTZ_LEFT, 32, f);
    ASSERT_EQ_FMT(32, (int)f[4], "%d");            // pan speed is data 1
    ASSERT_EQ_FMT(0, (int)f[5], "%d");
    ASSERT_EQ_FMT((1 + 0x04 + 32) % 256, (int)f[6], "%d");

    ptz_frame(D, PTZ_UP, 32, f);
    ASSERT_EQ_FMT(0, (int)f[4], "%d");
    ASSERT_EQ_FMT(32, (int)f[5], "%d");            // tilt speed is data 2

    ptz_frame(D, PTZ_LEFT, 999, f);
    ASSERT_EQ_FMT(63, (int)f[4], "%d");            // clamped to the Pelco range

    // Zoom and focus carry no speed byte, whatever the caller asks for.
    ptz_frame(D, PTZ_NEAR, 40, f);
    ASSERT_EQ_FMT(0, (int)f[4], "%d");
    ASSERT_EQ_FMT(0, (int)f[5], "%d");
    PASS();
}

TEST presets_and_wake_match_the_lens_tool(void) {
    unsigned char f[PTZ_FRAME_MAX];
    // bin/btzoom pelcoD_start called these two vendor presets.
    ASSERT_EQ_FMT(7, ptz_preset_frame(D, 0x53, f), "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x01, 0x00, 0x07, 0x00, 0x53, 0x5b}),
                  f, 7);
    ASSERT_EQ_FMT(7, ptz_preset_frame(D, 0x52, f), "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x01, 0x00, 0x07, 0x00, 0x52, 0x5a}),
                  f, 7);
    // The XiongMai variant has no preset command.
    ASSERT_EQ_FMT(0, ptz_preset_frame(XM, 0x53, f), "%d");

    size_t n = 0;
    const unsigned char *w = ptz_wake_blob(D, &n);
    ASSERT_EQ_FMT(14, (int)n, "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0x51, 0x01, 0x04, 0x79, 0x01, 0x0d, 0x0a,
                                     0x2e, 0x02, 0x7e, 0x00, 0x00, 0x00, 0x95}),
                  w, 14);
    w = ptz_wake_blob(XM, &n);
    ASSERT_EQ_FMT(8, (int)n, "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0xa5, 0x7b, 0x9e, 0xf0, 0xef, 0xee, 0xe0,
                                     0xf4}),
                  w, 8);
    PASS();
}

TEST the_xm_variant_has_no_day_night(void) {
    // btzoom-xm refused both: the XiongMai MCU reports day/night, it takes no
    // command for them. A verb the protocol lacks must build no frame at all
    // rather than a plausible-looking one.
    unsigned char f[PTZ_FRAME_MAX];
    ASSERT(!ptz_proto_has(XM, PTZ_DAY));
    ASSERT(!ptz_proto_has(XM, PTZ_NIGHT));
    ASSERT_EQ_FMT(0, ptz_frame(XM, PTZ_DAY, 0, f), "%d");
    ASSERT_EQ_FMT(0, ptz_frame(XM, PTZ_NIGHT, 0, f), "%d");
    ASSERT(ptz_proto_has(D, PTZ_DAY));
    PASS();
}

TEST unknown_actuator_falls_back_to_the_default(void) {
    ASSERT_STR_EQ("pelco-xm", ptz_proto_name(ptz_proto(NULL)));
    ASSERT_STR_EQ("pelco-xm", ptz_proto_name(ptz_proto("")));
    ASSERT_STR_EQ("pelco-xm", ptz_proto_name(ptz_proto("nonsense")));
    ASSERT_STR_EQ("pelco-d", ptz_proto_name(ptz_proto("pelco-d")));
    ASSERT_STR_EQ("pelco-xm", ptz_proto_name(ptz_proto("pelco-xm")));
    PASS();
}

TEST verb_names_round_trip_and_reject_everything_else(void) {
    // This is the gate between a request token and the wire: j/ptz.cgi passes a
    // word through, and only the listed ones may become frames.
    for (int v = 0; v < PTZ_VERB_COUNT; v++) {
        enum PtzVerb back;
        const char *name = ptz_verb_name((enum PtzVerb)v);
        ASSERT(name[0] != 0);
        ASSERT(ptz_verb_parse(name, &back));
        ASSERT_EQ_FMT(v, (int)back, "%d");
    }
    ASSERT(!ptz_verb_parse("", NULL));
    ASSERT(!ptz_verb_parse("NEAR", NULL));
    ASSERT(!ptz_verb_parse("near ", NULL));
    ASSERT(!ptz_verb_parse("start", NULL));
    ASSERT(!ptz_verb_parse("../../etc/shadow", NULL));

    ASSERT(ptz_verb_is_focus(PTZ_NEAR) && ptz_verb_is_focus(PTZ_FAR));
    ASSERT(!ptz_verb_is_focus(PTZ_TELE) && !ptz_verb_is_focus(PTZ_STOP));
    ASSERT(ptz_verb_is_zoom(PTZ_TELE) && ptz_verb_is_zoom(PTZ_WIDE));
    ASSERT(!ptz_verb_is_zoom(PTZ_NEAR) && !ptz_verb_is_zoom(PTZ_STOP));
    PASS();
}

SUITE(proto_suite) {
    XM = ptz_proto("pelco-xm");
    D = ptz_proto("pelco-d");
    RUN_TEST(xm_frames_match_btzoom_xm);
    RUN_TEST(xm_pan_tilt_keep_the_scripts_speed);
    RUN_TEST(pelco_d_frames_match_btzoom);
    RUN_TEST(far_is_the_only_frame_the_two_rules_disagree_on);
    RUN_TEST(speed_lands_in_the_right_slot);
    RUN_TEST(presets_and_wake_match_the_lens_tool);
    RUN_TEST(the_xm_variant_has_no_day_night);
    RUN_TEST(unknown_actuator_falls_back_to_the_default);
    RUN_TEST(verb_names_round_trip_and_reject_everything_else);
}
