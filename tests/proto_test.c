// The wire, pinned.
//
// The Pelco-D frames are a literal transcript of what the WebUI's btzoom script
// put on the UART before majestic took the port over (majestic-webui
// bin/btzoom, deleted in the same change that added this file). The XiongMai
// zoom and focus frames are what the stock XM firmware sends its lens board,
// captured on that UART (OpenIPC/motors xm-uart/PROTOCOL.md) — NOT what
// btzoom-xm sent: that script had the focus bits the Pelco-D way round, so its
// `near` focused farther, and a mod-100 checksum the board ignores.

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

TEST xm_frames_match_the_stock_firmware(void) {
    // Stock DVRIP stop, FocusNear, FocusFar, ZoomTile, ZoomWide.
    FRAME_EQ(XM, PTZ_STOP, 0xc5, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01, 0x5c);
    FRAME_EQ(XM, PTZ_NEAR, 0xc5, 0x01, 0x00, 0x80, 0x00, 0x00, 0x81, 0x5c);
    FRAME_EQ(XM, PTZ_FAR, 0xc5, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02, 0x5c);
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

TEST only_focus_differs_between_the_two_protocols(void) {
    // The same verb sets the same command and data bytes on both wires, except
    // near and far, which the XiongMai board reads the other way round: its
    // `near` is Pelco-D's `far` and vice versa. Pan and tilt are skipped — the
    // two scripts sent different speeds, which has nothing to do with the bits.
    for (int v = 0; v < PTZ_VERB_COUNT; v++) {
        unsigned char xm[PTZ_FRAME_MAX], d[PTZ_FRAME_MAX];
        if (!ptz_proto_has(XM, (enum PtzVerb)v) || v == PTZ_UP || v == PTZ_DOWN ||
            v == PTZ_LEFT || v == PTZ_RIGHT) {
            continue;
        }
        enum PtzVerb dv = (enum PtzVerb)v;
        if (v == PTZ_NEAR) {
            dv = PTZ_FAR;
        } else if (v == PTZ_FAR) {
            dv = PTZ_NEAR;
        }
        ptz_frame(XM, (enum PtzVerb)v, 0, xm);
        ptz_frame(D, dv, 0, d);
        ASSERT_MEM_EQ(d + 1, xm + 1, 6);   // address, commands, data, checksum
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
    RUN_TEST(xm_frames_match_the_stock_firmware);
    RUN_TEST(xm_pan_tilt_keep_the_scripts_speed);
    RUN_TEST(pelco_d_frames_match_btzoom);
    RUN_TEST(only_focus_differs_between_the_two_protocols);
    RUN_TEST(speed_lands_in_the_right_slot);
    RUN_TEST(presets_and_wake_match_the_lens_tool);
    RUN_TEST(the_xm_variant_has_no_day_night);
    RUN_TEST(unknown_actuator_falls_back_to_the_default);
    RUN_TEST(verb_names_round_trip_and_reject_everything_else);
}
