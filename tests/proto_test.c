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

#include <limits.h>

#include "proto.h"

static const PtzProto *XM;
static const PtzProto *D;

// Assert a built frame equals the bytes the script sent.
#define FRAME_EQ(proto, verb, ...)                                             \
    do {                                                                       \
        const unsigned char want[] = {__VA_ARGS__};                            \
        unsigned char got[PTZ_FRAME_MAX];                                      \
        int n = ptz_frame((proto), PTZ_ADDR_DEFAULT, (verb), 0, got);         \
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

// The stock firmware's frames at each DVRIP Step, captured on the lens-board
// UART of an 85H50AI (V5.00.R02.000529B2) by uart-bridge, 2026-10-08: the
// pan/tilt speed byte is round(Step * 63 / 8). Given that byte, a frame here
// is the stock one byte for byte, checksum included; zoom and focus frames
// carry no speed at any Step, and do not here.
TEST xm_speed_bytes_match_the_stock_firmware(void) {
    static const unsigned char by_step[8] = {0x08, 0x10, 0x18, 0x20, 0x27, 0x2f, 0x37, 0x3f};
    static const unsigned char left_ck[8] = {0x0d, 0x15, 0x1d, 0x25, 0x2c, 0x34, 0x3c, 0x44};
    static const unsigned char up_ck[8] = {0x11, 0x19, 0x21, 0x29, 0x30, 0x38, 0x40, 0x48};
    for (int s = 0; s < 8; s++) {
        unsigned char got[PTZ_FRAME_MAX];
        const unsigned char left[] = {0xc5, 0x01, 0x00, 0x04, by_step[s], 0x00, left_ck[s], 0x5c};
        ASSERT_EQ(8, ptz_frame(XM, PTZ_ADDR_DEFAULT, PTZ_LEFT, by_step[s], got));
        ASSERT_MEM_EQ(left, got, sizeof left);
        const unsigned char up[] = {0xc5, 0x01, 0x00, 0x08, 0x00, by_step[s], up_ck[s], 0x5c};
        ASSERT_EQ(8, ptz_frame(XM, PTZ_ADDR_DEFAULT, PTZ_UP, by_step[s], got));
        ASSERT_MEM_EQ(up, got, sizeof up);
        const unsigned char tele[] = {0xc5, 0x01, 0x00, 0x20, 0x00, 0x00, 0x21, 0x5c};
        ASSERT_EQ(8, ptz_frame(XM, PTZ_ADDR_DEFAULT, PTZ_TELE, by_step[s], got));
        ASSERT_MEM_EQ(tele, got, sizeof tele);
        const unsigned char near[] = {0xc5, 0x01, 0x00, 0x80, 0x00, 0x00, 0x81, 0x5c};
        ASSERT_EQ(8, ptz_frame(XM, PTZ_ADDR_DEFAULT, PTZ_NEAR, by_step[s], got));
        ASSERT_MEM_EQ(near, got, sizeof near);
    }
    // Step 0, a missing Step and a non-integer one: the stock sends 01.
    unsigned char got[PTZ_FRAME_MAX];
    const unsigned char slowest[] = {0xc5, 0x01, 0x00, 0x04, 0x01, 0x00, 0x06, 0x5c};
    ASSERT_EQ(8, ptz_frame(XM, PTZ_ADDR_DEFAULT, PTZ_LEFT, 1, got));
    ASSERT_MEM_EQ(slowest, got, sizeof slowest);
    PASS();
}

TEST pelco_d_frames_match_btzoom(void) {
    // bin/btzoom: every pelcoD_* verb that carries no speed, byte for byte.
    FRAME_EQ(D, PTZ_STOP, 0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0x01);
    FRAME_EQ(D, PTZ_NEAR, 0xff, 0x01, 0x01, 0x00, 0x00, 0x00, 0x02);
    FRAME_EQ(D, PTZ_FAR, 0xff, 0x01, 0x00, 0x80, 0x00, 0x00, 0x81);
    FRAME_EQ(D, PTZ_TELE, 0xff, 0x01, 0x00, 0x20, 0x00, 0x00, 0x21);
    FRAME_EQ(D, PTZ_WIDE, 0xff, 0x01, 0x00, 0x40, 0x00, 0x00, 0x41);
    FRAME_EQ(D, PTZ_NIGHT, 0xff, 0x01, 0x00, 0x09, 0x00, 0x01, 0x0b);
    FRAME_EQ(D, PTZ_DAY, 0xff, 0x01, 0x00, 0x0b, 0x00, 0x01, 0x0d);
    PASS();
}

TEST pelco_d_pan_tilt_default_to_half_speed(void) {
    // A move that names no speed runs at 0x20, half of Pelco-D's 00..3F, in
    // the byte of the axis it moves. btzoom sent 00 here -- the stop end of the
    // range -- and a speed dome that follows the standard barely moved for it
    // (OpenIPC/majestic-webui#691). The direction bits are btzoom's.
    FRAME_EQ(D, PTZ_UP, 0xff, 0x01, 0x00, 0x08, 0x00, 0x20, 0x29);
    FRAME_EQ(D, PTZ_DOWN, 0xff, 0x01, 0x00, 0x10, 0x00, 0x20, 0x31);
    FRAME_EQ(D, PTZ_RIGHT, 0xff, 0x01, 0x00, 0x02, 0x20, 0x00, 0x23);
    FRAME_EQ(D, PTZ_LEFT, 0xff, 0x01, 0x00, 0x04, 0x20, 0x00, 0x25);
    PASS();
}

TEST only_focus_differs_between_the_two_protocols(void) {
    // The same verb sets the same command and data bytes on both wires, except
    // near and far, which the XiongMai board reads the other way round: its
    // `near` is Pelco-D's `far` and vice versa. Pan and tilt are skipped — the
    // two scripts sent different speeds, which has nothing to do with the bits.
    for (int v = 0; v < PTZ_VERB_COUNT; v++) {
        unsigned char xm[PTZ_FRAME_MAX], d[PTZ_FRAME_MAX];
        if (!ptz_proto_has(XM, (enum PtzVerb)v) || ptz_verb_is_pantilt((enum PtzVerb)v)) {
            continue;
        }
        enum PtzVerb dv = (enum PtzVerb)v;
        if (v == PTZ_NEAR) {
            dv = PTZ_FAR;
        } else if (v == PTZ_FAR) {
            dv = PTZ_NEAR;
        }
        ptz_frame(XM, PTZ_ADDR_DEFAULT, (enum PtzVerb)v, 0, xm);
        ptz_frame(D, PTZ_ADDR_DEFAULT, dv, 0, d);
        ASSERT_MEM_EQ(d + 1, xm + 1, 6);   // address, commands, data, checksum
    }
    PASS();
}

TEST speed_lands_in_the_right_slot(void) {
    unsigned char f[PTZ_FRAME_MAX];
    ptz_frame(D, PTZ_ADDR_DEFAULT, PTZ_LEFT, 32, f);
    ASSERT_EQ_FMT(32, (int)f[4], "%d");            // pan speed is data 1
    ASSERT_EQ_FMT(0, (int)f[5], "%d");
    ASSERT_EQ_FMT((1 + 0x04 + 32) % 256, (int)f[6], "%d");

    ptz_frame(D, PTZ_ADDR_DEFAULT, PTZ_UP, 32, f);
    ASSERT_EQ_FMT(0, (int)f[4], "%d");
    ASSERT_EQ_FMT(32, (int)f[5], "%d");            // tilt speed is data 2

    ptz_frame(D, PTZ_ADDR_DEFAULT, PTZ_LEFT, 999, f);
    ASSERT_EQ_FMT(63, (int)f[4], "%d");            // clamped to the Pelco range

    // Zoom and focus carry no speed byte, whatever the caller asks for.
    ptz_frame(D, PTZ_ADDR_DEFAULT, PTZ_NEAR, 40, f);
    ASSERT_EQ_FMT(0, (int)f[4], "%d");
    ASSERT_EQ_FMT(0, (int)f[5], "%d");
    PASS();
}

TEST presets_and_wake_match_the_lens_tool(void) {
    unsigned char f[PTZ_FRAME_MAX];
    // bin/btzoom pelcoD_start called these two vendor presets.
    ASSERT_EQ_FMT(7, ptz_preset_frame(D, PTZ_ADDR_DEFAULT, 0x53, f), "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x01, 0x00, 0x07, 0x00, 0x53, 0x5b}),
                  f, 7);
    ASSERT_EQ_FMT(7, ptz_preset_frame(D, PTZ_ADDR_DEFAULT, 0x52, f), "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x01, 0x00, 0x07, 0x00, 0x52, 0x5a}),
                  f, 7);
    // The XiongMai variant has no preset command.
    ASSERT_EQ_FMT(0, ptz_preset_frame(XM, PTZ_ADDR_DEFAULT, 0x53, f), "%d");

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

// ptz.address: the byte after the sync, and inside the checksum. A head
// strapped to another address ignores a frame whose address is not its own,
// and one whose checksum leaves the address out, so both are pinned.
TEST the_address_is_the_second_byte_and_is_summed(void) {
    unsigned char f[PTZ_FRAME_MAX];
    ASSERT_EQ_FMT(7, ptz_frame(D, 5, PTZ_STOP, 0, f), "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x05, 0x00, 0x00, 0x00, 0x00, 0x05}),
                  f, 7);
    // 0xfe + 0x02 + 0x20 passes 256: the checksum wraps, the address does not.
    ASSERT_EQ_FMT(7, ptz_frame(D, 0xfe, PTZ_RIGHT, 0x20, f), "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0xfe, 0x00, 0x02, 0x20, 0x00, 0x20}),
                  f, 7);
    ASSERT_EQ_FMT(7, ptz_preset_frame(D, 5, 0x53, f), "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x05, 0x00, 0x07, 0x00, 0x53, 0x5f}),
                  f, 7);
    // The XiongMai variant carries the same byte in the same place.
    ASSERT_EQ_FMT(8, ptz_frame(XM, 2, PTZ_TELE, 0, f), "%d");
    ASSERT_MEM_EQ(((unsigned char[]){0xc5, 0x02, 0x00, 0x20, 0x00, 0x00, 0x22,
                                     0x5c}),
                  f, 8);
    PASS();
}

// What reaches build() is a byte: a configured value that is not one is the
// default, not whatever it would wrap to, and the caller is told.
TEST only_a_pelco_address_is_used_as_one(void) {
    bool ok = false;
    ASSERT_EQ(0, ptz_addr(0, &ok));
    ASSERT(ok);
    ASSERT_EQ(255, ptz_addr(255, &ok));
    ASSERT(ok);
    ASSERT_EQ(PTZ_ADDR_DEFAULT, ptz_addr(256, &ok));
    ASSERT_FALSE(ok);
    ASSERT_EQ(PTZ_ADDR_DEFAULT, ptz_addr(-1, &ok));
    ASSERT_FALSE(ok);
    ASSERT_EQ(PTZ_ADDR_DEFAULT, ptz_addr(INT_MIN, NULL));
    PASS();
}

TEST the_xm_variant_has_no_day_night(void) {
    // btzoom-xm refused both: the XiongMai MCU reports day/night, it takes no
    // command for them. A verb the protocol lacks must build no frame at all
    // rather than a plausible-looking one.
    unsigned char f[PTZ_FRAME_MAX];
    ASSERT(!ptz_proto_has(XM, PTZ_DAY));
    ASSERT(!ptz_proto_has(XM, PTZ_NIGHT));
    ASSERT_EQ_FMT(0, ptz_frame(XM, PTZ_ADDR_DEFAULT, PTZ_DAY, 0, f), "%d");
    ASSERT_EQ_FMT(0, ptz_frame(XM, PTZ_ADDR_DEFAULT, PTZ_NIGHT, 0, f), "%d");
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
    ASSERT(ptz_verb_parse("upleft", NULL) && ptz_verb_parse("downright", NULL));
    ASSERT(!ptz_verb_parse("up-left", NULL) && !ptz_verb_parse("leftup", NULL));
    PASS();
}

// Each pan/tilt verb names its two directions; the diagonals are the four
// pairs, and nothing else is pan/tilt.
TEST pan_tilt_verbs_name_their_directions(void) {
    static const struct { enum PtzVerb v; int pan, tilt; } want[] = {
        {PTZ_LEFT, -1, 0},     {PTZ_RIGHT, 1, 0},      {PTZ_UP, 0, 1},
        {PTZ_DOWN, 0, -1},     {PTZ_UPLEFT, -1, 1},    {PTZ_UPRIGHT, 1, 1},
        {PTZ_DOWNLEFT, -1, -1}, {PTZ_DOWNRIGHT, 1, -1},
    };
    for (size_t i = 0; i < sizeof want / sizeof *want; i++) {
        int p = 9, t = 9;
        ASSERT(ptz_verb_dirs(want[i].v, &p, &t));
        ASSERT_EQ_FMT(want[i].pan, p, "%d");
        ASSERT_EQ_FMT(want[i].tilt, t, "%d");
        ASSERT(ptz_verb_is_pantilt(want[i].v));
    }
    ASSERT(!ptz_verb_dirs(PTZ_STOP, NULL, NULL));
    ASSERT(!ptz_verb_dirs(PTZ_TELE, NULL, NULL));
    ASSERT(!ptz_verb_is_pantilt(PTZ_NIGHT));
    PASS();
}

// A diagonal is one frame: both direction bits, the speed in both data bytes.
TEST diagonals_are_one_frame_with_both_speeds(void) {
    FRAME_EQ(D, PTZ_UPLEFT, 0xff, 0x01, 0x00, 0x0c, 0x20, 0x20, 0x4d);
    FRAME_EQ(D, PTZ_DOWNRIGHT, 0xff, 0x01, 0x00, 0x12, 0x20, 0x20, 0x53);
    FRAME_EQ(XM, PTZ_UPRIGHT, 0xc5, 0x01, 0x00, 0x0a, 0x3f, 0x3f, 0x89, 0x5c);
    unsigned char f[PTZ_FRAME_MAX];
    ptz_frame(D, PTZ_ADDR_DEFAULT, PTZ_DOWNLEFT, 20, f);
    ASSERT_EQ_FMT(0x14, (int)f[3], "%d");
    ASSERT_EQ_FMT(20, (int)f[4], "%d");
    ASSERT_EQ_FMT(20, (int)f[5], "%d");
    ASSERT_EQ_FMT((1 + 0x14 + 20 + 20) % 256, (int)f[6], "%d");
    PASS();
}

// The core's ptz command: a verb, an optional duration, an optional speed,
// and nothing else -- every malformed field is a refusal, not a default.
TEST ptz_commands_parse_and_refuse(void) {
    PtzCommand c;
    ASSERT(ptz_command_parse("left", &c));
    ASSERT_EQ(PTZ_LEFT, c.verb); ASSERT_EQ(0, c.ms); ASSERT_EQ(0, c.speed);
    ASSERT(ptz_command_parse("up:1500", &c));
    ASSERT_EQ(PTZ_UP, c.verb); ASSERT_EQ(1500, c.ms); ASSERT_EQ(0, c.speed);
    ASSERT(ptz_command_parse("right:1500:40", &c));
    ASSERT_EQ(PTZ_RIGHT, c.verb); ASSERT_EQ(1500, c.ms); ASSERT_EQ(40, c.speed);
    ASSERT(ptz_command_parse("down:0:100", &c));
    ASSERT_EQ(0, c.ms); ASSERT_EQ(100, c.speed);
    ASSERT(ptz_command_parse("tele:100000:1", &c));
    ASSERT_EQ(1, c.speed);
    const char *bad[] = {"", ":", "left:", "left:abc", "left:500junk", "left:-5",
                         "left:100001", "left:1500:", "left:1500:0", "left:1500:101",
                         "left:1500:4x", "left:1500:40:1", "sideways:1500", "left:1500: 40",
                         "aaaaaaaaaaaaaaaaaaaa:10"};
    for (unsigned i = 0; i < sizeof bad / sizeof *bad; i++) {
        c = (PtzCommand){PTZ_NEAR, 7, 7};
        ASSERT_FALSEm(bad[i], ptz_command_parse(bad[i], &c));
        ASSERT_EQ(PTZ_NEAR, c.verb);   // a refusal leaves the output alone
    }
    PASS();
}

// The core's ptzstep command: a pan/tilt verb, a step count, an optional
// speed. A zoom or a focus has nothing to count, so it is refused.
TEST ptz_step_commands_parse_and_refuse(void) {
    PtzStepCommand c;
    ASSERT(ptz_step_command_parse("left:23", &c));
    ASSERT_EQ(PTZ_LEFT, c.verb); ASSERT_EQ(23, c.steps); ASSERT_EQ(0, c.speed);
    ASSERT(ptz_step_command_parse("up:1:70", &c));
    ASSERT_EQ(PTZ_UP, c.verb); ASSERT_EQ(1, c.steps); ASSERT_EQ(70, c.speed);
    ASSERT(ptz_step_command_parse("down:1000:100", &c));
    ASSERT_EQ(1000, c.steps);
    const char *bad[] = {"", "left", "left:", "left:0", "left:1001", "left:-3", "left:5x",
                         "left:5:0", "left:5:101", "left:5:7:1", "tele:5", "near:5",
                         "stop:5", "sideways:5"};
    for (unsigned i = 0; i < sizeof bad / sizeof *bad; i++) {
        c = (PtzStepCommand){PTZ_NEAR, 7, 7};
        ASSERT_FALSEm(bad[i], ptz_step_command_parse(bad[i], &c));
        ASSERT_EQ(PTZ_NEAR, c.verb);
    }
    PASS();
}

// A head that turns the other way: each pan/tilt verb mirrored on the axes
// asked, and every other verb left alone.
TEST a_verb_mirrors_on_the_axes_asked(void) {
    ASSERT_EQ(PTZ_RIGHT, ptz_verb_mirror(PTZ_LEFT, true, false));
    ASSERT_EQ(PTZ_LEFT, ptz_verb_mirror(PTZ_LEFT, false, true));
    ASSERT_EQ(PTZ_DOWN, ptz_verb_mirror(PTZ_UP, false, true));
    ASSERT_EQ(PTZ_UP, ptz_verb_mirror(PTZ_UP, true, false));
    ASSERT_EQ(PTZ_DOWNRIGHT, ptz_verb_mirror(PTZ_UPLEFT, true, true));
    ASSERT_EQ(PTZ_UPRIGHT, ptz_verb_mirror(PTZ_UPLEFT, true, false));
    ASSERT_EQ(PTZ_DOWNLEFT, ptz_verb_mirror(PTZ_UPLEFT, false, true));
    ASSERT_EQ(PTZ_LEFT, ptz_verb_mirror(PTZ_LEFT, false, false));
    ASSERT_EQ(PTZ_TELE, ptz_verb_mirror(PTZ_TELE, true, true));
    ASSERT_EQ(PTZ_NEAR, ptz_verb_mirror(PTZ_NEAR, true, true));
    ASSERT_EQ(PTZ_STOP, ptz_verb_mirror(PTZ_STOP, true, true));
    PASS();
}

// The three preset frames, byte for byte: what the speed dome of OpenIPC/firmware#2385
// answered to (set 2 then go 2, from its owner's test), and clear. The address is the
// caller's. The XiongMai lens wire has no presets and gets no frame.
TEST preset_frames_set_go_clear(void) {
    const PtzProto *D = ptz_proto("pelco-d"), *XM = ptz_proto("pelco-xm");
    unsigned char f[PTZ_FRAME_MAX];
    ASSERT_EQ(7, ptz_preset_op_frame(D, 1, PTZ_PRESET_SET, 2, f));
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x01, 0x00, 0x03, 0x00, 0x02, 0x06}), f, 7);
    ASSERT_EQ(7, ptz_preset_op_frame(D, 1, PTZ_PRESET_GO, 2, f));
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x01, 0x00, 0x07, 0x00, 0x02, 0x0a}), f, 7);
    ASSERT_EQ(7, ptz_preset_op_frame(D, 1, PTZ_PRESET_CLEAR, 2, f));
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x01, 0x00, 0x05, 0x00, 0x02, 0x08}), f, 7);
    ASSERT_EQ(7, ptz_preset_op_frame(D, 9, PTZ_PRESET_GO, 32, f));
    ASSERT_MEM_EQ(((unsigned char[]){0xff, 0x09, 0x00, 0x07, 0x00, 0x20, 0x30}), f, 7);
    ASSERT_EQ(0, ptz_preset_op_frame(XM, 1, PTZ_PRESET_GO, 2, f));
    ASSERT_EQ(0, ptz_preset_op_frame(D, 1, PTZ_PRESET_GO, 256, f));
    ASSERT(ptz_proto_has_presets(D));
    ASSERT_FALSE(ptz_proto_has_presets(XM));
    PASS();
}

TEST preset_commands_parse_and_refuse(void) {
    enum PtzPresetOp op;
    int n;
    ASSERT(ptz_preset_command_parse("go:2", &op, &n) && op == PTZ_PRESET_GO && n == 2);
    ASSERT(ptz_preset_command_parse("set:32", &op, &n) && op == PTZ_PRESET_SET && n == 32);
    ASSERT(ptz_preset_command_parse("clear:255", &op, &n) && op == PTZ_PRESET_CLEAR && n == 255);
    const char *bad[] = {"", "go:", "go:0", "go:256", "go:-1", "go:2x", "go: 2", "goto:2",
                         "go:99999999999", "set", "stop:1"};
    for (size_t i = 0; i < sizeof bad / sizeof *bad; i++) {
        ASSERT_FALSEm(bad[i], ptz_preset_command_parse(bad[i], &op, &n));
    }
    ASSERT_FALSE(ptz_preset_command_parse(NULL, &op, &n));
    PASS();
}

SUITE(proto_suite) {
    RUN_TEST(preset_frames_set_go_clear);
    RUN_TEST(preset_commands_parse_and_refuse);
    RUN_TEST(a_verb_mirrors_on_the_axes_asked);
    XM = ptz_proto("pelco-xm");
    D = ptz_proto("pelco-d");
    RUN_TEST(xm_frames_match_the_stock_firmware);
    RUN_TEST(xm_pan_tilt_keep_the_scripts_speed);
    RUN_TEST(xm_speed_bytes_match_the_stock_firmware);
    RUN_TEST(pelco_d_pan_tilt_default_to_half_speed);
    RUN_TEST(pelco_d_frames_match_btzoom);
    RUN_TEST(only_focus_differs_between_the_two_protocols);
    RUN_TEST(speed_lands_in_the_right_slot);
    RUN_TEST(presets_and_wake_match_the_lens_tool);
    RUN_TEST(the_address_is_the_second_byte_and_is_summed);
    RUN_TEST(only_a_pelco_address_is_used_as_one);
    RUN_TEST(the_xm_variant_has_no_day_night);
    RUN_TEST(unknown_actuator_falls_back_to_the_default);
    RUN_TEST(verb_names_round_trip_and_reject_everything_else);
    RUN_TEST(pan_tilt_verbs_name_their_directions);
    RUN_TEST(diagonals_are_one_frame_with_both_speeds);
    RUN_TEST(ptz_commands_parse_and_refuse);
    RUN_TEST(ptz_step_commands_parse_and_refuse);
}
