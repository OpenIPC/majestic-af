#include "proto.h"

#include <string.h>

// Command bits, shared by both protocols — this is the Pelco-D layout, which
// the XiongMai variant adopts wholesale:
//
//   command 1  bit 0  focus near
//   command 2  bit 7  focus far      bit 6  zoom wide    bit 5  zoom tele
//              bit 4  tilt down      bit 3  tilt up
//              bit 2  pan left       bit 1  pan right
//
// The standard's sense is followed for both protocols. btzoom-xm spelled pan
// the other way round — its "left" set bit 1 — but the XiongMai hardware this
// reached is a zoom block with no pan axis at all (ptz_caps is `zoom focus`),
// so that mapping was never exercised against a moving motor and is not worth
// carrying forward.
//
// day/night are Pelco-D auxiliary commands (command 2 = 0x0b / 0x09 with the
// aux index in data 2), not movement bits.
//
// speed_slot says where a 0..63 speed byte belongs for this verb: 1 = data 1
// (pan), 2 = data 2 (tilt), 0 = the verb has no speed. Zoom and focus in Pelco
// carry no speed byte at all; the lens controller runs them at its own rate.
typedef struct {
    unsigned char c1, c2, d1, d2;
    unsigned char speed_slot;
} VerbBits;

static const VerbBits VERB[PTZ_VERB_COUNT] = {
    [PTZ_STOP] = {0x00, 0x00, 0x00, 0x00, 0},
    [PTZ_NEAR] = {0x01, 0x00, 0x00, 0x00, 0},
    [PTZ_FAR] = {0x00, 0x80, 0x00, 0x00, 0},
    [PTZ_TELE] = {0x00, 0x20, 0x00, 0x00, 0},
    [PTZ_WIDE] = {0x00, 0x40, 0x00, 0x00, 0},
    [PTZ_UP] = {0x00, 0x08, 0x00, 0x00, 2},
    [PTZ_DOWN] = {0x00, 0x10, 0x00, 0x00, 2},
    [PTZ_LEFT] = {0x00, 0x04, 0x00, 0x00, 1},
    [PTZ_RIGHT] = {0x00, 0x02, 0x00, 0x00, 1},
    [PTZ_DAY] = {0x00, 0x0b, 0x00, 0x01, 0},
    [PTZ_NIGHT] = {0x00, 0x09, 0x00, 0x01, 0},
};

static const char *const VERB_NAME[PTZ_VERB_COUNT] = {
    [PTZ_STOP] = "stop",   [PTZ_NEAR] = "near", [PTZ_FAR] = "far",
    [PTZ_TELE] = "tele",   [PTZ_WIDE] = "wide", [PTZ_UP] = "up",
    [PTZ_DOWN] = "down",   [PTZ_LEFT] = "left", [PTZ_RIGHT] = "right",
    [PTZ_DAY] = "day",     [PTZ_NIGHT] = "night",
};

struct PtzProto {
    const char *name;
    unsigned char sync;
    unsigned char addr;
    unsigned char term;      // trailing byte, 0 = none
    bool terminated;
    int csum_mod;            // 100 for the XiongMai variant, 256 for Pelco-D
    unsigned char def_speed; // pan/tilt speed when the caller names none
    bool aux;                // carries the day/night auxiliary commands
    bool presets;            // carries the preset-call command
    const unsigned char *wake;
    size_t wake_len;
};

// Sent once at startup by the vendor tools. Neither is a Pelco frame; both are
// opaque vendor byte strings, reproduced exactly as the scripts had them.
static const unsigned char WAKE_XM[] = {0xa5, 0x7b, 0x9e, 0xf0,
                                        0xef, 0xee, 0xe0, 0xf4};
static const unsigned char WAKE_D[] = {0x51, 0x01, 0x04, 0x79, 0x01, 0x0d, 0x0a,
                                       0x2e, 0x02, 0x7e, 0x00, 0x00, 0x00, 0x95};

// The checksum rule is the one difference that ever mattered. mod 100 is what
// the XiongMai vendor tool emits and what every OpenIPC port of it has sent
// since 2023; mod 256 is the Pelco-D standard. Measured on a lab 85H50AI
// (2026-09-13): that MCU validates neither — correct, garbage and constant-1
// checksums all drive the motor identically — so the rule is followed because
// it is the protocol, not because any lens is known to enforce it.
static const PtzProto PROTO_XM = {
    .name = "pelco-xm",
    .sync = 0xc5,
    .addr = 0x01,
    .term = 0x5c,
    .terminated = true,
    .csum_mod = 100,
    .def_speed = 63,   // btzoom-xm drove pan/tilt flat out; reproduced exactly
    .aux = false,      // the XM MCU only reports day/night; there is no send command
    .presets = false,
    .wake = WAKE_XM,
    .wake_len = sizeof WAKE_XM,
};

static const PtzProto PROTO_D = {
    .name = "pelco-d",
    .sync = 0xff,
    .addr = 0x01,
    .term = 0x00,
    .terminated = false,
    .csum_mod = 256,
    .def_speed = 0,    // btzoom sent a zero speed byte; reproduced exactly
    .aux = true,
    .presets = true,
    .wake = WAKE_D,
    .wake_len = sizeof WAKE_D,
};

static const PtzProto *const PROTOS[] = {&PROTO_XM, &PROTO_D};

const PtzProto *ptz_proto(const char *name) {
    if (name && *name) {
        for (unsigned i = 0; i < sizeof PROTOS / sizeof *PROTOS; i++) {
            if (!strcmp(name, PROTOS[i]->name)) {
                return PROTOS[i];
            }
        }
    }
    return &PROTO_XM;
}

const char *ptz_proto_name(const PtzProto *p) { return p ? p->name : ""; }

bool ptz_proto_has(const PtzProto *p, enum PtzVerb v) {
    if (!p || v < 0 || v >= PTZ_VERB_COUNT) {
        return false;
    }
    if (v == PTZ_DAY || v == PTZ_NIGHT) {
        return p->aux;
    }
    return true;
}

// One frame: sync, address, the two command bytes, the two data bytes, the
// checksum over address..data2, and the terminator where the protocol has one.
static int build(const PtzProto *p, unsigned char c1, unsigned char c2,
                 unsigned char d1, unsigned char d2,
                 unsigned char out[PTZ_FRAME_MAX]) {
    int sum = p->addr + c1 + c2 + d1 + d2;
    int n = 0;
    out[n++] = p->sync;
    out[n++] = p->addr;
    out[n++] = c1;
    out[n++] = c2;
    out[n++] = d1;
    out[n++] = d2;
    out[n++] = (unsigned char)(sum % p->csum_mod);
    if (p->terminated) {
        out[n++] = p->term;
    }
    return n;
}

int ptz_frame(const PtzProto *p, enum PtzVerb v, int speed,
              unsigned char out[PTZ_FRAME_MAX]) {
    if (!ptz_proto_has(p, v)) {
        return 0;
    }
    const VerbBits *b = &VERB[v];
    unsigned char d1 = b->d1, d2 = b->d2;
    if (b->speed_slot) {
        if (speed <= 0) {
            speed = p->def_speed;
        } else if (speed > 63) {
            speed = 63;   // Pelco's speed range; 0x3f is "turbo" on some domes
        }
        if (b->speed_slot == 1) {
            d1 = (unsigned char)speed;
        } else {
            d2 = (unsigned char)speed;
        }
    }
    return build(p, b->c1, b->c2, d1, d2, out);
}

int ptz_preset_frame(const PtzProto *p, int preset,
                     unsigned char out[PTZ_FRAME_MAX]) {
    if (!p || !p->presets || preset < 0 || preset > 255) {
        return 0;
    }
    return build(p, 0x00, 0x07, 0x00, (unsigned char)preset, out);
}

const unsigned char *ptz_wake_blob(const PtzProto *p, size_t *len) {
    if (!p || !p->wake) {
        if (len) {
            *len = 0;
        }
        return NULL;
    }
    if (len) {
        *len = p->wake_len;
    }
    return p->wake;
}

const char *ptz_verb_name(enum PtzVerb v) {
    return (v >= 0 && v < PTZ_VERB_COUNT) ? VERB_NAME[v] : "";
}

bool ptz_verb_parse(const char *s, enum PtzVerb *out) {
    if (!s || !*s) {
        return false;
    }
    for (int v = 0; v < PTZ_VERB_COUNT; v++) {
        if (!strcmp(s, VERB_NAME[v])) {
            if (out) {
                *out = (enum PtzVerb)v;
            }
            return true;
        }
    }
    return false;
}

bool ptz_verb_is_focus(enum PtzVerb v) {
    return v == PTZ_NEAR || v == PTZ_FAR;
}

bool ptz_verb_is_zoom(enum PtzVerb v) {
    return v == PTZ_TELE || v == PTZ_WIDE;
}
