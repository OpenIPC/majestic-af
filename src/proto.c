#include "proto.h"

#include <string.h>

#include <majestic/af_plugin_abi.h>

// Command bits, shared by both protocols — this is the Pelco-D layout, which
// the XiongMai variant adopts wholesale:
//
//   command 1  bit 0  focus near
//   command 2  bit 7  focus far      bit 6  zoom wide    bit 5  zoom tele
//              bit 4  tilt down      bit 3  tilt up
//              bit 2  pan left       bit 1  pan right
//
// The standard's sense is followed for both protocols, with one exception: the
// two focus bits. On the XiongMai lens board command 2 bit 7 focuses NEARER and
// command 1 bit 0 FARTHER — the reverse of the Pelco-D names, and what the
// stock firmware's own FocusNear/FocusFar commands send. Measured on an 85H50AI
// with near and far targets whose depth order is certain from occlusion
// (OpenIPC/motors xm-uart/PROTOCOL.md, "Focus direction"); btzoom-xm had the
// Pelco-D sense, so its `near` button focused farther. focus_swapped says so.
//
// btzoom-xm spelled pan
// the other way round — its "left" set bit 1 — but the XiongMai hardware this
// reached is a zoom block with no pan axis at all (ptz_caps is `zoom focus`),
// so that mapping was never exercised against a moving motor and is not worth
// carrying forward.
//
// day/night are Pelco-D auxiliary commands (command 2 = 0x0b / 0x09 with the
// aux index in data 2), not movement bits.
//
// speed_slot says where a 0..63 speed byte belongs for this verb: 1 = data 1
// (pan), 2 = data 2 (tilt), 3 = both (a diagonal), 0 = the verb has no speed. Zoom and focus in Pelco
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
    // Pelco-D's direction bits combine: up|left and so on, one frame, both
    // speed bytes.
    [PTZ_UPLEFT] = {0x00, 0x08 | 0x04, 0x00, 0x00, 3},
    [PTZ_UPRIGHT] = {0x00, 0x08 | 0x02, 0x00, 0x00, 3},
    [PTZ_DOWNLEFT] = {0x00, 0x10 | 0x04, 0x00, 0x00, 3},
    [PTZ_DOWNRIGHT] = {0x00, 0x10 | 0x02, 0x00, 0x00, 3},
};

static const char *const VERB_NAME[PTZ_VERB_COUNT] = {
    [PTZ_STOP] = "stop",   [PTZ_NEAR] = "near", [PTZ_FAR] = "far",
    [PTZ_TELE] = "tele",   [PTZ_WIDE] = "wide", [PTZ_UP] = "up",
    [PTZ_DOWN] = "down",   [PTZ_LEFT] = "left", [PTZ_RIGHT] = "right",
    [PTZ_DAY] = "day",     [PTZ_NIGHT] = "night",
    [PTZ_UPLEFT] = "upleft", [PTZ_UPRIGHT] = "upright",
    [PTZ_DOWNLEFT] = "downleft", [PTZ_DOWNRIGHT] = "downright",
};

struct PtzProto {
    const char *name;
    unsigned char sync;
    unsigned char term;      // trailing byte, 0 = none
    bool terminated;
    bool focus_swapped;      // near/far are the other way round from Pelco-D
    unsigned char def_speed; // pan/tilt speed when the caller names none
    bool aux;                // carries the day/night auxiliary commands
    bool presets;            // carries the preset-call command
    bool framed_reports;     // the MCU reports in 51 01 04 frames, not ASCII
    const unsigned char *wake;
    size_t wake_len;
};

// Sent once at startup by the vendor tools. Neither is a Pelco frame; both are
// opaque vendor byte strings, reproduced exactly as the scripts had them.
static const unsigned char WAKE_XM[] = {0xa5, 0x7b, 0x9e, 0xf0,
                                        0xef, 0xee, 0xe0, 0xf4};
static const unsigned char WAKE_D[] = {0x51, 0x01, 0x04, 0x79, 0x01, 0x0d, 0x0a,
                                       0x2e, 0x02, 0x7e, 0x00, 0x00, 0x00, 0x95};
// The stock firmware's focus-statistics frame. The Anjoy MC800S lens board
// ignores every motor command after power-up until it has seen one, and the
// WAKE_D blob is not one. The statistics may be fixed: majestic's own search
// takes focus from the board's autofocus with its first move.
static const unsigned char WAKE_ANJOY[] = {0x51, 0x01, 0x04, 0x78, 0x01, 0x0d, 0x10,
                                           0x00, 0x10, 0x00, 0x40, 0x01, 0x00, 0x3d};

// The checksum: mod 256, as the stock XiongMai firmware sends it (captured on
// the camera's own UART, OpenIPC/motors xm-uart/PROTOCOL.md). btzoom-xm and
// every OpenIPC port of it used mod 100, which differs only on the one frame
// whose byte sum passes 100. The board validates neither — correct, garbage and
// constant checksums all drive the motor identically (lab 85H50AI, 2026-09-13)
// — so this only makes the bytes match what the lens board was built against.
static const PtzProto PROTO_XM = {
    .name = "pelco-xm",
    .sync = 0xc5,
    .term = 0x5c,
    .terminated = true,
    .focus_swapped = true,
    .def_speed = 63,   // btzoom-xm drove pan/tilt flat out; reproduced exactly
    .aux = false,      // the XM MCU only reports day/night; there is no send command
    .presets = false,
    .wake = WAKE_XM,
    .wake_len = sizeof WAKE_XM,
};

static const PtzProto PROTO_D = {
    .name = "pelco-d",
    .sync = 0xff,
    .term = 0x00,
    .terminated = false,
    .focus_swapped = false,
    // Half the range, for a move that names no speed: an arrow key on the
    // WebUI, a script, a webhook. Pelco-D's pan and tilt speed bytes run from
    // 00, the stop end, to 3F; btzoom sent 00 and this did too, which left a
    // speed dome that follows the standard all but still for every such move
    // (OpenIPC/majestic-webui#691). XiongMai's board is unaffected -- its
    // default is its own, above.
    .def_speed = 32,
    .aux = true,
    .presets = true,
    .wake = WAKE_D,
    .wake_len = sizeof WAKE_D,
};

static const PtzProto PROTO_ANJOY = {
    .name = "anjoy",
    .sync = 0xff,
    .def_speed = 32,
    .aux = true,
    .presets = true,
    .framed_reports = true,
    .wake = WAKE_ANJOY,
    .wake_len = sizeof WAKE_ANJOY,
};

static const PtzProto *const PROTOS[] = {&PROTO_XM, &PROTO_D, &PROTO_ANJOY};

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

int ptz_addr(int configured, bool *ok) {
    bool good = configured >= 0 && configured <= 255;
    if (ok) {
        *ok = good;
    }
    return good ? configured : PTZ_ADDR_DEFAULT;
}

int ptz_baud(int configured) {
    switch (configured) {
    case 1200:
    case 2400:
    case 4800:
    case 9600:
    case 19200:
    case 38400:
    case 57600:
        return configured;
    default:
        return 115200;
    }
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
// checksum (mod 256) over address..data2, and the terminator where the protocol has one.
static int build(const PtzProto *p, unsigned char addr, unsigned char c1,
                 unsigned char c2, unsigned char d1, unsigned char d2,
                 unsigned char out[PTZ_FRAME_MAX]) {
    int sum = addr + c1 + c2 + d1 + d2;
    int n = 0;
    out[n++] = p->sync;
    out[n++] = addr;
    out[n++] = c1;
    out[n++] = c2;
    out[n++] = d1;
    out[n++] = d2;
    out[n++] = (unsigned char)(sum % 256);
    if (p->terminated) {
        out[n++] = p->term;
    }
    return n;
}

int ptz_frame(const PtzProto *p, unsigned char addr, enum PtzVerb v, int speed,
              unsigned char out[PTZ_FRAME_MAX]) {
    if (!ptz_proto_has(p, v)) {
        return 0;
    }
    if (p->focus_swapped && ptz_verb_is_focus(v)) {
        v = v == PTZ_NEAR ? PTZ_FAR : PTZ_NEAR;
    }
    const VerbBits *b = &VERB[v];
    unsigned char d1 = b->d1, d2 = b->d2;
    if (b->speed_slot) {
        if (speed <= 0) {
            speed = p->def_speed;
        } else if (speed > 63) {
            speed = 63;   // Pelco's speed range; 0x3f is "turbo" on some domes
        }
        if (b->speed_slot & 1) {
            d1 = (unsigned char)speed;
        }
        if (b->speed_slot & 2) {
            d2 = (unsigned char)speed;
        }
    }
    return build(p, addr, b->c1, b->c2, d1, d2, out);
}

int ptz_preset_frame(const PtzProto *p, unsigned char addr, int preset,
                     unsigned char out[PTZ_FRAME_MAX]) {
    return ptz_preset_op_frame(p, addr, PTZ_PRESET_GO, preset, out);
}

int ptz_preset_op_frame(const PtzProto *p, unsigned char addr, enum PtzPresetOp op,
                        int preset, unsigned char out[PTZ_FRAME_MAX]) {
    if (!p || !p->presets || preset < 0 || preset > 255) {
        return 0;
    }
    unsigned char c2 = op == PTZ_PRESET_SET ? 0x03 : op == PTZ_PRESET_GO ? 0x07
                     : op == PTZ_PRESET_CLEAR ? 0x05 : 0;
    if (!c2) {
        return 0;
    }
    return build(p, addr, 0x00, c2, 0x00, (unsigned char)preset, out);
}

bool ptz_proto_has_presets(const PtzProto *p) { return p && p->presets; }

bool ptz_proto_framed_reports(const PtzProto *p) { return p && p->framed_reports; }

bool ptz_report_feed(PtzReport *r, unsigned char c) {
    static const unsigned char HEAD[] = {0x51, 0x01, 0x04};
    r->buf[r->n++] = c;
    while (r->n > 0) {
        bool good = true;
        for (int i = 0; i < r->n && i < (int)sizeof HEAD; i++) {
            good = good && r->buf[i] == HEAD[i];
        }
        int len = r->n >= 6 ? r->buf[5] : 0;
        if (good && r->n >= 6 && (len < 1 || len > PTZ_REPORT_PAYLOAD_MAX)) {
            good = false;
        }
        if (good) {
            if (r->n < 6 || r->n < len + 7) {
                return false;
            }
            unsigned char sum = 0;
            for (int i = 0; i < len + 6; i++) {
                sum = (unsigned char)(sum + r->buf[i]);
            }
            if (sum == r->buf[len + 6]) {
                r->n = 0;
                return true;
            }
        }
        memmove(r->buf, r->buf + 1, (size_t)--r->n);
    }
    return false;
}

// media_server's glyph table for these frames (thread_af_console_data).
static const char OSD_GLYPHS[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ!? &<>,.:; *%+-X/=\"`   ";

bool ptz_report_zoom(const PtzReport *r, float *mag) {
    const unsigned char *b = r->buf;
    int len = b[5];
    if (b[3] != 0x78 || b[4] != 0x01 || len < 3) {
        return false;
    }
    char text[PTZ_REPORT_PAYLOAD_MAX + 1];
    int n = 0;
    for (int i = 2; i < len; i++) {
        unsigned char g = b[6 + i];
        char c = g < sizeof OSD_GLYPHS - 1 ? OSD_GLYPHS[g] : '?';
        if (c != ' ') {
            text[n++] = c;
        }
    }
    text[n] = 0;
    if (n < 2 || text[n - 1] != 'X') {
        return false;
    }
    float v = 0, scale = 0;
    for (int i = 0; i < n - 1; i++) {
        if (text[i] == '.' && scale == 0) {
            scale = 1;
        } else if (text[i] >= '0' && text[i] <= '9') {
            if (scale > 0) {
                scale /= 10;
                v += (float)(text[i] - '0') * scale;
            } else {
                v = v * 10 + (float)(text[i] - '0');
            }
        } else {
            return false;
        }
    }
    if (v < 1 || v > 99) {
        return false;
    }
    *mag = v;
    return true;
}

// The scale the board settled on in the 2026-11-26 tests: both windows are the
// four banks summed and quartered, the second over the centre quarter of the
// grid, and luma the zone mean over 16. day=1, IR-cut 0, as in WAKE_ANJOY.
int ptz_anjoy_stats_frame(const unsigned short *zones, unsigned rows, unsigned cols,
                          unsigned char out[PTZ_ANJOY_STATS_LEN]) {
    if (!rows || !cols) {
        return 0;
    }
    unsigned long all = 0, centre = 0, y = 0;
    for (unsigned r = 0; r < rows; r++) {
        for (unsigned c = 0; c < cols; c++) {
            const unsigned short *z = zones + (r * cols + c) * AF_ZONE_FIELDS;
            unsigned long s = (unsigned long)z[0] + z[1] + z[2] + z[3];
            all += s;
            if (r >= rows / 4 && r < rows - rows / 4 && c >= cols / 4 && c < cols - cols / 4) {
                centre += s;
            }
            y += z[4];
        }
    }
    unsigned long w0 = all / 4, w1 = centre / 4, luma = y / (rows * cols) / 16;
    if (w0 > 0xffff) w0 = 0xffff;
    if (w1 > 0xffff) w1 = 0xffff;
    if (luma > 0xff) luma = 0xff;
    memcpy(out, WAKE_ANJOY, PTZ_ANJOY_STATS_LEN);
    out[6] = (unsigned char)(w0 >> 8);
    out[7] = (unsigned char)w0;
    out[8] = (unsigned char)(w1 >> 8);
    out[9] = (unsigned char)w1;
    out[10] = (unsigned char)luma;
    unsigned char sum = 0;
    for (int i = 0; i < PTZ_ANJOY_STATS_LEN - 1; i++) {
        sum = (unsigned char)(sum + out[i]);
    }
    out[PTZ_ANJOY_STATS_LEN - 1] = sum;
    return PTZ_ANJOY_STATS_LEN;
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

// One plain decimal field of a command, up to the next ':' or the end: strtol
// with its end checked, so "500junk" is refused rather than read as 500.
static bool command_field(const char *s, const char *end, long lo, long hi, long *out) {
    if (s == end) {
        return false;
    }
    long v = 0;
    for (const char *p = s; p < end; p++) {
        if (*p < '0' || *p > '9' || v > hi) {
            return false;
        }
        v = v * 10 + (*p - '0');
    }
    if (v < lo || v > hi) {
        return false;
    }
    *out = v;
    return true;
}

bool ptz_command_parse(const char *s, PtzCommand *out) {
    if (!s || !out) {
        return false;
    }
    const char *c1 = strchr(s, ':');
    const char *c2 = c1 ? strchr(c1 + 1, ':') : NULL;
    char name[16];
    size_t n = c1 ? (size_t)(c1 - s) : strlen(s);
    if (n == 0 || n >= sizeof name) {
        return false;
    }
    memcpy(name, s, n);
    name[n] = 0;
    PtzCommand cmd = {PTZ_STOP, 0, 0};
    if (!ptz_verb_parse(name, &cmd.verb)) {
        return false;
    }
    if (c1 && !command_field(c1 + 1, c2 ? c2 : c1 + 1 + strlen(c1 + 1), 0, 100000, &cmd.ms)) {
        return false;
    }
    if (c2) {
        long sp;
        if (!command_field(c2 + 1, c2 + 1 + strlen(c2 + 1), 1, 100, &sp)) {
            return false;
        }
        cmd.speed = (int)sp;
    }
    *out = cmd;
    return true;
}

bool ptz_step_command_parse(const char *s, PtzStepCommand *out) {
    if (!s || !out) {
        return false;
    }
    const char *c1 = strchr(s, ':');
    if (!c1) {
        return false;   // a step count is not optional here
    }
    const char *c2 = strchr(c1 + 1, ':');
    char name[16];
    size_t n = (size_t)(c1 - s);
    if (n == 0 || n >= sizeof name) {
        return false;
    }
    memcpy(name, s, n);
    name[n] = 0;
    PtzStepCommand cmd = {PTZ_STOP, 0, 0};
    if (!ptz_verb_parse(name, &cmd.verb) || !ptz_verb_is_pantilt(cmd.verb)) {
        return false;
    }
    long v;
    if (!command_field(c1 + 1, c2 ? c2 : c1 + 1 + strlen(c1 + 1), 1, PTZ_STEPS_MAX, &v)) {
        return false;
    }
    cmd.steps = (int)v;
    if (c2) {
        if (!command_field(c2 + 1, c2 + 1 + strlen(c2 + 1), 1, 100, &v)) {
            return false;
        }
        cmd.speed = (int)v;
    }
    *out = cmd;
    return true;
}

bool ptz_verb_is_focus(enum PtzVerb v) {
    return v == PTZ_NEAR || v == PTZ_FAR;
}

bool ptz_verb_is_zoom(enum PtzVerb v) {
    return v == PTZ_TELE || v == PTZ_WIDE;
}

bool ptz_verb_is_pantilt(enum PtzVerb v) {
    return ptz_verb_dirs(v, NULL, NULL);
}

bool ptz_verb_dirs(enum PtzVerb v, int *pan, int *tilt) {
    int p, t;
    switch (v) {
    case PTZ_LEFT:      p = -1; t = 0;  break;
    case PTZ_RIGHT:     p = 1;  t = 0;  break;
    case PTZ_UP:        p = 0;  t = 1;  break;
    case PTZ_DOWN:      p = 0;  t = -1; break;
    case PTZ_UPLEFT:    p = -1; t = 1;  break;
    case PTZ_UPRIGHT:   p = 1;  t = 1;  break;
    case PTZ_DOWNLEFT:  p = -1; t = -1; break;
    case PTZ_DOWNRIGHT: p = 1;  t = -1; break;
    default:            return false;
    }
    if (pan) {
        *pan = p;
    }
    if (tilt) {
        *tilt = t;
    }
    return true;
}

enum PtzVerb ptz_verb_mirror(enum PtzVerb v, bool pan, bool tilt) {
    int p, t;
    if ((!pan && !tilt) || !ptz_verb_dirs(v, &p, &t)) {
        return v;
    }
    if (pan) {
        p = -p;
    }
    if (tilt) {
        t = -t;
    }
    for (int u = 0; u < PTZ_VERB_COUNT; u++) {
        int up, ut;
        if (ptz_verb_dirs((enum PtzVerb)u, &up, &ut) && up == p && ut == t) {
            return (enum PtzVerb)u;
        }
    }
    return v;
}

bool ptz_preset_command_parse(const char *s, enum PtzPresetOp *op, int *n) {
    static const struct { const char *word; enum PtzPresetOp op; } ops[] = {
        {"set:", PTZ_PRESET_SET}, {"go:", PTZ_PRESET_GO}, {"clear:", PTZ_PRESET_CLEAR}};
    if (!s) {
        return false;
    }
    for (size_t i = 0; i < sizeof ops / sizeof *ops; i++) {
        size_t w = strlen(ops[i].word);
        if (strncmp(s, ops[i].word, w)) {
            continue;
        }
        const char *d = s + w;
        int v = 0;
        if (!*d) {
            return false;
        }
        for (; *d; d++) {
            if (*d < '0' || *d > '9' || v > 255) {
                return false;
            }
            v = v * 10 + (*d - '0');
        }
        if (v < 1 || v > 255) {
            return false;
        }
        if (op) *op = ops[i].op;
        if (n) *n = v;
        return true;
    }
    return false;
}
