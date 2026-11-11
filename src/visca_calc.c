// VISCA backend — pure logic: the bytes per verb, the reply parser, and the
// zoom-position -> magnification curve. No tty, no thread, no HAL seams, so
// tests/visca_test.c links it with nothing stubbed.

#include "act_visca.h"

#include <string.h>

static int put(unsigned char *out, int n, const unsigned char *f, int len) {
    memcpy(out + n, f, (size_t)len);
    return n + len;
}

bool visca_has(enum PtzVerb v) {
    return v == PTZ_STOP || v == PTZ_NEAR || v == PTZ_FAR || v == PTZ_TELE ||
           v == PTZ_WIDE;
}

int visca_frame(enum PtzVerb v, int speed, bool focus_manual, bool *now_manual,
                unsigned char out[VISCA_FRAME_MAX]) {
    static const unsigned char focus_auto[] = {0x81, 0x01, 0x04, 0x38, 0x02, 0xff};
    static const unsigned char focus_man[] = {0x81, 0x01, 0x04, 0x38, 0x03, 0xff};
    static const unsigned char zoom_stop[] = {0x81, 0x01, 0x04, 0x07, 0x00, 0xff};
    static const unsigned char focus_stop[] = {0x81, 0x01, 0x04, 0x08, 0x00, 0xff};
    unsigned char move[] = {0x81, 0x01, 0x04, 0x00, 0x00, 0xff};
    int n = 0;

    if (speed < 0 || speed > 7) {
        speed = VISCA_SPEED_DEFAULT;
    }
    bool manual = focus_manual;
    switch (v) {
    case PTZ_STOP:
        n = put(out, n, zoom_stop, sizeof zoom_stop);
        n = put(out, n, focus_stop, sizeof focus_stop);
        break;
    case PTZ_TELE:
    case PTZ_WIDE:
        if (manual) {
            n = put(out, n, focus_auto, sizeof focus_auto);
            manual = false;
        }
        move[3] = 0x07;   // CAM_Zoom
        move[4] = (unsigned char)((v == PTZ_TELE ? 0x20 : 0x30) | speed);
        n = put(out, n, move, sizeof move);
        break;
    case PTZ_NEAR:
    case PTZ_FAR:
        if (!manual) {
            n = put(out, n, focus_man, sizeof focus_man);
            manual = true;
        }
        move[3] = 0x08;   // CAM_Focus
        move[4] = (unsigned char)((v == PTZ_NEAR ? 0x30 : 0x20) | speed);
        n = put(out, n, move, sizeof move);
        break;
    default:
        return 0;
    }
    if (now_manual) {
        *now_manual = manual;
    }
    return n;
}

int visca_zoom_inquiry(unsigned char out[VISCA_FRAME_MAX]) {
    static const unsigned char inq[] = {0x81, 0x09, 0x04, 0x47, 0xff};
    return put(out, 0, inq, sizeof inq);
}

int visca_parse(const unsigned char *f, size_t n, int *pos, int *mag) {
    if (!f || !n || f[n - 1] != 0xff) {
        return 0;
    }
    // Zoom position: 90 50 0p 0q 0r 0s FF (camera address 1 answers as 0x90).
    if (n == 7 && f[0] == 0x90 && f[1] == 0x50 && !(f[2] & 0xf0) &&
        !(f[3] & 0xf0) && !(f[4] & 0xf0) && !(f[5] & 0xf0)) {
        if (pos) {
            *pos = (f[2] << 12) | (f[3] << 8) | (f[4] << 4) | f[5];
        }
        return 1;
    }
    // The lens MCU's status: BB 82 b2 b3 b4 b5 m6 m7 .. FF, 13 bytes; bytes 6..7
    // are the magnification, 1 at wide .. 18 at full tele.
    if (n == 13 && f[0] == 0xbb && f[1] == 0x82) {
        if (mag) {
            *mag = (f[6] << 8) | f[7];
        }
        return 2;
    }
    return 0;
}

// CAM_Zoom position -> magnification, measured on the N51813L: CAM_Zoom Direct
// to each position, then CAM_ZoomPosInq for where the lens actually stopped and
// the MCU's own status frame for the magnification it reports there.
static const struct {
    int pos;
    float mag;
} CURVE[] = {
    {0x0000, 1.0f},  {0x0ff3, 4.0f},  {0x17eb, 6.0f},  {0x1ff7, 9.0f},
    {0x2803, 12.0f}, {0x2ffb, 14.0f}, {0x37ce, 17.0f}, {0x3f0e, 18.0f},
};

float visca_pos_to_mag(int pos) {
    const int last = (int)(sizeof CURVE / sizeof *CURVE) - 1;
    if (pos <= CURVE[0].pos) {
        return CURVE[0].mag;
    }
    if (pos >= CURVE[last].pos) {
        return CURVE[last].mag;
    }
    for (int i = 1; i <= last; i++) {
        if (pos <= CURVE[i].pos) {
            float t = (float)(pos - CURVE[i - 1].pos) /
                      (float)(CURVE[i].pos - CURVE[i - 1].pos);
            return CURVE[i - 1].mag + t * (CURVE[i].mag - CURVE[i - 1].mag);
        }
    }
    return CURVE[last].mag;
}
