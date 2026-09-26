// MS41908M backend — the pure, hardware-free logic: the verb->axis mapping, the
// soft-limit clamp, and the zoom-position->magnification curve. Split out of
// act_ms41908.c so tests/actuator_test.c can pin it without pulling in SPI,
// /dev/mem, the stepping thread, or the log/HAL seams.

#include "act_ms41908.h"

bool ms_verb_axis(enum PtzVerb v, bool *is_zoom, int *dir) {
    switch (v) {
    case PTZ_NEAR: *is_zoom = false; *dir = -1; return true;
    case PTZ_FAR:  *is_zoom = false; *dir = +1; return true;
    case PTZ_TELE: *is_zoom = true;  *dir = +1; return true;
    case PTZ_WIDE: *is_zoom = true;  *dir = -1; return true;
    default: return false;   // stop, and the pan/tilt/aux verbs this lens lacks
    }
}

int ms_clamp_step(int pos, int dir, int want, int max) {
    if (want < 0) want = 0;
    if (dir > 0) {
        int room = max - pos;
        return room < want ? (room < 0 ? 0 : room) : want;
    }
    if (dir < 0) {
        return pos < want ? (pos < 0 ? 0 : pos) : want;
    }
    return 0;
}

float ms_zoom_mag(int zoom_pos) {
    if (zoom_pos <= 0) return MS_MAG_MIN;
    if (zoom_pos >= MS_ZOOM_MAX) return MS_MAG_MAX;
    return MS_MAG_MIN + (MS_MAG_MAX - MS_MAG_MIN) * ((float)zoom_pos / MS_ZOOM_MAX);
}
