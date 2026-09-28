// MS41908M backend — the pure, hardware-free logic: the verb->axis mapping, the
// soft-limit clamp, and the zoom-position->magnification curve. Split out of
// act_ms41908.c so tests/actuator_test.c can pin it without pulling in SPI,
// /dev/mem, the stepping thread, or the log/HAL seams.

#include "act_ms41908.h"

bool ms_verb_axis(enum PtzVerb v, bool *is_zoom, int *dir) {
    switch (v) {
    case PTZ_NEAR: *is_zoom = false; *dir = -1; return true;
    case PTZ_FAR:  *is_zoom = false; *dir = +1; return true;
    // Zoom direction is set by the lens's mechanics, verified on hardware by
    // photographing both stops: driving the motor toward its home stop (dir < 0)
    // narrows the field of view -- that stop is TELE (fully zoomed IN), not wide.
    // So TELE drives toward 0 and WIDE away from it; the magnification curve below
    // is inverted to match (pos 0 = max zoom). Getting this backwards put "wide"
    // and "tele" the wrong way round and left "wide" dead at the max-zoom stop.
    case PTZ_TELE: *is_zoom = true;  *dir = -1; return true;
    case PTZ_WIDE: *is_zoom = true;  *dir = +1; return true;
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

// pos is microsteps from the TELE (home) stop, verified on hardware to be the
// MAX-zoom / narrowest-FOV end: pos 0 is fully zoomed IN (mag MAX), pos
// MS_ZOOM_MAX is fully WIDE (mag MIN). The curve is therefore DESCENDING in pos.
float ms_zoom_mag(int zoom_pos) {
    if (zoom_pos <= 0) return MS_MAG_MAX;
    if (zoom_pos >= MS_ZOOM_MAX) return MS_MAG_MIN;
    return MS_MAG_MAX - (MS_MAG_MAX - MS_MAG_MIN) * ((float)zoom_pos / MS_ZOOM_MAX);
}

// Inverse of ms_zoom_mag(): the step position that reports `mag`, clamped to
// [0, MS_ZOOM_MAX]. Used to seed the dead-reckoned zoom position from a restored
// magnification so /zoom reports correctly from boot without a physical home seek.
int ms_zoom_pos_for_mag(float mag) {
    if (mag >= MS_MAG_MAX) return 0;
    if (mag <= MS_MAG_MIN) return MS_ZOOM_MAX;
    return (int)((MS_MAG_MAX - mag) / (MS_MAG_MAX - MS_MAG_MIN) * MS_ZOOM_MAX + 0.5f);
}
