// MS41908M backend — the pure, hardware-free logic: the verb->axis mapping, the
// soft-limit clamp, and the zoom-position->magnification curve. Split out of
// act_ms41908.c so tests/actuator_test.c can pin it without pulling in SPI,
// /dev/mem, the stepping thread, or the log/HAL seams.

#include "act_ms41908.h"

#include <stdio.h>
#include <string.h>

bool ms_verb_axis(enum PtzVerb v, bool *is_zoom, int *dir) {
    MsLens l = MS_LENS_DEFAULT;
    return ms_lens_verb_axis(&l, v, is_zoom, dir);
}

bool ms_lens_verb_axis(const MsLens *l, enum PtzVerb v, bool *is_zoom, int *dir) {
    // A reversed lens has its WIDE end at the home stop: the zoom verbs swap.
    int tele = l->zoom_reversed ? +1 : -1;
    switch (v) {
    case PTZ_NEAR: *is_zoom = false; *dir = -1; return true;
    case PTZ_FAR:  *is_zoom = false; *dir = +1; return true;
    // Zoom direction is set by the lens's mechanics, verified on hardware by
    // photographing both stops: driving the motor toward its home stop (dir < 0)
    // narrows the field of view -- that stop is TELE (fully zoomed IN), not wide.
    // So TELE drives toward 0 and WIDE away from it; the magnification curve below
    // is inverted to match (pos 0 = max zoom). Getting this backwards put "wide"
    // and "tele" the wrong way round and left "wide" dead at the max-zoom stop.
    case PTZ_TELE: *is_zoom = true;  *dir = tele; return true;
    case PTZ_WIDE: *is_zoom = true;  *dir = -tele; return true;
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

int ms_clamp_span(int pos, int dir, int want, int lo, int max) {
    if (dir < 0) {
        if (pos <= lo) return 0;
        return ms_clamp_step(pos - lo, dir, want, max - lo);
    }
    return ms_clamp_step(pos, dir, want, max);
}

int ms_home_seek(int known, int max) {
    if (known < 0 || known > max) return max;
    // A quarter of the figure for a count that lost steps on the way there, and a
    // tenth of the travel for a figure near the stop, where a quarter is nothing.
    int seek = known + known / 4 + max / 10;
    return seek < max ? seek : max;
}

int ms_unhomed_step(int dir, int want) {
    return dir < 0 && want > 0 ? want : 0;
}

int ms_burst_intct(int psum, unsigned oscin_hz, long rot_us) {
    if (psum < 1 || rot_us < 1) return 1;
    unsigned long long v = (unsigned long long)oscin_hz * (unsigned long long)rot_us /
                           (24ull * (unsigned)psum * 1000000ull);
    return v < 1 ? 1 : v > 65535 ? 65535 : (int)v;
}

unsigned ms_oscin_from_rotation(int intct, int psum, long rot_us) {
    if (intct < 1 || psum < 1 || rot_us < 1) return 0;
    unsigned long long hz = 24ull * (unsigned)intct * (unsigned)psum * 1000000ull /
                            (unsigned long long)rot_us;
    return hz >= MS_OSCIN_MIN_HZ && hz <= MS_OSCIN_MAX_HZ ? (unsigned)hz : 0;
}

void ms_lens_state_format(char *buf, size_t n, int zoom, int focus, const MsLens *l,
                          bool swap, bool rest) {
    snprintf(buf, n, "%d %d %d %d %d %s\n", zoom, focus, l->zoom_max, l->focus_max,
             swap ? 1 : 0, rest ? "rest" : "moving");
}

bool ms_lens_state_parse(const char *line, const MsLens *l, bool swap, int *zoom,
                         int *focus) {
    int z, f, zmax, fmax, sw;
    char what[8];
    if (!line || sscanf(line, "%d %d %d %d %d %7s", &z, &f, &zmax, &fmax, &sw, what) != 6)
        return false;
    if (strcmp(what, "rest") != 0) return false;
    if (zmax != l->zoom_max || fmax != l->focus_max || (sw != 0) != swap) return false;
    if (z < 0 || z > zmax || f < 0 || f > fmax) return false;
    *zoom = z;
    *focus = f;
    return true;
}

// pos is microsteps from the TELE (home) stop, verified on hardware to be the
// MAX-zoom / narrowest-FOV end: pos 0 is fully zoomed IN (mag MAX), pos
// MS_ZOOM_MAX is fully WIDE (mag MIN). The curve is therefore DESCENDING in pos.
float ms_zoom_mag(int zoom_pos) {
    MsLens l = MS_LENS_DEFAULT;
    return ms_lens_zoom_mag(&l, zoom_pos);
}

float ms_lens_zoom_mag(const MsLens *l, int zoom_pos) {
    // The fraction of the travel from the TELE end, whichever end is home.
    float f = (float)zoom_pos / l->zoom_max;
    if (f < 0.0f) f = 0.0f;
    if (f > 1.0f) f = 1.0f;
    if (l->zoom_reversed) f = 1.0f - f;
    return l->mag_max - (l->mag_max - MS_MAG_MIN) * f;
}

// Inverse of ms_zoom_mag(): the step position that reports `mag`, clamped to
// [0, MS_ZOOM_MAX]. Used to seed the dead-reckoned zoom position from a restored
// magnification so /zoom reports correctly from boot without a physical home seek.
int ms_zoom_pos_for_mag(float mag) {
    MsLens l = MS_LENS_DEFAULT;
    return ms_lens_zoom_pos_for_mag(&l, mag);
}

int ms_lens_zoom_pos_for_mag(const MsLens *l, float mag) {
    if (mag > l->mag_max) mag = l->mag_max;
    if (mag < MS_MAG_MIN) mag = MS_MAG_MIN;
    float f = (l->mag_max - mag) / (l->mag_max - MS_MAG_MIN);   // from the TELE end
    if (l->zoom_reversed) f = 1.0f - f;
    return (int)(f * l->zoom_max + 0.5f);
}
