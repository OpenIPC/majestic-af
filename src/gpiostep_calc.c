// The gpiostep backend's pure logic: the board config parser, verb -> axis, and the
// soft-limit clamp. No device, no thread, no HAL seam, so tests/gpiostep_test.c
// links it with nothing stubbed.

#include "act_gpiostep.h"

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

void gs_config_defaults(GsConfig *c) {
    c->travel[GS_PAN] = 0;
    c->travel[GS_TILT] = 0;
    c->left_sign = 1;
    c->up_sign = 1;
    c->delay_us[GS_PAN] = 2000;
    c->delay_us[GS_TILT] = 3000;
    // The rates every head has homed at so far, before running speeds became
    // configurable upwards of them.
    c->home_delay_us[GS_PAN] = 2000;
    c->home_delay_us[GS_TILT] = 3000;
    c->home = false;
    c->frame_steps[GS_PAN] = 0;
    c->frame_steps[GS_TILT] = 0;
}

static bool parse_int(const char *s, long lo, long hi, long *out) {
    char *end;
    long v = strtol(s, &end, 10);
    while (*end && isspace((unsigned char)*end)) {
        end++;
    }
    if (end == s || *end || v < lo || v > hi) {
        return false;
    }
    *out = v;
    return true;
}

bool gs_config_line(GsConfig *c, const char *line) {
    while (*line && isspace((unsigned char)*line)) {
        line++;
    }
    if (!*line || *line == '#') {
        return true;
    }
    const char *eq = strchr(line, '=');
    if (!eq) {
        return false;
    }
    size_t klen = (size_t)(eq - line);
    while (klen && isspace((unsigned char)line[klen - 1])) {
        klen--;
    }
    const char *val = eq + 1;
    while (*val && isspace((unsigned char)*val)) {
        val++;
    }

    // Travel up to 100000 steps: the heads this was written for turn 580 and 170;
    // anything bigger than this is a typo, not a gearbox. Delays from 200 us (the
    // fastest a stepper of this class follows unloaded) to 100 ms.
    static const struct {
        const char *key;
        long lo, hi;
    } keys[] = {
        {"pan_travel", 0, 100000},  {"tilt_travel", 0, 100000},
        {"pan_left", -1, 1},        {"tilt_up", -1, 1},
        {"pan_delay_us", 200, 100000}, {"tilt_delay_us", 200, 100000},
        {"home", 0, 1},
        {"pan_home_delay_us", 200, 100000}, {"tilt_home_delay_us", 200, 100000},
        {"pan_frame_steps", 0, 2 * PTZ_STEPS_MAX}, {"tilt_frame_steps", 0, 2 * PTZ_STEPS_MAX},
    };
    for (size_t i = 0; i < sizeof keys / sizeof *keys; i++) {
        if (strlen(keys[i].key) != klen || strncmp(keys[i].key, line, klen)) {
            continue;
        }
        long v;
        if (!parse_int(val, keys[i].lo, keys[i].hi, &v)) {
            return false;
        }
        switch (i) {
        case 0: c->travel[GS_PAN] = (int)v; break;
        case 1: c->travel[GS_TILT] = (int)v; break;
        case 2:
            if (!v) return false;   // a sign, so +1 or -1, never 0
            c->left_sign = (int)v;
            break;
        case 3:
            if (!v) return false;
            c->up_sign = (int)v;
            break;
        case 4: c->delay_us[GS_PAN] = (int)v; break;
        case 5: c->delay_us[GS_TILT] = (int)v; break;
        case 6: c->home = v != 0; break;
        case 7: c->home_delay_us[GS_PAN] = (int)v; break;
        case 8: c->home_delay_us[GS_TILT] = (int)v; break;
        case 9: c->frame_steps[GS_PAN] = (int)v; break;
        case 10: c->frame_steps[GS_TILT] = (int)v; break;
        }
        return true;
    }
    return false;
}

int gs_home_delay(const GsConfig *c, int axis) {
    return c->home_delay_us[axis] > c->delay_us[axis] ? c->home_delay_us[axis]
                                                      : c->delay_us[axis];
}

int gs_speed_delay(const GsConfig *c, int axis, int speed) {
    int d = c->delay_us[axis];
    if (speed <= 0 || speed >= 63) {
        return d;
    }
    // One step is 8 microsteps; it must fit a running ioctl's budget.
    long floor_us = GS_RUN_CHUNK_US / 8;
    long cap = d > floor_us ? d : floor_us;
    long slow = (long)d * 63 / speed;
    return slow > cap ? (int)cap : (int)slow;
}

int gs_run_chunk(int delay_us) {
    long per_step = 8L * (delay_us > 0 ? delay_us : 1);
    long n = GS_RUN_CHUNK_US / per_step;
    return n < 1 ? 1 : n > GS_CHUNK_MAX ? GS_CHUNK_MAX : (int)n;
}

int gs_home_chunk(int delay_us) {
    // A step is one 8-phase cycle: 8 microsteps at delay_us each.
    long per_step = 8L * (delay_us > 0 ? delay_us : 1);
    long n = GS_HOME_CHUNK_US / per_step;
    return n < 1 ? 1 : n > 20 ? 20 : (int)n;
}

bool gs_verb_dirs(const GsConfig *c, enum PtzVerb v, int dir[GS_AXES]) {
    int pan, tilt;
    if (!ptz_verb_dirs(v, &pan, &tilt)) {
        return false;
    }
    // ptz_verb_dirs says right and up are +1; the board says which raw sign
    // turns the head left and which up.
    dir[GS_PAN] = pan * -c->left_sign;
    dir[GS_TILT] = tilt * c->up_sign;
    return true;
}

void gs_arm_axes(const int dir[GS_AXES], int steps, int speed, int cur_dir[GS_AXES],
                 int cur_left[GS_AXES], int cur_speed[GS_AXES], bool armed[GS_AXES]) {
    for (int a = 0; a < GS_AXES; a++) {
        armed[a] = false;
        if (!dir[a] && steps > 0 && cur_dir[a] && cur_left[a] > 0) {
            continue;   // that axis's own count goes on, at its own speed
        }
        cur_dir[a] = dir[a];
        cur_left[a] = dir[a] ? steps : -1;
        if (dir[a]) {
            cur_speed[a] = speed;
        }
        armed[a] = dir[a] != 0;
    }
}

int gs_next_axis(const int dir[GS_AXES], int last) {
    bool pan = dir[GS_PAN] != 0, tilt = dir[GS_TILT] != 0;
    if (pan && tilt) {
        return last == GS_PAN ? GS_TILT : GS_PAN;
    }
    return pan ? GS_PAN : tilt ? GS_TILT : -1;
}

int gs_parse_pos(const GsConfig *c, const char *line, int pos[GS_AXES],
                 bool known[GS_AXES]) {
    int v[GS_AXES];
    int n = 0;
    bool parsed = sscanf(line, "%d %d", &v[GS_PAN], &v[GS_TILT]) == 2;
    for (int a = 0; a < GS_AXES; a++) {
        known[a] = parsed && c->travel[a] > 0 && v[a] >= 0 && v[a] <= c->travel[a];
        if (known[a]) {
            pos[a] = v[a];
            n++;
        }
    }
    return n;
}

int gs_clamp_step(int pos, int dir, int want, int travel) {
    if (want <= 0) {
        return 0;
    }
    if (travel <= 0) {
        return want;
    }
    int room = dir > 0 ? travel - pos : pos;
    if (room <= 0) {
        return 0;
    }
    return want < room ? want : room;
}
