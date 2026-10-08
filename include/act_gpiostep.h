// gpiostep backend — the pure, hardware-free helpers, exposed so the unit test
// (tests/gpiostep_test.c) can pin the board config parsing, the verb mapping and
// the soft-limit clamp without a camera. The transport (/dev/motorDev) and the
// stepping thread stay private to src/act_gpiostep.c.

#ifndef MAJESTIC_AF_ACT_GPIOSTEP_H
#define MAJESTIC_AF_ACT_GPIOSTEP_H

#include <stdbool.h>

#include "proto.h"

// The board description, read from GS_CONF_PATH. Everything here is a property of
// the camera, not of this plugin: which way the coils turn the head, how far it
// can turn before the gearbox hits its stop, and how fast the motors may be
// stepped without losing steps. A board that ships no file gets the defaults,
// which drive the motors but know no limits.
#define GS_CONF_PATH "/etc/gpiostep.conf"

enum { GS_PAN = 0, GS_TILT = 1, GS_AXES = 2 };

typedef struct GsConfig {
    // Full travel stop to stop, in gpiostep steps (one 8-phase cycle each);
    // 0 = unknown, so no soft limit and no homing on that axis.
    int travel[GS_AXES];
    // The sign of a raw step that turns the head left (pan) or up (tilt):
    // +1 or -1. Wiring decides it, and so does how the camera is mounted.
    int left_sign;
    int up_sign;
    // Per-microstep delay handed to the module, in microseconds.
    int delay_us[GS_AXES];
    // The same for the homing seek. Homing drives each axis into its gearbox
    // stop and stalls there for as long as the overshoot lasts, and a head
    // slammed into a stop at speed can slip somewhere homing cannot see -- on a
    // GK7205V510 head the view moved for good relative to the stops. So the
    // seek runs at this rate whatever delay_us is set to (see gs_home_delay()).
    int home_delay_us[GS_AXES];
    // Seek both stops once per boot to learn where the head is.
    bool home;
} GsConfig;

void gs_config_defaults(GsConfig *c);

// Apply one `key=value` line. Blank lines and `#` comments are accepted and
// change nothing. Returns false for a line that is neither, or for a value out
// of range, leaving the config as it was.
bool gs_config_line(GsConfig *c, const char *line);

// The delay the homing seek steps an axis at: home_delay_us, or delay_us when
// that is slower still. Never faster than either.
int gs_home_delay(const GsConfig *c, int axis);

// Steps per ioctl while homing at `delay_us`: long enough not to waste time on
// ioctls, short enough (GS_HOME_CHUNK_US, ~0.4 s) that a stop or a shutdown
// during the seek is not held up by one -- at least 1 step, at most 20.
#define GS_HOME_CHUNK_US 400000
int gs_home_chunk(int delay_us);

// verb -> axis + raw step direction (+1/-1, already signed by the config).
// Returns false for stop and for every verb a pan/tilt head does not carry.
bool gs_verb_axis(const GsConfig *c, enum PtzVerb v, int *axis, int *dir);

// Clamp a move of `want` (>= 0) steps in direction `dir` from `pos` to [0, travel].
// Returns the allowed magnitude: 0 at the stop in that direction. A travel of 0
// (unknown) allows the whole move.
int gs_clamp_step(int pos, int dir, int want, int travel);

// Read a saved position, "pan tilt", into `pos`, marking in `known` the axes it
// can vouch for: an axis with a travel and a value inside [0, travel]. An axis
// with no travel is never known -- there is nothing to measure it against.
// Returns how many axes are known.
int gs_parse_pos(const GsConfig *c, const char *line, int pos[GS_AXES],
                 bool known[GS_AXES]);

#endif
