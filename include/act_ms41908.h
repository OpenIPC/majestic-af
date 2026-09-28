// MS41908M backend — the pure, hardware-free helpers, exposed so the unit test
// (tests/actuator_test.c) can pin the verb mapping, the soft-limit clamp and the
// zoom->magnification curve without a camera. The transport (SPI + /dev/mem) and
// the stepping thread stay private to src/act_ms41908.c.

#ifndef MAJESTIC_AF_ACT_MS41908_H
#define MAJESTIC_AF_ACT_MS41908_H

#include <stdbool.h>

#include "proto.h"

// Soft travel limits, in microsteps, from libxmaf's motor_config_register for
// LENS_LH13_FHD_X16 (the values the ms41908-lens tool proved on hardware).
#define MS_ZOOM_MAX 2610
#define MS_FOCUS_MAX 1280

// Magnification the zoom stepper spans, WIDE..TELE. Placeholder for a 16x lens;
// the true curve is calibrated on hardware (see the backend's header comment).
#define MS_MAG_MIN 1.0f
#define MS_MAG_MAX 16.0f

// Stepping cadence. Each move_axis burst is held to MS_BURST_MS of wall time
// (see the backend), whether or not the completion ISR fires, so steps track time
// and the focus mechanics handed to af2 stay exact. MS_STEP_BURST microsteps per
// burst sets the focus resolution. The absolute cadence is calibrated on hardware;
// what matters here is that the travel figures are DERIVED from it, not guessed.
//
// LARGE bursts on purpose: the AN41908A emits the loaded micro-steps at its own
// PPS rate, so a burst of N steps IS a continuous N/PPS-second glide, not a jump.
// The audible click is the per-burst re-latch (VD_FZ pulse + ctrl rewrite), so
// fewer, longer bursts click far less for the SAME speed and travel -- 40 steps
// over 75 ms re-latches ~13x/s where 8 over 15 ms did ~66x/s. MS_STEP_BURST stays
// <= 63 (the ctrl step field) and divides MS_FOCUS_MAX so MS_TRAVEL_MS is exact.
#define MS_STEP_BURST 40
#define MS_BURST_MS 75
// Focus mechanics for af2's timed model, in ms of travel, derived from the cadence
// so dead-reckoning lands on real step counts. backlash is a placeholder.
#define MS_TRAVEL_MS ((MS_FOCUS_MAX / MS_STEP_BURST) * MS_BURST_MS)
#define MS_TRAVEL_MAX_MS (MS_TRAVEL_MS + MS_TRAVEL_MS / 4)
#define MS_BACKLASH_MS 150

// verb -> axis + direction. Sets *is_zoom and *dir (+1/-1) and returns true for a
// motion verb (near/far/tele/wide); returns false for stop and for any verb this
// lens does not carry (pan/tilt, day/night).
bool ms_verb_axis(enum PtzVerb v, bool *is_zoom, int *dir);

// Clamp a step of magnitude `want` (>=0) in direction `dir` from `pos` to the
// closed range [0, max]. Returns the allowed magnitude — 0 when already at the
// stop in that direction, so the motor is never driven past an end stop.
int ms_clamp_step(int pos, int dir, int want, int max);

// Zoom step position (0..MS_ZOOM_MAX) -> magnification (MS_MAG_MIN..MS_MAG_MAX),
// monotonic and clamped at the ends.
float ms_zoom_mag(int zoom_pos);

// Inverse of ms_zoom_mag(): magnification -> step position, clamped to the ends.
int ms_zoom_pos_for_mag(float mag);

#endif
