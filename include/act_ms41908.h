// MS41908M backend — the pure, hardware-free helpers, exposed so the unit test
// (tests/actuator_test.c) can pin the verb mapping, the soft-limit clamp and the
// zoom->magnification curve without a camera. The transport (SPI + /dev/mem) and
// the stepping thread stay private to src/act_ms41908.c.

#ifndef MAJESTIC_AF_ACT_MS41908_H
#define MAJESTIC_AF_ACT_MS41908_H

#include <stdbool.h>
#include <stddef.h>

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
// The AN41908A emits a burst's micro-steps at the rate its step cycle sets, and the
// step cycle is chosen per burst so the burst turns for MS_ROT_US of the cadence (see
// there): a run of bursts is then one continuous glide. At a fixed full rate a burst
// finished early and the motor stood still until the next VD -- a stop and a restart
// from standstill at full rate every 75 ms, which clicked and lost steps FAR.
// MS_STEP_BURST stays <= 63 (the ctrl step field) and divides MS_FOCUS_MAX so
// MS_TRAVEL_MS is exact.
#define MS_STEP_BURST 40
#define MS_BURST_MS 75
// How long each burst turns the motor, inside the MS_BURST_MS cadence. The AN41908A
// application note (rev 0.06, p. 17) asks for exactly this: "when driving the motor
// continuously for several VD period, it is best to match rotation time (per VD) to VD
// period" -- INTCT x PSUM x 24 = OSCIN / VD frequency -- and a rotation shorter than the
// period "will cause discontinuous rotation". The step cycle (INTCT) is therefore worked
// out per burst from its size: a run of bursts turns without stopping, and a short af3
// step creeps through the cadence instead of jolting at full rate from standstill. Short
// of the cadence by enough for the VD pulse, the start-point waits and timer slack, since
// a rotation that overruns its VD period is cancelled by the chip -- steps lost silently.
#define MS_ROT_US 66000
// The chip's clock (OSCIN) is the board's, not known here: assumed the slowest plausible
// until a burst has measured it, so an assumption can only make a rotation shorter than
// MS_ROT_US, never long enough to be cancelled. A measurement outside the window is
// discarded.
#define MS_OSCIN_MIN_HZ 24000000u
#define MS_OSCIN_MAX_HZ 80000000u

// Focus mechanics for af2's timed model, in ms of travel, derived from the cadence
// so dead-reckoning lands on real step counts. backlash is a placeholder.
#define MS_TRAVEL_MS ((MS_FOCUS_MAX / MS_STEP_BURST) * MS_BURST_MS)
#define MS_TRAVEL_MAX_MS (MS_TRAVEL_MS + MS_TRAVEL_MS / 4)
#define MS_BACKLASH_MS 150

// The lens behind the chip. The defaults are LENS_LH13_FHD_X16, the 16x lens of
// the varifocal HI3516D_N81820 (the libxmaf figures above); a board with another
// lens on the same chip -- the 20x speed dome of OpenIPC/firmware#2385 -- sets its
// own from ptz.lens* (act_ms41908.c, ms_lens_from_config). Pure data, so the
// helpers below stay testable without a camera.
typedef struct MsLens {
    int zoom_max;        // zoom microsteps end to end
    int focus_max;       // focus microsteps end to end
    float mag_max;       // magnification at the tele end (mag MS_MAG_MIN at wide)
    bool zoom_reversed;  // the home stop (pos 0) is WIDE, not TELE
} MsLens;

// LENS_LH13_FHD_X16.
#define MS_LENS_DEFAULT ((MsLens){MS_ZOOM_MAX, MS_FOCUS_MAX, MS_MAG_MAX, false})

// verb -> axis + direction, on `l`: TELE drives toward the home stop on the
// default lens, away from it on a reversed one.
bool ms_lens_verb_axis(const MsLens *l, enum PtzVerb v, bool *is_zoom, int *dir);
// Zoom step position (0..l->zoom_max) <-> magnification (MS_MAG_MIN..l->mag_max).
float ms_lens_zoom_mag(const MsLens *l, int zoom_pos);
int ms_lens_zoom_pos_for_mag(const MsLens *l, float mag);

// verb -> axis + direction. Sets *is_zoom and *dir (+1/-1) and returns true for a
// motion verb (near/far/tele/wide); returns false for stop and for any verb this
// lens does not carry (pan/tilt, day/night).
bool ms_verb_axis(enum PtzVerb v, bool *is_zoom, int *dir);

// Clamp a step of magnitude `want` (>=0) in direction `dir` from `pos` to the
// closed range [0, max]. Returns the allowed magnitude — 0 when already at the
// stop in that direction, so the motor is never driven past an end stop.
int ms_clamp_step(int pos, int dir, int want, int max);

// How close a move may take an axis to its home stop: positions count from the stop,
// but nothing drives below this. Driven into at speed, the stop clicks -- the
// HI3516D_N81820 dome's focus did from 60 microsteps out, and was silent from 100 --
// so the stop is only ever met by homing, which is how the origin is found.
#define MS_STOP_CLEAR 100

// ms_clamp_step over [lo, max]: a position below lo (an axis just homed onto its stop)
// may move away from it, never further toward it.
int ms_clamp_span(int pos, int dir, int want, int lo, int max);

// How far a homing seek drives toward the home stop before it takes the axis as
// there. `known` is where the last run left the axis (<0 for no idea): with no
// idea, the whole travel, the only seek certain to reach the stop from the far
// end; with a figure, that plus a margin for steps the count may have lost, so a
// lens left close to its stop is not held against it for most of a travel.
int ms_home_seek(int known, int max);

// The steps a move may take on an axis with no origin yet: toward the home stop,
// where a seek finds one, as asked; away from it none, since nothing bounds the
// travel on that side until the origin is known.
int ms_unhomed_step(int dir, int want);

// The step cycle (INTCTxx) that turns a burst of `psum` (PSUMxx) for `rot_us` on a chip
// clocked at `oscin_hz`: INTCT x PSUM x 24 = OSCIN x rotation time. Within 1..65535.
int ms_burst_intct(int psum, unsigned oscin_hz, long rot_us);

// The chip clock a burst implies: `psum` at `intct` that turned for `rot_us`. 0 when the
// figure is outside [MS_OSCIN_MIN_HZ, MS_OSCIN_MAX_HZ] -- a timing that cannot be right.
unsigned ms_oscin_from_rotation(int intct, int psum, long rot_us);

// The lens's own record of where it stands (see act_ms41908.c, MS_LENS_STATE): one
// line, "<zoom> <focus> <zoom_max> <focus_max> <swap> rest|moving". Formats into buf.
void ms_lens_state_format(char *buf, size_t n, int zoom, int focus, const MsLens *l,
                          bool swap, bool rest);
// True, with the positions, only for a record of a lens at rest, made for this lens
// (the same travel and wiring) and inside its travel -- the only kind homing may skip
// on. A move that was under way when the record was made says nothing of where it
// stopped.
bool ms_lens_state_parse(const char *line, const MsLens *l, bool swap, int *zoom,
                         int *focus);

// Zoom step position (0..MS_ZOOM_MAX) -> magnification (MS_MAG_MIN..MS_MAG_MAX),
// monotonic and clamped at the ends.
float ms_zoom_mag(int zoom_pos);

// Inverse of ms_zoom_mag(): magnification -> step position, clamped to the ends.
int ms_zoom_pos_for_mag(float mag);

#endif
