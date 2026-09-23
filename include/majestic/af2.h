// af2 — parfocal autofocus. This lens is strongly PARFOCAL: the focus peak position moves
// with zoom (measured x1.0->near stop, x5.0->~27500 ms of a ~38 s travel, roughly linear).
// Most of the focus range is a flat, low, integer-quantized contrast floor with a sharp
// peak, so a blind contrast hunt cannot reliably cross the floor to a zoom-dependent peak.
// So af2 uses the calibrated zoom->focus curve, driven by the live magnification the lens
// MCU reports:
//   * TRACK (a zoom changed since the last focus): drive focus by the curve DIFFERENCE
//     peakFoc(mag_now) - peakFoc(mag_ref). This is a relative move — no absolute reference
//     needed — and to first order it is subject-distance independent, so it keeps whatever
//     was in focus in focus as you zoom (the vendor's "emits near/far on the fly"). A short
//     contrast trim nails the residual.
//   * COLD (no reference yet): seek the near stop (a repeatable reference), drive to the
//     nominal peakFoc(mag_now), then a wider contrast trim to cover the scene's distance
//     offset from the (infinity) calibration.
// Iterated against the offline model in scratchpad/model/ (parfocal curve + integer FV +
// flat floor + backlash + a scene offset from the calibration) before any hardware.

#ifndef AF2_H
#define AF2_H

enum { AF2_NEAR = -1, AF2_STOP = 0, AF2_FAR = +1 };

typedef struct {
    void (*drive)(void *ctx, int dir);   // dir: AF2_NEAR / AF2_STOP / AF2_FAR
    unsigned (*fv)(void *ctx);           // sample focus value (bigger = sharper)
    long (*now_ms)(void *ctx);           // monotonic clock
    void (*sleep_ms)(void *ctx, long ms);
    void *ctx;
} AfIO;

typedef struct {
    // mechanics (measured)
    long travel_max_ms;  // upper bound on full near<->far travel; caps the cold seek (44000)
    long travel_ms;      // best estimate of the actual near<->far travel (38000); lets a cold
                         // tele focus seek the FAR stop (closer to the nominal) instead of a
                         // slow near-stop seek + long far drive
    long backlash_ms;    // gear slack paid on a direction reversal (~400)
    long settle_ms;      // pause after a stop/reversal before measuring (160)
    long budget_ms;      // hard time cap for the whole pass
    int  fv_samples;     // FV reads to median per stationary measurement (5)
    long fv_frame_ms;    // spacing between those reads (40); the sweep samples at twice this
    // optional trajectory trace: invoked at every FV measurement with the dead-reckoned
    // position and the median FV. For offline diagnosis of a real pass; NULL disables.
    void (*trace)(void *ctx, long pos, unsigned fv);
    void *trace_ctx;
    // cooperative cancel: if non-NULL and *cancel turns non-zero, the pass abandons its
    // remaining moves and returns promptly. Used to preempt a focus that a fresh zoom has made
    // stale (it was driving toward the old magnification's peak). NULL = never cancelled.
    const volatile int *cancel;
    // zoom state (the curve is driven by this — 0 means "unknown")
    float mag_now;       // current lens magnification (>= 1.0)
    // dead-reckoned focus position (ms of FAR travel from the near stop), carried across
    // passes. Because there is no focus-position sensor, af2 tracks it by integrating every
    // focus command from a known origin (a near-stop seek). This lets each pass drive to the
    // ABSOLUTE parfocal position for the current zoom rather than relative to the last
    // (possibly imperfect) landing, which is what made a sequence of zoom tracks cascade.
    long in_focus_pos;   // current known focus position; < 0 = unknown (forces a cold seek)
    // outputs
    unsigned out_peak_seen; // best (median) FV observed during the pass
    unsigned out_peak_fv;   // FV where it landed
    int out_steps;          // measurements taken (diagnostic)
    int out_path;           // 1 = track (had a position), 2 = cold seek (diagnostic)
    int out_found_crest;    // 1 = the sweep recognised a real crest above the statistic's own
                            // floor. 0 means the pass had no gradient to work with and its
                            // landing is bookkeeping, not a measurement — the caller must not
                            // report that as a focused result.
    long out_focus_pos;     // dead-reckoned focus position after the pass, to feed back in —
                            // or < 0 when the pass has none to offer. A position is only
                            // meaningful WITH the magnification it was measured at, so a pass
                            // run without one anchors nothing and must not seed a later TRACK.
                            // This is the value to carry; out_landed_pos is the one to print.
    long out_landed_pos;    // where the lens actually ended up, always, anchored or not. A
                            // diagnostic only — never feed this back.
    float out_mag;          // mag_now (diagnostic)
} AfParams;

// Run one pass. Returns the final FV. Never blocks beyond budget_ms.
unsigned af2_run(AfIO *io, AfParams *p);

// Calibrated zoom->focus curve: focus-peak position (ms of FAR drive from the near stop)
// for a given magnification. Exposed so callers (and the model) share one calibration.
long af2_parfocal_foc(float mag);

#endif
