// af3 — step-based bracket-and-return contrast autofocus, for a MICROSTEP lens whose
// backend can move an exact number of steps and hold position (the MS41908M). This is
// the Xiongmai libxmaf method, documented in OpenIPC/motors ms41908-lens/AUTOFOCUS.md:
// a stepper is NOT swept and sampled on the fly (that smears the focus statistic with
// motion). It is stepped, LET SETTLE, then measured — and the search brackets the peak,
// records the best (position, focus-value) seen, and drives back to it, bounded so it
// converges and stops instead of hunting.
//
// This is deliberately separate from af2 (the 85H50AI UART lens, continuous drive, timed
// dead-reckoning). af2's model cannot express a sub-burst microstep or a settle-then-read,
// and must not regress; a step lens gets af3, a continuous one keeps af2. Both are iterated
// against an offline model (tests/af3_model.c here) before any hardware.
//
// Why the vendor method, restated as the four things a naive hill-climb gets wrong:
//   1. It reads the focus value only AFTER the lens has settled (settle_frames). Reading a
//      frame after a move samples the lens mid-transit and chases a settling transient.
//   2. It records the best (pos, fv) and drives back to it — it does not stop where it
//      happened to notice the peak.
//   3. Coarse peak detection uses a drop accumulator that RESETS on any recovery, so a
//      transient dip cannot trip an early stop.
//   4. It is BOUNDED: after a couple of reversals it commits to the recorded best and stops.
//      That bound is why the motor falls silent instead of clicking around the peak forever.

#ifndef AF3_H
#define AF3_H

enum { AF3_NEAR = -1, AF3_STOP = 0, AF3_FAR = +1 };

typedef struct {
    // Move EXACTLY n (>0) microsteps in dir (AF3_FAR/AF3_NEAR). Returns the number of
    // microsteps actually advanced — less than n (down to 0) when an end stop is reached.
    // Blocking: it returns once the motor has been commanded the move (the caller then
    // waits settle_frames before measuring). The backend takes up reversal backlash itself.
    int (*step)(void *ctx, int dir, int n);
    unsigned (*fv)(void *ctx);            // focus statistic (bigger = sharper)
    long (*now_ms)(void *ctx);            // monotonic clock
    void (*sleep_ms)(void *ctx, long ms);
    void *ctx;
} Af3IO;

typedef struct {
    long focus_steps;     // full near<->far travel in microsteps (MS_FOCUS_MAX)
    long backlash_steps;  // gear slack taken up on a direction reversal, in microsteps;
                          // used to size the final approach so it lands from one side
    long frame_ms;        // one sensor frame / VD period in ms (drives the settle wait)
    int  settle_frames;   // frames to wait after a move before trusting fv (vendor: 3-4)
    int  fv_samples;      // fv reads to median per stationary measurement
    long budget_ms;       // hard time cap for the whole pass
    // cooperative cancel: when non-NULL and *cancel turns non-zero the pass abandons its
    // remaining moves and returns promptly (a fresh zoom / a pad press preempts it).
    const volatile int *cancel;
    // known focus position in microsteps from the near stop, carried across passes;
    // < 0 = unknown. af3 does not depend on it being right — it searches from wherever
    // the lens is — but a good value lets a re-AF start near the peak.
    long in_pos;
    // true only for a WARM re-AF: a prior pass focused this same zoom and the lens has not been
    // disturbed since, so the start may already BE the crest — af3 then checks for that and skips
    // the sweep if so. FALSE for a cold pass (just homed to the near stop, definitely NOT focused)
    // and after a zoom (focus displaced): there the "already on the peak?" test must never fire, or
    // it mistakes the floor at the stop for a peak and lands there. The engine sets this.
    int warm;
    // for a warm re-AF, the focus value at or above which the lens is taken to be ALREADY in focus:
    // af3 reads once WITHOUT moving and, if the reading clears this bar, holds position and returns.
    // This is essential on a razor-thin crest, where even the find-direction probes would step off
    // the peak and backlash makes the tiny correction back impossible — so a re-AF of an already
    // sharp image must not move at all. 0 disables (always search). The engine sets it from the
    // last pass's peak. Ignored when !warm.
    unsigned hold_fv;
    // optional trajectory trace, one call per measurement (dead-reckoned pos, median fv).
    void (*trace)(void *ctx, long pos, unsigned fv);
    void *trace_ctx;

    // outputs
    unsigned out_peak_fv;   // fv where it landed
    unsigned out_peak_seen; // best (median) fv observed during the pass
    int  out_steps;         // measurements taken (diagnostic)
    int  out_reversals;     // motor direction reversals (the journey; a hunt would be many)
    int  out_found_crest;   // 1 = a real crest above the statistic's floor was recognised;
                            // 0 = no gradient to work with, landing is bookkeeping only
    long out_pos;           // landed focus position in microsteps, to carry forward
} Af3Params;

// Run one pass. Returns the final (landed) focus value. Never blocks beyond budget_ms.
unsigned af3_run(Af3IO *io, Af3Params *p);

#endif
