// af2 — parfocal autofocus. See af2.h for the rationale. The focus peak position moves with
// zoom along a calibrated curve; af2 uses the curve to get NEAR the peak — from the far side —
// then makes ONE smooth continuous sweep onto the crest and stops (sweep_to_crest). It never
// hill-climbs or hunts around the peak: the lens goes one way, slips just past focus, settles
// back once. TRACK approaches from the overshoot the zoom left (sweep NEAR); COLD seeks the near
// stop and sweeps FAR. Iterated against the offline model in scratchpad/model/ (which now scores
// the JOURNEY — reversal count — not just the landed sharpness) before any hardware.

#include <majestic/af2.h>

// Calibrated zoom->focus curve, measured on the 85H50AI 2026-09-02 (btzoom to a zoom, read
// the magnification cleanly from the reader, afcap zsweep for the focus peak): focus-peak
// position in MOTION-ms of FAR drive from the near stop vs magnification, distant scene.
// NONLINEAR — flat at wide, steepening toward tele. The raw sweep drive-times (measured from
// the near stop, so the first FAR pulse is a reversal) were {0,1920,3840,6400,9760,17120,
// 27840}; the measured reversal backlash (~400 ms, afcap backlash from a clean power-up) is
// subtracted (clamped >=0) so these are true motion, matching drive_focus() which pays that
// same 400 ms back on a reversal. Getting this backlash figure right is load-bearing: the
// peaks are NARROW, so a curve off by more than a peak-width seeds the trim in the flat floor
// where there is no gradient. Piecewise-linear interpolation, clamped at the ends.
long af2_parfocal_foc(float mag) {
    static const struct { float mag; long foc; } C[] = {
        {1.0f, 0},    {1.8f, 1520},  {2.2f, 3440},  {2.6f, 6000},
        {3.1f, 9360}, {3.9f, 16720}, {5.0f, 27440},
    };
    int n = (int)(sizeof(C) / sizeof(C[0]));
    if (mag <= C[0].mag) return C[0].foc;
    if (mag >= C[n - 1].mag) return C[n - 1].foc;
    for (int i = 1; i < n; i++) {
        if (mag <= C[i].mag) {
            float t = (mag - C[i - 1].mag) / (C[i].mag - C[i - 1].mag);
            return C[i - 1].foc + (long)(t * (C[i].foc - C[i - 1].foc) + 0.5f);
        }
    }
    return C[n - 1].foc;
}

typedef struct {
    AfIO *io;
    AfParams *p;
    long deadline;
    unsigned peak_seen;
    int last_dir;   // last non-STOP drive direction, for backlash accounting
    long pos;       // dead-reckoned focus position, ms of FAR travel from the near stop
    long travel;    // near<->far travel estimate, for clamping pos
} S;

static long now(S *s) { return s->io->now_ms(s->io->ctx); }
static void nap(S *s, long ms) { if (ms > 0) s->io->sleep_ms(s->io->ctx, ms); }
static void motor(S *s, int d) { s->io->drive(s->io->ctx, d); }
static int cancelled(S *s) { return s->p->cancel && *s->p->cancel; }

// Median of n reads spaced ~one sensor frame apart, measured STOPPED (no motion smear).
static unsigned fv_med(S *s) {
    int n = s->p->fv_samples > 0 ? s->p->fv_samples : 1;
    if (n > 9) n = 9;
    unsigned a[9];
    long frame = s->p->fv_frame_ms > 0 ? s->p->fv_frame_ms : 40;
    for (int i = 0; i < n; i++) {
        a[i] = s->io->fv(s->io->ctx);
        if (i < n - 1) nap(s, frame);
    }
    for (int i = 1; i < n; i++) {
        unsigned x = a[i]; int j = i - 1;
        while (j >= 0 && a[j] > x) { a[j + 1] = a[j]; j--; }
        a[j + 1] = x;
    }
    unsigned m = a[n / 2];
    if (m > s->peak_seen) s->peak_seen = m;
    s->p->out_steps++;
    if (s->p->trace) s->p->trace(s->p->trace_ctx, s->pos, m);
    return m;
}

// Drive focus by a signed amount of MOTION (ms; + = FAR). Backlash is added when the
// direction reverses, so the caller names the real distance to travel and the lens gets it.
static void drive_focus(S *s, long signed_ms) {
    if (signed_ms == 0 || cancelled(s)) return;
    int dir = signed_ms > 0 ? AF2_FAR : AF2_NEAR;
    long mag = signed_ms > 0 ? signed_ms : -signed_ms;
    long extra = (dir != s->last_dir && s->last_dir != 0) ? s->p->backlash_ms : 0;
    motor(s, dir);
    // Nap the drive in short slices so a cancel (a fresh zoom) stops the motor within a slice
    // rather than after a multi-second move — it must release the UART well inside the zoom
    // caller's lock-retry window.
    for (long left = mag + extra; left > 0 && !cancelled(s); left -= 200)
        nap(s, left < 200 ? left : 200);
    motor(s, AF2_STOP);
    nap(s, s->p->settle_ms);
    s->last_dir = dir;
    s->pos += signed_ms;                              // dead-reckon the new position
    if (s->pos < 0) s->pos = 0;
    if (s->pos > s->travel) s->pos = s->travel;
}

// Blind timed drive into an end stop: a repeatable position reference (the motor stalls
// against the stop, so the dead-reckoned position is re-anchored exactly to 0 or travel).
static void seek_stop(S *s, int dir) {
    motor(s, dir);
    long until = now(s) + s->p->travel_max_ms;
    while (now(s) < until && now(s) < s->deadline && !cancelled(s)) nap(s, 200);
    motor(s, AF2_STOP);
    nap(s, s->p->settle_ms);
    s->last_dir = dir;
    s->pos = dir > 0 ? s->travel : 0;                 // re-anchor at the stop
}

// Smooth single-direction approach to the peak — the whole point is NO hunting. Run the motor
// CONTINUOUSLY toward the crest: cover the bulk of the gap blind, then sample FV on the fly and
// stop the moment FV has clearly crested; finally ONE short move back onto the best FV seen. The
// lens travels one way, slips just past focus, and settles back once — it never oscillates
// around the peak the way a hill-climb does. FV, not the backlash-prone dead-reckoning, decides
// where the crest is, so the landing is immune to backlash and to an inexact start. `blind_ms`
// is motion to cover before sampling begins (smoother, and the flat floor carries no signal
// anyway). Returns the FV where it lands.
static unsigned sweep_to_crest(S *s, int dir, long blind_ms, long budget_ms) {
    long extra = (dir != s->last_dir && s->last_dir != 0) ? s->p->backlash_ms : 0;
    long start = s->pos;
    long frame = s->p->fv_frame_ms > 0 ? s->p->fv_frame_ms * 2 : 80;
    motor(s, dir);
    long t0 = now(s);
    // Blind prefix: the reversal backlash plus the bulk of the approach, in cancel-checkable
    // slices (a fresh zoom must still be able to stop the motor promptly).
    for (long left = blind_ms + extra; left > 0 && !cancelled(s); left -= 200)
        nap(s, left < 200 ? left : 200);
    long st0 = now(s);
    unsigned floor = s->io->fv(s->io->ctx);          // FV where sampling begins (off-peak floor)
    unsigned top = floor;
    long top_on = now(s) - t0;
    long on = top_on;
    int rose = 0, plateau = 0;
    while (now(s) - st0 < budget_ms && now(s) < s->deadline && !cancelled(s)) {
        nap(s, frame);
        unsigned v = s->io->fv(s->io->ctx);
        on = now(s) - t0;
        if (v > s->peak_seen) s->peak_seen = v;
        if (v < floor) floor = v;
        s->p->out_steps++;
        if (s->p->trace) {
            long moved = on - extra; if (moved < 0) moved = 0;
            s->p->trace(s->p->trace_ctx, start + (long)dir * moved, v);
        }
        if (top > floor + 40) rose = 1;              // a real peak (not integer-floor noise) exists
        if (v > top) { top = v; top_on = on; plateau = 0; }
        else if (rose && (long)v * 100 < (long)top * 85) {
            // Crested and clearly fell: the crest is behind us. Stop; the return below lands
            // back on it. (An x1.0 crest on the near stop we just left is this case too — FV
            // only ever falls, top stays the start value at top_on~0, we return to it.)
            break;
        } else if (rose && (long)v * 100 >= (long)top * 96) {
            // FV flat at the top, not climbing: EITHER the peak is parked on an end stop (a low
            // zoom's focus sits on the near stop, where the lens clamps and FV goes flat forever)
            // OR this is just a flat SHOULDER on the way to a higher crest further along (a wide
            // scene has these). Only the clamp holds flat for a long time, so require a long run
            // before stopping — a brief shoulder is passed, and the sweep goes on to the real
            // crest. When it does stop, we are ON the crest: no overshoot to undo.
            if (++plateau >= 14) { top_on = on; break; }
        } else plateau = 0;
    }
    motor(s, AF2_STOP);
    nap(s, s->p->settle_ms);
    s->last_dir = dir;
    long moved = on - extra; if (moved < 0) moved = 0;
    long crest_pos = start + (long)dir * (top_on - extra);   // where FV actually peaked
    if (crest_pos < 0) crest_pos = 0;
    if (crest_pos > s->travel) crest_pos = s->travel;
    s->pos = start + (long)dir * moved;              // dead-reckon where we stopped
    if (s->pos < 0) s->pos = 0;
    if (s->pos > s->travel) s->pos = s->travel;
    // Return onto the crest, FV-GUIDED so the reversal's backlash — which is not constant, and
    // grows sharply toward the near stop — cannot displace the landing. A counted hop back
    // either falls short (the slack swallows it, near the stop) or overshoots; and stopping AFTER
    // FV falls always overshoots by the detection lag. So reverse and climb back UP the flank,
    // and stop the instant FV reaches the crest value again — on the rising side, one clean turn,
    // no counted distance and no overshoot. The reversal slack (FV flat at the stop value) sits
    // well below the crest, so it can't trip the stop early.
    long back = on - top_on;                         // motion travelled past the crest
    if (back > 0 && top > floor + 40) {
        // 95%, not 98%. The threshold is a fraction of a peak measured on the
        // FORWARD sweep, and the same crest reads lower coming back: traced at
        // 96.7% on an 85H50AI, which slipped under a 98% bar and sent the lens
        // over the crest and down the far flank instead. Stopping on the RISING
        // flank a few percent short of the crest costs a sliver of sharpness;
        // missing the bar cost thirty percent of it.
        unsigned target = (unsigned)((long)top * 95 / 100);
        motor(s, -dir);
        long rlimit = now(s) + back + 1000 + 1500;   // bound: overshoot + worst slack + margin
        // The absolute threshold cannot be the only way out of this loop, and
        // that was the whole fault: `target` is a fraction of the peak measured
        // on the FORWARD sweep, and the same crest reads a little lower coming
        // back — different backlash, different sampling phase, plain noise.
        // Traced on an 85H50AI: forward peak 12371 at pos 10930, the return
        // climbed to 11969 and stopped rising. 11969 is 96.7% of the peak and
        // the target was 98%, so the test never fired, the motor kept going,
        // and the lens sailed over the crest and down the far flank until the
        // time bound expired — finishing at 8623, thirty percent below the
        // sharpest point it had just measured. Every pass that "found the peak
        // and parked well below it" is this, on both paths.
        //
        // So watch for the crest on the way back too. Climbing then clearly
        // falling means it is behind us, and stopping there costs a sample or
        // two of overshoot — tens of milliseconds out of a 38 s travel —
        // instead of the whole far flank.
        unsigned rtop = 0;
        int back_rose = 0;
        while (now(s) < rlimit && now(s) < s->deadline && !cancelled(s)) {
            nap(s, frame / 2 > 0 ? frame / 2 : 40);
            unsigned v = s->io->fv(s->io->ctx);
            if (v > s->peak_seen) s->peak_seen = v;
            s->p->out_steps++;
            if (s->p->trace) s->p->trace(s->p->trace_ctx, crest_pos, v);
            if (v > rtop) rtop = v;
            if (rtop > floor + 40) back_rose = 1;
            if (v >= target) break;                  // climbed back onto the crest — stop here
            if (back_rose && (long)v * 100 < (long)rtop * 92) {
                break;                               // crested on the way back; it is behind us
            }
        }
        motor(s, AF2_STOP);
        nap(s, s->p->settle_ms);
        s->last_dir = -dir;
        s->pos = crest_pos;
    }
    return fv_med(s);
}

unsigned af2_run(AfIO *io, AfParams *p) {
    if (p->travel_max_ms <= 0) p->travel_max_ms = 42000;
    if (p->backlash_ms <= 0) p->backlash_ms = 400;
    if (p->settle_ms <= 0) p->settle_ms = 160;
    if (p->budget_ms <= 0) p->budget_ms = 55000;
    if (p->fv_samples <= 0) p->fv_samples = 5;
    if (p->fv_frame_ms <= 0) p->fv_frame_ms = 40;

    S s = {.io = io, .p = p, .peak_seen = 0, .last_dir = 0};
    s.travel = p->travel_ms > 0 ? p->travel_ms : 38000;
    s.pos = p->in_focus_pos;
    s.deadline = now(&s) + p->budget_ms;
    p->out_steps = 0;

    float mag = p->mag_now >= 1.0f ? p->mag_now : 1.0f;
    long target = af2_parfocal_foc(mag);              // absolute parfocal peak for this zoom
    const long PRE = 4000;                            // start a sweep this far to one side of the
                                                      // curve target — must exceed the largest
                                                      // scene offset so the start is truly off-peak
    const long SWEEP = 8000;                          // FV-sampled reach past the blind approach

    unsigned final;
    if (p->in_focus_pos >= 0) {
        // TRACK: after a zoom the lens sits FAR of the new peak (the measured ~constant
        // overshoot). Approach it from the FAR side in ONE smooth NEAR sweep that stops at the
        // crest — no hill-climb, no oscillation. Cover the bulk blind, sample the rest. If the
        // carried position is NOT clearly FAR of the peak (a same-zoom re-AF), back off FAR to
        // the sweep's start first.
        long begin = target + PRE;                    // FAR-side start of the NEAR sweep
        if (s.pos >= begin) {
            final = sweep_to_crest(&s, AF2_NEAR, s.pos - begin, SWEEP);
        } else {
            drive_focus(&s, begin - s.pos);           // back off FAR to the sweep start
            final = sweep_to_crest(&s, AF2_NEAR, 0, SWEEP);
        }
        p->out_path = 1;
    } else {
        // COLD: position unknown. Seek the NEAR stop (an exact hard reference), cover the bulk
        // of the way to the curve target blind, then one smooth FAR sweep onto the crest.
        seek_stop(&s, AF2_NEAR);
        long blind = target > PRE ? target - PRE : 0;
        final = sweep_to_crest(&s, AF2_FAR, blind, SWEEP);
        p->out_path = 2;
    }

    p->out_peak_fv = final;
    p->out_peak_seen = s.peak_seen;
    p->out_focus_pos = s.pos;                         // dead-reckoned position to carry forward
    p->out_mag = p->mag_now;
    motor(&s, AF2_STOP);
    return final;
}
