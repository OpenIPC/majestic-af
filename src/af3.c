// af3 — step-based bracket-and-return contrast autofocus. See af3.h for the rationale and
// OpenIPC/motors ms41908-lens/AUTOFOCUS.md for the vendor algorithm this reproduces. The shape:
//
//   find direction -> COARSE (accelerating steps, settle+read each, detect we passed the peak)
//                  -> FINE (small steps, bracket the crest, bounded reversals)
//                  -> RETURN to the recorded best (pos, fv), from a fixed side for backlash
//                  -> a short FV-guided 3-point landing so we sit on the local FV maximum.
//
// Everything is in MICROSTEPS and FRAMES. The one rule that separates this from a hill-climb:
// the focus value is read only after the lens has been let settle (settle_frames), because a
// reading taken right after a move is a motion transient, not sharpness.
//
// Iterated against tests/af3_model.c (a stepper lens with a Gaussian peak over an integer floor,
// reversal backlash, AND a post-move settle transient) before any hardware.

#include <majestic/af3.h>

typedef struct {
    Af3IO *io;
    Af3Params *p;
    long deadline;
    long T;              // focus travel (microsteps)
    long pos;            // dead-reckoned position, microsteps from the near stop
    int  last_nz;        // last non-zero drive direction, for the reversal count
    unsigned floor;      // lowest median FV seen this pass (the off-peak floor)
    unsigned best_fv;    // highest median FV seen
    long best_pos;       // position where best_fv was measured
    unsigned peak_seen;
    int crest;           // a real crest above the floor was recognised
} S;

static long now(S *s) { return s->io->now_ms(s->io->ctx); }
static void nap(S *s, long ms) { if (ms > 0) s->io->sleep_ms(s->io->ctx, ms); }
static int cancelled(S *s) { return (s->p->cancel && *s->p->cancel) || now(s) >= s->deadline; }

// A real crest, or the statistic's own quantisation noise? The focus value has no absolute
// scale (a wide/dim scene compresses the whole peak-to-floor range into a few dozen counts),
// so the bar is a fraction of the floor with a small absolute guard beneath it — the same rule
// af2 settled on. Everything downstream (the coarse drop bar, "found a crest") hangs off this.
#define AF3_RISE_MIN 8
static int peak_is_real(unsigned top, unsigned floorv) {
    if (top <= floorv) return 0;
    unsigned rise = top - floorv;
    unsigned bar = floorv >> 3;
    if (bar < AF3_RISE_MIN) bar = AF3_RISE_MIN;
    return rise >= bar;
}

// Move up to n microsteps in dir; the backend advances what it can (less at an end stop) and
// returns that, which is the ONLY end-stop signal — a caller compares it to n. `pos` is a FREE
// relative integrator, never clamped: when the pass starts with no known position (in_pos < 0)
// its origin is a guess offset from the real dead reckoning, so clamping it to [0,T] would fire
// the stop test at the wrong place. Everything downstream (best_pos, the landing moves) is
// relative, so the offset is harmless. Keeps the reversal count.
static int drive(S *s, int dir, int n) {
    if (n <= 0 || cancelled(s)) return 0;
    int moved = s->io->step(s->io->ctx, dir, n);
    if (moved < 0) moved = 0;
    s->pos += (long)dir * moved;
    if (dir != 0 && dir != s->last_nz) {
        if (s->last_nz != 0) s->p->out_reversals++;
        s->last_nz = dir;
    }
    return moved;
}

// Let the lens settle, then measure: median of a few FV reads one frame apart. Reading only
// after settle_frames is the whole point — before that the statistic is the move's transient.
static unsigned measure(S *s) {
    long frame = s->p->frame_ms > 0 ? s->p->frame_ms : 40;
    int settle = s->p->settle_frames > 0 ? s->p->settle_frames : 4;
    nap(s, frame * settle);                       // wait for the lens to stop ringing
    int n = s->p->fv_samples > 0 ? s->p->fv_samples : 3;
    if (n > 9) n = 9;
    unsigned a[9];
    for (int i = 0; i < n; i++) {
        a[i] = s->io->fv(s->io->ctx);
        if (i < n - 1) nap(s, frame);
    }
    for (int i = 1; i < n; i++) {                 // insertion sort → median
        unsigned x = a[i]; int j = i - 1;
        while (j >= 0 && a[j] > x) { a[j + 1] = a[j]; j--; }
        a[j + 1] = x;
    }
    unsigned m = a[n / 2];
    if (m > s->peak_seen) s->peak_seen = m;
    if (m < s->floor) s->floor = m;
    if (m > s->best_fv) { s->best_fv = m; s->best_pos = s->pos; }
    s->p->out_steps++;
    if (s->p->trace) s->p->trace(s->p->trace_ctx, s->pos, m);
    return m;
}

// Land on best_pos approaching from the NEAR side and finishing on a FAR move, so the reversal
// backlash is taken up by that last move and the landing is repeatable (the vendor's fixed-side
// approach). `margin` must exceed the gear slack.
static void land_on(S *s, long target, long margin) {
    long pre = target - margin;   // pos is a free relative integrator (may be < 0); do NOT clamp
    if (s->pos > pre) drive(s, AF3_NEAR, (int)(s->pos - pre));
    else if (s->pos < pre) drive(s, AF3_FAR, (int)(pre - s->pos));
    if (target > s->pos) drive(s, AF3_FAR, (int)(target - s->pos));
}

unsigned af3_run(Af3IO *io, Af3Params *p) {
    if (p->focus_steps <= 0) p->focus_steps = 1280;
    if (p->frame_ms <= 0) p->frame_ms = 40;
    if (p->settle_frames <= 0) p->settle_frames = 4;
    if (p->fv_samples <= 0) p->fv_samples = 3;
    if (p->budget_ms <= 0) p->budget_ms = 20000;

    S s = {.io = io, .p = p};
    s.T = p->focus_steps;
    s.pos = (p->in_pos >= 0 && p->in_pos <= s.T) ? p->in_pos : s.T / 2;
    s.floor = (unsigned)-1;
    s.best_fv = 0;
    s.best_pos = s.pos;
    s.deadline = now(&s) + p->budget_ms;
    p->out_steps = 0;
    p->out_reversals = 0;

    // Step sizes, scaled to the travel so the same code fits any microstep count. The coarse
    // step is ADAPTIVE: big while the reading is down on the flat floor (cross it fast), small
    // once the reading has climbed onto the peak's flank (so the crest is not jumped over). The
    // landing creeps in FINE microsteps.
    const int PROBE = (int)(s.T / 64) > 4 ? (int)(s.T / 64) : 4;    // find-direction probe
    // Coarse step, UNIFORM. It must be no wider than a focus peak or it strides clean over one (a
    // big "cross the floor fast" step jumps a narrow crest and never samples it); the peaks on this
    // lens are narrow, so a 64th of the travel (~20 microsteps) is the safe ceiling. A far cold
    // start therefore pays a longer sweep, but a re-AF near focus — the common case — is short.
    const int CSTEP = (int)(s.T / 64) > 6 ? (int)(s.T / 64) : 6;
    const int FINE  = 1;  // landing creep step: the crest is razor-thin (measured: ~1 microstep is
                          // tens of percent of FV), so refine at the finest step the lens allows
    const long BL   = p->backlash_steps > 0 ? p->backlash_steps : (long)CSTEP;
    // The creep must reverse across the backlash and the deliberate overshoot and back over the
    // crest the coarse phase left behind, so bound it by that distance in FINE steps, with margin.
    const long CREEP_MAX = (3 * BL + 3 * (long)CSTEP + (long)s.T / 16) / (FINE > 0 ? FINE : 1) + 24;

    // ---- find direction: sample one PROBE to each side of the start, and go where FV is higher ----
    // Two things make this correct: (1) both readings are taken AFTER a move of the same size, so
    // each carries the same settle transient — comparing a post-move reading against a long-idle
    // one (which has none) is what lets the transient pick the wrong way, the field failure. (2) The
    // NEAR probe drives backlash_steps EXTRA, because the first move after the FAR probe reverses
    // and the gear slack would otherwise swallow the whole probe and the lens would not actually
    // move NEAR — so the two readings would be at the same place and the direction would be noise.
    unsigned f_start = measure(&s);
    // Already in focus? On a warm re-AF, read once WITHOUT moving and hold if the image is still
    // sharp — the find-direction probes below would step off a razor-thin crest that backlash then
    // forbids returning to, so re-focusing an already sharp frame must not move the lens at all.
    if (p->warm && p->hold_fv > 0 && f_start >= p->hold_fv) {
        p->out_peak_fv = f_start;
        p->out_peak_seen = s.peak_seen;
        p->out_found_crest = 1;
        p->out_pos = s.pos;
        p->out_reversals = 0;
        return f_start;
    }
    drive(&s, AF3_FAR, PROBE);                          // first move: no backlash to pay
    unsigned fFar = measure(&s);                        // at start + PROBE
    drive(&s, AF3_NEAR, (int)BL + 2 * PROBE);           // take up slack, then reach start - PROBE
    unsigned fNear = measure(&s);
    int dir = fFar > fNear ? AF3_FAR : AF3_NEAR;

    // Already on/near the crest? If NEITHER side beats the start, a full sweep would only wander
    // off a focus we already have (re-AF when the lens is basically sharp — and the two probes are
    // then symmetric, so find-direction is a coin flip that half the time sweeps the wrong way to a
    // stop). The settle transient inflates the probes, so this fires only when the start is
    // genuinely the highest — a conservative, correct on-peak guard. Drive back to the start and
    // hand it to the landing creep, which brackets the crest locally without a long sweep.
    // Only for a warm re-AF (p->warm): a cold pass has just homed to the near stop, where the floor
    // reads a hair above the first steps away (noise + the stop clamp) and would false-trigger this,
    // landing on the stop. The margin is max(a 16th of the start, an absolute floor): the fraction
    // handles a tall crest, the absolute one keeps floor jitter from ever looking like a local max.
    unsigned onpk = f_start / 16 > 10u ? f_start / 16 : 10u;
    if (p->warm && fFar + onpk <= f_start && fNear + onpk <= f_start) {
        drive(&s, AF3_FAR, (int)BL + PROBE);            // undo the NEAR probe: back to ~start
        s.best_fv = measure(&s);
        s.best_pos = s.pos;
        s.crest = 1;
        dir = AF3_FAR;                                  // landing overshoots FAR then creeps back
    }

    // ---- COARSE: step in `dir` until FV has clearly ROSE and then FELL — the crest is behind us ----
    // Per-leg rise-then-fall, not "best stopped improving": a start that sits a little above the
    // true floor (a transient, or a slightly-defocused start) would otherwise read as a peak the
    // moment the reading drops onto the floor, stopping the search before the real flank begins.
    // Requiring a real RISE above this leg's baseline first also makes the frozen first step after a
    // reversal (backlash: the lens sits still, FV flat) a non-event — no prime move needed. The
    // global best (position of the highest FV over the whole search) is what the landing returns to.
    if (!s.crest) {
        unsigned leg_start = measure(&s);
        if (leg_start > s.best_fv) { s.best_fv = leg_start; s.best_pos = s.pos; }
        unsigned leg_max = leg_start;
        int rose_leg = 0, other_stop = 0;
        while (!cancelled(&s)) {
            int moved = drive(&s, dir, CSTEP);
            unsigned v = measure(&s);
            if (v > leg_max) leg_max = v;
            unsigned rmargin = leg_start / 8 > 6u ? leg_start / 8 : 6u;
            if (leg_max > leg_start + rmargin && peak_is_real(leg_max, s.floor)) rose_leg = 1;
            unsigned drop = leg_max / 25 > 2u ? leg_max / 25 : 2u;
            if (rose_leg && v + drop < leg_max) { s.crest = 1; break; }   // climbed then fell
            if (moved < CSTEP) {                          // hit an end stop
                if (other_stop) break;                    // swept both ends: use the global best
                other_stop = 1;
                dir = -dir;
                drive(&s, dir, (int)BL);                  // absorb the reversal slack before resuming
                leg_start = measure(&s);                  // fresh baseline for the new leg
                leg_max = leg_start;
                rose_leg = 0;
            }
        }
    }

    // ---- LAND: reverse toward the crest the coarse phase just overshot, and creep onto it ----
    // Coarse passed the peak going `dir`, so the crest is back toward -dir. Creep that way in FINE
    // microsteps and let FV place the landing (a return by count would be off by a whole backlash
    // width, which the dead reckoning cannot see). The reversal's first steps are swallowed by the
    // gear slack — the lens sits still, FV flat and LOW — so the fall test is ARMED only once FV has
    // climbed back to within ~1/8 of the crest height coarse actually measured (`target`). That one
    // reference beats every scheme tried before: it clears the flat-slack plateau (FV stuck low, far
    // from target) with no fixed prime move (a prime overshoots a peak nearer than the backlash), and
    // it scales to a shallow crest (target is then small). Once armed, stop the instant FV falls
    // clearly from its run-max — max(4%, 2 counts) — landing a single FINE step off the crest.
    if (s.crest && !cancelled(&s)) {
        // Deliberately overshoot the crest by the backlash estimate FIRST, so the reversal below
        // takes up its slack while the lens is still PAST the peak — then the creep's climb back
        // over the crest is a genuine, detectable rise from a low start (not a near-peak start the
        // arming test cannot tell from noise). Then creep back with an adaptive step: medium while
        // FV is still low (cover the overshoot + slack fast), fine once near the crest (land on it).
        drive(&s, dir, (int)BL);
        int cdir = -dir;
        unsigned target = s.best_fv;                               // crest height coarse found
        unsigned runmax = 0;
        for (long it = 0; it < CREEP_MAX && !cancelled(&s); it++) {
            int cstep = (unsigned)(runmax * 2) < target ? CSTEP : FINE;
            if (drive(&s, cdir, cstep) == 0) break;                // hit an end stop = the crest
            unsigned v = measure(&s);
            if (v > runmax) runmax = v;
            unsigned drop = runmax / 25 > 2u ? runmax / 25 : 2u;   // max(4%, 2 counts)
            int near_crest = (long)runmax * 8 >= (long)target * 7; // climbed back to ~0.875*crest
            if (near_crest && v + drop < runmax) break;            // fell off the crest → stop on it
        }
    } else if (!cancelled(&s)) {
        land_on(&s, s.best_pos, BL + 8 * FINE);   // no gradient: park on the best seen (bookkeeping)
    }

    unsigned final = measure(&s);
    p->out_peak_fv = final;
    p->out_peak_seen = s.peak_seen;
    p->out_found_crest = s.crest;
    p->out_pos = s.pos;
    return final;
}
