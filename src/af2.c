// af2 — autofocus for a lens board that tracks focus through a zoom by itself. See af2.h for
// the rationale. The board leaves focus close to the crest, so af2 searches a short window
// around wherever the lens is, direction-first: a smooth continuous sweep FAR that stops just
// past the crest and climbs back onto it (sweep_to_crest), then one NEAR if FV only fell, and
// further on the same way if a sweep ends still climbing. It never hill-climbs or hunts around
// the peak. Only when neither direction shows a crest does it widen once. No end stop, no
// position carried between passes, no zoom->focus curve.

#include <majestic/af2.h>

typedef struct {
    AfIO *io;
    AfParams *p;
    long deadline;
    unsigned peak_seen;
    int last_dir;   // last non-STOP drive direction, for backlash accounting
    int crest;      // a sweep recognised a real crest at some point in this pass
    int inside;     // the last sweep saw FV rise AND fall (or clamp at a stop) within its reach
    int fell_first; // the last sweep saw FV only fall from its first sample: the crest is behind
                    // it (or it started on the crest), and its return brought the lens back
    int rising_end; // the last sweep ran out of reach with FV still climbing: the crest is ahead
    unsigned start_v; // FV the last sweep started on
    unsigned base_v;  // nonzero: the sweep continues one that climbed from here; a rise counts from it
    unsigned crest_v; // the last sweep's top, capped at its higher neighbour (a lone spike filtered)
    long pos;       // dead-reckoned focus position, ms of FAR travel from where the pass began
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
    // Never past the pass deadline (af2_run promises budget_ms): drive what fits, settle
    // included, and dead-reckon only what was driven.
    long run = mag + extra;
    long room = s->deadline - now(s) - s->p->settle_ms;
    if (room <= 0) return;
    if (run > room) run = room;
    motor(s, dir);
    // Nap the drive in short slices so a cancel (a fresh zoom) stops the motor within a slice
    // rather than after a multi-second move — it must release the UART well inside the zoom
    // caller's lock-retry window.
    long driven = 0;
    while (driven < run && !cancelled(s)) {
        long step = run - driven < 200 ? run - driven : 200;
        nap(s, step);
        driven += step;
    }
    motor(s, AF2_STOP);
    nap(s, s->p->settle_ms);
    s->last_dir = dir;
    long moved = driven - extra;                      // the slack is taken up first
    if (moved > 0) s->pos += dir * moved;             // dead-reckon the new position
}

// Is there a real peak here, or is this the statistic's own noise?
//
// It cannot be a fixed number of counts. The focus statistic has no absolute
// scale -- engine.c's own header puts the ceiling swing at ~20x between
// daylight and dusk on one view -- and the WIDE end compresses the whole range
// again, because the depth of field at 2.8 mm keeps a defocused image from ever
// going flat. `floor + 40` therefore reads a bright tele scene and a wide or dim
// one as different kinds of thing. Measured on an 85H50AI at the x1.0 stop: a
// crest of 31 over a floor of 9 was dismissed as noise, and because EVERYTHING
// below hangs off this one flag -- the crest break, the plateau break, and the
// return onto the crest -- the sweep then ran its whole budget away from the
// peak it had been standing on and stopped there, 8 s off, reporting `done`.
//
// So scale the bar with the floor the sweep actually found, and keep a small
// absolute guard underneath it so the integer quantisation at the very bottom
// (values of 2..9) cannot pass on its own. An eighth is wide enough for the
// shallow wide-angle crest measured on this lens (13018 over 10914 inside the
// sweep window) and tight enough to reject the few-percent jitter of a
// high-gain night frame.
#define AF2_RISE_MIN 8
static int peak_is_real(unsigned top, unsigned floorv) {
    if (top <= floorv) return 0;
    unsigned rise = top - floorv;
    unsigned bar = floorv >> 3;
    if (bar < AF2_RISE_MIN) bar = AF2_RISE_MIN;
    return rise >= bar;
}

// The FV-guided return stops on a READING, and on the 85H50AI a reading is late: the board takes
// 60-180 ms to act on a command and the statistic shows the lens ~80 ms back, so the lens goes
// on 100-250 ms past wherever a reading said stop -- varying from one stop to the next. At the
// wide stop the crest is narrower than that (100 ms of drive off it costs 4 %, 270 ms a third),
// so the return landed on it or well down the far side, by luck (traced: 7 of 10 passes at the
// wide stop on the far side, at 68-89 %). So check the landing with the lens stopped, which no
// lag can fool, and if it is short of the crest, creep onto it in short pulses, each measured
// stopped. Which way: if FV settled BELOW the last reading the return stopped on, the lens went
// over the crest (it is behind); otherwise the crest is still ahead. A pulse moves the lens
// irregularly (0-200 ms of drive for 100 ms asked, measured), so the creep follows FV, not a count.
#define AF2_PULSE_MS 80
static void pulse(S *s, int c, long settle) {
    motor(s, c);
    nap(s, AF2_PULSE_MS);
    motor(s, AF2_STOP);
    nap(s, settle);
    s->last_dir = c;
    s->pos += c * AF2_PULSE_MS;   // dead reckoning only; the creep follows FV
}
// Returns FV where it leaves the lens, measured stopped.
static unsigned creep_onto_crest(S *s, unsigned top, unsigned r_stop, int ret_dir, int behind) {
    long settle = s->p->settle_ms > 250 ? s->p->settle_ms : 250;   // past the command + FV lag
    if (s->deadline - now(s) < settle + 400) return fv_med(s);
    nap(s, settle - s->p->settle_ms);
    unsigned v = fv_med(s);
    if ((long)v * 100 >= (long)top * 90) return v;                 // landed on it
    unsigned good = (unsigned)((long)top * 95 / 100);
    // Carried over the crest (FV settled below the reading the return stopped on): it is behind.
    int c = (behind || (long)v * 100 < (long)r_stop * 97) ? -ret_dir : ret_dir;
    long budget_end = now(s) + 6000;                                // a correction, not a search
    // Take up the slack first if this is a reversal: pulse until FV starts to change.
    int slack = c != s->last_dir ? (int)(2 * s->p->backlash_ms / AF2_PULSE_MS) + 1 : 0;
    unsigned ref = v, best = v;
    int climbed = 0, flat = 0, turned = 0;
    while (!cancelled(s) && now(s) < budget_end &&
           s->deadline - now(s) > AF2_PULSE_MS + settle + 400) {
        pulse(s, c, settle);
        v = fv_med(s);
        int up = (long)v * 100 > (long)ref * 102, down = (long)v * 100 < (long)ref * 97;
        if (slack > 0 && !up && !down) { slack--; continue; }       // still in the gear slack
        slack = 0;
        if (v >= good) break;                                       // on the crest
        if (up) { ref = v; climbed = 1; flat = 0; if (v > best) best = v; continue; }
        if (down) {
            if (climbed || turned) break;                           // one pulse past: as close as it gets
            c = -c; turned = 1; ref = v;                            // wrong way: turn once
            slack = (int)(2 * s->p->backlash_ms / AF2_PULSE_MS) + 1;
            continue;
        }
        if (++flat >= 3) break;                                     // no gradient to follow
    }
    return v;
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
    for (long left = blind_ms + extra; left > 0 && !cancelled(s) && now(s) < s->deadline; left -= 200) {
        long slice = left < 200 ? left : 200;
        long rest = s->deadline - now(s);
        nap(s, slice < rest ? slice : rest);
    }
    long st0 = now(s);
    unsigned floor = s->io->fv(s->io->ctx);          // FV where sampling begins (off-peak floor)
    s->start_v = floor;
    // A continuation goes on climbing a rise the previous sweep measured: its rise and its floor
    // count from where that sweep began, not from here, near the top of a broad crest.
    if (s->base_v && s->base_v < floor) floor = s->base_v;
    const unsigned start_v = floor;                  // and what a real rise is measured from
    unsigned top = s->start_v;
    unsigned prev = s->start_v, top_prev = 0, top_next = 0;
    long top_on = now(s) - t0;
    long on = top_on;
    int nsamp = 0, top_at = 0;   // samples taken on the move; which one the top is (0 = floor)
    int rose = 0, plateau = 0;
    unsigned last = floor;
    s->inside = 0;
    s->fell_first = 0;
    s->rising_end = 0;
    int broke = 0;
    while (now(s) - st0 < budget_ms && now(s) < s->deadline && !cancelled(s)) {
        nap(s, frame);
        unsigned v = s->io->fv(s->io->ctx);
        on = now(s) - t0;
        nsamp++;
        last = v;
        unsigned pv = prev;
        prev = v;
        if (top_at == nsamp - 1) top_next = v;      // the sample after the top
        if (v > s->peak_seen) s->peak_seen = v;
        if (v < floor) floor = v;
        s->p->out_steps++;
        if (s->p->trace) {
            long moved = on - extra; if (moved < 0) moved = 0;
            s->p->trace(s->p->trace_ctx, start + (long)dir * moved, v);
        }
        if (peak_is_real(top, floor)) {              // a real peak (not integer-floor noise) exists
            rose = 1;
            s->crest = 1;
        }
        if (v > top) { top = v; top_on = on; top_at = nsamp; plateau = 0; top_prev = pv; top_next = 0; }
        else if (rose && (long)v * 100 < (long)top * 85) {
            // Crested and clearly fell: the crest is behind us -- inside the window, unless FV
            // has only ever fallen from the first sample, which says the crest lies back past
            // the edge the sweep started from. Stop; the return below lands
            // back on it. (A crest right at the window edge the sweep starts from is this case
            // too — FV only ever falls, top stays the start value at top_on~0, we return to it.)
            // Counted in samples, not ms: a real clock stamps the first sample late (msleep
            // overshoots), which in ms would pass a crest on the starting edge as inside.
            // A crest the sweep climbed to: the top came after the first sample on the move AND
            // clears the sweep's starting value by a real margin. The gear slack at the start of
            // a sweep holds the lens still for up to half a second, and frame noise on that flat
            // start (about 1 %) otherwise reads as a "rise" to a later sample.
            unsigned bar = start_v >> 4;
            if (bar < AF2_RISE_MIN) bar = AF2_RISE_MIN;
            int rose_real = top_at > 1 && top >= start_v + bar;
            s->inside = rose_real;
            s->fell_first = !rose_real;
            broke = 1;
            break;
        } else if (rose && (long)v * 100 >= (long)top * 96) {
            // FV flat at the top, not climbing: EITHER the sweep has run into an end stop (the
            // lens clamps there and FV goes flat forever)
            // OR this is just a flat SHOULDER on the way to a higher crest further along (a wide
            // scene has these). Only the clamp holds flat for a long time, so require a long run
            // before stopping — a brief shoulder is passed, and the sweep goes on to the real
            // crest. When it does stop, we are ON the crest: no overshoot to undo.
            if (++plateau >= 14) { top_on = on; s->inside = 1; broke = 1; break; }
        } else plateau = 0;
    }
    // Out of reach while still climbing: FV ends at its top (within frame noise -- the top
    // itself can be a sample or two back) and the top clears the start by a real margin. The
    // crest lies further on, not behind.
    {
        unsigned bar = start_v >> 4;
        if (bar < AF2_RISE_MIN) bar = AF2_RISE_MIN;
        int rose_real = top_at > 1 && top >= start_v + bar;
        s->rising_end = !broke && !cancelled(s) && rose_real && (long)last * 100 >= (long)top * 96;
        // Out of reach past a broad crest: FV climbed for real, then eased off without the 15 %
        // fall that breaks a sweep (the wide end's crest is that broad). The crest is behind,
        // inside what this sweep covered; the return below lands on it.
        if (!broke && !cancelled(s) && rose_real && s->crest && (long)last * 100 < (long)top * 96)
            s->inside = 1;
        // The crest's FV with a lone bright frame filtered out: the top, capped at the higher of
        // its neighbours (a real crest is broader than one frame).
        unsigned nb = top_prev > top_next ? top_prev : top_next;
        s->crest_v = nb && nb < top ? nb : top;
    }
    motor(s, AF2_STOP);
    nap(s, s->p->settle_ms);
    s->last_dir = dir;
    long moved = on - extra; if (moved < 0) moved = 0;
    long crest_pos = start + (long)dir * (top_on - extra);   // where FV actually peaked
    s->pos = start + (long)dir * moved;              // dead-reckon where we stopped
    // Return onto the crest, FV-GUIDED so the reversal's backlash — which is not constant, and
    // at half a second and more on an 85H50AI is wider than the crest itself — cannot displace
    // the landing. A counted hop back either falls short (the slack swallows it) or overshoots; and stopping AFTER
    // FV falls always overshoots by the detection lag. So reverse and climb back UP the flank,
    // and stop the instant FV reaches the crest value again — on the rising side, one clean turn,
    // no counted distance and no overshoot. The reversal slack (FV flat at the stop value) sits
    // well below the crest, so it can't trip the stop early.
    long back = on - top_on;                         // motion travelled past the crest
    if (back > 0 && peak_is_real(top, floor)) {
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
        // two of overshoot — tens of milliseconds of drive —
        // instead of the whole far flank.
        // rtop is the highest reading since the lowest one: right after the stop, readings go
        // on falling while they catch up with the lens (the statistic lags it), and a "crest"
        // counted before that bottom would turn the return round before it began.
        unsigned rtop = 0, rmin = ~0u, rlast = 0;
        int back_rose = 0, returned = 0, behind = 0;
        while (now(s) < rlimit && now(s) < s->deadline && !cancelled(s)) {
            nap(s, frame / 2 > 0 ? frame / 2 : 40);
            unsigned v = s->io->fv(s->io->ctx);
            if (v > s->peak_seen) s->peak_seen = v;
            s->p->out_steps++;
            if (s->p->trace) s->p->trace(s->p->trace_ctx, crest_pos, v);
            rlast = v;
            if (v < rmin) { rmin = v; rtop = v; }
            else if (v > rtop) rtop = v;
            if (peak_is_real(rtop, rmin)) back_rose = 1;
            if (v >= target) { returned = 1; break; }   // climbed back onto the crest — stop here
            if (back_rose && (long)v * 100 < (long)rtop * 92) {
                returned = 1;                        // crested on the way back; it is behind us
                behind = 1;
                break;
            }
        }
        // The pass's deadline cut the return short: the lens is somewhere on the far flank, not
        // on the crest, so this sweep has not found it.
        if (!returned && now(s) >= s->deadline) s->inside = s->fell_first = 0;
        motor(s, AF2_STOP);
        nap(s, s->p->settle_ms);
        s->last_dir = -dir;
        s->pos = crest_pos;
        if (returned && !cancelled(s)) return creep_onto_crest(s, top, rlast, -dir, behind);
    }
    // With no usable gradient there is no flank to climb, and the FV-guided return above would
    // stop on whichever noisy sample happened to reach 95 % of a top that means nothing. So the
    // sweep simply ends; af2_run() widens, or puts the lens back where the pass began.
    return fv_med(s);
}

unsigned af2_run(AfIO *io, AfParams *p) {
    if (p->backlash_ms <= 0) p->backlash_ms = AF2_BACKLASH_MS;
    if (p->settle_ms <= 0) p->settle_ms = 160;
    if (p->budget_ms <= 0) p->budget_ms = 30000;
    if (p->fv_samples <= 0) p->fv_samples = 5;
    if (p->fv_frame_ms <= 0) p->fv_frame_ms = 40;
    if (p->window_ms <= 0) p->window_ms = AF2_WINDOW_MS;
    if (p->wide_ms <= 0) p->wide_ms = AF2_WIDE_MS;

    S s = {.io = io, .p = p, .peak_seen = 0, .last_dir = 0, .crest = 0, .pos = 0};
    s.deadline = now(&s) + p->budget_ms;
    p->out_steps = 0;
    p->out_window = 0;

    unsigned final = 0;
    int found = 0;   // a crest seen to rise AND fall, or the start seen to fall away on both sides

    // Direction first. The crest is usually within a fraction of a second of where the board left
    // focus, and more often on the FAR side of it (measured on the 85H50AI). So sweep FAR from
    // right here, with no back-off: a crest on that side is found in one sweep and one return. If
    // FV only falls from the first sample, the crest is behind (or here); the sweep's return has
    // already climbed back to the start, so sweep NEAR from there. FV falling away on BOTH sides
    // means the start was the crest, and the second return has landed on it. Each sweep reaches
    // the window, one backlash (the gear's slack state at the start of a pass is not known) and
    // the edge margin. Typically 1.5-2.5 s, against the ~5 s of a back-off and a full sweep.
    p->out_window = 1;
    long reach = p->window_ms + p->backlash_ms + AF2_EDGE_MS;
    long more = p->wide_ms + p->backlash_ms + AF2_EDGE_MS;   // the further reach, past the window
    int fell_far = 0;
    const int dirs[2] = {AF2_FAR, AF2_NEAR};
    for (int k = 0; k < 2 && !found && !cancelled(&s) && now(&s) < s.deadline; k++) {
        // The NEAR sweep starts wherever the FAR one left the lens: back past the start first.
        long back = k == 1 && s.pos > 0 ? s.pos : 0;
        s.crest = 0;
        final = sweep_to_crest(&s, dirs[k], 0, back + reach);
        if (s.crest && s.inside) {
            found = 1;
        } else if (s.rising_end && !cancelled(&s) && now(&s) < s.deadline) {
            // Still climbing where the window ended: the crest is ahead, a little past it (a
            // broad crest -- the wide end's -- or the board left focus further off). Go on the
            // same way rather than turn round; no reversal, so no backlash to pay.
            s.crest = 0;
            s.base_v = s.start_v;   // the rise so far counts: it climbed from there
            final = sweep_to_crest(&s, dirs[k], 0, more);
            s.base_v = 0;
            // FV only falls ahead: the climb had already reached the crest, and the return has
            // landed back on it.
            found = s.crest && (s.inside || s.fell_first);
        } else if (k == 0) {
            fell_far = s.fell_first;
        } else {
            found = s.crest && fell_far && s.fell_first;   // falls away on both sides: the start
        }
    }

    // Wide: neither direction showed a crest within the window -- FV still rising where a sweep
    // ended, or no gradient at all. Back off FAR to the wide window's edge, measured from where
    // the pass began, and sweep NEAR across all of it. Only if the whole sweep fits the budget:
    // drive_focus() sleeps the full distance it is handed, and a sweep cut short measures nothing.
    if (!found && !cancelled(&s) && p->wide_ms > p->window_ms) {
        // The dead reckoning can be off by one backlash (the slack's state when the pass began is
        // unknown), so overshoot the edge by that much and sweep that much further.
        long wreach = 2 * p->wide_ms + 2 * p->backlash_ms + AF2_EDGE_MS;
        long need = (p->wide_ms > s.pos ? p->wide_ms - s.pos : s.pos - p->wide_ms) + 2 * p->backlash_ms +
                    wreach + 3 * p->settle_ms + 1000;
        if (s.deadline - now(&s) >= need) {
            p->out_window = 2;
            drive_focus(&s, p->wide_ms + p->backlash_ms - s.pos);
            if (now(&s) < s.deadline) {
                s.crest = 0;
                final = sweep_to_crest(&s, AF2_NEAR, 0, wreach);
                found = s.crest && s.inside;
            }
        }
    }

    // No crest anywhere: the pass measured nothing, and the best guess at focus is the one the
    // board's own tracking made. Put the lens back there rather than leave it on whichever noisy
    // sample was highest. Bounded by the deadline like every other move.
    // (A rise that never fell inside a window is not a crest: its "top" is only the sweep's edge.)
    // The first sweep goes FAR from wherever the board left the gear's slack, which may take up
    // to one backlash of that sweep's drive without moving the lens, counted as travel. So the
    // start lies between s.pos and one backlash FAR of it: aim at the middle, half a backlash of
    // doubt either way rather than a whole one.
    if (!found && s.pos != 0 && !cancelled(&s) && now(&s) < s.deadline) {
        drive_focus(&s, -s.pos + p->backlash_ms / 2);  // bounded by the deadline itself
        final = fv_med(&s);
    }

    p->out_peak_fv = final;
    p->out_peak_seen = s.peak_seen;
    p->out_found_crest = found;
    p->out_crest_fv = found ? s.crest_v : 0;          // the sweep that found it was the last one
    p->out_landed_pos = s.pos;                        // relative to the start, for the log
    motor(&s, AF2_STOP);
    return final;
}
