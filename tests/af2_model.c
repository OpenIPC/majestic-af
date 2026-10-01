/* af2 against a synthetic lens and scene: the 85H50AI as measured on its healthy lenses
 * (OpenIPC/motors xm-uart/PROTOCOL.md, "Zoom tracking inside the board").
 *
 *  - The crest is narrow: 90 % of peak sharpness spans 0.1-0.3 s of focus drive, and the
 *    statistic sits on a flat floor a second either side. Modelled as a Gaussian of width
 *    200-450 ms over a floor.
 *  - A reversal takes up ~0.5 s of gear slack before focus moves (measured >= 0.50 s).
 *  - After a zoom the board has left focus within about 0.6 s of the crest, and the slack is in
 *    whatever state the board's own last move left it.
 *  - Optionally (lagged()), the delays measured on the rig at the wide stop: a command takes
 *    effect 60-180 ms after it is sent, varying from one to the next (the board's own command
 *    latency), and the statistic shows the lens as it was ~80 ms earlier (ISP latency). Together
 *    they carry the lens 150-250 ms past wherever a reading tells af2 to stop. And a reading
 *    averages its 40 ms frame: on the move a crest reads lower than it does stopped.
 *
 * Self-contained, virtual-clock, deterministic. The engine is told none of the model's numbers;
 * the backlash and the crest position both vary around what af2 assumes. Positions are ms of
 * FAR drive from where the pass starts. */
#include <greatest.h>

#include <majestic/af2.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    long vclock;
    double pos, slack;
    int cmd, last_cmd;
    double peak, width, floor, peak_h;   /* engine is NOT told these */
    double backlash;
    double sh_lo, sh_hi;   /* a flat FV shelf on the FAR approach: [peak+lo, peak+hi] */
    double rev_pen;        /* the same crest reads LOWER once the motor has reversed --
                            * different backlash, different sampling phase. Traced at 96.7 %
                            * on an 85H50AI, under the return's 95 % target, so the fall
                            * detector is the only thing left to stop the return. */
    long jitter;           /* every sleep overshoots by this much, as msleep does on the camera */
    unsigned rng;
    int reversals, last_nz;   /* motor direction reversals this pass -- the JOURNEY (no hunting) */
    long cmd_lag, cmd_jit;    /* a command takes effect cmd_lag + [0, cmd_jit) ms after it is sent */
    long fv_lag;              /* the statistic reads the position fv_lag ms ago */
    int pend[8];              /* commands on their way to the board, in order */
    long pend_at[8];          /* when each takes effect (never before the one ahead of it) */
    int npend;
    double hist[64];          /* position every 5 ms, for fv_lag */
    int hist_n;
} Lens;

static double lr(Lens *l) { l->rng = l->rng * 1103515245u + 12345u; return ((l->rng >> 16) & 0x7fff) / 32767.0; }
static double sharpf(Lens *l) {
    double d = l->pos - l->peak;
    double ad = d < 0 ? -d : d;
    if (l->sh_hi > 0 && d > l->sh_lo && d < l->sh_hi) ad = l->sh_hi;
    double x = ad / l->width;
    return exp(-x * x);
}

static long io_now(void *c) { return ((Lens *)c)->vclock; }
static void apply_cmd(Lens *l, int d) {
    if (d != 0) {
        if (d != l->last_cmd && l->last_cmd != 0) l->slack = l->backlash;
        l->last_cmd = d;
        if (l->last_nz != 0 && d != l->last_nz) l->reversals++;
        l->last_nz = d;
    }
    l->cmd = d;
}
static void io_drive(void *c, int d) {
    Lens *l = c;
    if (!l->cmd_lag && !l->cmd_jit) { apply_cmd(l, d); return; }
    long at = l->vclock + l->cmd_lag + (l->cmd_jit ? (long)(lr(l) * l->cmd_jit) : 0);
    if (l->npend && at < l->pend_at[l->npend - 1]) at = l->pend_at[l->npend - 1];   /* in order */
    if (l->npend == 8) {   /* never in practice: make room by applying the oldest now */
        apply_cmd(l, l->pend[0]);
        memmove(l->pend, l->pend + 1, 7 * sizeof l->pend[0]);
        memmove(l->pend_at, l->pend_at + 1, 7 * sizeof l->pend_at[0]);
        l->npend--;
    }
    l->pend[l->npend] = d;
    l->pend_at[l->npend++] = at;
}
static void advance(Lens *l, long ms) {
    if (l->cmd != 0 && ms > 0) {
        double move = ms;
        if (l->slack > 0) { double k = move < l->slack ? move : l->slack; l->slack -= k; move -= k; }
        l->pos += l->cmd * move;
    }
    l->vclock += ms;
}
static void io_sleep(void *c, long ms) {
    Lens *l = c;
    if (ms > 0) ms += l->jitter;
    if (!l->cmd_lag && !l->cmd_jit && !l->fv_lag) { advance(l, ms); return; }
    while (ms > 0) {   /* 5 ms steps: commands land on time, and the position history fills */
        long step = ms < 5 ? ms : 5;
        while (l->npend && l->vclock >= l->pend_at[0]) {
            apply_cmd(l, l->pend[0]);
            memmove(l->pend, l->pend + 1, (size_t)(l->npend - 1) * sizeof l->pend[0]);
            memmove(l->pend_at, l->pend_at + 1, (size_t)(l->npend - 1) * sizeof l->pend_at[0]);
            l->npend--;
        }
        advance(l, step);
        ms -= step;
        memmove(l->hist + 1, l->hist, sizeof l->hist - sizeof l->hist[0]);
        l->hist[0] = l->pos;
        if (l->hist_n < 64) l->hist_n++;
    }
}
static double sharp_at(Lens *l, double pos) {
    double keep = l->pos; l->pos = pos;
    double s = sharpf(l);
    l->pos = keep;
    return s;
}
static unsigned io_fv(void *c) {
    Lens *l = c;
    int back = (int)(l->fv_lag / 5);
    double seen = sharpf(l);
    if (l->fv_lag && back + 8 <= l->hist_n) {
        /* one 40 ms frame's exposure: a lens moving through it blurs the reading (the moving
         * crest reads lower than the same crest stopped, ~91-97 % on the rig) */
        seen = 0;
        for (int i = 0; i < 8; i++) seen += sharp_at(l, l->hist[back + i]);
        seen /= 8;
    } else if (l->fv_lag && back < l->hist_n) {
        seen = sharp_at(l, l->hist[back]);
    }
    double v = l->floor + (l->peak_h - l->floor) * seen;
    if (l->rev_pen > 0 && l->reversals >= 1) v *= (1.0 - l->rev_pen);
    v *= (1.0 + 0.01 * (lr(l) - 0.5) * 2.0);
    if (v < 1) v = 1;
    return (unsigned)(v + 0.5);
}

/* A lens the board has just left at `offset` ms from the crest (crest = start + offset), its
 * slack last taken up in direction `board_dir`. */
static Lens lens(double offset, double width, double backlash, int board_dir, unsigned seed) {
    Lens l; memset(&l, 0, sizeof l);
    l.peak = offset; l.width = width; l.backlash = backlash;
    l.floor = 20; l.peak_h = 2500;
    l.last_cmd = board_dir;
    l.rng = seed;
    return l;
}
/* The rig's delays at the wide stop (see the header). */
static Lens lagged(Lens l) { l.cmd_lag = 60; l.cmd_jit = 120; l.fv_lag = 80; return l; }
static AfParams defaults(void) {
    AfParams p; memset(&p, 0, sizeof p);
    p.settle_ms = 160; p.budget_ms = 30000;
    p.fv_samples = 5; p.fv_frame_ms = 40;
    return p;   /* backlash and windows: af2's own defaults */
}
static double run(Lens *l, AfParams *p) {
    AfIO io = {io_drive, io_fv, io_now, io_sleep, l};
    af2_run(&io, p);
    return sharpf(l);
}

/* The common case, and the one that has to be fast: a zoom, the board's tracking left focus
 * near the crest, af2 finds it in the short window. Across the measured spread of tracking
 * error, crest width, backlash and the slack's starting state, land on the crest, in one
 * smooth journey (out, across, back: no hunting), within a few seconds. */
TEST lands_after_the_boards_tracking(void) {
    double offsets[] = {-700, -400, -150, 0, 150, 400, 700};
    double widths[] = {200, 300, 450};
    double backlash[] = {450, 550, 700};
    int dirs[] = {AF2_NEAR, AF2_FAR};
    for (unsigned o = 0; o < sizeof offsets / sizeof *offsets; o++)
    for (unsigned w = 0; w < 3; w++)
    for (unsigned b = 0; b < 3; b++)
    for (unsigned d = 0; d < 2; d++) {
        Lens l = lens(offsets[o], widths[w], backlash[b], dirs[d],
                      0x99u ^ (unsigned)(o * 131 + w * 17 + b * 5 + d));
        AfParams p = defaults();
        double f = run(&l, &p);
        if (!p.out_found_crest || p.out_window != 1 || f < 0.85 || l.reversals > 3 ||
            l.vclock > 8000) {
            static char msg[200];
            snprintf(msg, sizeof msg,
                     "offset %.0f width %.0f backlash %.0f board %+d: crest=%d window=%d "
                     "land=%.0f%% reversals=%d time=%ld ms",
                     offsets[o], widths[w], backlash[b], dirs[d], p.out_found_crest,
                     p.out_window, f * 100, l.reversals, l.vclock);
            FAILm(msg);
        }
    }
    PASS();
}

/* A subject much nearer (or farther) than the scene the board's curve was made for leaves the
 * crest outside the short window. af2 widens once and still lands. */
TEST widens_for_a_crest_outside_the_window(void) {
    double offsets[] = {-3500, -2200, 2200, 3500};
    for (unsigned o = 0; o < sizeof offsets / sizeof *offsets; o++)
    for (int d = -1; d <= 1; d += 2) {
        Lens l = lens(offsets[o], 300, 550, d, 0x5eedu ^ (unsigned)(o * 7 + d + 1));
        AfParams p = defaults();
        double f = run(&l, &p);
        /* Which window finds it does not matter: the NEAR sweep reaches back past the start and
         * across the near side, so a crest at -2.2 s is found without widening. */
        if (!p.out_found_crest || f < 0.85 || l.vclock > 25000) {
            static char msg[160];
            snprintf(msg, sizeof msg, "offset %.0f board %+d: crest=%d window=%d land=%.0f%% time=%ld",
                     offsets[o], d, p.out_found_crest, p.out_window, f * 100, l.vclock);
            FAILm(msg);
        }
    }
    PASS();
}

/* Nothing to focus on (a blank wall, a covered lens): no crest anywhere. The pass must say so
 * -- out_found_crest = 0, which the engine reports as a failure -- rather than a `done`, and
 * must put the lens back where the board's tracking left it, the best guess there is. (Within
 * the gear slack: the first move pays a backlash the lens may not have owed.) */
TEST no_contrast_is_reported_not_invented(void) {
    Lens l = lens(0, 300, 550, AF2_FAR, 0x77a1);
    l.peak_h = l.floor;   /* flat */
    AfParams p = defaults();
    run(&l, &p);
    GREATEST_ASSERTm("a flat scene reported a crest", !p.out_found_crest);
    GREATEST_ASSERTm("a flat pass overran its budget", l.vclock <= p.budget_ms + 1000);
    if (fabs(l.pos) > l.backlash + 50) {
        static char msg[80];
        snprintf(msg, sizeof msg, "a flat pass ended at %.0f, not back at the start", l.pos);
        FAILm(msg);
    }
    PASS();
}

/* af2.h promises the pass never blocks beyond budget_ms, including the counted return of a
 * sweep that found no gradient. A budget that expires in the middle of the wide sweep is the
 * way to reach that branch with time already spent. */
TEST a_short_budget_is_respected(void) {
    Lens l = lens(0, 300, 550, AF2_FAR, 0x1234);
    l.peak_h = l.floor;
    AfParams p = defaults();
    p.budget_ms = 9000;
    run(&l, &p);
    long over = l.vclock - p.budget_ms;
    if (over > 1000) {
        static char msg[120];
        snprintf(msg, sizeof msg, "pass ran %ld ms past its %ld ms budget", over, p.budget_ms);
        FAILm(msg);
    }
    PASS();
}

/* A crest only TENS of counts above the floor must still be found. The statistic has no
 * absolute scale, so a wide or dim scene can put the whole peak-to-floor range inside a few
 * dozen counts: `done fv=25 peak=27` was an 85H50AI at x1.0 before the crest bar scaled with
 * the floor. */
TEST lands_on_a_crest_barely_above_the_floor(void) {
    const double floors[] = {9, 9, 5, 3};
    const double peaks[] = {31, 48, 22, 14};
    for (unsigned i = 0; i < sizeof peaks / sizeof *peaks; i++) {
        Lens l = lens(250, 300, 550, AF2_NEAR, 0x9e37u ^ (unsigned)peaks[i]);
        l.floor = floors[i]; l.peak_h = peaks[i];
        AfParams p = defaults();
        double f = run(&l, &p);
        if (!p.out_found_crest || f < 0.75) {
            static char msg[160];
            snprintf(msg, sizeof msg, "peak %.0f over floor %.0f: crest=%d land=%.0f%%",
                     peaks[i], floors[i], p.out_found_crest, f * 100);
            FAILm(msg);
        }
    }
    PASS();
}

/* The return onto the crest reads the same crest a few percent LOWER than the forward sweep
 * did, so it never reaches its 95 % target and the fall detector must stop it -- or the lens
 * sails over the crest and down the far flank. */
TEST return_recognises_a_crest_that_reads_lower(void) {
    double offsets[] = {-300, 0, 300};
    for (unsigned o = 0; o < 3; o++) {
        Lens l = lens(offsets[o], 300, 550, AF2_FAR, 0x2f1bu ^ o);
        l.floor = 9; l.peak_h = 50; l.rev_pen = 0.12;
        AfParams p = defaults();
        double f = run(&l, &p);
        if (f < 0.70) {
            static char msg[120];
            snprintf(msg, sizeof msg, "offset %.0f: land=%.0f%% (sailed past the crest?)",
                     offsets[o], f * 100);
            FAILm(msg);
        }
    }
    PASS();
}

/* A wide scene lays a FLAT SHELF on the FV rise short of the true crest (a nearer plane's
 * contrast). The sweep must pass it and land on the crest, not stop on the shelf. */
TEST passes_a_shelf_on_the_way_to_the_crest(void) {
    double offsets[] = {-400, 0, 400};
    for (unsigned o = 0; o < 3; o++) {
        Lens l = lens(offsets[o], 300, 550, AF2_NEAR, 0x51u ^ o);
        l.sh_lo = 150; l.sh_hi = 400;   /* flat at ~0.17 of the peak, 250 ms wide */
        AfParams p = defaults();
        double f = run(&l, &p);
        if (f < 0.85) {
            static char msg[120];
            snprintf(msg, sizeof msg, "offset %.0f: land=%.0f%% (stopped on the shelf?)",
                     offsets[o], f * 100);
            FAILm(msg);
        }
    }
    PASS();
}

/* A crest just beyond the wide window: the sweep either starts on its falling flank (FV only
 * falls from the first sample) or ends on its rising one. Neither is a crest INSIDE a window --
 * the top is only the sweep's edge -- so the pass must not report it, and must put the lens back
 * where the board left it rather than on that edge. */
TEST a_crest_beyond_the_wide_window_is_not_found(void) {
    double offsets[] = {5200, -5600};
    for (unsigned o = 0; o < 2; o++)
    for (int d = -1; d <= 1; d += 2) {
        Lens l = lens(offsets[o], 300, 550, d, 0xbeefu ^ (unsigned)(o * 3 + d + 1));
        AfParams p = defaults();
        p.budget_ms = 90000;   /* the engine's (AF_TOTAL_BUDGET_MS): room for the drive back too */
        run(&l, &p);
        /* Back where the board left it: the return is FV-guided onto the starting value, so the
         * unknown slack state when the pass began does not displace it. */
        if (p.out_found_crest || fabs(l.pos) > l.backlash + 50) {
            static char msg[160];
            snprintf(msg, sizeof msg, "crest at %.0f board %+d: found=%d ended at %.0f",
                     offsets[o], d, p.out_found_crest, l.pos);
            FAILm(msg);
        }
    }
    PASS();
}

/* A budget shorter than the first move to the window edge: the move itself must stop at the
 * deadline (af2.h: never blocks beyond budget_ms), not only the sweep after it. */
TEST a_budget_shorter_than_the_first_move(void) {
    long budgets[] = {300, 1000, 1800};
    for (unsigned i = 0; i < 3; i++) {
        Lens l = lens(0, 300, 550, AF2_FAR, 0x4242u ^ i);
        AfParams p = defaults();
        p.budget_ms = budgets[i];
        run(&l, &p);
        /* The motor stops at the deadline; all that may follow is the stop's settle and the
         * one final stationary read (fv_samples frames). */
        long over = l.vclock - p.budget_ms - p.settle_ms - p.fv_samples * p.fv_frame_ms;
        if (over > 0) {
            static char msg[120];
            snprintf(msg, sizeof msg, "budget %ld: ran %ld ms over", budgets[i], over);
            FAILm(msg);
        }
    }
    PASS();
}

/* The camera's clock is not the model's: msleep overshoots, so the first sample on a sweep is
 * stamped later than one frame in. Whichever side of the start the crest lies on, within the
 * window, the pass must find it in the short window -- including a crest right where the first
 * sweep starts, which it sees only fall -- on an overshooting clock. */
TEST a_crest_near_the_start_on_a_real_clock(void) {
    for (double off = -1000; off <= 1000; off += 50)
    for (int d = -1; d <= 1; d += 2) {
        Lens l = lens(off, 250, 550, d, 0x7e57u ^ (unsigned)(off + 2000 + d));
        l.jitter = 12;
        AfParams p = defaults();
        double f = run(&l, &p);
        if (!p.out_found_crest || f < 0.85 || p.out_window != 1) {
            static char msg[160];
            snprintf(msg, sizeof msg, "crest at %.0f board %+d: found=%d window=%d land=%.0f%%",
                     off, d, p.out_found_crest, p.out_window, f * 100);
            FAILm(msg);
        }
    }
    PASS();
}

/* Zoomed out into the wide stop, the board leaves focus about a second off, and at the wide end
 * the crest is broad: the first sweep runs out of reach while the picture is still sharpening.
 * The crest is ahead, so the pass goes on the same way -- it must not turn round, find nothing,
 * and fall back to the slow wide window (measured on the rig: 134 steps instead of ~50). */
TEST a_broad_crest_just_past_the_window_is_followed(void) {
    double offsets[] = {1300, 1700, -1300, -1700};
    for (unsigned o = 0; o < 4; o++)
    for (int d = -1; d <= 1; d += 2) {
        Lens l = lens(offsets[o], 900, 550, d, 0xb0adu ^ (unsigned)(o * 5 + d + 1));
        AfParams p = defaults();
        double f = run(&l, &p);
        if (!p.out_found_crest || f < 0.85 || p.out_window != 1 || l.vclock > 9000) {
            static char msg[160];
            snprintf(msg, sizeof msg, "broad crest at %.0f board %+d: found=%d window=%d land=%.0f%% time=%ld",
                     offsets[o], d, p.out_found_crest, p.out_window, f * 100, l.vclock);
            FAILm(msg);
        }
    }
    PASS();
}

/* A fresh zoom cancels the pass: it must stop moving promptly, not finish its sweep. */
static volatile int g_cancel;
static int g_cancel_after;
static void io_drive_cancelling(void *c, int d) {
    if (--g_cancel_after == 0) g_cancel = 1;
    io_drive(c, d);
}
TEST a_cancel_stops_the_pass(void) {
    Lens l = lens(0, 300, 550, AF2_FAR, 0xcafe);
    AfParams p = defaults();
    g_cancel = 0; g_cancel_after = 2;   /* just after the sweep starts */
    p.cancel = &g_cancel;
    AfIO io = {io_drive_cancelling, io_fv, io_now, io_sleep, &l};
    af2_run(&io, &p);
    GREATEST_ASSERTm("a cancelled pass left the motor running", l.cmd == 0);
    GREATEST_ASSERTm("a cancelled pass kept going", l.vclock < 3500);
    PASS();
}

/* Through the rig's delays (lagged(): the board acts on a command 60-180 ms late, the statistic
 * shows the lens ~80 ms back), a return that stops on a reading carries the lens 100-250 ms on --
 * over a narrow crest, by luck (the wide stop on the rig: 7 of 10 passes landed down the far side,
 * at 68-89 %). The landing is checked stopped and crept onto the crest. */
TEST lands_through_the_rig_delays(void) {
    double offs[] = {-1500, -600, 0, 300, 900, 1500, 2100};
    double widths[] = {250, 350, 450, 700};
    for (unsigned w = 0; w < 4; w++) {
        int n = 0; double sum = 0;
        for (unsigned o = 0; o < 7; o++) for (int d = -1; d <= 1; d += 2) for (unsigned k = 0; k < 4; k++) {
            Lens l = lagged(lens(offs[o], widths[w], 550, d, 0x1a9u ^ (o * 131 + w * 17 + k * 7 + (unsigned)(d + 1))));
            AfParams p = defaults(); p.budget_ms = 90000;   /* the engine's */
            p.in_start_fv = io_fv(&l);                       /* the engine measures it first */
            double f = run(&l, &p);
            /* 250 ms is narrower than any crest the statistic shows on the rig (~450-500 ms at
             * the wide stop, the narrowest); with pulses as irregular as the board's (+-120 ms)
             * single landings there are luck. Only its mean is held. */
            if (widths[w] >= 350 && f < 0.85) {
                static char msg[160];
                snprintf(msg, sizeof msg, "crest at %.0f width %.0f board %+d seed %u: land=%.0f%%",
                         offs[o], widths[w], d, k, f * 100);
                FAILm(msg);
            }
            n++; sum += f;
        }
        if (sum / n < 0.95) {
            static char msg[120];
            snprintf(msg, sizeof msg, "width %.0f: mean landing %.1f%%", widths[w], 100 * sum / n);
            FAILm(msg);
        }
    }
    PASS();
}

SUITE(af2_suite) {
    RUN_TEST(lands_after_the_boards_tracking);
    RUN_TEST(widens_for_a_crest_outside_the_window);
    RUN_TEST(no_contrast_is_reported_not_invented);
    RUN_TEST(a_short_budget_is_respected);
    RUN_TEST(lands_on_a_crest_barely_above_the_floor);
    RUN_TEST(return_recognises_a_crest_that_reads_lower);
    RUN_TEST(passes_a_shelf_on_the_way_to_the_crest);
    RUN_TEST(a_crest_beyond_the_wide_window_is_not_found);
    RUN_TEST(a_budget_shorter_than_the_first_move);
    RUN_TEST(a_crest_near_the_start_on_a_real_clock);
    RUN_TEST(a_broad_crest_just_past_the_window_is_followed);
    RUN_TEST(a_cancel_stops_the_pass);
    RUN_TEST(lands_through_the_rig_delays);
}

