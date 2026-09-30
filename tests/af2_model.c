/* af2 against a synthetic lens and scene: the 85H50AI as measured on its healthy lenses
 * (OpenIPC/motors xm-uart/PROTOCOL.md, "Zoom tracking inside the board").
 *
 *  - The crest is narrow: 90 % of peak sharpness spans 0.1-0.3 s of focus drive, and the
 *    statistic sits on a flat floor a second either side. Modelled as a Gaussian of width
 *    200-450 ms over a floor.
 *  - A reversal takes up ~0.5 s of gear slack before focus moves (measured >= 0.50 s).
 *  - After a zoom the board has left focus within about 0.6 s of the crest, and the slack is in
 *    whatever state the board's own last move left it.
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
static void io_drive(void *c, int d) {
    Lens *l = c;
    if (d != 0) {
        if (d != l->last_cmd && l->last_cmd != 0) l->slack = l->backlash;
        l->last_cmd = d;
        if (l->last_nz != 0 && d != l->last_nz) l->reversals++;
        l->last_nz = d;
    }
    l->cmd = d;
}
static void io_sleep(void *c, long ms) {
    Lens *l = c;
    if (ms > 0) ms += l->jitter;
    if (l->cmd != 0 && ms > 0) {
        double move = ms;
        if (l->slack > 0) { double k = move < l->slack ? move : l->slack; l->slack -= k; move -= k; }
        l->pos += l->cmd * move;
    }
    l->vclock += ms;
}
static unsigned io_fv(void *c) {
    Lens *l = c;
    double v = l->floor + (l->peak_h - l->floor) * sharpf(l);
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
        if (!p.out_found_crest || p.out_window != 2 || f < 0.85 || l.vclock > 25000) {
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
        run(&l, &p);
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
 * stamped later than one frame in. A crest right at the edge the sweep starts from -- FV only
 * falls from there -- must still send the pass to the wide window, not be taken as a crest
 * inside the short one. Swept across that edge, with a sleep that overshoots. */
TEST a_crest_at_the_starting_edge_on_a_real_clock(void) {
    for (double off = 1300; off <= 2100; off += 50)
    for (int d = -1; d <= 1; d += 2) {
        Lens l = lens(off, 250, 550, d, 0x7e57u ^ (unsigned)(off + d));
        l.jitter = 12;
        AfParams p = defaults();
        double f = run(&l, &p);
        /* Where the short sweep starts: the window edge, plus the backlash af2 paid on its
         * first move -- which the gear only owed if the board's last move was NEAR. */
        double start = AF2_WINDOW_MS + (d == AF2_FAR ? AF2_BACKLASH_MS : 0);
        int beyond = off > start + 100;   /* FV can only fall in the short sweep */
        if (!p.out_found_crest || f < 0.85 || (beyond && p.out_window != 2)) {
            static char msg[160];
            snprintf(msg, sizeof msg, "crest at %.0f board %+d: found=%d window=%d land=%.0f%%",
                     off, d, p.out_found_crest, p.out_window, f * 100);
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
    RUN_TEST(a_crest_at_the_starting_edge_on_a_real_clock);
    RUN_TEST(a_cancel_stops_the_pass);
}
