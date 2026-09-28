/* af3 (step-based bracket-and-return AF) against a synthetic STEPPER lens+scene model.
 *
 * This is the regression guard for the vendor libxmaf method (OpenIPC/motors AUTOFOCUS.md).
 * It drives the real src/af3.c against a lens that has the two properties that defeat a naive
 * search on this hardware:
 *
 *   1. A POST-MOVE SETTLE TRANSIENT. Right after a step the focus statistic is dominated by a
 *      large motion smear that decays over ~5 frames. A search that reads the FV a frame after
 *      moving samples the transient, not the sharpness. af3 must wait settle_frames — the model
 *      makes a too-early reader land nowhere.
 *   2. REVERSAL BACKLASH. The dead-reckoned position (what the backend returns and af3 integrates)
 *      does NOT track the physical position across a direction reversal — the first `backlash`
 *      commanded steps take up gear slack and move the lens not at all. The Gaussian sharpness is
 *      computed from the PHYSICAL position, so a landing that trusts the count is exposed here.
 *
 * Self-contained, virtual clock (ms), deterministic. The engine is told none of truepk / width /
 * floor / peakh / backlash; it must find and hold the crest from wherever the lens starts. */
#include <greatest.h>

#include <majestic/af3.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#define MS_PER_STEP 2       /* a microstep burst's wall time, ms */
#define SETTLE_TAU  78.0    /* motion-smear decay constant, ms (measured: ~gone by ~300ms) */

typedef struct {
    long vclock;
    long T;              /* travel, microsteps */
    long drpos;          /* dead-reckoned position (what step() returns / af3 integrates) */
    double phys;         /* physical position (drives sharpness) — drifts from drpos via slack */
    long slack;          /* backlash steps still to be taken up in the current direction */
    int last_dir;
    long last_move;      /* vclock of the last move, for the settle transient */
    /* engine is NOT told any of these: */
    double truepk, width, floor, peakh, backlash, smear;
    double tex;          /* floor TEXTURE amplitude, as a fraction of the floor (0 = flat floor) */
    unsigned rng;
} Lens;

static double cl(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }
static double lr(Lens *l) { l->rng = l->rng * 1103515245u + 12345u; return ((l->rng >> 16) & 0x7fff) / 32767.0; }

static double gauss(Lens *l, double at) {
    double x = (at - l->truepk) / l->width;
    return exp(-x * x);
}
static double sharp_now(Lens *l) { return gauss(l, l->phys); }

/* The defocused floor is not flat: the statistic picks up whatever low-frequency structure the
 * blurred scene still has, and that varies with focus position. Measured on the HI3516D_N81820
 * (imx291) lab camera stepping the whole travel: a floor of 19-28 with bumps to 41, i.e. up to
 * 2.2x the minimum, hundreds of microsteps from the crest. Three fixed bumps reproduce that. */
static double texture(Lens *l, double at) {
    static const double pos[] = {583, 905, 1130}, wid[] = {35, 40, 30}, ht[] = {1.0, 0.75, 0.9};
    double t = 0;
    for (int i = 0; i < 3; i++) {
        double x = (at - pos[i]) / wid[i];
        t += ht[i] * exp(-x * x);
    }
    (void)l;
    return t;
}

static long io_now(void *c) { return ((Lens *)c)->vclock; }
static void io_sleep(void *c, long ms) { ((Lens *)c)->vclock += ms > 0 ? ms : 0; }

static int io_step(void *c, int dir, int n) {
    Lens *l = c;
    if (n <= 0) return 0;
    /* dead-reckoned advance, clamped to the soft travel [0,T] — this is what the real backend
     * returns (it counts commanded steps; it knows nothing of backlash). */
    long want = n;
    if (dir > 0 && l->drpos + want > l->T) want = l->T - l->drpos;
    if (dir < 0 && l->drpos - want < 0) want = l->drpos;
    if (want < 0) want = 0;
    long moved = want;
    l->drpos += (long)dir * moved;

    /* physical motion: a reversal first takes up gear slack, which moves the lens not at all. */
    if (dir != l->last_dir && l->last_dir != 0) l->slack = (long)l->backlash;
    long cmd = moved;
    long absorb = cmd < l->slack ? cmd : l->slack;
    l->slack -= absorb;
    cmd -= absorb;
    l->phys = cl(l->phys + (double)dir * cmd, 0, l->T);
    l->last_dir = dir;

    l->vclock += moved * MS_PER_STEP;   /* the burst takes wall time */
    l->last_move = l->vclock;           /* restart the settle transient */
    return (int)moved;
}

static unsigned io_fv(void *c) {
    Lens *l = c;
    double floorv = l->floor * (1.0 + l->tex * texture(l, l->phys));
    double base = floorv + (l->peakh - l->floor) * sharp_now(l);
    double dt = (double)(l->vclock - l->last_move);
    double smear = l->smear * exp(-dt / SETTLE_TAU);   /* motion transient, decays after a move */
    double v = base + smear;
    v *= (1.0 + 0.01 * (lr(l) - 0.5) * 2.0);           /* ~1% frame noise */
    if (v < 1) v = 1;
    return (unsigned)(v + 0.5);
}

static Af3IO lens_io(Lens *l) { Af3IO io = {io_step, io_fv, io_now, io_sleep, l}; return io; }
static Af3Params defaults(long T, long backlash) {
    Af3Params p; memset(&p, 0, sizeof p);
    p.focus_steps = T; p.backlash_steps = backlash;
    p.frame_ms = 40; p.settle_frames = 6; p.fv_samples = 3;
    // A full-travel cold sweep of a 1280-microstep lens at a settle-limited read rate is inherently
    // slow (tens of seconds); the on-hardware budget (AF_TOTAL_BUDGET_MS) is 90 s. A warm re-AF near
    // focus is far quicker. 40 s bounds the model's worst case (a blind sweep from the far stop).
    p.budget_ms = 40000;
    return p;
}
static void lens_init(Lens *l, long T, double truepk, double width, double floor,
                      double peakh, double backlash, double start, unsigned seed) {
    memset(l, 0, sizeof *l);
    l->T = T; l->truepk = truepk; l->width = width; l->floor = floor; l->peakh = peakh;
    /* Post-move motion smear ~ the scene's edge contrast, i.e. a fraction of the peak's rise.
     * Measured on this hardware: a spike of ~275 over a floor of 11 with a sharp peak near 2000,
     * i.e. ~14% of the rise, decaying (SETTLE_TAU) to the floor by ~300 ms. It is negligible next
     * to a sharp peak but dominates the floor, which is exactly what fools a too-early reader. */
    l->backlash = backlash; l->smear = (peakh - floor) * 0.14;
    l->drpos = (long)start; l->phys = start; l->last_move = -100000; l->rng = seed;
}

/* CONVERGE: from starts all over the travel, across peak position / width / floor / peak height,
 * af3 must land on the crest (sharpness of the PHYSICAL position), recognise the crest, stay
 * bounded (no hunting), and never overrun the budget. */
TEST af3_converges_and_is_bounded(void) {
    const long T = 1280, BL = 40;
    // Realistic for this lens (measured on hardware): a NARROW focus peak (~20-70 microsteps),
    // HIGH contrast (a sharp FV near ~2000 over a floor of tens), across the travel and from any
    // cold start. `peaks[]` is the RISE above the floor; the lowest here is still ~4:1 contrast.
    double pks[]   = {160, 420, 640, 900, 1120};
    double wids[]  = {10, 25, 50};
    double floors[]= {3, 12, 50};
    double peaks[] = {200, 700, 1800};
    int total = 0, ok = 0, rev_max = 0;
    for (unsigned a = 0; a < 5; a++)
    for (unsigned b = 0; b < 3; b++)
    for (unsigned c = 0; c < 3; c++)
    for (unsigned d = 0; d < 3; d++) {
        Lens l;
        unsigned seed = 0x100 ^ (unsigned)(long)(pks[a] * 7 + wids[b] * 13 + floors[c] * 5 +
                                                 peaks[d] * 3);
        // A cold pass homes to the near stop first (the engine calls motion_focus_home), so af3
        // starts at a KNOWN 0 and sweeps FAR — there is no "unknown position, guess the direction"
        // on hardware. Model that: lens homed at 0, in_pos = 0. (A warm re-AF from near focus is
        // covered by af3_refines_from_a_good_start.)
        lens_init(&l, T, pks[a], wids[b], floors[c], floors[c] + peaks[d], BL, 0, seed);
        Af3IO io = lens_io(&l);
        Af3Params p = defaults(T, BL);
        p.in_pos = 0;
        long t0 = l.vclock;
        af3_run(&io, &p);
        long elapsed = l.vclock - t0;
        double f = sharp_now(&l);
        total++;
        if (f >= 0.85 && p.out_found_crest && elapsed <= p.budget_ms + 1000) ok++;
        else {
            static char msg[224];
            snprintf(msg, sizeof msg,
                     "pk=%.0f w=%.0f fl=%.0f ph=%.0f: land=%.0f%% crest=%d rev=%d %ldms",
                     pks[a], wids[b], floors[c], floors[c] + peaks[d],
                     f * 100, p.out_found_crest, p.out_reversals, elapsed);
            FAILm(msg);
        }
        if (p.out_reversals > rev_max) rev_max = p.out_reversals;
    }
    (void)ok; (void)total;
    GREATEST_ASSERTm("af3 hunts (too many motor reversals)", rev_max <= 8);
    PASS();
}

/* SHALLOW crest: only a few counts above the floor (a dim/wide scene), still found and held.
 * Every crest here is at least 3x its floor: the floor's own texture reaches 2.2x (measured, see
 * af3_ignores_floor_texture), so a crest under that is indistinguishable from it on this lens. */
TEST af3_lands_on_a_shallow_crest(void) {
    const long T = 1280, BL = 40;
    double floors[] = {9, 5, 20};
    double peaks[]  = {30, 16, 80};   /* absolute peak height */
    double starts[] = {200, 700, 1150};
    for (unsigned i = 0; i < 3; i++)
    for (unsigned s = 0; s < 3; s++) {
        Lens l;
        lens_init(&l, T, 520, 80, floors[i], peaks[i], BL, starts[s],
                  0x9e37 ^ (unsigned)(long)(peaks[i] * 91 + starts[s]));
        Af3IO io = lens_io(&l);
        Af3Params p = defaults(T, BL);
        p.in_pos = -1;
        af3_run(&io, &p);
        double f = sharp_now(&l);
        if (f < 0.75) {
            static char msg[192];
            snprintf(msg, sizeof msg, "shallow floor=%.0f peak=%.0f from=%.0f: land=%.0f%% crest=%d",
                     floors[i], peaks[i], starts[s], f * 100, p.out_found_crest);
            FAILm(msg);
        }
    }
    PASS();
}

/* NO GRADIENT: the scene is flat (peak ~ floor), so there is no crest to find. af3 must not
 * claim one, must not hunt, and must respect the budget. */
TEST af3_no_gradient_is_honest_and_bounded(void) {
    const long T = 1280, BL = 40;
    Lens l;
    lens_init(&l, T, 640, 80, 30, 33, BL, 300, 0x77a1);  /* peak only 3 over floor of 30: noise */
    Af3IO io = lens_io(&l);
    Af3Params p = defaults(T, BL);
    p.in_pos = -1;
    long t0 = l.vclock;
    af3_run(&io, &p);
    long over = (l.vclock - t0) - p.budget_ms;
    GREATEST_ASSERTm("flat scene wrongly reported a crest", !p.out_found_crest);
    GREATEST_ASSERTm("flat scene hunted", p.out_reversals <= 8);
    if (over > 1000) {
        static char msg[128];
        snprintf(msg, sizeof msg, "no-gradient pass ran %ld ms past its %ld ms budget", over, p.budget_ms);
        FAILm(msg);
    }
    PASS();
}

/* BUDGET: sized to expire mid-search; af3 must not run past it (af3.h: never blocks beyond budget). */
TEST af3_respects_the_budget(void) {
    const long T = 1280, BL = 40;
    Lens l;
    lens_init(&l, T, 1120, 60, 9, 200, BL, 0, 0x515);   /* peak far from the start */
    Af3IO io = lens_io(&l);
    Af3Params p = defaults(T, BL);
    p.in_pos = -1;
    p.budget_ms = 3000;                                  /* forces an early cut */
    long t0 = l.vclock;
    af3_run(&io, &p);
    long over = (l.vclock - t0) - p.budget_ms;
    if (over > 1000) {
        static char msg[128];
        snprintf(msg, sizeof msg, "pass ran %ld ms past its %ld ms budget", over, p.budget_ms);
        FAILm(msg);
    }
    PASS();
}

/* A GOOD START (a re-AF at the same zoom): in_pos near the peak, af3 refines quickly and lands. */
TEST af3_refines_from_a_good_start(void) {
    const long T = 1280, BL = 40;
    for (int off = -120; off <= 120; off += 60) {
        Lens l;
        double start = cl(640 + off, 0, T);
        lens_init(&l, T, 640, 70, 12, 400, BL, start, 0x2f1b ^ (unsigned)off);
        Af3IO io = lens_io(&l);
        Af3Params p = defaults(T, BL);
        p.in_pos = (long)start;
        p.warm = 1;   // a re-AF at a known focus: the "already on the crest?" fast path is allowed
        af3_run(&io, &p);
        double f = sharp_now(&l);
        if (f < 0.85) {
            static char msg[160];
            snprintf(msg, sizeof msg, "good-start off=%d: land=%.0f%% rev=%d", off, f * 100, p.out_reversals);
            FAILm(msg);
        }
    }
    PASS();
}

/* FLOOR TEXTURE (measured on the HI3516D_N81820 / imx291 lab camera, 2026-09-28): the defocused
 * statistic is NOT flat. Stepping the whole travel in 53-microstep pulses read a floor of 19-28
 * with position-dependent bumps up to 41 (2.2x the minimum), while the crest read 3160 and its
 * flank was already down to 31-58 one pulse away. Two cold passes from the far stop stopped on a
 * bump near 600 and reported "done" with a peak of 53 and 44 -- a crest recognised in floor
 * texture, 200 microsteps short of focus. A crest that is only floor-sized is not a crest. The
 * passes the engine runs after a manual move are warm (position known) with no hold value. */
TEST af3_ignores_floor_texture(void) {
    const long T = 1280, BL = 40;
    double starts[] = {1280, 600, 900, 0};
    for (unsigned i = 0; i < 4; i++) {
        Lens l;
        lens_init(&l, T, 387, 25, 23, 3160, BL, starts[i], 0x5a17 ^ (unsigned)(long)starts[i]);
        l.tex = 1.2;
        Af3IO io = lens_io(&l);
        Af3Params p = defaults(T, BL);
        p.in_pos = (long)starts[i];
        p.warm = starts[i] != 0;      /* 0 = a cold pass straight off the home stop */
        p.hold_fv = 0;
        af3_run(&io, &p);
        double f = sharp_now(&l);
        if (f < 0.85 || !p.out_found_crest) {
            static char msg[192];
            snprintf(msg, sizeof msg, "texture from=%.0f: land=%.0f%% crest=%d peak=%u pos=%ld rev=%d",
                     starts[i], f * 100, p.out_found_crest, p.out_peak_seen, p.out_pos,
                     p.out_reversals);
            FAILm(msg);
        }
    }
    PASS();
}

SUITE(af3_suite) {
    RUN_TEST(af3_converges_and_is_bounded);
    RUN_TEST(af3_lands_on_a_shallow_crest);
    RUN_TEST(af3_no_gradient_is_honest_and_bounded);
    RUN_TEST(af3_respects_the_budget);
    RUN_TEST(af3_refines_from_a_good_start);
    RUN_TEST(af3_ignores_floor_texture);
}
