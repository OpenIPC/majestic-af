/* af2 (parfocal autofocus) against a synthetic PARFOCAL lens+scene model. Guards what the
 * offline model in scratchpad/model/ proved: the focus peak moves with zoom along
 * af2_parfocal_foc(mag); the engine COLD-focuses from an unknown position by seeking the near
 * stop and driving the curve, and TRACKS across a sequence of zoom changes by driving to the
 * ABSOLUTE parfocal target for the new zoom from the carried (dead-reckoned) position — both
 * landing on a sharp peak that sits over a flat, low, integer contrast floor, without being
 * told the travel time or backlash.
 *
 * Self-contained, virtual-clock, deterministic. The true peak is af2_parfocal_foc(mag) plus a
 * scene offset the engine does NOT know, so the trim has to find it. The model varies the real
 * backlash around the engine's assumption so the dead-reckoned position drifts, exactly as the
 * hardware does — the test asserts the engine still lands. */
#include <greatest.h>

#include <majestic/af2.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    long vclock;
    double pos, slack; int cmd, last_cmd;
    double travel, backlash, offset, width, floor;  /* engine is NOT told these */
    double sh_lo, sh_hi;      /* a flat FV shoulder on the FAR approach: [truepk+lo, truepk+hi] */
    double peak_h;            /* > 0 overrides peakh(mag): a scene whose whole FV scale is
                               * collapsed, which peakh() (a function of zoom alone) cannot say */
    double rev_pen;           /* the same crest reads LOWER once the motor has reversed --
                               * different backlash, different sampling phase. Traced at 96.7%
                               * on an 85H50AI, which is under the return's 95% target, so the
                               * absolute-threshold stop is the only thing left to catch it.
                               * Applied from the first reversal, which on the TRACK path (no
                               * cold seek ahead of the sweep) is the return itself. */
    double mag;
    unsigned rng;
    int reversals, last_nz;   /* motor direction reversals this pass — the JOURNEY (no hunting) */
} Lens;

static double cl(double x, double lo, double hi) { return x < lo ? lo : x > hi ? hi : x; }
static double lr(Lens *l) { l->rng = l->rng * 1103515245u + 12345u; return ((l->rng >> 16) & 0x7fff) / 32767.0; }
static double truepk(Lens *l) { return cl(af2_parfocal_foc((float)l->mag) + l->offset, 0, l->travel); }
/* Sharpness = a Gaussian on the true peak. A wide multi-plane scene (a near frame plus a far
 * subject) also puts a nearer plane's contrast on the FV rise: model it as a FLAT SHELF held at
 * the [truepk+sh_hi] level across [truepk+sh_lo, truepk+sh_hi] on the FAR side, so the monotonic
 * rise toward the crest flattens there for a stretch and then climbs on — exactly the shoulder
 * the hardware shows, which a too-eager plateau stop mistakes for the crest. */
static double sharpf(Lens *l) {
    double d = l->pos - truepk(l);
    double ad = d < 0 ? -d : d;
    if (l->sh_hi > 0 && d > l->sh_lo && d < l->sh_hi) ad = l->sh_hi;
    double x = ad / l->width; return exp(-x * x);
}
static double peakh(double mag) { return 600.0 + (mag - 1.0) * 1520.0; }

static long io_now(void *c) { return ((Lens *)c)->vclock; }
static void io_drive(void *c, int d) {
    Lens *l = c;
    if (d != 0) {
        /* Reversal backlash grows toward the near stop on the real lens (~400 ms out, ~700 near),
         * so a landing that leans on a fixed backlash is exposed here, not only on hardware. */
        if (d != l->last_cmd && l->last_cmd != 0)
            l->slack = l->backlash + 250.0 * (1.0 - l->pos / l->travel);
        l->last_cmd = d;
        if (l->last_nz != 0 && d != l->last_nz) l->reversals++;
        l->last_nz = d;
    }
    l->cmd = d;
}
static void io_sleep(void *c, long ms) {
    Lens *l = c;
    if (l->cmd != 0 && ms > 0) {
        double move = ms;
        if (l->slack > 0) { double k = move < l->slack ? move : l->slack; l->slack -= k; move -= k; }
        if (move > 0) l->pos = cl(l->pos + l->cmd * move, 0, l->travel);
    }
    l->vclock += ms;
}
static unsigned io_fv(void *c) {
    Lens *l = c;
    double ph = l->peak_h > 0 ? l->peak_h : peakh(l->mag);
    double v = l->floor + (ph - l->floor) * sharpf(l);
    if (l->rev_pen > 0 && l->reversals >= 1) v *= (1.0 - l->rev_pen);
    v *= (1.0 + 0.01 * (lr(l) - 0.5) * 2.0);
    if (v < 1) v = 1;
    return (unsigned)(v + 0.5);
}
static AfIO lens_io(Lens *l) { AfIO io = {io_drive, io_fv, io_now, io_sleep, l}; return io; }
static AfParams defaults(void) {
    AfParams p; memset(&p, 0, sizeof p);
    p.travel_max_ms = 42000; p.travel_ms = 38000; p.backlash_ms = 400; p.settle_ms = 160;
    p.budget_ms = 90000;
    p.fv_samples = 5; p.fv_frame_ms = 40;
    return p;
}
/* Run one pass at magnification `mag`, threading the dead-reckoned focus position through
 * `*focus_pos` (< 0 = unknown -> cold). Returns the landed sharpness fraction (1.0 = on peak). */
static double run_pass_told(Lens *l, double mag, double told_mag, long *focus_pos, int *path) {
    l->mag = mag;
    l->reversals = 0; l->last_nz = 0;
    AfIO io = lens_io(l); AfParams p = defaults();
    p.mag_now = (float)told_mag; p.in_focus_pos = *focus_pos;
    af2_run(&io, &p);
    *focus_pos = p.out_focus_pos;
    if (path) *path = p.out_path;
    return sharpf(l);
}
static double run_pass(Lens *l, double mag, long *focus_pos, int *path) {
    return run_pass_told(l, mag, mag, focus_pos, path);
}

/* The calibrated curve is monotonic in magnification and hits the measured anchor points. */
TEST parfocal_curve_is_monotonic(void) {
    long prev = -1;
    for (float m = 1.0f; m <= 5.0f; m += 0.1f) {
        long f = af2_parfocal_foc(m);
        GREATEST_ASSERTm("parfocal curve not monotonic in zoom", f >= prev);
        prev = f;
    }
    GREATEST_ASSERTm("wide should be at the near stop", af2_parfocal_foc(1.0f) == 0);
    GREATEST_ASSERTm("tele should be far down the range", af2_parfocal_foc(5.0f) > 20000);
    PASS();
}

/* TRACK after a zoom, as the hardware does it: zooming mechanically displaces the focus element
 * to ~peak + a roughly constant overshoot toward FAR (measured), which INVALIDATES any carried
 * position — so the engine re-seeds in_focus_pos = curve(mag) + a nominal overshoot and drives
 * NEAR onto the peak. Model that for a whole zoom itinerary: place the lens at the true peak
 * plus a *varying* real overshoot, seed the *nominal* one, and require the seeded TRACK path to
 * land on the crest, for distant AND offset scenes and a range of peak widths. */
TEST tracks_a_zoom_itinerary(void) {
    const long nominal_overshoot = 6800;   /* what af.c seeds; the real one varies below */
    double mags[] = {1.4, 1.8, 2.6, 3.4, 4.2, 5.0, 3.0, 1.6};
    double offsets[] = {0, 1500, -1500};
    double kact[] = {5800, 6800, 7800};    /* real post-zoom overshoot, off the nominal */
    double widths[] = {1500, 1900, 2600};
    for (unsigned o = 0; o < 3; o++)
    for (unsigned k = 0; k < 3; k++)
    for (unsigned w = 0; w < 3; w++)
    for (unsigned i = 0; i < sizeof(mags)/sizeof(mags[0]); i++) {
        Lens l; memset(&l, 0, sizeof l);
        l.travel = 38000; l.backlash = 400; l.offset = offsets[o];
        l.width = widths[w]; l.floor = 3; l.mag = mags[i];
        // Cast through a signed integer: converting a negative float straight to
        // unsigned is undefined (C11 6.3.1.4); float->long->unsigned is defined and
        // deterministic. (offsets can be negative.)
        l.rng = 0x99 ^ (unsigned)(long)(mags[i] * 91 + offsets[o] + kact[k] + widths[w]);
        l.pos = cl(truepk(&l) + kact[k], 0, l.travel);   /* where the zoom left focus */
        long fp = af2_parfocal_foc((float)mags[i]) + nominal_overshoot;  /* the seed */
        int path;
        double f = run_pass(&l, mags[i], &fp, &path);
        /* Land on the crest AND get there smoothly: a couple of motor reversals (sweep, one
         * return), never the hunting oscillation a hill-climb makes. */
        if (path != 1 || f < 0.80 || l.reversals > 4) {
            static char msg[176];
            snprintf(msg, sizeof msg, "track ->%.1f off=%.0f Kact=%.0f w=%.0f: path=%d land=%.0f%% reversals=%d",
                     mags[i], offsets[o], kact[k], widths[w], path, f * 100, l.reversals);
            FAILm(msg);
        }
    }
    PASS();
}

/* COLD: from an unknown focus position, seek the near stop then a single smooth sweep onto the
 * crest, across zoom and scene distance — and, like TRACK, without hunting. */
TEST cold_focus_from_unknown(void) {
    double mags[] = {1.0, 1.5, 2.0, 3.0};
    double offsets[] = {0, 2000, -1500};
    int total = 0, ok = 0, rev_max = 0;
    for (unsigned m = 0; m < 4; m++)
    for (unsigned o = 0; o < 3; o++)
    for (unsigned s = 0; s < 3; s++) {
        Lens l; memset(&l, 0, sizeof l);
        l.travel = 38000; l.backlash = 400; l.offset = offsets[o];
        l.width = 1900; l.floor = 3; l.pos = s * 18000;
        l.rng = 0x1234 ^ (unsigned)(long)(mags[m] * 131 + offsets[o] + s);
        long fp = -1; int path;
        double f = run_pass(&l, mags[m], &fp, &path);
        total++; if (f >= 0.80 && path == 2) ok++;
        if (l.reversals > rev_max) rev_max = l.reversals;
    }
    GREATEST_ASSERTm("cold focus reliability below 85%", ok * 100 >= total * 85);
    GREATEST_ASSERTm("cold focus hunts (too many motor reversals)", rev_max <= 4);
    PASS();
}

/* A wide scene lays a FLAT SHOULDER on the FV rise short of the true crest (a near plane's
 * contrast), and the sweep must PASS it and land on the crest — not stop on the shoulder, which
 * on hardware left a wide zoom soft. The shoulder is ~640 ms of flat, narrower than the sustained
 * flat only an end-stop clamp holds, so the plateau stop must not fire on it. Runs the seeded
 * TRACK path (as after a zoom) toward a mid-range and a wide-end peak. */
TEST tracks_past_a_shoulder(void) {
    const long nominal_overshoot = 6800;
    double mags[] = {1.2, 2.6, 3.4};        /* wide-end (peak near the stop) and mid-range */
    for (unsigned i = 0; i < sizeof(mags)/sizeof(mags[0]); i++) {
        Lens l; memset(&l, 0, sizeof l);
        l.travel = 38000; l.backlash = 400; l.offset = 0;
        l.width = 1900; l.floor = 3; l.mag = mags[i];
        l.sh_lo = 960; l.sh_hi = 1600;      /* flat shelf at ~0.49 of peak, ~640 ms wide */
        l.rng = 0x51 ^ (unsigned)(long)(mags[i] * 97);
        l.pos = cl(truepk(&l) + 6800, 0, l.travel);   /* where a zoom left focus */
        long fp = af2_parfocal_foc((float)mags[i]) + nominal_overshoot;
        int path;
        double f = run_pass(&l, mags[i], &fp, &path);
        if (path != 1 || f < 0.80 || l.reversals > 4) {
            static char msg[176];
            snprintf(msg, sizeof msg, "shoulder ->%.1f: path=%d land=%.0f%% reversals=%d (stopped on the shoulder?)",
                     mags[i], path, f * 100, l.reversals);
            FAILm(msg);
        }
    }
    PASS();
}

/* A crest only TENS of counts above the floor must still be found — and, far more important,
 * the pass must never END further from the best focus it measured than where it began.
 *
 * This is the 2026-09-17 x1.0 capture on an 85H50AI, in numbers: `done fv=25 peak=27 start=31
 * mag=1.0 pos=8030 steps=90 path=2`. The statistic has no absolute scale, so a wide or dim
 * scene can put the whole peak-to-floor range inside a few dozen counts; the sweep used to
 * require a fixed 40 of rise before it would believe a peak existed, and EVERYTHING hung off
 * that one flag — the crest break, the plateau break, and the return onto the crest. Below the
 * bar the lens swept its entire budget away from the crest it had been standing on and stopped
 * there, reporting `done`.
 *
 * At mag 1.0 the curve target IS the near stop, so a cold pass starts on the peak and drives
 * away from it: the crest is at top_on ~ 0 and only the return brings the lens back. That makes
 * this the exact shape the old code could not handle. */
TEST lands_on_a_crest_barely_above_the_floor(void) {
    /* floor/peak pairs spanning the collapse: the field capture, and tighter still. */
    const double floors[] = {9, 9, 5, 3};
    const double peaks[]  = {31, 48, 22, 14};
    for (unsigned i = 0; i < sizeof(peaks)/sizeof(peaks[0]); i++) {
        Lens l; memset(&l, 0, sizeof l);
        l.travel = 38000; l.backlash = 400; l.offset = 0;   /* true peak = curve target = near stop */
        l.width = 1900; l.floor = floors[i]; l.peak_h = peaks[i];
        l.pos = 12000;                                      /* where a zoom left focus */
        l.rng = 0x9e37 ^ (unsigned)(long)peaks[i];
        l.mag = 1.0;
        AfIO io = lens_io(&l); AfParams p = defaults();
        p.mag_now = 1.0f; p.in_focus_pos = -1;              /* cold, as the first pass always is */
        long t0 = l.vclock;
        af2_run(&io, &p);
        long elapsed = l.vclock - t0;
        double f = sharpf(&l);
        /* af2.h: "Never blocks beyond budget_ms." The counted return added for the
         * no-gradient case sleeps a distance, so it has to respect the deadline like
         * everything else; the final fv_med is the only slack allowed. */
        if (elapsed > p.budget_ms + 1000) {
            static char tmsg[160];
            snprintf(tmsg, sizeof tmsg, "peak %.0f: pass ran %ld ms past its %ld ms budget",
                     peaks[i], elapsed - p.budget_ms, p.budget_ms);
            FAILm(tmsg);
        }
        if (!p.out_found_crest || f < 0.80 || p.out_focus_pos > 2500) {
            static char msg[208];
            snprintf(msg, sizeof msg,
                     "peak %.0f over floor %.0f: crest=%d land=%.0f%% pos=%ld (walked off the crest?)",
                     peaks[i], floors[i], p.out_found_crest, f * 100, p.out_focus_pos);
            FAILm(msg);
        }
    }
    PASS();
}

/* The RETURN onto the crest must recognise the crest for a shallow peak too.
 *
 * Distinct from the case above in one decisive way: the crest is MID-RANGE, not on
 * the near stop, so the lens CAN sail past it -- at the wide stop it simply clamps and
 * the defect is invisible. The return reads 6% low (measured 96.7% on an 85H50AI), so
 * it never reaches the 95% target and the fall detector is the only thing that can stop
 * the motor; an absolute rise threshold inside the return loop switches that detector
 * off for a peak this shallow and the lens runs to the time bound, ending down the far
 * flank. */
TEST return_recognises_a_shallow_crest(void) {
    const double mags[] = {2.6, 3.1};
    for (unsigned i = 0; i < sizeof(mags)/sizeof(mags[0]); i++) {
        Lens l; memset(&l, 0, sizeof l);
        l.travel = 38000; l.backlash = 400; l.offset = 0;
        l.width = 1900; l.floor = 9; l.peak_h = 50; l.rev_pen = 0.12;
        l.mag = mags[i];
        l.rng = 0x2f1b ^ (unsigned)(long)(mags[i] * 131);
        l.pos = cl(truepk(&l) + 6800, 0, l.travel);        /* where a zoom left focus */
        long fp = af2_parfocal_foc((float)mags[i]) + 6800; /* the nominal seed: TRACK path */
        int path;
        double f = run_pass(&l, mags[i], &fp, &path);
        if (path != 1 || f < 0.70) {
            static char msg[176];
            snprintf(msg, sizeof msg,
                     "shallow crest at x%.1f: path=%d land=%.0f%% (sailed past the crest?)",
                     mags[i], path, f * 100);
            FAILm(msg);
        }
    }
    PASS();
}

/* af2.h promises the pass never blocks beyond budget_ms. The counted return added for
 * the no-gradient case sleeps a distance, so it must respect the deadline like every
 * other move. Budget is sized to expire DURING the sweep, which is the only way to
 * reach that branch with time already spent. */
TEST no_gradient_return_respects_the_budget(void) {
    Lens l; memset(&l, 0, sizeof l);
    l.travel = 38000; l.backlash = 400; l.offset = 0;
    l.width = 1900; l.floor = 9; l.peak_h = 12;    /* below the bar: no crest recognised */
    l.pos = 12000; l.mag = 1.0; l.rng = 0x77a1;
    AfIO io = lens_io(&l); AfParams p = defaults();
    p.budget_ms = 46000;                            /* cold seek ~42 s, then the sweep expires */
    p.mag_now = 1.0f; p.in_focus_pos = -1;
    long t0 = l.vclock;
    af2_run(&io, &p);
    long over = (l.vclock - t0) - p.budget_ms;
    if (over > 1000) {
        static char msg[160];
        snprintf(msg, sizeof msg, "pass ran %ld ms past its %ld ms budget", over, p.budget_ms);
        FAILm(msg);
    }
    PASS();
}

/* COLD with NO MAGNIFICATION — the lens is somewhere, the engine has not been told where.
 *
 * The 85H50AI MCU reports its magnification only WHILE the zoom motor turns, so a camera that
 * has rebooted and not been zoomed since has none, and a freshly flashed one has never had any.
 * af2.h documents mag_now = 0 as exactly that. The engine used to substitute x1.0, whose curve
 * target is the near stop, and then sample the 8 s window that only means something around a
 * target worth trusting: on the lab camera, sitting at x4.7 with a peak 24 s down the travel,
 * that searched the first 8 s of 38 s and landed 16 s short, every pass, for ever.
 *
 * So: the model's lens is at a REAL magnification the engine is not told (told_mag = 0), from
 * start positions all over the travel, for distant and offset scenes. Nothing about the peak is
 * discoverable except by looking, which is the point — the pass has to look everywhere.
 *
 * Deliberately NOT relaxed for the 90 s budget: a sweep that stops at the crest reaches the far
 * end only when there is no crest out there, so the full traversal is the no-signal case, not
 * the working one. If this ever needs a longer budget than the curve-driven paths, that is a
 * finding and not a number to raise. */
TEST cold_focus_without_a_magnification(void) {
    double mags[] = {1.6, 2.6, 3.4, 4.2, 5.0};   /* where the lens really is */
    double offsets[] = {0, 2000, -1500};
    double starts[] = {0, 12000, 26000, 38000};
    for (unsigned m = 0; m < sizeof(mags)/sizeof(mags[0]); m++)
    for (unsigned o = 0; o < 3; o++)
    for (unsigned st = 0; st < 4; st++) {
        Lens l; memset(&l, 0, sizeof l);
        l.travel = 38000; l.backlash = 400; l.offset = offsets[o];
        l.width = 1900; l.floor = 3; l.pos = starts[st];
        l.rng = 0x7a1 ^ (unsigned)(long)(mags[m] * 173 + offsets[o] + starts[st]);
        long fp = -1; int path;
        double f = run_pass_told(&l, mags[m], 0.0, &fp, &path);
        if (path != 2 || f < 0.80) {
            static char msg[192];
            snprintf(msg, sizeof msg,
                     "blind cold at real x%.1f off=%.0f from %.0f: path=%d land=%.0f%% "
                     "(searched a window it was never told to trust?)",
                     mags[m], offsets[o], starts[st], path, f * 100);
            FAILm(msg);
        }
    }
    PASS();
}

/* And the position it carries out must not outlive the magnification it lacks: a pass run with
 * no magnification has nothing to pair a focus position with, so handing one back as though a
 * later TRACK could use it re-introduces the same fiction one pass later. Feeding the carried
 * position straight back in, still with no magnification, must cold-seek again rather than
 * track from it. */
TEST a_blind_pass_does_not_become_a_track(void) {
    Lens l; memset(&l, 0, sizeof l);
    l.travel = 38000; l.backlash = 400; l.offset = 0;
    l.width = 1900; l.floor = 3; l.pos = 30000;
    l.rng = 0x5150;
    long fp = -1; int path;
    run_pass_told(&l, 3.4, 0.0, &fp, &path);
    GREATEST_ASSERTm("first blind pass should be the cold path", path == 2);
    double f = run_pass_told(&l, 3.4, 0.0, &fp, &path);
    GREATEST_ASSERTm("a carried position with no magnification must not TRACK", path == 2);
    GREATEST_ASSERTm("second blind pass lost the crest", f >= 0.80);
    PASS();
}

SUITE(af2_suite) {
    RUN_TEST(parfocal_curve_is_monotonic);
    RUN_TEST(tracks_a_zoom_itinerary);
    RUN_TEST(cold_focus_from_unknown);
    RUN_TEST(cold_focus_without_a_magnification);
    RUN_TEST(a_blind_pass_does_not_become_a_track);
    RUN_TEST(tracks_past_a_shoulder);
    RUN_TEST(lands_on_a_crest_barely_above_the_floor);
    RUN_TEST(return_recognises_a_shallow_crest);
    RUN_TEST(no_gradient_return_respects_the_budget);
}
