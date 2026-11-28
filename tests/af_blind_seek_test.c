// Host offline test for af_algorithm_parse() and af_blind_seek_run():
// verifies algorithm name parsing, synthetic single-peak landing, and
// multi-peak tournament selection where zone sharpness overrides a higher
// false-glare FV peak and writes the atomic JSON manifest.

#include <greatest.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <majestic/af_algorithm.h>
#include <majestic/af_blind_seek.h>

typedef struct {
    long pos;
    long time_ms;
    int dir;
    long peak1_pos;
    unsigned peak1_fv;
    unsigned peak1_sharp;
    long peak2_pos;
    unsigned peak2_fv;
    unsigned peak2_sharp;
} SimLens;

static void sim_advance(SimLens *l, long ms) {
    l->time_ms += ms;
    if (l->dir != 0) {
        l->pos += l->dir * ms;
    }
}

static unsigned sim_gauss(long d, unsigned amp, long sigma) {
    double x = (double)d / (double)sigma;
    return (unsigned)(amp * exp(-0.5 * x * x));
}

static void sim_drive(void *ctx, int dir) {
    ((SimLens *)ctx)->dir = dir;
}

static unsigned sim_fv(void *ctx) {
    SimLens *l = ctx;
    unsigned v = 200;
    v += sim_gauss(l->pos - l->peak1_pos, l->peak1_fv, 180);
    if (l->peak2_fv > 0) {
        v += sim_gauss(l->pos - l->peak2_pos, l->peak2_fv, 180);
    }
    return v;
}

static unsigned sim_sharp(void *ctx) {
    SimLens *l = ctx;
    unsigned s = 1000;
    s += sim_gauss(l->pos - l->peak1_pos, l->peak1_sharp, 160);
    if (l->peak2_sharp > 0) {
        s += sim_gauss(l->pos - l->peak2_pos, l->peak2_sharp, 160);
    }
    return s;
}

static long sim_now(void *ctx) {
    return ((SimLens *)ctx)->time_ms;
}

static void sim_sleep(void *ctx, long ms) {
    sim_advance((SimLens *)ctx, ms);
}

static long sim_pulse(void *ctx, int dir, long ms) {
    SimLens *l = ctx;
    l->pos += dir * ms;
    l->time_ms += ms;
    return ms;
}

TEST parses_algorithm_names(void) {
    ASSERT_EQ(AF_ALGORITHM_AF2, af_algorithm_parse("af2"));
    ASSERT_EQ(AF_ALGORITHM_BLIND_SEEK, af_algorithm_parse("blind_seek"));
    ASSERT_EQ(AF_ALGORITHM_BLIND_SEEK, af_algorithm_parse("blind-seek"));
    ASSERT_EQ(AF_ALGORITHM_INVALID, af_algorithm_parse("unknown"));
    ASSERT_EQ(AF_ALGORITHM_INVALID, af_algorithm_parse(NULL));
    PASS();
}

TEST finds_single_peak_continuously(void) {
    SimLens lens = {.pos = 0, .peak1_pos = 1100, .peak1_fv = 3000};
    AfBlindSeekIO io = {
        .base = {.drive = sim_drive, .fv = sim_fv, .now_ms = sim_now,
                 .sleep_ms = sim_sleep, .ctx = &lens},
    };
    AfBlindSeekParams p = {
        .micro_ms = 30, .nudge_ms = 60, .recovery_ms = 80, .settle_ms = 40,
        .budget_ms = 20000, .fv_samples = 3, .fv_frame_ms = 20,
        .direction_ms = 2500, .sweep_ms = 5000, .live_sample_ms = 40,
    };
    unsigned final = af_blind_seek_run(&io, &p);
    ASSERT(p.out_converged);
    ASSERT(final >= 2800);
    ASSERT(labs(lens.pos - lens.peak1_pos) < 250);
    PASS();
}

TEST tournament_picks_sharpest_peak_and_writes_manifest(void) {
    char manifest[128];
    snprintf(manifest, sizeof manifest, "/tmp/test_af_peaks_%d.json", (int)getpid());
    unlink(manifest);

    /* Peak 1 has higher raw FV (glare spike), Peak 2 has much higher true sharpness. */
    SimLens lens = {
        .pos = 0,
        .peak1_pos = 500, .peak1_fv = 4500, .peak1_sharp = 15000,
        .peak2_pos = 2000, .peak2_fv = 3800, .peak2_sharp = 38000,
    };
    AfBlindSeekIO io = {
        .base = {.drive = sim_drive, .fv = sim_fv, .now_ms = sim_now,
                 .sleep_ms = sim_sleep, .ctx = &lens},
        .pulse_ms = sim_pulse,
        .sharpness = sim_sharp,
    };
    AfBlindSeekParams p = {
        .micro_ms = 30, .nudge_ms = 60, .recovery_ms = 0, .settle_ms = 40,
        .budget_ms = 40000, .fv_samples = 3, .fv_frame_ms = 20,
        .direction_ms = 2500, .sweep_ms = 7000, .live_sample_ms = 40,
        .manifest_path = manifest,
    };
    (void)af_blind_seek_run(&io, &p);
    ASSERT(p.out_converged);
    ASSERT(labs(lens.pos - lens.peak2_pos) < 250);

    FILE *f = fopen(manifest, "r");
    ASSERT(f != NULL);
    char buf[1024] = "";
    size_t n = fread(buf, 1, sizeof buf - 1, f);
    fclose(f);
    buf[n] = '\0';
    unlink(manifest);

    ASSERT(strstr(buf, "\"winner\":") != NULL);
    ASSERT(strstr(buf, "\"sharp\":") != NULL);
    ASSERT(strstr(buf, "\"metric\":\"fv\"") != NULL);
    ASSERT(strstr(buf, "focus_origin_ms") == NULL);
    PASS();
}

SUITE(af_blind_seek_suite) {
    RUN_TEST(parses_algorithm_names);
    RUN_TEST(finds_single_peak_continuously);
    RUN_TEST(tournament_picks_sharpest_peak_and_writes_manifest);
}

GREATEST_MAIN_DEFS();

int main(int argc, char **argv) {
    GREATEST_MAIN_BEGIN();
    RUN_SUITE(af_blind_seek_suite);
    GREATEST_MAIN_END();
}
