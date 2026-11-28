#ifndef AF_BLIND_SEEK_H
#define AF_BLIND_SEEK_H

#include <stdbool.h>
#include <majestic/af2.h>

typedef struct {
    AfIO base;
    /* Optional hardware-timed focus pulse (ms); returns ms driven or <0 if unavailable. */
    long (*pulse_ms)(void *ctx, int dir, long ms);
    /* Optional settled zone-grid sharpness scorer (in-process, e.g. sdk_get_focus_zones). */
    unsigned (*sharpness)(void *ctx);
    /* Optional gallery snapshot hook; returns saved byte count (0 if disabled/failed). */
    unsigned (*save_snapshot)(void *ctx, int candidate_id);
    /* Optional progress reporter invoked on each sample. */
    void (*progress)(void *ctx, const char *phase, unsigned fv, unsigned peak);
} AfBlindSeekIO;

typedef struct {
    long micro_ms;
    long nudge_ms;
    long recovery_ms;
    long settle_ms;
    long budget_ms;
    int fv_samples;
    long fv_frame_ms;
    long direction_ms;
    long sweep_ms;
    long live_sample_ms;
    unsigned min_improve_percent;
    unsigned drop_percent;
    bool fast_mode;
    const char *metric_name;     /* "zones" or "fv" for manifest labeling (NULL -> "fv") */
    const char *manifest_path;   /* optional JSON manifest output (NULL disables) */
    void (*trace)(void *ctx, long pos, unsigned fv);
    void *trace_ctx;
    void (*trace_sample)(void *ctx, const char *phase, long started_ms,
                         long ended_ms, long pos, unsigned fv,
                         int direction, long pulse_ms);
    const volatile int *cancel;

    unsigned out_start_fv;
    bool out_converged;
    unsigned out_peak_seen;
    unsigned out_peak_fv;
    int out_steps;
    long out_focus_pos;
} AfBlindSeekParams;

/*
 * Bounded bidirectional focus search with optional multi-peak tournament
 * verification for continuous or timed-pulse lenses without position feedback.
 */
unsigned af_blind_seek_run(AfBlindSeekIO *io, AfBlindSeekParams *p);

#endif
