#ifndef AF_BLIND_SEEK_H
#define AF_BLIND_SEEK_H

#include <majestic/af2.h>

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
 * Bounded search for controllers that give no focus position or movement
 * timing feedback. The relative position is valid only during one search.
 */
unsigned af_blind_seek_run(AfIO *io, AfBlindSeekParams *p);

#endif
