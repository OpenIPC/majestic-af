#include <majestic/af_blind_seek.h>

#include <stdbool.h>

#define BLIND_SEEK_LAND_PERCENT 98U

/*
 * af_blind_seek has no knowledge of lens position. It issues relative focus
 * movements and observes only the ISP focus metric. It does not use motor
 * feedback, position tracking, zoom magnification, or a calibrated
 * zoom-to-focus model.
 */

typedef struct {
    AfIO *io;
    AfBlindSeekParams *p;
    long deadline;
    long pos;
    unsigned peak;
    const char *phase;
    int direction;
    long pulse_ms;
} BlindSeek;

static long now(BlindSeek *s) { return s->io->now_ms(s->io->ctx); }

static bool cancelled(BlindSeek *s) {
    return now(s) >= s->deadline || (s->p->cancel && *s->p->cancel);
}

static void nap(BlindSeek *s, long ms) {
    while (ms > 0 && !cancelled(s)) {
        long slice = ms > 50 ? 50 : ms;
        s->io->sleep_ms(s->io->ctx, slice);
        ms -= slice;
    }
}

static unsigned sample(BlindSeek *s) {
    long started = now(s);
    unsigned values[9];
    int count = s->p->fv_samples;
    if (count < 1) count = 1;
    if (count > 9) count = 9;

    for (int i = 0; i < count; i++) {
        values[i] = s->io->fv(s->io->ctx);
        if (i + 1 < count) nap(s, s->p->fv_frame_ms);
    }
    for (int i = 1; i < count; i++) {
        unsigned value = values[i];
        int j = i - 1;
        while (j >= 0 && values[j] > value) {
            values[j + 1] = values[j];
            j--;
        }
        values[j + 1] = value;
    }

    unsigned value = values[count / 2];
    if (value > s->peak) s->peak = value;
    s->p->out_steps++;
    if (s->p->trace) s->p->trace(s->p->trace_ctx, s->pos, value);
    if (s->p->trace_sample)
        s->p->trace_sample(s->p->trace_ctx, s->phase, started, now(s),
                           s->pos, value, s->direction, s->pulse_ms);
    if (s->io->progress)
        s->io->progress(s->io->ctx, s->phase ? s->phase : "search", value, s->peak);
    return value;
}

static bool pulse(BlindSeek *s, int dir, long ms) {
    if (ms <= 0 || cancelled(s)) return false;
    s->direction = dir;
    s->pulse_ms = ms;
    /* Let the motor driver control pulse timing when this operation exists. */
    if (s->io->pulse_ms) {
        if (!s->io->pulse_ms(s->io->ctx, dir, ms)) return false;
    } else {
        s->io->drive(s->io->ctx, dir);
        nap(s, ms);
        s->io->drive(s->io->ctx, AF2_STOP);
    }
    if (cancelled(s)) return false;
    s->pos += dir * ms;
    nap(s, s->p->settle_ms);
    return !cancelled(s);
}

static bool move_to(BlindSeek *s, long target) {
    long delta = target - s->pos;
    if (!delta) return true;
    return pulse(s, delta > 0 ? AF2_FAR : AF2_NEAR,
                 delta > 0 ? delta : -delta);
}

static bool improved(unsigned value, unsigned baseline, unsigned percent) {
    return (unsigned long long)value * 100 >=
           (unsigned long long)baseline * (100 + percent);
}

static bool dropped(unsigned value, unsigned best, unsigned percent) {
    return (unsigned long long)value * 100 <=
           (unsigned long long)best * (100 - percent);
}

/* Keep correction pulses short near the recorded peak. A longer pulse can
 * cross the complete useful peak before the next metric becomes available. */
static long correction_ms(BlindSeek *s, unsigned value, unsigned peak) {
    if ((unsigned long long)value * 100 >= (unsigned long long)peak * 97)
        return s->p->micro_ms;
    return s->p->nudge_ms;
}

static void keep_best(unsigned value, long pos, unsigned *best, long *best_pos) {
    if (value > *best) {
        *best = value;
        *best_pos = pos;
    }
}

typedef enum {
    SWEEP_FLAT,
    SWEEP_WRONG_DIRECTION,
    SWEEP_PASSED_PEAK,
    SWEEP_LIMIT,
} SweepResult;

static void account_movement(BlindSeek *s, int dir, long *accounted_at) {
    long current = now(s);
    if (current > *accounted_at) s->pos += dir * (current - *accounted_at);
    *accounted_at = current;
}

/* Move continuously and use the focus metric as the only direction signal.
 * Require two falling samples so one noisy frame cannot reverse the lens. */
static SweepResult sweep(BlindSeek *s, int dir, unsigned baseline,
                         unsigned *best, long *best_pos, bool reject_wrong) {
    long started = now(s);
    long accounted_at = started;
    int falling = 0;
    int rising_samples = 0;
    unsigned previous = baseline;
    unsigned sweep_peak = 0;
    unsigned sweep_floor = baseline;
    bool peak_armed = false;
    bool recovering = !reject_wrong;
    s->direction = dir;
    s->pulse_ms = 0;

    s->phase = recovering
                   ? (dir == AF2_NEAR ? "recover-near" : "recover-far")
                   : (dir == AF2_NEAR ? "sweep-near" : "sweep-far");
    s->io->drive(s->io->ctx, dir);
    while (!cancelled(s)) {
        nap(s, s->p->live_sample_ms);
        account_movement(s, dir, &accounted_at);
        if (cancelled(s)) break;

        unsigned value = sample(s);
        account_movement(s, dir, &accounted_at);
        keep_best(value, s->pos, best, best_pos);
        if (value < sweep_floor) sweep_floor = value;

        if (recovering && !peak_armed) {
            /* The first direction reduced focus. Backlash can make the value
             * fall briefly after reversal. Wait for a sustained rise, then
             * arm detection against this sweep's peak, not the historic peak. */
            if (value > previous)
                rising_samples++;
            else if (dropped(value, previous, s->p->min_improve_percent))
                rising_samples = 0;
            if (rising_samples >= 2 &&
                improved(value, sweep_floor, s->p->min_improve_percent)) {
                peak_armed = true;
                sweep_peak = value;
                s->phase = dir == AF2_NEAR ? "track-near" : "track-far";
            }
        } else if (!recovering &&
                   improved(value, baseline, s->p->min_improve_percent)) {
            peak_armed = true;
        }
        if (peak_armed && value > sweep_peak) sweep_peak = value;

        unsigned fall_percent = recovering
                                    ? s->p->min_improve_percent
                                    : s->p->drop_percent;
        bool falling_now = peak_armed
                               ? dropped(value, sweep_peak, fall_percent)
                               : reject_wrong &&
                                     dropped(value, baseline,
                                             s->p->min_improve_percent);
        falling = falling_now ? falling + 1 : 0;
        previous = value;

        if (falling >= 2 && (peak_armed || reject_wrong)) {
            s->io->drive(s->io->ctx, AF2_STOP);
            return peak_armed ? SWEEP_PASSED_PEAK : SWEEP_WRONG_DIRECTION;
        }
        long elapsed = now(s) - started;
        if (!peak_armed && elapsed >= s->p->direction_ms) {
            s->io->drive(s->io->ctx, AF2_STOP);
            return SWEEP_FLAT;
        }
        if (elapsed >= s->p->sweep_ms) {
            s->io->drive(s->io->ctx, AF2_STOP);
            return SWEEP_LIMIT;
        }
    }

    account_movement(s, dir, &accounted_at);
    s->io->drive(s->io->ctx, AF2_STOP);
    return SWEEP_LIMIT;
}

unsigned af_blind_seek_run(AfIO *io, AfBlindSeekParams *p) {
    if (p->micro_ms <= 0) p->micro_ms = 40;
    if (p->nudge_ms <= 0) p->nudge_ms = 70;
    if (p->recovery_ms <= 0) p->recovery_ms = 160;
    if (p->settle_ms <= 0) p->settle_ms = 160;
    if (p->budget_ms <= 0) p->budget_ms = 30000;
    if (p->fv_samples <= 0) p->fv_samples = 5;
    if (p->fv_frame_ms <= 0) p->fv_frame_ms = 40;
    if (p->direction_ms <= 0) p->direction_ms = 5000;
    if (p->sweep_ms <= 0) p->sweep_ms = 12000;
    if (p->live_sample_ms <= 0) p->live_sample_ms = 40;
    if (p->min_improve_percent == 0) p->min_improve_percent = 2;
    if (p->drop_percent == 0) p->drop_percent = 12;

    BlindSeek s = {.io = io,
               .p = p,
               .deadline = io->now_ms(io->ctx) + p->budget_ms,
               .phase = "start"};

    p->out_steps = 0;
    p->out_converged = false;
    p->out_start_fv = sample(&s);
    unsigned best = p->out_start_fv;
    long best_pos = 0;

    /* No position model exists. Choose the first direction without pretending
     * to know it, then let the live focus metric prove or reject that choice. */
    int dir = ((now(&s) + p->out_start_fv) & 1) ? AF2_NEAR : AF2_FAR;
    SweepResult result = sweep(&s, dir, p->out_start_fv, &best, &best_pos, true);

    if (result == SWEEP_FLAT && !cancelled(&s)) {
        /* Five seconds without a 2 percent change gives no direction signal.
         * Return for the same time, then search the other side. */
        if (!pulse(&s, -dir, p->direction_ms)) goto finish;
    }
    if ((result == SWEEP_FLAT || result == SWEEP_WRONG_DIRECTION) &&
        !cancelled(&s)) {
        sweep(&s, -dir, p->out_start_fv, &best, &best_pos, false);
    }

finish:
    /* Return to the best dead-reckoned position. The position is relative to this pass because
     * a P035 focus-position report is not available. */
    int return_dir = best_pos > s.pos ? AF2_FAR : best_pos < s.pos ? AF2_NEAR : 0;
    if (!cancelled(&s)) move_to(&s, best_pos);

    /* Correct the settled landing with short pulses. Keep the pass target even
     * when a correction misses; replacing it would turn a miss into success. */
    long step_limit = p->nudge_ms < 70 ? p->nudge_ms : 70;
    long reversal_travel = 0;
    unsigned direction_peak = 0;
    unsigned direction_start = 0;
    int correction_dir = return_dir ? return_dir : -dir;
    int direction_changes = 0;
    bool confirmed = false;
    bool verified = false;
    s.phase = "return";
    unsigned value = cancelled(&s) ? 0 : sample(&s);
    direction_peak = direction_start = value;

    for (int i = 0; i <= 32 && !cancelled(&s); i++) {
        if (value > best) best = value;
        if ((unsigned long long)value * 100 >=
            (unsigned long long)best * BLIND_SEEK_LAND_PERCENT) {
            s.phase = "verify";
            nap(&s, p->settle_ms);
            unsigned check = sample(&s);
            if (check > best) best = check;
            if ((unsigned long long)check * 100 >=
                (unsigned long long)best * BLIND_SEEK_LAND_PERCENT) {
                value = check;
                verified = !cancelled(&s);
                break;
            }
            value = check;
        }
        if (i == 32) break;
        long correction = correction_ms(&s, value, best);
        if (correction > step_limit) correction = step_limit;
        s.phase = reversal_travel > 0 ? "clear-reversal" : "correct";
        if (!pulse(&s, correction_dir, correction)) break;
        value = sample(&s);
        if (value > direction_peak) direction_peak = value;
        if (improved(value, direction_start, p->min_improve_percent)) {
            confirmed = true;
            reversal_travel = 0;
        }
        if (reversal_travel > 0) {
            reversal_travel -= correction;
            continue;
        }
        if (dropped(value, direction_peak, p->min_improve_percent)) {
            if (direction_changes >= 2) break;
            correction_dir = -correction_dir;
            direction_changes++;
            /* A decline after a proven rise is an overshoot. Reduce the step,
             * but do not shrink merely because the initial direction was wrong. */
            if (confirmed) {
                step_limit /= 2;
                if (step_limit < 20) step_limit = 20;
            }
            confirmed = false;
            direction_peak = direction_start = value;
            reversal_travel = p->recovery_ms * 2;
        }
    }
    p->out_converged = verified;

    io->drive(io->ctx, AF2_STOP);
    p->out_focus_pos = s.pos;
    s.phase = "final";
    p->out_peak_fv = cancelled(&s) ? 0 : value;
    if (cancelled(&s)) p->out_converged = false;
    p->out_peak_seen = s.peak;
    return p->out_peak_fv;
}
