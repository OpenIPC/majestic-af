/*
 * ============================================================================
 * Bounded Blind-Seek Autofocus with Multi-Peak Candidate Tournament
 * ============================================================================
 *
 * Designed for continuous or timed-pulse lenses without absolute focus
 * position encoders:
 *
 * Stage 1: Bidirectional Sweep & Candidate Clustering (record_candidate)
 *   Sweeps through the search window recording distinct local focus maxima
 *   separated by at least 400 ms of focus travel.
 *
 * Stage 2: Settled Tournament Verification (when timed pulses are available)
 *   Revisits each candidate peak at stationary rest (after settle_ms), measures
 *   both settled ISP FV and optional in-process sharpness (zone-grid edge
 *   metric from sdk_get_focus_zones), and picks the sharpest peak immune to
 *   motion blur and night highlight halos.
 *
 * Stage 3: Final Park, Micro-Correction & Optional Manifest
 *   Parks at the winning position, performs bounded fine micro-nudges if
 *   needed, and atomically writes p->manifest_path (if set) for inspection UIs.
 */

#include <majestic/af_blind_seek.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define BLIND_SEEK_LAND_PERCENT 98U
#define BLIND_SEEK_MAX_CANDIDATES 6
#define BLIND_SEEK_CAND_MIN_GAP_MS 400L

typedef struct {
    long pos;
    unsigned fv;
} BlindCandidate;

typedef struct {
    AfBlindSeekIO *io;
    AfBlindSeekParams *p;
    long deadline;
    long pos;
    int direction;
    long pulse_ms;
    unsigned peak;
    const char *phase;
    bool returning;
    BlindCandidate candidates[BLIND_SEEK_MAX_CANDIDATES];
    int candidate_count;
} BlindSeek;

static long now(const BlindSeek *s) {
    return s->io->base.now_ms(s->io->base.ctx);
}

static bool cancelled(const BlindSeek *s) {
    if (s->p->cancel && *s->p->cancel) return true;
    return !s->returning && now(s) >= s->deadline;
}

static void nap(BlindSeek *s, long ms) {
    while (ms > 0 && !cancelled(s)) {
        long slice = ms > 50 ? 50 : ms;
        s->io->base.sleep_ms(s->io->base.ctx, slice);
        ms -= slice;
    }
}

static unsigned sample_median(BlindSeek *s, unsigned (*fn)(void *)) {
    unsigned values[9];
    int count = s->p->fv_samples;
    if (count < 1) count = 1;
    if (count > 9) count = 9;

    for (int i = 0; i < count; i++) {
        values[i] = fn(s->io->base.ctx);
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
    return values[count / 2];
}

static unsigned sample(BlindSeek *s) {
    long started = now(s);
    unsigned value = sample_median(s, s->io->base.fv);
    if (value > s->peak) s->peak = value;
    s->p->out_steps++;
    if (s->p->trace) s->p->trace(s->p->trace_ctx, s->pos, value);
    if (s->p->trace_sample) {
        s->p->trace_sample(s->p->trace_ctx, s->phase, started, now(s),
                           s->pos, value, s->direction, s->pulse_ms);
    }
    if (s->io->progress) {
        s->io->progress(s->io->base.ctx, s->phase ? s->phase : "search", value, s->peak);
    }
    return value;
}

static bool pulse(BlindSeek *s, int dir, long ms) {
    if (ms <= 0 || cancelled(s)) return false;
    long backlash = (s->io->pulse_ms && dir != s->direction && s->direction != 0)
                        ? s->p->recovery_ms
                        : 0;
    long drive_ms = ms + backlash;
    s->direction = dir;
    s->pulse_ms = drive_ms;
    if (s->io->pulse_ms) {
        if (s->io->pulse_ms(s->io->base.ctx, dir, drive_ms) < 0) return false;
    } else {
        s->io->base.drive(s->io->base.ctx, dir);
        nap(s, drive_ms);
        s->io->base.drive(s->io->base.ctx, AF2_STOP);
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

static long correction_ms(const BlindSeek *s, unsigned value, unsigned peak) {
    if ((unsigned long long)value * 100 >= (unsigned long long)peak * 97)
        return s->p->micro_ms;
    return s->p->nudge_ms;
}

static void record_candidate(BlindSeek *s, unsigned value, long pos) {
    if (!s || !s->io->pulse_ms || value == 0) return;
    int replace_idx = -1;
    for (int i = 0; i < s->candidate_count; i++) {
        long dist = pos - s->candidates[i].pos;
        if (dist < 0) dist = -dist;
        if (dist < BLIND_SEEK_CAND_MIN_GAP_MS) {
            replace_idx = (value > s->candidates[i].fv) ? i : -2;
            break;
        }
    }
    if (replace_idx >= 0) {
        s->candidates[replace_idx].pos = pos;
        s->candidates[replace_idx].fv = value;
    } else if (replace_idx == -1) {
        if (s->candidate_count < BLIND_SEEK_MAX_CANDIDATES) {
            s->candidates[s->candidate_count].pos = pos;
            s->candidates[s->candidate_count].fv = value;
            s->candidate_count++;
        } else {
            int min_idx = 0;
            for (int i = 1; i < BLIND_SEEK_MAX_CANDIDATES; i++) {
                if (s->candidates[i].fv < s->candidates[min_idx].fv) min_idx = i;
            }
            if (value > s->candidates[min_idx].fv) {
                s->candidates[min_idx].pos = pos;
                s->candidates[min_idx].fv = value;
            }
        }
    }
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

static SweepResult sweep(BlindSeek *s, int dir, unsigned baseline,
                         unsigned *best, long *best_pos, bool reject_wrong) {
    long started = now(s);
    long accounted_at = started;
    int falling = 0;
    int rising_samples = 0;
    unsigned previous = baseline;
    unsigned sweep_peak = 0;
    unsigned sweep_floor = baseline;
    long sweep_peak_time = 0;
    bool peak_armed = false;
    bool recovering = !reject_wrong;
    bool crest_armed = false;
    unsigned crest_fv = 0;
    long crest_pos = 0;
    s->direction = dir;
    s->pulse_ms = 0;

    s->phase = recovering
                   ? (dir == AF2_NEAR ? "recover-near" : "recover-far")
                   : (dir == AF2_NEAR ? "sweep-near" : "sweep-far");
    s->io->base.drive(s->io->base.ctx, dir);
    while (!cancelled(s)) {
        nap(s, s->p->live_sample_ms);
        account_movement(s, dir, &accounted_at);
        if (cancelled(s)) break;

        long sample_start_pos = s->pos;
        unsigned value = sample(s);
        account_movement(s, dir, &accounted_at);
        long sample_mid_pos = (sample_start_pos + s->pos) / 2;
        long cand_pos = s->io->pulse_ms ? sample_mid_pos : s->pos;
        keep_best(value, cand_pos, best, best_pos);

        if (value > previous && (!crest_armed || value >= crest_fv)) {
            crest_fv = value;
            crest_pos = cand_pos;
            crest_armed = true;
        } else if (crest_armed && value >= crest_fv) {
            crest_fv = value;
            crest_pos = cand_pos;
        } else if (crest_armed && dropped(value, crest_fv, s->p->min_improve_percent)) {
            record_candidate(s, crest_fv, crest_pos);
            crest_armed = false;
            crest_fv = value;
            crest_pos = cand_pos;
        }
        if (value < sweep_floor) sweep_floor = value;

        if (value > previous)
            rising_samples++;
        else if (dropped(value, previous, s->p->min_improve_percent))
            rising_samples = 0;

        unsigned arm_pct = s->p->min_improve_percent;
        if (recovering && !peak_armed) {
            if ((rising_samples >= 2 && improved(value, sweep_floor, arm_pct)) ||
                (s->io->pulse_ms && value > previous &&
                 improved(value, sweep_floor, s->p->drop_percent))) {
                peak_armed = true;
                sweep_peak = value;
                sweep_peak_time = now(s);
                s->phase = dir == AF2_NEAR ? "track-near" : "track-far";
            }
        } else if (!recovering && !peak_armed) {
            if ((rising_samples >= 2 && improved(value, baseline, arm_pct)) ||
                (s->io->pulse_ms && value > previous &&
                 improved(value, baseline, s->p->drop_percent))) {
                peak_armed = true;
                sweep_peak = value;
                sweep_peak_time = now(s);
            }
        }
        if (peak_armed && value > sweep_peak) {
            sweep_peak = value;
            sweep_peak_time = now(s);
        }

        unsigned fall_percent = s->p->drop_percent;
        unsigned wrong_percent = s->io->pulse_ms ? s->p->drop_percent : s->p->min_improve_percent;
        bool falling_now = peak_armed
                               ? dropped(value, sweep_peak, fall_percent)
                               : reject_wrong && dropped(value, baseline, wrong_percent);
        falling = falling_now ? falling + 1 : 0;
        previous = value;

        if (falling >= 2 && (peak_armed || reject_wrong)) {
            if (!(peak_armed && s->io->pulse_ms != NULL)) {
                if (crest_armed) record_candidate(s, crest_fv, crest_pos);
                s->io->base.drive(s->io->base.ctx, AF2_STOP);
                return peak_armed ? SWEEP_PASSED_PEAK : SWEEP_WRONG_DIRECTION;
            }
        }
        long elapsed = now(s) - started;
        if (!s->io->pulse_ms && peak_armed && sweep_peak_time > 0 &&
            (now(s) - sweep_peak_time >= 1200) &&
            dropped(value, sweep_peak, s->p->min_improve_percent)) {
            if (crest_armed) record_candidate(s, crest_fv, crest_pos);
            s->io->base.drive(s->io->base.ctx, AF2_STOP);
            return SWEEP_PASSED_PEAK;
        }
        if (reject_wrong && !peak_armed && elapsed >= s->p->direction_ms) {
            if (crest_armed) record_candidate(s, crest_fv, crest_pos);
            s->io->base.drive(s->io->base.ctx, AF2_STOP);
            return SWEEP_FLAT;
        }
        if (elapsed >= s->p->sweep_ms) {
            if (crest_armed) record_candidate(s, crest_fv, crest_pos);
            s->io->base.drive(s->io->base.ctx, AF2_STOP);
            return SWEEP_LIMIT;
        }
    }

    if (crest_armed) record_candidate(s, crest_fv, crest_pos);
    account_movement(s, dir, &accounted_at);
    s->io->base.drive(s->io->base.ctx, AF2_STOP);
    return SWEEP_LIMIT;
}

static void write_manifest(const BlindSeek *s, int peak_count, int winner,
                           const unsigned cand_fv[BLIND_SEEK_MAX_CANDIDATES],
                           const unsigned cand_bytes[BLIND_SEEK_MAX_CANDIDATES],
                           const unsigned cand_sharp[BLIND_SEEK_MAX_CANDIDATES]) {
    if (!s->p->manifest_path || !*s->p->manifest_path || peak_count <= 1) {
        return;
    }
    char tmp_path[256];
    snprintf(tmp_path, sizeof tmp_path, "%s.tmp", s->p->manifest_path);
    FILE *mf = fopen(tmp_path, "w");
    if (!mf) return;

    const char *metric = (s->p->metric_name && *s->p->metric_name) ? s->p->metric_name : "fv";
    fprintf(mf,
            "{\"count\":%d,\"current_pos\":%ld,\"last_dir\":%d,"
            "\"recovery_ms\":%ld,\"winner\":%d,\"metric\":\"%s\",\"candidates\":[",
            peak_count, s->pos, s->direction, s->p->recovery_ms, winner, metric);
    for (int c = 0; c < peak_count; c++) {
        fprintf(mf, "%s{\"id\":%d,\"pos\":%ld,\"fv\":%u,\"bytes\":%u,\"sharp\":%u}",
                c > 0 ? "," : "", c, s->candidates[c].pos,
                cand_fv[c] ? cand_fv[c] : s->candidates[c].fv,
                cand_bytes[c], cand_sharp[c]);
    }
    fprintf(mf, "]}\n");
    fclose(mf);
    chmod(tmp_path, 0644);
    rename(tmp_path, s->p->manifest_path);
}

unsigned af_blind_seek_run(AfBlindSeekIO *io, AfBlindSeekParams *p) {
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
                   .deadline = io->base.now_ms(io->base.ctx) + p->budget_ms,
                   .phase = "start"};

    if (s.io->pulse_ms && p->manifest_path && *p->manifest_path) {
        unlink(p->manifest_path);
    }

    p->out_steps = 0;
    p->out_converged = false;
    p->out_start_fv = sample(&s);
    unsigned best = p->out_start_fv;
    long best_pos = 0;
    unsigned cand_bytes[BLIND_SEEK_MAX_CANDIDATES] = {0};
    unsigned cand_sharp[BLIND_SEEK_MAX_CANDIDATES] = {0};
    unsigned cand_fv[BLIND_SEEK_MAX_CANDIDATES] = {0};
    int peak_saved_count = 0;
    int verified_best_id = 0;
    bool all_sharp_ok = false;

    int dir = ((now(&s) + p->out_start_fv) & 1) ? AF2_NEAR : AF2_FAR;
    SweepResult result = sweep(&s, dir, p->out_start_fv, &best, &best_pos, true);

    if (result == SWEEP_FLAT && !cancelled(&s)) {
        if (!pulse(&s, -dir, p->direction_ms)) goto finish;
    }
    bool need_reverse = (result == SWEEP_FLAT ||
                         result == SWEEP_WRONG_DIRECTION ||
                         result == SWEEP_LIMIT ||
                         (s.io->pulse_ms != NULL && !p->fast_mode) ||
                         (result == SWEEP_PASSED_PEAK &&
                          !improved(best, p->out_start_fv, p->drop_percent)));
    if (need_reverse && !cancelled(&s)) {
        sweep(&s, -dir, p->out_start_fv, &best, &best_pos, false);
    }

finish:
    if (s.io->pulse_ms && !p->fast_mode && s.candidate_count > 1 && !cancelled(&s)) {
        s.phase = "select-best";
        all_sharp_ok = (s.io->sharpness != NULL);
        for (int c = 0; c < s.candidate_count && !cancelled(&s); c++) {
            if (!move_to(&s, s.candidates[c].pos)) break;
            nap(&s, p->settle_ms);
            if (cancelled(&s)) break;
            cand_fv[c] = sample(&s);
            if (s.io->sharpness) {
                cand_sharp[c] = sample_median(&s, s.io->sharpness);
                if (cand_sharp[c] == 0) all_sharp_ok = false;
            }
            if (s.io->save_snapshot) {
                cand_bytes[c] = s.io->save_snapshot(s.io->base.ctx, c);
            }
            peak_saved_count = c + 1;
        }
        if (peak_saved_count > 0 && !cancelled(&s)) {
            unsigned verified_best_score = 0;
            long verified_best_pos = best_pos;
            for (int c = 0; c < peak_saved_count; c++) {
                unsigned score = all_sharp_ok ? cand_sharp[c] : cand_fv[c];
                if (score > verified_best_score) {
                    verified_best_score = score;
                    verified_best_pos = s.candidates[c].pos;
                    verified_best_id = c;
                }
            }
            best_pos = verified_best_pos;
            if (cand_fv[verified_best_id] > 0) {
                if (all_sharp_ok || cand_fv[verified_best_id] > best) {
                    best = cand_fv[verified_best_id];
                }
            }
        }
    }

    s.returning = true;
    int return_dir = best_pos > s.pos ? AF2_FAR : best_pos < s.pos ? AF2_NEAR : 0;
    bool returned = !cancelled(&s) && move_to(&s, best_pos);

    bool verified = false;
    s.phase = "return";
    unsigned value = 0;

    if (s.io->pulse_ms) {
        nap(&s, p->settle_ms);
        value = cancelled(&s) ? 0 : sample(&s);
        unsigned target_fv = (peak_saved_count > 0 && cand_fv[verified_best_id] > 0)
                                 ? cand_fv[verified_best_id]
                                 : best;
        if (returned && !cancelled(&s) &&
            (target_fv == 0 || (unsigned long long)value * 100 >= (unsigned long long)target_fv * 92U)) {
            verified = true;
            goto done;
        }
    }

    long step_limit = p->nudge_ms < 70 ? p->nudge_ms : 70;
    long reversal_travel = 0;
    unsigned direction_peak = 0;
    unsigned direction_start = 0;
    int correction_dir = return_dir ? return_dir : -dir;
    int direction_changes = 0;
    int fine_falling = 0;
    bool confirmed = false;
    value = cancelled(&s) ? 0 : sample(&s);
    direction_peak = direction_start = value;

    unsigned land_pct = s.io->pulse_ms ? 97U : BLIND_SEEK_LAND_PERCENT;

    for (int i = 0; i <= 32 && !cancelled(&s); i++) {
        if (value > best) {
            best = value;
            best_pos = s.pos;
        }
        if ((unsigned long long)value * 100 >= (unsigned long long)best * land_pct) {
            s.phase = "verify";
            nap(&s, p->settle_ms);
            unsigned check = sample(&s);
            if (check > best) {
                best = check;
                best_pos = s.pos;
            }
            if ((unsigned long long)check * 100 >= (unsigned long long)best * land_pct) {
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
        if (value > direction_peak) {
            direction_peak = value;
            fine_falling = 0;
        } else if (dropped(value, direction_peak, p->min_improve_percent)) {
            fine_falling++;
        }
        if (improved(value, direction_start, p->min_improve_percent)) {
            confirmed = true;
            reversal_travel = 0;
        }
        if (reversal_travel > 0) {
            reversal_travel -= correction;
            continue;
        }
        if (fine_falling >= 2) {
            fine_falling = 0;
            if (direction_changes >= 2) break;
            correction_dir = -correction_dir;
            direction_changes++;
            if (confirmed) {
                step_limit /= 2;
                if (step_limit < 20) step_limit = 20;
            }
            confirmed = false;
            direction_peak = direction_start = value;
            reversal_travel = s.io->pulse_ms ? 0 : p->recovery_ms * 2;
        }
    }
    if (!verified && !cancelled(&s) && s.pos != best_pos) {
        if (move_to(&s, best_pos)) {
            value = sample(&s);
            if (value > best) best = value;
            if ((unsigned long long)value * 100 >= (unsigned long long)best * land_pct)
                verified = !cancelled(&s);
        }
    }

done:
    p->out_converged = verified;
    s.io->base.drive(s.io->base.ctx, AF2_STOP);
    p->out_focus_pos = s.pos;
    if (s.io->pulse_ms && !cancelled(&s) && peak_saved_count > 1) {
        write_manifest(&s, peak_saved_count, verified_best_id, cand_fv, cand_bytes, cand_sharp);
    }
    s.phase = "final";
    p->out_peak_fv = cancelled(&s) ? 0 : value;
    if (cancelled(&s)) p->out_converged = false;
    p->out_peak_seen = s.peak;
    return p->out_peak_fv;
}
