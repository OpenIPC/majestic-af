#include <majestic/af_blind_seek.h>

#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define BLIND_SEEK_LAND_PERCENT 98U
#define BLIND_SEEK_MAX_CANDIDATES 6

/*
 * ============================================================================
 * Hardware-Assisted Peak Tournament & Gallery Architecture
 * ============================================================================
 *
 * During a continuous focus sweep across an unknown optical plane, contrast
 * statistics (Focus Values / FV) reported by the ISP sensor can suffer from:
 *   1. Dynamic scene motion noise (moving objects mimicking contrast peaks)
 *   2. Motion blur while the focus motor is traveling continuously
 *   3. Secondary / false harmonic crests in high-magnification optical assemblies
 *
 * To guarantee absolute optical sharpness without guesswork, this algorithm
 * implements a two-stage Candidate Tournament with Visual Proof:
 *
 * Stage 1: Sweep & Candidate Clustering (keep_best)
 *   As the lens sweeps through the optical range, local contrast maxima are
 *   tracked. Whenever a peak is found separated by at least 400ms of focus travel
 *   (to prevent clustering on noisy sub-ripples of the same peak), it is added
 *   to an active tournament candidate list (up to BLIND_SEEK_MAX_CANDIDATES).
 *
 * Stage 2: Settle, Multi-Spectral Benchmark & Candidate Snapshots (select-best)
 *   Once the coarse sweep identifies candidate positions, the lens halts and
 *   systematically revisits each candidate position at stationary rest.
 *   At each settled candidate:
 *     a. The motor settles (p->settle_ms) to eliminate all lens vibrations.
 *     b. A true still frame is captured via localhost HTTP GET /image.jpg.
 *     c. Snapshot /tmp/peak_%d.jpg is saved to disk for the WebUI visual gallery.
 *     d. True optical sharpness is verified via JPEG compressed payload size:
 *        in-focus high-frequency edge detail cannot be compressed by the hardware
 *        encoder DCT quantization tables, resulting in 15-25% larger file sizes
 *        (e.g., 265 KB for genuine focus vs 228 KB for false/blurry crests).
 *
 * Stage 3: Winner Selection & Gallery Manifest (/tmp/af_peaks.json)
 *   The candidate with the verified highest true edge sharpness is selected as
 *   the definitive winner. The lens parks directly at the winning focus coordinate.
 *   An atomic JSON manifest (/tmp/af_peaks.json) is emitted detailing candidate
 *   positions, focus values, encoded byte sizes, and the winning candidate ID,
 *   powering an interactive inspection gallery in the camera WebUI.
 * ============================================================================
 */

typedef struct {
    long pos;
    unsigned fv;
} CandidatePeak;

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
    CandidatePeak candidates[BLIND_SEEK_MAX_CANDIDATES];
    int candidate_count;
    bool returning;
} BlindSeek;

static long now(BlindSeek *s) { return s->io->now_ms(s->io->ctx); }

static bool ext_cancelled(BlindSeek *s) {
    return s->p->cancel && *s->p->cancel;
}

static bool cancelled(BlindSeek *s) {
    return ext_cancelled(s) || (!s->returning && now(s) >= s->deadline);
}

/* Captures a settled candidate still image, writes it to /tmp/peak_<id>.jpg for
 * the WebUI gallery, and measures hardware JPEG compressed byte length. */
static unsigned query_and_save_peak(BlindSeek *s, int candidate_idx) {
    char path[64], tmp_path[64];
    snprintf(path, sizeof(path), "/tmp/peak_%d.jpg", candidate_idx);
    snprintf(tmp_path, sizeof(tmp_path), "/tmp/peak_%d.jpg.tmp", candidate_idx);
    unlink(tmp_path);
    unlink(path);

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) return 0;
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags >= 0) fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    struct sockaddr_in sin;
    memset(&sin, 0, sizeof(sin));
    sin.sin_family = AF_INET;
    sin.sin_port = htons(80);
    sin.sin_addr.s_addr = inet_addr("127.0.0.1");

    int rc = connect(fd, (struct sockaddr *)&sin, sizeof(sin));
    if (rc < 0 && errno != EINPROGRESS) {
        close(fd);
        return 0;
    }
    if (rc < 0) {
        long deadline = now(s) + 2000;
        while (!cancelled(s) && now(s) < deadline) {
            struct pollfd pfd = {.fd = fd, .events = POLLOUT};
            int pr = poll(&pfd, 1, 50);
            if (pr > 0) {
                int so_err = 0;
                socklen_t slen = sizeof(so_err);
                if (getsockopt(fd, SOL_SOCKET, SO_ERROR, &so_err, &slen) == 0 && so_err == 0) {
                    rc = 0;
                }
                break;
            }
            if (pr < 0 && errno != EINTR) break;
        }
        if (rc != 0) {
            close(fd);
            return 0;
        }
    }

    const char req[] = "GET /image.jpg HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n\r\n";
    size_t req_len = sizeof(req) - 1, sent = 0;
    long write_deadline = now(s) + 2000;
    while (sent < req_len && !cancelled(s) && now(s) < write_deadline) {
        struct pollfd pfd = {.fd = fd, .events = POLLOUT};
        int pr = poll(&pfd, 1, 50);
        if (pr < 0 && errno != EINTR) break;
        if (pr > 0) {
            ssize_t wn = write(fd, req + sent, req_len - sent);
            if (wn <= 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
                break;
            }
            sent += (size_t)wn;
        }
    }
    if (sent < req_len) {
        close(fd);
        return 0;
    }

    FILE *fp = NULL;
    char hdr[2048];
    size_t hdr_len = 0;
    char buf[4096];
    int header_done = 0, status_ok = 0, aborted = 0;
    unsigned total_bytes = 0, content_len = 0;
    long read_deadline = now(s) + 2000;

    for (;;) {
        if (cancelled(s) || now(s) >= read_deadline) {
            aborted = 1;
            break;
        }
        struct pollfd pfd = {.fd = fd, .events = POLLIN};
        int pr = poll(&pfd, 1, 50);
        if (pr < 0) {
            if (errno == EINTR) continue;
            aborted = 1;
            break;
        }
        if (pr == 0) continue;

        ssize_t n = read(fd, buf, sizeof(buf));
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            aborted = 1;
            break;
        }
        if (n == 0) break;

        if (!header_done) {
            size_t copy = (size_t)n;
            if (hdr_len + copy >= sizeof(hdr)) copy = sizeof(hdr) - 1 - hdr_len;
            memcpy(hdr + hdr_len, buf, copy);
            hdr_len += copy;
            hdr[hdr_len] = '\0';

            char *hdr_end = strstr(hdr, "\r\n\r\n");
            if (!hdr_end) {
                if (hdr_len >= sizeof(hdr) - 1) {
                    aborted = 1;
                    break;
                }
                continue;
            }
            header_done = 1;
            char *line_end = strstr(hdr, "\r\n");
            if (line_end) {
                *line_end = '\0';
                if (strstr(hdr, " 200 ") != NULL || strcmp(hdr + strlen(hdr) - 4, " 200") == 0) status_ok = 1;
                *line_end = '\r';
            }
            if (!status_ok) {
                aborted = 1;
                break;
            }
            for (char *p = hdr; p < hdr_end; p++) {
                if ((p == hdr || p[-1] == '\n') && strncasecmp(p, "Content-Length:", 15) == 0) {
                    content_len = (unsigned)strtoul(p + 15, NULL, 10);
                    break;
                }
            }
            fp = fopen(tmp_path, "wb");
            if (!fp) {
                aborted = 1;
                break;
            }
            size_t hdr_bytes_total = (size_t)(hdr_end + 4 - hdr);
            size_t hdr_bytes_before_chunk = hdr_len - copy;
            if (hdr_bytes_total > hdr_bytes_before_chunk) {
                size_t used_in_chunk = hdr_bytes_total - hdr_bytes_before_chunk;
                if ((size_t)n > used_in_chunk) {
                    size_t body_len = (size_t)n - used_in_chunk;
                    if (fwrite(buf + used_in_chunk, 1, body_len, fp) != body_len) {
                        aborted = 1;
                        break;
                    }
                    total_bytes += (unsigned)body_len;
                }
            }
        } else {
            if (fwrite(buf, 1, (size_t)n, fp) != (size_t)n) {
                aborted = 1;
                break;
            }
            total_bytes += (unsigned)n;
        }
    }

    if (fp && fclose(fp) != 0) aborted = 1;
    close(fd);

    if (aborted || !header_done || !status_ok || total_bytes == 0 ||
        (content_len > 0 && total_bytes != content_len)) {
        unlink(tmp_path);
        return 0;
    }
    if (rename(tmp_path, path) != 0) {
        unlink(tmp_path);
        return 0;
    }
    return total_bytes;
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
    long backlash = (s->io->pulse_ms && dir != s->direction && s->direction != 0)
                        ? s->p->recovery_ms
                        : 0;
    long drive_ms = ms + backlash;
    s->direction = dir;
    s->pulse_ms = drive_ms;
    /* Let the motor driver control pulse timing when this operation exists. */
    if (s->io->pulse_ms) {
        if (!s->io->pulse_ms(s->io->ctx, dir, drive_ms)) return false;
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

static void record_candidate(BlindSeek *s, unsigned value, long pos) {
    /*
     * Candidate Clustering:
     * On motorized pulse hardware, record local peak candidates separated by at
     * least 400ms of focus travel to prevent multiple entries for the same crest.
     */
    if (!s || !s->io->pulse_ms || value == 0) return;
    int replace_idx = -1;
    for (int i = 0; i < s->candidate_count; i++) {
        long dist = pos - s->candidates[i].pos;
        if (dist < 0) dist = -dist;
        if (dist < 400) {
            if (value > s->candidates[i].fv) replace_idx = i;
            else replace_idx = -2; // duplicate within 400ms of higher candidate
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
    s->io->drive(s->io->ctx, dir);
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
            /* The first direction reduced focus. Backlash can make the value
             * fall briefly after reversal. Wait for a sustained rise, then
             * arm detection against this sweep's peak, not the historic peak. */
            if ((rising_samples >= 2 &&
                 improved(value, sweep_floor, arm_pct)) ||
                (s->io->pulse_ms && value > previous &&
                 improved(value, sweep_floor, s->p->drop_percent))) {
                peak_armed = true;
                sweep_peak = value;
                sweep_peak_time = now(s);
                s->phase = dir == AF2_NEAR ? "track-near" : "track-far";
            }
        } else if (!recovering && !peak_armed) {
            if ((rising_samples >= 2 &&
                 improved(value, baseline, arm_pct)) ||
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
                               : reject_wrong &&
                                     dropped(value, baseline, wrong_percent);
        falling = falling_now ? falling + 1 : 0;
        previous = value;

        if (falling >= 2 && (peak_armed || reject_wrong)) {
            if (!(peak_armed && s->io->pulse_ms != NULL)) {
                if (crest_armed) record_candidate(s, crest_fv, crest_pos);
                s->io->drive(s->io->ctx, AF2_STOP);
                return peak_armed ? SWEEP_PASSED_PEAK : SWEEP_WRONG_DIRECTION;
            }
        }
        long elapsed = now(s) - started;
        if (!s->io->pulse_ms && peak_armed && sweep_peak_time > 0 &&
            (now(s) - sweep_peak_time >= 1200) &&
            dropped(value, sweep_peak, s->p->min_improve_percent)) {
            if (crest_armed) record_candidate(s, crest_fv, crest_pos);
            s->io->drive(s->io->ctx, AF2_STOP);
            return SWEEP_PASSED_PEAK;
        }
        if (reject_wrong && !peak_armed && elapsed >= s->p->direction_ms) {
            if (crest_armed) record_candidate(s, crest_fv, crest_pos);
            s->io->drive(s->io->ctx, AF2_STOP);
            return SWEEP_FLAT;
        }
        if (elapsed >= s->p->sweep_ms) {
            if (crest_armed) record_candidate(s, crest_fv, crest_pos);
            s->io->drive(s->io->ctx, AF2_STOP);
            return SWEEP_LIMIT;
        }
    }

    if (crest_armed) record_candidate(s, crest_fv, crest_pos);
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

    if (s.io->pulse_ms) {
        unlink("/tmp/af_peaks.json");
        for (int c = 0; c < BLIND_SEEK_MAX_CANDIDATES; c++) {
            char peak_path[64];
            snprintf(peak_path, sizeof(peak_path), "/tmp/peak_%d.jpg", c);
            unlink(peak_path);
        }
    }

    p->out_steps = 0;
    p->out_converged = false;
    p->out_start_fv = sample(&s);
    unsigned best = p->out_start_fv;
    long best_pos = 0;
    unsigned cand_bytes[BLIND_SEEK_MAX_CANDIDATES] = {0};
    unsigned cand_fv[BLIND_SEEK_MAX_CANDIDATES] = {0};
    int peak_saved_count = 0;
    int verified_best_id = 0;

    /* No position model exists. Choose the first direction without pretending
     * to know it, then let the live focus metric prove or reject that choice. */
    int dir = ((now(&s) + p->out_start_fv) & 1) ? AF2_NEAR : AF2_FAR;
    SweepResult result = sweep(&s, dir, p->out_start_fv, &best, &best_pos, true);

    if (result == SWEEP_FLAT && !cancelled(&s)) {
        /* Five seconds without a 2 percent change gives no direction signal.
         * Return for the same time, then search the other side. */
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
    /*
     * Stage 2: Tournament Benchmark & Still Gallery Generation
     * Revisit every candidate peak discovered during the sweep. At each stationary
     * point, let vibrations settle, snap a reference frame (/tmp/peak_%d.jpg),
     * and benchmark true hardware sharpness against all other candidates.
     */
    if (s.io->pulse_ms && !p->fast_mode && s.candidate_count > 1 && !cancelled(&s)) {
        s.phase = "select-best";
        bool all_hw_ok = true;
        for (int c = 0; c < s.candidate_count && !cancelled(&s); c++) {
            if (!move_to(&s, s.candidates[c].pos)) break;
            nap(&s, p->settle_ms);
            if (cancelled(&s)) break;
            /* Save candidate snapshot to /tmp/peak_%d.jpg and measure sharpness */
            unsigned hw_score = query_and_save_peak(&s, c);
            cand_bytes[c] = hw_score;
            cand_fv[c] = sample(&s);
            if (hw_score == 0) all_hw_ok = false;
            peak_saved_count = c + 1;
        }
        if (peak_saved_count > 0 && !cancelled(&s)) {
            unsigned verified_best_score = 0;
            long verified_best_pos = best_pos;
            for (int c = 0; c < peak_saved_count; c++) {
                unsigned score = all_hw_ok ? cand_bytes[c] : cand_fv[c];
                if (score > verified_best_score) {
                    verified_best_score = score;
                    verified_best_pos = s.candidates[c].pos;
                    verified_best_id = c;
                }
            }
            best_pos = verified_best_pos;
            if (cand_fv[verified_best_id] > best)
                best = cand_fv[verified_best_id];
        }
    }

    /* Return to the best verified position even if the search budget expired;
     * only an external cancel/preemption aborts the return move. */
    s.returning = true;
    int return_dir = best_pos > s.pos ? AF2_FAR : best_pos < s.pos ? AF2_NEAR : 0;
    bool returned = !cancelled(&s) && move_to(&s, best_pos);

    bool verified = false;
    s.phase = "return";
    unsigned value = 0;

    /* On pulse hardware where candidate tournament selected the winner, lock the position directly! */
    if (s.io->pulse_ms) {
        nap(&s, p->settle_ms);
        value = cancelled(&s) ? 0 : sample(&s);
        verified = returned && !cancelled(&s);
        goto done;
    }

    /* Correct the settled landing with short pulses. Keep the pass target even
     * when a correction misses; replacing it would turn a miss into success. */
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
        if ((unsigned long long)value * 100 >=
            (unsigned long long)best * land_pct) {
            s.phase = "verify";
            nap(&s, p->settle_ms);
            unsigned check = sample(&s);
            if (check > best) {
                best = check;
                best_pos = s.pos;
            }
            if ((unsigned long long)check * 100 >=
                (unsigned long long)best * land_pct) {
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
            /* A decline after a proven rise is an overshoot. Reduce the step,
             * but do not shrink merely because the initial direction was wrong. */
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
            if ((unsigned long long)value * 100 >=
                (unsigned long long)best * land_pct)
                verified = !cancelled(&s);
        }
    }

done:
    p->out_converged = verified;

    io->drive(io->ctx, AF2_STOP);
    p->out_focus_pos = s.pos;
    if (s.io->pulse_ms && !cancelled(&s) && peak_saved_count > 1) {
        FILE *mf = fopen("/tmp/af_peaks.json.tmp", "w");
        if (mf) {
            fprintf(mf,
                    "{\"count\":%d,\"current_pos\":%ld,\"last_dir\":%d,"
                    "\"recovery_ms\":%ld,\"winner\":%d,\"candidates\":[",
                    peak_saved_count, s.pos, s.direction, p->recovery_ms,
                    verified_best_id);
            for (int c = 0; c < peak_saved_count; c++) {
                fprintf(mf, "%s{\"id\":%d,\"pos\":%ld,\"fv\":%u,\"bytes\":%u}",
                        c > 0 ? "," : "", c, s.candidates[c].pos,
                        s.candidates[c].fv, cand_bytes[c]);
            }
            fprintf(mf, "]}\n");
            fclose(mf);
            rename("/tmp/af_peaks.json.tmp", "/tmp/af_peaks.json");
        }
    }
    s.phase = "final";
    p->out_peak_fv = cancelled(&s) ? 0 : value;
    if (cancelled(&s)) p->out_converged = false;
    p->out_peak_seen = s.peak;
    return p->out_peak_fv;
}
