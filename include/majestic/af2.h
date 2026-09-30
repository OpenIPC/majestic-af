// af2 — autofocus for a continuous (timed, UART-driven) lens whose board tracks focus through a
// zoom by itself: the XiongMai 85H50AI. Its lens MCU moves focus along its own zoom curve while
// the zoom runs and for up to ~10 s after the stop, and leaves it CLOSE to the crest — within about
// 0.6 s of focus drive on the two healthy lenses measured (OpenIPC/motors xm-uart/PROTOCOL.md,
// "Zoom tracking inside the board") — but not on it: at X2.0 one board settled at 43 % of the
// sharpest focus, and the crest is only ~0.3 s of drive wide. So af2 does what the vendors'
// own cameras do (Sony's zoom-trigger AF, ONVIF's OnceAfterMove): let the board's coarse
// tracking finish, then run a short contrast search around wherever the lens is. The window
// is ±window_ms, widened once to ±wide_ms if the short one shows no crest (a subject much
// nearer than the scene the board's curve was made for).
//
// It keeps no absolute position and no zoom->focus curve. The earlier af2 did — a calibrated
// curve, a fixed 6.8 s post-zoom "overshoot" and a ~38 s travel with a seek to the near stop —
// and every number in that model came from one 85H50AI whose lens was later found degraded:
// replaced by a healthy one of the same model, the board's own tracking put focus back on the
// crest, which the old model never saw it do.
//
// The search moves: back off FAR to the window edge, then ONE smooth continuous sweep NEAR
// across it, stopping just past the crest and climbing back onto it, FV-guided so the
// reversal's backlash cannot displace the landing. It never hill-climbs or hunts.
//
// Frames that move zoom and focus TOGETHER are not an option: the board ignores a frame with
// both bits set (measured), and Pelco-D itself calls combining them "not recommended".

#ifndef AF2_H
#define AF2_H

enum { AF2_NEAR = -1, AF2_STOP = 0, AF2_FAR = +1 };

typedef struct {
    void (*drive)(void *ctx, int dir);   // dir: AF2_NEAR / AF2_STOP / AF2_FAR
    unsigned (*fv)(void *ctx);           // sample focus value (bigger = sharper)
    long (*now_ms)(void *ctx);           // monotonic clock
    void (*sleep_ms)(void *ctx, long ms);
    void *ctx;
} AfIO;

// Defaults, measured on the 85H50AI's healthy lenses (see the header). Backlash is the gear
// slack a reversal takes up before focus moves; the windows cover the tracking error with
// margin, then a near subject.
#define AF2_BACKLASH_MS 550
#define AF2_WINDOW_MS 1000
#define AF2_WIDE_MS 4000
#define AF2_EDGE_MS 400   // sweep this far past a window's edge, so a crest ON it is seen to fall

typedef struct {
    // mechanics (measured)
    long backlash_ms;    // gear slack paid on a direction reversal (AF2_BACKLASH_MS)
    long settle_ms;      // pause after a stop/reversal before measuring (160)
    long budget_ms;      // hard time cap for the whole pass
    int  fv_samples;     // FV reads to median per stationary measurement (5)
    long fv_frame_ms;    // spacing between those reads (40); the sweep samples at twice this
    // search window, ms of focus drive either side of where the lens is when the pass starts
    long window_ms;      // first, short window (AF2_WINDOW_MS)
    long wide_ms;        // the one widening when the short window has no crest (AF2_WIDE_MS)
    // optional trajectory trace: invoked at every FV measurement with the dead-reckoned
    // position (relative to the start of the pass) and the FV. For offline diagnosis of a real
    // pass; NULL disables.
    void (*trace)(void *ctx, long pos, unsigned fv);
    void *trace_ctx;
    // cooperative cancel: if non-NULL and *cancel turns non-zero, the pass abandons its
    // remaining moves and returns promptly. Used to preempt a focus that a fresh zoom has made
    // stale. NULL = never cancelled.
    const volatile int *cancel;
    // outputs
    unsigned out_peak_seen; // best (median) FV observed during the pass
    unsigned out_peak_fv;   // FV where it landed
    int out_steps;          // measurements taken (diagnostic)
    int out_window;         // 1 = the short window found it, 2 = it took the wide one
    int out_found_crest;    // 1 = the sweep recognised a real crest above the statistic's own
                            // floor. 0 means the pass had no gradient to work with and its
                            // landing is bookkeeping, not a measurement — the caller must not
                            // report that as a focused result.
    long out_landed_pos;    // where the lens ended up, ms of FAR drive from where it started.
                            // A diagnostic only: nothing absolute is known, or carried.
} AfParams;

// Run one pass. Returns the final FV. The motor is stopped by budget_ms; only the stop's
// settle_ms and one final stationary read may follow it.
unsigned af2_run(AfIO *io, AfParams *p);

#endif
