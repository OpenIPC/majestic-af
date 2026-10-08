// A motor transport, behind the motion policy layer.
//
// motion.c owns the arbitration that is the same for every lens — the watchdog,
// press/release deadlines, manual-preempts-search, the after-zoom focus booking,
// the teardown order — and an Actuator owns the part that is not: how a verb
// becomes motion, over what wire, and where (if anywhere) magnification comes
// from. Three exist: `act_uart` (the Pelco/XiongMai byte-frame path over a tty,
// proto.c), `act_ms41908` (the MS41908M SPI stepper on these Xiongmai boards) and
// `act_gpiostep` (a pan/tilt head of two GPIO steppers, through gpiostep.ko).
//
// Locking contract — the reason a stepper fits behind the same policy as a Pelco
// wire. emit() is called by motion.c with its policy mutex (mo_mu) held and MUST
// NOT block: a backend that actuates asynchronously records the request and
// returns, doing the slow SPI work on its own thread under its OWN lock, which
// must never take mo_mu (mo_mu -> backend lock is the only order that occurs).
// open()/close() run on the load and teardown paths; close() stops the motor,
// joins any thread the backend started, and releases the transport LAST — after
// motion.c has joined the watchdog and the engine has joined its worker/reader.

#ifndef MAJESTIC_AF_ACTUATOR_H
#define MAJESTIC_AF_ACTUATOR_H

#include <stdbool.h>

#include "proto.h"

typedef struct Actuator {
    const char *name;   // family, for logs (e.g. "pelco", "ms41908")
    // True when the backend reports motion itself (sdk_ptz_motion) from where
    // it actually drives the motor; motion.c then says nothing, rather than
    // reporting from the verbs, which only say what was asked.
    bool reports_motion;
    // The resolved wire name for the WebUI status line — for the UART family this
    // is the protocol chosen at open (pelco-xm / pelco-d), known only after open().
    const char *(*proto_name)(void);

    // Acquire the transport and initialise the lens; false if it cannot be
    // opened (motion.c then has no motor and every verb answers "unavailable").
    bool (*open)(void);
    // Stop the motor, join the backend's own threads, release the transport.
    void (*close)(void);
    // Put one verb on the wire. NON-BLOCKING (see the locking contract above);
    // false if it did not reach the lens. `speed` is 0..63 for the pan/tilt
    // verbs, ignored by the rest.
    bool (*emit)(enum PtzVerb v, int speed);
    // Does this actuator carry the verb at all? (The MS41908M lens has no
    // pan/tilt and no ICR, so it carries only stop/near/far/tele/wide.)
    bool (*has)(enum PtzVerb v);
    // The light one-shot wake sent on the open transition, and the full vendor
    // wake sequence. Backends that need neither return true.
    bool (*wake_blob)(void);
    bool (*wake)(void);
    // A descriptor a magnification READER can share (the UART RX carrying the
    // lens MCU's reports), or -1 when there is none. A -1 here is what keeps the
    // engine's UART zoom reader (and its wake-retry) from ever starting.
    int (*fd)(void);

    // How magnification is known. A UART lens MCU reports it on fd() and the
    // engine's reader parses it; a backend that commands the zoom motor ITSELF
    // derives it from the dead-reckoned zoom position and pushes it through
    // af_zoom_report(). derives_mag says which, so the two never both run.
    bool derives_mag;

    // Optional. A dead-reckoning backend (derives_mag) has no absolute zoom
    // reference until it seeks a stop, so from boot it reports nothing and /zoom
    // reads the stale restored value while the lens moves. Given the magnification
    // persisted from the last run -- and a zoom stepper holds position with no
    // power, so that IS where the lens still sits -- this seeds the dead-reckoned
    // position and marks the axis homed, WITHOUT a physical seek, so reports
    // resume from the first move. NULL for a backend whose position is absolute
    // (the UART MCU): there is nothing to seed.
    bool (*seed_mag)(float mag);

    // Focus gear slack on a reversal, in ms of drive, for af2 (0 = af2's measured
    // default). A step actuator computes it from its step cadence.
    long backlash_ms;
    // How long the lens keeps moving focus BY ITSELF after a zoom stops: a lens MCU
    // that tracks focus through a zoom finishes that move after the stop frame. The
    // after-zoom focus pass is booked no earlier than this, so it measures a lens
    // that has stopped. 0 = the lens does nothing of the kind.
    long zoom_settle_ms;
    // How long after a zoom stops the lens MCU may still make one LAST focus move, to its
    // own curve, undoing whatever focus was set in between. The after-zoom pass starts
    // after zoom_settle_ms and then watches the picture until this long after the stop,
    // refocusing once if that move comes. 0 = never.
    long zoom_late_ms;
    // End every zoom-out with a short zoom-in: carry the zoom-out on this long past the stop,
    // then zoom in for as long. A lens MCU that sets focus from its own zoom count leaves it
    // well off after a zoom-out, the zoom gear's slack putting the lens short of that count;
    // approaching the final ratio from the wide side takes the slack up. Per protocol (measured
    // on the XM board only); NULL or 0 = no bounce.
    long (*zoom_out_bounce_ms)(void);

    // Step-based focus, for the af3 bracket-and-return search on a MICROSTEP lens.
    // focus_steps > 0 is what marks an actuator step-capable: the engine then runs
    // af3 (settle-then-read, records the best, drives back to it, bounded) instead
    // of af2's timed continuous sweep. The three ops move an exact microstep count
    // (returning what actually moved, less at a stop), take an absolute reference by
    // ramping to the near stop, and report the dead-reckoned position (-1 = unknown).
    // All three block the CALLER only (the engine worker), never emit()/motion.c.
    // Zero / NULL on a continuous (UART) lens, which keeps af2.
    long focus_steps;            // full near<->far focus travel, microsteps
    long focus_backlash_steps;   // gear slack on a reversal, microsteps (best estimate)
    int (*focus_step)(int dir, int n);
    bool (*focus_home)(void);
    int (*focus_pos)(void);
} Actuator;

// Pick the actuator named by isp.autofocus.actuator. An unknown, empty or NULL
// name falls back to the UART Pelco path — what the key has always meant. Never
// returns NULL. The name the caller passed and the one that was resolved may
// differ (the fallback), which motion.c logs.
const Actuator *actuator_select(const char *name);

#endif
