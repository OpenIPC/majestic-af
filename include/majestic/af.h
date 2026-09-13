#ifndef MAJESTIC_AF_H
#define MAJESTIC_AF_H

#include <stdbool.h>

/* Contrast autofocus over the vendor focus statistic (sdk_get_focus_value).
 * Exists only on cameras that declared a focus motor: isp.autofocus.enabled
 * gates everything, so a lens without one never grows an /autofocus
 * endpoint. */

bool af_available(void);

/* Kick a one-shot pass in a worker thread. 0 = started, 1 = already
 * running, -1 = not available. Never blocks. With `settle`, the pass first
 * waits until the operator has stopped driving the pad, so a held button
 * finishes before the engine takes the wire. */
int af_trigger(bool settle);

/* "idle", "running", or the last pass's one-line result. */
const char *af_status(void);

/* One zoom step (dir > 0 tele, < 0 wide), the compatibility spelling of the
 * `ptz` verbs — see af_ptz_move() in motion.h. Returns 0 started, -1 not
 * available. Never blocks: the motor stops on its own deadline and the
 * follow-up focus is booked once the operator has finished zooming. */
int af_zoom_pulse(int dir);

/* Bring the motor up: open the port and start the thread that sniffs the lens
 * MCU's magnification reports ("X<ratio>" ASCII on the RX line, emitted while
 * zoom moves), caching the latest value. A no-op where no focus motor is
 * declared (af_available() false). Safe to call once at startup. */
void af_engine_start(void);

/* Last zoom magnification reported by the lens (e.g. 3.2 for "X3.2"), or -1 if none
 * has been seen yet (the MCU only reports while zoom is moving). The OSD `%@` token
 * renders this. Cheap and non-blocking — reads a mutex-guarded cache. */
float af_zoom_mag(void);

/* Milliseconds since the magnification last changed (large if never). Small = a zoom is
 * happening or just finished; large = stable (still the correct current zoom). */
long af_zoom_age(void);

#endif
