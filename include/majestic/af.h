#ifndef MAJESTIC_AF_H
#define MAJESTIC_AF_H

#include <stdbool.h>

/* Contrast autofocus over the vendor focus statistic (sdk_get_focus_value).
 * Exists only when isp.autofocus.enabled is true and
 * isp.autofocus.algorithm names a supported algorithm. */

bool af_available(void);

/* Start one AF pass in a worker thread. 0 = started, 1 = already running,
 * -1 = not available. This function does not block. If `settle` is true, the
 * pass waits for the zoom sequence to end before it starts the AF algorithm. */
int af_trigger(bool settle);

/* Cancel the active AF job and discard pending automatic work. `motorsd`
 * handles motor lease preemption. Returns 0 if work was active, 1 if idle,
 * or -1 if AF is unavailable. */
int af_cancel_pass(void);

/* Cancel AF and discard dead-reckoned focus position after manual focus. */
void af_note_manual_focus(void);

/* "idle", "running", or the last pass's one-line result. */
const char *af_status(void);

/* Request one timed zoom pulse through the motor service. A positive value
 * moves toward tele. A negative value moves toward wide. The worker starts AF
 * after the zoom sequence. Returns 0 if started or -1 if AF is unavailable. */
int af_zoom_pulse(int dir);

/* Start the background reader for optional zoom magnification events from the
 * motor service. This function has no effect if AF is unavailable. */
void af_zoom_start(void);

/* Return the last zoom magnification from the motor service. Return -1 if the
 * driver has not supplied a value. The OSD `%@` token shows this value. */
float af_zoom_mag(void);

/* Milliseconds since the magnification last changed (large if never). Small = a zoom is
 * happening or just finished; large = stable (still the correct current zoom). */
long af_zoom_age(void);

#endif
