# Blind-Seek Autofocus

Blind seek uses relative focus movements and the ISP focus metric. It has no
knowledge of the physical lens position.

It does not use these inputs:

- motor position or movement feedback
- zoom magnification
- a saved focus position
- a calibrated zoom-to-focus model

The P035 uses this algorithm because its controller keeps the lens near focus
during zoom. The AF pass starts from that position and searches for the local
metric peak.

## Why the P035 does not use AF2

The P035 does not report focus position, movement time, or zoom magnification.
Its controller can also move focus during zoom without an AF request.

AF2 uses a calibrated lens model and a time-based focus position. Unreported
movement makes that position invalid. Blind seek keeps an estimate during one
pass and discards it after the pass.

`motorsd` removes client scheduling delay from each requested pulse. The driver
reports when it finishes the delivery sequence. This event does not confirm
physical lens movement because the P035 provides no motor feedback.

## Search

The first direction is arbitrary because the controller gives no position
data. The focus metric then controls the search:

1. Read the initial metric.
2. Move continuously in the first direction.
3. Reverse when two readings confirm a decrease.
4. Try the other direction if the first direction gives no useful change.
5. Record the best metric and its estimated position.
6. Continue past the peak to confirm its location.
7. Return to the best estimated position.
8. Use short correction pulses when reversal slack affects the return.
9. Take two settled measurements before success.

The search treats a 2 percent change as significant. A normal sweep permits a
larger decrease before it abandons the direction. Five samples form each
stationary measurement, which reduces short metric spikes.

Correction pulses do not exceed 70 ms. The algorithm reduces the pulse length
after an overshoot. This limit prevents one correction from moving far away
from the observed peak.

## Timing and cancellation

A settled AF request waits 1200 ms for the zoom sequence. It then waits another
150 ms before the first focus measurement.

One search has a 30-second budget. One continuous sweep can run for at most 12
seconds. A new zoom request or a manual motor request cancels the active pass.

A manual client has higher priority than AF. `motorsd` revokes the AF lease
before it gives the motor to that client.

## Result and diagnostics

Success requires two settled measurements near the best value from the pass.
The status reports an incomplete result if the algorithm cannot confirm the
landing.

Create `/tmp/af_trace.on` to write `/tmp/af_trace.csv`. The trace contains the
metric, phase, sample timestamps, requested direction, and pulse length.

The trace position is only an estimate. A command timestamp does not prove that
the physical lens moved at that time.

See [Focus characterization](focus-characterization.md) for repeatable lens
sweeps. See [P035 field notes](hieasy-p035-field-notes.md) for observed hardware
behavior.
