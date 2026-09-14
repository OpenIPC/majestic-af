# Future AF Research

These experiments can improve autofocus after the current blind-seek model is
stable. They are not requirements for the first implementation.

## Compare focus regions

Record the ISP focus grid during the same lens sweep. Calculate several metrics
from the saved frames:

- the complete frame
- a small center region
- a selected region
- the region with the strongest contrast.

Compare noise, false peaks, peak width, and the final focus position. This test
will show whether a center or selected region is more useful than the current
frame-wide metric.

## Test predictive stopping

Run a predictor beside the current algorithm without giving it motor control.
Record its predicted peak and stop time. Compare these predictions with the
peak that the current algorithm finds.

Use the result to decide whether predictive stopping can reduce overshoot. Keep
blind seek as the fallback when prediction confidence is low.

## Characterize more conditions

Repeat the focus sweeps at different zoom levels, object distances, light
levels, and scene textures. Measure peak shape, peak width, metric noise, and
the delay after each direction change.

Use only stable measurements in a lens model. Do not assume that one scene
describes the complete lens.

## Separate timing effects

Measure these delays separately:

- request delivery
- driver response
- physical lens response
- ISP metric response
- direction-change slack.

Also record movements that do not stop as invalid runs. Invalid runs must not
train or validate a focus model.

## Measure the P035 focus matching

Record the controller's focus movement after zoom stops. Measure its breathing
cycles, settle time, and response to a short manual focus pulse.

This experiment can test a fast path that accepts a good controller result. The
blind-seek search remains available when the result is not sharp enough.
