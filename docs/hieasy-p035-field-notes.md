# HiEasy P035 Focus Behavior

These observations apply to a HiEasy P035 camera with a p6slite controller.
They do not describe the H07.

## Observed behavior

- The controller keeps the image near focus during zoom when focus matching is active.
- The focus can move back and forth after zoom. This document calls that movement breathing.
- A standard Pelco-D stop command does not stop the breathing.
- A manual focus pulse of approximately 70 ms stops the breathing.
- The controller can miss one stop frame after a requested movement.

The P035 driver sends three stop frames with 2 ms between frames. This delivery
rule belongs to the driver, not the AF algorithm.

The controller does not report focus position, movement time, or zoom
magnification. It can also move focus without an AF request. Therefore, its
movement invalidates a saved software position.

## Current AF model

Blind seek uses only the live ISP focus metric. It estimates a relative position
during one pass and never seeks a lens endpoint.

Tests show that the algorithm can return near the observed metric peak from
both directions. The correction stage waits 300 ms after a focus pulse because
shorter waits exposed stale metric readings.

More tests are necessary across scenes, zoom levels, and light levels. The
[blind-seek guide](blind-seek-autofocus.md) describes the current algorithm.

## Unknown controller command

The controller can have a vendor command that stops breathing directly. That
command is not known. A stock camera remains available for a future UART
capture.
