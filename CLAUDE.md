# majestic-af development guide

This repository contains the autofocus plugin for Majestic. Read `README.md`
for build, installation, configuration, and test commands.

## Ownership

Majestic provides the ISP focus metric. This plugin owns AF policy and results.
It sends logical movement requests through `libmotors`.

`motorsd` owns leases and preemption. The selected driver owns hardware access,
movement timing, protocol frames, and hardware delivery rules.

Do not add a hardware protocol or device configuration to this repository.

## Plugin ABI

`include/majestic/af_plugin_abi.h` is the complete boundary between Majestic
and this plugin. Majestic contains an identical copy.

The plugin exports `af_plugin_call()` and `af_plugin_exit()`. Majestic calls
them for `autofocus`, `ptz`, and compatibility `zoom` requests.

Majestic exports these functions for the plugin:

- `sdk_get_focus_value()` supplies the ISP focus metric.
- `sdk_set_zoom_mag()` updates the Majestic zoom cache.
- `config_get_*()` reads `isp.autofocus` configuration.
- `log_log()` writes to the Majestic log.

Only C functions with scalar or pointer arguments cross this boundary. Internal
AF structures do not cross it.

Change both copies of the ABI header in the same change.

## Source layout

- `src/plugin.c` adapts Majestic commands to AF and `libmotors`.
- `src/engine.c` owns workers, status, cancellation, and algorithm selection.
- `src/af_motor.c` adapts AF operations to `libmotors`.
- `src/af_blind_seek.c` implements metric-only autofocus.
- `src/af2.c` implements calibrated autofocus with zoom magnification.
- `tests/af2_model.c` tests both algorithms with synthetic lens models.
- `tests/af_motor_test.c` tests the `libmotors` adapter.

## Thread teardown

The AF worker and telemetry reader are joinable threads. Keep them joinable.

`af_plugin_exit()` calls `af_engine_stop()`. This function cancels and joins
both threads before Majestic unloads the plugin.

A detached thread can execute unloaded plugin code after `dlclose()`. This can
cause a fault during a Majestic reload.

## Algorithm boundary

The configured values are `blind_seek` and `af2`. The plugin reads the selected
value once. A missing or invalid value makes AF unavailable.

Use `blind_seek` for the P035. Use `af2` only with its calibrated lens and valid
zoom magnification telemetry.

The driver never selects the AF algorithm.
