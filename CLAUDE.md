# CLAUDE.md

Guidance for Claude Code (claude.ai/code) working in this repository.

Orientation only: what this is, how it builds, where things live, and the rules
that are specific to it. It is not a place to restate general coding standards.

## What this is

`majestic-af` is the **out-of-core autofocus / PTZ engine** for the majestic IP
camera streamer. majestic loads it at runtime as a plugin (`/usr/lib/majestic-af.so`)
and hands it two commands (`autofocus`, `zoom`); everything else — the contrast
search, the motor protocol, the threads — lives here, out of majestic's core.

The point of the split is that the engine and the actuator wire-protocols evolve
here (open source), while majestic's core stays small and carries none of it. The
core exposes only the one thing a plugin cannot produce itself — the vendor ISP
**focus value** — as a HAL seam this plugin calls.

## The ABI (the one contract that must not drift)

`include/majestic/af_plugin_abi.h` is **vendored byte-identical** from majestic.
It is the entire boundary:

- **This plugin defines** `af_plugin_call(cmd, val)` and `af_plugin_exit()`.
  majestic `dlsym`s them and calls them from its `/autofocus` and `/zoom` handlers.
- **majestic defines** the HAL seams this plugin calls — `sdk_get_focus_value`
  (the focus statistic), `sdk_set_zoom_mag` (push magnification back for the OSD /
  `/zoom` GET), `config_get_string/int/boolean`, `log_log`. They are left
  **undefined** in the `.so` and resolve at `dlopen` against the majestic
  executable, which exports them via its `cmake/dynamic-list.txt` when built
  `WITH_PLUGINS_SUPPORT=ON`.

Only C functions with scalar/pointer arguments cross this boundary — no structs
(`AfIO`/`AfParams` stay inside the plugin) — so the ABI is immune to struct-layout
drift between the firmware toolchain and this one. If you change the ABI, change
the copy in majestic in the same breath.

## Layout

- `src/plugin.c` — the thin adapter from the two-token command ABI to the engine.
  Starts the magnification reader in a constructor at load.
- `src/engine.c` — ported from majestic's `src/af.c`: the XiongMai UART actuator,
  the `/tmp/btzoom.lock` discipline, the worker + magnification-reader threads,
  preemption/cancel, dead-reckoning, and the `AfIO` adapter (`fv` → the imported
  `sdk_get_focus_value`, `drive` → the selected actuator).
- `src/af2.c` — the search itself (a parfocal-curve-seeded single-sweep hunt with a
  closed-loop landing), portable, over the `AfIO` vtable. Ported verbatim.
- `include/majestic/` — the vendored self-contained headers (`af_plugin_abi.h`,
  `af2.h`, `af.h`, `log.h`).

## Build

Cross-compile against the same OpenIPC toolchain majestic uses:

```
cmake -Bbuild -DCMAKE_TOOLCHAIN_FILE=<majestic>/tools/cmake/toolchains/<cc>.cmake
cmake --build build
```

Produces `majestic-af.so`. Deploy it to `/usr/lib/majestic-af.so`. majestic loads
it iff `isp.autofocus.enabled` is true **and** the majestic binary was built
`WITH_PLUGINS_SUPPORT=ON` (that flag is what exports the HAL seams). If the seams
are missing, `RTLD_NOW` makes the `dlopen` fail and majestic keeps its built-in
engine — so a mismatched pair degrades, it does not crash.

## Tests

`tests/af2_model.c` is the af2 search's regression guard: it drives `src/af2.c`
against a synthetic parfocal lens+scene on a virtual clock — no hardware, runs in
milliseconds — using the vendored `greatest` framework (`tests/greatest.h`). It
builds host-native (CMake adds the test target only when NOT cross-compiling, since
a cross build has no host runner) and runs under `ctest`:

```
cmake -Bbuild && cmake --build build && ctest --test-dir build --output-on-failure
```

That same native configure builds the `.so` on the host too, which catches compile
errors without the cross toolchain. CI (`.github/workflows/ci.yml`) runs both on
every push and pull request.

## The rule that must not be broken (teardown)

The worker and reader threads are **joinable**, and `af_plugin_exit()` →
`af_engine_stop()` sets cancel, joins **both**, and returns **before** majestic
`dlclose`s this `.so`. A detached thread that outlives the unmap runs freed code
and faults on the next SIGHUP reload. Keep threads joinable; never detach them.

## Actuator backends

The actuator protocol is chosen at runtime from
`config_get_string("isp.autofocus","actuator")`, matched against the `ActuatorProto`
table in `engine.c`. Two are implemented: `pelco-xm` (the XiongMai near-Pelco
variant — `0xC5` sync, `0x5C` terminator, `sum % 100`; the default) and `pelco-d`
(standard Pelco-D — `0xFF` sync, 7 bytes, `sum % 256`). The command bits are shared
across them, so a backend is just another table entry; an external-exec backend
(hand the near/far/stop verbs to a user-supplied helper) is the natural next one.
majestic already registers the `isp.autofocus.actuator` key but its enum must list
a value for config to accept it.

## Workflow

`master` is protected: **all changes land through pull requests.** Branch off
`master`, push the branch, open a PR. Do not push to `master` directly.

## Relationship to majestic

This repo drives the focus/zoom motor and reads the focus value through majestic's
HAL seam; it never links majestic. The vendored headers under `include/majestic/`
must stay in sync with majestic's copies — the ABI header especially. Calibration
constants in `af2.c` (the parfocal curve, travel, backlash) are measured per lens;
the values in-tree are for the 85H50AI.
