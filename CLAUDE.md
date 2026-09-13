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
  Opens the port and starts the magnification reader in a constructor at load.
- `src/proto.c` — the wire: one frame builder and two protocol descriptors. Pure,
  no HAL seams, no port, so `tests/proto_test.c` can pin every byte it emits.
- `src/motion.c` — the port's single owner. Holds the one descriptor, serialises
  every write, and runs the watchdog that stops a manual move on its deadline.
  Manual verbs outrank the search: `motion_engine_drive()` writes nothing while
  an operator is driving.
- `src/engine.c` — the pass: the worker thread, preemption/cancel, dead-reckoning,
  and the `AfIO` adapter (`fv` → the imported `sdk_get_focus_value`, `drive` →
  `motion_engine_drive`).
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

The worker, reader and motion threads are **joinable**, and `af_plugin_exit()` →
`af_engine_stop()` sets cancel, joins **all three**, stops the motor and closes the
port, and returns **before** majestic `dlclose`s this `.so`. A detached thread that
outlives the unmap runs freed code and faults on the next SIGHUP reload. Keep
threads joinable; never detach them. The port is closed **last**, after both
threads that touch it are joined.

## Actuator backends

The actuator protocol is chosen at runtime from
`config_get_string("isp.autofocus","actuator")`, matched against the `PtzProto`
descriptors in `proto.c`. Two are implemented: `pelco-xm` (the XiongMai near-Pelco
variant — `0xC5` sync, `0x5C` terminator, `sum % 100`; the default) and `pelco-d`
(standard Pelco-D — `0xFF` sync, 7 bytes, `sum % 256`). The command bits are shared
across them, so a backend is a descriptor plus, at most, a verb the others lack;
an external-exec backend (hand the verbs to a user-supplied helper) is the natural
next one. majestic already registers the `isp.autofocus.actuator` key but its enum
must list a value for config to accept it.

`isp.autofocus.pulse` sizes **operator** movements: one tap, and the window the
watchdog stops the motor after. The af2 search takes no timing from config — its
move lengths come closed-loop from the measured lens mechanics in `af2.c`, and a
search told to move in the wrong-sized steps does not converge.

Add a verb by adding a row to `VERB[]` in `proto.c` and a case in `tests/proto_test.c`
— never by writing a frame out by hand. The mod-100 checksum was wrong on exactly
one frame (`far`, the only verb whose byte sum exceeds 100) for two years because
the frames were a hand-typed table nothing checked.

## One writer on the wire

`motion.c` is the only thing in this plugin — and, once the WebUI stopped shipping
its own Pelco scripts, the only thing on the camera — that writes to the motor UART. The WebUI's `btzoom` and
`btzoom-xm` scripts are gone, and so is the `/tmp/btzoom.lock` they were arbitrated
with. Do not reintroduce a second writer, and do not "just take the lock" from
somewhere else: a lock cannot make a three-step movement (drive, wait, stop) atomic
against another process, which is the whole reason those scripts were removed.

## Workflow

`master` is protected: **all changes land through pull requests.** Branch off
`master`, push the branch, open a PR. Do not push to `master` directly.

## Relationship to majestic

This repo drives the focus/zoom motor and reads the focus value through majestic's
HAL seam; it never links majestic. The vendored headers under `include/majestic/`
must stay in sync with majestic's copies — the ABI header especially. Calibration
constants in `af2.c` (the parfocal curve, travel, backlash) are measured per lens;
the values in-tree are for the 85H50AI.
