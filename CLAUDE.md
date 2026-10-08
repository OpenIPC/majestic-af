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
  `/zoom` GET), `config_get_string/int/boolean`, `log_log`, and, referenced
  weakly so an older core still loads the plugin, `sdk_ptz_motion` (the motor
  started or stopped: act_gpiostep reports it from its step thread, other
  backends through motion.c's verbs). They are left
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
- `src/proto.c` — the Pelco wire: one frame builder and two protocol descriptors.
  Pure, no HAL seams, no port, so `tests/proto_test.c` can pin every byte it emits.
- `include/actuator.h` + `src/actuator.c` — the actuator vtable and its registry.
  A backend owns the wire (how a verb becomes motion, and where magnification
  comes from); motion.c owns the arbitration. `act_uart` (Pelco/XiongMai over a
  tty) and `act_ms41908` (the MS41908M SPI stepper) implement it.
- `src/act_uart.c` — the UART transport half lifted out of motion.c: the tty,
  termios, the frame writer, the vendor wake sequence, and the fd the reader shares.
- `src/act_ms41908.c` — the MS41908M SPI stepper (Xiongmai HI3516D_N81820 /
  Hi3516A V100): SPI + PL061 GPIO + VD_FZ, a stepping thread that emulates
  continuous drive so af2's timed model is unchanged, and magnification derived
  from the dead-reckoned zoom position. Ported from OpenIPC/motors `ms41908-lens`.
- `src/ms41908_calc.c` — that backend's pure logic (verb→axis, soft-limit clamp,
  zoom→magnification curve), split out so `tests/actuator_test.c` pins it with no
  hardware, the way proto_test pins the frames.
- `src/act_gpiostep.c` — a pan/tilt head of two 4-wire steppers driven straight off
  GPIO by OpenIPC/firmware's `gpiostep.ko` (`/dev/motorDev`, one ioctl per move):
  the Goke GK7205V510 PTZ cameras. A stepping thread in act_ms41908's shape, homing
  into both gearbox stops once per boot, soft limits from `/etc/gpiostep.conf`.
- `src/gpiostep_calc.c` — its pure logic (the config parser, verb→axis/direction,
  the soft-limit clamp), pinned by `tests/gpiostep_test.c`.
- `src/motion.c` — the motor's single owner. Holds the arbitration — one verb on
  the wire at a time, the watchdog that stops a manual move on its deadline, the
  after-zoom booking — and reaches the wire ONLY through the actuator vtable.
  Manual verbs outrank the search: `motion_engine_drive()` writes nothing while
  an operator is driving.
- `src/engine.c` — the pass: the worker thread, preemption/cancel, dead-reckoning,
  and the `AfIO` adapter (`fv` → the imported `sdk_get_focus_value`, `drive` →
  `motion_engine_drive`).
- `src/af2.c` — the search for a continuous (UART) lens: after the lens MCU has
  finished its own focus tracking, one sweep across a short window around wherever
  the lens is, with a closed-loop landing; widened once if the window has no crest.
  Portable, over the `AfIO` vtable. No zoom→focus curve and no absolute position:
  the 85H50AI's MCU tracks focus through a zoom itself (af2.h).
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
against a synthetic lens+scene (the 85H50AI's measured crest width, backlash and
tracking error) on a virtual clock — no hardware, runs in
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
`af_engine_stop()` joins **all three** before majestic `dlclose`s this `.so`. A
detached thread that outlives the unmap runs freed code and faults on the next
SIGHUP reload. Keep threads joinable; never detach them.

The **order** is load-bearing, in both directions:

1. set `af_shutdown` / `af_cancel`
2. `motion_stop_watchdog()` — the watchdog can *start* a pass (`af_book_tick`),
   so joining the worker while it still runs leaves a window where it spawns one
   into a shutdown that has already decided there was nothing to join
3. join the worker
4. `af_reader_stop`, join the reader
5. `motion_close()` — the port goes **last**, after everything that touches it

`af_spawn()` holds `af_mu` across the `pthread_create`, because `af_worker_valid`
is what teardown reads to decide whether to join: setting it after the thread
exists but outside the lock leaves a moment where a live worker looks like none.

## Actuator backends

The actuator is chosen at runtime from `config_get_string("isp.autofocus","actuator")`
via `actuator_select()`, behind the `Actuator` vtable (`include/actuator.h`). The
vtable is the seam: motion.c keeps the arbitration and reaches the wire only through
`emit`/`open`/`close`/`wake`/`fd`. Backends:

- **`act_uart`** — the Pelco family over a tty (`proto.c`): `pelco-xm` (the XiongMai
  near-Pelco variant — `0xC5` sync, `0x5C` terminator, `sum % 256`; the default) and
  `pelco-d` (standard Pelco-D — `0xFF` sync, 7 bytes, `sum % 256`). The command bits
  are shared but for focus: the XiongMai board reads the two focus bits the other
  way round from Pelco-D (cmd2 `0x80` focuses NEARER, measured), so `pelco-xm`
  swaps them. Otherwise a protocol is a descriptor plus at most a verb the others
  lack. The `act_uart` vtable also carries `zoom_settle_ms` and `zoom_late_ms`: the XiongMai
  MCU goes on moving focus after a zoom stop, usually for under 3 s but sometimes up to ~10 s.
  The after-zoom pass waits out the usual settle (`zoom_settle_ms`, 3 s). engine.c's
  `af_watch_late_move` then watches FV until `zoom_late_ms` (10 s) has passed, and refocuses
  if the board's late move undid the pass. `zoom_out_bounce_ms` (200 ms on `pelco-xm`) ends every zoom-out
  with a short zoom-in: the board sets focus from its zoom count, and after a zoom-out the zoom
  gear's slack leaves the lens short of that count (focus at ~33 % of best, against ~90 % with
  the bounce). The
  lens MCU reports magnification on the RX line, read by engine.c's `af_zoom_thread`.
- **`act_ms41908`** — the Panasonic MS41908M SPI lens stepper (Xiongmai
  HI3516D_N81820 / Hi3516A V100). No UART, no MCU. `emit` sets a stepping direction
  (NON-BLOCKING); a stepping thread issues micro-step bursts at a fixed cadence while
  a direction is held, so it looks like a continuous lens (its search is af3, the
  step-based one; `focus_steps > 0` selects it). It has no
  magnification report, so it DERIVES magnification from the zoom position it
  dead-reckons and pushes it through `af_zoom_report()`; `fd()` returns -1, which is
  what keeps the UART reader and wake-retry off. `has()` carries only
  stop/near/far/tele/wide (no pan/tilt, no ICR). The motor only steps while the ISP
  is producing VD, so majestic must be streaming for motion. The soft travel limits
  are the exact libxmaf values; the zoom→magnification and parfocal curves are
  best-effort placeholders calibrated on hardware.

- **`act_gpiostep`** — a pan/tilt head with no MCU: two 4-wire steppers on SoC GPIOs,
  energised by `gpiostep.ko` from kernel context (Goke GK7205V510 PTZ cameras). `emit`
  records a direction; a stepping thread moves `GS_CHUNK` steps at a time while it is
  held, so a stop lands within ~50 ms. `has()` is up/down/left/right/stop only, so
  `motion_can_focus()` is false and `/autofocus` answers `unavailable`. The board
  describes the head in `/etc/gpiostep.conf` (`pan_travel`, `tilt_travel`, `pan_left`,
  `tilt_up`, the per-step delays, `home`). With a travel known the position is
  dead-reckoned from a homing seek into both stops, done once per boot and kept in
  `/tmp/gpiostep.pos` across majestic restarts, and every move is clamped short of
  the stops. All of that is per axis. A stop ends the seek, directions are refused
  while it runs, and an axis whose seek did not finish refuses moves until the next
  restart homes it. The position file is removed while the head moves, so a kill
  mid-move leaves nothing stale to trust. No magnification: `fd()` is -1 and
  `derives_mag` is false.

Adding a backend is a new `Actuator` (a new file + a row in `actuator_select`). A
verb still goes in `VERB[]`/`proto.c` and `tests/proto_test.c` — never a hand-typed
frame. To make a new actuator selectable, majestic's `isp.autofocus.actuator` enum
must list its name (the key exists; its enum must carry the value) — a companion
change in the majestic repo.

`isp.autofocus.pulse` sizes **operator** movements: one tap, and the window the
watchdog stops the motor after. The af2 search takes no timing from config — its
window and backlash are the measured lens mechanics in `af2.h`, and a search told to
move in the wrong-sized steps does not converge.

Add a verb by adding a row to `VERB[]` in `proto.c` and a case in `tests/proto_test.c`
— never by writing a frame out by hand. The frames used to be a hand-typed table
nothing checked, and the XiongMai `near` and `far` were swapped for years.

## One writer on the wire

`motion.c` is the only thing in this plugin — and, once the WebUI stopped shipping
its own Pelco scripts, the only thing on the camera — that writes to the motor
(the tty for the Pelco family, or the SPI bus for the MS41908M). The WebUI's `btzoom` and
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
constants in `af2.h` (window, backlash) and `act_uart.c` (`zoom_settle_ms`, `zoom_late_ms`, `zoom_out_bounce_ms`) are
measured per lens; the values in-tree are for the 85H50AI. Measure on a HEALTHY
lens: the previous af2 model (a zoom→focus curve, a 6.8 s post-zoom overshoot, a
38 s travel) was calibrated on one whose motors had degraded, and described that
lens rather than the model. The method and captures are OpenIPC/motors
`xm-uart/PROTOCOL.md`, "Zoom tracking inside the board".
