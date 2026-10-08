# majestic-af

Out-of-core **autofocus / PTZ engine** for the majestic IP camera streamer,
loaded as a runtime plugin.

majestic keeps the parts only it can provide — the vendor ISP focus statistic —
and hands the motor work to this plugin. The contrast-autofocus search, the
motorized-lens actuator protocols, and the worker threads all live here, so they
can evolve independently of majestic's core.

## How it plugs in

majestic `dlopen`s `/usr/lib/majestic-af.so` and drives it with two commands
(`autofocus`, `zoom`) over a tiny C ABI (`include/majestic/af_plugin_abi.h`). The
plugin resolves the focus value and a few helpers back from the majestic
executable at load time. Nothing links majestic; the two sides only share one
header.

The plugin owns the motor wire outright — it is the only writer on it. Zoom,
focus, pan and tilt, the autofocus pass and the lens's magnification all go
through one actuator behind one mutex, so a manual move can preempt a running
search cleanly instead of interleaving with it. The wire itself — a serial tty
for Pelco lenses, or SPI + GPIO for the MS41908M — sits behind an actuator vtable
(`include/actuator.h`).

## Build

Cross-compile against the same OpenIPC toolchain as majestic:

```sh
cmake -Bbuild -DCMAKE_TOOLCHAIN_FILE=<majestic>/tools/cmake/toolchains/<cc>.cmake
cmake --build build
```

This produces `majestic-af.so`. Copy it to `/usr/lib/majestic-af.so` on the
camera. It is picked up when `isp.autofocus.enabled` is set and the majestic
binary was built with plugin-symbol export enabled; otherwise majestic falls back
to its built-in engine, so a missing or mismatched plugin degrades rather than
breaks.

## Status

Works on HiSilicon (the focus statistic is implemented there). The actuator is
chosen at runtime by `isp.autofocus.actuator`, behind a small backend vtable
(`include/actuator.h`):

- `pelco-xm` (the XiongMai near-Pelco variant, the default, field-tested) and
  `pelco-d` (standard Pelco-D) — UART byte-frame protocols over a serial tty; the
  lens MCU reports magnification on the same RX line.
- `ms41908` — the Panasonic MS41908M lens stepper the Xiongmai HI3516D_N81820
  boards (HiSilicon Hi3516A V100) drive over SPI + GPIO, with no UART MCU. It has
  no magnification report, so the plugin derives it from the zoom position it
  dead-reckons itself. Motion needs majestic to be streaming (the MS41908M steps
  on the ISP's VD timing). Ported from the OpenIPC/motors `ms41908-lens` tool; the
  zoom→magnification and parfocal curves are calibrated on hardware.
- `gpiostep` — pan/tilt heads whose two steppers hang straight off SoC GPIOs with
  no motor MCU (Goke GK7205V510 PTZ cameras), through OpenIPC/firmware's
  `gpiostep.ko`. Pan and tilt only; the head's travel and directions come from
  `/etc/gpiostep.conf`, and it homes into both stops once per boot -- slowly
  (`pan_home_delay_us` / `tilt_home_delay_us`, 2000/3000 us by default), however
  fast `pan_delay_us` / `tilt_delay_us` let the head run.

Focus-value support on other SoCs (Ingenic T31 has the metric) widens where the
plugin is useful.

## Contributing

`master` is protected — please open a pull request. CI builds the plugin and runs
the offline af2 model test on every PR; run it locally with
`cmake -Bbuild && cmake --build build && ctest --test-dir build`. See `CLAUDE.md`
for the architecture, the ABI contract, and the one hard rule (thread teardown
before `dlclose`).
