# majestic-af

Out-of-core **autofocus / PTZ engine** for the majestic IP camera streamer,
loaded as a runtime plugin.

majestic keeps the parts only it can provide — the vendor ISP focus statistic —
and hands the motor work to this plugin. The contrast-autofocus search, the
motorized-lens actuator protocols, and the worker threads all live here, so they
can evolve independently of majestic's core.

## How it plugs in

majestic `dlopen`s `/usr/lib/majestic-af.so` and drives it with a few commands
(`autofocus`, `ptz`, `zoom`) over a tiny C ABI (`include/majestic/af_plugin_abi.h`).
A `schema` command tells majestic which settings the plugin has — today, the
list of motor drivers — so they appear in the WebUI without a change to
majestic when a driver is added here. The
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
camera. It is picked up when `ptz.enabled` is set and the majestic
binary was built with plugin-symbol export enabled; otherwise majestic falls back
to its built-in engine, so a missing or mismatched plugin degrades rather than
breaks.

## Status

Works on HiSilicon (the focus statistic is implemented there). The actuator is
chosen at runtime by `ptz.actuator`, behind a small backend vtable
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
  fast `pan_delay_us` / `tilt_delay_us` let the head run. A move's speed (the
  core's `ptz` command `<verb>:<ms>:<speed>`, 1-100 % of that top rate) scales the
  step rate; the capability line says `speeds=1-100` where it does.

A camera with two motors on two wires -- a speed dome whose zoom/focus lens and
pan/tilt base are separate (the HI3516D_N81820 dome: an MS41908M lens on SPI, a
Pelco-D base on UART1; OpenIPC/firmware#2385) -- names the base in `ptz.head`
beside the lens in `ptz.actuator`:

```yaml
ptz:
  enabled: true
  actuator: ms41908     # the lens: zoom, focus, autofocus
  head: pelco-d         # the base: pan and tilt
  port: /dev/ttyAMA1    # the head's serial line
  baud: 9600
  address: 1
```

Pan and tilt go to the head, everything else to the lens, one verb at a time as
on a single wire (a pan stops a zoom, and the other way round). The capability
line is one line for both, with ` head=<protocol> headstate=ready|closed` at the
end. `head` is `none` by default, and a camera that leaves it so is driven
exactly as before. Today the head can be `pelco-d`; a head on the lens's own
transport, or beside a lens that already pans, is refused.

`ptz.panInvert` / `ptz.tiltInvert` turn the pan/tilt motors the other way, for
a head mounted so that every arrow moves it backwards even with the picture set
upright (the core turns each direction by the picture's mirror and flip, which
say nothing about how the base is mounted).

The `ms41908` backend's lens geometry defaults to the 16x LENS_LH13 lens of the
varifocal HI3516D_N81820. Another lens on the same chip sets its own:
`ptz.lensZoomSteps` and `ptz.lensFocusSteps` (end-to-end travel in microsteps),
`ptz.lensZoomMax` (the ratio at the tele end), `ptz.lensZoomReversed` (the zoom
motor's home stop is the wide end), and `ptz.lensSwap` (zoom and focus on each
other's motor). `ptz.lensHome` drives both axes into their end stops when the
lens opens and then zooms back to where the last run left it, for a lens whose
photo-interrupters read nothing useful; focus goes back too, to the position the
last run saved, even across a reboot (a homed origin makes it boot-independent); the backend logs each axis's position
at the end of every move (`ms41908: zoom at N of M ...`), which is how a lens's
travel is measured on the camera.

Focus-value support on other SoCs (Ingenic T31 has the metric) widens where the
plugin is useful.

## Contributing

`master` is protected — please open a pull request. CI builds the plugin and runs
the offline af2 model test on every PR; run it locally with
`cmake -Bbuild && cmake --build build && ctest --test-dir build`. See `CLAUDE.md`
for the architecture, the ABI contract, and the one hard rule (thread teardown
before `dlclose`).
