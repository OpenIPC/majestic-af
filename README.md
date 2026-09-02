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

The plugin owns the focus UART: it drives both zoom and focus on it, reads the
lens MCU's magnification reports, and runs the follow-up focus in one place.

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

Works on HiSilicon (the focus statistic is implemented there). Two UART actuator
protocols are implemented and chosen at runtime by `isp.autofocus.actuator` —
`pelco-xm` (the XiongMai near-Pelco variant, the default, field-tested) and
`pelco-d` (standard Pelco-D). An external-exec backend is the next one.
Focus-value support on other SoCs (Ingenic T31 has the metric) widens where the
plugin is useful.

## Contributing

`master` is protected — please open a pull request. CI builds the plugin and runs
the offline af2 model test on every PR; run it locally with
`cmake -Bbuild && cmake --build build && ctest --test-dir build`. See `CLAUDE.md`
for the architecture, the ABI contract, and the one hard rule (thread teardown
before `dlclose`).
