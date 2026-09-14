# majestic-af

`majestic-af` is an autofocus plugin for the Majestic camera streamer.
Majestic supplies the ISP focus metric. The plugin owns AF policy and sends
logical movement requests through `libmotors`.

The plugin handles three Majestic commands:

- `autofocus` starts, cancels, or reports an AF pass.
- `ptz` controls pan, tilt, zoom, and focus through `motorsd`.
- `zoom` keeps compatibility with the existing Majestic zoom interface.

`motorsd` coordinates clients. Its selected driver owns the hardware, movement
timing, and device-specific delivery rules.

## Build

Cross-compile with the same OpenIPC toolchain as Majestic:

```sh
cmake -Bbuild -DCMAKE_TOOLCHAIN_FILE=<majestic>/tools/cmake/toolchains/<cc>.cmake
cmake --build build
```

The build requires `json-c` and the `motorsd` source tree. CMake looks for
`motors/motorsd` beside this repository by default. Set a different location
when necessary:

```sh
cmake -Bbuild -DLIBMOTORS_DIR=/path/to/motorsd
```

Install the result as `/usr/lib/majestic-af.so`. Majestic loads it when both
plugin support and `isp.autofocus.enabled` are active.

## Select an algorithm

Select the algorithm in `majestic.yaml`:

```yaml
isp:
  autofocus:
    enabled: true
    algorithm: blind_seek
```

The available values are:

- `blind_seek` follows only the ISP focus metric. Use it for the P035.
- `af2` uses a calibrated lens model and live zoom magnification.

The plugin reads this value once. Reload the plugin after a change. A missing
or invalid value disables AF and writes an error to the log.

The motor driver reports capabilities and telemetry. It does not select the AF
algorithm.

## Test

Run the host tests with these commands:

```sh
cmake -Bbuild
cmake --build build
ctest --test-dir build --output-on-failure
```

The tests cover both AF algorithms and the `libmotors` adapter.

## More information

- [Blind-seek autofocus](docs/blind-seek-autofocus.md)
- [P035 field notes](docs/hieasy-p035-field-notes.md)
- [Focus characterization](docs/focus-characterization.md)
- [Future AF research](docs/future-af-research.md)

See `CLAUDE.md` for source layout, the plugin ABI, and thread teardown rules.
