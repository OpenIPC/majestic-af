# Focus Characterization

This procedure records the focus metric while the lens crosses a local focus
peak. The result shows the peak shape, actuator delay, and direction effects.

The current tool is for development. It uses the local
`/tmp/af_metric_stream.on` marker to return raw metrics from the AF status URL.

## Requirements

- Install `motorsd`, `motorsctl`, the selected motor driver, and `majestic-af`.
- Keep the camera and the scene stationary during the capture.
- Use a scene with visible contrast at the selected zoom level.
- Make sure that no other motor client sends commands during the capture.

## Capture the data

Run these commands from the repository root:

```sh
RESULT=test-results/p035-focus-model-YYYYMMDD
mkdir -p "$RESULT"

scp -O tools/capture-focus-sweeps.sh root@CAMERA:/tmp/focus-capture.sh
ssh root@CAMERA 'chmod +x /tmp/focus-capture.sh && /tmp/focus-capture.sh 4'

scp -O root@CAMERA:/tmp/p035-focus-sweeps.csv "$RESULT/raw.csv"
scp -O root@CAMERA:/tmp/p035-focus-events.log "$RESULT/events.log"
python3 tools/analyze-focus-sweeps.py "$RESULT"
```

The argument `4` records four sweeps in each direction. Each measurement uses
this sequence:

1. Run autofocus.
2. Move 3000 ms away from the focus position.
3. Sweep 6000 ms in the opposite direction.
4. Record raw focus values during both movements.

The script alternates the movement directions. Thus, each pair crosses the
same focus peak from both sides.

## Read the results

`raw.csv` contains the ISP values and monotonic timestamps. `events.log`
contains the motor start and end timestamps from `motorsd`.

`sweeps.svg` shows the metric against the nominal motor-time position.
`peak-aligned.svg` compares the normalized peak shapes from all sweeps.

The nominal position is not a measured lens position. The P035 controller does
not provide position or movement feedback. Backlash and controller delay are
part of the measured difference between the two directions.

Repeat the capture for different zoom levels, distances, scenes, and light
levels before you use the data for a general focus model.
