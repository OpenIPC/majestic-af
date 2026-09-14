#!/usr/bin/env python3
"""Build reproducible summaries and SVG plots for the P035 focus sweeps."""

import csv
import json
import shutil
import statistics
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

ROOT = (Path(sys.argv[1]).resolve() if len(sys.argv) > 1
        else Path(__file__).resolve().parent)


def median_smooth(points, radius=2):
    result = []
    for index, (x_value, _) in enumerate(points):
        window = points[max(0, index - radius):index + radius + 1]
        result.append((x_value, statistics.median(value for _, value in window)))
    return result


def load_runs():
    with (ROOT / "raw.csv").open(newline="") as source:
        rows = list(csv.DictReader(source))
    with (ROOT / "events.log").open() as source:
        events = [json.loads(line) for line in source]

    starts = [event["t_mono_ms"] for event in events
              if event.get("state") == "start_sent"]
    groups = defaultdict(list)
    for row in rows:
        key = (int(row["run"]), row["phase"], row["direction"])
        groups[key].append((int(row["t_mono_ms"]), int(row["fv"])))

    runs = []
    for index, (key, samples) in enumerate(groups.items()):
        run, phase, direction = key
        start = starts[index]
        points = [(stamp - start, value) for stamp, value in samples if stamp >= start]
        smooth = median_smooth(points)
        runs.append({"run": run, "phase": phase, "direction": direction,
                     "start": start, "points": points, "smooth": smooth})
    return runs


def svg_plot(path, title, x_label, series, x_range, y_range):
    width, height = 1100, 650
    left, right, top, bottom = 85, 30, 65, 70
    plot_w, plot_h = width - left - right, height - top - bottom
    xmin, xmax = x_range
    ymin, ymax = y_range

    def sx(value):
        return left + (value - xmin) * plot_w / (xmax - xmin)

    def sy(value):
        return top + (ymax - value) * plot_h / (ymax - ymin)

    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#111827"/>',
        f'<text x="{width / 2}" y="32" text-anchor="middle" fill="#f3f4f6" font-family="sans-serif" font-size="22">{title}</text>',
    ]
    for step in range(6):
        y_value = ymin + (ymax - ymin) * step / 5
        y = sy(y_value)
        lines.append(f'<line x1="{left}" y1="{y:.1f}" x2="{width-right}" y2="{y:.1f}" stroke="#374151"/>')
        lines.append(f'<text x="{left-10}" y="{y+4:.1f}" text-anchor="end" fill="#9ca3af" font-family="sans-serif" font-size="13">{y_value:.1f}</text>')
    for step in range(7):
        x_value = xmin + (xmax - xmin) * step / 6
        x = sx(x_value)
        lines.append(f'<line x1="{x:.1f}" y1="{top}" x2="{x:.1f}" y2="{height-bottom}" stroke="#374151"/>')
        lines.append(f'<text x="{x:.1f}" y="{height-bottom+22}" text-anchor="middle" fill="#9ca3af" font-family="sans-serif" font-size="13">{x_value:.0f}</text>')

    for label, color, points in series:
        coords = " ".join(f"{sx(x):.1f},{sy(y):.1f}" for x, y in points)
        lines.append(f'<polyline points="{coords}" fill="none" stroke="{color}" stroke-width="2" opacity="0.72"/>')

    lines.extend([
        f'<text x="{width/2}" y="{height-18}" text-anchor="middle" fill="#d1d5db" font-family="sans-serif" font-size="15">{x_label}</text>',
        f'<text x="18" y="{height/2}" text-anchor="middle" fill="#d1d5db" font-family="sans-serif" font-size="15" transform="rotate(-90 18 {height/2})">Focus metric</text>',
        '<line x1="760" y1="35" x2="800" y2="35" stroke="#38bdf8" stroke-width="3"/><text x="810" y="40" fill="#d1d5db" font-family="sans-serif" font-size="14">NEAR sweep</text>',
        '<line x1="910" y1="35" x2="950" y2="35" stroke="#fb923c" stroke-width="3"/><text x="960" y="40" fill="#d1d5db" font-family="sans-serif" font-size="14">FAR sweep</text>',
        '</svg>',
    ])
    path.write_text("\n".join(lines) + "\n")


def main():
    runs = load_runs()
    sweeps = [run for run in runs if run["phase"] == "sweep"]
    colors = {"near": "#38bdf8", "far": "#fb923c"}
    processed = []
    summaries = []

    for sweep in sweeps:
        smooth = sweep["smooth"]
        peak_ms, peak_fv = max(smooth, key=lambda point: point[1])
        raw_values = [value for _, value in sweep["points"]]
        intervals = [sweep["points"][i + 1][0] - sweep["points"][i][0]
                     for i in range(len(sweep["points"]) - 1)]
        direction_sign = -1 if sweep["direction"] == "near" else 1
        valid = (2000 <= peak_ms <= 4500 and
                 raw_values[0] < peak_fv * 0.75 and
                 raw_values[-1] < peak_fv * 0.75)
        for elapsed, value in smooth:
            nominal_position = direction_sign * (elapsed - 3000)
            processed.append((sweep["run"], sweep["direction"], elapsed,
                              nominal_position, elapsed - peak_ms, value,
                              value / peak_fv, int(valid)))
        summaries.append({
            "run": sweep["run"], "direction": sweep["direction"],
            "peak_ms": peak_ms, "peak_fv": peak_fv,
            "sample_ms": statistics.median(intervals),
            "start_fv": raw_values[0], "end_fv": raw_values[-1],
            "valid": valid,
        })

    with (ROOT / "processed.csv").open("w", newline="") as target:
        writer = csv.writer(target)
        writer.writerow(["run", "direction", "elapsed_ms", "nominal_position_ms",
                         "peak_relative_ms", "median5_fv", "normalized_fv", "valid"])
        writer.writerows(processed)

    raw_series = []
    aligned_series = []
    for summary in summaries:
        if not summary["valid"]:
            continue
        selected = [row for row in processed
                    if row[0] == summary["run"] and row[1] == summary["direction"]]
        color = colors[summary["direction"]]
        label = f'{summary["direction"]} {summary["run"]}'
        raw_series.append((label, color, [(row[3], row[5]) for row in selected]))
        aligned_series.append((label, color, [(row[4], row[6]) for row in selected]))

    max_fv = max(row[5] for row in processed if row[7])
    svg_plot(ROOT / "sweeps.svg", "P035 focus sweeps",
             "Nominal focus position from initial peak (motor ms)", raw_series,
             (-3200, 3200), (0, max_fv * 1.08))
    svg_plot(ROOT / "peak-aligned.svg", "P035 peak-aligned focus response",
             "Time from detected peak (ms)", aligned_series,
             (-2500, 2500), (0, 1.08))

    converter = shutil.which("rsvg-convert")
    if converter:
        for name in ("sweeps", "peak-aligned"):
            subprocess.run([converter, "-o", str(ROOT / f"{name}.png"),
                            str(ROOT / f"{name}.svg")], check=True)

    by_direction = defaultdict(list)
    valid_summaries = [summary for summary in summaries if summary["valid"]]
    for summary in valid_summaries:
        by_direction[summary["direction"]].append(summary)

    def stats(direction, field):
        values = [entry[field] for entry in by_direction[direction]]
        return statistics.mean(values), statistics.stdev(values)

    near_peak, near_sd = stats("near", "peak_ms")
    far_peak, far_sd = stats("far", "peak_ms")
    slack_low = min(near_peak, far_peak) - 3000
    slack_high = max(near_peak, far_peak) - 3000
    all_intervals = [entry["sample_ms"] for entry in valid_summaries]

    def peak_width(direction, fraction):
        bins = defaultdict(list)
        for row in processed:
            if row[1] == direction and row[7]:
                bins[round(row[4] / 100) * 100].append(row[6])
        curve = sorted((position, statistics.mean(values))
                       for position, values in bins.items())
        floor = statistics.median(value for position, value in curve
                                  if abs(position) >= 2200)
        target = floor + (1 - floor) * fraction
        left = max(position for position, value in curve
                   if position <= 0 and value <= target)
        right = min(position for position, value in curve
                    if position >= 0 and value <= target)
        return right - left

    width_90 = [peak_width(direction, 0.90) for direction in ("near", "far")]
    width_75 = [peak_width(direction, 0.75) for direction in ("near", "far")]
    width_50 = [peak_width(direction, 0.50) for direction in ("near", "far")]
    excluded = [summary for summary in summaries if not summary["valid"]]
    excluded_text = ("None." if not excluded else "\n".join(
        f'- Run {item["run"]} {item["direction"].upper()}: '
        f'peak at {item["peak_ms"]} ms'
        for item in excluded))
    report = f"""# P035 focus response capture

## Method

The camera ran autofocus before each measurement. Each test moved 3000 ms away
from focus and then swept 6000 ms through the peak. Four tests ran in each
direction. The capture contains {sum(len(run['points']) for run in runs)} raw focus values.

The driver lifecycle timestamps define movement start. Samples arrived every
{statistics.median(all_intervals):.0f} ms (median). Each plotted curve uses a five-sample rolling median.

## Results

![Focus sweeps](sweeps.svg)

| Sweep direction | Peak after sweep start, mean | Standard deviation | Implied reversal slack |
| --- | ---: | ---: | ---: |
| NEAR | {near_peak:.0f} ms | {near_sd:.0f} ms | {near_peak - 3000:.0f} ms |
| FAR | {far_peak:.0f} ms | {far_sd:.0f} ms | {far_peak - 3000:.0f} ms |

The valid response has one clear peak in both directions. The peak time repeats
well across the selected runs. Direction reversal adds about {slack_low:.0f} to
{slack_high:.0f} ms before the lens returns to the original focal point.

The normalized peak has these approximate direction ranges:

- 90 percent height: {min(width_90)} to {max(width_90)} ms
- 75 percent height: {min(width_75)} to {max(width_75)} ms
- 50 percent height: {min(width_50)} to {max(width_50)} ms

## Excluded sweeps

The analysis excludes a sweep if its peak is outside 2000 to 4500 ms. It also
excludes a sweep if either endpoint is more than 75 percent of its peak.

{excluded_text}

This result supports an online predictive focus controller. A useful first
model can estimate direction-specific reversal slack, metric slope, and the
remaining time to the local peak. It must still update the model during every
search because this camera gives no lens position or movement feedback.

This capture is not enough for a fixed global model. It covers one zoom level,
scene, and lighting condition. More captures must vary zoom, scene distance,
contrast, light level, and the initial focus error.

![Peak-aligned response](peak-aligned.svg)

## Repeat the capture

See [Focus Characterization](../../docs/focus-characterization.md) for the
camera commands and capture requirements.

## Files

- `raw.csv`: unmodified timestamped ISP values
- `events.log`: motor service lifecycle events
- `processed.csv`: smoothed, normalized, and peak-aligned values
- `sweeps.svg`: curves on the nominal motor-time position axis
- `peak-aligned.svg`: normalized curves aligned at each detected peak
- `sweeps.png` and `peak-aligned.png`: raster copies of the plots
- `tools/capture-focus-sweeps.sh`: camera-side capture tool
- `tools/analyze-focus-sweeps.py`: standard-library analysis and plot generator
"""
    (ROOT / "README.md").write_text(report)

    for summary in summaries:
        print(summary)


if __name__ == "__main__":
    main()
