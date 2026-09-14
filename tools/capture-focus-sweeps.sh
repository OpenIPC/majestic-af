#!/bin/sh
set -eu

SOCKET=/run/motorsd.sock
CSV=/tmp/p035-focus-sweeps.csv
EVENTS=/tmp/p035-focus-events.log
RUNS=${1:-4}

echo 'run,phase,direction,t_mono_ms,fv' > "$CSV"
: > "$EVENTS"

sample_once() {
    run=$1
    phase=$2
    direction=$3
    response=$(curl -fsS http://127.0.0.1/autofocus/status) || return
    case "$response" in
        metric\ t_mono_ms=*\ fv=*)
            stamp=${response#*t_mono_ms=}
            stamp=${stamp%% *}
            value=${response##*fv=}
            echo "$run,$phase,$direction,$stamp,$value" >> "$CSV"
            ;;
    esac
}

move_and_sample() {
    run=$1
    phase=$2
    direction=$3
    duration=$4
    lease=$((duration + 3000))

    motorsctl --socket "$SOCKET" --wait-event \
        '{"version":1,"id":"events","op":"subscribe"}' \
        "{\"version\":1,\"id\":\"lease\",\"op\":\"acquire\",\"role\":\"manual\",\"axis\":\"focus\",\"lease_ms\":$lease}" \
        "{\"version\":1,\"id\":\"move\",\"op\":\"move\",\"axis\":\"focus\",\"direction\":\"$direction\",\"duration_ms\":$duration}" \
        >> "$EVENTS" 2>&1 &
    move_pid=$!

    while kill -0 "$move_pid" 2>/dev/null; do
        sample_once "$run" "$phase" "$direction"
        usleep 20000
    done
    wait "$move_pid"
    sample_once "$run" "$phase" "$direction"
}

focus_first() {
    rm -f /tmp/af_metric_stream.on
    curl -fsS http://127.0.0.1/autofocus >/dev/null
    count=0
    while [ "$count" -lt 40 ]; do
        state=$(curl -fsS http://127.0.0.1/autofocus/status || true)
        case "$state" in
            running*) sleep 1 ;;
            *) echo "AF: $state"; return ;;
        esac
        count=$((count + 1))
    done
    echo 'AF timed out' >&2
    exit 1
}

trap 'rm -f /tmp/af_metric_stream.on' EXIT

i=1
while [ "$i" -le "$RUNS" ]; do
    echo "Run $i/$RUNS: FAR 3000 ms, then NEAR 6000 ms"
    focus_first
    touch /tmp/af_metric_stream.on
    move_and_sample "$i" defocus far 3000
    move_and_sample "$i" sweep near 6000

    echo "Run $i/$RUNS: NEAR 3000 ms, then FAR 6000 ms"
    focus_first
    touch /tmp/af_metric_stream.on
    move_and_sample "$i" defocus near 3000
    move_and_sample "$i" sweep far 6000
    i=$((i + 1))
done

echo "Saved $CSV"
