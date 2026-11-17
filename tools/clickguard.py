#!/usr/bin/env python3
"""clickguard -- listen to a lens or PTZ test through a camera's microphone and stop
it the moment the motor clicks.

A stepper that clicks is skipping or stalling: the count no longer matches the
lens, and the gear train is taking the blows. A run that clicks has already
failed, and every further click does damage, so the guard aborts on the first
one (--count raises that) -- it does not wait for the test to finish.

The microphone is any camera running majestic with audio enabled (`/audio.pcm`,
16-bit mono PCM at the camera's audio.srate), placed next to the device under
test. Nothing runs on that camera; the detector runs here.

    clickguard.py selftest --mic HOST               # plays clicks on the mic's speaker,
                                                    # proves they are heard (--ref-cmd: any
                                                    # command that clicks, instead)
    clickguard.py listen   --mic HOST               # print every click heard, until ^C
    clickguard.py run      --mic HOST --dut HOST -- CMD ...
                                                    # run CMD; abort it and the DUT's
                                                    # motors on a repeated click
    clickguard.py analyze  FILE.pcm --rate 48000    # the detector over a recording

`run` first runs `selftest` and starts nothing unless it passes (exit 3; skip with
--no-preflight). It exits 0 when CMD finished with no click, 2 when the guard
aborted it, and CMD's own status otherwise. Every run records the audio it
judged (`--save`), so a verdict can be listened to afterwards.

Abort, by default: POST /ptz?move=stop to the DUT and, at the same moment,
`killall -9 majestic` over an ssh connection opened before CMD started, after
killing the init script's `supervise` loop, which would otherwise start majestic
again -- and a fresh majestic homes its lens into the stops -- and then disarming
the hardware watchdog majestic held, which would otherwise reset the board. The stop alone is
not enough -- the ms41908 backend ignores a stop while it homes -- and a
killed majestic sends no more VD pulses, so no motor on its chip turns again.
"""

import argparse
import base64
import collections
import math
import os
import signal
import subprocess
import sys
import threading
import time
import urllib.request

import numpy as np
from scipy import signal as sps

# ---- the detector ----------------------------------------------------------------

HP_HZ = 2000.0        # motor clicks are broadband ticks; room hum and voice live below
FRAME_MS = 1.0        # envelope resolution
FLOOR_S = 1.5         # the noise floor is the median envelope over this much history
RATIO = 6.0           # a click stands this far above the floor (~16 dB) ...
ABS_MIN = 150.0       # ... and above this, in high-passed sample units, whatever the
                      # room: the lab mic hears the room tick by itself at up to ~94
RISE = 3.0            # and rises this sharply over the frames just before it
MAX_LEN_MS = 40       # and is over within this: longer is a sound, not a click
REFRACTORY_MS = 25    # one click is one click, however it rings


class Detector:
    """Streaming impulsive-transient detector. feed() samples, get clicks back as
    (time_s, level_over_floor) -- time in seconds of stream."""

    def __init__(self, rate, abs_min=ABS_MIN):
        self.rate = rate
        self.abs_min = abs_min
        self.sos = sps.butter(4, HP_HZ, "highpass", fs=rate, output="sos")
        self.zi = sps.sosfilt_zi(self.sos) * 0.0
        self.flen = max(1, int(rate * FRAME_MS / 1000))
        self.pending = np.zeros(0)
        self.hist = collections.deque(maxlen=int(FLOOR_S * 1000 / FRAME_MS))
        self.recent = collections.deque(maxlen=5)
        self.frame_no = 0
        self.floor = None
        self.open = None          # (start_frame, peak) of a candidate being measured
        self.quiet_until = -1
        self.tops = []            # the absolute peak of every click reported

    def feed(self, samples):
        if len(samples) == 0:
            return []
        y, self.zi = sps.sosfilt(self.sos, samples, zi=self.zi)
        y = np.concatenate([self.pending, y])
        n = len(y) // self.flen
        self.pending = y[n * self.flen:]
        clicks = []
        for peak in np.abs(y[: n * self.flen]).reshape(n, self.flen).max(axis=1):
            clicks.extend(self._frame(float(peak)))
        return clicks

    def _frame(self, peak):
        out = []
        f = self.frame_no
        self.frame_no += 1
        if self.floor is None or f % 20 == 0:
            self.floor = float(np.median(self.hist)) if self.hist else peak
        thr = max(self.floor * RATIO, self.abs_min)
        if self.open is not None:
            start, top = self.open
            if peak >= thr / 2:
                self.open = (start, max(top, peak))
                if (f - start) * FRAME_MS > MAX_LEN_MS:
                    self.open = None          # a sustained sound, not a click
                    self.quiet_until = f + int(REFRACTORY_MS / FRAME_MS)
            else:
                out.append((start * FRAME_MS / 1000.0, top / max(self.floor, 1e-9)))
                self.tops.append(top)
                self.open = None
                self.quiet_until = f + int(REFRACTORY_MS / FRAME_MS)
        elif f >= self.quiet_until and peak > thr:
            before = max(self.recent) if self.recent else 0.0
            if peak > RISE * max(before, self.floor):
                self.open = (f, peak)
        # A candidate's own frames stay out of the floor, so a click does not raise
        # the bar for the next one.
        if self.open is None:
            self.hist.append(peak)
        self.recent.append(peak)
        return out


class Verdict:
    """Fails on `count` clicks within `window` seconds."""

    def __init__(self, count, window):
        self.count, self.window = count, window
        self.times = collections.deque()

    def add(self, t):
        self.times.append(t)
        while self.times and t - self.times[0] > self.window:
            self.times.popleft()
        return len(self.times) >= self.count


CALIBRATE_S = 10.0    # quiet room listened to before a run
MARGIN = 2.0          # the bar stands this far over the loudest background tick: a
                      # short listen does not hear the room's loudest, only a sample


def calibrate(gen, rate, first=None, seconds=CALIBRATE_S):
    """Listen to the room with the device under test idle, and return the bar a click
    has to clear: MARGIN over the loudest transient the room made by itself. A room
    that ticks at -50 dBFS on its own (the lab does) would otherwise fail every run."""
    det = Detector(rate)
    got = 0
    if first is not None:
        det.feed(first)
        got += len(first)
    for x in gen:
        det.feed(x)
        got += len(x)
        if got >= seconds * rate:
            break
    loudest = max(det.tops, default=0.0)
    return max(ABS_MIN, loudest * MARGIN), len(det.tops), loudest


# ---- the microphone --------------------------------------------------------------

def _auth(user, password):
    return "Basic " + base64.b64encode(f"{user}:{password}".encode()).decode()


class Mic:
    """/audio.pcm from a majestic camera. The rate is measured from the stream (bytes
    per second, snapped to a standard rate) unless given."""

    RATES = (8000, 11025, 16000, 22050, 24000, 32000, 44100, 48000)

    def __init__(self, host, user, password, rate=None):
        req = urllib.request.Request(f"http://{host}/audio.pcm",
                                     headers={"Authorization": _auth(user, password)})
        self.resp = urllib.request.urlopen(req, timeout=10)
        self.rate = rate
        self.t0 = None
        self.got = 0
        self.save = None

    def chunks(self):
        if self.rate is None:
            buf, t0 = b"", time.monotonic()
            while time.monotonic() - t0 < 1.5:
                buf += self.resp.read1(65536)
            bps = len(buf) / (time.monotonic() - t0) / 2
            self.rate = min(self.RATES, key=lambda r: abs(r - bps))
            first = buf
        else:
            first = b""
        self.t0 = time.monotonic()
        odd = b""
        data = first
        while True:
            if not data:
                data = self.resp.read1(65536)
                if not data:
                    return
            data, odd = odd + data, b""
            if len(data) % 2:
                data, odd = data[:-1], data[-1:]
            if not data:
                continue
            if self.save:
                self.save.write(data)
            self.got += len(data) // 2
            yield np.frombuffer(data, dtype="<i2").astype(np.float64)
            data = b""


# ---- the device under test -------------------------------------------------------

class Dut:
    """What an abort does to the device under test. The ssh connection is opened up
    front (ControlMaster), so an abort costs one round trip, not a handshake."""

    def __init__(self, host, user, password, ssh_user):
        self.host, self.user, self.password = host, user, password
        self.ctl = f"/tmp/clickguard-{os.getpid()}.ssh"
        self.ssh = ["ssh", "-o", "ControlPath=" + self.ctl, "-o", "BatchMode=yes",
                    f"{ssh_user}@{host}"]
        self.master = subprocess.Popen(
            ["ssh", "-o", "ControlMaster=yes", "-o", "ControlPath=" + self.ctl,
             "-o", "BatchMode=yes", "-N", f"{ssh_user}@{host}"])
        for _ in range(50):
            if os.path.exists(self.ctl):
                break
            time.sleep(0.1)

    def stop_http(self):
        try:
            req = urllib.request.Request(f"http://{self.host}/ptz?move=stop", method="POST",
                                         headers={"Authorization": _auth(self.user, self.password)})
            urllib.request.urlopen(req, timeout=2).read()
            return "stop sent"
        except Exception as e:  # noqa: BLE001 -- the kill below is the backstop
            return f"stop failed: {e}"

    # The supervisor first: newer init scripts run `S95majestic supervise`, which
    # restarts a majestic that died -- and a restarted majestic homes its lens, into
    # the stops, with nothing listening. Then majestic; then a look a moment later
    # that it stayed down.
    # Then the watchdog: majestic holds it, and a SIGKILL leaves it armed, so the
    # board resets a minute or so later and boots majestic again -- which homes, with
    # nothing listening. The magic close (`V`) disarms it where the driver allows,
    # as the HiSilicon one does with nowayout=0.
    KILL = ("for p in $(ps | awk '/[S]95majestic supervise/ {print $1}'); do kill -9 $p; done; "
            "killall -9 majestic; sleep 0.2; "
            "[ -c /dev/watchdog ] && printf V > /dev/watchdog 2>/dev/null; echo killed")
    CHECK = "if pidof majestic >/dev/null; then echo RESPAWNED; else echo down; fi"

    def kill(self):
        r = subprocess.run(self.ssh + [self.KILL], capture_output=True, text=True, timeout=8)
        return (r.stdout + r.stderr).strip()

    def check(self):
        r = subprocess.run(self.ssh + [self.CHECK], capture_output=True, text=True, timeout=8)
        return (r.stdout + r.stderr).strip()

    def abort(self):
        out = {}
        th = threading.Thread(target=lambda: out.__setitem__("stop", self.stop_http()))
        th.start()
        out["kill"] = self.kill()
        th.join(3)
        return out

    def close(self):
        self.master.terminate()
        try:
            self.master.wait(3)
        except subprocess.TimeoutExpired:
            self.master.kill()


# ---- commands --------------------------------------------------------------------

def _stamp():
    return time.strftime("%H:%M:%S")


def guard(mic, verdict, on_click, stop_flag):
    """Run the detector over the mic until stop_flag is set or the verdict fails.
    Returns the clicks heard."""
    det = None
    heard = []
    for x in mic.chunks():
        if det is None:
            det = Detector(mic.rate)
        for t, lvl in det.feed(x):
            heard.append((t, lvl))
            failed = verdict.add(t)
            on_click(t, lvl, failed)
            if failed:
                return heard, True
        if stop_flag.is_set():
            break
    return heard, False


def cmd_listen(a):
    mic = Mic(a.mic, a.user, a.password, a.rate)
    if a.save:
        mic.save = open(a.save, "wb")
    stop = threading.Event()

    def on_click(t, lvl, failed):
        print(f"{_stamp()} click at {t:8.3f}s  {20 * math.log10(lvl):5.1f} dB over floor"
              + ("  REPEATED" if failed else ""), flush=True)

    print(f"listening on {a.mic} (rate measured from the stream)", flush=True)
    try:
        while True:
            heard, failed = guard(mic, Verdict(a.count, a.window), on_click, stop)
            if not failed:
                break
    except KeyboardInterrupt:
        pass
    return 0


def _clicks_pcm(rate, times, amp, quiet_s=1.0):
    """Raw s16le mono -- what /play_audio takes at the device rate -- of silence with a
    short broadband tick at each of `times`."""
    n = int(rate * (max(times) + quiet_s))
    x = np.zeros(n)
    m = int(rate * 0.0015)
    tick = np.random.default_rng(1).uniform(-1, 1, m) * np.hanning(m)
    for t in times:
        i = int(t * rate)
        x[i:i + m] += tick
    return (np.clip(x * amp, -1, 1) * 32767).astype("<i2").tobytes()


def cmd_selftest(a):
    """Calibrate on the room, then make a known sound and require the detector to hear it
    -- the guard is only worth running when it is known to hear.

    By default the microphone camera plays a tick train on its own speaker, which
    proves the microphone, the stream and the detector. --ref-cmd takes any command that
    makes a known number of clicks instead -- something at the camera under test, to
    prove the microphone hears what happens THERE."""
    src = a.mic
    mic = Mic(a.mic, a.user, a.password, a.rate)
    if a.save:
        mic.save = open(a.save, "wb")
    gen = mic.chunks()
    first = next(gen)
    rate = mic.rate
    bar, n_bg, loudest = calibrate(gen, rate, first, a.calibrate)
    bar = a.bar or bar
    print(f"rate {rate} Hz; room: {n_bg} background tick(s), loudest {loudest:.0f}; "
          f"bar {bar:.0f}")
    det = Detector(rate, bar)
    heard = []

    if a.ref_cmd:
        # A reference the harness cannot time: count what it hears while the command
        # runs, and for a moment after (the stream is a fraction of a second behind).
        # Exactly that many, while it runs (and the stream's lag after): one more is a
        # sound that is not the reference, and a reference heard as more than it made
        # proves nothing about which was which. A reference that hangs fails.
        print(f"reference: {a.ref_cmd} ({a.ref_clicks} click(s) expected)")
        proc = subprocess.Popen(a.ref_cmd, shell=True, start_new_session=True)
        t_end, t_limit, stuck = None, mic.got / rate + a.ref_timeout, False
        try:
            for x in gen:
                heard.extend(det.feed(x))
                now_s = mic.got / rate
                if t_end is None and proc.poll() is not None:
                    t_end = now_s + 2.0
                if t_end is None and now_s >= t_limit:
                    stuck = True
                    break
                if t_end is not None and now_s >= t_end:
                    break
        finally:
            if proc.poll() is None:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
        levels = ", ".join(f"{p:.0f}" for p in det.tops)
        print(f"heard {len(heard)} click(s); levels {levels or '-'}")
        if stuck:
            print(f"SELFTEST FAIL: the reference command ran past {a.ref_timeout:g} s")
            return 1
        if len(heard) != a.ref_clicks:
            print(f"SELFTEST FAIL: the reference makes {a.ref_clicks}, the microphone heard "
                  f"{len(heard)} -- " + ("move it closer, or raise its volume"
                                         if len(heard) < a.ref_clicks
                                         else "something else clicked too"))
            return 1
        print(f"SELFTEST PASS (quietest {min(det.tops) / bar:.1f}x the bar)")
        return 0

    ticks = [1.0, 1.4, 1.8, 3.0, 3.25, 4.5]
    pcm = _clicks_pcm(rate, ticks, a.amp)
    print(f"reference: {len(ticks)} ticks on {src}'s speaker, amplitude {a.amp:g} -- this proves "
          f"the microphone, not where it stands; --ref-cmd at the device under test does")

    def play():
        # The rate is declared, so a speaker that runs at another rate resamples
        # rather than playing the train at the wrong speed.
        req = urllib.request.Request(f"http://{src}/play_audio", data=pcm, method="POST",
                                     headers={"Authorization": _auth(a.user, a.password),
                                              # raw s16le, not audio/L16: that is
                                              # big-endian by RFC 2586
                                              "Content-Type": f"application/octet-stream; rate={rate}; channels=1"})
        try:
            urllib.request.urlopen(req, timeout=30).read()
        except Exception as e:  # noqa: BLE001
            print(f"play_audio on {src}: {e}", file=sys.stderr)

    t_play = mic.got / rate
    th = threading.Thread(target=play)
    th.start()
    for x in gen:
        heard.extend(det.feed(x))
        if mic.got / rate > t_play + max(ticks) + 3.0:
            break
    th.join(5)
    got = [t for t, _ in heard]
    lv = det.tops
    # The played train, found by its spacing: the speaker's amplifier pops as it
    # switches on and off, and those are real sounds this must hear as well.
    best = []
    for anchor in got:
        hit = []
        for tk in ticks:
            want = anchor + tk - ticks[0]
            near = [i for i, g in enumerate(got) if abs(g - want) < 0.03]
            if near:
                hit.append(near[0])
        if len(hit) > len(best):
            best = hit
    extra = [i for i in range(len(got)) if i not in best]
    print(f"heard {len(best)} of the {len(ticks)} ticks, levels "
          + ", ".join(f"{lv[i]:.0f}" for i in best))
    if extra:
        print("also heard (the speaker switching, or the room): "
              + ", ".join(f"{lv[i]:.0f}" for i in extra))
    if len(best) != len(ticks):
        print(f"SELFTEST FAIL: heard {len(best)} of the {len(ticks)} ticks played")
        return 1
    print(f"SELFTEST PASS (quietest {min(lv[i] for i in best) / bar:.1f}x the bar)")
    return 0


def cmd_play(a):
    """Play a tick train on a camera's speaker -- a stand-in for a clicking motor, to
    prove `run` aborts."""
    times = [1.0 + i * a.every for i in range(a.ticks)]
    pcm = _clicks_pcm(a.rate or 48000, times, a.amp)
    req = urllib.request.Request(f"http://{a.mic}/play_audio", data=pcm, method="POST",
                                 headers={"Authorization": _auth(a.user, a.password),
                                          "Content-Type": "application/octet-stream"})
    urllib.request.urlopen(req, timeout=60).read()
    return 0


def cmd_run(a):
    if not a.cmd:
        print("run: give the test command after --", file=sys.stderr)
        return 64
    if a.preflight:
        # Nothing moves until the microphone has been heard to hear: a guard on a dead
        # or muted microphone, or one moved out of earshot, passes every test.
        # The run's own rate, bar and calibration, so the preflight judges with the
        # detector the run will use. The password goes through the environment, as it
        # came, not onto a command line any local process can read.
        pre = [sys.executable, os.path.abspath(__file__), "selftest", "--mic", a.mic,
               "--user", a.user, "--calibrate", str(a.calibrate)]
        if a.rate:
            pre += ["--rate", str(a.rate)]
        if a.bar:
            pre += ["--bar", str(a.bar)]
        if a.ref_cmd:
            pre += ["--ref-cmd", a.ref_cmd, "--ref-clicks", str(a.ref_clicks),
                    "--ref-timeout", str(a.ref_timeout)]
        env = dict(os.environ, CLICKGUARD_PASSWORD=a.password)
        r = subprocess.run(pre, capture_output=True, text=True, env=env)
        verdict = (r.stdout.strip().splitlines() or ["(no output)"])[-1]
        print(f"{_stamp()} clickguard: preflight {verdict}", flush=True)
        if r.returncode != 0:
            print(r.stdout + r.stderr, end="", file=sys.stderr)
            print(f"{_stamp()} clickguard: not starting -- the microphone did not hear the "
                  f"reference", flush=True)
            return 3
    dut = Dut(a.dut, a.user, a.password, a.ssh_user) if a.dut else None
    try:
        return _run(a, dut)
    finally:
        if dut:
            dut.close()


def _run(a, dut):
    mic = Mic(a.mic, a.user, a.password, a.rate)
    if a.save:
        mic.save = open(a.save, "wb")
    gen = mic.chunks()
    first = next(gen)
    bar, n_bg, loudest = calibrate(gen, mic.rate, first, a.calibrate)
    bar = a.bar or bar
    print(f"{_stamp()} clickguard: room {n_bg} background tick(s), loudest {loudest:.0f}; "
          f"bar {bar:.0f}", flush=True)
    det = Detector(mic.rate, bar)
    # The floor needs a moment of the room before the test makes any noise.
    t_end = mic.got / mic.rate + FLOOR_S
    for x in gen:
        det.feed(x)
        if mic.got / mic.rate >= t_end:
            break
    verdict = Verdict(a.count, a.window)
    proc = subprocess.Popen(a.cmd, start_new_session=True)
    print(f"{_stamp()} clickguard: {mic.rate} Hz from {a.mic}; running {' '.join(a.cmd)}",
          flush=True)
    failed, heard, why = False, [], None
    try:
        for x in gen:
            for t, lvl in det.feed(x):
                heard.append(t)
                print(f"{_stamp()} clickguard: click {20 * math.log10(lvl):.1f} dB over floor",
                      flush=True)
                if verdict.add(t):
                    failed = True
                    why = f"{a.count} click(s) within {a.window:g} s"
                    break
            if failed or proc.poll() is not None:
                break
        else:
            # The stream ended under a test still running: nothing is listening any
            # more, and a test nobody hears is not allowed to go on.
            if proc.poll() is None:
                failed, why = True, "the microphone stream ended"
    except Exception as e:  # noqa: BLE001 -- any way of losing the ear stops the test
        failed, why = True, f"the listener failed: {e!r}"
    if failed:
        t0 = time.monotonic()
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        res = dut.abort() if dut else {}
        took = (time.monotonic() - t0) * 1000
        if dut:
            time.sleep(1.0)
            res["after 1 s"] = dut.check()
        print(f"{_stamp()} clickguard: FAIL -- {why}; "
              f"test killed, DUT {res}; abort took {took:.0f} ms", flush=True)
        rc = 2
    else:
        rc = proc.wait()
        print(f"{_stamp()} clickguard: {'PASS' if rc == 0 else 'test failed'} -- "
              f"{len(heard)} click(s) heard, none repeated", flush=True)
    return rc


def cmd_analyze(a):
    x = np.fromfile(a.file, dtype="<i2").astype(np.float64)
    det = Detector(a.rate)
    v = Verdict(a.count, a.window)
    for t, lvl in det.feed(x):
        print(f"click at {t:8.3f}s  {20 * math.log10(lvl):5.1f} dB over floor"
              + ("  REPEATED" if v.add(t) else ""))
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sub = p.add_subparsers(dest="what", required=True)
    for name in ("listen", "selftest", "run", "analyze", "play"):
        s = sub.add_parser(name)
        if name == "analyze":
            s.add_argument("file")
            s.add_argument("--rate", type=int, required=True)
        else:
            s.add_argument("--mic", required=True, help="camera whose microphone listens")
            s.add_argument("--user", default="root")
            s.add_argument("--password", default=os.environ.get("CLICKGUARD_PASSWORD", "12345"))
            s.add_argument("--rate", type=int, help="the mic's sample rate (measured if not)")
            s.add_argument("--save", help="write the PCM judged here")
            s.add_argument("--bar", type=float,
                           help="the click level to clear, instead of calibrating on the room")
        s.add_argument("--count", type=int, default=1,
                       help="clicks that fail a run: the first, unless told otherwise")
        s.add_argument("--window", type=float, default=2.0, help="... within this many s")
        if name in ("selftest", "play"):
            s.add_argument("--amp", type=float, default=0.02, help="tick amplitude, 0..1")
        if name == "selftest":
            s.add_argument("--ref-cmd", help="instead of the speaker: a command that clicks")
            s.add_argument("--ref-clicks", type=int, default=1,
                           help="exactly how many clicks --ref-cmd makes")
            s.add_argument("--ref-timeout", type=float, default=60.0)
            s.add_argument("--calibrate", type=float, default=CALIBRATE_S)
        if name == "play":
            s.add_argument("--every", type=float, default=0.5)
            s.add_argument("--ticks", type=int, default=10)
        if name == "run":
            s.add_argument("--no-preflight", dest="preflight", action="store_false",
                           help="skip the selftest that has to pass before the test starts")
            s.add_argument("--ref-cmd", help="the preflight's reference: a command that clicks")
            s.add_argument("--ref-clicks", type=int, default=1)
            s.add_argument("--ref-timeout", type=float, default=60.0)
            s.add_argument("--calibrate", type=float, default=CALIBRATE_S,
                           help="seconds of quiet room before the test starts")
            s.add_argument("--dut", help="majestic camera to stop on a fail")
            s.add_argument("--ssh-user", default="root")
            s.add_argument("cmd", nargs=argparse.REMAINDER)
    a = p.parse_args()
    if getattr(a, "cmd", None) and a.cmd[0] == "--":
        a.cmd = a.cmd[1:]
    return {"listen": cmd_listen, "selftest": cmd_selftest, "run": cmd_run,
            "analyze": cmd_analyze, "play": cmd_play}[a.what](a)


if __name__ == "__main__":
    sys.exit(main())
