"""The stack's clock: the time of the world the robot lives in. Not a tier of the robot's stack: a detail of where it
runs, like the Link.

Every tier counts the world's time here, never with the time module: how long a move may take, how old a sighting is,
how long to wait for the camera or for an adult. Only the calls to a model and their waits stay on real time (a
provider answers when it answers), and say so where they are.

ROBOT_CLOCK chooses the clock:
- "real" (the default, and always with the real robot): the time module;
- "stepped": the simulator and the stack in one process (sim/headless.py). The world's time only moves when the stack
  waits: each wait makes the simulated world step forward until the answer comes or the wait is over. As fast as the
  computer can go, and the same run again with the same seed;
- "sim": the time the simulator publishes in .robot-sim/clock (sim/robot_sim.py, which may run k times faster than
  real time: --speed, or the page). The stack waits for the world's time, not the computer's.

While a model thinks (a call to its provider), the world goes on: in real time by itself; stepped, ROBOT_THINK says by
how much: "real" (the default: as long as the call that answered took; the waits before trying a call again do not
count, a session with a key of its own does not have them), "zero" (the world waits for the model), a number of
seconds (the same for every call, to compare models whatever their speed), or "file:<path>", a JSON list of seconds,
one per call of the model in order (a run played again, sim/headless.py --replay).
"""
import contextlib
import datetime
import json
import os
import select
import struct
import time
from pathlib import Path

MODE = os.environ.get("ROBOT_CLOCK", "real")
if MODE not in ("real", "stepped", "sim"):
    raise SystemExit(f"ROBOT_CLOCK={MODE!r}: real, stepped or sim")
# What the simulator publishes, at each tick: the world's seconds since it started (never going back, even after
# "Start again"), the date the world started at (seconds since 1970), and how many times faster than real time it runs.
RECORD = struct.Struct("<ddd")
STALE_S = 5.0  # real seconds without the world's time moving: the simulator is gone, say so rather than wait forever

_world = {"step": None, "tick_s": 0.01, "seconds": 0.0, "epoch": time.time()}
THINK = os.environ.get("ROBOT_THINK", "real")
_think_list = None  # ROBOT_THINK=file:<path>: the seconds of each call, in order
if THINK.startswith("file:"):
    try:
        _think_list = [float(x) for x in json.loads(Path(THINK[5:]).read_text())]
    except (OSError, ValueError, TypeError):
        raise SystemExit(f"ROBOT_THINK={THINK!r}: no list of seconds can be read there") from None
elif THINK not in ("real", "zero"):
    try:
        float(THINK)
    except ValueError:
        raise SystemExit(f"ROBOT_THINK={THINK!r}: real, zero, a number of seconds or file:<path>") from None


def drive_with(step, tick_s, epoch=None):
    """Stepped clock: step(dt) makes the simulated world move forward by dt seconds (sim/headless.py). epoch: the date
    the world starts at, in seconds since 1970 (a fixed one makes the journals of two runs identical)."""
    _world.update(step=step, tick_s=tick_s, seconds=0.0, epoch=time.time() if epoch is None else epoch)


def _tick():
    """Stepped clock: the world moves forward by one tick."""
    if _world["step"] is None:
        raise RuntimeError("ROBOT_CLOCK=stepped, but no simulated world drives the clock (sim/headless.py)")
    _world["step"](_world["tick_s"])
    _world["seconds"] += _world["tick_s"]


def _advance(seconds):
    """Stepped clock: the world moves forward by `seconds`, one tick at a time."""
    end = _world["seconds"] + max(0.0, seconds)
    while _world["seconds"] < end - 1e-9:
        _tick()


_sim = {"fd": None, "seconds": None, "moved_at": 0.0}


def _published():
    """Sim clock: (world seconds, date the world started at, speed factor), as the simulator last wrote them."""
    path = Path(os.environ.get("ROBOT_STATE_DIR", Path(__file__).resolve().parent.parent / ".robot-sim")) / "clock"
    for attempt in (1, 2):
        if _sim["fd"] is None:
            try:
                _sim["fd"] = os.open(path, os.O_RDONLY)
            except OSError:
                raise RuntimeError(f"ROBOT_CLOCK=sim, but no simulator publishes its time ({path})") from None
        data = os.pread(_sim["fd"], RECORD.size, 0)
        if len(data) == RECORD.size:
            seconds, epoch, factor = RECORD.unpack(data)
            if seconds != _sim["seconds"]:
                _sim.update(seconds=seconds, moved_at=time.monotonic())
                return seconds, epoch, factor
            if time.monotonic() - _sim["moved_at"] < STALE_S:
                return seconds, epoch, factor
        os.close(_sim["fd"])  # a simulator started again writes a new file: open it once more
        _sim["fd"] = None
    raise RuntimeError("ROBOT_CLOCK=sim: the simulator's time has not moved for "
                       f"{STALE_S:.0f} s; is the simulator still running?")


def _sim_wait(ready, seconds):
    """Sim clock: True as soon as ready() is, within `seconds` of the world's time; real time goes by in small naps."""
    start, _, factor = _published()
    end = start + seconds
    while True:
        if ready():
            return True
        world, _, factor = _published()
        if world >= end - 1e-9:
            return bool(ready())
        time.sleep(min(0.005, max(0.0005, (end - world) / max(factor, 1e-3))))


def monotonic():
    """Seconds, for durations: how long something took, how long a wait may still go on."""
    if MODE == "real":
        return time.monotonic()
    if MODE == "sim":
        return _published()[0]
    return _world["seconds"]


def now():
    """Seconds since 1970, for the age of what the robot knows (a sighting, a free cell)."""
    if MODE == "real":
        return time.time()
    if MODE == "sim":
        seconds, epoch, _ = _published()
        return epoch + seconds
    return _world["epoch"] + _world["seconds"]


def sleep(seconds):
    if MODE == "real":
        time.sleep(seconds)
    elif MODE == "sim":
        _sim_wait(lambda: False, seconds)
    else:
        _advance(seconds)


def stamp(fmt="%H:%M:%S", seconds_ago=0.0):
    """The world's time as people read it in the journals: now, or that many seconds ago."""
    return datetime.datetime.fromtimestamp(now() - seconds_ago).strftime(fmt)


def iso():
    return datetime.datetime.fromtimestamp(now()).isoformat(timespec="seconds")


def wait_for(ready, timeout):
    """True as soon as ready() is true, within `timeout` seconds of the world's time; with 0, look once."""
    if MODE == "sim":
        return _sim_wait(ready, timeout)
    if MODE == "real":
        end = time.monotonic() + timeout
        while not ready():
            if time.monotonic() >= end:
                return False
            time.sleep(0.001)
        return True
    end = _world["seconds"] + timeout
    while True:
        if ready():
            return True
        if timeout <= 0:
            return False
        # A wait longer than nothing lets the world move at least one tick: a caller left with a few nanoseconds to
        # wait (rounding) would otherwise ask again and again while the world stands still.
        _tick()
        if _world["seconds"] >= end - 1e-9:
            return bool(ready())


def readable(fd, timeout):
    """True when fd has something to read before `timeout` seconds of the world's time; with 0, look once."""
    if MODE == "real":
        ready, _, _ = select.select([fd], [], [], timeout)
        return bool(ready)
    return wait_for(lambda: bool(select.select([fd], [], [], 0)[0]), timeout)  # stepped or sim: the world's time


def _thought(took):
    """Stepped clock: how far the world goes on for one call of the model (ROBOT_THINK, above)."""
    if THINK == "zero":
        return 0.0
    if THINK == "real":
        return took["answered_s"] if took["answered_s"] is not None else took["seconds"]
    if _think_list is not None:  # more calls than seconds (a replay that went another way): the world waits
        return _think_list.pop(0) if _think_list else 0.0
    return float(THINK)


@contextlib.contextmanager
def thinking():
    """Around a call to a model: what the world does meanwhile (ROBOT_THINK, above). In "took": "seconds", how long the
    call took in real time, waits before a new try included; "answered_s", how long the try that answered took (set by
    the caller, strategist.api_model); "world_s", how far the world went on."""
    took = {"seconds": 0.0, "answered_s": None, "world_s": None}
    start = time.monotonic()  # real time: how long the provider takes
    try:
        yield took
    finally:
        took["seconds"] = time.monotonic() - start
        if MODE == "stepped":
            took["world_s"] = _thought(took)
            _advance(took["world_s"])
        elif MODE == "real":
            took["world_s"] = took["seconds"]  # the world went on by itself, as long as the call took
