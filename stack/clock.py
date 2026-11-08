"""The stack's clock: the time of the world the robot lives in. Not a tier of the robot's stack: a detail of where it
runs, like the Link.

Every tier counts the world's time here, never with the time module: how long a move may take, how old a sighting is,
how long to wait for the camera or for an adult. Only the calls to a model and their waits stay on real time (a
provider answers when it answers), and say so where they are.

ROBOT_CLOCK chooses the clock:
- "real" (the default, and always with the real robot): the time module;
- "stepped": the simulator and the stack in one process (sim/headless.py). The world's time only moves when the stack
  waits: each wait makes the simulated world step forward until the answer comes or the wait is over. As fast as the
  computer can go, and the same run again with the same seed.
"""
import datetime
import os
import select
import time

MODE = os.environ.get("ROBOT_CLOCK", "real")
if MODE not in ("real", "stepped"):
    raise SystemExit(f"ROBOT_CLOCK={MODE!r}: real or stepped")

_world = {"step": None, "tick_s": 0.01, "seconds": 0.0, "epoch": time.time()}


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


def monotonic():
    """Seconds, for durations: how long something took, how long a wait may still go on."""
    return time.monotonic() if MODE == "real" else _world["seconds"]


def now():
    """Seconds since 1970, for the age of what the robot knows (a sighting, a free cell)."""
    return time.time() if MODE == "real" else _world["epoch"] + _world["seconds"]


def sleep(seconds):
    if MODE == "real":
        time.sleep(seconds)
    else:
        _advance(seconds)


def stamp(fmt="%H:%M:%S", seconds_ago=0.0):
    """The world's time as people read it in the journals: now, or that many seconds ago."""
    return datetime.datetime.fromtimestamp(now() - seconds_ago).strftime(fmt)


def iso():
    return datetime.datetime.fromtimestamp(now()).isoformat(timespec="seconds")


def wait_for(ready, timeout):
    """True as soon as ready() is true, within `timeout` seconds of the world's time; with 0, look once."""
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
    return wait_for(lambda: bool(select.select([fd], [], [], 0)[0]), timeout)
