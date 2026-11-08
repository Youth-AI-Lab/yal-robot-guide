"""One mission in the simulator, without the page, as fast as the computer goes: the simulated robot and the whole
stack in one process, on the stepped clock of stack/clock.py.

The world's time only moves when the stack waits (for an answer of the robot, for the camera, for an adult): each
wait makes the simulated world step forward, 10 ms at a time, until the answer comes or the wait is over. Nothing else
changes: the same firmware, the same Reflex, the same Referee, Cartographer, Pilot and Strategist, the same lines of
the robot's protocol between them. Only the cable differs: a wire in memory instead of the pseudo-terminal, because
Linux hands a pseudo-terminal's bytes over in the background, sometimes a tick late on a busy computer, and the run
would then not be the same again. The same seed gives the same mission again, line for line.

A simulated adult stands by the mat: when the robot waits for one, it puts the robot back where the Cartographer asks
and presses Resume, after --adult-s seconds of the world's time. Each mission has a limit in the world's time
(--limit-s): then the mission is told to stop, as the Interpreter would; if it still does not end, it is cut.

Usage (the folder of the guide is the current folder):
    python3 -I sim/headless.py --mission mission.json [--model none|fake] [--world sim/worlds/forest.json] [--seed 1]
A model of a provider works too (its key on stdin, as for stack/strategist.py). While it thinks, the world goes on
(--think): as long as each call took (real, the default), not at all (zero), or a fixed number of seconds per call.

Prints one line of JSON: the report of the mission, and how long it took in the world and on the computer. The
mission's files are in --out (by default a new folder in .robot-sim/headless/).
"""
import argparse
import collections
import datetime
import importlib.util
import json
import os
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
GUIDE = HERE.parent
# The date the world starts at: fixed, so that the journals of two runs with the same seed are the same.
EPOCH = datetime.datetime(2026, 1, 1, 9, 0).timestamp()


class Wire:
    """The cable in memory, both ends: what the stack sends waits in to_robot, what the robot says in to_stack. Read
    at once by the other end on its next look, never a tick late."""

    def __init__(self):
        self.to_robot, self.to_stack = bytearray(), bytearray()


class RobotEnd:
    """The robot's end of the wire, in place of the simulator's pseudo-terminal (sim/robot_sim.py, Link)."""

    def __init__(self, wire, log):
        self.wire, self.log = wire, log
        self.path, self.plugged, self.buffer = "memory", True, b""
        self.received = collections.Counter()

    def read_lines(self, now):
        data = bytes(self.wire.to_robot)
        self.wire.to_robot.clear()
        if not self.plugged:
            self.buffer = b""
            return []
        self.buffer += data
        lines = []
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            text = line.decode("ascii", "replace").strip()
            if text:
                lines.append(text)
                self.received[text.split()[0]] += 1
                self.log.append({"t": round(now, 2), "dir": "in", "line": text})
        return lines

    def write_line(self, text, now):
        if not self.plugged:
            self.log.append({"t": round(now, 2), "dir": "lost", "line": text})
            return
        self.log.append({"t": round(now, 2), "dir": "out", "line": text})
        self.wire.to_stack += (text + "\n").encode("ascii")


def stack_end(link, clock, wire):
    """The stack's end of the wire: the Link of stack/link.py, with its cable replaced (connect, send, read_line);
    request() and drain() are its own."""

    class MemoryLink(link.Link):
        def port(self):
            return "memory"

        def connect(self):
            if self.fd is not None:
                return
            self.fd, self.path, self.buffer = "memory", "memory", b""
            self.drain()
            reply = self.request("HELLO")
            if not reply.startswith("OK HELLO robot"):
                self.close()
                raise link.RobotNotConnected(f"this is not the robot (it answered {reply!r})")

        def close(self):
            self.fd = None

        def send(self, line):
            if self.fd is None:
                raise link.RobotNotConnected("the link to the robot is closed")
            wire.to_robot += (line + "\n").encode("ascii")

        def read_line(self, timeout):
            def line_ready():
                if wire.to_stack:
                    self.buffer += bytes(wire.to_stack)
                    wire.to_stack.clear()
                return b"\n" in self.buffer

            if not clock.wait_for(line_ready, timeout):
                return None
            line, self.buffer = self.buffer.split(b"\n", 1)
            return line.decode("ascii", "replace").strip()

    return MemoryLink


class WorldTimeOver(BaseException):
    """The mission went on past its limit and did not stop when asked: cut. A BaseException, so that no `except
    Exception` of the stack (a tool's error goes back to the model) catches it."""


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main():
    parser = argparse.ArgumentParser(description="One mission in the simulator, as fast as the computer goes")
    parser.add_argument("--mission", required=True, help="a mission file, as for stack/strategist.py")
    parser.add_argument("--model", default="none")
    parser.add_argument("--tools", default="route", choices=["route", "goal", "step"])
    parser.add_argument("--mat", default=str(GUIDE / "mat.json"))
    parser.add_argument("--world", default=str(HERE / "worlds" / "forest.json"))
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--adult-s", type=float, default=10.0, help="world seconds before the adult puts the robot back")
    parser.add_argument("--limit-s", type=float, default=1800.0, help="world seconds a mission may last")
    parser.add_argument("--think", default="real", help="real, zero, or seconds: how far the world goes on while the "
                                                        "model thinks")
    parser.add_argument("--out", help="the folder of this run (by default a new one in .robot-sim/headless/)")
    args = parser.parse_args()

    out = Path(args.out) if args.out else GUIDE / ".robot-sim" / "headless" / f"seed{args.seed}-{os.getpid()}"
    out.mkdir(parents=True, exist_ok=True)
    state = out / "state"
    state.mkdir(exist_ok=True)
    mission = json.loads(Path(args.mission).read_text(encoding="utf-8"))
    key = "" if args.model in ("none", "fake") or args.model.startswith("ollama:") else sys.stdin.read().strip()

    # The simulated robot first: it writes the mat in use, which the stack reads when it is imported.
    sim = load_module("robot_sim", HERE / "robot_sim.py")
    world = sim.World(json.loads(Path(args.mat).read_text(encoding="utf-8")),
                      json.loads(Path(args.world).read_text(encoding="utf-8")), state, seed=args.seed)
    wire = Wire()
    os.close(world.link.master)  # the simulator's pseudo-terminal is not used: the wire in memory takes its place
    os.close(world.link.slave)
    world.link = RobotEnd(wire, world.wire)
    os.environ.update(ROBOT_CLOCK="stepped", ROBOT_THINK=args.think, ROBOT_STATE_DIR=str(state), ROBOT_MAT=str(state / "mat.json"),
                      ROBOT_SERIAL="memory")
    sys.path.insert(0, str(GUIDE / "stack"))
    import clock  # noqa: E402  (after ROBOT_CLOCK: it reads it once)
    import link  # noqa: E402
    link.Link = stack_end(link, clock, wire)  # before the Referee is imported: it takes Link from here
    import strategist  # noqa: E402

    adult = {"since": None, "resumes": 0, "stop_asked_at": None}

    def step(dt):
        """One tick of the world, then what the people around the mat do."""
        world.step(dt)
        if world.firmware.wait_adult and world.firmware.motion is None:
            adult["since"] = world.time if adult["since"] is None else adult["since"]
            if world.time - adult["since"] >= args.adult_s:
                world.action({"do": "adult_resume"})
                adult["resumes"] += 1
                adult["since"] = None
        else:
            adult["since"] = None
        if world.time >= args.limit_s and adult["stop_asked_at"] is None:
            adult["stop_asked_at"] = world.time  # as the Interpreter would: stop the mission
            (state / "control.json").write_text(json.dumps({"do": "stop"}) + "\n", encoding="utf-8")
        if adult["stop_asked_at"] is not None and world.time >= adult["stop_asked_at"] + 120:
            raise WorldTimeOver()

    clock.drive_with(step, sim.TICK_S, epoch=EPOCH)
    started, cut = time.monotonic(), False
    try:
        report = strategist.run(mission, args.model, key, args.tools, out=out / "mission")
    except WorldTimeOver:
        report, cut = {"outcome": "cut", "cards": [], "why": "the mission did not stop when asked"}, True
    computer_s = time.monotonic() - started
    result = {"report": report, "seed": args.seed, "world_s": round(world.time, 1), "computer_s": round(computer_s, 2),
              "faster_than_real_time": round(world.time / computer_s, 1) if computer_s else None,
              "time_limit_reached": adult["stop_asked_at"] is not None, "cut": cut, "adult_resumes": adult["resumes"],
              "bumps": world.bumps, "collisions": world.collisions, "robot_cell": world.cell_of(world.x, world.y)}
    (out / "result.json").write_text(json.dumps(result, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False), flush=True)


if __name__ == "__main__":
    main()
