"""Checks the robot's program, the simulator and the stack together, as an agent or the Strategist would use them.

No model and no key: this script calls the step tools of the stack itself, through MCP, in the order a
model could, then gives missions to the Strategist with no model or a scripted one, and reads in the
simulator what really happened (where the robot is, whether it hit something, when it stopped). Each
scenario starts again from a fresh mat.

Usage, in the Codespace of this guide (the folder of the guide is the current folder):
    /opt/workshop/venv/bin/python -I ci/check_sim.py
It takes about five minutes. It needs the `mcp` package, which is in /opt/workshop/venv.
"""
import ast
import asyncio
import contextlib
import importlib.util
import http.client
import json
import math
import os
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

GUIDE = Path(__file__).resolve().parent.parent
PYTHON = sys.executable
TOOLS = {"robot_state", "step_forward", "move", "turn", "look_around", "face", "set_light", "wait_for_event",
         "seen_cards"}
# A small mat for the checks: 5 x 5 cells of 30 cm, start on C5 facing north, 6 moves.
MAT = {"cols": 5, "rows": 5, "cell_cm": 30, "start": {"cell": "C5", "facing": "north"},
       "max_moves": 6, "move_timeout_s": 10,
       "cards": {"fire-station": "the fire station", "dry-grass": "dry grass",
                 "barbecue": "a barbecue", "water-point": "a water point"}}
results = []
CALLS = []  # the kinds of content of each answer the agent received


def check(name, ok, detail=""):
    results.append(ok)
    print(("PASS  " if ok else "FAIL  ") + name + ("" if ok else f"\n      {detail}"), flush=True)


class Sim:
    def __init__(self, state_dir, mat_path):
        self.proc = subprocess.Popen(
            [PYTHON, "-I", str(GUIDE / "sim" / "robot_sim.py"), "--port", "0", "--state-dir", str(state_dir),
             "--mat", str(mat_path), "--seed", "1"],
            stdout=subprocess.PIPE, text=True)
        for line in self.proc.stdout:
            if line.startswith("READY "):
                fields = dict(part.split("=", 1) for part in line.split()[1:])
                self.url = f"http://127.0.0.1:{fields['port']}"
                self.serial = fields["serial"]
                return
        raise RuntimeError("the simulator did not start")

    def act(self, **data):
        request = urllib.request.Request(self.url + "/api/action", data=json.dumps(data).encode(),
                                         headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(request, timeout=5) as r:
            return json.loads(r.read())

    def drive(self, **data):
        """A Drive button of the page: (HTTP status, answer)."""
        request = urllib.request.Request(self.url + "/api/drive", data=json.dumps(data).encode(),
                                         headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(request, timeout=20) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())

    def state(self):
        with urllib.request.urlopen(self.url + "/api/state", timeout=5) as r:
            return json.loads(r.read())

    def fresh(self, boxes=(), balls=(), cards=()):
        self.act(do="load_world", mat=MAT, world={"cards": list(cards), "boxes": list(boxes), "balls": list(balls)})

    def stop(self):
        self.proc.terminate()
        self.proc.wait(timeout=5)


class Agent:
    """A pretend agent: it calls the step tools of the stack (stack/drive.py) through MCP, as a model would."""

    def __init__(self, session):
        self.session = session

    async def call(self, name, **arguments):
        try:
            result = await self.session.call_tool(name, arguments)
        except Exception as e:  # a refused argument may come back as a protocol error
            return {"is_error": True, "text": f"{type(e).__name__}: {e}"}
        CALLS.append({item.type for item in result.content})
        text = result.content[0].text
        if result.is_error:
            return {"is_error": True, "text": text}
        return json.loads(text)


@contextlib.asynccontextmanager
async def agent(env):
    params = StdioServerParameters(command=PYTHON, args=["-I", str(GUIDE / "stack" / "drive.py")], env=env)
    async with stdio_client(params) as (read, write):
        async with ClientSession(read, write) as session:
            await session.initialize()
            yield Agent(session)


def wire_count(sim, word):
    """How many commands starting with this word the robot received since the mat was loaded."""
    return sim.state()["link"]["received"].get(word, 0)


def near(a, b, tolerance=1.0):
    return abs(a - b) <= tolerance


def box_in_front(sim, gap):
    """A box dropped right in front of the robot, `gap` centimetres from its front."""
    r = sim.state()["robot"]
    bottom = r["y"] - r["radius"] - gap
    return sim.act(do="add_box", x=r["x"] - 10, y=bottom - 20, w=20, h=20)["id"]


async def later(seconds, function, *args, **kwargs):
    await asyncio.sleep(seconds)
    return await asyncio.to_thread(function, *args, **kwargs)


async def channel_session(env, seconds):
    """Speak raw JSON-RPC to stack/drive.py, as Claude Code does with MCP_PROTOCOL_NEGOTIATION=legacy:
    initialize, initialized, then listen. Returns (the initialize answer, the channel notifications)."""
    process = await asyncio.create_subprocess_exec(
        PYTHON, "-I", str(GUIDE / "stack" / "drive.py"), env=dict(env, ROBOT_CHANNEL="1"),
        stdin=asyncio.subprocess.PIPE, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)

    async def send(message):
        process.stdin.write((json.dumps(message) + "\n").encode())
        await process.stdin.drain()

    await send({"jsonrpc": "2.0", "id": 1, "method": "initialize", "params": {
        "protocolVersion": "2025-06-18", "capabilities": {}, "clientInfo": {"name": "check", "version": "1"}}})
    init = json.loads(await asyncio.wait_for(process.stdout.readline(), 10))
    await send({"jsonrpc": "2.0", "method": "notifications/initialized"})
    pushed, end = [], time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            line = await asyncio.wait_for(process.stdout.readline(), end - time.monotonic())
        except asyncio.TimeoutError:
            break
        message = json.loads(line) if line.strip() else {}
        if message.get("method") == "notifications/claude/channel":
            pushed.append(message)
    process.terminate()
    await process.wait()
    return init, pushed


def micropython_subset():
    """The program of the robot must also run on the board, under MicroPython: keep to its part of
    Python (checked by reading the code; it is not run under MicroPython here)."""
    for path in sorted((GUIDE / "robot").glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"))
        bad = []
        for node in ast.walk(tree):
            if isinstance(node, (ast.Import, ast.ImportFrom, ast.JoinedStr, ast.AnnAssign, ast.NamedExpr,
                                 ast.Match, ast.AsyncFunctionDef, ast.Await)):
                bad.append(f"{type(node).__name__} line {node.lineno}")
            if isinstance(node, ast.FunctionDef) and (node.returns or any(a.annotation for a in node.args.args)):
                bad.append(f"annotation line {node.lineno}")
        check(f"{path.name} keeps to MicroPython's part of Python", not bad, ", ".join(bad))


class FakeHal:
    """The board, as the firmware sees it, written by hand: the Reflex alone, one tick at a time, no simulator."""

    def __init__(self):
        self.t, self.out, self.inbox, self.speeds, self.cut = 0, [], [], (0, 0, 0), False
        self.dist, self.frame, self.bump_n, self.press_n = (100, 0), (None, None, []), 0, {"stop": 0, "resume": 0}
        self.touch, self.held, self.odo, self.fail_distance, self.fresh = False, set(), [0.0, 0.0], False, True
        self.fail_write = 0

    def ticks_ms(self): return self.t
    def ticks_diff(self, new, old): return new - old
    def read_lines(self):
        lines, self.inbox = self.inbox, []
        return lines
    def write_line(self, text):
        if self.fail_write:
            self.fail_write -= 1
            raise OSError("serial write would block")
        self.out.append(text)
    def drive(self, forward, right, turn):
        if not self.cut:
            self.speeds = (forward, right, turn)
    def release(self, bumps):
        if bumps == self.bump_n and "stop" not in self.held:
            self.cut = False
    def odometry(self): return tuple(self.odo)
    def distance_cm(self):
        if self.fail_distance:
            raise OSError("VL53L1X data ready timeout")
        return self.dist
    def battery(self): return 100
    def button(self, name): return name in self.held
    def presses(self, name): return self.press_n[name]
    def bumper(self): return self.touch
    def bumps(self): return self.bump_n
    def camera_view(self): return self.frame
    def led(self, color): pass

    def touched(self):
        """The board's interrupt: count the touch, cut the motors."""
        self.bump_n += 1
        self.cut, self.speeds = True, (0, 0, 0)


class Reflex:
    STOP_DISTANCE_CM, MAX_SPEED_CM_S, WATCHDOG_MS, MIN_BATTERY, OBSTACLE_WAIT_MS = 8, 10, 100000, 20, 3000
    broken = ""  # "must_stop" or "warn": that function raises, as a mistake in reflex.py would

    def must_stop(self, distance_cm, battery):
        if self.broken == "must_stop":
            return 1 / 0
        return "obstacle" if distance_cm is not None and distance_cm < 8 else None

    def warn(self, distance_cm):
        if self.broken == "warn":
            return [][0]
        return None


def reflex_unit():
    """The Reflex alone (robot/firmware.py with a hand-made board): what protects, tick by tick."""
    spec = importlib.util.spec_from_file_location("firmware_alone", GUIDE / "robot" / "firmware.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    def robot(**changes):
        hal, reflex = FakeHal(), Reflex()
        for k, v in changes.items():
            setattr(reflex, k, v)
        return module.Firmware(hal, reflex), hal, reflex

    def run(fw, hal, ms, *lines):
        hal.inbox.extend(lines)
        for _ in range(ms // 10):
            hal.t += 10
            hal.odo[0] += abs(hal.speeds[0]) * 0.01 + abs(hal.speeds[1]) * 0.01
            hal.odo[1] += abs(hal.speeds[2]) * 0.01
            if hal.fresh:
                hal.dist = (100, hal.t)
            fw.tick()

    fw, hal, reflex = robot(broken="must_stop")
    run(fw, hal, 20, "MOVE F")
    refused = "ERR fault" in hal.out and "EVT FAULT:reflex" in hal.out and hal.speeds == (0, 0, 0)
    run(fw, hal, 20, "MOVE F", "STATE")
    state = [line for line in hal.out if line.startswith("OK STATE")][-1]
    check("Reflex alone: an error in reflex.py answers ERR fault, says EVT FAULT:reflex once, the wheels stay "
          "stopped, STATE fault=reflex", refused and hal.out.count("ERR fault") == 2
          and hal.out.count("EVT FAULT:reflex") == 1 and "fault=reflex" in state, hal.out)
    reflex.broken = ""
    hal.press_n["resume"] += 1
    run(fw, hal, 20)
    run(fw, hal, 20, "MOVE F")
    check("Reflex alone: after Resume (reflex.py mended), it moves again", "OK MOVE F" in hal.out
          and hal.speeds[0] > 0, hal.out[-4:])

    fw, hal, reflex = robot()
    run(fw, hal, 300, "MOVE F")
    driving = hal.speeds[0] > 0
    hal.fail_distance = True
    run(fw, hal, 10)
    check("Reflex alone: a sensor that raises while it drives: the wheels stop in that tick, EVT FAULT:error, "
          "EVT STOP fault", driving and hal.speeds == (0, 0, 0) and "EVT FAULT:error" in hal.out
          and "EVT STOP fault" in hal.out, (driving, hal.speeds, hal.out))

    fw, hal, reflex = robot(broken="warn")
    run(fw, hal, 1000)
    check("Reflex alone: warn() broken while it stands still: one FAULT, not one per tick",
          hal.out.count("EVT FAULT:reflex") == 1, hal.out)

    fw, hal, reflex = robot()
    run(fw, hal, 200, "MOVE F")
    hal.t += 120  # the program was late: two ticks 130 ms apart while the wheels turn
    run(fw, hal, 10)
    late = hal.speeds == (0, 0, 0) and "EVT STOP late" in hal.out
    hal.press_n["resume"] += 1
    run(fw, hal, 20)
    hal.t += 500  # late while it stands still: nothing watched, nothing moved
    run(fw, hal, 10, "STATE")
    state = [line for line in hal.out if line.startswith("OK STATE")][-1]
    check("Reflex alone: a late tick while the wheels turn stops it (EVT STOP late); standing still, a late tick "
          "changes nothing but tick_max", late and hal.out.count("EVT STOP late") == 1 and "tick_max=510" in state,
          (hal.out, state))
    run(fw, hal, 10, "STATE")
    state = [line for line in hal.out if line.startswith("OK STATE")][-1]
    check("Reflex alone: tick_max counts since the last STATE", "tick_max=10" in state and "fault=none" in state, state)

    fw, hal, reflex = robot()
    run(fw, hal, 300, "MOVE F")
    hal.touched()  # a touch shorter than a tick: the bumper is already free again when the tick comes
    run(fw, hal, 10)
    check("Reflex alone: a touch shorter than a tick, counted by the board: EVT BUMP, EVT STOP bump, the motors "
          "released once dealt with", "EVT BUMP" in hal.out and "EVT STOP bump" in hal.out and not hal.cut
          and hal.speeds == (0, 0, 0), (hal.out, hal.cut))

    fw, hal, reflex = robot()
    run(fw, hal, 300, "MOVE F")
    hal.fresh = False  # the distance sensor stops giving readings
    run(fw, hal, 400)
    stale = hal.speeds == (0, 0, 0)
    run(fw, hal, 1200, "STATE")
    state = [line for line in hal.out if line.startswith("OK STATE")][-1]
    check("Reflex alone: no recent distance forwards: the motors stop, it waits, then EVT STOP sensor; STATE "
          "dist=none", stale and "EVT STOP sensor" in hal.out and "dist=none" in state, (hal.out, state))
    fw, hal, reflex = robot()
    hal.fresh = False
    run(fw, hal, 400)  # its last reading is now older than 300 ms
    run(fw, hal, 1500, "MOVE F")
    check("Reflex alone: no recent distance before it starts: EVT REFUSED sensor, it never moved",
          "EVT REFUSED sensor" in hal.out and hal.odo[0] == 0, hal.out)

    fw, hal, reflex = robot()
    run(fw, hal, 100)
    hal.frame = (1, -500, [["barbecue", 0, 30]])  # an image taken before it stopped (the camera answers late)
    run(fw, hal, 500, "STATE")
    before = [line for line in hal.out if line.startswith("OK STATE")][-1]
    n = 1
    for _ in range(40):
        n += 1
        hal.frame = (n, hal.t, [["barbecue", 0, 30]])
        run(fw, hal, 10)
    run(fw, hal, 10, "STATE")
    after = [line for line in hal.out if line.startswith("OK STATE")][-1]
    check("Reflex alone: an image taken before it stopped is not believed; images taken after are, once stable",
          "looked=0" in before and "card=none" in before and "looked=1" in after and "card=barbecue" in after
          and any(line.startswith("EVT SEEN:barbecue") for line in hal.out), (before, after))

    fw, hal, reflex = robot()
    hal.frame = (1, -500, [])
    run(fw, hal, 2500, "MOVE F 2 UNTIL CARD")
    check("Reflex alone: a slide that must look, with no image since it stopped: EVT DONE F 0 camera, it never "
          "moved", "EVT DONE F 0 camera" in hal.out and hal.odo[0] == 0, hal.out)


    # Found by the review of protocol 5 (2026-11-07).
    fw, hal, reflex = robot()
    run(fw, hal, 300, "MOVE F")
    hal.touched()
    run(fw, hal, 10)
    stopped_at = hal.t
    hal.frame = (5, hal.t + 500, [])  # taken at the old place, after the stop, given late
    run(fw, hal, 1000)
    hal.press_n["resume"] += 1  # the adult put it back in front of a card
    run(fw, hal, 20)
    run(fw, hal, 2500, "MOVE F 2 UNTIL CARD")
    check("Reflex alone: after Resume, an image taken before (at the old place) is not believed: the slide stops on "
          "its cell (camera), it never drives", "EVT DONE F 0 camera" in hal.out and hal.speeds == (0, 0, 0)
          and stopped_at < 2000, hal.out[-4:])

    fw, hal, reflex = robot()
    run(fw, hal, 300, "MOVE F")
    hal.bump_n += 1
    hal.cut = True  # a board whose motor driver keeps its last order while the motors are cut
    run(fw, hal, 10)
    check("Reflex alone: once a touch is dealt with, the motors are released with the order stop, never an older one",
          not hal.cut and hal.speeds == (0, 0, 0), (hal.cut, hal.speeds))

    fw, hal, reflex = robot()
    run(fw, hal, 300, "MOVE F 3")
    reflex.broken, hal.fail_write = "must_stop", 3
    run(fw, hal, 10)
    reflex.broken = ""
    run(fw, hal, 200)
    check("Reflex alone: an error while saying the error (the cable's writes fail): nothing moves any more, fault set",
          hal.speeds == (0, 0, 0) and fw.motion is None and fw.fault == "reflex", (hal.speeds, fw.motion, fw.fault))

    fw, hal, reflex = robot()
    run(fw, hal, 100)
    hal.t += 60  # late while it stands still, and the command comes in that tick
    run(fw, hal, 300, "MOVE F")
    check("Reflex alone: a late tick before a move starts is not a late tick of a move: it drives",
          "EVT STOP late" not in hal.out and hal.speeds[0] > 0, hal.out)

    fw, hal, reflex = robot()
    hal.fail_distance = True
    run(fw, hal, 100, "STATE")
    hal.fail_distance = False
    hal.press_n["resume"] += 1
    run(fw, hal, 100, "STATE")
    states = [line for line in hal.out if line.startswith("OK STATE")]
    check("Reflex alone: a sensor that fails at every tick does not make the robot deaf: STATE answers fault=error, "
          "Resume is heard", len(states) == 2 and "fault=error" in states[0] and "fault=none" in states[1]
          and hal.out.count("EVT FAULT:error") == 1 and "EVT RESUMED" in hal.out, hal.out)

    # Found by ci/probe_tiers.py (random generator, 27 runs in 2000): a touch and a move in the same tick.
    fw, hal, reflex = robot()
    run(fw, hal, 100)
    hal.touched()
    run(fw, hal, 100, "MOVE R 3")
    check("Reflex alone: a move asked in the very tick of a touch is refused (ERR bump), the wheels stay stopped",
          "EVT BUMP" in hal.out and "ERR bump" in hal.out and hal.speeds == (0, 0, 0) and hal.odo[0] == 0, hal.out)

    fw, hal, reflex = robot(MAX_SPEED_CM_S="fast")
    run(fw, hal, 100, "MOVE F")
    check("Reflex alone: a constant of reflex.py that is not a number: FAULT:reflex, the wheels stay stopped",
          "EVT FAULT:reflex" in hal.out and hal.speeds == (0, 0, 0) and "MAX_SPEED_CM_S" in (fw.fault_text or ""),
          (hal.out, fw.fault_text))


async def timing_scenarios(sim, env):
    """The board's side, in the simulator: a touch shorter than a tick, a late program, a dead distance sensor,
    a camera that answers late."""
    sim.fresh()
    async with agent(env) as a:
        tap = asyncio.create_task(later(0.8, sim.act, do="tap_bumper"))
        reply = await a.call("step_forward")
        await tap
        r1 = sim.state()["robot"]
        await asyncio.sleep(0.3)
        r2 = sim.state()["robot"]
        check("a touch shorter than a tick during a move (counted by the board): the robot stops, the motors were cut",
              reply.get("result") == "stopped" and "bumper" in reply.get("why", "") and not r2["moving"]
              and near(r1["y"], r2["y"], 0.01) and r2["touches"] == 1, (reply, r1, r2))
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)

    sim.fresh()
    async with agent(env) as a:
        stall = asyncio.create_task(later(0.8, sim.act, do="stall", ms=150))
        reply = await a.call("step_forward")
        await stall
        check("the program late during a move (no tick for 150 ms): the robot stops by itself",
              reply.get("result") == "stopped" and "late" in reply.get("why", "") and not sim.state()["robot"]["moving"],
              reply)
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)

    sim.fresh()
    sim.act(do="distance_sensor", working=False)
    async with agent(env) as a:
        reply = await a.call("step_forward")
        r = sim.state()["robot"]
        check("a distance sensor that gives no reading: step_forward refused after waiting for one, the robot never moved",
              reply.get("result") == "refused" and "distance sensor" in reply.get("why", "")
              and "new reading" in reply.get("waited", "") and r["cell"] == "C5" and near(r["y"], 135), (reply, r))
    sim.act(do="distance_sensor", working=True)

    sim.act(do="timing", camera_delay_ms=1500)
    sim.fresh(cards=[{"cell": "C2", "label": "barbecue"}])
    async with agent(env) as a:
        reply = await a.call("move", direction="north", cells=3, stop_if=["card_ahead"])
        r = sim.state()["robot"]
        check("a camera 1.5 s late: move north until a card is ahead still stops on C3, in front of the card",
              reply.get("result") == "done" and r["cell"] == "C3"
              and (reply["state"]["card_ahead"] or {}).get("card") == "barbecue", (reply, r))
    sim.act(do="timing", camera_delay_ms=0)


async def main():
    micropython_subset()
    reflex_unit()
    with tempfile.TemporaryDirectory() as tmp:
        state_dir = Path(tmp)
        mat_path = state_dir / "mat.json"
        mat_path.write_text(json.dumps(MAT), encoding="utf-8")
        env = dict(os.environ, ROBOT_STATE_DIR=str(state_dir), ROBOT_MAT=str(mat_path))
        # The step tools are checked with a limit of the adults on the moves (none by default): MAT["max_moves"].
        limited = json.loads((GUIDE / "stack" / "rules.json").read_text(encoding="utf-8"))
        limited["limits"] = {"moves": MAT["max_moves"]}
        (state_dir / "rules-limited.json").write_text(json.dumps(limited), encoding="utf-8")
        env["ROBOT_RULES"] = str(state_dir / "rules-limited.json")

        # No robot yet: a clear answer, no crash.
        async with agent(dict(env, ROBOT_SERIAL=str(state_dir / "no-such-port"))) as a:
            reply = await a.call("robot_state")
            check("no robot: the gateway says so", reply.get("result") == "error" and "not connected" in reply.get("why", ""),
                  reply)

        sim = Sim(state_dir, mat_path)
        try:
            await scenarios(sim, env)
            await timing_scenarios(sim, env)
            stack_scenarios(sim, env)
            await interpreter_scenarios(sim, env)
        finally:
            sim.stop()
        check(f"only text reached the agent, never an image ({len(CALLS)} answers)",
              CALLS and all(kinds == {"text"} for kinds in CALLS), CALLS)

    failed = results.count(False)
    print(f"\n{len(results) - failed} PASS / {failed} FAIL")
    sys.exit(1 if failed else 0)


async def scenarios(sim, env):
    # The tools: a closed list, only robot_state is read only.
    sim.fresh()
    async with agent(env) as a:
        tools = (await a.session.list_tools()).tools
        names = {t.name for t in tools}
        check("the gateway offers exactly nine tools", names == TOOLS, names)
        read_only = {t.name for t in tools if t.annotations and t.annotations.read_only_hint}
        check("only robot_state, wait_for_event and seen_cards are marked read only",
              read_only == {"robot_state", "wait_for_event", "seen_cards"}, read_only)

    # The first prototype: one cell forwards, then it refuses to go on in front of a box.
    sim.fresh(boxes=[{"x": 65, "y": 65, "w": 20, "h": 20, "name": "box"}])
    async with agent(env) as a:
        reply = await a.call("step_forward")
        r = sim.state()["robot"]
        check("prototype: one cell forwards", reply.get("result") == "done" and r["cell"] == "C4"
              and near(r["x"], 75) and near(r["y"], 105), (reply, r))
        check("prototype: the gateway knows where it is", reply.get("state", {}).get("cell") == "C4", reply)
        reply = await a.call("step_forward")
        r = sim.state()["robot"]
        check("prototype: it refuses to go on in front of the box",
              reply.get("result") == "refused" and "in front" in reply.get("why", "") and r["cell"] == "C4", (reply, r))
        reply = await a.call("robot_state")
        check("prototype: after waiting in vain, a simple refusal: no adult needed, the position is known",
              reply.get("position_known") is True and reply.get("waiting_for_adult") is False
              and reply.get("cell") == "C4", reply)
        check("prototype: no collision", r["collisions"] == 0, r)

    # A box dropped in front of the robot while it moves: the reflex stops it, not the agent.
    sim.fresh()
    async with agent(env) as a:
        drop = asyncio.create_task(later(1.0, box_in_front, sim, 4))
        reply = await a.call("step_forward")
        box = await drop
        r = sim.state()["robot"]
        check("box during a move: the robot stops by itself",
              reply.get("result") == "stopped" and "in front" in reply.get("why", "") and not r["moving"], (reply, r))
        check("box during a move: no collision", r["collisions"] == 0, r)
        reply = await a.call("step_forward")
        check("after a stop: the gateway refuses until an adult comes",
              reply.get("result") == "refused" and "adult" in reply.get("what_to_do", ""), reply)
        reply = await a.call("robot_state")
        check("after a stop: the position is unknown", reply.get("position_known") is False and reply.get("cell") is None,
              reply)
        sim.act(do="remove_box", id=box)
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)
        r = sim.state()["robot"]
        check("the adult puts it back on C5", r["cell"] == "C5" and near(r["y"], 135), r)
        reply = await a.call("step_forward")
        check("after Resume: it moves again", reply.get("result") == "done"
              and reply.get("state", {}).get("cell") == "C4", reply)

    # Something crosses its way during a move and goes: the robot waits, then finishes the cell.
    sim.fresh()
    async with agent(env) as a:
        drop = asyncio.create_task(later(1.0, box_in_front, sim, 4))
        moving = asyncio.create_task(a.call("step_forward"))
        box = await drop
        await asyncio.sleep(1.0)
        sim.act(do="remove_box", id=box)
        reply = await moving
        r = sim.state()["robot"]
        heard = " ".join(e["event"] for e in reply.get("since_last_call", []))
        check("a box dropped, then taken away within 3 s: the robot waits, then finishes the cell",
              reply.get("result") == "done" and r["cell"] == "C4" and near(r["y"], 105) and r["collisions"] == 0
              and "waited" in heard and "went on" in heard, (reply, r))

    # The cable comes loose during a move: the robot stops by itself (watchdog).
    sim.fresh()
    async with agent(env) as a:
        unplug = asyncio.create_task(later(1.0, sim.act, do="cable", plugged=False))
        reply = await a.call("step_forward")
        await unplug
        s = sim.state()
        stop = next((x for x in s["stops"] if x["why"] == "watchdog"), None)
        limit = s["reflex"]["WATCHDOG_MS"] / 1000 + 0.1
        check("cable unplugged: the robot stops within the watchdog delay",
              stop is not None and stop["t"] - stop["unplugged_at"] <= limit and not s["robot"]["moving"], (stop, limit))
        check("cable unplugged: the agent is told the move was stopped", reply.get("result") == "stopped", reply)
        sim.act(do="cable", plugged=True)
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)
        reply = await a.call("step_forward")
        check("cable back, adult came: it moves again", reply.get("result") == "done", reply)

    # The emergency button, before and during a move.
    sim.fresh()
    async with agent(env) as a:
        sim.act(do="button", name="stop")
        await asyncio.sleep(0.3)
        before = sim.state()["robot"]
        reply = await a.call("step_forward")
        after = sim.state()["robot"]
        check("emergency button: no move", reply.get("result") == "refused" and "emergency" in reply.get("why", "")
              and near(before["y"], after["y"], 0.01), reply)
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)
        press = asyncio.create_task(later(1.0, sim.act, do="button", name="stop"))
        reply = await a.call("step_forward")
        await press
        check("emergency button during a move: it stops", reply.get("result") == "stopped"
              and "emergency" in reply.get("why", "") and not sim.state()["robot"]["moving"], reply)

    # Never off the mat, never below the battery limit, never more moves than allowed.
    sim.fresh()
    async with agent(env) as a:
        await a.call("turn", direction="left")
        await a.call("turn", direction="left")
        moves = wire_count(sim, "MOVE")
        reply = await a.call("step_forward")
        check("off the mat: refused by the gateway, nothing sent to the robot",
              reply.get("result") == "refused" and "outside the mat" in reply.get("why", "")
              and wire_count(sim, "MOVE") == moves and sim.state()["robot"]["cell"] == "C5", reply)
        sim.act(do="battery", percent=15)
        reply = await a.call("turn", direction="left")
        check("battery low: refused", reply.get("result") == "refused" and "battery" in reply.get("why", ""), reply)
        sim.act(do="battery", percent=100)
        for _ in range(MAT["max_moves"] - 2):
            await a.call("turn", direction="right")
        reply = await a.call("turn", direction="right")
        check("no more moves than allowed", reply.get("result") == "refused" and "no moves left" in reply.get("why", ""),
              reply)

    # look_around: a ball in view on one side is said, once the camera has seen it twice.
    sim.fresh(balls=[{"from": [135, 135], "to": [135, 135], "radius": 7, "speed_cm_s": 0}])
    async with agent(env) as a:
        reply = await a.call("look_around")
        east = (reply.get("looked") or [{}])[0]
        check("look_around: a ball standing east (E5) is said while the robot faces east",
              east.get("facing") == "east" and any("ball" in e for e in east.get("what_the_camera_saw", [])), reply)

    # look_around and face: several quarter turns, one move.
    sim.fresh(cards=[{"cell": "C4", "label": "barbecue"}])
    async with agent(env) as a:
        turns = wire_count(sim, "TURN")
        reply = await a.call("look_around")
        looked = reply.get("looked", [])
        check("look_around: four quarter turns, back facing north, one move",
              [x["facing"] for x in looked] == ["east", "south", "west", "north"]
              and wire_count(sim, "TURN") == turns + 4 and reply["state"]["facing"] == "north"
              and reply["state"]["moves_left"] == MAT["max_moves"] - 1, reply)
        check("look_around: the card in front, in the direction where it is",
              (looked[3]["card_ahead"] or {}).get("card") == "barbecue" and looked[0]["card_ahead"] is None, looked)
        reply = await a.call("face", direction="south")
        check("face('south'): two quarter turns, one move",
              reply.get("result") == "done" and reply["state"]["facing"] == "south"
              and wire_count(sim, "TURN") == turns + 6 and reply["state"]["moves_left"] == MAT["max_moves"] - 2, reply)
        reply = await a.call("face", direction="south")
        check("face the way it already faces: nothing sent, no move",
              wire_count(sim, "TURN") == turns + 6 and reply["state"]["moves_left"] == MAT["max_moves"] - 2, reply)
        reply = await a.call("face", direction="up")
        check("face('up'): an error, nothing sent", reply.get("is_error") is True and wire_count(sim, "TURN") == turns + 6,
              reply)
        for _ in range(MAT["max_moves"] - 2):
            await a.call("turn", direction="right")
        reply = await a.call("look_around")
        check("look_around with no moves left: refused, nothing sent",
              reply.get("result") == "refused" and "no moves left" in reply.get("why", "")
              and wire_count(sim, "TURN") == turns + 6 + MAT["max_moves"] - 2, reply)

    # move: the mecanum wheels slide the robot several cells without turning, one move per command.
    sim.fresh(cards=[{"cell": "C2", "label": "barbecue"}])
    async with agent(env) as a:
        sent = wire_count(sim, "MOVE")
        reply = await a.call("move", direction="north", cells=3, stop_if=["card_ahead"])
        r = sim.state()["robot"]
        check("move north, 3 cells until a card is ahead: it stops on C3, in front of the card (C2), one move",
              reply.get("result") == "done" and reply.get("cells_moved") == 2 and r["cell"] == "C3"
              and (reply["state"]["card_ahead"] or {}).get("card") == "barbecue"
              and reply["state"]["moves_left"] == MAT["max_moves"] - 1 and wire_count(sim, "MOVE") == sent + 1, (reply, r))
        reply = await a.call("move", direction="north", cells=2, stop_if=["card_ahead"])
        check("move with a card on the next cell: refused, nothing sent, it never drives onto a card",
              reply.get("result") == "refused" and "never drives onto a card" in reply.get("why", "")
              and wire_count(sim, "MOVE") == sent + 1 and sim.state()["robot"]["cell"] == "C3", reply)
        reply = await a.call("move", direction="east", cells=2)
        check("sideways into cells the sensor never saw: refused, nothing sent",
              reply.get("result") == "refused" and "saw free" in reply.get("why", "") and wire_count(sim, "MOVE") == sent + 1,
              reply)
        await a.call("face", direction="east")
        await a.call("face", direction="north")
        reply = await a.call("move", direction="east", cells=2)
        r = sim.state()["robot"]
        check("sideways once the sensor saw the way free: two cells east, still facing north, no collision",
              reply.get("result") == "done" and reply.get("cells_moved") == 2 and r["cell"] == "E3"
              and near(r["heading"] % 360, 0, 2) and r["collisions"] == 0, (reply, r))
        reply = await a.call("move", direction="east")
        check("move off the mat: refused, nothing sent",
              reply.get("result") == "refused" and "outside the mat" in reply.get("why", "")
              and wire_count(sim, "MOVE") == sent + 2, reply)

    sim.fresh(boxes=[{"x": 95, "y": 125, "w": 20, "h": 20, "name": "box"}])
    async with agent(env) as a:
        await a.call("face", direction="east")
        await a.call("face", direction="north")
        reply = await a.call("move", direction="east")
        check("a box beside the robot (D5): the sensor saw it, the slide is refused",
              reply.get("result") == "refused" and "D5" in reply.get("why", "") and sim.state()["robot"]["collisions"] == 0,
              reply)

    sim.fresh()
    async with agent(env) as a:
        await a.call("face", direction="west")
        reply = await a.call("move", direction="east")
        check("backwards into a cell the sensor never saw (D5, facing west): refused", reply.get("result") == "refused",
              reply)
        await a.call("look_around")
        reply = await a.call("move", direction="east")
        r = sim.state()["robot"]
        check("backwards after look_around: on D5, still facing west",
              reply.get("result") == "done" and r["cell"] == "D5" and near(r["heading"] % 360, 270, 2), (reply, r))

    sim.fresh(balls=[{"from": [105, 15], "to": [105, 15], "radius": 7, "speed_cm_s": 0}])
    async with agent(env) as a:
        reply = await a.call("move", direction="north", cells=2, stop_if=["ball_seen"])
        check("move until the camera sees a ball (D1, out of sight from C5): it stops on C4, where it sees it",
              reply.get("cells_moved") == 1 and "ball" in reply.get("stopped_because", "")
              and sim.state()["robot"]["cell"] == "C4", reply)

    sim.fresh(cards=[{"cell": "E3", "label": "water-point"}])
    async with agent(env) as a:
        await a.call("face", direction="east")
        await a.call("face", direction="north")
        reply = await a.call("move", direction="east", cells=2, stop_if=["new_card"])
        check("move until a new card: it stops on D5, where the camera first shows the water point (E3)",
              reply.get("cells_moved") == 1 and "had not shown" in reply.get("stopped_because", "")
              and sim.state()["robot"]["cell"] == "D5", reply)

    sim.fresh()
    async with agent(env) as a:
        press = asyncio.create_task(later(4.5, sim.act, do="button", name="stop"))
        reply = await a.call("move", direction="north", cells=3)
        await press
        check("emergency button in the second cell: stopped, one cell done, an adult must put it back on C4",
              reply.get("result") == "stopped" and reply.get("cells_moved") == 1 and "C4" in reply.get("what_to_do", ""),
              reply)
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)
        r = sim.state()["robot"]
        reply = await a.call("robot_state")
        check("the adult puts it back on C4, the gateway counts from C4",
              r["cell"] == "C4" and reply.get("cell") == "C4" and reply.get("position_known") is True, (r, reply))

    sim.fresh(balls=[{"from": [135, 105], "to": [135, 105], "radius": 7, "speed_cm_s": 0}])
    async with agent(env) as a:
        await a.call("face", direction="east")
        await asyncio.sleep(1.2)  # the robot notes a ball once it has looked twice
        await a.call("face", direction="north")
        sent = wire_count(sim, "MOVE")
        reply = await a.call("move", direction="east", cells=2)
        check("sideways with a ball seen near the way (E4): refused, nothing sent, even with the cells seen free",
              reply.get("result") == "refused" and "ball" in reply.get("why", "") and wire_count(sim, "MOVE") == sent,
              reply)

    sim.fresh()
    state_file = Path(env["ROBOT_STATE_DIR"]) / "terrain.json"
    async with agent(env) as a:
        await a.call("face", direction="east")
        await a.call("face", direction="north")
        saved = json.loads(state_file.read_text())
        saved["free"] = {cell: when - 200 for cell, when in saved["free"].items()}
        state_file.write_text(json.dumps(saved))
        reply = await a.call("move", direction="east")
        check("a free reading older than two minutes: the slide is refused",
              reply.get("result") == "refused" and "saw free" in reply.get("why", ""), reply)
        await a.call("face", direction="east")
        sim.act(do="add_box", x=95, y=125, w=20, h=20)
        await a.call("face", direction="east")  # already facing east: nothing moves, the sensor reads again
        await a.call("face", direction="north")
        reply = await a.call("move", direction="east")
        check("a box put down after a free reading: the next reading forgets the free cell, the slide is refused",
              reply.get("result") == "refused" and "D5" in reply.get("why", "") and sim.state()["robot"]["collisions"] == 0,
              reply)

    sim.fresh(boxes=[{"x": 65, "y": 65, "w": 20, "h": 20, "name": "box"}])
    async with agent(env) as a:
        reply = await a.call("move", direction="north", cells=3)
        r = sim.state()["robot"]
        check("a box on the way of a slide forwards (C3): it stays on the last cell it reached (C4), no adult needed",
              reply.get("result") == "done" and reply.get("cells_moved") == 1 and "in the way" in reply.get("stopped_because", "")
              and r["cell"] == "C4" and reply["state"]["position_known"] is True
              and reply["state"]["moves_left"] == MAT["max_moves"] - 1 and r["collisions"] == 0, (reply, r))

    sim.fresh(cards=[{"cell": "C4", "label": "barbecue"}])
    async with agent(env) as a:
        await a.call("face", direction="east")
        await a.call("face", direction="north")
        reply = await a.call("move", direction="east", stop_if=["card_ahead"])
        check("card_ahead looks the way the robot faces: a slide sideways is not held by the card in front",
              reply.get("cells_moved") == 1 and sim.state()["robot"]["cell"] == "D5", reply)
        sent = wire_count(sim, "MOVE")
        left = reply["state"]["moves_left"]
        for bad in (0, 9):
            reply = await a.call("move", direction="north", cells=bad)
            check(f"move with cells={bad}: refused, nothing sent, no move counted",
                  reply.get("result") == "refused" and wire_count(sim, "MOVE") == sent
                  and (await a.call("robot_state"))["moves_left"] == left, reply)

    # Never onto a card: step_forward, a slide forwards without stop_if, a slide sideways past a known card.
    sim.fresh(cards=[{"cell": "C4", "label": "barbecue"}])
    async with agent(env) as a:
        sent = wire_count(sim, "MOVE")
        reply = await a.call("step_forward")
        check("step_forward with a card on the next cell: refused, nothing sent",
              reply.get("result") == "refused" and "never drives onto a card" in reply.get("why", "")
              and wire_count(sim, "MOVE") == sent and sim.state()["robot"]["cell"] == "C5", reply)
    sim.fresh(cards=[{"cell": "C2", "label": "barbecue"}])
    async with agent(env) as a:
        reply = await a.call("move", direction="north", cells=3)
        check("a slide forwards without stop_if stops in front of the card (C3), not on it",
              reply.get("cells_moved") == 2 and sim.state()["robot"]["cell"] == "C3"
              and "card" in reply.get("stopped_because", ""), reply)
    sim.fresh(cards=[{"cell": "E5", "label": "water-point"}])
    async with agent(env) as a:
        await a.call("face", direction="east")
        await asyncio.sleep(0.6)
        await a.call("face", direction="north")
        reply = await a.call("move", direction="east", cells=2)
        check("a slide sideways stops before a card the gateway placed (E5): one cell, on D5",
              reply.get("cells_moved") == 1 and sim.state()["robot"]["cell"] == "D5"
              and "E5" in reply.get("stopped_because", ""), reply)
    sim.fresh()
    async with agent(env) as a:
        await a.call("move", direction="north")
        reply = await a.call("move", direction="south")
        check("back to the cell it just left: known free, the slide backwards goes, no look needed",
              reply.get("result") == "done" and sim.state()["robot"]["cell"] == "C5", reply)

    # A ball's lane: seen rolling north and south in column D, the robot never slides into it sideways.
    sim.fresh(balls=[{"from": [105, 15], "to": [105, 75], "radius": 7, "speed_cm_s": 8}])
    async with agent(env) as a:
        await asyncio.sleep(3.0)
        await a.call("robot_state")
        await a.call("face", direction="east")
        await a.call("face", direction="north")
        sent = wire_count(sim, "MOVE")
        reply = await a.call("move", direction="east")
        check("sideways into a ball's lane (column D): refused, nothing sent",
              reply.get("result") == "refused" and "lane" in reply.get("why", "") and wire_count(sim, "MOVE") == sent,
              reply)

    # Forwards, never onto a cell where a ball was seen just before: step_forward and move alike.
    sim.fresh(balls=[{"from": [75, 105], "to": [75, 105], "radius": 7, "speed_cm_s": 0}])
    async with agent(env) as a:
        await asyncio.sleep(1.2)
        await a.call("robot_state")
        sent = wire_count(sim, "MOVE")
        slide = await a.call("move", direction="north")
        step = await a.call("step_forward")
        check("forwards, onto the cell of a ball seen just before (C4): move and step_forward refused, nothing sent",
              slide.get("result") == "refused" and step.get("result") == "refused" and "less than" in step.get("why", "")
              and wire_count(sim, "MOVE") == sent, (slide, step))

    sim.fresh(balls=[{"from": [75, 45], "to": [75, 45], "radius": 7, "speed_cm_s": 0}])
    async with agent(env) as a:
        await asyncio.sleep(1.2)
        await a.call("robot_state")
        reply = await a.call("move", direction="north")
        check("forwards, a still ball seen two cells ahead (C2): the slide to C4 goes, the robot watches it",
              reply.get("result") == "done" and sim.state()["robot"]["cell"] == "C4", reply)

    # A ball rolling along row 3: the gateway notes row 3 as its lane.
    sim.fresh(balls=[{"from": [15, 75], "to": [145, 75], "radius": 7, "speed_cm_s": 8}])
    async with agent(env) as a:
        await asyncio.sleep(4.0)
        reply = await a.call("robot_state")
        lanes = json.loads((Path(env["ROBOT_STATE_DIR"]) / "terrain.json").read_text()).get("lanes", {})
        check("a ball rolling along row 3: row 3 is noted as its lane, the robot on C5 is not on it",
              "row:2" in lanes and reply.get("on_a_ball_lane") is False, (lanes, reply.get("on_a_ball_lane")))
        check("the lane is the line the ball rolls on (rows only, at most the two of its sightings), and robot_state "
              "names it", all(k.startswith("row:") for k in lanes) and len(lanes) <= 2 and "row 3" in reply.get(
              "ball_lanes", []), (lanes, reply.get("ball_lanes")))

    # A ball seen only by its bump: the robot's cell becomes a lane.
    sim.fresh(balls=[{"from": [15, 135], "to": [145, 135], "radius": 7, "speed_cm_s": 20}])
    async with agent(env) as a:
        await asyncio.sleep(3.0)
        await a.call("robot_state")
        lanes = json.loads((Path(env["ROBOT_STATE_DIR"]) / "terrain.json").read_text()).get("lanes", {})
        check("a ball bumps the robot standing on C5, unseen by the camera: C5 becomes a lane", "cell:C5" in lanes, lanes)

    # Stopped during a quarter turn on a lane: the adult is sent to the nearest cell off it.
    sim.fresh()
    async with agent(env) as a:
        await a.call("robot_state")
        state_file = Path(env["ROBOT_STATE_DIR"]) / "terrain.json"
        saved = json.loads(state_file.read_text())
        saved["lanes"] = {"row:4": time.time()}  # row 5, the robot's, as if a ball had been seen rolling there
        state_file.write_text(json.dumps(saved))
        press = asyncio.create_task(later(0.3, sim.act, do="button", name="stop"))
        reply = await a.call("turn", direction="right")
        await press
        check("stopped during a quarter turn on C5, on a ball's lane: the adult is asked to put the robot on C4, off it",
              reply.get("result") == "stopped" and "C4" in reply.get("what_to_do", ""), reply)
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)
        reply = await a.call("robot_state")
        check("after the adult: both on C4", reply.get("cell") == "C4" and sim.state()["robot"]["cell"] == "C4",
              (reply, sim.state()["robot"]))

    # go_to_cell never stops on a lane: refused, with the nearest cell off it.
    sim.fresh()
    time.sleep(0.6)
    state_file = Path(env["ROBOT_STATE_DIR"]) / "terrain.json"
    script = ("import json, sys, time; sys.path.insert(0, 'stack'); import pilot; pilot.robot_state(); "
              "from pathlib import Path; f = Path(sys.argv[1]); t = json.loads(f.read_text()); "
              "t['lanes'] = {'row:2': time.time()}; f.write_text(json.dumps(t)); "
              "print(json.dumps(pilot.go_to_cell('C3')))")
    done = subprocess.run([PYTHON, "-I", "-c", script, str(state_file)], env=env, cwd=GUIDE, capture_output=True,
                          text=True, timeout=120)
    reply = json.loads(done.stdout.strip().splitlines()[-1]) if done.stdout.strip() else {}
    check("go_to_cell to C3, on a ball's lane (row 3): refused, nothing moved, the answer names C4",
          reply.get("result") == "refused" and "lane" in reply.get("why", "") and "C4" in reply.get("what_to_do", "")
          and sim.state()["robot"]["cell"] == "C5", (reply, done.stderr[-400:]))

    # Crossing a lane whose ball was not seen going away: the Pilot watches first (careful_balls), then crosses.
    sim.fresh()
    time.sleep(0.6)
    script = ("import json, sys, time; sys.path.insert(0, 'stack'); import pilot; pilot.robot_state(); "
              "from pathlib import Path; f = Path(sys.argv[1]); t = json.loads(f.read_text()); "
              "t['lanes'] = {'row:3': time.time()}; f.write_text(json.dumps(t)); "
              "start = time.monotonic(); r = pilot.go_to_cell('C3'); print(json.dumps([r, time.monotonic() - start]))")
    done = subprocess.run([PYTHON, "-I", "-c", script, str(state_file)], env=env, cwd=GUIDE, capture_output=True,
                          text=True, timeout=120)
    reply, took = json.loads(done.stdout.strip().splitlines()[-1]) if done.stdout.strip() else ({}, 0)
    check("go_to_cell to C3 across row 4, a lane whose ball was not seen: the Pilot watches (about 15 s), then crosses",
          reply.get("result") == "done" and sim.state()["robot"]["cell"] == "C3" and took >= 14
          and wire_count(sim, "MOVE") >= 1, (reply, took, done.stderr[-400:]))
    sim.fresh()
    time.sleep(0.6)
    script = script.replace("pilot.robot_state(); ", "pilot.PILOT.configure({'careful_balls': False}); pilot.robot_state(); ")
    done = subprocess.run([PYTHON, "-I", "-c", script, str(state_file)], env=env, cwd=GUIDE, capture_output=True,
                          text=True, timeout=120)
    reply, took = json.loads(done.stdout.strip().splitlines()[-1]) if done.stdout.strip() else ({}, 0)
    check("the same without careful_balls (an adventurous personality): no watch, it crosses at once",
          reply.get("result") == "done" and sim.state()["robot"]["cell"] == "C3" and took < 10,
          (reply, took, done.stderr[-400:]))

    # Stopped on a lane: the adult is sent to the last cell before it, and the instruction does not change.
    sim.fresh()
    state_file = Path(env["ROBOT_STATE_DIR"]) / "terrain.json"
    async with agent(env) as a:
        await a.call("robot_state")
        saved = json.loads(state_file.read_text())
        saved["lanes"] = {"row:3": time.time()}  # row 4, as if a ball had been seen rolling there
        state_file.write_text(json.dumps(saved))
        press = asyncio.create_task(later(4.5, sim.act, do="button", name="stop"))
        reply = await a.call("move", direction="north", cells=3)
        await press
        check("stopped after C4, on a ball's lane: the adult is asked to put the robot back on C5, off the lane",
              reply.get("result") == "stopped" and "C5" in reply.get("what_to_do", ""), reply)
        await asyncio.sleep(1.0)
        again = await a.call("step_forward")
        check("the instruction stays the same while the robot waits", "C5" in again.get("what_to_do", ""), again)
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)
        reply = await a.call("robot_state")
        check("after the adult: both on C5", reply.get("cell") == "C5" and sim.state()["robot"]["cell"] == "C5",
              (reply, sim.state()["robot"]))

    # A slide interrupted on a loose cable: the gateway, the adult and the robot agree on the cell.
    sim.fresh()
    async with agent(env) as a:
        unplug = asyncio.create_task(later(4.5, sim.act, do="cable", plugged=False))
        plug = asyncio.create_task(later(7.0, sim.act, do="cable", plugged=True))
        reply = await a.call("move", direction="north", cells=3)
        await unplug
        await plug
        check("cable loose in the second cell of a slide: stopped, an adult must come",
              reply.get("result") == "stopped" and "C4" in reply.get("what_to_do", ""), reply)
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)
        reply = await a.call("robot_state")
        r = sim.state()["robot"]
        check("after the adult: the gateway's cell is the robot's cell (C4)",
              reply.get("cell") == "C4" and r["cell"] == "C4" and reply.get("position_known") is True, (reply, r))

    # A gateway killed during a slide: the next one knows the position is lost, and where the robot is.
    sim.fresh()
    by_hand = subprocess.Popen([PYTHON, "-I", str(GUIDE / "stack" / "drive.py"), "move", "north", "3"],
                               env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    await asyncio.sleep(4.5)
    by_hand.kill()
    by_hand.wait()
    await asyncio.sleep(2.0)  # the robot's watchdog stops it
    async with agent(env) as a:
        reply = await a.call("robot_state")
        check("a gateway killed in the second cell: the next one says the position is unknown",
              reply.get("position_known") is False, reply)
        reply = await a.call("step_forward")
        check("and asks the adult to put the robot on the last cell it reached (C4)",
              reply.get("result") == "refused" and "C4" in reply.get("what_to_do", ""), reply)
        sim.act(do="adult_resume")
        await asyncio.sleep(0.5)
        reply = await a.call("robot_state")
        r = sim.state()["robot"]
        check("after the adult: both on C4", reply.get("cell") == "C4" and r["cell"] == "C4", (reply, r))

    # A wrong argument never reaches the robot.
    sim.fresh()
    async with agent(env) as a:
        turns = wire_count(sim, "TURN")
        reply = await a.call("turn", direction="up")
        check("turn('up'): an error, nothing sent", reply.get("is_error") is True and wire_count(sim, "TURN") == turns,
              reply)
        moves = wire_count(sim, "MOVE")
        reply = await a.call("move", direction="up")
        check("move('up'): an error, nothing sent", reply.get("is_error") is True and wire_count(sim, "MOVE") == moves, reply)
        reply = await a.call("move", direction="north", stop_if=["fire"])
        check("move with an unknown condition: an error, nothing sent",
              reply.get("is_error") is True and wire_count(sim, "MOVE") == moves, reply)
        reply = await a.call("set_light", color="purple")
        check("set_light('purple'): an error", reply.get("is_error") is True, reply)
        reply = await a.call("set_light", color="red")
        check("set_light('red'): the light changes", reply.get("result") == "done" and sim.state()["robot"]["led"] == "red",
              reply)

    # The card in front, as a label from a closed list.
    sim.fresh(cards=[{"cell": "C4", "label": "water-point"}])
    async with agent(env) as a:
        reply = await a.call("robot_state")
        check("the camera gives a label, with its meaning",
              reply.get("card_ahead") == {"card": "water-point", "means": "a water point"}, reply)
        lines = (Path(env["ROBOT_STATE_DIR"]) / "context.jsonl").read_text(encoding="utf-8").splitlines()
        check("context.jsonl: what the agent received is written down", json.loads(lines[-1])["tool"] == "robot_state",
              lines[-1:])

    # The Drive buttons of the page: the same gateway, its log says who drove.
    sim.fresh()
    status, reply = sim.drive(command="forward")
    r = sim.state()["robot"]
    check("Drive button: one cell forwards, through the gateway",
          status == 200 and reply["result"].get("result") == "done" and r["cell"] == "C4", (status, reply, r))
    last = json.loads((Path(env["ROBOT_STATE_DIR"]) / "context.jsonl").read_text(encoding="utf-8").splitlines()[-1])
    check("Drive button: the log says the page drove", last["caller"] == "page" and last["tool"] == "step_forward",
          last)
    first = asyncio.create_task(asyncio.to_thread(sim.drive, command="forward"))
    await asyncio.sleep(0.5)
    status, reply = await asyncio.to_thread(sim.drive, command="left")
    await first
    check("Drive button: one command at a time", status == 409, (status, reply))
    async with agent(env) as a:
        moving = asyncio.create_task(a.call("step_forward"))
        await asyncio.sleep(0.5)
        status, reply = await asyncio.to_thread(sim.drive, command="state")
        await moving
        check("an agent moves, the page asks: refused, the cable is not shared",
              status == 200 and "another command" in reply["result"].get("why", ""), (status, reply))
    status, _ = sim.drive(command="backward")
    check("Drive button: an unknown command is refused", status == 400, status)
    status, _ = sim.drive(command="light", color="purple")
    check("Drive button: an unknown colour is refused", status == 400, status)

    # What a person can move on the mat: cards (on a cell) and the ball (with its path).
    sim.fresh(cards=[{"cell": "B2", "label": "water-point"}],
              balls=[{"from": [15, 15], "to": [15, 45], "radius": 5, "speed_cm_s": 0}])
    sim.act(do="move_card", **{"from": "B2", "to": "C4"})
    async with agent(env) as a:
        reply = await a.call("robot_state")
        check("a card moved by hand: the camera sees it on its new cell",
              (reply.get("card_ahead") or {}).get("card") == "water-point", reply)
    refused = sim.act(do="move_card", **{"from": "C4", "to": "Z9"})
    check("a card cannot leave the mat", "error" in refused, refused)
    before = sim.state()["balls"][0]
    sim.act(do="move_ball", index=0, dx=60, dy=0)
    after = sim.state()["balls"][0]
    check("the ball moves with its path", near(after["x"], before["x"] + 60, 0.2), (before, after))

    # Resume, then a new gateway process (each Drive button and each terminal command is one).
    sim.fresh()
    first = asyncio.create_task(asyncio.to_thread(sim.drive, command="forward"))
    await asyncio.sleep(1.0)
    sim.act(do="button", name="stop")
    status, reply = await first
    stopped = reply["result"].get("result") == "stopped"
    sim.act(do="adult_resume")
    await asyncio.sleep(0.5)
    status, reply = sim.drive(command="forward")
    check("after Resume, a new gateway process moves the robot again",
          stopped and reply["result"].get("result") == "done" and reply["result"]["state"]["cell"] == "C4", reply)

    # "Start again" while a command runs: refused, the gateway would keep a wrong position.
    sim.fresh()
    first = asyncio.create_task(asyncio.to_thread(sim.drive, command="forward"))
    await asyncio.sleep(0.7)
    refused = sim.act(do="reset")
    await first
    check("Start again while a command runs: refused", "error" in refused, refused)

    # Only this computer (or a Codespace's forwarded port) may use the page.
    port = int(sim.url.rsplit(":", 1)[1])
    conn = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
    conn.request("POST", "/api/drive", body=json.dumps({"command": "forward"}),
                 headers={"Host": "evil.example:%d" % port, "Content-Type": "application/json"})
    answer = conn.getresponse()
    check("a request for another host name is refused (DNS rebinding)", answer.status == 403, answer.status)
    conn.close()

    # Arrow keys: a side of the mat.
    sim.fresh()
    status, reply = sim.drive(command="go", direction="west")
    r = sim.state()["robot"]
    check("go west: a turn, then one cell", reply.get("moves") == ["left", "forward"] and r["cell"] == "B5"
          and near(r["heading"], 270), (reply, r))
    status, reply = sim.drive(command="go", direction="south")
    check("go south on the bottom row: it turns, then refuses to leave the mat",
          reply.get("moves") == ["left", "forward"] and reply["result"].get("result") == "refused", reply)

    # A ball rolls into the robot standing still: it bounces off the bumper, the robot stays.
    sim.fresh(balls=[{"from": [15, 135], "to": [145, 135], "radius": 7, "speed_cm_s": 20}])
    closest, bumped = 999, False
    for _ in range(60):
        s = sim.state()
        b, r = s["balls"][0], s["robot"]
        closest = min(closest, math.hypot(b["x"] - r["x"], b["y"] - r["y"]))
        await asyncio.sleep(0.05)
    s = sim.state()
    check("a ball hits the robot standing still: it bounces, the robot does not move",
          s["robot"]["bumps"] >= 1 and closest >= 10 + 7 - 1 and s["robot"]["cell"] == "C5"
          and any(w["line"] == "EVT BUMP" for w in s["wire"]), (s["robot"], closest))

    # Something comes close in front of the robot standing still: its light warns (reflex.warn).
    sim.fresh(balls=[{"from": [75, 30], "to": [75, 110], "radius": 7, "speed_cm_s": 20}])
    warned = False
    for _ in range(80):
        s = sim.state()
        warned = warned or (s["robot"]["warning"] and s["robot"]["led"] == "red")
        await asyncio.sleep(0.05)
    check("something close in front, robot still: the light warns, the robot stays",
          warned and any(w["line"] == "EVT NEAR" for w in sim.state()["wire"]) and sim.state()["robot"]["cell"] == "C5",
          sim.state()["robot"])

    # A ball hits the robot from the side while it moves: the bumper stops it.
    sim.fresh(balls=[{"from": [15, 120], "to": [145, 120], "radius": 7, "speed_cm_s": 30}])
    async with agent(env) as a:
        reply = await a.call("step_forward")
        r = sim.state()["robot"]
        check("a ball hits the moving robot: it stops", reply.get("result") == "stopped"
              and ("bumper" in reply.get("why", "") or "in front" in reply.get("why", "")) and not r["moving"]
              and r["collisions"] == 0, (reply, r))

    sim.fresh(balls=[{"from": [15, 15], "to": [45, 15], "radius": 5, "speed_cm_s": 0}])
    sim.act(do="move_ball", index=0, dx=1000, dy=-1000)
    ball = sim.state()["balls"][0]
    check("a ball dragged off the mat stays on it", ball["x"] <= MAT["cols"] * MAT["cell_cm"] - 5 + 0.1
          and ball["y"] >= 5 - 0.1, ball)

    # Events between two calls: the agent hears them at its next call, once; the page has its own count.
    sim.fresh(balls=[{"from": [15, 135], "to": [145, 135], "radius": 7, "speed_cm_s": 20}])
    async with agent(env) as a:
        await a.call("robot_state")
        await asyncio.sleep(3.0)
        reply = await a.call("robot_state")
        heard = [e["event"] for e in reply.get("since_last_call", [])]
        check("an event between two calls reaches the agent at its next call",
              any("bumper" in e for e in heard), reply.get("since_last_call"))
        sim.act(do="mode", mode="frozen")  # the ball stops: nothing new happens
        await asyncio.sleep(0.3)
        await a.call("robot_state")
        reply = await a.call("robot_state")
        check("each event is given once", not reply.get("since_last_call"), reply.get("since_last_call"))
        sim.act(do="mode", mode="realtime")
    status, reply = sim.drive(command="state")
    check("the page has its own count: it hears the same bump",
          any("bumper" in e["event"] for e in reply["result"].get("since_last_call", [])), reply)
    sim.act(do="button", name="stop")
    await asyncio.sleep(0.3)
    async with agent(env) as a:
        reply = await a.call("robot_state")
        check("the emergency button, pressed while nobody asked, is heard", reply.get("emergency_stop") is True
              and any("emergency" in e["event"] for e in reply.get("since_last_call", [])), reply)

    # wait_for_event: answers as soon as something happens, or when the time is over.
    sim.fresh(balls=[{"from": [15, 135], "to": [145, 135], "radius": 7, "speed_cm_s": 20}])
    async with agent(env) as a:
        await a.call("robot_state")
        started = time.monotonic()
        reply = await a.call("wait_for_event", seconds=10)
        took = time.monotonic() - started
        check("wait_for_event: answers when the ball touches the robot", reply.get("result") == "something happened"
              and any("bumper" in e["event"] for e in reply.get("events", [])) and took < 6, (reply, round(took, 1)))
    sim.fresh(balls=[{"from": [15, 135], "to": [145, 135], "radius": 7, "speed_cm_s": 20}])
    await asyncio.sleep(3.0)  # the ball touches the robot (about 2 s) before anybody watches
    async with agent(env) as a:
        reply = await a.call("wait_for_event", seconds=5)
        before = [e for e in reply.get("events", []) if "bumper" in e["event"]]
        check("wait_for_event: an event from before the wait is given, marked, and does not end the wait",
              before and before[0].get("before_you_started_waiting") is True
              and (reply.get("waited_seconds", 0) >= 4.5 or any(not e.get("before_you_started_waiting")
                                                                  for e in reply.get("events", []))), reply)
    sim.fresh(balls=[{"from": [15, 135], "to": [145, 135], "radius": 7, "speed_cm_s": 20}])
    await asyncio.sleep(3.0)  # the ball touches the robot before anybody watches
    sim.act(do="mode", mode="frozen")  # then the world waits: nothing new can happen
    async with agent(env) as a:
        started = time.monotonic()
        reply = await a.call("wait_for_event", seconds=3)
        took = time.monotonic() - started
        old = [e for e in reply.get("events", []) if "bumper" in e["event"]]
        check("wait_for_event with only events from before: it waits the whole time, gives them, and their age",
              reply.get("result") == "nothing new happened" and took >= 2.8 and old
              and old[0].get("before_you_started_waiting") is True and old[0]["seconds_before_this_answer"] >= 3,
              (reply, round(took, 1)))
    sim.act(do="mode", mode="realtime")
    sim.fresh()
    async with agent(env) as a:
        await a.call("robot_state")
        reply = await a.call("wait_for_event", seconds=1)
        check("wait_for_event: nothing happens, it answers after the time", reply.get("result") == "nothing happened",
              reply)
        watching = asyncio.create_task(a.call("wait_for_event", seconds=4))
        await asyncio.sleep(1.0)
        status, reply = await asyncio.to_thread(sim.drive, command="state")
        await watching
        check("while an agent watches, the page can still ask", status == 200
              and reply["result"].get("result") != "refused", (status, reply))

    # ROBOT_CHANNEL=1: the gateway pushes each event (Claude Code channel), on the legacy handshake.
    sim.fresh(balls=[{"from": [15, 135], "to": [145, 135], "radius": 7, "speed_cm_s": 20}])
    init, pushed = await channel_session(env, seconds=6)
    capabilities = init.get("result", {}).get("capabilities", {})
    check("channel: the gateway declares claude/channel and says how to read the events",
          "claude/channel" in capabilities.get("experimental", {})
          and "<channel" in init.get("result", {}).get("instructions", ""), init)
    check("channel: a bump is pushed without any call", any("bumper" in p["params"]["content"] for p in pushed),
          pushed)

    # The simulator restarts on another port while an agent keeps the gateway: it follows the new port.
    sim.fresh()
    async with agent(env) as a:
        await a.call("robot_state")
        sim.stop()
        sim.__init__(Path(env["ROBOT_STATE_DIR"]), Path(env["ROBOT_MAT"]))
        sim.fresh()
        reply = await a.call("robot_state")
        check("the simulator restarts: the same gateway follows its new port", reply.get("cell") == "C5", reply)

    # A cell of the edge can be entered (the fence is outside the mat), and the gateway stops at the edge.
    sim.fresh()
    async with agent(env) as a:
        await a.call("step_forward")
        await a.call("turn", direction="right")
        await a.call("turn", direction="right")
        reply = await a.call("step_forward")
        r = sim.state()["robot"]
        check("a cell of the edge can be entered facing the fence (C4 to C5, facing south)",
              reply.get("result") == "done" and r["cell"] == "C5" and r["collisions"] == 0, (reply, r))

    # What the camera saw: noted by the robot while it stands still, put on its cell by the gateway.
    sim.fresh(cards=[{"cell": "C3", "label": "water-point"}])
    sim.act(do="noise", camera_distance=0.05)
    async with agent(env) as a:
        step = await a.call("step_forward")
        await asyncio.sleep(0.6)
        reply = await a.call("robot_state")
        # The answer of the step already waited for the camera to look: the card may come with it.
        heard = [e["event"] for e in step.get("since_last_call", []) + reply.get("since_last_call", [])]
        check("the camera's card reaches the agent, with where it was seen from, its distance and cell",
              any("water-point" in e and "seen from C4 facing north" in e and "cm away" in e and "C3" in e for e in heard),
              heard)
        cards = (await a.call("seen_cards"))["cards_seen"]
        check("seen_cards: one card, its cell, seen again from closer (C4)",
              len(cards) == 1 and cards[0]["cell"] == "C3" and cards[0]["seen_from"] == "C4 facing north", cards)
        sim.act(do="move_card", **{"from": "C3", "to": "B2"})
        await asyncio.sleep(0.6)
        cards = (await a.call("seen_cards"))["cards_seen"]
        check("a card moved by hand while the robot looks: it leaves C3, and is seen again in B2",
              [x["cell"] for x in cards] == ["B2"] or ("B2" in cards[0]["could_also_be"] and len(cards) == 1), cards)

    sim.fresh(cards=[{"cell": "C4", "label": "barbecue"}])
    sim.act(do="noise", camera_miss=1.0)
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        await a.call("robot_state")
        cards = (await a.call("seen_cards"))["cards_seen"]
        check("a camera that misses everything: nothing is noted", cards == [], cards)
        sim.act(do="noise", camera_miss=0.0)
        await asyncio.sleep(0.6)
        cards = (await a.call("seen_cards"))["cards_seen"]
        check("the camera sees again: the card is noted", [x["cell"] for x in cards] == ["C4"], cards)

    sim.fresh(cards=[{"cell": "D4", "label": "dry-grass"}])
    async with agent(env) as a:
        await a.call("turn", direction="right")
        await asyncio.sleep(0.6)
        reply = await a.call("seen_cards")
        check("a card out of the cone (D4, 45 degrees to the side) is not seen", reply["cards_seen"] == []
              and not any("dry-grass" in e["event"] for e in reply.get("since_last_call", [])), reply)

    # Farther: less sure of the cell; seen again from closer, the estimate gets better.
    sim.fresh(cards=[{"cell": "C2", "label": "barbecue"}])
    sim.act(do="noise", camera_distance=0.1)
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        far = (await a.call("seen_cards"))["cards_seen"]
        await a.call("step_forward")
        await asyncio.sleep(0.6)
        close = (await a.call("seen_cards"))["cards_seen"]
        check("a card 90 cm away is seen, its cell not sure",
              len(far) == 1 and "C2" in [far[0]["cell"]] + far[0]["could_also_be"] and far[0]["could_also_be"], far)
        check("seen again from closer: the same card, a better estimate",
              len(close) == 1 and close[0]["cell"] == "C2" and close[0]["distance_cm"] < far[0]["distance_cm"]
              and len(close[0]["could_also_be"]) < len(far[0]["could_also_be"]), (far, close))

    sim.fresh(cards=[{"cell": "D3", "label": "water-point"}])
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        cards = (await a.call("seen_cards"))["cards_seen"]
        check("the cone sees to the side when it sees far (D3 from C5 facing north)",
              len(cards) == 1 and "D3" in [cards[0]["cell"]] + cards[0]["could_also_be"], cards)

    sim.fresh(cards=[{"cell": "C2", "label": "barbecue"}], boxes=[{"x": 65, "y": 65, "w": 20, "h": 20, "name": "box"}])
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        hidden = (await a.call("seen_cards"))["cards_seen"]
        sim.act(do="remove_box", id=0)
        await asyncio.sleep(0.6)
        shown = (await a.call("seen_cards"))["cards_seen"]
        check("a box hides the card behind it; without the box, the card is seen", hidden == [] and len(shown) == 1,
              (hidden, shown))

    sim.fresh(cards=[{"cell": "C2", "label": "dry-grass"}, {"cell": "D2", "label": "dry-grass"}])
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        cards = (await a.call("seen_cards"))["cards_seen"]
        check("two like cards side by side stay two", len(cards) == 2, cards)

    # What the camera has looked at: cells in its cone, unless what the distance sensor met stands between.
    sim.fresh()
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        await a.call("robot_state")
        cover = (await a.call("seen_cards"))["camera_coverage"]
        left = set(cover["not_looked_at_yet"])
        check("coverage, from C5 facing north: C1 to C4, B2, B3, D2, D3 (in the cone); not B4 (45 degrees)",
              not left & {"C1", "C2", "C3", "C4", "C5", "B2", "B3", "D2", "D3"} and "B4" in left
              and cover["cells_on_the_mat"] == 25 and cover["cells_looked_at"] == 9, cover)
        await a.call("look_around")
        cover = (await a.call("seen_cards"))["camera_coverage"]
        left = set(cover["not_looked_at_yet"])
        check("coverage after look_around: the row of C5 too", not left & {"A5", "B5", "D5", "E5"}, cover)
        await a.call("step_forward")
        await asyncio.sleep(0.6)
        state = await a.call("robot_state")
        cover = (await a.call("seen_cards"))["camera_coverage"]
        await a.call("face", direction="east")
        await asyncio.sleep(0.6)
        state = await a.call("robot_state")
        cover = (await a.call("seen_cards"))["camera_coverage"]
        check("coverage grows as the robot goes (C4 facing east: D4, E4, E3), and robot_state counts what is left",
              not set(cover["not_looked_at_yet"]) & {"D4", "E4", "E3"}
              and state["cells_not_looked_at_yet"] == len(cover["not_looked_at_yet"]), (state, cover))

    sim.fresh(boxes=[{"x": 65, "y": 65, "w": 20, "h": 20, "name": "box"}])
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        await a.call("robot_state")
        cover = (await a.call("seen_cards"))["camera_coverage"]
        check("coverage: a box in C3 hides C2, C1 and B2 (the line to B2 crosses C3), and its own cell until it is "
              "set apart (the camera sees its side in front of the centre)",
              {"C2", "C1", "B2", "D2", "C3"} <= set(cover["not_looked_at_yet"])
              and "B3" not in cover["not_looked_at_yet"], cover)
        check("coverage: the box seen from one point of view only: not yet said to stand there",
              cover["something_stands_there"] == [], cover)

    # A camera that does not name the box (it misses it): the distance sensor alone, at two moments.
    sim.fresh(boxes=[{"x": 65, "y": 65, "w": 20, "h": 20, "name": "box"}])
    sim.act(do="noise", camera_miss=1.0)
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        await a.call("robot_state")
        cover = (await a.call("seen_cards"))["camera_coverage"]
        check("coverage, the box not named by the camera: met once by the sensor, not yet said to stand there (a ball "
              "rolls on)", cover["something_stands_there"] == [], cover)
        await asyncio.sleep(10.5)
        await a.call("robot_state")
        cover = (await a.call("seen_cards"))["camera_coverage"]
        check("coverage: met again 10 s later, something stands on C3, set apart from the cells looked at",
              cover["something_stands_there"] == ["C3"] and "C3" not in cover["not_looked_at_yet"], cover)
    sim.act(do="noise", camera_miss=0.0)

    # A box the distance sensor does not see (to one side, in B3): the camera names it, and the cell behind it (B2) is
    # not counted looked at (facilitator/workshops/measures/2026-11-07-robot-diagonal/).
    sim.fresh(boxes=[{"x": 40, "y": 62, "w": 18, "h": 18, "name": "box"}], cards=[{"cell": "B2", "label": "barbecue"}])
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        state = await a.call("robot_state")
        result = await a.call("seen_cards")
        cover = result["camera_coverage"]
        check("a box to one side, that the distance sensor never meets: the cell it hides (B2, with a card) is left to "
              "look at, C2 and C3 are looked at",
              "B2" in cover["not_looked_at_yet"] and not {"C2", "C3"} & set(cover["not_looked_at_yet"])
              and result["cards_seen"] == [] and state.get("distance_ahead_cm", 0) > 100, (state, cover))
        await a.call("look_around")
        await a.call("move", direction="west")
        await a.call("move", direction="west")
        cover = (await a.call("seen_cards"))["camera_coverage"]
        check("the same box seen from three points of view (C5, B5, A5 facing north): B3 stands apart",
              "B3" in cover["something_stands_there"] and sim.state()["robot"]["cell"] == "A5", cover)
        poses = [json.loads(line) for line in (Path(env["ROBOT_STATE_DIR"]) / "poses.jsonl").read_text().splitlines()]
        stood = [(p["cell"], p["facing"]) for p in poses if p["looked"] and not p["moving"]]
        check("poses.jsonl keeps every pose, with where the robot stood and whether its camera looked",
              ("C5", "north") in stood and ("B5", "north") in stood and ("A5", "north") in stood
              and all(set(p) == {"pose", "cell", "facing", "looked", "moving", "at"} for p in poses), poses[-6:])

    # Every cell looked at: the agent hears it once, with the cards seen.
    state_file = Path(env["ROBOT_STATE_DIR"]) / "terrain.json"
    sim.fresh(cards=[{"cell": "C3", "label": "water-point"}])
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        await a.call("robot_state")
        saved = json.loads(state_file.read_text())
        saved["looked"] = {chr(65 + c) + str(r + 1): saved["looked"]["C5"] for c in range(5) for r in range(5)
                           if (c, r) != (2, 2)}
        state_file.write_text(json.dumps(saved))
        first = await a.call("robot_state")
        again = await a.call("robot_state")
        heard = [e["event"] for e in first.get("since_last_call", [])]
        check("every cell looked at: said once, with the cards seen",
              any("every cell" in e and "water-point" in e and "any other card is not on the mat" in e for e in heard)
              and not any("every cell" in e["event"] for e in again.get("since_last_call", []))
              and first["cells_not_looked_at_yet"] == 0, (first, again))
    sim.fresh()
    await asyncio.sleep(0.6)
    async with agent(env) as a:
        cover = (await a.call("seen_cards"))["camera_coverage"]
        check("put back on its start cell: the coverage starts again", cover["cells_looked_at"] <= 9, cover)

    # A moving ball in the camera's view: which way it goes, and whether it comes closer.
    sim.fresh(cards=[{"cell": "C2", "label": "barbecue"}],
              balls=[{"from": [75, 15], "to": [75, 95], "radius": 7, "speed_cm_s": 10}])
    async with agent(env) as a:
        first = await a.call("robot_state")  # the ball starts coming closer within the first second
        await asyncio.sleep(13.0)
        reply = await a.call("robot_state")
        heard = [e["event"] for r in (first, reply) for e in r.get("since_last_call", []) if "ball" in e["event"]]
        check("the camera sees the ball coming closer (moving south, column C)",
              any("getting closer" in e and "moving south" in e and "C" in e.split(": ")[1][:20] for e in heard), heard)
        check("then moving away (moving north)", any("moving away" in e and "moving north" in e for e in heard), heard)
        check("robot_state gives the last sighting of the ball", (reply.get("ball_last_seen") or {}).get("how")
              in ("getting closer to the robot", "moving away from the robot", "at the same distance from the robot"), reply.get("ball_last_seen"))
        cards = (await a.call("seen_cards"))["cards_seen"]
        check("a card is still noted while a ball moves in the view", [x["card"] for x in cards] == ["barbecue"], cards)

    # Several moves sent at once (parallel tool calls): one runs, the others are refused.
    sim.fresh()
    async with agent(env) as a:
        replies = await asyncio.gather(a.call("step_forward"), a.call("step_forward"), a.call("turn", direction="right"))
        done = [r for r in replies if r.get("result") == "done"]
        refused = [r for r in replies if "one move at a time" in r.get("why", "")]
        r = sim.state()["robot"]
        check("moves sent at once: one runs, the others are refused (never an order nobody chose)",
              len(done) == 1 and len(refused) == 2 and r["collisions"] == 0, (replies, r))

    # Real time and frozen time.
    sim.fresh(balls=[{"from": [15, 15], "to": [135, 15], "radius": 5, "speed_cm_s": 10}])
    sim.act(do="mode", mode="realtime")
    first = sim.state()["balls"][0]
    await asyncio.sleep(1.0)
    check("real time: the world goes on while nobody moves", sim.state()["balls"][0] != first)
    sim.act(do="mode", mode="frozen")
    first = sim.state()["balls"][0]
    await asyncio.sleep(1.0)
    check("frozen: the world waits for the robot", sim.state()["balls"][0] == first)
    sim.act(do="mode", mode="realtime")


def mission(env, *args, wait=True):
    """The Interpreter's stand-in: stack/mission.py, in a process of its own."""
    command = [PYTHON, "-I", str(GUIDE / "stack" / "mission.py"), *args]
    if not wait:
        return subprocess.Popen(command, env=env, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    return subprocess.run(command, env=env, capture_output=True, text=True, timeout=300)


def stack_scenarios(sim, env):
    """The stack above the Referee: a mission in the closed format, through the Strategist, the Pilot and the
    Cartographer, with no model (the Pilot alone) or a scripted one. No network, no key."""
    env = {k: v for k, v in env.items() if k != "ROBOT_RULES"}  # the rules of the guide: no limit on the moves
    big = dict(MAT, max_moves=40)
    world = {"cards": [{"cell": "C2", "label": "barbecue"}], "boxes": [], "balls": []}
    sim.act(do="load_world", mat=big, world=world)
    done = mission(env, "reach_card:barbecue", "set_light:green", "--model", "none")
    report = json.loads(done.stdout or "{}")
    r = sim.state()["robot"]
    check("mission without a model: the Pilot alone stops in front of the barbecue, light green",
          report.get("outcome") == "done" and {"card": "barbecue", "cell": "C2"} in report.get("cards", [])
          and r["cell"] == "C3" and r["collisions"] == 0, (done.stderr[-500:], report, r))
    counts = report.get("counts") or {}
    check("the report counts the mission: one mission, two operations, no call to the model, the cells the robot "
          "went through (C5 to C3), the commands sent; no limit on the moves by default",
          counts.get("missions") == 1 and counts.get("operations") == 2 and counts.get("model_calls") == 0
          and counts.get("cells") == 2 and counts.get("commands", 0) >= 2 and counts.get("moves", 0) >= 1
          and report.get("moves_left") is None, report)

    sim.act(do="load_world", mat=big, world=world)
    done = mission(env, "find_card:barbecue", "--model", "fake")
    report = json.loads(done.stdout or "{}")
    check("mission with a scripted model: it looks, reads the record, reports the card and its cell",
          report.get("outcome") == "done" and report.get("cards") == [{"card": "barbecue", "cell": "C2"}]
          and report.get("turns") == 4, (done.stderr[-500:], report))
    counts = report.get("counts") or {}
    check("its counts: one mission (the counts start again), four calls to the model, the operations it asked",
          counts.get("missions") == 1 and counts.get("model_calls") == 4 and counts.get("operations", 0) >= 2, report)
    runs = sorted((Path(env["ROBOT_STATE_DIR"]) / "missions").iterdir())
    sent = (runs[-1] / "transcript.jsonl").read_text(encoding="utf-8").splitlines()
    check("the model received the mission built from the closed format, never other words",
          json.loads(sent[1])["content"] == 'You drive a small robot on a mat with the robot tools. Find out whether '
          'the card "barbecue" is on the mat, and on which cell.', sent[1])

    sent = wire_count(sim, "MOVE") + wire_count(sim, "TURN")
    for args, what in ((["reach_card:picnic"], "a card the camera does not know"),
                       (["dance:barbecue"], "a step that is not on the list"),
                       (["set_light:purple"], "a colour that is not on the list"),
                       (["reach_card:barbecue", "--personality", "reckless"], "a personality that does not exist")):
        done = mission(env, *args, "--model", "fake")
        check(f"mission refused, nothing sent: {what}", done.returncode != 0 and "refused" in done.stderr + done.stdout
              and wire_count(sim, "MOVE") + wire_count(sim, "TURN") == sent, (done.stdout, done.stderr[-300:]))

    sim.act(do="load_world", mat=big, world=world)
    mission(env, "find_card:barbecue", "--model", "fake", "--personality", "cautious")
    runs = sorted((Path(env["ROBOT_STATE_DIR"]) / "missions").iterdir())
    kept = json.loads((runs[-1] / "mission.json").read_text(encoding="utf-8"))
    system = json.loads((runs[-1] / "transcript.jsonl").read_text(encoding="utf-8").splitlines()[0])["content"]
    check("personality cautious: a brick of the Pilot (2 cells at most) and the instructions to the model",
          kept["settings"]["max_leg"] == 2 and kept["settings"]["careful_balls"] is True
          and "call `look_around` before every `go_to_cell`" in system, (kept["settings"], system[-400:]))

    sim.act(do="load_world", mat=big, world=world)
    running = mission(env, "find_card:barbecue", "--model", "fake", wait=False)
    time.sleep(1.5)
    mission(env, "stop")
    out, err = running.communicate(timeout=60)
    report = json.loads(out or "{}")
    check("mission stopped by the Interpreter: the loop ends it without asking the model",
          report.get("outcome") == "stopped" and report.get("turns", 9) < 4, (report, err[-300:]))

    # The robot waits for an adult in the middle of a mission (the emergency button during look_around): the Strategist
    # waits without its model, says it for the Interpreter (waiting.json), and calls the model again on Resume.
    state_dir = Path(env["ROBOT_STATE_DIR"])
    sim.act(do="load_world", mat=big, world=world)
    running = mission(env, "find_card:barbecue", "--model", "fake", wait=False)
    for _ in range(40):
        if sim.state()["robot"]["moving"]:
            break
        time.sleep(0.25)
    sim.act(do="button", name="stop")
    waiting = None
    for _ in range(60):
        if (state_dir / "waiting.json").exists():
            waiting = json.loads((state_dir / "waiting.json").read_text(encoding="utf-8"))
            break
        time.sleep(0.5)
    runs = sorted((state_dir / "missions").iterdir())
    lines = len((runs[-1] / "transcript.jsonl").read_text(encoding="utf-8").splitlines())
    time.sleep(3.0)
    still = len((runs[-1] / "transcript.jsonl").read_text(encoding="utf-8").splitlines())
    sim.act(do="adult_resume")
    out, err = running.communicate(timeout=120)
    report = json.loads(out or "{}")
    told = (runs[-1] / "transcript.jsonl").read_text(encoding="utf-8")
    check("the robot waits for an adult during a mission: the model is not called meanwhile, the Interpreter can say "
          "what to do (waiting.json), and the model hears the event on Resume, then reports",
          waiting is not None and "adult" in (waiting.get("what_to_do") or "") and still == lines
          and "Event: an adult put the robot back and pressed Resume" in told
          and report.get("outcome") in ("done", "impossible")
          and not (state_dir / "waiting.json").exists(), (waiting, lines, still, report, err[-400:]))

    # The same without a model: a box dropped in front during a slide north; after the adult, the step again.
    sim.act(do="load_world", mat=big, world=world)
    running = mission(env, "reach_card:barbecue", "--model", "none", wait=False)
    box = None
    for _ in range(120):
        r = sim.state()["robot"]
        if r["moving"] and near(r["heading"] % 360, 0, 3) and r["y"] < 133:
            box = box_in_front(sim, 4)
            break
        time.sleep(0.1)
    waiting = None
    for _ in range(60):
        if (state_dir / "waiting.json").exists():
            waiting = json.loads((state_dir / "waiting.json").read_text(encoding="utf-8"))
            break
        time.sleep(0.5)
    if box is not None:
        sim.act(do="remove_box", id=box)
    sim.act(do="adult_resume")
    out, err = running.communicate(timeout=180)
    report = json.loads(out or "{}")
    r = sim.state()["robot"]
    check("without a model, a box dropped in front during the mission: the Strategist waits for the adult, then tries "
          "the step again and stops in front of the barbecue", box is not None and waiting is not None
          and report.get("outcome") == "done" and r["cell"] == "C3" and r["collisions"] == 0, (box, waiting, report, r,
                                                                                               err[-400:]))

    # The pause during a long slide: the robot stops on the next cell, as soon as possible, no adult needed (HOLD).
    sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
    script = ("import json, sys, threading; sys.path.insert(0, 'stack'); import common, pilot; "
              "say = lambda do: (common.STATE_DIR / 'control.json').write_text(json.dumps({'do': do})); "
              "say('go'); pilot.OBEY_MISSION_CONTROL = True; threading.Timer(2.0, say, ['pause']).start(); "
              "print(json.dumps(pilot.move('north', 4)))")
    done = subprocess.run([PYTHON, "-I", "-c", script], env=env, cwd=GUIDE, capture_output=True, text=True, timeout=60)
    reply = json.loads(done.stdout or "{}")
    r = sim.state()["robot"]
    check("pause during a slide of 4 cells: HOLD, it stops on the next cell (C4), no adult needed",
          reply.get("result") == "done" and reply.get("cells_moved") == 1 and "next cell" in reply.get("stopped_because", "")
          and r["cell"] == "C4" and wire_count(sim, "HOLD") == 1 and not reply["state"]["waiting_for_adult"],
          (reply, r, done.stderr[-400:]))
    (Path(env["ROBOT_STATE_DIR"]) / "control.json").write_text(json.dumps({"do": "go"}))

    # Where the robot is on the mat, when nobody says where it starts: rules of the Referee with the start "unknown".
    rules = json.loads((GUIDE / "stack" / "rules.json").read_text(encoding="utf-8"))
    state_dir = Path(env["ROBOT_STATE_DIR"])

    def with_rules(name, **changes):
        path = state_dir / f"rules-{name}.json"
        path.write_text(json.dumps(dict(rules, **changes)), encoding="utf-8")
        return dict(env, ROBOT_RULES=str(path))

    def drive(env_rules, *args):
        done = subprocess.run([PYTHON, "-I", str(GUIDE / "stack" / "drive.py"), *args], env=env_rules,
                              capture_output=True, text=True, timeout=60)
        return json.loads(done.stdout or "{}")

    # A limit of the adults on another counter: two commands, the third move is refused, nothing sent.
    two = with_rules("two-commands", limits={"commands": 2})
    sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
    drive(two, "forward")
    drive(two, "forward")
    sent = wire_count(sim, "MOVE")
    third = drive(two, "forward")
    state = drive(two, "state")
    check("rules with at most 2 commands: the third move is refused in words, nothing sent; robot_state shows the "
          "counts and the limit", third.get("result") == "refused" and "the adults allow 2 commands" in third.get("why", "")
          and wire_count(sim, "MOVE") == sent and (state.get("counts") or {}).get("commands") == 2
          and (state.get("counts") or {}).get("limits_per_mission") == {"commands": 2}
          and (state.get("counts") or {}).get("cells") == 2, (third, state))

    # Found by the tier probes (ci/probe_tiers.py, 2026-11-07): each check fails on the stack before the fix.
    # A step whose own reading settles the place: the cell ahead is taken after that reading, on the mat.
    script = ("import json, sys; sys.path.insert(0, 'stack'); import cartographer; "
              f"mat = {MAT!r}; rules = dict(json.load(open('stack/rules.json')), start='unknown'); "
              "c = cartographer.Cartographer(mat, rules); sent = []; "
              "c.fields = lambda: (c.frame and c.settle([2, 4, 0], 'a test'), {'card': 'none', 'dist': '200'})[1]; "
              "c.referee.step = lambda t, target, card: sent.append(list(target)) or {'result': 'refused', 'rule': 'off_mat'}; "
              "c.referee.step_blind = lambda t, target, *a: sent.append(['blind'] + list(target)) or {'result': 'refused', 'rule': 'off_mat'}; "
              "c.step_forward(); print(json.dumps(sent))")
    fresh = Path(tempfile.mkdtemp(dir=env["ROBOT_STATE_DIR"]))  # a Cartographer of its own: no terrain.json yet
    done = subprocess.run([PYTHON, "-I", "-c", script], env=dict(env, ROBOT_STATE_DIR=str(fresh)), cwd=GUIDE,
                          capture_output=True, text=True, timeout=60)
    sent = json.loads(done.stdout or "null")
    check("a step whose own reading settles the place: the Referee judges the cell ahead on the mat (C4), not a cell "
          "of the frame relative to the start", sent == [[2, 3]], (sent, done.stderr[-300:]))

    # A ball's lane on a cell that falls off the mat when the place is found: forgotten, never 'cell:'.
    script = ("import json, sys, time; sys.path.insert(0, 'stack'); import cartographer; "
              f"mat = {MAT!r}; rules = dict(json.load(open('stack/rules.json')), start='unknown'); "
              "c = cartographer.Cartographer(mat, rules); now = time.time(); "
              "c.lanes = {'cell:A1': now, 'col:0': now, 'row:2': now}; "
              "c.settle([2, 4, 0], 'a test'); found = dict(c.lanes); "
              "c.unsettle(None, 1); print(json.dumps([sorted(found), sorted(c.lanes)]))")
    fresh = Path(tempfile.mkdtemp(dir=env["ROBOT_STATE_DIR"]))  # a Cartographer of its own: no terrain.json yet
    done = subprocess.run([PYTHON, "-I", "-c", script], env=dict(env, ROBOT_STATE_DIR=str(fresh)), cwd=GUIDE,
                          capture_output=True, text=True, timeout=60)
    lanes = json.loads(done.stdout or "null")
    check("a ball's lane on a cell that falls off the mat when the place is found is forgotten (never an empty name), "
          "the row kept, and the place can be lost again", lanes is not None and "cell:" not in lanes[0] + lanes[1]
          and "cell:A1" not in lanes[0] and "row:2" in lanes[0], (lanes, done.stderr[-300:]))

    # Place unknown: the last ball seen is given without a cell (a cell of the frame relative to the start means nothing).
    script = ("import json, sys, time; sys.path.insert(0, 'stack'); import cartographer; "
              f"mat = {MAT!r}; rules = dict(json.load(open('stack/rules.json')), start='unknown'); "
              "c = cartographer.Cartographer(mat, rules); "
              "c.ball = {'cell': 'E3', 'could_also_be': ['E2'], 'moving': 'south', 'how': 'getting closer to the robot', "
              "'seen_from': 'where it started, facing the way it faced at the start', 'from': [4, 4, 0], 'at': '12:00:00', "
              "'t': time.time()}; "
              "c.fields_looked = lambda: {'card': 'none', 'dist': 'none'}; "
              "print(json.dumps(c.robot_state()['ball_last_seen']))")
    fresh = Path(tempfile.mkdtemp(dir=env["ROBOT_STATE_DIR"]))  # a Cartographer of its own: no terrain.json yet
    done = subprocess.run([PYTHON, "-I", "-c", script], env=dict(env, ROBOT_STATE_DIR=str(fresh)), cwd=GUIDE,
                          capture_output=True, text=True, timeout=60)
    ball = json.loads(done.stdout or "null") or {}
    check("place unknown: the last ball seen has no cell and no side of the mat, only how it moved and when",
          ball.get("cell") is None and ball.get("could_also_be") == [] and ball.get("moving") is None
          and ball.get("how") == "getting closer to the robot" and ball.get("t"), (ball, done.stderr[-300:]))

    # Two missions in one process (the Interpreter's link): between them the robot is driven back by hand, and the
    # Pilot's count of the first mission is used up. The second mission reads the robot's state again.
    six = with_rules("six-moves", limits={"moves": 6})
    sim.act(do="load_world", mat=big, world=world)
    script = ("import json, sys; sys.path.insert(0, 'stack'); import pilot, strategist; "
              "one = {'steps': [{'do': 'reach_card', 'card': 'barbecue'}]}; first = strategist.run(one, model='none'); "
              "pilot.OBEY_MISSION_CONTROL = False; pilot.move('south', 2); pilot.PILOT.moves_left = 0; "
              "print(json.dumps([first, strategist.run(one, model='none')]))")
    done = subprocess.run([PYTHON, "-I", "-c", script], env=six, cwd=GUIDE, capture_output=True, text=True, timeout=120)
    first, second = (json.loads(done.stdout) if done.stdout else [{}, {}])
    r = sim.state()["robot"]
    check("two missions in one process, the robot moved back by hand between them: the second starts from where the "
          "robot is, with the moves of a new mission, and stops in front of the barbecue again",
          first.get("outcome") == "done" and second.get("outcome") == "done" and "no moves left" not in str(second)
          and r["cell"] == "C3" and r["collisions"] == 0, (first, second, r, done.stderr[-400:]))

    # A model that sends the same refused call again and again (Mistral Small 3.2 on Albert, 44 times): it is told from
    # the second time, and the mission ends at the fourth, without using up the quota.
    out = Path(tempfile.mkdtemp(dir=env["ROBOT_STATE_DIR"]))
    script = ("import json, sys; sys.path.insert(0, 'stack'); import strategist; "
              "bad = {'tool_calls': [{'id': 'c', 'type': 'function', 'function': {'name': 'look_around', "
              "'arguments': '{\"seconds\": 1}'}}]}; "
              "strategist.api_model = lambda *a, **k: (dict(bad), {}, 'tool_calls'); "
              f"print(json.dumps(strategist.run({{'steps': [{{'do': 'find_card', 'card': 'barbecue'}}]}}, "
              f"model='loop', out={str(out)!r})))")
    done = subprocess.run([PYTHON, "-I", "-c", script], env=env, cwd=GUIDE, capture_output=True, text=True, timeout=120)
    looped = json.loads(done.stdout or "{}")
    told = [json.loads(line)["content"] for line in (out / "transcript.jsonl").read_text().splitlines()
            if json.loads(line).get("role") == "tool"] if (out / "transcript.jsonl").exists() else []
    check("the same refused call four times in a row: the model is told from the second, the mission ends (limit)",
          looped.get("outcome") == "limit" and looped.get("turns") == 4 and len(told) == 4
          and "refused" in told[0] and "times in a row" not in told[0] and "refused 2 times in a row" in told[1],
          (looped, told[:2], done.stderr[-400:]))

    # A report that names a card the record does not place there (Mistral Small 3.2 on Albert: the barbecue on D4) is
    # refused, with the record; the model reports again, with what was seen. The world has no water point. Giving up
    # with cells the camera has not looked at is refused once too (Qwen 3 Coder on Albert gave up with two cells left);
    # the third report is taken.
    out = Path(tempfile.mkdtemp(dir=env["ROBOT_STATE_DIR"]))
    script = ("import json, sys; sys.path.insert(0, 'stack'); import strategist; "
              "calls = iter([{'cards': [{'card': 'water-point', 'cell': 'A1'}], 'outcome': 'done'}, "
              "{'cards': [], 'outcome': 'impossible', 'why': 'not seen yet'}, "
              "{'cards': [], 'outcome': 'impossible', 'why': 'not seen yet'}]); "
              "strategist.api_model = lambda *a, **k: ({'tool_calls': [{'id': 'r', 'type': 'function', 'function': "
              "{'name': 'report', 'arguments': json.dumps(next(calls))}}]}, {}, 'tool_calls'); "
              f"print(json.dumps(strategist.run({{'steps': [{{'do': 'find_card', 'card': 'water-point'}}]}}, "
              f"model='invents', out={str(out)!r})))")
    done = subprocess.run([PYTHON, "-I", "-c", script], env=env, cwd=GUIDE, capture_output=True, text=True, timeout=120)
    invented = json.loads(done.stdout or "{}")
    told = [json.loads(line)["content"] for line in (out / "transcript.jsonl").read_text().splitlines()
            if json.loads(line).get("role") == "tool"] if (out / "transcript.jsonl").exists() else []
    check("a report with a card the record does not place there is refused, with the record; the next report, "
          "without it, is refused once for the cells left, the third ends the mission",
          invented.get("outcome") == "impossible" and invented.get("cards") == []
          and len(told) == 3 and told[0].startswith('{"result": "refused", "why": "the record does not place '
                                                     'water-point on A1') and '"seen_cards"' in told[0]
          and told[1].startswith('{"result": "refused", "why": "the camera has not looked at every cell of the mat yet: ')
          and '"not_looked_at_yet"' in told[1] and '"done"' in told[2], (invented, told, done.stderr[-400:]))

    # An HTTP error of the model's API that the same request sent again does not give (Albert, 2026-11-08): tried again,
    # the mission goes on; an error that stays ends it after three tries. The provider's words go to usage.json only.
    for name, errors, want in (("transient", 1, "impossible"), ("lasting", 9, "api error")):
        out = Path(tempfile.mkdtemp(dir=env["ROBOT_STATE_DIR"]))
        script = "\n".join([
            "import io, json, sys, urllib.error",
            "sys.path.insert(0, 'stack')",
            "import strategist",
            "strategist.API_WAITS_S = (0, 0, 0)",
            "left = [int(sys.argv[1])]",
            "call = {'id': 'r', 'type': 'function', 'function': {'name': 'report', 'arguments': "
            "json.dumps({'outcome': 'impossible', 'why': 'not seen yet'})}}",
            "ok = json.dumps({'choices': [{'message': {'tool_calls': [call]}, 'finish_reason': 'tool_calls'}], "
            "'usage': {}}).encode()",
            "def urlopen(*a, **k):",
            "    if left[0] > 0:",
            "        left[0] -= 1",
            "        raise urllib.error.HTTPError('u', 400, 'Bad Request', {}, io.BytesIO(b'upstream busy'))",
            "    return io.BytesIO(ok)",
            "strategist.urllib.request.urlopen = urlopen",
            f"print(json.dumps(strategist.run({{'steps': [{{'do': 'set_light', 'color': 'green'}}]}}, "
            f"model='openrouter:test/model', key='fake-key-for-the-test-0000', out={str(out)!r})))"])
        done = subprocess.run([PYTHON, "-I", "-c", script, str(errors)], env=env, cwd=GUIDE, capture_output=True,
                              text=True, timeout=120)
        final = json.loads(done.stdout.strip().splitlines()[-1]) if done.stdout.strip() else {}
        usage = json.loads((out / "usage.json").read_text()) if (out / "usage.json").exists() else {}
        check(f"an HTTP 400 of the model's API, {name}: tried again ({'the mission goes on' if errors == 1 else 'three tries, then api error'}); "
              "the provider's words in usage.json, not in the report",
              final.get("outcome") == want and len(usage.get("api_errors", [])) == min(errors, 3)
              and "upstream busy" in (usage.get("api_errors") or [""])[0] and "upstream busy" not in json.dumps(final)
              and (errors == 1 or "upstream busy" in usage.get("api_error", "")), (final, usage, done.stderr[-400:]))

    # A card seen while the place on the mat is unknown has no cell in the record yet: the Pilot finds its place first,
    # then refuses a cell that holds a card.
    unknown = with_rules("unknown", start="unknown")
    sim.act(do="load_world", mat=big, world={"cards": [{"cell": "C3", "label": "barbecue"}], "boxes": [], "balls": []})
    time.sleep(0.6)
    script = ("import json, sys; sys.path.insert(0, 'stack'); import pilot; pilot.robot_state(); "
              "print(json.dumps([pilot.cards(), pilot.go_to_cell('C3')]))")
    done = subprocess.run([PYTHON, "-I", "-c", script], env=unknown, cwd=GUIDE, capture_output=True, text=True, timeout=120)
    record, reply = (json.loads(done.stdout) if done.stdout else [{}, {}])
    r = sim.state()["robot"]
    check("start unknown, a card seen before the place is found (no cell in the record): go_to_cell finds its place, "
          "then refuses the card's cell, no error", any(s.get("cell") is None for s in record.get("cards_seen", []))
          and reply.get("result") == "refused" and "a card is on C3" in reply.get("why", "") and r["cell"] == "C5"
          and r["collisions"] == 0, (record, reply, r, done.stderr[-400:]))

    # The second quarter turn of a face refused (the adults allow one command): the refusal carries no state, the
    # Pilot reads it again and says where the robot really faces.
    one = with_rules("one-command", limits={"commands": 1})
    sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
    script = ("import json, sys; sys.path.insert(0, 'stack'); import common, pilot; common.mission_begins(); "
              "pilot.state(); print(json.dumps(pilot.face_towards('south')))")
    done = subprocess.run([PYTHON, "-I", "-c", script], env=one, cwd=GUIDE, capture_output=True, text=True, timeout=60)
    reply = json.loads(done.stdout or "{}")
    r = sim.state()["robot"]
    check("face south with one command allowed: the second quarter turn is refused, the Pilot answers not done with "
          "the state read again, facing east as the robot does", reply.get("result") == "not done"
          and (reply.get("state") or {}).get("facing") == "east" and near(r["heading"] % 360, 90, 2),
          (reply, r, done.stderr[-400:]))

    # A mission file of the wrong shape, straight to strategist.py: the closed refusal, never a traceback.
    missions_dir = Path(env["ROBOT_STATE_DIR"]) / "missions"
    bad_file = Path(env["ROBOT_STATE_DIR"]) / "bad-mission.json"
    light = [{"do": "set_light", "color": "green"}]
    for bad, what in ((light, "a JSON array, not an object"),
                      ({"steps": [{"do": ["reach_card"], "card": "barbecue"}]}, "a step whose do is a list"),
                      ({"steps": [{"do": "reach_card", "card": ["barbecue"]}]}, "a step whose card is a list"),
                      ({"steps": light, "personality": True}, "a personality true"),
                      ({"steps": light, "personality": 5}, "a personality 5"),
                      ({"steps": light, "personality": [["short_legs"]]}, "a list inside the list of bricks"),
                      ({"steps": light, "personality": {"short_legs": 1}}, "a personality that is an object")):
        bad_file.write_text(json.dumps(bad), encoding="utf-8")
        before = set(missions_dir.iterdir()) if missions_dir.exists() else set()
        done = subprocess.run([PYTHON, "-I", str(GUIDE / "stack" / "strategist.py"), str(bad_file)], env=env,
                              stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=60)
        check(f"mission file refused, no traceback, nothing written: {what}",
              done.returncode != 0 and done.stderr.startswith("refused: ") and "Traceback" not in done.stderr
              and (set(missions_dir.iterdir()) if missions_dir.exists() else set()) == before,
              (done.stdout, done.stderr[-300:]))

    # Without a model, with no landmark and no compass: the cards of the report always have a cell of the mat.
    lost = with_rules("lost", start="unknown", mat_north_up=False)
    sim.act(do="load_world", mat=big, world={"cards": [{"cell": "C2", "label": "barbecue"}], "boxes": [], "balls": []})
    done = subprocess.run([PYTHON, "-I", str(GUIDE / "stack" / "mission.py"), "survey:barbecue", "--model", "none"],
                          env=lost, stdin=subprocess.DEVNULL, capture_output=True, text=True, timeout=300)
    report = json.loads(done.stdout or "{}")
    cells = [f"{c}{r}" for r in range(1, MAT["rows"] + 1) for c in "ABCDEFGHIJ"[:MAT["cols"]]]
    check("start unknown, no landmark, no compass, a survey without a model: no card without a cell of the mat in the "
          "report, and an impossible survey says why",
          all(isinstance(c.get("cell"), str) and c["cell"] in cells for c in report.get("cards", []))
          and (report.get("outcome") != "impossible" or report.get("why") in (
              "the robot does not know where it is", "not seen yet", "not on the mat", "no moves left", "no way there")),
          (report, done.stderr[-400:]))

    unknown = with_rules("unknown", start="unknown")
    sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
    first = drive(unknown, "state")
    sent = wire_count(sim, "MOVE")
    slide = drive(unknown, "move", "north", "2")
    face = drive(unknown, "face", "east")
    check("start unknown: no cell, and the Referee refuses what needs the mat (move, face), nothing sent",
          first.get("cell") is None and first.get("position") in ("unknown", "probable") and slide.get("result") == "refused"
          and "does not know yet" in slide.get("why", "") and face.get("result") == "refused" and wire_count(sim, "MOVE") == sent,
          (first, slide, face))
    check("start unknown: the compass (north at the top of the mat) and one reading leave the cells of row 5, facing "
          "north", sorted(first.get("could_be") or []) == [f"{c}5 facing north" for c in "ABCDE"], first)
    around = drive(unknown, "around")
    heard = [e["event"] for e in around.get("since_last_call", [])]
    check("start unknown, no landmark: after a look around, the compass and the border give C5 facing north, and the "
          "Cartographer says so", (around.get("state") or {}).get("cell") == "C5"
          and around["state"].get("facing") == "north" and any("compass" in e for e in heard), (around.get("state"), heard))
    no_compass = with_rules("no-compass", start="unknown", mat_north_up=False)
    sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
    around = drive(no_compass, "around")
    places = (around.get("state") or {}).get("could_be") or []
    check("start unknown, without the compass: the border alone leaves 4 places on a square mat that looks the same at "
          "each quarter turn, C5 facing north among them", len(places) == 4 and "C5 facing north" in places,
          around.get("state"))
    facing_up = with_rules("facing-up", start={"facing": "north"}, mat_north_up=False)
    sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
    around = drive(facing_up, "around")
    check("a start known only by its facing (the top of the mat), without the compass: the border settles the cell, "
          "C5 facing north", (around.get("state") or {}).get("cell") == "C5" and around["state"].get("facing") == "north",
          around.get("state"))
    sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
    drive(unknown, "state")
    step = drive(unknown, "forward")
    check("start unknown: one cell forwards, where the sensor sees the way free", step.get("result") == "done"
          and sim.state()["robot"]["cell"] == "C4", step)

    landmark = with_rules("landmark", start="unknown", landmarks={"water-point": "C2"})
    sim.act(do="load_world", mat=big, world={"cards": [{"cell": "C2", "label": "water-point"}], "boxes": [], "balls": []})
    time.sleep(0.6)
    drive(landmark, "state")
    around = drive(landmark, "around")
    heard = [e["event"] for e in around.get("since_last_call", [])]
    check("a landmark of the rules (the water point in C2): the Cartographer finds C5 facing north, and says so",
          around.get("state", {}).get("cell") == "C5" and around["state"].get("facing") == "north"
          and any("now knows where the robot is" in e for e in heard), (around.get("state"), heard))
    cards = drive(landmark, "seen").get("cards_seen", [])
    check("the cards seen before it knew are now on the mat (the water point in C2)",
          any(c["card"] == "water-point" and c["cell"] == "C2" for c in cards), cards)

    wrong = with_rules("wrong", start={"cell": "C4", "facing": "north"})
    sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
    state = drive(wrong, "state")
    heard = [e["event"] for e in state.get("since_last_call", [])]
    check("a start the rules give wrong (C4, the robot on C5): the first reading beyond the border, the Cartographer no "
          "longer believes it, and says so", state.get("cell") is None and any("no longer knows" in e for e in heard),
          (state, heard))

    turned = with_rules("turned", start={"cell": "C5", "facing": "east"})
    sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
    state = drive(turned, "state")
    heard = [e["event"] for e in state.get("since_last_call", [])]
    check("a start the rules give turned (east, the robot faces north): the compass contradicts it, the Cartographer no "
          "longer believes it, and says so", state.get("cell") is None
          and any("compass says it faces north" in e for e in heard), (state, heard))

    world = {"cards": [{"cell": "C2", "label": "water-point"}, {"cell": "E2", "label": "barbecue"}], "boxes": [], "balls": []}
    sim.act(do="load_world", mat=big, world=world)
    done = subprocess.run([PYTHON, "-I", str(GUIDE / "stack" / "mission.py"), "reach_card:barbecue", "--model", "none"],
                          env=landmark, capture_output=True, text=True, timeout=300)
    report = json.loads(done.stdout or "{}")
    r = sim.state()["robot"]
    check("start unknown, a mission without a model: the Pilot finds where it is first, then stops in front of the "
          "barbecue (E2)", report.get("outcome") == "done" and {"card": "barbecue", "cell": "E2"} in report.get("cards", [])
          and r["collisions"] == 0, (report, r, done.stderr[-400:]))

    journal = [json.loads(line) for line in
               (Path(env["ROBOT_STATE_DIR"]) / "referee.jsonl").read_text(encoding="utf-8").splitlines()]
    check("the Referee's journal: every command it sent or refused, with the rule",
          any(e["decision"] == "sent" for e in journal) and any(e["decision"] == "refused" and e.get("rule")
                                                                 for e in journal), journal[-3:])


async def interpreter_scenarios(sim, env):
    """The link from the Interpreter to the Strategist (stack/interpreter_link.py): MCP over HTTP with a token, as Open
    WebUI would use it. The Strategist with no model: no network, no key."""
    import secrets
    env = {k: v for k, v in env.items() if k != "ROBOT_RULES"}  # the rules of the guide
    from mcp.client.streamable_http import create_mcp_http_client, streamable_http_client

    token, port = secrets.token_hex(24), 18790
    big = dict(MAT, max_moves=40)
    sim.act(do="load_world", mat=big, world={"cards": [{"cell": "C2", "label": "barbecue"}], "boxes": [], "balls": []})
    link = subprocess.Popen([PYTHON, "-I", str(GUIDE / "stack" / "interpreter_link.py"), "--port", str(port),
                             "--allowed-host", f"127.0.0.1:{port}", "--model", "none"],
                            env=dict(env, MCP_TOKEN=token), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        ready = link.stdout.readline()
        check("interpreter link: it starts, and says what it serves", ready.startswith("READY robot-mission"), ready)
        import socket
        for _ in range(50):  # READY comes just before the server listens
            try:
                socket.create_connection(("127.0.0.1", port), timeout=1).close()
                break
            except OSError:
                time.sleep(0.2)
        url = f"http://127.0.0.1:{port}/mcp"
        for headers, what in (({}, "no token"), ({"Authorization": "Bearer " + "x" * 48}, "a wrong token")):
            request = urllib.request.Request(url, data=b"{}", headers=dict(headers, **{"Content-Type": "application/json"}))
            try:
                urllib.request.urlopen(request, timeout=5)
                status = 200
            except urllib.error.HTTPError as e:
                status = e.code
            check(f"interpreter link: {what}, 401", status == 401, status)
        bad_host = http.client.HTTPConnection("127.0.0.1", port, timeout=5)
        bad_host.request("POST", "/mcp", body=b"{}", headers={"Host": "evil.example:80", "Authorization": "Bearer " + token,
                                                              "Content-Type": "application/json",
                                                              "Accept": "application/json, text/event-stream"})
        status = bad_host.getresponse().status
        check("interpreter link: another Host header (DNS rebinding), refused", status in (400, 403, 421), status)

        client = create_mcp_http_client(headers={"Authorization": "Bearer " + token})
        async with streamable_http_client(url, http_client=client) as streams:
            read, write = streams[0], streams[1]
            async with ClientSession(read, write) as session:
                await session.initialize()

                async def call(name, **arguments):
                    result = await session.call_tool(name, arguments)
                    return json.loads(result.content[0].text)

                names = {t.name for t in (await session.list_tools()).tools}
                check("interpreter link: six tools, none of them drives the robot",
                      names == {"mission_options", "start_mission", "pause_mission", "resume_mission", "stop_mission",
                                "mission_status"}, names)
                options = await call("mission_options")
                check("interpreter link: the closed lists (steps, cards, colours, personalities)",
                      "barbecue" in options["cards"] and "reach_card" in options["steps"]
                      and {"cautious", "adventurous"} <= set(options["personalities"]), options)
                sent = wire_count(sim, "MOVE") + wire_count(sim, "TURN")
                reply = await call("start_mission", steps=[{"do": "reach_card", "card": "picnic"}])
                check("interpreter link: a card the camera does not know, refused, nothing sent",
                      reply.get("result") == "refused" and wire_count(sim, "MOVE") + wire_count(sim, "TURN") == sent, reply)
                reply = await call("start_mission", steps=[{"do": "reach_card", "card": "barbecue"}], personality="cautious")
                again = await call("start_mission", steps=[{"do": "set_light", "color": "red"}])
                check("interpreter link: a mission starts; a second one is refused while it runs",
                      reply.get("result") == "started" and again.get("result") == "refused", (reply, again))
                status = {}
                for _ in range(120):
                    status = await call("mission_status")
                    if status["state"] not in ("running", "paused"):
                        break
                    await asyncio.sleep(1)
                r = sim.state()["robot"]
                check("interpreter link: the mission ends with its report, the barbecue in C2, the robot in front of it",
                      status.get("state") == "ended" and status["report"]["outcome"] == "done"
                      and {"card": "barbecue", "cell": "C2"} in status["report"]["cards"] and r["cell"] == "C3", (status, r))

                sim.act(do="load_world", mat=big, world={"cards": [], "boxes": [], "balls": []})
                reply = await call("start_mission", steps=[{"do": "survey", "card": "dry-grass"}])
                await asyncio.sleep(2.0)
                paused = await call("pause_mission")
                # A move under way ends first (a look around finishes its four quarter turns): wait until the robot
                # has sent nothing for 3 s, then watch 5 s more.
                last, still = -1, 0
                for _ in range(30):
                    count = wire_count(sim, "MOVE") + wire_count(sim, "TURN")
                    still = still + 1 if count == last else 0
                    last = count
                    if still >= 3:
                        break
                    await asyncio.sleep(1.0)
                before = wire_count(sim, "MOVE") + wire_count(sim, "TURN")
                await asyncio.sleep(5.0)
                after = wire_count(sim, "MOVE") + wire_count(sim, "TURN")
                status = await call("mission_status")
                check("interpreter link: pausing, then paused once nothing moves, with the cell where the robot stands",
                      paused.get("result") in ("pausing", "paused") and status["state"] == "paused"
                      and (status.get("robot_stands_on") or {}).get("cell") and before == after,
                      (paused, status, before, after))
                await call("resume_mission")
                await asyncio.sleep(2.0)
                stopping = await call("stop_mission")
                for _ in range(60):
                    status = await call("mission_status")
                    if status["state"] not in ("running", "paused"):
                        break
                    await asyncio.sleep(1)
                check("interpreter link: stopped, the report says so", stopping.get("result") == "stopping"
                      and status["state"] == "stopped" and status["report"]["outcome"] == "stopped", status)
    finally:
        link.terminate()
        link.wait(timeout=10)


if __name__ == "__main__":
    asyncio.run(main())
