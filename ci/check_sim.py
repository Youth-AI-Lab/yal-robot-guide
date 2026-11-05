"""Checks the robot's program, the simulator and the gateway together, as an agent would use them.

No model and no key: this script calls the tools of the gateway itself, through MCP, in the order a
model could, and reads in the simulator what really happened (where the robot is, whether it hit
something, when it stopped). Each scenario starts again from a fresh mat.

Usage, in the Codespace of this guide (the folder of the guide is the current folder):
    /opt/workshop/venv/bin/python -I ci/check_sim.py
It takes about two minutes. It needs the `mcp` package, which is in /opt/workshop/venv.
"""
import ast
import asyncio
import contextlib
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
TOOLS = {"robot_state", "step_forward", "turn", "set_light", "wait_for_event", "seen_cards"}
# A small mat for the checks: 5 x 5 cells of 30 cm, start on C5 facing north, 6 moves.
MAT = {"cols": 5, "rows": 5, "cell_cm": 30, "start": {"cell": "C5", "facing": "north"},
       "max_moves": 6, "move_timeout_s": 6,
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
    """A pretend agent: it calls the tools of the gateway through MCP, as a model would."""

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
    params = StdioServerParameters(command=PYTHON, args=["-I", str(GUIDE / "gateway" / "robot_gateway.py")], env=env)
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
    """Speak raw JSON-RPC to the gateway, as Claude Code does with MCP_PROTOCOL_NEGOTIATION=legacy:
    initialize, initialized, then listen. Returns (the initialize answer, the channel notifications)."""
    process = await asyncio.create_subprocess_exec(
        PYTHON, "-I", str(GUIDE / "gateway" / "robot_gateway.py"), env=dict(env, ROBOT_CHANNEL="1"),
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


async def main():
    micropython_subset()
    with tempfile.TemporaryDirectory() as tmp:
        state_dir = Path(tmp)
        mat_path = state_dir / "mat.json"
        mat_path.write_text(json.dumps(MAT), encoding="utf-8")
        env = dict(os.environ, ROBOT_STATE_DIR=str(state_dir), ROBOT_MAT=str(mat_path))

        # No robot yet: a clear answer, no crash.
        async with agent(dict(env, ROBOT_SERIAL=str(state_dir / "no-such-port"))) as a:
            reply = await a.call("robot_state")
            check("no robot: the gateway says so", reply.get("result") == "error" and "not connected" in reply.get("why", ""),
                  reply)

        sim = Sim(state_dir, mat_path)
        try:
            await scenarios(sim, env)
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
        check("the gateway offers exactly six tools", names == TOOLS, names)
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

    # A wrong argument never reaches the robot.
    sim.fresh()
    async with agent(env) as a:
        turns = wire_count(sim, "TURN")
        reply = await a.call("turn", direction="up")
        check("turn('up'): an error, nothing sent", reply.get("is_error") is True and wire_count(sim, "TURN") == turns,
              reply)
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
        check("wait_for_event: an event from before the wait is marked so, with the real wait",
              before and before[0].get("before_you_started_waiting") is True and reply.get("waited_seconds", 9) < 1,
              reply)
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
        await a.call("step_forward")
        await asyncio.sleep(0.6)
        reply = await a.call("robot_state")
        heard = [e["event"] for e in reply.get("since_last_call", [])]
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


if __name__ == "__main__":
    asyncio.run(main())
