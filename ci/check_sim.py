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
import json
import os
import subprocess
import sys
import tempfile
import time
import urllib.request
from pathlib import Path

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

GUIDE = Path(__file__).resolve().parent.parent
PYTHON = sys.executable
TOOLS = {"robot_state", "step_forward", "turn", "set_light"}
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


def wire_count(sim, prefix):
    return sum(1 for w in sim.state()["wire"] if w["dir"] == "in" and w["line"].startswith(prefix))


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
        check("the gateway offers exactly four tools", names == TOOLS, names)
        read_only = {t.name for t in tools if t.annotations and t.annotations.read_only_hint}
        check("only robot_state is marked read only", read_only == {"robot_state"}, read_only)

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
