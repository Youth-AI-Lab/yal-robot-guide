"""What every tier of the stack shares: where the files are, the mat, the names of the cells, the log of what the
caller received. No decision here."""
import fcntl
import json
import os
from pathlib import Path

import clock

GUIDE = Path(__file__).resolve().parent.parent
MAT_PATH = Path(os.environ.get("ROBOT_MAT", GUIDE / "mat.json"))
STATE_DIR = Path(os.environ.get("ROBOT_STATE_DIR", GUIDE / ".robot-sim"))
RULES_PATH = Path(os.environ.get("ROBOT_RULES", GUIDE / "stack" / "rules.json"))
# Who sends the orders, written in the logs: the Strategist, an agent (MCP), a person in a terminal, or the Drive
# buttons of the simulator's page (which set ROBOT_CALLER=page).
CALLER = os.environ.get("ROBOT_CALLER", "agent")
FACING = ["north", "east", "south", "west"]
FACING_STEP = ((0, -1), (1, 0), (0, 1), (-1, 0))  # north, east, south, west
COLORS = ("off", "red", "orange", "green", "blue")


def load_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def cell_name(col, row):
    return chr(ord("A") + col) + str(row + 1)


def cell_xy(name):
    return ord(name[0].upper()) - ord("A"), int(name[1:]) - 1


def log(tool, arguments, result, caller=None):
    """Write what the caller receives, as it receives it (the simulator's page shows it): exactly what entered an
    agent's context."""
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    entry = {"at": clock.iso(), "caller": caller or CALLER, "tool": tool,
             "arguments": arguments, "result": result}
    with (STATE_DIR / "context.jsonl").open("a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def say_held(where):
    """The acknowledgement of a pause, up the chain: where the robot stands still, or None when it moves again
    (.robot-sim/held.json). Written by the Pilot (or the Strategist's loop between two tools) once nothing moves."""
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    path = STATE_DIR / "held.json"
    if where is None:
        path.unlink(missing_ok=True)
    else:
        path.write_text(json.dumps(where) + "\n", encoding="utf-8")


def held_where():
    try:
        return load_json(STATE_DIR / "held.json")
    except (OSError, ValueError):
        return None


def say_waiting(info):
    """The robot waits for an adult during a mission: what the Interpreter tells the group ({since, what_to_do}), or None
    once it moves again (.robot-sim/waiting.json). Written by the Strategist's loop, which waits without its model."""
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    path = STATE_DIR / "waiting.json"
    if info is None:
        path.unlink(missing_ok=True)
    else:
        path.write_text(json.dumps(info, ensure_ascii=False) + "\n", encoding="utf-8")


def waiting_info():
    try:
        return load_json(STATE_DIR / "waiting.json")
    except (OSError, ValueError):
        return None


def say_progress(progress):
    """Where the mission stands, one line per step, for the Interpreter (.robot-sim/progress.json). Written by the
    Strategist's loop, which keeps it; None when no mission has begun."""
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    path = STATE_DIR / "progress.json"
    if progress is None:
        path.unlink(missing_ok=True)
    else:
        path.write_text(json.dumps(progress, ensure_ascii=False) + "\n", encoding="utf-8")


def progress_now():
    try:
        return load_json(STATE_DIR / "progress.json")
    except (OSError, ValueError):
        return None


def add_steps(steps):
    """Steps the group adds to the paused mission (the Interpreter's add_to_mission), for the Strategist's loop to take
    (.robot-sim/additions.json)."""
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    path = STATE_DIR / "additions.json"
    try:
        waiting = load_json(path)
    except (OSError, ValueError):
        waiting = []
    path.write_text(json.dumps(waiting + list(steps)) + "\n", encoding="utf-8")


def take_additions():
    """The steps added since the last look, which are then gone from the file."""
    path = STATE_DIR / "additions.json"
    try:
        steps = load_json(path)
    except (OSError, ValueError):
        return []
    path.unlink(missing_ok=True)
    return steps if isinstance(steps, list) else []


def mission_control():
    """What the Interpreter asks of the mission running: "go", "pause" or "stop" (.robot-sim/control.json)."""
    try:
        return load_json(STATE_DIR / "control.json").get("do", "go")
    except (OSError, ValueError, AttributeError):
        return "go"


# The counters of the session, kept by the tier that does the thing (.robot-sim/counters.json, deleted with the
# session): missions the Strategist was given, its calls to the model and their cost (as the provider says it),
# operations the Pilot was asked (the tools), commands the Referee sent to the robot (MOVE, TURN, HOLD, STOP, LED: what
# acts, not STATE, EVENTS or PING), cells the robot really went through (EVT CELL, or MOVE F done), and the moves of
# the Referee's rule (a step, a slide or a quarter turn; look_around counts one). The same counts since the mission
# began, while one runs or after it ended, until the next one.
COUNTS = ("missions", "model_calls", "model_usd", "operations", "commands", "cells", "moves")


def _counters(change=None):
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    with (STATE_DIR / "counters.lock").open("w") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)  # several processes count: the Strategist, an agent's MCP door, a terminal
        try:
            data = load_json(STATE_DIR / "counters.json")
        except (OSError, ValueError):
            data = {}
        data = {"session": dict({k: 0 for k in COUNTS}, **data.get("session", {})), "mission": data.get("mission")}
        if change:
            change(data)
            (STATE_DIR / "counters.json").write_text(json.dumps(data) + "\n", encoding="utf-8")
        return data


def count(name, n=1):
    """Add n to a counter (a cost in dollars for model_usd), for the session and the mission."""
    def add(data):
        for part in (data["session"], data["mission"]):
            if part is not None:
                part[name] = round(part.get(name, 0) + n, 6)
    _counters(add)


def mission_begins():
    """A new mission: its own counts start from zero (the session's go on)."""
    def begin(data):
        data["mission"] = {k: 0 for k in COUNTS}
    _counters(begin)
    count("missions")


def counts():
    """{"session": {...}, "this_mission": {...} or None}."""
    data = _counters()
    return {"session": data["session"], "this_mission": data["mission"]}


def used():
    """What the adults' limits count: since the mission began, or since the session began when there was none."""
    data = _counters()
    return data["mission"] if data["mission"] is not None else data["session"]
