"""What every tier of the stack shares: where the files are, the mat, the names of the cells, the log of what the
caller received. No decision here."""
import datetime
import json
import os
from pathlib import Path

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
    entry = {"at": datetime.datetime.now().isoformat(timespec="seconds"), "caller": caller or CALLER, "tool": tool,
             "arguments": arguments, "result": result}
    with (STATE_DIR / "context.jsonl").open("a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")
