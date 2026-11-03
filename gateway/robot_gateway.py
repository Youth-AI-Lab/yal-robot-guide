"""The gateway between an agent and the robot: the only door.

It is an MCP server with four tools, and nothing else reaches the robot:
  - robot_state()        what the robot measures now, and where the gateway thinks it is (read only)
  - step_forward()       one cell forwards
  - turn(direction)      a quarter turn, "left" or "right"
  - set_light(color)     the light of the robot, from a short list

The gateway keeps its own limits, whatever the agent asks: one cell per call, never off the mat,
a number of moves per session, a time limit on each move. It never sends an image: the agent gets
short texts, and each one is written to .robot-sim/context.jsonl, so that people can read exactly
what entered the agent's context (the simulator shows it).

It talks to the robot through a serial port: the USB cable of the robot, or the pseudo-terminal of
the simulator (its path is in .robot-sim/serial). Set ROBOT_SERIAL to choose another port.

Usage:
    as an MCP server (stdio), for an agent:  /opt/workshop/venv/bin/python -I gateway/robot_gateway.py
    by hand, without an agent:               python3 -I gateway/robot_gateway.py state|forward|left|right|light <color>
"""
import datetime
import fcntl
import json
import os
import select
import sys
import termios
import threading
import time
import tty
from pathlib import Path
from typing import Literal

GUIDE = Path(__file__).resolve().parent.parent
MAT = Path(os.environ.get("ROBOT_MAT", GUIDE / "mat.json"))
STATE_DIR = Path(os.environ.get("ROBOT_STATE_DIR", GUIDE / ".robot-sim"))
HEARTBEAT_S = 0.2
# Who sends the orders, written in the log: an agent (MCP), a person in a terminal, or the Drive
# buttons of the simulator's page (which set ROBOT_CALLER=page).
CALLER = os.environ.get("ROBOT_CALLER", "agent")
FACING = ["north", "east", "south", "west"]
COLORS = ("off", "red", "orange", "green", "blue")
REASONS = {
    "obstacle": "something is in front of the robot",
    "battery": "the battery is too low",
    "estop": "someone pressed the emergency stop button on the robot",
    "wait_adult": "the robot waits for an adult (a move was interrupted)",
    "busy": "the robot is still moving",
    "watchdog": "the robot stopped by itself: it heard nothing from the computer",
    "bump": "something touched the robot's bumper",
    "command": "the gateway stopped the robot",
    "timeout": "the move took too long: the gateway stopped the robot",
    "unknown": "the robot did not understand the command",
}


def log(tool, arguments, result):
    """Write what the agent receives, as it receives it."""
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    entry = {"at": datetime.datetime.now().isoformat(timespec="seconds"), "caller": CALLER, "tool": tool,
             "arguments": arguments, "result": result}
    with (STATE_DIR / "context.jsonl").open("a", encoding="utf-8") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def cell_name(col, row):
    return chr(ord("A") + col) + str(row + 1)


class RobotNotConnected(Exception):
    pass


class Gateway:
    def __init__(self, mat):
        self.mat = mat
        self.cards = mat["cards"]
        self.fd = None
        self.buffer = b""
        self.lock = threading.Lock()
        self.load_state()

    # Where the gateway thinks the robot is: kept in a file, so that it survives a new agent session
    # or a command typed by hand. No file: the robot is on its start cell (the simulator removes
    # the file when it puts the robot back there; on the real robot, `robot_gateway.py reset`).
    def load_state(self):
        try:
            saved = json.loads((STATE_DIR / "gateway.json").read_text(encoding="utf-8"))
            self.col, self.row, self.facing = saved["col"], saved["row"], saved["facing"]
            self.sure, self.back, self.moves = saved["sure"], saved["back"], saved["moves"]
            self.resumes_at_stop = saved.get("resumes_at_stop")
        except (OSError, ValueError, KeyError):
            start = self.mat["start"]
            self.col, self.row = ord(start["cell"][0].upper()) - ord("A"), int(start["cell"][1:]) - 1
            self.facing = FACING.index(start["facing"])
            self.sure = True  # False after an interrupted move, until an adult puts the robot back
            self.back = None
            self.moves = 0
            self.resumes_at_stop = None  # how many times Resume had been pressed when a move stopped

    def save_state(self):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        saved = {"col": self.col, "row": self.row, "facing": self.facing, "sure": self.sure,
                 "back": self.back, "moves": self.moves, "resumes_at_stop": self.resumes_at_stop}
        (STATE_DIR / "gateway.json").write_text(json.dumps(saved) + "\n", encoding="utf-8")

    # The serial link -------------------------------------------------------------------------
    def port(self):
        path = os.environ.get("ROBOT_SERIAL")
        if not path:
            try:
                path = (STATE_DIR / "serial").read_text(encoding="utf-8").strip()
            except OSError:
                raise RobotNotConnected("no robot: start the simulator, or set ROBOT_SERIAL") from None
        return path

    def connect(self):
        if self.fd is not None:
            return
        path = self.port()
        try:
            fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        except OSError as e:
            raise RobotNotConnected(f"the robot is not connected ({path}: {e.strerror})") from None
        # TCSANOW: keep what the robot already said (an old answer is dropped by drain(), an event
        # is read). Nothing that matters depends on catching an event, though: see check_resumed().
        tty.setraw(fd, termios.TCSANOW)
        attrs = termios.tcgetattr(fd)
        attrs[4] = attrs[5] = termios.B115200  # the real robot; no effect on the simulator
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        self.fd, self.buffer = fd, b""
        self.drain()
        reply = self.request("HELLO")
        if not reply.startswith("OK HELLO robot"):
            self.close()
            raise RobotNotConnected(f"this is not the robot (it answered {reply!r})")

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
        self.fd = None

    def send(self, line):
        try:
            os.write(self.fd, (line + "\n").encode("ascii"))
        except OSError as e:
            self.close()
            raise RobotNotConnected(f"the link to the robot is broken ({e.strerror})") from None

    def read_line(self, timeout):
        end = time.monotonic() + timeout
        while b"\n" not in self.buffer:
            left = max(0.0, end - time.monotonic())
            ready, _, _ = select.select([self.fd], [], [], left)  # with 0: look once, do not wait
            if not ready and left == 0:
                return None
            if ready:
                try:
                    self.buffer += os.read(self.fd, 4096)
                except BlockingIOError:
                    pass
                except OSError as e:
                    self.close()
                    raise RobotNotConnected(f"the link to the robot is broken ({e.strerror})") from None
        line, self.buffer = self.buffer.split(b"\n", 1)
        return line.decode("ascii", "replace").strip()

    def event(self, line):
        """An event from the robot that is not the answer to a question (EVT BUMP, EVT RESUMED).
        Nothing depends on it: a new gateway may start after the event, so the state of the robot
        is always read again with STATE."""

    def fields(self):
        reply = self.request("STATE")
        return dict(part.split("=", 1) for part in reply.split()[2:])

    def check_resumed(self):
        """After an interrupted move: has an adult pressed Resume since? The robot counts the presses;
        the gateway compares with the count it saw when the move stopped."""
        fields = self.fields()
        presses = int(fields.get("resumes", 0))
        released = fields.get("wait_adult") == "0" and fields.get("estop") == "0"
        if self.resumes_at_stop is None:
            # The count was not known when the move stopped (the cable was loose). A robot that no
            # longer waits, remembers its last stop and has seen Resume was released by an adult;
            # a robot that restarted remembers neither.
            if released and presses > 0 and fields.get("last_stop", "none") != "none":
                resumed = True
            else:
                self.resumes_at_stop, resumed = presses, False
        elif presses < self.resumes_at_stop:
            self.resumes_at_stop, resumed = presses, False  # the robot restarted: count from now
        else:
            resumed = presses > self.resumes_at_stop and released
        if resumed:
            self.col, self.row, self.facing = self.back
            self.sure, self.back, self.resumes_at_stop = True, None, None

    def drain(self):
        """Read what the robot said while nobody asked (Resume pressed, old answers)."""
        while True:
            line = self.read_line(0)
            if line is None:
                return
            if line.startswith("EVT "):
                self.event(line)

    def request(self, line, timeout=1.0):
        self.send(line)
        end = time.monotonic() + timeout
        while True:
            reply = self.read_line(max(0.0, end - time.monotonic()))
            if reply is None:
                raise RobotNotConnected("the robot does not answer")
            if reply.startswith("EVT "):
                self.event(reply)
                continue
            if reply == "OK PONG" and line != "PING":
                continue
            return reply

    # What the agent can ask ------------------------------------------------------------------
    def robot_state(self):
        fields = self.fields()
        card = fields.get("card", "none")
        return {
            "cell": cell_name(self.col, self.row) if self.sure else None,
            "facing": FACING[self.facing] if self.sure else None,
            "position_known": self.sure,
            "distance_ahead_cm": None if fields.get("dist") == "none" else int(fields["dist"]),
            "card_ahead": None if card == "none" else {"card": card, "means": self.cards.get(card, "unknown card")},
            "battery_percent": int(fields.get("battery", 0)),
            "emergency_stop": fields.get("estop") == "1",
            "waiting_for_adult": fields.get("wait_adult") == "1" or not self.sure,
            "bumper_touched": fields.get("bumper") == "1",
            "moves_left": self.mat["max_moves"] - self.moves,
            "mat": f"{self.mat['cols']} columns (A to {cell_name(self.mat['cols'] - 1, 0)[0]}), "
                   f"{self.mat['rows']} rows (1 at the top to {self.mat['rows']})",
        }

    def refused(self, why, **extra):
        return dict({"result": "refused", "why": REASONS.get(why, why)}, **extra)

    def move(self, command, kind):
        """MOVE F, TURN L or TURN R: one move, watched until it ends."""
        if not self.sure:
            return self.refused("wait_adult", what_to_do=self.adult_instruction())
        if self.moves >= self.mat["max_moves"]:
            return self.refused(f"no moves left in this session ({self.mat['max_moves']})")
        if kind == "F":
            dc, dr = ((0, -1), (1, 0), (0, 1), (-1, 0))[self.facing]
            target = (self.col + dc, self.row + dr)
            if not (0 <= target[0] < self.mat["cols"] and 0 <= target[1] < self.mat["rows"]):
                return self.refused("the next cell is outside the mat")
        reply = self.request(command)
        if reply.startswith("ERR "):
            return self.refused(reply[4:])
        self.moves += 1
        start = (self.col, self.row, self.facing)
        end = time.monotonic() + self.mat["move_timeout_s"]
        next_ping = time.monotonic() + HEARTBEAT_S
        while time.monotonic() < end:
            line = self.read_line(max(0.0, min(end, next_ping) - time.monotonic()))
            if time.monotonic() >= next_ping:
                self.send("PING")  # "I am still here": without it, the robot stops (its watchdog)
                next_ping = time.monotonic() + HEARTBEAT_S
            if line is None or line == "OK PONG":
                continue
            if line == "EVT DONE " + kind:
                if kind == "F":
                    self.col, self.row = target
                else:
                    self.facing = (self.facing + (1 if kind == "R" else -1)) % 4
                return {"result": "done", "state": self.robot_state()}
            if line.startswith("EVT STOP "):
                return self.interrupted(line[9:], start)
            if line.startswith("EVT "):
                self.event(line)
        try:
            self.send("STOP")
        except RobotNotConnected:
            pass
        return self.interrupted("timeout", start)

    def interrupted(self, why, start):
        self.sure, self.back = False, start
        try:
            self.resumes_at_stop = int(self.fields().get("resumes", 0))
        except (RobotNotConnected, ValueError):
            self.resumes_at_stop = None  # the cable is loose: counted when the robot answers again
        return {"result": "stopped", "why": REASONS.get(why, why), "position_known": False,
                "what_to_do": self.adult_instruction()}

    def adult_instruction(self):
        col, row, facing = self.back if self.back else (self.col, self.row, self.facing)
        return (f"Stop and ask an adult to put the robot back on cell {cell_name(col, row)}, facing "
                f"{FACING[facing]}, then to press Resume on the robot. Do not try to move it before.")

    def set_light(self, color):
        reply = self.request("LED " + color)
        return {"result": "done"} if reply.startswith("OK") else self.refused(reply[4:])

    def run(self, tool, arguments, action):
        """Run one tool call for the agent: connect if needed, never raise, log what it gets."""
        with self.lock:
            # One command at a time on the cable, whoever sends it: an agent, a terminal, the page.
            # Two programs reading the same serial port would take each other's answers.
            STATE_DIR.mkdir(parents=True, exist_ok=True)
            with (STATE_DIR / "gateway.lock").open("w") as lock:
                try:
                    fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                except BlockingIOError:
                    result = self.refused("another command is running (an agent, a terminal or the page): "
                                          "wait for it to end")
                else:
                    self.load_state()
                    try:
                        self.connect()
                        self.drain()
                        if not self.sure:
                            self.check_resumed()
                        result = action()
                    except RobotNotConnected as e:
                        self.close()
                        result = {"result": "error", "why": str(e)}
                    self.save_state()
            text = json.dumps(result, ensure_ascii=False)
            log(tool, arguments, text)
            return text


GATEWAY = Gateway(json.loads(MAT.read_text(encoding="utf-8")))
MAT_START = f"{GATEWAY.mat['start']['cell']}, facing {GATEWAY.mat['start']['facing']}"


def robot_state() -> str:
    """What the robot measures now (distance to what is in front, the card in front, battery,
    emergency stop) and the cell where the gateway thinks it is. Moves nothing. The cell is a
    count of the moves done, not a measure: after an interrupted move it is unknown."""
    return GATEWAY.run("robot_state", {}, GATEWAY.robot_state)


def step_forward() -> str:
    """Move the robot one cell forwards, then stop. The robot refuses when something is in front,
    and stops by itself if something appears. Never off the mat. If the result is "stopped",
    do what "what_to_do" says before anything else."""
    return GATEWAY.run("step_forward", {}, lambda: GATEWAY.move("MOVE F", "F"))


def turn(direction: Literal["left", "right"]) -> str:
    """Turn the robot a quarter turn on the spot, to its left or to its right, then stop."""
    kind = {"left": "L", "right": "R"}[direction]
    return GATEWAY.run("turn", {"direction": direction}, lambda: GATEWAY.move("TURN " + kind, kind))


def set_light(color: Literal["off", "red", "orange", "green", "blue"]) -> str:
    """Change the colour of the robot's light."""
    return GATEWAY.run("set_light", {"color": color}, lambda: GATEWAY.set_light(color))


def serve():
    from mcp.server.mcpserver import MCPServer
    from mcp.types import ToolAnnotations

    server = MCPServer("robot")
    server.tool(annotations=ToolAnnotations(read_only_hint=True))(robot_state)
    for tool in (step_forward, turn, set_light):
        server.tool(annotations=ToolAnnotations(read_only_hint=False))(tool)
    server.run()


def by_hand(args):
    """Drive the robot without an agent: the same gateway, the same limits."""
    commands = {"state": robot_state, "forward": step_forward,
                "left": lambda: turn("left"), "right": lambda: turn("right")}
    if args[0] in commands and len(args) == 1:
        print(commands[args[0]]())
    elif args == ["reset"]:
        # Only for a person who has just put the robot on its start cell, facing the right way.
        (STATE_DIR / "gateway.json").unlink(missing_ok=True)
        print("The gateway now counts from the start cell: " + MAT_START)
    elif args[0] == "light" and len(args) == 2 and args[1] in COLORS:
        print(set_light(args[1]))
    else:
        sys.exit("Usage: robot_gateway.py state | forward | left | right | light " + "|".join(COLORS)
                 + " | reset")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        CALLER = os.environ.get("ROBOT_CALLER", "terminal")
        by_hand(sys.argv[1:])
    else:
        serve()
