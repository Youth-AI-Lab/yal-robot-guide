"""The gateway between an agent and the robot: the only door.

It is an MCP server with four tools, and nothing else reaches the robot:
  - robot_state()        what the robot measures now, and where the gateway thinks it is (read only)
  - step_forward()       one cell forwards
  - turn(direction)      a quarter turn, "left" or "right"
  - set_light(color)     the light of the robot, from a short list
  - wait_for_event(s)    watch the robot for up to s seconds, until something happens (read only)
  - seen_cards()         the cards the camera recognised, with the cell where each one is (read only)
Every answer also carries "since_last_call": what happened to the robot since this caller's last
call (a bump, a warning, a stop, Resume), from the robot's own memory of its last events.

With ROBOT_CHANNEL=1, the gateway also pushes each event to the agent as it happens, as a Claude
Code "channel" (research preview of Claude Code: interactive sessions only, started with
--dangerously-load-development-channels server:robot). No other agent reads these notifications.

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
import math
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
# Options being measured (facilitator/workshops/measures/2026-11-05-robot-variantes/), off by default.
TOOLS_PLUS = os.environ.get("ROBOT_TOOLS") == "plus"  # two more tools: look_around and face
BALL_GUARD = os.environ.get("ROBOT_BALL_GUARD") == "1"  # no step into a cell the ball may roll through
BALL_GUARD_S = 10  # a sighting older than this says nothing about where the ball is now
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


EVENTS = {
    "BUMP": "something touched the robot's bumper",
    "NEAR": "something came close in front of the robot standing still: its warning light turned on",
    "RESUMED": "an adult pressed Resume on the robot",
    "ESTOP": "someone pressed the emergency stop button",
    "STOP": "a move stopped: ",
    "SEEN": "the camera recognised a card in front: ",
    "PAUSE": "something was in the robot's way: it stopped its motors and waited",
    "GO_ON": "the way was clear again: the robot went on with its move",
}
FACING_STEP = ((0, -1), (1, 0), (0, 1), (-1, 0))  # north, east, south, west
# How sure the gateway is of a distance given by the camera: this share of the distance, plus a few
# centimetres. The simulator's camera errs by 10 % by default; the OpenMV N6's is TO BE COMPLETED.
CAMERA_ERROR = 0.15
CAMERA_MIN_ERROR_CM = 5


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
        self.path = None
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
            self.cursors = saved.get("cursors", {})
            self.poses = saved.get("poses", {})
            self.ball = saved.get("ball")
            self.seen = saved.get("seen", [])
            if not isinstance(self.seen, list):
                self.seen = []  # a record of an earlier version of the gateway
        except (OSError, ValueError, KeyError):
            start = self.mat["start"]
            self.col, self.row = ord(start["cell"][0].upper()) - ord("A"), int(start["cell"][1:]) - 1
            self.facing = FACING.index(start["facing"])
            self.sure = True  # False after an interrupted move, until an adult puts the robot back
            self.back = None
            self.moves = 0
            self.resumes_at_stop = None  # how many times Resume had been pressed when a move stopped
            self.cursors = {}  # the last robot event each caller (agent, terminal, page) was given
            self.poses = {}  # the robot's pose number -> [col, row, facing], as counted by the gateway
            self.seen = []  # the cards the camera recognised: estimates, the most precise kept
            self.ball = None  # the last sighting of a moving ball

    def save_state(self):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        saved = {"col": self.col, "row": self.row, "facing": self.facing, "sure": self.sure,
                 "back": self.back, "moves": self.moves, "resumes_at_stop": self.resumes_at_stop,
                 "cursors": self.cursors, "poses": self.poses, "seen": self.seen,
                 "ball": self.ball}
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
        path = self.port()
        # An agent may keep this gateway running for hours (Hermes does, from one command to the next):
        # when the robot's port changes (the simulator restarted), connect to the new one.
        if self.fd is not None and path == self.path:
            return
        self.close()
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
        self.fd, self.buffer, self.path = fd, b"", path
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

    def new_events(self):
        """What happened to the robot since this caller last heard, from the robot's own memory:
        nothing is lost when nobody was listening (up to the 16 events the robot keeps)."""
        seen = self.cursors.get(CALLER, 0)
        reply = self.request(f"EVENTS {seen}")
        parts = reply.split()
        head = dict(p.split("=", 1) for p in parts[2:4])
        now, following = int(head["now"]), int(head["next"])
        if following - 1 < seen:  # the robot restarted: its numbers start again
            seen = 0
            self.poses = {}
            parts = self.request("EVENTS 0").split()
        events = []
        entries = [p.split(":", 2) for p in parts[4:]]
        if entries and int(entries[0][0]) > seen + 1 and seen > 0:
            events.append({"event": "some older events were lost (the robot keeps only the last ones)"})
        for number, ms, name in entries:
            kind, _, reason = name.partition(":")
            if kind == "BALL":
                text = self.saw_ball(reason, now - int(ms))
                events.append({"seconds_before_this_answer": round((now - int(ms)) / 1000, 1), "event": text})
                continue
            if kind in ("SEEN", "GONE"):
                text = self.saw(kind, reason, now - int(ms))
                if text is None:
                    continue
                events.append({"seconds_before_this_answer": round((now - int(ms)) / 1000, 1), "event": text})
                continue
            text = EVENTS.get(kind, name)
            if kind == "STOP":
                text += REASONS.get(reason, reason)
            # "seconds_ago" alone was read by a real model (DeepSeek v4 Flash behind Hermes, 2026-11-03) as
            # a time since the start of a wait: the name says what it counts from.
            events.append({"seconds_before_this_answer": round((now - int(ms)) / 1000, 1), "event": text})
        self.cursors[CALLER] = following - 1
        return events

    def saw(self, kind, reason, age_ms):
        """A camera event (SEEN or GONE): where the card is, from the pose the robot was at, the angle
        and the distance the camera gave. Returns the text for the agent, or None (GONE only updates the
        record)."""
        label, angle, cm, pose = reason.rsplit(":", 3)
        angle, cm = int(angle), int(cm)
        where = self.poses.get(pose)
        means = self.cards.get(label, "unknown card")
        side = "straight ahead" if abs(angle) < 5 else f"{abs(angle)} degrees to the {'right' if angle > 0 else 'left'}"
        if where is None:
            if kind == "GONE":
                return None
            return EVENTS["SEEN"] + f"{label} ({means}), about {cm} cm away, {side}; where: not known (the robot's position was not known)"
        cell_cm = self.mat["cell_cm"]
        heading = math.radians(where[2] * 90 + angle)
        x = (where[0] + 0.5) * cell_cm + math.sin(heading) * cm
        y = (where[1] + 0.5) * cell_cm - math.cos(heading) * cm
        u = max(CAMERA_MIN_ERROR_CM, CAMERA_ERROR * cm + CAMERA_MIN_ERROR_CM)  # grows with the distance
        if kind == "GONE":
            self.seen = [s for s in self.seen if not (s["card"] == label and math.hypot(s["x"] - x, s["y"] - y) < max(s["u"], u))]
            return None
        cell, others = self.cells_near(x, y, u)
        at = (datetime.datetime.now() - datetime.timedelta(milliseconds=age_ms)).strftime("%H:%M:%S")
        entry = {"card": label, "means": means, "cell": cell, "could_also_be": others, "x": round(x, 1),
                 "y": round(y, 1), "u": round(u, 1), "seen_from": f"{cell_name(where[0], where[1])} facing "
                 f"{FACING[where[2]]}", "distance_cm": cm, "at": at}
        # The same card seen again: keep the most precise estimate (seen from closer).
        # The larger uncertainty, not their sum: two like cards side by side (dry grass in C2 and D2) stay two.
        same = [s for s in self.seen if s["card"] == label and math.hypot(s["x"] - x, s["y"] - y) < max(s["u"], u)]
        if not same:
            self.seen.append(entry)
        elif u <= min(s["u"] for s in same):
            self.seen = [s for s in self.seen if s not in same] + [entry]
        else:
            entry = min(same, key=lambda s: s["u"])
        where_text = entry["cell"] if not entry["could_also_be"] else \
            f"probably {entry['cell']} (could also be {', '.join(entry['could_also_be'])})"
        # Where it was seen from: the event can reach the agent after the next move (a model took a card
        # seen from C2 for one seen from D2, 2026-11-05).
        seen_from = f"{cell_name(where[0], where[1])} facing {FACING[where[2]]}"
        return (EVENTS["SEEN"] + f"{label} ({means}), seen from {seen_from}, about {cm} cm away, {side}: "
                f"{where_text}, as the gateway counts the moves")

    def place(self, where, angle, cm):
        """A sighting (angle, distance) from a pose, as a point of the mat (centimetres)."""
        heading = math.radians(where[2] * 90 + angle)
        cell_cm = self.mat["cell_cm"]
        return ((where[0] + 0.5) * cell_cm + math.sin(heading) * cm,
                (where[1] + 0.5) * cell_cm - math.cos(heading) * cm)

    def saw_ball(self, reason, age_ms):
        """A moving ball: where it is, which way it goes, and whether it comes closer to the robot."""
        trend, a1, c1, a2, c2, pose = reason.split(":")
        a1, c1, a2, c2 = int(a1), int(c1), int(a2), int(c2)
        how = {"closer": "getting closer to the robot", "away": "moving away from the robot",
               # "still": its distance does not change; it may still be moving, across (seen 2026-11-05).
               "still": "at the same distance from the robot", "gone": "out of the camera's sight now"}[trend]
        where = self.poses.get(pose)
        at = (datetime.datetime.now() - datetime.timedelta(milliseconds=age_ms)).strftime("%H:%M:%S")
        if where is None:
            self.ball = {"how": how, "at": at}
            return f"the camera sees a ball about {c2} cm away, {how}; where: not known (the robot's position was not known)"
        x1, y1 = self.place(where, a1, c1)
        x2, y2 = self.place(where, a2, c2)
        cell, others = self.cells_near(x2, y2, max(CAMERA_MIN_ERROR_CM, CAMERA_ERROR * c2 + CAMERA_MIN_ERROR_CM))
        dx, dy = x2 - x1, y2 - y1
        going = "" if math.hypot(dx, dy) < 2 or trend == "gone" else \
            (", moving " + (("east" if dx > 0 else "west") if abs(dx) > abs(dy) else ("south" if dy > 0 else "north")))
        in_cell = cell if not others else f"probably {cell} (could also be {', '.join(others)})"
        seen_from = f"{cell_name(where[0], where[1])} facing {FACING[where[2]]}"
        self.ball = {"cell": cell, "could_also_be": others, "moving": going[9:] or None, "how": how,
                     "seen_from": seen_from, "at": at}
        if trend == "gone":
            return f"the ball the camera was watching left its sight, last {in_cell}, seen from {seen_from}"
        return f"the camera sees a ball, seen from {seen_from}, about {c2} cm away: {in_cell}{going}, {how}"

    def cells_near(self, x, y, u):
        """The cell of the estimate, and the other cells of the mat within its uncertainty."""
        cell_cm, cols, rows = self.mat["cell_cm"], self.mat["cols"], self.mat["rows"]
        col = min(max(int(x // cell_cm), 0), cols - 1)
        row = min(max(int(y // cell_cm), 0), rows - 1)
        others = []
        for c in range(cols):
            for r in range(rows):
                nx = min(max(x, c * cell_cm), (c + 1) * cell_cm)
                ny = min(max(y, r * cell_cm), (r + 1) * cell_cm)
                if (c, r) != (col, row) and math.hypot(nx - x, ny - y) < u:
                    others.append(cell_name(c, r))
        return cell_name(col, row), others

    def seen_cards(self):
        keys = ("card", "means", "cell", "could_also_be", "distance_cm", "seen_from", "at")
        cards = [{k: s[k] for k in keys} for s in sorted(self.seen, key=lambda s: s["at"])]
        return {"cards_seen": cards,
                "note": "The camera sees up to 120 cm ahead, wider when farther, and guesses the distance: the "
                        "farther the card, the less sure its cell (could_also_be). Seen again from closer, the "
                        "estimate gets better. The robot's own cell comes from counting its moves, not from a measure."}

    def fields(self):
        reply = self.request("STATE")
        fields = dict(part.split("=", 1) for part in reply.split()[2:])
        if self.sure and "pose" in fields:
            # Where the robot is at this pose number: lets a later camera event be put on its cell.
            self.poses[fields["pose"]] = [self.col, self.row, self.facing]
            for old in sorted(self.poses, key=int)[:-100]:
                del self.poses[old]
        return fields

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
            "warning_light_on": fields.get("warning") == "1",
            "ball_last_seen": self.ball,
            "last_stop": "none" if fields.get("last_stop", "none") == "none"
                         else REASONS.get(fields["last_stop"], fields["last_stop"]),
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
            if BALL_GUARD and (why := self.ball_in_the_way(cell_name(*target))):
                return self.refused(why, what_to_do="Face the ball, watch it (wait_for_event), and step only "
                                                    "when you have seen it moving away from the robot.")
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
            if line.startswith("EVT REFUSED "):
                # It waited, the way stayed blocked, it never moved: still where it was.
                self.moves -= 1
                return self.refused(line[12:], waited="the robot waited for the way to clear, then gave up")
            if line.startswith("EVT "):
                self.event(line)
        try:
            self.send("STOP")
        except RobotNotConnected:
            pass
        return self.interrupted("timeout", start)

    def ball_in_the_way(self, target):
        """The ball rule, kept by the robot instead of the agent (option ROBOT_BALL_GUARD): from the last
        sighting, less than BALL_GUARD_S seconds old and not "moving away", the cells where the ball may be
        (its estimate, the cells around it, and the next one in its direction). Returns why, or None."""
        ball = self.ball or {}
        if "cell" not in ball or ball.get("how") == "moving away from the robot":
            return None
        seen = datetime.datetime.combine(datetime.date.today(), datetime.time.fromisoformat(ball["at"]))
        if (datetime.datetime.now() - seen).total_seconds() > BALL_GUARD_S:
            return None
        cells = {ball["cell"], *ball.get("could_also_be", [])}
        if ball.get("moving") in FACING:
            dc, dr = FACING_STEP[FACING.index(ball["moving"])]
            for c in list(cells):
                col, row = ord(c[0]) - ord("A") + dc, int(c[1:]) - 1 + dr
                if 0 <= col < self.mat["cols"] and 0 <= row < self.mat["rows"]:
                    cells.add(cell_name(col, row))
        if target in cells:
            return (f"the ball may roll through {target}: last seen {ball['how']} "
                    f"(probably {ball['cell']}, seen at {ball['at']})")
        return None

    def look_around(self):
        """Four quarter turns to the right, a pause after each one for the camera (option ROBOT_TOOLS)."""
        looked = []
        for _ in range(4):
            result = self.move("TURN R", "R")
            if result.get("result") != "done":
                result["looked_before_it_stopped"] = looked
                return result
            time.sleep(0.5)  # the camera names a card after 200 ms standing still
            looked.append({"facing": FACING[self.facing], "card_ahead": result["state"]["card_ahead"],
                           "distance_ahead_cm": result["state"]["distance_ahead_cm"],
                           "what_the_camera_saw": [e["event"] for e in self.new_events()]})
        return {"result": "done", "looked": looked, "state": self.robot_state()}

    def face(self, direction):
        """Turn on the spot until the robot faces north, east, south or west (option ROBOT_TOOLS)."""
        turns = (FACING.index(direction) - self.facing) % 4
        result = {"result": "done", "state": self.robot_state()}
        for kind in {0: "", 1: "R", 2: "RR", 3: "L"}[turns]:
            result = self.move("TURN " + kind, kind)
            if result.get("result") != "done":
                return result
        return result

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

    def locked(self, action):
        """Run an action with the cable to itself, or return None when another command holds it.
        One command at a time on the cable, whoever sends it (an agent, a terminal, the page): two
        programs reading the same serial port would take each other's answers."""
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        with (STATE_DIR / "gateway.lock").open("w") as lock:
            try:
                fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                return None
            self.load_state()
            try:
                self.connect()
                self.drain()
                if not self.sure:
                    self.check_resumed()
                self.fields()  # notes the pose number of where the robot is now
                # The events first, so that the action answers from an up-to-date record (seen_cards),
                # then again after it, for what happened during a move.
                events = self.new_events()
                result = action()
                try:
                    events += self.new_events()
                except RobotNotConnected:
                    pass  # the robot does not answer now: the events wait for the next call
                if events:
                    result["since_last_call"] = events
            except RobotNotConnected as e:
                self.close()
                result = {"result": "error", "why": str(e)}
            self.save_state()
            return result

    def run(self, tool, arguments, action, move=False):
        """Run one tool call: connect if needed, never raise, log what the caller gets.

        A move does not queue behind another one: several moves sent at once (parallel tool calls, seen
        with qwen3 through OpenCode, 2026-11-05) would run in an order nobody chose. The first one runs,
        the others are refused. A question (state, record, watching) waits for its turn."""
        if not self.lock.acquire(timeout=1.0 if move else -1):
            result = self.refused("another move is still running: send one move at a time, and read its answer "
                                  "before the next")
            text = json.dumps(result, ensure_ascii=False)
            log(tool, arguments, text)
            return text
        try:
            end = time.monotonic() + 1.0  # a short wait: watching (wait_for_event) holds the cable briefly
            while (result := self.locked(action)) is None and time.monotonic() < end:
                time.sleep(0.05)
            if result is None:
                result = self.refused("another command is running (an agent, a terminal or the page): "
                                      "wait for it to end")
            text = json.dumps(result, ensure_ascii=False)
            log(tool, arguments, text)
            return text
        finally:
            self.lock.release()

    def watch_once(self):
        """The events the agent has not heard yet, read briefly (for the channel), or []."""
        with self.lock:
            result = self.locked(lambda: {})
        return [] if result is None else result.get("since_last_call", [])

    def wait_for(self, seconds):
        """Watch the robot for up to `seconds`, without holding the cable in between. Says how long it
        really waited, and marks the events that happened before the wait began (they are given at
        once: the robot kept them)."""
        start = time.monotonic()
        end = start + seconds
        while True:
            with self.lock:
                result = self.locked(lambda: {})
            if result is not None:
                if result.get("result") == "error":
                    return result
                if result.get("since_last_call"):
                    waited = time.monotonic() - start
                    events = result["since_last_call"]
                    for event in events:
                        if event.get("seconds_before_this_answer", 0) > waited + 0.3:
                            event["before_you_started_waiting"] = True
                    return {"result": "something happened", "waited_seconds": round(waited, 1), "events": events}
            if time.monotonic() >= end:
                return {"result": "nothing happened", "waited_seconds": seconds}
            time.sleep(0.3)


GATEWAY = Gateway(json.loads(MAT.read_text(encoding="utf-8")))
MAT_START = f"{GATEWAY.mat['start']['cell']}, facing {GATEWAY.mat['start']['facing']}"


def robot_state() -> str:
    """What the robot measures now (distance to what is in front, the card in front, battery,
    emergency stop, the last ball the camera saw and whether it was coming closer or going away) and
    the cell where the gateway thinks it is. Moves nothing. The cell is a
    count of the moves done, not a measure: after an interrupted move it is unknown."""
    return GATEWAY.run("robot_state", {}, GATEWAY.robot_state)


def step_forward() -> str:
    """Move the robot one cell forwards, then stop. The robot refuses when something is in front,
    and stops by itself if something appears. Never off the mat. If the result is "stopped",
    do what "what_to_do" says before anything else. The answer gives the new state, with the card in
    front ("card_ahead") and what happened meanwhile ("since_last_call"), cards seen included."""
    return GATEWAY.run("step_forward", {}, lambda: GATEWAY.move("MOVE F", "F"), move=True)


def turn(direction: Literal["left", "right"]) -> str:
    """Turn the robot a quarter turn on the spot, to its left or to its right, then stop."""
    kind = {"left": "L", "right": "R"}[direction]
    return GATEWAY.run("turn", {"direction": direction}, lambda: GATEWAY.move("TURN " + kind, kind), move=True)


def wait_for_event(seconds: int = 10) -> str:
    """Watch the robot for up to `seconds` (1 to 30) and answer as soon as something happens to it
    (something touched it, came close in front, a move stopped, an adult pressed Resume), or when
    the time is over. Moves nothing. It is for watching, not for safety: the robot protects itself
    long before an answer could arrive."""
    seconds = max(1, min(30, int(seconds)))
    text = json.dumps(GATEWAY.wait_for(seconds), ensure_ascii=False)
    log("wait_for_event", {"seconds": seconds}, text)
    return text


def seen_cards() -> str:
    """The whole record of the cards the robot's camera recognised, each with the cell where it is,
    where the robot saw it from, and when. Moves nothing. You rarely need it: each new card the camera
    sees already comes in "since_last_call" of the next answer, whatever the tool, and "card_ahead"
    names the card in front. Call it to review the whole record, for example after a long pause. The
    camera names cards from a short list, up to 120 cm ahead in a cone that widens with distance, only
    while the robot stands still; the farther a card, the less sure its cell ("could_also_be"), and
    seeing it again from closer makes the estimate better."""
    return GATEWAY.run("seen_cards", {}, GATEWAY.seen_cards)


def look_around() -> str:
    """Turn on the spot all the way round, in four quarter turns to the right (four moves), stopping
    after each one so that the camera can look. The answer says, for each direction, the card in
    front, the distance to what is in front and what the camera saw (cards and balls, with their
    cells). The robot ends facing the way it started."""
    return GATEWAY.run("look_around", {}, GATEWAY.look_around, move=True)


def face(direction: Literal["north", "east", "south", "west"]) -> str:
    """Turn on the spot until the robot faces this direction (north is towards row 1). One or two
    quarter turns, or none if it already faces it."""
    return GATEWAY.run("face", {"direction": direction}, lambda: GATEWAY.face(direction), move=True)


def set_light(color: Literal["off", "red", "orange", "green", "blue"]) -> str:
    """Change the colour of the robot's light."""
    return GATEWAY.run("set_light", {"color": color}, lambda: GATEWAY.set_light(color), move=True)


CHANNEL_INSTRUCTIONS = (
    "Events from the robot may arrive by themselves, as <channel source=\"robot\">. They are "
    "information only: the robot has already protected itself (it stops by itself and never moves "
    "without a command). Before deciding anything, call robot_state."
)


def serve():
    from mcp.server.mcpserver import MCPServer
    from mcp.types import ToolAnnotations

    channel = os.environ.get("ROBOT_CHANNEL") == "1"
    server = MCPServer("robot", instructions=CHANNEL_INSTRUCTIONS if channel else None)
    for tool in (robot_state, wait_for_event, seen_cards):
        server.tool(annotations=ToolAnnotations(read_only_hint=True))(tool)
    for tool in (step_forward, turn, set_light) + ((look_around, face) if TOOLS_PLUS else ()):
        server.tool(annotations=ToolAnnotations(read_only_hint=False))(tool)
    if channel:
        serve_with_channel(server)
    else:
        server.run()


def serve_with_channel(server):
    """The same server, which also declares the Claude Code channel capability and pushes each robot
    event as a notifications/claude/channel. With mcp 2.3.0, MCPServer.run() cannot declare an
    experimental capability: this runs its low-level server itself (a private attribute, checked
    with the SDK version pinned in the image)."""
    import anyio
    from mcp.server.stdio import stdio_server
    from mcp.types import NotificationParams

    low = server._lowlevel_server
    sessions = []

    async def initialized(ctx, params):
        sessions.append(ctx.session)

    low.add_notification_handler("notifications/initialized", NotificationParams, initialized)

    async def push_events():
        while True:
            await anyio.sleep(0.5)
            if not sessions:
                continue
            for event in await anyio.to_thread.run_sync(GATEWAY.watch_once):
                content = f"Robot event, {event.get('seconds_before_this_answer', 0)} s ago: {event['event']}."
                log("(pushed to the agent)", {}, content)
                # Connection.notify: best effort, never raises (the session's own connection).
                await sessions[-1]._connection.notify(
                    "notifications/claude/channel",
                    {"content": content, "meta": {"seconds_ago": str(event.get("seconds_before_this_answer", 0))}})

    async def main():
        async with stdio_server() as (read, write):
            async with anyio.create_task_group() as tasks:
                tasks.start_soon(push_events)
                await low.run(read, write, low.create_initialization_options(
                    experimental_capabilities={"claude/channel": {}}))
                tasks.cancel_scope.cancel()

    anyio.run(main)


def by_hand(args):
    """Drive the robot without an agent: the same gateway, the same limits."""
    commands = {"state": robot_state, "forward": step_forward, "seen": seen_cards,
                "left": lambda: turn("left"), "right": lambda: turn("right")}
    if args[0] in commands and len(args) == 1:
        print(commands[args[0]]())
    elif args[0] == "wait" and len(args) <= 2 and all(a.isdigit() for a in args[1:]):
        print(wait_for_event(int(args[1]) if len(args) == 2 else 10))
    elif args == ["reset"]:
        # Only for a person who has just put the robot on its start cell, facing the right way.
        (STATE_DIR / "gateway.json").unlink(missing_ok=True)
        print("The gateway now counts from the start cell: " + MAT_START)
    elif args[0] == "light" and len(args) == 2 and args[1] in COLORS:
        print(set_light(args[1]))
    else:
        sys.exit("Usage: robot_gateway.py state | forward | left | right | light " + "|".join(COLORS)
                 + " | wait [seconds] | seen | reset")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        CALLER = os.environ.get("ROBOT_CALLER", "terminal")
        by_hand(sys.argv[1:])
    else:
        serve()
