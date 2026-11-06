"""The gateway between an agent and the robot: the only door.

It is an MCP server with nine tools, and nothing else reaches the robot:
  - robot_state()        what the robot measures now, and where the gateway thinks it is (read only)
  - step_forward()       one cell forwards
  - move(direction, cells, stop_if)
                         slide up to four cells north, east, south or west without turning (mecanum
                         wheels), stopping on a cell when a condition is met (one move)
  - turn(direction)      a quarter turn, "left" or "right"
  - look_around()        a full turn on the spot, looking in each direction (one move)
  - face(direction)      turn on the spot towards north, east, south or west (one move)
  - set_light(color)     the light of the robot, from a short list
  - wait_for_event(s)    watch the robot for up to s seconds, until something happens (read only)
  - seen_cards()         the cards the camera recognised, with the cell where each one is (read only)
Every answer also carries "since_last_call": what happened to the robot since this caller's last
call (a bump, a warning, a stop, Resume), from the robot's own memory of its last events.

With ROBOT_CHANNEL=1, the gateway also pushes each event to the agent as it happens, as a Claude
Code "channel" (research preview of Claude Code: interactive sessions only, started with
--dangerously-load-development-channels server:robot). No other agent reads these notifications.

The gateway keeps its own limits, whatever the agent asks: four cells per call at most, never off
the mat, backwards or sideways only into cells its distance sensor saw free, a number of moves per
session (a command counts one, however many cells), a time limit on each cell. It never sends an image: the agent gets
short texts, and each one is written to .robot-sim/context.jsonl, so that people can read exactly
what entered the agent's context (the simulator shows it).

It talks to the robot through a serial port: the USB cable of the robot, or the pseudo-terminal of
the simulator (its path is in .robot-sim/serial). Set ROBOT_SERIAL to choose another port.

Usage:
    as an MCP server (stdio), for an agent:  /opt/workshop/venv/bin/python -I gateway/robot_gateway.py
    by hand, without an agent:               python3 -I gateway/robot_gateway.py state|forward|left|right|around|face <direction>|move <direction> [cells] [conditions]|light <color>
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
# move(): the most cells per call (the robot's own MAX_CELLS), how long a free reading of the distance
# sensor stays believed (a box can be put down meanwhile), how long a ball seen near the way forbids a
# move backwards or sideways. Choices of the gateway, not measures.
MAX_CELLS = 4
FREE_FOR_S = 120
BALL_RECENT_S = 10
# A ball rolls back and forth on a line: once seen moving along a row or a column, that line is its lane for
# LANE_FOR_S. The robot never slides into a lane sideways or backwards (it would not see the ball coming).
LANE_FOR_S = 600
# A cell counts as free when the sensor sees nothing up to its far side plus this margin (as
# reflex.STOP_DISTANCE_CM). The beam is narrower than the robot (5 degrees each side in the simulator,
# TO BE COMPLETED on the robot): a free reading only says the line in front of it is clear.
STOP_MARGIN_CM = 8
STOP_IF = {"card_ahead": "CARD", "ball_seen": "BALL", "new_card": "NEW"}
WHY_DONE = {
    "all": "it moved all the cells asked",
    "card": "a card is right in front of the camera",
    "ball": "the camera sees a ball",
    "new": "the camera shows a kind of card (a name) it had not shown when the move began",
    "obstacle": "something was in the way: it waited, then stayed on the last cell it reached",
    "bump": "something touched the robot's bumper while it stood on a cell",
    "estop": "someone pressed the emergency stop button while it stood on a cell",
    "watchdog": "it heard nothing from the computer while it stood on a cell",
    "command": "the gateway stopped it on a cell",
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
            self.free = saved.get("free", {})
            self.lanes = saved.get("lanes", {})
            self.way = saved.get("way")
            self.back_fixed = saved.get("back_fixed", False)
            self.placed = saved.get("placed", {})
            self.presses = saved.get("presses", 0)
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
            self.free = {}  # cell -> when the distance sensor last saw it free (time.time())
            self.lanes = {}  # "row:3", "col:4" or "cell:E3" -> when a ball was last seen there (time.time())
            self.way = None  # during a move: its start cell, then the cells of a slide
            self.back_fixed = False  # True once the cell given to the adult is final
            self.placed = {}  # cell -> when a card was last placed there for sure (kept when a box hides it)
            self.presses = 0  # the robot's count of Resume presses, at its last answer

    def save_state(self):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        saved = {"col": self.col, "row": self.row, "facing": self.facing, "sure": self.sure,
                 "back": self.back, "moves": self.moves, "resumes_at_stop": self.resumes_at_stop,
                 "cursors": self.cursors, "poses": self.poses, "seen": self.seen,
                 "ball": self.ball, "free": self.free, "lanes": self.lanes, "way": self.way, "back_fixed": self.back_fixed,
                 "placed": self.placed, "presses": self.presses}
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
        if not reply.startswith("OK EVENTS"):
            raise RobotNotConnected(f"the robot's answer could not be read ({reply[:40]!r})")
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
        if not others:
            self.placed[cell] = time.time()
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
            self.ball = {"how": how, "at": at, "t": time.time() - age_ms / 1000}
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
                     "seen_from": seen_from, "at": at, "t": time.time() - age_ms / 1000}
        # Its lane, for every cell it could be on (from far, the estimate can fall on the next row).
        moving, when = going[9:], time.time() - age_ms / 1000
        for name in [cell] + others:
            lane = (f"row:{int(name[1:]) - 1}" if moving in ("east", "west") else
                    f"col:{ord(name[0]) - ord('A')}" if moving in ("north", "south") else f"cell:{name}")
            self.lanes[lane] = when
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

    def card_cells(self, all_seen=False):
        """The cells where the gateway placed a card the camera recognised, when it is sure of the cell (a card
        seen from far could be on a neighbour: it would then block the cell in front of it), and the cells of
        sure cards that left the camera's view in the last LANE_FOR_S (hidden by a box, they are still there).
        all_seen: every probable cell, sure or not (for a slide sideways, which no camera watches)."""
        now = time.time()
        sure = {s["cell"] for s in self.seen if all_seen or not s.get("could_also_be")}
        return sure | {c for c, t in self.placed.items() if now - t < LANE_FOR_S}

    def seen_cards(self):
        keys = ("card", "means", "cell", "could_also_be", "distance_cm", "seen_from", "at")
        cards = [{k: s[k] for k in keys} for s in sorted(self.seen, key=lambda s: s["at"])]
        return {"cards_seen": cards,
                "note": "The camera sees up to 120 cm ahead, wider when farther, and guesses the distance: the "
                        "farther the card, the less sure its cell (could_also_be). Seen again from closer, the "
                        "estimate gets better. The robot's own cell comes from counting its moves, not from a measure."}

    def fields(self):
        reply = self.request("STATE")
        if not reply.startswith("OK STATE"):
            raise RobotNotConnected(f"the robot's answer could not be read ({reply[:40]!r})")
        fields = dict(part.split("=", 1) for part in reply.split()[2:])
        if fields.get("resumes", "").isdigit():
            self.presses = int(fields["resumes"])
        if self.sure and "pose" in fields:
            # Where the robot is at this pose number: lets a later camera event be put on its cell.
            self.poses[fields["pose"]] = [self.col, self.row, self.facing]
            for old in sorted(self.poses, key=int)[:-100]:
                del self.poses[old]
        if (self.sure and fields.get("moving") == "0" and fields.get("wait_adult") == "0"
                and fields.get("estop") == "0" and fields.get("dist", "none") != "none"):
            self.note_free(int(fields["dist"]))
        return fields

    def note_free(self, dist):
        """The distance sensor, standing still, sees nothing for `dist` cm in front: the cells the robot
        could slide into on that line, with its stopping margin, are free (for boxes; a ball moves)."""
        dc, dr = FACING_STEP[self.facing]
        now = time.time()
        k = 1
        while True:
            col, row = self.col + dc * k, self.row + dr * k
            if not (0 <= col < self.mat["cols"] and 0 <= row < self.mat["rows"]):
                break
            if k * self.mat["cell_cm"] + STOP_MARGIN_CM <= dist:
                self.free[cell_name(col, row)] = now
            else:
                # Something is there, or behind it the sensor cannot tell: an older "free" no longer holds.
                self.free.pop(cell_name(col, row), None)
            k += 1

    def on_lane(self, col, row):
        """A cell on the lane of a ball seen in the last LANE_FOR_S (its row or column, or the cell)."""
        now = time.time()
        for key, when in self.lanes.items():
            kind, _, value = key.partition(":")
            if now - when > LANE_FOR_S:
                continue
            if (kind == "row" and int(value) == row) or (kind == "col" and int(value) == col) \
                    or (kind == "cell" and value == cell_name(col, row)):
                return True
        return False

    def ball_near(self, cells):
        """The first of these cells next to where a ball was seen less than BALL_RECENT_S ago, or None."""
        ball = self.ball or {}
        if not ball.get("cell") or time.time() - ball.get("t", 0) >= BALL_RECENT_S:
            return None
        near = set()
        for name in [ball["cell"]] + ball.get("could_also_be", []):
            c, r = ord(name[0]) - ord("A"), int(name[1:]) - 1
            near |= {(c, r), (c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)}
        for col, row in cells:
            if (col, row) in near:
                return cell_name(col, row)
        return None

    def blind_way(self, cells):
        """Why a move backwards or sideways through these cells is not allowed, or None."""
        now = time.time()
        for col, row in cells:
            if now - self.free.get(cell_name(col, row), 0) > FREE_FOR_S:
                return cell_name(col, row), "not_seen"
            if self.on_lane(col, row):
                return cell_name(col, row), "lane"
        near = self.ball_near(cells)
        return (near, "ball") if near else None

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
        if not resumed and not self.back_fixed:
            # Before the adult acts: the instruction names the cell the robot itself says it reached
            # (a gateway killed during a move, or a lost EVT CELL). Once given, it never changes (the adult
            # may already be acting on it), and never after Resume.
            self.reached(fields)
        if resumed:
            self.col, self.row, self.facing = self.back
            self.sure, self.back, self.way, self.resumes_at_stop = True, None, None, None

    def drain(self):
        """Read what the robot said while nobody asked (Resume pressed, old answers)."""
        while True:
            line = self.read_line(0)
            if line is None:
                return
            if line.startswith("EVT "):
                self.event(line)

    def request(self, line, timeout=1.0):
        """Send a command and return its answer: "OK <the same word> …" or "ERR …". An older answer
        still on the wire (OK PONG, the OK STOP of a move that timed out) is skipped: taken for this
        answer, it would shift every answer after it."""
        self.send(line)
        word = line.split()[0]
        end = time.monotonic() + timeout
        while True:
            reply = self.read_line(max(0.0, end - time.monotonic()))
            if reply is None:
                raise RobotNotConnected("the robot does not answer")
            if reply.startswith("EVT "):
                self.event(reply)
                continue
            if reply.startswith("OK ") and not reply.startswith("OK " + word):
                continue
            return reply

    def begin(self, start, way, counted):
        """Just before a move is sent: written to disk as if it were interrupted (position unknown, put
        back on its start cell), so that a gateway killed during the move leaves the truth behind.
        `way`: the cells of a slide; the robot's own count (STATE slid=) says which one it reached."""
        self.sure, self.back, self.way = False, start, [list(start[:2])] + [list(c) for c in way]
        self.back_fixed = False
        self.resumes_at_stop = self.presses
        self.moves += counted
        self.save_state()

    def ended(self, counted_back=0):
        """The move ended on a cell the gateway knows (done, or refused without moving)."""
        self.sure, self.back, self.way, self.resumes_at_stop = True, None, None, None
        self.moves -= counted_back
        self.save_state()

    def reached(self, fields):
        """Where an interrupted slide left the robot: the last cell it reached, by its own count."""
        if self.back and self.way and fields.get("slid", "").isdigit():
            k = min(int(fields["slid"]), len(self.way) - 1)
            while k > 0 and self.on_lane(*self.way[k]):
                k -= 1  # never put it back on a ball's lane: on the last cell before it
            col, row = self.way[k]
            self.back = (col, row, self.back[2])
            self.back_fixed = True

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
            "on_a_ball_lane": self.on_lane(self.col, self.row) if self.sure else None,
            "last_stop": "none" if fields.get("last_stop", "none") == "none"
                         else REASONS.get(fields["last_stop"], fields["last_stop"]),
            "moves_left": self.mat["max_moves"] - self.moves,
            "mat": f"{self.mat['cols']} columns (A to {cell_name(self.mat['cols'] - 1, 0)[0]}), "
                   f"{self.mat['rows']} rows (1 at the top to {self.mat['rows']})",
        }

    def refused(self, why, **extra):
        return dict({"result": "refused", "why": REASONS.get(why, why)}, **extra)

    def move(self, command, kind, counted=True):
        """MOVE F, TURN L or TURN R: one move, watched until it ends. Not counted: the next quarter
        turns of look_around and face, which count as one move in all."""
        if not self.sure:
            return self.refused("wait_adult", what_to_do=self.adult_instruction())
        if counted and self.moves >= self.mat["max_moves"]:
            return self.refused(f"no moves left in this session ({self.mat['max_moves']})")
        if kind == "F":
            dc, dr = ((0, -1), (1, 0), (0, 1), (-1, 0))[self.facing]
            target = (self.col + dc, self.row + dr)
            if not (0 <= target[0] < self.mat["cols"] and 0 <= target[1] < self.mat["rows"]):
                return self.refused("the next cell is outside the mat")
            if self.fields().get("card", "none") != "none" or cell_name(*target) in self.card_cells():
                return self.refused(f"a card is on the next cell ({cell_name(*target)}): the robot never drives onto a "
                                    "card", what_to_do="stop here if it is the one you look for, or go round it")
            near = self.ball_near([target])
            if near:
                return self.refused(f"a ball was seen near {near} less than {BALL_RECENT_S} s ago",
                                    what_to_do="watch it go away (wait_for_event), then try again")
        start = (self.col, self.row, self.facing)
        self.begin(start, [target] if kind == "F" else [], counted)
        reply = self.request(command)
        if reply.startswith("ERR "):
            self.ended(counted_back=counted)
            return self.refused(reply[4:])
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
                    self.free[cell_name(self.col, self.row)] = self.free[cell_name(*target)] = time.time()
                    self.col, self.row = target
                else:
                    self.facing = (self.facing + (1 if kind == "R" else -1)) % 4
                self.ended()
                return {"result": "done", "state": self.robot_state()}
            if line.startswith("EVT STOP "):
                return self.interrupted(line[9:], start)
            if line.startswith("EVT REFUSED "):
                # It waited, the way stayed blocked, it never moved: still where it was.
                self.ended(counted_back=counted)
                return self.refused(line[12:], waited="the robot waited for the way to clear, then gave up")
            if line.startswith("EVT "):
                self.event(line)
        self.stop_now()
        return self.interrupted("timeout", start)

    def stop_now(self):
        """A move took too long: stop the robot, and read its OK STOP (left unread, it would be taken for
        the answer to the next question)."""
        try:
            self.request("STOP")
        except RobotNotConnected:
            pass

    def translate(self, direction, cells, stop_if):
        """MOVE <d> <n> [UNTIL …]: slide towards a side of the mat without turning, one move however many
        cells. The robot reports each cell it reaches (EVT CELL), so the gateway keeps counting."""
        if not self.sure:
            return self.refused("wait_adult", what_to_do=self.adult_instruction())
        if self.moves >= self.mat["max_moves"]:
            return self.refused(f"no moves left in this session ({self.mat['max_moves']})")
        side = FACING.index(direction)
        kind = "FRBL"[(side - self.facing) % 4]
        dc, dr = FACING_STEP[side]
        way = []
        for k in range(1, cells + 1):
            col, row = self.col + dc * k, self.row + dr * k
            if not (0 <= col < self.mat["cols"] and 0 <= row < self.mat["rows"]):
                break
            way.append((col, row))
        if not way:
            return self.refused("the next cell is outside the mat")
        edge = len(way) < cells
        cards, card_stop = self.card_cells(all_seen=kind != "F"), None
        for k, cell in enumerate(way):
            if cell_name(*cell) in cards:
                card_stop, way = cell_name(*cell), way[:k]
                break
        if not way:
            return self.refused(f"a card was seen on the next cell that way ({card_stop}): the robot never drives "
                                "onto a card", what_to_do="stop here if it is the one you look for, or go round it")
        if kind == "F":
            near = self.ball_near(way)
            if near:
                return self.refused(f"a ball was seen near {near} less than {BALL_RECENT_S} s ago",
                                    what_to_do="watch it go away (wait_for_event), then try again")
        else:
            blocked = self.blind_way(way)
            if blocked:
                cell, why = blocked
                if why == "lane":
                    return self.refused(f"{cell} is on the lane of a ball, and the robot cannot see {direction} while "
                                        f"it faces {FACING[self.facing]}: it never slides into a ball's lane sideways "
                                        "or backwards", what_to_do=f"face {direction}, watch the ball go away, then "
                                        "cross forwards")
                if why == "ball":
                    return self.refused(f"a ball was seen near {cell} less than {BALL_RECENT_S} s ago, and the robot "
                                        f"cannot see {direction} while it faces {FACING[self.facing]}",
                                        what_to_do="watch it go away (wait_for_event, face it), then try again")
                return self.refused(f"the robot only sees in front of it ({FACING[self.facing]}): it slides {direction} "
                                    f"only into cells its distance sensor saw free in the last {FREE_FOR_S // 60} "
                                    f"minutes, and {cell} is not one",
                                    what_to_do=f"face {direction} (one move) or look_around, then move again")
        conditions = [STOP_IF[c] for c in stop_if]
        if kind == "F" and "CARD" not in conditions:
            conditions.append("CARD")  # forwards, it always stops in front of a card, never on it
        command = f"MOVE {kind} {len(way)}" + (" UNTIL " + ",".join(conditions) if conditions else "")
        start, done = (self.col, self.row, self.facing), 0
        self.begin(start, way, 1)
        reply = self.request(command)
        if reply.startswith("ERR "):
            self.ended(counted_back=1)
            return self.refused(reply[4:])
        # The robot stands on each cell until its camera's view is believed (2 s at most).
        look = 2.5 if conditions else 0
        end = time.monotonic() + self.mat["move_timeout_s"] * 2 + look  # sideways: half the speed
        next_ping = time.monotonic() + HEARTBEAT_S
        while time.monotonic() < end:
            line = self.read_line(max(0.0, min(end, next_ping) - time.monotonic()))
            if time.monotonic() >= next_ping:
                self.send("PING")  # "I am still here": without it, the robot stops (its watchdog)
                next_ping = time.monotonic() + HEARTBEAT_S
            if line is None or line == "OK PONG":
                continue
            if line.startswith("EVT CELL "):
                k, pose = line.split()[2:4]
                done = int(k)
                before = cell_name(self.col, self.row)
                self.col, self.row = way[done - 1]
                self.free[before] = self.free[cell_name(self.col, self.row)] = time.time()  # it went through them
                self.poses[pose] = [self.col, self.row, self.facing]
                if not self.on_lane(self.col, self.row):
                    start = (self.col, self.row, self.facing)  # an adult would put it back here
                end = time.monotonic() + self.mat["move_timeout_s"] * 2 + look
                continue
            if line.startswith(f"EVT DONE {kind} "):
                why = line.split()[4]
                reason = WHY_DONE.get(why, why)
                if why == "all" and card_stop:
                    reason = f"it stopped before {card_stop}, where the camera saw a card: it never drives onto a card"
                elif why == "all" and edge:
                    reason = f"the mat ends after {len(way)} cell{'s' if len(way) > 1 else ''} that way"
                self.ended(counted_back=1 if done == 0 else 0)  # it never moved: like a refusal, not counted
                return {"result": "done", "cells_moved": done, "cells_asked": cells, "stopped_because": reason,
                        "state": self.robot_state()}
            if line.startswith("EVT STOP "):
                result = self.interrupted(line[9:], start)
                result["cells_moved"] = done
                return result
            if line.startswith("EVT REFUSED "):
                self.ended(counted_back=1)
                return self.refused(line[12:], waited="the robot waited for the way to clear, then gave up")
            if line.startswith("EVT "):
                self.event(line)
        self.stop_now()
        result = self.interrupted("timeout", start)
        result["cells_moved"] = done
        return result

    def look_around(self):
        """Four quarter turns to the right, a pause after each one for the camera: one move in all."""
        looked = []
        for n in range(4):
            result = self.move("TURN R", "R", counted=n == 0)
            if result.get("result") != "done":
                result["looked_before_it_stopped"] = looked
                return result
            time.sleep(0.5)  # the camera names a card after 200 ms standing still
            looked.append({"facing": FACING[self.facing], "card_ahead": result["state"]["card_ahead"],
                           "distance_ahead_cm": result["state"]["distance_ahead_cm"],
                           "what_the_camera_saw": [e["event"] for e in self.new_events()]})
        return {"result": "done", "looked": looked, "state": self.robot_state()}

    def face(self, direction):
        """Turn on the spot until the robot faces north, east, south or west: one move, or none."""
        if not self.sure:
            return self.refused("wait_adult", what_to_do=self.adult_instruction())
        turns = (FACING.index(direction) - self.facing) % 4
        result = {"result": "done", "state": self.robot_state()}
        for n, kind in enumerate({0: "", 1: "R", 2: "RR", 3: "L"}[turns]):
            result = self.move("TURN " + kind, kind, counted=n == 0)
            if result.get("result") != "done":
                return result
        return result

    def interrupted(self, why, start):
        self.sure, self.back = False, start
        try:
            fields = self.fields()
            self.resumes_at_stop = int(fields.get("resumes", 0))
            self.reached(fields)  # an EVT CELL may have been lost on a loose cable
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
            except (ValueError, KeyError, IndexError) as e:  # an answer of the robot that makes no sense
                self.close()
                result = {"result": "error", "why": f"the robot's answer could not be read ({type(e).__name__})"}
            finally:
                self.save_state()  # whatever happened, what the gateway knows reaches the disk
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
        """Watch the robot for up to `seconds`, without holding the cable in between. Answers at the first
        event that happens during the wait. Events from before the wait (the robot kept them) come with the
        answer, marked, but do not end it: an agent that waits for a Resume or a ball would otherwise get them
        back at once, again and again (seen on 5 and 6 November)."""
        start = time.monotonic()
        end = start + seconds
        earlier = []  # [time of the poll that read them, event]: events from before the wait

        def kept():
            # Their age as of this answer, not as of the poll that read them.
            now = time.monotonic()
            return [dict(e, seconds_before_this_answer=round(e.get("seconds_before_this_answer", 0) + now - at, 1))
                    for at, e in earlier]

        while True:
            with self.lock:
                result = self.locked(lambda: {})
            if result is not None:
                if result.get("result") == "error":
                    if earlier:
                        result["events"] = kept()
                    return result
                events = result.get("since_last_call", [])
                waited = time.monotonic() - start
                for event in events:
                    if event.get("seconds_before_this_answer", 0) > waited + 0.3:
                        event["before_you_started_waiting"] = True
                if any(not e.get("before_you_started_waiting") for e in events):
                    return {"result": "something happened", "waited_seconds": round(waited, 1), "events": kept() + events}
                earlier += [(time.monotonic(), e) for e in events]
            if time.monotonic() >= end:
                answer = {"result": "nothing new happened" if earlier else "nothing happened", "waited_seconds": seconds}
                if earlier:
                    answer["events"] = kept()
                return answer
            time.sleep(0.3)


GATEWAY = Gateway(json.loads(MAT.read_text(encoding="utf-8")))
MAT_START = f"{GATEWAY.mat['start']['cell']}, facing {GATEWAY.mat['start']['facing']}"


def robot_state() -> str:
    """What the robot measures now (distance to what is in front, the card in front, battery,
    emergency stop, the last ball the camera saw and whether it was coming closer or going away) and
    the cell where the gateway thinks it is, and whether that cell is on a ball's lane. Moves nothing. The cell is a
    count of the moves done, not a measure: after an interrupted move it is unknown."""
    return GATEWAY.run("robot_state", {}, GATEWAY.robot_state)


def step_forward() -> str:
    """Move the robot one cell forwards, then stop. The robot refuses when something is in front,
    and stops by itself if something appears. Never off the mat, never onto a card, never near a ball
    seen in the last 10 seconds. If the result is "stopped",
    do what "what_to_do" says before anything else. The answer gives the new state, with the card in
    front ("card_ahead") and what happened meanwhile ("since_last_call"), cards seen included."""
    return GATEWAY.run("step_forward", {}, lambda: GATEWAY.move("MOVE F", "F"), move=True)


def move(direction: Literal["north", "east", "south", "west"], cells: int = 1,
         stop_if: list[Literal["card_ahead", "ball_seen", "new_card"]] | None = None) -> str:
    """Slide the robot 1 to 4 cells (`cells`) towards north, east, south or west (north is towards
    row 1), without turning: its wheels move it in any of the four directions, and the camera keeps
    looking the way the robot faces. It counts as one move, however many cells. It stops on a cell, at
    the first of: all the cells done, the edge of the mat, or a condition of stop_if met:
    "card_ahead" (a card right in front of the camera, the way the robot faces; a slide forwards always
    stops in front of a card, with or without it), "ball_seen" (the camera sees a ball), "new_card" (the
    camera shows a kind of card, a name, it did not show when the move began). The robot never drives
    onto a card it knows: forwards it stops in front of any card; every slide stops before a cell where the
    gateway placed a card for sure, and a slide sideways or backwards before any card the camera placed,
    even from far. With stop_if, the robot
    stands still on each cell until its camera has looked. Its distance sensor and camera only look the
    way it faces: in any other direction it slides only into cells the sensor saw free in the last two
    minutes (face that way once, or look_around; the cells it has just been through count too), never into
    the lane of a ball (the row or column where one was seen rolling) and never near a ball seen just
    before (from far, a lane may cover two rows or columns); forwards too, never near a ball seen just
    before. The sensor's beam
    is narrow, so a small thing near the side of a cell can be missed, and then the bumper stops the
    robot. The answer says how many cells it moved and why it stopped; "stopped" means an adult must
    come, as for step_forward."""
    stop_if = list(dict.fromkeys(stop_if or []))
    arguments = {"direction": direction, "cells": cells, "stop_if": stop_if}
    if not isinstance(cells, int) or not 1 <= cells <= MAX_CELLS:
        text = json.dumps({"result": "refused", "why": f"cells must be 1 to {MAX_CELLS}: nothing was sent"})
        log("move", arguments, text)
        return text
    return GATEWAY.run("move", arguments, lambda: GATEWAY.translate(direction, cells, stop_if), move=True)


def turn(direction: Literal["left", "right"]) -> str:
    """Turn the robot a quarter turn on the spot, to its left or to its right, then stop."""
    kind = {"left": "L", "right": "R"}[direction]
    return GATEWAY.run("turn", {"direction": direction}, lambda: GATEWAY.move("TURN " + kind, kind), move=True)


def wait_for_event(seconds: int = 10) -> str:
    """Watch the robot for up to `seconds` (1 to 30) and answer as soon as something happens to it
    (something touched it, came close in front, a move stopped, an adult pressed Resume, the camera saw a
    ball or a card), or when the time is over. Events from before the wait come with the answer, marked
    "before_you_started_waiting", but do not end it. Moves nothing. It is for watching, not for safety: the
    robot protects itself long before an answer could arrive."""
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
    """Turn on the spot all the way round, in four quarter turns to the right, stopping after each one
    so that the camera can look. It counts as one move. The answer says, for each direction, the card in
    front, the distance to what is in front and what the camera saw (cards and balls, with their
    cells). The robot ends facing the way it started."""
    return GATEWAY.run("look_around", {}, GATEWAY.look_around, move=True)


def face(direction: Literal["north", "east", "south", "west"]) -> str:
    """Turn on the spot until the robot faces this direction (north is towards row 1): one or two
    quarter turns, counted as one move, or nothing if it already faces it."""
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
    for tool in (step_forward, move, turn, look_around, face, set_light):
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
    commands = {"state": robot_state, "forward": step_forward, "seen": seen_cards, "around": look_around,
                "left": lambda: turn("left"), "right": lambda: turn("right")}
    if args[0] in commands and len(args) == 1:
        print(commands[args[0]]())
    elif args[0] == "face" and len(args) == 2 and args[1] in FACING:
        print(face(args[1]))
    elif (args[0] == "move" and 2 <= len(args) <= 4 and args[1] in FACING and (len(args) < 3 or args[2].isdigit())
          and all(c in STOP_IF for c in (args[3].split(",") if len(args) == 4 else []))):
        print(move(args[1], int(args[2]) if len(args) >= 3 else 1, args[3].split(",") if len(args) == 4 else []))
    elif args[0] == "wait" and len(args) <= 2 and all(a.isdigit() for a in args[1:]):
        print(wait_for_event(int(args[1]) if len(args) == 2 else 10))
    elif args == ["reset"]:
        # Only for a person who has just put the robot on its start cell, facing the right way.
        (STATE_DIR / "gateway.json").unlink(missing_ok=True)
        print("The gateway now counts from the start cell: " + MAT_START)
    elif args[0] == "light" and len(args) == 2 and args[1] in COLORS:
        print(set_light(args[1]))
    else:
        sys.exit("Usage: robot_gateway.py state | forward | left | right | around | face " + "|".join(FACING)
                 + " | move " + "|".join(FACING) + " [cells 1-4] [" + ",".join(STOP_IF) + "]"
                 + " | light " + "|".join(COLORS) + " | wait [seconds] | seen | reset")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        CALLER = os.environ.get("ROBOT_CALLER", "terminal")
        by_hand(sys.argv[1:])
    else:
        serve()
