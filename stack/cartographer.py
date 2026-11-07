"""The Cartographer: keeps the map of the terrain and translates commands of the mat into moves of the robot.
Executive layer, between the Pilot (above) and the Referee (below).

The map of the terrain (.robot-sim/terrain.json), built from what the Referee passes up (the robot's state and
events), every piece of information dated and with its uncertainty:
  - where the robot probably is: a count of the moves done, not a measure; unknown after an interrupted move, until
    an adult puts it back and presses Resume;
  - the cards the camera recognised, each with the cell where it probably is and the other cells it could be on;
  - the last sighting of a ball and the lanes balls roll on (a row or a column, for LANE_FOR_S);
  - the cells the distance sensor saw free, the cells where it met something, the cells the camera looked at.
It forgets what got old, and everything when the robot is put back on its start cell. It never decides a move and
checks no rule: it answers the Referee's questions about the terrain.

Its language, from the Pilot (the mat: cells A1, B3…, north is towards row 1):
    step_forward(), move(direction, cells, stop_if), turn(left|right), look_around(), face(direction),
    set_light(color), robot_state(), seen_cards(), wait_for(seconds)
Each answer is a dict: "done", "refused" (with why and what_to_do, in words), "stopped" (an adult must come), or
"error"; every answer also carries "since_last_call": what happened to the robot since this caller's last call.
To the Referee: moves of the robot (forwards, backwards, to its left, to its right, n cells; a quarter turn).
"""
import datetime
import json
import math
import time

import common
from common import FACING, FACING_STEP, STATE_DIR, cell_name, cell_xy
from link import RobotNotConnected
from referee import Referee

REASONS = {
    "obstacle": "something is in front of the robot",
    "battery": "the battery is too low",
    "estop": "someone pressed the emergency stop button on the robot",
    "wait_adult": "the robot waits for an adult (a move was interrupted)",
    "busy": "the robot is still moving",
    "watchdog": "the robot stopped by itself: it heard nothing from the computer",
    "bump": "something touched the robot's bumper",
    "command": "the Referee stopped the robot",
    "timeout": "the move took too long: the Referee stopped the robot",
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
WHY_DONE = {
    "all": "it moved all the cells asked",
    "card": "a card is right in front of the camera",
    "ball": "the camera sees a ball",
    "new": "the camera shows a kind of card (a name) it had not shown when the move began",
    "obstacle": "something was in the way: it waited, then stayed on the last cell it reached",
    "bump": "something touched the robot's bumper while it stood on a cell",
    "estop": "someone pressed the emergency stop button while it stood on a cell",
    "watchdog": "it heard nothing from the computer while it stood on a cell",
    "command": "the Referee stopped it on a cell",
    "hold": "the mission was paused or stopped: it stopped on the next cell",
}
STOP_IF = {"card_ahead": "CARD", "ball_seen": "BALL", "new_card": "NEW"}
# How sure the Cartographer is of a distance given by the camera: this share of the distance, plus a few
# centimetres. The simulator's camera errs by 10 % by default; the OpenMV N6's is TO BE COMPLETED.
CAMERA_ERROR = 0.15
CAMERA_MIN_ERROR_CM = 5
# A ball rolls back and forth on a line: once seen moving along a row or a column, that line is its lane for
# LANE_FOR_S. A card placed for sure stays on its cell this long when it leaves the camera's view (a box hides it).
LANE_FOR_S = 600
# A cell counts as free when the sensor sees nothing up to its far side plus this margin (as
# reflex.STOP_DISTANCE_CM). The beam is narrower than the robot (5 degrees each side in the simulator, TO BE
# COMPLETED on the robot): a free reading only says the line in front of it is clear.
STOP_MARGIN_CM = 8
# What the camera has looked at (seen_cards): the cells whose centre is in its cone from where the robot stood still.
# The simulator's cone and robot (its distance sensor this far in front of its centre); the OpenMV N6's field of view
# and range, and the real robot's, are TO BE COMPLETED. Cards do not move during a mission: what the camera looked at
# stays looked at, until the robot is put back on its start cell.
CAMERA_HALF_ANGLE_DEG = 30
CAMERA_RANGE_CM = 120
SENSOR_FROM_CENTRE_CM = 10
# Something the distance sensor met on a cell at two moments this far apart stands there (a box): a ball rolls on.
STANDS_FOR_S = 10
# The border around the mat, which the distance sensor sees, this far outside its edge: the simulator's fence; the
# real mat has a visible border too, its distance TO BE COMPLETED. A reading longer than the way to the border, by more
# than READING_TOLERANCE_CM, rules a place out; by more than DOUBT_CM from the place the Cartographer believed, it no
# longer believes it (the wheels slipped).
BORDER_CM = 10
READING_TOLERANCE_CM = 6
DOUBT_CM = 20
# The compass is believed only near a quarter turn (a reading in the middle of a turn, or disturbed by a magnet, is not).
COMPASS_TOLERANCE_DEG = 25
TURNED = ["the way it faced at the start", "to the right of it", "the other way", "to the left of it"]


class Cartographer:
    def __init__(self, mat, rules):
        self.mat = mat
        self.rules = rules
        self.cards = mat["cards"]
        self.referee = Referee(mat, rules)
        # Asked during a slide and between the quarter turns of look_around and face: should the robot stop on the
        # next cell? The Pilot sets it while a mission runs (the Interpreter's pause or stop).
        self.hold_wanted = lambda: False
        self.load()

    # The map of the terrain: kept in a file, so that it survives a new process (a terminal, the page). No file: the
    # robot is on its start cell (the simulator removes the file when it puts the robot back there; on the real robot,
    # `drive.py reset`).
    def load(self):
        try:
            saved = common.load_json(STATE_DIR / "terrain.json")
            self.col, self.row, self.facing = saved["col"], saved["row"], saved["facing"]
            self.sure, self.back = saved["sure"], saved["back"]
            self.resumes_at_stop = saved.get("resumes_at_stop")
            self.poses = saved.get("poses", {})
            self.ball = saved.get("ball")
            self.seen = saved.get("seen", [])
            self.free = saved.get("free", {})
            self.lanes = saved.get("lanes", {})
            self.way = saved.get("way")
            self.back_fixed = saved.get("back_fixed", False)
            self.placed = saved.get("placed", {})
            self.presses = saved.get("presses", 0)
            self.looked = saved.get("looked", {})
            self.blocked = {c: v for c, v in saved.get("blocked", {}).items() if isinstance(v, list)}
            self.covered_said = saved.get("covered_said", False)
            self.frame = saved.get("frame")
            self.notices = saved.get("notices", [])
        except (OSError, ValueError, KeyError):
            # Where the robot starts: the rule "start" of the Referee's rules (adults), else the start of the mat.
            # "unknown": the Cartographer works relative to where it started, every place of the mat still possible.
            start = self.rules.get("start") or self.mat["start"]
            self.frame = None  # None: cells of the mat; else relative to the start, with the places still possible
            self.notices = []  # what the Cartographer has to tell the next caller (its position found, or lost)
            if start == "unknown" or "cell" not in start or "facing" not in start:
                # Unknown, or known in part: only the way it faces ({"facing": "north"}: it starts facing the top of
                # the mat), or only its cell. The places that fit are kept.
                cells = [cell_xy(start["cell"])] if isinstance(start, dict) and "cell" in start else \
                    [(c, r) for c in range(self.mat["cols"]) for r in range(self.mat["rows"])]
                facings = [FACING.index(start["facing"])] if isinstance(start, dict) and "facing" in start else range(4)
                self.frame = {"candidates": [[c, r, f] for c, r in cells for f in facings]}
                self.col, self.row, self.facing = self.mat["cols"] - 1, self.mat["rows"] - 1, 0
            else:
                self.col, self.row = cell_xy(start["cell"])
                self.facing = FACING.index(start["facing"])
            self.sure = True  # False after an interrupted move, until an adult puts the robot back
            self.back = None
            self.resumes_at_stop = None  # how many times Resume had been pressed when a move stopped
            self.poses = {}  # the robot's pose number -> [col, row, facing], as counted here
            self.seen = []  # the cards the camera recognised: estimates, the most precise kept
            self.ball = None  # the last sighting of a moving ball
            self.free = {}  # cell -> when the distance sensor last saw it free (time.time())
            self.lanes = {}  # "row:3", "col:4" or "cell:E3" -> when a ball was last seen there (time.time())
            self.way = None  # during a move: its start cell, then the cells of a slide
            self.back_fixed = False  # True once the cell given to the adult is final
            self.placed = {}  # cell -> when a card was last placed there for sure (kept when a box hides it)
            self.presses = 0  # the robot's count of Resume presses, at its last answer
            self.looked = {}  # cell -> when the camera last looked at it, or the robot stood on it (time.time())
            self.blocked = {}  # cell -> [first, last] time the distance sensor met something there (a box, a ball)
            self.covered_said = False  # the caller was told the camera has looked at every cell it can

    def save(self):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        saved = {"col": self.col, "row": self.row, "facing": self.facing, "sure": self.sure, "back": self.back,
                 "resumes_at_stop": self.resumes_at_stop, "poses": self.poses, "seen": self.seen, "ball": self.ball,
                 "free": self.free, "lanes": self.lanes, "way": self.way, "back_fixed": self.back_fixed,
                 "placed": self.placed, "presses": self.presses, "looked": self.looked, "blocked": self.blocked,
                 "covered_said": self.covered_said, "frame": self.frame, "notices": self.notices}
        (STATE_DIR / "terrain.json").write_text(json.dumps(saved) + "\n", encoding="utf-8")

    # Where the robot is on the mat --------------------------------------------------------------------------------
    # While the Cartographer does not know it, it counts in a frame relative to where the robot started: the start is
    # the middle cell of a grid twice the mat's size, facing "north" of that grid, and every place of the mat the start
    # could have been is kept (frame["candidates"], [col, row, facing] each). Readings of the distance sensor and
    # landmarks rule places out; when one is left, everything is moved onto the mat.
    @property
    def cols(self):
        return 2 * self.mat["cols"] - 1 if self.frame else self.mat["cols"]

    @property
    def rows(self):
        return 2 * self.mat["rows"] - 1 if self.frame else self.mat["rows"]

    def origin(self):
        return self.mat["cols"] - 1, self.mat["rows"] - 1

    def to_mat(self, start, col, row, facing=0):
        """A place of the relative frame, on the mat, had the robot started at `start` [col, row, facing]."""
        oc, orow = self.origin()
        dx, dy = col - oc, row - orow
        for _ in range(start[2]):
            dx, dy = -dy, dx  # a quarter turn to the right
        return start[0] + dx, start[1] + dy, (facing + start[2]) % 4

    def point_to_mat(self, start, x, y):
        cell = self.mat["cell_cm"]
        oc, orow = self.origin()
        dx, dy = x - (oc + 0.5) * cell, y - (orow + 0.5) * cell
        for _ in range(start[2]):
            dx, dy = -dy, dx
        return (start[0] + 0.5) * cell + dx, (start[1] + 0.5) * cell + dy

    def from_mat(self, here, col, row, facing=0):
        """A place of the mat, in a relative frame whose start is `here` [col, row, facing]."""
        oc, orow = self.origin()
        dx, dy = col - here[0], row - here[1]
        for _ in range(here[2]):
            dx, dy = dy, -dx  # a quarter turn to the left
        return oc + dx, orow + dy, (facing - here[2]) % 4

    def point_from_mat(self, here, x, y):
        cell = self.mat["cell_cm"]
        oc, orow = self.origin()
        dx, dy = x - (here[0] + 0.5) * cell, y - (here[1] + 0.5) * cell
        for _ in range(here[2]):
            dx, dy = dy, -dx
        return (oc + 0.5) * cell + dx, (orow + 0.5) * cell + dy

    def on_mat(self, col, row):
        return 0 <= col < self.mat["cols"] and 0 <= row < self.mat["rows"]

    def way_to_border(self, col, row, facing):
        """What the distance sensor reads at most from this cell of the mat, facing this way: the border."""
        cells = (row, self.mat["cols"] - 1 - col, self.mat["rows"] - 1 - row, col)[facing]
        return (cells + 0.5) * self.mat["cell_cm"] + BORDER_CM - SENSOR_FROM_CENTRE_CM

    def possible(self, col, row, facing, dist):
        """Could the robot stand there and read `dist`? Something in between makes a reading shorter, never longer."""
        return self.on_mat(col, row) and dist <= self.way_to_border(col, row, facing) + READING_TOLERANCE_CM

    def places(self):
        """The places of the mat the robot could be on now, [col, row, facing] each (one when it is known)."""
        if not self.frame:
            return [[self.col, self.row, self.facing]]
        return [list(self.to_mat(s, self.col, self.row, self.facing)) for s in self.frame["candidates"]]

    def compass(self, fields):
        """The way the robot faces on the mat, from its compass (the rule mat_north_up: north at the top), or None."""
        heading = fields.get("heading", "none")
        if not self.rules.get("mat_north_up", True) or not heading.isdigit():
            return None
        degrees = int(heading) % 360
        quarter = round(degrees / 90) % 4
        off = abs((degrees - quarter * 90 + 180) % 360 - 180)
        return quarter if off <= COMPASS_TOLERANCE_DEG else None

    def orient(self, facing):
        """The compass says which way the robot faces on the mat: only the places facing that way are kept. With a
        place it believed facing another way: it no longer believes it (a turn went wrong, or an adult put it back
        turned)."""
        if self.frame:
            kept = [st for st in self.frame["candidates"] if (self.facing + st[2]) % 4 == facing]
            if kept:
                self.frame["candidates"] = kept
                if len(kept) == 1:
                    self.settle(kept[0], "the compass")
        elif self.facing != facing:
            self.unsettle(None, facing)

    def localize(self, dist):
        """A reading of the distance sensor, standing still: rules out the places it could not come from. One place left:
        the Cartographer knows where the robot is. A reading far beyond the border from the place it believed: it no
        longer believes it."""
        if self.frame:
            kept = [st for st in self.frame["candidates"]
                    if self.possible(*self.to_mat(st, self.col, self.row, self.facing), dist)]
            if kept:  # none left: a reading that fits no place (a misreading): ignored
                self.frame["candidates"] = kept
                if len(kept) == 1:
                    self.settle(kept[0], ("the compass, " if self.rules.get("mat_north_up", True) else "")
                                + "the distance sensor and the border")
        elif dist > self.way_to_border(self.col, self.row, self.facing) + DOUBT_CM:
            self.unsettle(dist)

    def stepped(self):
        """A move that went: the places from which it would have left the mat are ruled out."""
        if self.frame:
            kept = [st for st in self.frame["candidates"] if self.on_mat(*self.to_mat(st, self.col, self.row)[:2])]
            if kept:
                self.frame["candidates"] = kept
                if len(kept) == 1:
                    self.settle(kept[0], "the border")

    def landmark(self, label, x, y, u):
        """A landmark of the rules (a card the adults put on a known cell) seen at (x, y) of the relative frame."""
        cell = (self.rules.get("landmarks") or {}).get(label)
        if not self.frame or not cell:
            return
        lc, lr = cell_xy(cell)
        size = self.mat["cell_cm"]
        kept = []
        for st in self.frame["candidates"]:
            mx, my = self.point_to_mat(st, x, y)
            if math.hypot(mx - (lc + 0.5) * size, my - (lr + 0.5) * size) <= u + size / 2:
                kept.append(st)
        if kept:
            self.frame["candidates"] = kept
            if len(kept) == 1:
                self.settle(kept[0], f"the landmark {label} in {cell}")

    def settle(self, start, how):
        self.rebase(lambda c, r, f=0: self.to_mat(start, c, r, f), lambda x, y: self.point_to_mat(start, x, y), None)
        self.notices.append(f"the Cartographer now knows where the robot is on the mat: {cell_name(self.col, self.row)}, "
                            f"facing {FACING[self.facing]} at that moment (found with {how}); the cards seen before are "
                            "now on the mat")

    def unsettle(self, dist, facing=None):
        """No longer believe the place on the mat: count from here, every place still possible kept (with the reading,
        and the way the compass says it faces)."""
        here = [self.col, self.row, self.facing]
        places = [[c, r, f] for c in range(self.mat["cols"]) for r in range(self.mat["rows"]) for f in range(4)
                  if (dist is None or self.possible(c, r, f, dist)) and (facing is None or f == facing)]
        self.rebase(lambda c, r, f=0: self.from_mat(here, c, r, f), lambda x, y: self.point_from_mat(here, x, y),
                    {"candidates": places})
        why = ("the distance sensor reads farther than the border from where it believed it (the wheels may have "
               "slipped)" if dist is not None else f"the compass says it faces {FACING[facing]}, not {FACING[here[2]]}")
        self.notices.append(f"the Cartographer no longer knows where the robot is: {why}; it counts from here again, "
                            "and looks for its place")

    def rebase(self, pose, point, frame):
        """Everything the Cartographer keeps, moved from its frame to another (the mat, or relative to a start)."""
        old_frame = self.frame
        self.frame = frame

        def cell(name):
            c, r, _ = pose(*cell_xy(name))
            return cell_name(c, r) if 0 <= c < self.cols and 0 <= r < self.rows else None

        def cells(d):
            out = {}
            for k, v in d.items():
                if cell(k):
                    out[cell(k)] = v
            return out

        def line(key):
            kind, _, value = key.partition(":")
            if kind == "cell":
                return "cell:" + (cell(value) or "")
            a = (0, int(value)) if kind == "row" else (int(value), 0)
            b = (1, int(value)) if kind == "row" else (int(value), 1)
            (ac, ar, _), (bc, br, _) = pose(*a), pose(*b)
            return f"row:{ar}" if ar == br else f"col:{ac}"

        self.col, self.row, self.facing = pose(self.col, self.row, self.facing)
        self.poses = {k: list(pose(*v)) for k, v in self.poses.items()}
        self.back = list(pose(*self.back)) if self.back else None
        self.way = [list(pose(c, r)[:2]) for c, r in self.way] if self.way else None
        self.free, self.looked, self.placed, self.blocked = (cells(self.free), cells(self.looked), cells(self.placed),
                                                             cells(self.blocked))
        self.lanes = {line(k): v for k, v in self.lanes.items()}
        if self.ball and self.ball.get("from"):
            b = dict(self.ball, **{"from": list(pose(*self.ball["from"]))})
            if b.get("cell"):
                b["cell"] = cell(b["cell"])
                b["could_also_be"] = [cell(x) for x in b.get("could_also_be", []) if cell(x)]
            if b.get("moving"):
                b["moving"] = FACING[pose(0, 0, FACING.index(b["moving"]))[2]]
            b["seen_from"] = self.pose_text(*b["from"])
            self.ball = b
        elif self.ball:
            self.ball = None  # a sighting with no place to move: forgotten (a ball is seen again within seconds)
        seen = []
        for entry in self.seen:
            e = dict(entry)
            e["x"], e["y"] = (round(v, 1) for v in point(entry["x"], entry["y"]))
            if entry.get("from"):
                e["from"] = list(pose(*entry["from"]))
                e["seen_from"] = self.pose_text(*e["from"])
            e["cell"], e["could_also_be"] = self.cells_near(e["x"], e["y"], e["u"])
            seen.append(e)
        self.seen = seen
        if old_frame is None or frame is None:
            self.covered_said = False

    def pose_text(self, col, row, facing):
        if not self.frame:
            return f"{cell_name(col, row)} facing {FACING[facing]}"
        oc, orow = self.origin()
        ahead, right = orow - row, col - oc
        parts = []
        if ahead:
            parts.append(f"{abs(ahead)} cell{'s' if abs(ahead) > 1 else ''} {'ahead of' if ahead > 0 else 'behind'}")
        if right:
            parts.append(f"{abs(right)} cell{'s' if abs(right) > 1 else ''} to the {'right' if right > 0 else 'left'} of")
        where = " and ".join(parts) + " where it started" if parts else "where it started"
        return f"{where}, facing {TURNED[facing]}"

    # What the Referee passes up ---------------------------------------------------------------------------------
    def fields(self):
        """The robot's state, read through the Referee, and what it says about the terrain."""
        fields = self.referee.state()
        if fields.get("resumes", "").isdigit():
            self.presses = int(fields["resumes"])
        if self.sure and "pose" in fields:
            # Where the robot is at this pose number: lets a later camera event be put on its cell.
            self.poses[fields["pose"]] = [self.col, self.row, self.facing]
            for old in sorted(self.poses, key=int)[:-100]:
                del self.poses[old]
        if self.sure and fields.get("moving") == "0" and fields.get("wait_adult") == "0":
            facing = self.compass(fields)
            if facing is not None:
                self.orient(facing)
        if (self.sure and fields.get("moving") == "0" and fields.get("wait_adult") == "0"
                and fields.get("estop") == "0" and fields.get("dist", "none") != "none"):
            self.localize(int(fields["dist"]))
            self.note_free(int(fields["dist"]))
        if self.sure and fields.get("looked") == "1":
            self.note_looked(None if fields.get("dist", "none") == "none" else int(fields["dist"]))
        return fields

    def new_events(self):
        """What happened to the robot since this caller last heard, in words, with where on the mat."""
        entries, restarted, lost = self.referee.events()
        if restarted:
            self.poses = {}
        events = []
        if lost:
            events.append({"event": "some older events were lost (the robot keeps only the last ones)"})
        for age, kind, reason in entries:
            if kind == "BALL":
                text = self.saw_ball(reason, age)
            elif kind in ("SEEN", "GONE"):
                text = self.saw(kind, reason, age)
                if text is None:
                    continue
            else:
                text = EVENTS.get(kind, kind + (":" + reason if reason else ""))
                if kind == "STOP":
                    text += REASONS.get(reason, reason)
            # "seconds_ago" alone was read by a real model (DeepSeek v4 Flash behind Hermes, 2026-11-03) as a time
            # since the start of a wait: the name says what it counts from.
            events.append({"seconds_before_this_answer": round(age / 1000, 1), "event": text})
        return events

    def saw(self, kind, reason, age_ms):
        """A camera event (SEEN or GONE): where the card is, from the pose the robot was at, the angle and the
        distance the camera gave. Returns the text for the caller, or None (GONE only updates the record)."""
        label, angle, cm, pose = reason.rsplit(":", 3)
        angle, cm = int(angle), int(cm)
        where = self.poses.get(pose)
        means = self.cards.get(label, "unknown card")
        side = "straight ahead" if abs(angle) < 5 else f"{abs(angle)} degrees to the {'right' if angle > 0 else 'left'}"
        if where is None:
            if kind == "GONE":
                return None
            return EVENTS["SEEN"] + f"{label} ({means}), about {cm} cm away, {side}; where: not known (the robot's position was not known)"
        x, y = self.place(where, angle, cm)
        if kind == "SEEN":
            self.landmark(label, x, y, max(CAMERA_MIN_ERROR_CM, CAMERA_ERROR * cm + CAMERA_MIN_ERROR_CM))
        u = max(CAMERA_MIN_ERROR_CM, CAMERA_ERROR * cm + CAMERA_MIN_ERROR_CM)  # grows with the distance
        if kind == "GONE":
            self.seen = [s for s in self.seen if not (s["card"] == label and math.hypot(s["x"] - x, s["y"] - y) < max(s["u"], u))]
            return None
        cell, others = self.cells_near(x, y, u)
        at = (datetime.datetime.now() - datetime.timedelta(milliseconds=age_ms)).strftime("%H:%M:%S")
        if self.frame is None and kind == "SEEN" and where is not None and self.poses.get(pose) != where:
            where = self.poses.get(pose) or where  # the landmark just told the Cartographer where it is: on the mat now
            x, y = self.place(where, angle, cm)
            cell, others = self.cells_near(x, y, u)
        entry = {"card": label, "means": means, "cell": cell, "could_also_be": others, "x": round(x, 1),
                 "y": round(y, 1), "u": round(u, 1), "seen_from": self.pose_text(*where), "from": list(where),
                 "distance_cm": cm, "at": at}
        # The same card seen again: keep the most precise estimate (seen from closer). The larger uncertainty, not
        # their sum: two like cards side by side (dry grass in C2 and D2) stay two.
        if not others and not self.frame:
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
        if self.frame:
            where_text = "where on the mat: not known yet (the Cartographer does not know yet where the robot is)"
        # Where it was seen from: the event can reach the caller after the next move (a model took a card seen from
        # C2 for one seen from D2, 2026-11-05).
        seen_from = self.pose_text(*where)
        return (EVENTS["SEEN"] + f"{label} ({means}), seen from {seen_from}, about {cm} cm away, {side}: "
                f"{where_text}, as the Cartographer counts the moves")

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
        seen_from = self.pose_text(*where)
        self.ball = {"cell": cell, "could_also_be": others, "moving": going[9:] or None, "how": how,
                     "seen_from": seen_from, "from": list(where), "at": at, "t": time.time() - age_ms / 1000}
        if self.frame:
            in_cell = "where on the mat: not known yet"
            going = ""
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
        cell_cm, cols, rows = self.mat["cell_cm"], self.cols, self.rows
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

    def note_free(self, dist):
        """The distance sensor, standing still, sees nothing for `dist` cm in front: the cells the robot could slide
        into on that line, with its stopping margin, are free (for boxes; a ball moves)."""
        dc, dr = FACING_STEP[self.facing]
        now = time.time()
        k = 1
        while True:
            col, row = self.col + dc * k, self.row + dr * k
            if not (0 <= col < self.cols and 0 <= row < self.rows):
                break
            if k * self.mat["cell_cm"] + STOP_MARGIN_CM <= dist:
                self.free[cell_name(col, row)] = now
                self.blocked.pop(cell_name(col, row), None)
            else:
                # Something is there, or behind it the sensor cannot tell: an older "free" no longer holds.
                self.free.pop(cell_name(col, row), None)
            k += 1

    def note_looked(self, dist):
        """The robot stands still and the camera's view is believed: a cell whose centre is in the camera's cone
        counts as looked at, unless the line to it crosses a cell where the distance sensor met something (a box would
        hide it), or it is straight ahead beyond what the sensor met. What met the sensor is on a cell: something
        stands there, until the sensor sees it free. A box the sensor never met can still hide a cell (seen_cards says
        so). Replayed on nine runs of the absent mission (2026-11-06): one cell in nine runs counted although a box hid
        it; asking every crossed cell to have been seen free instead left 38 cells to look at that the camera had seen,
        against 21 with this rule, and also counted one hidden cell. Tried on 2026-11-06 and 2026-11-07: a cell counted
        only when every cell on the line is known free; the notice "every cell" then never came, the absent card 0 out of
        9 (facilitator/workshops/measures/2026-11-07-robot-absent-strict/); back to this rule, decision of Sébastien on
        2026-11-07."""
        now = time.time()
        cell_cm = self.mat["cell_cm"]
        self.went(self.col, self.row)
        heading = self.facing * 90
        hit = None if dist is None else dist + SENSOR_FROM_CENTRE_CM  # from the robot's centre
        if hit is not None:
            dc, dr = FACING_STEP[self.facing]
            k = round(hit / cell_cm)
            col, row = self.col + dc * k, self.row + dr * k
            if k >= 1 and 0 <= col < self.cols and 0 <= row < self.rows:
                self.blocked.setdefault(cell_name(col, row), [now, now])[1] = now
        for col in range(self.cols):
            for row in range(self.rows):
                dx, dy = (col - self.col) * cell_cm, (row - self.row) * cell_cm
                d = math.hypot(dx, dy)
                if d == 0 or d > CAMERA_RANGE_CM:
                    continue
                angle = (math.degrees(math.atan2(dx, -dy)) - heading + 540) % 360 - 180
                if abs(angle) > CAMERA_HALF_ANGLE_DEG:
                    continue
                if abs(angle) < 1 and hit is not None and d - cell_cm / 2 > hit:
                    continue  # straight ahead, behind what the sensor met
                if not any(c in self.blocked for c in self.between((self.col, self.row), (col, row))):
                    self.looked[cell_name(col, row)] = now

    def between(self, a, b):
        """The cells the straight line from the centre of cell a to the centre of cell b crosses, a and b left out."""
        steps = 4 * max(abs(b[0] - a[0]), abs(b[1] - a[1]))
        out = set()
        for i in range(1, steps):
            x = a[0] + 0.5 + (b[0] - a[0]) * i / steps
            y = a[1] + 0.5 + (b[1] - a[1]) * i / steps
            for col in {math.floor(x - 1e-6), math.floor(x + 1e-6)}:
                for row in {math.floor(y - 1e-6), math.floor(y + 1e-6)}:
                    out.add((col, row))  # on a corner: both sides
        return {cell_name(*c) for c in out if c != tuple(a) and c != tuple(b)}

    def went(self, col, row):
        """The robot stood on this cell, or went through it: nothing stands there, no card is there."""
        now = time.time()
        self.looked[cell_name(col, row)] = now
        self.blocked.pop(cell_name(col, row), None)

    def coverage(self):
        """What the camera has looked at: a count, and the cells it has not looked at yet. A cell where something
        stands (met by the distance sensor at two moments STANDS_FOR_S apart, not seen free since) is left out: there
        is nothing there for the camera to look at. The note gives no advice: "a last look at a cell next to a box
        costs little" sent a model round a rock for 25 calls, sure the card was under it (DeepSeek v4 Flash behind
        Hermes, 2026-11-06). A box the distance sensor never met can still hide a cell: the Cartographer cannot know
        it, and the caller is not asked to guess."""
        if self.frame:
            return {"cells_looked_at": None, "cells_on_the_mat": self.mat["cols"] * self.mat["rows"],
                    "not_looked_at_yet": None, "something_stands_there": [],
                    "note": "Not counted while the Cartographer does not know where the robot is on the mat."}
        cells = [cell_name(c, r) for r in range(self.mat["rows"]) for c in range(self.mat["cols"])]
        stands = [c for c in cells if c in self.blocked and self.blocked[c][1] - self.blocked[c][0] >= STANDS_FOR_S]
        left = [c for c in cells if c not in self.looked and c not in stands]
        return {"cells_looked_at": len(cells) - len(left) - len(stands), "cells_on_the_mat": len(cells),
                "not_looked_at_yet": left, "something_stands_there": stands,
                "note": "A cell counts once the robot stood still with it in the camera's cone, or stood on it; a "
                        "cell where something stands has nothing to look at. Cards do not move: a cell looked at "
                        "stays looked at. When no cell is left, a card that is not in cards_seen is not on the mat."}

    # The Referee's questions about the terrain ----------------------------------------------------------------
    def card_cells(self, caution=False):
        """The cells where the Cartographer placed a card the camera recognised, when it is sure of the cell (a card
        seen from far could be on a neighbour: it would then block the cell in front of it), and the cells of sure
        cards that left the camera's view in the last LANE_FOR_S (hidden by a box, they are still there). caution:
        every probable cell, sure or not (for a slide sideways, which no camera watches)."""
        now = time.time()
        sure = {s["cell"] for s in self.seen if caution or not s.get("could_also_be")}
        return sure | {c for c, t in self.placed.items() if now - t < LANE_FOR_S}

    def on_lane(self, cell):
        """A cell on the lane of a ball seen in the last LANE_FOR_S (its row or column, or the cell)."""
        col, row = cell
        now = time.time()
        for key, when in self.lanes.items():
            kind, _, value = key.partition(":")
            if now - when > LANE_FOR_S:
                continue
            if (kind == "row" and int(value) == row) or (kind == "col" and int(value) == col) \
                    or (kind == "cell" and value == cell_name(col, row)):
                return True
        return False

    def seen_free_s_ago(self, cell):
        return time.time() - self.free.get(cell_name(*cell), 0)

    def ball_sighting(self):
        """The last ball placed on the mat: the cells it could be on, how long ago, whether it came closer."""
        ball = self.ball or {}
        if self.frame and ball.get("t"):
            return {"cells": [], "s_ago": time.time() - ball["t"], "closer": ball.get("how", "").startswith("getting")}
        if not ball.get("cell"):
            return None
        return {"cells": [cell_xy(n) for n in [ball["cell"]] + ball.get("could_also_be", [])],
                "s_ago": time.time() - ball.get("t", 0), "closer": ball.get("how", "").startswith("getting closer")}

    def moving(self, cells, counted):
        """Just before a move is sent: written to disk as if it were interrupted (position unknown, put back on its
        start cell), so that a stack killed during the move leaves the truth behind. `cells`: the cells of a slide;
        the robot's own count (STATE slid=) says which one it reached."""
        start = (self.col, self.row, self.facing)
        self.sure, self.back, self.way = False, start, [list(start[:2])] + [list(c) for c in cells]
        self.back_fixed = False
        self.resumes_at_stop = self.presses
        self.save()

    def reached(self, cell, pose):
        """A slide reached one more cell (EVT CELL)."""
        before = cell_name(self.col, self.row)
        self.col, self.row = cell
        self.free[before] = self.free[cell_name(self.col, self.row)] = time.time()  # it went through them
        self.went(self.col, self.row)
        self.poses[pose] = [self.col, self.row, self.facing]
        if not self.on_lane((self.col, self.row)):
            self.back = (self.col, self.row, self.facing)  # an adult would put it back here

    # Where the robot is ------------------------------------------------------------------------------------------
    def check_resumed(self):
        """After an interrupted move: has an adult pressed Resume since? The robot counts the presses; the
        Cartographer compares with the count it saw when the move stopped."""
        fields = self.fields()
        presses = int(fields.get("resumes", 0))
        released = fields.get("wait_adult") == "0" and fields.get("estop") == "0"
        if self.resumes_at_stop is None:
            # The count was not known when the move stopped (the cable was loose). A robot that no longer waits,
            # remembers its last stop and has seen Resume was released by an adult; a robot that restarted remembers
            # neither.
            if released and presses > 0 and fields.get("last_stop", "none") != "none":
                resumed = True
            else:
                self.resumes_at_stop, resumed = presses, False
        elif presses < self.resumes_at_stop:
            self.resumes_at_stop, resumed = presses, False  # the robot restarted: count from now
        else:
            resumed = presses > self.resumes_at_stop and released
        if not resumed and not self.back_fixed:
            # Before the adult acts: the instruction names the cell the robot itself says it reached (a stack killed
            # during a move, or a lost EVT CELL). Once given, it never changes (the adult may already be acting on it),
            # and never after Resume.
            self.reached_by_count(fields)
        if resumed:
            self.col, self.row, self.facing = self.back
            self.sure, self.back, self.way, self.resumes_at_stop = True, None, None, None

    def ended(self):
        """The move ended on a cell the Cartographer knows (done, or refused without moving)."""
        self.sure, self.back, self.way, self.resumes_at_stop = True, None, None, None
        self.save()

    def reached_by_count(self, fields):
        """Where an interrupted slide left the robot: the last cell it reached, by its own count."""
        if self.back and self.way and fields.get("slid", "").isdigit():
            k = min(int(fields["slid"]), len(self.way) - 1)
            while k > 0 and self.on_lane(self.way[k]):
                k -= 1  # never put it back on a ball's lane: on the last cell before it
            col, row = self.way[k]
            self.back = (col, row, self.back[2])
            self.back_fixed = True

    def interrupted(self, why):
        self.sure = False
        try:
            fields = self.fields()
            self.resumes_at_stop = int(fields.get("resumes", 0))
            self.reached_by_count(fields)  # an EVT CELL may have been lost on a loose cable
        except (RobotNotConnected, ValueError):
            self.resumes_at_stop = None  # the cable is loose: counted when the robot answers again
        return {"result": "stopped", "why": REASONS.get(why, why), "position_known": False,
                "what_to_do": self.adult_instruction()}

    def adult_instruction(self):
        if self.frame:
            return ("Stop and ask an adult to put the robot back on the cell where its last move started, facing the "
                    "way it faced then, then to press Resume on the robot. Do not try to move it before.")
        col, row, facing = self.back if self.back else (self.col, self.row, self.facing)
        return (f"Stop and ask an adult to put the robot back on cell {cell_name(col, row)}, facing "
                f"{FACING[facing]}, then to press Resume on the robot. Do not try to move it before.")

    # The Referee's answers, in words ----------------------------------------------------------------------------
    def refused(self, why, **extra):
        return dict({"result": "refused", "why": REASONS.get(why, why)}, **extra)

    def say(self, answer, direction=None, step=False):
        """A refusal of the Referee, in words; the end of a move on the map."""
        if answer.get("result") != "refused":
            return answer
        rule, cell = answer["rule"], answer.get("cell")
        recent = self.rules["ball_seen_recently_s"]
        facing = FACING[self.facing]
        if rule == "robot":
            if answer.get("moved") is False:
                self.ended()  # a move the robot refused: it never left its cell
            extra = {"waited": "the robot waited for the way to clear, then gave up"} if answer.get("waited") else {}
            return self.refused(answer["why"], **extra)
        if rule == "no_moves":
            return self.refused(f"no moves left in this session ({answer['max']})")
        if rule == "off_mat":
            return self.refused("the next cell is outside the mat")
        if rule == "card":
            what = "stop here if it is the one you look for, or go round it"
            if step:
                return self.refused(f"a card is on the next cell ({cell}): the robot never drives onto a card", what_to_do=what)
            return self.refused(f"a card was seen on the next cell that way ({cell}): the robot never drives onto a card",
                                what_to_do=what)
        if rule == "ball_forward":
            return self.refused(f"a ball was seen near {cell} less than {recent} s ago",
                                what_to_do="watch it go away (wait_for_event), then try again")
        if rule == "blind_lane":
            return self.refused(f"{cell} is on the lane of a ball, and the robot cannot see {direction} while it faces "
                                f"{facing}: it never slides into a ball's lane sideways or backwards",
                                what_to_do=f"face {direction}, watch the ball go away, then cross forwards")
        if rule == "blind_ball":
            return self.refused(f"a ball was seen near {cell} less than {recent} s ago, and the robot cannot see "
                                f"{direction} while it faces {facing}",
                                what_to_do="watch it go away (wait_for_event, face it), then try again")
        if rule == "blind_not_seen":
            return self.refused(f"the robot only sees in front of it ({facing}): it slides {direction} only into cells "
                                f"its distance sensor saw free in the last {answer['free_for_min']} minutes, and {cell} "
                                "is not one", what_to_do=f"face {direction} (one move) or look_around, then move again")
        if rule == "position_unknown":
            return self.refused("the Cartographer does not know yet where the robot is on the mat, so north, east, south "
                                "and west mean nothing yet: only quarter turns, look_around, and step_forward where the "
                                "distance sensor sees the way free", what_to_do="look_around: the distance sensor sees "
                                "the border of the mat, and the camera the landmarks")
        if rule == "unknown_way":
            return self.refused("the Cartographer does not know yet where the robot is on the mat: it goes one cell "
                                "forwards only where its distance sensor sees the way free beyond that cell",
                                what_to_do="turn towards a free way (look_around tells which), then step_forward")
        if rule == "ball_unknown":
            return self.refused(f"a ball was seen less than {recent} s ago, and the Cartographer does not know yet where "
                                "it is on the mat", what_to_do="watch it go away (wait_for_event), then try again")
        if rule == "one_move_at_a_time":
            return self.refused("another move is still running: send one move at a time, and read its answer before "
                                "the next")
        if rule == "another_command":
            return self.refused("another command is running (an agent, a terminal or the page): wait for it to end")
        return self.refused(rule)

    # The Pilot's commands: the mat -> the robot -------------------------------------------------------------
    def robot_state(self):
        fields = self.fields()
        card = fields.get("card", "none")
        known = self.sure and not self.frame
        places = self.places()
        state = {
            "cell": cell_name(self.col, self.row) if known else None,
            "facing": FACING[self.facing] if known else None,
            "position_known": known,
            "distance_ahead_cm": None if fields.get("dist") == "none" else int(fields["dist"]),
            "card_ahead": None if card == "none" else {"card": card, "means": self.cards.get(card, "unknown card")},
            "battery_percent": int(fields.get("battery", 0)),
            "emergency_stop": fields.get("estop") == "1",
            "waiting_for_adult": fields.get("wait_adult") == "1" or not self.sure,
            "bumper_touched": fields.get("bumper") == "1",
            "warning_light_on": fields.get("warning") == "1",
            "ball_last_seen": self.ball,
            "on_a_ball_lane": self.on_lane((self.col, self.row)) if known else None,
            "cells_not_looked_at_yet": None if self.frame else len(self.coverage()["not_looked_at_yet"]),
            "last_stop": "none" if fields.get("last_stop", "none") == "none"
                         else REASONS.get(fields["last_stop"], fields["last_stop"]),
            "moves_left": self.referee.moves_left(),
            "mat": f"{self.mat['cols']} columns (A to {cell_name(self.mat['cols'] - 1, 0)[0]}), "
                   f"{self.mat['rows']} rows (1 at the top to {self.mat['rows']})",
        }
        if self.frame and self.sure:
            state["position"] = "probable" if len(places) <= 6 else "unknown"
            state["since_start"] = self.pose_text(self.col, self.row, self.facing)
            if len(places) <= 6:
                state["could_be"] = [f"{cell_name(c, r)} facing {FACING[f]}" for c, r, f in places]
            state["how_to_find_it"] = ("look_around (the distance sensor sees the border of the mat; a landmark of the "
                                       "rules, once the camera sees it, settles it), then step_forward where the way is "
                                       "free")
        return state

    def seen_cards(self):
        keys = ("card", "means", "cell", "could_also_be", "distance_cm", "seen_from", "at")
        cards = [{k: s[k] for k in keys} for s in sorted(self.seen, key=lambda s: s["at"])]
        if self.frame:
            cards = [dict(c, cell=None, could_also_be=[], where="not known yet: the Cartographer does not know yet where "
                                                              "the robot is on the mat") for c in cards]
        return {"cards_seen": cards, "camera_coverage": self.coverage(),
                "note": "The camera sees up to 120 cm ahead, wider when farther, and guesses the distance: the "
                        "farther the card, the less sure its cell (could_also_be). Seen again from closer, the "
                        "estimate gets better. The robot's own cell comes from counting its moves, not from a measure."}

    def ahead(self):
        dc, dr = FACING_STEP[self.facing]
        col, row = self.col + dc, self.row + dr
        return (col, row) if 0 <= col < self.cols and 0 <= row < self.rows else None

    def turned(self, answer, kind):
        """A quarter turn of the Referee, on the map."""
        if answer["result"] == "done":
            self.facing = (self.facing + (1 if kind == "R" else -1)) % 4
            self.ended()
            return {"result": "done", "state": self.robot_state()}
        if answer["result"] == "stopped":
            return self.interrupted(answer["why"])
        return self.say(answer)

    def step_forward(self):
        if not self.sure:
            return self.refused("wait_adult", what_to_do=self.adult_instruction())
        target = self.ahead()
        fields = self.fields()
        card_ahead = fields.get("card", "none") != "none"
        start = (self.col, self.row)
        if self.frame:
            # Where it is on the mat is not known: one cell forwards only where the sensor sees the way free beyond it.
            dist = None if fields.get("dist", "none") == "none" else int(fields["dist"])
            answer = self.referee.step_blind(self, target, card_ahead, dist, self.mat["cell_cm"] + STOP_MARGIN_CM)
        else:
            answer = self.referee.step(self, target, card_ahead)
        if answer["result"] == "done":
            self.free[cell_name(*start)] = self.free[cell_name(*target)] = time.time()
            self.col, self.row = target
            self.went(self.col, self.row)
            self.stepped()
            self.ended()
            return {"result": "done", "state": self.robot_state()}
        if answer["result"] == "stopped":
            return self.interrupted(answer["why"])
        return self.say(answer, step=True)

    def turn(self, direction, counted=True):
        if not self.sure:
            return self.refused("wait_adult", what_to_do=self.adult_instruction())
        kind = {"left": "L", "right": "R"}[direction]
        return self.turned(self.referee.turn(self, kind, counted), kind)

    def move(self, direction, cells, stop_if):
        """Slide towards a side of the mat without turning: the Cartographer turns the direction of the mat into a
        direction of the robot (forwards, backwards, to its left, to its right), from the way it faces."""
        if not self.sure:
            return self.refused("wait_adult", what_to_do=self.adult_instruction())
        if self.frame:
            return self.say({"result": "refused", "rule": "position_unknown"})
        side = FACING.index(direction)
        kind = "FRBL"[(side - self.facing) % 4]
        dc, dr = FACING_STEP[side]
        way = []
        for k in range(1, cells + 1):
            col, row = self.col + dc * k, self.row + dr * k
            if not (0 <= col < self.mat["cols"] and 0 <= row < self.mat["rows"]):
                break
            way.append((col, row))
        answer = self.referee.slide(self, kind, way, [STOP_IF[c] for c in stop_if], cells, hold=self.hold_wanted)
        if answer["result"] == "stopped":
            result = self.interrupted(answer["why"])
            result["cells_moved"] = answer.get("cells_done", 0)
            return result
        if answer["result"] != "done":
            return self.say(answer, direction=direction)
        why = answer["why"]
        reason = WHY_DONE.get(why, why)
        if why == "all" and answer.get("card_stop"):
            reason = f"it stopped before {answer['card_stop']}, where the camera saw a card: it never drives onto a card"
        elif why == "all" and answer.get("edge"):
            n = answer["cells_done"]
            reason = f"the mat ends after {n} cell{'s' if n > 1 else ''} that way"
        self.ended()
        return {"result": "done", "cells_moved": answer["cells_done"], "cells_asked": cells, "stopped_because": reason,
                "state": self.robot_state()}

    def look_around(self):
        """Four quarter turns to the right, a pause after each one for the camera: one move in all."""
        looked = []
        for n in range(4):
            if n > 0 and self.hold_wanted():
                return {"result": "paused", "why": "the mission was paused or stopped: the robot stopped turning",
                        "looked_before_it_stopped": looked, "state": self.robot_state()}
            result = self.turn("right", counted=n == 0)
            if result.get("result") != "done":
                result["looked_before_it_stopped"] = looked
                return result
            time.sleep(0.5)  # the camera names a card after 200 ms standing still
            self.fields()  # what the camera looked at, facing this way (seen_cards)
            looked.append({"facing": FACING[self.facing], "card_ahead": result["state"]["card_ahead"],
                           "distance_ahead_cm": result["state"]["distance_ahead_cm"],
                           "what_the_camera_saw": [e["event"] for e in self.new_events()]})
        return {"result": "done", "looked": looked, "state": self.robot_state()}

    def face(self, direction):
        """Turn on the spot until the robot faces north, east, south or west: one move, or none."""
        if not self.sure:
            return self.refused("wait_adult", what_to_do=self.adult_instruction())
        if self.frame:
            return self.say({"result": "refused", "rule": "position_unknown"})
        turns = (FACING.index(direction) - self.facing) % 4
        result = {"result": "done", "state": self.robot_state()}
        for n, kind in enumerate({0: "", 1: "R", 2: "RR", 3: "L"}[turns]):
            if n > 0 and self.hold_wanted():
                return {"result": "paused", "why": "the mission was paused or stopped: the robot stopped turning",
                        "state": self.robot_state()}
            result = self.turn({"R": "right", "L": "left"}[kind], counted=n == 0)
            if result.get("result") != "done":
                return result
        return result

    def set_light(self, color):
        return self.say(self.referee.light(color))

    # One call ---------------------------------------------------------------------------------------------------
    def call(self, action, is_move=False):
        """One call of the Pilot, through the Referee's door: up-to-date map before (the events first, so that the
        action answers from an up-to-date record), what happened during it after, and the map on disk whatever
        happened."""
        def body():
            self.load()
            try:
                self.referee.connect()
                if not self.sure:
                    self.check_resumed()
                self.fields()  # notes the pose number of where the robot is now
                events = self.new_events()
                result = action()
                try:
                    events += self.new_events()
                except RobotNotConnected:
                    pass  # the robot does not answer now: the events wait for the next call
                events += [{"event": n} for n in self.notices]
                self.notices = []
                left = self.coverage()["not_looked_at_yet"]
                if left is None:
                    pass  # not counted while the position on the mat is not known
                elif not left and not self.covered_said:
                    self.covered_said = True
                    names = sorted({c["card"] for c in self.seen})
                    events.append({"event": "the camera has now looked at every cell of the mat (seen_cards: "
                                   "camera_coverage): the cards on the mat are the ones it saw, "
                                   + (", ".join(names) if names else "none") + "; any other card is not on the mat"})
                elif left:
                    self.covered_said = False
                if events:
                    result["since_last_call"] = events
            except RobotNotConnected as e:
                self.referee.close()
                result = {"result": "error", "why": str(e)}
            except (ValueError, KeyError, IndexError) as e:  # an answer of the robot that makes no sense
                self.referee.close()
                result = {"result": "error", "why": f"the robot's answer could not be read ({type(e).__name__})"}
            finally:
                self.save()
            return result

        result = self.referee.door(is_move, body)
        return self.say(result) if result.get("rule") else result

    def watch_once(self):
        """The events the caller has not heard yet, read briefly (for a channel), or []."""
        result = self.call(lambda: {})
        return [] if result.get("result") in ("refused", "error") else result.get("since_last_call", [])

    def wait_for(self, seconds):
        """Watch the robot for up to `seconds`, without holding the cable in between. Answers at the first event that
        happens during the wait. Events from before the wait (the robot kept them) come with the answer, marked, but
        do not end it: a caller that waits for a Resume or a ball would otherwise get them back at once, again and
        again (seen on 5 and 6 November)."""
        start = time.monotonic()
        end = start + seconds
        earlier = []  # [time of the poll that read them, event]: events from before the wait

        def kept():
            # Their age as of this answer, not as of the poll that read them.
            now = time.monotonic()
            return [dict(e, seconds_before_this_answer=round(e.get("seconds_before_this_answer", 0) + now - at, 1))
                    for at, e in earlier]

        while True:
            result = self.call(lambda: {})
            if result.get("result") == "error":
                if earlier:
                    result["events"] = kept()
                return result
            if result.get("result") != "refused":
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
