"""The Cartographer alone (stack/cartographer.py), probed by ci/probe_tiers.py --tier cartographer: see the head of
that file.

The generator plays the Pilot and writes the Cartographer's language, one call per line. The mat has columns A to H
and rows 1 (at the top) to 6; north is towards row 1:
    step_forward                                  one cell forwards
    move <north|east|south|west> <1-4> [<stop_if>,<stop_if>…]
                                                  slide that many cells towards a side of the mat, without turning;
                                                  stop_if: card_ahead, ball_seen, new_card
    turn <left|right>                             a quarter turn on the spot
    look_around                                   four quarter turns to the right, a look after each
    face <north|east|south|west>                  turn on the spot until the robot faces that way
    set_light <off|red|orange|green|blue>         the robot's light
    robot_state                                   what the robot measures, and where the Cartographer thinks it is
    seen_cards                                    the cards the camera saw, and the cells it has looked at
    wait_for <1-30>                               watch the robot for that many seconds
Below it, a fake Referee that answers in its closed language (done, refused with a code, stopped with a reason), over
a small world drawn for each run: where the robot really is, boxes, cards, the landmark of the rules, a compass and a
distance sensor that tell the truth (in some runs, now and then, a reading that lies), the camera's events (cards seen
from where the robot really stands, balls), stops, and an adult who puts the robot back where the answer said and
presses Resume. The rules (where the robot starts, the compass, a landmark, the adults' limits) are drawn for each
run. A fake clock: the Cartographer's waits cost no time, and what gets old (cells seen free, a ball's lane, something
that stands) gets old as it would.
"""
import collections
import json
import math
import os
import random
import re
import shutil
import sys
import tempfile
import time
from pathlib import Path

GUIDE = Path(__file__).resolve().parent.parent
STATE = Path(tempfile.mkdtemp(prefix="probe-cartographer-"))
os.environ["ROBOT_STATE_DIR"] = str(STATE)
os.environ.setdefault("ROBOT_CALLER", "probe")
sys.path.insert(0, str(GUIDE / "stack"))
import common  # noqa: E402
import cartographer as cartographer_module  # noqa: E402
from common import COLORS, FACING, FACING_STEP, cell_name, cell_xy  # noqa: E402
from link import RobotNotConnected  # noqa: E402

import probe_tiers  # noqa: E402

COLS, ROWS, CELL_CM = 8, 6, 30
CELLS = [cell_name(c, r) for r in range(ROWS) for c in range(COLS)]
MAT = {"cols": COLS, "rows": ROWS, "cell_cm": CELL_CM, "start": {"cell": "A6", "facing": "north"},
       "move_timeout_s": 12, "cards": common.load_json(GUIDE / "mat.json")["cards"]}
CARDS = sorted(MAT["cards"])
RULES = json.loads((GUIDE / "stack" / "rules.json").read_text(encoding="utf-8"))
# What the invariants hold the Cartographer to: the words of the module as it is on disk (a mutant changes its copy).
C = cartographer_module
STOP_IF = dict(C.STOP_IF)
MOVES = {"step_forward", "move", "turn", "look_around", "face", "set_light"}
LANGUAGE = __doc__[__doc__.index("The generator plays"):__doc__.index("Below it")].strip()
PERSONAS = {
    "careful": "You are the Pilot of a small robot: you send commands of the mat to its Cartographer, one call per "
               "line, in its language. Explore the mat, look for cards, read the robot's state, and do what the "
               "answers say (an adult must come after a stop). Follow the language exactly.",
    "careless": "You are a careless or hostile Pilot of a small robot. Send calls that test the Cartographer's "
                "limits: moves while the robot waits for an adult, slides towards the edge, turns and looks at the "
                "wrong time, long waits, wrong words. Keep each to one line.",
}
ERRS = ["busy", "estop", "wait_adult", "bump", "battery", "fault"]
STOPS = ["bump", "estop", "watchdog", "late", "obstacle", "sensor", "fault"]
# The refusals of the Referee whose answer gives the Pilot something to do (spec C2), and their words.
TO_DO = {"card", "ball_forward", "blind_lane", "blind_ball", "blind_not_seen", "position_unknown", "unknown_way",
         "ball_unknown"}
PHRASE = {
    "no_moves": r"no moves left: the adults allow \d+ moves per mission",
    "limit": r"no (commands|cells) left: the adults allow \d+ (commands|cells) per mission",
    "off_mat": r"the next cell is outside the mat",
    "card": r"a card (is on the next cell|was seen on the next cell that way) \((?P<cell>\w+)\): the robot never "
            r"drives onto a card",
    "ball_forward": r"a ball was seen near (?P<cell>\w+) less than \d+ s ago",
    "blind_lane": r"(?P<cell>\w+) is on the lane of a ball, and the robot cannot see \w+ while it faces \w+: .+",
    "blind_ball": r"a ball was seen near (?P<cell>\w+) less than \d+ s ago, and the robot cannot see \w+ while .+",
    "blind_not_seen": r"the robot only sees in front of it \(\w+\): it slides \w+ only into cells its distance sensor "
                      r"saw free in the last \d+ minutes, and (?P<cell>\w+) is not one",
    "position_unknown": r"the Cartographer does not know yet where the robot is on the mat, so north, .+",
    "unknown_way": r"the Cartographer does not know yet where the robot is on the mat: it goes one cell forwards .+",
    "ball_unknown": r"a ball was seen less than \d+ s ago, and the Cartographer does not know yet where it is .+",
    "one_move_at_a_time": r"another move is still running: .+",
    "another_command": r"another command is running .+",
}
ADULT = "Stop and ask an adult to put the robot back on "
NAME = re.compile(r"\b[A-Z][0-9]{1,2}\b")
STATE_KEYS = {"cell", "facing", "position_known", "distance_ahead_cm", "card_ahead", "battery_percent",
              "emergency_stop", "waiting_for_adult", "bumper_touched", "warning_light_on", "ball_last_seen",
              "on_a_ball_lane", "cells_not_looked_at_yet", "last_stop", "moves_left", "counts", "mat"}
CARD_KEYS = {"card", "means", "cell", "could_also_be", "distance_cm", "seen_from", "at"}
COVERAGE_KEYS = {"cells_looked_at", "cells_on_the_mat", "not_looked_at_yet", "something_stands_there", "note"}


class Clock:
    """The Cartographer's time module, for one run: sleeping moves the clock, nothing waits."""

    def __init__(self):
        self.t = 0.0

    def time(self): return 1.8e9 + self.t
    def monotonic(self): return self.t
    def strftime(self, *a): return time.strftime(*a)

    def sleep(self, s):
        self.t += max(0.0, s)


def on_mat(col, row):
    return 0 <= col < COLS and 0 <= row < ROWS


def bearing(frm, to, facing):
    """The angle (degrees, right positive) and the distance (cm) of the centre of cell `to` from the centre of `frm`."""
    dx, dy = (to[0] - frm[0]) * CELL_CM, (to[1] - frm[1]) * CELL_CM
    angle = (math.degrees(math.atan2(dx, -dy)) - facing * 90 + 540) % 360 - 180
    return angle, math.hypot(dx, dy)


# --- the fake below: a Referee and the world under it -------------------------------------------------------------------
class Referee:
    """A Referee that answers in its closed language over a world drawn at random: where the robot really is (it moves
    only when the answer says so), boxes the robot cannot enter, cards, a distance sensor that reads the border or a
    box, a compass, and an adult. Its answers are drawn too (refusals of every code, stops, partial slides), but never
    one the real Referee could not give (a step done off the mat, a slide through a box)."""

    def __init__(self, rng, clock, rules, start, lies, broken):
        self.rng, self.clock, self.rules, self.broken = rng, clock, rules, broken
        self.col, self.row, self.facing = start  # where the robot really is
        self.lies = lies  # this run's sensors lie now and then
        self.lied = False  # they did (or the rules gave a wrong start): the place found may be wrong
        free = [(c, r) for c in range(COLS) for r in range(ROWS) if (c, r) != (self.col, self.row)]
        rng.shuffle(free)
        self.boxes = set(free[:rng.randint(0, 3)])
        self.cards = {}
        for label in CARDS:
            if rng.random() < 0.6:
                self.cards[free.pop()] = label
        self.between = False  # stopped between two cells: an adult must put it back
        self.wait_adult, self.estop, self.fault, self.last_stop = 0, 0, "none", "none"
        self.resumes, self.pose, self.slid, self.fresh = 0, 1, 0, False
        self.instruction = None  # what the last stop asked of the adult (the probe gives it, once answered)
        self.started = self.true_start = None  # at the start of the last move: the Cartographer's place, the real one
        self.queue = []  # events the robot keeps: [time ms, kind, details]
        self.cards_said = []  # cards the camera said it saw, for GONE
        self.notes = []  # what reached the Referee, in order, for the invariants
        self.said = []  # its answers
        self.troubled = False  # an answer that could not be read, in this call
        self.last_dist = None
        self.views = []  # (col, row, facing, dist) where the camera's view was believed
        self.visited = {(self.col, self.row)}
        self.hold_rate = 0.0

    def broke(self, name, detail):
        self.broken.append({"invariant": name, "detail": detail})

    def now_ms(self):
        return int(self.clock.t * 1000)

    def event(self, kind, details=""):
        self.queue.append([self.now_ms(), kind, details])

    # The door and the questions --------------------------------------------------------------------------------
    def connect(self): pass
    def close(self): pass

    def door(self, is_move, body):
        r = self.rng.random()
        if r < 0.015:
            self.said.append(["refused", "another_command", {}])
            return {"result": "refused", "rule": "another_command"}
        if is_move and r < 0.03:
            self.said.append(["refused", "one_move_at_a_time", {}])
            return {"result": "refused", "rule": "one_move_at_a_time"}
        return body()

    def limits(self):
        return {k: v for k, v in (self.rules.get("limits") or {}).items() if isinstance(v, int)}

    def moves_left(self):
        limit = self.limits().get("moves")
        return None if limit is None else max(0, limit - common.used()["moves"])

    def hold(self):
        """hold_wanted, as the Pilot would answer it: drawn, and noted."""
        said = self.rng.random() < self.hold_rate
        self.notes.append({"kind": "hold", "said": said})
        return said

    def trouble(self):
        if self.rng.random() < 0.003:
            self.troubled = True
            raise RobotNotConnected("the robot does not answer (fake)")

    def reading(self):
        """What the distance sensor reads, facing the way the robot really faces: the border, or a box."""
        rng = self.rng
        dc, dr = FACING_STEP[self.facing]
        k = 1
        while True:
            c, r = self.col + dc * k, self.row + dr * k
            if not on_mat(c, r):
                dist = (k - 1) * CELL_CM + 15  # the fence, 10 cm beyond the edge, from the sensor, 10 cm in front
                break
            if (c, r) in self.boxes:
                dist = k * CELL_CM - 20  # a box 20 cm wide in the middle of its cell
                break
            k += 1
        dist = max(0, dist - rng.randint(0, 3))
        x = rng.random()
        if x < 0.08:
            return None
        if x < 0.14:
            return rng.randint(5, max(5, dist))  # a hand, a ball: closer, never farther
        if self.lies and x < 0.17:
            self.lied = True
            return dist + rng.randint(40, 150)
        return dist

    def heading(self):
        rng = self.rng
        if not self.rules.get("mat_north_up", True):
            return str(rng.randint(0, 359))  # the mat is not laid north up: the compass says nothing of it
        x = rng.random()
        if x < 0.08:
            return "none"
        if x < 0.13:
            return str((self.facing * 90 + rng.choice([-1, 1]) * rng.randint(40, 50)) % 360)  # not believed
        if self.lies and x < 0.16:
            self.lied = True
            return str(((self.facing + rng.choice([1, 2, 3])) * 90 + rng.randint(-10, 10)) % 360)  # a magnet
        return str((self.facing * 90 + rng.randint(-10, 10)) % 360)

    def in_cone(self, cell, half=28, far=120):
        angle, d = bearing((self.col, self.row), cell, self.facing)
        return (angle, d) if 0 < d <= far and abs(angle) <= half and not self.hidden(cell) else None

    def hidden(self, cell):
        for i in range(1, 10):
            x = self.col + 0.5 + (cell[0] - self.col) * i / 10
            y = self.row + 0.5 + (cell[1] - self.row) * i / 10
            c = (math.floor(x), math.floor(y))
            if c in self.boxes and c != tuple(cell):
                return True
        return False

    def state(self):
        self.adult()
        self.trouble()
        rng = self.rng
        dist = self.reading()
        looked = 0 if self.between or self.fresh else 1
        self.fresh = False
        ahead = (self.col + FACING_STEP[self.facing][0], self.row + FACING_STEP[self.facing][1])
        card = self.cards.get(ahead, "none") if not self.between else "none"
        points = []
        if looked:
            for box in sorted(self.boxes):
                seen = self.in_cone(box, half=30, far=100)
                if seen:
                    a, d = seen
                    points += ["%d:%d" % (round(a + s + rng.uniform(-2, 2)), round((d - 10) * rng.uniform(0.9, 1.1)))
                               for s in (-3, 0, 3)]
            self.views.append((self.col, self.row, self.facing, dist))
        self.last_dist = dist
        return {"dist": "none" if dist is None else str(dist), "card": card, "battery": str(rng.randint(20, 95)),
                "estop": str(self.estop), "wait_adult": str(self.wait_adult), "moving": "0",
                "last_stop": self.last_stop, "bumper": "1" if rng.random() < 0.03 else "0",
                "warning": "1" if rng.random() < 0.03 else "0", "resumes": str(self.resumes), "pose": str(self.pose),
                "slid": str(self.slid), "looked": str(looked), "heading": "none" if self.between else self.heading(),
                "fault": self.fault, "tick_max": "10", "boxes": ",".join(points) or "none"}

    def events(self):
        """[age ms, kind, details] each, as the real Referee gives them, with its two flags."""
        self.trouble()
        rng = self.rng
        if not self.between and rng.random() < 0.6:
            for cell, label in sorted(self.cards.items()):
                seen = self.in_cone(cell)
                if seen and rng.random() < 0.4:
                    a, d = seen
                    self.event("SEEN", "%s:%d:%d:%d" % (label, round(a + rng.uniform(-2, 2)),
                                                         round(d * rng.uniform(0.92, 1.08)), self.pose))
                    self.cards_said.append((label, round(a), round(d)))
            if rng.random() < 0.08:
                a1, c1 = rng.randint(-25, 25), rng.randint(30, 110)
                self.event("BALL", "%s:%d:%d:%d:%d:%d" % (rng.choice(["closer", "away", "still", "gone"]), a1, c1,
                                                          a1 + rng.randint(-12, 12), c1 + rng.randint(-25, 25),
                                                          self.pose))
            if self.cards_said and rng.random() < 0.03:
                label, a, d = rng.choice(self.cards_said)
                self.event("GONE", "%s:%d:%d:%d" % (label, a, d, self.pose))
        x = rng.random()
        if x < 0.02:
            self.event("BUMP")
        elif x < 0.04:
            self.event("NEAR")
        elif x < 0.043:
            self.troubled = True
            self.event("SEEN", "dry-grass:12")  # an answer that makes no sense: the Cartographer says error
        now = self.now_ms()
        out = [[now - t, kind, details] for t, kind, details in self.queue]
        self.queue = []
        restarted = rng.random() < 0.004
        lost = rng.random() < 0.01
        return out, restarted, lost

    def adult(self):
        """An adult comes now and then, once the answer said where to put the robot back: puts it there, presses
        Resume."""
        if not (self.wait_adult or self.estop) or self.instruction is None or self.rng.random() > 0.5:
            return
        m = re.search(r"back on cell ([A-Z])([0-9]+), facing (\w+)", self.instruction)
        if m:
            self.col, self.row, self.facing = cell_xy(m[1] + m[2])[0], int(m[2]) - 1, FACING.index(m[3])
        else:
            self.col, self.row, self.facing = self.true_start  # "the cell where its last move started"
        self.visited.add((self.col, self.row))
        self.between, self.wait_adult, self.estop, self.fault = False, 0, 0, "none"
        self.resumes += 1
        self.pose += 1
        self.instruction = None
        self.event("RESUMED")
        self.notes.append({"kind": "resume"})

    # The moves ------------------------------------------------------------------------------------------------
    def note(self, kind, terrain, **details):
        self.notes.append(dict(details, kind=kind, belief=(terrain.col, terrain.row, terrain.facing),
                               frame=terrain.frame is not None, waiting=bool(self.wait_adult or self.estop)))

    def refuse(self, rule, **details):
        self.said.append(["refused", rule, details])
        return dict({"result": "refused", "rule": rule}, **details)

    def over(self, counted):
        limits, used = self.limits(), common.used()
        if counted and "moves" in limits and used["moves"] >= limits["moves"]:
            return self.refuse("no_moves", max=limits["moves"])
        for counter in ("commands", "cells"):
            if counter in limits and used[counter] >= limits[counter]:
                return self.refuse("limit", counter=counter, max=limits[counter])
        return None

    def begin(self, terrain, cells, counted):
        if self.wait_adult or self.estop:
            self.broke("nothing moves until an adult pressed Resume", self.notes[-1])
        self.started = (terrain.col, terrain.row, terrain.facing)
        self.true_start = (self.col, self.row, self.facing)
        terrain.moving(cells, counted=counted)
        self.slid = 0
        common.count("commands")
        if counted:
            common.count("moves")

    def robot_refuses(self, why, counted, waited=False):
        if counted:
            common.count("moves", -1)
        self.said.append(["refused", "robot", {"why": why, "waited": waited}])
        out = {"result": "refused", "rule": "robot", "why": why, "moved": False}
        if waited:
            out["waited"] = True
        return out

    def stop(self, why, done):
        self.between, self.wait_adult, self.last_stop = True, 1, why
        if why == "estop":
            self.estop = 1
        if why == "fault":
            self.fault = "reflex"
        self.pose += 1
        self.instruction = None
        if why == "timeout":
            common.count("commands")  # the Referee's STOP
            self.last_stop = "command"
        self.event("STOP", self.last_stop)
        self.said.append(["stopped", why, {"cells_done": done}])
        return {"result": "stopped", "why": why, "cells_done": done}

    def forwards(self, terrain, target):
        rng = self.rng
        self.begin(terrain, [target], True)
        if self.wait_adult or self.estop:
            return self.robot_refuses("wait_adult", True)
        r = rng.random()
        if r < 0.05:
            return self.robot_refuses(rng.choice(ERRS), True)
        dc, dr = FACING_STEP[self.facing]
        nc, nr = self.col + dc, self.row + dr
        if not on_mat(nc, nr) or (nc, nr) in self.boxes or r < 0.08:
            return self.robot_refuses(rng.choice(["obstacle", "sensor"]) if r < 0.08 else "obstacle", True, True)
        if r < 0.18:
            return self.stop(rng.choice(STOPS), 0)
        if r < 0.21:
            return self.stop("timeout", 0)
        self.col, self.row, self.pose, self.slid, self.fresh = nc, nr, self.pose + 1, 1, rng.random() < 0.5
        self.visited.add((nc, nr))
        common.count("cells")
        self.said.append(["done", "F", {}])
        return {"result": "done", "cells_done": 1}

    def step(self, terrain, target, card_ahead):
        self.note("step", terrain, target=target, card_ahead=card_ahead)
        over = self.over(True)
        if over:
            return over
        if target is None:
            return self.refuse("off_mat")
        if card_ahead:
            return self.refuse("card", cell=cell_name(*target))
        r = self.rng.random()
        if r < 0.03:
            return self.refuse("card", cell=cell_name(*target))
        if r < 0.06:
            return self.refuse("ball_forward", cell=cell_name(*target))
        return self.forwards(terrain, target)

    def step_blind(self, terrain, target, card_ahead, dist, need_cm):
        self.note("step_blind", terrain, target=target, card_ahead=card_ahead, dist=dist, need_cm=need_cm,
                  read=self.last_dist)
        over = self.over(True)
        if over:
            return over
        if card_ahead:
            return self.refuse("card", cell="ahead")
        ball = terrain.ball_sighting() or {}
        if ball and ball["s_ago"] < self.rules["ball_seen_recently_s"]:
            return self.refuse("ball_unknown")
        if dist is None or dist < need_cm:
            return self.refuse("unknown_way", seen_cm=dist)
        return self.forwards(terrain, target)

    def slide(self, terrain, way, cells, until, asked, hold=None):
        rng = self.rng
        self.note("slide", terrain, way=way, cells=[tuple(c) for c in cells], until=list(until), asked=asked,
                  lanes=[terrain.on_lane(c) for c in cells])
        over = self.over(True)
        if over:
            return over
        if not cells:
            return self.refuse("off_mat")
        card_stop = None
        if rng.random() < 0.1:
            j = rng.randrange(len(cells))
            card_stop, cells = cell_name(*cells[j]), cells[:j]
            if not cells:
                return self.refuse("card", cell=card_stop)
        if way == "F":
            if rng.random() < 0.04:
                return self.refuse("ball_forward", cell=cell_name(*cells[0]))
        else:
            for c in cells:
                if terrain.seen_free_s_ago(c) > self.rules["blind_only_into_cells_seen_free_for_s"]:
                    return self.refuse("blind_not_seen", cell=cell_name(*c), free_for_min=2)
                if terrain.on_lane(c):
                    return self.refuse("blind_lane", cell=cell_name(*c))
            if rng.random() < 0.04:
                return self.refuse("blind_ball", cell=cell_name(*cells[0]))
        self.begin(terrain, cells, True)
        if self.wait_adult or self.estop:
            return self.robot_refuses("wait_adult", True)
        if rng.random() < 0.05:
            return self.robot_refuses(rng.choice(ERRS), True)
        n = len(cells)
        dc, dr = FACING_STEP[(self.facing + "FRBL".index(way)) % 4]
        room = 0  # how far it can really go: the edge of the mat, a box
        while room < n and on_mat(self.col + dc * (room + 1), self.row + dr * (room + 1)) and \
                (self.col + dc * (room + 1), self.row + dr * (room + 1)) not in self.boxes:
            room += 1
        o, why, stopped = rng.random(), None, None
        lanes = [i for i in range(room) if terrain.on_lane(cells[i])]
        if lanes and rng.random() < 0.4:
            k, stopped = lanes[0] + 1, "bump"  # the ball rolling on its lane hits the robot on that cell
        elif hold is not None and hold():
            k, why = min(1, room), "hold"
        elif o < 0.03:
            return self.robot_refuses("obstacle", True, True)
        elif o < 0.15:
            k, stopped = rng.randint(0, room), rng.choice(STOPS)
        elif o < 0.18:
            k, stopped = rng.randint(0, room), "timeout"
        elif o < 0.32:
            k = rng.randint(0, room)
            why = rng.choice([w.lower() for w in until] + ["camera", "sensor", "bump", "estop", "watchdog", "command"])
        else:
            k = room
        for i in range(k):
            self.col, self.row = self.col + dc, self.row + dr
            self.visited.add((self.col, self.row))
            self.pose += 1
            self.slid = i + 1
            common.count("cells")
            terrain.reached(cells[i], str(self.pose))
        if stopped:
            return self.stop(stopped, k)
        why = why or ("all" if k == n else "obstacle")
        if k == 0:
            common.count("moves", -1)
        self.fresh = rng.random() < 0.5
        self.said.append(["done", why, {"cells_done": k, "card_stop": card_stop}])
        return {"result": "done", "cells_done": k, "why": why, "card_stop": card_stop,
                "edge": n < asked and card_stop is None}

    def turn(self, terrain, way, counted):
        rng = self.rng
        self.note("turn", terrain, way=way, counted=counted)
        over = self.over(counted)
        if over:
            return over
        self.begin(terrain, [], counted)
        if self.wait_adult or self.estop:
            return self.robot_refuses("wait_adult", counted)
        r = rng.random()
        if r < 0.05:
            return self.robot_refuses(rng.choice(ERRS), counted)
        if r < 0.12:
            return self.stop(rng.choice(STOPS), 0)
        if r < 0.14:
            return self.stop("timeout", 0)
        self.facing = (self.facing + (1 if way == "R" else -1)) % 4
        self.pose += 1
        self.fresh = rng.random() < 0.5
        self.said.append(["done", "turn", {}])
        return {"result": "done", "cells_done": 0}

    def light(self, color):
        self.notes.append({"kind": "light", "color": color, "waiting": bool(self.wait_adult or self.estop)})
        common.count("commands")
        if self.rng.random() < 0.03:
            self.said.append(["refused", "robot", {"why": "busy"}])
            return {"result": "refused", "rule": "robot", "why": "busy"}  # the real light() says nothing of moving
        self.said.append(["done", "light", {}])
        return {"result": "done"}


# --- the generators -------------------------------------------------------------------------------------------------------
JUNK = ["move up 2", "move north 9", "move north 0", "turn around", "face up", "step", "step_forward 2",
        "set_light purple", "wait_for 0", "wait_for 60", "move east 2 card", "", "fly", "look around",
        "move west 2 stop_if=card_ahead", "move south two", "robot_state now", "seen_cards all", "turn L"]


def random_line(rng, persona, queue):
    """One line; queue: lines this run still has to send (face a way, then slide forwards, as go_to_cell does)."""
    if queue:
        return queue.pop(0)
    if persona == "careless" and rng.random() < 0.4:
        return rng.choice(JUNK)
    kind = rng.choices(["step_forward", "move", "turn", "look_around", "face", "set_light", "robot_state",
                        "seen_cards", "wait_for"], [5, 6, 3, 2, 2, 1, 2, 3, 1])[0]
    if kind == "move":
        stop_if = rng.sample(sorted(STOP_IF), rng.choice([0, 0, 1, 2]))
        line = "move %s %d%s" % (rng.choice(FACING), rng.randint(1, 4), " " + ",".join(stop_if) if stop_if else "")
        if rng.random() < 0.3:
            queue.append(line)
            return "face " + line.split()[1]
        return line
    if kind == "turn":
        return "turn " + rng.choice(["left", "right"])
    if kind == "face":
        return "face " + rng.choice(FACING)
    if kind == "set_light":
        return "set_light " + rng.choice(COLORS)
    if kind == "wait_for":
        return "wait_for %d" % rng.choice([1, 2, 3, 5, 10, 30])
    return kind


def parse(line):
    """A line of the language, as a call: (method, arguments), or None when it is not the language."""
    w = line.split()
    if w in (["step_forward"], ["look_around"], ["robot_state"], ["seen_cards"]):
        return w[0], {}
    if len(w) in (3, 4) and w[0] == "move" and w[1] in FACING and re.fullmatch(r"[1-4]", w[2]):
        stop_if = w[3].split(",") if len(w) == 4 else []
        if all(s in STOP_IF for s in stop_if):
            return "move", {"direction": w[1], "cells": int(w[2]), "stop_if": list(dict.fromkeys(stop_if))}
    if len(w) == 2 and w[0] == "turn" and w[1] in ("left", "right"):
        return "turn", {"direction": w[1]}
    if len(w) == 2 and w[0] == "face" and w[1] in FACING:
        return "face", {"direction": w[1]}
    if len(w) == 2 and w[0] == "set_light" and w[1] in COLORS:
        return "set_light", {"color": w[1]}
    if len(w) == 2 and w[0] == "wait_for" and re.fullmatch(r"[0-9]{1,2}", w[1]) and 1 <= int(w[1]) <= 30:
        return "wait_for", {"seconds": int(w[1])}
    return None


def act(cart, method, args):
    if method == "move":
        return cart.move(args["direction"], args["cells"], args["stop_if"])
    if method in ("turn", "face"):
        return getattr(cart, method)(args["direction"])
    if method == "set_light":
        return cart.set_light(args["color"])
    return getattr(cart, method)()


# --- the invariants --------------------------------------------------------------------------------------------------------
def snap(cart):
    # notices: a settle or an unsettle adds one; two in a call (found, then lost) start the relative frame again.
    return {"col": cart.col, "row": cart.row, "facing": cart.facing, "sure": cart.sure, "frame": cart.frame is not None,
            "notices": len(cart.notices)}


def strings(obj, skip=(), path=""):
    """Every text of an answer, with where it is (state.ball_last_seen.cell)."""
    if isinstance(obj, str):
        yield path, obj
    elif isinstance(obj, dict):
        for k, v in obj.items():
            if k not in skip:
                yield from strings(v, skip, f"{path}.{k}" if path else k)
    elif isinstance(obj, list):
        for i, v in enumerate(obj):
            yield from strings(v, skip, f"{path}[{i}]")


def visible(view):
    """The cells the camera could have looked at from a view (cell, facing, reading): an upper bound, in its cone
    and range, never straight ahead behind what the sensor met."""
    col, row, facing, dist = view
    out = {(col, row)}
    for c in range(COLS):
        for r in range(ROWS):
            angle, d = bearing((col, row), (c, r), facing)
            if d == 0 or d > C.CAMERA_RANGE_CM + 1e-6 or abs(angle) > C.CAMERA_HALF_ANGLE_DEG + 1e-6:
                continue
            if abs(angle) < 1 and dist is not None and d - CELL_CM / 2 > dist + 10:
                continue
            out.add((c, r))
    return out


def check_refusal(result, code, details, method, broke):
    why = result.get("why", "")
    if code == "robot":
        if why != C.REASONS.get(details["why"]):
            broke("a refusal of the Referee is said in its phrase", [code, details, why])
        if bool(result.get("waited")) != bool(details.get("waited")):
            broke("a refusal after a wait says it waited", [details, result])
        if ("what_to_do" in result) != (details["why"] == "fault"):
            broke("what_to_do only for the codes that have one", [code, details, result])
        return
    m = re.fullmatch(PHRASE[code], why)
    if not m:
        broke("a refusal of the Referee is said in its phrase", [code, why])
    elif "cell" in m.groupdict() and details.get("cell") and m["cell"] != details["cell"]:
        broke("the phrase names the cell the Referee gave", [code, details, why])
    if ("what_to_do" in result) != (code in TO_DO):
        broke("what_to_do only for the codes that have one", [code, result])


def check_state(state, after, limits, broke):
    keys = set(state) - {"since_last_call"}
    extra = keys - STATE_KEYS
    if not STATE_KEYS <= keys or extra - {"program_error", "position", "since_start", "how_to_find_it", "could_be"}:
        broke("robot_state has its closed fields", sorted(keys ^ STATE_KEYS))
        return
    known = state["position_known"]
    if known != (state["cell"] is not None) or known != (state["facing"] is not None):
        broke("robot_state: a cell and a facing exactly when the position is known", state)
    if known and (state["cell"] not in CELLS or state["facing"] not in FACING):
        broke("robot_state: a cell of the mat", [state["cell"], state["facing"]])
    if known and (not after["sure"] or after["frame"]):
        broke("robot_state: no cell while the place is unknown or relative", [state["cell"], after])
    if not after["sure"] and not state["waiting_for_adult"]:
        broke("robot_state: after a stop, it waits for an adult", state)
    frame_part = {"position", "since_start", "how_to_find_it"}
    if (frame_part <= keys) != (after["frame"] and after["sure"]) or (keys & frame_part and not frame_part <= keys):
        broke("robot_state: position, since_start, how_to_find_it exactly while the place is relative", sorted(keys))
    if "position" in state:
        if ("could_be" in state) != (state["position"] == "probable") or len(state.get("could_be", [])) > 6:
            broke("robot_state: could_be only when probable, 6 places at most", [state["position"],
                                                                                   state.get("could_be")])
    moves = limits.get("moves")
    if (moves is None) != (state["moves_left"] is None) or (
            moves is not None and not (isinstance(state["moves_left"], int) and state["moves_left"] >= 0)):
        broke("moves_left is null without a limit of the adults", [limits, state["moves_left"]])
    counts = state["counts"]
    if not isinstance(counts, dict) or not set(common.COUNTS) - {"missions"} <= set(counts) or "since" not in counts \
            or ("limits_per_mission" in counts) != bool(limits):
        broke("counts: the counters, since when, and the limits when there are", counts)
    if (state["cells_not_looked_at_yet"] is None) != after["frame"]:
        broke("cells_not_looked_at_yet: counted only while the place on the mat is known", [state, after])


def check_coverage(cov, after, broke):
    if set(cov) != COVERAGE_KEYS or cov["cells_on_the_mat"] != COLS * ROWS:
        broke("camera_coverage has its closed fields", cov)
        return None
    if after["frame"]:
        if cov["cells_looked_at"] is not None or cov["not_looked_at_yet"] is not None or cov["something_stands_there"]:
            broke("camera_coverage: not counted while the place is relative", cov)
        return None
    left, stands = cov["not_looked_at_yet"], cov["something_stands_there"]
    if not isinstance(left, list) or any(c not in CELLS for c in left + stands) or len(set(left)) != len(left) \
            or len(set(stands)) != len(stands) or set(left) & set(stands):
        broke("camera_coverage: cells of the mat, each once, never both to look at and standing", cov)
        return None
    if cov["cells_looked_at"] + len(left) + len(stands) != COLS * ROWS:
        broke("camera_coverage: looked at, to look at and standing add up to the mat", cov)
    if after["sure"] and cell_name(after["col"], after["row"]) in stands:
        broke("nothing stands where the robot stands", [cell_name(after["col"], after["row"]), stands])
    return set(CELLS) - set(left) - set(stands)


def check_call(method, args, result, before, after, fake, said, notes, run, broken):
    def broke(name, detail):
        broken.append({"invariant": name, "detail": detail})
    end = snap(run["cart"])
    if not isinstance(result, dict):
        broke("an answer is a dict", result)
        return
    word = result.get("result")
    moves = [n for n in notes if n["kind"] in ("step", "step_blind", "slide", "turn")]
    same_frame = before is not None and after is not None and before["frame"] == after["frame"] and \
        before["notices"] == after["notices"]
    # The closed format.
    allowed = {"step_forward": {"done", "refused", "stopped", "error"}, "move": {"done", "refused", "stopped", "error"},
               "turn": {"done", "refused", "stopped", "error"}, "set_light": {"done", "refused", "error"},
               "look_around": {"done", "refused", "stopped", "error", "paused"},
               "face": {"done", "refused", "stopped", "error", "paused"},
               "robot_state": {None, "error", "refused"}, "seen_cards": {None, "error", "refused"},
               "wait_for": {"something happened", "nothing happened", "nothing new happened", "error"}}[method]
    if word not in allowed:
        broke("an answer in the closed format", [method, word])
        return
    if word == "error" and not fake.troubled:
        broke("an error only when an answer of the robot could not be read", result)
    for entry in result.get("since_last_call", []) + (result.get("events", []) if method == "wait_for" else []):
        if not isinstance(entry, dict) or not isinstance(entry.get("event"), str) or set(entry) - {
                "event", "seconds_before_this_answer", "before_you_started_waiting"} or \
                entry.get("seconds_before_this_answer", 0) < 0:
            broke("since_last_call: events in words, with how long before", entry)
    # Never a cell name while the place is not known on the mat: the answer, built during the action (robot_state's
    # could_be lists the places of the mat it could be on, on purpose); the events, when the place stayed relative.
    if after and after["frame"]:
        for path, text in strings({k: v for k, v in result.items() if k != "since_last_call"}, skip=("could_be",)):
            if NAME.search(text):
                broke("never a cell name while the place on the mat is not known", [path, text])
                break
    if before and before["frame"] and after and after["frame"] and end["frame"]:
        for path, text in strings(result.get("since_last_call", []), path="since_last_call"):
            if NAME.search(text) and "now knows where the robot is" not in text:
                broke("never a cell name while the place on the mat is not known", [path, text])
                break
    if method == "wait_for":
        events = result.get("events", [])
        fresh = [e for e in events if not e.get("before_you_started_waiting")]
        if (word == "something happened" and not fresh) or (word == "nothing new happened" and (fresh or not events)) \
                or (word == "nothing happened" and "events" in result):
            broke("wait_for: ends on an event of the wait, and marks the ones from before", result)
        if word != "error" and result.get("waited_seconds", 0) > args["seconds"] + 0.5:
            broke("wait_for: never longer than asked", result)
        return
    if word == "error":
        return
    # Who refused: the door, the Referee (its last word), or the Cartographer itself (nothing sent below).
    if word == "refused":
        if before is not None and not before["sure"] and method != "set_light":
            if result.get("why") != C.REASONS["wait_adult"] or not str(result.get("what_to_do", "")).startswith(ADULT):
                broke("waiting for an adult: refused, with the adult's instruction", result)
            if moves:
                broke("a refusal sends nothing below", moves)
        elif before is not None and before["frame"] and method in ("move", "face") and not moves:
            check_refusal(result, "position_unknown", {}, method, broke)
        elif said and said[-1][0] == "refused":
            check_refusal(result, said[-1][1], said[-1][2], method, broke)
        else:
            broke("a refusal comes from a rule", [result, said])
        if same_frame:
            keys = ("col", "row", "sure") + (("facing",) if method in ("step_forward", "move", "turn") else ())
            if any(before[k] != after[k] for k in keys):
                broke("a refusal leaves the place as it was", [before, after])
    if method == "set_light" and before is not None and not before["sure"] and not any(
            n["kind"] == "light" for n in notes):
        broke("set_light passes while the robot waits for an adult", result)
    # A stop: an adult must come, put the robot back where the last move started (a slide: the last cell it reached
    # by its own count, never on a ball's lane).
    if word == "stopped":
        todo = str(result.get("what_to_do", ""))
        if result.get("position_known") is not False or not todo.startswith(ADULT):
            broke("after a stop: position_known false, and the adult's instruction", result)
        if after is not None and after["sure"]:
            broke("after a stop, the place is no longer known", after)
        stop = said[-1] if said and said[-1][0] == "stopped" else None
        if stop is None or result.get("why") != C.REASONS.get(stop[1]):
            broke("a stop is said in the words of its reason", [result.get("why"), stop])
        if method == "move" and stop is not None and result.get("cells_moved") != stop[2]["cells_done"]:
            broke("a stopped slide says the cells it moved", [result, stop])
        last = moves[-1] if moves else None
        if last is not None and not last["frame"] and after is not None and not after["frame"]:
            col, row, facing = last["belief"]
            if last["kind"] == "slide" and stop is not None:
                way = [(col, row)] + list(last["cells"])
                k = min(stop[2]["cells_done"], len(way) - 1)
                while k > 0 and last["lanes"][k - 1]:
                    k -= 1
                col, row = way[k]
            expected = f"{ADULT}cell {cell_name(col, row)}, facing {FACING[facing]}"
            if not todo.startswith(expected):
                broke("the adult puts the robot back on the last cell it reached, off a ball's lane", [todo, expected])
        elif last is not None and last["frame"] and NAME.search(todo):
            broke("after a stop, place relative: the instruction names no cell", todo)
        run["instruction"] = (todo, after["frame"] if after else None)
    elif run["instruction"] and fake.instruction is not None:
        todo = result.get("what_to_do", "")
        if todo.startswith(ADULT) and after is not None and after["frame"] == run["instruction"][1] \
                and todo != run["instruction"][0]:
            broke("the adult's instruction does not change while the robot waits", [run["instruction"][0], todo])
    # A move done, on the map.
    if word == "done" and same_frame:
        dc, dr = FACING_STEP[before["facing"]]
        expect = None
        if method == "step_forward":
            expect = (before["col"] + dc, before["row"] + dr, before["facing"])
        elif method == "move":
            side = FACING.index(args["direction"])
            k = result.get("cells_moved")
            if not isinstance(k, int) or not 0 <= k <= args["cells"] or result.get("cells_asked") != args["cells"]:
                broke("a slide says the cells it moved and the cells asked", result)
            else:
                expect = (before["col"] + FACING_STEP[side][0] * k, before["row"] + FACING_STEP[side][1] * k,
                          before["facing"])
            done = said[-1] if said and said[-1][0] == "done" else None
            if done is None or done[2].get("cells_done") != k:
                broke("a slide done says the cells the Referee gave", [result, done])
            else:
                reason = result.get("stopped_because")
                if done[1] == "all" and done[2]["card_stop"]:
                    ok = reason == (f"it stopped before {done[2]['card_stop']}, where the camera saw a card: it never "
                                    "drives onto a card")
                elif done[1] == "all" and len(moves[-1]["cells"]) < args["cells"]:
                    ok = re.fullmatch(r"the mat ends after %d cells? that way" % k, reason or "") is not None
                else:
                    ok = reason == C.WHY_DONE.get(done[1])
                if not ok:
                    broke("a slide done says why it stopped, in its words", [done, reason])
        elif method == "turn":
            expect = (before["col"], before["row"], (before["facing"] + (1 if args["direction"] == "right" else -1)) % 4)
        elif method == "face":
            expect = (before["col"], before["row"], FACING.index(args["direction"]))
        elif method == "look_around":
            expect = (before["col"], before["row"], before["facing"])
        if expect is not None and (after["col"], after["row"], after["facing"]) != expect:
            broke("a move done moves the place one way, as asked", [method, args, before, after])
        if after["frame"] is False and not on_mat(after["col"], after["row"]):
            broke("a known place is on the mat", after)
    # What reached the Referee.
    if method == "move" and moves and before is not None and not before["frame"]:
        sent = moves[0]
        side = FACING.index(args["direction"])
        way = "FRBL"[(side - before["facing"]) % 4]
        cells = []
        for k in range(1, args["cells"] + 1):
            c, r = before["col"] + FACING_STEP[side][0] * k, before["row"] + FACING_STEP[side][1] * k
            if not on_mat(c, r):
                break
            cells.append((c, r))
        if sent["kind"] != "slide" or sent["way"] != way or sent["cells"] != cells or sent["asked"] != args["cells"] \
                or sent["until"] != [STOP_IF[s] for s in args["stop_if"]]:
            broke("a move of the mat becomes the slide of the robot that goes that way", [args, before, sent])
    if method == "step_forward" and moves:
        sent = moves[0]
        if sent["kind"] == "step_blind":
            if not sent["frame"] or sent["need_cm"] != CELL_CM + C.STOP_MARGIN_CM or sent["dist"] != sent["read"]:
                broke("a blind step only while the place is relative, with the reading and 38 cm", sent)
        elif sent["frame"]:
            broke("a step of the mat only while the place on the mat is known", sent)
        else:
            col, row, facing = sent["belief"]
            ahead = (col + FACING_STEP[facing][0], row + FACING_STEP[facing][1])
            if sent["target"] != (ahead if on_mat(*ahead) else None):
                broke("a step asks for the cell ahead, or none off the mat", sent)
    turns = [n["counted"] for n in moves if n["kind"] == "turn"]
    if turns and turns != [True] + [False] * (len(turns) - 1):
        broke("only the first quarter turn of a call is counted", [method, turns])
    held = False
    for n in notes:
        if n["kind"] == "hold" and n["said"] and method in ("look_around", "face"):
            held = True
        elif n["kind"] == "turn" and held:
            broke("no quarter turn after the mission was paused", [method, notes])
            break
    if word == "paused":
        if not held or set(result) - {"since_last_call"} != {"result", "why", "state"} | (
                {"looked_before_it_stopped"} if method == "look_around" else set()):
            broke("paused only when the mission was paused, with why and state", result)
    # The answers' fields.
    state = (result if word is None else None) if method == "robot_state" else result.get("state")
    if isinstance(state, dict) and after is not None:
        check_state(state, after, fake.limits(), broke)
    if method == "look_around" and word == "done":
        looked = result.get("looked")
        if not isinstance(looked, list) or len(looked) != 4 or any(
                set(e) != {"facing", "card_ahead", "distance_ahead_cm", "what_the_camera_saw"} for e in looked):
            broke("look_around: four looks, each with its closed fields", looked)
    if method == "seen_cards" and word is None:
        cards = result.get("cards_seen")
        if not isinstance(cards, list) or "camera_coverage" not in result:
            broke("seen_cards has its closed fields", sorted(result))
            return
        for card in cards:
            names = [card.get("cell")] + list(card.get("could_also_be") or [])
            if after["frame"]:
                if card.get("cell") is not None or card.get("could_also_be") or "where" not in card:
                    broke("seen_cards: no cell while the place is relative", card)
            elif set(card) != CARD_KEYS or any(n not in CELLS for n in names) or card["card"] not in MAT["cards"]:
                broke("seen_cards: cards of the mat on cells of the mat", card)
        looked = check_coverage(result["camera_coverage"], after, broke)
        if looked is not None:
            if run["looked"] is not None and run["looked"][1] == run["epoch"] and not run["looked"][0] <= looked | set(
                    result["camera_coverage"]["something_stands_there"]):
                broke("a cell looked at stays looked at", sorted(run["looked"][0] - looked))
            run["looked"] = (looked, run["epoch"])
            if not fake.lied:
                seen = set()
                for view in fake.views:
                    seen |= visible(view)
                seen |= fake.visited
                extra = sorted(looked - {cell_name(*c) for c in seen})
                if extra:
                    broke("a cell counts as looked at only if the camera could have seen it", extra)


def check_truth(cart, fake, broken):
    """Readings that never lied: a place the Cartographer is sure of is the true one; a place it looks for keeps
    the true one among those it could be."""
    if fake.lied or fake.between or not cart.sure:
        return
    true = [fake.col, fake.row, fake.facing]
    if cart.frame is None:
        if [cart.col, cart.row, cart.facing] != true:
            broken.append({"invariant": "with true readings, the place found is the true one",
                           "detail": [[cart.col, cart.row, cart.facing], true]})
    elif true not in [list(p) for p in cart.places()]:
        broken.append({"invariant": "with true readings, the true place stays among the places possible",
                       "detail": [true, len(cart.frame["candidates"])]})


# --- one run ------------------------------------------------------------------------------------------------------------------
def draw_rules(world):
    rules = dict(RULES)
    if world.random() < 0.3:
        rules["limits"] = {world.choice(["moves", "commands", "cells"]): world.randint(2, 20)}
    rules["mat_north_up"] = world.random() < 0.75
    x = world.random()
    cell, facing = (world.randrange(COLS), world.randrange(ROWS)), world.randrange(4)
    if x < 0.3:
        rules["start"], truth = None, (0, 5, 0)
    elif x < 0.45:
        rules["start"], truth = {"cell": cell_name(*cell), "facing": FACING[facing]}, cell + (facing,)
    elif x < 0.6:
        rules["start"], truth = {"facing": "north"}, cell + (0,)
    elif x < 0.7:
        rules["start"], truth = {"cell": cell_name(*cell)}, cell + (facing,)
    else:
        rules["start"], truth = "unknown", cell + (facing,)
    wrong = world.random() < 0.1 and rules["start"] != "unknown"
    if wrong:  # the adults gave a wrong start: the robot really faces another way
        truth = truth[:2] + ((truth[2] + world.choice([1, 2, 3])) % 4,)
    return rules, truth, wrong


def run(seed, generator, persona, steps, replay=None, module=cartographer_module):
    rng, pace = random.Random(seed), random.Random(seed * 31 + 7)
    world = random.Random(seed * 7919 + 1)
    for f in STATE.iterdir():
        if f.is_file():
            f.unlink()
    clock = Clock()
    module.time = clock
    broken, sent, answers, history, cover = [], [], [], [], collections.Counter()
    rules, truth, wrong = draw_rules(world)
    fake = Referee(world, clock, rules, truth, lies=world.random() < 0.3, broken=broken)
    fake.lied = wrong
    if fake.cards and world.random() < 0.4:
        where, label = world.choice(sorted(fake.cards.items()))
        rules["landmarks"] = {label: cell_name(*where)}
    fake.hold_rate = 0.2 if world.random() < 0.25 else 0.0
    cart = module.Cartographer(MAT, rules)
    cart.referee = fake
    cart.hold_wanted = fake.hold
    state = {"cart": cart, "instruction": None, "looked": None, "epoch": 0}
    start = rules["start"]
    cover["start " + (start if isinstance(start, str) else "+".join(sorted(start)) if start else "mat")] += 1
    error, queue = None, []
    frame_was = cart.frame is not None
    for n in range(steps):
        if replay is not None:
            if n >= len(replay["sent"]):
                break
            line = replay["sent"][n]
        elif generator == "random":
            line = random_line(rng, persona, queue)
        else:
            try:
                line = probe_tiers.ollama_line(generator[len("ollama:"):], persona, history, seed * 1000 + n,
                                               system=PERSONAS[persona] + "\n\nThe language:\n\n" + LANGUAGE)
            except (OSError, ValueError, KeyError) as e:
                error = f"model: {type(e).__name__}: {e}"
                break
        sent.append(line)
        history.append("> " + line)
        clock.sleep(pace.choice([0.3, 1, 2, 4, 12, 30]))
        call = parse(line)
        if call is None:
            answers.append("not the language")
            history.append("< not understood: not a call of the language")
            continue
        method, args = call
        fake.notes, fake.said, fake.troubled = [], [], False
        seen = {}

        def action():
            seen["before"] = snap(cart)
            out = act(cart, method, args)
            seen["after"] = snap(cart)
            return out
        try:
            if method == "wait_for":
                result = cart.wait_for(args["seconds"])
            else:
                result = cart.call(action, is_move=method in MOVES)
        except Exception as e:  # noqa: BLE001
            broken.append({"invariant": "no error escapes a call", "detail": repr(e)})
            result = {"result": "error", "why": repr(e)}
            fake.troubled = True
        if isinstance(result, dict) and result.get("result") == "stopped":
            fake.instruction = result.get("what_to_do")
        frame_now = cart.frame is not None
        if frame_now != frame_was:
            state["epoch"] += 1
        frame_was = frame_now
        check_call(method, args, result, seen.get("before"), seen.get("after"), fake, fake.said, fake.notes, state,
                   broken)
        check_truth(cart, fake, broken)
        word = result.get("result") if isinstance(result, dict) else None
        if word is None:
            label = "answered " + method
        elif word == "refused":
            label = "refused " + (fake.said[-1][1] if fake.said and fake.said[-1][0] == "refused" else
                                  "by the Cartographer")
        elif word == "stopped":
            label = "stopped " + (fake.said[-1][1] if fake.said else "?")
        elif word == "done" and method == "move" and fake.said:
            label = "done move " + fake.said[-1][1]
        elif word == "done":
            label = "done " + method
        else:
            label = word
        answers.append(label)
        for event in result.get("since_last_call", []) + result.get("events", []):
            text = event.get("event", "")
            for key in ("now knows where the robot is", "no longer knows", "every cell of the mat",
                        "older events were lost", "an adult pressed Resume", "the camera recognised a card",
                        "the camera sees a ball"):
                if key in text:
                    cover["event: " + key] += 1
            m = re.search(r"\(found with ([^)]*)\)", text)
            if m:
                cover["found with " + m[1]] += 1
        history.append("< " + json.dumps(result)[:300])
        for b in broken:
            b.setdefault("line", line)
            b.setdefault("n", n)
    if not fake.lied:
        cover["runs with true readings"] += 1
    cover["runs ending with the place known" if cart.frame is None else "runs ending with the place relative"] += 1
    return {"seed": seed, "sent": sent, "answers": answers, "broken": broken, "error": error, "rules": rules,
            "truth": list(truth), "coverage": dict(cover)}


def summary(results, generator, persona):
    lines = [l for r in results for l in r["sent"]]
    words = collections.Counter(a.split()[0] if not a.startswith("not") else "not the language"
                                for r in results for a in r["answers"])
    labels = collections.Counter(a for r in results for a in r["answers"])
    runs = collections.Counter()
    for r in results:
        runs.update(r["coverage"])
    rejected = words["not the language"] + words["refused"] + words["error"]
    return {"tier": "cartographer", "generator": generator, "persona": persona, "runs": len(results),
            "lines_sent": len(lines), "answers": dict(words),
            "accepted_share": round(1 - rejected / len(lines), 3) if lines else None,
            "not_understood_share": round(words["not the language"] / len(lines), 3) if lines else None,
            "coverage": dict(sorted(labels.items())), "coverage_of_the_runs": dict(sorted(runs.items())),
            "commonest_not_understood": collections.Counter(l for r in results for l, a in zip(r["sent"], r["answers"])
                                                            if a == "not the language").most_common(8),
            "runs_breaking_an_invariant": sum(1 for r in results if r["broken"]),
            "invariants_broken": dict(collections.Counter(b["invariant"] for r in results for b in r["broken"])),
            "model_errors": [r["error"] for r in results if r["error"]]}


# --- the probe checks itself ---------------------------------------------------------------------------------------------------
def mutants():
    def relative_cell(m):
        state = m.Cartographer.robot_state
        def patched(self):
            out = state(self)
            if self.frame and self.sure:
                out["cell"] = m.cell_name(self.col, self.row)  # a cell of the frame relative to the start
            return out
        m.Cartographer.robot_state = patched
    def known_after_stop(m):
        interrupted = m.Cartographer.interrupted
        def patched(self, why):
            out = interrupted(self, why)
            self.sure = True
            return out
        m.Cartographer.interrupted = patched
    def moved_after_refusal(m):
        say = m.Cartographer.say
        def patched(self, answer, direction=None, step=False):
            if answer.get("result") == "refused" and answer.get("rule") not in (None, "robot") and self.ahead():
                self.col, self.row = self.ahead()
            return say(self, answer, direction, step)
        m.Cartographer.say = patched
    def stands_counted(m):
        coverage = m.Cartographer.coverage
        def patched(self):
            out = coverage(self)
            if out["cells_looked_at"] is not None:
                out["cells_looked_at"] += len(out["something_stands_there"])
            return out
        m.Cartographer.coverage = patched
    def behind_counted(m):
        note = m.Cartographer.note_looked
        m.Cartographer.note_looked = lambda self, dist, boxes=(): note(self, None, ())
    def bare_code(m):
        say = m.Cartographer.say
        def patched(self, answer, direction=None, step=False):
            if answer.get("result") == "refused" and answer.get("rule") in ("card", "blind_not_seen", "off_mat"):
                return {"result": "refused", "why": answer["rule"]}
            return say(self, answer, direction, step)
        m.Cartographer.say = patched
    def mirrored(m):
        move = m.Cartographer.move
        def patched(self, direction, cells, stop_if):
            referee = self.referee
            class Mirror:
                def __getattr__(_, name): return getattr(referee, name)
                def slide(_, terrain, way, cells, until, asked, hold=None):
                    return referee.slide(terrain, {"L": "R", "R": "L"}.get(way, way), cells, until, asked, hold=hold)
            self.referee = Mirror()
            try:
                return move(self, direction, cells, stop_if)
            finally:
                self.referee = referee
        m.Cartographer.move = patched
    def all_counted(m):
        turn = m.Cartographer.turn
        m.Cartographer.turn = lambda self, direction, counted=True: turn(self, direction, True)
    def moves_while_waiting(m):
        step = m.Cartographer.step_forward
        def patched(self):
            self.sure = True
            return step(self)
        m.Cartographer.step_forward = patched
    def compass_loose(m): m.COMPASS_TOLERANCE_DEG = 60
    def lane_ignored(m):
        def reached_by_count(self, fields):
            if self.back and self.way and fields.get("slid", "").isdigit():
                col, row = self.way[min(int(fields["slid"]), len(self.way) - 1)]
                self.back, self.back_fixed = (col, row, self.back[2]), True
        m.Cartographer.reached_by_count = reached_by_count
    def moves_left_zero(m):
        state = m.Cartographer.robot_state
        def patched(self):
            out = state(self)
            out["moves_left"] = 0 if out.get("moves_left") is None else out["moves_left"]
            return out
        m.Cartographer.robot_state = patched
    def no_pause(m):
        look = m.Cartographer.look_around
        def patched(self):
            asked = self.hold_wanted
            self.hold_wanted = lambda: asked() and False
            try:
                return look(self)
            finally:
                self.hold_wanted = asked
        m.Cartographer.look_around = patched
    return {"a cell named in a relative frame": relative_cell, "the place still known after a stop": known_after_stop,
            "the place moves after a refusal": moved_after_refusal, "something standing counted as looked at":
            stands_counted, "a cell behind what the sensor met counted as looked at": behind_counted,
            "a refusal said as its bare code": bare_code, "west taken for east (L and R swapped)": mirrored,
            "every quarter turn counted": all_counted, "a step while the robot waits for an adult": moves_while_waiting,
            "the compass believed far from a quarter turn": compass_loose,
            "an adult sent onto a ball's lane": lane_ignored, "moves_left without a limit": moves_left_zero,
            "look_around goes on after a pause": no_pause}


def load_fresh():
    return probe_tiers.load("cartographer_mutant", GUIDE / "stack" / "cartographer.py")


def main(args):
    if args.mutants:
        # The stack as it is may already break invariants (a defect found): a run catches a mutant when it breaks an
        # invariant that the same run, without the mutant, does not break.
        def kinds(module, k):
            return {b["invariant"] for b in run(1 + k, "random", "careful" if k % 2 else "careless", args.steps,
                                                module=module)["broken"]}
        plain = [kinds(load_fresh(), k) for k in range(args.runs)]
        caught, by = {}, {}
        for name, patch in mutants().items():
            module = load_fresh()
            patch(module)
            new = [kinds(module, k) - plain[k] for k in range(args.runs)]
            caught[name] = sum(1 for n in new if n)
            by[name] = dict(collections.Counter(i for n in new for i in n).most_common(3))
        print(json.dumps({"tier": "cartographer", "runs_per_mutant": args.runs, "runs_that_caught_it": caught,
                          "caught_by": by, "missed": [k for k, v in caught.items() if not v],
                          "runs_breaking_an_invariant_without_a_mutant": sum(1 for p in plain if p)}, indent=1))
        shutil.rmtree(STATE, ignore_errors=True)
        return 1 if any(not v for v in caught.values()) else 0
    if args.replay:
        saved = json.loads(Path(args.replay).read_text(encoding="utf-8"))
        again = run(saved["seed"], "replay", saved.get("persona", "careful"), len(saved["sent"]), replay=saved)
        print(json.dumps({"broken": again["broken"],
                          "same_as_saved": [b["invariant"] for b in again["broken"]] ==
                          [b["invariant"] for b in saved["broken"]]}, indent=1))
        shutil.rmtree(STATE, ignore_errors=True)
        return 1 if again["broken"] else 0
    out = Path(args.out or GUIDE / ".robot-sim" / "probes" / time.strftime("cartographer-%Y%m%d-%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    results = []
    for k in range(args.runs):
        result = run(args.seed + k, args.generator, args.persona, args.steps)
        results.append(result)
        if result["broken"]:
            (out / f"violation-{result['seed']}.json").write_text(json.dumps(dict(result, persona=args.persona),
                                                                              indent=1) + "\n", encoding="utf-8")
    report = summary(results, args.generator, args.persona)
    (out / "summary.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    (out / "runs.jsonl").write_text("".join(json.dumps({k: r[k] for k in ("seed", "sent", "answers", "error")}) + "\n"
                                            for r in results), encoding="utf-8")
    print(json.dumps(report, indent=1))
    shutil.rmtree(STATE, ignore_errors=True)
    return 0
