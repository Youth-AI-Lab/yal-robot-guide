"""The Pilot alone (stack/pilot.py), probed by ci/probe_tiers.py --tier pilot: see the head of that file.

The generator plays the Strategist and writes the Pilot's language: one tool call per line, the name of a tool, then
its arguments as a JSON object (nothing when there are none):
    go_to_cell {"cell": "C3"}
    move {"direction": "north", "cells": 2, "stop_if": ["card_ahead"]}
    robot_state
The tools are those of one toolset of TOOLSETS (goal, route or step), drawn for each run, with their schemas. A call
outside its schema is refused at the door, as the Strategist (or an MCP server) does; now and then a move with a wrong
number of cells, or a cell off the mat, goes straight to the Pilot, as typed by hand, to try the Pilot's own checks.
Below it, a fake Cartographer that answers in its closed language, at random but plausibly: the mat of mat.json with
cards and boxes drawn for the run, moves done, refused or stopped (then an adult puts the robot back, or never comes),
cards seen from near or far, balls that come closer or go away, a limit of moves or none, a place on the mat not known
at the start, or lost on the way. It runs on a clock of the probe: nobody waits.
Around it, the mission: most runs are missions (the Pilot obeys the pause and the stop, which the group presses at
random, also in the middle of a move), the others are drive.py (no mission: a pause left written is ignored). A
mission begins with configure(), with settings drawn at random (some wrong), and ends now and then, as when the model
reports; between two missions, someone may drive the robot by hand.
"""
import collections
import copy
import json
import os
import random
import re
import shutil
import sys
import tempfile
import time
import traceback
from pathlib import Path

GUIDE = Path(__file__).resolve().parent.parent
STATE = Path(tempfile.mkdtemp(prefix="probe-pilot-"))
os.environ["ROBOT_STATE_DIR"] = str(STATE)
os.environ.setdefault("ROBOT_CALLER", "probe")
sys.path.insert(0, str(GUIDE / "stack"))
import common  # noqa: E402

import probe_tiers  # noqa: E402


def load_fresh():
    return probe_tiers.load("pilot_mutant", GUIDE / "stack" / "pilot.py")


BASE = load_fresh()  # the language and the mat; every run loads its own copy of the Pilot
MAT, RULES = BASE.MAT, BASE.RULES
COLS, ROWS, CELL_CM, MAX_CELLS = BASE.COLS, BASE.ROWS, BASE.CELL_CM, BASE.MAX_CELLS
CELLS = list(BASE.CELLS)
FACING = list(common.FACING)
STEP = dict(BASE.STEP)
CARDS = sorted(MAT["cards"])
COLORS = list(common.COLORS)
STOP_IF = sorted(BASE.STOP_IF)
RECENT_S = RULES["ball_seen_recently_s"]
DEFAULTS = {k: v["default"] for k, v in BASE.SETTINGS.items()}
CART_MOVES = {"step_forward", "move", "turn", "look_around", "face", "set_light"}
# A step tool and the Cartographer's call it passes on as it is.
PASSED = {"step_forward": "step_forward", "move": "move", "turn": "turn", "look_around": "look_around", "face": "face",
          "set_light": "set_light", "robot_state": "robot_state", "wait_for_event": "wait_for",
          "seen_cards": "seen_cards"}
LOGGED = {"state", "go_to_card", "explore", "light", "go_to_cell", "look", "face_towards"}  # the Pilot's own log
CALLS_PER_TOOL = 3000   # calls to the Cartographer in one tool, at most, before the probe calls it endless
HOURS_PER_TOOL = 4      # hours on the probe's clock in one tool, at most
GIVE_UP = ("the mission was stopped", "no moves left", "the robot still waits for an adult",
           "the robot waits for an adult",  # under the Strategist: its loop waits for the adult, without the model
           "the robot could not find where it is on the mat", "the Cartographer did not give the cards seen",
           "the robot did not find itself in front of the card")
FORMAT = ("One tool call per line: the name of a tool, then its arguments as a JSON object (nothing when there are "
          "none), for example:\n" + __doc__[__doc__.index("    go_to_cell"):__doc__.index("The tools are")].rstrip())
PERSONAS = {
    "careful": "You are the Strategist of a small robot on a mat of cells (columns A to H, rows 1 at the top to 6): "
               "you reach the goal of the mission by calling the Pilot's tools, one call per line, in its language. "
               "Find the cards on the mat, go in front of one, read the robot's state, follow the language exactly.",
    "careless": "You are a careless or hostile caller of the Pilot of a small robot. Send calls that test its limits: "
                "values out of the lists, numbers out of range, missing or unknown arguments, tools that do not exist, "
                "moves one after the other without reading the answers. Keep each to one line.",
}


def describe(prop):
    if "enum" in prop:
        return "one of " + ", ".join(prop["enum"])
    if prop["type"] == "integer":
        return f"a whole number from {prop['minimum']} to {prop['maximum']}"
    if prop["type"] == "array":
        return "a list of " + ", ".join(prop["items"]["enum"])
    return prop["type"]


def language(toolset):
    """The language as the generator reads it: the format, then each tool of the toolset with its description."""
    lines = [FORMAT, "", "The tools:"]
    for name, (fn, _, props, required) in BASE.TOOLSETS[toolset].items():
        args = ", ".join(f'"{k}": {describe(p)}' + ("" if k in required else " (optional)") for k, p in props.items())
        lines.append(f"{name} {{{args}}}" if props else name)
        lines.append("    " + " ".join((fn.__doc__ or "").split()))
    return "\n".join(lines)


# --- the clock and the mission around the Pilot ------------------------------------------------------------------------
class Clock:
    """The Pilot's time module for a run: time passes only when the Pilot sleeps or the fake Cartographer moves."""

    def __init__(self):
        self.t, self.m = 1.8e9, 1000.0

    def time(self): return self.t
    def monotonic(self): return self.m
    def sleep(self, s): self.advance(s)
    def strftime(self, fmt, *a): return time.strftime(fmt, *a)

    def advance(self, s):
        self.t += s
        self.m += s


class Mission:
    """The Interpreter's go, pause or stop (control.json), on the run's clock. obey: a mission runs (the Strategist
    set OBEY_MISSION_CONTROL); else drive.py, and a pause left written must change nothing."""

    def __init__(self, clock, obey):
        self.clock, self.obey = clock, obey
        self.control, self.until, self.then = "go", None, None
        self.pauses = self.stops = 0

    def now(self):
        if self.control == "pause" and self.clock.monotonic() >= self.until:
            self.control = self.then
            self.stops += self.then == "stop"
        return self.control

    def pause(self, seconds, then):
        self.control, self.until, self.then = "pause", self.clock.monotonic() + seconds, then
        self.pauses += 1

    def stop(self):
        self.control = "stop"
        self.stops += 1


class Common:
    """What `common` gives the Pilot during a run: the counters (in memory), the mission control of the run, held.json
    (written by the real common.say_held, in the probe's state folder), checked as they are used."""

    def __init__(self, mission, fake, broken):
        self.mission, self.fake, self.broken = mission, fake, broken
        self.counted = []
        self.acks = 0

    def __getattr__(self, name):
        return getattr(common, name)

    def count(self, name, n=1):
        self.counted.append(name)

    def mission_control(self):
        value = self.mission.now()
        if value == "pause" and not self.fake.inside:
            self.fake.saw_pause = True
        return value

    def say_held(self, where):
        def broke(name, detail):
            self.broken.append({"invariant": name, "detail": detail})
        fake = self.fake
        if not self.mission.obey:
            broke("without a mission (drive.py), a pause left written changes nothing", where)
        if where is None:
            fake.cleared = fake.acked
        else:
            self.acks += 1
            known = fake.sure and not fake.frame
            want = {"cell": BASE.cell_name(fake.col, fake.row) if known else None,
                    "facing": FACING[fake.facing] if known else None}
            if set(where) != {"cell", "facing", "since"} or {k: where.get(k) for k in want} != want or \
                    not re.match(r"^\d\d:\d\d:\d\d$", str(where.get("since"))):
                broke("the acknowledgement of a pause gives the cell and the way it faces, null when the place is not "
                      "known", [where, want])
            fake.acked, fake.cleared = True, False
        common.say_held(where)
        if where is not None and common.held_where() != where:
            broke("the acknowledgement of a pause is in held.json", [where, common.held_where()])


class ProbeLimit(BaseException):
    """A tool that does not end: raised by the fake Cartographer, never caught by the Pilot."""


# --- the fake below: the Cartographer -------------------------------------------------------------------------------------
def name_of(c, r):
    return BASE.cell_name(c, r)


def on_mat(c, r):
    return 0 <= c < COLS and 0 <= r < ROWS


class Cartographer:
    """A Cartographer written by hand, in its closed language (facilitator/ROBOT-LANGAGES.md, section 4): the methods
    the Pilot calls through call() and wait_for(), the attributes hold_still() reads (sure, frame, col, row, facing),
    and hold_wanted, set by the Pilot. The world is drawn from a seeded generator: the same seed, the same world."""

    def __init__(self, rng, clock, mission, broken, pilot, toolset, defects=True):
        self.rng, self.clock, self.mission, self.broken, self.pilot = rng, clock, mission, broken, pilot
        self.defects = defects  # False: none of the situations of the defects found (for the mutants)
        self.hold_wanted = lambda: False
        free = list(CELLS)
        rng.shuffle(free)
        self.truth = {}  # cell -> card, where the cards really are
        for _ in range(rng.randint(0, 4)):
            self.truth[free.pop()] = rng.choice(CARDS)
        self.boxes = {free.pop() for _ in range(rng.randint(0, 3))}
        self.col, self.row = BASE.cell_xy(free.pop())
        self.facing = rng.randrange(4)
        self.frame = rng.random() < 0.15  # the start is not known: the place on the mat is found by looking
        self.sure = True
        self.back = None
        self.seen, self.looked, self.ball = [], set(), None
        self.limit = None if rng.random() < 0.6 else rng.randint(2, 25)
        self.used = 0
        self.adult_comes = rng.random() < 0.9
        self.inside = self.flag = self.busy = self.unplugged = False
        self.saw_pause = self.acked = self.cleared = False
        self.view_ok = True       # False after a hand move, until the Pilot is handed a state again
        self.read_moves_left = (RULES.get("limits") or {}).get("moves")  # what the Pilot last read
        self.last_state, self.last_records = None, set()
        self.op = None
        self.coverage = collections.Counter()

    # The run's bookkeeping -----------------------------------------------------------------------------------------
    def broke(self, name, detail):
        self.broken.append({"invariant": name, "detail": detail})

    def begin(self, toolset, tool, settings):
        self.op = {"toolset": toolset, "tool": tool, "settings": dict(settings), "calls": [], "answers": [],
                   "looked_since_move": False, "start": self.clock.monotonic()}

    def known(self):
        return self.sure and not self.frame

    def here(self):
        return name_of(self.col, self.row)

    def free_cell(self, c, r):
        return on_mat(c, r) and name_of(c, r) not in self.boxes and name_of(c, r) not in self.truth

    def hand_move(self):
        """Someone drove the robot by hand (drive.py, the page) between two missions."""
        cells = [c for c in CELLS if c not in self.boxes and c not in self.truth]
        self.col, self.row = BASE.cell_xy(self.rng.choice(cells))
        self.facing = self.rng.randrange(4)
        self.sure, self.frame = True, False
        self.view_ok = False
        self.coverage["hand move between missions"] += 1

    def spend(self):
        if self.op is not None:
            if len(self.op["calls"]) > CALLS_PER_TOOL or self.clock.monotonic() - self.op["start"] > HOURS_PER_TOOL * 3600:
                raise ProbeLimit()

    def tick(self, chance):
        """The group may press pause or stop, at any time (in a mission or not: drive.py must ignore it)."""
        if self.mission.now() != "go":
            return
        r = self.rng.random()
        if r < chance:
            self.mission.pause(self.rng.uniform(1, 40), "stop" if self.rng.random() < 0.2 else "go")
        elif r < chance + 0.003:
            self.mission.stop()

    def ask_hold(self):
        value = self.hold_wanted()
        want = self.mission.obey and self.mission.now() in ("pause", "stop")
        if bool(value) != want:
            self.broke("hold_wanted is true exactly while a mission is paused or stopped",
                       [value, self.mission.obey, self.mission.now()])
        return bool(value)

    # One call of the Pilot -------------------------------------------------------------------------------------------
    def call(self, action, is_move=False):
        self.spend()
        if self.inside:
            self.broke("one call to the Cartographer at a time", self.op and self.op["calls"][-1:])
        self.inside, self.flag = True, is_move
        try:
            if self.rng.random() < 0.01 and self.known() and self.defects:
                self.frame = True  # a reading farther than the border: the place is lost (fields() at every call)
                self.coverage["place lost"] += 1
                lost = True
            else:
                lost = False
            self.busy = self.rng.random() < 0.02
            self.unplugged = self.rng.random() < 0.005
            result = action()
            self.tick(0.03)
            events = []
            if lost:
                events.append({"event": "the Cartographer no longer knows where the robot is: the distance sensor reads "
                                        "farther than the border; it counts from here again, and looks for its place"})
            if self.rng.random() < 0.06:
                self.see_ball()
                events.append({"seconds_before_this_answer": 0.4, "event": "the camera sees a ball"})
            if self.rng.random() < 0.05:
                events.append({"seconds_before_this_answer": 1.2, "event": "something touched the robot's bumper"})
            if events:
                result["since_last_call"] = events
        finally:
            self.inside = False
        return self.answer(result)

    def answer(self, result):
        if self.op is not None:
            self.op["answers"].append(copy.deepcopy(result))
        state = result.get("state") if isinstance(result.get("state"), dict) else result
        if "cell" in state and "position_known" in state:
            self.view_ok = True
            self.last_state = copy.deepcopy(state)
            self.read_moves_left = state["moves_left"] if isinstance(state.get("moves_left"), int) else None
        if "cards_seen" in result and not self.frame:
            self.last_records = {s["cell"] for s in result["cards_seen"]}
        return result

    def enter(self, name, **args):
        """What every method checks first: the call, its values, and for a move what must hold before it."""
        self.coverage["cart " + name] += 1
        if self.op is not None:
            self.op["calls"].append([name, args])
        if self.flag != (name in CART_MOVES):
            self.broke("every move goes through the Referee's door as a move, and only moves", [name, self.flag])
        why = cart_language(name, args)
        if why:
            self.broke("the Cartographer gets only values of its language", [name, args, why])
            return {"result": "error", "why": "the robot's answer could not be read (ValueError)"}
        if name in CART_MOVES:
            self.before_move(name, args)
        if self.unplugged:
            return {"result": "error", "why": "the robot does not answer: is its cable plugged in?"}
        return None

    def before_move(self, name, args):
        mission = self.mission
        if mission.obey and mission.now() in ("pause", "stop"):
            self.broke("in a mission, nothing moves while it is paused or after it is stopped", [name, mission.now()])
        if self.saw_pause and not (self.acked and self.cleared):
            self.broke("a pause the Pilot met is acknowledged in held.json before the robot moves again",
                       [name, self.acked, self.cleared])
        self.saw_pause = self.acked = self.cleared = False
        if (STATE / "held.json").exists():
            self.broke("held.json is cleared before the robot moves again", name)
        op = self.op
        if op is None or op["toolset"] == "step" or name == "set_light":
            return  # what follows holds for the moves the Pilot decides itself, in a goal or route tool
        if not self.view_ok and name in ("move", "look_around"):  # face is absolute: from anywhere, the same
            self.broke("the Pilot moves the robot only from where the Cartographer last said it is", [
                name, (self.pilot.PILOT.state.get("cell"), self.pilot.PILOT.state.get("facing")),
                (self.here(), FACING[self.facing])])
        if name != "move" or not self.known():
            if name == "look_around":
                op["looked_since_move"] = True
            return
        settings = op["settings"]
        if args["cells"] > max(1, min(MAX_CELLS, settings["max_leg"])):
            self.broke("a leg is never longer than max_leg", [args, settings])
        if settings["look_each_leg"] and not op["looked_since_move"]:
            self.broke("with look_each_leg, a look_around before every slide", [args, op["calls"][-6:]])
        ball = (self.last_state or {}).get("ball_last_seen") or {}
        if settings["careful_balls"] and ball.get("t") and self.clock.time() - ball["t"] < RECENT_S and \
                "closer" in ball.get("how", ""):
            self.broke("with careful_balls, no leg while a ball seen just before comes closer", [args, ball])
        dc, dr = STEP[args["direction"]]
        first = name_of(self.col + dc, self.row + dr) if on_mat(self.col + dc, self.row + dr) else None
        if first in self.last_records:
            self.broke("a leg never starts onto a card the Pilot knows", [args, first, sorted(self.last_records)])

    # What the Cartographer says --------------------------------------------------------------------------------------
    def distance(self):
        if self.rng.random() < 0.1:
            return None
        dc, dr = FACING_STEP[self.facing]
        k = 1
        while on_mat(self.col + dc * k, self.row + dr * k) and name_of(self.col + dc * k, self.row + dr * k) not in \
                self.boxes:
            k += 1
        return (k - 1) * CELL_CM + self.rng.randint(0, CELL_CM - 1)

    def ahead(self):
        dc, dr = FACING_STEP[self.facing]
        return (self.col + dc, self.row + dr) if on_mat(self.col + dc, self.row + dr) else None

    def state(self):
        known = self.known()
        ahead = self.ahead()
        card = self.truth.get(name_of(*ahead)) if ahead and known else None
        ball = copy.deepcopy(self.ball)
        lane = False
        if ball and ball.get("cell") and known:
            bc, br = BASE.cell_xy(ball["cell"])
            lane = (ball.get("moving") in ("east", "west") and br == self.row) or \
                (ball.get("moving") in ("north", "south") and bc == self.col) or (bc, br) == (self.col, self.row)
        state = {
            "cell": self.here() if known else None, "facing": FACING[self.facing] if known else None,
            "position_known": known, "distance_ahead_cm": self.distance(),
            "card_ahead": {"card": card, "means": MAT["cards"][card]} if card else None,
            "battery_percent": self.rng.randint(40, 100), "emergency_stop": False, "waiting_for_adult": not self.sure,
            "bumper_touched": False, "warning_light_on": False, "ball_last_seen": ball,
            "on_a_ball_lane": (lane or self.rng.random() < 0.03 and self.defects) if known else None,
            "cells_not_looked_at_yet": None if self.frame else len(CELLS) - len(self.looked), "last_stop": "none",
            "moves_left": None if self.limit is None else max(0, self.limit - self.used),
            "counts": {"since": "the mission began", "moves": self.used},
            "mat": f"{COLS} columns (A to {name_of(COLS - 1, 0)[0]}), {ROWS} rows (1 at the top to {ROWS})"}
        if self.frame and self.sure:
            n = self.rng.randint(1, 12)
            state["position"] = "probable" if n <= 6 else "unknown"
            state["since_start"] = "2 cells in front of where it started"
            if n <= 6:
                state["could_be"] = [f"{c} facing {self.rng.choice(FACING)}" for c in self.rng.sample(CELLS, n)]
            state["how_to_find_it"] = "look_around, then step_forward where the way is free"
        return state

    def refused(self, why, **extra):
        return dict({"result": "refused", "why": why}, **extra)

    def instruction(self):
        if self.frame:
            return ("Stop and ask an adult to put the robot back on the cell where its last move started, facing the "
                    "way it faced then, then to press Resume on the robot. Do not try to move it before.")
        c, r, f = self.back
        return (f"Stop and ask an adult to put the robot back on cell {name_of(c, r)}, facing {FACING[f]}, then to press "
                "Resume on the robot. Do not try to move it before.")

    def interrupted(self, back):
        self.sure, self.back = False, back
        self.coverage["a move stopped: an adult must come"] += 1
        return {"result": "stopped", "why": self.rng.choice(["something touched the robot's bumper",
                                                             "someone pressed the emergency stop button on the robot"]),
                "position_known": False, "what_to_do": self.instruction()}

    def waiting(self):
        return self.refused("the robot waits for an adult (a move was interrupted)", what_to_do=self.instruction())

    def no_moves(self):
        return self.limit is not None and self.used >= self.limit

    def no_moves_answer(self):
        return self.refused(f"no moves left: the adults allow {self.limit} moves per mission")

    def busy_answer(self):
        return self.refused("another command is running (an agent, a terminal or the page): wait for it to end")

    def place_found(self):
        if self.frame and self.rng.random() < 0.5:
            self.frame = False
            self.coverage["place found"] += 1

    def see_ball(self):
        cell = self.rng.choice(CELLS)
        how = self.rng.choice(["getting closer to the robot", "moving away from the robot",
                               "at the same distance from the robot", "out of the camera's sight now"])
        t = self.clock.time() - self.rng.uniform(0, 15)
        at = time.strftime("%H:%M:%S")
        if self.frame or not self.defects:  # without its cell: no lane, so the Pilot never leaves one in a leg
            self.ball = {"how": how, "at": at, "t": t}
        else:
            c, r = BASE.cell_xy(cell)
            near = [name_of(c + dc, r + dr) for dc, dr in STEP.values() if on_mat(c + dc, r + dr)]
            self.ball = {"cell": cell, "could_also_be": self.rng.sample(near, self.rng.randint(0, 1)),
                         "moving": self.rng.choice([None, "north", "east", "south", "west"]), "how": how,
                         "seen_from": "C5 facing north", "from": [self.col, self.row, self.facing], "at": at, "t": t}

    def sight(self, card, c, r, d):
        """The camera saw a card d cells away: sure up close, else on its cell or a neighbour, with could_also_be."""
        cell = name_of(c, r)
        if d <= 2:
            where, also = cell, []
        else:
            near = [name_of(c + dc, r + dr) for dc, dr in STEP.values() if on_mat(c + dc, r + dr)]
            also = self.rng.sample(near, min(len(near), self.rng.randint(1, 2)))
            if self.rng.random() < 0.4:
                where, also = also[0], [cell] + also[1:]
            else:
                where = cell
        record = {"card": card, "means": MAT["cards"][card], "cell": where, "could_also_be": also,
                  "distance_cm": d * CELL_CM, "seen_from": f"{self.here()} facing {FACING[self.facing]}",
                  "at": time.strftime("%H:%M:%S")}
        for k, s in enumerate(self.seen):
            if s["card"] == card and (s["cell"] in [where] + also or where in [s["cell"]] + s["could_also_be"]):
                if record["distance_cm"] <= s["distance_cm"]:
                    self.seen[k] = record
                return
        self.seen.append(record)

    def camera(self):
        """What the camera sees, facing this way: cards up to 4 cells ahead, wider when farther."""
        saw = []
        dc, dr = FACING_STEP[self.facing]
        for d in range(1, 5):
            for side in (-1, 0, 1) if d >= 2 else (0,):
                c = self.col + dc * d + (side if dr else 0)
                r = self.row + dr * d + (side if dc else 0)
                if not on_mat(c, r):
                    continue
                self.looked.add(name_of(c, r))
                card = self.truth.get(name_of(c, r))
                if card and self.rng.random() < 0.75 and (not self.frame or self.defects):  # cell null while unknown
                    self.sight(card, c, r, d)
                    saw.append(f"the camera recognised a card in front: {card}")
        if self.rng.random() < 0.03 and not self.frame:
            c, r = BASE.cell_xy(self.rng.choice(CELLS))
            self.sight(self.rng.choice(CARDS), c, r, 4)  # a card that is not there: the camera can be wrong
        return saw

    # The Pilot's commands ----------------------------------------------------------------------------------------------
    def robot_state(self):
        said = self.enter("robot_state")
        self.clock.advance(0.2)
        return said or self.state()

    def seen_cards(self):
        said = self.enter("seen_cards")
        if said:
            return said
        cards = [dict(s) for s in self.seen]
        if self.frame:
            cards = [dict(c, cell=None, could_also_be=[], where="not known yet: the Cartographer does not know yet "
                                                              "where the robot is on the mat") for c in cards]
        left = sorted(set(CELLS) - self.looked)
        return {"cards_seen": cards,
                "camera_coverage": {"cells_looked_at": None if self.frame else len(self.looked),
                                    "cells_on_the_mat": len(CELLS), "not_looked_at_yet": None if self.frame else left,
                                    "something_stands_there": [] if self.frame else sorted(self.boxes & self.looked),
                                    "note": "the cells the camera has looked at"},
                "note": "The camera sees up to 120 cm ahead."}

    def set_light(self, color):
        said = self.enter("set_light", color=color)
        if said:
            return said
        if self.busy:
            return self.busy_answer()
        self.clock.advance(0.1)
        return {"result": "done"} if self.rng.random() < 0.97 else self.refused("the robot did not understand the command")

    def step_forward(self):
        said = self.enter("step_forward")
        if said:
            return said
        if not self.sure:
            return self.waiting()
        if self.busy:
            return self.busy_answer()
        if self.no_moves():
            return self.no_moves_answer()
        self.clock.advance(2)
        ahead = self.ahead()
        if self.frame:
            if ahead is None or not self.free_cell(*ahead) or self.rng.random() < 0.3:
                return self.refused("the Cartographer does not know yet where the robot is on the mat: it goes one cell "
                                    "forwards only where its distance sensor sees the way free beyond that cell",
                                    what_to_do="turn towards a free way (look_around tells which), then step_forward")
            self.col, self.row = ahead
            self.used += 1
            self.place_found()
            return {"result": "done", "state": self.state()}
        if ahead is None:
            return self.refused("the next cell is outside the mat")
        if name_of(*ahead) in self.truth:
            return self.refused(f"a card is on the next cell ({name_of(*ahead)}): the robot never drives onto a card",
                                what_to_do="stop here if it is the one you look for, or go round it")
        if name_of(*ahead) in self.boxes or self.rng.random() < 0.05:
            return self.refused("something is in front of the robot")
        if self.rng.random() < 0.05:
            return self.interrupted([self.col, self.row, self.facing])
        self.col, self.row = ahead
        self.used += 1
        return {"result": "done", "state": self.state()}

    def turn(self, direction, counted=True):
        said = self.enter("turn", direction=direction)
        if said:
            return said
        return self.quarter(1 if direction == "right" else -1, counted)

    def quarter(self, way, counted):
        if not self.sure:
            return self.waiting()
        if self.busy:
            return self.busy_answer()
        if counted and self.no_moves():
            return self.no_moves_answer()
        self.clock.advance(1)
        r = self.rng.random()
        if r < 0.03:
            return self.interrupted([self.col, self.row, self.facing])
        if r < 0.06 and (counted or self.defects):  # a turn refused after a first one: the facing has changed
            return self.refused("the robot is still moving")
        self.facing = (self.facing + way) % 4
        self.used += counted
        return {"result": "done", "state": self.state()}

    def move(self, direction, cells, stop_if):
        said = self.enter("move", direction=direction, cells=cells, stop_if=stop_if)
        if said:
            return said
        if not self.sure:
            return self.waiting()
        if self.frame:
            return self.refused("the Cartographer does not know yet where the robot is on the mat, so north, east, south "
                                "and west mean nothing yet", what_to_do="look_around: the distance sensor sees the "
                                "border of the mat, and the camera the landmarks")
        if self.busy:
            return self.busy_answer()
        dc, dr = STEP[direction]
        way = [(self.col + dc * k, self.row + dr * k) for k in range(1, cells + 1)
               if on_mat(self.col + dc * k, self.row + dr * k)]
        if not way:
            return self.refused("the next cell is outside the mat")
        if self.no_moves():
            return self.no_moves_answer()
        sideways = FACING.index(direction) != self.facing
        r = self.rng.random()
        if r < 0.15:
            first = name_of(*way[0])
            return self.refused(self.rng.choice([
                f"the robot only sees in front of it ({FACING[self.facing]}): it slides {direction} only into cells its "
                f"distance sensor saw free in the last 2 minutes, and {first} is not one" if sideways else
                f"a ball was seen near {first} less than {RECENT_S} s ago",
                f"{first} is on the lane of a ball, and the robot cannot see {direction}",
                "another move is still running: send one move at a time, and read its answer before the next",
                "something is in front of the robot"]), what_to_do="look_around, then move again")
        moved, why = 0, "it moved all the cells asked"
        for k, (c, r) in enumerate(way):
            if name_of(c, r) in self.boxes:
                why = "something was in the way: it waited, then stayed on the last cell it reached"
                break
            if name_of(c, r) in self.truth:
                why = f"it stopped before {name_of(c, r)}, where the camera saw a card: it never drives onto a card"
                break
            if k > 0:
                self.tick(0.08)
                if self.ask_hold():
                    why = "the mission was paused or stopped: it stopped on the next cell"
                    self.coverage["a slide held on the next cell"] += 1
                    self.col, self.row = c, r
                    moved = k + 1
                    break
            self.clock.advance(1.5)
            if self.rng.random() < 0.04:
                self.used += moved > 0
                result = self.interrupted([self.col, self.row, self.facing])
                result["cells_moved"] = moved
                return result
            self.col, self.row = c, r
            moved = k + 1
        else:
            if len(way) < cells:
                why = f"the mat ends after {len(way)} cell{'s' if len(way) > 1 else ''} that way"
        self.used += moved > 0
        return {"result": "done", "cells_moved": moved, "cells_asked": cells, "stopped_because": why,
                "state": self.state()}

    def look_around(self):
        said = self.enter("look_around")
        if said:
            return said
        if not self.sure:
            return self.waiting()
        if self.busy:
            return self.busy_answer()
        if self.no_moves():
            return self.no_moves_answer()
        looked = []
        for n in range(4):
            if n > 0:
                self.tick(0.05)
                if self.ask_hold():
                    self.coverage["a look_around paused"] += 1
                    return {"result": "paused", "why": "the mission was paused or stopped: the robot stopped turning",
                            "looked_before_it_stopped": looked, "state": self.state()}
            result = self.quarter(1, n == 0)
            if result.get("result") != "done":
                result["looked_before_it_stopped"] = looked
                return result
            self.clock.advance(0.5)
            state = result["state"]
            looked.append({"facing": FACING[self.facing], "card_ahead": state["card_ahead"],
                           "distance_ahead_cm": state["distance_ahead_cm"], "what_the_camera_saw": self.camera()})
        self.place_found()
        return {"result": "done", "looked": looked, "state": self.state()}

    def face(self, direction):
        said = self.enter("face", direction=direction)
        if said:
            return said
        if not self.sure:
            return self.waiting()
        if self.frame:
            return self.refused("the Cartographer does not know yet where the robot is on the mat, so north, east, south "
                                "and west mean nothing yet", what_to_do="look_around")
        turns = {0: [], 1: [1], 2: [1, 1], 3: [-1]}[(FACING.index(direction) - self.facing) % 4]
        result = {"result": "done", "state": self.state()}
        for n, way in enumerate(turns):
            if n > 0:
                self.tick(0.05)
                if self.ask_hold():
                    return {"result": "paused", "why": "the mission was paused or stopped: the robot stopped turning",
                            "state": self.state()}
            result = self.quarter(way, n == 0)
            if result.get("result") != "done":
                return result
        return result

    def wait_for(self, seconds):
        self.spend()
        if self.inside:
            self.broke("one call to the Cartographer at a time", "wait_for inside a call")
        self.coverage["cart wait_for"] += 1
        if self.op is not None:
            self.op["calls"].append(["wait_for", {"seconds": seconds}])
        why = cart_language("wait_for", {"seconds": seconds})
        if why:
            self.broke("the Cartographer gets only values of its language", ["wait_for", seconds, why])
            return self.answer({"result": "error", "why": "the robot's answer could not be read (TypeError)"})
        if not self.sure and self.adult_comes and self.rng.random() < 0.35:
            waited = self.rng.uniform(1, seconds)
            self.clock.advance(waited)
            self.sure = True
            self.col, self.row, self.facing = self.back
            self.coverage["an adult pressed Resume"] += 1
            return self.answer({"result": "something happened", "waited_seconds": round(waited, 1),
                                "events": [{"seconds_before_this_answer": 0.3,
                                            "event": "an adult pressed Resume on the robot"}]})
        if self.rng.random() < 0.15:
            waited = self.rng.uniform(0.3, seconds)
            self.clock.advance(waited)
            self.see_ball()
            return self.answer({"result": "something happened", "waited_seconds": round(waited, 1),
                                "events": [{"seconds_before_this_answer": 0.2, "event": "the camera sees a ball"}]})
        self.clock.advance(seconds)
        if self.mission.now() == "go":
            self.tick(0.03)
        return self.answer({"result": "nothing happened", "waited_seconds": seconds})


FACING_STEP = common.FACING_STEP


def cart_language(name, args):
    """Why a call is outside the Cartographer's language (facilitator/ROBOT-LANGAGES.md, section 4), or None."""
    def whole(v, low, high):
        return isinstance(v, int) and not isinstance(v, bool) and low <= v <= high
    if name in ("move", "face") and args.get("direction") not in FACING:
        return "direction"
    if name == "move" and not whole(args.get("cells"), 1, MAX_CELLS):
        return "cells"
    if name == "move" and (not isinstance(args.get("stop_if"), list) or any(s not in STOP_IF for s in args["stop_if"])
                           or len(set(args["stop_if"])) != len(args["stop_if"])):
        return "stop_if"
    if name == "turn" and args.get("direction") not in ("left", "right"):
        return "direction"
    if name == "set_light" and args.get("color") not in COLORS:
        return "color"
    if name == "wait_for" and not whole(args.get("seconds"), 1, 30):
        return "seconds"
    return None


# --- the language: the door, and the spec ------------------------------------------------------------------------------
def schema_says(props, required, args):
    """Why arguments do not fit a tool's schema of TOOLSETS, or None: what the Strategist (or an MCP server) checks."""
    if not isinstance(args, dict):
        return "the arguments are not a JSON object"
    for key in required:
        if key not in args:
            return f"missing argument {key}"
    for key, value in args.items():
        prop = props.get(key)
        if prop is None:
            return f"unknown argument {key}"
        if prop["type"] == "string" and (not isinstance(value, str) or "enum" in prop and value not in prop["enum"]):
            return f"{key} is not one of its values"
        if prop["type"] == "integer" and (not isinstance(value, int) or isinstance(value, bool)
                                          or not prop.get("minimum", -1e9) <= value <= prop.get("maximum", 1e9)):
            return f"{key} is not a whole number in its range"
        if prop["type"] == "array" and (not isinstance(value, list) or any(v not in prop["items"]["enum"] for v in value)):
            return f"{key} is not a list of its values"
    return None


# The Pilot's language, written from facilitator/ROBOT-LANGAGES.md (section 5), not from TOOLSETS: tool -> (arguments,
# required). The values are closed: a card of mat.json, a cell of the mat, a direction, a colour, 1 to 30 seconds.
SPEC = {
    "goal": {"go_to_card": ({"card"}, {"card"}), "explore": ({"look_for"}, set()), "robot_state": (set(), set()),
             "seen_cards": (set(), set()), "set_light": ({"color"}, {"color"})},
    "route": {"go_to_cell": ({"cell"}, {"cell"}), "look_around": (set(), set()), "face": ({"direction"}, {"direction"}),
              "wait_for_event": ({"seconds"}, set()), "robot_state": (set(), set()), "seen_cards": (set(), set()),
              "set_light": ({"color"}, {"color"})},
    "step": {"step_forward": (set(), set()), "move": ({"direction", "cells", "stop_if"}, {"direction"}),
             "turn": ({"direction"}, {"direction"}), "look_around": (set(), set()),
             "face": ({"direction"}, {"direction"}), "set_light": ({"color"}, {"color"}),
             "wait_for_event": ({"seconds"}, set()), "seen_cards": (set(), set()), "robot_state": (set(), set())},
}


def language_says(toolset, name, args):
    """Why a call of a tool of the toolset is outside the Pilot's language, or None."""
    allowed, required = SPEC[toolset][name]
    if not isinstance(args, dict):
        return "not an object"
    if set(args) - allowed or required - set(args):
        return "arguments"
    for key, value in args.items():
        if key in ("card", "look_for") and value not in CARDS or key == "cell" and value not in CELLS or \
                key == "color" and value not in COLORS or \
                key == "direction" and value not in (("left", "right") if name == "turn" else FACING):
            return key
        if key in ("seconds", "cells") and (not isinstance(value, int) or isinstance(value, bool)
                                            or not 1 <= value <= (30 if key == "seconds" else MAX_CELLS)):
            return key
        if key == "stop_if" and (not isinstance(value, list) or any(v not in STOP_IF for v in value)):
            return key
    return None


def parse(line):
    """A line, as a call: (name, arguments), or None when it is not a name followed by JSON."""
    m = re.match(r"^\s*([A-Za-z_]\w*)\s*(.*?)\s*$", line)
    if not m:
        return None
    name, rest = m.groups()
    if not rest:
        return name, {}
    try:
        return name, json.loads(rest)
    except ValueError:
        return None


# --- the generators -------------------------------------------------------------------------------------------------------
def valid_args(rng, name):
    if name == "go_to_card":
        return {"card": rng.choice(CARDS)}
    if name == "explore":
        return {} if rng.random() < 0.4 else {"look_for": rng.choice(CARDS)}
    if name == "go_to_cell":
        return {"cell": rng.choice(CELLS)}
    if name == "face":
        return {"direction": rng.choice(FACING)}
    if name == "turn":
        return {"direction": rng.choice(["left", "right"])}
    if name == "set_light":
        return {"color": rng.choice(COLORS)}
    if name == "wait_for_event":
        return {} if rng.random() < 0.3 else {"seconds": rng.randint(1, 30)}
    if name == "move":
        args = {"direction": rng.choice(FACING)}
        if rng.random() < 0.8:
            args["cells"] = rng.randint(1, MAX_CELLS)
        if rng.random() < 0.5:
            args["stop_if"] = rng.sample(STOP_IF, rng.randint(0, len(STOP_IF)))
            if args["stop_if"] and rng.random() < 0.2:
                args["stop_if"].append(args["stop_if"][0])  # a duplicate: in the schema, removed by the Pilot
        return args
    return {}


BAD = {"card": ["unicorn", "Barbecue", ""], "look_for": ["nothing", 3], "cell": ["Z9", "A7", "I1", "a1", "A0", "C", ""],
       "direction": ["up", "North", "left", 2], "color": ["purple", "Red", None], "seconds": [0, 31, "10", True, 2.5],
       "cells": [0, 5, 9, -1, "2", True, 2.5], "stop_if": [["wall"], "card_ahead", [1]]}


def random_line(rng, persona, toolset):
    names = list(BASE.TOOLSETS[toolset])
    if persona == "careless" and rng.random() < 0.4:
        if rng.random() < 0.25:
            return rng.choice(["jump", 'report {"outcome": "done"}', "go_to_cell {cell: C3}", 'move {"direction": "north"',
                               "", "step_forward()", 'go_to {"cell": "C3"}', "look around", "face north",
                               "robot_state []", "step_forward", 'go_to_card {"card": "barbecue"}', "explore"])
        name = rng.choice(names)
        args = valid_args(rng, name)
        if name == "move" and rng.random() < 0.6:
            args["cells"] = rng.choice(BAD["cells"])
        elif args:
            key = rng.choice(sorted(args))
            args[key] = rng.choice(BAD[key])
        elif SPEC[toolset][name][0]:
            key = rng.choice(sorted(SPEC[toolset][name][0]))
            if key in SPEC[toolset][name][1] or rng.random() < 0.5:
                args = {} if key in SPEC[toolset][name][1] else {key: rng.choice(BAD[key])}
            else:
                args = {key: rng.choice(BAD[key])}
            if not args:
                args = {"speed": 2} if rng.random() < 0.5 else {}
        else:
            args = {"speed": rng.choice([2, "fast"])}
        return name + (" " + json.dumps(args) if args else "")
    name = rng.choice(names)
    args = valid_args(rng, name)
    return name + (" " + json.dumps(args) if args else "")


# --- configure, as the Strategist calls it at each mission -----------------------------------------------------------------
def draw_settings(rng):
    r = rng.random()
    if r < 0.3:
        return {}
    good = {"careful_balls": [True, False], "max_leg": [1, 2, 3, 4, 0, 9], "look_each_leg": [True, False]}
    settings = {k: rng.choice(v) for k, v in good.items() if rng.random() < 0.6}
    if r > 0.8:
        wrong = rng.choice([("x", 1), ("speed", 3), ("careful_balls", "no"), ("careful_balls", 1), ("max_leg", True),
                            ("max_leg", "2"), ("max_leg", 2.5), ("look_each_leg", 0), ("look_each_leg", None)])
        settings = dict(settings)
        settings.pop(wrong[0], None)
        if rng.random() < 0.5:
            settings = dict([wrong] + list(settings.items()))
        else:
            settings[wrong[0]] = wrong[1]
    return settings


def settings_wrong(settings):
    """The first setting of the call outside the language (unknown, or of the wrong type), or None."""
    for key, value in settings.items():
        if key not in DEFAULTS or isinstance(value, bool) != isinstance(DEFAULTS[key], bool) or \
                not isinstance(value, type(DEFAULTS[key])):
            return key
    return None


def configure(pilot, settings, expected, broken):
    """configure(), checked: the settings expected after it, or the ones before when it must refuse."""
    def broke(name, detail):
        broken.append({"invariant": name, "detail": detail})
    before = dict(pilot.PILOT.settings)
    wrong = settings_wrong(settings)
    try:
        pilot.PILOT.configure(copy.deepcopy(settings))
    except ValueError as e:
        if wrong is None:
            broke("configure takes every setting of the language", [settings, str(e)])
        elif not str(e).startswith((f"no setting {wrong} ", f"the setting {wrong} ")):
            broke("configure names the first wrong setting", [settings, str(e)])
        if pilot.PILOT.settings != before:
            broke("configure refuses before changing anything", [settings, before, pilot.PILOT.settings])
        return expected, "refused"
    if wrong is not None:
        broke("configure refuses an unknown setting or a value of the wrong type", [settings, pilot.PILOT.settings])
        return dict(pilot.PILOT.settings), "taken (wrong)"
    want = dict(DEFAULTS, **settings)
    want["max_leg"] = max(1, min(MAX_CELLS, want["max_leg"]))
    if pilot.PILOT.settings != want:
        broke("configure starts from the defaults, then the settings given (max_leg between 1 and 4)",
              [settings, before, pilot.PILOT.settings, want])
    return want, "taken"


# --- the invariants after a tool -----------------------------------------------------------------------------------------
def check_call(toolset, name, args, fn, result, error, op, counted, log, fake, mission, broken):
    def broke(invariant, detail):
        broken.append({"invariant": invariant, "detail": detail})
    calls, answers = op["calls"], op["answers"]
    if counted != ["operations"]:
        broke("every tool asked is counted once as an operation, and the Pilot counts nothing else", counted)
    stopped = mission.obey and mission.now() == "stop"
    if error is not None:
        if not (error == "Stuck: the mission was stopped" and toolset == "step" and stopped):
            broke("no error escapes a tool (but a step tool that meets a stop)", error)
        return
    if not isinstance(result, dict) or not isinstance(result.get("result", ""), str):
        broke("a tool answers a dictionary", result)
        return
    logged = fn.__name__ in LOGGED
    lines = [l["tool"] for l in log]
    want = [fn.__name__] if logged else []
    if logged and fn.__name__ == "go_to_card":
        lines = [l for l in lines if l != "explore"]
    if lines != want or logged and log and log[-1]["tool"] != fn.__name__:
        broke("pilot.jsonl: one line per goal or route tool (not wait_for_event nor seen_cards)", [fn.__name__,
                                                                                                    [l["tool"] for l in log]])
    gave_up = result.get("result") == "gave up"
    if logged:
        expected = fake.read_moves_left
        if "moves_left" not in result or result["moves_left"] != expected:
            broke("a goal or route tool answers the moves left as last read (null without a limit)",
                  [result.get("moves_left"), expected])
    if gave_up:
        why = str(result.get("why", ""))
        if not why.startswith(GIVE_UP) or set(result) - {"result", "why", "cards_seen", "moves_left"}:
            broke("gave up: a reason of the closed list, and cards_seen", result)
        if why == "no moves left" and not (isinstance(fake.read_moves_left, int) and fake.read_moves_left <= 1):
            broke("no moves left only under a limit of the adults", [why, fake.read_moves_left])
        if why == "the mission was stopped" and not stopped:
            broke("the mission was stopped only after a stop", why)
        return
    if toolset == "step":
        cart = PASSED[name]
        cells = args.get("cells", 1)
        if name == "move" and (not isinstance(cells, int) or isinstance(cells, bool) or not 1 <= cells <= MAX_CELLS):
            return  # the Pilot's own check (check_raw)
        sent = {"move": {"direction": args.get("direction"), "cells": args.get("cells", 1),
                         "stop_if": list(dict.fromkeys(args.get("stop_if") or []))},
                "turn": {"direction": args.get("direction")}, "face": {"direction": args.get("direction")},
                "set_light": {"color": args.get("color")},
                "wait_for": {"seconds": max(1, min(30, int(args.get("seconds", 10))))}}.get(cart, {})
        if [c[0] for c in calls] != [cart] or calls[0][1] != sent:
            broke("a step tool is passed to the Cartographer as it is: one call, the same arguments", [name, args, calls])
        elif answers and result != answers[-1]:
            broke("a step tool answers what the Cartographer answered, as it is", [result, answers[-1]])
        return
    if fn.__name__ in ("watch", "cards", "state", "light"):
        cart = {"watch": "wait_for", "cards": "seen_cards", "state": "robot_state", "light": "set_light"}[fn.__name__]
        same = dict(answers[-1], moves_left=result.get("moves_left")) if answers and fn.__name__ in ("state", "light") \
            else answers[-1] if answers else None
        if [c[0] for c in calls] != [cart] or result != same:
            broke("wait_for_event, seen_cards, robot_state and set_light answer the Cartographer's answer as it is",
                  [name, calls, result])
        return
    outcome = result.get("result")
    if fn.__name__ == "go_to_cell":
        if args["cell"] in op["cards_at_start"]:
            if outcome != "refused" or any(c[0] in CART_MOVES for c in calls):
                broke("go_to_cell never drives to a card it knows: refused, nothing moved", [args, result.get("result")])
        if outcome not in ("done", "not there", "refused"):
            broke("go_to_cell answers done, not there or refused", outcome)
        elif outcome == "done" and (result.get("cell") != args["cell"] or fake.here() != args["cell"]):
            broke("go_to_cell done: the robot is on that cell", [args, result.get("cell"), fake.here()])
        elif outcome != "refused" and not ({"cell", "facing", "state"} <= set(result)):
            broke("go_to_cell answers the cell, the way it faces and the state", result)
    elif fn.__name__ == "face_towards":
        if outcome not in ("done", "not done") or "state" not in result:
            broke("face answers done or not done, with the state", result)
        elif outcome == "done" and FACING[fake.facing] != args["direction"]:
            broke("face done: the robot faces that way", [args, FACING[fake.facing]])
    elif fn.__name__ == "go_to_card":
        if outcome == "done":
            ahead = fake.ahead()
            if not ({"cell", "facing", "card_ahead"} <= set(result)) or not ahead or \
                    fake.truth.get(name_of(*ahead)) != args["card"] or result.get("cell") != fake.here():
                broke("go_to_card done: the card is right in front of the robot", [args, result, fake.here(), ahead])
        elif outcome != "not found" or "why" not in result:
            broke("go_to_card answers done, not found or gave up", result)
    elif fn.__name__ == "explore":
        look_for = args.get("look_for")
        cards = result.get("cards_seen")
        if outcome not in ("found", "not found", "done") or not isinstance(cards, dict) or \
                not all(isinstance(v, list) and all(isinstance(c, str) for c in v) for v in cards.values()) or \
                not isinstance(result.get("not_sure"), list) or not isinstance(result.get("mat_covered"), bool):
            broke("explore answers found, not found or done, the cards seen, not_sure and mat_covered", result)
        elif (outcome == "found") != (look_for is not None and look_for in cards) or \
                (outcome == "done") != (look_for is None):
            broke("explore: found when the card looked for is among the cards seen, done without one", [args, result])


def check_raw(name, args, result, op, broken):
    """A call outside the schema that reached the Pilot as typed by hand: its own checks refuse it, nothing sent."""
    if not isinstance(result, dict) or result.get("result") != "refused" or op["calls"]:
        broken.append({"invariant": "a move of a wrong number of cells, or a cell off the mat, is refused by the Pilot "
                                    "itself, nothing sent", "detail": [name, args, result, op["calls"]]})
    elif name == "move" and not result.get("why", "").endswith("nothing was sent"):
        broken.append({"invariant": "a refused move says nothing was sent", "detail": result})


# --- one run ------------------------------------------------------------------------------------------------------------------
def run(seed, generator, persona, steps, replay=None, patch=None, defects=True):
    """One run. defects=False leaves out the situations of the defects this probe found (a place lost on the way, a
    card seen while the place is unknown, the robot driven by hand between two missions, a ball's lane, which a leg
    with look_each_leg may leave first, a quarter turn refused after a first one): the mutants are counted against a
    Pilot that breaks nothing."""
    rng, pace = random.Random(seed), random.Random(seed * 31 + 7)
    world, setup = random.Random(seed * 7919 + 1), random.Random(seed * 13 + 5)
    for f in STATE.iterdir():
        if f.is_file():
            f.unlink()
    pilot = load_fresh()
    if patch:
        patch(pilot)
    clock = Clock()
    pilot.time = clock
    obey = setup.random() < 0.85
    toolset = setup.choice(["goal", "route", "route", "step"]) if obey else "step"
    if replay is not None:
        toolset = replay.get("toolset", toolset)
    broken, sent, answers, history = [], [], [], []
    mission = Mission(clock, obey)
    fake = Cartographer(world, clock, mission, broken, pilot, toolset, defects)
    fake.hold_wanted = pilot.CART.hold_wanted  # the Pilot's own rule, set on its Cartographer at import
    pilot.CART = fake
    frame_at_start = fake.frame
    proxy = Common(mission, fake, broken)
    pilot.common = proxy
    expected = dict(DEFAULTS)
    missions = collections.Counter()
    error = None

    def begin_mission(first):
        nonlocal expected
        mission.control, mission.until = "go", None
        fake.used = 0
        fake.saw_pause = fake.acked = fake.cleared = False
        common.say_held(None)
        if not first and toolset != "step" and setup.random() < 0.5 and defects:
            fake.hand_move()
        settings = draw_settings(setup)
        # The rules the Pilot reads are the fake's (its limit of moves); configure starts each mission from them.
        pilot.RULES = dict(pilot.RULES, limits={"moves": fake.limit} if fake.limit is not None else {})
        expected, how = configure(pilot, settings, expected, broken)
        fake.read_moves_left = fake.limit  # configure forgets the count read before, refused or not
        missions["configure " + how] += 1
        missions["missions"] += 1
        pilot.OBEY_MISSION_CONTROL = True

    if obey:
        begin_mission(True)
    elif setup.random() < 0.5:
        mission.pause(10 ** 6, "go")  # drive.py, with a pause left written in control.json
    for n in range(steps):
        if replay is not None:
            if n >= len(replay["sent"]):
                break
            line = replay["sent"][n]
        elif generator == "random":
            line = random_line(rng, persona, toolset)
        else:
            try:
                line = probe_tiers.ollama_line(generator[len("ollama:"):], persona, history, seed * 1000 + n,
                                               system=PERSONAS[persona] + "\n\nThe language:\n\n" + language(toolset))
            except (OSError, ValueError, KeyError) as e:
                error = f"model: {type(e).__name__}: {e}"
                break
        sent.append(line)
        history.append("> " + line)
        call = parse(line)
        if call is None or call[0] not in pilot.TOOLSETS[toolset]:
            answers.append("not the language")
            history.append("< " + ("no tool named " + call[0] if call else "not understood: a tool name, then its "
                                                                               "arguments as a JSON object"))
            continue
        name, args = call
        fn, _, props, required = pilot.TOOLSETS[toolset][name]
        says, spec = schema_says(props, required, args), language_says(toolset, name, args)
        if (says is None) != (spec is None):
            broken.append({"invariant": "the tool's schema holds the closed values of the language",
                           "detail": [name, args, says, spec]})
        raw = says is not None and isinstance(args, dict) and pace.random() < 0.5 and (
            name == "move" and language_says(toolset, name, dict(args, cells=1)) is None or
            name == "go_to_cell" and set(args) == {"cell"} and isinstance(args["cell"], str))
        if says is not None and not raw:
            answers.append("invalid")
            history.append("< " + json.dumps({"result": "refused", "why": f"invalid arguments: {says}: nothing was sent"}))
            continue
        if obey and not raw:
            pilot.hold_still()  # the Strategist's loop, between two tools
            if mission.now() == "stop":
                answers.append("refused (the mission was stopped)")
                history.append("< " + json.dumps({"result": "refused", "why": "the mission was stopped"}))
                history.append("< a new mission begins")
                begin_mission(False)
                continue
        fake.begin(toolset, name, expected)
        fake.op["cards_at_start"] = set() if fake.frame else {s["cell"] for s in fake.seen}
        proxy.counted = []
        log_before = len(_log())
        result, failed = None, None
        try:
            result = pilot.operate(fn, **args)
        except ProbeLimit:
            broken.append({"invariant": "a tool ends (calls to the Cartographer and time bounded)",
                           "detail": [name, len(fake.op["calls"]), round(clock.monotonic() - fake.op["start"])]})
            failed = "probe limit"
        except Exception as e:  # noqa: BLE001  (the Strategist turns it into an error for the model)
            failed = f"{type(e).__name__}: {e}"
            if not isinstance(e, pilot.Stuck):
                failed += " (at " + " < ".join(f"{Path(w.filename).name}:{w.lineno} {w.line}" for w in
                                              traceback.extract_tb(e.__traceback__)[:0:-1][:3]) + ")"
        if raw:
            check_raw(name, args, result, fake.op, broken)
            answers.append("raw " + str((result or {}).get("result")))
        else:
            if failed != "probe limit":
                check_call(toolset, name, args, fn, result, failed, fake.op, proxy.counted, _log()[log_before:], fake,
                           mission, broken)
            answers.append(("error " + failed.split(":")[0]) if failed else str(result.get("result", "answer")))
        history.append("< " + (json.dumps({"result": "error", "why": failed}) if failed else
                               json.dumps(result, ensure_ascii=False)[:300]))
        fake.op = None
        if obey and setup.random() < 0.06:
            history.append("< a new mission begins")
            begin_mission(False)  # the model reported: the Interpreter starts the next mission
        for b in broken:
            b.setdefault("line", line)
            b.setdefault("step", n)
    coverage = collections.Counter(fake.coverage)
    coverage.update(missions)
    coverage["pauses pressed"] += mission.pauses
    coverage["stops pressed"] += mission.stops
    coverage["pauses acknowledged"] += proxy.acks
    return {"seed": seed, "sent": sent, "answers": answers, "broken": broken, "error": error, "toolset": toolset,
            "mission": obey, "limit": fake.limit, "frame_at_start": frame_at_start, "defects": defects,
            "coverage": dict(coverage)}


def _log():
    path = STATE / "pilot.jsonl"
    return [json.loads(l) for l in path.read_text().splitlines()] if path.exists() else []


def summary(results, generator, persona):
    lines = [l for r in results for l in r["sent"]]
    words = collections.Counter(a for r in results for a in r["answers"])
    reached = sum(v for k, v in words.items() if k not in ("not the language", "invalid") and not k.startswith("refused ("))
    coverage = collections.Counter()
    for r in results:
        coverage.update(r["coverage"])
        coverage[f"toolset {r['toolset']}" + ("" if r["mission"] else " (drive.py)")] += 1
        coverage["runs with a limit of moves" if r["limit"] is not None else "runs without a limit of moves"] += 1
        for line, a in zip(r["sent"], r["answers"]):
            call = parse(line)
            if call and a not in ("not the language", "invalid"):
                coverage[f"{call[0]}: {a}"] += 1
    return {"tier": "pilot", "generator": generator, "persona": persona, "runs": len(results), "lines_sent": len(lines),
            "answers": dict(words.most_common()),
            "accepted_share": round(reached / len(lines), 3) if lines else None,
            "not_understood_share": round(words["not the language"] / len(lines), 3) if lines else None,
            "invalid_share": round(words["invalid"] / len(lines), 3) if lines else None,
            "coverage": dict(sorted(coverage.items())),
            "commonest_not_understood": collections.Counter(l for r in results for l, a in zip(r["sent"], r["answers"])
                                                            if a in ("not the language", "invalid")).most_common(8),
            "runs_breaking_an_invariant": sum(1 for r in results if r["broken"]),
            "invariants_broken": dict(collections.Counter(b["invariant"] for r in results for b in r["broken"])),
            "model_errors": [r["error"] for r in results if r["error"]]}


# --- the probe checks itself: one rule of the Pilot removed at a time ---------------------------------------------------------
def mutants():
    def tool(m, toolset, name, fn):
        _, ro, props, req = m.TOOLSETS[toolset][name]
        m.TOOLSETS[toolset][name] = (fn, ro, props, req)

    def loose_schemas(m):
        for toolset in m.TOOLSETS.values():
            for name, (fn, ro, props, req) in list(toolset.items()):
                props = {k: dict(v) for k, v in props.items()}
                for key, prop in props.items():
                    if key == "cells":
                        prop["maximum"] = 9
                    elif key == "seconds":
                        prop["maximum"] = 99
                    elif "enum" in prop:
                        prop["enum"] = list(prop["enum"]) + ["up", "purple", "Z9", "unicorn"]
                toolset[name] = (fn, ro, props, req)

    def no_cells_check(m):
        def move(direction, cells=1, stop_if=None):
            return m.command("move", direction, cells, list(dict.fromkeys(stop_if or [])))
        tool(m, "step", "move", move)

    def moves_as_reads(m):
        command = m.command
        def patched(name, *args):
            if name == "wait_for_event":
                return command(name, *args)
            if m.OBEY_MISSION_CONTROL and name in m.MOVES | {"set_light"}:
                m.hold_still()
                if m.common.mission_control() == "stop":
                    raise m.Stuck("the mission was stopped")
            return m.CART.call(lambda: getattr(m.CART, name)(*args), is_move=False)
        m.command = patched

    def pause_ignored(m):
        def patched(name, *args):
            if name == "wait_for_event":
                return m.CART.wait_for(*args)
            moves = {"step_forward", "move", "turn", "look_around", "face", "set_light"}
            return m.CART.call(lambda: getattr(m.CART, name)(*args), is_move=name in moves)
        m.command = patched

    def no_hold(m): m.CART.hold_wanted = lambda: False

    def uncounted(m): m.operate = lambda fn, **arguments: fn(**arguments)

    def keeps_settings(m):
        configure = m.Pilot.configure
        def patched(self, settings):
            old = dict(self.settings)
            configure(self, settings)
            self.settings = dict(old, **(settings or {}))
            self.settings["max_leg"] = max(1, min(m.MAX_CELLS, self.settings["max_leg"]))
        m.Pilot.configure = patched

    def changes_first(m):
        configure = m.Pilot.configure
        def patched(self, settings):
            for key, value in (settings or {}).items():
                if key in m.SETTINGS and isinstance(value, type(m.SETTINGS[key]["default"])):
                    self.settings[key] = value
            configure(self, settings)
        m.Pilot.configure = patched

    def none_is_zero(m): m.Pilot.short = lambda self, n: self.moves_left is None or self.moves_left <= n

    def balls_ignored(m):
        leg = m.Pilot.leg
        def patched(self, direction, cells):
            saved = self.settings["careful_balls"]
            self.settings["careful_balls"] = False
            try:
                return leg(self, direction, cells)
            finally:
                self.settings["careful_balls"] = saved
        m.Pilot.leg = patched

    def not_acknowledged(m):
        def patched():
            while m.common.mission_control() == "pause":
                m.time.sleep(0.5)
        m.hold_still = patched

    def cards_ignored(m):
        m.Pilot.card_cells = lambda self: set()

    def long_legs(m):
        travel = m.Pilot.travel
        def patched(self, goals):
            saved = self.settings["max_leg"]
            self.settings["max_leg"] = m.MAX_CELLS
            try:
                return travel(self, goals)
            finally:
                self.settings["max_leg"] = saved
        m.Pilot.travel = patched

    def no_look(m):
        travel = m.Pilot.travel
        def patched(self, goals):
            saved = self.settings["look_each_leg"]
            self.settings["look_each_leg"] = False
            try:
                return travel(self, goals)
            finally:
                self.settings["look_each_leg"] = saved
        m.Pilot.travel = patched

    return {"schemas loosened": loose_schemas, "move without its own cells check": no_cells_check,
            "two moves at once (moves go through the door as reads)": moves_as_reads, "pause ignored": pause_ignored,
            "hold_wanted always false": no_hold, "operations not counted": uncounted,
            "configure keeps the settings of the mission before": keeps_settings,
            "configure changes before checking": changes_first, "no limit read as no moves left": none_is_zero,
            "careful_balls ignored": balls_ignored, "pause not acknowledged in held.json": not_acknowledged,
            "cards the Pilot knows not avoided": cards_ignored, "max_leg ignored": long_legs,
            "look_each_leg ignored": no_look}


def main(args):
    if args.generator != "random" and not (args.generator.startswith("ollama:") and len(args.generator) > 7):
        print("--generator is random, or ollama:<model>")
        return 2
    try:
        if args.mutants:
            caught, by = {}, {}
            for name, patch in [("(none: the Pilot as it is)", None)] + list(mutants().items()):
                hits, how = 0, collections.Counter()
                for k in range(args.runs):
                    broken = run(1 + k, "random", "careful" if k % 2 else "careless", args.steps, patch=patch,
                                 defects=False)["broken"]
                    hits += bool(broken)
                    how.update({b["invariant"] for b in broken})
                caught[name], by[name] = hits, dict(how.most_common(3))
            baseline = caught.pop("(none: the Pilot as it is)")
            print(json.dumps({"tier": "pilot", "runs_per_mutant": args.runs, "baseline_runs_breaking": baseline,
                              "runs_that_caught_it": caught, "missed": [k for k, v in caught.items() if not v],
                              "caught_by": by}, indent=1))
            return 1 if baseline or any(not v for v in caught.values()) else 0
        if args.replay:
            saved = json.loads(Path(args.replay).read_text(encoding="utf-8"))
            again = run(saved["seed"], "replay", saved.get("persona", "careful"), len(saved["sent"]), replay=saved,
                        defects=saved.get("defects", True))
            print(json.dumps({"broken": again["broken"],
                              "same_as_saved": [b["invariant"] for b in again["broken"]] ==
                              [b["invariant"] for b in saved["broken"]]}, indent=1))
            return 1 if again["broken"] else 0
        out = Path(args.out or GUIDE / ".robot-sim" / "probes" / time.strftime("pilot-%Y%m%d-%H%M%S"))
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
        (out / "runs.jsonl").write_text("".join(json.dumps({k: r[k] for k in ("seed", "toolset", "sent", "answers",
                                                                               "error")}) + "\n" for r in results),
                                        encoding="utf-8")
        print(json.dumps(report, indent=1))
        return 0
    finally:
        shutil.rmtree(STATE, ignore_errors=True)
