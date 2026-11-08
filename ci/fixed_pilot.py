"""The Pilot alone (stack/pilot.py), in fixed scenarios: the cases P1 to P23 of
facilitator/robot-langages/5-pilote.md that the probe (ci/probe_pilot.py) reaches by chance or never.

Each scenario sets a world by hand on the probe's fake Cartographer, made calm: nothing is drawn at random any more
(no pause, no stop, no ball, no place lost, no move refused, no adult), and the camera sees every card in its cone.
Then it calls one or more tools of the Pilot and checks what they sent below and what they answered, with the exact
strings of pilot.py. The probe's own checks still run inside the fake (a move during a pause, a leg longer than
max_leg...): a scenario that breaks one fails too.

The scenarios, by theme:
    the way and its legs   F1 no moves left, F2 max_leg, F3 the way and its costs, F4 look_each_leg,
                           F16 and F17 two legs that go wrong, F20 pilot.MOVES
    balls                  F5 a ball coming closer, F6 careless and refused, F12 balls' lanes
    cards                  F7a-F7d the surest place first, ruled out for one call, F9a-F9c the 8 attempts and
                           their reason, F18 done only with a place known
    explore                F8a-F8c go_to_card explores first, F15 a limit of the adults, F15b explore without a bound,
                           F21 what is kept
    adult and place        F10a-F10d an adult must come, F11a-F11d the place is not known, F19a-F19c face
    the mission            F13 reads during a pause, F14 configure, F22-F22e a stop or a pause during the Pilot's
                           own waits, F23 a limit of the adults in a goal tool
Each scenario is followed by its mutants: a rule of the Pilot removed, changed by rewrite() (one function or method
of pilot.py, its text changed in place) or by a patch of the loaded module.

Outcomes of a scenario: PASS; FAIL; KNOWN, a defect written down and not fixed yet (the scenario checks the Pilot
does exactly that; when the Pilot changes, it becomes a FAIL, to look at).
Outcomes of a mutant (--mutants): CAUGHT, at least one scenario fails with it (the scenarios that fail are listed,
and the expected ones too when none of them failed); MISSED, every scenario passes or is KNOWN: a rule nobody checks;
BROKEN, the mutant cannot be applied to pilot.py as it is (its text is not there exactly once, or its patch raises):
never counted as caught, to rewrite. The run fails on any FAIL, MISSED or BROKEN.

Usage (the folder of the guide is the current folder):
    python3 -I ci/fixed_pilot.py              every scenario
    python3 -I ci/fixed_pilot.py F5 F12       some of them
    python3 -I ci/fixed_pilot.py --mutants    each mutant (a rule of the Pilot removed) must fail a scenario
Only the standard library; no simulator, no model; a fraction of a second per scenario.
"""
import ast
import functools
import json
import random
import sys
import textwrap
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import probe_pilot as pp  # noqa: E402  (first: it sets ROBOT_STATE_DIR before stack/common.py is read)
from cartographer import REASONS  # noqa: E402  (the Cartographer's words, stack/cartographer.py)


SCENARIOS = {}   # name -> (function, cases of 5-pilote.md, invariants of ROBOT-PILOTE.md)
MUTANTS = {}     # name -> (patch(pilot), scenarios that must fail)
HARNESSES = []   # those of the scenario being played: the probe's checks inside the fake must not break


class Known(Exception):
    """The Pilot does exactly the defect written down: not fixed yet."""


class Calm(random.Random):
    """Every draw of the fake is its highest: random() is 0.999, and so are randint, choice and sample, which go
    through random() in a subclass that does not define getrandbits."""

    def random(self):
        return 0.999


def scenario(name, cases, invariants):
    def add(fn):
        SCENARIOS[name] = (fn, cases, invariants)
        return fn
    return add


def mutant(name, *catchers):
    def add(fn):
        MUTANTS[name] = (fn, catchers)
        return fn
    return add


class Fake(pp.Cartographer):
    """The probe's Cartographer with a world set by hand. script[name](**args) may answer instead of the fake (None:
    the fake answers); state_hook(state) may change each state it gives."""

    def __init__(self, h, at, facing, truth, boxes, limit, frame):
        super().__init__(Calm(), h.clock, h.mission, h.broken, h.pilot, h.toolset, defects=False)
        self.truth, self.boxes = dict(truth), set(boxes)
        self.col, self.row = pp.BASE.cell_xy(at)
        self.facing = pp.FACING.index(facing)
        self.frame, self.sure, self.back = frame, True, None
        self.limit, self.used = limit, 0
        self.adult_comes = False
        self.seen, self.ball = [], None
        self.script, self.state_hook = {}, None

    def enter(self, name, **args):
        said = super().enter(name, **args)
        if said:
            return said
        hook = self.script.get(name)
        return hook(**args) if hook else None

    def state(self):
        state = super().state()
        return self.state_hook(state) or state if self.state_hook else state

    def camera(self):
        """Every card in the cone, up to 4 cells: sure (could_also_be empty)."""
        saw = []
        dc, dr = pp.FACING_STEP[self.facing]
        for d in range(1, 5):
            for side in (-1, 0, 1) if d >= 2 else (0,):
                c, r = self.col + dc * d + (side if dr else 0), self.row + dr * d + (side if dc else 0)
                if not pp.on_mat(c, r):
                    continue
                cell = pp.name_of(c, r)
                self.looked.add(cell)
                card = self.truth.get(cell)
                if card and not self.frame:
                    self.seen = [s for s in self.seen if not (s["card"] == card and s["cell"] == cell)]
                    self.seen.append({"card": card, "means": pp.MAT["cards"][card], "cell": cell, "could_also_be": [],
                                      "distance_cm": d * pp.CELL_CM, "seen_from": f"{self.here()} facing "
                                      f"{pp.FACING[self.facing]}", "at": "12:00:00"})
                    saw.append(f"the camera recognised a card in front: {card}")
        return saw

    def set_light(self, color):
        said = self.enter("set_light", color=color)
        return said or {"result": "done"}

    def wait_for(self, seconds):
        hook = self.script.get("wait_for")
        if hook:
            self.spend()
            if self.op is not None:
                self.op["calls"].append(["wait_for", {"seconds": seconds}])
            said = hook(seconds=seconds)
            if said is not None:
                return self.answer(said)
        return super().wait_for(seconds)


class Harness:
    """One scenario: a fresh Pilot on the probe's clock, its mission (obey: under the Strategist) and the fake."""

    def __init__(self, at="C5", facing="north", truth=(), boxes=(), limit=None, frame=False, obey=True,
                 toolset="route", settings=None):
        pp.STATE.mkdir(parents=True, exist_ok=True)  # main() removes it at the end
        for f in pp.STATE.iterdir():
            if f.is_file():
                f.unlink()
        self.pilot = pp.load_fresh()
        self.clock = pp.Clock()
        self.pilot.clock = self.clock
        self.toolset = toolset
        self.broken = []
        HARNESSES.append(self)
        self.mission = pp.Mission(self.clock, obey)
        self.fake = Fake(self, at, facing, dict(truth), set(boxes), limit, frame)
        self.fake.hold_wanted = self.pilot.CART.hold_wanted
        self.pilot.CART = self.fake
        self.common = pp.Common(self.mission, self.fake, self.broken)
        self.pilot.common = self.common
        pp.common.say_held(None)
        self.configure(settings or {})
        if obey:
            self.pilot.obey(self.common.mission_control, self.common.say_held)
        else:
            self.pilot.obey()

    @property
    def P(self):
        return self.pilot.PILOT

    def configure(self, settings):
        self.pilot.RULES = dict(self.pilot.RULES, limits={"moves": self.fake.limit} if self.fake.limit is not None
                                else {})
        self.P.configure(settings)
        self.settings = dict(self.P.settings)

    def tool(self, name, **args):
        """A tool of the toolset, as the Strategist calls it: counted, through operate."""
        fn = self.pilot.TOOLSETS[self.toolset][name][0]
        self.fake.begin(self.toolset, name, self.P.settings)
        self.log_start = len(pp._log())
        try:
            return self.pilot.operate(fn, **args)
        except pp.ProbeLimit:
            return {"result": "PROBE LIMIT", "calls": len(self.fake.op["calls"])}

    def calls(self, *names):
        """The calls the fake received in the last tool, [name, args], only these names if given."""
        calls = (self.fake.op or {}).get("calls", [])
        return [c for c in calls if not names or c[0] in names]

    def moves(self):
        return self.calls(*pp.CART_MOVES)

    def log(self):
        return pp._log()[self.log_start:]

    def cell(self):
        return pp.name_of(self.fake.col, self.fake.row)


def check(cond, what, got=None):
    if not cond:
        raise AssertionError(what + ("" if got is None else f": got {json.dumps(got, default=str)[:600]}"))


def xy(cell):
    return pp.BASE.cell_xy(cell)



def names(h):
    """The names of the calls the fake received in the last tool."""
    return [c[0] for c in h.calls()]


def lines(h):
    """The lines of pilot.jsonl since the last tool began: (tool, arguments, result)."""
    return [(l["tool"], l["arguments"], l["result"]) for l in h.log()]


# --- mutants by text --------------------------------------------------------------------------------------------------
class MutantBroken(Exception):
    """A mutant that cannot be applied to pilot.py as it is: never counted as caught."""


@functools.lru_cache(maxsize=None)
def pilot_source(path):
    """pilot.py's lines and syntax tree, read once."""
    text = Path(path).read_text(encoding="utf-8")
    return text.splitlines(keepends=True), ast.parse(text)


def rewrite(m, name, *changes):
    """A mutant by text, on the loaded Pilot m: the function `name`, of the module ("go_to_cell") or a method
    ("Pilot.travel"), taken from pilot.py with its decorators, each pair (old, new) replaced, then compiled again in
    the module's globals. Each old text is written as in pilot.py, indentation included, and must be in that function
    exactly once, else MutantBroken. All the pairs of one function go in one call: a second rewrite of it would start
    again from pilot.py. A method is set on its class; a function of the module replaces the module's own (the other
    functions call the new one), and a tool keeps its @logged (part of the text) and its place in TOOLSETS."""
    text, tree = pilot_source(m.__file__)
    cls, _, fn = name.rpartition(".")
    body = tree.body
    if cls:
        body = next((n.body for n in body if isinstance(n, ast.ClassDef) and n.name == cls), [])
    node = next((n for n in body if isinstance(n, ast.FunctionDef) and n.name == fn), None)
    if node is None:
        raise MutantBroken(f"{name}: no such function in pilot.py")
    first = min([node.lineno] + [d.lineno for d in node.decorator_list])
    src = "".join(text[first - 1:node.end_lineno])
    for old, new in changes:
        if src.count(old) != 1:
            raise MutantBroken(f"{name}: {old!r} found {src.count(old)} times")
        src = src.replace(old, new)
    space = {}
    # Blank lines first: a traceback gives pilot.py's line numbers.
    exec(compile("\n" * (first - 1) + textwrap.dedent(src), m.__file__, "exec"), m.__dict__, space)
    if cls:
        setattr(getattr(m, cls), fn, space[fn])
        return
    old_fn, m.__dict__[fn] = m.__dict__[fn], space[fn]
    for tools in m.TOOLSETS.values():
        for tool, entry in tools.items():
            if entry[0] is old_fn:
                tools[tool] = (space[fn],) + tuple(entry[1:])


# --- what the scenarios send and set ----------------------------------------------------------------------------------
READ = ["robot_state", {}]
SEEN = ["seen_cards", {}]   # card_cells(): once in go_to_cell before and after here(), then once at each plan
WAIT5 = ["wait_for", {"seconds": 5}]
LEG_CALLS = ("robot_state", "face", "move", "wait_for", "step_forward", "turn", "look_around", "set_light")
BUSY = "another move is still running: send one move at a time, and read its answer before the next"
# What the Cartographer answers to every command that acts, under a limit of commands of the adults
# (stack/cartographer.py:932-934): the Referee refuses MOVE, TURN and LED alike.
NO_COMMANDS = {"result": "refused", "why": "no commands left: the adults allow 3 commands per mission"}
NO_CELLS = {"result": "refused", "why": "no cells left: the adults allow 6 cells per mission"}
NO_MOVES = {"result": "refused", "why": "no moves left: the adults allow 5 moves per mission"}
OBSTACLE = REASONS["obstacle"]    # "something is in front of the robot": the only refusal that blocks a cell
STILL_MOVING = {"result": "refused", "why": REASONS["busy"]}  # refused again and again, not a limit of the adults
ACTS = ("look_around", "move", "face", "turn", "step_forward", "set_light")
GO_TO_CARD_GAVE_UP = "the robot did not find itself in front of the card barbecue"
NO_WAY = "no way to the card barbecue now (a ball's lane, or something in the way)"
ADULT = "the robot waits for an adult"
STILL = "the robot still waits for an adult"
LOST = ("the robot could not find where it is on the mat (the border alone leaves two places on a mat that looks the "
        "same turned round: a landmark in the rules settles it)")


def face(direction):
    return ["face", {"direction": direction}]


def slide(direction, cells):
    return ["move", {"direction": direction, "cells": cells, "stop_if": []}]


# Round C4 (or C3) by column B, or by column D: the same moves, mirrored.
BY_B = [face("west"), slide("west", 1), face("north"), slide("north", 2), face("east"), slide("east", 1)]
BY_D = [face("east"), slide("east", 1), face("north"), slide("north", 2), face("west"), slide("west", 1)]


def acting(h):
    """The calls that act, as [name, direction or None, cells or None]."""
    return [[n, a.get("direction"), a.get("cells")] for n, a in h.moves()]


def record(card, cell, distance_cm, also=(), seen_from="C5 facing north"):
    """One record of the Cartographer, set by hand (as Cartographer.sight writes it)."""
    return {"card": card, "means": pp.MAT["cards"][card], "cell": cell, "could_also_be": list(also),
            "distance_cm": distance_cm, "seen_from": seen_from, "at": "12:00:00"}


def block(h, *cells):
    """Cells the Pilot has seen blocked (a dict cell -> time)."""
    for cell in cells:
        h.P.blocked[xy(cell)] = h.clock.now()


def at(h, cell, facing, free=(), blocked=(), lanes=()):
    """The Pilot told by hand where it is and what it knows: path() then reads nothing from the fake."""
    h.P.state = {"cell": cell, "facing": facing, "position_known": True}
    h.P.free |= {xy(c) for c in free}
    h.P.blocked.update({xy(c): 0 for c in blocked})
    h.P.lanes |= {("cell", xy(c)) for c in lanes}


def way(h, goal):
    return [pp.BASE.cell_name(*c) for c in h.P.path({xy(goal)}, set()) or []]


def either(got, *ways):
    """Two ways of the same cost: the heap's order picks one (stack/pilot.py:420, 438), P2 does not say which."""
    return got in ways


# --- what the fake gives ----------------------------------------------------------------------------------------------
def lanes(h, which):
    """Every state of the fake carries the Cartographer's ball_lanes (`which`: "row 4", "column C"...), and
    on_a_ball_lane for the robot's cell (the fake's own state has neither)."""
    ways = h.pilot.Pilot.lanes_of(which)

    def hook(state):
        state["ball_lanes"] = list(which)
        if state.get("position_known"):
            c, r = xy(state["cell"])
            state["on_a_ball_lane"] = bool({("col", c), ("row", r), ("cell", (c, r))} & ways)
    h.fake.state_hook = hook


def ball_closer(h, how="getting closer to the robot", ago=1):
    """Each state the fake gives: a ball seen `ago` s before, `how` it moved, without a cell (so no lane)."""
    def hook(state):
        state["ball_last_seen"] = {"how": how, "at": "12:00:00", "t": h.clock.now() - ago}
    return hook


def stop_in_first_wait(h, stopped):
    """wait_for: the group stops the mission during the first wait; each wait passes in full, nothing happens."""
    def wait(seconds):
        if not stopped:
            h.mission.stop()
            stopped.append(h.clock.monotonic())
        h.clock.advance(seconds)
        return {"result": "nothing happened", "waited_seconds": seconds}
    return wait


def waiting(h, back):
    """The robot waits for an adult: a move was interrupted, an adult must put it back on `back` (col, row, facing)."""
    h.fake.sure, h.fake.back = False, back


def resume_at(h, n):
    """fake.wait_for: an adult presses Resume at the n-th wait and puts the robot on fake.back. It always answers: a
    hook answering None has the call recorded twice (Fake.wait_for)."""
    waits = [0]

    def wait_for(seconds):
        waits[0] += 1
        if waits[0] == n:
            h.clock.advance(12)
            h.fake.sure = True
            h.fake.col, h.fake.row, h.fake.facing = h.fake.back
            return {"result": "something happened", "waited_seconds": 12,
                    "events": [{"seconds_before_this_answer": 0.3, "event": "an adult pressed Resume on the robot"}]}
        h.clock.advance(seconds)
        return {"result": "nothing happened", "waited_seconds": seconds}
    h.fake.script["wait_for"] = wait_for


def lost(h, places):
    """The place on the mat is not known (a frame relative to the start). places[k] is the count of places the
    Cartographer gives after the (k+1)th look_around, None for more than 6 (no could_be): it changes at each
    look_around, not at each read."""
    h.fake.frame = True
    rounds = [0]

    def count(**_):
        rounds[0] += 1
    h.fake.script["look_around"] = count

    def hook(state):
        n = places[min(max(rounds[0], 1), len(places)) - 1]
        if n is not None:
            state["position"] = "probable"
            state["could_be"] = [f"{c} facing north" for c in pp.CELLS[:n]]
        return state
    h.fake.state_hook = hook


# --- what the Pilot did -----------------------------------------------------------------------------------------------
def where_calls(h):
    """[(call, its arguments, cell of the robot when it came)], for every call to the fake, wait_for included."""
    seen = []
    enter, wait_for = h.fake.enter, h.fake.wait_for

    def rec_enter(name, **args):
        seen.append((name, args, h.fake.here()))
        return enter(name, **args)

    def rec_wait(seconds):
        seen.append(("wait_for", {"seconds": seconds}, h.fake.here()))
        return wait_for(seconds)
    h.fake.enter, h.fake.wait_for = rec_enter, rec_wait
    return seen


def cards_ahead(h):
    """Every (cell, facing, card ahead) the fake gave in a state."""
    seen = []

    def hook(state):
        seen.append((state["cell"], state["facing"], (state.get("card_ahead") or {}).get("card")))
    h.fake.state_hook = hook
    return seen


def log_sleeps(h):
    """The Pilot's sleeps among the fake's calls, as ["sleep", {"seconds": s}]."""
    sleep = h.clock.sleep

    def logged(seconds):
        if h.fake.op is not None:
            h.fake.op["calls"].append(["sleep", {"seconds": seconds}])
        sleep(seconds)
    h.clock.sleep = logged


def still_before_reading(h):
    """Seconds slept between the last face or move and the last robot_state of the tool."""
    calls = h.calls()
    last_read = max(i for i, c in enumerate(calls) if c[0] == "robot_state")
    last_move = max(i for i, c in enumerate(calls[:last_read]) if c[0] in pp.CART_MOVES)
    return sum(c[1]["seconds"] for c in calls[last_move:last_read] if c[0] == "sleep")


def read_after_each_wait(h):
    """Each wait_for followed at once by a robot_state."""
    calls = h.calls()
    return all(i + 1 < len(calls) and calls[i + 1][0] == "robot_state"
               for i, c in enumerate(calls) if c[0] == "wait_for")


def spy_travel(h):
    """Every travel of the Pilot and its outcome, [(goals, there)], without changing what it does."""
    seen = []
    travel = h.P.travel

    def spied(goals):
        there = travel(goals)
        seen.append((set(goals), there))
        return there
    h.P.travel = spied
    return seen


def explore_bounded(h, calls):
    """explore() with the probe's bound on calls to the Cartographer lowered: an endless loop is caught fast."""
    keep = pp.CALLS_PER_TOOL
    pp.CALLS_PER_TOOL = calls
    try:
        return h.tool("explore")
    finally:
        pp.CALLS_PER_TOOL = keep


# --- F1: no moves left (P14) ------------------------------------------------------------------------------------------
@scenario("F1", "P14", "I12")
def f1_no_moves_left():
    h = Harness(at="C5", facing="north", limit=1)
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "gave up", "why": "no moves left", "cards_seen": {}, "moves_left": 1},
          "one move left: gave up, no moves left", answer)
    check(not h.moves(), "nothing moved", h.moves())


@mutant("leg ignores the last move", "F1")
def m_short(m):
    m.Pilot.short = lambda self, n: False


# --- F2: legs of max_leg cells (P18) ----------------------------------------------------------------------------------
@scenario("F2", "P18", "I9")
def f2_max_leg():
    h = Harness(at="C5", facing="north", settings={"max_leg": 2})
    answer = h.tool("go_to_cell", cell="C1")
    check(answer["result"] == "done" and answer["cell"] == "C1", "done on C1", answer)
    want = [slide("north", 2)] * 2
    check(h.calls("move") == want, "two slides of 2 cells", h.calls("move"))
    h = Harness(at="C5", facing="north")
    h.tool("go_to_cell", cell="C1")
    check(h.calls("move") == [slide("north", 4)], "one slide of 4 cells",
          h.calls("move"))


@mutant("legs one cell shorter than max_leg", "F2")
def m_short_legs(m):
    travel = m.Pilot.travel
    def patched(self, goals):
        keep = self.settings["max_leg"]
        self.settings["max_leg"] = max(1, keep - 1)
        try:
            return travel(self, goals)
        finally:
            self.settings["max_leg"] = keep
    m.Pilot.travel = patched


# --- F3: the way and its costs (P2) -----------------------------------------------------------------------------------
@scenario("F3", "P2", "I17")
def f3_way_round():
    # Round C4 by column B (BY_B): B5, B4, B3 never seen (3 each), C3 too: 12. Column D (BY_D) costs the same: either.
    # (a) The first reading sees a box on C4 (29 cm: the next cell), which becomes blocked.
    h = Harness(at="C5", facing="north", boxes={"C4"})
    answer = h.tool("go_to_cell", cell="C3")
    check(h.P.is_blocked(xy("C4")), "C4 blocked by the first reading", sorted(h.P.blocked))
    check(either(h.moves(), BY_B, BY_D), "round C4 by column B or D, never north from C5", h.moves())
    check(answer["result"] == "done" and answer["cell"] == "C3", "done on C3", answer)
    # (b) D5, D4, D3 already seen free cost 1 each: column D (6) is cheaper than column B (12).
    h = Harness(at="C5", facing="north", boxes={"C4"})
    h.P.free |= {xy("D5"), xy("D4"), xy("D3")}
    h.tool("go_to_cell", cell="C3")
    check(h.moves() == BY_D, "cells seen free cost 1: by column D", h.moves())
    # (c) The same, column D a ball's lane: 9 per cell (30), column B again. (e) pins the 8.
    h = Harness(at="C5", facing="north", boxes={"C4"})
    h.P.free |= {xy("D5"), xy("D4"), xy("D3")}
    h.P.lanes.add(("col", 3))
    h.tool("go_to_cell", cell="C3")
    check(h.moves() == BY_B, "a ball's lane costs 8 more per cell: by column B", h.moves())
    # (d) No box, a card known on C4: the way never goes through it (the sensor sees C4 free, C3 then costs 1: 10).
    h = Harness(at="C5", facing="north", truth={"C4": "barbecue"})
    h.fake.seen = [record("barbecue", "C4", pp.CELL_CM)]
    answer = h.tool("go_to_cell", cell="C3")
    check(either(h.moves(), BY_B, BY_D), "round the card on C4 by column B or D", h.moves())
    check(answer["result"] == "done" and answer["cell"] == "C3", "done on C3", answer)
    # (e) The costs, from path() alone: seen free 1, never seen 3, a ball's lane 8 more. Costs are odd, so two ways
    # always differ by an even number: each case keeps a margin of 1 or 2 on one side of the rule.
    h = Harness()
    at(h, "C5", "north", free={"D5", "D4", "D3", "D2", "C2"})
    check(way(h, "C2") == ["D5", "D4", "D3", "D2", "C2"], "five cells seen free (5) before two never seen (7): a free "
          "cell costs less than half an unseen one", way(h, "C2"))
    h = Harness()
    at(h, "C5", "north", free={"D5", "E5", "E4", "E3", "D3", "C3"})
    check(way(h, "C3") == ["C4", "C3"], "one cell never seen (4) before five seen free (6): an unseen cell costs less "
          "than five free ones, and a free cell is not free of cost", way(h, "C3"))
    h = Harness()
    at(h, "C5", "north", free={"C4", "C3", "B5"}, lanes={"C4"})
    check(way(h, "C3") == ["B5", "B4", "B3", "C3"], "C4 on a lane (1 + 8, 10) loses to B5, B4, B3 (8): a lane costs "
          "more than 6", way(h, "C3"))
    h = Harness()
    at(h, "C5", "north", free={"B5"}, blocked={"B4", "D4"}, lanes={"C4"})
    check(way(h, "C3") == ["C4", "C3"], "C4 on a lane (3 + 8 + 3, 14) wins over B5, A5, A4, A3, B3 (16): a lane costs "
          "less than 10", way(h, "C3"))
    # (f) The sensor blind (no distance), a box on C3: the slide north 3 stops on C4, "something was in the way", and
    # the cell after it, C3, becomes blocked. Then round it by B or D (12 each).
    h = Harness(at="C5", facing="north", boxes={"C3"})
    h.fake.state_hook = lambda state: dict(state, distance_ahead_cm=None)
    answer = h.tool("go_to_cell", cell="C2")
    check(h.moves()[:1] == [slide("north", 3)], "first, north 3 (all unseen, 9)", h.moves())
    check(h.P.is_blocked(xy("C3")), "C3 blocked after the slide stopped", sorted(h.P.blocked))
    check(either(h.moves()[1:], BY_B, BY_D), "then round C3 by column B or D", h.moves())
    check(answer["result"] == "done" and answer["cell"] == "C2", "done on C2", answer)
    # (g) C4 blocked earlier, nothing there now: the first reading (north, up to the border) sees it free and unblocks
    # it, and the way goes straight.
    h = Harness(at="C5", facing="north")
    h.P.blocked[xy("C4")] = 0
    answer = h.tool("go_to_cell", cell="C3")
    check(not h.P.is_blocked(xy("C4")), "C4 unblocked by the reading", sorted(h.P.blocked))
    check(h.moves() == [slide("north", 2)], "straight north 2", h.moves())
    check(answer["result"] == "done" and answer["cell"] == "C3", "done on C3", answer)


@mutant("is_blocked always false", "F3")
def m_never_blocked(m):
    m.Pilot.is_blocked = lambda self, cell: False


@mutant("a cell seen free costs 3 too", "F3")
def m_free_cost(m):
    rewrite(m, "Pilot.path", ("(1 if nxt in self.free else 3)", "3"))


@mutant("no extra cost on a ball's lane", "F3")
def m_lane_cost(m):
    rewrite(m, "Pilot.path", ("(8 if self.on_lane(nxt) else 0)", "0"))


@mutant("a ball's lane costs 2 more", "F3")
def m_lane_2(m):
    rewrite(m, "Pilot.path", ("(8 if self.on_lane(nxt) else 0)", "(2 if self.on_lane(nxt) else 0)"))


@mutant("a ball's lane costs 16 more", "F3")
def m_lane_16(m):
    rewrite(m, "Pilot.path", ("(8 if self.on_lane(nxt) else 0)", "(16 if self.on_lane(nxt) else 0)"))


@mutant("a cell seen free costs 2", "F3")
def m_free_2(m):
    rewrite(m, "Pilot.path", ("(1 if nxt in self.free else 3)", "(2 if nxt in self.free else 3)"))


@mutant("a cell seen free costs nothing", "F3")
def m_free_0(m):
    rewrite(m, "Pilot.path", ("(1 if nxt in self.free else 3)", "(0 if nxt in self.free else 3)"))


@mutant("a cell never seen costs 9", "F3")
def m_unseen_9(m):
    rewrite(m, "Pilot.path", ("(1 if nxt in self.free else 3)", "(1 if nxt in self.free else 9)"))


@mutant("a slide stopped by something in the way blocks nothing", "F3")
def m_stopped_blocks_nothing(m):
    rewrite(m, "Pilot.leg", ('if moved < cells and "something was in the way" in why and not self.ball_recent():',
                             "if False:"))


@mutant("a reading that sees a cell free leaves it blocked", "F3")
def m_no_unblock(m):
    rewrite(m, "Pilot.note_distance", ("self.blocked.pop((cell[0] + dc * d, cell[1] + dr * d), None)", "pass"))


@mutant("the sensor sees the thing one cell too far", "F3", "F17")
def m_sensor_off_by_one(m):
    rewrite(m, "Pilot.note_distance", ("k = 1 + int(dist) // CELL_CM", "k = 2 + int(dist) // CELL_CM"))


@mutant("the way may cross a card the Pilot knows", "F3")
def m_through_cards(m):
    rewrite(m, "Pilot.travel", ("way = self.path(goals - cards, cards | kept)", "way = self.path(goals - cards, kept)"))


@mutant("the distance sensor never blocks a cell", "F3", "F17")
def m_sensor_blocks_nothing(m):
    rewrite(m, "Pilot.note_distance", ("self.blocked[thing] = clock.now()", "pass"))


# --- F4: look_each_leg (P19) ------------------------------------------------------------------------------------------
@scenario("F4", "P19", "I9")
def f4_look_each_leg():
    # (a) A look_around before each leg, not only the first.
    h = Harness(at="C5", facing="north", settings={"look_each_leg": True, "max_leg": 2})
    answer = h.tool("go_to_cell", cell="C1")
    check(answer["result"] == "done" and answer["cell"] == "C1", "done on C1", answer)
    want = [["look_around", None, None], ["move", "north", 2], ["look_around", None, None], ["move", "north", 2]]
    check(acting(h) == want, "look, slide 2, look, slide 2", acting(h))
    # (b) The look shows the next cell blocked (a box), or carrying a card: the plan is made after it, round that cell.
    for what, world in [("a box on C4", {"boxes": {"C4"}}), ("a card on C4", {"truth": {"C4": "barbecue"}})]:
        h = Harness(at="C5", facing="east", settings={"look_each_leg": True, "max_leg": 2}, **world)
        before = []  # at each look: (C4 blocked for the Pilot, a card recorded on C4)
        h.fake.script["look_around"] = lambda h=h, before=before: before.append(
            (xy("C4") in h.P.blocked, any(s["cell"] == "C4" for s in h.fake.seen))) or None
        seen = where_calls(h)
        answer = h.tool("go_to_cell", cell="C1")
        check(answer["result"] == "done" and answer["cell"] == "C1", f"{what}: done on C1", answer)
        check(before[:1] == [(False, False)], f"{what}: C4 unknown before the first look", before)
        if "box" in what:
            check(xy("C4") in h.P.blocked, "the look blocked C4 (distance sensor, pilot.py:501-502)", h.P.blocked)
        acts = acting(h)
        check(acts[0] == ["look_around", None, None], f"{what}: the first leg starts with a look", acts)
        first = next((k for k, a in enumerate(acts) if a[0] == "move"), None)
        check(first is not None and acts[first][1] in ("east", "west"), f"{what}: the first slide goes east or west",
              acts)
        # It starts facing east: a slide east needs no turn first; a slide west does.
        check(acts[first][1] == "east" or acts[first - 1] == ["face", acts[first][1], None],
              f"{what}: the robot faced that way first", acts)
        check(not [s for s in seen if s[0] == "move" and s[1]["direction"] == "north" and s[2] == "C5"],
              f"{what}: never a slide north from C5", seen)
        moves = [k for k, a in enumerate(acts) if a[0] == "move"]
        check(all("look_around" in [a[0] for a in acts[(moves[i - 1] + 1 if i else 0):k]] for i, k in enumerate(moves)),
              f"{what}: a look_around before every slide", acts)
    # (c) The same at a later leg: from A6 the way to C2 goes up column A to A4, east along row 4 to C4, then up
    # column C (the cheapest way since 011ff0d, ties broken by path()). Only the look from C4 shows the box on C3: the
    # plan of that leg is made after it, round C3.
    h = Harness(at="A6", facing="north", boxes={"C3"}, settings={"look_each_leg": True, "max_leg": 2})
    before = []  # at each look: (where the robot is, C3 blocked for the Pilot)
    h.fake.script["look_around"] = lambda: before.append((h.fake.here(), xy("C3") in h.P.blocked)) or None
    seen = where_calls(h)
    answer = h.tool("go_to_cell", cell="C2")
    check(answer["result"] == "done" and answer["cell"] == "C2", "a later leg: done on C2", answer)
    check(("C4", False) in before and xy("C3") in h.P.blocked, "C3 unknown before the look from C4, blocked after it",
          [before, h.P.blocked])
    # Never a turn north from C4 either: since E9, a leg planned before the look would face C3, see it blocked and end
    # without a slide; only its quarter turn shows the plan came first.
    check(not [s for s in seen if s[0] in ("move", "face") and s[1]["direction"] == "north" and s[2] == "C4"],
          "a later leg: never a turn nor a slide north from C4, towards C3", seen)
    acts = acting(h)
    moves = [k for k, a in enumerate(acts) if a[0] == "move"]
    check(all("look_around" in [a[0] for a in acts[(moves[i - 1] + 1 if i else 0):k]] for i, k in enumerate(moves)),
          "a later leg: a look_around before every slide", acts)


@mutant("look_each_leg only before the first leg", "F4")
def m_look_first_leg(m):
    leg = m.Pilot.leg

    def patched(self, direction, cells):
        moved = leg(self, direction, cells)
        self.settings["look_each_leg"] = False
        return moved
    m.Pilot.leg = patched


LOOK = """            if self.settings["look_each_leg"]:
                self.look_around()  # before the plan: what it sees changes the way, and leaving a ball's lane moves it
                (c, r), _ = self.here()
                if (c, r) in goals:
                    return True
"""
PLAN = "            way = self.path(goals - cards, cards | kept)  # a card seen on the way may lie on a goal\n"


@mutant("the plan of a leg made before its look", "F4")
def m_plan_before_look(m):
    rewrite(m, "Pilot.travel", (LOOK, ""), (PLAN, PLAN + LOOK))


@mutant("later legs planned before their look", "F4")
def m_later_plan_before_look(m):
    first = LOOK.replace('if self.settings["look_each_leg"]:', 'if self.settings["look_each_leg"] and it == 0:')
    later = '            if self.settings["look_each_leg"] and it > 0:\n                self.look_around()\n'
    rewrite(m, "Pilot.travel", ("for _ in range(REPLANS):", "for it in range(REPLANS):"), (LOOK, first),
            (PLAN, PLAN + later))


# --- F5: a ball coming closer (P3) ------------------------------------------------------------------------------------
@scenario("F5", "P3", "I9 I16")
def f5_ball_closer():
    h = Harness(at="C5", facing="east")
    h.fake.state_hook = ball_closer(h)
    answer = h.tool("go_to_cell", cell="C3")
    # The leg faces its way first (a move, the ball does not stop it), then waits: 6 waits a leg, 8 plans, each plan
    # opened by seen_cards (the cards of the record are cells the way avoids).
    want = [SEEN, READ, SEEN] + [SEEN, face("north")] + [WAIT5, READ] * 6 \
        + ([SEEN] + [WAIT5, READ] * 6) * 7
    got = h.calls(*LEG_CALLS, "seen_cards")
    check(got == want, "one face north, then 8 plans of 6 waits of 5 s, each wait followed by a reading",
          [c[0][:4] for c in got])
    check(answer.get("result") == "not there" and answer.get("cell") == "C5" and answer.get("facing") == "north"
          and answer.get("why") == "the way was blocked or refused; look around, then choose again"
          and answer.get("moves_left") is None and answer.get("state") == h.P.state, "not there, on C5 facing north",
          {k: v for k, v in answer.items() if k != "state"})
    check(h.cell() == "C5" and not h.P.ball_refused, "nothing moved, no cell refused for a ball",
          [h.cell(), h.P.ball_refused])
    # The other way round: a ball going away, or coming closer but seen BALL_RECENT_S + 1 s before, is not waited for.
    north2 = slide("north", 2)
    for how, ago, what in [("moving away from the robot", 1, "a ball going away"),
                           ("getting closer to the robot", pp.BASE.BALL_RECENT_S + 1, "a ball seen too long ago")]:
        h = Harness(at="C5", facing="north")
        h.fake.state_hook = ball_closer(h, how, ago)
        answer = h.tool("go_to_cell", cell="C3")
        check(not h.calls("wait_for") and h.moves() == [north2], f"{what}: one move north at once, no wait",
              [h.calls("wait_for"), h.moves()])
        check(answer.get("result") == "done" and answer.get("cell") == "C3", f"{what}: done on C3",
              {k: v for k, v in answer.items() if k != "state"})


@mutant("ball waits never end (BALL_WAITS endless)", "F5")
def m_ball_waits_endless(m):
    m.BALL_WAITS = 10 ** 6


@mutant("nine plans instead of REPLANS", "F5", "F22b")
def m_replans(m):
    m.REPLANS = 9


@mutant("eight waits a leg, six plans (the same 48 waits)", "F5")
def m_waits_plans(m):
    m.BALL_WAITS, m.REPLANS = 8, 6


@mutant("any ball seen just before is waited for, even going away", "F5")
def m_wait_any_ball(m):
    rewrite(m, "Pilot.leg", (' and "closer" in ball.get("how", "")', ""))


@mutant("a ball coming closer is waited for however long ago it was seen", "F5")
def m_wait_old_ball(m):
    rewrite(m, "Pilot.leg", ('self.settings["careful_balls"] and self.ball_recent() and "closer"',
                             'self.settings["careful_balls"] and "closer"'))


@mutant("careful_balls ignored in a leg", "F5", "F22")
def m_careful_ignored(m):
    leg = m.Pilot.leg
    def patched(self, direction, cells, cards):
        keep = self.settings["careful_balls"]
        self.settings["careful_balls"] = False
        try:
            return leg(self, direction, cells, cards)
        finally:
            self.settings["careful_balls"] = keep
    m.Pilot.leg = patched


@mutant("no wait of 5 s in a leg", "F5", "F6", "F22")
def m_leg_no_wait(m):
    command = m.command
    def patched(name, *args):
        if name == "wait_for_event" and args == (5,):
            return {"result": "nothing happened", "waited_seconds": 0}
        return command(name, *args)
    m.command = patched


# --- F6: careless, a move refused for a ball (P4) ---------------------------------------------------------------------
@scenario("F6", "P4", "I17")
def f6_careless_refused():
    h = Harness(at="C6", facing="north", settings={"careful_balls": False})
    t0 = h.clock.now()
    h.P.blocked = {xy("B6"): t0, xy("D6"): t0}   # C6, bottom row: the only way out is north
    h.fake.state_hook = ball_closer(h)            # careless: it does not wait before asking
    refuse = [True]
    h.fake.script["move"] = lambda **a: ({"result": "refused", "why": "a ball was seen near C5 less than 10 s ago"}
                                         if refuse[0] else None)
    north3 = slide("north", 3)
    not_there = {"result": "not there", "cell": "C6", "facing": "north",
                 "why": "the way was blocked or refused; look around, then choose again", "moves_left": None}

    def short(answer):
        return {k: v for k, v in answer.items() if k != "state"}

    # (1) Six moves, each refused for the ball, each followed by a wait of 5 s and a reading; then C5 is kept away.
    answer = h.tool("go_to_cell", cell="C3")
    check(h.calls(*LEG_CALLS) == [READ] + [north3, WAIT5, READ] * 6,
          "(1) six refused moves, each then a wait of 5 s and a reading, no wait before asking",
          [c[0] for c in h.calls(*LEG_CALLS)])
    check(short(answer) == not_there, "(1) not there", short(answer))
    check(h.P.ball_refused == {xy("C5"): h.clock.now()}, "(1) C5 refused for a ball, now", h.P.ball_refused)
    # (2) At once, the same goal: no way but C5, avoided BALL_AVOID_S; without cross_when_boxed, nothing is sent.
    answer = h.tool("go_to_cell", cell="C3")
    check(not h.moves(), "(2) no move while C5 is avoided", h.moves())
    check(short(answer) == not_there, "(2) not there", short(answer))
    # (3) cross_when_boxed: the boxed robot tries C5 again, one leg of six refusals, and the goal is given up.
    h.P.configure({"careful_balls": False, "cross_when_boxed": True})
    answer = h.tool("go_to_cell", cell="C3")
    check(h.calls("move") == [north3] * 6, "(3) with cross_when_boxed, one leg north again: six moves",
          h.calls("move"))
    check(short(answer) == not_there, "(3) not there", short(answer))
    t3 = h.P.ball_refused.get(xy("C5"))
    check(t3 == h.clock.now(), "(3) C5 refused again, now", h.P.ball_refused)
    # (4) Without cross_when_boxed: still avoided just before BALL_AVOID_S, tried again once it is over.
    h.P.configure({"careful_balls": False})       # the next tool reads the state again: 0.2 s on the fake's clock
    h.clock.advance(t3 + pp.BASE.BALL_AVOID_S - 0.4 - h.clock.now())
    answer = h.tool("go_to_cell", cell="C3")
    check(not h.moves() and short(answer) == not_there, f"(4) {pp.BASE.BALL_AVOID_S - 0.2:g} s later: still avoided",
          [h.moves(), short(answer)])
    h.clock.advance(0.4)
    refuse[0] = False
    answer = h.tool("go_to_cell", cell="C3")
    check(h.calls("move") == [north3] and answer.get("result") == "done" and answer.get("cell") == "C3",
          f"(4) {pp.BASE.BALL_AVOID_S + 0.2:g} s later: one leg north, done on C3", [h.calls("move"), short(answer)])
    # (5) cross_when_boxed only when boxed: C5 just refused, B6 free, the way goes west round C5, never through it.
    h = Harness(at="C6", facing="north", settings={"careful_balls": False, "cross_when_boxed": True})
    h.P.blocked = {xy("D6"): h.clock.now()}
    h.P.ball_refused = {xy("C5"): h.clock.now()}
    answer = h.tool("go_to_cell", cell="C3")
    moves = h.calls("move")
    check(moves[:1] == [slide("west", 1)],
          "(5) another way left: the first leg goes west to B6, not north through C5", moves)
    check(answer.get("result") == "done" and answer.get("cell") == "C3", "(5) done on C3", short(answer))


@mutant("cross_when_boxed crosses a refused cell even when another way exists", "F6")
def m_cross_always(m):
    rewrite(m, "Pilot.travel", ("way = self.path(goals - cards, cards | kept)",
                                "way = self.path(goals - cards, cards if self.settings['cross_when_boxed'] "
                                "else cards | kept)"))


@mutant("careless still waits for a ball before asking", "F6")
def m_careless_waits(m):
    configure = m.Pilot.configure
    def patched(self, settings):
        configure(self, settings)
        self.settings["careful_balls"] = True
    m.Pilot.configure = patched


@mutant("a cell refused for a ball is not kept (BALL_AVOID_S 0)", "F6")
def m_avoid_zero(m):
    m.BALL_AVOID_S = 0


@mutant("a cell refused for a ball is kept for ever", "F6")
def m_avoid_long(m):
    m.BALL_AVOID_S = 10 ** 6


@mutant("ball_refused never written", "F6")
def m_no_ball_refused(m):
    class Forget(dict):
        def __setitem__(self, key, value):
            pass
    m.PILOT.ball_refused = Forget()  # the module's Pilot is made at import, before the patch


@mutant("travel goes on after a leg refused for a ball (-2 ignored)", "F6")
def m_minus_two(m):
    leg = m.Pilot.leg
    m.Pilot.leg = lambda self, direction, cells, cards: (lambda n: 0 if n == -2 else n)(leg(self, direction, cells, cards))


@mutant("cross_when_boxed ignored", "F6")
def m_cross_ignored(m):
    travel = m.Pilot.travel
    def patched(self, goals):
        keep = self.settings["cross_when_boxed"]
        self.settings["cross_when_boxed"] = False
        try:
            return travel(self, goals)
        finally:
            self.settings["cross_when_boxed"] = keep
    m.Pilot.travel = patched


# --- F7: go_to_card, the surest place first (P5) ----------------------------------------------------------------------
# The cells ruled out live in go_to_card, for one call (ROBOT-PILOTE.md, E4): they are seen only through what the
# robot does, the next attempt of the same call and a second call that tries them again.
@scenario("F7a", "P5", "I15 I20")
def f7a_surest_first():
    # Seen from far: C2 probable, D2 possible; the card is on D2. C2 first (120 cm against 150), faced from C3 without
    # the card: ruled out, so the next attempt goes to D2, faced from D3.
    h = Harness(at="C5", facing="north", truth={"D2": "barbecue"}, toolset="goal")
    h.fake.seen = [record("barbecue", "C2", 120, ["D2"])]
    log_sleeps(h)
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "done", "cell": "D3", "facing": "north", "card_ahead": "barbecue", "moves_left": None},
          "done in front of D2, from D3 facing north", answer)
    # The camera names a card after 200 ms standing still (stack/pilot.py:675): still that long before the reading.
    check(still_before_reading(h) >= 0.2, "at least 0.2 s still between the last turn and the reading", h.calls()[-4:])
    want = [slide("north", 2), face("east"),
            slide("east", 1), face("north")]
    check(h.moves() == want, "C3 faced first (the surest), then D3: C2 ruled out after one attempt", h.moves())
    check([l["tool"] for l in h.log()] == ["go_to_card"], "no explore", [l["tool"] for l in h.log()])
    # A second call, the same mission, nothing else changed: the goal was forgotten with its cells ruled out, so C2 is
    # tried again first (from C3, the cheapest front), then D2. The cost written in the decision E4.
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "done", "cell": "D3", "facing": "north", "card_ahead": "barbecue", "moves_left": None},
          "second call: done again from D3", answer)
    check(h.moves() == [face("west"), slide("west", 1), face("north"), face("east"), slide("east", 1), face("north")],
          "second call: C2 tried again from C3, then D2 from D3", h.moves())


@scenario("F7b", "P5", "I15 I20")
def f7b_failed_face_rules_out():
    # The card is on C2, sure enough to try first; D2 is blocked, so is every way to C2 but C3. The robot reaches C3
    # facing east; the quarter turn north is refused once, without a state. The reading after it shows the robot on C3
    # facing east: C2 was not faced, so it is not ruled out (ROBOT-PILOTE.md, E4), and the next attempt turns again.
    h = Harness(at="A3", facing="east", truth={"C2": "barbecue"}, toolset="goal")
    h.fake.seen = [record("barbecue", "C2", 120, ["D2"])]
    block(h, "B2", "C1", "D2")
    refused = []

    def refuse_first(direction):
        if not refused:
            refused.append(direction)
            return {"result": "refused", "why": "the robot is still moving"}
        return None
    h.fake.script["face"] = refuse_first
    states = cards_ahead(h)
    answer = h.tool("go_to_card", card="barbecue")
    check(refused == ["north"] and ("C3", "east", None) in states, "the face north refused, read on C3 facing east",
          [refused, states])
    check(h.moves() == [slide("east", 2), face("north"), face("north")],
          "one leg, the refused face, the same face again at the next attempt", h.moves())
    check(answer == {"result": "done", "cell": "C3", "facing": "north", "card_ahead": "barbecue", "moves_left": None},
          "done in front of C2, from C3 facing north", answer)
    check([l["tool"] for l in h.log()] == ["go_to_card"], "no explore: C2 never ruled out", [l["tool"] for l in h.log()])
    # Without the Strategist, the quarter turn north is stopped once: the adult puts the robot back on C4 facing north,
    # the way C2 is faced from C3. Facing the right way from another cell rules nothing out: C3, then C2 faced.
    h = Harness(at="A3", facing="east", truth={"C2": "barbecue"}, toolset="goal", obey=False)
    h.fake.seen = [record("barbecue", "C2", 120, ["D2"])]
    block(h, "B2", "C1", "D2")
    stopped = []

    def stop_first(direction):
        if not stopped:
            stopped.append(direction)
            waiting(h, (2, 3, 0))
            return {"result": "stopped", "why": "something touched the robot's bumper", "position_known": False,
                    "what_to_do": h.fake.instruction()}
        return None
    h.fake.script["face"] = stop_first
    resume_at(h, 1)
    states = cards_ahead(h)
    answer = h.tool("go_to_card", card="barbecue")
    check(stopped == ["north"] and ("C4", "north", None) in states, "the face stopped, the robot put back on C4 facing "
          "north, no card ahead", [stopped, states])
    check(h.moves() == [slide("east", 2), face("north"), slide("north", 1)], "then from C4 to C3, already facing north",
          h.moves())
    check(answer == {"result": "done", "cell": "C3", "facing": "north", "card_ahead": "barbecue", "moves_left": None},
          "done in front of C2, from C3 facing north", answer)


@scenario("F7c", "P5", "I15 I20")
def f7c_other_card_ahead():
    # As F7a, but another card lies on C2: faced from C3, it is not the barbecue, so C2 is ruled out all the same.
    h = Harness(at="C5", facing="north", truth={"C2": "dry-grass", "D2": "barbecue"}, toolset="goal")
    h.fake.seen = [record("barbecue", "C2", 120, ["D2"])]
    seen = cards_ahead(h)
    answer = h.tool("go_to_card", card="barbecue")
    check(("C3", "north", "dry-grass") in seen, "faced dry-grass from C3", seen)
    check(answer == {"result": "done", "cell": "D3", "facing": "north", "card_ahead": "barbecue", "moves_left": None},
          "another card ahead is not done: on to D2, from D3", answer)
    want = [slide("north", 2), face("east"),
            slide("east", 1), face("north")]
    check(h.moves() == want, "C3 faced first, then D3, never C3 again", h.moves())


@scenario("F7d", "P5", "I15 I20")
def f7d_could_also_be_less_sure():
    # Two records: C2 at 100 cm (D2 possible), B2 at 120 cm (A2 possible); the card is on B2. A cell of could_also_be
    # counts one cell farther (130 and 150 cm): C2, then B2, never D2 nor A2.
    h = Harness(at="C5", facing="north", truth={"B2": "barbecue"}, toolset="goal")
    h.fake.seen = [record("barbecue", "C2", 100, ["D2"]), record("barbecue", "B2", 120, ["A2"])]
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "done", "cell": "B3", "facing": "north", "card_ahead": "barbecue", "moves_left": None},
          "done in front of B2, from B3 facing north", answer)
    want = [slide("north", 2), face("west"),
            slide("west", 1), face("north")]
    check(h.moves() == want, "C3 faced first, then B3 (B2 at 120 cm before D2 at 130), D2 never tried", h.moves())


RULE_OUT = '        if state.get("cell") == cell_name(c, r) and state.get("facing") == fronts[(c, r)]:\n'


@mutant("go_to_card tries the least sure place first", "F7a")
def m_least_sure(m):
    rewrite(m, "go_to_card", ("cell = min(candidates, key=candidates.get)",
                              "cell = max(candidates, key=candidates.get)"))


@mutant("go_to_card never rules a place out", "F7a", "F7c")
def m_never_ruled_out(m):
    rewrite(m, "go_to_card", ("ruled_out.add(cell)  # faced", "pass  # faced"))


@mutant("go_to_card rules a place out even when the face failed (before E4)", "F7b")
def m_rule_out_always(m):
    rewrite(m, "go_to_card", (RULE_OUT, "        if True:\n"))


@mutant("go_to_card rules a place out when facing it from another cell", "F7b")
def m_rule_out_facing_only(m):
    rewrite(m, "go_to_card", (RULE_OUT, '        if state.get("facing") == fronts[(c, r)]:\n'))


@mutant("go_to_card rules a place out when on the cell, whatever the way", "F7b")
def m_rule_out_cell_only(m):
    rewrite(m, "go_to_card", (RULE_OUT, '        if state.get("cell") == cell_name(c, r):\n'))


@mutant("the cells ruled out kept for the process (Pilot.ruled_out, before E4)", "F7a")
def m_keep_ruled_out(m):
    m.KEPT_RULED_OUT = set()
    rewrite(m, "go_to_card", ("unreachable, ruled_out = set(), set()", "unreachable, ruled_out = set(), KEPT_RULED_OUT"))


@mutant("the cells ruled out kept for the mission, forgotten at configure", "F7a")
def m_keep_ruled_out_mission(m):
    m.KEPT_RULED_OUT = set()
    rewrite(m, "go_to_card", ("unreachable, ruled_out = set(), set()", "unreachable, ruled_out = set(), KEPT_RULED_OUT"))
    configure = m.Pilot.configure

    def patched(self, settings):
        configure(self, settings)
        m.KEPT_RULED_OUT.clear()
    m.Pilot.configure = patched


@mutant("go_to_card answers done without the card ahead", "F7a")
def m_no_card_check(m):
    rewrite(m, "go_to_card", ('if (state.get("card_ahead") or {}).get("card") == card:', "if True:"))


@mutant("a place of could_also_be is as sure as the record's cell", "F7d")
def m_also_as_sure(m):
    rewrite(m, "go_to_card", ('s["distance_cm"] + CELL_CM', 's["distance_cm"]'))


@mutant("any card ahead counts as done", "F7c")
def m_any_card(m):
    rewrite(m, "go_to_card", ('.get("card") == card:', '.get("card"):'))


@mutant("go_to_card reads the card without standing still", "F7a")
def m_no_sleep(m):
    rewrite(m, "go_to_card", ("clock.sleep(0.5)", "pass"))


# --- F8: go_to_card explores first (P6, P16, P12) ---------------------------------------------------------------------
@scenario("F8a", "P6 P16", "I12 I13")
def f8a_explore_first():
    # A card the camera never saw, on the mat: go_to_card explores (its own line in pilot.jsonl, before its own), the
    # first look shows it on D2 from C5, then the cheapest front is C2 (C4, C3, C2 seen free by the first reading).
    h = Harness(at="C5", facing="north", truth={"D2": "barbecue"}, toolset="goal")
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "done", "cell": "C2", "facing": "east", "card_ahead": "barbecue", "moves_left": None},
          "in front of the barbecue, from C2 facing east", answer)
    check(h.moves() == [["look_around", {}], slide("north", 3),
                        face("east")], "a look, one slide of 3, a quarter turn", h.moves())
    check(lines(h) == [("explore", [["barbecue"], {}], {"result": "found", "cards_seen": {"barbecue": ["D2"]},
                                                         "not_sure": [], "mat_covered": False, "moves_left": None}),
                       ("go_to_card", [[], {"card": "barbecue"}], answer)], "pilot.jsonl: explore, then go_to_card",
          lines(h))
    # Not on the mat: the whole mat explored, then not found.
    h = Harness(at="C5", facing="north", toolset="goal")
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "not found", "why": "the camera never saw a card barbecue", "moves_left": None},
          "not found, the camera never saw it", answer)
    check(lines(h) == [("explore", [["barbecue"], {}], {"result": "not found", "cards_seen": {}, "not_sure": [],
                                                         "mat_covered": True, "moves_left": None}),
                       ("go_to_card", [[], {"card": "barbecue"}], answer)], "pilot.jsonl: explore, then go_to_card",
          lines(h))


@scenario("F8b", "P16", "I12 I13")
def f8b_short_no_explore():
    # Three moves left: go_to_card explores; the first look (one move, as for the Referee) leaves 2, and explore stops
    # there (stack/pilot.py:598), with most of the mat not seen.
    h = Harness(at="C5", facing="north", limit=3, toolset="goal")
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "not found", "why": "the camera never saw a card barbecue", "moves_left": 2},
          "3 moves left: one look, then not found", answer)
    check(h.moves() == [["look_around", {}]], "one look and nothing else", h.moves())
    check(lines(h) == [("explore", [["barbecue"], {}], {"result": "not found", "cards_seen": {}, "not_sure": [],
                                                         "mat_covered": False, "moves_left": 2}),
                       ("go_to_card", [[], {"card": "barbecue"}], answer)], "pilot.jsonl: explore, then go_to_card",
          lines(h))
    # Two moves left, the card never seen: no exploration, and the reason is the moves (ROBOT-PILOTE.md, E6).
    h = Harness(at="C5", facing="north", limit=2, toolset="goal")
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "gave up", "why": "no moves left", "cards_seen": {}, "moves_left": 2},
          "gave up at once: no moves left", answer)
    check(not h.moves() and names(h)[:1] == ["robot_state"] and set(names(h)[1:]) == {"seen_cards"},
          "a reading and the record, nothing else", h.calls())
    check([l[0] for l in lines(h)] == ["go_to_card"], "no explore line", lines(h))
    # Two moves left, the card recorded on C3 but no place left to try (the first reading sees a box there: blocked):
    # the reason stays "did not find itself", the moves are not the cause.
    h = Harness(at="C5", facing="north", limit=2, toolset="goal", boxes={"C3"})
    h.fake.seen = [record("barbecue", "C3", 60)]
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "gave up", "why": GO_TO_CARD_GAVE_UP, "moves_left": 2},
          "seen, no place to try, 2 moves left: did not find itself", answer)
    check(not h.moves() and [l[0] for l in lines(h)] == ["go_to_card"], "nothing moved, no explore",
          [h.moves(), lines(h)])


@scenario("F8c", "P12 P16", "I1 I12 I13")
def f8c_stop_during_explore():
    # The mission stopped during the first look of the exploration: the next move raises the stop, explore gives up,
    # and go_to_card passes on its gave up with the same why (without cards_seen: its own answer, stack/pilot.py:656).
    h = Harness(at="C5", facing="north", toolset="goal")
    looks = []
    def look(**_):
        looks.append(1)
        if len(looks) == 1:
            h.mission.stop()
    h.fake.script["look_around"] = look
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "gave up", "why": "the mission was stopped", "moves_left": None},
          "gave up: the mission was stopped", answer)
    check(h.moves() == [["look_around", {}]], "nothing moves after the stop", h.moves())
    check(lines(h) == [("explore", [["barbecue"], {}], {"result": "gave up", "why": "the mission was stopped",
                                                         "cards_seen": {}, "moves_left": None}),
                       ("go_to_card", [[], {"card": "barbecue"}], answer)],
          "pilot.jsonl: explore gave up, then go_to_card the same", lines(h))


@mutant("go_to_card answers without exploring", "F8a")
def m_no_explore(m):
    rewrite(m, "go_to_card", ("looked = explore(card)", "looked = {}"))


@mutant("go_to_card explores with 2 moves left or fewer", "F8b")
def m_explore_short(m):
    rewrite(m, "go_to_card", ("            if PILOT.short(2):\n", "            if False:\n"))


@mutant("go_to_card explores only with more than 3 moves left", "F8b")
def m_explore_short3(m):
    rewrite(m, "go_to_card", ("            if PILOT.short(2):\n", "            if PILOT.short(3):\n"))


NEVER_SEEN = '                if not any(s["card"] == card for s in PILOT.records()):\n                    raise Stuck("no moves left")'


@mutant("go_to_card short of moves answers did not find itself for a card never seen (before E6)", "F8b")
def m_short_never_seen(m):
    rewrite(m, "go_to_card", (NEVER_SEEN, "                pass"))


@mutant("go_to_card short of moves answers no moves left for a card seen too", "F8b")
def m_short_always_no_moves(m):
    rewrite(m, "go_to_card", (NEVER_SEEN, '                raise Stuck("no moves left")'))


@mutant("go_to_card does not pass on the gave up of its explore", "F8c")
def m_explore_gave_up_lost(m):
    rewrite(m, "go_to_card", ('            if looked.get("result") == "gave up":\n', "            if False:\n"))


# --- F9: go_to_card, 8 attempts and their two end messages (P17) ------------------------------------------------------
# The reason is kept during the attempts (ROBOT-PILOTE.md, E6): "no way ... now" as soon as one place had no way to it,
# whatever the count of places and whatever the 8th attempt did.
@scenario("F9a", "P17", "I12 I16")
def f9a_one_place_no_way():
    # One place (C3), its four fronts blocked: no way, then a wait of 10 s, again, for 8 attempts. The 8th waits and
    # empties the places with no way: the reason is still the way.
    h = Harness(at="A6", facing="north", toolset="goal")
    h.fake.seen = [record("barbecue", "C3", 60)]
    block(h, "B3", "D3", "C2", "C4")
    answer = h.tool("go_to_card", card="barbecue")
    check(h.calls("wait_for") == [["wait_for", {"seconds": 10}]] * 4, "4 waits of 10 s (attempts 2, 4, 6, 8)",
          h.calls("wait_for"))
    check(read_after_each_wait(h), "the state read again after each wait", h.calls())
    check(not h.moves(), "nothing moved", h.moves())
    check([l["tool"] for l in h.log()] == ["go_to_card"], "a place with no way is not ruled out: no explore",
          [l["tool"] for l in h.log()])
    check(answer == {"result": "gave up", "why": NO_WAY, "moves_left": None}, "gave up: no way to the card now", answer)


@scenario("F9b", "P17", "I12 I16")
def f9b_places_no_way():
    # Two places (C3, then D3), no way to either: C3 and D3 tried, a wait; three times, then C3 and D3 again.
    h = Harness(at="A6", facing="north", toolset="goal")
    h.fake.seen = [record("barbecue", "C3", 60, ["D3"])]
    block(h, "C4", "B3", "C2", "D4", "D2", "E3")
    answer = h.tool("go_to_card", card="barbecue")
    check(h.calls("wait_for") == [["wait_for", {"seconds": 10}]] * 2, "2 waits of 10 s (attempts 3 and 6)",
          h.calls("wait_for"))
    check(read_after_each_wait(h), "the state read again after each wait", h.calls())
    check(not h.moves(), "nothing moved", h.moves())
    check([l["tool"] for l in h.log()] == ["go_to_card"], "neither place ruled out: no explore",
          [l["tool"] for l in h.log()])
    check(answer == {"result": "gave up", "why": NO_WAY, "moves_left": None}, "gave up: no way to the card now", answer)
    # Three places (C3, then B3 and D3), no way to any: three tried, a wait, three again, a wait at the 8th attempt.
    # A3 is a box, which the first reading (column A) blocks; the others are blocked by hand, out of its sight.
    h = Harness(at="A6", facing="north", toolset="goal", boxes={"A3"})
    h.fake.seen = [record("barbecue", "C3", 60, ["B3", "D3"])]
    block(h, "C4", "C2", "B4", "B2", "D4", "D2", "E3")
    answer = h.tool("go_to_card", card="barbecue")
    check(h.calls("wait_for") == [["wait_for", {"seconds": 10}]] * 2, "three places: 2 waits of 10 s (attempts 4 and 8)",
          h.calls("wait_for"))
    check(not h.moves() and [l["tool"] for l in h.log()] == ["go_to_card"], "three places: nothing moved, no explore",
          [h.moves(), [l["tool"] for l in h.log()]])
    check(answer == {"result": "gave up", "why": NO_WAY, "moves_left": None},
          "three places: gave up, no way to the card now", answer)


@scenario("F9c", "P17", "I12 I16")
def f9c_one_place_no_way_one_reached():
    # C3 has no way to it; E5 is reached, but the quarter turn towards it is always refused, so no attempt waits and
    # E5 is never ruled out. One place had no way: the reason is the way, not "did not find itself" (E6).
    h = Harness(at="A6", facing="north", toolset="goal")
    h.fake.seen = [record("barbecue", "C3", 60), record("barbecue", "E5", 90)]
    block(h, "B3", "D3", "C2", "C4", "D5", "F5", "E4")

    def face_refused(direction, **_):
        if (h.cell(), direction) in (("E6", "north"), ("D5", "east"), ("F5", "west"), ("E4", "south")):
            return {"result": "refused", "why": "the robot is still moving"}
        return None
    h.fake.script["face"] = face_refused
    answer = h.tool("go_to_card", card="barbecue")
    check(h.calls("wait_for") == [], "no wait: a place was always left to try", h.calls("wait_for"))
    check(h.moves() == [face("east"), slide("east", 4)] + [face("north")] * 7,
          "to E6, then the refused face north at each of the 7 other attempts", h.moves())
    check(answer == {"result": "gave up", "why": NO_WAY, "moves_left": None}, "gave up: no way to the card now", answer)


@mutant("go_to_card tries 80 times", "F9a", "F9b")
def m_80_attempts(m):
    rewrite(m, "go_to_card", ("for attempt in range(8):", "for attempt in range(80):"))


@mutant("a place with no way is ruled out", "F9a")
def m_no_way_ruled_out(m):
    rewrite(m, "go_to_card", ("unreachable.add(cell)", "ruled_out.add(cell)"))


@mutant("go_to_card tries again without waiting", "F9a", "F9b")
def m_card_no_wait(m):
    rewrite(m, "go_to_card", ("wait(10)", "None"))


@mutant("go_to_card ends on did not find itself, way or no way", "F9a", "F9b")
def m_one_message(m):
    rewrite(m, "go_to_card", ("    if no_way:\n        return", "    if False:\n        return"))


@mutant("go_to_card's end message follows its last attempt (if unreachable, before E6)", "F9a", "F9b")
def m_last_attempt_message(m):
    rewrite(m, "go_to_card", ("    if no_way:\n        return", "    if unreachable:\n        return"))


@mutant("go_to_card tries again without reading the state after the wait", "F9a", "F9b")
def m_no_read_after_wait(m):
    rewrite(m, "go_to_card", ('    PILOT.read(command("robot_state"))\n            unreachable.clear()',
                              '    unreachable.clear()'))


@mutant("go_to_card says no way only once every place had none", "F9c")
def m_no_way_when_all(m):
    rewrite(m, "go_to_card", ("            unreachable.add(cell)  # no way there now: try the next place, and this one "
                              "again later\n            no_way = True\n",
                              "            unreachable.add(cell)  # no way there now: try the next place, and this one "
                              "again later\n"),
            ("            unreachable.clear()\n", "            unreachable.clear()\n            no_way = True\n"))


# --- F10: the robot waits for an adult --------------------------------------------------------------------------------
@scenario("F10a", "P8", "I7")
def f10a_adult_under_strategist():
    h = Harness(at="C5", facing="north")
    waiting(h, (2, 4, 0))
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "gave up", "why": ADULT, "cards_seen": {}, "moves_left": None},
          "under the Strategist: gave up at once, the robot waits for an adult", answer)
    check(names(h) == ["seen_cards", "robot_state", "seen_cards"], "one read, no wait for the adult", h.calls())
    check(h.P.state == {}, "the state forgotten: the adult will put the robot somewhere", h.P.state)


@scenario("F10b", "P15", "I7")
def f10b_adult_never_comes():
    h = Harness(at="C5", facing="north", obey=False)
    waiting(h, (2, 4, 0))
    t0 = h.clock.monotonic()
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "gave up", "why": STILL, "cards_seen": {}, "moves_left": None},
          "without the Strategist: gave up after the waits, the robot still waits", answer)
    check(names(h) == ["seen_cards", "robot_state"] + ["wait_for", "robot_state"] * 5 + ["seen_cards"],
          "five waits, each followed by a read, nothing moved", h.calls())
    check(h.calls("wait_for") == [["wait_for", {"seconds": 30}]] * 5, "waits of 30 s", h.calls("wait_for"))
    check(abs(h.clock.monotonic() - t0 - 151.2) < 1e-6, "151.2 s in the tool: 0.2 s, then 5 slices of 30.2 s",
          h.clock.monotonic() - t0)


@scenario("F10c", "P8", "I7 I8")
def f10c_resume_then_leave_lane():
    h = Harness(at="C5", facing="north", obey=False)
    waiting(h, (2, 4, 0))   # put back on C5, facing north: row 5 is a ball's lane
    lanes(h, ["row 5"])
    resume_at(h, 2)
    answer = h.tool("go_to_cell", cell="C3")
    check(answer["result"] == "done" and answer["cell"] == "C3" and answer["facing"] == "north",
          "done on C3, facing north", answer)
    check(answer["state"] is h.P.state and answer["moves_left"] is None, "the state last read, no limit", answer)
    # Resumed on C5, in the lane: out of it first (to C4, the cheapest cell off the lane), before the cards are read
    # again for C3; then C4 -> C3.
    check(names(h) == ["seen_cards", "robot_state", "wait_for", "robot_state", "wait_for", "robot_state",
                       "seen_cards", "seen_cards", "move", "seen_cards", "seen_cards", "move"],
          "two waits, Resume, out of the lane, then the way", h.calls())
    check(h.calls("move") == [slide("north", 1)] * 2, "two slides north of one cell", h.calls("move"))
    # A cell on the lane: go_to_cell refuses it without travelling (pilot.py:705-710). Only the wait for the adult
    # takes the robot out of the lane (pilot.py:303), so it stands on C4, not C5.
    h = Harness(at="C5", facing="north", obey=False)
    waiting(h, (2, 4, 0))
    lanes(h, ["row 5"])
    resume_at(h, 1)
    answer = h.tool("go_to_cell", cell="D5")
    check(answer == {"result": "refused", "why": "D5 is on the lane of a ball: the robot never stops where a ball rolls",
                     "what_to_do": "go to D4 instead, next to it and off every lane", "moves_left": None},
          "D5 refused, D4 proposed", answer)
    check(h.cell() == "C4", "out of the lane after Resume, even when the cell asked for is refused", h.cell())
    check(names(h) == ["seen_cards", "robot_state", "wait_for", "robot_state", "seen_cards", "seen_cards", "move",
                       "seen_cards", "seen_cards"], "Resume, out of the lane, then the refusal", h.calls())


@scenario("F10d", "P8 P9", "I7")
def f10d_adult_while_localizing():
    h = Harness(at="C5", facing="north", frame=True)

    def stopped(**_):   # the first quarter turn of the look_around is stopped: an adult must come
        waiting(h, (2, 4, 0))
        return {"result": "stopped", "why": "something touched the robot's bumper", "position_known": False,
                "what_to_do": h.fake.instruction()}
    h.fake.script["look_around"] = stopped
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "gave up", "why": ADULT, "cards_seen": {}, "moves_left": None},
          "under the Strategist: gave up as soon as localize reads the robot waits", answer)
    check(names(h) == ["seen_cards", "robot_state", "look_around", "robot_state", "seen_cards"],
          "no step and no second look_around while the robot waits", h.calls())
    check(h.P.state == {}, "the state forgotten", h.P.state)


@mutant("under the Strategist, wait_for_adult waits itself", "F10a")
def m_adult_no_ask(m):
    rewrite(m, "Pilot.wait_for_adult", ("if ASK is not None:", "if False:"))


@mutant("under the Strategist, the state is kept for the adult", "F10a")
def m_adult_keeps_state(m):
    rewrite(m, "Pilot.wait_for_adult", ("self.state = {}", "pass"))


@mutant("gave up for the adult without a mission too", "F10b")
def m_adult_always_gives_up(m):
    rewrite(m, "Pilot.wait_for_adult", ("if ASK is not None:", "if True:"))


@mutant("no wait for the adult (ADULT_WAIT_S 0)", "F10b")
def m_adult_wait_zero(m):
    m.ADULT_WAIT_S = 0


@mutant("Resume not seen: the wait goes on", "F10c")
def m_resume_ignored(m):
    rewrite(m, "Pilot.wait_for_adult", ('if not self.state.get("waiting_for_adult"):', "if False:"))


@mutant("after Resume, no leaving the lane", "F10c")
def m_resume_stays_on_lane(m):
    rewrite(m, "Pilot.wait_for_adult", ("self.leave_lane()", "pass"))


@mutant("localize does not see the robot waits for an adult", "F10d")
def m_localize_no_adult(m):
    rewrite(m, "Pilot.localize", ('if self.state.get("waiting_for_adult"):\n                self.wait_for_adult()',
                                  "pass"))


# --- F11: the place on the mat is not known ---------------------------------------------------------------------------
@scenario("F11a", "P9 P10", "I7 I16")
def f11a_two_places():
    h = Harness(at="C5", facing="north")
    lost(h, [2])
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "gave up", "why": LOST, "cards_seen": {}, "moves_left": None},
          "two places, always the same: gave up", answer)
    check(names(h) == ["seen_cards", "robot_state", "look_around", "robot_state", "step_forward", "robot_state",
                       "look_around", "robot_state", "seen_cards"],
          "a look_around, a step, a look_around that rules nothing out", h.calls())


@scenario("F11b", "P9 P10", "I7 I16")
def f11b_more_than_six():
    h = Harness(at="C5", facing="north")
    lost(h, [None])   # "unknown", no could_be: what the Cartographer says with more than 6 places
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "gave up", "why": LOST, "cards_seen": {}, "moves_left": None},
          "more than 6 places: gave up", answer)
    check(names(h).count("look_around") == 2 and names(h).count("step_forward") == 1,
          "two look_arounds: more than 6 counts as the same number each round", h.calls())
    raise Known("with more than 6 places the Cartographer sends no could_be, so localize counts 99 places each round "
                "and gives up at the second round even when places were ruled out (40, then 8), not only when none was "
                "as 5-pilote.md P10 says: stack/pilot.py:318-319, stack/cartographer.py:1019-1022")


@scenario("F11c", "P9 P10", "I7 I16")
def f11c_never_repeats():
    h = Harness(at="C5", facing="north")
    lost(h, [None, 6, 5, 4, 3, 2, 1, None])
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "gave up", "why": LOST, "cards_seen": {}, "moves_left": None},
          "eight rounds without finding its place: gave up", answer)
    got = names(h)
    # C5 -> C1 north, refused off the mat on the fifth step, a quarter turn right, then D1, E1, F1.
    check(got.count("look_around") == 8 and got.count("step_forward") == 8 and got.count("turn") == 1,
          "8 look_arounds, 8 steps, one quarter turn where the step was refused", h.calls())
    check(h.calls("turn") == [["turn", {"direction": "right"}]], "the quarter turn is to the right", h.calls("turn"))
    check(h.cell() == "F1", "the robot ends on F1", h.cell())


@scenario("F11d", "P10", "I7 I16")
def f11d_moves_short():
    h = Harness(at="C5", facing="north", limit=3)
    lost(h, [2])
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "gave up", "why": LOST, "cards_seen": {}, "moves_left": 2},
          "2 moves left after one look_around: gave up", answer)
    check(names(h) == ["seen_cards", "robot_state", "look_around", "robot_state", "seen_cards"],
          "one look_around, no step", h.calls())
    # With 4: 3 left after the first look_around, a step; 1 left after the second, which still rules places out.
    h = Harness(at="C5", facing="north", limit=4)
    lost(h, [3, 2])
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "gave up", "why": LOST, "cards_seen": {}, "moves_left": 1},
          "3 moves left: a step; 1 left: gave up", answer)
    check(names(h) == ["seen_cards", "robot_state", "look_around", "robot_state", "step_forward", "robot_state",
                       "look_around", "robot_state", "seen_cards"], "a step at 3 moves left, none at 1", h.calls())


@mutant("localize goes on when a round ruled nothing out", "F11a", "F11b")
def m_localize_no_repeat(m):
    rewrite(m, "Pilot.localize", ("if self.short(2) or now == places:", "if self.short(2):"))


@mutant("localize ignores the moves left", "F11d")
def m_localize_no_short(m):
    rewrite(m, "Pilot.localize", ("if self.short(2) or now == places:", "if now == places:"))


@mutant("localize stops at 3 moves left", "F11d")
def m_localize_short3(m):
    rewrite(m, "Pilot.localize", ("self.short(2)", "self.short(3)"))


@mutant("localize tries 9 rounds", "F11c")
def m_localize_nine(m):
    rewrite(m, "Pilot.localize", ("for _ in range(8):", "for _ in range(9):"))


# --- F12: balls' lanes (P20) ------------------------------------------------------------------------------------------
@scenario("F12", "P20", "I8")
def f12_lanes():
    # (a) look() from a ball's lane: first off every lane (C3: one cell, seen free from C4), then the look; no look
    # from row 4.
    h = Harness(at="C4", facing="north")
    lanes(h, ["row 4"])
    seen = where_calls(h)
    answer = h.tool("look_around")
    check(answer["result"] == "done", "the look is done", answer)
    check(acting(h) == [["move", "north", 1], ["look_around", None, None]], "slide north 1 to C3, then look",
          acting(h))
    check([pos for name, _, pos in seen if name == "look_around"] == ["C3"], "the look from C3", seen)
    # Row 4 and column C: no cell one step away is off both lanes; the look goes two cells away (pilot.py:234).
    h = Harness(at="C4", facing="north")
    lanes(h, ["row 4", "column C"])
    seen = where_calls(h)
    answer = h.tool("look_around")
    check(answer["result"] == "done", "two lanes: the look is done", answer)
    check([pos for name, _, pos in seen if name == "look_around"] in (["B3"], ["D3"], ["B5"], ["D5"]),
          "two lanes: the look from B3, D3, B5 or D5", seen)
    # (b) go_to_card: the arrival off the lane (B3, C2 or D3), never C4, where the robot would face the card on row 4.
    h = Harness(at="C6", facing="north", truth={"C3": "barbecue"}, toolset="goal")
    h.fake.seen = [record("barbecue", "C3", 90, seen_from="C6 facing north")]
    lanes(h, ["row 4"])
    seen = where_calls(h)
    answer = h.tool("go_to_card", card="barbecue")
    check(answer.get("result") == "done" and answer.get("card_ahead") == "barbecue", "done, facing the barbecue",
          answer)
    front = {"B3": "east", "C2": "south", "D3": "west"}
    check(answer.get("cell") in front and front[answer["cell"]] == answer.get("facing"),
          "in front of C3 from B3, C2 or D3", answer)
    check(not [s for s in seen if s[2][1:] == "4"], "the robot never stood still on row 4", seen)
    # (c) go_to_cell onto a lane: refused at once, with the nearest cell off every lane, or the advice to wait.
    h = Harness(at="C5", facing="north")
    lanes(h, ["row 3"])
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "refused", "why": "C3 is on the lane of a ball: the robot never stops where a ball rolls",
                     "what_to_do": "go to C4 instead, next to it and off every lane", "moves_left": None},
          "refused, go to C4 instead", answer)
    check(not h.moves(), "nothing moved", h.moves())
    h = Harness(at="C5", facing="north")
    lanes(h, ["row 2", "row 3", "row 4", "column B", "column C", "column D"])
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "refused", "why": "C3 is on the lane of a ball: the robot never stops where a ball rolls",
                     "what_to_do": "wait for the ball to go (wait_for_event), look around, then choose again",
                     "moves_left": None}, "refused, no cell off every lane within two steps: wait", answer)
    check(not h.moves(), "nothing moved", h.moves())
    # Row 3 and column C: the four cells next to C3 are on a lane; B4 and D4, two steps away, are the nearest to the
    # robot in C5 (B4 first, pilot.py:212-214).
    h = Harness(at="C5", facing="north")
    lanes(h, ["row 3", "column C"])
    answer = h.tool("go_to_cell", cell="C3")
    check(answer == {"result": "refused", "why": "C3 is on the lane of a ball: the robot never stops where a ball rolls",
                     "what_to_do": "go to B4 instead, next to it and off every lane", "moves_left": None},
          "refused, go to B4 instead (two steps away)", answer)
    check(not h.moves(), "nothing moved", h.moves())
    # (d) travel leaves the lane before it plans the way (pilot.py:444-445): from C4 on row 4 to C2, first a stop on
    # C3, off the lane, then the way; not one slide of 2 cells planned from the lane.
    h = Harness(at="C4", facing="north")
    lanes(h, ["row 4"])
    answer = h.tool("go_to_cell", cell="C2")
    check(answer["result"] == "done" and answer["cell"] == "C2", "done on C2", answer)
    check(acting(h) == [["move", "north", 1], ["move", "north", 1]], "off the lane on C3 first, then on to C2",
          acting(h))


@mutant("look_around from a ball's lane", "F12")
def m_look_on_lane(m):
    look = m.Pilot.look_around

    def patched(self):
        self.leave_lane = lambda: None
        try:
            return look(self)
        finally:
            del self.leave_lane
    m.Pilot.look_around = patched


@mutant("leave_lane one cell away at most", "F12")
def m_leave_one_cell(m):
    rewrite(m, "Pilot.leave_lane", ("abs(x - c) + abs(y - r) <= 2", "abs(x - c) + abs(y - r) <= 1"))


@mutant("travel plans from a ball's lane", "F12")
def m_travel_on_lane(m):
    rewrite(m, "Pilot.travel", ("            if not self.leaving:\n                self.leave_lane()",
                                "            pass"))


@mutant("go_to_card arrives on a lane like anywhere", "F12")
def m_card_on_lane(m):
    rewrite(m, "go_to_card", ("goals = {f for f in goals if not PILOT.on_lane(f)} or goals", "pass"))


@mutant("go_to_cell goes onto a lane", "F12")
def m_cell_on_lane(m):
    rewrite(m, "go_to_cell", ("if PILOT.on_lane(target):", "if False:"))


@mutant("off_lane_near proposes a cell on a lane", "F12")
def m_near_on_lane(m):
    rewrite(m, "Pilot.off_lane_near", ("not self.on_lane((c, r)) and ", ""))


@mutant("off_lane_near one step away at most", "F12")
def m_near_one_step(m):
    rewrite(m, "Pilot.off_lane_near", ("for d in (1, 2):", "for d in (1,):"))


# --- F13: reads during a pause (P11) ----------------------------------------------------------------------------------
@scenario("F13", "P11", "I14")
def f13_reads_in_a_pause():
    # Inside the Pilot, the reads do not wait for the end of a pause and give no acknowledgement. The pause lasts 10
    # minutes: long against the 5 s watch, short enough for a mutant that waits to end.
    h = Harness(at="C5", facing="north", limit=6, toolset="route")
    h.mission.pause(600, "go")
    t = h.clock.monotonic()
    end = t + 600
    answer = h.tool("robot_state")
    check(answer == h.fake.op["answers"][0] and answer["moves_left"] == 6, "state: the fake's answer, moves_left 6",
          answer)
    check(answer["cell"] == "C5" and h.calls() == [["robot_state", {}]], "one read of the state", h.calls())
    check(abs(h.clock.monotonic() - t - 0.2) < 1e-6, "0.2 s for the state", h.clock.monotonic() - t)
    # The Pilot adds the moves left it read last, when the Cartographer's state has none; and logs the tool.
    h.fake.state_hook = lambda state: state.pop("moves_left", None) and None
    answer = h.tool("robot_state")
    check("moves_left" not in h.fake.op["answers"][0], "the fake's state without moves_left", h.fake.op["answers"][0])
    check(answer == dict(h.fake.op["answers"][0], moves_left=6), "state: moves_left 6 added by the Pilot", answer)
    check([line["tool"] for line in h.log()] == ["state"], "one line in pilot.jsonl", h.log())
    h.fake.state_hook = None
    t = h.clock.monotonic()
    answer = h.tool("seen_cards")
    check(answer == h.fake.op["answers"][0] and "cards_seen" in answer, "seen_cards: the fake's answer", answer)
    check(h.clock.monotonic() == t, "no time for the cards", h.clock.monotonic() - t)
    t = h.clock.monotonic()
    answer = h.tool("wait_for_event", seconds=5)
    check(answer == {"result": "nothing happened", "waited_seconds": 5}, "watch: the fake's answer", answer)
    check(abs(h.clock.monotonic() - t - 5) < 1e-6, "5 s for the watch", h.clock.monotonic() - t)
    check(h.common.acks == 0 and pp.common.held_where() is None, "no acknowledgement of the pause",
          [h.common.acks, pp.common.held_where()])
    check(h.mission.now() == "pause", "the mission is still paused", h.mission.now())
    # The control: set_light, a command that acts, waits for the end of the pause and acknowledges it once.
    answer = h.tool("set_light", color=pp.COLORS[0])
    check(answer.get("result") == "done" and h.clock.monotonic() >= end, "set_light done after the pause",
          [answer, end - h.clock.monotonic()])
    check(h.common.acks == 1 and pp.common.held_where() is None, "one acknowledgement, cleared before the light",
          [h.common.acks, pp.common.held_where()])


@mutant("robot_state without @logged", "F13")
def m_state_not_logged(m):
    def state():
        return m.PILOT.read(m.command("robot_state"))
    m.TOOLSETS["route"]["robot_state"] = (state,) + m.TOOLSETS["route"]["robot_state"][1:]


@mutant("every read waits for the end of a pause", "F13")
def m_reads_wait(m):
    command = m.command

    def patched(name, *args):
        if m.ASK is not None:
            m.hold_still()
        return command(name, *args)
    m.command = patched


# --- F14: configure (P1a, P1b, P22, P23) ------------------------------------------------------------------------------
@scenario("F14", "P1a P1b P22 P23", "I10")
def f14_configure():
    h = Harness(limit=6)
    defaults = {k: v["default"] for k, v in h.pilot.SETTINGS.items()}
    h.P.state, h.P.moves_left = {"cell": "C5", "position_known": True}, 0
    for settings, why in [({"max_leg": 2, "x": 1}, "no setting x for the Pilot"),
                          ({"careful_balls": "no"}, "the setting careful_balls is a bool"),
                          ({"max_leg": True}, "the setting max_leg is a int")]:
        before = dict(h.P.settings)
        try:
            h.P.configure(settings)
            check(False, f"configure({settings}) refused")
        except ValueError as e:
            check(str(e) == why, f"configure({settings}) says {why!r}", str(e))
        check(h.P.settings == before, "no setting changed", h.P.settings)
        check(h.P.state == {} and h.P.moves_left == 6, "the state and the count forgotten, even when refused",
              [h.P.state, h.P.moves_left])
    h.P.configure({"max_leg": 9})
    check(h.P.settings["max_leg"] == 4, "max_leg 9 becomes 4", h.P.settings)
    h.P.configure({"max_leg": 0})
    check(h.P.settings["max_leg"] == 1, "max_leg 0 becomes 1", h.P.settings)
    h.P.configure({"look_each_leg": True, "max_leg": 3})
    h.P.configure({})
    check(h.P.settings == defaults, "a mission without bricks starts from the defaults", h.P.settings)


@mutant("configure forgets the state only when it accepts", "F14")
def m_forget_late(m):
    configure = m.Pilot.configure
    def patched(self, settings):
        state, left = self.state, self.moves_left
        try:
            configure(self, settings)
        except ValueError:
            self.state, self.moves_left = state, left
            raise
    m.Pilot.configure = patched


# --- F15: explore under a limit of the adults, and without a bound ----------------------------------------------------
@scenario("F15", "-", "I16")
def f15_explore_limit():
    # Every command that acts refused for good (a commands limit, no moves limit: short() never sees it). A refusal
    # that says a limit of the adults ends the tool at once, with its own words (ROBOT-PILOTE.md, E2 (a)). Before, the
    # refused look added nothing to covered, the robot's own cell stayed the best vantage, and explore never ended.
    h = Harness(at="C5", facing="north", toolset="goal")
    for name in ACTS:
        h.fake.script[name] = lambda **_: dict(NO_COMMANDS)
    answer = explore_bounded(h, 300)
    check(answer == {"result": "gave up", "why": NO_COMMANDS["why"], "cards_seen": {}, "moves_left": None},
          "commands limit: gave up with the Cartographer's words", answer)
    check(names(h) == ["robot_state", "look_around", "seen_cards"], "a reading, the refused look, the record for "
          "cards_seen; nothing after the refusal", h.calls())
    check(h.cell() == "C5" and lines(h) == [("explore", [[], {}], answer)], "nothing moved, one line",
          [h.cell(), lines(h)])
    # A cells limit: the first look is done, a slide to the next vantage too; the look there is refused.
    h = Harness(at="C5", facing="north", toolset="goal")
    looks = []

    def look(**_):
        looks.append(h.cell())
        return dict(NO_CELLS) if len(looks) == 2 else None
    h.fake.script["look_around"] = look
    answer = explore_bounded(h, 300)
    check(answer.get("result") == "gave up" and answer.get("why") == NO_CELLS["why"]
          and set(answer) == {"result", "why", "cards_seen", "moves_left"}, "cells limit: gave up with the "
          "Cartographer's words", answer)
    check(len(looks) == 2 and looks[1] != "C5" and h.calls()[-2:] == [["look_around", {}], ["seen_cards", {}]],
          "the second look, refused on another cell, is the last command", [looks, h.calls()[-3:]])


@scenario("F15b", "-", "I16")
def f15b_explore_bounded():
    # Every look_around refused for a reason that is neither a limit of the adults nor the battery (the robot still
    # moving): the robot's own cell C5 stays the best vantage, travel there is True at once and a refused look adds
    # nothing to covered. The loop ends after VANTAGES rounds (ROBOT-PILOTE.md, E2 (b)): done, the mat not covered.
    h = Harness(at="C5", facing="north", toolset="goal")
    for name in ACTS:
        h.fake.script[name] = lambda **_: dict(STILL_MOVING)
    answer = explore_bounded(h, 3000)
    looks = len(h.calls("look_around"))
    check(answer.get("result") == "done" and answer.get("mat_covered") is False and answer.get("cards_seen") == {},
          "done, the mat not covered, no card", answer)
    check(looks == h.pilot.VANTAGES + 1, f"one look, then one per round: {h.pilot.VANTAGES + 1} looks", looks)
    calls = names(h)
    check(all(calls[k + 1:k + 2] == ["robot_state"] for k, n in enumerate(calls) if n == "look_around"),
          "each refused look is followed by a reading (it carries no state)", calls[:12])
    check(h.P.covered == set() and h.cell() == "C5", "nothing covered, nothing moved", [sorted(h.P.covered), h.cell()])
    # Under the Strategist, a stop ends it sooner: the next look raises it.
    h = Harness(at="C5", facing="north", toolset="goal")
    for name in ACTS:
        h.fake.script[name] = lambda **_: dict(STILL_MOVING)
    looks = []

    def look(**_):
        looks.append(1)
        if len(looks) == 20:
            h.mission.stop()
        return dict(STILL_MOVING)
    h.fake.script["look_around"] = look
    answer = explore_bounded(h, 3000)
    check(answer == {"result": "gave up", "why": "the mission was stopped", "cards_seen": {}, "moves_left": None},
          "a stop ends it", answer)
    check(len(looks) == 20 and not h.calls("move", "face", "turn", "step_forward"), "20 refused looks, no other move",
          names(h)[-6:])


@mutant("explore without its bound (E2 (b))", "F15b")
def m_explore_unbounded(m):
    m.VANTAGES = 10 ** 6


@mutant("a refusal for a limit of the adults goes on (before E2 (a))", "F15", "F23")
def m_limit_goes_on(m):
    rewrite(m, "Pilot.read", ("            raise Stuck(answer[\"why\"])", "            pass"))


@mutant("only the moves limit ends a tool", "F15", "F23")
def m_limit_moves_only(m):
    rewrite(m, "Pilot.read", ('"the adults allow" in str(answer.get("why", ""))',
                              'str(answer.get("why", "")).startswith("no moves left")'))


@mutant("a limit of the adults ends the tool with its own words lost", "F15", "F23")
def m_limit_words_lost(m):
    rewrite(m, "Pilot.read", ("            raise Stuck(answer[\"why\"])", "            raise Stuck(\"no moves left\")"))


@mutant("a refused look_around does not read the state again", "F15b")
def m_refused_look_no_read(m):
    rewrite(m, "Pilot.look_around", ('            self.read(command("robot_state"))  # refused after',
                                     "            pass  # refused after"))


# --- F16, F17: two legs that go wrong (P2) ----------------------------------------------------------------------------
@scenario("F16", "P2", "I17")
def f16_busy_blocks():
    # A refusal blocks the next cell only when it says a thing in front (ROBOT-PILOTE.md, E9): "another move is still
    # running" blocks nothing, and the same slide goes again.
    h = Harness(at="C5", facing="north")
    refused = []

    def once(**args):
        if not refused:
            refused.append(args)
            return {"result": "refused", "why": BUSY}  # the Cartographer's words for the rule one_move_at_a_time
        return None
    h.fake.script["move"] = once
    answer = h.tool("go_to_cell", cell="C3")
    check(refused == [slide("north", 2)[1]], "the first slide refused: another move is running", refused)
    check(h.moves() == [slide("north", 2)] * 2, "C4, seen free, not blocked: the same slide again", h.moves())
    check(answer["result"] == "done" and answer["cell"] == "C3", "done on C3", answer)
    check(not h.P.is_blocked(xy("C4")), "C4 not blocked", sorted(h.P.blocked))
    # The other way round: "something is in front of the robot" (REASONS["obstacle"]) blocks C4, and the way goes round.
    h = Harness(at="C5", facing="north")
    refused = []

    def obstacle(**args):
        if not refused:
            refused.append(args)
            return {"result": "refused", "why": OBSTACLE}
        return None
    h.fake.script["move"] = obstacle
    answer = h.tool("go_to_cell", cell="C3")
    check(h.P.is_blocked(xy("C4")), "an obstacle refused: C4 blocked", sorted(h.P.blocked))
    check(either(h.moves(), [slide("north", 2)] + BY_B, [slide("north", 2)] + BY_D),
          "an obstacle refused: round C4 by column B or D (10 each)", h.moves())
    check(answer["result"] == "done" and answer["cell"] == "C3", "an obstacle refused: done on C3", answer)


@scenario("F17", "P2", "I17")
def f17_face_shows_blocked():
    h = Harness(at="C5", facing="east", boxes={"C4"})
    sent = []

    def note(**args):
        sent.append((args, h.P.is_blocked(xy("C4"))))  # what the Pilot knew of C4 when the slide left
        return None
    h.fake.script["move"] = note
    answer = h.tool("go_to_cell", cell="C3")
    # Facing east, the Pilot knows nothing of C4: north 2 (6) is the cheapest. The face north reads 29 cm: C4 blocked,
    # and the leg ends there, without a slide (ROBOT-PILOTE.md, E9). The new plan goes by column D, whose D5 the first
    # reading saw free (10).
    check(sent[:1] == [(slide("east", 1)[1], True)], "no slide north from C5: the first slide goes east, C4 blocked",
          sent)
    want = [face("north"), face("east"), slide("east", 1), face("north"), slide("north", 2),
            face("west"), slide("west", 1)]
    check(h.moves() == want, "the face north, then round C4 by column D", h.moves())
    check(answer["result"] == "done" and answer["cell"] == "C3" and answer["facing"] == "west", "done on C3", answer)
    # The other half of E9: the state read after the face names a card on the first cell (card_ahead, the signal that
    # stops a step). The plan went round the cards the Pilot knew; this one is not in the record yet. On the real stack
    # the Cartographer records the card the camera saw, so the next plan goes round it: the check saves one refused
    # slide per card newly seen. The fake records nothing, so without the check it slid north 8 times, all stopped.
    # (b) A5 facing south, a card on A4 the record does not hold: no slide north after the face north.
    h = Harness(at="A5", facing="south", truth={"A4": "barbecue"})
    answer = h.tool("go_to_cell", cell="A2")
    after = h.moves()[h.moves().index(face("north")) + 1:] if face("north") in h.moves() else None
    check(after is not None and not [m for m in after if m[0] == "move"], "(b) the face north showed a card on A4: no "
          "slide after it", h.moves())
    # (c) C5 facing east, a card on C4: the face north shows it, and the camera records it then (as the real
    # Cartographer does): no slide north, the new plan goes round C4 by column B or D.
    h = Harness(at="C5", facing="east", truth={"C4": "water-point"})

    def seen_on_face(direction, **_):
        if direction == "north" and h.cell() == "C5":
            h.fake.seen = [record("water-point", "C4", 30)]
        return None
    h.fake.script["face"] = seen_on_face
    answer = h.tool("go_to_cell", cell="C3")
    check(h.moves()[:1] == [face("north")] and h.moves()[1:2] != [slide("north", 2)],
          "(c) the face north showed a card on C4: no slide north from C5", h.moves())
    check(either(h.moves(), [face("north")] + BY_B, [face("north")] + BY_D), "(c) the face north, then round C4 by "
          "column B or D", h.moves())
    check(answer["result"] == "done" and answer["cell"] == "C3", "(c) done on C3",
          {k: v for k, v in answer.items() if k != "state"})


FIRST_TAKEN = ('            if self.is_blocked((c0 + dc, r0 + dr)) or (self.state.get("card_ahead") or {}).get("card"):'
               '\n')
CARD_AHEAD = ' or (self.state.get("card_ahead") or {}).get("card"):\n'


@mutant("a refused leg blocks nothing", "F16")
def m_refusal_blocks_nothing(m):
    rewrite(m, "Pilot.leg", ('if answer.get("why") == REASONS["obstacle"] and not self.ball_recent():', "if False:"))


@mutant("any refusal without card blocks the next cell (before E9)", "F16")
def m_refusal_blocks_any(m):
    rewrite(m, "Pilot.leg", ('if answer.get("why") == REASONS["obstacle"] and not self.ball_recent():',
                             'if "card" not in why and not self.ball_recent():'))


@mutant("a leg slides after its face showed the first cell blocked (before E9)", "F17")
def m_leg_ignores_face(m):
    rewrite(m, "Pilot.leg", (FIRST_TAKEN, "            if False:\n"))


@mutant("a leg slides after its face showed a card on the first cell (before E9)", "F17")
def m_leg_ignores_card_ahead(m):
    rewrite(m, "Pilot.leg", (CARD_AHEAD, ":\n"))


@mutant("the cells before the thing are not marked free", "F17")
def m_sensor_no_free(m):
    rewrite(m, "Pilot.note_distance", ("self.free.add((cell[0] + dc * d, cell[1] + dr * d))", "pass"))


# --- F18: done only with a place known (P5) ---------------------------------------------------------------------------
@scenario("F18", "P5", "I15")
def f18_done_place_lost():
    # The card on C3, sure; the robot slides to C4 and faces it. The reading in front of the card says the place is
    # lost (the real Cartographer still gives card_ahead then, stack/cartographer.py:995-999). go_to_card finds its
    # place first (one look_around, which finds it), then tries again: done names C4 (ROBOT-PILOTE.md, E3).
    h = Harness(at="C5", facing="north", truth={"C3": "barbecue"}, toolset="goal")
    h.fake.seen = [record("barbecue", "C3", 60)]
    lost_once = []

    def robot_state():
        if h.cell() != "C4" or lost_once:
            return None
        lost_once.append(1)
        state = h.fake.state()
        h.fake.frame = True  # a reading farther than the border: the Cartographer counts from here again
        return dict(state, cell=None, facing=None, position_known=False)
    h.fake.script["robot_state"] = robot_state

    def found(**_):
        h.fake.frame = False  # the look_around finds the place again
    h.fake.script["look_around"] = found
    answer = h.tool("go_to_card", card="barbecue")
    check(lost_once, "the reading in front of the card said the place lost", lost_once)
    check(h.moves() == [slide("north", 1), ["look_around", {}]], "one leg to C4, then one look to find its place",
          h.moves())
    check(answer == {"result": "done", "cell": "C4", "facing": "north", "card_ahead": "barbecue", "moves_left": None},
          "done from C4 facing north, once the place is known", answer)
    check([l["tool"] for l in h.log()] == ["go_to_card"], "no explore", [l["tool"] for l in h.log()])


@mutant("go_to_card trusts the state of its last move, not a new reading", "F18")
def m_no_new_reading(m):
    rewrite(m, "go_to_card", ('state = PILOT.read(command("robot_state"))', "state = PILOT.state"))


@mutant("go_to_card answers done with the place lost (before E3)", "F18")
def m_done_place_lost(m):
    rewrite(m, "go_to_card", ('        if not state.get("position_known"):\n', "        if False:\n"))


# --- F19: face, in the route set --------------------------------------------------------------------------------------
@scenario("F19a", "P9", "I7")
def f19a_face_no_read():
    h = Harness(at="C5", facing="east")   # just after configure: the Pilot knows nothing of the robot
    answer = h.tool("face", direction="north")
    check(h.calls() == [face("north")], "one call, face north, no read before", h.calls())
    check(answer == {"result": "done", "state": h.P.state, "moves_left": None} and h.P.state["facing"] == "north",
          "done, with the state of the face's answer", answer)
    check(pp.FACING[h.fake.facing] == "north", "the robot faces north", h.fake.facing)


@scenario("F19b", "P8", "I7")
def f19b_face_while_waiting():
    h = Harness(at="C5", facing="east")
    waiting(h, (2, 4, 1))
    answer = h.tool("face", direction="north")
    check(names(h) == ["face", "robot_state"], "the face refused without a state, then one read", h.calls())
    check(answer == {"result": "not done", "state": h.P.state, "moves_left": None}, "not done, with the state", answer)
    check(h.P.state.get("waiting_for_adult") is True and h.P.state.get("position_known") is False,
          "the state says the robot waits for an adult, and is kept", h.P.state)
    # Decided (ROBOT-PILOTE.md, E8): not done with the state, not gave up; the Strategist's loop sees waiting_for_adult.


@scenario("F19c", "P9", "I7")
def f19c_face_place_unknown():
    h = Harness(at="C5", facing="east", frame=True)
    h.tool("robot_state")   # the Pilot has read that the place is not known
    check(h.P.state.get("position_known") is False, "the place is not known", h.P.state)
    answer = h.tool("face", direction="north")
    check(names(h) == ["face", "robot_state"], "face does not find its place first: refused, then one read", h.calls())
    check(answer == {"result": "not done", "state": h.P.state, "moves_left": None}, "not done, with the state", answer)
    check(pp.FACING[h.fake.facing] == "east", "the robot did not turn", h.fake.facing)


@mutant("face finds its place first", "F19a")
def m_face_reads_first(m):
    pilot_face = m.Pilot.face
    def patched(self, direction):
        self.here()
        return pilot_face(self, direction)
    m.Pilot.face = patched


@mutant("face finds its place when it is known lost", "F19c")
def m_face_localizes(m):
    pilot_face = m.Pilot.face
    def patched(self, direction):
        if self.state.get("position_known") is False:
            self.here()
        return pilot_face(self, direction)
    m.Pilot.face = patched


@mutant("face never turns", "F19a")
def m_face_never_turns(m):
    m.Pilot.face = lambda self, direction: self.state.get("facing") == direction


@mutant("face does not read the state after a refusal without one", "F19b")
def m_face_no_read(m):
    rewrite(m, "Pilot.face", ('and "state" not in answer', "and False"))


# --- F20: pilot.MOVES (I19) -------------------------------------------------------------------------------------------
@scenario("F20", "-", "I19")
def f20_moves():
    # Each tool of each toolset, once, from C5 facing north, with arguments that make a moving tool move.
    card = "barbecue"
    args = {"go_to_cell": {"cell": "C3"}, "go_to_card": {"card": card}, "face": {"direction": "south"},
            "move": {"direction": "north", "cells": 1}, "turn": {"direction": "right"}, "set_light": {"color": "red"},
            "wait_for_event": {"seconds": 1}}
    moving = {"step_forward", "move", "turn", "look_around", "face"}
    tools_seen = set()
    for toolset, tools in pp.BASE.TOOLSETS.items():
        for name, (_, read_only, _, _) in tools.items():
            tools_seen.add(name)
            h = Harness(at="C5", facing="north", truth={"C2": card}, toolset=toolset)
            h.tool(name, **args.get(name, {}))
            sends = sorted({c[0] for c in h.calls()} & moving)
            check((name in h.pilot.MOVES) == bool(sends), f"{toolset} {name} in MOVES exactly when it moves the robot",
                  [sends, sorted(h.pilot.MOVES)])
            check(not (read_only and sends), f"{toolset} {name}, read only, moves nothing", sends)
    check(h.pilot.MOVES <= tools_seen, "MOVES names tools only", sorted(h.pilot.MOVES - tools_seen))
    check("set_light" not in h.pilot.MOVES, "set_light is not a move")


@mutant("MOVES without face", "F20")
def m_moves_no_face(m):
    m.MOVES = m.MOVES - {"face"}


@mutant("set_light counted as a move", "F20")
def m_moves_light(m):
    m.MOVES = m.MOVES | {"set_light"}


# --- F21: what the Pilot knows of the mat, kept between two missions (P23) --------------------------------------------
@scenario("F21", "P23", "I20")
def f21_memory_kept():
    # Mission 1: a wall of boxes on column D. The robot, on C5, sees D5 blocked at its first look; the other cells of D
    # are found blocked only when a leg tries them, so a vantage beyond the wall looks reachable until the last one is
    # found: that target goes into covered (stack/pilot.py:606). The camera never looks at column H (4 cells from C).
    wall = {f"D{r}" for r in range(1, 7)}
    h = Harness(at="C5", facing="north", boxes=wall, toolset="goal")
    travels = spy_travel(h)
    first = h.tool("explore")
    check(first["result"] == "done" and first["mat_covered"] is True and first["cards_seen"] == {},
          "mission 1: explore done, mat_covered", first)
    failed = {next(iter(goals)) for goals, there in travels if len(goals) == 1 and not there}
    cards = h.P.card_cells()
    check(failed and failed <= h.P.covered and all(h.P.path({f}, cards) is None for f in failed),
          "an unreachable target went into covered", [sorted(failed), sorted(h.P.covered)])
    check({pp.BASE.cell_xy(c) for c in wall} <= set(h.P.blocked), "the whole wall is blocked", sorted(h.P.blocked))
    never = sorted(set(pp.CELLS) - h.fake.looked)
    column_h = {f"H{r}" for r in range(1, 7)}
    check(column_h <= set(never), "the camera never looked at column H", never)
    check(not {pp.BASE.cell_xy(c) for c in column_h} & h.P.covered, "column H is not in covered either",
          sorted(h.P.covered))
    # Still mission 1: a card recorded on B2 where there is none. go_to_card faces B2, rules it out (for this call,
    # F7a), then explores (found: the record is still there) up to its 8th attempt.
    h.fake.seen.append(record("barbecue", "B2", 60, seen_from="B4 facing north"))
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "gave up", "why": GO_TO_CARD_GAVE_UP, "moves_left": None}, "the card is not on B2",
          answer)
    # Mission 2, without bricks: the settings and the robot's state start again, what it knows of the mat stays. No
    # ball in this world: a lane and a refused cell are put by hand on D2, a box (blocked: no way, no vantage, changes
    # nothing in mission 2), so that keeping them is checked too.
    h.P.lanes.add(("cell", xy("D2")))
    h.P.ball_refused[xy("D2")] = h.clock.now()
    kept = {k: type(getattr(h.P, k))(getattr(h.P, k)) for k in
            ("covered", "blocked", "free", "looked_from", "lanes", "ball_refused")}
    h.configure({})
    check(h.P.settings == pp.DEFAULTS and h.P.state == {}, "defaults, the state forgotten", [h.P.settings, h.P.state])
    check({k: getattr(h.P, k) for k in kept} == kept, "covered, blocked, free, looked_from... kept", sorted(
        k for k in kept if getattr(h.P, k) != kept[k]))
    second = h.tool("explore")
    check(second == {"result": "done", "cards_seen": {"barbecue": ["B2"]}, "not_sure": [], "mat_covered": True,
                     "moves_left": None}, "mission 2: explore has nothing left to see", second)
    check(not h.moves(), "mission 2: explore moves nothing", h.moves())
    raise Known(f"explore answers mat_covered true while the camera never looked at {', '.join(never)}: it means no "
                "vantage left with a way to it (stack/pilot.py:511-528, 616), not the whole mat seen (its docstring, "
                f":591-593); and {', '.join(pp.name_of(*f) for f in sorted(failed))}, targets with no way to them, went "
                "into covered for good, kept in the next missions (:606)")


@mutant("an unreachable vantage is not marked covered", "F21")
def m_unreachable_not_covered(m):
    rewrite(m, "explore", ("            PILOT.covered.add(target)  # unreachable for now: do not choose it again",
                           "            pass"))


@mutant("configure forgets what the Pilot knows of the mat", "F21")
def m_forget_mat(m):
    configure = m.Pilot.configure
    def patched(self, settings):
        configure(self, settings)
        self.covered, self.blocked, self.free, self.looked_from = set(), {}, set(), set()
    m.Pilot.configure = patched


@mutant("configure forgets the lanes of the balls and the cells refused for a ball", "F21")
def m_forget_balls(m):
    configure = m.Pilot.configure
    def patched(self, settings):
        configure(self, settings)
        self.lanes, self.ball_refused = set(), {}
    m.Pilot.configure = patched


# --- F22: a stop or a pause during the Pilot's own waits (P12, P3, P11) ----------------------------------------------
# Before each wait of its own (a lane's watch, a ball coming closer, go_to_card's 10 s), the Pilot heeds the mission
# (ROBOT-PILOTE.md, E1): a stop ends the tool before the next wait, a pause is acknowledged before it.
@scenario("F22", "P12", "I21")
def f22_stop_in_lane_watch():
    # (0) Without a stop: LANE_WATCHES watches of LANE_WATCH_S, each followed by a reading, then the leg crosses.
    h = Harness(at="C5", facing="north")
    lanes(h, ["row 4"])
    answer = h.tool("go_to_cell", cell="C3")
    want = [READ] + [WAIT5, READ] * 3 + [slide("north", 2)]
    check(h.calls(*LEG_CALLS) == want, "(0) three watches of 5 s, then one move north", h.calls(*LEG_CALLS))
    check(answer.get("result") == "done" and answer.get("cell") == "C3", "(0) done on C3",
          {k: v for k, v in answer.items() if k != "state"})
    # (1) The group stops the mission during the first watch: seen before the second, the move never sent.
    h = Harness(at="C5", facing="north")
    lanes(h, ["row 4"])       # C4 on the way, its ball never seen going away
    stopped = []
    h.fake.script["wait_for"] = stop_in_first_wait(h, stopped)
    answer = h.tool("go_to_cell", cell="C3")
    got = h.calls(*LEG_CALLS)
    check(got == [READ, WAIT5, READ], "(1) one watch of 5 s and its reading, then nothing", [c[0] for c in got])
    check(answer == {"result": "gave up", "why": "the mission was stopped", "cards_seen": {}, "moves_left": None},
          "(1) gave up: the mission was stopped", answer)
    late = round(h.clock.monotonic() - stopped[0], 1)
    check(late <= 5.2, "(1) the answer at most 5.2 s after the stop (the rest of the watch, one reading)", late)


@scenario("F22b", "P3 P12", "I16 I21")
def f22b_stop_in_ball_wait():
    # A ball keeps coming closer; the group stops the mission during the first wait. Before, no command that acts
    # followed: go_to_cell waited 47 more times (249.6 s) and answered not there, without the stop.
    h = Harness(at="C5", facing="north")
    h.fake.state_hook = ball_closer(h)
    stopped = []
    h.fake.script["wait_for"] = stop_in_first_wait(h, stopped)
    answer = h.tool("go_to_cell", cell="C3")
    check(not h.moves(), "nothing moved after the stop", h.moves())
    check(h.calls("wait_for") == [WAIT5] and h.calls()[-3:-1] == [WAIT5, READ],
          "one wait, the one the stop came in, then its reading: no wait after the stop", h.calls())
    check(answer == {"result": "gave up", "why": "the mission was stopped", "cards_seen": {}, "moves_left": None},
          "gave up: the mission was stopped", answer)
    late = round(h.clock.monotonic() - stopped[0], 1)
    check(late <= 5.2, "the answer at most 5.2 s after the stop", late)


@scenario("F22c", "P12 P17", "I16 I21")
def f22c_stop_in_card_wait():
    # The world of F9a (one place, no way to it): the group stops the mission during go_to_card's first wait of 10 s.
    # The next attempt finds no way again, and the stop is seen before the second wait.
    h = Harness(at="A6", facing="north", toolset="goal")
    h.fake.seen = [record("barbecue", "C3", 60)]
    block(h, "B3", "D3", "C2", "C4")
    stopped = []
    h.fake.script["wait_for"] = stop_in_first_wait(h, stopped)
    answer = h.tool("go_to_card", card="barbecue")
    check(h.calls("wait_for") == [["wait_for", {"seconds": 10}]], "one wait of 10 s, the one the stop came in",
          h.calls("wait_for"))
    check(not h.moves(), "nothing moved", h.moves())
    check(answer == {"result": "gave up", "why": "the mission was stopped", "cards_seen": {"barbecue": ["C3"]},
                     "moves_left": None}, "gave up: the mission was stopped", answer)
    late = round(h.clock.monotonic() - stopped[0], 1)
    check(late <= 10.2, "the answer at most 10.2 s after the stop (the rest of the wait, one reading)", late)


@scenario("F22d", "P3 P11", "I2 I21")
def f22d_pause_in_ball_wait():
    # A ball coming closer during the first two waits, then gone. The group pauses the mission for 30 s during the
    # first wait: the pause is acknowledged before the second wait (held.json written, where the robot stands), cleared
    # once the mission goes on, and only then the robot moves. Before, the second wait went on without a word, and the
    # acknowledgement came only at the move.
    h = Harness(at="C5", facing="north")
    waits = []

    def ball(state):
        if len(waits) < 2:
            state["ball_last_seen"] = {"how": "getting closer to the robot", "at": "12:00:00", "t": h.clock.now() - 1}
    h.fake.state_hook = ball

    def wait_for(seconds):
        waits.append(h.clock.monotonic())
        if len(waits) == 1:
            h.mission.pause(30, "go")
        h.clock.advance(seconds)
        return {"result": "nothing happened", "waited_seconds": seconds}
    h.fake.script["wait_for"] = wait_for
    held, when = h.pilot.HELD, []

    def spy(where):
        h.fake.op["calls"].append(["held", where and {k: where[k] for k in ("cell", "facing")}])
        when.append(round(h.clock.monotonic() - waits[0], 1))
        held(where)
    h.pilot.HELD = spy
    answer = h.tool("go_to_cell", cell="C3")
    got = h.calls(*LEG_CALLS, "held")
    want = [READ, WAIT5, READ, ["held", {"cell": "C5", "facing": "north"}], ["held", None], WAIT5, READ,
            slide("north", 2)]
    check(got == want, "a wait, its reading, the pause acknowledged on C5 then cleared, the second wait, the move", got)
    check(when == [5.2, 30.2], "acknowledged 5.2 s after the pause began, cleared when it ended", when)
    check(h.common.acks == 1 and pp.common.held_where() is None, "one acknowledgement, cleared before the move",
          [h.common.acks, pp.common.held_where()])
    check(answer.get("result") == "done" and answer.get("cell") == "C3", "done on C3",
          {k: v for k, v in answer.items() if k != "state"})


@scenario("F22e", "P12", "I16 I21")
def f22e_stop_before_ball_refusal_wait():
    # The slide north is refused for a ball, and the group stopped the mission meanwhile: the wait that follows a
    # refusal for a ball heeds the mission too (E1), so the tool ends without waiting.
    h = Harness(at="C5", facing="north")
    stopped = []

    def move(**_):
        if not stopped:
            h.mission.stop()
            stopped.append(h.clock.monotonic())
        return {"result": "refused", "why": "a ball was seen near C4 less than 10 s ago",
                "what_to_do": "watch it go away (wait_for_event), then try again"}
    h.fake.script["move"] = move
    answer = h.tool("go_to_cell", cell="C3")
    check(h.calls("wait_for") == [] and len(h.calls("move")) == 1, "one slide, refused, then no wait",
          h.calls(*LEG_CALLS))
    check(answer == {"result": "gave up", "why": "the mission was stopped", "cards_seen": {}, "moves_left": None},
          "gave up: the mission was stopped", answer)
    late = round(h.clock.monotonic() - stopped[0], 1)
    check(late <= 0.2, "the answer at most 0.2 s after the stop", late)


@mutant("a lane is watched once more (LANE_WATCHES 4)", "F22")
def m_lane_watches(m):
    m.LANE_WATCHES = 4


@mutant("a lane is watched once less (LANE_WATCHES 2)", "F22")
def m_lane_watches_less(m):
    m.LANE_WATCHES = 2


@mutant("lanes on the way not watched", "F22")
def m_lanes_not_watched(m):
    m.Pilot.lanes_crossed = lambda self, dc, dr, cells: set()


@mutant("the stop not checked before a move", "F8c", "F15b")
def m_stop_ignored(m):
    moves = {"step_forward", "move", "turn", "look_around", "face", "set_light"}
    def patched(name, *args):
        if m.ASK is not None and name in moves:
            m.hold_still()
        if name == "wait_for_event":
            return m.CART.wait_for(*args)
        return m.CART.call(lambda: getattr(m.CART, name)(*args), is_move=name in moves)
    m.command = patched


@mutant("a wait of the Pilot's own heeds neither stop nor pause (before E1)", "F22", "F22b", "F22c", "F22d")
def m_wait_not_heeded(m):
    rewrite(m, "wait", ("    heed()\n", ""))


@mutant("a wait of the Pilot's own heeds the pause, not the stop", "F22", "F22b", "F22c")
def m_wait_pause_only(m):
    rewrite(m, "wait", ("    heed()\n", "    if ASK is not None:\n        hold_still()\n"))


@mutant("a wait of the Pilot's own heeds the stop, not the pause", "F22d")
def m_wait_stop_only(m):
    rewrite(m, "wait", ("    heed()\n", '    if ASK is not None and word() == "stop":\n'
                                       '        raise Stuck("the mission was stopped")\n'))


@mutant("go_to_card waits 10 s without heeding the mission", "F22c")
def m_card_wait_not_heeded(m):
    rewrite(m, "go_to_card", ("wait(10)", 'command("wait_for_event", 10)'))


@mutant("a leg waits after a refusal for a ball without heeding the mission", "F22e")
def m_ball_refusal_wait_not_heeded(m):
    rewrite(m, "Pilot.leg", ("                for_a_ball += 1\n                wait(5)\n",
                             '                for_a_ball += 1\n                command("wait_for_event", 5)\n'))


@mutant("a leg watches a lane without heeding the mission", "F22")
def m_lane_watch_not_heeded(m):
    rewrite(m, "Pilot.leg", ("                wait(LANE_WATCH_S)\n", '                command("wait_for_event", LANE_WATCH_S)\n'))


@mutant("a leg waits for a ball without heeding the mission", "F22b", "F22d")
def m_ball_wait_not_heeded(m):
    rewrite(m, "Pilot.leg", ("                wait(5)  # cross only when the ball goes away",
                             '                command("wait_for_event", 5)'))


# --- F23: a limit of the adults inside a goal tool (E2 (a), E9) -------------------------------------------------------
@scenario("F23", "P14", "I12 I16 I17")
def f23_limit_in_goal_tool():
    # go_to_card to a card sure on C2: the slide north 2 towards C3 is refused for a limit of the adults (cells, or
    # moves while the Pilot read no limit). The tool gives up at once with the Cartographer's words, and C4 is not
    # blocked: a limit says nothing of the mat (ROBOT-PILOTE.md, E2 (a), E9).
    for refusal in (NO_CELLS, NO_MOVES):
        what = refusal["why"].split(":")[0]
        h = Harness(at="C5", facing="north", truth={"C2": "barbecue"}, toolset="goal")
        h.fake.seen = [record("barbecue", "C2", 90)]
        h.fake.script["move"] = lambda refusal=refusal, **_: dict(refusal)
        answer = h.tool("go_to_card", card="barbecue")
        check(answer == {"result": "gave up", "why": refusal["why"], "cards_seen": {"barbecue": ["C2"]},
                         "moves_left": None}, f"{what}: gave up with the Cartographer's words", answer)
        check(h.moves() == [slide("north", 2)] and h.calls()[-1] == SEEN, f"{what}: one slide, refused, then only the "
              "record for cards_seen", h.calls())
        check(not h.P.is_blocked(xy("C4")), f"{what}: C4 not blocked", sorted(h.P.blocked))
        check([l["tool"] for l in h.log()] == ["go_to_card"], f"{what}: no explore", [l["tool"] for l in h.log()])
    # A commands limit: the first quarter turn of the leg is refused, the same way.
    h = Harness(at="C5", facing="east", truth={"C2": "barbecue"}, toolset="goal")
    h.fake.seen = [record("barbecue", "C2", 90)]
    h.fake.script["face"] = lambda **_: dict(NO_COMMANDS)
    answer = h.tool("go_to_card", card="barbecue")
    check(answer == {"result": "gave up", "why": NO_COMMANDS["why"], "cards_seen": {"barbecue": ["C2"]},
                     "moves_left": None}, "commands: gave up with the Cartographer's words", answer)
    check(h.moves() == [face("north")], "commands: the refused quarter turn, nothing else", h.moves())
    check(h.P.state == {}, "commands: the state forgotten (a first quarter turn may be done): the next tool reads "
          "where the robot is", h.P.state)


@mutant("a limit refusal keeps the state read before it", "F23")
def m_limit_keeps_state(m):
    rewrite(m, "Pilot.read", ("            self.state = {}  # a first quarter turn may be done", "            pass  #"))


@mutant("a limit refusal goes on and blocks the next cell (before E2 (a) and E9)", "F23")
def m_limit_blocks(m):
    rewrite(m, "Pilot.read", ("            raise Stuck(answer[\"why\"])", "            pass"))
    rewrite(m, "Pilot.leg", ('if answer.get("why") == REASONS["obstacle"] and not self.ball_recent():',
                             'if "card" not in why and not self.ball_recent():'))


# --- running ----------------------------------------------------------------------------------------------------------
def patched_loader(load, patch):
    """pp.load_fresh with the mutant applied; a patch that raises makes MutantBroken, never a FAIL of the scenario."""
    def load_patched():
        m = load()
        try:
            patch(m)
        except Exception as e:  # noqa: BLE001
            raise MutantBroken(f"{type(e).__name__}: {e}") from e
        return m
    return load_patched


def play(name, patch=None):
    fn = SCENARIOS[name][0]
    load = pp.load_fresh
    if patch:
        pp.load_fresh = patched_loader(load, patch)
    HARNESSES.clear()
    try:
        fn()
        broken = [b for h in HARNESSES for b in h.broken]
        if broken:
            return "FAIL", "a check of the probe broke: " + json.dumps(broken[0], default=str)[:600]
        return "PASS", ""
    except MutantBroken as e:
        return "BROKEN", str(e)
    except Known as e:
        return "KNOWN", str(e)
    except AssertionError as e:
        return "FAIL", str(e)
    except Exception as e:  # noqa: BLE001
        return "FAIL", f"{type(e).__name__}: {e} ({traceback.extract_tb(e.__traceback__)[-1].lineno})"
    finally:
        pp.load_fresh = load


def play_mutant(patch):
    """(scenarios that fail, why the mutant is broken or None). The patch is tried once on a fresh Pilot first."""
    try:
        patched_loader(pp.load_fresh, patch)()
    except MutantBroken as e:
        return [], str(e)
    failed = []
    for scenario_name in SCENARIOS:
        outcome, detail = play(scenario_name, patch)
        if outcome == "BROKEN":
            return failed, f"in {scenario_name}: {detail}"
        if outcome == "FAIL":
            failed.append(scenario_name)
    return failed, None


def main(argv):
    if argv[:1] == ["--mutants"]:
        counts = {"CAUGHT": 0, "MISSED": 0, "BROKEN": 0}
        for name, (patch, catchers) in MUTANTS.items():
            failed, why = play_mutant(patch)
            outcome = "BROKEN" if why else "CAUGHT" if failed else "MISSED"
            counts[outcome] += 1
            if why:
                print(f"BROKEN  {name}: {why}")
                continue
            print(f"{outcome}  {name}: {' '.join(failed) or '-'}"
                  f"{'' if set(catchers) & set(failed) else '  (expected: ' + ' '.join(catchers) + ')'}")
        print(f"{counts['CAUGHT']} caught / {counts['MISSED']} missed / {counts['BROKEN']} broken")
        pp.shutil.rmtree(pp.STATE, ignore_errors=True)
        return 1 if counts["MISSED"] or counts["BROKEN"] else 0
    chosen = argv or list(SCENARIOS)
    counts = {"PASS": 0, "FAIL": 0, "KNOWN": 0}
    for name in chosen:
        outcome, detail = play(name)
        counts[outcome] += 1
        _, cases, invariants = SCENARIOS[name]
        print(f"{outcome:5}  {name} ({cases}; {invariants}){': ' + detail if detail else ''}")
    print(f"{counts['PASS']} PASS / {counts['FAIL']} FAIL / {counts['KNOWN']} KNOWN")
    pp.shutil.rmtree(pp.STATE, ignore_errors=True)
    return 1 if counts["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
