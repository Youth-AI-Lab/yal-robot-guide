"""The Cartographer alone (stack/cartographer.py), in fixed scenarios: the invariants of
facilitator/ROBOT-CARTOGRAPHE.md that the probe (ci/probe_cartographer.py) and check_sim reach by chance or never.

Each scenario sets a world by hand: a fake Referee below that answers exactly what the scenario says (nothing drawn at
random), the probe's fake clock (waits cost no time, what gets old gets old), and a Cartographer loaded fresh from
cartographer.py, its terrain.json removed first. Then it calls the Cartographer and checks what it sent below, what it
wrote on disk and what it answered.

The scenarios:
    FC1   moving() writes the map before the move goes out (I6); the fake calls moving() itself, as the real Referee
          does, so the order in referee.py is not tested here
    FC2   a sure card that left the camera's view stays forbidden for 10 min (I18); FC2b: dated at its last SEEN (KNOWN)
    FC3   a bump that stops a slide becomes a lane, on its cell and the line across, for 600 s (I19); FC3b: less than
          half a cell from the last one; FC3c: two bumps on one column make the column, dated by the newer
    FC4   two like cards side by side stay two (I17); FC4b: a closer sighting between them merges them (KNOWN)
    FC5   the compass is believed at 25 degrees from a quarter turn at most, robot still, nobody awaited (I13)
    FC6   a pause between the two quarter turns of face (I10), and only the first counted (I9)
    FC7   a card seen from far blocks nothing forwards; with caution, the cells of could_also_be are not given to the
          Referee (E1, KNOWN)
Each scenario is followed by its mutants: a rule of the Cartographer removed or changed by rewrite() (one method of
cartographer.py, its text changed in place) or by a constant of the loaded module.

Outcomes of a scenario: PASS; FAIL; KNOWN, a defect written down and not fixed yet (the scenario checks the
Cartographer does exactly that; when the code changes, it becomes a FAIL, to look at and rewrite).
Outcomes of a mutant (--mutants): CAUGHT, at least one scenario fails with it; MISSED, every scenario passes or is
KNOWN; BROKEN, the mutant cannot be applied to cartographer.py as it is (its text is not there exactly once): never
counted as caught, to rewrite. The run fails on any FAIL, MISSED or BROKEN.

Usage (the folder of the guide is the current folder):
    python3 -I ci/fixed_cartographer.py              every scenario
    python3 -I ci/fixed_cartographer.py FC3 FC5      some of them
    python3 -I ci/fixed_cartographer.py --mutants    each mutant must fail a scenario
Only the standard library; no simulator, no model; a fraction of a second in all.
"""
import ast
import json
import sys
import textwrap
import traceback
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import probe_cartographer as pc  # noqa: E402  (first: it sets ROBOT_STATE_DIR before stack/common.py is read)
import probe_tiers  # noqa: E402

SOURCE = pc.GUIDE / "stack" / "cartographer.py"
SCENARIOS = {}   # name -> (function, invariants of ROBOT-CARTOGRAPHE.md)
MUTANTS = {}     # name -> (patch(module), scenarios that must fail)


class Known(Exception):
    """The Cartographer does exactly the defect written down: not fixed yet."""


class MutantBroken(Exception):
    pass


def scenario(name, invariants):
    def add(fn):
        SCENARIOS[name] = (fn, invariants)
        return fn
    return add


def mutant(name, *catchers):
    def add(fn):
        MUTANTS[name] = (fn, catchers)
        return fn
    return add


def check(cond, what, got=None):
    if not cond:
        raise AssertionError(what + ("" if got is None else f": {json.dumps(got, default=str)[:400]}"))


def rewrite(m, name, *changes):
    """A mutant by text, on the loaded module m: the method `name` of Cartographer, taken from cartographer.py, each
    pair (old, new) replaced exactly once, then compiled again in the module's globals and set on the class."""
    text = SOURCE.read_text(encoding="utf-8").splitlines(keepends=True)
    cls = next(n for n in ast.parse("".join(text)).body if isinstance(n, ast.ClassDef) and n.name == "Cartographer")
    node = next((n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name), None)
    if node is None:
        raise MutantBroken(f"{name}: no such method in cartographer.py")
    src = "".join(text[node.lineno - 1:node.end_lineno])
    for old, new in changes:
        if src.count(old) != 1:
            raise MutantBroken(f"{name}: {old!r} found {src.count(old)} times")
        src = src.replace(old, new)
    space = {}
    exec(compile("\n" * (node.lineno - 1) + textwrap.dedent(src), str(SOURCE), "exec"), m.__dict__, space)
    setattr(m.Cartographer, name, space[name])


# --- the fake below ----------------------------------------------------------------------------------------------------
class Referee:
    """A Referee that answers what the scenario says. fields: the robot's STATE; queue: the events of the next read;
    on_slide(terrain, cells) and on_turn(terrain, way, counted): what a move does once sent (the scenario writes it),
    after moving() as the real Referee does; sent: what reached it, in order. It judges no rule."""

    def __init__(self):
        self.fields = {"resumes": "0", "pose": "1", "moving": "0", "wait_adult": "0", "estop": "0", "dist": "none",
                       "looked": "1", "heading": "none", "boxes": "none", "card": "none", "battery": "90",
                       "bumper": "0", "warning": "0", "last_stop": "none", "fault": "none", "slid": "0"}
        self.queue, self.sent = [], []
        self.on_slide = self.on_turn = None

    def door(self, is_move, body): return body()
    def connect(self): pass
    def close(self): pass
    def limits(self): return {}
    def moves_left(self): return None
    def state(self): return dict(self.fields)

    def events(self):
        out, self.queue = self.queue, []
        return out, False, False

    def light(self, color):
        self.sent.append(["light", color])
        return {"result": "done"}

    def turn(self, terrain, way, counted):
        self.sent.append(["turn", way, counted])
        terrain.moving([], counted=counted)
        if self.on_turn:
            return self.on_turn(terrain, way, counted)
        return {"result": "done", "cells_done": 0}

    def slide(self, terrain, way, cells, until, asked, hold=None):
        self.sent.append(["slide", way, [tuple(c) for c in cells], list(until), asked])
        terrain.moving(cells, counted=True)  # as the real Referee does (referee.py:283); the order is not tested
        return self.on_slide(terrain, cells)


class World:
    """One scenario: a fresh Cartographer (cartographer.py loaded again, or the mutant), the probe's clock, the fake."""

    def __init__(self, m, rules=None, at=None):
        for f in pc.STATE.iterdir():
            if f.is_file():
                f.unlink()
        self.clock = pc.Clock()
        m.clock = self.clock
        self.m = m
        # The guide's rules.json, with what the scenarios rely on pinned (the adults may edit that file).
        self.rules = {**pc.RULES, "start": None, "mat_north_up": True, "landmarks": {}, "limits": {}, **(rules or {})}
        self.cart = m.Cartographer(pc.MAT, self.rules)
        self.ref = Referee()
        self.cart.referee = self.ref
        if at:
            self.put(*at)

    def put(self, cell, facing):
        """The Cartographer believes the robot stands there, sure, on the mat; written on disk, which each call reads."""
        self.cart.col, self.cart.row = pc.cell_xy(cell)
        self.cart.facing = pc.FACING.index(facing)
        self.cart.frame, self.cart.sure = None, True
        self.cart.save()

    def saved(self):
        try:
            return json.loads((pc.STATE / "terrain.json").read_text(encoding="utf-8"))
        except (OSError, ValueError):
            return None

    def sight(self, kind, card, angle, cm, pose="7"):
        """A camera event (SEEN or GONE) from the pose the Cartographer believes now."""
        self.cart.poses[pose] = [self.cart.col, self.cart.row, self.cart.facing]
        self.cart.saw(kind, f"{card}:{angle}:{cm}:{pose}", 0)

    def entries(self, card):
        return sorted((s["cell"], tuple(s["could_also_be"])) for s in self.cart.seen if s["card"] == card)


def world(m=None, **kw):
    return World(m or MODULE["now"], **kw)


MODULE = {"now": None}  # the module the scenarios use: cartographer.py loaded fresh, or a mutant


def seen_free(w, *cells):
    """The distance sensor saw these cells free just now (a slide sideways goes only there), written on disk."""
    for cell in cells:
        w.cart.free[cell] = w.clock.now()
    w.cart.save()


def bumped(w):
    """The robot stopped by a bump during a move: its STATE and its two events, in the firmware's order (BUMP noted,
    then STOP:bump)."""
    w.ref.fields.update(slid="1", wait_adult="1", last_stop="bump")
    w.ref.queue = [[0, "BUMP", ""], [0, "STOP", "bump"]]


# --- the scenarios ------------------------------------------------------------------------------------------------------
@scenario("FC1", "I6")
def fc1():
    """The map on disk says where the robot started and that its place is unknown, before the Referee sends the move."""
    w = world()  # no terrain.json, start pinned to null by World: the start of the probe's MAT, A6 facing north
    at_send = {}

    def slide(terrain, cells):
        at_send["saved"] = w.saved()  # what a stack killed now would leave
        terrain.reached(cells[0], "2")
        terrain.reached(cells[1], "3")
        return {"result": "done", "cells_done": 2, "why": "all", "card_stop": None, "edge": False}
    w.ref.on_slide = slide
    answer = w.cart.call(lambda: w.cart.move("north", 2, []), is_move=True)
    check(w.ref.sent and w.ref.sent[0][:3] == ["slide", "F", [(0, 4), (0, 3)]], "move north, facing north: F, A5 A4",
          w.ref.sent)
    saved = at_send.get("saved")
    check(saved is not None, "terrain.json is written before the move is sent")
    check(saved["sure"] is False and saved["back"] == [0, 5, 0] and saved["way"] == [[0, 5], [0, 4], [0, 3]],
          "before the move is sent, terrain.json says: place not sure, put back on A6 facing north",
          {k: saved.get(k) for k in ("sure", "back", "way")})
    check(answer.get("result") == "done" and answer.get("cells_moved") == 2, "the slide is done, two cells", answer)
    after = w.saved()
    check(after["sure"] is True and (after["col"], after["row"]) == (0, 3), "after it: A4, sure", after)


@mutant("moving does not write terrain.json", "FC1")
def m_moving_unsaved(m):
    rewrite(m, "moving", ("        self.save()", "        pass"))


@scenario("FC2", "I18")
def fc2():
    """A card placed for sure, then out of the camera's view (GONE) at the same moment: its cell stays forbidden
    10 min. The SEEN and the GONE together, so that the scenario holds whichever of the two placed is dated at (E11)."""
    w = world(at=("C5", "north"))
    w.sight("SEEN", "barbecue", 0, 60)  # straight ahead, 60 cm: the centre of C3, sure
    check(w.entries("barbecue") == [("C3", ())], "seen from C5 at 60 cm: C3, sure", w.entries("barbecue"))
    w.sight("GONE", "barbecue", 0, 60)
    check(w.entries("barbecue") == [], "GONE: out of the record")
    w.clock.sleep(5)
    check("C3" in w.cart.card_cells() and "C3" in w.cart.card_cells(caution=True),
          "5 s after GONE, C3 is still a card cell for the Referee", sorted(w.cart.card_cells()))
    w.clock.sleep(599 - 5)
    check("C3" in w.cart.card_cells(), "599 s after the SEEN and the GONE: still forbidden",
          sorted(w.cart.card_cells()))
    w.clock.sleep(2)
    check("C3" not in w.cart.card_cells(), "601 s after the SEEN and the GONE: no longer", sorted(w.cart.card_cells()))


@mutant("card_cells forgets the cards that left the view", "FC2")
def m_placed_ignored(m):
    rewrite(m, "card_cells", ("return sure | {c for c, t in self.placed.items() if now - t < LANE_FOR_S}",
                              "return sure"))


@mutant("a card that left the view is forbidden for ever", "FC2")
def m_placed_forever(m):
    rewrite(m, "card_cells", ("if now - t < LANE_FOR_S}", "}"))


@scenario("FC2b", "I18, E11")
def fc2b():
    """A sure card that stays 11 min in view, then GONE: placed is dated at the SEEN, not at the GONE."""
    w = world(at=("C5", "north"))
    w.sight("SEEN", "barbecue", 0, 60)
    w.clock.sleep(660)
    w.sight("GONE", "barbecue", 0, 60)
    w.clock.sleep(1)
    check("C3" not in w.cart.card_cells(caution=True), "E11 as written: no longer forbidden 1 s after GONE",
          sorted(w.cart.card_cells(caution=True)))
    raise Known("E11: a sure card kept 11 min in view, then GONE, is no longer forbidden 1 s later "
                "(placed dated at its last sure SEEN, not at GONE)")


@scenario("FC3", "I19")
def fc3():
    """A slide east from C5, the robot facing north, stopped by a bump 4 s after it reached D5 (more than half a cell
    away: (4 - 0.3 s standing) x 5 cm/s sideways = 18.5 cm): E5 and the column E become lanes, for 600 s; the adult
    puts it back on D5. D5, E5, F5 seen free just before, as the real Referee wants for a slide sideways."""
    w = world(at=("C5", "north"))
    seen_free(w, "D5", "E5", "F5")

    def slide(terrain, cells):
        terrain.reached(cells[0], "2")
        w.clock.sleep(4)
        bumped(w)
        return {"result": "stopped", "why": "bump", "cells_done": 1}
    w.ref.on_slide = slide
    answer = w.cart.call(lambda: w.cart.move("east", 3, []), is_move=True)
    check(w.ref.sent[0][:3] == ["slide", "R", [(3, 4), (4, 4), (5, 4)]], "east, facing north: R, D5 E5 F5", w.ref.sent)
    check(answer.get("result") == "stopped" and answer.get("position_known") is False, "stopped, place unknown", answer)
    check("back on cell D5, facing north" in answer.get("what_to_do", ""), "the adult puts it back on D5", answer)
    lanes = sorted(w.cart.lanes)
    check({"cell:E5", "col:4"} <= set(lanes), "E5 and the column E are lanes", lanes)
    check(w.cart.on_lane((4, 0)) and not w.cart.on_lane((3, 0)) and not w.cart.on_lane((2, 4)),
          "on the column E (E1), not D1 nor C5", lanes)
    w.clock.sleep(599)
    check(w.cart.on_lane((4, 0)) and w.cart.on_lane((4, 4)), "599 s later: still lanes", lanes)
    w.clock.sleep(2)
    check(not w.cart.on_lane((4, 0)) and not w.cart.on_lane((4, 4)), "601 s later: no longer", lanes)


@mutant("a bump in a slide east marks the row", "FC3")
def m_bump_row(m):
    rewrite(m, "bump_lanes", ('keys.append(f"col:{col}" if dc else f"row:{row}")',
                              'keys.append(f"row:{row}" if dc else f"col:{col}")'))


@mutant("lanes are never forgotten", "FC3")
def m_lanes_forever(m):
    rewrite(m, "on_lane", ("if now - when > LANE_FOR_S:", "if False:"))


@scenario("FC3b", "I19, I4")
def fc3b():
    """The same bump 3.1 s after the robot reached D5: (3.1 - 0.3 s standing) x 5 cm/s sideways = 14 cm, less than
    half a cell (15 cm), so still on D5. With the forwards speed (28 cm) or without the stand (15.5 cm) it would be E5.
    D5 and the column D are lanes, and the adult puts the robot back on C5, off them."""
    w = world(at=("C5", "north"))
    seen_free(w, "D5", "E5", "F5")

    def slide(terrain, cells):
        terrain.reached(cells[0], "2")
        w.clock.sleep(3.1)
        bumped(w)
        return {"result": "stopped", "why": "bump", "cells_done": 1}
    w.ref.on_slide = slide
    answer = w.cart.call(lambda: w.cart.move("east", 3, []), is_move=True)
    check({"cell:D5", "col:3"} <= set(w.cart.lanes), "D5 and the column D are lanes", sorted(w.cart.lanes))
    check("back on cell C5, facing north" in answer.get("what_to_do", ""), "the adult puts it back on C5", answer)


@mutant("a bump is put on the next cell, however near the last one", "FC3b")
def m_no_half_cell(m):
    rewrite(m, "bump_lanes", ("                k -= 1", "                pass"))


@mutant("a sideways slide counted at the forwards speed", "FC3b")
def m_forwards_speed(m):
    rewrite(m, "bump_lanes", ('SLIDE_CM_S["forwards" if forwards else "blind"]',
                              'SLIDE_CM_S["blind" if forwards else "forwards"]'))


@mutant("the stand on the reached cell not counted", "FC3b")
def m_no_stand(m):
    rewrite(m, "bump_lanes", ("(CELL_STAND_S if left[1] else 0)", "0"))


@scenario("FC3c", "I19")
def fc3c():
    """Two bumps on the robot standing, on E5 then, 100 s later, on E4: each marks its cell, and the column E becomes a
    lane, dated by the newer of the two (still a lane 650 s after E5, 550 s after E4)."""
    w = world(at=("E5", "north"))
    w.ref.queue = [[0, "BUMP", ""]]
    w.cart.new_events()
    check(w.cart.on_lane((4, 4)) and not w.cart.on_lane((4, 0)), "one bump: its cell only", sorted(w.cart.lanes))
    w.clock.sleep(100)
    w.put("E4", "north")
    w.ref.queue = [[0, "BUMP", ""]]
    w.cart.new_events()
    check({"cell:E5", "cell:E4"} <= set(w.cart.lanes), "each bump marks its cell", sorted(w.cart.lanes))
    check(w.cart.on_lane((4, 0)), "two cells on the column E: the column E", sorted(w.cart.lanes))
    w.clock.sleep(550)
    check(w.cart.on_lane((4, 0)), "650 s after E5, 550 s after E4: still the column", sorted(w.cart.lanes))


@mutant("two cells on a line do not make the line", "FC3c")
def m_no_lines(m):
    rewrite(m, "lines_from_cells", ("now = clock.now()", "return"))


@mutant("a line dated by the older of its two cells", "FC3c")
def m_line_older(m):
    rewrite(m, "lines_from_cells", ("max(t1, t2)", "min(t1, t2)"))


@mutant("a lane of one cell ignored by on_lane", "FC3c")
def m_no_cell_lane(m):
    rewrite(m, "on_lane", ('or (kind == "cell" and value == cell_name(col, row)):', "or False:"))


@scenario("FC4", "I17")
def fc4():
    """Two dry grass cards in C2 and D2, seen from C5 (90 and 95 cm away): two entries, 30 cm apart, each uncertainty
    about 19 cm. Their sum (38 cm) would merge them; the larger one does not."""
    w = world(at=("C5", "north"))
    w.sight("SEEN", "dry-grass", 0, 90)
    w.sight("SEEN", "dry-grass", 18, 95)
    cells = [c for c, _ in w.entries("dry-grass")]
    check(cells == ["C2", "D2"], "two entries, C2 and D2", w.entries("dry-grass"))


@mutant("two like cards merged when their uncertainties add up", "FC4")
def m_sum(m):
    rewrite(m, "saw", ('same = [s for s in self.seen if s["card"] == label and math.hypot(s["x"] - x, s["y"] - y) '
                       '< max(s["u"], u)]',
                       'same = [s for s in self.seen if s["card"] == label and math.hypot(s["x"] - x, s["y"] - y) '
                       '< s["u"] + u]'))


@mutant("the same card only within the smaller uncertainty", "FC4b")
def m_min(m):
    rewrite(m, "saw", ('same = [s for s in self.seen if s["card"] == label and math.hypot(s["x"] - x, s["y"] - y) '
                       '< max(s["u"], u)]',
                       'same = [s for s in self.seen if s["card"] == label and math.hypot(s["x"] - x, s["y"] - y) '
                       '< min(s["u"], u)]'))


@scenario("FC4b", "I17")
def fc4b():
    """The same two cards, then a dry grass seen from C4, closer, on the border of C2 and D2 (within the uncertainty of
    both): the two entries become one."""
    w = world(at=("C5", "north"))
    w.sight("SEEN", "dry-grass", 0, 90)
    w.sight("SEEN", "dry-grass", 18, 95)
    w.put("C4", "north")
    w.sight("SEEN", "dry-grass", 14, 62, pose="8")
    check(len(w.entries("dry-grass")) == 1, "I17 as written: the two entries merged into one", w.entries("dry-grass"))
    raise Known("I17: a closer sighting between two like cards replaces both entries by one "
                f"({w.entries('dry-grass')}); the behaviour wanted is to decide")


@scenario("FC5", "I13")
def fc5():
    """The compass, believed at 25 degrees from a quarter turn, not at 26; with the start cell C5 known only by its
    cell, a believed reading leaves one place: C5 facing north."""
    w = world(rules={"mat_north_up": True})
    said = {h: w.cart.compass({"heading": h}) for h in ("25", "26", "335", "334", "115", "116", "none")}
    check(said == {"25": 0, "26": None, "335": 0, "334": None, "115": 1, "116": None, "none": None},
          "the compass: within 25 degrees of a quarter turn only", said)
    w2 = world(rules={"start": {"cell": "C5"}, "mat_north_up": True})
    check(w2.cart.frame and len(w2.cart.frame["candidates"]) == 4, "start C5 alone: four places", w2.cart.frame)
    w2.ref.fields["heading"] = "26"
    w2.cart.fields()
    check(w2.cart.frame and len(w2.cart.frame["candidates"]) == 4, "26 degrees: nothing ruled out", w2.cart.frame)
    for busy in ("wait_adult", "moving"):
        w2.ref.fields.update({"heading": "25", busy: "1"})
        w2.cart.fields()
        check(w2.cart.frame and len(w2.cart.frame["candidates"]) == 4, f"25 degrees, {busy}=1: nothing ruled out",
              w2.cart.frame)
        w2.ref.fields[busy] = "0"
    w2.ref.fields["heading"] = "25"
    w2.cart.fields()
    check(w2.cart.frame is None and (w2.cart.col, w2.cart.row, w2.cart.facing) == (2, 4, 0),
          "25 degrees: C5 facing north", [w2.cart.frame, w2.cart.col, w2.cart.row, w2.cart.facing])
    w3 = world(rules={"mat_north_up": False})
    check(w3.cart.compass({"heading": "0"}) is None, "mat_north_up false: never believed")


@mutant("the compass believed at 30 degrees", "FC5")
def m_compass_loose(m):
    m.COMPASS_TOLERANCE_DEG = 30


@mutant("the compass believed while the robot moves", "FC5")
def m_compass_moving(m):
    rewrite(m, "fields", ('if self.sure and fields.get("moving") == "0" and fields.get("wait_adult") == "0":\n'
                          '            facing',
                          'if self.sure and fields.get("wait_adult") == "0":\n            facing'))


@mutant("the compass believed while the robot waits for an adult", "FC5")
def m_compass_waiting(m):
    rewrite(m, "fields", ('if self.sure and fields.get("moving") == "0" and fields.get("wait_adult") == "0":\n'
                          '            facing',
                          'if self.sure and fields.get("moving") == "0":\n            facing'))


@mutant("the compass not believed at exactly 25 degrees", "FC5")
def m_compass_strict(m):
    rewrite(m, "compass", ("off <= COMPASS_TOLERANCE_DEG", "off < COMPASS_TOLERANCE_DEG"))


@scenario("FC6", "I10, I9")
def fc6():
    """face south from north is a half turn, two quarter turns: the first counted, the second not; a pause asked after
    the first stops it there, paused, with why and state, and nothing more is sent."""
    w = world(at=("C5", "north"))
    answer = w.cart.call(lambda: w.cart.face("south"), is_move=True)
    turns = [s for s in w.ref.sent if s[0] == "turn"]
    check(turns == [["turn", "R", True], ["turn", "R", False]], "two quarter turns, the first counted", turns)
    check(answer.get("result") == "done" and w.cart.facing == 2, "done, facing south", answer)
    w = world(at=("C5", "north"))
    w.cart.hold_wanted = lambda: len(w.ref.sent) >= 1
    answer = w.cart.call(lambda: w.cart.face("south"), is_move=True)
    turns = [s for s in w.ref.sent if s[0] == "turn"]
    check(turns == [["turn", "R", True]], "a pause after the first quarter turn: no second one", turns)
    check(answer.get("result") == "paused" and set(answer) - {"since_last_call"} == {"result", "why", "state"}
          and w.cart.facing == 1, "paused, with why and state, facing east", answer)


@mutant("face goes on after a pause", "FC6")
def m_face_no_pause(m):
    rewrite(m, "face", ("if n > 0 and self.hold_wanted():", "if False:"))


@mutant("every quarter turn of face counted", "FC6")
def m_face_counted(m):
    rewrite(m, "face", ("counted=n == 0", "counted=True"))


@scenario("FC7", "E1")
def fc7():
    """A dry grass card in C2, seen from C5 at 90 cm: probable C2, could also be B2, D2, C1, C3. With caution (a slide
    sideways or backwards), the Referee should count every cell it could be on."""
    w = world(at=("C5", "north"))
    w.sight("SEEN", "dry-grass", 0, 90)
    (cell, also), = w.entries("dry-grass")
    check(cell == "C2" and {"B2", "D2", "C1", "C3"} <= set(also), "C2, could also be B2, D2, C1, C3", [cell, also])
    check("C2" not in w.cart.card_cells(), "forwards, a card seen from far blocks nothing", sorted(w.cart.card_cells()))
    given = w.cart.card_cells(caution=True)
    check(given == {"C2"}, "E1 as written: with caution, C2 alone", sorted(given))
    raise Known(f"E1: with caution, card_cells gives {sorted(given)}, not the cells of could_also_be {list(also)}")


@mutant("a card seen from far placed too", "FC7")
def m_far_card_placed(m):
    rewrite(m, "saw", ("if not others and not self.frame:", "if not self.frame:"))


# --- running them -------------------------------------------------------------------------------------------------------
def fresh(patch=None):
    m = probe_tiers.load("cartographer_fixed", SOURCE)
    if patch:
        patch(m)
    return m


def play(name, patch=None):
    fn = SCENARIOS[name][0]
    try:
        MODULE["now"] = fresh(patch)
        fn()
        return "PASS", ""
    except MutantBroken as e:
        return "BROKEN", str(e)
    except Known as e:
        return "KNOWN", str(e)
    except AssertionError as e:
        return "FAIL", str(e)
    except Exception as e:  # noqa: BLE001
        return "FAIL", f"{type(e).__name__}: {e} (line {traceback.extract_tb(e.__traceback__)[-1].lineno})"


def play_mutant(patch):
    try:
        fresh(patch)
    except MutantBroken as e:
        return [], str(e)
    except Exception as e:  # noqa: BLE001
        return [], f"{type(e).__name__}: {e}"
    failed = []
    for name in SCENARIOS:
        outcome, detail = play(name, patch)
        if outcome == "BROKEN":
            return failed, f"in {name}: {detail}"
        if outcome == "FAIL":
            failed.append(name)
    return failed, None


def main(argv):
    try:
        return run(argv)
    finally:
        pc.shutil.rmtree(pc.STATE, ignore_errors=True)


def run(argv):
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
        return 1 if counts["MISSED"] or counts["BROKEN"] else 0
    chosen = argv or list(SCENARIOS)
    unknown = [n for n in chosen if n not in SCENARIOS]
    if unknown:
        print("unknown scenario: " + " ".join(unknown) + " (known: " + " ".join(SCENARIOS) + ")")
        return 2
    counts = {"PASS": 0, "FAIL": 0, "KNOWN": 0}
    for name in chosen:
        outcome, detail = play(name)
        counts[outcome] += 1
        print(f"{outcome:5}  {name} ({SCENARIOS[name][1]}){': ' + detail if detail else ''}")
    print(f"{counts['PASS']} PASS / {counts['FAIL']} FAIL / {counts['KNOWN']} KNOWN")
    return 1 if counts["FAIL"] else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
