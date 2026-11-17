"""The Pilot: how to reach the goal the Strategist chose. Executive layer, code without a model, about a second.

It finds the way (the shortest it knows through cells seen free, around cards, boxes and balls' lanes), explores to
find a card, waits for an adult when the robot asks for one, always under the Referee's control. It never chooses the
goal. It knows the terrain only through the Cartographer's answers.

Its language, from the Strategist (tools; TOOLS below gives each one's description and arguments):
    goal tools    explore(look_for), go_to_card(card)
    route tools   go_to_cell(cell), look_around(), face(direction), wait_for_event(seconds)
    step tools    step_forward(), move(direction, cells, stop_if), turn(direction), look_around(), face(direction):
                  passed to the Cartographer as they are
    and           robot_state(), seen_cards(), set_light(color)
Its settings are bricks of the group's personality (SETTINGS), always applied: code, not text.
To the Cartographer: commands of the mat (cells, north, east, south, west).
"""
import collections
import heapq
import json
import math

import clock
import common
from cartographer import REASONS, STOP_IF, Cartographer
from common import COLORS, FACING, STATE_DIR, cell_name, cell_xy

MAT = common.load_json(common.MAT_PATH)
RULES = common.load_json(common.RULES_PATH)
CART = Cartographer(MAT, RULES)
COLS, ROWS, CELL_CM = MAT["cols"], MAT["rows"], MAT["cell_cm"]
MAX_CELLS = RULES["max_cells_per_move"]
STEP = {"north": (0, -1), "east": (1, 0), "south": (0, 1), "west": (-1, 0)}
CAMERA_CM = 120
REPLANS = 8          # new plans in one go_to before giving up
BALL_WAITS = 6       # waits for a ball, in one leg, before giving up the leg
# A cell where a leg was refused BALL_WAITS times for a ball: the ways avoid it this long, and the travel gives up its
# goal at once. Before (8 November 2026), a survey without a model stayed more than ten minutes on E4, a cell of the
# ball's column, refused 93 times towards E3: 6 waits per leg, 8 plans per goal, every goal through E3.
BALL_AVOID_S = 60
LANE_WATCHES, LANE_WATCH_S = 3, 5  # before crossing a lane whose ball was not seen going away: watch, at most this long
BALL_RECENT_S = RULES["ball_seen_recently_s"]
ADULT_WAIT_S = 150   # how long to wait for an adult's Resume
VANTAGES = COLS * ROWS  # vantages one explore goes to, at most: a refusal that is not a limit of the adults (the robot
                        # still moving, a command not understood) could leave the same vantage chosen for ever (E2)
# The Pilot's settings: bricks of the group's personality (personalities/*.json), always applied.
SETTINGS = {
    "careful_balls": {"default": True, "means": "before each leg, wait while a ball seen just before comes closer, and "
                                                "watch a ball's lane before crossing it until its ball is seen going away"},
    "max_leg": {"default": 4, "means": "cells in one move, at most (1 to 4)"},
    "look_each_leg": {"default": False, "means": "look around before every leg"},
    "cross_when_boxed": {"default": False, "means": "when the only way left goes through a cell refused for a ball, "
                                                    "try it again anyway (the Referee still refuses while the ball is "
                                                    "near); without it, the robot gives up: no way there"},
}


# While the Strategist runs a mission, the Pilot obeys the mission's pause and stop, and has the robot stop on the next
# cell as soon as possible: before each move, and during one (the Cartographer asks hold_wanted during a slide and
# between quarter turns; the Referee then sends HOLD to the Reflex). The Pilot asks its Strategist, which relays the
# Interpreter's word: it never reads the Interpreter itself. Never for stack/drive.py: a person or an agent drives by
# hand, and nobody gives the Pilot a word.
ASK, HELD = None, None


def obey(ask=None, held=None):
    """The Strategist runs a mission: ask() says what the mission wants now, "go", "pause" or "stop"; held(where) takes
    the acknowledgement of a pause (where the robot stands still, or None once it moves again). obey() with nothing: no
    mission, no pause, no stop."""
    global ASK, HELD
    ASK, HELD = ask, held


def word():
    """What the mission wants now, as the Strategist says: "go" when no mission runs."""
    return ASK() if ASK is not None else "go"


CART.hold_wanted = lambda: word() in ("pause", "stop")


def command(name, *args):
    """One command of the mat, through the Cartographer (and the Referee's door)."""
    moves = {"step_forward", "move", "turn", "look_around", "face", "set_light"}
    if name in moves:
        heed()
    if name == "wait_for_event":
        return CART.wait_for(*args)
    return CART.call(lambda: getattr(CART, name)(*args), is_move=name in moves)


def heed():
    """Before a command that acts, or a wait of the Pilot's own: wait while the mission is paused, then stop the tool
    if it is stopped. Nothing without a mission."""
    if ASK is not None:
        hold_still()
        if word() == "stop":
            raise Stuck("the mission was stopped")


def wait(seconds):
    """A wait of the Pilot's own, in a goal or route tool. No command that acts follows a wait for a ball: without
    heed() first, a stop was seen only minutes later, and a pause acknowledged at the end
    of the tool."""
    heed()
    return command("wait_for_event", seconds)


class Stuck(Exception):
    pass


class AdultNeeded(Stuck):
    """The robot waits for an adult. Under the Strategist, the tool ends at once: its loop waits, without the model."""


def hold_still():
    """While the mission is paused, nothing moves: say so up the chain once, to the Strategist (where the robot stands),
    then wait."""
    if word() != "pause":
        return
    known = CART.sure and not CART.frame  # in a frame relative to the start, a cell name would mean nothing
    HELD({"cell": cell_name(CART.col, CART.row) if known else None,
          "facing": FACING[CART.facing] if known else None, "since": clock.stamp()})
    while word() == "pause":
        clock.sleep(0.5)
    HELD(None)


class Pilot:
    def __init__(self):
        self.blocked = {}      # cell -> time it was seen blocked
        self.ball_refused = {} # cell -> time a leg towards it was refused BALL_WAITS times for a ball
        self.free = set()      # cells the robot went through or the distance sensor saw free
        self.looked_from = set()
        self.pending = set()   # cells the Cartographer says the camera has not looked at yet (camera_coverage)
        self.tried = set()     # vantages chosen once already for them
        self.lanes = set()     # ("col", c), ("row", r) or ("cell", (c, r)): where a ball was seen rolling
        self.leaving = False
        self.covered = set()   # cells the camera has looked at
        self.state = {}
        self.moves_left = (RULES.get("limits") or {}).get("moves")  # None: the adults set no limit
        self.settings = {k: v["default"] for k, v in SETTINGS.items()}

    def configure(self, settings):
        """The settings of a new mission: the defaults, then the personality's bricks (a setting not given goes back to
        its default: a mission never keeps the bricks of the one before). What it knows of the mat stays; the robot's
        state is read again at its next move, even when a setting is refused (no setting changes then)."""
        # Between two missions someone may drive the robot by hand, and the Referee counts the moves of each mission
        # from zero: the state read before is not the robot's any more.
        self.state = {}
        self.moves_left = (RULES.get("limits") or {}).get("moves")
        self.tried, self.pending = set(), set()  # the extra look for the Cartographer's cells: once per mission
        for key, value in (settings or {}).items():
            if key not in SETTINGS:
                raise ValueError(f"no setting {key} for the Pilot")  # checked first: no setting changed
            if not isinstance(value, type(SETTINGS[key]["default"])) or isinstance(value, bool) != isinstance(
                    SETTINGS[key]["default"], bool):
                raise ValueError(f"the setting {key} is a {type(SETTINGS[key]['default']).__name__}")
        self.settings = {k: v["default"] for k, v in SETTINGS.items()}
        self.settings.update(settings or {})
        self.settings["max_leg"] = max(1, min(MAX_CELLS, self.settings["max_leg"]))

    def short(self, n):
        """n moves or fewer left under the adults' limit (never without a limit)."""
        return self.moves_left is not None and self.moves_left <= n

    # --- what the Cartographer says ------------------------------------------------------------------------------
    def read(self, answer):
        if answer.get("result") == "refused" and answer.get("why") == "the battery is too low":
            # The robot refuses every move until it is charged: before (8 November 2026), explore asked again and again,
            # 11,000 quarter turns refused in 20 minutes of a mission.
            raise Stuck("the battery is too low")
        if answer.get("result") == "refused" and "the adults allow" in str(answer.get("why", "")):
            # A limit of the adults (moves, commands, cells): nothing more of that kind this mission. Before, a look
            # refused for good left explore looping on the same cell.
            self.state = {}  # a first quarter turn may be done: the next tool reads where the robot is
            raise Stuck(answer["why"])
        state = answer.get("state") if isinstance(answer.get("state"), dict) else answer
        if "cell" in state and "position_known" in state:
            self.state = state
            if "moves_left" in state:
                self.moves_left = state["moves_left"] if isinstance(state["moves_left"], int) else None
            if state.get("position_known") and state.get("cell") and state.get("facing"):
                self.note_distance(cell_xy(state["cell"]), state["facing"], state.get("distance_ahead_cm"))
                if state.get("on_a_ball_lane"):
                    self.lanes.add(("cell", cell_xy(state["cell"])))
            if isinstance(state.get("ball_lanes"), list):
                self.lanes = self.lanes_of(state["ball_lanes"])  # the Cartographer's, on each ball's own line
            else:
                self.note_ball(state.get("ball_last_seen"))
        return answer

    def note_ball(self, ball):
        """The lane of a ball, as the Cartographer reports it: its column if it rolls north or south, its row if east or
        west, else its cell; for every cell it could be on."""
        if not ball or not ball.get("cell"):
            return
        for cell in [ball["cell"]] + list(ball.get("could_also_be") or []):
            c, r = cell_xy(cell)
            moving = ball.get("moving")
            self.lanes.add(("row", r) if moving in ("east", "west") else ("col", c) if moving in ("north", "south")
                           else ("cell", (c, r)))

    @staticmethod
    def lanes_of(names):
        """The Cartographer's lanes ("row 4", "column E", "cell E3") as the Pilot keeps them."""
        out = set()
        for name in names:
            kind, _, value = name.partition(" ")
            if kind == "row":
                out.add(("row", int(value) - 1))
            elif kind == "column":
                out.add(("col", ord(value) - ord("A")))
            elif kind == "cell":
                out.add(("cell", cell_xy(value)))
        return out

    def off_lane_near(self, cell):
        """The nearest cell off every lane (two steps at most), not a card, not seen blocked, the closest to the robot
        first, or None."""
        cards = self.card_cells()
        (rc, rr), _ = self.here()
        for d in (1, 2):
            near = [(cell[0] + dc, cell[1] + dr) for dc in range(-d, d + 1) for dr in range(-d, d + 1)
                    if abs(dc) + abs(dr) == d]
            for c, r in sorted(near, key=lambda p: (abs(p[0] - rc) + abs(p[1] - rr), p)):
                if 0 <= c < COLS and 0 <= r < ROWS and not self.on_lane((c, r)) and (c, r) not in cards \
                        and not self.is_blocked((c, r)):
                    return c, r
        return None

    def on_lane(self, cell):
        return ("col", cell[0]) in self.lanes or ("row", cell[1]) in self.lanes or ("cell", cell) in self.lanes

    def leave_lane(self):
        """Standing in a ball's lane, the robot gets hit: go to the nearest cell out of every lane first."""
        if self.leaving:
            return
        (c, r), _ = self.here()
        if not self.on_lane((c, r)):
            return
        self.leaving = True
        try:
            cards = self.card_cells()
            out = {(x, y) for x in range(COLS) for y in range(ROWS) if abs(x - c) + abs(y - r) <= 2
                   and not self.on_lane((x, y)) and not self.is_blocked((x, y)) and (x, y) not in cards}
            if out:
                self.travel(out)
        finally:
            self.leaving = False

    def lanes_crossed(self, dc, dr, cells):
        """The lanes ("row", r), ("col", c) or ("cell", cell) on the cells a leg goes through."""
        (c, r), _ = self.here()
        out = set()
        for k in range(1, cells + 1):
            x, y = c + dc * k, r + dr * k
            out |= {lane for lane in (("col", x), ("row", y), ("cell", (x, y))) if lane in self.lanes}
        return out

    def going_away(self, lanes):
        """The last ball, seen just before on one of these lanes, was moving away from the robot."""
        ball = self.state.get("ball_last_seen") or {}
        if not (self.ball_recent() and ball.get("how", "").startswith("moving away") and ball.get("cell")):
            return False
        c, r = cell_xy(ball["cell"])
        return bool({("col", c), ("row", r), ("cell", (c, r))} & lanes)

    def ball_recent(self):
        ball = self.state.get("ball_last_seen") or {}
        return bool(ball.get("t")) and clock.now() - ball["t"] < BALL_RECENT_S

    def note_distance(self, cell, facing, dist):
        """A reading of the distance sensor: the cells before the thing it sees are free, the cell of the thing is
        blocked (k cells ahead, k = 1 + dist // cell: 9 cm is the next cell, 69 cm the third). A cell stays blocked
        until a later reading sees it free. Nothing is blocked while a ball was seen in the last 10 s (the thing may
        be the ball). Cards lie flat: the sensor does not see them."""
        if dist is None:
            return
        dc, dr = STEP[facing]
        k = 1 + int(dist) // CELL_CM
        self.free.add(cell)
        for d in range(1, k):
            self.blocked.pop((cell[0] + dc * d, cell[1] + dr * d), None)
            self.free.add((cell[0] + dc * d, cell[1] + dr * d))
        thing = (cell[0] + dc * k, cell[1] + dr * k)
        if 0 <= thing[0] < COLS and 0 <= thing[1] < ROWS and not self.ball_recent():
            self.blocked[thing] = clock.now()

    def is_blocked(self, cell):
        return cell in self.blocked

    def here(self):
        if not self.state.get("position_known"):
            answer = self.read(command("robot_state"))
            if not self.state:  # the robot did not answer, and nothing was read since the mission began: never move
                raise Stuck(f"the robot could not find where it is on the mat: {answer.get('why', 'no state')}")
        if self.state.get("waiting_for_adult"):
            self.wait_for_adult()
        if not self.state.get("position_known"):
            self.localize()
        return cell_xy(self.state["cell"]), self.state["facing"]

    def wait_for_adult(self):
        if ASK is not None:  # under the Strategist, whose loop waits for the adult without its model
            self.state = {}  # the adult will put the robot somewhere: the next tool reads where
            raise AdultNeeded("the robot waits for an adult")
        end = clock.monotonic() + ADULT_WAIT_S
        while clock.monotonic() < end:
            command("wait_for_event", 30)
            self.read(command("robot_state"))
            if not self.state.get("waiting_for_adult"):
                if self.state.get("position_known"):
                    self.leave_lane()  # an adult may put it back in a ball's lane
                return
        raise Stuck("the robot still waits for an adult")

    def localize(self):
        """Find where the robot is on the mat before going anywhere: look around (the distance sensor sees the border, the
        camera the landmarks), then one cell forwards where the way is free, or a quarter turn, and again."""
        places = None
        for _ in range(8):
            self.read(command("look_around"))
            self.read(command("robot_state"))
            if self.state.get("waiting_for_adult"):
                self.wait_for_adult()
            if self.state.get("position_known"):
                return
            now = len(self.state.get("could_be") or [None] * 99)
            if self.short(2) or now == places:
                break  # a whole round ruled nothing out: the border alone will not settle it
            places = now
            answer = self.read(command("step_forward"))
            if answer.get("result") == "stopped":
                self.wait_for_adult()
            elif answer.get("result") != "done":
                self.read(command("turn", "right"))
            self.read(command("robot_state"))
            if self.state.get("position_known"):
                return
        raise Stuck("the robot could not find where it is on the mat (the border alone leaves two places on a mat that "
                    "looks the same turned round: a landmark in the rules settles it)")

    def records(self):
        answer = command("seen_cards")
        if "cards_seen" not in answer:
            raise Stuck(f"the Cartographer did not give the cards seen: {answer.get('why', answer.get('result'))}")
        left = (answer.get("camera_coverage") or {}).get("not_looked_at_yet")
        self.pending = {cell_xy(c) for c in left} if isinstance(left, list) else set()
        return [s for s in answer["cards_seen"] if s.get("cell")]  # no cell while the place on the mat is unknown

    def cards(self):
        """{card: [(cell, sure)]} from the Cartographer's record."""
        out = collections.defaultdict(list)
        for s in self.records():
            out[s["card"]].append((s["cell"], not s.get("could_also_be")))
        return out

    def card_cells(self):
        return {cell_xy(cell) for found in self.cards().values() for cell, _ in found}

    # --- moving -------------------------------------------------------------------------------------------------
    def check(self, answer):
        if answer.get("result") == "stopped":
            self.wait_for_adult()
            return False
        return answer.get("result") == "done"

    def face(self, direction):
        if self.state.get("facing") != direction:
            answer = self.read(command("face", direction))
            if not self.check(answer) and "state" not in answer:
                self.read(command("robot_state"))  # a refusal carries no state, and a first quarter turn may be done
        return self.state.get("facing") == direction

    def leg(self, direction, cells):
        """Face the way, then go forwards: the distance sensor and the camera watch the way."""
        dc, dr = STEP[direction]
        start = self.state.get("cell")
        watched, for_a_ball = 0, 0
        for _ in range(BALL_WAITS):
            if self.short(1):
                raise Stuck("no moves left")
            if not self.face(direction) or self.state.get("cell") != start:
                return 0  # plan again: finding its place or an adult's help may move the robot first
            c0, r0 = cell_xy(start)
            if self.is_blocked((c0 + dc, r0 + dr)) or (self.state.get("card_ahead") or {}).get("card"):
                # The state read after the face shows the first cell taken, by a thing or a card: plan again.
                # The plan already went round the cards the Pilot knew.
                return 0
            ball = self.state.get("ball_last_seen") or {}
            if self.settings["careful_balls"] and self.ball_recent() and "closer" in ball.get("how", ""):
                wait(5)  # cross only when the ball goes away
                self.read(command("robot_state"))
                continue
            crossed = self.lanes_crossed(dc, dr, cells)
            if self.settings["careful_balls"] and crossed and watched < LANE_WATCHES and not self.going_away(crossed):
                # A lane on the way, its ball not seen going away just before: the camera, facing the way, watches
                # where the leg crosses it (each run of the balls mission was hit once crossing a lane unseen, bench
                # of 7 November). After LANE_WATCHES watches without seeing it, the ball is far: go.
                watched += 1
                wait(LANE_WATCH_S)
                self.read(command("robot_state"))
                continue
            answer = self.read(command("move", direction, cells, []))
            why = answer.get("why", "") + " " + answer.get("stopped_because", "")
            if answer.get("result") == "refused" and "ball" in why:
                for_a_ball += 1
                wait(5)
                self.read(command("robot_state"))
                continue
            (c, r), _ = self.here()
            if answer.get("result") == "refused":
                if answer.get("why") == REASONS["obstacle"] and not self.ball_recent():
                    # Only a thing in front blocks the cell; not another move running, nor a rule (E9).
                    self.blocked[(c + dc, r + dr)] = clock.now()
                return 0
            moved = answer.get("cells_moved", 0)
            if not self.check(answer):
                return -1  # stopped on the way: an adult put it back, plan again
            if moved < cells and "something was in the way" in why and not self.ball_recent():
                self.blocked[(c + dc, r + dr)] = clock.now()
            return moved
        if for_a_ball:
            (c, r), _ = self.here()
            self.ball_refused[(c + dc, r + dr)] = clock.now()
            return -2  # the ball keeps the way: the travel gives up this goal, and the ways avoid that cell a while
        return 0

    def path(self, goals, avoid, turns_on_lanes=False):
        """Cheapest way (cells) from here to the nearest goal, through cells not blocked, not cards: a cell seen free
        costs 1, a cell never seen 3 (a wall may hide there)."""
        (c0, r0), _ = self.here()
        # A state is a cell and the step that led there: a way never turns on a ball's lane (out of it the same way it
        # came in), so that a leg never ends there. Before (8 November 2026), a way D3 -> E3 -> E2 stopped on E3, in
        # column E, a lane; the next plan left the lane back to D3, and planned E3 again: ten minutes of it.
        start = ((c0, r0), None)
        cost, prev, heap = {start: 0}, {start: None}, [(0, start)]
        while heap:
            k, state = heapq.heappop(heap)
            if k > cost[state]:
                continue
            cur, came = state
            if cur in goals:
                way = []
                while state != start:
                    way.append(state[0])
                    state = prev[state]
                return way[::-1]
            for dc, dr in STEP.values():
                if came is not None and came != (dc, dr) and self.on_lane(cur) and not turns_on_lanes:
                    continue
                nxt = (cur[0] + dc, cur[1] + dr)
                if not (0 <= nxt[0] < COLS and 0 <= nxt[1] < ROWS) or self.is_blocked(nxt) or nxt in avoid:
                    continue
                step = k + (1 if nxt in self.free else 3) + (8 if self.on_lane(nxt) else 0)
                if step < cost.get((nxt, (dc, dr)), 1e9):
                    cost[(nxt, (dc, dr))], prev[(nxt, (dc, dr))] = step, state
                    heapq.heappush(heap, (step, (nxt, (dc, dr))))
        return None

    def travel(self, goals):
        """Go to the nearest of `goals`; True when there, False when no way is left."""
        for _ in range(REPLANS):
            if not self.leaving:
                self.leave_lane()  # never wait on a ball's lane: the ball hits a robot that stands still too
            (c, r), _ = self.here()
            if (c, r) in goals:
                return True
            if self.settings["look_each_leg"]:
                self.look_around()  # before the plan: what it sees changes the way, and leaving a ball's lane moves it
                (c, r), _ = self.here()
                if (c, r) in goals:
                    return True
            cards = self.card_cells()
            now = clock.now()
            kept = {cell for cell, when in self.ball_refused.items() if now - when < BALL_AVOID_S}
            way = self.path(goals - cards, cards | kept)  # a card seen on the way may lie on a goal
            if way is None and self.leaving:
                # Already on a lane, every way out may have to turn on one (C4 on row 4 and column C, fixed_pilot F12)
                way = self.path(goals - cards, cards | kept, turns_on_lanes=True)
            if way is None and kept and self.settings["cross_when_boxed"]:
                # Boxed in, the ball's cell the only way out (seeds 10 and 18 of the forest, 8 November 2026): crossing
                # unseen got the robot hit 4 times; giving up never did. A choice of the group's personality.
                way = self.path(goals - cards, cards)
            if way is None:
                return False
            dc, dr = way[0][0] - c, way[0][1] - r
            direction = next(d for d, s in STEP.items() if s == (dc, dr))
            n = 1
            while n < len(way) and n < self.settings["max_leg"] and \
                    (way[n][0] - way[n - 1][0], way[n][1] - way[n - 1][1]) == (dc, dr):
                n += 1
            while n > 1 and self.on_lane(way[n - 1]) and way[n - 1] not in goals:
                n -= 1  # the longest leg cut on a ball's lane: stop before it, cross it in the next leg
            if self.leg(direction, n) == -2:
                return False
        (c, r), _ = self.here()
        return (c, r) in goals

    # --- looking ------------------------------------------------------------------------------------------------
    def visible_one_way(self, c, r, facing):
        """The cells the camera looks at from (c, r) facing this way: cone of about 30° each side, 120 cm, a ray stops
        at a cell seen blocked (a box hides what is behind it)."""
        dc, dr = STEP[facing]
        out = set()
        for side in range(-2, 3):
            for d in range(1, 5):
                if abs(side) > 0.58 * d or math.hypot(d, side) * CELL_CM > CAMERA_CM:
                    continue
                x, y = c + dc * d + (side if dr else 0), r + dr * d + (side if dc else 0)
                if not (0 <= x < COLS and 0 <= y < ROWS):
                    break
                out.add((x, y))
                if self.is_blocked((x, y)):
                    break
        return out

    def visible(self, c, r):
        return set().union(*(self.visible_one_way(c, r, f) for f in STEP))

    def look_around(self):
        self.leave_lane()
        (c, r), _ = self.here()
        answer = self.read(command("look_around"))
        looked = answer.get("looked") or answer.get("looked_before_it_stopped") or []
        for one in looked:
            self.note_distance((c, r), one["facing"], one.get("distance_ahead_cm"))
            self.covered |= self.visible_one_way(c, r, one["facing"])
        if looked:
            self.covered.add((c, r))
            self.looked_from.add((c, r))
        if not self.check(answer) and "state" not in answer:
            self.read(command("robot_state"))  # refused after a first quarter turn: the robot faces another way
        return answer

    def next_vantage(self):
        """The reachable cell that shows the most new cells, for the shortest way."""
        cards = self.card_cells()
        best, best_score = None, 0
        for c in range(COLS):
            for r in range(ROWS):
                if self.is_blocked((c, r)) or (c, r) in cards or self.on_lane((c, r)):
                    continue
                shown = self.visible(c, r) | {(c, r)}
                new = len(shown - self.covered)
                if (c, r) not in self.looked_from and (c, r) not in self.tried:
                    # A cell the Pilot counts as seen, the Cartographer not (a box hides it, or the cone is narrower
                    # than the Pilot's): one more look from a new place, once. Before (8 November 2026), the robot
                    # stopped exploring with C2 never looked at, and answered "not seen yet" instead of "not on the
                    # mat".
                    new += len(shown & self.covered & self.pending)
                if new == 0:
                    continue
                way = self.path({(c, r)}, cards)
                if way is None:
                    continue
                score = new / (1 + 0.5 * len(way))
                if score > best_score:
                    best, best_score = (c, r), score
        return best

    def refine(self, only=None):
        """Look again, from at most two cells away, at the cards whose cell is not sure."""
        for _ in range(4):
            if self.short(3):
                return
            unsure = [s for s in self.records() if s.get("could_also_be") and (only is None or s["card"] == only)]
            if not unsure:
                return
            tc, tr = cell_xy(unsure[0]["cell"])
            cards = self.card_cells()
            near = {(c, r) for c in range(COLS) for r in range(ROWS)
                    if 1 <= abs(c - tc) + abs(r - tr) <= 2 and (c, r) not in cards and not self.is_blocked((c, r))
                    and (c, r) not in self.looked_from and not self.on_lane((c, r))}  # a new point of view
            if not self.travel(near):
                return
            self.look_around()


PILOT = Pilot()


def logged(fn):
    """What each tool of the Pilot did, in .robot-sim/pilot.jsonl (the Cartographer's answers are in the Referee's log
    and the context of the caller)."""
    def wrapper(*args, **kwargs):
        start = clock.monotonic()
        try:
            result = fn(*args, **kwargs)
        except Stuck as e:
            result = {"result": "gave up", "why": str(e)}
            try:
                result["cards_seen"] = {k: [c for c, _ in v] for k, v in PILOT.cards().items()}
            except Stuck:
                pass
        result["moves_left"] = PILOT.moves_left
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        with (STATE_DIR / "pilot.jsonl").open("a") as f:
            f.write(json.dumps({"tool": fn.__name__, "arguments": [args, kwargs], "result": result,
                                "seconds": round(clock.monotonic() - start, 1)}, ensure_ascii=False) + "\n")
        return result
    wrapper.__name__ = fn.__name__
    wrapper.__doc__ = fn.__doc__
    return wrapper


def merged(records):
    """The Cartographer keeps one record per sighting it could not match: a card seen from far, then from closer, may
    be there twice. A record that is not sure is dropped when another record of the same card is on one of its cells
    (or has it among its own); a sure record is always kept (two cards of a kind can lie side by side)."""
    def cells(s):
        return {s["cell"], *(s.get("could_also_be") or [])}
    kept = [s for s in records if not s.get("could_also_be")]
    for s in sorted((s for s in records if s.get("could_also_be")), key=lambda s: s.get("distance_cm", 999)):
        if not any(k["card"] == s["card"] and (k["cell"] in cells(s) or s["cell"] in cells(k)) for k in kept):
            kept.append(s)
    return kept


# --- goal tools: the Pilot finds the way, looks and waits by itself ---------------------------------------------
@logged
def explore(look_for=None):
    """Look around, then move to the places that show the most of the mat not seen yet, until the card `look_for` is
    seen, or, without look_for, until the whole mat has been seen or the moves run out; then look again from closer at
    the cards whose cell is not sure. Answers every card seen, with its cell."""
    PILOT.here()
    if not PILOT.covered:
        PILOT.look_around()
    for _ in range(VANTAGES):
        if look_for and look_for in PILOT.cards():
            break
        if PILOT.short(2):
            break
        target = PILOT.next_vantage()
        if target is not None:
            PILOT.tried.add(target)
        if target is None:
            break
        if PILOT.travel({target}):
            PILOT.look_around()
        else:
            PILOT.covered.add(target)  # unreachable for now: do not choose it again
    PILOT.refine(only=look_for)
    records = merged(PILOT.records())
    cards = collections.defaultdict(list)
    for s in records:
        cards[s["card"]].append(s["cell"])
    return {"result": "found" if look_for in cards else ("not found" if look_for else "done"),
            "cards_seen": dict(cards),
            "not_sure": [{"card": s["card"], "cell": s["cell"], "could_also_be": s["could_also_be"]}
                         for s in records if s.get("could_also_be")],
            "mat_covered": PILOT.next_vantage() is None}


@logged
def go_to_card(card):
    """Take the robot right in front of a card, facing it, by the shortest way the robot knows. If the camera has not
    seen this card yet, explore the mat first. Answers where the robot stopped, or why it could not."""
    # A card seen from far may be on a neighbour of its probable cell (could_also_be): the cells are tried from the
    # surest, and a cell faced from next to it without that card is ruled out, for this call only: the Pilot forgets
    # the goal once reached or given up. Before, a cell stayed ruled out for the whole process, even
    # when the turn towards it had failed.
    # A place with no way to it now (a ball keeps its lane) is not ruled out: before (8 November 2026, seeds 10 and
    # 18 of the forest), the barbecue on F3 was ruled out with its two neighbours because the ball kept column E, and
    # the robot gave up. The ways are tried again once the ball has had time to pass.
    PILOT.here()  # the cells of the record mean something once the robot knows where it is on the mat
    unreachable, ruled_out = set(), set()
    no_way = False  # one of its places had no way to it: the reason of a give up (E6)
    for attempt in range(8):
        candidates = {}
        for s in PILOT.records():
            if s["card"] != card:
                continue
            for cell, far in [(s["cell"], s["distance_cm"])] + [(c, s["distance_cm"] + CELL_CM) for c in s["could_also_be"]]:
                if cell not in ruled_out and cell not in unreachable and \
                        not PILOT.is_blocked(cell_xy(cell)):
                    candidates[cell] = min(far, candidates.get(cell, 1e9))
        if not candidates and unreachable:
            wait(10)  # no way to any of its places now: let the ball pass, then try again
            PILOT.read(command("robot_state"))
            unreachable.clear()
            continue
        if not candidates:
            if PILOT.short(2):
                if not any(s["card"] == card for s in PILOT.records()):
                    raise Stuck("no moves left")  # never seen, and too few moves to look for it (E6)
                break
            looked = explore(card)  # never seen, or every place ruled out: look again
            if looked.get("result") == "gave up":
                # explore is a tool of its own (@logged): it turned an adult's wait, or another reason to stop, into
                # an answer. Passed on as it is: else the Strategist never waits for the adult (4a's review, 8 Nov).
                return {"result": "gave up", "why": looked.get("why", "")}
            if not any(s["card"] == card for s in PILOT.records()):
                return {"result": "not found", "why": f"the camera never saw a card {card}"}
            continue
        cell = min(candidates, key=candidates.get)
        tc, tr = cell_xy(cell)
        fronts = {}
        for direction, (dc, dr) in STEP.items():
            fc, fr = tc - dc, tr - dr
            if 0 <= fc < COLS and 0 <= fr < ROWS:
                fronts[(fc, fr)] = direction  # from (fc, fr), the card is `direction`
        goals = set(fronts) - PILOT.card_cells()
        goals = {f for f in goals if not PILOT.on_lane(f)} or goals  # never wait in a ball's lane if it can be helped
        if not PILOT.travel(goals):
            unreachable.add(cell)  # no way there now: try the next place, and this one again later
            no_way = True
            continue
        (c, r), _ = PILOT.here()
        PILOT.face(fronts[(c, r)])
        clock.sleep(0.5)  # the camera names a card after 200 ms standing still
        state = PILOT.read(command("robot_state"))
        if not state.get("position_known"):
            PILOT.here()  # the place was lost on the way: find it, then try again; done names the card's cell (E3)
            continue
        if (state.get("card_ahead") or {}).get("card") == card:
            return {"result": "done", "cell": state["cell"], "facing": state["facing"], "card_ahead": card}
        if state.get("cell") == cell_name(c, r) and state.get("facing") == fronts[(c, r)]:
            ruled_out.add(cell)  # faced from next to it, and that card was not there; else the attempt counts only
    if no_way:
        return {"result": "gave up", "why": f"no way to the card {card} now (a ball's lane, or something in the way)"}
    return {"result": "gave up", "why": f"the robot did not find itself in front of the card {card}"}


# --- route tools: the Pilot finds the way; the Strategist chooses when to look, how far to go, whether to wait ----
@logged
def go_to_cell(cell):
    """Take the robot to a cell by the shortest way it knows, without looking around on the way. It never drives onto a
    card it knows, and stops by itself in front of a card or of an obstacle. Answers where the robot is and its
    state."""
    if not (len(cell) >= 2 and cell[0] in "ABCDEFGH"[:COLS] and cell[1:].isdigit() and 1 <= int(cell[1:]) <= ROWS):
        return {"result": "refused", "why": f"{cell} is not a cell of the mat"}
    target = cell_xy(cell)
    refused = {"result": "refused", "why": f"a card is on {cell}: the robot never drives onto a card; go to a cell next "
                                           "to it and face it"}
    if target in PILOT.card_cells():
        return refused
    PILOT.here()  # the cells of the record mean something once the robot knows where it is on the mat
    if target in PILOT.card_cells():
        return refused
    if PILOT.on_lane(target):
        # The robot would stop where a ball rolls (two bumps and two adults in C4, balls mission, 7 November).
        near = PILOT.off_lane_near(target)
        return {"result": "refused", "why": f"{cell} is on the lane of a ball: the robot never stops where a ball rolls",
                "what_to_do": (f"go to {cell_name(*near)} instead, next to it and off every lane" if near else
                               "wait for the ball to go (wait_for_event), look around, then choose again")}
    there = PILOT.travel({target})
    (c, r), facing = PILOT.here()
    answer = {"result": "done" if there else "not there", "cell": cell_name(c, r), "facing": facing, "state": PILOT.state}
    if not there:
        answer["why"] = "the way was blocked or refused; look around, then choose again"
    return answer


@logged
def look():
    """Turn on the spot all the way round, in four quarter turns to the right, stopping after each one so that the
    camera can look. It counts as one move. The answer says, for each direction, the card in front, the distance to
    what is in front and what the camera saw (cards and balls, with their cells). The robot ends facing the way it
    started."""
    return PILOT.look_around()


@logged
def face_towards(direction):
    """Turn on the spot until the robot faces this direction (north is towards row 1): one or two quarter turns,
    counted as one move, or nothing if it already faces it."""
    PILOT.face(direction)
    return {"result": "done" if PILOT.state.get("facing") == direction else "not done", "state": PILOT.state}


@logged
def state():
    """Where the robot is (its cell, the way it faces), what is in front, the last ball seen, the moves left. Moves
    nothing."""
    return PILOT.read(command("robot_state"))


@logged
def light(color):
    """Change the colour of the robot's light."""
    return command("set_light", color)


def watch(seconds=10):
    """Watch the robot for up to `seconds` (1 to 30) and answer as soon as something happens to it (something touched
    it, came close in front, a move stopped, an adult pressed Resume, the camera saw a ball or a card), or when the time
    is over. Events from before the wait come with the answer, marked "before_you_started_waiting", but do not end it.
    Moves nothing. It is for watching, not for safety: the robot protects itself long before an answer could arrive."""
    return command("wait_for_event", max(1, min(30, int(seconds))))


def cards():
    """The whole record of the cards the robot's camera recognised, each with the cell where it is, where the robot
    saw it from, and when. Moves nothing. You rarely need it: each new card the camera sees already comes in
    "since_last_call" of the next answer, whatever the tool, and "card_ahead" names the card in front. Call it to review
    the whole record, for example after a long pause. The camera names cards from a short list, up to 120 cm ahead in a
    cone that widens with distance, only while the robot stands still; the farther a card, the less sure its cell
    ("could_also_be"), and seeing it again from closer makes the estimate better. "camera_coverage" lists the cells the
    camera has not looked at yet: when a card you look for is not in "cards_seen" and no cell is left, it is not on the
    cells the camera could reach."""
    return command("seen_cards")


# --- step tools: passed to the Cartographer as they are ---------------------------------------------------------
def step_forward():
    """Move the robot one cell forwards, then stop. The robot refuses when something is in front, and stops by itself
    if something appears. Never off the mat, never onto a card, never onto a cell where a ball was seen in the last 10
    seconds (nor next to one that was coming closer). If the result is "stopped", do what "what_to_do" says before
    anything else. The answer gives the new state, with the card in front ("card_ahead") and what happened meanwhile
    ("since_last_call"), cards seen included."""
    return command("step_forward")


def move(direction, cells=1, stop_if=None):
    """Slide the robot 1 to 4 cells (`cells`) towards north, east, south or west (north is towards row 1), without
    turning: its wheels move it in any of the four directions, and the camera keeps looking the way the robot faces. It
    counts as one move, however many cells. It stops on a cell, at the first of: all the cells done, the edge of the
    mat, or a condition of stop_if met: "card_ahead" (a card right in front of the camera, the way the robot faces; a
    slide forwards always stops in front of a card, with or without it), "ball_seen" (the camera sees a ball),
    "new_card" (the camera shows a kind of card, a name, it did not show when the move began). The robot never drives
    onto a card it knows: forwards it stops in front of any card; every slide stops before a cell where the
    Cartographer placed a card for sure, and a slide sideways or backwards before any card the camera placed, even
    from far. With stop_if, the robot stands still on each cell until its camera has looked. Its distance sensor and
    camera only look the way it faces: in any other direction it slides only into cells the sensor saw free in the
    last two minutes (face that way once, or look_around; the cells it has just been through count too), never into
    the lane of a ball (the row or column where one was seen rolling) and never near a ball seen just before (from
    far, a lane may cover two rows or columns); forwards, never onto a cell where a ball was seen just before, nor
    next to one that was coming closer. The sensor's beam is narrow, so a small thing near the side of a cell can be
    missed, and then the bumper stops the robot. The answer says how many cells it moved and why it stopped;
    "stopped" means an adult must come, as for step_forward."""
    stop_if = list(dict.fromkeys(stop_if or []))
    if not isinstance(cells, int) or isinstance(cells, bool) or not 1 <= cells <= MAX_CELLS:
        return {"result": "refused", "why": f"cells must be 1 to {MAX_CELLS}: nothing was sent"}
    return command("move", direction, cells, stop_if)


def turn(direction):
    """Turn the robot a quarter turn on the spot, to its left or to its right, then stop."""
    return command("turn", direction)


def look_around():
    """Turn on the spot all the way round, in four quarter turns to the right, stopping after each one so that the
    camera can look. It counts as one move. The answer says, for each direction, the card in front, the distance to
    what is in front and what the camera saw (cards and balls, with their cells). The robot ends facing the way it
    started."""
    return command("look_around")


def face(direction):
    """Turn on the spot until the robot faces this direction (north is towards row 1): one or two quarter turns,
    counted as one move, or nothing if it already faces it."""
    return command("face", direction)


def robot_state():
    """What the robot measures now (distance to what is in front, the card in front, battery, emergency stop, the last
    ball the camera saw and whether it was coming closer or going away) and the cell where the Cartographer thinks it
    is, and whether that cell is on a ball's lane. Moves nothing. The cell is a count of the moves done, not a measure:
    after an interrupted move it is unknown."""
    return command("robot_state")


def set_light(color):
    """Change the colour of the robot's light."""
    return command("set_light", color)


# The tools of the Pilot, by kind: name -> (function, read only, properties, required). The Strategist offers a model
# one kind (the architecture keeps the route tools: the personality changes how the robot behaves, without every step
# going through the model); stack/drive.py serves the step tools through MCP.
DIRECTION = {"type": "string", "enum": FACING}
CARD = {"type": "string", "enum": sorted(MAT["cards"])}
CELLS = [cell_name(c, r) for r in range(ROWS) for c in range(COLS)]
COLOR = {"type": "string", "enum": list(COLORS)}
SECONDS = {"type": "integer", "minimum": 1, "maximum": 30}
TOOLSETS = {
    "goal": {
        "robot_state": (state, True, {}, []),
        "go_to_card": (go_to_card, False, {"card": CARD}, ["card"]),
        "explore": (explore, False, {"look_for": CARD}, []),
        "set_light": (light, False, {"color": COLOR}, ["color"]),
        "seen_cards": (cards, True, {}, []),
    },
    "route": {
        "robot_state": (state, True, {}, []),
        "go_to_cell": (go_to_cell, False, {"cell": {"type": "string", "enum": CELLS}}, ["cell"]),
        "look_around": (look, False, {}, []),
        "face": (face_towards, False, {"direction": DIRECTION}, ["direction"]),
        "wait_for_event": (watch, True, {"seconds": SECONDS}, []),
        "seen_cards": (cards, True, {}, []),
        "set_light": (light, False, {"color": COLOR}, ["color"]),
    },
    "step": {
        "robot_state": (robot_state, True, {}, []),
        "step_forward": (step_forward, False, {}, []),
        "move": (move, False, {"direction": DIRECTION, "cells": {"type": "integer", "minimum": 1, "maximum": MAX_CELLS},
                               "stop_if": {"type": "array", "items": {"type": "string", "enum": sorted(STOP_IF)}}},
                 ["direction"]),
        "turn": (turn, False, {"direction": {"type": "string", "enum": ["left", "right"]}}, ["direction"]),
        "look_around": (look_around, False, {}, []),
        "face": (face, False, {"direction": DIRECTION}, ["direction"]),
        "set_light": (set_light, False, {"color": COLOR}, ["color"]),
        "wait_for_event": (watch, True, {"seconds": SECONDS}, []),
        "seen_cards": (cards, True, {}, []),
    },
}
MOVES = {"step_forward", "move", "turn", "look_around", "face", "go_to_card", "explore", "go_to_cell"}


def operate(fn, **arguments):
    """One operation asked of the Pilot (a tool, whoever asks: the Strategist's model, the Strategist without a model,
    an agent through stack/drive.py): counted, then done."""
    common.count("operations")
    return fn(**arguments)
