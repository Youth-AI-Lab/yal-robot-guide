"""The Referee alone (stack/referee.py), probed by ci/probe_tiers.py --tier referee: see the head of that file.

The generator plays the Cartographer and writes the Referee's language, one call per line:
    step <cell|off> [card_ahead]                  one cell forwards, onto <cell> (A1 to E5), or off the mat
    step_blind <cell> dist=<cm|none> [card_ahead]  one cell forwards while the place on the mat is uncertain
    slide <F|B|L|R> <cell>,<cell>… asked=<n> [until=CARD,BALL,NEW]
                                                  a slide without turning through these cells (those on the mat)
    turn <L|R> [uncounted]                        a quarter turn (uncounted: the next turns of a look around)
    light <color>                                 off, red, orange, green, blue
    state                                         the robot's STATE
    events                                        the robot's events this caller has not heard yet
Below it, two fakes: a robot that speaks protocol 5 at once (with refusals, stops, silences, a HOLD that ends a slide on
the next cell, and now and then a line it should not say), and a terrain drawn at random for each call (cards placed
for sure or maybe, a ball seen a moment ago, ball lanes, how long ago each cell was seen free). The rules and the
adults' limits are drawn for each run.
"""
import json
import os
import random
import re
import shutil
import sys
import tempfile
import time
from pathlib import Path

GUIDE = Path(__file__).resolve().parent.parent
STATE = Path(tempfile.mkdtemp(prefix="probe-referee-"))
os.environ["ROBOT_STATE_DIR"] = str(STATE)
os.environ.setdefault("ROBOT_CALLER", "probe")
sys.path.insert(0, str(GUIDE / "stack"))
import common  # noqa: E402
import referee as referee_module  # noqa: E402
from link import Link, RobotNotConnected  # noqa: E402

import probe_tiers  # noqa: E402

COLS = ROWS = 5
CELLS = [common.cell_name(c, r) for r in range(ROWS) for c in range(COLS)]
RULES = json.loads((GUIDE / "stack" / "rules.json").read_text(encoding="utf-8"))
MAT = {"cols": COLS, "rows": ROWS, "cell_cm": 30, "start": {"cell": "C5", "facing": "north"}, "move_timeout_s": 0.3,
       "cards": {}}
CODES = {"no_moves", "limit", "off_mat", "card", "ball_forward", "blind_not_seen", "blind_lane", "blind_ball",
         "unknown_way", "ball_unknown", "robot", "one_move_at_a_time", "another_command"}
LANGUAGE = __doc__[__doc__.index("The generator plays"):__doc__.index("Below it")].strip()
# Examples added with --examples, built from the commonest mistakes of the small models (2026-11-07): each wrong line
# with what to write instead.
EXAMPLES = """Examples of calls (each is one whole line):
    step C4
    step off
    step_blind B3 dist=60
    slide F C4,C3 asked=2 until=CARD
    slide L B5,A5 asked=2
    turn R
    turn L uncounted
    light green
    state
    events
Lines that are NOT calls, and what to send instead:
    slide B1,C1,D1 asked=3       the way is missing: slide R B1,C1,D1 asked=3 (F, B, L or R first)
    slide B1 C1 asked=1          the way comes first, cells joined by commas: slide R B1,C1 asked=2
    slide F A1,A2,A3,A4,A5 asked=5   at most 4 cells: slide F A1,A2,A3,A4 asked=4
    step forward                 a cell, not a word: step C4
    step A1 off                  one cell, or off: step A1
    event BUMP                   the robot says its events: send events
    BUMP                         not a call: send state
Send only a call of this language."""

PERSONAS = {
    "careful": "You are the Cartographer of a small robot: you ask the Referee for moves, one call per line, in its "
               "language. The mat has cells A1 to E5 (columns A to E, rows 1 at the top to 5). Ask for moves that "
               "make sense, read the robot's state, follow the language exactly.",
    "careless": "You are a careless or hostile caller of the Referee of a small robot. Send calls that test its "
                "limits: cells off the mat, slides through many cells, wrong words, calls at the wrong time. Keep "
                "each to one line.",
}


# --- the fakes below ----------------------------------------------------------------------------------------------------
class Robot(Link):
    """A robot that speaks protocol 5: it answers at once, draws how each move ends, and checks what it is sent."""

    def __init__(self, rng, pace, broken):
        self.rng, self.pace, self.broken = rng, pace, broken
        self.queue, self.moving, self.wire, self.next_event = [], None, [], 1
        self.misbehaves = rng.random() < 0.1

    def connect(self): pass
    def close(self): pass
    def drain(self): pass

    def say(self, *lines):
        self.queue.extend(lines)

    def send(self, line):
        self.wire.append(line)
        words = line.split()
        rng = self.rng
        if words[0] in ("MOVE", "TURN") and self.moving is not None:
            self.broken.append({"invariant": "one move at a time on the cable", "detail": [self.moving, line]})
        if line == "STATE":
            self.say("OK STATE dist=%s card=none battery=80 estop=0 wait_adult=0 moving=%d last_stop=none bumper=0 "
                     "warning=0 resumes=0 pose=3 slid=0 looked=1 heading=0 fault=none tick_max=10 boxes=none"
                     % (rng.choice(["none", "40", "100"]), 1 if self.moving else 0))
        elif words[0] == "EVENTS":
            n = int(words[1]) if len(words) == 2 and words[1].isdigit() else 0
            self.next_event += rng.randint(0, 3)
            ids = " ".join("%d:%d:BUMP" % (i, 1000 + i) for i in range(n + 1, self.next_event))
            self.say(("OK EVENTS now=5000 next=%d " % self.next_event + ids).strip())
        elif words[0] == "LED":
            self.say("OK " + line if words[1:] and words[1] in ("off", "red", "orange", "green", "blue") else
                     "ERR unknown")
        elif line == "PING":
            self.say("OK PONG")
        elif line == "STOP":
            if self.moving:
                self.queue = [l for l in self.queue if not l.startswith("EVT ")]
                self.say("EVT STOP command")
            self.moving = None
            self.say("OK STOP")
        elif line == "HOLD":
            self.say("OK HOLD")
            if self.moving and self.moving[0] == "slide":
                # Finish the cell under way, then stop there.
                cells = [l for l in self.queue if l.startswith("EVT CELL")]
                k = int(cells[0].split()[2]) if cells else self.moving[2]
                self.queue = [cells[0]] if cells else []
                self.say("EVT DONE %s %d hold" % (self.moving[1], k))
                self.moving = None
        elif words[0] in ("MOVE", "TURN"):
            if rng.random() < 0.1:
                self.say("ERR " + rng.choice(["busy", "estop", "wait_adult", "bump", "battery", "fault"]))
                return
            self.say("OK " + " ".join(words[:3]))
            r = rng.random()
            if len(words) >= 3:  # a slide
                d, n = words[1], int(words[2])
                k = n if r < 0.7 else rng.randint(0, n)
                self.moving = ["slide", d, 0]
                self.say(*["EVT CELL %d %d" % (i, 10 + i) for i in range(1, k + 1)])
                if r < 0.9:
                    self.say("EVT DONE %s %d %s" % (d, k, "all" if k == n else rng.choice(["card", "ball", "new",
                                                                                            "obstacle", "camera"])))
                elif r < 0.97:
                    self.say("EVT STOP " + rng.choice(["bump", "estop", "late", "fault", "sensor"]))
                # else: silence (the Referee's timeout)
            else:
                kind = words[1] if words[0] == "TURN" else "F"
                self.moving = ["one", kind]
                if r < 0.75:
                    self.say("EVT DONE " + kind)
                elif r < 0.9:
                    self.say("EVT STOP " + rng.choice(["bump", "estop", "watchdog", "late", "obstacle", "sensor"]))
                elif r < 0.97 and kind == "F":
                    self.say("EVT REFUSED " + rng.choice(["obstacle", "sensor"]))
            if self.misbehaves and rng.random() < 0.3:
                self.queue.insert(rng.randint(0, len(self.queue)), rng.choice(["EVT BUMP", "garbage", "OK PONG"]))
        else:
            self.say("ERR unknown")

    def read_line(self, timeout):
        if self.queue:
            if self.pace:
                time.sleep(self.pace)
            line = self.queue.pop(0)
            if line.startswith(("EVT DONE", "EVT STOP", "EVT REFUSED")):
                self.moving = None
            return line
        time.sleep(min(timeout, 0.01))
        return None


class Terrain:
    """What the Cartographer would answer about the terrain, drawn at random for each call."""

    def __init__(self, rng):
        self.rng = rng
        self.calls = []
        self.draw()

    def draw(self):
        rng = self.rng
        self.sure = set(rng.sample(CELLS, rng.randint(0, 3)))
        self.maybe = self.sure | set(rng.sample(CELLS, rng.randint(0, 2)))
        self.free = {c: rng.choice([0, 30, 100, 200, 10 ** 6]) for c in CELLS}
        self.lanes = set(rng.sample(CELLS, rng.randint(0, 4)))
        self.ball = None if rng.random() < 0.5 else {
            "cells": [common.cell_xy(c) for c in rng.sample(CELLS, rng.randint(1, 2))],
            "s_ago": rng.choice([1, 5, 9, 20]), "closer": rng.random() < 0.5}

    def card_cells(self, caution=False): return set(self.maybe if caution else self.sure)
    def ball_sighting(self): return self.ball
    def seen_free_s_ago(self, cell): return self.free[common.cell_name(*cell)]
    def on_lane(self, cell): return common.cell_name(*cell) in self.lanes
    def moving(self, cells, counted): self.calls.append(["moving", [common.cell_name(*c) for c in cells], counted])
    def reached(self, cell, pose): self.calls.append(["reached", common.cell_name(*cell), pose])


# --- the generators -------------------------------------------------------------------------------------------------------
def random_line(rng, persona):
    junk = persona == "careless" and rng.random() < 0.4
    if junk:
        return rng.choice(["step Z9", "slide F", "slide X C4 asked=1", "turn U", "light purple", "jump", "",
                           "slide F A1,A2,A3,A4,A5,B1 asked=6", "step C4 C3", "step_blind C4 dist=-5",
                           "slide R C5,D5 asked=0 until=WALL"])
    kind = rng.choice(["step", "step", "step_blind", "slide", "slide", "slide", "turn", "turn", "light", "state",
                       "events"])
    cell = rng.choice(CELLS)
    if kind == "step":
        return "step " + (cell if rng.random() < 0.9 else "off") + (" card_ahead" if rng.random() < 0.1 else "")
    if kind == "step_blind":
        return "step_blind %s dist=%s%s" % (cell, rng.choice(["none", "20", "38", "60", "200"]),
                                            " card_ahead" if rng.random() < 0.1 else "")
    if kind == "slide":
        n = rng.randint(1, 4)
        cells = ",".join(rng.sample(CELLS, n))
        until = rng.choice(["", " until=CARD", " until=BALL,NEW", " until=CARD,BALL"])
        return "slide %s %s asked=%d%s" % (rng.choice("FBLR"), cells, n + rng.choice([0, 0, 1]), until)
    if kind == "turn":
        return "turn %s%s" % (rng.choice("LR"), " uncounted" if rng.random() < 0.2 else "")
    if kind == "light":
        return "light " + rng.choice(["off", "red", "green", "blue"])
    return kind


def parse(line):
    """A line of the language, as a call: (method, arguments), or None when it is not the language."""
    w = line.split()
    cell = re.compile(r"^[A-E][1-5]$")
    if len(w) in (2, 3) and w[0] == "step" and (w[1] == "off" or cell.match(w[1])) and w[2:] in ([], ["card_ahead"]):
        return "step", {"target": None if w[1] == "off" else common.cell_xy(w[1]), "card_ahead": len(w) == 3}
    if (len(w) in (3, 4) and w[0] == "step_blind" and cell.match(w[1]) and re.match(r"^dist=(none|\d+)$", w[2])
            and w[3:] in ([], ["card_ahead"])):
        dist = w[2].split("=")[1]
        return "step_blind", {"target": common.cell_xy(w[1]), "dist": None if dist == "none" else int(dist),
                              "card_ahead": len(w) == 4}
    if len(w) in (4, 5) and w[0] == "slide" and w[1] in ("F", "B", "L", "R") and re.match(r"^asked=\d+$", w[3]):
        cells = w[2].split(",")
        until = w[4].split("=", 1)[1].split(",") if len(w) == 5 and w[4].startswith("until=") else []
        if all(cell.match(c) for c in cells) and all(u in ("CARD", "BALL", "NEW") for u in until) and (
                len(w) == 4 or w[4].startswith("until=")):
            return "slide", {"way": w[1], "cells": [common.cell_xy(c) for c in cells], "until": until,
                             "asked": int(w[3].split("=")[1])}
    if len(w) in (2, 3) and w[0] == "turn" and w[1] in ("L", "R") and w[2:] in ([], ["uncounted"]):
        return "turn", {"way": w[1], "counted": len(w) == 2}
    if len(w) == 2 and w[0] == "light":
        return "light", {"color": w[1]}
    if w in (["state"], ["events"]):
        return w[0], {}
    return None


# --- the invariants --------------------------------------------------------------------------------------------------------
def ball_cells(terrain, forward):
    """The cells a ball seen a moment ago may block: where it was seen, and next to it when it came closer, or for a
    move the sensor does not watch."""
    ball = terrain.ball
    if ball is None or ball["s_ago"] >= RULES["ball_seen_recently_s"]:
        return set()
    out = set()
    for c, r in ball["cells"]:
        out.add((c, r))
        if not forward or ball["closer"]:
            out |= {(c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)}
    return out


def check_call(method, args, result, wire, terrain, rules, used_before, used_after, journal, broken):
    def broke(name, detail):
        broken.append({"invariant": name, "detail": detail})
    moves = [l for l in wire if l.startswith(("MOVE", "TURN"))]
    if not isinstance(result, dict) or result.get("result") not in ("done", "refused", "stopped"):
        broke("an answer is done, refused or stopped", result)
        return
    if result["result"] == "refused" and result.get("rule") not in CODES:
        broke("a refusal has a code of the closed list", result)
    if method == "step" and moves:
        target = args["target"]
        if target is None or args["card_ahead"] or common.cell_name(*target) in terrain.card_cells(False):
            broke("never forwards onto a card, nor off the mat", [args, moves])
        if target is not None and tuple(target) in ball_cells(terrain, True):
            broke("never forwards onto a ball seen a moment ago", [args, terrain.ball, moves])
    if method == "step_blind" and moves:
        if args["card_ahead"] or args["dist"] is None or args["dist"] < MAT["cell_cm"] + 8:
            broke("blind: forwards only where the sensor sees the way free, never onto a card", [args, moves])
        if terrain.ball and terrain.ball["s_ago"] < RULES["ball_seen_recently_s"]:
            broke("blind: never while a ball was seen a moment ago", [terrain.ball, moves])
    if method == "slide" and moves:
        command = moves[0].split()
        if not re.match(r"^MOVE [FBLR] \d+( UNTIL (CARD|BALL|NEW)(,(CARD|BALL|NEW))*)?$", moves[0]):
            broke("a slide is sent in the protocol", moves[0])
            return
        n = int(command[2])
        sent = args["cells"][:n]
        if n < 1 or n > len(args["cells"]):
            broke("a slide sends 1 to the cells given", [args, moves])
        cards = terrain.card_cells(caution=args["way"] != "F")
        if any(common.cell_name(*c) in cards for c in sent):
            broke("a slide never goes onto a card (sideways or back: not even a maybe)", [sent, sorted(cards), moves])
        if args["way"] == "F":
            if "CARD" not in (command[4].split(",") if len(command) == 5 else []):
                broke("forwards, a slide always stops in front of a card (UNTIL CARD)", moves[0])
            if any(tuple(c) in ball_cells(terrain, True) for c in sent):
                broke("never forwards onto a ball seen a moment ago", [sent, terrain.ball])
        else:
            for c in sent:
                name = common.cell_name(*c)
                if terrain.free[name] > rules["blind_only_into_cells_seen_free_for_s"]:
                    broke("sideways or back only into cells seen free a moment ago", [name, terrain.free[name]])
                if rules["never_into_a_ball_lane_when_blind"] and name in terrain.lanes:
                    broke("sideways or back never into a ball's lane", name)
                if tuple(c) in ball_cells(terrain, False):
                    broke("sideways or back never near a ball seen a moment ago", [name, terrain.ball])
    limits = {k: v for k, v in (rules.get("limits") or {}).items()}
    for key in ("commands", "cells"):
        if key in limits and used_before[key] >= limits[key] and moves:
            broke("the adults' limits hold (%s)" % key, [used_before[key], limits[key], moves])
    if "moves" in limits and used_before["moves"] >= limits["moves"] and any(
            not (method == "turn" and not args["counted"]) for _ in moves):
        broke("the adults' limits hold (moves)", [used_before["moves"], limits["moves"], moves])
    acting = [l for l in wire if l.split()[0] in ("MOVE", "TURN", "HOLD", "STOP", "LED")]
    if used_after["commands"] - used_before["commands"] != len(acting):
        broke("every command that acts is counted, and only those", [acting, used_before["commands"],
                                                                    used_after["commands"]])
    if moves and result["result"] == "stopped" and result.get("why") == "timeout" and "STOP" not in wire:
        broke("a move that took too long is stopped (STOP)", wire)
    decisions = [e["decision"] for e in journal]
    if result["result"] == "refused" and result.get("rule") not in ("robot", "one_move_at_a_time", "another_command") \
            and "refused" not in decisions:
        broke("every refusal of the Referee is in its journal", [result, journal])
    if moves and not {"sent", "refused by the robot"} & set(decisions):
        broke("every command sent is in its journal", [moves, journal])
    never_moved = result["result"] == "refused" and result.get("rule") == "robot" or (
        result["result"] == "done" and method == "slide" and result.get("cells_done") == 0)
    if moves and never_moved and used_after["moves"] != used_before["moves"]:
        broke("a move the robot refused or never made is not counted", [result, used_before["moves"],
                                                                         used_after["moves"]])


# --- one run ------------------------------------------------------------------------------------------------------------------
def run(seed, generator, persona, steps, replay=None, module=referee_module, examples=False):
    rng, pace_rng = random.Random(seed), random.Random(seed * 31 + 7)
    world = random.Random(seed * 7919 + 1)
    for f in STATE.iterdir():
        if f.is_file():
            f.unlink()
    rules = dict(RULES)
    if world.random() < 0.3:
        rules["limits"] = {world.choice(["moves", "commands", "cells"]): world.randint(1, 8)}
    broken, sent, answers, history = [], [], [], []
    robot = Robot(world, 0.03 if pace_rng.random() < 0.15 else 0, broken)
    referee = module.Referee(MAT, rules)
    referee.link = robot
    terrain = Terrain(world)
    error = None
    for n in range(steps):
        if replay is not None:
            if n >= len(replay["sent"]):
                break
            line = replay["sent"][n]
        elif generator == "random":
            line = random_line(rng, persona)
        else:
            try:
                line = probe_tiers.ollama_line(generator[len("ollama:"):], persona, history, seed * 1000 + n,
                                               system=PERSONAS[persona] + "\n\nThe language:\n\n" + LANGUAGE
                                               + ("\n\n" + EXAMPLES if examples else ""))
            except (OSError, ValueError, KeyError) as e:
                error = f"model: {type(e).__name__}: {e}"
                break
        sent.append(line)
        history.append("> " + line)
        terrain.draw()
        call = parse(line)
        if call is None:
            answers.append("not the language")
            history.append("< not understood: not a call of the language")
            continue
        method, args = call
        robot.wire = []
        journal_before = len(_journal())
        used_before = common.used()
        hold = (lambda: True) if method == "slide" and pace_rng.random() < 0.2 else None
        try:
            if method == "step":
                result = referee.door(True, lambda: referee.step(terrain, args["target"], args["card_ahead"]))
            elif method == "step_blind":
                result = referee.door(True, lambda: referee.step_blind(terrain, args["target"], args["card_ahead"],
                                                                       args["dist"], MAT["cell_cm"] + 8))
            elif method == "slide":
                result = referee.door(True, lambda: referee.slide(terrain, args["way"], args["cells"], args["until"],
                                                                  args["asked"], hold=hold))
            elif method == "turn":
                result = referee.door(True, lambda: referee.turn(terrain, args["way"], args["counted"]))
            elif method == "light":
                result = referee.door(True, lambda: referee.light(args["color"]))
            elif method == "state":
                result = {"result": "done", "state": referee.door(False, referee.state)}
            else:
                result = {"result": "done", "events": referee.door(False, referee.events)[0]}
        except RobotNotConnected as e:
            result = {"result": "error", "why": str(e)}
        except Exception as e:  # noqa: BLE001
            broken.append({"invariant": "no error escapes a call", "detail": repr(e)})
            result = {"result": "error"}
        answers.append(result.get("result") + (" " + str(result.get("rule") or result.get("why") or "")).rstrip())
        history.append("< " + json.dumps(result)[:300])
        if result.get("result") != "error":
            check_call(method, args, result, robot.wire, terrain, rules, used_before, common.used(),
                       _journal()[journal_before:], broken)
        for b in broken:
            b.setdefault("line", line)
    return {"seed": seed, "sent": sent, "answers": answers, "broken": broken, "error": error, "rules": rules}


def _journal():
    path = STATE / "referee.jsonl"
    return [json.loads(l) for l in path.read_text().splitlines()] if path.exists() else []


def summary(results, generator, persona):
    import collections
    lines = [l for r in results for l in r["sent"]]
    words = collections.Counter(a.split()[0] if not a.startswith("not") else "not the language"
                                for r in results for a in r["answers"])
    codes = collections.Counter(a for r in results for a in r["answers"])
    return {"tier": "referee", "generator": generator, "persona": persona, "runs": len(results), "lines_sent": len(lines),
            **probe_tiers.variety(lines, EXAMPLES),
            "answers": dict(words), "accepted_share": round(words["done"] / len(lines), 3) if lines else None,
            "not_understood_share": round(words["not the language"] / len(lines), 3) if lines else None,
            "coverage": dict(sorted(codes.items())),
            "commonest_not_understood": collections.Counter(l for r in results for l, a in zip(r["sent"], r["answers"])
                                                            if a == "not the language").most_common(8),
            "runs_breaking_an_invariant": sum(1 for r in results if r["broken"]),
            "invariants_broken": dict(collections.Counter(b["invariant"] for r in results for b in r["broken"])),
            "model_errors": [r["error"] for r in results if r["error"]]}


# --- the probe checks itself ---------------------------------------------------------------------------------------------------
def mutants():
    def no_card_step(m):
        step = m.Referee.step
        def patched(self, terrain, target, card_ahead):
            saved = self.rules["never_onto_a_card"]
            self.rules = dict(self.rules, never_onto_a_card=False)
            try:
                return step(self, terrain, target, False)
            finally:
                self.rules = dict(self.rules, never_onto_a_card=saved)
        m.Referee.step = patched
    def no_card_slide(m):
        slide = m.Referee.slide
        def patched(self, terrain, way, cells, until, asked, hold=None):
            class Blind:
                def __getattr__(_, name): return getattr(terrain, name)
                def card_cells(_, caution=False): return set()
            return slide(self, Blind(), way, cells, until, asked, hold)
        m.Referee.slide = patched
    def no_blind(m): m.Referee.blind_way = lambda self, terrain, cells: None
    def no_ball(m): m.Referee.ball_near = lambda self, terrain, cells, forward=False: None
    def no_limits(m): m.Referee.budget = lambda self, counted: None
    def no_card_until(m):
        slide = m.Referee.slide
        def patched(self, terrain, way, cells, until, asked, hold=None):
            send = self.link.request
            def without_card(line):
                head, _, until = line.partition(" UNTIL ")
                kept = [c for c in until.split(",") if c and c != "CARD"]
                return head + (" UNTIL " + ",".join(kept) if kept else "")
            self.link.request = lambda line, timeout=1.0: send(without_card(line), timeout)
            try:
                return slide(self, terrain, way, cells, until, asked, hold)
            finally:
                self.link.request = send
        m.Referee.slide = patched
    def no_stop(m):
        watch = m.Referee.watch
        def patched(self, *a, **k):
            request = self.link.request
            self.link.request = lambda line, timeout=1.0: "OK STOP" if line == "STOP" else request(line, timeout)
            try:
                return watch(self, *a, **k)
            finally:
                self.link.request = request
        m.Referee.watch = patched
    def uncounted(m):
        count = m.common.count
        m.common.count = lambda name, n=1: None if name == "commands" else count(name, n)
    def no_journal(m):
        refuse = m.Referee.refuse
        def patched(self, command, rule, **details):
            return dict({"result": "refused", "rule": rule}, **details)
        m.Referee.refuse = patched
    def whole_slide(m):
        slide = m.Referee.slide
        def patched(self, terrain, way, cells, until, asked, hold=None):
            class Blind:
                def __getattr__(_, name): return getattr(terrain, name)
                def card_cells(_, caution=False): return set()
                def seen_free_s_ago(_, cell): return 0
                def on_lane(_, cell): return False
                def ball_sighting(_): return None
            return slide(self, Blind(), way, cells, until, asked, hold)
        m.Referee.slide = patched
    return {"steps onto a card": no_card_step, "slides onto a card": no_card_slide, "blind into any cell": no_blind,
            "balls ignored": no_ball, "limits ignored": no_limits, "forward slide without UNTIL CARD": no_card_until,
            "no STOP after a timeout": no_stop, "commands not counted": uncounted, "refusals not in the journal": no_journal,
            "a slide with no rule": whole_slide}


def load_fresh():
    return probe_tiers.load("referee_mutant", GUIDE / "stack" / "referee.py")


def main(args):
    if args.mutants:
        caught = {}
        for name, patch in mutants().items():
            module = load_fresh()
            saved = (module.common.count,)
            patch(module)
            hits = 0
            for k in range(args.runs):
                hits += bool(run(1 + k, "random", "careful" if k % 2 else "careless", args.steps, module=module)["broken"])
            module.common.count = saved[0]
            caught[name] = hits
        print(json.dumps({"tier": "referee", "runs_per_mutant": args.runs, "runs_that_caught_it": caught,
                          "missed": [k for k, v in caught.items() if not v]}, indent=1))
        return 1 if any(not v for v in caught.values()) else 0
    if args.replay:
        saved = json.loads(Path(args.replay).read_text(encoding="utf-8"))
        again = run(saved["seed"], "replay", saved.get("persona", "careful"), len(saved["sent"]), replay=saved)
        print(json.dumps({"broken": again["broken"],
                          "same_as_saved": [b["invariant"] for b in again["broken"]] ==
                          [b["invariant"] for b in saved["broken"]]}, indent=1))
        return 1 if again["broken"] else 0
    out = Path(args.out or GUIDE / ".robot-sim" / "probes" / time.strftime("referee-%Y%m%d-%H%M%S"))
    out.mkdir(parents=True, exist_ok=True)
    results = []
    for k in range(args.runs):
        result = run(args.seed + k, args.generator, args.persona, args.steps, examples=getattr(args, "examples", False))
        results.append(result)
        if result["broken"]:
            (out / f"violation-{result['seed']}.json").write_text(json.dumps(dict(result, persona=args.persona),
                                                                              indent=1) + "\n", encoding="utf-8")
    report = dict(summary(results, args.generator, args.persona), examples=getattr(args, "examples", False),
                  first_seed=args.seed)
    (out / "summary.json").write_text(json.dumps(report, indent=1) + "\n", encoding="utf-8")
    (out / "runs.jsonl").write_text("".join(json.dumps({k: r[k] for k in ("seed", "sent", "answers", "error")}) + "\n"
                                            for r in results), encoding="utf-8")
    print(json.dumps(report, indent=1))
    shutil.rmtree(STATE, ignore_errors=True)
    return 0
