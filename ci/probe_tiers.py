"""Probes one tier of the robot alone, with inputs written by a generator that only knows the description of the
tier's language: a small local model (through Ollama), or a random generator with no model.

Unit tests, the simulator's checks and the missions use inputs we wrote. Here a generator plays the tier above and
writes the inputs, many times over, with different seeds; the tier runs alone, with fakes below it. What is measured:
  - how generable the language is: the share of written lines the tier accepts, and the most frequent mistakes;
  - how the tier behaves: invariants checked after every step, whatever came in. A run that breaks one keeps its
    sequence (lines and world), replayable without the model (--replay), to become a fixed check;
  - what the inputs made the tier do (coverage): a share of accepted lines says little if the robot never moved.
The probe checks itself (--mutants): each mutant is the firmware with one safety rule removed on purpose; the probe
must catch every one, or it is the probe that is wrong.

Today one tier: the Reflex (robot/firmware.py with the reflex.py of the guide), and its language, the protocol in
the head of robot/firmware.py. The generator plays the Referee; while a move runs, the probe sends PING as the Referee
does (not in every run, so that the watchdog is tried too). A fake board below (no simulator) draws world events at
random: touches on the bumper (short, or held), presses on the buttons, an adult who presses Resume when the robot
waits, a program late, a distance sensor that stops or fails for a while, images of the camera (cards, a ball, the
side of a box, some late), a battery that runs down in some runs.

Usage (the folder of the guide is the current folder):
    python3 -I ci/probe_tiers.py --generator random --runs 200
    python3 -I ci/probe_tiers.py --generator ollama:qwen3:8b --runs 5 --persona careful
    python3 -I ci/probe_tiers.py --mutants
    python3 -I ci/probe_tiers.py --replay <folder>/violation-<seed>.json
Only the standard library; the model is reached at OLLAMA_URL (default http://127.0.0.1:11434). Results in --out
(default .robot-sim/probes/<time>): summary.json, runs.jsonl, and one file per run that broke an invariant.
"""
import argparse
import collections
import importlib.util
import json
import os
import random
import re
import sys
import time
import urllib.request
from pathlib import Path

GUIDE = Path(__file__).resolve().parent.parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


FIRMWARE = load("firmware_probe", GUIDE / "robot" / "firmware.py")
REFLEX = load("reflex_probe", GUIDE / "robot" / "reflex.py")
# The language as the generator reads it: the head of robot/firmware.py, up to what the board gives.
LANGUAGE = FIRMWARE.__doc__[:FIRMWARE.__doc__.index("What the program needs")].strip()
# The limits the invariants hold the firmware to: its own, and the reflex.py of the guide (never less careful).
TOP = min(REFLEX.MAX_SPEED_CM_S, FIRMWARE.HARD_MAX_SPEED_CM_S)
BLIND = min(TOP, FIRMWARE.HARD_BLIND_SPEED_CM_S)
MAX_AGE = min(getattr(REFLEX, "DISTANCE_MAX_AGE_MS", FIRMWARE.HARD_DISTANCE_MAX_AGE_MS),
              FIRMWARE.HARD_DISTANCE_MAX_AGE_MS)

PERSONAS = {
    "careful": "You are the Referee of a small robot: the computer that sends it one command per line. Send commands "
               "that drive it around safely and read its state. Follow the protocol exactly.",
    "careless": "You are a careless or hostile caller of a small robot. Send commands that test its limits: wrong "
                "words, wrong numbers, commands at the wrong time, a move while it moves, nonsense. Keep each to one "
                "line.",
}

# --- the fake board -------------------------------------------------------------------------------------------------
CARDS = ["fire-station", "dry-grass", "barbecue", "water-point"]


class Board:
    """The board under the Reflex, written by hand: what the firmware asks of the wheels (asked) and what they do
    (speeds: nothing while the motors are cut), the odometry, and world events drawn from a seeded random generator
    (the same seed, the same world)."""

    def __init__(self, rng):
        self.rng, self.t, self.out, self.inbox = rng, 0, [], []
        self.asked, self.speeds, self.cut, self.odo = (0, 0, 0), (0, 0, 0), False, [0.0, 0.0]
        self.dist, self.dist_stopped_until, self.dist_failing_until = (100, 0), -1, -1
        self.frame, self.frame_no, self.view = (None, None, []), 0, None
        self.bump_n, self.press_n, self.held, self.touch_until = 0, {"stop": 0, "resume": 0}, {}, -1
        self.delivered_at = 0
        self.drain = 0.0 if rng.random() < 0.8 else rng.uniform(0.02, 0.05)  # percent per tick: low in 15 to 35 s

    def ticks_ms(self): return self.t
    def ticks_diff(self, new, old): return new - old
    def write_line(self, text): self.out.append(text)
    def odometry(self): return tuple(self.odo)
    def battery(self): return max(0.0, 90 - self.drain * self.t / 10)
    def compass(self): return 0.0
    def led(self, color): pass
    def bumper(self): return self.touch_until > self.t
    def bumps(self): return self.bump_n
    def presses(self, name): return self.press_n[name]
    def button(self, name): return self.held.get(name, -1) > self.t
    def camera_view(self): return self.frame

    def read_lines(self):
        lines, self.inbox = self.inbox, []
        if lines:
            self.delivered_at = self.t
        return lines

    def drive(self, forward, right, turn):
        self.asked = (forward, right, turn)
        if not self.cut:
            self.speeds = self.asked

    def release(self, bumps):
        if bumps == self.bump_n and not self.button("stop"):
            self.cut = False

    def distance_cm(self):
        if self.dist_failing_until > self.t:
            raise OSError("VL53L1X data ready timeout")
        return self.dist

    def press(self, name):
        self.press_n[name] += 1
        self.held[name] = self.t + 200
        if name == "stop":
            self.cut, self.speeds = True, (0, 0, 0)

    def world(self, fw, events):
        """One tick of the world: maybe an event (noted, for a replay), then the measures."""
        rng, event = self.rng, None
        r = rng.random()
        waiting = fw.estop or fw.wait_adult or fw.fault
        if waiting and r < 0.01:
            event = "adult:resume"  # an adult comes, puts the robot back, presses Resume
            self.press("resume")
        elif r < 0.0015:
            held = rng.choice([0, 0, 0, 200, 500])
            event = f"touch:{held}"
            self.bump_n += 1
            self.touch_until = self.t + held
            self.cut, self.speeds = True, (0, 0, 0)
        elif r < 0.0018:
            event = "press:stop"
            self.press("stop")
        elif r < 0.0020:
            event = "press:resume"
            self.press("resume")
        elif r < 0.0025:
            late = rng.choice([60, 150, 400])
            event = f"late:{late}"
            self.t += late
        elif r < 0.0029:
            span = rng.randint(100, 2000)
            event = f"distance:stops:{span}"
            self.dist_stopped_until = self.t + span
        elif r < 0.0030:
            span = rng.randint(30, 600)
            event = f"distance:fails:{span}"
            self.dist_failing_until = self.t + span
        if event:
            events.append([self.t, event])
        if self.dist_stopped_until <= self.t:
            if self.t % 500 == 0 or self.dist[0] is None and self.t % 100 == 0:
                self.dist = (rng.choice([None, 6, 12, 40, 100, 200]), self.t)
            else:
                self.dist = (self.dist[0], self.t)
        if self.t % 50 == 0:
            # An image every 50 ms; what it shows changes now and then (a card moved, the robot turned): cards,
            # sometimes a ball, sometimes the side of a box; some images come late (taken 1.5 s before).
            if self.view is None or rng.random() < 0.02:
                self.view = [[c, rng.randint(-20, 20), rng.randint(25, 110)] for c in CARDS if rng.random() < 0.3]
                if rng.random() < 0.2:
                    self.view.append(["ball", rng.randint(-25, 25), rng.randint(20, 100)])
                if rng.random() < 0.2:
                    a, cm = rng.randint(-25, 10), rng.randint(30, 90)
                    self.view += [["box", a + k, cm] for k in range(0, 12, 3)]
            self.frame_no += 1
            self.frame = (self.frame_no, self.t - rng.choice([0, 0, 0, 1500]), [list(v) for v in self.view])


# --- the invariants ---------------------------------------------------------------------------------------------------
ANSWER = re.compile(r"^(OK (HELLO robot \d+|PONG|STATE( \w+=\S*)+|EVENTS now=\d+ next=\d+( \d+:\d+:\S+)*"
                    r"|MOVE [FBLR]( \d+)?|TURN [LR]|LED \w+|STOP|HOLD)|ERR \w+)$")
EVENT = re.compile(r"^EVT (DONE [FLR]|DONE [FBLR] \d+ \w+|CELL \d+ \d+|STOP \w+|REFUSED \w+|BUMP|NEAR|RESUMED"
                   r"|FAULT:(reflex|error)|SEEN:[\w-]+:-?\d+:\d+:\d+|BALL:\w+(:-?\d+)+)$")
STATE_FIELDS = {"dist", "card", "battery", "estop", "wait_adult", "moving", "last_stop", "bumper", "warning", "resumes",
                "pose", "slid", "looked", "heading", "fault", "tick_max", "boxes"}
STATE_CLOSED = {"estop": {"0", "1"}, "wait_adult": {"0", "1"}, "moving": {"0", "1"}, "bumper": {"0", "1"},
                "warning": {"0", "1"}, "looked": {"0", "1"}, "fault": {"none", "reflex", "error"}}


class Watch:
    """What a run has said so far, to check what must hold across ticks: the cells of a slide, the events' numbers."""

    def __init__(self):
        self.slide, self.cells, self.events_next = None, 0, 0


def check_tick(fw, board, before, delivered, watch, broken):
    """After one tick: what must always hold, whatever the inputs. before: the state before the tick."""
    def broke(name, detail):
        broken.append({"t": board.t, "invariant": name, "detail": detail})
    new = board.out[before["out"]:]
    f, r, turn = board.asked
    moving = board.asked != (0, 0, 0)
    if (fw.fault or fw.estop or fw.wait_adult) and moving:
        broke("the wheels are asked to stop in error, emergency stop or waiting for an adult", board.asked)
    if f > TOP or abs(turn) > FIRMWARE.TURN_DEG_S:
        broke("never faster than reflex.py allows", board.asked)
    if (r != 0 or f < 0) and max(abs(f), abs(r)) > BLIND:
        broke("slow where the sensor does not look", board.asked)
    touched = board.bump_n != before["bumps"] or board.press_n["stop"] != before["stops"]
    if touched and (moving or fw.motion is not None):
        broke("a touch or the stop button ends the move in that tick", [board.asked, fw.motion])
    if before["driving"] and before["gap"] > FIRMWARE.LATE_TICK_MS and (moving or fw.motion is not None):
        broke("a late tick while the wheels turned stops the move", [before["gap"], board.asked])
    if f > 0 and board.t - board.dist[1] > MAX_AGE:
        broke("never forwards on a distance too old", board.t - board.dist[1])
    if (f > 0 and fw.motion is not None and fw.motion[0] == "F" and board.dist[0] is not None
            and board.t - board.dist[1] <= MAX_AGE):
        left = FIRMWARE.CELL_CM - (board.odo[0] - fw.motion[1])
        if board.dist[0] < left + REFLEX.STOP_DISTANCE_CM:
            broke("never forwards towards something closer than the rest of the cell", [board.dist[0], round(left, 1)])
    if moving and board.t - board.delivered_at > REFLEX.WATCHDOG_MS:
        broke("the wheels stop when nothing came from the computer for WATCHDOG_MS", board.t - board.delivered_at)
    if before["fault"] is None and fw.fault is not None and "EVT FAULT:" + fw.fault not in new:
        broke("an error is said (EVT FAULT)", fw.fault)
    if before["motion"] is None and fw.motion is not None and not any(l.startswith(("OK MOVE", "OK TURN")) for l in new):
        broke("a move starts only with OK MOVE or OK TURN", fw.motion)
    if delivered and sum(1 for l in new if l.startswith(("OK ", "ERR "))) != delivered:
        broke("one answer per command", [delivered, new])
    for line in new:
        if not (ANSWER.match(line) or EVENT.match(line)):
            broke("every line in the protocol", line)
            continue
        if line.startswith("OK STATE"):
            parts = [p.split("=", 1) for p in line.split()[2:]]
            keys = [p[0] for p in parts]
            if set(keys) != STATE_FIELDS or len(keys) != len(set(keys)):
                broke("STATE has its fields, once each", keys)
            for key, value in parts:
                if key in STATE_CLOSED and value not in STATE_CLOSED[key]:
                    broke("STATE values in their closed sets", line)
        elif line.startswith("OK EVENTS"):
            words = line.split()
            nxt = int(words[3].split("=")[1])
            ids = [int(p.split(":")[0]) for p in words[4:]]
            if ids != sorted(set(ids)) or any(i >= nxt for i in ids) or nxt < watch.events_next:
                broke("events numbered in order, before next, next never going back", line)
            watch.events_next = nxt
        elif re.match(r"^OK MOVE [FBLR] \d+$", line):
            watch.slide, watch.cells = line.split()[2], 0
        elif line.startswith("EVT CELL "):
            k = int(line.split()[2])
            if watch.slide is None or k != watch.cells + 1:
                broke("EVT CELL counts 1, 2, … within a slide", [watch.slide, watch.cells, line])
            watch.cells = k
        elif re.match(r"^EVT DONE [FBLR] \d+ \w+$", line):
            d, k = line.split()[2], int(line.split()[3])
            if watch.slide != d or k != watch.cells:
                broke("EVT DONE of a slide gives its way and the cells reached", [watch.slide, watch.cells, line])
            watch.slide = None
        elif line.startswith("EVT STOP ") or line.startswith("EVT REFUSED "):
            watch.slide = None


# --- the generators ---------------------------------------------------------------------------------------------------
WORDS = ["HELLO", "PING", "STATE", "EVENTS 0", "EVENTS 5", "MOVE F", "MOVE F", "MOVE F 2", "MOVE F 3 UNTIL CARD",
         "MOVE R 3 UNTIL CARD", "MOVE L 1", "MOVE B 2", "MOVE B 4 UNTIL BALL,NEW", "MOVE R 2", "TURN L", "TURN R",
         "LED green", "LED off", "STOP", "HOLD"]
JUNK = ["MOVE F 9", "MOVE X 1", "TURN", "LED purple", "EVENTS x", "move f", "MOVE F 2 UNTIL WALL", "", "STATE now",
        "PING PING", "MOVE R 0", "HOLD ON", "jump", "MOVE F 2 UNTIL CARD, BALL"]


def random_line(rng, persona):
    return rng.choice(JUNK if persona == "careless" and rng.random() < 0.5 else WORDS)


def ollama_line(model, persona, history, seed):
    """One line from a small local model: the protocol, then the exchanges as turns of a conversation (what it sent,
    what the robot answered), and "your next command". A first try gave the exchanges as one text with > and <: the
    model copied the robot's answers (qwen3:1.7b, 134 lines in 150 were "OK MOVE F")."""
    url = os.environ.get("OLLAMA_URL", "http://127.0.0.1:11434") + "/api/chat"
    window = history[-40:]
    while window and not window[0].startswith("> "):
        window = window[1:]  # start the window on a command of the caller
    messages = [{"role": "system", "content": PERSONAS[persona] + "\n\nThe protocol:\n\n" + LANGUAGE}]
    first = "The robot is on. Your first command." if len(history) <= 40 else \
        "Earlier exchanges are left out. Your next command."
    turn = []
    for line in window:
        if line.startswith("> "):
            if messages[-1]["role"] == "system":
                messages.append({"role": "user", "content": first})
            elif messages[-1]["role"] == "assistant":
                messages.append({"role": "user", "content": "The robot answered:\n" + "\n".join(turn) if turn else
                                 "The robot answered nothing."})
            turn = []
            messages.append({"role": "assistant", "content": line[2:]})
        else:
            turn.append(line[2:])
    end = "\nYour next command: one line, nothing else."
    if messages[-1]["role"] == "system":
        messages.append({"role": "user", "content": first + end})
    else:
        messages.append({"role": "user", "content": ("The robot answered:\n" + "\n".join(turn) if turn else
                                                     "The robot answered nothing.") + end})
    body = {"model": model, "stream": False, "think": False,
            "options": {"seed": seed, "temperature": 0.8, "num_predict": 40}, "messages": messages}
    request = urllib.request.Request(url, data=json.dumps(body).encode(), headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=120) as r:
        text = json.loads(r.read())["message"]["content"]
    lines = [l.strip().strip("`") for l in text.strip().splitlines() if l.strip().strip("`")]
    return lines[0] if lines else ""


# --- one run ------------------------------------------------------------------------------------------------------------
def run(seed, generator, persona, steps, replay=None, firmware=FIRMWARE, reflex=REFLEX):
    rng = random.Random(seed)  # the lines of the random generator
    pace = random.Random(seed * 31 + 7)  # how long the robot lives between two lines, and the heartbeat
    board = Board(random.Random(seed * 7919 + 1))
    fw = firmware.Firmware(board, reflex)
    heartbeat = pace.random() < 0.9  # most runs send PING while a move runs, as the Referee does
    history, sent, answers, broken, events, watch = [], [], [], [], [], Watch()
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
                line = ollama_line(generator[len("ollama:"):], persona, history, seed * 1000 + n)
            except (OSError, ValueError, KeyError) as e:
                error = f"model: {type(e).__name__}: {e}"
                break
        sent.append(line)
        history.append("> " + line)
        board.inbox.append(line)
        start = len(board.out)
        for k in range(pace.randint(10, 150)):  # 0.1 to 1.5 s of the robot's life before the next line
            if heartbeat and k and k % 30 == 0 and fw.motion is not None:
                board.inbox.append("PING")
            board.t += firmware.TICK_MS
            board.odo[0] += (abs(board.speeds[0]) + abs(board.speeds[1])) * firmware.TICK_MS / 1000
            board.odo[1] += abs(board.speeds[2]) * firmware.TICK_MS / 1000
            before = {"out": len(board.out), "bumps": board.bump_n, "stops": board.press_n["stop"],
                      "driving": fw.driving, "gap": None, "fault": fw.fault, "motion": fw.motion}
            last = fw.last_tick
            board.world(fw, events)
            before["gap"] = board.t - last
            delivered = sum(1 for l in board.inbox if l.strip())
            try:
                fw.tick()
            except Exception as e:  # noqa: BLE001  (what the board would get: a crash of the program)
                broken.append({"t": board.t, "invariant": "no error escapes a tick", "detail": repr(e)})
            if k == 0:
                said = [l for l in board.out[before["out"]:] if l.startswith(("OK ", "ERR "))]
                answers.append(said[0] if said and line.strip() else None)
            check_tick(fw, board, before, delivered, watch, broken)
        history += ["< " + l for l in board.out[start:]]
    return {"seed": seed, "sent": sent, "answers": answers, "out": board.out, "events": events, "broken": broken,
            "error": error}


def summary(results, generator, persona):
    lines = [l for r in results for l in r["sent"]]
    answers = [a for r in results for a in r["answers"]]
    words = collections.Counter(("OK" if a.startswith("OK") else " ".join(a.split()[:2])) if a else "(no answer)"
                                for a in answers)
    out = [l for r in results for l in r["out"]]
    coverage = collections.Counter()
    for l in out:
        if l.startswith("EVT "):
            parts = re.split(r"[ :]", l)
            key = parts[1] + (" " + parts[-1] if parts[1] in ("DONE", "STOP", "REFUSED") and len(parts) > 2 else "")
            coverage[key] += 1
    mistakes = collections.Counter(l for r in results for l, a in zip(r["sent"], r["answers"])
                                   if a and a.startswith("ERR unknown"))
    return {"generator": generator, "persona": persona, "runs": len(results), "lines_sent": len(lines),
            "empty_lines": sum(1 for l in lines if not l.strip()), "answers": dict(words),
            "accepted_share": round(words["OK"] / len(lines), 3) if lines else None,
            "not_understood_share": round(words["ERR unknown"] / len(lines), 3) if lines else None,
            "commonest_not_understood": mistakes.most_common(8),
            "runs_breaking_an_invariant": sum(1 for r in results if r["broken"]),
            "invariants_broken": dict(collections.Counter(b["invariant"] for r in results for b in r["broken"])),
            "model_errors": [r["error"] for r in results if r["error"]], "coverage": dict(sorted(coverage.items()))}


# --- the probe checks itself: one safety rule removed at a time ----------------------------------------------------------
def mutants():
    def late(m, x): m.LATE_TICK_MS = 10 ** 9
    def age(m, x): m.HARD_DISTANCE_MAX_AGE_MS = 10 ** 9
    def watchdog(m, x): x.WATCHDOG_MS = 10 ** 9
    def obstacle(m, x):
        reason = m.Firmware.stop_reason
        m.Firmware.stop_reason = lambda self, kind: (lambda why: None if why == "obstacle" else why)(reason(self, kind))
    def speed(m, x):
        const = m.Firmware.const
        m.Firmware.const = lambda self, name, default=None: m.HARD_MAX_SPEED_CM_S if name == "MAX_SPEED_CM_S" \
            else const(self, name, default)
    def estop(m, x):
        halt = m.Firmware.halt
        m.Firmware.halt = lambda self, why: None if why == "estop" else halt(self, why)
    def cell(m, x):
        say = m.Firmware.say
        m.Firmware.say = lambda self, text: None if text.startswith("EVT CELL") else say(self, text)
    def same_tick(m, x):
        inputs = m.Firmware.inputs
        def patched(self):
            inputs(self)
            self.touched_now = False
        m.Firmware.inputs = patched
    def fault_said(m, x):
        note = m.Firmware.note
        m.Firmware.note = lambda self, name: None if name.startswith("FAULT:") else note(self, name)
    def wheels_on_error(m, x):
        def failed(self, kind, text):
            self.fault = self.fault or kind
        m.Firmware.failed = failed
    return {"no late-tick stop": late, "distance never too old": age, "no watchdog": watchdog,
            "obstacle ignored": obstacle, "reflex.py speed ignored": speed, "emergency stop does not stop": estop,
            "EVT CELL not said": cell, "a move in the tick of a touch": same_tick, "an error not said": fault_said,
            "the wheels go on after an error": wheels_on_error}


def check_mutants(runs, steps):
    caught = {}
    for name, patch in mutants().items():
        firmware = load("firmware_mutant", GUIDE / "robot" / "firmware.py")
        reflex = load("reflex_mutant", GUIDE / "robot" / "reflex.py")
        patch(firmware, reflex)
        hits = 0
        for k in range(runs):
            result = run(1 + k, "random", "careful" if k % 2 else "careless", steps, firmware=firmware, reflex=reflex)
            hits += bool(result["broken"])
        caught[name] = hits
    return caught


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--tier", default="reflex", choices=["reflex"])
    parser.add_argument("--generator", default="random", help="random, or ollama:<model>")
    parser.add_argument("--persona", default="careful", choices=sorted(PERSONAS))
    parser.add_argument("--runs", type=int, default=10)
    parser.add_argument("--steps", type=int, default=30, help="lines sent per run")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--out")
    parser.add_argument("--replay", help="a violation file: the same lines and the same world, no model")
    parser.add_argument("--mutants", action="store_true", help="check the probe: it must catch each mutant")
    args = parser.parse_args()
    if args.generator != "random" and not (args.generator.startswith("ollama:") and len(args.generator) > 7):
        parser.error("--generator is random, or ollama:<model>")
    if args.mutants:
        caught = check_mutants(args.runs, args.steps)
        print(json.dumps({"runs_per_mutant": args.runs, "runs_that_caught_it": caught,
                          "missed": [k for k, v in caught.items() if not v]}, indent=1))
        sys.exit(1 if any(not v for v in caught.values()) else 0)
    if args.replay:
        saved = json.loads(Path(args.replay).read_text(encoding="utf-8"))
        again = run(saved["seed"], "replay", saved.get("persona", "careful"), len(saved["sent"]), replay=saved)
        same = [b["invariant"] for b in again["broken"]] == [b["invariant"] for b in saved["broken"]]
        print(json.dumps({"broken": again["broken"], "same_as_saved": same}, indent=1))
        sys.exit(1 if again["broken"] else 0)
    out = Path(args.out or GUIDE / ".robot-sim" / "probes" / time.strftime("%Y%m%d-%H%M%S"))
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


if __name__ == "__main__":
    main()
