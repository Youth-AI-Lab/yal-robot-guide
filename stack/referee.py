"""The Referee: the only door to the robot. Executive layer, about a tenth of a second per decision.

It judges each command of the Cartographer against the rules (rules.json, written by the adults; the number of
moves of a session in mat.json), refuses, sends what it allows to the Reflex through the link, watches the move
until it ends, keeps the count of moves and the log of its decisions (.robot-sim/referee.jsonl). It never decides
a move and never writes the rules it applies. For its rules about the terrain (a card on this cell? a ball's lane?)
it asks the Cartographer, and it counts every cell an estimate could fall on (caution).

Its language, from the Cartographer (moves of the robot, never directions of the mat):
    step(target)                   one cell forwards (MOVE F); target: the (col, row) of that cell, or None off the mat
    slide(way, cells, until, asked)
                                   way F, B, L or R (forwards, backwards, to its left, to its right); cells: the
                                   (col, row) of the cells on the way that are on the mat; until: CARD, BALL, NEW;
                                   asked: how many cells were asked
    turn(way, counted)             a quarter turn L or R; counted: whether it is a new move of the session
    light(color)                   the robot's light
    state()                        the fields of STATE, as the robot gives them
    events(caller)                 the robot's events this caller has not heard yet
Its answers: {"result": "done", ...}, {"result": "stopped", "why": <code>} (the robot stopped on the way: an adult
must come), or {"result": "refused", "rule": <code>, ...}: the Cartographer says it in words.
To the Reflex: the protocol of robot/firmware.py (protocol 5), one line per command.
"""
import fcntl
import json
import threading

import clock
import common
from common import STATE_DIR, cell_name
from link import Link, RobotNotConnected

HEARTBEAT_S = 0.2


class Referee:
    def __init__(self, mat, rules):
        self.mat = mat
        self.rules = rules
        self.link = Link(STATE_DIR)
        self.lock = threading.Lock()
        self.load()

    # What it keeps between two commands (a new process may answer the next one: a terminal, the page) --------
    def load(self):
        try:
            saved = common.load_json(STATE_DIR / "referee.json")
            self.moves, self.cursors = saved["moves"], saved.get("cursors", {})
        except (OSError, ValueError, KeyError):
            self.moves = 0      # moves of this session
            self.cursors = {}   # the last robot event each caller (agent, terminal, page) was given

    def save(self):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        (STATE_DIR / "referee.json").write_text(json.dumps({"moves": self.moves, "cursors": self.cursors}) + "\n",
                                                 encoding="utf-8")

    def journal(self, command, decision, **details):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        entry = dict({"at": clock.iso(), "caller": common.CALLER,
                      "command": command, "decision": decision}, **details)
        with (STATE_DIR / "referee.jsonl").open("a", encoding="utf-8") as f:
            f.write(json.dumps(entry, ensure_ascii=False) + "\n")

    def moved(self, n):
        """A move counted (n = 1), or given back when the robot never moved (n = -1)."""
        self.moves += n
        common.count("moves", n)

    def limits(self):
        """The adults' optional limits ("limits" in rules.json): moves, commands, cells, per mission."""
        return {k: v for k, v in (self.rules.get("limits") or {}).items() if isinstance(v, int) and not isinstance(v, bool)}

    def moves_left(self):
        """Moves left under the adults' limit, or None when they set none."""
        limit = self.limits().get("moves")
        return None if limit is None else max(0, limit - common.used()["moves"])

    # One command at a time ------------------------------------------------------------------------------------
    def door(self, is_move, body):
        """Run one call of the stack: one command at a time on the cable, whoever sends it (the Strategist, an agent,
        a terminal, the page): two programs reading the same serial port would take each other's answers.

        A move does not queue behind another one: several moves sent at once (parallel tool calls, seen with qwen3
        through OpenCode, 2026-11-05) would run in an order nobody chose. The first one runs, the others are
        refused. A question (state, record, watching) waits for its turn."""
        if not self.lock.acquire(timeout=1.0 if is_move else -1):
            return {"result": "refused", "rule": "one_move_at_a_time"}
        try:
            end = clock.monotonic() + 1.0  # a short wait: watching (wait_for_event) holds the cable briefly
            while True:
                STATE_DIR.mkdir(parents=True, exist_ok=True)
                with (STATE_DIR / "stack.lock").open("w") as lock:
                    try:
                        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    except BlockingIOError:
                        pass
                    else:
                        self.load()
                        try:
                            return body()
                        finally:
                            self.save()
                if clock.monotonic() >= end:
                    return {"result": "refused", "rule": "another_command"}
                clock.sleep(0.05)
        finally:
            self.lock.release()

    # Questions to the robot -----------------------------------------------------------------------------------
    def connect(self):
        self.link.connect()
        self.link.drain()

    def close(self):
        self.link.close()

    def state(self):
        reply = self.link.request("STATE")
        if not reply.startswith("OK STATE"):
            raise RobotNotConnected(f"the robot's answer could not be read ({reply[:40]!r})")
        return dict(part.split("=", 1) for part in reply.split()[2:])

    def events(self):
        """What happened to the robot since this caller last heard, from the robot's own memory: nothing is lost when
        nobody was listening (up to the events the robot keeps). Returns (entries, restarted, lost): entries are
        [age in ms, kind, details]."""
        seen = self.cursors.get(common.CALLER, 0)
        reply = self.link.request(f"EVENTS {seen}")
        if not reply.startswith("OK EVENTS"):
            raise RobotNotConnected(f"the robot's answer could not be read ({reply[:40]!r})")
        parts = reply.split()
        head = dict(p.split("=", 1) for p in parts[2:4])
        now, following = int(head["now"]), int(head["next"])
        restarted = following - 1 < seen  # the robot restarted: its numbers start again
        if restarted:
            seen = 0
            parts = self.link.request("EVENTS 0").split()
        entries = [p.split(":", 2) for p in parts[4:]]
        lost = bool(entries) and int(entries[0][0]) > seen + 1 and seen > 0
        out = []
        for number, ms, name in entries:
            kind, _, details = name.partition(":")
            out.append([now - int(ms), kind, details])
        self.cursors[common.CALLER] = following - 1
        return out, restarted, lost

    def light(self, color):
        common.count("commands")
        reply = self.link.request("LED " + color)
        self.journal(f"LED {color}", "sent")
        return {"result": "done"} if reply.startswith("OK") else {"result": "refused", "rule": "robot", "why": reply[4:]}

    # The rules ------------------------------------------------------------------------------------------------
    def budget(self, counted):
        """A limit of the adults reached: {"rule": "no_moves", "max": n} for the moves (a counted move only), or
        {"rule": "limit", "counter": "commands" | "cells", "max": n}; None when none is reached or set."""
        limits, used = self.limits(), common.used()
        if counted and "moves" in limits and used["moves"] >= limits["moves"]:
            return {"rule": "no_moves", "max": limits["moves"]}
        for counter in ("commands", "cells"):
            if counter in limits and used[counter] >= limits[counter]:
                return {"rule": "limit", "counter": counter, "max": limits[counter]}
        return None

    def over(self, command, counted):
        """The refusal of a move when a limit is reached, or None."""
        reached = self.budget(counted)
        if reached is None:
            return None
        rule = reached.pop("rule")
        return self.refuse(command, rule, **reached)

    def ball_near(self, terrain, cells, forward=False):
        """The first of these cells where a ball seen less than ball_seen_recently_s ago could be in the way, or None.
        Sideways or backwards (the camera does not look there): on a cell where it was seen or next to one. Forwards
        (the camera looks, and the robot stops by itself in front of it): only on a cell where it was seen, or next to
        one when it was coming closer. Refusing forwards near any ball seen just before kept the robot from crossing
        at all (0 out of 3 in the mission without a card, 6 November)."""
        ball = terrain.ball_sighting()
        if ball is None or ball["s_ago"] >= self.rules["ball_seen_recently_s"]:
            return None
        seen, near = set(), set()
        for c, r in ball["cells"]:
            seen.add((c, r))
            near |= {(c, r), (c + 1, r), (c - 1, r), (c, r + 1), (c, r - 1)}
        if forward and not ball["closer"]:
            near = seen
        for cell in cells:
            if tuple(cell) in near:
                return cell
        return None

    def blind_way(self, terrain, cells):
        """Why a move backwards or sideways through these cells is not allowed, or None: its distance sensor and
        camera only look forwards."""
        for cell in cells:
            if terrain.seen_free_s_ago(cell) > self.rules["blind_only_into_cells_seen_free_for_s"]:
                return cell, "not_seen"
            if self.rules["never_into_a_ball_lane_when_blind"] and terrain.on_lane(cell):
                return cell, "lane"
        near = self.ball_near(terrain, cells)
        return (near, "ball") if near else None

    def refuse(self, command, rule, **details):
        self.journal(command, "refused", rule=rule, **{k: v for k, v in details.items() if k != "what"})
        return dict({"result": "refused", "rule": rule}, **details)

    # The moves ------------------------------------------------------------------------------------------------
    def step(self, terrain, target, card_ahead):
        """One cell forwards."""
        refused = self.over("MOVE F", True)
        if refused:
            return refused
        if target is None:
            return self.refuse("MOVE F", "off_mat")
        if self.rules["never_onto_a_card"] and (card_ahead or cell_name(*target) in terrain.card_cells(caution=False)):
            return self.refuse("MOVE F", "card", cell=cell_name(*target))
        near = self.ball_near(terrain, [target], forward=True)
        if near:
            return self.refuse("MOVE F", "ball_forward", cell=cell_name(*near))
        terrain.moving([target], counted=True)
        self.moved(1)
        return self.watch("MOVE F", "F", terrain, counted=True)

    def step_blind(self, terrain, target, card_ahead, dist, need_cm):
        """One cell forwards while the Cartographer does not know where the robot is on the mat: only when the distance
        sensor sees the way free beyond that cell (so it is on the mat, and nothing is in the way), never onto a card in
        front of the camera, never while a ball was seen a moment ago (where it is cannot be known)."""
        refused = self.over("MOVE F", True)
        if refused:
            return refused
        if self.rules["never_onto_a_card"] and card_ahead:
            return self.refuse("MOVE F", "card", cell="ahead")
        ball = terrain.ball_sighting() or {}
        if ball and ball["s_ago"] < self.rules["ball_seen_recently_s"]:
            return self.refuse("MOVE F", "ball_unknown")
        if dist is None or dist < need_cm:
            return self.refuse("MOVE F", "unknown_way", seen_cm=dist)
        terrain.moving([target], counted=True)
        self.moved(1)
        return self.watch("MOVE F", "F", terrain, counted=True)

    def turn(self, terrain, way, counted):
        refused = self.over("TURN " + way, counted)
        if refused:
            return refused
        terrain.moving([], counted=counted)
        self.moved(1 if counted else 0)
        return self.watch("TURN " + way, way, terrain, counted=counted)

    def slide(self, terrain, way, cells, until, asked, hold=None):
        """Slide without turning: one move however many cells, stopping on a cell. A card on the way shortens it (the
        robot never drives onto a card it knows): forwards before any card placed for sure, sideways or backwards
        before any card the camera placed, even from far."""
        refused = self.over(f"MOVE {way} {asked}", True)
        if refused:
            return refused
        if not cells:
            return self.refuse(f"MOVE {way} {asked}", "off_mat")
        card_stop = None
        if self.rules["never_onto_a_card"]:
            cards = terrain.card_cells(caution=way != "F")
            for k, cell in enumerate(cells):
                if cell_name(*cell) in cards:
                    card_stop, cells = cell_name(*cell), cells[:k]
                    break
        if not cells:
            return self.refuse(f"MOVE {way} {asked}", "card", cell=card_stop)
        if way == "F":
            near = self.ball_near(terrain, cells, forward=True)
            if near:
                return self.refuse(f"MOVE {way} {asked}", "ball_forward", cell=cell_name(*near))
        else:
            blocked = self.blind_way(terrain, cells)
            if blocked:
                cell, why = blocked
                return self.refuse(f"MOVE {way} {asked}", "blind_" + why, cell=cell_name(*cell),
                                   free_for_min=self.rules["blind_only_into_cells_seen_free_for_s"] // 60)
        conditions = list(until)
        if way == "F" and "CARD" not in conditions:
            conditions.append("CARD")  # forwards, it always stops in front of a card, never on it
        command = f"MOVE {way} {len(cells)}" + (" UNTIL " + ",".join(conditions) if conditions else "")
        terrain.moving(cells, counted=True)
        self.moved(1)
        result = self.watch(command, way, terrain, counted=True, slide=cells, look=bool(conditions), hold=hold)
        if result["result"] == "done":
            result.update(card_stop=card_stop, edge=len(cells) < asked and card_stop is None)
        return result

    def watch(self, command, kind, terrain, counted, slide=None, look=False, hold=None):
        """Send one command and watch it until it ends, sending PING meanwhile ("I am still here": without it, the
        robot stops, its watchdog). hold: asked at each heartbeat during a slide; when it says yes (the mission is
        paused or stopped), HOLD: the robot stops on the next cell, as soon as possible, with no adult needed."""
        common.count("commands")
        reply = self.link.request(command)
        if reply.startswith("ERR "):
            self.moved(-1 if counted else 0)
            self.journal(command, "refused by the robot", why=reply[4:])
            return {"result": "refused", "rule": "robot", "why": reply[4:], "moved": False}
        self.journal(command, "sent")
        # Sideways: half the speed; with conditions, the robot stands on each cell until its camera has looked.
        allowed = self.mat["move_timeout_s"] * (2 if slide is not None else 1) + (2.5 if look else 0)
        end = clock.monotonic() + allowed
        next_ping = clock.monotonic() + HEARTBEAT_S
        done, held = 0, False
        while clock.monotonic() < end:
            line = self.link.read_line(max(0.0, min(end, next_ping) - clock.monotonic()))
            if clock.monotonic() >= next_ping:
                self.link.send("PING")
                next_ping = clock.monotonic() + HEARTBEAT_S
                if slide is not None and hold is not None and not held and hold():
                    self.link.send("HOLD")
                    common.count("commands")
                    held = True
                    self.journal(command, "hold: stop on the next cell")
            if line is None or line in ("OK PONG", "OK HOLD"):
                continue
            if slide is not None and line.startswith("EVT CELL "):
                k, pose = line.split()[2:4]
                done = int(k)
                common.count("cells")
                terrain.reached(slide[done - 1], pose)
                end = clock.monotonic() + allowed
                continue
            if slide is None and line == "EVT DONE " + kind:
                if kind == "F":
                    common.count("cells")
                return {"result": "done", "cells_done": 1 if kind == "F" else 0}
            if slide is not None and line.startswith(f"EVT DONE {kind} "):
                if done == 0:
                    self.moved(-1)  # it never moved: like a refusal, not counted
                return {"result": "done", "cells_done": done, "why": line.split()[4]}
            if line.startswith("EVT STOP "):
                self.journal(command, "stopped by the robot", why=line[9:])
                return {"result": "stopped", "why": line[9:], "cells_done": done}
            if line.startswith("EVT REFUSED "):
                # It waited, the way stayed blocked, it never moved: still where it was.
                self.moved(-1 if counted else 0)
                return {"result": "refused", "rule": "robot", "why": line[12:], "moved": False, "waited": True}
        try:
            common.count("commands")
            self.link.request("STOP")  # read its OK STOP: left unread, it would be taken for the next answer
        except RobotNotConnected:
            pass
        self.journal(command, "stopped: too long")
        return {"result": "stopped", "why": "timeout", "cells_done": done}
