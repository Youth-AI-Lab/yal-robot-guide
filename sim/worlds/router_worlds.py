"""The worlds of the router's data (facilitator/ROBOT-STRATEGE.md, section 10): five worlds, four kinds of mission,
twenty variants in sim/worlds/router/, each with its mat, its mission and its truth.

The five worlds: forest (the guide's), balls, wall and recess (worlds of earlier benches, copied unchanged), crates
(new). Each variant keeps its world's cards and boxes and adds only what its kind needs:
- reach: stop in front of the barbecue (every world has one), with no ball (the balls kind is the same mission with
  balls: the difference between the two is what the balls cost);
- absent: find the picnic, a card the camera knows (the mat lists it) that is on no world;
- survey: find every dry grass card; the variant has three (dry grass added on free cells when the world has fewer);
- balls: stop in front of the barbecue with at least one ball rolling (a ball added on a free line when the world has
  none).
Every variant is played with --ball-phase random: the seed draws where each ball starts.

The truth, written by code from the world, never by a model: the cells of each card, the cells the robot can stand on
and reach from its start (a box that covers the middle of a cell, where the robot stands, blocks it; the robot never
drives onto a card), the cells next to each card of the mission it can reach, the lines the balls roll on, and the
SHA-256 of the world, the mat and the mission.

usage (the folder of the guide is the current folder):
    python3 -I sim/worlds/router_worlds.py           write sim/worlds/router/
    python3 -I sim/worlds/router_worlds.py --check   the files are what this code writes (nothing written)
"""
import argparse
import hashlib
import json
import random
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
GUIDE = HERE.parent.parent
OUT = HERE / "router"
ROBOT_RADIUS_CM = 10  # sim/robot_sim.py
# The worlds, and where the robot starts on each (the balls world of the benches starts at the fire station, H1).
WORLDS = {
    "forest": {"start": {"cell": "A6", "facing": "north"}},
    "balls": {"start": {"cell": "H1", "facing": "south"}},
    "wall": {"start": {"cell": "A6", "facing": "north"}},
    "recess": {"start": {"cell": "A6", "facing": "north"}},
    "crates": {"start": {"cell": "A6", "facing": "north"}},
}
# The ball of the balls kind, for the worlds that have none: a line that crosses the way to the barbecue, away from
# the start's row and column and from every box (checked below).
ADDED_BALL = {
    "wall": {"from": [165, 15], "to": [165, 165], "radius": 7, "speed_cm_s": 8},   # column F
    "recess": {"from": [15, 105], "to": [225, 105], "radius": 7, "speed_cm_s": 8},  # row 4
}
KINDS = {
    "reach": {"steps": [{"do": "reach_card", "card": "barbecue"}]},
    "absent": {"steps": [{"do": "find_card", "card": "picnic"}]},
    "survey": {"steps": [{"do": "survey", "card": "dry-grass"}]},
    "balls": {"steps": [{"do": "reach_card", "card": "barbecue"}]},
}
DRY_GRASS = 3


def cell_name(col, row):
    return chr(ord("A") + col) + str(row + 1)


def cell_xy(name):
    return ord(name[0]) - ord("A"), int(name[1:]) - 1


def blocked_cells(world, mat):
    """The cells a box covers where the robot would stand: the middle square of the robot's size."""
    cm, out = mat["cell_cm"], set()
    for c in range(mat["cols"]):
        for r in range(mat["rows"]):
            x0, y0 = c * cm + cm / 2 - ROBOT_RADIUS_CM, r * cm + cm / 2 - ROBOT_RADIUS_CM
            x1, y1 = x0 + 2 * ROBOT_RADIUS_CM, y0 + 2 * ROBOT_RADIUS_CM
            for b in world["boxes"]:
                if b["x"] < x1 and b["x"] + b["w"] > x0 and b["y"] < y1 and b["y"] + b["h"] > y0:
                    out.add(cell_name(c, r))
    return out


def ball_cells(ball, mat):
    """The cells a ball's line goes through (a row or a column)."""
    cm = mat["cell_cm"]
    (x0, y0), (x1, y1) = ball["from"], ball["to"]
    if y0 == y1:
        r = int(y0 // cm)
        return {cell_name(c, r) for c in range(int(min(x0, x1) // cm), int(max(x0, x1) // cm) + 1)}
    c = int(x0 // cm)
    return {cell_name(c, r) for r in range(int(min(y0, y1) // cm), int(max(y0, y1) // cm) + 1)}


def reachable(world, mat, start):
    """The cells the robot can stand on and reach from its start, never through a box or onto a card."""
    blocked = blocked_cells(world, mat)
    cards = {c["cell"] for c in world["cards"]}
    seen, todo = {start}, [start]
    while todo:
        col, row = cell_xy(todo.pop())
        for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            c, r = col + dc, row + dr
            name = cell_name(c, r)
            if 0 <= c < mat["cols"] and 0 <= r < mat["rows"] and name not in seen and name not in blocked \
                    and name not in cards:
                seen.add(name)
                todo.append(name)
    return seen


def neighbours(cell, mat):
    col, row = cell_xy(cell)
    return {cell_name(col + dc, row + dr) for dc, dr in ((1, 0), (-1, 0), (0, 1), (0, -1))
            if 0 <= col + dc < mat["cols"] and 0 <= row + dr < mat["rows"]}


def variant(name, kind, mat_base):
    world = json.loads((HERE / f"{name}.json").read_text(encoding="utf-8"))
    world = {k: world[k] for k in ("cards", "boxes", "balls")}
    mat = dict(mat_base, start=WORLDS[name]["start"])
    mat["cards"] = dict(mat_base["cards"], picnic="a picnic area")
    start = mat["start"]["cell"]
    about = f"{name} for the router's data, kind {kind}: written by sim/worlds/router_worlds.py."
    if kind == "survey":
        have = [c for c in world["cards"] if c["label"] == "dry-grass"]
        lanes = set().union(*(ball_cells(b, mat) for b in world["balls"])) if world["balls"] else set()
        rng = random.Random(f"{name}:survey")
        added = []
        while len(have) + len(added) < DRY_GRASS:
            free = sorted(c for c in reachable(world, mat, start) - {start} - neighbours(start, mat) - lanes)
            rng.shuffle(free)
            for cell in free:
                trial = dict(world, cards=world["cards"] + [{"cell": cell, "label": "dry-grass"}])
                # Every card still has a cell next to it that the robot can reach, and the reachable cells stay one
                # piece but for the new card.
                if reachable(trial, mat, start) == reachable(world, mat, start) - {cell} and all(
                        neighbours(c["cell"], mat) & reachable(trial, mat, start) for c in trial["cards"]
                        if c["cell"] != start):
                    world, added = trial, added + [cell]
                    break
            else:
                raise SystemExit(f"{name}: no free cell left for dry grass")
        about += f" Dry grass added on {', '.join(added) or 'no cell'}."
    if kind == "reach" and world["balls"]:
        world = dict(world, balls=[])
        about += " Its balls removed."
    if kind == "balls" and not world["balls"]:
        ball = ADDED_BALL[name]
        crossed = ball_cells(ball, mat)
        if crossed & blocked_cells(world, mat) or start in crossed:
            raise SystemExit(f"{name}: the added ball crosses a box or the start")
        world = dict(world, balls=[ball])
        about += " A ball added."
    world = {"_about": about, **world}
    mission = dict(KINDS[kind])
    target = mission["steps"][0]["card"]
    cells = sorted(c["cell"] for c in world["cards"] if c["label"] == target)
    reach = reachable(world, mat, start)
    truth = {"world": name, "kind": kind, "mission": mission, "start": mat["start"], "ball_phase": "random",
             "target": target, "target_cells": cells,
             "cards": {label: sorted(c["cell"] for c in world["cards"] if c["label"] == label)
                       for label in sorted({c["label"] for c in world["cards"]})},
             "reachable": sorted(reach),
             "next_to_target": {c: sorted(neighbours(c, mat) & reach) for c in cells},
             "blocked": sorted(blocked_cells(world, mat)),
             "ball_lines": [sorted(ball_cells(b, mat)) for b in world["balls"]]}
    return world, mat, mission, truth


def judge(truth, result):
    """Right (True), wrong (False) or no answer (None), against the truth of the variant: result is the line of
    sim/headless.py (its report, robot_cell, robot_heading, not_looked_at_yet).
    - reach and balls: done, the robot on a cell next to the barbecue and facing it;
    - survey: done, and the dry grass named is exactly the dry grass of the world;
    - absent: impossible, once the camera has looked at every cell but the cells under a box; before that, a give-up;
    - done when the card is not there, or "not on the mat" when it is: wrong; any other impossible (not seen yet, no
      way there, the robot waits for an adult, a limit of the adults), no report, a stop, a limit, a cut: no answer.
    """
    report = result.get("report") or {}
    outcome, cells = report.get("outcome"), set(truth["target_cells"])
    said = {c.get("cell") for c in report.get("cards") or [] if c.get("card") == truth["target"]}
    if outcome == "done":
        if not cells:
            return False
        if truth["kind"] in ("reach", "balls"):
            here, facing = result.get("robot_cell"), result.get("robot_heading")
            if here is None or facing is None:
                return False
            col, row = cell_xy(here)
            dc, dr = {0: (0, -1), 90: (1, 0), 180: (0, 1), 270: (-1, 0)}[round(facing / 90) % 4 * 90]
            return cell_name(col + dc, row + dr) in cells
        if truth["kind"] == "survey":
            return said == cells
        return bool(said) and said <= cells
    if outcome == "impossible":
        if cells:
            return False if report.get("why") == "not on the mat" else None
        left = result.get("not_looked_at_yet")
        return True if left is not None and set(left) <= set(truth["blocked"]) else None
    return None


def text(data):
    return json.dumps(data, indent=1, ensure_ascii=False) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--check", action="store_true", help="compare with the files, write nothing")
    args = parser.parse_args()
    mat_base = json.loads((GUIDE / "mat.json").read_text(encoding="utf-8"))
    files = {}
    for name in WORLDS:
        for kind in KINDS:
            world, mat, mission, truth = variant(name, kind, mat_base)
            folder = f"{name}-{kind}"
            parts = {"world.json": text(world), "mat.json": text(mat), "mission.json": text(mission)}
            truth["sha256"] = {k: hashlib.sha256(v.encode()).hexdigest() for k, v in parts.items()}
            parts["truth.json"] = text(truth)
            for k, v in parts.items():
                files[OUT / folder / k] = v
            if kind in ("reach", "balls") and not all(truth["next_to_target"].values()):
                raise SystemExit(f"{folder}: the barbecue cannot be reached")
    if args.check:
        bad = [str(p.relative_to(GUIDE)) for p, v in files.items() if not p.exists() or p.read_text() != v]
        extra = [str(p.relative_to(GUIDE)) for p in OUT.rglob("*.json") if p not in files]
        if bad or extra:
            sys.exit("not what router_worlds.py writes: " + ", ".join(bad + extra))
        print(f"{len(files) // 4} variants, as written by router_worlds.py")
        return
    for p, v in files.items():
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(v, encoding="utf-8")
    print(f"{len(files) // 4} variants in {OUT.relative_to(GUIDE)}/")


if __name__ == "__main__":
    main()
