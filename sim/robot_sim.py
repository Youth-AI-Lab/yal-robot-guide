"""A simulator of the robot, seen from above, to try the whole chain before the robot exists.

The simulator replaces only the physical part: the wheels, the distance sensor, the camera, the
buttons, the battery and the USB cable. The program of the robot (robot/firmware.py and
robot/reflex.py) runs here unchanged, and the gateway (gateway/robot_gateway.py) talks to it
through a pseudo-terminal, as it would talk to the robot through its USB cable. So the gateway and
the agent cannot tell the simulator from the robot.

The camera gives a card label, never an image: this is what the camera of the robot will send.

Usage (the folder of the guide is the current folder):
    python3 -I sim/robot_sim.py            then open http://127.0.0.1:8780 (Ports tab in a Codespace)
    python3 -I sim/robot_sim.py --port 0   any free port, printed on the line "READY port=… serial=…"

Only the standard library. The page listens on 127.0.0.1 only.
"""
import argparse
import collections
import importlib.util
import json
import math
import os
import random
import threading
import time
import tty
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
GUIDE = HERE.parent
STATIC = HERE / "static"
TICK_S = 0.01
# The simulated robot. The real robot's sizes are TO BE COMPLETED.
ROBOT_RADIUS_CM = 10
SENSOR_RANGE_CM = 400  # VL53L1X of the STeaMi board: 0 to 4 m (wiki.steami.cc)
SENSOR_RAYS_DEG = (-5, 0, 5)  # a narrow cone: a choice of the simulator, not a measure
BATTERY_PER_CM = 0.05
BUTTON_PRESS_S = 0.2
STATIC_FILES = {"/": ("index.html", "text/html"), "/sim.js": ("sim.js", "text/javascript"),
                "/sim.css": ("sim.css", "text/css")}


def load_module(name, path):
    """Load robot/firmware.py or robot/reflex.py by its path (python -I does not add folders)."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def cell_xy(name):
    """'A6' -> (column 0, row 5)."""
    return ord(name[0].upper()) - ord("A"), int(name[1:]) - 1


def cell_name(col, row):
    return chr(ord("A") + col) + str(row + 1)


HEADINGS = {"north": 0, "east": 90, "south": 180, "west": 270}


class Link:
    """The USB cable: a pseudo-terminal. The gateway opens its other end like a serial port."""

    def __init__(self, log):
        self.master, self.slave = os.openpty()
        tty.setraw(self.slave)
        os.set_blocking(self.master, False)
        self.path = os.ttyname(self.slave)
        self.plugged = True
        self.buffer = b""
        self.log = log

    def read_lines(self, now):
        try:
            data = os.read(self.master, 4096)
        except (BlockingIOError, OSError):
            data = b""
        if not self.plugged:
            self.buffer = b""
            return []
        self.buffer += data
        lines = []
        while b"\n" in self.buffer:
            line, self.buffer = self.buffer.split(b"\n", 1)
            text = line.decode("ascii", "replace").strip()
            if text:
                lines.append(text)
                self.log.append({"t": round(now, 2), "dir": "in", "line": text})
        return lines

    def write_line(self, text, now):
        if not self.plugged:
            self.log.append({"t": round(now, 2), "dir": "lost", "line": text})
            return
        self.log.append({"t": round(now, 2), "dir": "out", "line": text})
        try:
            os.write(self.master, (text + "\n").encode("ascii"))
        except (BlockingIOError, OSError):
            pass  # nobody reads the other end: the line is lost, as on a real cable


def ray_box(ox, oy, dx, dy, x0, y0, x1, y1):
    """Distance along the ray to the box, or None."""
    tmin, tmax = 0.0, float("inf")
    for o, d, lo, hi in ((ox, dx, x0, x1), (oy, dy, y0, y1)):
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None
            continue
        t1, t2 = (lo - o) / d, (hi - o) / d
        tmin, tmax = max(tmin, min(t1, t2)), min(tmax, max(t1, t2))
    return tmin if tmin <= tmax else None


def circle_box(cx, cy, r, x0, y0, x1, y1):
    nx, ny = min(max(cx, x0), x1), min(max(cy, y0), y1)
    return (cx - nx) ** 2 + (cy - ny) ** 2 < r * r


class World:
    def __init__(self, mat, world, state_dir, seed=None):
        self.state_dir = Path(state_dir)
        self.lock = threading.RLock()
        self.random = random.Random(seed)
        self.wire = collections.deque(maxlen=300)
        self.context = collections.deque(maxlen=100)
        self.link = Link(self.wire)
        self.mode = "realtime"
        self.noise = {"distance_cm": 0.0, "slip": 0.0, "camera_miss": 0.0}
        self.reflex_error = None
        self.load(mat, world)

    # The world ------------------------------------------------------------------------------
    def load(self, mat, world):
        with self.lock:
            self.mat, self.world = mat, world
            self.cell = mat["cell_cm"]
            self.width, self.height = mat["cols"] * self.cell, mat["rows"] * self.cell
            col, row = cell_xy(mat["start"]["cell"])
            self.x, self.y = (col + 0.5) * self.cell, (row + 0.5) * self.cell
            self.heading = float(HEADINGS[mat["start"]["facing"]])
            self.cards = {tuple(cell_xy(c["cell"])): c["label"] for c in world.get("cards", [])}
            self.boxes = [dict(b, id=i) for i, b in enumerate(world.get("boxes", []))]
            self.balls = [dict(b, t=0.0) for b in world.get("balls", [])]
            self.next_box = len(self.boxes)
            self.time = 0.0
            self.speed, self.turn = 0.0, 0.0
            self.odo_cm, self.odo_deg = 0.0, 0.0
            self.battery_level = 100.0
            self.led_color = "off"
            self.buttons = {}
            self.collisions, self.touching = 0, False
            self.move_start = None
            self.unplugged_at = None
            self.link.plugged = True
            self.stops = []
            self.last_command_at = None
            self.firmware_module = load_module("firmware", GUIDE / "robot" / "firmware.py")
            self.reflex = load_module("reflex", GUIDE / "robot" / "reflex.py")
            self.firmware = self.firmware_module.Firmware(Hal(self), self.reflex)
            # The robot is back on its start cell: the gateway must count from there again.
            (self.state_dir / "gateway.json").unlink(missing_ok=True)

    def reload_reflex(self):
        with self.lock:
            try:
                reflex = load_module("reflex", GUIDE / "robot" / "reflex.py")
                for name in ("STOP_DISTANCE_CM", "MAX_SPEED_CM_S", "WATCHDOG_MS", "MIN_BATTERY", "must_stop"):
                    getattr(reflex, name)
            except Exception as e:  # a mistake in the file the participants edit: show it, keep the old one
                self.reflex_error = f"{type(e).__name__}: {e}"
                return False
            self.reflex = reflex
            self.firmware.reflex = reflex
            self.reflex_error = None
            return True

    def obstacles(self):
        """Boxes and balls, as boxes (x0, y0, x1, y1)."""
        out = [(b["x"], b["y"], b["x"] + b["w"], b["y"] + b["h"]) for b in self.boxes]
        for ball in self.balls:
            bx, by = self.ball_xy(ball)
            r = ball["radius"]
            out.append((bx - r, by - r, bx + r, by + r))
        return out

    def ball_xy(self, ball):
        (x0, y0), (x1, y1) = ball["from"], ball["to"]
        length = math.hypot(x1 - x0, y1 - y0) or 1
        s = (ball["t"] * ball["speed_cm_s"]) % (2 * length)
        f = s / length if s <= length else 2 - s / length
        return x0 + (x1 - x0) * f, y0 + (y1 - y0) * f

    # The sensors (what the robot's board would measure) --------------------------------------
    def distance(self):
        if abs(self.turn) > 0:
            return None  # turning on the spot: the firmware does not ask
        best = None
        for offset in SENSOR_RAYS_DEG:
            h = math.radians(self.heading + offset)
            dx, dy = math.sin(h), -math.cos(h)
            hits = [ray_box(self.x, self.y, dx, dy, *box) for box in self.obstacles()]
            # The border of the mat: the simulated mat has a low fence that the sensor sees.
            for t in (-self.x / dx if dx < 0 else (self.width - self.x) / dx if dx > 0 else None,
                      -self.y / dy if dy < 0 else (self.height - self.y) / dy if dy > 0 else None):
                hits.append(t)
            for t in hits:
                if t is not None and t >= 0 and (best is None or t < best):
                    best = t
        if best is None:
            return None
        d = best - ROBOT_RADIUS_CM + self.random.gauss(0, self.noise["distance_cm"])
        return None if d > SENSOR_RANGE_CM else max(0.0, d)

    def card_ahead(self):
        h = math.radians(self.heading)
        x, y = self.x + math.sin(h) * self.cell, self.y - math.cos(h) * self.cell
        label = self.cards.get((int(x // self.cell), int(y // self.cell)))
        if label and self.random.random() < self.noise["camera_miss"]:
            return None
        return label

    # One step of time ------------------------------------------------------------------------
    def step(self, dt):
        with self.lock:
            self.time += dt
            moving_before = self.firmware.motion is not None
            self.firmware.tick()
            moving = self.firmware.motion is not None
            if moving and not moving_before:
                self.move_start = (self.x, self.y, self.heading)
            # In "frozen" mode the world waits while the robot waits (like a game, not like a room).
            if self.mode == "realtime" or moving:
                for ball in self.balls:
                    ball["t"] += dt
            self.move(dt)

    def move(self, dt):
        if self.speed == 0 and self.turn == 0:
            self.touching = False
            return
        real = 1 - self.noise["slip"]
        self.odo_cm += self.speed * dt
        self.odo_deg += self.turn * dt
        self.battery_level = max(0.0, self.battery_level - abs(self.speed * dt) * BATTERY_PER_CM)
        heading = self.heading + self.turn * dt * real
        h = math.radians(heading)
        x = self.x + math.sin(h) * self.speed * dt * real
        y = self.y - math.cos(h) * self.speed * dt * real
        r = ROBOT_RADIUS_CM
        blocked = (x < r or y < r or x > self.width - r or y > self.height - r
                   or any(circle_box(x, y, r, *box) for box in self.obstacles()))
        if blocked:
            if not self.touching:
                self.collisions += 1
            self.touching = True
            self.heading = heading % 360
            return
        self.touching = False
        self.x, self.y, self.heading = x, y, heading % 360

    # The page --------------------------------------------------------------------------------
    def snapshot(self):
        with self.lock:
            fw = self.firmware
            return {
                "time": round(self.time, 2), "mode": self.mode, "noise": self.noise,
                "mat": {"cols": self.mat["cols"], "rows": self.mat["rows"], "cell_cm": self.cell},
                "robot": {"x": round(self.x, 2), "y": round(self.y, 2), "heading": round(self.heading, 2),
                          "radius": ROBOT_RADIUS_CM, "led": self.led_color,
                          "moving": fw.motion is not None, "estop": fw.estop, "wait_adult": fw.wait_adult,
                          "last_stop": fw.last_stop, "battery": round(self.battery_level, 1),
                          "collisions": self.collisions, "cell": self.cell_of(self.x, self.y)},
                "sensors": {"distance_cm": self.rounded(self.distance()), "card_ahead": self.card_ahead()},
                "cards": [{"cell": cell_name(*k), "label": v} for k, v in self.cards.items()],
                "boxes": self.boxes,
                "balls": [{"x": round(bx, 1), "y": round(by, 1), "radius": b["radius"]}
                          for b in self.balls for bx, by in [self.ball_xy(b)]],
                "link": {"plugged": self.link.plugged, "serial": self.link.path},
                "reflex": {k: getattr(self.reflex, k, None)
                           for k in ("STOP_DISTANCE_CM", "MAX_SPEED_CM_S", "WATCHDOG_MS", "MIN_BATTERY")},
                "reflex_error": self.reflex_error,
                "stops": self.stops[-20:],
                "unplugged_at": self.unplugged_at,
                "since_last_command": None if self.last_command_at is None else round(self.time - self.last_command_at, 1),
                "wire": list(self.wire)[-60:],
                "context": list(self.context)[-30:],
            }

    @staticmethod
    def rounded(value):
        return None if value is None else round(value, 1)

    def cell_of(self, x, y):
        col, row = int(x // self.cell), int(y // self.cell)
        if 0 <= col < self.mat["cols"] and 0 <= row < self.mat["rows"]:
            return cell_name(col, row)
        return None

    def action(self, data):
        """What the page (or the checks) can do: everything a person could do around the mat."""
        do = data.get("do")
        with self.lock:
            if do == "reset":
                self.load(self.mat, self.world)
            elif do == "load_world":
                self.load(data.get("mat", self.mat), data["world"])
            elif do == "move_box":
                for b in self.boxes:
                    if b["id"] == data["id"]:
                        b["x"], b["y"] = float(data["x"]), float(data["y"])
            elif do == "add_box":
                box = {"id": self.next_box, "x": float(data["x"]), "y": float(data["y"]),
                       "w": float(data.get("w", 20)), "h": float(data.get("h", 20)), "name": "box"}
                self.next_box += 1
                self.boxes.append(box)
                return {"id": box["id"]}
            elif do == "remove_box":
                self.boxes = [b for b in self.boxes if b["id"] != data["id"]]
            elif do == "button":
                if data["name"] not in ("stop", "resume"):
                    return {"error": "unknown button"}
                self.buttons[data["name"]] = self.time + BUTTON_PRESS_S
            elif do == "adult_resume":
                # An adult puts the robot back on the cell where the interrupted move started,
                # facing the same way, then presses Resume on the robot.
                if self.move_start is not None and self.firmware.motion is None:
                    x, y, h = self.move_start
                    self.x = (int(x // self.cell) + 0.5) * self.cell
                    self.y = (int(y // self.cell) + 0.5) * self.cell
                    self.heading = (round(h / 90) * 90) % 360
                self.buttons["resume"] = self.time + BUTTON_PRESS_S
            elif do == "cable":
                self.link.plugged = bool(data["plugged"])
                self.unplugged_at = None if self.link.plugged else self.time
            elif do == "battery":
                self.battery_level = float(data["percent"])
            elif do == "mode":
                if data["mode"] not in ("realtime", "frozen"):
                    return {"error": "unknown mode"}
                self.mode = data["mode"]
            elif do == "noise":
                for key in self.noise:
                    if key in data:
                        self.noise[key] = max(0.0, float(data[key]))
            elif do == "reload_reflex":
                return {"ok": self.reload_reflex(), "error": self.reflex_error}
            else:
                return {"error": "unknown action"}
        return {"ok": True}

    def add_context(self, entry):
        with self.lock:
            self.context.append(entry)
            self.last_command_at = self.time


class Hal:
    """What the board of the robot gives the firmware, simulated."""

    def __init__(self, world):
        self.w = world

    def ticks_ms(self):
        return int(self.w.time * 1000)

    def ticks_diff(self, new, old):
        return new - old

    def read_lines(self):
        return self.w.link.read_lines(self.w.time)

    def write_line(self, text):
        self.w.link.write_line(text, self.w.time)
        if text.startswith("EVT STOP"):
            self.w.stops.append({"t": round(self.w.time, 2), "why": text.split()[2],
                                 "unplugged_at": self.w.unplugged_at})

    def drive(self, speed_cm_s, turn_deg_s):
        self.w.speed, self.w.turn = float(speed_cm_s), float(turn_deg_s)

    def odometry(self):
        return self.w.odo_cm, self.w.odo_deg

    def distance_cm(self):
        return self.w.distance()

    def battery(self):
        return self.w.battery_level

    def button(self, name):
        return self.w.buttons.get(name, -1) > self.w.time

    def camera_label(self):
        return self.w.card_ahead()

    def led(self, color):
        self.w.led_color = color


def tail_context(world, path, stop):
    """Show on the page what the gateway gave the agent (the gateway writes it to context.jsonl)."""
    position = path.stat().st_size if path.exists() else 0
    while not stop.is_set():
        try:
            size = path.stat().st_size
            if size < position:
                position = 0
            if size > position:
                with path.open(encoding="utf-8") as f:
                    f.seek(position)
                    for line in f:
                        if line.strip():
                            world.add_context(json.loads(line))
                    position = f.tell()
        except (OSError, ValueError):
            pass
        stop.wait(0.2)


def physics(world, stop):
    next_tick = time.monotonic()
    while not stop.is_set():
        world.step(TICK_S)
        next_tick += TICK_S
        delay = next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            next_tick = time.monotonic()


def make_handler(world):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, code, body, kind="application/json"):
            data = body if isinstance(body, bytes) else json.dumps(body).encode()
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in STATIC_FILES:
                name, kind = STATIC_FILES[self.path]
                return self.send(200, (STATIC / name).read_bytes(), kind + "; charset=utf-8")
            if self.path == "/api/state":
                return self.send(200, world.snapshot())
            if self.path == "/api/events":
                self.send_response(200)
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                try:
                    while True:
                        self.wfile.write(b"data: " + json.dumps(world.snapshot()).encode() + b"\n\n")
                        self.wfile.flush()
                        time.sleep(0.1)
                except (BrokenPipeError, ConnectionResetError):
                    return
            self.send(404, {"error": "not found"})

        def do_POST(self):
            # JSON only: a page of another site cannot send JSON here without asking first (CORS),
            # and this server never says yes.
            if self.path != "/api/action" or not self.headers.get("Content-Type", "").startswith("application/json"):
                return self.send(400, {"error": "POST /api/action with JSON"})
            try:
                length = min(int(self.headers.get("Content-Length", 0)), 1_000_000)
                data = json.loads(self.rfile.read(length))
                return self.send(200, world.action(data))
            except (ValueError, KeyError, TypeError) as e:
                return self.send(400, {"error": f"{type(e).__name__}: {e}"})

    return Handler


def main():
    parser = argparse.ArgumentParser(description="Simulator of the Youth AI Lab robot")
    parser.add_argument("--port", type=int, default=int(os.environ.get("ROBOT_SIM_PORT", 8780)))
    parser.add_argument("--mat", default=str(GUIDE / "mat.json"))
    parser.add_argument("--world", default=str(HERE / "worlds" / "forest.json"))
    parser.add_argument("--state-dir", default=os.environ.get("ROBOT_STATE_DIR", str(GUIDE / ".robot-sim")))
    parser.add_argument("--seed", type=int, default=None)
    args = parser.parse_args()

    mat = json.loads(Path(args.mat).read_text(encoding="utf-8"))
    state = Path(args.state_dir)
    state.mkdir(parents=True, exist_ok=True)
    world = World(mat, json.loads(Path(args.world).read_text(encoding="utf-8")), state, seed=args.seed)
    # Where the gateway finds the cable of the simulated robot.
    (state / "serial").write_text(world.link.path + "\n", encoding="utf-8")
    context = state / "context.jsonl"

    stop = threading.Event()
    threading.Thread(target=physics, args=(world, stop), daemon=True).start()
    threading.Thread(target=tail_context, args=(world, context, stop), daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), make_handler(world))
    server.daemon_threads = True
    port = server.server_address[1]
    print(f"Robot simulator: http://127.0.0.1:{port}  (in a Codespace: Ports tab, port {port})")
    print(f"Simulated USB cable: {world.link.path}")
    print(f"READY port={port} serial={world.link.path}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("Simulator stopped.")
    finally:
        stop.set()
        (state / "serial").unlink(missing_ok=True)


if __name__ == "__main__":
    main()
