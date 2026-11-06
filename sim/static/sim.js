// The page of the simulator: it draws what the simulator sends ten times per second, and sends
// back what a person does around the mat (drag a box, press a button). It never talks to the robot
// directly: only the Referee does.
"use strict";

const canvas = document.getElementById("mat");
const ctx = canvas.getContext("2d");
const MARGIN = 26; // pixels, for the letters and the numbers of the cells
const CARD_ICONS = { "fire-station": "🚒", "dry-grass": "🌾", "barbecue": "🔥", "water-point": "💧" };
let state = null;
let drag = null; // while something is dragged: {kind: "box" | "ball" | "card", ...}

function css(name) { return getComputedStyle(document.documentElement).getPropertyValue(name).trim(); }

async function act(data) {
  const r = await fetch("/api/action", {
    method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(data),
  });
  return r.json();
}

// Sizes: the mat in centimetres, the canvas in pixels.
function scale() {
  const m = state.mat;
  const width = canvas.clientWidth;
  const s = (width - MARGIN) / (m.cols * m.cell_cm);
  const height = MARGIN + m.rows * m.cell_cm * s;
  const ratio = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(width * ratio) || canvas.height !== Math.round(height * ratio)) {
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
    canvas.style.height = height + "px";
  }
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return s;
}
const px = (s, cm) => MARGIN + cm * s;

function draw() {
  if (!state) return;
  const s = scale();
  const m = state.mat, c = m.cell_cm;
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  ctx.fillStyle = css("--mat");
  ctx.fillRect(MARGIN, MARGIN, m.cols * c * s, m.rows * c * s);

  // Grid, letters and numbers.
  ctx.strokeStyle = css("--grid");
  ctx.fillStyle = css("--muted");
  ctx.font = "13px system-ui";
  ctx.textAlign = "center";
  ctx.textBaseline = "middle";
  for (let i = 0; i <= m.cols; i++) {
    ctx.beginPath(); ctx.moveTo(px(s, i * c), MARGIN); ctx.lineTo(px(s, i * c), px(s, m.rows * c)); ctx.stroke();
    if (i < m.cols) ctx.fillText(String.fromCharCode(65 + i), px(s, (i + 0.5) * c), MARGIN / 2);
  }
  for (let j = 0; j <= m.rows; j++) {
    ctx.beginPath(); ctx.moveTo(MARGIN, px(s, j * c)); ctx.lineTo(px(s, m.cols * c), px(s, j * c)); ctx.stroke();
    if (j < m.rows) ctx.fillText(String(j + 1), MARGIN / 2, px(s, (j + 0.5) * c));
  }

  // Cards on the floor.
  for (const card of state.cards) {
    let col = card.cell.charCodeAt(0) - 65, row = Number(card.cell.slice(1)) - 1;
    const dragged = drag && drag.kind === "card" && drag.cell === card.cell;
    if (dragged) { // shown on the cell where it would land
      col = Math.min(m.cols - 1, Math.max(0, Math.floor(drag.x / c)));
      row = Math.min(m.rows - 1, Math.max(0, Math.floor(drag.y / c)));
    }
    ctx.globalAlpha = dragged ? 0.6 : 1;
    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.fillRect(px(s, col * c + 3), px(s, row * c + 3), (c - 6) * s, (c - 6) * s);
    ctx.font = Math.round(c * s * 0.4) + "px system-ui";
    ctx.fillText(CARD_ICONS[card.label] || "?", px(s, (col + 0.5) * c), px(s, (row + 0.45) * c));
    if (c * s >= 70) { // the name only when the cell is large enough to hold it
      ctx.font = "10px system-ui";
      ctx.fillStyle = "#333";
      ctx.fillText(card.label, px(s, (col + 0.5) * c), px(s, (row + 0.85) * c));
    }
    ctx.globalAlpha = 1;
  }

  // What the camera can see: a fan in front of the robot.
  const r = state.robot;
  const h = r.heading * Math.PI / 180;
  const cone = state.sensors.cone;
  if (cone) {
    const a0 = h - Math.PI / 2 - cone.half_angle * Math.PI / 180, a1 = h - Math.PI / 2 + cone.half_angle * Math.PI / 180;
    ctx.save();
    ctx.beginPath();
    ctx.rect(MARGIN, MARGIN, m.cols * c * s, m.rows * c * s);
    ctx.clip();
    ctx.fillStyle = css("--camera");
    ctx.beginPath();
    ctx.moveTo(px(s, r.x), px(s, r.y));
    ctx.arc(px(s, r.x), px(s, r.y), cone.range * s, a0, a1);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
  // Where the camera thinks the cards it sees are (its angle and its distance, with its error).
  for (const [label, angle, cm] of state.sensors.camera || []) {
    const a = h + angle * Math.PI / 180;
    ctx.fillStyle = css("--robot");
    ctx.beginPath();
    ctx.arc(px(s, r.x + Math.sin(a) * cm), px(s, r.y - Math.cos(a) * cm), 3.5, 0, 2 * Math.PI);
    ctx.fill();
  }
  // Where the Cartographer places the cards the camera saw: what the Pilot and the Strategist are told.
  ctx.setLineDash([4, 3]);
  ctx.strokeStyle = css("--belief");
  ctx.fillStyle = css("--belief");
  ctx.font = "10px system-ui";
  for (const b of state.beliefs || []) {
    if (b.x === undefined) continue;
    ctx.beginPath();
    ctx.arc(px(s, b.x), px(s, b.y), b.u * s, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText(b.card + "?", px(s, b.x), px(s, b.y - b.u - 4));
  }
  ctx.setLineDash([]);

  // Boxes and balls.
  for (const b of state.boxes) {
    const mine = drag && drag.kind === "box" && drag.id === b.id;
    const x = mine ? drag.x : b.x, y = mine ? drag.y : b.y;
    ctx.fillStyle = css("--box");
    ctx.fillRect(px(s, x), px(s, y), b.w * s, b.h * s);
    ctx.fillStyle = "#fff";
    ctx.font = "10px system-ui";
    ctx.fillText(b.name, px(s, x + b.w / 2), px(s, y + b.h / 2));
  }
  state.balls.forEach((b, i) => {
    const mine = drag && drag.kind === "ball" && drag.index === i;
    const x = mine ? b.x + drag.x - drag.x0 : b.x, y = mine ? b.y + drag.y - drag.y0 : b.y;
    ctx.fillStyle = css("--ball");
    ctx.beginPath(); ctx.arc(px(s, x), px(s, y), b.radius * s, 0, 2 * Math.PI); ctx.fill();
  });

  // The distance sensor: a line as long as what it measures.
  const d = state.sensors.distance_cm;
  if (d !== null) {
    const len = d + r.radius;
    ctx.strokeStyle = "rgba(179,38,30,0.7)";
    ctx.beginPath();
    ctx.moveTo(px(s, r.x), px(s, r.y));
    ctx.lineTo(px(s, r.x + Math.sin(h) * len), px(s, r.y - Math.cos(h) * len));
    ctx.stroke();
  }

  // The robot: its light around it, an arrow where it looks.
  ctx.lineWidth = 4;
  ctx.strokeStyle = r.led === "off" ? css("--line") : r.led;
  ctx.fillStyle = css("--robot");
  ctx.beginPath(); ctx.arc(px(s, r.x), px(s, r.y), r.radius * s, 0, 2 * Math.PI); ctx.fill(); ctx.stroke();
  ctx.lineWidth = 1;
  ctx.fillStyle = "#fff";
  ctx.beginPath();
  ctx.moveTo(px(s, r.x + Math.sin(h) * r.radius * 0.9), px(s, r.y - Math.cos(h) * r.radius * 0.9));
  ctx.lineTo(px(s, r.x + Math.sin(h + 2.5) * r.radius * 0.5), px(s, r.y - Math.cos(h + 2.5) * r.radius * 0.5));
  ctx.lineTo(px(s, r.x + Math.sin(h - 2.5) * r.radius * 0.5), px(s, r.y - Math.cos(h - 2.5) * r.radius * 0.5));
  ctx.fill();
}

function tile(label, value, alert) {
  const div = document.createElement("div");
  if (alert) div.className = "alert";
  const b = document.createElement("b");
  b.textContent = label;
  div.append(b, String(value));
  return div;
}

function panels() {
  const r = state.robot, sensors = state.sensors;
  document.getElementById("status").replaceChildren(
    tile("Real cell (only the simulator knows it)", r.cell ?? "off the mat"),
    tile("Distance in front", sensors.distance_cm === null ? "nothing in range" : sensors.distance_cm + " cm"),
    tile("Card just in front", sensors.card_ahead ?? "none"),
    tile("Camera sees", (sensors.camera || []).map(([l, a, d]) => l + " " + d + " cm " + (a >= 0 ? "+" : "") + a + "°").join(", ") || "nothing"),
    tile("Battery", r.battery + " %", r.battery < (state.reflex.MIN_BATTERY ?? 0)),
    tile("Moving", r.moving ? "yes" : "no"),
    tile("Emergency stop", r.estop ? "PRESSED" : "no", r.estop),
    tile("Waits for an adult", r.wait_adult ? "YES" : "no", r.wait_adult),
    tile("Last stop", r.last_stop),
    tile("Collisions", r.collisions, r.collisions > 0),
    tile("Bumper", r.bumper ? "TOUCHED" : "free", r.bumper),
    tile("Bumps (the ball)", r.bumps, r.bumps > 0),
    tile("Warning light (reflex.warn)", r.warning ? "ON" : "off", r.warning),
    tile("Simulated time", state.time + " s"),
  );

  const cable = document.getElementById("cable");
  cable.textContent = state.link.plugged ? "Unplug the cable" : "Plug the cable back";
  cable.classList.toggle("on", !state.link.plugged);
  document.getElementById("estop").classList.toggle("on", r.estop);
  const mode = document.getElementById("mode");
  if (document.activeElement !== mode) mode.value = state.mode;

  const reflex = document.getElementById("reflex");
  reflex.replaceChildren(Object.entries(state.reflex).map(([k, v]) => k + " = " + v).join("\n"));
  if (state.reflex_error) {
    const e = document.createElement("div");
    e.className = "error";
    e.textContent = "reflex.py has a mistake, the old reflexes still run: " + state.reflex_error;
    reflex.append(e);
  }

  fill("wire", state.wire, (w) => {
    const li = document.createElement("li");
    li.className = w.dir;
    const arrow = w.dir === "in" ? "computer → robot  " : w.dir === "out" ? "robot → computer  " : "lost (cable)  ";
    li.textContent = w.t.toFixed(1) + " s  " + arrow + w.line;
    return li;
  });
  fill("context", state.context, (c) => {
    const li = document.createElement("li");
    const tool = document.createElement("span");
    tool.className = "tool";
    tool.textContent = c.at.slice(11) + "  " + c.tool + "(" + JSON.stringify(c.arguments).slice(1, -1) + ")";
    const caller = document.createElement("span");
    caller.className = "caller";
    caller.textContent = "  from " + ({ page: "the Drive buttons", terminal: "a terminal" }[c.caller] || "an agent");
    li.append(tool, caller, document.createElement("br"), c.result);
    return li;
  });
  document.getElementById("thinking").textContent =
    state.since_last_command === null ? "" : "(last call " + state.since_last_command + " s ago)";
}

// Rebuild a log only when it changed, and keep it scrolled to the newest line.
const shown = {};
function fill(id, items, make) {
  const key = JSON.stringify(items.at(-1) ?? null) + items.length;
  if (shown[id] === key) return;
  shown[id] = key;
  const ol = document.getElementById(id);
  const atBottom = ol.scrollTop + ol.clientHeight >= ol.scrollHeight - 5;
  ol.replaceChildren(...items.map(make));
  if (atBottom) ol.scrollTop = ol.scrollHeight;
}

// Dragging what is on the mat. The robot is not dragged: an adult who moves it by hand makes the
// Cartographer lose count of where it is (it counts the moves). It moves only through the stack.
function toCm(event) {
  const rect = canvas.getBoundingClientRect();
  const s = (canvas.clientWidth - MARGIN) / (state.mat.cols * state.mat.cell_cm);
  return { x: (event.clientX - rect.left - MARGIN) / s, y: (event.clientY - rect.top - MARGIN) / s };
}
function boxAt(p) {
  return state.boxes.find((b) => p.x >= b.x && p.x <= b.x + b.w && p.y >= b.y && p.y <= b.y + b.h);
}
function grab(p) {
  const box = boxAt(p);
  if (box) return { kind: "box", id: box.id, dx: p.x - box.x, dy: p.y - box.y, x: box.x, y: box.y };
  const ball = state.balls.findIndex((b) => Math.hypot(p.x - b.x, p.y - b.y) <= b.radius + 3);
  if (ball >= 0) return { kind: "ball", index: ball, x0: p.x, y0: p.y, x: p.x, y: p.y };
  const r = state.robot;
  if (Math.hypot(p.x - r.x, p.y - r.y) <= r.radius) return { kind: "robot" };
  const c = state.mat.cell_cm;
  const cell = String.fromCharCode(65 + Math.floor(p.x / c)) + (Math.floor(p.y / c) + 1);
  if (state.cards.some((k) => k.cell === cell)) return { kind: "card", cell, x: p.x, y: p.y };
  return null;
}
let noticeTimer = null;
function notice(text) {
  const hint = document.getElementById("hint");
  if (!hint.dataset.text) hint.dataset.text = hint.textContent;
  hint.textContent = text;
  hint.classList.add("notice");
  clearTimeout(noticeTimer);
  noticeTimer = setTimeout(() => { hint.textContent = hint.dataset.text; hint.classList.remove("notice"); }, 5000);
}
canvas.addEventListener("pointerdown", (e) => {
  if (!state) return;
  const g = grab(toCm(e));
  if (!g) return;
  if (g.kind === "robot") {
    notice("The robot cannot be moved by hand here: the Cartographer counts its moves to know where it is. "
           + "Drive it with the Drive buttons, or press \"Start again\".");
    return;
  }
  drag = g;
  canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener("pointermove", (e) => {
  if (!drag) return;
  const p = toCm(e);
  if (drag.kind === "box") { drag.x = p.x - drag.dx; drag.y = p.y - drag.dy; } else { drag.x = p.x; drag.y = p.y; }
  draw();
});
canvas.addEventListener("pointerup", async () => {
  if (!drag) return;
  const d = drag;
  const c = state.mat.cell_cm;
  if (d.kind === "box") await act({ do: "move_box", id: d.id, x: d.x, y: d.y });
  if (d.kind === "ball") await act({ do: "move_ball", index: d.index, dx: d.x - d.x0, dy: d.y - d.y0 });
  if (d.kind === "card") {
    const col = Math.min(state.mat.cols - 1, Math.max(0, Math.floor(d.x / c)));
    const row = Math.min(state.mat.rows - 1, Math.max(0, Math.floor(d.y / c)));
    const to = String.fromCharCode(65 + col) + (row + 1);
    if (to !== d.cell) await act({ do: "move_card", from: d.cell, to });
  }
  drag = null;
});
canvas.addEventListener("dblclick", (e) => {
  const b = state && boxAt(toCm(e));
  if (b) act({ do: "remove_box", id: b.id });
});

// Driving: each button runs the stack (POST /api/drive), as a terminal command would.
const driveButtons = ["d-forward", "d-left", "d-right", "d-state", "d-light", "d-seen", "d-wait", "g-north", "g-west", "g-south", "g-east"]
  .map((id) => document.getElementById(id));
async function driveRobot(command, extra) {
  if (driveButtons[0].disabled) return;
  const out = document.getElementById("d-result");
  driveButtons.forEach((b) => { b.disabled = true; });
  out.className = "result";
  out.textContent = command + "…";
  try {
    const r = await fetch("/api/drive", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ command, ...extra }),
    });
    const data = await r.json();
    const result = data.result || data;
    out.className = "result " + (result.result || (data.error ? "error" : ""));
    out.textContent = (data.moves ? "moves: " + data.moves.join(", ") + "\n" : "") + JSON.stringify(result, null, 1);
  } catch (err) {
    out.className = "result error";
    out.textContent = String(err);
  } finally {
    driveButtons.forEach((b) => { b.disabled = false; });
  }
}
document.getElementById("d-forward").addEventListener("click", () => driveRobot("forward"));
document.getElementById("d-left").addEventListener("click", () => driveRobot("left"));
document.getElementById("d-right").addEventListener("click", () => driveRobot("right"));
document.getElementById("d-state").addEventListener("click", () => driveRobot("state"));
document.getElementById("d-wait").addEventListener("click", () => driveRobot("wait"));
document.getElementById("d-seen").addEventListener("click", () => driveRobot("seen"));
document.getElementById("d-light").addEventListener("click",
  () => driveRobot("light", { color: document.getElementById("d-color").value }));
for (const side of ["north", "west", "south", "east"]) {
  document.getElementById("g-" + side).addEventListener("click", () => driveRobot("go", { direction: side }));
}
// Arrow keys: a side of the mat, as people point to where the robot should go.
document.addEventListener("keydown", (e) => {
  if (e.target.closest("input, select, textarea")) return;
  if (e.altKey || e.ctrlKey || e.metaKey || e.shiftKey) return; // Alt+← is the browser's Back
  const direction = { ArrowUp: "north", ArrowRight: "east", ArrowDown: "south", ArrowLeft: "west" }[e.key];
  if (!direction) return;
  e.preventDefault();
  if (e.repeat) return; // a key held down sends one order, not one per repeat
  driveRobot("go", { direction });
});

// The buttons.
const on = (id, f) => document.getElementById(id).addEventListener("click", f);
on("estop", () => act({ do: "button", name: "stop" }));
on("resume", () => act({ do: "adult_resume" }));
on("cable", () => act({ do: "cable", plugged: !state.link.plugged }));
on("battery", () => act({ do: "battery", percent: 15 }));
on("addbox", () => act({ do: "add_box", x: state.mat.cols * state.mat.cell_cm / 2 - 10, y: 5 }));
on("reset", () => act({ do: "reset" }));
on("reload", () => act({ do: "reload_reflex" }));
document.getElementById("mode").addEventListener("change", (e) => act({ do: "mode", mode: e.target.value }));
for (const [id, out, key, factor] of [["noise-dist", "n-dist", "distance_cm", 1], ["noise-slip", "n-slip", "slip", 100],
                                       ["noise-cam", "n-cam", "camera_miss", 100], ["noise-camd", "n-camd", "camera_distance", 100]]) {
  const input = document.getElementById(id);
  input.addEventListener("input", () => {
    document.getElementById(out).textContent = input.value;
    act({ do: "noise", [key]: Number(input.value) / factor });
  });
}

const events = new EventSource("/api/events");
events.onmessage = (e) => {
  state = JSON.parse(e.data);
  draw();
  panels();
};
window.addEventListener("resize", draw);
