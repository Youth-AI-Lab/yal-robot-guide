// The page of the simulator: it draws what the simulator sends ten times per second, and sends
// back what a person does around the mat (drag a box, press a button). It never talks to the robot
// directly: only the gateway does.
"use strict";

const canvas = document.getElementById("mat");
const ctx = canvas.getContext("2d");
const MARGIN = 26; // pixels, for the letters and the numbers of the cells
const CARD_ICONS = { "fire-station": "🚒", "dry-grass": "🌾", "barbecue": "🔥", "water-point": "💧" };
let state = null;
let drag = null; // {id, dx, dy, x, y} while a box is dragged

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
    const col = card.cell.charCodeAt(0) - 65, row = Number(card.cell.slice(1)) - 1;
    ctx.fillStyle = "rgba(255,255,255,0.55)";
    ctx.fillRect(px(s, col * c + 3), px(s, row * c + 3), (c - 6) * s, (c - 6) * s);
    ctx.font = Math.round(c * s * 0.4) + "px system-ui";
    ctx.fillText(CARD_ICONS[card.label] || "?", px(s, (col + 0.5) * c), px(s, (row + 0.45) * c));
    if (c * s >= 70) { // the name only when the cell is large enough to hold it
      ctx.font = "10px system-ui";
      ctx.fillStyle = "#333";
      ctx.fillText(card.label, px(s, (col + 0.5) * c), px(s, (row + 0.85) * c));
    }
  }

  // The cell the camera looks at.
  const r = state.robot;
  const h = r.heading * Math.PI / 180;
  const ax = r.x + Math.sin(h) * c, ay = r.y - Math.cos(h) * c;
  if (ax >= 0 && ay >= 0 && ax < m.cols * c && ay < m.rows * c) {
    ctx.setLineDash([4, 4]);
    ctx.strokeStyle = css("--robot");
    ctx.strokeRect(px(s, Math.floor(ax / c) * c + 1), px(s, Math.floor(ay / c) * c + 1), (c - 2) * s, (c - 2) * s);
    ctx.setLineDash([]);
  }

  // Boxes and balls.
  for (const b of state.boxes) {
    const x = drag && drag.id === b.id ? drag.x : b.x, y = drag && drag.id === b.id ? drag.y : b.y;
    ctx.fillStyle = css("--box");
    ctx.fillRect(px(s, x), px(s, y), b.w * s, b.h * s);
    ctx.fillStyle = "#fff";
    ctx.font = "10px system-ui";
    ctx.fillText(b.name, px(s, x + b.w / 2), px(s, y + b.h / 2));
  }
  for (const b of state.balls) {
    ctx.fillStyle = css("--ball");
    ctx.beginPath(); ctx.arc(px(s, b.x), px(s, b.y), b.radius * s, 0, 2 * Math.PI); ctx.fill();
  }

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
    tile("Card the camera sees", sensors.card_ahead ?? "none"),
    tile("Battery", r.battery + " %", r.battery < (state.reflex.MIN_BATTERY ?? 0)),
    tile("Moving", r.moving ? "yes" : "no"),
    tile("Emergency stop", r.estop ? "PRESSED" : "no", r.estop),
    tile("Waits for an adult", r.wait_adult ? "YES" : "no", r.wait_adult),
    tile("Last stop", r.last_stop),
    tile("Collisions", r.collisions, r.collisions > 0),
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
    li.append(tool, document.createElement("br"), c.result);
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

// Dragging the boxes.
function toCm(event) {
  const rect = canvas.getBoundingClientRect();
  const s = (canvas.clientWidth - MARGIN) / (state.mat.cols * state.mat.cell_cm);
  return { x: (event.clientX - rect.left - MARGIN) / s, y: (event.clientY - rect.top - MARGIN) / s };
}
function boxAt(p) {
  return state.boxes.find((b) => p.x >= b.x && p.x <= b.x + b.w && p.y >= b.y && p.y <= b.y + b.h);
}
canvas.addEventListener("pointerdown", (e) => {
  if (!state) return;
  const p = toCm(e), b = boxAt(p);
  if (!b) return;
  drag = { id: b.id, dx: p.x - b.x, dy: p.y - b.y, x: b.x, y: b.y };
  canvas.setPointerCapture(e.pointerId);
});
canvas.addEventListener("pointermove", (e) => {
  if (!drag) return;
  const p = toCm(e);
  drag.x = p.x - drag.dx;
  drag.y = p.y - drag.dy;
  draw();
});
canvas.addEventListener("pointerup", async () => {
  if (!drag) return;
  const d = drag;
  await act({ do: "move_box", id: d.id, x: d.x, y: d.y });
  drag = null;
});
canvas.addEventListener("dblclick", (e) => {
  const b = state && boxAt(toCm(e));
  if (b) act({ do: "remove_box", id: b.id });
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
                                       ["noise-cam", "n-cam", "camera_miss", 100]]) {
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
