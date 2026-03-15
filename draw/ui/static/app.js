/**
 * Spot Canvas Draw — Web UI
 * =========================
 * Responsibilities:
 *   1. Open and maintain a WebSocket connection to the Python server.
 *   2. Render stroke events onto an HTML5 canvas in real-time.
 *   3. Capture mouse drag events and send "move" commands when in TELEOP mode.
 *   4. Reflect robot mode changes in the UI (badge, overlays, button states).
 *   5. Route button clicks (E-Stop, Takeover, Release, Reset, Clear, Save).
 *
 * Protocol (JSON over WebSocket):
 *   Server → Client:
 *     { type:"stroke",  u, v, pen_down }
 *     { type:"clear" }
 *     { type:"mode",    data: "IDLE"|"AUTONOMOUS"|"TELEOP"|"ESTOP" }
 *     { type:"vicon",   spot_body, spot_ee, brush_tip, canvas }   (optional)
 *
 *   Client → Server:
 *     { type:"move",     u, v, pen_down }
 *     { type:"estop" }
 *     { type:"takeover" }
 *     { type:"release" }
 *     { type:"autonomous" }
 *     { type:"reset" }
 *     { type:"clear" }
 */

"use strict";

// ---------------------------------------------------------------------------
// Canvas setup
// ---------------------------------------------------------------------------

const CANVAS_MAX_PX = 600;   // max pixel dimension
const GRID_LINES = 10;

const drawCanvas = document.getElementById("draw-canvas");
const ctx = drawCanvas.getContext("2d");

// Physical canvas dimensions (updated from server)
let canvasWidthMm  = 400;
let canvasHeightMm = 400;

// Display rotation sent by server (0 / 90 / 180 / 270).
//   0:   Spot at south — bottom of display = v=1 edge (no transform)
//   90:  Spot at west  — bottom of display = u=0 edge (90° CCW)
//   180: Spot at north — bottom of display = v=0 edge (180°)
//   270: Spot at east  — bottom of display = u=1 edge (90° CW)
let canvasRotation = 0;

// Internal canvas resolution (set once and reused)
let CANVAS_W = CANVAS_MAX_PX;
let CANVAS_H = CANVAS_MAX_PX;

// ---------------------------------------------------------------------------
// UV coordinate transforms
// ---------------------------------------------------------------------------

// Canvas UV (server / physical) → display UV (browser pixels)
function canvasToDisplayUV(u_c, v_c) {
  switch (canvasRotation) {
    case  90: return [v_c,       1 - u_c];
    case 180: return [1 - u_c,   1 - v_c];
    case 270: return [1 - v_c,   u_c    ];
    default:  return [u_c,       v_c    ];
  }
}

// Display UV (browser pixels) → canvas UV (server / physical)
function displayToCanvasUV(u_d, v_d) {
  switch (canvasRotation) {
    case  90: return [1 - v_d,   u_d    ];
    case 180: return [1 - u_d,   1 - v_d];
    case 270: return [v_d,       1 - u_d];
    default:  return [u_d,       v_d    ];
  }
}

function applyCanvasDims(wMm, hMm, rotation = 0) {
  canvasWidthMm  = wMm;
  canvasHeightMm = hMm;
  canvasRotation = rotation;
  // For 90/270 the display axes are swapped
  const dispW = (rotation === 90 || rotation === 270) ? hMm : wMm;
  const dispH = (rotation === 90 || rotation === 270) ? wMm : hMm;
  const ratio = dispW / dispH;
  if (ratio >= 1) {
    CANVAS_W = CANVAS_MAX_PX;
    CANVAS_H = Math.round(CANVAS_MAX_PX / ratio);
  } else {
    CANVAS_H = CANVAS_MAX_PX;
    CANVAS_W = Math.round(CANVAS_MAX_PX * ratio);
  }
  drawCanvas.width  = CANVAS_W;
  drawCanvas.height = CANVAS_H;
  offscreen.width   = CANVAS_W;
  offscreen.height  = CANVAS_H;
  drawGrid();
  resizeCanvas();
}

// CSS responsive sizing
function resizeCanvas() {
  const wrap   = document.getElementById("canvas-wrap");
  const availW = wrap.clientWidth  - 32;
  const availH = wrap.clientHeight - 32;
  const scaleW = availW / CANVAS_W;
  const scaleH = availH / CANVAS_H;
  const scale  = Math.min(scaleW, scaleH, 1);
  drawCanvas.style.width  = Math.round(CANVAS_W * scale) + "px";
  drawCanvas.style.height = Math.round(CANVAS_H * scale) + "px";
}
window.addEventListener("resize", resizeCanvas);
resizeCanvas();

// Offscreen canvas for persistent strokes
const offscreen = document.createElement("canvas");
offscreen.width  = CANVAS_W;
offscreen.height = CANVAS_H;
const offCtx = offscreen.getContext("2d");

// IK reachability grid (pushed from server once per second)
let ikGrid    = null;   // flat bool array, row-major
let ikGridN   = 0;      // grid dimension
let showIk    = true;   // toggle via button

// Draw grid on the offscreen canvas
function drawGrid() {
  offCtx.fillStyle = "#ffffff";
  offCtx.fillRect(0, 0, CANVAS_W, CANVAS_H);

  offCtx.strokeStyle = "#e8e8e8";
  offCtx.lineWidth = 0.5;
  const stepW = CANVAS_W / GRID_LINES;
  const stepH = CANVAS_H / GRID_LINES;
  for (let i = 1; i < GRID_LINES; i++) {
    offCtx.beginPath();
    offCtx.moveTo(i * stepW, 0);
    offCtx.lineTo(i * stepW, CANVAS_H);
    offCtx.stroke();

    offCtx.beginPath();
    offCtx.moveTo(0, i * stepH);
    offCtx.lineTo(CANVAS_W, i * stepH);
    offCtx.stroke();
  }

  // Axis labels
  offCtx.fillStyle = "#cccccc";
  offCtx.font = "10px monospace";
  for (let i = 0; i <= GRID_LINES; i++) {
    const val = (i / GRID_LINES).toFixed(1);
    offCtx.fillText(val, i * stepW + 2, CANVAS_H - 4);
    if (i > 0) offCtx.fillText(val, 2, i * stepH - 3);
  }
}
drawGrid();

// Draw IK reachability overlay onto a canvas context.
// ikGrid is in canvas UV order; we remap each display cell to canvas UV.
function drawIkOverlay(targetCtx) {
  if (!ikGrid || !ikGridN || !showIk) return;
  const n = ikGridN;
  const cellW = CANVAS_W / n;
  const cellH = CANVAS_H / n;
  targetCtx.save();
  for (let row_d = 0; row_d < n; row_d++) {
    for (let col_d = 0; col_d < n; col_d++) {
      // Centre of this display cell in display UV
      const u_d = (col_d + 0.5) / n;
      const v_d = (row_d + 0.5) / n;
      // Convert to canvas UV, find the grid cell
      const [u_c, v_c] = displayToCanvasUV(u_d, v_d);
      const row_c = Math.max(0, Math.min(n - 1, Math.floor(v_c * n)));
      const col_c = Math.max(0, Math.min(n - 1, Math.floor(u_c * n)));
      const ok = ikGrid[row_c * n + col_c];
      targetCtx.fillStyle = ok
        ? "rgba(0, 200, 100, 0.18)"
        : "rgba(220, 53, 69, 0.18)";
      targetCtx.fillRect(col_d * cellW, row_d * cellH, cellW, cellH);
    }
  }
  targetCtx.restore();
}

// Composite offscreen onto visible canvas
function render() {
  ctx.clearRect(0, 0, CANVAS_W, CANVAS_H);
  ctx.drawImage(offscreen, 0, 0);

  // IK reachability overlay (semi-transparent, drawn over strokes)
  drawIkOverlay(ctx);

  // "▲ SPOT" indicator — always at bottom centre, shows which edge faces Spot
  ctx.save();
  ctx.fillStyle = "#aaaaaa";
  ctx.font = "10px monospace";
  ctx.textAlign = "center";
  ctx.fillText("▲ SPOT", CANVAS_W / 2, CANVAS_H - 5);
  ctx.textAlign = "left";
  ctx.restore();

  // Cursor crosshair (state.brushU/V are in display UV)
  if (state.brushU !== null) {
    const px = state.brushU * CANVAS_W;
    const py = state.brushV * CANVAS_H;
    const radius = 6;
    const color = state.mode === "TELEOP" ? "#e63946" : "#457b9d";

    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(px, py, radius, 0, Math.PI * 2);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(px - radius * 1.8, py);
    ctx.lineTo(px + radius * 1.8, py);
    ctx.moveTo(px, py - radius * 1.8);
    ctx.lineTo(px, py + radius * 1.8);
    ctx.stroke();
    ctx.restore();
  }
}

// ---------------------------------------------------------------------------
// Application state
// ---------------------------------------------------------------------------

const state = {
  mode:     "IDLE",        // IDLE | AUTONOMOUS | TELEOP | ESTOP
  brushU:   null,          // last known brush U [0,1]
  brushV:   null,          // last known brush V [0,1]
  penDown:  false,
  mouseDown:  false,
  lastMouseU: null,
  lastMouseV: null,
  strokeCount: 0,
  ws: null,
};

// ---------------------------------------------------------------------------
// Stroke rendering
// ---------------------------------------------------------------------------

let prevPt = null;   // {u, v} of the last stroke point drawn on offscreen

// u_c, v_c are canvas UV (physical coords from server).
// All rendering uses display UV after rotating through canvasRotation.
function applyStroke(u_c, v_c, pen_down) {
  const [u_d, v_d] = canvasToDisplayUV(u_c, v_c);
  state.brushU  = u_d;
  state.brushV  = v_d;
  state.penDown = pen_down;

  const px = u_d * CANVAS_W;
  const py = v_d * CANVAS_H;

  if (pen_down) {
    if (prevPt) {
      offCtx.beginPath();
      offCtx.strokeStyle = "#1a1a1a";
      offCtx.lineWidth   = 2;
      offCtx.lineCap     = "round";
      offCtx.lineJoin    = "round";
      offCtx.moveTo(prevPt.u * CANVAS_W, prevPt.v * CANVAS_H);
      offCtx.lineTo(px, py);
      offCtx.stroke();
      state.strokeCount++;
    } else {
      offCtx.beginPath();
      offCtx.fillStyle = "#1a1a1a";
      offCtx.arc(px, py, 1.5, 0, Math.PI * 2);
      offCtx.fill();
    }
    prevPt = {u: u_d, v: v_d};
  } else {
    prevPt = null;
  }

  document.getElementById("stroke-num").textContent = state.strokeCount;
  document.getElementById("info-u").textContent   = u_c.toFixed(4);
  document.getElementById("info-v").textContent   = v_c.toFixed(4);
  document.getElementById("info-pen").textContent = pen_down ? "DOWN" : "UP";
}

// ---------------------------------------------------------------------------
// WebSocket
// ---------------------------------------------------------------------------

const WS_URL = `ws://${location.host}/ws`;
let reconnectTimer = null;

function connect() {
  if (reconnectTimer) clearTimeout(reconnectTimer);

  const ws = new WebSocket(WS_URL);
  state.ws = ws;

  ws.onopen = () => {
    console.log("[WS] Connected.");
    setConnectionDot(true);
  };

  ws.onmessage = (evt) => {
    let msg;
    try { msg = JSON.parse(evt.data); } catch { return; }

    switch (msg.type) {
      case "stroke":
        applyStroke(msg.u, msg.v, msg.pen_down);
        break;

      case "clear":
        drawGrid();
        prevPt = null;
        state.strokeCount = 0;
        document.getElementById("stroke-num").textContent = 0;
        break;

      case "mode":
        applyMode(msg.data);
        break;

      case "vicon":
        applyViconStatus(msg);
        break;

      case "canvas_dims":
        applyCanvasDims(msg.width_mm, msg.height_mm, msg.rotation || 0);
        break;

      case "ik_grid":
        ikGrid  = msg.data;
        ikGridN = msg.n;
        break;
    }
    render();
  };

  ws.onclose = () => {
    console.warn("[WS] Disconnected. Reconnecting in 2 s…");
    setConnectionDot(false);
    reconnectTimer = setTimeout(connect, 2000);
  };

  ws.onerror = (err) => {
    console.error("[WS] Error:", err);
    ws.close();
  };
}

function send(msg) {
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify(msg));
  }
}

function setConnectionDot(connected) {
  const dot = document.getElementById("connection-dot");
  dot.className = connected ? "connected" : "disconnected";
}

// ---------------------------------------------------------------------------
// Mode UI
// ---------------------------------------------------------------------------

const MODE_CLASSES = {
  IDLE:       "mode-idle",
  AUTONOMOUS: "mode-autonomous",
  TELEOP:     "mode-teleop",
  ESTOP:      "mode-estop",
};

function applyMode(mode) {
  state.mode = mode;
  const badge = document.getElementById("mode-badge");
  badge.className = ""; // clear all classes
  badge.textContent = mode;
  badge.classList.add(MODE_CLASSES[mode] || "mode-idle");

  const overlay = document.getElementById("estop-overlay");
  overlay.style.display = mode === "ESTOP" ? "block" : "none";

  // Button enable/disable logic
  document.getElementById("btn-estop").disabled    = (mode === "ESTOP");
  document.getElementById("btn-takeover").disabled = (mode === "ESTOP");
  document.getElementById("btn-release").disabled  = (mode !== "TELEOP");
  document.getElementById("btn-auto").disabled     = (mode === "AUTONOMOUS" || mode === "ESTOP");
  document.getElementById("btn-reset").disabled    = (mode !== "ESTOP");

  // Show/hide the canvas hint
  updateCanvasHint();
}

function updateCanvasHint() {
  const hint = document.getElementById("canvas-hint");
  if (state.mode !== "TELEOP") {
    hint.classList.add("visible");
    hint.textContent = state.mode === "ESTOP"
      ? "E-Stop active — click Reset to resume"
      : 'Click \u201cTakeover\u201d to draw';
  } else {
    hint.classList.remove("visible");
  }
}

// ---------------------------------------------------------------------------
// Vicon object status dots
// ---------------------------------------------------------------------------

function applyViconStatus(msg) {
  setDot("dot-spot-body",  msg.spot_body);
  setDot("dot-spot-ee",    msg.spot_ee);
  setDot("dot-brush-tip",  msg.brush_tip);
  setDot("dot-canvas",     msg.canvas);
}

function setDot(id, ok) {
  const el = document.getElementById(id);
  el.className = "vicon-dot " + (ok ? "ok" : "bad");
}

// ---------------------------------------------------------------------------
// Mouse teleop
// ---------------------------------------------------------------------------

// Returns display UV {u_d, v_d} from a mouse/touch event.
function mouseDisplayUV(evt) {
  const rect = drawCanvas.getBoundingClientRect();
  const u_d = (evt.clientX - rect.left) / rect.width;
  const v_d = (evt.clientY - rect.top)  / rect.height;
  return {
    u_d: Math.max(0, Math.min(1, u_d)),
    v_d: Math.max(0, Math.min(1, v_d)),
  };
}

// Throttle: only send if display-UV position changed by more than this.
const SEND_THRESHOLD = 0.002;

function shouldSend(u_d, v_d) {
  if (state.lastMouseU === null) return true;
  const du = u_d - state.lastMouseU;
  const dv = v_d - state.lastMouseV;
  return Math.sqrt(du*du + dv*dv) >= SEND_THRESHOLD;
}

drawCanvas.addEventListener("mousemove", (evt) => {
  if (state.mode !== "TELEOP") return;
  const {u_d, v_d} = mouseDisplayUV(evt);
  state.brushU = u_d;
  state.brushV = v_d;

  if (state.mouseDown && shouldSend(u_d, v_d)) {
    const [u_c, v_c] = displayToCanvasUV(u_d, v_d);
    send({ type: "move", u: u_c, v: v_c, pen_down: true });
    state.lastMouseU = u_d;
    state.lastMouseV = v_d;
  }
  render();
});

drawCanvas.addEventListener("mousedown", (evt) => {
  if (evt.button !== 0) return;
  if (state.mode !== "TELEOP") return;
  state.mouseDown = true;
  const {u_d, v_d} = mouseDisplayUV(evt);
  const [u_c, v_c] = displayToCanvasUV(u_d, v_d);
  send({ type: "move", u: u_c, v: v_c, pen_down: true });
  state.lastMouseU = u_d;
  state.lastMouseV = v_d;
  render();
});

drawCanvas.addEventListener("mouseup", () => {
  if (!state.mouseDown) return;
  state.mouseDown  = false;
  state.lastMouseU = null;
  state.lastMouseV = null;
  if (state.mode === "TELEOP" && state.brushU !== null) {
    const [u_c, v_c] = displayToCanvasUV(state.brushU, state.brushV);
    send({ type: "move", u: u_c, v: v_c, pen_down: false });
  }
});

// Cancel drag if mouse leaves the window
window.addEventListener("mouseup", () => {
  if (state.mouseDown) {
    state.mouseDown  = false;
    state.lastMouseU = null;
    state.lastMouseV = null;
    if (state.mode === "TELEOP" && state.brushU !== null) {
      const [u_c, v_c] = displayToCanvasUV(state.brushU, state.brushV);
      send({ type: "move", u: u_c, v: v_c, pen_down: false });
    }
  }
});

drawCanvas.addEventListener("mouseleave", () => {
  updateCanvasHint();
});

drawCanvas.addEventListener("mouseenter", () => {
  if (state.mode !== "TELEOP") {
    document.getElementById("canvas-hint").classList.add("visible");
  }
});

// Touch support (basic)
drawCanvas.addEventListener("touchmove", (evt) => {
  evt.preventDefault();
  if (state.mode !== "TELEOP") return;
  const touch = evt.touches[0];
  const {u_d, v_d} = mouseDisplayUV(touch);
  if (shouldSend(u_d, v_d)) {
    const [u_c, v_c] = displayToCanvasUV(u_d, v_d);
    send({ type: "move", u: u_c, v: v_c, pen_down: true });
    state.lastMouseU = u_d;
    state.lastMouseV = v_d;
  }
  state.brushU = u_d;
  state.brushV = v_d;
  render();
}, { passive: false });

drawCanvas.addEventListener("touchend", () => {
  if (state.mode === "TELEOP" && state.brushU !== null) {
    const [u_c, v_c] = displayToCanvasUV(state.brushU, state.brushV);
    send({ type: "move", u: u_c, v: v_c, pen_down: false });
  }
  state.lastMouseU = null;
  state.lastMouseV = null;
});

// ---------------------------------------------------------------------------
// Button wiring
// ---------------------------------------------------------------------------

document.getElementById("btn-estop").addEventListener("click", () => {
  send({ type: "estop" });
});

document.getElementById("btn-takeover").addEventListener("click", () => {
  send({ type: "takeover" });
});

document.getElementById("btn-release").addEventListener("click", () => {
  send({ type: "release" });
});

document.getElementById("btn-auto").addEventListener("click", () => {
  send({ type: "autonomous" });
});

document.getElementById("btn-reset").addEventListener("click", () => {
  send({ type: "reset" });
});

document.getElementById("btn-ik").addEventListener("click", () => {
  showIk = !showIk;
  document.getElementById("btn-ik").textContent = showIk ? "Hide IK overlay" : "Show IK overlay";
});

document.getElementById("btn-clear").addEventListener("click", () => {
  send({ type: "clear" });
  drawGrid();
  prevPt = null;
  state.strokeCount = 0;
  document.getElementById("stroke-num").textContent = 0;
  render();
});

document.getElementById("btn-save").addEventListener("click", () => {
  // Composite grid + strokes (no cursor) for clean PNG export
  const exportCanvas = document.createElement("canvas");
  exportCanvas.width  = CANVAS_W;
  exportCanvas.height = CANVAS_H;
  const expCtx = exportCanvas.getContext("2d");
  expCtx.drawImage(offscreen, 0, 0);

  const link = document.createElement("a");
  link.download = `canvas_${Date.now()}.png`;
  link.href     = exportCanvas.toDataURL("image/png");
  link.click();
});

// Keyboard shortcuts
window.addEventListener("keydown", (evt) => {
  if (evt.target.tagName === "INPUT") return;
  switch (evt.key) {
    case " ":   // space — E-Stop
      evt.preventDefault();
      send({ type: "estop" });
      break;
    case "t":
      send({ type: "takeover" });
      break;
    case "r":
      send({ type: "release" });
      break;
    case "Escape":
      send({ type: "reset" });
      break;
    case "c":
      document.getElementById("btn-clear").click();
      break;
  }
});

// ---------------------------------------------------------------------------
// Render loop (cursor animation even without new server data)
// ---------------------------------------------------------------------------

function loop() {
  render();
  requestAnimationFrame(loop);
}

// ---------------------------------------------------------------------------
// Boot
// ---------------------------------------------------------------------------

applyMode("IDLE");
connect();
requestAnimationFrame(loop);
