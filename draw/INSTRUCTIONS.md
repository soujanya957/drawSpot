# Spot Vicon Canvas Draw

A system for having a Boston Dynamics Spot robot draw on a physical canvas,
guided by Vicon motion capture data and a real-time browser-based interface.

---

## Table of Contents

1. [System Architecture](#system-architecture)
2. [Hardware Setup](#hardware-setup)
3. [Vicon Setup](#vicon-setup)
4. [Software Installation](#software-installation)
5. [Configuration](#configuration)
6. [Running the System](#running-the-system)
7. [Web Interface](#web-interface)
8. [Operating Modes](#operating-modes)
9. [Keyboard Shortcuts](#keyboard-shortcuts)
10. [Troubleshooting](#troubleshooting)

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        draw/ package                            │
│                                                                 │
│  ┌──────────────┐     ViconFrame      ┌────────────────────┐   │
│  │ vicon/       │ ─────────────────▶  │ robot/             │   │
│  │              │   (100 Hz thread)   │                    │   │
│  │ client.py    │                     │ controller.py      │   │
│  │  ├ ViconClient  (real hardware)    │  ├ SpotController  │   │
│  │  └ MockClient  (simulation)        │  └ move_brush()    │   │
│  │              │                     │                    │   │
│  │ types.py     │                     │ state.py           │   │
│  │  ├ Marker                          │  └ RobotStateMachine│  │
│  │  ├ RigidBody                       │    IDLE            │   │
│  │  ├ CanvasFrame                     │    AUTONOMOUS      │   │
│  │  └ ViconFrame                      │    TELEOP          │   │
│  │              │                     │    ESTOP           │   │
│  │ transform.py │                     └────────┬───────────┘   │
│  │  ├ world_to_canvas()                        │               │
│  │  └ canvas_to_world()              stroke events             │
│  └──────────────┘                             │               │
│                                               ▼               │
│                                    ┌────────────────────┐     │
│                                    │ ui/                │     │
│                                    │                    │     │
│                                    │ server.py          │     │
│                                    │  └ UIServer        │     │
│                                    │    FastAPI + WS    │     │
│                                    │    push loop 30fps │     │
│                                    │                    │     │
│                                    │ static/index.html  │     │
│                                    │ static/app.js      │     │
│                                    └────────────────────┘     │
└─────────────────────────────────────────────────────────────────┘
         │                                       │
         ▼                                       ▼
  Vicon DataStream                     http://localhost:8080
  (UDP/TCP port 801)                   (browser canvas UI)
         │
         ▼
  Boston Dynamics Spot
  (bosdyn SDK, port 443)
```

### Component Responsibilities

| Component | File | Role |
|---|---|---|
| **Vicon client** | `vicon/client.py` | Pulls frames from Vicon at ~100 Hz in a background thread. Exposes `latest_frame` (thread-safe). Two implementations: `ViconClient` (real) and `MockViconClient` (sim). |
| **Data types** | `vicon/types.py` | `Marker`, `RigidBody`, `CanvasFrame`, `ViconFrame` — immutable snapshots of one Vicon frame. |
| **Transforms** | `vicon/transform.py` | `world_to_canvas(point, canvas)` and `canvas_to_world(u, v, canvas)` — convert between Vicon world-frame mm and normalised canvas UV [0,1]². |
| **State machine** | `robot/state.py` | `RobotStateMachine` enforces legal mode transitions. Thread-safe. Fires registered callbacks on every change. |
| **Controller** | `robot/controller.py` | `SpotController` translates UV commands into Spot arm Cartesian motions (or logs them in sim mode). Buffers stroke events for the UI. |
| **UI server** | `ui/server.py` | FastAPI app with a WebSocket endpoint. A 30 fps push loop drains the controller's stroke buffer and broadcasts to all connected browsers. |
| **Browser UI** | `ui/static/` | HTML5 canvas that renders strokes in real-time. Mouse drag sends `move` commands when in TELEOP mode. Buttons and keyboard shortcuts control the robot mode. |
| **Entry point** | `main.py` | Starts all three components, wires the Vicon dispatch loop, handles Ctrl-C shutdown. |

### Data Flow

```
Vicon capture system
        │
        │  UDP multicast / TCP pull (vicon-dssdk)
        ▼
ViconClient._run()  [background thread, ~100 Hz]
        │
        │  stores latest ViconFrame (thread-safe)
        ▼
vicon_dispatch_loop()  [background thread, polls at 200 Hz]
        │
        │  calls controller.update_vicon(frame)
        ▼
SpotController._autonomous_tick()  [only in AUTONOMOUS mode]
        │
        │  world_to_canvas(brush_tip.position, canvas)
        │        ↓ UV coordinates
        │  move_brush(u, v, pen_down=True)
        │        ↓ buffers stroke event
        │        ↓ sends Spot arm command (real robot only)
        ▼
UIServer._push_loop()  [async, ~30 fps]
        │
        │  pops stroke events from controller
        │  WebSocket broadcast to all browsers
        ▼
Browser (app.js)
        │
        │  renders strokes on HTML5 canvas
        │  mouse drag → {"type":"move", u, v, pen_down}
        ▼
UIServer._handle_client_message()
        │
        │  controller.move_brush()  (TELEOP)
        │  controller.emergency_stop()
        │  controller.emergency_takeover()  etc.
```

### Canvas Coordinate Frame

The four Vicon corner markers define a local 2D coordinate system:

```
 Vicon world frame (mm)        Canvas UV frame [0,1]²
 ─────────────────────         ──────────────────────
 TL marker  ──→ TR marker      (0,0) ──────── (1,0)
     │                           │                │
     ↓                           │    u →         │
 BL marker     BR marker         │    v ↓         │
                                (0,1) ──────── (1,1)

 origin  = TL
 x_axis  = normalise(TR − TL)   → U direction
 y_axis  = normalise(BL − TL)   → V direction
 normal  = x_axis × y_axis      → points away from canvas face
```

`world_to_canvas` orthographically projects any 3D world point onto this
plane and returns normalised UV. Points outside the physical canvas have
UV values outside [0,1].

---

## Hardware Setup

1. Attach a marker (or rigid-body cluster) to Spot's body — at least 3
   non-collinear markers for a well-conditioned rigid body solve.
2. Attach a marker or small cluster to Spot's end-effector / wrist.
3. Attach a single marker to the brush tip (the very end of the marker held
   in the gripper).
4. Place one retroreflective marker at each of the four corners of the
   physical canvas (or tape / sticker markers flush with the surface).
5. Ensure the Vicon cameras have line-of-sight to all markers throughout
   the robot's full drawing range of motion.

---

## Vicon Setup

### Subject and marker naming

Create the following objects in **Vicon Tracker** (or Nexus):

| Vicon object | Type | Subject name | Marker / segment name |
|---|---|---|---|
| Spot body | Rigid body | `Spot` | segment = `Spot` |
| End-effector | Rigid body (or 1 marker) | `SpotEE` | segment = `SpotEE` |
| Brush tip | Single labeled marker | `BrushTip` | marker = `BrushTip` |
| Canvas corners | 4 labeled markers in one subject | `Canvas` | markers = `TL`, `TR`, `BR`, `BL` |

**Canvas marker order** — going clockwise from the top-left corner when
facing the canvas:

```
TL ── TR
│      │
BL ── BR
```

### DataStream settings

- Enable **Segment Data** and **Marker Data** in the DataStream output.
- Set the stream mode to **Server Push** or leave it on the default; the
  client uses `ClientPull` which works with both.
- Default port is `801`. If you changed it, pass `--vicon host:PORT`.

### Renaming subjects

If your Vicon project uses different names, edit the constants near the top
of `draw/vicon/client.py`:

```python
SPOT_SUBJECT    = "Spot"
EE_SUBJECT      = "SpotEE"
BRUSH_SUBJECT   = "BrushTip"
CANVAS_SUBJECT  = "Canvas"
CANVAS_MARKERS  = ("TL", "TR", "BR", "BL")
```

---

## Software Installation

### Required (always)

```bash
pip install fastapi "uvicorn[standard]" websockets numpy
```

### Required for real Vicon

```bash
pip install vicon-dssdk
```

### Required for real Spot (already installed per your setup)

```bash
pip install bosdyn-client bosdyn-mission bosdyn-api
```

---

## Configuration

### Robot connection

Edit `config/robot_config.py` at the repo root (shared with all other
scripts in this repo):

```python
ROBOT_IP  = "192.168.80.3"   # Spot's IP address
USERNAME  = "user"
PASSWORD  = "password"
```

Or set environment variables instead:

```bash
export SPOT_IP=192.168.80.3
export SPOT_USER=user
export SPOT_PASSWORD=password
```

### Canvas size

`draw/` reads the canvas geometry live from Vicon every frame — no manual
size configuration is needed. The four corner markers define everything.

### Drawing heights

Brush Z-offset above the canvas surface is set in `draw/robot/controller.py`:

```python
DRAW_HEIGHT_MM = 2.0    # brush touching canvas (pen down)
LIFT_HEIGHT_MM = 30.0   # brush raised (pen up / between strokes)
```

---

## Running the System

### Simulation — no hardware required

```bash
cd /path/to/drawSpot
python -m draw.main
```

Opens the web UI at `http://localhost:8080`. A simulated brush traces a
Lissajous figure over a 400 × 400 mm virtual canvas.

### Real Vicon, simulated robot

```bash
python -m draw.main --vicon 192.168.1.100:801
```

Connects to Vicon and transforms real marker positions, but does not move
Spot.

### Real Vicon + real Spot, start in AUTONOMOUS mode

```bash
python -m draw.main --vicon 192.168.1.100:801 --real-robot --autonomous
```

### All options

```
--vicon HOST:PORT    Vicon DataStream address (omit for simulation)
--real-robot        Connect to Spot and send arm commands
--autonomous        Start in AUTONOMOUS mode immediately
--host ADDR         Web UI bind address (default: 0.0.0.0)
--port PORT         Web UI port (default: 8080)
--log-level LEVEL   debug | info | warning | error (default: info)
```

---

## Web Interface

Open `http://localhost:8080` in any browser on the same network.

```
┌─────────────────────────────────────────────────────┐
│  🤖 Spot Canvas Draw   [ AUTONOMOUS ]          ● WS │  ← top bar
├─────────────────────────────────────┬───────────────┤
│                                     │  Emergency    │
│                                     │ ┌───────────┐ │
│                                     │ │ ⛔ E-STOP │ │
│                                     │ └───────────┘ │
│         HTML5 Canvas                │  Control      │
│         (600 × 600 px)              │ [⚙ Takeover ] │
│                                     │ [▶ Release  ] │
│    strokes rendered here            │ [↺ Autonomous] │
│    crosshair = current brush        │ [☑ Reset    ] │
│                                     │  Canvas       │
│                                     │ [✏ Clear    ] │
│                                     │ [💾 Save PNG] │
│                                     │  u: 0.4321    │
│                                     │  v: 0.5678    │
│                                     │  pen: DOWN    │
└─────────────────────────────────────┴───────────────┘
```

- **Mode badge** — colour-coded: grey=IDLE, blue=AUTONOMOUS, green=TELEOP,
  red flashing=ESTOP.
- **Crosshair** — shows current brush position. Red in TELEOP, blue otherwise.
- **Save PNG** — exports the canvas strokes (no cursor/grid) as a PNG file.

---

## Operating Modes

| Mode | Who drives the brush | How to enter |
|---|---|---|
| **IDLE** | Nobody | On startup, or after Reset |
| **AUTONOMOUS** | Vicon brush-tip marker drives the arm | Click *Autonomous*, or `--autonomous` flag |
| **TELEOP** | Mouse drag in the browser drives the arm | Click *Takeover* |
| **ESTOP** | All motion frozen | Click *E-Stop*, or press Space |

### Transition diagram

```
            start_autonomous()
  IDLE ──────────────────────────▶ AUTONOMOUS
   ▲                                    │
   │ reset()              takeover()    │
   │                           ▼        │
  ESTOP ◀──────────────── TELEOP ◀──────┘
   (any mode)   estop()    release() ──▶ AUTONOMOUS
```

### Emergency takeover

If the robot is running autonomously and you need to take over immediately:

1. Click **Takeover** (or press `t`) — mode switches to TELEOP instantly.
   Spot stops following the Vicon brush position.
2. You now control the brush with mouse drag.
3. Click **Release** (or press `r`) to hand control back to AUTONOMOUS.

### Emergency stop

- Click **E-Stop** or press **Space** from any mode.
- All motion freezes. On a real robot, a `stop_command()` is sent immediately.
- Click **Reset** (or press Escape) to clear the E-Stop and return to IDLE.
  You will need to click *Autonomous* or *Takeover* to resume drawing.

---

## Keyboard Shortcuts

| Key | Action |
|---|---|
| `Space` | E-Stop |
| `t` | Takeover (AUTONOMOUS → TELEOP) |
| `r` | Release (TELEOP → AUTONOMOUS) |
| `Escape` | Reset E-Stop → IDLE |
| `c` | Clear canvas |

Mouse drag on the canvas draws only when in **TELEOP** mode. A hint label
appears on the canvas when the mode is not TELEOP.

---

## Troubleshooting

**"vicon-dssdk not installed"**
Install the SDK: `pip install vicon-dssdk`. Or omit `--vicon` to run in
simulation mode.

**Canvas markers not found / `canvas` is None**
Check that the Canvas subject has exactly four markers named `TL`, `TR`,
`BR`, `BL` (case-sensitive) and that none are occluded. The system skips
frames where any canvas corner is missing.

**Brush position jumps / noisy strokes**
The `MockViconClient` adds 0.3 mm of Gaussian noise — normal. For real
Vicon, check camera coverage and marker reflectivity. The move threshold
(`_MOVE_THRESHOLD_UV = 0.002` in `controller.py`) filters sub-0.8 mm
jitter.

**Spot arm not moving**
Confirm `--real-robot` is set and `config/robot_config.py` has the correct
IP. Make sure no other process holds the Spot lease.

**Web UI shows "disconnected" dot**
The WebSocket auto-reconnects every 2 seconds. If it stays disconnected,
confirm the server started without errors and that your firewall allows
port 8080.

**Port 8080 already in use**
Pass a different port: `python -m draw.main --port 8181`.
