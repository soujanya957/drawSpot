# drawSpot

Spot robot draws on a canvas guided by Vicon motion capture.

## Setup

```bash
pip install bosdyn-client bosdyn-mission bosdyn-api opencv-python pygame
```

Copy `.env.example` to `.env` and fill in your credentials:

```bash
cp .env.example .env
# edit .env with your SPOT_IP, SPOT_USER, SPOT_PASSWORD, VICON_HOST, VICON_PORT
export $(cat .env | xargs)
```

Or use [direnv](https://direnv.net/) to load automatically on `cd`:
```bash
echo 'dotenv' > .envrc && direnv allow
```

---

## File Structure

```
config/
  robot_config.py       # reads SPOT_IP, SPOT_USER, SPOT_PASSWORD from env

draw/                   # Vicon-driven autonomous drawing system
  main.py               # entry point: starts Vicon + web UI + robot controller
  robot/
    controller.py       # translates canvas UV coords to Spot arm commands
    state.py            # robot state machine (IDLE / AUTONOMOUS / TELEOP / ESTOP)
  vicon/
    client.py           # Vicon DataStream client
    transform.py        # world ↔ canvas coordinate transforms
    types.py            # ViconFrame, RigidBody, Marker dataclasses
  ui/
    server.py           # Flask web server
    static/             # browser canvas (index.html + app.js)

src/
  teleop/               # Manual teleoperation scripts
    full_control.py     # base + arm + gripper + pick (all-in-one, recommended)
    teleop_base.py      # base movement only
    teleop_arm.py       # arm only
    teleop_full.py      # base + arm (no gripper/pick)
    canvas_draw.py      # mouse-drag → arm draws on physical canvas
    pick_object.py      # click-to-grasp from hand camera

  vicon/                # Vicon-driven positioning and drawing
    vicon_draw.py         # Manual: full teleop + live Vicon status + web UI canvas draw
    vicon_draw_auto.py    # Autonomous: walk→pick brush→walk→draw pattern (repeatable)
    vicon_base_follow.py  # Vicon body displacement → base walks to match
    vicon_ee_follow.py    # Vicon marker position → arm end-effector follows

patterns/               # Drawing pattern files for vicon_draw_auto.py
  square.json           # Square outline with diagonals
  cross.json            # Plus sign
  spiral.json           # Archimedean spiral

tests/
  test_nav.py           # Walk Spot to brush or canvas and stop at standoff distance
  test_draw.py          # Validate arm drawing from pattern JSON (no navigation)
  test_gcode.py         # Draw from a .gcode file using Vicon canvas as coordinate origin
```

---

## Running

**Autonomous drawing** (Vicon + web UI):
```bash
python -m draw.main --vicon 192.168.1.10:801
```

**Manual teleop** (all controls):
```bash
python -m src.teleop.full_control
```

**Vicon draw** (full teleop + live positions + canvas draw mode):
```bash
python -m src.vicon.vicon_draw
# override Vicon address:
python -m src.vicon.vicon_draw --vicon 192.168.10.1:801
# without Vicon hardware (mock):
python -m src.vicon.vicon_draw --mock
```

Connects to Spot first, then tries Vicon (5 s timeout). If Vicon is unavailable or
`VICON_HOST` is not set, the script continues in teleop-only mode — draw mode (`d` key)
is disabled until Vicon data arrives.

**Autonomous draw** (walk → pick brush → walk → draw pattern):
```bash
python -m src.vicon.vicon_draw_auto patterns/square.json
python -m src.vicon.vicon_draw_auto patterns/spiral.json --repeats 3
# SPACE = pause/resume   RETURN = emergency stop (stow + sit)
```

Pattern JSON format — each stroke is a list of `[u, v]` points (pen down for the full stroke, pen lifts between strokes):
```json
{
  "brush_world_mm": [500, 200, 50],
  "strokes": [
    [[0.2, 0.2], [0.8, 0.2], [0.8, 0.8], [0.2, 0.8], [0.2, 0.2]],
    [[0.2, 0.2], [0.8, 0.8]]
  ]
}
```

**Vicon EE follow test**:
```bash
python -m src.vicon.vicon_ee_follow --vicon 192.168.1.10:801
```

---

## Test scripts

**Navigation test** — walk Spot to brush or canvas and stop at standoff distance:
```bash
python -m tests.test_nav --target brush
python -m tests.test_nav --target canvas
python -m tests.test_nav --target 500,200,0   # world XYZ in mm
```

**Draw validation** — arm already at canvas, execute a pattern JSON:
```bash
python -m tests.test_draw patterns/square.json
python -m tests.test_draw patterns/spiral.json --speed 0.5
python -m tests.test_draw patterns/square.json --mock
```
Pre-validates all waypoints for IK reachability before starting.

**Gcode draw** — draw from a `.gcode` file using the Vicon canvas as the coordinate origin:
```bash
python -m tests.test_gcode my_drawing.gcode
python -m tests.test_gcode my_drawing.gcode --scale 0.001   # gcode in mm (default)
python -m tests.test_gcode my_drawing.gcode --scale 1.0     # gcode in metres
python -m tests.test_gcode my_drawing.gcode --mock
```
Uses `ArmSurfaceContact` for force-compliant drawing. Canvas TL corner and axes from Vicon replace the touch-to-find-ground step. Generate gcode from Inkscape via Extensions → Gcodetools → Path to Gcode, or any CAM tool that outputs G00/G01/G02/G03.

All test scripts: `SPACE` = pause/resume, `RETURN` = emergency stop (stow + sit).
