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
    vicon_draw.py         # Full teleop + live Vicon status + canvas draw mode
    vicon_base_follow.py  # Vicon body displacement → base walks to match
    vicon_ee_follow.py    # Vicon marker position → arm end-effector follows
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

**Vicon EE follow test**:
```bash
python -m src.vicon.vicon_ee_follow --vicon 192.168.1.10:801
```
