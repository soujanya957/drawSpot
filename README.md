# drawSpot

Spot robot draws on a canvas guided by Vicon motion capture.

## Setup

```bash
pip install bosdyn-client bosdyn-mission bosdyn-api opencv-python pygame
```

Copy `.env.example` to `.env` and fill in your robot's credentials:

```bash
cp .env.example .env
# edit .env
export $(cat .env | xargs)
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

  vicon/                # Vicon-driven positioning tests
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

**Vicon EE follow test**:
```bash
python -m src.vicon.vicon_ee_follow --vicon 192.168.1.10:801
```
