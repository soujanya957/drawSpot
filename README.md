# drawSpot

Spot robot draws on a canvas guided by Vicon motion capture.

## Setup

```bash
pip install bosdyn-client bosdyn-mission bosdyn-api opencv-python pygame
```

Copy `.env.example` to `.env` and fill in your credentials:

```bash
cp .env.example .env
# edit .env with ROBOT_IP, USERNAME, PASSWORD, VICON_ADDRESS
```

Or use [direnv](https://direnv.net/):
```bash
echo 'dotenv' > .envrc && direnv allow
```

---

## File Structure

```
config/
  robot_config.py       # loads credentials from env

vicon/                  # shared Vicon library
  client.py             # ViconClient + MockViconClient
  types.py              # ViconFrame, CanvasFrame, RigidBody dataclasses

draw_gcode/             # main drawing pipeline (Vicon-guided gcode)
  main.py               # entry point
  gcode.cfg             # tuning parameters (force, velocity, z offsets)
  g_codes/              # place .gcode / .nc files here
  robot/
    navigator.py        # walk_to_canvas, open_for_marker, move_to_draw_pose
  _ctrl.py              # emergency stop + key listener

draw/                   # web UI canvas draw (mouse → arm)
draw_autonomous/        # gcode drawing without web UI
gcode_manual/           # manual gcode execution primitives
gcode_vicon/            # Vicon-referenced gcode execution

src/
  teleop/               # manual teleoperation
    full_control.py     # base + arm + gripper (all-in-one, recommended)
    teleop_base.py      # base movement only
    teleop_arm.py       # arm only
    teleop_full.py      # base + arm (no gripper)
    canvas_draw.py      # mouse-drag → arm draws on canvas
    pick_object.py      # click-to-grasp from hand camera

tests/
  test_draw_gcode.py    # staged pipeline tests (vicon / nav / marker / draw / full)
  probe_vicon.py        # quick Vicon connection check
```

---

## Running

**Full autonomous draw** (main pipeline):
```bash
python -m draw_gcode.main --vicon <VICON_IP>:801
python -m draw_gcode.main --vicon <VICON_IP>:801 --repeats 2
```

**Manual teleop** (all controls):
```bash
python -m src.teleop.full_control
```

**Key bindings (all robot stages):**
- `SPACE` — emergency stop (stow + sit immediately)
- `ENTER` — confirm prompt (marker placement)
- `x` — sit and exit after a stage completes

---

## Staged tests

Run in order — see `draw_gcode_checklist.md` for full pass criteria.

```bash
# Check Vicon only (no Spot required)
python -m tests.test_draw_gcode --stage vicon  --vicon <VICON_IP>:801

# Walk Spot to canvas standoff
python -m tests.test_draw_gcode --stage nav    --vicon <VICON_IP>:801

# Open gripper and wait for marker placement
python -m tests.test_draw_gcode --stage marker --vicon <VICON_IP>:801

# Execute gcode (arm assumed already near canvas)
python -m tests.test_draw_gcode --stage draw   --vicon <VICON_IP>:801

# Full pipeline: nav → marker → draw
python -m tests.test_draw_gcode --stage full   --vicon <VICON_IP>:801
```

---

## Networking

If Vicon is unreachable, see `FAQ.md` for the ARP fix and `fixes.md` for the per-session route command.
