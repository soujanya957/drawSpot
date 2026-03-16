"""
draw_autonomous — Fully Autonomous Spot Gcode Drawing
======================================================
Pipeline:
  1. Walk to brush   (Vicon BrushTip marker)
  2. Pick up brush   (Vicon-guided arm → gripper close)
  3. Walk to canvas  (Vicon canvas corners)
  4. Position arm    (draw pose above canvas, brush pointing down)
  5. Draw            (execute .gcode file via ArmSurfaceContact)
  6. Repeat          (--repeats N)

Subpackages:
  draw_autonomous.robot.navigator  — navigation, pick, positioning
  draw_autonomous.robot.gcode_draw — gcode parsing and arm execution
  draw_autonomous._ctrl            — shared pause/estop events and key listener
"""
