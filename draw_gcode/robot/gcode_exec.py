"""
draw_gcode.robot.gcode_exec
============================
Deprecated — draw_gcode/main.py now uses _draw_file() directly,
which imports drawing primitives from gcode_manual.gcode.

Kept for backwards compatibility with tests/test_draw_gcode.py.
"""

from draw_autonomous.robot.gcode_draw import (
    scan_gcode_bounds,
)


def compute_canvas_scale(gcode_path: str, canvas, margin: float = 0.90) -> float:
    x_min, x_max, y_min, y_max = scan_gcode_bounds(gcode_path, scale=1.0)
    gw = x_max - x_min
    gh = y_max - y_min
    if gw <= 0 or gh <= 0:
        print("  [WARN] Could not determine gcode extents — using default scale 0.001")
        return 0.001
    canvas_w_m = (canvas.width_mm / 1000.0) * margin
    canvas_h_m = (canvas.height_mm / 1000.0) * margin
    scale = min(canvas_w_m / gw, canvas_h_m / gh)
    fitted_w_mm = gw * scale * 1000
    fitted_h_mm = gh * scale * 1000
    print(f"  Gcode extents   {gw:.2f} x {gh:.2f} raw units")
    print(
        f"  Canvas          {canvas.width_mm:.0f} x {canvas.height_mm:.0f} mm"
        f"  (margin {margin * 100:.0f}%)"
    )
    print(
        f"  Auto-scale      {scale:.6f} m/unit"
        f"  ->  drawing {fitted_w_mm:.0f} x {fitted_h_mm:.0f} mm on canvas"
    )
    return scale
