"""
Canvas Coordinate Frame Transforms
====================================
Functions for converting between Vicon world-frame 3-D coordinates and the
canvas-local 2-D UV coordinate system defined by the four corner markers.

Canvas UV convention:
    u ∈ [0, 1]   — horizontal (0 = left / TL corner, 1 = right / TR corner)
    v ∈ [0, 1]   — vertical   (0 = top  / TL corner, 1 = bottom / BL corner)

All positions in millimetres (matching Vicon's default output unit).
"""

from __future__ import annotations

from typing import Tuple

import numpy as np

from .types import CanvasFrame


def world_to_canvas(point_mm: np.ndarray, canvas: CanvasFrame) -> Tuple[float, float]:
    """
    Project a Vicon world-frame 3-D point onto the canvas plane and return
    its normalised UV coordinates.

    The projection is an orthographic projection onto the canvas plane
    (i.e. the point-to-plane distance along the normal is discarded).

    Args:
        point_mm: shape (3,) position in Vicon world frame [mm].
        canvas:   CanvasFrame defining the plane and local axes.

    Returns:
        (u, v) — normalised canvas coordinates.  Values outside [0, 1]
        mean the point is outside the physical canvas boundary.
    """
    local = point_mm - canvas.origin
    u = float(np.dot(local, canvas.x_axis)) / canvas.width_mm
    v = float(np.dot(local, canvas.y_axis)) / canvas.height_mm
    return u, v


def canvas_to_world(
    u: float,
    v: float,
    canvas: CanvasFrame,
    z_offset_mm: float = 0.0,
) -> np.ndarray:
    """
    Convert canvas UV coordinates back to Vicon world-frame 3-D position.

    Args:
        u, v:        Normalised canvas coordinates [0, 1].
        canvas:      CanvasFrame defining the plane and local axes.
        z_offset_mm: Distance along the canvas normal to lift the point
                     above the surface (positive = away from canvas face).

    Returns:
        shape (3,) position in Vicon world frame [mm].
    """
    return (
        canvas.origin
        + u * canvas.width_mm * canvas.x_axis
        + v * canvas.height_mm * canvas.y_axis
        + z_offset_mm * canvas.normal
    )


def distance_to_canvas_mm(point_mm: np.ndarray, canvas: CanvasFrame) -> float:
    """
    Signed distance from *point_mm* to the canvas plane.
    Positive → on the side the normal points toward (above the canvas surface).
    """
    return float(np.dot(point_mm - canvas.origin, canvas.normal))


def clamp_uv(u: float, v: float) -> Tuple[float, float]:
    """Clamp UV coordinates to the valid [0, 1] × [0, 1] canvas area."""
    return float(np.clip(u, 0.0, 1.0)), float(np.clip(v, 0.0, 1.0))
