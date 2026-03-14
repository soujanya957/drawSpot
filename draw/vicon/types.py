"""
Vicon Data Types
================
Dataclasses for every kind of object tracked by Vicon:

  Marker      — a single retroreflective marker (position only).
  RigidBody   — a named subject with a solved 6-DoF pose (position + quaternion)
                plus its individual marker positions.
  CanvasFrame — exactly 4 corner markers that define the drawing canvas plane
                and expose a local 2-D coordinate frame.
  ViconFrame  — one complete snapshot from Vicon, containing all tracked objects.

All positions are in millimetres in the Vicon world frame unless noted otherwise.
Quaternions are stored as [qx, qy, qz, qw].
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional

import numpy as np


@dataclass
class Marker:
    """A single retroreflective marker."""
    name: str
    position: np.ndarray    # shape (3,) — [x, y, z] in mm
    occluded: bool = False


@dataclass
class RigidBody:
    """
    A named Vicon subject with a solved 6-DoF pose.
    Covers both Spot's body and its end-effector.
    """
    name: str
    position: np.ndarray        # shape (3,) — [x, y, z] in mm
    rotation_quat: np.ndarray   # shape (4,) — [qx, qy, qz, qw]
    markers: List[Marker] = field(default_factory=list)
    occluded: bool = False

    @property
    def rotation_matrix(self) -> np.ndarray:
        """Return 3×3 rotation matrix from the stored quaternion."""
        qx, qy, qz, qw = self.rotation_quat
        return np.array([
            [1 - 2*(qy**2 + qz**2),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [    2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2),     2*(qy*qz - qx*qw)],
            [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)],
        ], dtype=float)


@dataclass
class CanvasFrame:
    """
    Four corner markers that define the canvas plane.

    Expected marker order:
        corners[0] — Top-Left  (TL) — origin of the canvas 2-D frame
        corners[1] — Top-Right (TR) — defines the +U (width) axis
        corners[2] — Bottom-Right (BR)
        corners[3] — Bottom-Left  (BL) — defines the +V (height) axis

    The canvas local coordinate system:
        origin  = TL corner
        x_axis  = unit vector TL → TR   (U direction)
        y_axis  = unit vector TL → BL   (V direction)
        normal  = x_axis × y_axis       (points away from the canvas surface)
    """
    corners: List[np.ndarray]   # list of 4 arrays, each shape (3,), in mm

    # ------------------------------------------------------------------ axes
    @property
    def origin(self) -> np.ndarray:
        return self.corners[0]

    @property
    def x_axis(self) -> np.ndarray:
        """Unit vector along the canvas width (TL → TR)."""
        v = self.corners[1] - self.corners[0]
        return v / np.linalg.norm(v)

    @property
    def y_axis(self) -> np.ndarray:
        """Unit vector along the canvas height (TL → BL)."""
        v = self.corners[3] - self.corners[0]
        return v / np.linalg.norm(v)

    @property
    def normal(self) -> np.ndarray:
        """Unit normal pointing away from the canvas surface."""
        n = np.cross(self.x_axis, self.y_axis)
        return n / np.linalg.norm(n)

    # ------------------------------------------------------------------ size
    @property
    def width_mm(self) -> float:
        return float(np.linalg.norm(self.corners[1] - self.corners[0]))

    @property
    def height_mm(self) -> float:
        return float(np.linalg.norm(self.corners[3] - self.corners[0]))

    def is_valid(self) -> bool:
        return (len(self.corners) == 4
                and self.width_mm > 1.0
                and self.height_mm > 1.0)


@dataclass
class ViconFrame:
    """
    One complete snapshot from the Vicon system.

    spot_body : RigidBody — Spot's body (4-marker model → full 6-DoF pose)
    spot_ee   : RigidBody — Spot's end-effector (rigid body or single-marker fallback)
    brush_tip : Marker    — tip of the marker/brush held in the gripper
    canvas    : CanvasFrame — 4 corner markers of the physical canvas
    timestamp : float     — Unix time of capture
    """
    timestamp: float
    spot_body: Optional[RigidBody] = None
    spot_ee:   Optional[RigidBody] = None
    brush_tip: Optional[Marker]    = None
    canvas:    Optional[CanvasFrame] = None
