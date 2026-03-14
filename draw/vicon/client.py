"""
Vicon DataStream Client
========================
Two implementations sharing a common interface:

  ViconClient      — wraps the real Vicon DataStream SDK (vicon-dssdk).
  MockViconClient  — generates synthetic motion for testing without hardware.

Both classes are thread-safe: a background thread continuously pulls frames
and stores the latest one.  Consumers call ``client.latest_frame`` from any
thread to get the most recent ViconFrame.

Subject / marker naming convention expected on the Vicon server:

    Subject "Spot"      → Spot body     (rigid body, segment = "Spot")
    Subject "SpotEE"    → end-effector  (rigid body, segment = "SpotEE")
    Subject "BrushTip"  → brush tip     (single unlabelled marker)
    Subject "Canvas"    → canvas plane  (4 markers: "TL", "TR", "BR", "BL")

Usage:
    client = MockViconClient()          # or ViconClient("192.168.1.10:801")
    client.start()
    ...
    frame = client.latest_frame         # ViconFrame | None
    ...
    client.stop()
"""

from __future__ import annotations

import logging
import math
import threading
import time
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from .types import CanvasFrame, Marker, RigidBody, ViconFrame

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Base class
# ---------------------------------------------------------------------------


class ViconClientBase(ABC):
    """Thread-safe base for real and mock Vicon clients."""

    def __init__(self) -> None:
        self._frame: Optional[ViconFrame] = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._running = False

    # ---------------------------------------------------------------- public
    @property
    def latest_frame(self) -> Optional[ViconFrame]:
        with self._lock:
            return self._frame

    def start(self) -> None:
        self._running = True
        self._thread = threading.Thread(
            target=self._run, daemon=True, name=self.__class__.__name__
        )
        self._thread.start()
        logger.info(f"[Vicon] {self.__class__.__name__} started.")

    def stop(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        logger.info(f"[Vicon] {self.__class__.__name__} stopped.")

    # ---------------------------------------------------------------- internal
    def _set_frame(self, frame: ViconFrame) -> None:
        with self._lock:
            self._frame = frame

    @abstractmethod
    def _run(self) -> None:
        raise NotImplementedError


# ---------------------------------------------------------------------------
# Real Vicon client
# ---------------------------------------------------------------------------


class ViconClient(ViconClientBase):
    """
    Real Vicon DataStream client.

    Requires:  pip install vicon-dssdk
    The SDK package exposes ``ViconDataStream`` at the top level.

    The client pulls frames as fast as Vicon delivers them (typically 100 Hz
    or 250 Hz depending on capture settings).
    """

    SPOT_SUBJECT = "Spot"
    EE_SUBJECT = "SpotEE"
    BRUSH_SUBJECT = "BrushTip"
    CANVAS_SUBJECT = "Canvas"
    CANVAS_MARKERS = ("TL", "TR", "BR", "BL")  # must match Vicon label names

    def __init__(self, host: str = "localhost:801") -> None:
        super().__init__()
        self.host = host

    def _run(self) -> None:
        try:
            from vicon_dssdk import ViconDataStream as vds  # type: ignore[import]
        except ImportError:
            raise RuntimeError(
                "vicon-dssdk is not installed. Run: pip install vicon-dssdk"
            )

        client = vds.Client()
        logger.info(f"[Vicon] Connecting to {self.host} …")
        client.Connect(self.host)
        client.EnableSegmentData()
        client.EnableMarkerData()
        client.SetStreamMode(vds.StreamMode.ClientPull)
        logger.info("[Vicon] Connected.")

        while self._running:
            result = client.GetFrame()
            if result != vds.Result.Success:
                time.sleep(0.002)
                continue
            self._set_frame(self._parse_frame(client, time.time()))

        client.Disconnect()
        logger.info("[Vicon] Disconnected.")

    # ---------------------------------------------------------------- parsing

    def _parse_frame(self, client, ts: float) -> ViconFrame:
        return ViconFrame(
            timestamp=ts,
            spot_body=self._get_rigid_body(client, self.SPOT_SUBJECT),
            spot_ee=self._get_rigid_body(client, self.EE_SUBJECT),
            brush_tip=self._get_single_marker(client, self.BRUSH_SUBJECT),
            canvas=self._get_canvas(client),
        )

    def _get_rigid_body(self, client, subject: str) -> Optional[RigidBody]:
        try:
            trans, occ = client.GetSegmentGlobalTranslation(subject, subject)
            rot, _ = client.GetSegmentGlobalRotationQuaternion(subject, subject)
        except Exception:
            return None

        markers = []
        try:
            n = client.GetMarkerCount(subject)
            for i in range(n):
                mname = client.GetMarkerName(subject, i)
                mpos, mocc = client.GetMarkerGlobalTranslation(subject, mname)
                markers.append(
                    Marker(
                        name=mname,
                        position=np.array(mpos, dtype=float),
                        occluded=bool(mocc),
                    )
                )
        except Exception:
            pass

        return RigidBody(
            name=subject,
            position=np.array(trans, dtype=float),
            rotation_quat=np.array(rot, dtype=float),
            markers=markers,
            occluded=bool(occ),
        )

    def _get_single_marker(self, client, subject: str) -> Optional[Marker]:
        try:
            pos, occ = client.GetMarkerGlobalTranslation(subject, subject)
            return Marker(
                name=subject, position=np.array(pos, dtype=float), occluded=bool(occ)
            )
        except Exception:
            return None

    def _get_canvas(self, client) -> Optional[CanvasFrame]:
        corners: list = []
        for mname in self.CANVAS_MARKERS:
            try:
                pos, occ = client.GetMarkerGlobalTranslation(self.CANVAS_SUBJECT, mname)
                if occ:
                    return None
                corners.append(np.array(pos, dtype=float))
            except Exception:
                return None
        return CanvasFrame(corners=corners) if len(corners) == 4 else None


# ---------------------------------------------------------------------------
# Mock / simulation client
# ---------------------------------------------------------------------------


class MockViconClient(ViconClientBase):
    """
    Simulated Vicon client for testing without any hardware.

    Generates a 400 × 400 mm canvas in the XY plane and animates:
      - Spot body  : fixed at a position ~800 mm behind the canvas
      - End-effector: 30 mm above the brush tip
      - Brush tip  : traces a Lissajous figure over the canvas at 100 Hz
    """

    # Canvas corners in world frame (mm).  Flat in the XY plane.
    _CANVAS_CORNERS = [
        np.array([0.0, 0.0, 0.0]),  # TL
        np.array([400.0, 0.0, 0.0]),  # TR
        np.array([400.0, -400.0, 0.0]),  # BR
        np.array([0.0, -400.0, 0.0]),  # BL
    ]
    _CANVAS = CanvasFrame(corners=_CANVAS_CORNERS)

    # Spot body fixed world position (mm)
    _SPOT_BODY_POS = np.array([-800.0, -200.0, 580.0])

    # Foot positions relative to body (approximate, for realism)
    _FOOT_OFFSETS = [
        np.array([450, 250, -580]),
        np.array([450, -250, -580]),
        np.array([-450, 250, -580]),
        np.array([-450, -250, -580]),
    ]
    _FOOT_NAMES = ["FL", "FR", "RL", "RR"]

    RATE_HZ: int = 100  # simulated capture rate

    # Lissajous parameters  (frequencies in Hz)
    _FX = 0.17
    _FY = 0.23
    _PHASE = math.pi / 4

    # Noise amplitude (mm)
    _NOISE_MM = 0.3

    def _run(self) -> None:
        dt = 1.0 / self.RATE_HZ
        canvas = self._CANVAS
        t = 0.0

        while self._running:
            t += dt

            # Lissajous brush trajectory across the canvas
            u = 0.5 + 0.45 * math.sin(2 * math.pi * self._FX * t)
            v = 0.5 + 0.45 * math.sin(2 * math.pi * self._FY * t + self._PHASE)

            brush_world = (
                canvas.origin
                + u * canvas.width_mm * canvas.x_axis
                + v * canvas.height_mm * canvas.y_axis
                + np.random.randn(3) * self._NOISE_MM
            )

            ee_world = brush_world + canvas.normal * 30.0  # 30 mm above canvas

            spot_body = RigidBody(
                name="Spot",
                position=self._SPOT_BODY_POS.copy(),
                rotation_quat=np.array([0.0, 0.0, 0.0, 1.0]),
                markers=[
                    Marker(name=n, position=self._SPOT_BODY_POS + off)
                    for n, off in zip(self._FOOT_NAMES, self._FOOT_OFFSETS)
                ],
            )

            spot_ee = RigidBody(
                name="SpotEE",
                position=ee_world,
                rotation_quat=np.array([0.0, 0.707, 0.0, 0.707]),
            )

            brush = Marker(name="BrushTip", position=brush_world)

            self._set_frame(
                ViconFrame(
                    timestamp=time.time(),
                    spot_body=spot_body,
                    spot_ee=spot_ee,
                    brush_tip=brush,
                    canvas=canvas,
                )
            )

            time.sleep(dt)
