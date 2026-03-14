"""
Robot State Machine
====================
Tracks the operating mode of the Spot robot and enforces legal transitions.

Modes
-----
IDLE        — robot is stationary; no autonomous or operator-driven motion.
AUTONOMOUS  — Vicon brush-tip data drives the end-effector automatically.
TELEOP      — a human operator drives the brush via the web UI.
ESTOP       — emergency stop; all motion is frozen until an explicit reset.

Transitions
-----------
Any mode  → ESTOP       : emergency_stop()      (always allowed)
ESTOP     → IDLE        : reset()
IDLE      → AUTONOMOUS  : start_autonomous()
IDLE      → TELEOP      : start_teleop()
AUTONOMOUS→ TELEOP      : emergency_takeover()  (operator grabs control mid-run)
TELEOP    → AUTONOMOUS  : release_teleop()
"""

from __future__ import annotations

import logging
import threading
from enum import Enum, auto
from typing import Callable, List, Optional

logger = logging.getLogger(__name__)


class RobotMode(Enum):
    IDLE       = auto()
    AUTONOMOUS = auto()
    TELEOP     = auto()
    ESTOP      = auto()


class RobotStateMachine:
    """
    Thread-safe state machine for the Spot robot operating mode.

    Listeners registered via ``add_listener`` are called (from the thread
    that triggers the transition) whenever the mode changes.
    """

    def __init__(self) -> None:
        self._mode = RobotMode.IDLE
        self._lock = threading.Lock()
        self._listeners: List[Callable[[RobotMode], None]] = []

    # ---------------------------------------------------------------- public
    @property
    def mode(self) -> RobotMode:
        with self._lock:
            return self._mode

    def add_listener(self, cb: Callable[[RobotMode], None]) -> None:
        """Register a callback that receives the new RobotMode on every change."""
        self._listeners.append(cb)

    # ---------------------------------------------------------------- transitions

    def start_autonomous(self) -> bool:
        """
        IDLE / TELEOP → AUTONOMOUS.
        Returns False and logs a warning if the transition is not permitted.
        """
        return self._transition_if(
            allowed_from={RobotMode.IDLE, RobotMode.TELEOP},
            new_mode=RobotMode.AUTONOMOUS,
            label="start_autonomous",
        )

    def start_teleop(self) -> bool:
        """IDLE → TELEOP."""
        return self._transition_if(
            allowed_from={RobotMode.IDLE},
            new_mode=RobotMode.TELEOP,
            label="start_teleop",
        )

    def emergency_stop(self) -> None:
        """
        Unconditionally transition to ESTOP from any mode.
        This is the only transition that is always permitted.
        """
        with self._lock:
            prev = self._mode
            self._mode = RobotMode.ESTOP
        if prev != RobotMode.ESTOP:
            logger.warning("[STATE] ESTOP activated (was %s).", prev.name)
            self._notify(RobotMode.ESTOP)

    def emergency_takeover(self) -> bool:
        """
        AUTONOMOUS → TELEOP.
        Lets an operator grab control from an autonomous run mid-flight.
        """
        return self._transition_if(
            allowed_from={RobotMode.AUTONOMOUS},
            new_mode=RobotMode.TELEOP,
            label="emergency_takeover",
        )

    def release_teleop(self) -> bool:
        """TELEOP → AUTONOMOUS."""
        return self._transition_if(
            allowed_from={RobotMode.TELEOP},
            new_mode=RobotMode.AUTONOMOUS,
            label="release_teleop",
        )

    def reset(self) -> None:
        """Clear ESTOP and return to IDLE."""
        with self._lock:
            prev = self._mode
            self._mode = RobotMode.IDLE
        if prev != RobotMode.IDLE:
            logger.info("[STATE] Reset: %s → IDLE.", prev.name)
            self._notify(RobotMode.IDLE)

    # ---------------------------------------------------------------- helpers

    def _transition_if(
        self,
        allowed_from: set,
        new_mode: RobotMode,
        label: str,
    ) -> bool:
        with self._lock:
            if self._mode == RobotMode.ESTOP:
                logger.warning("[STATE] %s rejected — currently in ESTOP.", label)
                return False
            if self._mode not in allowed_from:
                logger.warning(
                    "[STATE] %s rejected — current mode is %s, expected one of %s.",
                    label, self._mode.name, {m.name for m in allowed_from},
                )
                return False
            prev = self._mode
            self._mode = new_mode

        logger.info("[STATE] %s → %s.", prev.name, new_mode.name)
        self._notify(new_mode)
        return True

    def _notify(self, mode: RobotMode) -> None:
        for cb in self._listeners:
            try:
                cb(mode)
            except Exception as exc:
                logger.error("[STATE] Listener raised: %s", exc)
