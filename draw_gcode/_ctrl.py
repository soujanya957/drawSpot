"""
draw_gcode._ctrl
================
Emergency stop and confirmation via keyboard.

Key bindings
------------
  SPACE   — emergency stop (sets estop; pipeline aborts → stow + sit)
  ENTER   — confirm (used by wait_confirm() during marker placement)

Public API
----------
  check()               -> bool   False if estop is set
  sleep(secs)           -> bool   interruptible sleep; False if stopped
  wait_confirm(msg)     -> bool   block until ENTER pressed; False if stopped
  start_key_listener()           call once before the pipeline starts
"""

import select
import sys
import termios
import threading
import time
import tty

# ---------------------------------------------------------------------------
# Shared state
# ---------------------------------------------------------------------------

estop    = threading.Event()   # set on SPACE → abort pipeline
_confirm = threading.Event()   # set on ENTER → used by wait_confirm


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------

def check() -> bool:
    """Return False if emergency stop has been triggered."""
    return not estop.is_set()


def sleep(secs: float) -> bool:
    """Sleep for *secs* in 50 ms chunks. Returns False if stopped."""
    end = time.time() + secs
    while time.time() < end:
        if estop.is_set():
            return False
        time.sleep(0.05)
    return True


def wait_confirm(msg: str) -> bool:
    """
    Print *msg*, then block until the operator presses ENTER to confirm.
    SPACE still triggers emergency stop during this wait.
    Returns False if estop fires before confirmation.
    """
    _confirm.clear()
    print(f"\n  {msg}")
    print("  Press ENTER to confirm  |  SPACE = emergency stop\n", flush=True)
    while not _confirm.is_set():
        if estop.is_set():
            return False
        time.sleep(0.05)
    return not estop.is_set()


# ---------------------------------------------------------------------------
# Key listener
# ---------------------------------------------------------------------------

def _get_key(timeout: float = 0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)   # keeps OPOST on — \n still prints as CR+LF
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old)


def _listener_loop() -> None:
    while not estop.is_set():
        key = _get_key(timeout=0.15)
        if key == " ":
            estop.set()
            print("\n  [EMERGENCY STOP]  robot will stow and sit.", flush=True)
        elif key in ("\r", "\n"):
            _confirm.set()


def start_key_listener() -> threading.Thread:
    """Start the background key listener. Call once from main before the pipeline."""
    t = threading.Thread(target=_listener_loop, daemon=True, name="KeyListener")
    t.start()
    return t
