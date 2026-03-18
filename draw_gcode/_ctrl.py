"""
draw_gcode._ctrl
================
Emergency stop and confirmation via keyboard.

Key bindings
------------
  SPACE   — emergency stop (sets estop; pipeline aborts → stow + sit)
  ESC     — emergency stop (same as SPACE)
  ENTER   — confirm (used by wait_confirm() during marker placement)
  x       — finish (used by wait_finish() to sit and exit after a stage)

Public API
----------
  check()               -> bool   False if estop is set
  sleep(secs)           -> bool   interruptible sleep; False if stopped
  wait_confirm(msg)     -> bool   block until ENTER pressed; False if stopped
  wait_finish(msg)      -> None   block until x pressed (sit + exit prompt)
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
_finish  = threading.Event()   # set on x     → used by wait_finish
_stop    = threading.Event()   # set to shut down the key listener thread


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


def wait_finish(msg: str) -> None:
    """
    Print *msg*, then block until the operator presses x to sit and exit.
    SPACE still triggers emergency stop during this wait.
    """
    _finish.clear()
    print(f"\n  {msg}")
    print("  Press x to sit and exit  |  SPACE = emergency stop\n", flush=True)
    while not _finish.is_set():
        if estop.is_set():
            return
        time.sleep(0.05)


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
    while not estop.is_set() and not _stop.is_set():
        key = _get_key(timeout=0.15)
        if key == " ":
            estop.set()
            print("\n  [EMERGENCY STOP]  robot will stow and sit.", flush=True)
        elif key == "\x1b":
            estop.set()
            print("\n  [ESC]  stopping — robot will stow and sit.", flush=True)
        elif key in ("\r", "\n"):
            _confirm.set()
        elif key in ("x", "X"):
            _finish.set()


def start_key_listener() -> threading.Thread:
    """Start the background key listener. Safe to call multiple times."""
    _stop.clear()
    t = threading.Thread(target=_listener_loop, daemon=True, name="KeyListener")
    t.start()
    return t


def stop_key_listener() -> None:
    """Signal the key listener to exit. Call before using input()."""
    _stop.set()
    time.sleep(0.2)   # give the thread time to exit its current _get_key() poll
