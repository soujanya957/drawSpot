"""
draw_autonomous._ctrl
=====================
Shared pause / emergency-stop state and key listener.

Imported by both navigator and gcode_draw so that SPACE/RETURN work
consistently across the full pipeline.

    from draw_autonomous._ctrl import check, sleep, start_key_listener

    check()        — blocks while paused; returns False if estop fired
    sleep(secs)    — interruptible sleep (50 ms chunks); returns False if stopped
    start_key_listener() — start background thread (call once from main)
"""

import select
import sys
import termios
import threading
import time
import tty

# ---------------------------------------------------------------------------
# Shared events
# ---------------------------------------------------------------------------

pause = threading.Event()
pause.set()          # set = NOT paused  (clear = paused)
estop = threading.Event()


# ---------------------------------------------------------------------------
# Public helpers
# ---------------------------------------------------------------------------

def check() -> bool:
    """Block while paused. Return False if emergency stop was requested."""
    pause.wait()
    return not estop.is_set()


def sleep(secs: float) -> bool:
    """Sleep for *secs* honouring pause/estop every 50 ms. Returns False if stopped."""
    end = time.time() + secs
    while time.time() < end:
        if not check():
            return False
        time.sleep(0.05)
    return True


# ---------------------------------------------------------------------------
# Key listener
# ---------------------------------------------------------------------------

def _get_key(timeout: float = 0.1):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _listener_loop() -> None:
    while not estop.is_set():
        key = _get_key(timeout=0.15)
        if key == " ":
            if pause.is_set():
                pause.clear()
                print("\n  [PAUSED]   press SPACE to resume", flush=True)
            else:
                pause.set()
                print("\n  [RESUMED]", flush=True)
        elif key in ("\r", "\n"):
            estop.set()
            pause.set()   # unblock any check() waiters
            print("\n  [EMERGENCY STOP]", flush=True)


def start_key_listener() -> threading.Thread:
    """Start the background key listener. Call once from main before the pipeline."""
    t = threading.Thread(target=_listener_loop, daemon=True, name="KeyListener")
    t.start()
    return t
