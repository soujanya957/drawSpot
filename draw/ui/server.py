"""
Web UI Server
==============
FastAPI application providing:
  GET /          → canvas web UI (index.html)
  WS  /ws        → bidirectional WebSocket for real-time communication

Push loop (server → client, ~30 fps):
  {"type": "stroke", "u": 0.4, "v": 0.6, "pen_down": true}
  {"type": "clear"}
  {"type": "mode",   "data": "AUTONOMOUS"}

Messages from client → server:
  {"type": "move",     "u": 0.4, "v": 0.6, "pen_down": true}
  {"type": "estop"}
  {"type": "takeover"}
  {"type": "release"}
  {"type": "reset"}
  {"type": "clear"}
  {"type": "autonomous"}

Start as a background daemon thread via ``UIServer.start_background()``.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import threading
from contextlib import asynccontextmanager
from typing import Set

import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.staticfiles import StaticFiles

from ..robot.controller import SpotController

logger = logging.getLogger(__name__)

_STATIC_DIR = os.path.join(os.path.dirname(__file__), "static")


class UIServer:
    """
    Wraps a FastAPI application and runs it in a background daemon thread.

    The push loop polls the controller for new stroke events every ~33 ms
    and broadcasts them to all connected WebSocket clients.  Mode changes
    are also broadcast as they arrive.
    """

    def __init__(
        self,
        controller: SpotController,
        host: str = "0.0.0.0",
        port: int = 8080,
    ) -> None:
        self.controller = controller
        self.host = host
        self.port = port

        self._clients: Set[WebSocket] = set()
        self._clients_lock: asyncio.Lock = None     # created inside the event loop
        self._loop: asyncio.AbstractEventLoop = None

        # Register for mode-change notifications so we can push immediately.
        controller.state.add_listener(self._on_mode_change)

        self.app = self._build_app()

    # ------------------------------------------------------------------
    # App construction
    # ------------------------------------------------------------------

    def _build_app(self) -> FastAPI:
        @asynccontextmanager
        async def lifespan(app: FastAPI):
            self._loop = asyncio.get_event_loop()
            self._clients_lock = asyncio.Lock()
            task = asyncio.create_task(self._push_loop())
            yield
            task.cancel()

        app = FastAPI(title="Spot Canvas Draw", lifespan=lifespan)
        app.mount("/static", StaticFiles(directory=_STATIC_DIR), name="static")

        @app.get("/")
        async def index() -> HTMLResponse:
            with open(os.path.join(_STATIC_DIR, "index.html")) as fh:
                return HTMLResponse(fh.read())

        @app.websocket("/ws")
        async def ws_endpoint(websocket: WebSocket) -> None:
            await websocket.accept()
            async with self._clients_lock:
                self._clients.add(websocket)
            # Send current state on connect.
            await websocket.send_json(
                {"type": "mode", "data": self.controller.mode.name}
            )
            try:
                async for raw in websocket.iter_text():
                    try:
                        msg = json.loads(raw)
                    except json.JSONDecodeError:
                        continue
                    await self._handle_client_message(msg)
            except WebSocketDisconnect:
                pass
            finally:
                async with self._clients_lock:
                    self._clients.discard(websocket)

        return app

    # ------------------------------------------------------------------
    # WebSocket message handling (client → server)
    # ------------------------------------------------------------------

    async def _handle_client_message(self, msg: dict) -> None:
        t = msg.get("type")

        if t == "move":
            u   = float(msg.get("u",        0.5))
            v   = float(msg.get("v",        0.5))
            pen = bool (msg.get("pen_down", True))
            self.controller.move_brush(u, v, pen)

        elif t == "estop":
            self.controller.emergency_stop()

        elif t == "takeover":
            ok = self.controller.emergency_takeover()
            if not ok:
                # If IDLE, switch directly to TELEOP.
                self.controller.start_teleop()

        elif t == "release":
            ok = self.controller.release_teleop()
            if not ok:
                self.controller.start_autonomous()

        elif t == "autonomous":
            self.controller.start_autonomous()

        elif t == "reset":
            self.controller.reset()

        elif t == "clear":
            self.controller.clear_canvas()

    # ------------------------------------------------------------------
    # Push loop — runs inside the uvicorn event loop
    # ------------------------------------------------------------------

    async def _push_loop(self) -> None:
        prev_mode: str = ""
        while True:
            await asyncio.sleep(0.033)   # ~30 fps

            events = self.controller.pop_new_events()
            for ev in events:
                await self._broadcast(ev)

            # Broadcast mode if it changed (catches changes not triggered by
            # state-machine listeners, e.g. direct transitions).
            current_mode = self.controller.mode.name
            if current_mode != prev_mode:
                await self._broadcast({"type": "mode", "data": current_mode})
                prev_mode = current_mode

    # ------------------------------------------------------------------
    # Broadcast helpers
    # ------------------------------------------------------------------

    async def _broadcast(self, msg: dict) -> None:
        if self._clients_lock is None:
            return
        async with self._clients_lock:
            dead: Set[WebSocket] = set()
            for ws in self._clients:
                try:
                    await ws.send_json(msg)
                except Exception:
                    dead.add(ws)
            self._clients -= dead

    def _on_mode_change(self, mode) -> None:
        """Called from non-async threads when the robot mode changes."""
        if self._loop is None:
            return
        asyncio.run_coroutine_threadsafe(
            self._broadcast({"type": "mode", "data": mode.name}),
            self._loop,
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start_background(self) -> threading.Thread:
        """Launch the uvicorn server in a daemon thread.  Returns the thread."""

        def _run() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            config = uvicorn.Config(
                self.app,
                host=self.host,
                port=self.port,
                log_level="warning",
                loop="asyncio",
            )
            server = uvicorn.Server(config)
            loop.run_until_complete(server.serve())

        t = threading.Thread(target=_run, daemon=True, name="UIServer")
        t.start()
        logger.info(f"[UI] Server starting at http://{self.host}:{self.port}")
        return t
