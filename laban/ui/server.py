#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Manual gesture control for X1: a gesture list in the browser, nothing else.

Playback goes through GesturePlayer, which prefers the resident daemon
(laban/nodes/laban_daemon.py) and falls back to spawning laban/run_player.sh.
The daemon answers in ~0.4s; a spawn costs ~2.3s before the arms move, so the
page reports which one it used.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path

import fastapi
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

UI_DIR = Path(__file__).resolve().parent
LABAN_DIR = UI_DIR.parent
sys.path.insert(0, str(LABAN_DIR))
sys.path.insert(0, str(UI_DIR))

from gesture_player import GesturePlayer  # noqa: E402
from gesture_policy import GesturePolicy  # noqa: E402

LIBRARY = LABAN_DIR / "gestures" / "library"
SAMPLES = LABAN_DIR / "gestures"

DDS_MODE = os.environ.get("X1_DDS", "real")
MAPPER = os.environ.get("X1_MAPPER", "ik")

app = fastapi.FastAPI(title="X1 Gesture Control")
app.mount("/static", StaticFiles(directory=str(UI_DIR / "static")), name="static")

# One player for the page: manual control is single-user, and holding the
# instance keeps the daemon socket probe out of every request.
player = GesturePlayer(dds_mode=DDS_MODE, mapper=MAPPER)
policy = GesturePolicy(profile="chat")


def _catalog():
    out = []
    for path in sorted(LIBRARY.glob("*.json")) + sorted(SAMPLES.glob("*.total.json")):
        stem = path.stem.replace(".total", "")
        tier = policy.tier(path.stem)
        out.append(
            {
                "name": stem,
                "file": str(path.relative_to(LABAN_DIR)),
                "tier": tier,
                "playable": policy.allowed(path.stem),
                "reason": policy.reason(path.stem),
                "sample": path.parent == SAMPLES,
            }
        )
    return out


class PlayRequest(BaseModel):
    file: str
    speed: float = 1.0
    head: bool = True


@app.get("/", response_class=HTMLResponse)
async def index():
    return HTMLResponse((UI_DIR / "static" / "index.html").read_text(encoding="utf-8"))


@app.get("/api/gestures")
async def gestures():
    items = _catalog()
    return {
        "count": len(items),
        "playable": sum(1 for g in items if g["playable"]),
        "gestures": items,
    }


@app.get("/api/status")
async def status():
    player.daemon_ok = player._request({"cmd": "ping"}, timeout=1.0) is not None
    return {
        "dds": DDS_MODE,
        "mapper": MAPPER,
        "daemon": player.daemon_ok,
        "socket": player.socket_path,
        "lead_s": round(player.lead_s, 2),
        "last_gesture": player.last_gesture,
    }


@app.post("/api/play")
async def play(req: PlayRequest):
    # Resolve inside the library: the file name arrives from the browser.
    path = (LABAN_DIR / req.file).resolve()
    if LABAN_DIR not in path.parents or path.suffix != ".json":
        return JSONResponse({"ok": False, "error": "bad gesture path"}, status_code=400)
    if not path.is_file():
        return JSONResponse({"ok": False, "error": "no such gesture"}, status_code=404)
    if not policy.allowed(path.stem):
        return JSONResponse(
            {"ok": False, "error": "blocked: %s" % policy.reason(path.stem)},
            status_code=409,
        )

    player.speed = max(0.3, min(2.0, req.speed))
    player.no_head = not req.head
    return player.play(path)


@app.post("/api/stop")
async def stop():
    player.stop()
    return {"ok": True}


if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "server:app",
        host=os.environ.get("X1_UI_HOST", "0.0.0.0"),
        port=int(os.environ.get("X1_UI_PORT", "9200")),
        reload=False,
    )
