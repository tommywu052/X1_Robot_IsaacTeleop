#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Authoring service: a video goes in, a gesture X1 can perform comes out.

The heavy step is estimating the body, which runs at about two frames a second
and happens once per recording. Everything the page lets someone change after
that -- smoothing, which frames are keyframes, whether the head is read off the
face -- is recomputed in this process on each request and returns immediately.

Playback reuses the player from the playback UI next door rather than
reimplementing it, so a gesture authored here reaches the arms by exactly the
path every other gesture does.
"""

from __future__ import annotations

import os
import shutil
import sys
import threading
from pathlib import Path

import fastapi
from fastapi import File, Form, UploadFile
from fastapi.responses import FileResponse, HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

UI_DIR = Path(__file__).resolve().parent
AUTHOR_DIR = UI_DIR.parent
sys.path.insert(0, str(AUTHOR_DIR))

import pipeline  # noqa: E402
from pipeline import DEFAULT_MAX_FRAMES, Job, JobError, safe_name  # noqa: E402

# The playback stack. Its library is what this writes into, and its player is
# what this plays through, so both point at one deployment.
LABAN_DIR = pipeline.find_laban_dir()
LIBRARY = LABAN_DIR / 'gestures' / 'library'

DDS_MODE = os.environ.get('X1_DDS', 'real')
MAPPER = os.environ.get('X1_MAPPER', 'ik')

# Uploads are capped because the estimator is the slow part and a long
# recording is a long wait, not a better gesture.
MAX_UPLOAD_MB = int(os.environ.get('LABAN_AUTHOR_MAX_MB', '400'))
VIDEO_SUFFIXES = {'.mp4', '.mov', '.m4v', '.avi', '.mkv', '.webm'}


def _load_player():
    """GesturePlayer and GesturePolicy from the playback deployment.

    gesture_player.py finds run_player.sh relative to its own location, so it
    has to be imported from where it is installed rather than copied here.
    """
    ui = LABAN_DIR / 'ui'
    if not (ui / 'gesture_player.py').exists():
        return None, None, 'no gesture_player.py under %s' % ui
    sys.path.insert(0, str(LABAN_DIR))
    sys.path.insert(0, str(ui))
    try:
        from gesture_player import GesturePlayer
        from gesture_policy import GesturePolicy
    except ImportError as exc:
        return None, None, str(exc)
    return (GesturePlayer(dds_mode=DDS_MODE, mapper=MAPPER),
            GesturePolicy(profile='chat'), None)


player, policy, player_error = _load_player()

app = fastapi.FastAPI(title='X1 Gesture Authoring')
app.mount('/static', StaticFiles(directory=str(UI_DIR / 'static')),
          name='static')

JOBS: dict[str, Job] = {}
JOBS_LOCK = threading.Lock()


@app.on_event('startup')
def reopen_previous_jobs():
    """Bring back whatever was being edited before the last restart.

    A saved gesture is the durable artefact and survives regardless, but the
    editing session used to be lost on restart -- which is not academic:
    redeploying this service dropped a session someone was in the middle of.
    Rebuilding from the keypoints on disk costs a fraction of a second.
    """
    for job in Job.reopen_all():
        JOBS[job.id] = job
    if JOBS:
        print('[x1-author] reopened %d previous recording(s): %s'
              % (len(JOBS), ', '.join(j.name for j in JOBS.values())))


class ScoreRequest(BaseModel):
    name: str | None = None
    keyframes: list[int] | None = None
    gauss_window: int = 31
    gauss_sigma: float = 5.0
    derive_head: bool = False
    acromion: bool = False
    base_rotation: str = 'every'


class SaveRequest(ScoreRequest):
    pass


class PlayRequest(BaseModel):
    file: str
    speed: float = 1.0
    head: bool = True


def _job(job_id) -> Job:
    with JOBS_LOCK:
        job = JOBS.get(job_id)
    if job is None:
        raise fastapi.HTTPException(404, 'no job %s' % job_id)
    return job


def _built(job: Job, req: ScoreRequest):
    return job.build_score(
        name=req.name,
        keyframes=req.keyframes,
        gauss_window=max(5, min(151, int(req.gauss_window)) | 1),
        gauss_sigma=max(0.5, min(30.0, float(req.gauss_sigma))),
        derive_head=req.derive_head,
        use_acromion=req.acromion,
        base_rotation=req.base_rotation
        if req.base_rotation in ('every', 'first') else 'every',
    )


@app.get('/', response_class=HTMLResponse)
async def index():
    return HTMLResponse((UI_DIR / 'static' / 'index.html')
                        .read_text(encoding='utf-8'))


@app.get('/api/status')
async def status():
    # Assigned, not just reported. One failed socket call latches daemon_ok
    # off inside the player and it never tries again, which would quietly cost
    # 2.3s of spawn before the arms move on every later gesture.
    if player is not None:
        player.daemon_ok = player._request({'cmd': 'ping'},
                                           timeout=1.0) is not None
    return {
        'dds': DDS_MODE,
        'mapper': MAPPER,
        'daemon': bool(player and player.daemon_ok),
        'player_error': player_error,
        'library': str(LIBRARY),
        'library_count': len(list(LIBRARY.glob('*.json')))
        if LIBRARY.exists() else 0,
        'jobs_dir': str(pipeline.JOBS_DIR),
        'max_frames': DEFAULT_MAX_FRAMES,
        # So the wait the page quotes is the measured one, in one place.
        'batch_size': pipeline.BATCH_SIZE,
        'startup_s': pipeline.STARTUP_S,
        'per_frame_s': pipeline.PER_FRAME_S,
        'estimator_ready': pipeline.V2D_PYTHON.exists()
        and pipeline.WEIGHTS.exists(),
    }


@app.get('/api/jobs')
async def list_jobs():
    with JOBS_LOCK:
        jobs = list(JOBS.values())
    jobs.sort(key=lambda j: j.created, reverse=True)
    return {'jobs': [j.state() for j in jobs]}


@app.post('/api/jobs')
async def create_job(video: UploadFile = File(...),
                     name: str = Form(''),
                     max_frames: int = Form(DEFAULT_MAX_FRAMES)):
    suffix = Path(video.filename or '').suffix.lower()
    if suffix not in VIDEO_SUFFIXES:
        return JSONResponse(
            {'ok': False, 'error': 'expected a video file, got %r' % suffix},
            status_code=400)

    job = Job(name=name or Path(video.filename or 'gesture').stem,
              max_frames=max(30, min(1200, int(max_frames))))
    job.dir.mkdir(parents=True, exist_ok=True)

    limit = MAX_UPLOAD_MB * 1024 * 1024
    written = 0
    with open(job.video_path, 'wb') as handle:
        while True:
            chunk = await video.read(1 << 20)
            if not chunk:
                break
            written += len(chunk)
            if written > limit:
                handle.close()
                shutil.rmtree(job.dir, ignore_errors=True)
                return JSONResponse(
                    {'ok': False,
                     'error': 'upload over %d MB' % MAX_UPLOAD_MB},
                    status_code=413)
            handle.write(chunk)

    with JOBS_LOCK:
        JOBS[job.id] = job
    job._say('已接收 %s，%.1f MB' % (video.filename, written / 1e6))

    # One at a time is enforced inside the pipeline by a lock on the GPU stage;
    # a second upload queues there rather than being refused here.
    threading.Thread(target=job.run, daemon=True,
                     name='job-%s' % job.id).start()
    return {'ok': True, 'job': job.state()}


@app.get('/api/jobs/{job_id}')
async def job_state(job_id: str):
    return _job(job_id).state()


@app.delete('/api/jobs/{job_id}')
async def drop_job(job_id: str):
    job = _job(job_id)
    with JOBS_LOCK:
        JOBS.pop(job_id, None)
    shutil.rmtree(job.dir, ignore_errors=True)
    return {'ok': True}


@app.get('/api/jobs/{job_id}/overlay.png')
async def overlay(job_id: str):
    job = _job(job_id)
    if not job.overlay_path.exists():
        raise fastapi.HTTPException(404, 'no overlay for this job')
    return FileResponse(job.overlay_path, media_type='image/png')


@app.get('/api/jobs/{job_id}/analysis')
async def analysis(job_id: str):
    try:
        return _job(job_id).analysis()
    except JobError as exc:
        return JSONResponse({'ok': False, 'error': str(exc)}, status_code=409)


@app.post('/api/jobs/{job_id}/score')
async def score(job_id: str, req: ScoreRequest):
    try:
        return {'ok': True, **_built(_job(job_id), req)}
    except JobError as exc:
        return JSONResponse({'ok': False, 'error': str(exc)}, status_code=409)


@app.post('/api/jobs/{job_id}/save')
async def save(job_id: str, req: SaveRequest):
    job = _job(job_id)
    try:
        built = _built(job, req)
    except JobError as exc:
        return JSONResponse({'ok': False, 'error': str(exc)}, status_code=409)

    path = job.save_score(built, library=LIBRARY)
    tier = policy.tier(path.stem) if policy else 'unknown'

    # Said plainly because it decides where the gesture may be used: a new
    # gesture has never been measured against the collision meshes.
    note = ('新手勢的層級是 %r：可以在有人看著的情況下於此播放，'
            '在自我碰撞間隙被量測之前不會納入自主運行。' % tier)
    state = job.state()
    framing = state.get('framing')
    if framing and framing['verdict'] != 'ok':
        # Saving a score from a badly framed recording is allowed, since only
        # the person who made it knows whether that matters, but it should not
        # be forgotten between here and the robot moving.
        note = '來自取景有問題（%s）的錄影，請先在數位分身上確認。%s' \
            % (framing['verdict'], note)

    trimmed = state.get('trimmed')
    if trimmed:
        # Follows the gesture out of the page, because the shortfall is
        # invisible in the saved score: it looks like a complete gesture that
        # simply ends early.
        note = ('只涵蓋了錄影的前 %.1f 秒（全長 %.1f 秒）。%s'
                % (trimmed['used'] / (job.source_info.get('fps') or 30.0),
                   trimmed['available'] / (job.source_info.get('fps') or 30.0),
                   note))

    return {
        'ok': True,
        'name': built['name'],
        'file': str(path),
        'relative': str(path.relative_to(LABAN_DIR)),
        'keyframes': built['count'],
        'tier': tier,
        'framing': framing['verdict'] if framing else None,
        'trimmed': trimmed,
        'note': note,
    }


@app.post('/api/play')
async def play(req: PlayRequest):
    if player is None:
        return JSONResponse({'ok': False, 'error': player_error or 'no player'},
                            status_code=503)
    path = Path(req.file).resolve()
    if LIBRARY.resolve() not in path.parents or path.suffix != '.json':
        return JSONResponse({'ok': False, 'error': 'not a library gesture'},
                            status_code=400)
    if not path.is_file():
        return JSONResponse({'ok': False, 'error': 'no such gesture'},
                            status_code=404)
    if policy and not policy.allowed(path.stem):
        return JSONResponse(
            {'ok': False, 'error': 'blocked: %s' % policy.reason(path.stem)},
            status_code=409)

    player.speed = max(0.3, min(2.0, req.speed))
    player.no_head = not req.head
    return player.play(path)


@app.post('/api/stop')
async def stop():
    if player is not None:
        player.stop()
    return {'ok': True}


if __name__ == '__main__':
    import uvicorn

    uvicorn.run('server:app',
                host=os.environ.get('X1_UI_HOST', '0.0.0.0'),
                port=int(os.environ.get('X1_AUTHOR_PORT', '9300')),
                reload=False)
