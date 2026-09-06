"""Video in, Labanotation out, as a job with stages that can be watched.

Split in two on purpose. Estimating the body runs at about two frames a second
on an RTX PRO 6000, so it happens once per recording and its result is written
out as a plain array. Everything after that -- symbols, the energy curve,
keyframe choice, the score itself -- is numpy in this process and finishes
instantly, which is what makes moving a slider feel like editing rather than
re-rendering.

    upload ->  probe  trim  bbox  estimate  export  overlay   (once, minutes)
               analyse                                        (once, instant)
                  |
                  +-> build_score(window, sigma, keyframes, head)  (per edit)

Runnable without the web service, which is how it gets verified:

    python pipeline.py run clip.mp4 --name my_gesture --max-frames 240
"""

from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import uuid
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import numpy as np

from convert import (DEFAULT_GAUSS_SIGMA, DEFAULT_GAUSS_WINDOW,
                     build_labandata, energy_curve, keyframe_indices,
                     per_frame_symbols)
from poses import sam3d_body

HERE = Path(__file__).resolve().parent

V2D_ROOT = Path(os.environ.get('V2D_ROOT',
                               Path.home() / 'nvidia' / 'video_to_data'))
V2D_RECON = V2D_ROOT / 'reconstruction'
V2D_PYTHON = Path(os.environ.get('V2D_PYTHON', V2D_RECON / '.venv/bin/python'))
WEIGHTS = Path(os.environ.get('SAM3D_WEIGHTS',
                              V2D_RECON / 'data/weights/sam3d_body'))
IMAGE = os.environ.get('SAM3D_IMAGE', 'v2d_sam3d_body')

JOBS_DIR = Path(os.environ.get('LABAN_AUTHOR_JOBS',
                               Path.home() / '.laban_author' / 'jobs'))

def find_laban_dir():
    """The playback deployment this tool saves into and plays through.

    Two layouts are real and neither is wrong. In the development tree this
    directory is a sibling of `teleop_share`, so the deployment is one level
    further down; in a checkout of the robot repo it sits beside `laban`
    directly. Probed rather than assumed, because the two used to be separate
    hardcoded guesses in this file and in ui/server.py, which disagreed: the
    library defaulted to the development tree while the player defaulted to the
    checkout, so a fresh clone would have written scores where nothing would
    play them.

    Identified by ui/gesture_player.py rather than by the name alone, since the
    development tree also has a `laban` sibling that holds the autonomous stack
    and no playback UI at all.
    """
    override = os.environ.get('X1_LABAN_DIR')
    if override:
        return Path(override).resolve()
    candidates = (HERE.parent / 'laban',
                  HERE.parent / 'teleop_share' / 'laban')
    for path in candidates:
        if (path / 'ui' / 'gesture_player.py').exists():
            return path.resolve()
    return candidates[0].resolve()


LABAN_DIR = find_laban_dir()
LIBRARY = LABAN_DIR / 'gestures' / 'library'

# Ten seconds at 30 fps. The cap is about the authoring loop rather than the
# estimator: past a certain length you are waiting rather than authoring.
DEFAULT_MAX_FRAMES = 300

# The estimator defaults to one frame at a time, and its own source says the
# batching exists for slow CPUs -- which is the tell, because the cost at
# batch 1 is python dispatching operators, not the GPU doing arithmetic.
# Measured on the RTX PRO 6000 over the same 90 frames:
#
#   batch  wall   inference     gpu
#       1  42.6s  270 ms/frame  64%
#       4  22.8s   68 ms/frame  53%
#       8  21.5s   42 ms/frame  53%
#
# Byte-identical scores at all three, with keypoints agreeing to 1e-6, so this
# is throughput and not a trade. Eight rather than more because by then the
# fixed cost dominates: 3.1 GB of weights and CUDA init, and no batch size
# makes that shorter.
BATCH_SIZE = int(os.environ.get('SAM3D_BATCH_SIZE', '8'))

# What to tell somebody waiting. Both measured, and reported to the page so the
# estimate on screen comes from here rather than being written down twice.
STARTUP_S = 18.0
PER_FRAME_S = 0.042

STAGES = ('probe', 'trim', 'bbox', 'estimate', 'export', 'overlay', 'analyse')

# Only for the log line a person reads while waiting. The identifiers above stay
# as they are, because the page, the state file and the API all key off them.
STAGE_NAMES = {
    'probe': '讀取影片', 'trim': '裁切', 'bbox': '框選人體',
    'estimate': '估計姿態', 'export': '匯出關鍵點', 'overlay': '產生疊圖',
    'analyse': '分析',
}

# Indices into the estimator's 70 keypoints. Its own metadata names them; these
# are the ones a framing verdict can be reasoned about from.
KP_NOSE = 0
KP_SHOULDERS = (5, 6)
KP_HIPS = (9, 10)
KP_UPPER = (0, 5, 6, 7, 8, 62, 41, 9, 10)

# One GPU, so estimates queue rather than fight.
_gpu = threading.Lock()

_SAFE_NAME = re.compile(r'[^A-Za-z0-9 _-]+')


def safe_name(name, fallback='gesture'):
    cleaned = _SAFE_NAME.sub('', (name or '').strip())
    cleaned = re.sub(r'\s+', ' ', cleaned).strip()
    return cleaned[:64] or fallback


class JobError(RuntimeError):
    pass


class Job:
    """One recording on its way to a score."""

    def __init__(self, job_id=None, name=None, max_frames=DEFAULT_MAX_FRAMES,
                 fps=None):
        self.id = job_id or uuid.uuid4().hex[:12]
        self.dir = JOBS_DIR / self.id
        self.name = safe_name(name, 'gesture_%s' % self.id[:6])
        self.max_frames = int(max_frames)
        self.fps_override = fps

        self.stage = 'new'
        self.progress = 0.0
        self.error = None
        self.log = []
        self.clip = None
        self.source_info = {}
        self.created = time.time()
        self.finished = None

        # Filled by analyse(), then reused by every build_score().
        self._keypoints = None
        self._keypoints_2d = None
        self._framing = None
        self._trimmed = None
        self._frames = None
        self._times = None
        self._symbols = None
        self._energy = None
        self._peaks = None

    # ---------------------------------------------------------------- paths

    @property
    def video_path(self):
        return self.dir / 'source.mp4'

    @property
    def clip_path(self):
        return self.dir / 'clip.mp4'

    @property
    def bbox_path(self):
        return self.dir / 'bbox_track.pt'

    @property
    def params_path(self):
        return self.dir / 'mhr_params.pt'

    @property
    def keypoints_path(self):
        return self.dir / 'keypoints.npz'

    @property
    def overlay_path(self):
        return self.dir / 'overlay.png'

    # ----------------------------------------------------------- bookkeeping

    @classmethod
    def reopen(cls, job_dir):
        """Pick a finished recording back up off disk.

        Everything expensive about a job is already in its directory -- the
        clip, the overlay, and the keypoints -- so a restart only loses the
        editing session, and only because nothing used to read them back. It
        does now, because a restart during someone else's session is exactly
        what happened the first time this service was redeployed.
        """
        record = json.loads((Path(job_dir) / 'job.json').read_text())
        job = cls(job_id=record['id'], name=record.get('name'),
                  max_frames=record.get('max_frames', DEFAULT_MAX_FRAMES))
        job.source_info = record.get('source') or {}
        job.created = record.get('created', time.time())
        job.log = list(record.get('log') or [])
        if not job.keypoints_path.exists():
            raise JobError('no keypoints in %s' % job_dir)
        job.analyse()
        # Recomputed rather than read back. The keypoints on disk are the
        # ground truth for how much of the recording was used, and trusting a
        # persisted value instead meant one bad guess, once written, outlived
        # the fix for it.
        job._trimmed = job._infer_trim()
        job.stage = 'ready'
        job.progress = 1.0
        # Restored, not restamped. Taking time.time() here made elapsed_s the age
        # of the recording rather than how long it took, so a job reopened the
        # next day reported twenty-one hours of estimation on the page. Records
        # written before this was persisted have no honest answer, and get None.
        job.finished = record.get('finished')
        job.persist()
        return job

    @classmethod
    def reopen_all(cls, jobs_dir=None):
        """Every recording on disk that still has its keypoints, newest first."""
        root = Path(jobs_dir or JOBS_DIR)
        jobs = []
        if not root.is_dir():
            return jobs
        for path in sorted(root.iterdir(),
                           key=lambda p: p.stat().st_mtime, reverse=True):
            if not (path / 'job.json').exists():
                continue
            try:
                jobs.append(cls.reopen(path))
            except Exception:
                # A half-finished or hand-deleted job directory is not worth
                # refusing to start over.
                continue
        return jobs

    def _say(self, message):
        self.log.append('%s  %s' % (time.strftime('%H:%M:%S'), message))
        del self.log[:-200]

    def _set(self, stage, progress=None):
        self.stage = stage
        if progress is not None:
            self.progress = progress
        self._say('階段：%s' % STAGE_NAMES.get(stage, stage))
        self.persist()

    def state(self):
        return {
            'id': self.id,
            'name': self.name,
            'max_frames': self.max_frames,
            'stage': self.stage,
            'progress': round(self.progress, 3),
            'error': self.error,
            'source': self.source_info,
            'ready': self._symbols is not None,
            'frames': len(self._frames) if self._frames else 0,
            'peaks': list(self._peaks) if self._peaks else [],
            'framing': self._framing,
            'trimmed': self._trimmed,
            'created': self.created,
            'finished': self.finished,
            # Absolute timestamps are persisted and this is derived from them,
            # because the previous shape stored the derived number and a reopen
            # recomputed it against the wall clock: a job read back the next day
            # claimed twenty-one hours of estimation, and once written that was
            # what the next reopen read. None where it is genuinely unknown,
            # which the page renders as nothing rather than as a wrong duration.
            'elapsed_s': (round(self.finished - self.created, 1)
                          if self.finished
                          else (round(time.time() - self.created, 1)
                                if self.stage not in ('ready', 'failed')
                                else None)),
            'log': self.log[-30:],
        }

    def persist(self):
        self.dir.mkdir(parents=True, exist_ok=True)
        payload = {k: v for k, v in self.state().items() if k != 'log'}
        payload['log'] = self.log
        (self.dir / 'job.json').write_text(json.dumps(payload, indent=2))

    # -------------------------------------------------------------- helpers

    def _docker(self, *args, timeout=900):
        """Run clip_tools.py or make_bbox.py inside the estimator's image."""
        command = [
            'docker', 'run', '--rm',
            '-u', '%d:%d' % (os.getuid(), os.getgid()),
            '-v', '%s:/author:ro' % HERE,
            '-v', '%s:/work' % self.dir,
            '-e', 'MPLCONFIGDIR=/tmp/mpl',
            '-e', 'HOME=/tmp',
            '-w', '/author', IMAGE, 'python', *args,
        ]
        done = subprocess.run(command, capture_output=True, text=True,
                              timeout=timeout)
        if done.returncode != 0:
            tail = (done.stderr or done.stdout or '').strip().splitlines()
            raise JobError('%s failed: %s' % (args[0], ' | '.join(tail[-3:])))
        return _last_json(done.stdout)

    # --------------------------------------------------------------- stages

    def run(self, video_source=None):
        """Everything up to the point where scores can be built."""
        try:
            self.dir.mkdir(parents=True, exist_ok=True)
            if video_source is not None:
                shutil.copyfile(video_source, self.video_path)
            if not self.video_path.exists():
                raise JobError('no video at %s' % self.video_path)

            self._probe()
            self._trim()
            self._bbox()
            self._estimate()
            self._export()
            self._overlay()
            self.analyse()

            self.stage = 'ready'
            self.progress = 1.0
            self.finished = time.time()
            self._say('完成，耗時 %.0f 秒' % (self.finished - self.created))
            self.persist()
        except Exception as exc:  # surfaced to the page rather than a traceback
            self.error = str(exc)
            self.stage = 'failed'
            self.finished = time.time()
            self._say('失敗：%s' % exc)
            self.persist()
        return self.state()

    def _probe(self):
        self._set('probe', 0.02)
        info = self._docker('clip_tools.py', 'probe', '/work/source.mp4',
                            timeout=300)
        self.source_info = info
        self._say('來源影片 %dx%d，%.0f fps，%d 幀，%.1f 秒'
                  % (info['width'], info['height'], info['fps'],
                     info['frames'], info['duration_s']))
        if not info['frames']:
            raise JobError('the upload decoded to zero frames')

    def _trim(self):
        keep = min(self.source_info['frames'], self.max_frames)
        if keep == self.source_info['frames']:
            self._set('trim', 0.05)
            shutil.copyfile(self.video_path, self.clip_path)
            self._say('使用全部 %d 幀' % keep)
            return
        self._set('trim', 0.05)
        info = self._docker('clip_tools.py', 'trim', '/work/source.mp4',
                            '/work/clip.mp4', '--frames', str(keep),
                            timeout=600)
        # Recorded rather than only logged. A 20.7 s recording silently becoming
        # a 10 s one cost a real debugging session: the head moved throughout the
        # half that was dropped, and the score's flat head column looked like a
        # broken derivation rather than a missing tail.
        fps = self.source_info.get('fps') or 30.0
        self._trimmed = {
            'used': int(info['frames']),
            'available': int(self.source_info['frames']),
            'dropped_s': round(
                (self.source_info['frames'] - info['frames']) / fps, 1),
        }
        self._say('裁切為 %d 幀（全片 %d 幀），捨棄最後 %.1f 秒'
                  % (info['frames'], self.source_info['frames'],
                     self._trimmed['dropped_s']))

    def _bbox(self):
        self._set('bbox', 0.08)
        info = self._docker('make_bbox.py', '/work/clip.mp4',
                            '-o', '/work/bbox_track.pt', timeout=600)
        self.clip_frames = info if isinstance(info, dict) else {}
        self._say('已產生全畫面的人體框')

    def _estimate(self):
        """The slow one. Runs on the host, which then starts its own container."""
        if not V2D_PYTHON.exists():
            raise JobError('no V2D venv python at %s' % V2D_PYTHON)
        if not WEIGHTS.exists():
            raise JobError('no SAM3D-Body weights at %s' % WEIGHTS)

        self._set('estimate', 0.10)
        command = [
            str(V2D_PYTHON), '-m',
            'v2d.sam3d_body.docker.run_estimate_mhr_params',
            '--rgb_path', str(self.clip_path),
            '--bbox_path', str(self.bbox_path),
            '--weights_dir', str(WEIGHTS),
            '--output_params_path', str(self.params_path),
            '--batch_size', str(BATCH_SIZE),
        ]
        with _gpu:
            frames = min(self.source_info.get('frames', 0), self.max_frames)
            self._say('估計 %d 幀，批次 %d，約需 %d 秒'
                      % (frames, BATCH_SIZE,
                         STARTUP_S + PER_FRAME_S * frames))
            process = subprocess.Popen(command, cwd=str(V2D_RECON),
                                       stdout=subprocess.PIPE,
                                       stderr=subprocess.STDOUT,
                                       text=True, bufsize=1)
            # tqdm writes "  57/240" on a carriage return, which is the only
            # progress this step offers.
            pattern = re.compile(r'(\d+)/(\d+)')
            last = ''
            for chunk in iter(lambda: process.stdout.read(256), ''):
                for piece in chunk.replace('\r', '\n').splitlines():
                    match = pattern.search(piece)
                    if match:
                        done, total = int(match.group(1)), int(match.group(2))
                        if total:
                            # 0.10 .. 0.80 of the whole job
                            self.progress = 0.10 + 0.70 * done / total
                    if piece.strip():
                        last = piece.strip()
            process.wait()
            if process.returncode != 0:
                raise JobError('estimation failed: %s' % last[-200:])
        if not self.params_path.exists():
            raise JobError('estimation produced no parameters')
        self._say('估計完成')

    def _export(self):
        self._set('export', 0.84)
        info = self._docker('clip_tools.py', 'export', '/work/mhr_params.pt',
                            '/work/keypoints.npz', timeout=600)
        self._say('已匯出 %d 幀、每幀 %d 個關鍵點'
                  % (info['frames'], info['keypoints']))

    def _overlay(self):
        self._set('overlay', 0.90)
        try:
            self._docker('clip_tools.py', 'overlay', '/work/clip.mp4',
                         '/work/keypoints.npz', '/work/overlay.png',
                         timeout=600)
            self._say('疊圖已產生，請確認骨架有貼在人身上')
        except JobError as exc:
            # A missing picture is not worth losing the score over.
            self._say('略過疊圖：%s' % exc)

    def _infer_trim(self):
        """The trim of a job recorded before the trim was written down.

        Counted from the keypoints that exist rather than from max_frames,
        which jobs older than that field do not carry and which then defaults
        to a cap they were never run under. Reading the real count instead got
        this right on a recording the cap-based guess had called short.
        """
        total = int(self.source_info.get('frames') or 0)
        used = len(self._frames) if self._frames is not None else 0
        if not total or not used or used >= total:
            return None
        fps = self.source_info.get('fps') or 30.0
        return {'used': used, 'available': total,
                'dropped_s': round((total - used) / fps, 1)}

    def framing_report(self):
        """Was this recording framed for the job, judged from the estimate.

        The estimator always returns a whole body, including the parts it never
        saw, and it does so without complaint. That is the one failure that
        produces a plausible-looking score from a useless recording, so it is
        worth a verdict rather than leaving it to whoever looks at the overlay.

        The tell is where the estimated joints land relative to the picture: a
        nose above the top edge in most frames means the head was cropped and
        the shoulder line is a guess, and a body spanning a tenth of the height
        means there were not enough pixels to place the elbows.
        """
        if self._keypoints_2d is None:
            return None
        width = float(self.source_info.get('width') or 0)
        height = float(self.source_info.get('height') or 0)
        if not width or not height:
            return None

        points = self._keypoints_2d
        x, y = points[:, :, 0], points[:, :, 1]
        inside = (x >= 0) & (x < width) & (y >= 0) & (y < height)

        nose_seen = float(inside[:, KP_NOSE].mean())
        upper_seen = float(inside[:, list(KP_UPPER)].mean())
        top = np.minimum(y[:, KP_NOSE], y[:, list(KP_SHOULDERS)].min(axis=1))
        bottom = y[:, list(KP_HIPS)].max(axis=1)
        extent = float(np.median(bottom - top) / height)

        if nose_seen < 0.5:
            verdict, note = 'head-cropped', (
                'the head is outside the frame in most frames, so the '
                'shoulder line the arm directions are measured against is '
                'extrapolated rather than seen')
        elif upper_seen < 0.7:
            verdict, note = 'partly-cropped', (
                'a third of the upper body falls outside the frame; the '
                'symbols for whichever limb is cut off are guesses')
        elif extent < 0.25:
            verdict, note = 'too-far', (
                'the body spans %d%% of the frame height, which leaves too '
                'few pixels to place the elbows and wrists' % (extent * 100))
        elif extent > 1.4:
            verdict, note = 'too-close', (
                'the body is larger than the frame, so it is being cropped')
        else:
            verdict, note = 'ok', (
                'head to hips in frame, spanning %d%% of its height'
                % (extent * 100))

        return {'verdict': verdict, 'note': note,
                'nose_visible': round(nose_seen, 3),
                'upper_body_visible': round(upper_seen, 3),
                'body_height_fraction': round(extent, 3)}

    def analyse(self, fps=None):
        """Everything the interactive part needs, computed once."""
        self._set('analyse', 0.95)
        with np.load(self.keypoints_path) as bundle:
            self._keypoints = bundle['pred_keypoints_3d'].astype(float)
            self._keypoints_2d = bundle['pred_keypoints_2d'].astype(float)
        self._framing = self.framing_report()
        if self._framing:
            self._say('取景 %s：%s' % (self._framing['verdict'],
                                          self._framing['note']))

        rate = float(fps or self.fps_override
                     or self.source_info.get('fps') or 30.0)
        self._frames = sam3d_body.frames_from_keypoints(self._keypoints,
                                                        fps=rate)
        self._times, self._symbols, _ = per_frame_symbols(self._frames)
        self._energy = energy_curve(self._times, self._frames)
        self._peaks = keyframe_indices(self._energy)
        self._say('%d 幀，%d 個能量波峰'
                  % (len(self._frames), len(self._peaks)))
        return self.analysis()

    def analysis(self):
        """The curve and the per-frame symbols, for drawing and for editing."""
        if self._symbols is None:
            raise JobError('not analysed yet')
        return {
            'times_ms': [int(t) for t in self._times],
            'energy': [round(float(e), 5) for e in self._energy],
            'peaks': [int(i) for i in self._peaks],
            'framing': self._framing,
            'symbols': [[list(limb) for limb in frame]
                        for frame in self._symbols],
        }

    # ---------------------------------------------------------------- score

    def build_score(self, name=None, keyframes=None,
                    gauss_window=DEFAULT_GAUSS_WINDOW,
                    gauss_sigma=DEFAULT_GAUSS_SIGMA, derive_head=False,
                    use_acromion=False, base_rotation='every'):
        """A score from the current recording, with whatever choices are made.

        `keyframes` overrides the peak detection entirely, which is how the
        page's click-to-add and click-to-remove works. Left out, the peaks come
        from the energy curve at the given smoothing.
        """
        if self._keypoints is None:
            raise JobError('not analysed yet')

        rate = float(self.fps_override or self.source_info.get('fps') or 30.0)
        frames = sam3d_body.frames_from_keypoints(self._keypoints, fps=rate,
                                                  use_acromion=use_acromion)
        gaze = (sam3d_body.gaze_vectors(self._keypoints) if derive_head
                else None)
        times, symbols, heads = per_frame_symbols(frames, base_rotation, gaze)

        if keyframes:
            indices = sorted({max(0, min(len(frames) - 1, int(i)))
                              for i in keyframes})
            energy = self._energy
        else:
            energy = energy_curve(times, frames, gauss_window, gauss_sigma)
            indices = keyframe_indices(energy)

        if not indices:
            raise JobError('no keyframes; try less smoothing')

        labandata = build_labandata(times, symbols, indices, heads)
        score_name = safe_name(name or self.name)
        return {
            'name': score_name,
            'score': {score_name: labandata},
            'keyframes': list(indices),
            'energy': [round(float(e), 5) for e in energy],
            'count': len(labandata),
        }

    def save_score(self, built, library=None):
        target_dir = Path(library or LIBRARY)
        target_dir.mkdir(parents=True, exist_ok=True)
        path = target_dir / ('%s.json' % built['name'])
        path.write_text(json.dumps(built['score'], indent=2),
                        encoding='utf-8')
        self._say('已存入 %s' % path)
        self.persist()
        return path


def _last_json(text):
    """clip_tools prints one json object on its last non-empty line."""
    for line in reversed((text or '').strip().splitlines()):
        line = line.strip()
        if line.startswith('{'):
            try:
                return json.loads(line)
            except ValueError:
                continue
    return {}


def main():
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)

    p = sub.add_parser('run', help='a video all the way to a saved score')
    p.add_argument('video')
    p.add_argument('--name')
    p.add_argument('--max-frames', type=int, default=DEFAULT_MAX_FRAMES)
    p.add_argument('--derive-head', action='store_true')
    p.add_argument('--save', action='store_true',
                   help='write into the gesture library')
    p.add_argument('--library')

    args = parser.parse_args()

    job = Job(name=args.name or Path(args.video).stem,
              max_frames=args.max_frames)
    print('job %s in %s' % (job.id, job.dir))
    state = job.run(args.video)
    for line in job.log:
        print('  %s' % line)
    if state['error']:
        return 1

    if state.get('framing'):
        print('  framing %s: %s' % (state['framing']['verdict'],
                                    state['framing']['note']))

    built = job.build_score(derive_head=args.derive_head)
    print('  %d keyframes at frames %s'
          % (built['count'], built['keyframes']))
    for key, frame in built['score'][built['name']].items():
        print('    %-13s t=%-6s head=%-20s re=%-18s le=%s'
              % (key, frame['start time'][0], '/'.join(frame['head']),
                 '/'.join(frame['right elbow']),
                 '/'.join(frame['left elbow'])))

    if args.save:
        print('  -> %s' % job.save_score(built, args.library))
    else:
        out = job.dir / ('%s.json' % built['name'])
        out.write_text(json.dumps(built['score'], indent=2), encoding='utf-8')
        print('  -> %s (not in the library, pass --save)' % out)
    return 0


if __name__ == '__main__':
    sys.exit(main())
