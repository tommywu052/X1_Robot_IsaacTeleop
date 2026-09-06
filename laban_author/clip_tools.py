"""The steps that need the estimator's own container: opencv, torch, matplotlib.

Kept as one script with subcommands so the authoring service invokes containers
in a single uniform way instead of embedding python into shell strings.

    python clip_tools.py probe   clip.mp4
    python clip_tools.py trim    in.mp4 out.mp4 --frames 240
    python clip_tools.py export  mhr_params.pt keypoints.npz
    python clip_tools.py overlay clip.mp4 keypoints.npz overlay.png
"""

import argparse
import json
import os
import sys

# Upper body only. These are the joints Labanotation reads plus the hips that
# anchor the torso, by their index in the module's mhr70 metadata.
BODY = {0: 'nose', 5: 'l.shoulder', 6: 'r.shoulder', 7: 'l.elbow',
        8: 'r.elbow', 62: 'l.wrist', 41: 'r.wrist', 9: 'l.hip', 10: 'r.hip'}
LINKS = ((5, 7), (7, 62), (6, 8), (8, 41), (5, 6), (5, 9), (6, 10), (9, 10))


def probe(args):
    import cv2

    capture = cv2.VideoCapture(args.video)
    if not capture.isOpened():
        raise SystemExit('cannot open %s' % args.video)
    info = {
        'width': int(capture.get(cv2.CAP_PROP_FRAME_WIDTH)),
        'height': int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT)),
        'fps': round(capture.get(cv2.CAP_PROP_FPS) or 0.0, 3),
        # Decoded rather than read from the header: estimate_mhr_params asserts
        # the bbox track length matches the source exactly, and the header
        # count is occasionally a lie.
        'frames': 0,
    }
    while capture.grab():
        info['frames'] += 1
    capture.release()
    if info['fps'] <= 0:
        info['fps'] = 30.0
    info['duration_s'] = round(info['frames'] / info['fps'], 2)
    print(json.dumps(info))


def trim(args):
    import cv2

    capture = cv2.VideoCapture(args.input)
    if not capture.isOpened():
        raise SystemExit('cannot open %s' % args.input)
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0

    writer = cv2.VideoWriter(args.output,
                             cv2.VideoWriter_fourcc(*'mp4v'),
                             fps, (width, height))
    index = written = 0
    while written < args.frames:
        ok, image = capture.read()
        if not ok:
            break
        if index >= args.start:
            writer.write(image)
            written += 1
        index += 1
    writer.release()
    capture.release()
    print(json.dumps({'frames': written, 'fps': round(fps, 3),
                      'width': width, 'height': height}))


def export(args):
    """Take the keypoints out of the torch pickle into a plain array.

    This is what lets the service re-derive a score instantly when someone
    moves a slider: everything after this point is numpy, in process, with no
    container round trip.
    """
    import numpy as np
    import torch

    params = torch.load(args.params, map_location='cpu', weights_only=False)
    missing = [k for k in ('pred_keypoints_3d', 'pred_keypoints_2d')
               if k not in params]
    if missing:
        raise SystemExit('%s has no %s' % (args.params, ', '.join(missing)))

    keypoints_3d = params['pred_keypoints_3d'].numpy()
    keypoints_2d = params['pred_keypoints_2d'].numpy()
    np.savez_compressed(args.output,
                        pred_keypoints_3d=keypoints_3d,
                        pred_keypoints_2d=keypoints_2d)
    print(json.dumps({'frames': int(keypoints_3d.shape[0]),
                      'keypoints': int(keypoints_3d.shape[1])}))


def overlay(args):
    """Draw the estimated upper body over the frames it came from.

    The one failure mode that matters in practice is a badly framed recording:
    the estimator will fit a whole body to a torso crop and produce a score
    that is wrong in a way the symbols alone do not reveal. Looking at this
    picture is how someone authoring a gesture catches that.
    """
    import cv2
    import numpy as np

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    with np.load(args.keypoints) as bundle:
        keypoints_2d = bundle['pred_keypoints_2d']

    total = keypoints_2d.shape[0]
    wanted = ([int(x) for x in args.frames.split(',')] if args.frames
              else [0, total // 3, 2 * total // 3, total - 1])
    wanted = sorted({max(0, min(total - 1, f)) for f in wanted})

    capture = cv2.VideoCapture(args.video)
    images = {}
    index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if index in wanted:
            images[index] = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        index += 1
    capture.release()

    figure, axes = plt.subplots(1, len(wanted),
                               figsize=(3.4 * len(wanted), 5.0),
                               facecolor='white', squeeze=False)
    for axis, frame in zip(axes[0], wanted):
        if frame not in images:
            axis.set_axis_off()
            continue
        axis.imshow(images[frame])
        for a, b in LINKS:
            axis.plot([keypoints_2d[frame, a, 0], keypoints_2d[frame, b, 0]],
                      [keypoints_2d[frame, a, 1], keypoints_2d[frame, b, 1]],
                      '-', lw=2.0, color='#76b900')
        axis.scatter(keypoints_2d[frame, list(BODY), 0],
                     keypoints_2d[frame, list(BODY), 1],
                     s=22, c='#ff3b30', zorder=3, linewidths=0)
        axis.set_title('frame %d' % frame, fontsize=9)
        axis.set_xticks([])
        axis.set_yticks([])

    # No title. The page says what this is, in the interface language, in the
    # paragraph directly above it; a caption drawn in here would have to be
    # English regardless, because the estimator image carries no CJK font and
    # matplotlib's default DejaVu Sans has no glyph for U+624B.
    figure.tight_layout()
    figure.savefig(args.output, dpi=90)
    print(json.dumps({'frames': wanted, 'output': args.output}))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)

    p = sub.add_parser('probe')
    p.add_argument('video')
    p.set_defaults(func=probe)

    p = sub.add_parser('trim')
    p.add_argument('input')
    p.add_argument('output')
    p.add_argument('--frames', type=int, required=True)
    p.add_argument('--start', type=int, default=0)
    p.set_defaults(func=trim)

    p = sub.add_parser('export')
    p.add_argument('params')
    p.add_argument('output')
    p.set_defaults(func=export)

    p = sub.add_parser('overlay')
    p.add_argument('video')
    p.add_argument('keypoints')
    p.add_argument('output')
    p.add_argument('--frames', help='comma separated, defaults to four spread')
    p.set_defaults(func=overlay)

    args = parser.parse_args()
    # matplotlib and torch both want a writable home, and the container runs as
    # the calling uid with no home of its own.
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/mpl')
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
