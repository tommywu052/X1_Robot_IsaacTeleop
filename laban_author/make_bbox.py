"""Write the bbox track SAM3D-Body insists on.

`estimate_mhr_params` refuses to run without either a bbox track or a SAM2
mask, even though its own argument parser calls both optional. For authoring a
gesture that requirement is busywork: the performer is standing in front of the
camera filling the frame, so the box is the frame. Producing it here avoids
pulling SAM2 and GroundingDINO into a path that does not need them.

Give `--box` instead when the performer is off to one side, or when something
else in shot is being picked up as the person.

Needs torch, so run it where torch is -- inside v2d_sam3d_body, or in the
reconstruction venv.

    python make_bbox.py clip.mp4 -o bbox_track.pt
    python make_bbox.py clip.mp4 -o bbox_track.pt --box 420 60 1500 1040
"""

import argparse
import os


def frame_count_and_size(path):
    import cv2

    capture = cv2.VideoCapture(path)
    if not capture.isOpened():
        raise SystemExit('cannot open %s' % path)
    # The container's frame count is a header value and is occasionally a lie,
    # and estimate_mhr_params asserts the track length matches exactly, so
    # count by decoding.
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frames = 0
    while capture.grab():
        frames += 1
    capture.release()
    if not frames:
        raise SystemExit('%s decoded to zero frames' % path)
    return frames, width, height


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('video')
    parser.add_argument('-o', '--output', required=True)
    parser.add_argument('--box', nargs=4, type=float,
                        metavar=('X0', 'Y0', 'X1', 'Y1'),
                        help='one box for every frame, defaults to the frame')
    parser.add_argument('--inset', type=float, default=0.0,
                        help='shrink the full-frame box by this fraction on '
                             'each side, for footage with clutter at the edges')
    args = parser.parse_args()

    import numpy as np
    import torch

    frames, width, height = frame_count_and_size(args.video)

    if args.box:
        box = np.asarray(args.box, dtype=np.float32)
    else:
        pad_x = width * args.inset
        pad_y = height * args.inset
        box = np.asarray([pad_x, pad_y, width - pad_x, height - pad_y],
                         dtype=np.float32)

    track = np.repeat(box[None, :], frames, axis=0)
    torch.save({'bbox_track': torch.from_numpy(track)}, args.output)

    print('%s: %d frames %dx%d, box [%.0f %.0f %.0f %.0f] -> %s'
          % (os.path.basename(args.video), frames, width, height,
             box[0], box[1], box[2], box[3], args.output))


if __name__ == '__main__':
    main()
