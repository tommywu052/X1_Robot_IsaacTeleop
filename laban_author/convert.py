"""Joint frames to a Labanotation score, headless.

This is the numeric core of LabanEditor's `algtotal.py` with the matplotlib
axes and the global `settings.application` taken out, so it can run in a
pipeline instead of under a GUI. The arithmetic is unchanged, down to two
upstream oddities that are called out where they happen: dropping them would
produce different keyframes than the tool everyone's existing gestures came
from.

    frames  ->  a direction+level symbol per limb per frame   (labanProcessor)
            ->  a wrist-motion energy curve                   (kp_extractor)
            ->  keyframes at the peaks of that curve
            ->  the "PositionN" json the player already reads
"""

import argparse
import json
import os
import sys
from collections import OrderedDict

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import vendor  # noqa: F401  (puts the vendored directory on sys.path)
import accessory as ac
import kp_extractor as kpex
import labanProcessor as lp
import wavfilter as wf

# LabanEditor's defaults, from algtotal.Algorithm.
DEFAULT_GAUSS_WINDOW = 31
DEFAULT_GAUSS_SIGMA = 5

# The score is written for these four limb segments, in this order.
LIMBS = ('right elbow', 'right wrist', 'left elbow', 'left wrist')


def per_frame_symbols(frames, base_rotation_style='every', gaze=None):
    """A Labanotation symbol for each limb of each frame.

    `base_rotation_style` picks the frame of reference the limb directions are
    measured in. 'every' re-derives it from each frame's own shoulders and
    spine, so the symbols describe the gesture relative to the body wherever
    the body has turned to. 'first' locks it to the opening frame, which keeps
    a turn of the torso visible in the symbols instead of cancelling it out.

    `gaze` is an optional per-frame facing direction in the source coordinates.
    Given one, a head symbol comes back alongside the limbs; see head_symbols.
    """
    count = len(frames)
    base_rotation = None
    if base_rotation_style == 'first':
        base_rotation = lp.calculate_base_rotation(frames[0])

    times = np.zeros(count)
    elbow_r = np.zeros((count, 3))
    elbow_l = np.zeros((count, 3))
    wrist_r = np.zeros((count, 3))
    wrist_l = np.zeros((count, 3))
    heads = [] if gaze is not None else None

    for i in range(count):
        if base_rotation_style == 'every':
            base_rotation = lp.calculate_base_rotation(frames[i])
        times[i] = frames[i]['timeS'][0]
        (elbow_r[i], elbow_l[i], wrist_r[i], wrist_l[i]) = lp.raw2sphere(
            frames[i], base_rotation=base_rotation)
        if heads is not None:
            _, theta, phi = lp.to_sphere(np.dot(base_rotation.T, gaze[i]))
            heads.append(lp.coordinate2laban(theta, phi))

    # to_sphere returns (r, theta, phi); the symbol only needs the two angles.
    symbols = []
    for i in range(count):
        symbols.append([
            lp.coordinate2laban(elbow_r[i][1], elbow_r[i][2]),
            lp.coordinate2laban(wrist_r[i][1], wrist_r[i][2]),
            lp.coordinate2laban(elbow_l[i][1], elbow_l[i][2]),
            lp.coordinate2laban(wrist_l[i][1], wrist_l[i][2]),
        ])

    return times, symbols, heads


def energy_curve(times, frames, gauss_window=DEFAULT_GAUSS_WINDOW,
                 gauss_sigma=DEFAULT_GAUSS_SIGMA):
    """Wrist motion reduced to one number per frame.

    High where the hands are accelerating, low where they are travelling
    steadily, so its peaks land on the moments a gesture changes shape.
    """
    count = len(frames)
    hand_r = np.zeros((count, 3))
    hand_l = np.zeros((count, 3))
    for i in range(count):
        for axis, key in enumerate(('x', 'y', 'z')):
            hand_r[i][axis] = frames[i]['wristR'][key][0]
            hand_l[i][axis] = frames[i]['wristL'][key][0]

    gauss = wf.gaussFilter(gauss_window, gauss_sigma)
    hand_rf = wf.calcFilter(hand_r, gauss)
    hand_lf = wf.calcFilter(hand_l, gauss)

    vel_r = ac.vel(times, hand_rf)
    vel_l = ac.vel(times, hand_lf)
    acc_r = ac.acc(times, vel_r)
    acc_l = ac.acc(times, vel_l)

    return kpex.energy_function_ijcv(v_l=vel_l, a_l=acc_l,
                                     v_r=vel_r, a_r=acc_r)


def keyframe_indices(energy):
    return kpex.gaussian_pecdec(energy)


def _keyframe(time, duration, symbol, head=None):
    data = OrderedDict()
    data["start time"] = [str(time)]
    data["duration"] = [str(duration)]
    # Upstream never derived the head from the recording and always wrote
    # Forward/Normal. A derived symbol replaces it only when one was asked for;
    # a level gaze quantises to Forward/Normal anyway, so this widens the
    # original behaviour rather than contradicting it.
    data["head"] = list(head) if head else ['Forward', 'Normal']
    for limb, laban in zip(LIMBS, symbol):
        data[limb] = [laban[0], laban[1]]
    # Torso rotation, which no pose source here measures. The player reads it
    # as extra head yaw; leaving it at zero leaves the head symbol in charge.
    data["rotation"] = ['ToLeft', '0']
    return data


def build_labandata(times, symbols, indices, heads=None):
    """The "PositionN" mapping, keys and all, as LabanEditor emitted it.

    Two things here look like bugs and are kept anyway, because the reference
    scores in the gesture library were produced with them:

    - The last entry is keyed by frame number, not by keyframe number, which is
      why a score with nine keyframes ends at "Position155".
    - When the first peak is not frame 0, an opening keyframe is written and
      then immediately overwritten, because the key is built from the loop
      counter while the lookup uses a separate running index. The opening pose
      is lost and the first peak takes its place.
    """
    labandata = OrderedDict()
    positions = []
    count = len(indices)
    if count == 0:
        return labandata

    def head_at(frame):
        return heads[frame] if heads else None

    idx = 0
    for i in range(count):
        j = indices[i]

        if i == 0 and j != i:
            positions.append("Position" + str(i))
            labandata[positions[idx]] = _keyframe(
                int(times[i]), 1, symbols[i], head_at(i))
            idx += 1

        # Compares a frame number against a keyframe count, so in practice only
        # the appended final keyframe ever gets -1.
        duration = '-1' if j == (count - 1) else '1'

        positions.append("Position" + str(i))
        labandata[positions[idx]] = _keyframe(
            int(times[j]), duration, symbols[j], head_at(j))
        idx += 1

    last = len(symbols) - 1
    if indices[count - 1] != last:
        positions.append("Position" + str(last))
        labandata[positions[idx]] = _keyframe(
            int(times[last]), '-1', symbols[last], head_at(last))

    return labandata


def convert(frames, name, base_rotation_style='every',
            gauss_window=DEFAULT_GAUSS_WINDOW,
            gauss_sigma=DEFAULT_GAUSS_SIGMA, gaze=None):
    """Joint frames to the score dict, ready for json.dump."""
    times, symbols, heads = per_frame_symbols(frames, base_rotation_style,
                                              gaze)
    energy = energy_curve(times, frames, gauss_window, gauss_sigma)
    indices = keyframe_indices(energy)
    labandata = build_labandata(times, symbols, indices, heads)
    return OrderedDict([(name, labandata)]), indices, energy


def _load(path, source, fps, derive_head, use_acromion):
    """Frames, a name, and the gaze directions if the source can supply them."""
    if source == 'auto':
        source = 'kinect_csv' if path.endswith('.csv') else 'sam3d_body'

    if source == 'kinect_csv':
        from poses import kinect_csv
        if derive_head:
            raise SystemExit('--derive-head needs a pose source with a face; '
                             'the Kinect recordings carry no ear keypoints')
        return kinect_csv.load(path), kinect_csv.name_of(path), None

    from poses import sam3d_body
    keypoints = sam3d_body.read_keypoints(path)
    frames = sam3d_body.frames_from_keypoints(keypoints, fps=fps,
                                              use_acromion=use_acromion)
    gaze = sam3d_body.gaze_vectors(keypoints) if derive_head else None
    return frames, sam3d_body.name_of(path), gaze


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', help='a Kinect .csv or a SAM3D-Body .pt')
    parser.add_argument('-o', '--output', help='where to write the score')
    parser.add_argument('--source', default='auto',
                        choices=('auto', 'kinect_csv', 'sam3d_body'))
    parser.add_argument('--name', help='score name, defaults to the file stem')
    parser.add_argument('--base-rotation', default='every',
                        choices=('every', 'first'))
    parser.add_argument('--gauss-window', type=int,
                        default=DEFAULT_GAUSS_WINDOW)
    parser.add_argument('--gauss-sigma', type=float,
                        default=DEFAULT_GAUSS_SIGMA)
    parser.add_argument('--fps', type=float, default=30.0,
                        help='frame rate of the recording, for the time base')
    parser.add_argument('--derive-head', action='store_true',
                        help='read the head symbol off the face instead of '
                             'writing Forward/Normal as upstream did')
    parser.add_argument('--acromion', action='store_true',
                        help='take the upper arm from the bony shoulder tip')
    args = parser.parse_args()

    frames, stem, gaze = _load(args.input, args.source, args.fps,
                               args.derive_head, args.acromion)
    if not frames:
        parser.error('no frames read from %s' % args.input)

    score, indices, _ = convert(frames, args.name or stem,
                                args.base_rotation,
                                args.gauss_window, args.gauss_sigma, gaze)

    text = json.dumps(score, indent=2)
    if args.output:
        with open(args.output, 'w') as handle:
            handle.write(text)
        name = args.name or stem
        print('%s: %d frames, %d peaks, %d keyframes -> %s'
              % (name, len(frames), len(indices), len(score[name]),
                 args.output))
    else:
        print(text)


if __name__ == '__main__':
    main()
