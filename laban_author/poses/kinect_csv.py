"""Read the .csv files KinectReader wrote.

This pose source exists to be checked against, not to be used: nobody is
plugging in a Kinect v2. LabanotationSuite ships six of these recordings
together with the Labanotation each one produced, so reading them is what lets
`validate.py` show that this converter agrees with the original tool before any
other camera is trusted.

A faithful port of LabanEditor's `src/kinect.py loadKinectDataFile`, minus its
dependency on LabanEditor's global `settings`.
"""

import os

import numpy as np

from skeleton import JOINT_OFFSET, bType, jType

# The original ran under Python 2, where `/` on two ints floors. The timestamps
# in the reference output are exactly 1 + 33k, which only comes out that way
# with flooring: a 33 ms frame is 330000 ticks, and 330000/10000/30 is 1.1, so
# true division would advance the clock by 36 ms per frame instead of 33.
_TICKS_PER_MS = 10000
_TARGET_FPS = 30
_FRAME_MS = 33

# Above this, the first column is a Kinect tick count rather than milliseconds.
_TICK_THRESHOLD = 10e7


def load(file_path, fill_gap=True):
    """Return a list of one-element bType arrays, one per frame.

    Gap filling defaults on because the reference scores were made with it on:
    each of the six recordings is 1 to 5 frames short of the frame count its
    score ends at, and interpolating across the dropped frames accounts for
    every one of them exactly.
    """
    frames = []
    idx = 0
    start_time = 0
    last_time = 0

    with open(file_path) as handle:
        for line in handle:
            cols = line.split(',')
            if len(cols) < 1 + 25 * 4:
                break
            current_time = int(float(cols[0]))

            frame = np.zeros(1, dtype=bType)
            frame['filled'] = False
            if idx == 0:
                frame['timeS'] = 1
                start_time = current_time
            elif current_time > _TICK_THRESHOLD:
                count = ((current_time - last_time) // _TICKS_PER_MS) // _TARGET_FPS
                if count < 1:
                    count = 1
                frame['timeS'] = frames[-1][0][0] + count * _FRAME_MS
            else:
                frame['timeS'] = current_time - start_time

            for j in range(25):
                frame[0][j + JOINT_OFFSET] = np.array(
                    [(float(cols[1 + j * 4]), float(cols[2 + j * 4]),
                      float(cols[3 + j * 4]), int(float(cols[4 + j * 4])))],
                    dtype=jType)[0]

            if fill_gap and idx > 0:
                idx = _fill(frames, frame, current_time, last_time, idx)

            frames.append(frame)
            last_time = current_time
            idx += 1

    return frames


def _fill(frames, frame, current_time, last_time, idx):
    """Insert linearly interpolated frames across a dropped-frame gap."""
    time_gap = (current_time - last_time) // _TICKS_PER_MS
    if time_gap <= 40:
        return idx

    count = int(time_gap // _TARGET_FPS)
    before = frames[-1][0]
    after = frame[0]

    for j in range(1, count):
        extra = np.zeros(1, dtype=bType)
        extra['timeS'] = 1 + _FRAME_MS * idx
        extra['filled'] = True
        for k in range(JOINT_OFFSET, JOINT_OFFSET + 25):
            gap = [after[k][axis] - before[k][axis] for axis in range(3)]
            extra[0][k] = np.array(
                [(before[k][0] + gap[0] * float(j) / float(count),
                  before[k][1] + gap[1] * float(j) / float(count),
                  before[k][2] + gap[2] * float(j) / float(count),
                  0)], dtype=jType)[0]
        frames.append(extra)
        idx += 1

    return idx


def name_of(file_path):
    return os.path.splitext(os.path.basename(file_path))[0]
