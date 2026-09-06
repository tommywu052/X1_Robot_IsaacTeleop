"""The joint record the Labanotation algorithm reads.

Every pose source in this tool fills this same structure, whatever camera it
came from. That is the whole trick: the vendored algorithm was written against
Kinect's 25-joint body, so a pose source that produces a Kinect-shaped record
can drive it without the algorithm knowing a Kinect was never involved.
"""

import numpy as np

# a joint point, 'ts' stands for tracking status
jType = np.dtype({'names': ['x', 'y', 'z', 'ts'],
                  'formats': [float, float, float, int]})

# Field order matches Kinect v2's JointType enum, which is also the column order
# in the .csv files KinectReader wrote, so a reader can assign positionally.
JOINT_NAMES = (
    'spineB', 'spineM',
    'neck', 'head',
    'shoulderL', 'elbowL', 'wristL', 'handL',
    'shoulderR', 'elbowR', 'wristR', 'handR',
    'hipL', 'kneeL', 'ankleL', 'footL',
    'hipR', 'kneeR', 'ankleR', 'footR',
    'spineS', 'handTL', 'thumbL', 'handTR', 'thumbR',
)

# a body
bType = np.dtype({
    'names': ('timeS', 'filled') + JOINT_NAMES,
    'formats': [int, bool] + [jType] * len(JOINT_NAMES),
})

# Where the joints live once 'timeS' and 'filled' are in front of them.
JOINT_OFFSET = 2

# The algorithm reads exactly these. Everything else in the record is carried
# along for viewers and for the energy curve, or is simply unused.
REQUIRED = ('shoulderL', 'shoulderR', 'spineM', 'elbowL', 'elbowR',
            'wristL', 'wristR')

TRACKED = 2
INFERRED = 1
NOT_TRACKED = 0


def empty_frame(time_ms=0):
    """One body record, all joints at the origin and untracked."""
    frame = np.zeros(1, dtype=bType)
    frame['timeS'] = time_ms
    frame['filled'] = False
    return frame


def set_joint(frame, name, xyz, ts=TRACKED):
    frame[0][name] = np.array([(xyz[0], xyz[1], xyz[2], ts)], dtype=jType)[0]


def get_joint(frame, name):
    j = frame[0][name]
    return np.array([j['x'], j['y'], j['z']], dtype=float)
