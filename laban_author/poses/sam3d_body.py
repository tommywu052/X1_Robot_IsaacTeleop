"""Read SAM3D-Body's MHR parameters as joint frames.

This is the pose source that replaces the Kinect. V2D's `v2d_sam3d_body` module
estimates a parametric body per frame of ordinary video and writes, among other
things, `pred_keypoints_3d` -- 70 3D keypoints, which is a superset of the 25
Kinect v2 gave us for the joints Labanotation actually reads.

Two facts make the handoff work, and both were measured rather than assumed
(see README):

- Its axes are x toward the subject's left, y down, z away from the camera.
  That is a right-handed frame, and `calculate_base_rotation` derives the body
  frame from a cross product of two body vectors, so any right-handed source
  lands on the same (forward, left, up) basis. No axis remapping is needed.
- Scale is irrelevant. The symbols come from angles, and the energy curve
  normalises per axis, so metres versus millimetres changes nothing.

The keypoint order is close to COCO-WholeBody but is not COCO-17: the hips are
at 9 and 10 rather than 11 and 12, and the wrists sit inside the hand blocks at
41 and 62. The indices below come from the module's own `mhr70.py` metadata.
"""

import os

import numpy as np

from skeleton import TRACKED, bType, jType

# From sam_3d_body/metadata/mhr70.py, pose_info['keypoint_info'].
MHR70 = {
    'nose': 0, 'left_eye': 1, 'right_eye': 2, 'left_ear': 3, 'right_ear': 4,
    'left_shoulder': 5, 'right_shoulder': 6,
    'left_elbow': 7, 'right_elbow': 8,
    'left_hip': 9, 'right_hip': 10,
    'left_knee': 11, 'right_knee': 12,
    'left_ankle': 13, 'right_ankle': 14,
    'left_big_toe': 15, 'right_big_toe': 18,
    'right_thumb4': 21, 'right_forefinger4': 25, 'right_middle_finger4': 29,
    'right_middle_finger_third_joint': 32, 'right_wrist': 41,
    'left_thumb4': 42, 'left_forefinger4': 46, 'left_middle_finger4': 50,
    'left_middle_finger_third_joint': 53, 'left_wrist': 62,
    'left_olecranon': 63, 'right_olecranon': 64,
    'left_cubital_fossa': 65, 'right_cubital_fossa': 66,
    'left_acromion': 67, 'right_acromion': 68,
    'neck': 69,
}

# Kinect joint name -> the MHR keypoint standing in for it. Only the seven the
# algorithm reads have to be right; the rest are carried so that viewers and
# the energy curve see a whole body.
DIRECT = {
    'neck': 'neck',
    'shoulderL': 'left_shoulder', 'shoulderR': 'right_shoulder',
    'elbowL': 'left_elbow', 'elbowR': 'right_elbow',
    'wristL': 'left_wrist', 'wristR': 'right_wrist',
    'hipL': 'left_hip', 'hipR': 'right_hip',
    'kneeL': 'left_knee', 'kneeR': 'right_knee',
    'ankleL': 'left_ankle', 'ankleR': 'right_ankle',
    'footL': 'left_big_toe', 'footR': 'right_big_toe',
    # Kinect's "hand" is the palm and "handT" the fingertip.
    'handL': 'left_middle_finger_third_joint',
    'handR': 'right_middle_finger_third_joint',
    'handTL': 'left_middle_finger4', 'handTR': 'right_middle_finger4',
    'thumbL': 'left_thumb4', 'thumbR': 'right_thumb4',
}

# The acromion is the bony tip of the shoulder, a cleaner landmark for the
# direction of the upper arm than the soft-tissue shoulder centre. Selectable
# because it moves the symbol boundaries slightly and Kinect had no equivalent.
ACROMION = {'shoulderL': 'left_acromion', 'shoulderR': 'right_acromion'}

DEFAULT_FPS = 30.0


def load(path, fps=DEFAULT_FPS, use_acromion=False):
    """Return Kinect-shaped frames from an mhr_params .pt or an exported .npz."""
    keypoints = read_keypoints(path)
    return frames_from_keypoints(keypoints, fps=fps, use_acromion=use_acromion)


def read_keypoints(path):
    """The (frames, 70, 3) array, from whichever container it is in.

    `.pt` is what the module writes and needs torch to unpickle. `.npz` and
    `.npy` are what `export_keypoints.py` writes so that the conversion can run
    somewhere without torch installed.
    """
    if path.endswith('.npy'):
        return np.asarray(np.load(path), dtype=float)

    if path.endswith('.npz'):
        with np.load(path) as bundle:
            return np.asarray(bundle['pred_keypoints_3d'], dtype=float)

    try:
        import torch
    except ImportError:
        raise RuntimeError(
            'reading %s needs torch; run export_keypoints.py where torch is '
            'installed and pass the .npz it writes instead' % path)

    params = torch.load(path, map_location='cpu', weights_only=False)
    if 'pred_keypoints_3d' not in params:
        raise RuntimeError('%s has no pred_keypoints_3d, only %s'
                           % (path, sorted(params)))
    return params['pred_keypoints_3d'].numpy().astype(float)


def frames_from_keypoints(keypoints, fps=DEFAULT_FPS, use_acromion=False):
    keypoints = np.asarray(keypoints, dtype=float)
    if keypoints.ndim != 3 or keypoints.shape[2] != 3:
        raise ValueError('expected (frames, keypoints, 3), got %s'
                         % (keypoints.shape,))
    if keypoints.shape[1] < 70:
        raise ValueError('expected the 70-keypoint MHR set, got %d'
                         % keypoints.shape[1])

    mapping = dict(DIRECT)
    if use_acromion:
        mapping.update(ACROMION)

    # A uniform integer millisecond grid, the same shape of time base the
    # Kinect reader produces, so downstream velocity and acceleration behave
    # identically.
    step = int(round(1000.0 / fps))

    frames = []
    for i in range(keypoints.shape[0]):
        frame = np.zeros(1, dtype=bType)
        frame['timeS'] = 1 + i * step
        frame['filled'] = False

        for kinect_name, mhr_name in mapping.items():
            _set(frame, kinect_name, keypoints[i, MHR70[mhr_name]])

        # Kinect had a spine and a head centre; MHR does not, so derive them.
        shoulder_mid = _mid(keypoints[i], 'left_shoulder', 'right_shoulder')
        hip_mid = _mid(keypoints[i], 'left_hip', 'right_hip')
        _set(frame, 'spineS', shoulder_mid)
        _set(frame, 'spineB', hip_mid)
        # spineM only ever reaches the algorithm through the base rotation,
        # which uses the component of (spineM - shoulderR) perpendicular to the
        # shoulder line. Any point on the mid-sagittal plane below the
        # shoulders gives the same body frame, so the midpoint is enough.
        _set(frame, 'spineM', (shoulder_mid + hip_mid) / 2.0)
        _set(frame, 'head', _mid(keypoints[i], 'left_ear', 'right_ear'))

        frames.append(frame)

    return frames


def gaze_vectors(keypoints):
    """Where the face points, per frame, in the source coordinates.

    Out of the face: from the midpoint of the ears to the nose. Used for the
    head symbol, which upstream never derived and always wrote as Forward.
    """
    keypoints = np.asarray(keypoints, dtype=float)
    nose = keypoints[:, MHR70['nose']]
    ears = (keypoints[:, MHR70['left_ear']]
            + keypoints[:, MHR70['right_ear']]) / 2.0
    return nose - ears


def _mid(frame_keypoints, left, right):
    return (frame_keypoints[MHR70[left]] + frame_keypoints[MHR70[right]]) / 2.0


def _set(frame, name, xyz, ts=TRACKED):
    frame[0][name] = np.array([(xyz[0], xyz[1], xyz[2], ts)], dtype=jType)[0]


def name_of(path):
    stem = os.path.splitext(os.path.basename(path))[0]
    # mhr_params.pt says nothing about which gesture it is; the directory does.
    if stem in ('mhr_params', 'keypoints_3d'):
        parent = os.path.basename(os.path.dirname(os.path.abspath(path)))
        return parent or stem
    return stem
