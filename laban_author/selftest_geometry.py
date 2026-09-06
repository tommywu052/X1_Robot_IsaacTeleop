"""Check the coordinate handling with poses whose answer is known in advance.

`validate.py` proves the conversion reproduces the original tool, but it only
ever feeds it Kinect recordings. Pointing a different camera at the problem
rests on one claim that no recording tests directly:

    calculate_base_rotation derives the body frame as a cross product of two
    body vectors, so it lands on the same (forward, left, up) basis for any
    right-handed source coordinate system. No axis remapping is needed.

So build bodies with limbs aimed at each of the 27 Labanotation directions, in
each camera's convention, and check the symbols come back as aimed. If the
handedness reasoning is wrong, forward and backward swap and this fails
loudly instead of quietly producing plausible nonsense.

    python selftest_geometry.py
"""

import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from convert import per_frame_symbols
from skeleton import TRACKED, bType, jType

# Each convention as (left, up, forward) expressed in that camera's own axes.
# Kinect was measured off the recordings in testdata: the subject's left is at
# smaller x, up is larger y, and depth grows away from the sensor, so with the
# subject facing the sensor forward is -z.
# SAM3D-Body was measured off mhr_params.pt: the subject's left is at larger x,
# y grows downward, depth grows away from the camera.
CONVENTIONS = {
    'kinect': (np.array([-1.0, 0, 0]), np.array([0, 1.0, 0]),
               np.array([0, 0, -1.0])),
    'sam3d_body': (np.array([1.0, 0, 0]), np.array([0, -1.0, 0]),
                   np.array([0, 0, -1.0])),
}

# The centre of each azimuth bucket in coordinate2laban, in degrees.
DIRECTIONS = {
    'Forward': 0.0, 'Left Forward': 45.0, 'Left': 90.0,
    'Left Backward': 135.0, 'Backward': 180.0, 'Right Backward': -135.0,
    'Right': -90.0, 'Right Forward': -45.0,
}

# The centre of each zenith band. Place is the degenerate straight-up and
# straight-down case, which the azimuth cannot describe.
LEVELS = {'High': 45.0, 'Normal': 90.0, 'Low': 135.0}
PLACE = {'High': 5.0, 'Low': 175.0}

SHOULDER_HALF_WIDTH = 0.18
SPINE_DROP = 0.35
UPPER_ARM = 0.28
FOREARM = 0.25


def unit_in_body_frame(theta_deg, phi_deg):
    """The direction to_sphere would report as (theta, phi).

    to_sphere takes the zenith from the body frame's z, which is up, and the
    azimuth in the x-y plane from x, which is forward, toward y, which is the
    subject's left.
    """
    theta = math.radians(theta_deg)
    phi = math.radians(phi_deg)
    return np.array([math.sin(theta) * math.cos(phi),
                     math.sin(theta) * math.sin(phi),
                     math.cos(theta)])


def build_frame(convention, theta_deg, phi_deg):
    """A body whose arms both point along one direction, in one camera's axes."""
    left, up, forward = CONVENTIONS[convention]

    def to_camera(body_vector):
        return (forward * body_vector[0] + left * body_vector[1]
                + up * body_vector[2])

    aim = to_camera(unit_in_body_frame(theta_deg, phi_deg))

    shoulder_l = left * SHOULDER_HALF_WIDTH
    shoulder_r = -left * SHOULDER_HALF_WIDTH
    spine_m = -up * SPINE_DROP

    frame = np.zeros(1, dtype=bType)
    frame['timeS'] = 1

    joints = {
        'shoulderL': shoulder_l,
        'shoulderR': shoulder_r,
        'spineM': spine_m,
        'elbowL': shoulder_l + aim * UPPER_ARM,
        'elbowR': shoulder_r + aim * UPPER_ARM,
    }
    joints['wristL'] = joints['elbowL'] + aim * FOREARM
    joints['wristR'] = joints['elbowR'] + aim * FOREARM

    for name, xyz in joints.items():
        frame[0][name] = np.array([(xyz[0], xyz[1], xyz[2], TRACKED)],
                                  dtype=jType)[0]
    return frame


def cases():
    for level, theta in LEVELS.items():
        for direction, phi in DIRECTIONS.items():
            yield direction, level, theta, phi
    for level, theta in PLACE.items():
        yield 'Place', level, theta, 0.0


def main():
    failures = []
    per_convention = {}

    for convention in CONVENTIONS:
        results = {}
        for direction, level, theta, phi in cases():
            frame = build_frame(convention, theta, phi)
            _, symbols, _ = per_frame_symbols([frame])
            # All four limbs point the same way, so all four must agree.
            got = symbols[0]
            results[(direction, level)] = got
            for limb_symbol in got:
                if limb_symbol != [direction, level]:
                    failures.append('%s: aimed at %s/%s, got %s/%s'
                                    % (convention, direction, level,
                                       limb_symbol[0], limb_symbol[1]))
        per_convention[convention] = results

    total = len(list(cases()))
    print('aimed limbs at %d directions in %d conventions'
          % (total, len(CONVENTIONS)))
    for convention in CONVENTIONS:
        wrong = len([f for f in failures if f.startswith(convention)])
        print('  %-12s %d of %d recovered'
              % (convention, total - wrong // 4, total))

    names = list(CONVENTIONS)
    disagreements = [key for key in per_convention[names[0]]
                     if per_convention[names[0]][key]
                     != per_convention[names[1]][key]]
    print('  %s and %s agree on all %d: %s'
          % (names[0], names[1], total, not disagreements))

    if failures:
        print()
        for line in failures[:12]:
            print('  %s' % line)
        if len(failures) > 12:
            print('  ... %d more' % (len(failures) - 12))
        return 1

    print()
    print('the body frame is recovered from either camera without remapping')
    return 0


if __name__ == '__main__':
    sys.exit(main())
