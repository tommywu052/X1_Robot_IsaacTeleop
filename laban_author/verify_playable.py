"""Check that an authored score survives the trip to X1 joint angles.

A score can be valid Labanotation and still be useless: the player has to
parse it, the retarget has to produce joint angles inside the arm and head
limits, and the safety policy has to have an opinion about it. This runs a
score through the same modules the player uses and reports what comes out,
without commanding the robot.

Reads `laban/` next door, so it wants to run from a checkout that has both.

    python verify_playable.py testdata/ego_clip_head.json
    python verify_playable.py testdata/reference/Ges01_wavehand.total.json
"""

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
# x1_mapper does "from decoder import LimbSpherical", so laban has to be on the
# path as a directory of modules rather than imported as a package.
#
# Both layouts this tool lives in are handled the same way as the pipeline does
# it, but by hand: importing pipeline here would pull in numpy and the whole
# estimator path just to read a JSON file, and this script is the one thing
# that should still run on a machine with none of that installed.
def _find_laban():
    override = os.environ.get('X1_LABAN_DIR')
    if override:
        return os.path.abspath(override)
    for path in (os.path.join(HERE, '..', 'laban'),
                 os.path.join(HERE, '..', 'teleop_share', 'laban')):
        path = os.path.normpath(path)
        if os.path.isfile(os.path.join(path, 'x1_mapper.py')):
            return path
    return os.path.normpath(os.path.join(HERE, '..', 'laban'))


LABAN = _find_laban()


def _load_player_modules():
    if not os.path.isdir(LABAN):
        raise SystemExit('no laban directory at %s' % LABAN)
    sys.path.insert(0, LABAN)
    import decoder
    import x1_mapper
    return decoder, x1_mapper


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('score')
    parser.add_argument('--profile', default='chat',
                        choices=('chat', 'aspire'))
    args = parser.parse_args()

    decoder, x1_mapper = _load_player_modules()

    gesture = decoder.load_gesture(args.score)
    print('%s: %d keyframes, %.1f s'
          % (gesture.name, len(gesture.keyframes),
             gesture.duration_ms / 1000.0))
    print()

    limits = x1_mapper.HEAD_LIMITS
    lo, hi = x1_mapper._JOINT_CLAMP
    saturated_arm = 0
    saturated_head = 0

    print('%-12s %7s  %-38s %-38s %s'
          % ('keyframe', 'time', 'left arm j_1..j_6', 'right arm j_51..j_56',
             'head'))
    print('-' * 116)

    for frame in gesture.keyframes:
        left, right = x1_mapper.map_limbs_to_arms(frame.limbs)
        head = x1_mapper.map_head(frame.head, frame.rotation)

        for value in list(left) + list(right):
            if abs(abs(value) - max(abs(lo), abs(hi))) < 1e-9:
                saturated_arm += 1
        for name, value in zip(x1_mapper.HEAD_NAMES, head):
            bound_lo, bound_hi = limits[name]
            if abs(value - bound_lo) < 1e-9 or abs(value - bound_hi) < 1e-9:
                saturated_head += 1

        print('%-12s %6.0fms  %-38s %-38s %s'
              % (frame.name, frame.time_ms,
                 ' '.join('%+.2f' % v for v in left),
                 ' '.join('%+.2f' % v for v in right),
                 ' '.join('%+.2f' % v for v in head)))

    print('-' * 116)
    print('arm joints at the +/-%.1f clamp: %d' % (hi, saturated_arm))
    print('head joints at their limit:     %d  (limits %s)'
          % (saturated_head,
             ', '.join('%s %+.2f..%+.2f' % (n, limits[n][0], limits[n][1])
                       for n in x1_mapper.HEAD_NAMES)))

    stem = os.path.splitext(os.path.basename(args.score))[0]
    stem = stem[:-6] if stem.endswith('.total') else stem
    _report_policy(stem, args.profile)
    return 0


def _report_policy(stem, profile):
    try:
        import gesture_policy
    except ImportError as exc:
        print('policy: not checked (%s)' % exc)
        return

    policy = gesture_policy.default_policy(profile=profile)
    tier = policy.tier(stem)
    allowed = policy.allowed(stem)
    print()
    print('policy:  %r is tier %r, %s under profile %r'
          % (stem, tier, 'allowed' if allowed else 'blocked', profile))
    print('         %s' % policy.reason(stem))
    if tier == 'unknown':
        # Worth saying plainly: a newly authored gesture has never been
        # measured against the collision meshes, so it is playable while a
        # human is watching and excluded from anything choosing on its own.
        print('         a new gesture has no measured clearance yet; measure '
              'it before letting an agent pick it')


if __name__ == '__main__':
    sys.exit(main())
