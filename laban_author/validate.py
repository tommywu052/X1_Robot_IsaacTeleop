"""Check this converter against the scores the original tool produced.

LabanotationSuite ships six Kinect recordings, and the gesture library holds
the Labanotation LabanEditor produced from each one. Reading the recordings
back and comparing gives a pass/fail on the whole chain -- the reader, the
vendored algorithm, the keyframe selection and the json shape -- without
needing a Kinect or the original GUI.

This has to pass before any other camera is worth wiring up. Until it does, a
disagreement between a new pose source and the reference scores cannot be told
apart from a mistake in the conversion.

Two checks, because they fail for different reasons:

  symbols    At each keyframe time the reference names, is our symbol for all
             four limbs the same? This tests the pose-to-Labanotation core on
             its own, independently of which frames got picked.

  score      Is the whole emitted score identical -- same keys, same times,
             same symbols? This additionally tests the energy curve and the
             peak detection that choose the keyframes.

    python validate.py
    python validate.py --verbose
"""

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from convert import LIMBS, convert, per_frame_symbols
from poses import kinect_csv

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_INPUTS = os.path.join(HERE, 'testdata', 'kinect_csv')
# Bundled, so the check runs anywhere this directory is copied to rather than
# only next to a gesture library.
DEFAULT_REFERENCE = os.path.join(HERE, 'testdata', 'reference')

# Comparing parsed json, not text: the reference files came out of Python 2's
# json.dump, which leaves a space after every comma. Not a difference worth
# reproducing.

# Ges06's reference picks nine keyframes where the energy curve peaks eleven
# times, and no (window, sigma) in a 61 x 13 sweep reproduces its choice. Its
# symbols agree with ours at every frame it does name, so the disagreement is
# only about which frames were kept -- which is what LabanEditor's right-click
# add/remove keyframe editing changes. Recorded rather than silently tolerated:
# if the score check ever starts passing here, that assumption was wrong.
HAND_EDITED = {
    'Ges06_chickenwing': 'reference keyframes look hand-picked in the GUI',
}


def diff_score(produced, reference):
    """Every disagreement between two score bodies, as readable lines."""
    problems = []

    got_keys = list(produced.keys())
    want_keys = list(reference.keys())
    if got_keys != want_keys:
        problems.append('keyframe keys differ\n      got  %s\n      want %s'
                        % (got_keys, want_keys))

    for key in want_keys:
        if key not in produced:
            problems.append('%s: missing' % key)
            continue
        got, want = produced[key], reference[key]
        for field in want:
            if field not in got:
                problems.append('%s.%s: missing' % (key, field))
            elif list(got[field]) != list(want[field]):
                problems.append('%s.%s: got %s, want %s'
                                % (key, field, list(got[field]),
                                   list(want[field])))
        for field in got:
            if field not in want:
                problems.append('%s.%s: unexpected' % (key, field))

    return problems


def diff_symbols(times, symbols, reference):
    """Disagreements at the frames the reference names, ignoring frame choice."""
    problems = []
    by_time = {int(t): i for i, t in enumerate(times)}

    for key, keyframe in reference.items():
        time = int(keyframe['start time'][0])
        if time not in by_time:
            problems.append('%s: time %d is not a frame time' % (key, time))
            continue
        ours = symbols[by_time[time]]
        for limb, symbol in zip(LIMBS, ours):
            want = list(keyframe[limb])
            if [symbol[0], symbol[1]] != want:
                problems.append('%s.%s: got %s, want %s'
                                % (key, limb, [symbol[0], symbol[1]], want))

    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', default=DEFAULT_INPUTS)
    parser.add_argument('--reference', default=DEFAULT_REFERENCE)
    parser.add_argument('--verbose', action='store_true')
    parser.add_argument('--limit', type=int, default=6,
                        help='differing fields to print per gesture')
    parser.add_argument('--strict', action='store_true',
                        help='fail on the hand-edited references too')
    args = parser.parse_args()

    recordings = sorted(glob.glob(os.path.join(args.inputs, '*.csv')))
    if not recordings:
        parser.error('no recordings in %s' % args.inputs)

    print('%-24s %6s %5s  %-9s %s'
          % ('gesture', 'frames', 'keys', 'symbols', 'score'))
    print('-' * 64)

    checked = 0
    symbol_failures = []
    score_failures = []
    known = []

    for path in recordings:
        name = kinect_csv.name_of(path)
        ref_path = os.path.join(args.reference, '%s.total.json' % name)
        if not os.path.exists(ref_path):
            print('%-24s %6s %5s  no reference' % (name, '-', '-'))
            continue

        with open(ref_path) as handle:
            reference = json.load(handle)
        ref_body = reference[list(reference.keys())[0]]

        frames = kinect_csv.load(path)
        times, symbols, _ = per_frame_symbols(frames)
        score, _, _ = convert(frames, name)
        body = score[name]

        sym_problems = diff_symbols(times, symbols, ref_body)
        score_problems = diff_score(body, ref_body)
        checked += 1

        if sym_problems:
            symbol_failures.append(name)
            sym_verdict = 'FAIL %d' % len(sym_problems)
        else:
            sym_verdict = 'ok %d' % (len(ref_body) * len(LIMBS))

        if not score_problems:
            score_verdict = 'ok'
        elif name in HAND_EDITED and not args.strict:
            known.append(name)
            score_verdict = 'known: %s' % HAND_EDITED[name]
        else:
            score_failures.append(name)
            score_verdict = 'FAIL %d' % len(score_problems)

        print('%-24s %6d %5d  %-9s %s'
              % (name, len(frames), len(body), sym_verdict, score_verdict))

        if args.verbose:
            for problems in (sym_problems, score_problems):
                for line in problems[:args.limit]:
                    print('    %s' % line)
                if len(problems) > args.limit:
                    print('    ... %d more' % (len(problems) - args.limit))

    print('-' * 64)
    print('symbols: %d of %d gestures agree on every reference keyframe'
          % (checked - len(symbol_failures), checked))
    print('scores:  %d of %d reproduced exactly%s'
          % (checked - len(score_failures) - len(known), checked,
             ', %d known hand-edited' % len(known) if known else ''))

    if symbol_failures or score_failures:
        if symbol_failures:
            print('symbol mismatches: %s' % ', '.join(symbol_failures))
        if score_failures:
            print('score mismatches: %s' % ', '.join(score_failures))
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
