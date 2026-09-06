# Vendored from LabanotationSuite

These four files are the gesture-to-Labanotation algorithm from Microsoft's
LabanotationSuite, MIT licensed. They are copied rather than reimplemented so
that the symbols this tool produces are the ones the original tool produced,
not an approximation of them.

    https://github.com/microsoft/LabanotationSuite
    GestureAuthoringTools/LabanEditor/src/

| file | upstream path | sha256 (first 16) |
|---|---|---|
| `labanProcessor.py` | `labanotation/labanProcessor.py` | `26085C33007DCB90` |
| `accessory.py` | `labanotation/tool/accessory.py` | `D8B503FDF3DB144F` |
| `wavfilter.py` | `labanotation/tool/wavfilter.py` | `1BF4269498784999` |
| `kp_extractor.py` | `labanotation/tool/kp_extractor.py` | `AFFEA5E04FDBEF9E` |

Fetched at master, 2026-09-05. The hashes are of the files as downloaded, before
the edit below.

## The only edit

`wavfilter.py` lost `import matplotlib.pyplot as pl` and the `__main__` block
that was its only user. Nothing else in this tool draws anything, and the import
would otherwise make matplotlib a hard dependency of a headless converter.

Everything else is byte-identical, including two upstream quirks that the
converter deliberately reproduces because changing them would change the output:

- `accessory.norm` returns `np.array(len(temp))` for constant input, which is a
  scalar array rather than a run of zeros. Only reachable when a limb never
  moves for the whole clip.
- `labanProcessor.to_sphere` computes the azimuth with `atan`, not `atan2`, and
  patches up the quadrant afterwards. The patch-up leaves `phi` in `(-180, 180]`
  as documented, so it agrees with `atan2`, but the arithmetic is not the same.

## What is not vendored

`algtotal.py` holds the keyframe-selection algorithm, but it is welded to
matplotlib axes and to LabanEditor's global `settings.application`. Its numeric
core is reproduced in `convert.py` instead, and `validate.py` is what shows the
reproduction agrees with the original.
