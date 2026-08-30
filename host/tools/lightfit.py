#!/usr/bin/env python3
"""Fit the depth-shading model to the exporter's reference renders.

Coordinate descent over the shading constants - depth pivot and slope,
geometry floor, shadow terms - with facecheck's light agreement as the
objective (silhouette weighted in at a quarter, so brightness never
buys itself by erasing shape). The cube fixtures anchor the ramp:
their flat faces turn any response bug into a wrong character.
Prints each accepted step and the final constants to bake into
wireframe.py.

    python tools/lightfit.py            # ~a minute of fitting
"""
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from coaxial import wireframe                              # noqa: E402
import facecheck                                           # noqa: E402


def fixtures():
    got = []
    for name in sorted(os.listdir(facecheck.RENDERS)):
        stamp = re.match(r'(?:ascii|cube)-x(\d+)y(\d+)z(\d+)\.txt$', name)
        if not stamp:
            continue
        with open(os.path.join(facecheck.RENDERS, name),
                  encoding='ascii') as f:
            text = f.read()
        got.append((facecheck.euler(*(int(v) for v in stamp.groups()),
                                    order='xyz'),
                    facecheck.resample(facecheck.occupancy(text)),
                    facecheck.shades(text),
                    facecheck.solid_for(name)))
    return got


def objective(refs):
    """Mean light + a quarter silhouette, MINUS a hard penalty when any
    cube fixture's light drops under 0.85. The cube is the response
    curve's ground truth: outvoted by the board's seven fixtures, every
    earlier fit saturated the ramp and painted the cube one flat ':'."""
    wireframe._SHADOWS.clear()
    sil = lgt = 0.0
    worst_cube = 1.0
    for q, ref_sil, ref_shade, solid in refs:
        art = wireframe.render(q, 100, 50, colour=False, 
                               horizon=False, tip=0.0, solid=solid)
        sil += facecheck.iou(facecheck.resample(facecheck.occupancy(art)),
                             ref_sil)
        got = facecheck.agreement(facecheck.shades(art), ref_shade)
        lgt += got
        if solid is not None and got < worst_cube:
            worst_cube = got
    n = float(len(refs))
    score = (lgt / n + 0.25 * (sil / n)
             - 4.0 * max(0.0, 0.85 - worst_cube))
    return score, lgt / n, sil / n


#: (name, initial step, low clamp, high clamp) - every constant the
#: depth-shading model exposes.
KNOBS = (
    ('PIVOT', 0.15, 0.6, 2.8),
    ('SLOPE', 0.20, 0.1, 2.5),
    ('FLOOR', 0.08, 0.0, 1.2),
    ('SHADOW_DIM', 0.15, 0.0, 1.5),
    ('BIAS', 0.04, 0.04, 0.4),
    ('SUN_MIN', 0.08, 0.05, 0.8),
)


def put(state):
    for name, _step, _lo, _hi in KNOBS:
        setattr(wireframe, name, state[name])


def main():
    refs = fixtures()
    state = {name: getattr(wireframe, name)
             for name, _step, _lo, _hi in KNOBS}

    put(state)
    best, lgt, sil = objective(refs)
    print('start: mal %.4f ljus %.3f siluett %.3f' % (best, lgt, sil))

    steps = {name: step for name, step, _lo, _hi in KNOBS}
    for sweep in range(4):
        moved = False
        for name, _step, lo, hi in KNOBS:
            for sign in (1.0, -1.0):
                trial = dict(state)
                trial[name] = min(hi, max(lo, state[name]
                                          + sign * steps[name]))
                if trial[name] == state[name]:
                    continue
                put(trial)
                score, lgt, sil = objective(refs)
                if score > best + 1e-4:
                    state, best, moved = trial, score, True
                    print('  %-10s -> %6.3f  mal %.4f ljus %.3f sil %.3f'
                          % (name, state[name], best, lgt, sil))
                    break
        if not moved:
            for name in steps:
                steps[name] *= 0.5
            print('  krymper stegen (svep %d)' % (sweep + 1))
    put(state)
    _score, lgt, sil = objective(refs)

    print('\nslut: ljus %.3f siluett %.3f' % (lgt, sil))
    for name, _step, _lo, _hi in KNOBS:
        print('%s = %.3f' % (name, state[name]))
    return 0


if __name__ == '__main__':
    sys.exit(main())
