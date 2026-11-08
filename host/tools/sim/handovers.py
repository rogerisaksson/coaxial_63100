"""A handover row's sums past its landings: the double support's load planning, an ensemble's summary.

    row.update(handovers.planned(A, at, dt, fwd, b10, landed))   # the plan's loads against the sensed, medians
    print(handovers.ensemble(spec, rows))                        # one line for a spec's runs over ENSEMBLE

`A` the run's log as arrays (`handover.ran`), `at` its clean landings (sample, sole), `dt` the log's step, `fwd`
and `b10` samples (0.15 s on, 0.10 s back), `landed` the load share a landing is; `rows` the runs' rows (dicts).
"""
import json

import numpy as np

#: An ensemble's starts: the walk's drive and its axial target, a run each pair (`handover --ensemble`).
ENSEMBLE = ((0.84, 0.85, 0.86), (0.028, 0.03, 0.032))


def planned(A, at, dt, fwd, b10, landed):
    """The double support's load planning against the sensed loads, medians over the landings `at`: the mean
    |planned - real| share over `fwd` passes from a landing for the landing sole and the trailing one (from `b10`
    passes before), ms after the landing when the plan zeroes the trailing sole and when its load leaves."""
    errs_b, errs_o, zero, left = [], [], [], []
    for i, b in at:
        o, hi = 1 - b, min(len(A['t']), i + fwd)
        errs_b.append(np.abs(A['qp'][i:hi, b] - A['ld'][i:hi, b]).mean())
        errs_o.append(np.abs(A['qp'][i - b10:hi, o] - A['ld'][i - b10:hi, o]).mean())
        gone = np.flatnonzero(A['qp'][i:hi, o] <= 0.02)
        off = np.flatnonzero(A['ld'][i:hi, o] < landed)
        zero.append(1e3 * dt * (gone[0] if len(gone) else fwd))
        left.append(1e3 * dt * (off[0] if len(off) else fwd))
    return {k: round(float(np.median(v)), 3) for k, v in (('plan_err_land', errs_b), ('plan_err_trail', errs_o),
                                                         ('plan_zero_ms', zero), ('trail_off_ms', left)) if v}


def ensemble(spec, rows):
    """One line for `spec`'s ensemble `rows` (dicts): how many stood of how many, when the rest fell, the hip clamp
    passes' median and worst, the speed's, the landing knee's and the plan's load errors' medians."""
    med = lambda k: round(float(np.median([r[k] for r in rows if r.get(k) is not None])), 3) if any(r.get(k) is not None for r in rows) else None
    return json.dumps({'spec': spec, 'runs': len(rows), 'stood': sum(bool(r.get('stood')) for r in rows),
                       'fell_at': sorted(round(r['fell'], 1) for r in rows if r.get('fell') is not None),
                       'hip_passes_med': med('hip_passes'), 'hip_passes_max': max((r.get('hip_passes') or 0) for r in rows),
                       'v_med': med('v'), 'land_knee_med': med('land_knee'), 'knee_min_med': med('knee_min'),
                       'plan_err_land_med': med('plan_err_land'), 'plan_err_trail_med': med('plan_err_trail'),
                       'plan_zero_ms_med': med('plan_zero_ms'), 'trail_off_ms_med': med('trail_off_ms')})
