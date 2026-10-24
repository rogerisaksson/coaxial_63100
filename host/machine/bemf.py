"""Her drives' torque bounds under their back-EMF (`update-asymmetric-bemf-bounds`) by speed, volts.

    lo, hi = bemf.bounds(dq, volts, peak)    # each driven joint's (DRIVEN's order), N m, at speeds `dq`

The text's quadrant law: tau within [max(-peak, min(0, K (-V m - E dq))), min(peak, max(0, K (V m -
E dq)))], K = kt / R a joint's torque a volt across its winding, E its back-EMF a rad/s, V her DC
link's volts, m the space vector's linear reach (`svpwm-modulation-limit`) - her boards' current
loops' own law (`buses`): motoring the back-EMF takes from the supply, braking it adds. The peak
whoever asks: her law's its clamp or its war emergency power's (`shrunk.box`), none past the
volts' own. A pair of drives on one joint is taken as one, its torque and back-EMF the joint's.
"""
from typing import Any

from machine import drives, kernel
from machine.figure import JOINTS

#: Her driven joints, her law's order.
DRIVEN = tuple(j for j in JOINTS if drives.passive(j) is None)
#: Each one's torque a volt (N m/V), back-EMF a rad/s (V s), peak (N m).
K = tuple(drives.kt(j) / drives.of(j)[1].r for j in DRIVEN)
E = tuple(drives.emf(j) for j in DRIVEN)
PEAK = tuple(drives.peak(j) for j in DRIVEN)
#: The share of her link's volts a phase reaches.
REACH = kernel.node('update-asymmetric-bemf-bounds', 'bemf-envelope')['svpwm-modulation-limit']


def bounds(dq, volts, peak: Any = float('inf')):
    """(lo, hi) each driven joint's torque bounds, N m, at its speed `dq` (rad/s) on `volts`, within
    `peak` (N m, each or all)."""
    import numpy as np
    k, e, peak = np.asarray(K, float), np.asarray(E, float), np.asarray(peak, float)
    v, back = REACH * volts, e * np.asarray(dq, float)
    return (np.maximum(-peak, np.minimum(0.0, k * (-v - back))),
            np.minimum(peak, np.maximum(0.0, k * (v - back))))

