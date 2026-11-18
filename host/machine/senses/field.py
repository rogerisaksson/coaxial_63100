"""The robot's pelvis's blind seek: where its rows wait on a seat not bearing it, its height seeks it.

    f = field.laid()
    field.step(f, tab, y, lean, goal, t, dt, out)                  # out's axial and sagittal references offset, their rates
    field.seek(f, field.due(tab, t, dt), y, lean, goal, dt, out)   # the same, the robot's table scanned apart

The kernel's `project-blind-admittance-field` (its blind settle's): while a seat's contact rows
(`ortho.CONTACT`) are due and the seat bears less of the robot than its support counts it from
(`impedance.PROP_ON`, `waits.borne_rows`) and the robot's rows' height has landed - within a pass of the
seek of its goal -, the robot's height goes down at the text's free-air seek (0.30 m/s) - the rows held a
pass on by then through its virtual mass on its damping (12 kg on
280 N s/m: 43 ms) - an offset on the robot's axial and sagittal modes, its rows' own descent its floor
of speed. Down no further than the robot's knee's floor (`kinematic-singularity-floor-rad`, FLOOR); held while
its rows lean on a manifold (`observer.ENGAGE`), let go at the field's rate as they lean on none. Where no
seat comes, a squat on the robot's soles. The field's seek back (0.12 m/s) is the robot's sit's own row: sought too,
the robot's capture point went 0.18 m behind its soles' middle and it fell; with the robot's
capture point's barrier on as it sought, the robot's chair's sit landed elsewhere and its rise fell (9.88 s);
sought before the robot's rows landed, it pressed the robot's chair's seat and its pelvis's jerk rose
283 -> 490 m/s^3.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import libm
from machine.body import figure
from machine.law import impedance, latent, modal
from machine.texts import kernel, maneuver
from machine.senses import observer
from machine.moves import seated

_FIELD = kernel.node('xform-blind-posterior-settle', 'admittance-field')
_MODES, _BODY = maneuver._manifest()
#: The seek, the robot's modes' units a second: down its axial, none its sagittal; the field's rate,
#: /s; its seek down, m/s.
SEEK = np.array([-_FIELD['free-air-seek-velocity'][2] / _MODES['mode_axial_compression'][0], 0.0])
SEEK_M_S = _FIELD['free-air-seek-velocity'][2]
RATE = _FIELD['damping-matrix-nsm'] / _FIELD['virtual-inertia-kg']
MODES = np.array([modal.AXIAL, modal.SAGITTAL])


def _folded():
    """The robot's knee's bend where its thigh's last capsule meets its shank's first (`figure.BODY`), rad."""
    (_a, rt, t0, t1), (_b, rs, _s0, _s1) = [c for c in figure.BODY if c[0] == 'thigh'][-1], [c for c in figure.BODY if c[0] == 'shank'][0]
    span = math.dist(t0, t1)
    return math.pi - 2.0 * libm.arcsin((rt + rs) / (2.0 * span))


#: The robot's axial's floor: its knee bent to where its thigh meets its shank less the text's clearance -
#: there, sought to its knee's stop, its thighs pressed its calves and its observer read a seat.
FLOOR = seated.axial(_folded() - kernel.said('enforce-dcm-barrier-zcbf', 'kinematic-singularity-floor-rad'), _MODES, _BODY)


def laid():
    """The field at rest: its offset and its rate on the robot's axial and sagittal modes."""
    return {'offset': np.zeros(2), 'rate': np.zeros(2)}


def due(tab, t, dt):
    """1.0 where a seat's contact rows on the robot's table `tab` are due within a pass `dt` of `t`, else
    0.0."""
    lo, hi = t - dt + latent.EDGE_S, t + dt + latent.EDGE_S
    return float(any(c > 0.5 and lo < at <= hi for c, at in zip(tab[:, latent.CONTACT].tolist(), tab[:, latent.AT].tolist())))


def seek(f, due, y, lean, goal, dt, out):
    """The field `dt` s on, `due` a seat's rows' (`field.due`), modal's `y`, the robot's rows leaning `lean`
    (their told share off its soles), their axial's `goal`: `out`'s axial and sagittal references and rates
    offset."""
    landed = abs(goal - out[latent.REF][modal.AXIAL]) <= SEEK[0] * dt
    waiting = landed * (due * (y[modal.PROP] < impedance.PROP_ON))
    asked = out[latent.RATE][MODES]
    want = waiting * np.sign(SEEK) * np.maximum(np.abs(SEEK) - np.sign(SEEK) * asked, 0.0)
    f['rate'] += min(1.0, RATE * dt) * (want - f['rate'])
    held = np.minimum(1.0, lean / observer.ENGAGE)
    f['offset'] += dt * (f['rate'] - (1.0 - held) * RATE * f['offset'])
    f['offset'][0] = np.minimum(f['offset'][0], np.maximum(0.0, FLOOR - out[latent.REF][modal.AXIAL]))
    out[latent.REF][MODES] += f['offset']
    out[latent.RATE][MODES] += f['rate']


def step(f, tab, y, lean, goal, t, dt, out):
    """`seek` at `t` on the robot's table `tab` (`due`): `f`."""
    seek(f, due(tab, t, dt), y, lean, goal, dt, out)
    return f
