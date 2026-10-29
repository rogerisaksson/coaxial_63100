"""Her pelvis's blind seek: where her rows wait on a seat not bearing her, her height seeks it.

    f = field.laid()
    field.step(f, tab, y, lean, goal, t, dt, out)                  # out's axial and sagittal references offset, their rates
    field.seek(f, field.due(tab, t, dt), y, lean, goal, dt, out)   # the same, her table scanned apart

The kernel's `project-blind-admittance-field` (its blind settle's): while a seat's contact rows
(`ortho.CONTACT`) are due and the seat bears less of her than her support counts it from
(`impedance.PROP_ON`, `waits.borne_rows`) and her rows' height has landed - within a pass of the
seek of its goal -, her height goes down at the text's free-air seek (0.30 m/s) - the rows held a
pass on by then through its virtual mass on its damping (12 kg on
280 N s/m: 43 ms) - an offset on her axial and sagittal modes, her rows' own descent its floor of
speed. Down no further than her knee's floor (`kinematic-singularity-floor-rad`, FLOOR); held while her
rows lean on a manifold (`observer.ENGAGE`), let go at the field's rate as they lean on none.
Where no seat comes, a squat on her soles. Its seek back (0.12 m/s) is her sit's own row: sought
too, her capture point went 0.18 m behind her soles' middle and she fell; with her capture point's
barrier on as it sought, her chair's sit landed elsewhere and its rise fell (9.88 s); sought
before her rows landed, it pressed her chair's seat and her pelvis's jerk rose 283 -> 490 m/s^3.
"""
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine import figure, impedance, kernel, latent, maneuver, modal, observer, seated

_FIELD = kernel.node('xform-blind-posterior-settle', 'admittance-field')
_MODES, _BODY = maneuver._manifest()
#: The seek, her modes' units a second: down her axial, none her sagittal; the field's rate, /s; its seek down, m/s.
SEEK = np.array([-_FIELD['free-air-seek-velocity'][2] / _MODES['mode_axial_compression'][0], 0.0])
SEEK_M_S = _FIELD['free-air-seek-velocity'][2]
RATE = _FIELD['damping-matrix-nsm'] / _FIELD['virtual-inertia-kg']
MODES = np.array([modal.AXIAL, modal.SAGITTAL])


def _folded():
    """Her knee's bend where her thigh's last capsule meets her shank's first (`figure.BODY`), rad."""
    (_a, rt, t0, t1), (_b, rs, _s0, _s1) = [c for c in figure.BODY if c[0] == 'thigh'][-1], [c for c in figure.BODY if c[0] == 'shank'][0]
    span = math.dist(t0, t1)
    return math.pi - 2.0 * math.asin((rt + rs) / (2.0 * span))


#: Her axial's floor: her knee bent to where her thigh meets her shank less the text's clearance -
#: there, sought to her knee's stop, her thighs pressed her calves and her observer read a seat.
FLOOR = seated.axial(_folded() - kernel.said('enforce-dcm-barrier-zcbf', 'kinematic-singularity-floor-rad'), _MODES, _BODY)


def laid():
    """The field at rest: its offset and its rate on her axial and sagittal modes."""
    return {'offset': np.zeros(2), 'rate': np.zeros(2)}


def due(tab, t, dt):
    """1.0 where a seat's contact rows on her table `tab` are due within a pass `dt` of `t`, else 0.0."""
    at = tab[:, latent.AT]
    return float(((tab[:, latent.CONTACT] > 0.5) & (at <= t + dt + latent.EDGE_S) & (at > t - dt + latent.EDGE_S)).any())


def seek(f, due, y, lean, goal, dt, out):
    """The field `dt` s on, `due` a seat's rows' (`field.due`), modal's `y`, her rows leaning `lean` (their
    told share off her soles), their axial's `goal`: `out`'s axial and sagittal references and rates offset."""
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
    """`seek` at `t` on her table `tab` (`due`): `f`."""
    seek(f, due(tab, t, dt), y, lean, goal, dt, out)
    return f
