"""Her corrections as identified, over her build: the least move that fits a campaign, kept, laid.

    deltas = identified.correction(lay, body, Y, tau)   # {name: correction} on body's model
    identified.save(path, deltas, got)                  # kept as TOML; IDENTIFIED her law's
    identified.correct(body, identified.load(path))     # laid over her law's model, whole

Each link's mass and first moment and each drive's armature and friction move; a link's
inertia about its centre stays as built and follows them about its origin, so every link stays
a body. `wbc.Body` lays IDENTIFIED over her law's model when the file is there.
"""
import os
import tomllib

from machine import mechatronics
from machine.errors import MachineError
from machine.sysid import FRICTION, INERTIAL, following, laid, shifted, truth

#: A correction's unit move a kind (kg, kg m; kg m^2, N m s/rad, N m): a payload is a mass
#: where it hangs, not first moments up the limb - priced of each parameter's own size a hand's
#: 0.5 kg cost 14 and landed in the trunk's; the ridge against moving at all, of the
#: regressor's mean column.
UNITS = {'m': 0.1, 'hx': 0.02, 'hy': 0.02, 'hz': 0.02, 'J': 0.01, 'b': 0.05, 'f': 0.1}
RIDGE = 0.3
#: The least of its mass a link keeps under a correction.
KEPT = 0.25
#: Her corrections as identified, over her build (`save`, `load`, `correct`): her law's model
#: reads them when the file is there (`wbc.Body`).
IDENTIFIED = os.path.join(mechatronics.ROBOTS, mechatronics.ROBOT + '.identified.toml')


def _links(lay, m, theta):
    """[(body, mass, centre, principal moments, axes, a body's)] of `theta`, every link: a
    body's where its mass and its spatial inertia's moments are positive."""
    import numpy as np
    out = []
    for k, b in enumerate(lay['bodies']):
        mass, h = float(theta[10 * k]), theta[10 * k + 1:10 * k + 4]
        about = np.zeros((3, 3))
        about[np.triu_indices(3)] = theta[10 * k + 4:10 * k + 10] - shifted(mass, h)
        eig, axes = np.linalg.eigh(about + np.triu(about, 1).T)
        out.append((b, mass, h / mass if mass > 0.0 else np.zeros(3), eig, axes, mass > 0.0 and eig.min() > 0.0))
    return out


def correction(lay, body, Y, tau, ridge=RIDGE):
    """The correction on `body`'s model that fits `Y` (N, driven, P) to `tau` (N, driven) with
    every parameter moved as little as the fit allows: each link's mass and first moment and
    each drive's armature and friction, least squares on their moves in UNITS, `ridge` of the
    regressor's mean column against each; a link's inertia about its
    centre stays as built and its inertia about its origin follows its mass and centre
    (linearised in the fit, exact in the result), so every link stays a body; no mass under
    KEPT of itself, no drive parameter under none. {name: correction}."""
    import numpy as np
    nominal = truth(lay, body.m)
    n_b, n_d = len(lay['bodies']), len(lay['driven'])
    rows_ = Y.reshape(-1, Y.shape[-1])
    left = tau.reshape(-1) - rows_ @ nominal
    # z: each link's (m, hx, hy, hz), then each drive's (J, b, f), in UNITS; D theta's move a unit
    # of each, a link's inertia about its origin following its mass and first moment.
    moved = np.r_[(10 * np.arange(n_b)[:, None] + np.arange(4)).ravel(), lay['drive'] + np.arange(3 * n_d)]
    kinds = np.array(INERTIAL[:4] * n_b + FRICTION * n_d)
    unit = np.array([UNITS[k] for k in kinds])
    D = np.zeros((len(nominal), len(moved)))
    D[moved, np.arange(len(moved))] = 1.0
    for k in range(n_b):
        D[10 * k + 4:10 * k + 10, 4 * k:4 * k + 4] = following(float(nominal[10 * k]), nominal[10 * k + 1:10 * k + 4]).T
    D *= unit
    G = D.T @ (rows_.T @ rows_) @ D
    lam = ridge * float(np.sqrt(np.diag(G).mean()))
    G += lam ** 2 * np.eye(len(moved))
    rhs = D.T @ (rows_.T @ left)
    # No mass under KEPT of itself, no drive parameter under none: a bound met is held there
    # and the rest fitted again.
    low = np.where(kinds == 'm', -(1.0 - KEPT) * nominal[moved] / unit,
                   np.where(np.isin(kinds, FRICTION), -nominal[moved] / unit, -np.inf))
    held = np.zeros(len(moved), bool)
    z = np.zeros(len(moved))
    for _ in range(len(moved)):
        free = np.flatnonzero(~held)
        z[free] = np.linalg.solve(G[np.ix_(free, free)], rhs[free] - G[np.ix_(free, held)] @ z[held])
        under = (z < low) & ~held
        if not under.any():
            break
        held[under], z[under] = True, low[under]
    theta = nominal.copy()
    theta[moved] += z * unit
    for k in range(n_b):
        link, was = theta[10 * k:10 * k + 10], nominal[10 * k:10 * k + 10]
        link[4:] += shifted(float(link[0]), link[1:4]) - shifted(float(was[0]), was[1:4])
    return {n: float(d) for n, d in zip(lay['names'], theta - nominal) if d != 0.0}


def save(path, deltas, got):
    """`deltas` {name: correction} to `path` (TOML), each link's and each drive's under its name,
    the campaign's residual a drive (`got`, `identify`) above; `deltas`."""
    lines = ['# %s as identified on the stand (machine.sysid): each parameter\'s correction on the'
             % mechatronics.ROBOT,
             '# build\'s value, what its campaign told (`sysid.correction`).',
             '# residual N m rms a drive: %s' % ' '.join('%.3f' % r for r in got['residual']),
             '# rows read a drive: %s' % ' '.join('%d' % r for r in got['read']), '']
    tables = {}
    for name, delta in deltas.items():
        who, p = name.rsplit('.', 1)
        tables.setdefault(('drives' if p in FRICTION else 'links', who), []).append('%s = %.6g' % (p, delta))
    for (kind_, who), rows_ in sorted(tables.items()):
        lines += ['[%s.%s]' % (kind_, who)] + rows_ + ['']
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write('\n'.join(lines))
    return deltas


def load(path):
    """{name: correction} of a file `save` wrote; {} where there is none."""
    if not os.path.exists(path):
        return {}
    with open(path, 'rb') as f:
        read = tomllib.load(f)
    return {'%s.%s' % (who, p): float(v) for kind_ in ('links', 'drives')
            for who, table in read.get(kind_, {}).items() for p, v in table.items()}


def correct(body, deltas):
    """`deltas` {name: correction} laid over `body`'s model, whole: each link touched its mass,
    centre and inertia again, each drive touched its armature, damping and friction; the names
    corrected. Refused by name, nothing laid, where a link would be no body's (`_links`) or a
    drive would have a friction or an armature under none."""
    import numpy as np
    if not deltas:
        return []
    m, mj = body.m, body.mj
    lay = laid(body)
    theta = truth(lay, m)
    for name, delta in deltas.items():
        if name not in lay['names']:
            raise MachineError('no parameter %s of hers to correct' % name)
        theta[lay['names'].index(name)] += delta
    touched = {name.rsplit('.', 1)[0] for name in deltas}
    links = [row for row in _links(lay, m, theta) if m.body(row[0]).name in touched]
    refused = ['%s (mass %.3f, moments %s)' % (m.body(b).name, mass, np.round(eig, 4))
               for b, mass, _c, eig, _axes, ok in links if not ok]
    drives_ = []
    k = lay['drive']
    for i, (name, dof) in enumerate(zip(lay['drives'], lay['dof'])):
        if name not in touched:
            continue
        arm, damp, fric = theta[k + 3 * i:k + 3 * i + 3]
        if min(arm, damp, fric) < 0.0:
            refused.append('%s (J %.3f, b %.3f, f %.3f)' % (name, arm, damp, fric))
        drives_.append((dof, arm, damp, fric))
    if refused:
        raise MachineError('corrections refused, none laid: %s' % '; '.join(refused))
    for b, mass, c, eig, axes, _ok in links:
        if np.linalg.det(axes) < 0.0:
            axes[:, 2] *= -1.0
        quat = np.zeros(4)
        mj.mju_mat2Quat(quat, axes.ravel())
        m.body_mass[b], m.body_ipos[b], m.body_inertia[b], m.body_iquat[b] = mass, c, eig, quat
    for dof, arm, damp, fric in drives_:
        m.dof_armature[dof], m.dof_damping[dof], m.dof_frictionloss[dof] = arm, damp, fric
    mj.mj_setConst(m, body.d)
    if hasattr(body, 'mass'):
        body.mass = float(m.body_subtreemass[body.pelvis])
    return sorted(touched)
