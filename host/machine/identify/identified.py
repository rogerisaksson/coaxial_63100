"""The robot's corrections as identified over its build: the least move fitting a campaign, kept, laid.

    deltas = identified.correction(lay, body, Y, tau)   # {name: correction} on body's model
    deltas = identified.correction(lay, body, Y, tau, (Yw, tw))  # the walk's rows too, each share held out
    identified.save(path, deltas, got)                  # kept as TOML; IDENTIFIED the robot's law's
    identified.correct(body, identified.load(path))     # laid over its law's model, whole

Each link's mass and first moment and each drive's armature and friction move; a link's
inertia about its centre stays as built and follows them about its origin, so every link stays
a body. Each link's and each drive's share of the move is laid as far as it harms no drive on a
campaign it was not fitted on (`shares`), one campaign's halves two (`halves`). `wbc.Body` lays IDENTIFIED
over the robot's law's model when the file is there.
"""
import os
import tomllib

from machine.body import mechatronics, peripherals
from machine.errors import MachineError
from machine.identify.campaign import STAGES
from machine.identify.sysid import FRICTION, INERTIAL, following, laid, shifted, truth

#: A correction's unit move a kind (kg, kg m; kg m^2, N m s/rad, N m), ridge regularization and
#: minimum retained link mass share (`peripherals.identify()['identified']`).
_ident = peripherals.identify()['identified']
UNITS, RIDGE, KEPT = dict(_ident['units']), _ident['ridge'], _ident['kept']
#: The robot's corrections as identified, over its build (`save`, `load`, `correct`): its law's model
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


def correction(lay, body, Y, tau, *campaigns, ridge=RIDGE):
    """The correction on `body`'s model that fits `Y` (N, driven, P) to `tau` (N, driven) with
    every parameter moved as little as the fit allows: each link's mass and first moment and
    each drive's armature and friction, least squares on their moves in UNITS, `ridge` of the
    regressor's mean column against each; a link's inertia about its
    centre stays as built and its inertia about its origin follows its mass and centre
    (linearised in the fit, exact in the result), so every link stays a body; no mass under
    KEPT of itself, no drive parameter under none. {name: correction}. Given further
    `campaigns` ((Y, tau) each) it fits them all, and each link's and each drive's share of the
    move is laid as far as it harms no drive on a campaign it was not fitted on (`shares`):
    fitted on the stand and its walk's first half, a shank's mass took its hem's torque and
    raised the knee's misfit on the walk's second half 0.49 -> 2.13 N m while the drives' own
    moves held - held out, the links' shares fall and the knee reads 0.43. Given none, the
    campaign's own `halves` are held out against each other so: laid whole, the payload's
    campaign moved the links it does not tell apart - the right foot's, toes', hem's, shank's -
    and raised the right hip's misfit on a third campaign +0.093 N m; held out, +0.023, while the
    arm's fell the same (3.28 -> 0.05)."""
    import numpy as np
    rows = [(Y, tau)] + list(campaigns)
    delta = _fit(lay, body, np.concatenate([r[0] for r in rows]), np.concatenate([r[1] for r in rows]), ridge)
    delta = delta * shares(lay, body, rows if campaigns else halves(Y, tau), ridge)
    return {n: float(d) for n, d in zip(lay['names'], delta) if d != 0.0}


def halves(Y, tau, stages=len(STAGES)):
    """[(Y, tau)] two of a campaign's rows: the first half of each of its `stages` equal spans
    (`campaign.setpoints`, a limb a span), then the second - each half swinging every limb."""
    import numpy as np
    edges = np.linspace(0, len(Y), stages + 1).round().astype(int)
    first = np.concatenate([np.arange(a, (a + b) // 2) for a, b in zip(edges[:-1], edges[1:])])
    second = np.concatenate([np.arange((a + b) // 2, b) for a, b in zip(edges[:-1], edges[1:])])
    return [(Y[first], tau[first]), (Y[second], tau[second])]


def shares(lay, body, rows, ridge=RIDGE):
    """(P,) each column's share of a correction laid, 0 to 1: each link's ten and each drive's
    three as one, the share of the move fitted on any one of `rows` ((Y, tau) campaigns) alone
    that fits the rows of its drives on the others best - their misfit quadratic in the share,
    least at r.p / p.p over them (r their misfit as built, p the group's move on their rows) -,
    the least over the campaigns. Held to harm no drive at all (2 r.p / p.p a drive, the least)
    it took 0.52 of the way the world's own model takes on its walk and 0.41 on CI's runner;
    fitted best it takes 0.66 and still raises no drive."""
    import numpy as np
    nominal = truth(lay, body.m)
    n_b, n_d, P = len(lay['bodies']), len(lay['driven']), len(lay['names'])
    groups = [(np.arange(10 * k, 10 * k + 10), np.flatnonzero(lay['above'][k, lay['driven']])) for k in range(n_b)]
    groups += [(lay['drive'] + 3 * i + np.arange(3), np.array([i])) for i in range(n_d)]
    share = np.ones(P)
    fits = [_fit(lay, body, Yi, ti, ridge) for Yi, ti in rows]
    for i, d in enumerate(fits):
        for j, (Yh, th) in enumerate(rows):
            if j == i:
                continue
            for cols, drives in groups:
                part = np.zeros(P)
                part[cols] = d[cols]
                rp = pp = 0.0
                for k in drives:
                    r0, p = th[:, k] - Yh[:, k] @ nominal, Yh[:, k] @ part
                    rp, pp = rp + float(r0 @ p), pp + float(p @ p)
                if pp > 0.0:
                    share[cols] = np.minimum(share[cols], min(1.0, max(0.0, rp / pp)))
    return share


def _fit(lay, body, Y, tau, ridge):
    """theta less the build's of the correction fitted to `Y` and `tau` (`correction`), (P,)."""
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
    G += (lam * lam) * np.eye(len(moved))
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
    return theta - nominal


def save(path, deltas, got):
    """`deltas` {name: correction} to `path` (TOML), each link's and each drive's under its name,
    the campaign's residual a drive (`got`, `identify`) above; `deltas`."""
    lines = ['# %s as identified on the stand (machine.identify.sysid): each parameter\'s correction on the'
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
            raise MachineError('no parameter %s of the robot\'s own to correct' % name)
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
