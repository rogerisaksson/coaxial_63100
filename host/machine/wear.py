"""On-line identification under a load: her drift from her build read as she goes, a load told apart.

    w = wear.laid(body)                                  # what may move: a load a site, a drive's f, b, kt
    got = wear.window(w, ys, taus, dt)                   # a window's rows where her model holds
    wear.step(w, got)                                    # the filter walked the window on and corrected
    wear.boxed(w, ys, taus, dt)                          # her boxes' play, wind-up and zero off the window
    wear.loaded(w, g)                                    # the load laid onto her law's model and frame
    wear.told(w)                                         # the load, each drift, the alarms, the service

A load rides a SITE as a mass and where it hangs (a first moment), or her torso as her insoles
bear it (`torso_load`); a drive's Coulomb and viscous friction move and its torque an amp
scales: the movers, each in its UNIT, walking PACE of it a root second - a load comes and goes
in seconds, a drift over hours. Each window's rows are read where her model holds (`sysid.rows`,
`sysid.free`) against her build's prediction, each drive's rows weighed by their misfit the
window before, and a Kalman filter on that random walk keeps every mover with its doubt. Her
build's [service] (`mechatronics`) says the bands: an alarm names a drift past its band by
SIGMAS of its doubt; service is due at the interval's hours or where a drift's trend from the
last service meets its band. The same rows off a robot's boards, its currents its torques.
"""
import math

from machine import campaign, drives, edge, flight, identified, mechatronics, modal, sysid
from machine.errors import MachineError

#: Where a load may ride on her limbs: her hands. Her back's load is seen by her drives only
#: when she leans: her insoles' rows (`sysid.borne`, `torso_load`) read it while she stands on both.
SITES = ('left_hand', 'right_hand')
#: The movers, in this order: each site's mass and first moment, kg and kg m; each drive's
#: Coulomb friction, N m, and viscous, N m s/rad; each drive's torque an amp, of itself. Each
#: kind's unit, and how far it walks in a second, units a root second.
LOAD, DRIFT, SCALE = ('m', 'hx', 'hy', 'hz'), ('f', 'b'), ('kt',)
UNIT = {'m': 0.1, 'hx': 0.02, 'hy': 0.02, 'hz': 0.02, 'f': 0.1, 'b': 0.05, 'kt': 0.01}
PACE = {'m': 4.5, 'hx': 4.5, 'hy': 4.5, 'hz': 4.5, 'f': 0.5, 'b': 0.5, 'kt': 0.5}
#: Each one's doubt at the start, units; a row's noise, N m; a drift is past its band by this
#: many of its doubt before it is named.
PRIOR, NOISE, SIGMAS = 10.0, 0.5, 2.0
#: Every how many steps a window's rows are read, and a window, s.
EVERY, WINDOW_S = 10, 4.0
#: A box's read eased over this many windows; its play, its wind-up and its encoder's zero among
#: `campaign.BOX`.
BOX_WINDOWS = 3
PLAY, WIND, ZERO = campaign.BOX.index('play'), campaign.BOX.index('k'), campaign.BOX.index('zero')
#: A load is placed where it hangs from this mass up, kg.
HUNG_KG = 0.05


def service(build=None):
    """The bands and the interval of a build's [service]: friction (of its own, and at least N m),
    viscous (of its own, N m s/rad), torque (of itself), play (of its own, up), wind_up (of its
    own, down), zero (rad), load_kg, hours, trend_h (a drift's trend from the last service counts
    from)."""
    table = (build or mechatronics.robot()).get('service')
    if table is None:
        raise MachineError('%s names no service: a [service] table in machine/robots/%s.toml'
                           % (mechatronics.ROBOT, mechatronics.ROBOT))
    return table


def laid(body, build=None):
    """What may move on `body`'s model, and the filter at rest on it: {lay, body, nominal, names,
    unit, pace, D (P, the movers but the kt's: theta's move a unit of each), band, z, P, hours,
    serviced_at, residual, read, armature, boxes, box_n, zero, torso, nominal_boxes, service}."""
    import numpy as np
    lay = sysid.laid(body)
    nominal, table = sysid.truth(lay, body.m), service(build)
    sites = lay['col'][[body.m.body(site).id for site in SITES]]
    if (sites < 0).any():
        raise MachineError('%s is no link of hers to load' % SITES[int(np.argmin(sites))])
    at, names_d = lay['drive'], lay['drives']
    n_d, n_s = len(names_d), len(SITES)
    names = (['%s.%s' % (site, kind) for site in SITES for kind in LOAD]
             + ['%s.%s' % (name, kind) for name in names_d for kind in DRIFT]
             + ['%s.kt' % name for name in names_d])
    kinds = LOAD * n_s + DRIFT * n_d + SCALE * n_d
    unit = np.array([UNIT[k] for k in kinds])
    D = np.zeros((len(nominal), 4 * n_s + 2 * n_d))
    for s, k in enumerate(sites):
        D[k:k + 4, 4 * s:4 * s + 4] = np.eye(4)
        D[k + 4:k + 10, 4 * s:4 * s + 4] = sysid.following(float(nominal[k]), nominal[k + 1:k + 4]).T
    friction = nominal[at:].reshape(n_d, 3)
    for i in range(n_d):
        D[at + 3 * i + 2, 4 * n_s + 2 * i], D[at + 3 * i + 1, 4 * n_s + 2 * i + 1] = 1.0, 1.0
    band = np.concatenate([
        np.tile([float(table['load_kg']), math.inf, math.inf, math.inf], n_s),
        np.column_stack([np.maximum(table['friction'][0] * np.abs(friction[:, 2]), table['friction'][1]),
                         np.maximum(table['viscous'][0] * np.abs(friction[:, 1]), table['viscous'][1])]).ravel(),
        np.full(n_d, float(table['torque']))])
    unm = np.array([k > 0 and not np.isin(lay['hinge'][lay['span'][k, 0]:lay['span'][k, 1]], lay['vadr']).any()
                    for k in range(len(lay['bodies']))], bool)
    hems = [k for k in np.flatnonzero(unm) if (lay['above'][k, lay['driven']] @ lay['limb']).any()]
    hem_limb = np.array([(lay['limb'] @ (lay['above'][k, lay['driven']] @ lay['limb'])) > 0 for k in hems], bool)
    return {'lay': lay, 'body': body, 'nominal': nominal, 'names': names, 'unit': unit,
            'pace': np.array([PACE[k] for k in kinds]), 'D': D * unit[:D.shape[1]], 'band': band,
            'z': np.zeros(len(names)), 'P': PRIOR ** 2 * np.eye(len(names)), 'hours': 0.0, 'serviced_at': 0.0,
            'residual': np.zeros(n_d), 'read': np.zeros(n_d, int), 'armature': friction[:, 0].copy(),
            'boxes': np.zeros((n_d, len(campaign.BOX))), 'box_n': np.zeros(n_d, int),
            'zero': np.zeros(n_d), 'torso': (0.0, (0.0, 0.0, 0.0)), 'hems': hems, 'hem_limb': hem_limb,
            'nominal_boxes': np.array([(drives.play(j), drives.wind(j)) for j in names_d]), 'service': table}


def _hems(w, ys, dt, every):
    """((qadr, vadr, q, qd, qdd), free) her limb pendulums (`w['hems']`) integrated at `dt` over `ys`
    as 2-DoF pendulums with their stops, sampled every `every`th: what she does not measure on her
    shanks, following them from her IMU and her legs' encoders."""
    import numpy as np
    lay, body, hems = w['lay'], w['body'], w['hems']
    m, at = body.m, np.arange(0, len(ys) - 1, every)
    jids = [int(lay['jnt'][h]) for k in hems for h in range(lay['span'][k, 0], lay['span'][k, 1])]
    qadr, vadr = np.array(m.jnt_qposadr[jids], int), np.array(m.jnt_dofadr[jids], int)
    if len(ys) < 2 or not jids or float(m.body_mass[m.jnt_bodyid[jids[0]]]) <= 0.0:
        z = np.zeros((len(at), len(jids)))
        return (qadr, vadr, z, z, z), np.ones((len(at), len(hems)), bool)
    n, rot = len(ys) - 1, sysid._rotations
    R0 = rot(ys[:n, edge.QUAT])
    w0 = np.einsum('nij,nj->ni', R0, ys[:n, edge.GYRO])
    al0 = np.einsum('nij,nj->ni', R0, (ys[1:n + 1, edge.GYRO] - ys[:n, edge.GYRO]) / dt)
    a0 = np.einsum('nij,nj->ni', rot(ys[1:n + 1, edge.QUAT]), ys[1:n + 1, edge.ACCEL])
    qd_all = ys[:n, edge.QD]
    qdd_all = (ys[1:n + 1, edge.QD] - qd_all) / dt
    q_out, v_out, a_out = np.zeros((n, len(jids))), np.zeros((n, len(jids))), np.zeros((n, len(jids)))
    hit, prev = np.zeros((n, len(hems)), bool), w.get('_hem_prev')
    cont = prev is not None and float(np.max(np.abs(ys[0, edge.Q] - prev[0]))) < 1e-9
    next_state = []
    for s_idx, k in enumerate(hems):
        b = int(lay['bodies'][k])
        mass = float(m.body_mass[b])
        if mass <= 0.0:
            next_state.append((0.0, 0.0, 0.0, 0.0))
            continue
        chain, p = [], int(lay['parent'][k])
        while p > 0:
            chain.append(int(lay['bodies'][p]))
            p = int(lay['parent'][p])
        R, wv, al, a = R0, w0, al0, a0
        for link in reversed(chain):
            dp = np.einsum('nij,j->ni', R, m.body_pos[link])
            a = a + np.cross(al, dp) + np.cross(wv, np.cross(wv, dp))
            R = R @ rot(m.body_quat[link])
            j = int(m.body_jntadr[link])
            ax, ji = m.jnt_axis[j], int(np.flatnonzero(lay['vadr'] == m.jnt_dofadr[j])[0])
            q, qd, qdd_j = ys[:n, edge.Q][:, ji], qd_all[:, ji:ji + 1], qdd_all[:, ji:ji + 1]
            sw = np.einsum('nij,j->ni', R, ax)
            al, wv = al + sw * qdd_j + np.cross(wv, sw) * qd, wv + sw * qd
            c, s = np.cos(0.5 * q), np.sin(0.5 * q)
            R = R @ rot(np.column_stack([c, s * ax[0], s * ax[1], s * ax[2]]))
        dp = np.einsum('nij,j->ni', R, m.body_pos[b])
        ap = np.einsum('nji,nj->ni', R, a + np.cross(al, dp) + np.cross(wv, np.cross(wv, dp)))
        om, alb = np.einsum('nji,nj->ni', R, wv), np.einsum('nji,nj->ni', R, al)
        L, I0 = -float(m.body_ipos[b, 1]), float(m.body_inertia[b, 0])
        J, jx, jz = I0 + mass * L * L, jids[2 * s_idx], jids[2 * s_idx + 1]
        stiff, damp = float(m.jnt_stiffness[jx]), float(m.dof_damping[vadr[2 * s_idx]])
        tc, dr = float(m.jnt_solref[jx, 0]), float(m.jnt_solref[jx, 1])
        dmin, dmax, width = float(m.jnt_solimp[jx, 0]), float(m.jnt_solimp[jx, 1]), float(m.jnt_solimp[jx, 2])
        invw = (float(m.dof_invweight0[vadr[2 * s_idx]]), float(m.dof_invweight0[vadr[2 * s_idx + 1]]))
        lo, hi = (float(m.jnt_range[jx, 0]), float(m.jnt_range[jz, 0])), (float(m.jnt_range[jx, 1]), float(m.jnt_range[jz, 1]))
        k_imp, b_imp = 1.0 / (dmax * dmax * tc * tc * dr * dr), 2.0 / (dmax * tc)
        qx, qz, vx, vz = prev[1][s_idx] if cont and prev is not None else (0.0, 0.0, 0.0, 0.0)
        # floats a step: the window's 8 000 steps of np.cross on 3-vectors stood her 1.4 s
        aps, oms, albs = ap.tolist(), om.tolist(), alb.tolist()
        qs, vs, qdds = [], [], []
        for t in range(n):
            cx, sx, cz, sz = math.cos(qx), math.sin(qx), math.cos(qz), math.sin(qz)
            (p0, p1, p2), (o0, o1, o2), (b0, b1, b2) = aps[t], oms[t], albs[t]
            r0, r1, r2 = L * sz, -L * cx * cz, -L * sx * cz
            x1, x2 = L * sx * cz, -L * cx * cz
            z0, z1, z2 = L * cz, L * cx * sz, L * sx * sz
            d0, d1, d2 = o1 * r2 - o2 * r1, o2 * r0 - o0 * r2, o0 * r1 - o1 * r0
            v0, v1, v2 = 0.0 * vx + z0 * vz, x1 * vx + z1 * vz, x2 * vx + z2 * vz
            af0 = p0 + (b1 * r2 - b2 * r1) + (o1 * d2 - o2 * d1) + 2.0 * (o1 * v2 - o2 * v1)
            af1 = p1 + (b2 * r0 - b0 * r2) + (o2 * d0 - o0 * d2) + 2.0 * (o2 * v0 - o0 * v2)
            af2 = p2 + (b0 * r1 - b1 * r0) + (o0 * d1 - o1 * d0) + 2.0 * (o0 * v1 - o1 * v0)
            tx = -mass * (0.0 * af0 + x1 * af1 + x2 * af2) - I0 * b0 - stiff * qx - damp * vx
            tz = -mass * (z0 * af0 + z1 * af1 + z2 * af2) - I0 * (b2 * cx - b1 * sx) - stiff * qz - damp * vz
            qdd = [tx / J, tz / J]
            for i, (qi, vi) in enumerate(((qx, vx), (qz, vz))):
                r_pen, sgn = (lo[i] - qi, 1.0) if qi < lo[i] else ((qi - hi[i], -1.0) if qi > hi[i] else (0.0, 0.0))
                if r_pen > 0.0:
                    hit[t, s_idx], x = True, min(1.0, r_pen / width)
                    imp = dmin + (dmax - dmin) * (x * x * (3.0 - 2.0 * x))
                    aref = sgn * (k_imp * imp * r_pen) - b_imp * vi
                    if sgn * (aref - qdd[i]) > 0.0:
                        qdd[i] += (aref - qdd[i]) / (1.0 + (1.0 - imp) / imp * invw[i] * J)
            qs.append((qx, qz))
            vs.append((vx, vz))
            qdds.append(qdd)
            vx += qdd[0] * dt
            vz += qdd[1] * dt
            qx += vx * dt
            qz += vz * dt
        q_out[:, 2 * s_idx:2 * s_idx + 2], v_out[:, 2 * s_idx:2 * s_idx + 2], a_out[:, 2 * s_idx:2 * s_idx + 2] = qs, vs, qdds
        next_state.append((qx, qz, vx, vz))
    w['_hem_prev'] = (ys[-1, edge.Q].copy(), next_state)
    pad = np.zeros((len(at), len(hems)), bool)
    for d_step in range(-5, 6):
        pad |= hit[np.clip(at + d_step, 0, n - 1)]
    return (qadr, vadr, q_out[at], v_out[at], a_out[at]), ~pad


def window(w, ys, taus, dt):
    """A window's rows against her build: {A (R, movers), left (R,) N m, drive (R,) each row's
    drive, zero (driven,) rad, seconds} - her edge arrays `ys` (N + 1, SENSED) and her drives'
    torques `taus` (N, JOINTS) a step `dt` apart, every EVERYth step read where her model holds,
    a torque's misfit the target and each mover's column its share of it: a kt's its drive's
    predicted torque short of its friction, less - whole, it told nothing from the friction's own
    columns. While both soles bear weight through the window, her back's load is read off her
    insoles (`torso_load`)."""
    import numpy as np
    lay = w['lay']
    ys_arr = np.asarray(ys)
    extra, hem_free = _hems(w, ys_arr, dt, EVERY)
    Y, tau = sysid.rows(lay, w['body'], ys_arr, taus, dt, EVERY, extra)
    held = sysid.free(lay, ys_arr, EVERY) & (((~hem_free).astype(int) @ w['hem_limb']) == 0)
    Y, tau = Y * held[:, :, None], tau * held
    n, n_d, p = Y.shape
    rows_ = Y.reshape(-1, p)
    pred = rows_ @ w['nominal']
    sliding = rows_[:, lay['drive']:] @ w['nominal'][lay['drive']:]
    movers, drive = w['D'].shape[1], np.tile(np.arange(n_d), n)
    A = np.zeros((n * n_d, movers + n_d))
    A[:, :movers] = rows_ @ w['D']
    A[np.arange(n * n_d), movers + drive] = (sliding - pred) * UNIT['kt']
    keep = np.abs(rows_).sum(axis=1) > 0.0
    zero = (ys_arr[-1, edge.QM] - ys_arr[-1, edge.Q])[lay['joint']]
    if (ys_arr[:, edge.LOAD] >= flight.TOUCH_N).all(axis=1).mean() > 0.8:
        torso_load(w, ys_arr, dt)
    return {'A': A[keep], 'left': (tau.reshape(-1) - pred)[keep], 'drive': drive[keep],
            'zero': zero, 'seconds': (len(ys_arr) - 1) * dt}


def step(w, got):
    """The filter `got`'s seconds on - each mover walked its PACE - and corrected by its rows,
    each drive's weighed by its misfit the window before (NOISE at least): z and P (units), the
    hours run, the residual and the rows read a drive."""
    import numpy as np
    A, left, drive = got['A'], got['left'], got['drive']
    n_d = len(w['residual'])
    w['P'] = w['P'] + np.diag(w['pace'] ** 2 * got['seconds'])
    if len(left):
        sigma = np.maximum(NOISE, w['residual'])[drive]
        A, left = A / sigma[:, None], left / sigma
        w['P'] = np.linalg.inv(np.linalg.inv(w['P']) + A.T @ A)
        w['z'] = w['z'] + w['P'] @ (A.T @ (left - A @ w['z']))
        r = (left - A @ w['z']) * sigma
        w['read'] = np.bincount(drive, minlength=n_d)
        w['residual'] = np.sqrt(np.bincount(drive, r * r, minlength=n_d) / np.maximum(w['read'], 1))
    if 'zero' in got:
        w['zero'] = np.asarray(got['zero'], float)
    w['hours'] += got['seconds'] / 3600.0


def boxed(w, ys, taus, dt):
    """Her boxes off the window (`campaign.boxes`, her build's armature the rotors'): each read
    eased into `boxes` over BOX_WINDOWS where a box wound up, and each encoder's zero read;
    rigid, only `zero`."""
    import numpy as np
    read = campaign.boxes(w['body'], ys, taus, dt, w['armature'])
    w['zero'] = read[:, ZERO].copy()
    got = read[:, WIND] > 0.0
    n = np.minimum(w['box_n'][got] + 1, BOX_WINDOWS)
    w['boxes'][got] += (read[got] - w['boxes'][got]) / n[:, None]
    w['box_n'][got] += 1


def loaded(w, g=None):
    """The load identified in `w` - her hands' where past SIGMAS of their doubt, her torso's where
    past HUNG_KG - laid over `w['body']` from her build (`identified.correct`), and onto `g['fr']`
    (her weight, her hips' share and her modal masses) when `g` is given; the links touched."""
    import numpy as np
    lay, body, nominal = w['lay'], w['body'], w['nominal']
    theta = w['z'] * w['unit']
    doubt = np.sqrt(np.maximum(np.diag(w['P']), 0.0)) * w['unit']
    current = sysid.truth(lay, body.m)
    loads = {}
    for s, site in enumerate(SITES):
        mass, h = float(theta[4 * s]), theta[4 * s + 1:4 * s + 4]
        loads[site] = (mass, h) if mass > max(HUNG_KG, SIGMAS * float(doubt[4 * s])) else (0.0, np.zeros(3))
    t_mass, t_where = w['torso']
    loads['torso'] = (float(t_mass), float(t_mass) * np.asarray(t_where, float)) if t_mass > HUNG_KG else (0.0, np.zeros(3))
    deltas = {}
    for site, (dm, dh) in loads.items():
        k = int(lay['col'][body.m.body(site).id])
        was = nominal[k:k + 10]
        target = was.copy()
        target[0] += dm
        target[1:4] += dh
        target[4:] += sysid.shifted(float(target[0]), target[1:4]) - sysid.shifted(float(was[0]), was[1:4])
        diff = target - current[k:k + 10]
        if float(np.max(np.abs(diff))) > 1e-12:
            for p, d in enumerate(diff):
                if abs(float(d)) > 1e-12:
                    deltas[lay['names'][k + p]] = float(d)
    touched = identified.correct(body, deltas)
    if g is not None and deltas:
        m = body.m
        mass = float(m.body_subtreemass[body.pelvis])
        limb = float(m.body_subtreemass[m.body('left_hip_yaw').id])
        trunk = float(m.body_subtreemass[body.torso])
        arm = 0.5 * sum(float(m.body_subtreemass[m.body(s + 'upper_arm').id]) for s in ('left_', 'right_'))
        fr = g['fr']
        fr[modal.WEIGHT] = mass * modal.G
        fr[modal.HIPS_BY] = mass / (mass - limb)
        fr[modal.MASS] = mass, mass, limb, trunk, mass, limb, limb, trunk, trunk, arm, limb
    return touched


def told(w):
    """What the filter has: {load {site: (kg, where it hangs, m, its doubt kg)}, drift {drive: {f
    N m, b N m s/rad, kt of itself, zero rad}}, doubt {name: its}, boxes {drive: (play of its own,
    wind-up of its own)}, alarms [(name, value, band)], hours, since, left_h, next, residual,
    read}."""
    import numpy as np
    names, names_d, table = w['names'], w['lay']['drives'], w['service']
    theta, doubt = w['z'] * w['unit'], np.sqrt(np.maximum(np.diag(w['P']), 0.0)) * w['unit']
    first, n_d = 4 * len(SITES), len(names_d)
    load = {}
    for s, site in enumerate(SITES):
        mass, h = float(theta[4 * s]), theta[4 * s + 1:4 * s + 4]
        load[site] = (mass, tuple((h / mass).tolist()) if mass > HUNG_KG else (0.0, 0.0, 0.0),
                      float(doubt[4 * s]))
    t_mass, t_where = w['torso']
    load['torso'] = (float(t_mass), tuple(t_where), 0.0)
    drift = {name: {'f': float(theta[first + 2 * i]), 'b': float(theta[first + 2 * i + 1]),
                    'kt': float(theta[first + 2 * n_d + i]), 'zero': float(w['zero'][i])}
             for i, name in enumerate(names_d)}
    over = np.abs(theta) - SIGMAS * doubt > w['band']
    alarms = [(names[k], float(theta[k]), float(w['band'][k])) for k in np.flatnonzero(over)]
    if abs(float(t_mass)) > float(table['load_kg']):
        alarms.append(('torso.m', float(t_mass), float(table['load_kg'])))
    zero_band = float(table.get('zero', math.inf))
    for i, name in enumerate(names_d):
        if abs(float(w['zero'][i])) > zero_band:
            alarms.append(('%s.zero' % name, float(w['zero'][i]), zero_band))
    boxes = {}
    for i, name in enumerate(names_d):
        if w['box_n'][i] == 0:
            continue
        play0, k0 = w['nominal_boxes'][i]
        play, k = w['boxes'][i, PLAY] / play0 - 1.0, 1.0 - w['boxes'][i, WIND] / k0
        boxes[name] = (float(play), float(k))
        for kind, value in (('play', play), ('wind_up', k)):
            if value > table[kind]:
                alarms.append(('%s.%s' % (name, kind), float(value), float(table[kind])))
    since = w['hours'] - w['serviced_at']
    left_h, nxt = float(table['hours']) - since, 'interval'
    if since >= float(table['trend_h']):
        # each drift's trend from the last service, where it meets its band
        for k in range(first, len(names)):
            if abs(theta[k]) > SIGMAS * doubt[k]:
                eta = (w['band'][k] - abs(theta[k])) / abs(theta[k]) * since
                if eta < left_h:
                    left_h, nxt = eta, names[k]
    if alarms and left_h > 0.0:
        left_h, nxt = 0.0, alarms[0][0]
    return {'load': load, 'drift': drift, 'doubt': dict(zip(names, doubt.tolist())), 'boxes': boxes,
            'alarms': alarms, 'hours': w['hours'], 'since': since, 'left_h': max(0.0, left_h), 'next': nxt,
            'residual': w['residual'].tolist(), 'read': w['read'].tolist()}


def serviced(w):
    """A service done: the drifts' trends start anew from here."""
    w['serviced_at'] = w['hours']


def torso_load(w, ys, dt):
    """(mass kg, (hx, hy, hz) m) of a load riding her torso, read off her insoles' rows
    (`sysid.borne`) against her build's prediction on the samples her insoles bear her."""
    import numpy as np
    lay, body = w['lay'], w['body']
    Yb, fb = sysid.borne(lay, body, ys, dt, EVERY)
    borne = fb[:, 0] > 2.0 * flight.TOUCH_N * sysid.LEVER_M
    Yb, fb = Yb[borne], fb[borne]
    b = body.m.body('torso').id
    k = lay['col'][b]
    A = Yb[:, :, k:k + 4].reshape(-1, 4)
    left = (fb - Yb @ w['nominal']).reshape(-1)
    theta, *_ = np.linalg.lstsq(A, left, rcond=None)
    mass = float(theta[0])
    where = tuple((theta[1:4] / mass).tolist()) if abs(mass) > HUNG_KG else (0.0, 0.0, 0.0)
    w['torso'] = (mass, where)
    return mass, where

