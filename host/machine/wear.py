"""On-line identification under a load: her drift from her build read as she goes, a load told apart.

    w = wear.laid(body)                                  # what may move: a load a site, a drive's f, b, kt
    got = wear.window(w, ys, taus, dt)                   # a window's rows where her model holds
    wear.step(w, got)                                    # the filter walked the window on and corrected
    wear.boxed(w, ys, taus, dt)                          # her boxes' play and wind-up off the window
    wear.told(w)                                         # the load, each drift, the alarms, the service

A load rides a SITE as a mass and where it hangs (a first moment); a drive's Coulomb and viscous
friction move and its torque an amp scales: KINDS, each in its UNIT, walking PACE of it a root
second - a load comes and goes in seconds, a drift over hours. Each window's rows are read where
her model holds (`sysid.rows`, `sysid.free`) against her build's prediction, each drive's rows
weighed by their misfit the window before, and a Kalman filter on that random walk keeps every
mover with its doubt. Her build's [service] (`mechatronics`) says the bands: an alarm names a
drift past its band by SIGMAS of its doubt; service is due at the interval's hours or where a
drift's trend from the last service meets its band. The same rows off a robot's boards, its
currents its torques.
"""
import math

from machine import drives, mechatronics, sysid
from machine.errors import MachineError

#: Where a load may ride: her hands. Her back's load is seen by her drives only when she leans:
#: her insoles' rows (`sysid.borne`) for it.
SITES = ('left_hand', 'right_hand')
#: What may move: a site's mass and first moment, kg and kg m; a drive's Coulomb friction, N m,
#: its viscous friction, N m s/rad, and its torque an amp, of itself. Each kind's unit, and how far
#: it walks in a second, units a root second.
KINDS = ('m', 'hx', 'hy', 'hz', 'f', 'b', 'kt')
UNIT = {'m': 0.1, 'hx': 0.02, 'hy': 0.02, 'hz': 0.02, 'f': 0.1, 'b': 0.05, 'kt': 0.01}
PACE = {'m': 5.0, 'hx': 5.0, 'hy': 5.0, 'hz': 5.0, 'f': 0.5, 'b': 0.5, 'kt': 0.5}
#: Each one's doubt at the start, units; a row's noise, N m; a drift is past its band by this
#: many of its doubt before it is named.
PRIOR, NOISE, SIGMAS = 10.0, 0.5, 2.0
#: Every how many steps a window's rows are read, and a window, s.
EVERY, WINDOW_S = 10, 4.0
#: A box's read eased over this many windows.
BOX_WINDOWS = 3
#: A load is placed where it hangs from this mass up, kg.
HUNG_KG = 0.05


def service(build=None):
    """The bands and the interval of a build's [service]: friction (of its own, and at least N m),
    viscous (of its own, N m s/rad), torque (of itself), play (of its own, up), wind_up (of its
    own, down), load_kg, hours, trend_h (a drift's trend from the last service counts from)."""
    table = (build or mechatronics.robot()).get('service')
    if table is None:
        raise MachineError('%s names no service: a [service] table in machine/robots/%s.toml'
                           % (mechatronics.ROBOT, mechatronics.ROBOT))
    return table


def laid(body, build=None):
    """What may move on `body`'s model, and the filter at rest on it: {lay, body, nominal, names,
    unit, pace, D (P, the theta movers), kt (their drives), band, z, P, hours, serviced_at,
    residual, boxes, box_n, nominal_boxes, drives, service}."""
    import numpy as np
    lay = sysid.laid(body)
    m = body.m
    nominal = sysid.truth(lay, m)
    table = service(build)
    names, kinds, cols, bands = [], [], [], []
    up = np.triu_indices(3)
    for site in SITES:
        b = m.body(site).id
        if b not in lay['col']:
            raise MachineError('%s is no link of hers to load' % site)
        k = lay['col'][b]
        m0, h0 = float(nominal[k]), nominal[k + 1:k + 4]
        shift = [-((float(h0 @ h0) * np.eye(3) - np.outer(h0, h0)) / m0 ** 2)[up]]
        for a in range(3):
            e = np.zeros(3)
            e[a] = 1.0
            shift.append(((2.0 * h0[a] * np.eye(3) - np.outer(e, h0) - np.outer(h0, e)) / m0)[up])
        for q, kind_ in enumerate(KINDS[:4]):
            col = np.zeros(len(nominal))
            col[k + q] = 1.0
            col[k + 4:k + 10] = shift[q]            # its inertia about its origin follows
            names.append('%s.%s' % (site, kind_))
            kinds.append(kind_)
            cols.append(col)
            bands.append(float(table['load_kg']) if kind_ == 'm' else math.inf)
    driven = [m.joint(int(m.dof_jntid[lay['hinge'][j]])).name for j in lay['driven']]
    at = lay['drive']
    for i, name in enumerate(driven):
        f0, b0 = float(nominal[at + 3 * i + 2]), float(nominal[at + 3 * i + 1])
        for q, kind_, own, band in ((2, 'f', f0, table['friction']), (1, 'b', b0, table['viscous'])):
            col = np.zeros(len(nominal))
            col[at + 3 * i + q] = 1.0
            names.append('%s.%s' % (name, kind_))
            kinds.append(kind_)
            cols.append(col)
            bands.append(max(float(band[0]) * abs(own), float(band[1])))
    for name in driven:
        names.append('%s.kt' % name)
        kinds.append('kt')
        bands.append(float(table['torque']))
    unit = np.array([UNIT[k] for k in kinds])
    count = len(names)
    n_d = len(driven)
    return {'lay': lay, 'body': body, 'nominal': nominal, 'names': names, 'kinds': kinds, 'unit': unit,
            'pace': np.array([PACE[k] for k in kinds]), 'D': np.column_stack(cols) * unit[:count - n_d],
            'band': np.array(bands), 'z': np.zeros(count), 'P': PRIOR ** 2 * np.eye(count), 'hours': 0.0,
            'serviced_at': 0.0, 'residual': np.zeros(n_d), 'read': np.zeros(n_d, int), 'drives': driven,
            'armature': nominal[at::3][:n_d].copy(), 'boxes': np.zeros((n_d, 5)), 'box_n': np.zeros(n_d, int),
            'nominal_boxes': np.array([(drives.play(j), drives.wind(j)) for j in driven]), 'service': table}


def window(w, ys, taus, dt):
    """A window's rows against her build: {A (R, movers), left (R,) N m, drive (R,) each row's
    drive, seconds} - her edge arrays `ys` (N + 1, SENSED) and her drives' torques `taus` (N,
    JOINTS) a step `dt` apart, every EVERYth step read where her model holds, a torque's misfit
    the target and each mover's column its share of it: a kt's its drive's predicted torque
    short of its friction, less - whole, it told nothing from the friction's own columns."""
    import numpy as np
    lay, body = w['lay'], w['body']
    Y, tau = sysid.rows(lay, body, ys, taus, dt, EVERY)
    free = sysid.free(body, ys, EVERY)
    Y, tau = Y * free[:, :, None], tau * free
    n, n_d, p = Y.shape
    pred = Y @ w['nominal']
    sliding = Y[:, :, lay['drive']:] @ w['nominal'][lay['drive']:]
    rows_ = Y.reshape(-1, p)
    movers = w['D'].shape[1]
    A = np.zeros((n * n_d, movers + n_d))
    A[:, :movers] = rows_ @ w['D']
    kt = np.zeros((n, n_d, n_d))
    for i in range(n_d):
        kt[:, i, i] = -(pred[:, i] - sliding[:, i]) * UNIT['kt']
    A[:, movers:] = kt.reshape(n * n_d, n_d)
    keep = np.abs(rows_).sum(axis=1) > 0.0
    drive = np.tile(np.arange(n_d), n)
    return {'A': A[keep], 'left': (tau - pred).reshape(-1)[keep], 'drive': drive[keep],
            'seconds': (len(ys) - 1) * dt}


def step(w, got):
    """The filter `got`'s seconds on - each mover walked its PACE - and corrected by its rows,
    each drive's weighed by its misfit the window before (NOISE at least): z and P (units), the
    hours run, the residual and the rows read a drive."""
    import numpy as np
    A, left, drive = got['A'], got['left'], got['drive']
    w['P'] = w['P'] + np.diag(w['pace'] ** 2 * got['seconds'])
    if len(left):
        sigma = np.maximum(NOISE, w['residual'])[drive]
        A, left = A / sigma[:, None], left / sigma
        info = np.linalg.inv(w['P']) + A.T @ A
        w['P'] = np.linalg.inv(info)
        w['z'] = w['z'] + w['P'] @ (A.T @ (left - A @ w['z']))
        r = (left - A @ w['z']) * sigma
        for i in range(len(w['drives'])):
            mine = drive == i
            w['read'][i] = int(mine.sum())
            w['residual'][i] = math.sqrt(float(r[mine] @ r[mine]) / mine.sum()) if mine.any() else 0.0
    w['hours'] += got['seconds'] / 3600.0


def boxed(w, ys, taus, dt):
    """Her boxes off the window (`campaign.boxes`, her build's armature the rotors'): each read
    eased into `boxes` over BOX_WINDOWS where a box wound up; rigid, nothing."""
    import numpy as np
    from machine import campaign
    read = campaign.boxes(w['body'], ys, taus, dt, w['armature'])
    got = read[:, 0] > 0.0
    n = np.minimum(w['box_n'][got] + 1, BOX_WINDOWS)
    w['boxes'][got] += (read[got] - w['boxes'][got]) / n[:, None]
    w['box_n'][got] += 1


def told(w):
    """What the filter has: {load {site: (kg, where it hangs, m, its doubt kg)}, drift {drive: {f
    N m, b N m s/rad, kt of itself}}, doubt {name: its}, boxes {drive: (play of its own, wind-up
    of its own)}, alarms [(name, value, band)], hours, since, left_h, next, residual, read}."""
    import numpy as np
    z, unit = w['z'], w['unit']
    theta, doubt = z * unit, np.sqrt(np.maximum(np.diag(w['P']), 0.0)) * unit
    names, kinds = w['names'], w['kinds']
    load = {}
    for s, site in enumerate(SITES):
        mass, h = float(theta[4 * s]), theta[4 * s + 1:4 * s + 4]
        load[site] = (mass, tuple((h / mass).tolist()) if mass > HUNG_KG else (0.0, 0.0, 0.0),
                      float(doubt[4 * s]))
    drift = {}
    for k, name in enumerate(names):
        who, kind_ = name.rsplit('.', 1)
        if kind_ in ('f', 'b', 'kt'):
            drift.setdefault(who, {})[kind_] = float(theta[k])
    over = np.abs(theta) - SIGMAS * doubt > w['band']
    alarms = [(names[k], float(theta[k]), float(w['band'][k])) for k in np.flatnonzero(over)]
    boxes = {}
    table = w['service']
    for i, name in enumerate(w['drives']):
        if w['box_n'][i] == 0:
            continue
        play0, k0 = w['nominal_boxes'][i]
        play, k = w['boxes'][i, 1] / play0 - 1.0, 1.0 - w['boxes'][i, 0] / k0
        boxes[name] = (float(play), float(k))
        if play > table['play']:
            alarms.append((name + '.play', float(play), float(table['play'])))
        if k > table['wind_up']:
            alarms.append((name + '.wind_up', float(k), float(table['wind_up'])))
    since = w['hours'] - w['serviced_at']
    left_h, nxt = float(table['hours']) - since, 'interval'
    for k, name in enumerate(names):
        if kinds[k] in ('f', 'b', 'kt') and abs(theta[k]) > SIGMAS * doubt[k] and since >= float(table['trend_h']):
            eta = (w['band'][k] - abs(theta[k])) / abs(theta[k]) * since
            if eta < left_h:
                left_h, nxt = eta, name
    for name, value, band in alarms:
        if left_h > 0.0:
            left_h, nxt = 0.0, name
    return {'load': load, 'drift': drift, 'doubt': dict(zip(names, doubt.tolist())), 'boxes': boxes,
            'alarms': alarms, 'hours': w['hours'], 'since': since, 'left_h': max(0.0, left_h), 'next': nxt,
            'residual': w['residual'].tolist(), 'read': w['read'].tolist()}


def serviced(w):
    """A service done: the drifts' trends start anew from here."""
    w['serviced_at'] = w['hours']
