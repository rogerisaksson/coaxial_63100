"""On-line identification under a load: her drift from her build read as she goes, a load told apart.

    w = wear.laid(body)                                  # what may move: a load a site, a drive's f, b, kt
    got = wear.window(w, ys, taus, dt)                   # a window's rows where her model holds
    wear.step(w, got)                                    # the filter walked the window on and corrected
    wear.boxed(w, ys, taus, dt)                          # her boxes' play and wind-up off the window
    wear.told(w)                                         # the load, each drift, the alarms, the service

A load rides a SITE as a mass and where it hangs (a first moment); a drive's Coulomb and viscous
friction move and its torque an amp scales: the movers, each in its UNIT, walking PACE of it a
root second - a load comes and goes in seconds, a drift over hours. Each window's rows are read
where her model holds (`sysid.rows`, `sysid.free`) against her build's prediction, each drive's
rows weighed by their misfit the window before, and a Kalman filter on that random walk keeps
every mover with its doubt. Her build's [service] (`mechatronics`) says the bands: an alarm names
a drift past its band by SIGMAS of its doubt; service is due at the interval's hours or where a
drift's trend from the last service meets its band. The same rows off a robot's boards, its
currents its torques.
"""
import math

from machine import campaign, drives, mechatronics, sysid
from machine.errors import MachineError

#: Where a load may ride: her hands. Her back's load is seen by her drives only when she leans:
#: her insoles' rows (`sysid.borne`) for it.
SITES = ('left_hand', 'right_hand')
#: The movers, in this order: each site's mass and first moment, kg and kg m; each drive's
#: Coulomb friction, N m, and viscous, N m s/rad; each drive's torque an amp, of itself. Each
#: kind's unit, and how far it walks in a second, units a root second.
LOAD, DRIFT, SCALE = ('m', 'hx', 'hy', 'hz'), ('f', 'b'), ('kt',)
UNIT = {'m': 0.1, 'hx': 0.02, 'hy': 0.02, 'hz': 0.02, 'f': 0.1, 'b': 0.05, 'kt': 0.01}
PACE = {'m': 5.0, 'hx': 5.0, 'hy': 5.0, 'hz': 5.0, 'f': 0.5, 'b': 0.5, 'kt': 0.5}
#: Each one's doubt at the start, units; a row's noise, N m; a drift is past its band by this
#: many of its doubt before it is named.
PRIOR, NOISE, SIGMAS = 10.0, 0.5, 2.0
#: Every how many steps a window's rows are read, and a window, s.
EVERY, WINDOW_S = 10, 4.0
#: A box's read eased over this many windows; its play and its wind-up among `campaign.BOX`.
BOX_WINDOWS = 3
PLAY, WIND = campaign.BOX.index('play'), campaign.BOX.index('k')
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
    unit, pace, D (P, the movers but the kt's: theta's move a unit of each), band, z, P, hours,
    serviced_at, residual, read, armature, boxes, box_n, nominal_boxes, service}."""
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
    return {'lay': lay, 'body': body, 'nominal': nominal, 'names': names, 'unit': unit,
            'pace': np.array([PACE[k] for k in kinds]), 'D': D * unit[:D.shape[1]], 'band': band,
            'z': np.zeros(len(names)), 'P': PRIOR ** 2 * np.eye(len(names)), 'hours': 0.0, 'serviced_at': 0.0,
            'residual': np.zeros(n_d), 'read': np.zeros(n_d, int), 'armature': friction[:, 0].copy(),
            'boxes': np.zeros((n_d, len(campaign.BOX))), 'box_n': np.zeros(n_d, int),
            'nominal_boxes': np.array([(drives.play(j), drives.wind(j)) for j in names_d]), 'service': table}


def window(w, ys, taus, dt):
    """A window's rows against her build: {A (R, movers), left (R,) N m, drive (R,) each row's
    drive, seconds} - her edge arrays `ys` (N + 1, SENSED) and her drives' torques `taus` (N,
    JOINTS) a step `dt` apart, every EVERYth step read where her model holds, a torque's misfit
    the target and each mover's column its share of it: a kt's its drive's predicted torque
    short of its friction, less - whole, it told nothing from the friction's own columns."""
    import numpy as np
    lay = w['lay']
    Y, tau = sysid.rows(lay, w['body'], ys, taus, dt, EVERY)
    held = sysid.free(lay, ys, EVERY)
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
    return {'A': A[keep], 'left': (tau.reshape(-1) - pred)[keep], 'drive': drive[keep],
            'seconds': (len(ys) - 1) * dt}


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
    w['hours'] += got['seconds'] / 3600.0


def boxed(w, ys, taus, dt):
    """Her boxes off the window (`campaign.boxes`, her build's armature the rotors'): each read
    eased into `boxes` over BOX_WINDOWS where a box wound up; rigid, nothing."""
    import numpy as np
    read = campaign.boxes(w['body'], ys, taus, dt, w['armature'])
    got = read[:, WIND] > 0.0
    n = np.minimum(w['box_n'][got] + 1, BOX_WINDOWS)
    w['boxes'][got] += (read[got] - w['boxes'][got]) / n[:, None]
    w['box_n'][got] += 1


def told(w):
    """What the filter has: {load {site: (kg, where it hangs, m, its doubt kg)}, drift {drive: {f
    N m, b N m s/rad, kt of itself}}, doubt {name: its}, boxes {drive: (play of its own, wind-up
    of its own)}, alarms [(name, value, band)], hours, since, left_h, next, residual, read}."""
    import numpy as np
    names, names_d, table = w['names'], w['lay']['drives'], w['service']
    theta, doubt = w['z'] * w['unit'], np.sqrt(np.maximum(np.diag(w['P']), 0.0)) * w['unit']
    first, n_d = 4 * len(SITES), len(names_d)
    load = {}
    for s, site in enumerate(SITES):
        mass, h = float(theta[4 * s]), theta[4 * s + 1:4 * s + 4]
        load[site] = (mass, tuple((h / mass).tolist()) if mass > HUNG_KG else (0.0, 0.0, 0.0),
                      float(doubt[4 * s]))
    drift = {name: {'f': float(theta[first + 2 * i]), 'b': float(theta[first + 2 * i + 1]),
                    'kt': float(theta[first + 2 * n_d + i])} for i, name in enumerate(names_d)}
    over = np.abs(theta) - SIGMAS * doubt > w['band']
    alarms = [(names[k], float(theta[k]), float(w['band'][k])) for k in np.flatnonzero(over)]
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
    (`sysid.borne`) against her build's prediction."""
    import numpy as np
    lay, body = w['lay'], w['body']
    Yb, fb = sysid.borne(lay, body, ys, dt, EVERY)
    b = body.m.body('torso').id
    k = lay['col'][b]
    A = Yb[:, :, k:k + 4].reshape(-1, 4)
    left = (fb - Yb @ w['nominal']).reshape(-1)
    theta, *_ = np.linalg.lstsq(A, left, rcond=None)
    mass = float(theta[0])
    where = tuple((theta[1:4] / mass).tolist()) if abs(mass) > HUNG_KG else (0.0, 0.0, 0.0)
    return mass, where

