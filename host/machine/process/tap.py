"""Her loop's side of a tuner apart: her raw passes tapped, a table taken and laid at her barrier.

    state = tap.fresh()                          # a tap's state, held between passes
    x = tap.raw(s, state)                        # a pass of her session `s`, raw (RAW): her loop's copy
    samples = tap.cooked(raws)                   # passes as samples (SAMPLE): the tuner's work, in bulk
    m = tap.measured(samples, weight)            # a window's measures of its samples
    hook, link = tap.attached(name)              # her loop linked: `worker` calls hook(s) each pass

Her loop copies, the tuner computes: a raw pass is what the autotuner's classes need of her world,
copied as it stands - her contacts with whatever bears her, in C beside her contacts' read
(`wbc_pass.c`; CONTACT, CONTACTS of them: each one's place and frame, its force and tangential
velocity the solver's rows, her cones elliptic, its friction), her centre of mass, her master bus's
tank, her drives' watts, her feet's gap every ROW_S. Cooked, a sample holds the world's force on her, the centroidal pivot's margin in her contacts'
hull, her contacts' slip and their friction cone's use. A table the tuner publishes (`rcu`: a row a
column, `ortho.ROW`'s fields) is laid where it cannot jerk her (`due`): at her lateral voice's zero
crossing with both soles locked and her last blend done, or at rest her last blend done - its own
blend the minimum-jerk quintic (SHAPE), her voices' targets C2 through it.
"""
import ctypes
import math

#: A raw pass's head: its time and the sequence laid, the tank (J), her drives' watts, her centre of
#: mass (m), her feet's gap (mm, nan between its ROW_S), how many contacts she had, when she fell (s,
#: nan standing; her session's).
HEAD = ('t', 'laid', 'tank', 'watts', 'cx', 'cy', 'cz', 'gap', 'n', 'fell')
#: A raw contact: whether it is her first geom's, its place, its frame (normal, two tangents), its
#: force in it (N), its friction, its tangential velocity (m/s, the second geom's on the first's).
CONTACT = ('hers1', 'px', 'py', 'pz', 'nx', 'ny', 'nz', 'ax', 'ay', 'az', 'bx', 'by', 'bz', 'fn', 'fa', 'fb',
           'mu', 'va', 'vb')
#: Her contacts a raw pass holds: more are cut, their count kept in `n`.
CONTACTS = 16
RAW = len(HEAD) + CONTACTS * len(CONTACT)
HE, C = {name: i for i, name in enumerate(HEAD)}, {name: i for i, name in enumerate(CONTACT)}
#: Where `wbc_pass_tapped` writes a contact's fields: hers first, place, frame, force, friction, velocity.
AT = (C['hers1'], C['px'], C['nx'], C['fn'], C['mu'], C['va'])
#: A sample's fields: its time and the sequence laid, the world's force on her (N), her centre of
#: mass (m), the pivot's margin (mm, nan unborne), slip (m/s), the cone's use, the tank (J), her drives'
#: watts, her feet's gap (mm, nan between its ROW_S).
SAMPLE = ('t', 'laid', 'fx', 'fy', 'fz', 'cx', 'cy', 'cz', 'pivot', 'slip', 'cone', 'tank', 'watts', 'gap')
S = {name: i for i, name in enumerate(SAMPLE)}
#: A gap's and a look's sampling, s: the page's rows' (`look.RATE_HZ`).
ROW_S = 0.01
#: The feet a gap is between (`look`'s).
FEET = (('left_foot', 'left_toes'), ('right_foot', 'right_toes'))
#: A look's fields, the second ring's: what her page's row is made of (`humanoid_keys.row`) - her time,
#: speed, soles' loads, place and turn, each joint's angle (deg, `figure.JOINTS`).
LOOK = ('t', 'speed', 'left_load', 'right_load', 'x', 'y', 'z', 'qw', 'qx', 'qy', 'qz')
#: A table's blend: the minimum-jerk quintic, its rate and bend 0 at both ends (`curves.QUINTICS`).
SHAPE = 'minimum_jerk_deceleration'


def margin(p, points):
    """How far point `p` lies inside the hull of `points` (`sight.convex`): negative outside; the
    hull of one or two points a point or a segment, everything off it outside."""
    from machine.eyes import sight
    hull = sight.convex(points)
    if len(hull) < 3:
        a, b = hull[0], hull[-1]
        ab = (b[0] - a[0], b[1] - a[1])
        t = max(0.0, min(1.0, ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / max(ab[0] ** 2 + ab[1] ** 2, 1e-300)))
        return -math.hypot(p[0] - a[0] - t * ab[0], p[1] - a[1] - t * ab[1])
    return min(((p[1] - a[1]) * (b[0] - a[0]) - (p[0] - a[0]) * (b[1] - a[1])) / math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(hull, hull[1:] + hull[:1]))


def fresh():
    """A tap's state: her world, its contacts' read and the C tap (bound at a world's first pass: a
    fall lays another), its gap's last time, the sequence laid."""
    return {'world': None, 'gap_t': -math.inf, 'laid': 0}


def raw(s, state, x=None):
    """A pass of her session `s` as it stands (RAW), into `x` where given, its tap's `state`
    (`fresh`) carried on: copies, nothing computed but her feet's gap every ROW_S."""
    import numpy as np
    from machine.senses import edge
    g = s['g']
    world, t = g['world'], g['t']
    d = world.data
    if state['world'] is not world:
        from tools.cores import wbc as cores
        state.update(world=world, gap_t=-math.inf, c=world.contacts[0], at=np.array(AT, np.int32),
                     tapped=cores.library().wbh_pass_tapped, address=cores.address)
    x = np.zeros(RAW) if x is None else x
    ncon, n = d.ncon, 0
    if ncon:
        con, c, address = d.contact, state['c'], state['address']
        edge.layout(c, con)
        n = state['tapped'](ctypes.byref(c), ncon, address(con.pos), address(d.efc_force), address(d.efc_vel),
                            address(state['at']), len(CONTACT), CONTACTS, address(x) + 8 * len(HEAD))
    gap = math.nan
    if t - state['gap_t'] >= ROW_S - 1e-9:
        gap, state['gap_t'] = 1e3 * world.gap(*FEET), t
    x[:len(HEAD)] = (t, state['laid'], float(g['bus'][1].energy), world.drawn(), *d.subtree_com[world.pelvis], gap, n,
                     math.nan if s['fallen_at'] is None else s['fallen_at'])
    return x


def cooked(raws):
    """Raw passes (n, RAW) as samples (n, SAMPLE), in bulk: the world's force on her, each contact's
    force out of its frame, signed to her side; slip her contacts' tangential speed and the cone's use
    |F_t| over mu F_n, each weighed by the normal forces; the pivot - her centre of mass's line along
    that force to her contacts' height - its margin in their hull."""
    import numpy as np
    x = np.atleast_2d(np.asarray(raws, float))
    head, c = x[:, :len(HEAD)], x[:, len(HEAD):].reshape(len(x), CONTACTS, len(CONTACT))
    held = np.arange(CONTACTS)[None, :] < np.minimum(head[:, HE['n']], CONTACTS)[:, None]
    frame = c[..., C['nx']:C['bz'] + 1].reshape(len(x), CONTACTS, 3, 3)
    f = c[..., C['fn']:C['fb'] + 1] * held[..., None]
    f_on = np.einsum('nkji,nkj->ni', frame, f * np.where(c[..., C['hers1']] > 0, -1.0, 1.0)[..., None])
    normal = np.abs(f[..., 0])
    borne, grip = normal.sum(1), (c[..., C['mu']] * normal).sum(1)
    slip = (normal * np.hypot(c[..., C['va']], c[..., C['vb']])).sum(1) / np.where(borne > 0.0, borne, 1.0)
    cone = np.hypot(f[..., 1], f[..., 2]).sum(1) / np.where(grip > 0.0, grip, 1.0)
    pivot = np.full(len(x), math.nan)
    for i in np.flatnonzero((f_on[:, 1] > 0.0) & held.any(1)):
        pts, com = c[i, held[i]][:, [C['px'], C['pz']]], head[i, HE['cx']:HE['cz'] + 1]
        k = (com[1] - float(c[i, held[i], C['py']].mean())) / f_on[i, 1]
        pivot[i] = 1e3 * margin((com[0] - k * f_on[i, 0], com[2] - k * f_on[i, 2]), list(map(tuple, pts.tolist())))
    return np.column_stack([head[:, HE['t']], head[:, HE['laid']], f_on, head[:, HE['cx']:HE['cz'] + 1], pivot, slip, cone,
                            head[:, HE['tank']], head[:, HE['watts']], head[:, HE['gap']]])


def look(s):
    """Her look now (LOOK, then each joint's angle in `figure.JOINTS`' order)."""
    import numpy as np
    g = s['g']
    world = g['world']
    d = world.data
    return np.concatenate([[g['t'], float(d.qvel[2]), *(float(v) for v in g['loads'])], d.qpos[0:7],
                           np.degrees(d.qpos[world.qadr])])


def measured(samples, weight):
    """A window's measures of its `samples` (n, SAMPLE), her weight `weight` N: the pivot's margins
    (mm, borne), her load over her weight least and most, the friction cone's use (mean, borne), her
    slip (mm), her feet's least gap (mm), the tank's bleed (J: what drew it down), her drives' energy a
    metre of her centre of mass's way (J/m)."""
    import numpy as np
    x = np.asarray(samples, float)
    dt = np.diff(x[:, S['t']], prepend=x[0, S['t']])
    load, borne = x[:, S['fy']] / weight, x[:, S['fy']] > 0.0
    way = float(np.sum(np.linalg.norm(np.diff(x[:, S['cx']:S['cz'] + 1], axis=0), axis=1)))
    pivot, gap = x[:, S['pivot']], x[:, S['gap']]
    return {'pivot': pivot[np.isfinite(pivot)], 'load': float(load.min()), 'peak': float(load.max()),
            'cone': float(x[borne, S['cone']].mean()) if borne.any() else 0.0, 'slip': float(1e3 * np.sum(x[:, S['slip']] * dt)),
            'gap': float(np.min(gap[np.isfinite(gap)])), 'bleed': float(np.sum(np.maximum(0.0, -np.diff(x[:, S['tank']])))),
            'energy': float(np.sum(x[:, S['watts']] * dt)) / way}


def due(g, at):
    """(whether a table is laid now, the time to look from): at rest - her rhythm off - once her last
    blend is done; walking, past her lateral voice's zero crossing `at` with both soles locked and her
    last blend done (`patch.locked`), past its double support unlocked the next crossing."""
    from machine.law import latent
    from machine.texts import patch
    st, t = g['st'], g['t']
    done = t >= float(st[latent.T0] + st[latent.BLEND])
    if float(st[latent.NOW][latent.GAIN]) < 0.5:
        return done, at
    if t < at:
        return False, at
    if done and patch.locked(g['y'], g['fr'], g['a']):
        return True, at
    return False, at if t <= at + patch.window(st) else patch.crossing(st, t + 1e-3)


def table(row, blend_s, rid):
    """A one-row table (`rcu`'s: `ortho.ROW`'s fields a column) of `row` ({slot: value}) over her walk,
    id `rid`, her rhythm on, blended over `blend_s` minimum-jerk (SHAPE)."""
    import numpy as np
    from machine.texts import ortho
    p = np.full(ortho.P, np.nan)
    p[ortho.GAIN] = 1.0
    p[list(row)] = list(row.values())
    head = [rid, 0, int(round(1e3 * blend_s)), ortho.SHAPES.index(SHAPE) << ortho.SHAPE_BIT]
    return np.concatenate([head, p])[:, None]


def wire(table):
    """A published table's rows (`ortho.ROW`) as her table's wire: its columns id, at ms, blend ms,
    flags and the parameters."""
    from machine.texts import ortho
    return b''.join(ortho.ROW.pack(*(int(v) for v in row[:4]), *(float(v) for v in row[4:])) for row in table.T)


def made(seconds, rows=1):
    """The tuner's link to her loop (`rcu.Link.made`): rings for `seconds` of her raw passes, one a
    pass, and of her looks, one a ROW_S - the time the tuner may take between its drains -; banks of
    `rows` rows of her row's fields."""
    from machine.texts import ortho
    from machine.process import rcu
    from machine.body.figure import JOINTS
    return rcu.Link.made(((RAW, int(round(seconds * 1e3))), (len(LOOK) + len(JOINTS), int(round(seconds / ROW_S)))),
                         4 + ortho.P, rows)


def attached(name):
    """(her loop's hook - `worker` calls it each pass -, its link): her raw pass tapped onto the link
    `name`'s first ring (`rcu`), her look onto its second with her gap, every ROW_S; a table taken,
    laid when `due` from her next lateral crossing on, its laying said in the link and in her passes."""
    from machine.law import latent
    from machine.texts import patch
    from machine.process import rcu, worker
    link, state = rcu.Link.found(name), fresh()
    held = {'seq': None, 'table': None, 'at': math.inf}

    def hook(s):
        g = s['g']
        link.tap(raw(s, state))
        if state['gap_t'] == g['t']:
            link.tap(look(s), 1)
        got = link.taken() if held['table'] is None else None
        if got is not None:
            rest = float(g['st'][latent.NOW][latent.GAIN]) < 0.5
            held.update(seq=got[0], table=got[1], at=g['t'] if rest else patch.crossing(g['st'], g['t']))
        if held['table'] is None:
            return
        lay, held['at'] = due(g, held['at'])
        if lay:
            worker.apply(s, {'entries': [('patch', wire(held['table']), ('patch',))]})
            link.said('laid', held['seq'])
            state['laid'] = held['seq']
            held.update(seq=None, table=None, at=math.inf)
    return hook, link
