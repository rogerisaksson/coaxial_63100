"""Her figure's physics as static arrays: a link a degree of freedom, from the compiled figure.

    python tools/cores/model.py --write     # wbc/inc/wbc_model.h, wbc/src/wbc_model.c
    rows = model.links()                    # what the C holds, a dict a link

Link 0 is her pelvis, floating; each hinge after it a link in MuJoCo's order: its parent link,
its frame at rest in the parent's (R, p), its screw in its own frame (w, v), the spatial inertia
of what it carries (nothing for the first joint of a body with two), its mass and centre, the
rotor's armature, its stops, its spring and damper, whether a drive turns it, a stop holds it
or nothing does, the drive's clamp, peak and kt, the drive it shares its rods with. The soles'
points, the knees, the trunk and her mass go with them. Her chain is the body form of the
product of exponentials, T = T_parent X exp([S] q); the firmware runs it (`wbc/`), MuJoCo stays
the reference (tests/test_wbc_core.py).
"""
import importlib
import math
import os
import sys

import numpy as np

from tools import REPO

WBC = os.path.join(REPO, 'wbc')
HEADER = os.path.join(WBC, 'inc', 'wbc_model.h')
SOURCE = os.path.join(WBC, 'src', 'wbc_model.c')
NOTICE = 'her figure as static arrays: written by host/tools/cores/model.py, not by hand.'

#: A link's kind, as the C reads it.
FREE, DRIVEN, HELD = 0, 1, 2


def rotation(quat):
    """The 3x3 of a MuJoCo quaternion (w, x, y, z)."""
    w, x, y, z = (float(c) for c in quat)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def skew(a):
    return np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])


def inertia(m, body):
    """A body's spatial inertia in its own frame, 6x6 over (w, v): its mass, its centre, its
    inertia about the centre."""
    mass = float(m.body_mass[body])
    c = skew(np.array(m.body_ipos[body], float))
    r = rotation(m.body_iquat[body])
    g = np.zeros((6, 6))
    g[:3, :3] = r @ np.diag(np.array(m.body_inertia[body], float)) @ r.T - mass * c @ c
    g[:3, 3:] = mass * c
    g[3:, :3] = -mass * c
    g[3:, 3:] = mass * np.eye(3)
    return g


def links(b=None):
    """The links, link 0 the pelvis: {name, parent, R, p, S, G, mass, com, armature, stop,
    passive (k, d, rest), kind, clamp, peak, kt, pair, body, frame} - `frame` where the link's
    frame is its MuJoCo body's (its last joint); `pair` the link it shares its rods with plus
    one, negative on the second of the two, 0 alone."""
    mujoco = importlib.import_module('mujoco')
    if b is None:
        from machine import wbc
        b = wbc.Body()
    m = b.m
    pelvis = m.body('pelvis').id
    act, held = [int(d) for d in b.act], [int(d) for d in b.held]
    rows, last = [], {}
    for body in range(m.nbody):
        up = body
        while up and up != pelvis:
            up = int(m.body_parentid[up])
        if up != pelvis:
            continue
        first = int(m.body_jntadr[body])
        joints = range(first, first + int(m.body_jntnum[body]))
        for k, j in enumerate(joints):
            final = k == len(joints) - 1
            dof = int(m.jnt_dofadr[j])
            row = {'name': m.joint(j).name, 'body': body, 'frame': final,
                   'armature': float(m.dof_armature[dof]),
                   'stop': _stop(m, j),
                   'passive': (float(m.jnt_stiffness[j]), float(m.dof_damping[dof]),
                               float(m.qpos_spring[m.jnt_qposadr[j]])),
                   'G': inertia(m, body) if final else np.zeros((6, 6)),
                   'mass': float(m.body_mass[body]) if final else 0.0,
                   'com': np.array(m.body_ipos[body], float) if final else np.zeros(3),
                   'kind': DRIVEN if dof in act else HELD if dof in held else FREE,
                   'clamp': 0.0, 'peak': 0.0, 'kt': 0.0, 'pair': 0}
            if dof in act:
                i = act.index(dof)
                row.update(clamp=float(b.clamp[i]), peak=float(b.wep[i]), kt=float(b.kt[i]))
            if body == pelvis:
                assert m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE and final
                row.update(parent=-1, R=np.eye(3), p=np.zeros(3), S=np.zeros(6))
            else:
                assert m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
                assert dof == 5 + len(rows)
                a, c = np.array(m.jnt_axis[j], float), np.array(m.jnt_pos[j], float)
                row.update(parent=last[int(m.body_parentid[body])] if k == 0 else len(rows) - 1,
                           R=rotation(m.body_quat[body]) if k == 0 else np.eye(3),
                           p=np.array(m.body_pos[body], float) if k == 0 else np.zeros(3),
                           S=np.concatenate([a, np.cross(c, a)]))
            rows.append(row)
            last[body] = len(rows) - 1
    for p, r in b.pairs:
        one, two = act[p] - 5, act[r] - 5
        rows[one]['pair'], rows[two]['pair'] = two + 1, -(one + 1)
    return rows


def _stop(m, j):
    """A joint's stops, rad: the model's, else the stack's own ranges (`wbc.RANGES`), else
    none."""
    from machine import wbc
    from machine.drives import kind
    if m.jnt_limited[j]:
        return tuple(float(x) for x in m.jnt_range[j])
    own = wbc.RANGES.get(kind(m.joint(j).name))
    return tuple(math.radians(d) for d in own) if own else None


def _nums(xs, per=6, indent='   '):
    xs = [repr(float(x)) for x in xs]
    lines = [', '.join(xs[i:i + per]) for i in range(0, len(xs), per)]
    return (',\n' + indent).join(lines)


def _ints(name, xs):
    return 'const int %s = {%s};' % (name, ', '.join(str(int(x)) for x in xs))


def _flat(name, rows, cell):
    return 'const double %s[WBC_LINKS] = {\n   %s};' % (name, _nums([cell(r) for r in rows]))


def _table(name, width, rows, cells):
    body = ',\n'.join('  /* %s */\n  {%s}' % (r['name'], _nums(cells(r))) for r in rows)
    return 'const double %s[WBC_LINKS][%d] = {\n%s\n};\n' % (name, width, body)


def render(rows, b):
    """(header, source) for `rows`, her `wbc.Body`'s."""
    n = 5 + len(rows)
    names = [r['name'] for r in rows]
    driven = [names.index(wbc_name) for wbc_name in _driven(b)]
    held = [k for k, r in enumerate(rows) if r['kind'] == HELD]
    header = '\n'.join([
        '/** wbc_model.h - %s */' % NOTICE,
        '#ifndef WBC_MODEL_H', '#define WBC_MODEL_H', '',
        '/** Links, link 0 the floating pelvis; her degrees of freedom, 6 of them the pelvis\'s:',
        '    link k > 0 is u[5 + k]; the drives and the held joints, as links. */',
        '#define WBC_LINKS %d' % len(rows),
        '#define WBC_N %d' % n,
        '#define WBC_DRIVEN %d' % len(driven),
        '#define WBC_HELD %d' % len(held), '',
        '/** A link\'s kind (wbc_kind). */',
        '#define WBC_FREE %d' % FREE, '#define WBC_IS_DRIVEN %d' % DRIVEN,
        '#define WBC_IS_HELD %d' % HELD, '',
        'extern const int    wbc_parent[WBC_LINKS];   /**< a link\'s parent; -1, the world, under the pelvis */',
        'extern const double wbc_x[WBC_LINKS][12];    /**< its frame at rest in the parent\'s: R by rows, then p */',
        'extern const double wbc_s[WBC_LINKS][6];     /**< its screw in its own frame, (w, v) */',
        'extern const double wbc_g[WBC_LINKS][36];    /**< the spatial inertia it carries, by rows, over (w, v) */',
        'extern const double wbc_mass[WBC_LINKS];     /**< the mass it carries, kg */',
        'extern const double wbc_com[WBC_LINKS][3];   /**< its centre, in its frame */',
        'extern const double wbc_armature[WBC_LINKS]; /**< the rotor\'s through the box, kg m^2 */',
        'extern const double wbc_stop[WBC_LINKS][2];  /**< its stops, rad; lo > hi: none */',
        'extern const double wbc_passive[WBC_LINKS][3]; /**< its spring N m/rad, damper N m s/rad, rest rad */',
        'extern const int    wbc_kind[WBC_LINKS];     /**< free, driven or held */',
        'extern const double wbc_clamp[WBC_LINKS];    /**< its drive\'s clamp, N m; 0 undriven */',
        'extern const double wbc_peak[WBC_LINKS];     /**< its drive\'s peak at its board\'s amps, N m */',
        'extern const double wbc_kt[WBC_LINKS];       /**< its drive\'s N m an ampere at the joint */',
        'extern const int    wbc_pair[WBC_LINKS];     /**< the link sharing its rods + 1, negative on the second; 0 alone */',
        'extern const int    wbc_driven[WBC_DRIVEN];  /**< the drives\' links, in the drives\' order */',
        'extern const int    wbc_held[WBC_HELD];      /**< the held joints\' links */',
        'extern const int    wbc_sole[2];             /**< the feet\'s links, left and right */',
        'extern const int    wbc_knee[2];             /**< the knees\' links */',
        'extern const int    wbc_trunk;               /**< the trunk\'s link */',
        'extern const double wbc_sole_at[3];          /**< a sole\'s middle, in its foot\'s frame */',
        'extern const double wbc_sole_corner[4][3];   /**< its corners */',
        'extern const double wbc_total_mass;          /**< kg */',
        'extern const double wbc_gravity[3];          /**< m/s^2, world frame */',
        'extern const char *const wbc_name[WBC_LINKS]; /**< the joint\'s name */', '',
        '#endif', ''])
    source = '\n'.join([
        '/** wbc_model.c - %s */' % NOTICE,
        '#include "wbc_model.h"', '',
        'const double wbc_gravity[3] = {%s};' % _nums(b.m.opt.gravity),
        'const double wbc_total_mass = %r;' % float(b.mass), '',
        _ints('wbc_parent[WBC_LINKS]', [r['parent'] for r in rows]),
        _ints('wbc_kind[WBC_LINKS]', [r['kind'] for r in rows]),
        _ints('wbc_pair[WBC_LINKS]', [r['pair'] for r in rows]),
        _ints('wbc_driven[WBC_DRIVEN]', driven),
        _ints('wbc_held[WBC_HELD]', held),
        _ints('wbc_sole[2]', [names.index(s) for s in _soles(b, names)]),
        _ints('wbc_knee[2]', [names.index(s) for s in ('left_knee', 'right_knee')]),
        'const int wbc_trunk = %d;' % names.index(_last_joint(b, 'torso')), '',
        'const double wbc_sole_at[3] = {%s};' % _nums(b.sole),
        'const double wbc_sole_corner[4][3] = {\n  %s};'
        % (',\n  '.join('{%s}' % _nums(c) for c in b.corners)), '',
        'const char *const wbc_name[WBC_LINKS] = {\n  %s};' % (',\n  '.join('"%s"' % s for s in names)),
        '', _flat('wbc_mass', rows, lambda r: r['mass']),
        _flat('wbc_armature', rows, lambda r: r['armature']),
        _flat('wbc_clamp', rows, lambda r: r['clamp']),
        _flat('wbc_peak', rows, lambda r: r['peak']),
        _flat('wbc_kt', rows, lambda r: r['kt']), '',
        'const double wbc_stop[WBC_LINKS][2] = {\n  %s};'
        % (',\n  '.join('{%s}' % _nums(r['stop'] or (1.0, -1.0)) for r in rows)), '',
        'const double wbc_passive[WBC_LINKS][3] = {\n  %s};'
        % (',\n  '.join('{%s}' % _nums(r['passive']) for r in rows)), '',
        'const double wbc_com[WBC_LINKS][3] = {\n  %s};'
        % (',\n  '.join('{%s}' % _nums(r['com']) for r in rows)), '',
        _table('wbc_x', 12, rows, lambda r: list(r['R'].ravel()) + list(r['p'])),
        _table('wbc_s', 6, rows, lambda r: r['S']),
        _table('wbc_g', 36, rows, lambda r: r['G'].ravel())])
    return header, source


def _driven(b):
    """The drives' joint names in `b.act`'s order."""
    from machine import drives
    from machine.figure import JOINTS
    return [j for j in JOINTS if drives.passive(j) is None]


def _last_joint(b, body):
    """The name of `body`'s last joint: the one whose link frame is the body's."""
    m = b.m
    i = m.body(body).id
    return m.joint(int(m.body_jntadr[i]) + int(m.body_jntnum[i]) - 1).name


def _soles(b, names):
    return [_last_joint(b, side + '_foot') for side in ('left', 'right')]


def write(b=None):
    """The header and the source written; the rows they hold."""
    from machine import wbc
    b = b or wbc.Body()
    rows = links(b)
    for path, text in zip((HEADER, SOURCE), render(rows, b)):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
    return rows


if __name__ == '__main__':
    if '--write' not in sys.argv[1:]:
        sys.exit(__doc__)
    print('%d links written' % len(write()))
