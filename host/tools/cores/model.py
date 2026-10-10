"""Her figure's physics as static arrays: a link a degree of freedom, from the compiled figure.

    python tools/cores/model.py --write     # wbc/inc/wbc_model.h, wbc/src/wbc_model.c
    rows = model.links()                    # what the C holds, a dict a link

Link 0 is her pelvis, floating; each hinge after it a link in MuJoCo's order: its parent link,
its frame at rest in the parent's (R, p), its screw in its own frame (w, v), the spatial inertia
of what it carries (nothing for the first joint of a body with two) and the rotor's armature,
its stops. Her chain is the body form of the product of exponentials, T = T_parent X exp([S] q);
the firmware runs it (`wbc/`), MuJoCo stays the reference (tests/test_wbc_core.py).
"""
import importlib
import os
import sys

import numpy as np

from tools import REPO

WBC = os.path.join(REPO, 'wbc')
HEADER = os.path.join(WBC, 'inc', 'wbc_model.h')
SOURCE = os.path.join(WBC, 'src', 'wbc_model.c')
NOTICE = 'her figure as static arrays: written by host/tools/cores/model.py, not by hand.'


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


def links(m=None):
    """The links, link 0 the pelvis: {name, parent, R, p, S, G, armature, stop, body, frame} -
    `frame` where the link's frame is its MuJoCo body's (the body's last joint)."""
    mujoco = importlib.import_module('mujoco')
    if m is None:
        from machine import wbc
        m = wbc.Body().m
    pelvis = m.body('pelvis').id
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
            row = {'name': m.joint(j).name, 'body': body, 'frame': final,
                   'armature': float(m.dof_armature[m.jnt_dofadr[j]]),
                   'stop': tuple(float(x) for x in m.jnt_range[j]) if m.jnt_limited[j] else None,
                   'G': inertia(m, body) if final else np.zeros((6, 6))}
            if body == pelvis:
                assert m.jnt_type[j] == mujoco.mjtJoint.mjJNT_FREE and final
                row.update(parent=-1, R=np.eye(3), p=np.zeros(3), S=np.zeros(6))
            else:
                assert m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
                assert int(m.jnt_dofadr[j]) == 5 + len(rows)
                a, c = np.array(m.jnt_axis[j], float), np.array(m.jnt_pos[j], float)
                row.update(parent=last[int(m.body_parentid[body])] if k == 0 else len(rows) - 1,
                           R=rotation(m.body_quat[body]) if k == 0 else np.eye(3),
                           p=np.array(m.body_pos[body], float) if k == 0 else np.zeros(3),
                           S=np.concatenate([a, np.cross(c, a)]))
            rows.append(row)
            last[body] = len(rows) - 1
    return rows


def _nums(xs, per=6, indent='   '):
    xs = [repr(float(x)) for x in xs]
    lines = [', '.join(xs[i:i + per]) for i in range(0, len(xs), per)]
    return (',\n' + indent).join(lines)


def _table(name, width, rows, cells):
    body = ',\n'.join('  /* %s */\n  {%s}' % (r['name'], _nums(cells(r))) for r in rows)
    return 'const double %s[WBC_LINKS][%d] = {\n%s\n};\n' % (name, width, body)


def render(rows, gravity):
    """(header, source) for `rows` under `gravity` (world frame, m/s^2)."""
    n = 5 + len(rows)
    header = '\n'.join([
        '/** wbc_model.h - %s */' % NOTICE,
        '#ifndef WBC_MODEL_H', '#define WBC_MODEL_H', '',
        '/** Links, link 0 the floating pelvis; her degrees of freedom, 6 of them the pelvis\'s:',
        '    link k > 0 is u[5 + k]. */',
        '#define WBC_LINKS %d' % len(rows),
        '#define WBC_N %d' % n, '',
        'extern const int    wbc_parent[WBC_LINKS];   /**< a link\'s parent; -1, the world, under the pelvis */',
        'extern const double wbc_x[WBC_LINKS][12];    /**< its frame at rest in the parent\'s: R by rows, then p */',
        'extern const double wbc_s[WBC_LINKS][6];     /**< its screw in its own frame, (w, v) */',
        'extern const double wbc_g[WBC_LINKS][36];    /**< the spatial inertia it carries, by rows, over (w, v) */',
        'extern const double wbc_armature[WBC_LINKS]; /**< the rotor\'s through the box, kg m^2 */',
        'extern const double wbc_stop[WBC_LINKS][2];  /**< its stops, rad; lo > hi: none */',
        'extern const double wbc_gravity[3];          /**< m/s^2, world frame */',
        'extern const char *const wbc_name[WBC_LINKS]; /**< the joint\'s name */', '',
        '#endif', ''])
    source = '\n'.join([
        '/** wbc_model.c - %s */' % NOTICE,
        '#include "wbc_model.h"', '',
        'const double wbc_gravity[3] = {%s};' % _nums(gravity), '',
        'const int wbc_parent[WBC_LINKS] = {%s};' % ', '.join(str(r['parent']) for r in rows), '',
        'const char *const wbc_name[WBC_LINKS] = {\n  %s};'
        % (',\n  '.join('"%s"' % r['name'] for r in rows)), '',
        'const double wbc_armature[WBC_LINKS] = {\n   %s};' % _nums([r['armature'] for r in rows]), '',
        'const double wbc_stop[WBC_LINKS][2] = {\n  %s};'
        % (',\n  '.join('{%s}' % _nums(r['stop'] or (1.0, -1.0)) for r in rows)), '',
        _table('wbc_x', 12, rows, lambda r: list(r['R'].ravel()) + list(r['p'])),
        _table('wbc_s', 6, rows, lambda r: r['S']),
        _table('wbc_g', 36, rows, lambda r: r['G'].ravel())])
    return header, source


def write(m=None):
    """The header and the source written; the rows they hold."""
    from machine import wbc
    m = m or wbc.Body().m
    rows = links(m)
    for path, text in zip((HEADER, SOURCE), render(rows, m.opt.gravity)):
        with open(path, 'w', encoding='utf-8', newline='\n') as f:
            f.write(text)
    return rows


if __name__ == '__main__':
    if '--write' not in sys.argv[1:]:
        sys.exit(__doc__)
    print('%d links written' % len(write()))
