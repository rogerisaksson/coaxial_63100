"""Her chain on the static model (wbc/) against MuJoCo, and a step's time on this host.

The arrays as the figure compiles; the core's poses, Jacobians, mass matrix and bias at random
configurations."""
import ctypes
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.cores import model  # noqa: E402
from tools.cores import wbc as core  # noqa: E402
from tools.cores.build import build, find_cc  # noqa: E402
from tools.dev.focus import chosen  # noqa: E402
from gynoid_kit import Report  # noqa: E402

#: Configurations compared, and how far the core may sit from MuJoCo (m, kg m^2, N m).
POSES, APART = 40, 1e-9

#: Links whose Jacobians are compared, at the soles' corners, their middles and the origins.
POINTED = ('left_ankle_roll', 'right_ankle_roll', 'left_wrist', 'right_wrist', 'head', 'root')


def _configurations(b, rows, rng):
    """(qpos, qvel) within the stops, the pelvis anywhere, for POSES draws."""
    import numpy as np
    for _ in range(POSES):
        q = np.zeros(b.m.nq)
        quat = rng.normal(size=4)
        q[:3], q[3:7] = rng.normal(size=3), quat / np.linalg.norm(quat)
        for k, row in enumerate(rows[1:]):
            lo, hi = row['stop'] or (-1.5, 1.5)
            q[7 + k] = rng.uniform(lo, hi)
        v = np.zeros(b.m.nv)
        v[:b.n] = rng.normal(size=b.n)
        yield q, v


def _mujoco(b, q, v):
    """MuJoCo at (q, v): the map T from the core's u to its qvel, its M, bias and data."""
    import numpy as np
    m, d, mj, n = b.m, b.d, b.mj, b.n
    R = model.rotation(q[3:7])
    T = np.eye(n)
    T[:3, :3], T[:3, 3:6], T[3:6, :3], T[3:6, 3:6] = 0.0, R, np.eye(3), 0.0
    d.qpos[:], d.qvel[:] = q, v
    mj.mj_fwdPosition(m, d)
    mj.mj_fwdVelocity(m, d)
    mj.mj_fullM(m, d, b._M)
    M = b._M[:n, :n].copy()
    return {'T': T, 'R': R, 'M': M, 'h': d.qfrc_bias[:n].copy(), 'base': np.r_[R.ravel(), q[:3]],
            'u': np.linalg.solve(T, v[:n])}


def test_the_arrays_are_the_figures(report, c, b, rows):
    """The header and the source hold what the figure compiles to today."""
    for path, text in zip((model.HEADER, model.SOURCE), model.render(rows, b.m.opt.gravity)):
        with open(path, encoding='utf-8', newline='') as f:
            same = f.read() == text
        report.check('%s is the figure\'s' % os.path.relpath(path, model.REPO).replace(os.sep, '/'),
                     same, '' if same else 'python tools/cores/model.py --write')
    report.check('%d links, %d degrees of freedom' % (c.links, c.n),
                 c.links == len(rows) and c.n == b.n and c.names == [r['name'] for r in rows])


def test_the_frames_are_mujocos(report, c, b, rows):
    """Every body's frame in the world, over random configurations."""
    import numpy as np
    rng, worst = np.random.default_rng(3), 0.0
    for q, v in _configurations(b, rows, rng):
        at = _mujoco(b, q, v)
        t = c.pose(at['base'], q[7:7 + c.n - 6])
        for k, row in enumerate(rows):
            if row['frame']:
                worst = max(worst, float(np.abs(t[k, :9] - b.d.xmat[row['body']]).max()),
                            float(np.abs(t[k, 9:] - b.d.xpos[row['body']]).max()))
    report.check('frames within %.0e over %d configurations' % (APART, POSES), worst < APART,
                 'worst %.1e' % worst)


def test_the_mass_matrix_is_mujocos(report, c, b, rows):
    """M, the armatures on its diagonal, in the core's coordinates: T^T M_mujoco T."""
    import numpy as np
    rng, worst = np.random.default_rng(5), 0.0
    for q, v in _configurations(b, rows, rng):
        at = _mujoco(b, q, v)
        c.pose(at['base'], q[7:7 + c.n - 6])
        worst = max(worst, float(np.abs(c.mass() - at['T'].T @ at['M'] @ at['T']).max()))
    report.check('mass matrix within %.0e over %d configurations' % (APART, POSES), worst < APART,
                 'worst %.1e' % worst)


def test_the_bias_is_mujocos(report, c, b, rows):
    """C u + g in the core's coordinates: T^T (h_mujoco + M_mujoco dT u), dT u the pelvis's
    frame turning under its world-frame velocity."""
    import numpy as np
    rng, worst = np.random.default_rng(7), 0.0
    for q, v in _configurations(b, rows, rng):
        at = _mujoco(b, q, v)
        c.pose(at['base'], q[7:7 + c.n - 6])
        u = at['u']
        dT_u = np.zeros(c.n)
        dT_u[:3] = at['R'] @ np.cross(u[:3], u[3:6])
        h = at['T'].T @ (at['h'] + at['M'] @ dT_u)
        worst = max(worst, float(np.abs(c.bias(u) - h).max()))
    report.check('bias within %.0e over %d configurations' % (APART, POSES), worst < APART,
                 'worst %.1e' % worst)


def test_the_jacobians_are_mujocos(report, c, b, rows):
    """Points on the soles, the wrists, the head and the pelvis: where they are and their
    Jacobians over u, the world's rows w then v."""
    import numpy as np
    rng, worst, placed = np.random.default_rng(11), 0.0, 0.0
    jp, jr = np.zeros((3, b.m.nv)), np.zeros((3, b.m.nv))
    links = [c.names.index(name) for name in POINTED]
    for q, v in _configurations(b, rows, rng):
        at = _mujoco(b, q, v)
        c.pose(at['base'], q[7:7 + c.n - 6])
        for link in links:
            body = rows[link]['body']
            for r in np.vstack([b.corners, b.sole, np.zeros(3)]):
                at_world = c.point(link, r)
                placed = max(placed, float(np.abs(
                    at_world - (b.d.xpos[body] + b.d.xmat[body].reshape(3, 3) @ r)).max()))
                b.mj.mj_jac(b.m, b.d, jp, jr, at_world, body)
                j = np.vstack([jr[:, :c.n], jp[:, :c.n]]) @ at['T']
                worst = max(worst, float(np.abs(c.jacobian(link, r) - j).max()))
    report.check('points placed within %.0e' % APART, placed < APART, 'worst %.1e' % placed)
    report.check('Jacobians within %.0e over %d configurations' % (APART, POSES), worst < APART,
                 'worst %.1e' % worst)


def test_a_step_on_this_host(report, c, b, rows):
    """The pose, M, the bias and the soles' Jacobians: microseconds a step here, -O2."""
    import numpy as np
    q, v = next(_configurations(b, rows, np.random.default_rng(13)))
    at = _mujoco(b, q, v)
    feet = [c.names.index(name) for name in POINTED[:2]]
    us = 1e6 * c.seconds(at['base'], q[7:7 + c.n - 6], at['u'], feet[0], feet[1], 2000)
    report.check('a step on this host', us < 1000.0, '%.1f us' % us)


ROSTER = (test_the_arrays_are_the_figures, test_the_frames_are_mujocos,
          test_the_mass_matrix_is_mujocos, test_the_bias_is_mujocos,
          test_the_jacobians_are_mujocos, test_a_step_on_this_host)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    cc = find_cc()
    if cc is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    from machine import wbc
    report = Report()
    lib_path, warnings = build(cc, core.SOURCES, core.INCLUDES, 'wbccore', core.FLAGS)
    report.check('wbc/ builds warning-free with the firmware flags', not warnings,
                 '; '.join(warnings[:3]))
    c, b = core.Core(core.typed(ctypes.CDLL(lib_path))), wbc.Body()
    rows = model.links(b.m)
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, c, b, rows)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
