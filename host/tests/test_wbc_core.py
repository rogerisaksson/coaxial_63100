"""Her chain and the loop on the static model (wbc/) against MuJoCo and the python stack.

The arrays as the figure compiles; the core's poses, Jacobians, mass matrix and bias at random
configurations; the loop's torques standing against `machine.wbc.step`'s; her stand and a shove
in MuJoCo on the loop; a step's and a tick's time on this host."""
import ctypes
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.cores import model  # noqa: E402
from tools.cores import wbc as core  # noqa: E402
from tools.cores.build import build, find_cc  # noqa: E402
from tools.dev.focus import chosen  # noqa: E402
from gynoid_kit import Report  # noqa: E402
from machine import physics  # noqa: E402

#: The loop's world has the gearboxes with their play (`machine.gearbox`).
physics.BOXED = 1.0

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
    u = np.linalg.solve(T, v[:n])
    dT_u = np.zeros(n)
    dT_u[:3] = R @ np.cross(u[:3], u[3:6])
    return {'T': T, 'R': R, 'M': M, 'h': d.qfrc_bias[:n] - d.qfrc_passive[:n],
            'base': np.r_[R.ravel(), q[:3]], 'u': u, 'dT_u': dT_u}


def test_the_arrays_are_the_figures(report, c, b, rows):
    """The header and the source hold what the figure compiles to today."""
    for path, text in zip((model.HEADER, model.SOURCE), model.render(rows, b)):
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
    """C u + g less the springs' and dampers' torques, in the core's coordinates: T^T (h_mujoco
    + M_mujoco dT u), dT u the pelvis's frame turning under its world-frame velocity."""
    import numpy as np
    rng, worst = np.random.default_rng(7), 0.0
    for q, v in _configurations(b, rows, rng):
        at = _mujoco(b, q, v)
        c.pose(at['base'], q[7:7 + c.n - 6])
        h = at['T'].T @ (at['h'] + at['M'] @ at['dT_u'])
        worst = max(worst, float(np.abs(c.bias(at['u']) - h).max()))
    report.check('bias within %.0e over %d configurations' % (APART, POSES), worst < APART,
                 'worst %.1e' % worst)


def test_the_centre_of_mass_is_mujocos(report, c, b, rows):
    """Her centre of mass and its Jacobian, the angular momentum about it, a point's drift
    J-dot u (the world's, J_mujoco-dot v + J_mujoco dT u) on the soles and the trunk."""
    import numpy as np
    rng, apart = np.random.default_rng(17), {'centre': 0.0, 'Jacobian': 0.0, 'momentum': 0.0,
                                             'drift': 0.0}
    jp, jr, am = np.zeros((3, b.m.nv)), np.zeros((3, b.m.nv)), np.zeros((3, b.m.nv))
    links = [c.names.index(name) for name in POINTED[:2]] + [c.names.index('waist')]
    for q, v in _configurations(b, rows, rng):
        at = _mujoco(b, q, v)
        c.pose(at['base'], q[7:7 + c.n - 6])
        c.bias(at['u'])
        com, j = c.com()
        b.mj.mj_jacSubtreeCom(b.m, b.d, jp, b.pelvis)
        apart['centre'] = max(apart['centre'], float(np.abs(com - b.d.subtree_com[b.pelvis]).max()))
        apart['Jacobian'] = max(apart['Jacobian'], float(np.abs(j - jp[:, :c.n] @ at['T']).max()))
        b.mj.mj_angmomMat(b.m, b.d, am, b.pelvis)
        apart['momentum'] = max(apart['momentum'],
                                float(np.abs(c.momentum(com) - am[:, :c.n] @ v[:c.n]).max()))
        for link in links:
            body = rows[link]['body']
            for r in (b.sole, np.zeros(3)):
                at_world = b.d.xpos[body] + b.d.xmat[body].reshape(3, 3) @ r
                b.mj.mj_jac(b.m, b.d, jp, jr, at_world, body)
                drift = np.vstack([jr[:, :c.n], jp[:, :c.n]]) @ at['dT_u']
                b.mj.mj_jacDot(b.m, b.d, jp, jr, at_world, body)
                drift += np.vstack([jr[:, :c.n], jp[:, :c.n]]) @ v[:c.n]
                apart['drift'] = max(apart['drift'], float(np.abs(c.drift(link, r) - drift).max()))
    for name, worst in apart.items():
        report.check('%s within %.0e over %d configurations' % (name, APART, POSES),
                     worst < APART, 'worst %.1e' % worst)


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


def _standing(b):
    """Her stand's state in `b` and the ask that keeps it: both soles, no acceleration, the
    turns and the posture as they are."""
    import numpy as np
    from machine import gait, physics, wbc
    world = physics.World()
    world.reset(gait.stand())
    s = wbc.sense(b, world.data.qpos, world.data.qvel)
    low = min(float(f['pts'][:, 1].min()) for f in s['feet'])
    world.reset(gait.stand(), where=(0.0, 1.0 - low, 0.0))
    s = wbc.sense(b, world.data.qpos, world.data.qvel)
    ask = {'stance': (True, True), 'com_acc': np.zeros(3),
           'turns': (s['turns'][0]['quat'], s['turns'][1]['quat']), 'posture': s['q'].copy(),
           'derate': 1.0, 'wep': False}
    return s, ask


def test_the_loop_stands_as_the_python_stack(report, c, b, rows):
    """Standing still, asked to stay: the loop's torques against `machine.wbc.step`'s, whose
    inequalities all sleep there; its clip untouched; a tick's time."""
    import numpy as np
    from machine import wbc
    s, ask = _standing(b)
    theirs = wbc.step(b, s, ask)
    ours = core.stack_step(b, s, ask, c)
    apart = float(np.abs(ours['tau'] - theirs['tau']).max())
    scale = float(np.abs(theirs['tau']).max())
    report.check('torques within 2 %% of the largest (%.1f N m) of the python stack\'s' % scale,
                 apart < 0.02 * scale, 'apart %.2f N m' % apart)
    report.check('each sole bears as the python stack plans', all(
        abs(ours['bears'][k] - theirs['bears'][k]) < 0.02 * sum(theirs['bears']) for k in (0, 1)),
                 'C %.0f %.0f N, python %.0f %.0f N' % (*ours['bears'], *theirs['bears']))
    report.check('no bound at its edge standing', ours['held'] == 0,
                 '%d at the edge, the active set changed %d times' % (ours['held'], ours['passes']))
    base, q, u, _T = core.state_of(b)
    us = 1e6 * c.stack_seconds(base, q, u, ask, 100)
    report.check('a tick of the loop on this host', us < 4000.0, '%.0f us' % us)


def test_she_stands_on_the_loop(report, c, b, rows):
    """In MuJoCo on the loop's torques: standing 3 s, shoved 38 N from behind, and the walk
    asked 0.5 m/s for 8 s."""
    from tools.sim import wbc as sim
    out = sim.stand(seconds=3.0, core=True)
    report.check('stands 3 s on the loop', out['stood'],
                 'tilt %.1f deg, drift %.0f mm, %d us a pass' % (out['tilt'], out['drift_mm'],
                                                                 out['us']))
    out = sim.stand(38.0, 90.0, core=True)
    report.check('shoved 38 N from behind she stands', out['stood'],
                 'tilt %.1f deg, %d steps' % (out['tilt'], out['steps']))
    out = sim.stand(seconds=sim.AT_S + 8.0, walk=(0.5, 0.5), core=True)
    report.check('walks 8 s on the loop', out['stood'] and out['steps'] >= 12,
                 '%s m at %s m/s, %s J/m, %d steps, tilt %.1f deg' % (
                     out.get('walked_m'), out.get('m_s'), out.get('j_m'), out['steps'], out['tilt']))


ROSTER = (test_the_arrays_are_the_figures, test_the_frames_are_mujocos,
          test_the_mass_matrix_is_mujocos, test_the_bias_is_mujocos,
          test_the_jacobians_are_mujocos, test_the_centre_of_mass_is_mujocos,
          test_a_step_on_this_host, test_the_loop_stands_as_the_python_stack,
          test_she_stands_on_the_loop)


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
    rows = model.links(b)
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, c, b, rows)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
