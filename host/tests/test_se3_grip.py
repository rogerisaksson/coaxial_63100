#!/usr/bin/env python3
"""The pose's step under friction in C (se3/) through ctypes: its twin, its spring, its cone, what a twist leaves.

Each bearing point a bristle from its anchor in the ground. Every number of a step and of the anchors' return the
numpy twin's bits, in double and in float, on every target this processor runs, the anchors taken in the step's
own pass or by a call of their own; inside its reach a point is a spring and its anchor keeps its bits; past the
reach the pull stands on the cone, between rest and sliding by the point's speed, and the anchor follows at the
reach; a point with no load pulls nothing and its anchor stands under it; a sole twisted and brought back is
pulled the other way where it started and, let go, rests off its start on bristles still strained."""
import math
import os
import platform
import re
import shutil
import subprocess
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: In the machine's code, a root taken on a vector's lanes at once: the passes under friction alone take one.
ROOTS = {'x86': r'\sv?sqrtp[sd]\s', 'arm': r'\sfsqrt\s+[vz]\d+'}
#: A sole's bristles: their stiffness, N/m, the friction at rest and sliding, and 1 / v^2 of 1 cm/s between them.
STIFF, REST, SLIDE, FADE = 2e4, 0.8, 0.6, 1e4


def _sole(np, load=150.0):
    """(points, anchors, loads): sixty points of a sole flat on the floor, 0.22 by 0.08 m, each anchor under its
    point, `load` N shared evenly."""
    p = np.array([(x, y, 0.0) for x in np.linspace(-.11, .11, 12) for y in np.linspace(-.04, .04, 5)])
    return p, p.copy(), np.full(len(p), load / len(p))


def _contact(np, rng, n, off):
    """(points, anchors, loads, pose): `n` points of a sole anywhere in its body, loads of 5 to 20 N, and the pose
    (r, t) that lays each point within `off` m of its anchor."""
    p = rng.uniform(-1.0, 1.0, (n, 3)) * (0.12, 0.05, 0.01) + rng.normal(0.0, 0.2, 3)
    r, _ = np.linalg.qr(rng.normal(0.0, 1.0, (3, 3)))
    r, t = r * np.linalg.det(r), rng.normal(0.0, 0.5, 3)
    return p, p @ r.T + t + rng.uniform(-off, off, (n, 3)) / math.sqrt(3.0), rng.uniform(5.0, 20.0, n), (r, t)


def _yawed(np, angle):
    """The turn by `angle`, rad, about the floor's normal."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _planes(se3, planes, first, n):
    """Three of the arena's planes from `first` on as (n, 3), copied: 0 the points, 3 the anchors, 7 the pulls."""
    return planes.reshape(se3.PLANES, -1)[first:first + 3, :n].T.copy()


def test_the_c_is_its_twin(report):
    """A pose's steps under friction and its anchors' returns on a sole held inside its reach, one sliding, one
    under way with drag, one with points in the air and loads under zero, one point, none bearing and bristles of
    no stiffness: each of a step's 48 numbers and each anchor the numpy twin's bits, six ticks on, in double and in
    float; a step that takes no anchors leaves theirs, one that takes them in its own pass leaves the step's and
    the return's bits both; and the same bits from the core built for each target this processor runs."""
    import numpy as np
    from tools.cores import se3
    from tools.cores.build import TARGETS, held
    targets = [isa for isa in TARGETS if held(isa)]
    for single in (False, True):
        rng, word = np.random.default_rng(11), (np.uint32 if single else np.uint64)
        apart, across, passes, ticks = [], [], [], 0
        for name, n, stiff, drag in (('held', 37, STIFF, 0.0), ('sliding', 37, STIFF, 0.0), ('under way', 24, STIFF, 30.0),
                                     ('air', 21, STIFF, 0.0), ('point', 1, STIFF, 0.0), ('none', 16, STIFF, 0.0),
                                     ('slack', 9, 0.0, 0.0)):
            p, g, w, (r, t) = _contact(np, rng, n, 1e-4 if name == 'held' else 5e-3)
            if name == 'air':
                w[1::3], w[2::3] = 0.0, -w[2::3]
            if name == 'none':
                w = np.where(np.arange(n) % 2, 0.0, -w)
            spun, sped = ((rng.normal(0.0, 0.5, (3, 3)), rng.normal(0.0, 0.05, 3)) if name == 'under way'
                          else (None, None))
            rub = se3.friction(stiff, REST, SLIDE, FADE, drag, spun, sped, single)
            planes, config = se3.laid(p, g, w, single), se3.cfg(single)
            for _ in range(6):
                given = planes.reshape(se3.PLANES, -1)[:se3.GIVEN].copy()
                out, said = se3.grip(planes, config, rub, r, t), se3.grip_twin(planes, config, rub, r, t)
                want = se3.anchor_twin(planes, rub, r, t)
                if (planes.reshape(se3.PLANES, -1)[:se3.GIVEN].view(word) != given.view(word)).any():
                    passes.append('%s: a step that takes none moved them' % name)
                for isa in [''] + targets:
                    other = se3.laid(_planes(se3, planes, 0, n), _planes(se3, planes, 3, n),
                                     planes.reshape(se3.PLANES, -1)[6, :n], single)
                    both = se3.grip(other, config, rub, r, t, isa, take=True)
                    if ((both.view(word) != out.view(word)).any()
                            or (other.reshape(se3.PLANES, -1)[3:6].view(word) != want.view(word)).any()):
                        passes.append('%s: one pass %s' % (name, isa or 'here'))
                for isa in targets:
                    other = se3.laid(_planes(se3, planes, 0, n), _planes(se3, planes, 3, n),
                                     planes.reshape(se3.PLANES, -1)[6, :n], single)
                    step = se3.grip(other, config, rub, r, t, isa)
                    se3.anchor(other, rub, r, t, isa)
                    if ((step.view(word) != out.view(word)).any()
                            or (other.reshape(se3.PLANES, -1)[3:6].view(word) != want.view(word)).any()):
                        across.append(isa)
                se3.anchor(planes, rub, r, t)
                if ((out.view(word) != said.view(word)).any()
                        or (planes.reshape(se3.PLANES, -1)[3:6].view(word) != want.view(word)).any()):
                    apart.append(name)
                r, t, ticks = se3.read(out)['r'].copy(), se3.read(out)['t'].copy(), ticks + 1
        name = 'float' if single else 'double'
        report.check('%s: every number of a step and every anchor is its twin\'s bits' % name, not apart,
                     ', '.join(sorted(set(apart))) or '%d ticks of 7 arenas, %d numbers a step' % (ticks, se3.SIZE))
        report.check('%s: a step takes the anchors in its own pass to the return\'s bits, or leaves them theirs' % name,
                     not passes, '; '.join(passes[:3]) or 'here and on %s' % (' '.join(targets) or 'no other target'))
        report.check('%s: the same bits on every target this processor runs' % name, not across,
                     ', '.join(sorted(set(across))) or ' '.join(targets) or 'no other target here')


def test_inside_its_reach_a_point_is_a_spring(report):
    """A loaded sole twisted 0.2 mrad, every point inside its reach: the torque is the bristles' own, their
    stiffness times the sum of the levers' squares times the angle's sine, and the step is the fit's with each
    weight the stiffness; over 2000 returns under a pose trembling inside the reach no anchor moves a bit, in
    double or in float."""
    import numpy as np
    from tools.cores import se3
    p, g, w = _sole(np)
    angle, zero, config = 2e-4, np.zeros(3), se3.cfg()
    out = se3.read(se3.grip(se3.laid(p, g, w), config, se3.friction(STIFF, REST, SLIDE, FADE), _yawed(np, angle), zero))
    fit = se3.read(se3.step(se3.laid(p, g, np.full(len(p), STIFF)), config, _yawed(np, angle), zero))
    spring = STIFF * float((p ** 2).sum()) * math.sin(angle)
    report.check('the torque is the springs\': their stiffness times the levers\' squares times the angle\'s sine',
                 abs(out['turn'][2] - spring) < 1e-9 * spring and np.abs(out['turned'] - fit['turned']).max() < 1e-12,
                 '%.6f N m of %.6f, the step %.3e rad as the fit\'s' % (out['turn'][2], spring, out['turned'][2]))
    for single in (False, True):
        rng, planes = np.random.default_rng(5), se3.laid(p, g, w, single)
        rub, start = se3.friction(STIFF, REST, SLIDE, FADE, single=single), _planes(se3, planes, 3, len(p))
        for _ in range(2000):
            se3.anchor(planes, rub, _yawed(np, rng.uniform(-1.5e-4, 1.5e-4)), np.r_[rng.uniform(-2e-5, 2e-5, 2), 0.0])
        moved = int((_planes(se3, planes, 3, len(p)) != start).sum())
        report.check('%s: no anchor moves a bit over 2000 returns inside the reach' % ('float' if single else 'double'),
                     moved == 0, '%d of %d numbers moved' % (moved, start.size))


def test_past_its_reach_the_pull_stands_on_the_cone(report):
    """A loaded sole shoved 5 mm along the floor. At rest the pull is the friction at rest times the load, along
    the shove, and the anchors' return leaves each anchor its reach behind its point; under way at 5 cm/s the pull
    is the friction's at that speed, between sliding and rest, and a drag adds its own on the speed, a bearing
    point each."""
    import numpy as np
    from tools.cores import se3
    p, g, w = _sole(np)
    way, eye, load = np.array([math.cos(0.7), math.sin(0.7), 0.0]), np.eye(3), float(w.sum())
    planes, config, rub = se3.laid(p, g, w), se3.cfg(), se3.friction(STIFF, REST, SLIDE, FADE)
    pull = se3.read(se3.grip(planes, config, rub, eye, 5e-3 * way))['shift'].copy()
    se3.anchor(planes, rub, eye, 5e-3 * way)
    behind, reach = np.linalg.norm(p + 5e-3 * way - _planes(se3, planes, 3, len(p)), axis=1), REST * w / STIFF
    report.check('at rest the pull is the friction at rest times the load, along the shove; each anchor its reach behind',
                 np.abs(pull - REST * load * way).max() < 1e-9 * load and np.abs(behind - reach).max() < 1e-9 * reach.max(),
                 '%.4f N of %.4f, the anchors %.4e m behind of %.4e' % (np.linalg.norm(pull), REST * load, behind.mean(),
                                                                         reach.mean()))
    speed, said = 0.05, []
    mu = SLIDE + (REST - SLIDE) / (1.0 + speed ** 2 * FADE)
    for drag in (0.0, 4.0):
        rub = se3.friction(STIFF, REST, SLIDE, FADE, drag=drag, sped=speed * way)
        pull = se3.read(se3.grip(se3.laid(p, g, w), config, rub, eye, 5e-3 * way))['shift']
        said += [float(pull @ way), mu * load + drag * speed * len(p)]
    report.check('under way the pull is the friction\'s at that speed, and a drag adds its own on the speed',
                 abs(said[0] - said[1]) < 1e-9 * load and abs(said[2] - said[3]) < 1e-9 * load
                 and SLIDE * load < said[0] < REST * load,
                 '%.4f N of %.4f at 5 cm/s, %.4f of %.4f with the drag' % tuple(said))


def test_with_no_load_a_point_pulls_nothing(report):
    """Of a shoved sole every third point bears, the others with no load or one under zero: the sums are the
    bearing points' own. A sole in the air 0.3 m off its anchors weighs nothing and is not stepped, every number
    finite, in double and in float; and the anchors' return stands each anchor under its point, to the bit."""
    import numpy as np
    from tools.cores import se3
    p, g, w = _sole(np)
    some, n = w.copy(), len(p)
    some[1::3], some[2::3] = 0.0, -some[2::3]
    rub, pose = se3.friction(STIFF, REST, SLIDE, FADE), (_yawed(np, 0.01), np.array([2e-3, -1e-3, 0.0]))
    all_, own = (se3.read(se3.grip(se3.laid(*arena), se3.cfg(), rub, *pose))
                 for arena in ((p, g, some), (p[::3], g[::3], w[::3])))
    off = max(float((np.abs(np.asarray(all_[k]) - np.asarray(own[k])) / (1.0 + np.abs(np.asarray(own[k])))).max())
              for k in ('weight', 'centre', 'turn', 'shift', 'cost'))
    report.check('the sums are the bearing points\' own', off < 1e-10 and own['weight'] > 0.0,
                 '%.1e apart at most, the pull %.3f N' % (off, np.linalg.norm(own['shift'])))
    for single in (False, True):
        dt, planes = se3.kind(single), se3.laid(p, g, np.zeros(n), single)
        rub, r, t = se3.friction(STIFF, REST, SLIDE, FADE, single=single), _yawed(np, 0.3), np.array([0.1, -0.2, 0.3])
        raw = se3.grip(planes, se3.cfg(single), rub, r, t)
        out = se3.read(raw)
        se3.anchor(planes, rub, r, t)
        px, py, pz = planes.reshape(se3.PLANES, -1)[:3, :n]
        rr, tt = r.astype(dt).ravel(), t.astype(dt)
        under = np.array([(((rr[3 * i] * px) + (rr[3 * i + 1] * py)) + (rr[3 * i + 2] * pz)) + tt[i] for i in range(3)])
        still = (out['weight'] == 0.0 and not any(out[k].any() for k in ('turn', 'shift', 'turned', 'shifted'))
                 and np.array_equal(out['t'], tt) and np.abs(out['r'] - r).max() < 8.0 * np.finfo(dt).eps)
        report.check('%s: in the air a sole weighs nothing, is not stepped, and its anchors stand under its points'
                     % ('float' if single else 'double'),
                     bool(np.isfinite(raw).all() and still
                          and np.array_equal(planes.reshape(se3.PLANES, -1)[3:6, :n], under)),
                     'weight %.1f, %d numbers finite' % (out['weight'], int(np.isfinite(raw).sum())))


def test_a_twist_is_kept(report):
    """A loaded sole twisted to 0.08 rad and back, half a milliradian a tick, its anchors returned every tick. The
    first tick is the springs' torque; at 1.5 mrad the outer points slide on their cones while the inner hold; at
    the end every point slides, the torque the friction's times the loads on their levers; back where the sole
    started the same torque pulls the other way - the anchors hold what the pose alone does not tell. Let go there,
    the sole comes to rest off its start with no torque or force left on it, its bristles still strained each
    against the others."""
    import numpy as np
    from tools.cores import se3
    p, g, w = _sole(np)
    planes, n, zero = se3.laid(p, g, w), len(p), np.zeros(3)
    rub, config, levers = se3.friction(STIFF, REST, SLIDE, FADE), se3.cfg(), np.hypot(p[:, 0], p[:, 1])
    full, spring = REST * float((w * levers).sum()), STIFF * float((levers ** 2).sum())
    up = [0.0005 * k for k in range(1, 161)]
    torque, slides = [], []
    for angle in up + up[-2::-1] + [0.0]:
        torque.append(float(se3.read(se3.grip(planes, config, rub, _yawed(np, angle), zero))['turn'][2]))
        slides.append(np.linalg.norm(_planes(se3, planes, 7, n), axis=1) > (1.0 - 1e-9) * REST * w)
        se3.anchor(planes, rub, _yawed(np, angle), zero)
    report.check('the first tick is the springs\' torque, no point sliding',
                 abs(torque[0] - spring * math.sin(up[0])) < 1e-9 * full and not slides[0].any(),
                 '%.4f N m of %.4f' % (torque[0], spring * math.sin(up[0])))
    part = slides[2]
    report.check('at 1.5 mrad the outer points slide on their cones while the inner hold',
                 0 < part.sum() < n and levers[part].min() > levers[~part].max()
                 and torque[2] < min(full, spring * math.sin(up[2])),
                 '%d of %d sliding, from a lever of %.4f m out; %.4f N m, the springs\' %.4f and the cones\' %.4f'
                 % (part.sum(), n, levers[part].min(), torque[2], spring * math.sin(up[2]), full))
    report.check('at 0.08 rad every point slides: the torque is the friction\'s times the loads on their levers',
                 bool(slides[159].all()) and abs(torque[159] - full) < 1e-3 * full,
                 '%.4f N m of %.4f' % (torque[159], full))
    report.check('back where the sole started the same torque pulls the other way',
                 abs(torque[-1] + full) < 1e-3 * full, '%+.4f N m at 0 rad, %+.4f at 0.08' % (torque[-1], torque[159]))
    r, t = np.eye(3), zero
    for _ in range(60):
        out = se3.read(se3.grip(planes, config, rub, r, t))
        se3.anchor(planes, rub, r, t)
        r, t = out['r'].copy(), out['t'].copy()
    out, rest = se3.read(se3.grip(planes, config, rub, r, t)), math.atan2(r[1, 0], r[0, 0])
    pulls = np.linalg.norm(_planes(se3, planes, 7, n), axis=1)
    report.check('let go, the sole rests off its start, no torque or force left, its bristles still strained',
                 rest > 5e-4 and np.abs(out['turn']).max() < 1e-9 and np.abs(out['shift']).max() < 1e-9
                 and pulls.mean() > 0.5,
                 '%+.3e rad; %.1e N m and %.1e N left; a point bears %.2f N, the most %.2f'
                 % (rest, np.abs(out['turn']).max(), np.abs(out['shift']).max(), pulls.mean(), pulls.max()))


def test_its_form_and_its_pace(report):
    """se3_friction_t is seventeen numbers of its precision with no padding; a friction of another length and an
    arena off its shape are refused before the C; as this host's compiler makes them, both passes under friction
    take their root on a vector's lanes at once; and what a bearing sole costs a tick beside the fit - its step, the
    step taking its anchors with it, and the anchors' return in a call of its own."""
    import numpy as np
    from tools.cores import se3
    from tools.cores.build import FLAGS, OUT, find_cc
    p, g, w = _sole(np)
    cc, pose, said = find_cc(), (np.eye(3), np.array([3e-5, 0.0, 0.0])), []
    machine = 'arm' if platform.machine().lower() in ('arm64', 'aarch64') else 'x86'
    objdump = shutil.which('objdump') or os.path.join(os.path.dirname(cc or ''), 'objdump')
    source = [path for path in se3.SOURCES if path.endswith('se3_sums.c')][0]
    for single in (False, True):
        name, item, lib = ('float' if single else 'double'), np.dtype(se3.kind(single)).itemsize, se3.library(single)
        planes, config = se3.laid(p, g, w, single), se3.cfg(single)
        rub, refused = se3.friction(STIFF, REST, SLIDE, FADE, single=single), 0
        for arena, wrong in ((planes, rub[:-1]), (planes[1:], rub), (planes.astype(np.float16), rub)):
            for call in (lambda: se3.grip(arena, config, wrong, *pose), lambda: se3.anchor(arena, wrong, *pose)):
                try:
                    call()
                except ValueError:
                    refused += 1
        report.check('%s: a friction is seventeen numbers; another length or an arena off its shape is refused' % name,
                     lib.se3_size(4) == se3.RUBS * item == 17 * item and refused == 6,
                     'a struct of %d bytes, %d of 6 refused' % (lib.se3_size(4), refused))
        obj = os.path.join(OUT, 'se3_grip_%d.o' % os.getpid())
        flags = [flag for flag in se3.built(single) if not flag.startswith('-Wl')]
        subprocess.run([cc, '-c'] + FLAGS + flags + ['-I', se3.INCLUDES[0], source, '-o', obj], check=True)
        roots = len(re.findall(ROOTS[machine], subprocess.run([objdump, '-d', obj], capture_output=True, text=True,
                                                              check=True).stdout))
        os.remove(obj)
        report.check('%s: both passes under friction take their root on a vector\'s lanes' % name, roots >= 2,
                     '%d roots on lanes' % roots)
        cost = (se3.paced(se3.laid(p, g, w, single), config, *pose, 20000),
                se3.paced(se3.laid(p, g, w, single), config, *pose, 20000, rub=rub),
                se3.paced(se3.laid(p, g, w, single), config, *pose, 20000, rub=rub, take=True),
                se3.paced(se3.laid(p, g, w, single), config, *pose, 20000, rub=rub, anchors=True))
        said.append('%s 60 points: the fit %.0f ns, a step under friction %.0f, taking its anchors %.0f, their return '
                    'alone %.0f' % ((name,) + tuple(1e9 * took for took in cost)))
    report.check('a tick\'s cost under friction in C, each precision', True, '; '.join(said))


ROSTER = (test_the_c_is_its_twin, test_inside_its_reach_a_point_is_a_spring,
          test_past_its_reach_the_pull_stands_on_the_cone, test_with_no_load_a_point_pulls_nothing,
          test_a_twist_is_kept, test_its_form_and_its_pace)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
