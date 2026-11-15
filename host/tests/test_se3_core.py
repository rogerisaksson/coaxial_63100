#!/usr/bin/env python3
"""The pose's step in C (se3/) through ctypes: its form, its twin, its best pose, what it leaves alone.

No jump its data decides in its source or its machine code, its passes packed; every output the numpy twin's
bits, in double and in float, on every target this processor runs; from a pose far off the closed form's best
in a few steps; no turn the points cannot tell - about a line of them, of one point, of none -; a float's grain
kept far from the origin; on SO(3) and finite over ten thousand steps and wild arenas; a wild goal let go."""
import glob
import io
import math
import os
import platform
import re
import shutil
import subprocess
import sys
import time

from tools.dev.focus import chosen
from gynoid_kit import Report
from test_aero_core import BANNED, FLAGGED, JUMPS

#: In the machine's code: arithmetic on a vector's lanes at once, on a single number, and a vector's division.
PACKED = {'x86': (r'\sv?(?:mul|add|sub|div)p[sd]\s', r'\sv?(?:mul|add|sub|div)s[sd]\s', r'\sv?divp[sd]\s'),
          'arm': (r'\s(?:fmul|fadd|fsub|fdivr?)\s+[vz]\d+', r'\s(?:fmul|fadd|fsub|fdivr?)\s+[ds]\d+',
                  r'\sfdivr?\s+[vz]\d+')}


def _turned(np, a):
    """The turn by the vector `a`, rad: Rodrigues', in double."""
    angle = float(np.linalg.norm(a))
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    return np.eye(3) + math.sin(angle) / angle * k + (1.0 - math.cos(angle)) / angle ** 2 * (k @ k)


def _best(np, p, g, w):
    """(r, t, cost) the weighted squares' least over every pose, closed (Kabsch's, by the SVD), in double."""
    pc, gc = (w[:, None] * p).sum(0) / w.sum(), (w[:, None] * g).sum(0) / w.sum()
    u, _s, vt = np.linalg.svd((w[:, None] * (p - pc)).T @ (g - gc))
    r = vt.T @ np.diag([1.0, 1.0, np.sign(np.linalg.det(vt.T @ u.T))]) @ u.T
    t = gc - r @ pc
    return r, t, 0.5 * float((w * ((p @ r.T + t - g) ** 2).sum(1)).sum())


def _sole(np, rng, n, line=False):
    """`n` points of a sole in its foot's frame, off its origin and turned any way: 0.24 by 0.1 by 0.02 m, or a
    line of them; and the line's way."""
    way = _turned(np, rng.normal(0.0, 1.0, 3))
    wide = 0.0 if line else 1.0
    p = np.c_[rng.uniform(-.12, .12, n), wide * rng.uniform(-.05, .05, n), wide * rng.uniform(-.01, .01, n)]
    return p @ way.T + rng.normal(0.0, 0.2, 3), way[:, 0]


def _case(np, rng, n=37, line=False, noise=0.002):
    """(points, goals, weights, the pose asked, a pose to start from, the line's way in the body)."""
    p, way = _sole(np, rng, n, line)
    r, t = _turned(np, rng.normal(0.0, 0.6, 3)), rng.normal(0.0, 0.5, 3)
    g = p @ r.T + t + rng.normal(0.0, noise, (n, 3))
    r0 = _turned(np, rng.normal(0.0, 0.2, 3)) @ r
    return p, g, rng.uniform(5.0, 20.0, n), (r, t), (r0, t + rng.normal(0.0, 0.1, 3)), way


def _run(se3, planes, config, pose, steps):
    """The fields of each of `steps` steps of the C from `pose`, each from the last one's pose."""
    r, t = pose
    said = []
    for _ in range(steps):
        said.append(se3.read(se3.step(planes, config, r, t)))
        r, t = said[-1]['r'].copy(), said[-1]['t'].copy()
    return said


def test_a_step_has_no_branch(report):
    """se3/ holds no `if`, `else`, `switch`, `while`, `?`, `&&`, `||`, `break` or `goto`, its comments aside, and
    builds under the firmware's warnings with none in either precision; as this host's compiler makes it no flag is
    set from its data, and its passes are packed - their arithmetic a vector's lanes at once, the weights'
    division among it; what is left of single numbers is the sums' ends."""
    from tools.cores import se3
    from tools.cores.build import FLAGS, OUT, build, find_cc
    root = os.path.dirname(se3.INCLUDES[0])
    sources = sorted(glob.glob(os.path.join(root, 'src', '*.[ch]')) + glob.glob(os.path.join(root, 'inc', '*.h')))
    found = []
    for path in sources:
        with io.open(path, encoding='utf-8') as f:
            code = re.sub(r'"[^"\n]*"', '""', re.sub(r'/\*.*?\*/', ' ', f.read(), flags=re.S))
        found += ['%s: %s' % (os.path.basename(path), hit.group(0)) for hit in re.finditer(BANNED, code)]
    report.check('no jump the data decides in %d sources' % len(sources), len(sources) == 4 and not found,
                 ', '.join(found[:6]) or ' '.join(os.path.basename(p) for p in sources))
    cc = find_cc()
    machine = 'arm' if platform.machine().lower() in ('arm64', 'aarch64') else 'x86'
    objdump = shutil.which('objdump') or os.path.join(os.path.dirname(cc or ''), 'objdump')
    for single in (False, True):
        name = 'float' if single else 'double'
        flags = [flag for flag in se3.built(single) if not flag.startswith('-Wl')]
        _lib, warnings = build(cc, se3.SOURCES, se3.INCLUDES, 'se3_warned', list(se3.built(single)))
        report.check('%s: built under the firmware\'s warnings with none' % name, not warnings,
                     '; '.join(warnings[:2]) or '0 warnings')
        flagged, jumps, size, packed, alone, divided = [], 0, 0, 0, 0, 0
        for path in se3.SOURCES:
            obj = os.path.join(OUT, 'se3_%d_%s.o' % (os.getpid(), os.path.basename(path)[:-2]))
            subprocess.run([cc, '-c'] + FLAGS + flags + ['-I', se3.INCLUDES[0], path, '-o', obj], check=True)
            text = subprocess.run([objdump, '-d', obj], capture_output=True, text=True, check=True).stdout
            os.remove(obj)
            flagged += ['%s: %s' % (os.path.basename(path), hit) for hit in re.findall(FLAGGED[machine], text)]
            jumps += len(re.findall(JUMPS[machine], text))
            size += len(re.findall(r'^\s*[0-9a-f]+:\t', text, flags=re.M))
            if path.endswith('se3_sums.c'):
                packed, alone, divided = (len(re.findall(kind, text)) for kind in PACKED[machine])
        report.check('%s: in this host\'s code no flag is set from its data' % name, not flagged and size > 1000,
                     ', '.join(sorted(set(flagged))[:6])
                     or '%d instructions, %d conditional jumps: its loops\' ends' % (size, jumps))
        report.check('%s: its passes are packed, their division too' % name, packed > 2 * alone and divided >= 1,
                     '%d operations on a vector\'s lanes, %d on a single number' % (packed, alone))


def test_the_c_is_its_twin(report):
    """A pose's steps on a sole's points, a line of them, one point, none bearing, with damping, soft weights, a
    wild goal and weights under zero: each of a step's 48 numbers the numpy twin's bits, six steps on, in double
    and in float; and the same bits from the core built for each target this processor runs."""
    import numpy as np
    from tools.cores import se3
    from tools.cores.build import TARGETS, held
    targets = [isa for isa in TARGETS if held(isa)]
    for single in (False, True):
        rng, word = np.random.default_rng(7), (np.uint32 if single else np.uint64)
        apart, across, steps = [], [], 0
        for name, n, line, told in (('sole', 37, False, {'soft': 25.0}), ('line', 12, True, {}), ('point', 1, False, {}),
                                    ('none', 21, False, {}), ('damped', 16, False, {'damp_turn': 0.3, 'damp_shift': 2.0}),
                                    ('wild', 48, False, {'soft': 400.0}), ('capped', 5, False, {'most_turn': 0.05,
                                                                                               'most_shift': 0.01})):
            p, g, w, _asked, pose, _way = _case(np, rng, n, line)
            if name == 'none':
                w = np.where(np.arange(n) % 2, 0.0, -w)
            if name == 'wild':
                g[::7] += rng.normal(0.0, 0.6, (len(g[::7]), 3))
            planes, config = se3.laid(p, g, w, single), se3.cfg(single, **told)
            r, t = pose
            for _ in range(6):
                out, said = se3.step(planes, config, r, t), se3.twin(planes, config, r, t)
                if (out.view(word) != said.view(word)).any():
                    apart.append('%s: %s' % (name, [k for k, v in se3.read(out).items()
                                                     if np.any(np.asarray(v) != np.asarray(se3.read(said)[k]))]))
                across += [isa for isa in targets if (se3.step(planes, config, r, t, isa).view(word) != out.view(word)).any()]
                r, t, steps = se3.read(out)['r'].copy(), se3.read(out)['t'].copy(), steps + 1
        name = 'float' if single else 'double'
        report.check('%s: every number of a step is its twin\'s bits' % name, not apart,
                     '; '.join(apart[:2]) or '%d steps of 7 arenas, %d numbers each' % (steps, se3.SIZE))
        report.check('%s: the same bits on every target this processor runs' % name, not across,
                     ', '.join(sorted(set(across))) or ' '.join(targets) or 'no other target here')


def test_it_comes_to_the_best_pose(report):
    """From 0.2 rad and 0.1 m off, on goals 2 mm noisy, 4 to 48 points: its pose after forty steps the closed
    form's - the weighted squares' least over every pose, on the arena's own numbers - to the lanes' rounding; its
    cost never risen past the misses' own rounding (a miss of millimetres is the difference of places a metre
    off); and after one step alone the centroid on its goals' (the shift is the centroid's, no turn in it)."""
    import numpy as np
    from tools.cores import se3
    for single, near in ((False, 1e-12), (True, 1e-5)):
        rng, worst, rose, off, took = np.random.default_rng(11), [0.0, 0.0, 0.0], 0, 0.0, 0
        eps = float(np.finfo(se3.kind(single)).eps)
        for _ in range(60):
            p, g, w, _asked, pose, _way = _case(np, rng, int(rng.integers(4, 49)))
            planes = se3.laid(p, g, w, single)
            p64, g64, w64 = (planes.astype(float).reshape(se3.PLANES, -1)[rows, :len(w)].T for rows in ((0, 1, 2), (3, 4, 5), 6))
            r, t, cost = _best(np, p64, g64, w64.ravel())
            said = _run(se3, planes, se3.cfg(single, most_turn=1.0, most_shift=1.0), pose, 40)
            worst = [max(worst[0], float(np.abs(said[-1]['r'] - r).max())), max(worst[1], float(np.abs(said[-1]['t'] - t).max())),
                     max(worst[2], abs(float(said[-1]['cost']) - cost) / cost)]
            costs = [float(s['cost']) for s in said]
            grain = 4.0 * eps * float(np.abs(planes.reshape(se3.PLANES, -1)[:6]).max()) / math.sqrt(2.0 * cost / float(w64.sum()))
            rose += sum(b > a * (1.0 + grain) for a, b in zip(costs, costs[1:]))
            off = max(off, float(np.abs(said[1]['shift']).max() / said[1]['weight']))
            took = max(took, next((k for k, s in enumerate(said) if np.linalg.norm(s['turned']) < near * 100.0), 40))
        name = 'float' if single else 'double'
        report.check('%s: the closed form\'s best pose over 60 arenas, to %.0e' % (name, near),
                     worst[0] < near and worst[1] < near and worst[2] < max(near, 1e-9) * 100.0,
                     'turn %.1e, place %.1e m, cost %.1e of it, %d steps at most' % (*worst, took))
        report.check('%s: its cost never rises, and one step lays the centroid on its goals\'' % name,
                     rose == 0 and off < near, '%d rises, the mean miss after a step %.1e m' % (rose, off))


def test_what_the_points_cannot_tell_gets_no_step(report):
    """A line of points any way in the world: the step's turn about the line is rounding, its turn across it the
    least-norm answer of the curvature, and twelve steps reach the closed form's cost; one point: no turn, and the
    step lays it on its goal; no weight: no step at all, the place its own bits."""
    import numpy as np
    from tools.cores import se3
    for single, near in ((False, 1e-12), (True, 1e-5)):
        rng, about, across, cost, spun, missed = np.random.default_rng(13), 0.0, 0.0, 0.0, 0.0, 0.0
        config = se3.cfg(single, most_turn=1.0, most_shift=1.0)
        for _ in range(200):
            p, g, w, _asked, pose, way = _case(np, rng, 12, line=True)
            planes = se3.laid(p, g, w, single)
            said = _run(se3, planes, config, pose, 12)
            first, turns = said[0], [pose[0]] + [s['r'].astype(float) for s in said[:-1]]
            keep = np.eye(3) - np.outer(pose[0] @ way, pose[0] @ way)
            least = -np.linalg.pinv(keep @ first['curve'].astype(float) @ keep, rcond=1e-6, hermitian=True) @ (
                keep @ first['turn'].astype(float))
            about = max([about] + [abs(float(s['turned'].astype(float) @ (turn @ way))) for s, turn in zip(said, turns)])
            across = max(across, float(np.linalg.norm(keep @ (first['turned'] - least)) / np.linalg.norm(least)))
            p64, g64, w64 = (planes.astype(float).reshape(se3.PLANES, -1)[rows, :12].T for rows in ((0, 1, 2), (3, 4, 5), 6))
            cost = max(cost, abs(float(said[-1]['cost']) - _best(np, p64, g64, w64.ravel())[2])
                       / _best(np, p64, g64, w64.ravel())[2])
            p, g, w, _asked, pose, _way = _case(np, rng, 1)
            one = _run(se3, se3.laid(p, g, w, single), config, pose, 2)
            spun, missed = max(spun, float(np.linalg.norm(one[0]['turned']))), max(missed, float(np.abs(one[1]['shift']).max()
                                                                                                / one[1]['weight']))
        name = 'float' if single else 'double'
        report.check('%s: no turn about a line of points, over 200 lines any way' % name, about < near,
                     '%.1e rad about it at most' % about)
        report.check('%s: across the line the least-norm turn, and the closed form\'s cost in twelve steps' % name,
                     across < (1e-3 if single else 1e-9) and cost < max(near, 1e-9) * 100.0,
                     '%.1e of the turn, %.1e of the cost' % (across, cost))
        report.check('%s: one point is not turned, and a step lays it on its goal' % name, spun < near and missed < near,
                     '%.1e rad, %.1e m' % (spun, missed))
        p, g, w, _asked, pose, _way = _case(np, rng, 21)
        dt = se3.kind(single)
        for label, weights in (('no weight', np.zeros(21)), ('weights under zero', -w)):
            f = se3.read(se3.step(se3.laid(p, g, weights, single), config, *pose))
            report.check('%s: %s is no step - the place its own bits, the turn within its rounding' % (name, label),
                         not f['turned'].any() and not f['shifted'].any() and (f['t'] == pose[1].astype(dt)).all()
                         and float(np.abs(f['r'] - pose[0].astype(dt)).max()) < 4.0 * np.finfo(dt).eps,
                         'weight %.1f, turn off its own by %.1e' % (float(f['weight']), float(np.abs(f['r'] - pose[0].astype(dt)).max())))


def test_far_off_it_keeps_its_grain(report):
    """A pose 13 km from the world's origin in float lanes: laid about an anchor beside it (`se3.laid`) its step is
    the one it takes at the origin, to a goal's last bit - no place enters the curvature, and none the misses -;
    laid about the origin the goals themselves have lost a millimetre before the first sum."""
    import numpy as np
    from tools.cores import se3
    rng, far = np.random.default_rng(17), np.array([10000.0, -8500.0, 1200.0])
    near, lost, wide = [0.0, 0.0], [0.0, 0.0], [0.0, 0.0]
    for _ in range(40):
        p, g, w, _asked, pose, _way = _case(np, rng)
        config = se3.cfg(True, most_turn=1.0, most_shift=1.0)
        home = se3.read(se3.step(se3.laid(p, g, w, True), config, *pose))
        want = se3.read(se3.step(se3.laid(p, g, w), se3.cfg(most_turn=1.0, most_shift=1.0), *pose))
        runs = [(wide, want)] + [(kept, se3.read(se3.step(se3.laid(p, g + far, w, True, anchor), config, pose[0],
                                                          pose[1] + far - np.asarray(anchor))))
                                 for kept, anchor in ((near, far), (lost, (0.0, 0.0, 0.0)))]
        for kept, f in runs:
            kept[0] = max(kept[0], float(np.linalg.norm(f['turned'] - home['turned'])))
            kept[1] = max(kept[1], float(np.linalg.norm(f['shifted'] - home['shifted'])))
    report.check('float lanes 13 km off, about an anchor: the step at the origin, to 1e-6 rad and 1e-7 m',
                 near[0] < 1e-6 and near[1] < 1e-7, '%.1e rad, %.1e m apart; the double\'s %.1e rad, %.1e m from them'
                 % (*near, *wide))
    report.check('about the origin the goals have lost their grain: a float\'s 1 mm at 10 km',
                 lost[0] > 1e-4 and lost[1] > 1e-4, '%.1e rad, %.1e m apart' % tuple(lost))


def test_it_stays_a_pose(report):
    """Ten thousand steps after a goal that wanders, a tenth of its points wild, soft weights: its turn on SO(3)
    to the lanes' rounding all the way, each step within its caps; and over 3000 arenas drawn wild - weights of
    zero, under zero, 1e-30 and 1e6, points on one spot, goals a kilometre off, any soft, any cap - every number
    of every step finite."""
    import numpy as np
    from tools.cores import se3
    for single in (False, True):
        dt, rng = se3.kind(single), np.random.default_rng(19)
        eps, name = float(np.finfo(dt).eps), 'float' if single else 'double'
        p, _g, w, (r, t), pose, _way = _case(np, rng)
        config, off, over, bad = se3.cfg(single, soft=400.0, most_turn=0.05, most_shift=0.02), 0.0, 0.0, 0
        for k in range(10000):
            r, t = _turned(np, rng.normal(0.0, 0.02, 3)) @ r, t + rng.normal(0.0, 0.005, 3)
            g = p @ r.T + t + rng.normal(0.0, 0.002, p.shape)
            g[k % 10::10] += rng.normal(0.0, 0.5, g[k % 10::10].shape)
            f = se3.read(se3.step(se3.laid(p, g, w, single), config, *pose))
            pose = (f['r'].copy(), f['t'].copy())
            turn = f['r'].astype(float)
            off = max(off, float(np.abs(turn.T @ turn - np.eye(3)).max()))
            over = max(over, float(np.linalg.norm(f['turned'])) / 0.05, float(np.linalg.norm(f['shifted'])) / 0.02)
            bad += not all(np.isfinite(v).all() for v in f.values())
        report.check('%s: on SO(3) over 10000 steps, each within its caps' % name,
                     off < 8.0 * eps and over < 1.0 + 8.0 * eps and bad == 0 and np.linalg.det(turn) > 0.0,
                     'r\'r off one by %.1e, the longest step %.7f of its cap, %d not finite' % (off, over, bad))
        bad, most = 0, 0.0
        for _ in range(3000):
            n = int(rng.integers(1, 70))
            p, g, w, _asked, pose, _way = _case(np, rng, n, line=bool(rng.integers(0, 2)), noise=float(rng.choice([0.0, 0.01, 10.0])))
            w = rng.choice([0.0, -1.0, 1e-30, 1.0, 1e6], n)
            p = p if rng.integers(0, 3) else np.repeat(p[:1], n, axis=0)
            g = g + rng.choice([0.0, 1e3]) * rng.normal(0.0, 1.0, 3)
            config = se3.cfg(single, soft=float(rng.choice([0.0, 1.0, 1e6])), damp_turn=float(rng.choice([0.0, 1e-3])),
                             most_turn=float(rng.choice([0.0, 0.3, 50.0])), most_shift=float(rng.choice([0.0, 0.1, 1e9])))
            for f in _run(se3, se3.laid(p, g, w, single), config, pose, 3):
                bad += not all(np.isfinite(v).all() for v in f.values())
                most = max(most, float(np.linalg.norm(f['turned'])))
        report.check('%s: every number finite over 3000 wild arenas, no turn over a radian' % name,
                     bad == 0 and most <= se3.TURN * (1.0 + 8.0 * eps), '%d steps not finite, the longest turn %.6f rad' % (bad, most))


def test_a_wild_goal_is_let_go(report):
    """A seventh of a sole's goals half a metre wild, forty arenas: with soft weights at a 5 cm miss the pose found
    lays every point within 5 mm of where the pose asked does; the squares alone leave each arena ten times
    further off at least."""
    import numpy as np
    from tools.cores import se3
    rng, soft, least = np.random.default_rng(23), 0.0, math.inf
    for _ in range(40):
        p, g, w, (r, t), pose, _way = _case(np, rng, 42, noise=0.001)
        g[::7] += rng.normal(0.0, 0.5, (6, 3))
        errs = []
        for told in (400.0, 0.0):
            f = _run(se3, se3.laid(p, g, w), se3.cfg(soft=told, most_turn=1.0, most_shift=1.0), pose, 30)[-1]
            errs.append(float(np.abs(p @ f['r'].T + f['t'] - (p @ r.T + t)).max()))
        soft, least = max(soft, errs[0]), min(least, errs[1] / errs[0])
    report.check('soft weights hold the pose asked where the squares alone leave it', soft < 5e-3 and least > 10.0,
                 'a point %.1e m off at most with them, the squares alone %.0f times further at least' % (soft, least))


def test_its_memory_and_its_pace(report):
    """The arena is one block on a cache line's boundary, eleven planes of whole lanes, zeros past the last point,
    the structs their precision's numbers with no padding; an arena in another shape is refused before the C; and
    what a point costs a step in each precision beside the twin."""
    import numpy as np
    from tools.cores import se3
    rng, said = np.random.default_rng(29), []
    for single in (False, True):
        item, lib = np.dtype(se3.kind(single)).itemsize, se3.library(single)
        p, g, w, _asked, pose, _way = _case(np, rng, 37)
        planes = se3.laid(p, g, w, single)
        report.check('%s: an arena of 37 points is eleven planes of 48 on a cache line, zeros past the last'
                     % ('float' if single else 'double'),
                     len(planes) == 11 * 48 and planes.ctypes.data % 64 == 0 and not planes.reshape(11, 48)[:, 37:].any()
                     and [lib.se3_size(k) for k in range(4)] == [item, 5 * item, 18 * item, 48 * item],
                     'structs of %s bytes, no padding' % [lib.se3_size(k) for k in range(4)])
        refused = 0
        for wrong in (planes[1:], planes[:11 * 47], planes.astype(np.float16), planes.reshape(11, 48)):
            try:
                se3.step(wrong, se3.cfg(single), *pose)
            except ValueError:
                refused += 1
        report.check('%s: an arena off its line, off its lanes, of another kind or shape is refused' % (
            'float' if single else 'double'), refused == 4, '%d of 4' % refused)
        big, config = se3.laid(*_case(np, rng, 4096)[:3], single), se3.cfg(single, soft=25.0)
        step, sums, wide = (se3.paced(planes, config, *pose, 20000), se3.paced(planes, config, *pose, 20000, True),
                            se3.paced(big, config, *pose, 500))
        t0 = time.perf_counter()
        se3.twin(big, config, *pose)
        said.append('%s a step of 37 points %.0f ns, its sums %.0f; %.2f ns a point of 4096, the twin %.0f times the C'
                    % ('float' if single else 'double', step * 1e9, sums * 1e9, wide / 4096 * 1e9,
                       (time.perf_counter() - t0) / wide))
    report.check('a step\'s cost in C, each precision', True, '; '.join(said))


ROSTER = (test_a_step_has_no_branch, test_the_c_is_its_twin, test_it_comes_to_the_best_pose,
          test_what_the_points_cannot_tell_gets_no_step, test_far_off_it_keeps_its_grain, test_it_stays_a_pose,
          test_a_wild_goal_is_let_go, test_its_memory_and_its_pace)


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
