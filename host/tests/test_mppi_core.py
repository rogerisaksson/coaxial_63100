#!/usr/bin/env python3
"""The plane's rollouts in C (mppi/) through ctypes and NVRTC: its form, its twin, its plant.

No jump its data decides in its source and no flag set from its data in this host's code, built
under the aero core's warnings with none; its costs the numpy twin's on the CPU and on a card
where one answers; the twin's forces the plant's own (`aerobat`'s functions)."""
import glob
import io
import os
import platform
import re
import shutil
import subprocess
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report
from test_aero_core import BANNED, FLAGGED, JUMPS


def _state(np, rng, h=0.6):
    """A plane in the air at `h`, its nose up and leant, a little speed and spin."""
    from machine import se3mpc
    a, b = rng.normal(0.0, 0.4, 3), rng.normal(0.0, 0.4, 3)
    k = np.array([[0.0, -a[2], a[1]], [a[2], 0.0, -a[0]], [-a[1], a[0], 0.0]])
    t = float(np.linalg.norm(a))
    turn = np.eye(3) + np.sin(t) / t * k + (1.0 - np.cos(t)) / t ** 2 * (k @ k)
    return (np.array([0.0, h, 0.0]), b, turn, rng.normal(0.0, 0.3, 3)), se3mpc


def _asked(np, se3mpc, mpc):
    """A pass's reference: on along x at 4 m/s, the belly across it, the tail at the skim."""
    ref = np.zeros((mpc.h, se3mpc.REF))
    ref[:, se3mpc.PLACE] = 0.1 * np.arange(1, mpc.h + 1)
    ref[:, se3mpc.PLACE + 1], ref[:, se3mpc.SPEED] = 0.6, 4.0
    ref[:, se3mpc.BELLY + 2], ref[:, se3mpc.NOSE + 1] = 1.0, 1.0
    ref[:, se3mpc.W_BELLY:] = (12.0, 0.2, se3mpc.SKIM_M, 1.0, 0.05)
    return ref


def test_a_rollout_has_no_branch(report):
    """mppi/ holds no `if`, `else`, `while`, `?`, `&&`, `||`, `break` or `goto`, its comments and
    strings aside; it builds under the aero core's warnings with none; and as this host's
    compiler makes it, no flag is set from its data: what conditional jumps it has are its
    loops' ends."""
    from tools.cores import mppi
    from tools.cores.build import FLAGS, OUT, cached, find_cc
    here = os.path.dirname(mppi.INCLUDES[0])
    sources = sorted(glob.glob(os.path.join(here, '*', '*.[ch]')))
    found = []
    for path in sources:
        with io.open(path, encoding='utf-8') as f:
            code = re.sub(r'"[^"\n]*"', '""', re.sub(r'/\*.*?\*/', ' ', f.read(), flags=re.S))
        found += ['%s: %s' % (os.path.basename(path), hit.group(0)) for hit in re.finditer(BANNED, code)]
    report.check('no jump the data decides in %d sources' % len(sources), len(sources) == 2 and not found,
                 ', '.join(found[:6]) or ' '.join(os.path.basename(p) for p in sources))
    warnings = cached(mppi.SOURCES, mppi.INCLUDES, 'mppi', list(mppi.BUILT))[1]
    report.check('built under the aero core\'s warnings with none', not warnings, '; '.join(warnings[:2]) or '0')
    cc = find_cc()
    machine = 'arm' if platform.machine().lower() in ('arm64', 'aarch64') else 'x86'
    objdump = shutil.which('objdump') or os.path.join(os.path.dirname(cc or ''), 'objdump')
    obj = os.path.join(OUT, 'mppi_%d.o' % os.getpid())
    subprocess.run([cc, '-c'] + FLAGS + ['-O2', '-fno-math-errno', '-I', mppi.INCLUDES[0], mppi.SOURCES[0], '-o', obj],
                   check=True)
    text = subprocess.run([objdump, '-d', obj], capture_output=True, text=True, check=True).stdout
    os.remove(obj)
    flagged = sorted(set(re.findall(FLAGGED[machine], text)))
    size = len(re.findall(r'^\s*[0-9a-f]+:\t', text, flags=re.M))
    report.check('in this host\'s code no flag is set from its data', not flagged and size > 500,
                 ', '.join(flagged[:6]) or '%d instructions, %d conditional jumps: its loops\' ends'
                 % (size, len(re.findall(JUMPS[machine], text))))


def test_the_c_is_its_twin(report):
    """A pass's rollouts from states in the air, under controls about a hover's: each one's cost
    on the CPU's C and on a card where one answers, against the numpy twin's, to 1e-12."""
    import numpy as np
    from machine import aerobat
    rng = np.random.default_rng(7)
    x, se3mpc = _state(np, rng)
    cpu = se3mpc.Mppi(aerobat, k=256, xp=np)
    h = cpu.h
    u = np.clip(np.concatenate([rng.uniform(0.25, 0.4, (256, h, 1)), rng.normal(0.0, 0.15, (256, h, 3))], axis=2),
                cpu.lo, cpu.hi)
    ref = _asked(np, se3mpc, cpu)
    twin = cpu.twin(x, u, ref, 365.0, 345.0, (0.5, 0.0, -0.3))
    worst = lambda c: float(np.max(np.abs(c - twin) / np.maximum(1.0, np.abs(twin))))
    report.check('the CPU\'s C costs each rollout as its twin, to 1e-12',
                 worst(cpu.costs(x, u, ref, 365.0, 345.0, (0.5, 0.0, -0.3))) < 1e-12,
                 '%.1e, the median cost %.1f' % (worst(cpu.costs(x, u, ref, 365.0, 345.0, (0.5, 0.0, -0.3))),
                                                 float(np.median(twin))))
    xp = se3mpc.backend()
    if xp is np:
        report.check('no card answers here: the CPU\'s C is the one path', True, 'COAXIAL_GPU or no cupy')
        return
    card = se3mpc.Mppi(aerobat, k=256, xp=xp).costs(x, xp.asarray(u), ref, 365.0, 345.0, (0.5, 0.0, -0.3)).get()
    report.check('the card costs each rollout as the twin, to 1e-12', worst(card) < 1e-12, '%.1e' % worst(card))


def test_the_twin_is_the_plant(report):
    """At states in the air, under controls across their range: the twin's force and torque the
    plant's own - `aerobat`'s inflow, drag and propeller, `wingborne.Wing`'s wing and side."""
    import numpy as np
    from machine import aerobat
    rng = np.random.default_rng(11)
    worst = [0.0, 0.0]
    for _ in range(40):
        x, se3mpc = _state(np, rng)
        x = (x[0], rng.normal(0.0, 6.0, 3), x[2], x[3])
        u = np.concatenate([rng.uniform(0.0, 1.0, 1), rng.uniform(-1.0, 1.0, 3)])
        wind = rng.normal(0.0, 2.0, 3)
        f, t = se3mpc.forces(aerobat, x, u, 365.0, 345.0, wind)
        model = se3mpc.Batch(aerobat, np)
        f2, t2 = model.forces(x[1][None], x[2][None], x[3][None], u[None], 365.0, 345.0, wind)
        worst = [max(worst[0], float(np.abs(f - f2[0]).max())), max(worst[1], float(np.abs(t - t2[0]).max()))]
    report.check('the twin\'s force and torque are the plant\'s over 40 states, to 1e-9',
                 max(worst) < 1e-9, '%.1e N, %.1e N m' % tuple(worst))


ROSTER = (test_a_rollout_has_no_branch, test_the_c_is_its_twin, test_the_twin_is_the_plant)


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
