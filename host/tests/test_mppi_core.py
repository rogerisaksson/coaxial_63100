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
    from machine.aircraft import se3mpc
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
    subprocess.run([cc, '-c'] + FLAGS + ['-O2', '-fno-math-errno']
                   + [x for d in mppi.INCLUDES for x in ('-I', d)] + [mppi.SOURCES[0], '-o', obj],
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
    from machine.aircraft import aerobat
    rng = np.random.default_rng(7)
    x, se3mpc = _state(np, rng)
    cpu = se3mpc.Mppi(aerobat, k=256, xp=np)
    h = cpu.h
    u = np.clip(np.concatenate([rng.uniform(0.25, 0.4, (256, h, 1)), rng.normal(0.0, 0.15, (256, h, 3))], axis=2),
                cpu.lo, cpu.hi)
    ref = _asked(np, se3mpc, cpu)
    twin = cpu.twin(x, u, ref, 365.0, 345.0, (0.5, 0.0, -0.3))
    worst = lambda c: float(np.max(np.abs(c - twin) / np.maximum(1.0, np.abs(twin))))
    cpu_got = cpu.costs(x, u, ref, 365.0, 345.0, (0.5, 0.0, -0.3))
    serial = np.concatenate([cpu.costs(x, u[i:i + 32], ref, 365.0, 345.0, (0.5, 0.0, -0.3)) for i in range(0, 256, 32)])
    report.check('the CPU\'s C costs each rollout as its twin, to 1e-12',
                 worst(cpu_got) < 1e-12 and cpu_got.tobytes() == serial.tobytes(),
                 '%.1e, serial match %s, the median cost %.1f' % (worst(cpu_got), cpu_got.tobytes() == serial.tobytes(),
                                                                  float(np.median(twin))))
    tail = cpu.costs(x, u[:65], ref, 365.0, 345.0, (0.5, 0.0, -0.3))
    tail_serial = cpu.costs(x, u[:17], ref, 365.0, 345.0, (0.5, 0.0, -0.3))
    far_x = (x[0], x[1], x[2], np.array([2.2e8, 0.0, 0.0]))
    far_c = cpu.costs(far_x, u[:11], ref, 365.0, 345.0, (0.5, 0.0, -0.3))
    far_t = cpu.twin(far_x, u[:11], ref, 365.0, 345.0, (0.5, 0.0, -0.3))
    far_err = float(np.max(np.abs(far_c - far_t) / np.maximum(1.0, np.abs(far_t))))
    report.check('non-multiple-of-8 tails and large-angle LM_FAR fallback match the twin',
                 tail.tobytes() == cpu_got[:65].tobytes() and tail_serial.tobytes() == cpu_got[:17].tobytes()
                 and far_err < 1e-12,
                 'tail65=%s tail17=%s far=%.1e' % (
                     tail.tobytes() == cpu_got[:65].tobytes(),
                     tail_serial.tobytes() == cpu_got[:17].tobytes(), far_err))
    m_c, m_py = se3mpc.Mppi(aerobat, k=128, h=8, xp=np, seed=9), se3mpc.Mppi(aerobat, k=128, h=8, xp=np, seed=9)
    m_py._lib = None
    ref_s = _asked(np, se3mpc, m_c)
    pr_1d = (0.35, 0.1, -0.1, 0.05)
    pr_2d = np.full((8, 4), pr_1d)
    u_c1, u_p1 = m_c.solve(x, ref_s, 365.0, 345.0, prior=pr_1d), m_py.solve(x, ref_s, 365.0, 345.0, prior=pr_1d)
    u_c2, u_p2 = m_c.solve(x, ref_s, 365.0, 345.0, prior=pr_2d), m_py.solve(x, ref_s, 365.0, 345.0, prior=pr_2d)
    m_sc = se3mpc.Mppi(aerobat, k=64, h=8, sigma=0.1, xp=np, seed=9)
    m_vec = se3mpc.Mppi(aerobat, k=64, h=8, sigma=(0.1, 0.1, 0.1, 0.1), xp=np, seed=9)
    u_sc, u_vec = m_sc.solve(x, ref_s, 365.0, 345.0), m_vec.solve(x, ref_s, 365.0, 345.0)
    calls = []
    orig_costs = m_sc.costs
    m_sc.costs = lambda *a, **kw: (calls.append(1) or orig_costs(*a, **kw))
    m_sc.solve(x, ref_s, 365.0, 345.0)
    report.check('fused C11 solve matches Python fallback, broadcasts scalar sigma, and honours costs overrides',
                 u_c1 == u_p1 and u_c2 == u_p2 and u_sc == u_vec and len(calls) == 1,
                 '1d=%s 2d=%s scalar=%s calls=%d' % (u_c1 == u_p1, u_c2 == u_p2, u_sc == u_vec, len(calls)))
    xp = se3mpc.backend()
    if xp is np:
        report.check('no card answers here: the CPU\'s C is the one path', True, 'COAXIAL_GPU or no cupy')
        return
    card = se3mpc.Mppi(aerobat, k=256, xp=xp).costs(x, xp.asarray(u), ref, 365.0, 345.0, (0.5, 0.0, -0.3)).get()
    report.check('the card costs each rollout as the twin, to 1e-12', worst(card) < 1e-12, '%.1e' % worst(card))


def test_the_twin_is_the_plant(report):
    """At states in the air, under controls across their range: the twin's force and torque the
    plant's own - `aerobat`'s inflow, drag and propeller, `wingborne.Wing`'s wing and side."""
    import types
    import numpy as np
    from machine.aircraft import aerobat
    rng = np.random.default_rng(11)
    worst = [0.0, 0.0]
    for idx in range(40):
        x, se3mpc = _state(np, rng)
        x = (x[0], rng.normal(0.0, 6.0, 3), x[2], x[3])
        u = np.concatenate([rng.uniform(0.0, 1.0, 1), rng.uniform(-1.0, 1.0, 3)])
        wind = rng.normal(0.0, 2.0, 3)
        w_p = 20.0 if idx < 10 else 365.0
        f, t = se3mpc.forces(aerobat, x, u, w_p, 345.0, wind)
        model = se3mpc.Batch(aerobat, np)
        f2, t2 = model.forces(x[1][None], x[2][None], x[3][None], u[None], w_p, 345.0, wind)
        worst = [max(worst[0], float(np.abs(f - f2[0]).max())), max(worst[1], float(np.abs(t - t2[0]).max()))]
    report.check('the twin\'s force and torque are the plant\'s over 40 states, to 1e-9',
                 max(worst) < 1e-9, '%.1e N, %.1e N m' % tuple(worst))
    x0 = (np.array([0.0, 0.75, 0.0]), np.array([2.5, 0.0, 0.0]),
          np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]), np.zeros(3))
    u0 = np.clip(rng.normal(0.3, 0.15, (32, 4, 4)), [0.0, -1.0, -1.0, -1.0], [1.0, 1.0, 1.0, 1.0])
    alt = types.SimpleNamespace(**{
        **{k: getattr(aerobat, k) for k in dir(aerobat) if not k.startswith('__')},
        'INFLOW_FLOOR': 0.15, 'WAKE_FLOOR_M_S': 12.5, 'TOUCH_M': 0.08,
        '_aero_inflow': lambda along, vh, wp: (
            max(0.0, 1.0 - along / (aerobat.INFLOW_J0 * max(1.0, aerobat.BLADE_PITCH * abs(wp) / (2.0 * np.pi))))
            * (1.0 + 0.04 * aerobat.CL_ALPHA * min(0.15, vh / max(wp * aerobat.PROP_R, 12.5)))),
    })
    fa, ta = se3mpc.forces(alt, x0, u0[0, 0], 30.0, 345.0)
    fn, _ = se3mpc.forces(aerobat, x0, u0[0, 0], 30.0, 345.0)
    fb, tb = se3mpc.Batch(alt, np).forces(x0[1][None], x0[2][None], x0[3][None], u0[0, :1], 30.0, 345.0, np.zeros(3))
    mpc_nom, mpc_alt = se3mpc.Mppi(aerobat, k=32, h=4, xp=np), se3mpc.Mppi(alt, k=32, h=4, xp=np)
    ref = _asked(np, se3mpc, mpc_alt)
    c_alt, t_alt = mpc_alt.costs(x0, u0, ref, 30.0, 345.0), mpc_alt.twin(x0, u0, ref, 30.0, 345.0)
    c_nom = mpc_nom.costs(x0, u0, ref, 30.0, 345.0)
    report.check('derived inflow, wake-floor and skim constants propagate through C and twin',
                 float(np.abs(fa - fb[0]).max()) < 1e-9 and float(np.abs(ta - tb[0]).max()) < 1e-9
                 and float(np.abs(fa - fn).max()) > 1e-4
                 and float(np.max(np.abs(c_alt - t_alt) / np.maximum(1.0, np.abs(t_alt)))) < 1e-12
                 and float(np.max(np.abs(c_alt - c_nom))) > 10.0,
                 'C-vs-twin %.1e, force diff %.2e N, cost diff %.1f' % (
                     float(np.max(np.abs(c_alt - t_alt) / np.maximum(1.0, np.abs(t_alt)))),
                     float(np.abs(fa - fn).max()), float(np.max(np.abs(c_alt - c_nom)))))


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
