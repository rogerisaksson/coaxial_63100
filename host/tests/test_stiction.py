#!/usr/bin/env python3
"""Friction at rest and sliding (`machine.world.stiction`): MuJoCo's contacts' friction rows stuck and sliding.

A load a contact holds - a block on a slope inside its cone - creeps in MuJoCo's own step at R f / B and rests
with its rows stuck, R f / (K I) off where it stuck; a load past its cone slides on its sliding friction, at any
speed, and let go springs back no further than its cone held; a load between the two holds at rest and, once
moved, slides on; a ball rolls as freely; where no row grips the
step is MuJoCo's own to the bit; and the kernels in C (wbc_world_stuck, wbc_world_slipped) are their python twins
to the bit over contacts that slide, tumble and roll. The robot at rest, by the world's truth: tools/sim/creep.py.
The robot's world's steps in C as in python, its contacts' slips with them: test_tick."""
import importlib
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A flat block's half side and half height, m, and its mass, kg; the floor's friction under it, at rest; a slope
#: its cone holds sliding too, one between its friction at rest and sliding, and one past both.
HALF, LOW, MASS, MU, HELD, BETWEEN, PAST = 0.1, 0.02, 2.0, 0.5, math.atan(0.3), math.atan(0.45), math.atan(0.7)
#: A scene settles this long, s, and is read this long after; a step, s.
SETTLE_S, READ_S, STEP_S = 2.0, 1.0, 0.001
#: What rests moves less than this: a millimetre an hour, mm/s.
MM_H = 1.0 / 3600.0
BLOCK = '<body pos="0 0 %g"><freejoint/><geom type="box" size="%g %g %g" mass="%g"/></body>' % (
    LOW, HALF, HALF, LOW, MASS)


def _scene(bodies, tilt=0.0, condim=3):
    """MuJoCo's model of a floor and `bodies` on it, gravity turned `tilt` rad about y - a slope of that angle -,
    the robot's world's options (an elliptic cone, implicitfast, a step STEP_S); a contact of `condim` rows where
    its geoms state none."""
    mujoco = importlib.import_module('mujoco')
    return mujoco.MjModel.from_xml_string(
        '<mujoco><option timestep="%g" gravity="%.17g 0 %.17g" integrator="implicitfast" cone="elliptic" '
        'impratio="10"/><default><geom friction="%g 0.005 0.0001" condim="%d"/></default><worldbody>'
        '<geom type="plane" size="20 20 0.1"/>%s</worldbody></mujoco>'
        % (STEP_S, 9.81 * math.sin(tilt), -9.81 * math.cos(tilt), MU, condim, bodies))


def _stepped(m, seconds, stuck=True, in_c=True, d=None, state=None, each=None):
    """(MuJoCo's data, the contacts' slips) of model `m` stepped `seconds` on: its friction rows `stuck`
    (`stiction.stepped`, in C where `in_c`) or MuJoCo's own step; `each` called on them after every step."""
    mujoco = importlib.import_module('mujoco')
    from machine.world import stiction
    d = mujoco.MjData(m) if d is None else d
    state = stiction.laid(m) if state is None else state
    for _ in range(round(seconds / STEP_S)):
        if stuck:
            stiction.stepped(mujoco, m, d, state, in_c)
        else:
            mujoco.mj_step(m, d)
        if each is not None:
            each(d, state)
    return d, state


def _rows(d):
    """(each contact's first friction row's R, its spring K I, its damping B, its friction force along its plane)."""
    at = d.contact.efc_address[:d.ncon] + 1
    half = 0.5 * d.efc_KBIP[at, 1]
    return d.efc_R[at], half * half * d.efc_KBIP[at, 2], d.efc_KBIP[at, 1], np.hypot(d.efc_force[at], d.efc_force[at + 1])


def test_a_load_its_cone_holds_rests(report, _c):
    """A block on a slope its cone holds. In MuJoCo's own step it creeps down at what its friction rows' dampers
    let it, R f / B a contact (its weight along the slope over B times the sum of 1 / R); its rows stuck, it rests -
    under a millimetre an hour - with each contact R f / (K I) off where it stuck, as the kernel keeps it."""
    m = _scene(BLOCK, HELD)
    crept, held = {}, {}
    for into, stuck in ((crept, False), (held, True)):
        d, state = _stepped(m, SETTLE_S, stuck)
        into['from'] = float(d.qpos[0])
        d, state = _stepped(m, READ_S, stuck, d=d, state=state)
        into['speed'], into['rows'] = (float(d.qpos[0]) - into['from']) / READ_S, _rows(d)
        into['slip'] = np.linalg.norm(state[3][:d.ncon, :3], axis=1)
    pull = MASS * 9.81 * math.sin(HELD)
    r, spring, damp, force = crept['rows']
    creeps = pull / float((damp / r).sum())
    report.check("a block its cone holds creeps down a slope at R f / B in MuJoCo's own step",
                 1e3 * crept['speed'] > 100.0 * MM_H and abs(crept['speed'] / creeps - 1.0) < 0.01,
                 '%.4f mm/s, R f / B %.4f' % (1e3 * crept['speed'], 1e3 * creeps))
    r, spring, damp, force = held['rows']
    off = r * force / spring
    report.check('its rows stuck, it rests: under a millimetre an hour',
                 1e3 * abs(held['speed']) < MM_H, '%.3g mm/s' % (1e3 * held['speed']))
    report.check('each contact R f / (K I) off where it stuck',
                 bool(np.all(off > 1e-6) and np.allclose(held['slip'], off, rtol=1e-6)),
                 'slips %s um, R f / (K I) %s um' % (np.round(1e6 * held['slip'], 3), np.round(1e6 * off, 3)))
    report.check('its contacts bear its weight along the slope', abs(float(force.sum()) / pull - 1.0) < 1e-6,
                 '%.6f N of %.6f' % (float(force.sum()), pull))


def test_a_load_past_its_cone_slides_as_it_did(report, _c):
    """A block on a slope past its cone slides down as its friction lets it, g (sin - mu cos): on the one friction
    MuJoCo's own step knows, on its sliding friction with its rows stuck. A block sent sliding along a level floor
    stops where its sliding friction stops it (v^2 / (2 mu g)), at any speed. Stopped, it springs back no further
    than its cone held twice over (R mu N / (K I) a contact: from its cone one way to its cone the other), not the
    way it came."""
    from machine.world import stiction
    m, slides = _scene(BLOCK, PAST), MU * stiction.KINETIC
    laws = [0.5 * 9.81 * (math.sin(PAST) - mu * math.cos(PAST)) * READ_S ** 2 for mu in (MU, slides)]
    went = [float(_stepped(m, READ_S, stuck)[0].qpos[0]) for stuck in (False, True)]
    report.check("a block past its cone slides down a slope on MuJoCo's one friction, on its sliding friction stuck",
                 all(abs(x / law - 1.0) < 0.01 for x, law in zip(went, laws)),
                 "%.4f m MuJoCo's (its law %.4f), %.4f m stuck (its law %.4f)" % (went[0], laws[0], went[1], laws[1]))
    m, speed, far = _scene(BLOCK), 0.5, [0.0]
    d, state = _stepped(m, SETTLE_S)
    d.qvel[0], start = speed, float(d.qpos[0])

    def each(d, _state):
        far[0] = max(far[0], float(d.qpos[0]) - start)
    d, state = _stepped(m, SETTLE_S, d=d, state=state, each=each)
    r, spring, _damp, _force = _rows(d)
    cone = float((r * MU * d.efc_force[d.contact.efc_address[:d.ncon]] / spring).max())
    back, stops = far[0] - (float(d.qpos[0]) - start), speed * speed / (2.0 * slides * 9.81)
    report.check('a block sent sliding stops where its sliding friction stops it', abs(far[0] / stops - 1.0) < 0.02,
                 '%.2f mm, v^2 / (2 mu g) %.2f mm' % (1e3 * far[0], 1e3 * stops))
    report.check('stopped, it springs back no further than its cone held twice over', 0.0 < back < 2.0 * cone,
                 '%.2f um back, its cone held %.2f um; it came %.2f mm' % (1e6 * back, 1e6 * cone, 1e3 * far[0]))


def test_a_load_between_holds_at_rest_and_slides_once_moved(report, _c):
    """A block settled on a slope then raised between its friction at rest and its sliding friction: let be, it
    holds - under a millimetre an hour -; shoved, it slides on and gains speed as its sliding friction lets it
    (g (sin - mu cos)), where MuJoCo's one friction stops it."""
    from machine.world import stiction
    m, shove = _scene(BLOCK, HELD), 0.3

    def tilted(stuck):
        """The block settled on the slope its cone holds, the slope then raised between its two frictions."""
        m.opt.gravity[:] = (9.81 * math.sin(HELD), 0.0, -9.81 * math.cos(HELD))
        d, state = _stepped(m, SETTLE_S, stuck)
        m.opt.gravity[:] = (9.81 * math.sin(BETWEEN), 0.0, -9.81 * math.cos(BETWEEN))
        return _stepped(m, SETTLE_S, stuck, d=d, state=state)
    d, state = tilted(True)
    rest = float(d.qpos[0])
    d, state = _stepped(m, READ_S, d=d, state=state)
    held = (float(d.qpos[0]) - rest) / READ_S
    report.check('a block between its friction at rest and sliding holds: under a millimetre an hour',
                 1e3 * abs(held) < MM_H, '%.3g mm/s' % (1e3 * held))
    gains = 9.81 * (math.sin(BETWEEN) - MU * stiction.KINETIC * math.cos(BETWEEN))
    sped = []
    for stuck in (False, True):
        d, state = tilted(stuck)
        d.qvel[0] = shove
        d, state = _stepped(m, READ_S, stuck, d=d, state=state)
        sped.append(float(d.qvel[0]))
    report.check("shoved, it slides on and gains speed as its sliding friction lets it; MuJoCo's one friction stops it",
                 abs((sped[1] - shove) / (gains * READ_S) - 1.0) < 0.02 and abs(sped[0]) < 0.01 * shove,
                 "%.4f m/s stuck after %g s (its law %.4f), %.5f m/s MuJoCo's" % (
                     sped[1], READ_S, shove + gains * READ_S, sped[0]))


def test_a_contacts_friction_falls_by_its_speed(report, _c):
    """The SE(3) core's friction on the world's contacts (se3_rubs, wbc_world_stuck): a block on a floor bears its
    friction at rest, half way to its sliding share sliding at the Stribeck speed (`world.toml` stribeck_m_s), its
    sliding share sliding fast - each contact's coefficient the core's own of its slip's speed, to the bit, in C and
    in its twin."""
    mujoco = importlib.import_module('mujoco')
    from machine.body import mechatronics
    from machine.world import stiction
    from tools.cores import se3
    m, v_s, share = _scene(BLOCK), float(mechatronics.environment()['stribeck_m_s']), stiction.KINETIC
    told, exact = [], True
    for speed, law in ((0.0, 1.0), (v_s, 0.5 * (1.0 + share)), (100.0 * v_s, share + (1.0 - share) / 10001.0)):
        for in_c in (True, False):
            d, state = _stepped(m, SETTLE_S, in_c=in_c)
            d.qvel[:] = 0.0
            d.qvel[0] = speed
            mujoco.mj_step1(m, d)
            n = d.ncon
            laid, at = d.contact.friction[:n, 0].copy(), d.contact.efc_address[:n]
            stiction.stuck(d, state, in_c)
            v1, v2 = d.efc_vel[at + 1].tolist(), d.efc_vel[at + 2].tolist()
            want = [f * se3.rubs_twin(1.0, share, stiction.FADE, (a * a) + (b * b)) for f, a, b in zip(laid.tolist(), v1, v2)]
            exact = exact and d.contact.friction[:n, 0].tolist() == want
            told.append(float(np.mean(d.contact.friction[:n, 0] / laid)) - law)
    report.check('at rest, at the Stribeck speed and fast its friction at rest, half way and its sliding share',
                 max(abs(x) for x in told) < 1e-9, 'off the law %s' % ', '.join('%.1e' % x for x in told))
    report.check("and each contact's coefficient the SE(3) core's own of its slip's speed, in C and in its twin",
                 exact, 'the same bits' if exact else 'apart')


def test_a_ball_rolls_as_freely(report, _c):
    """A ball let go on a slope rolls down it without sliding, 5/7 g sin: MuJoCo's own step and its rows stuck
    alike - rolling carries no slip -, and stuck its contact slides slower than MuJoCo's own lets it (whose damper
    gives rolling its friction only by sliding)."""
    ball = '<body pos="0 0 0.1"><freejoint/><geom type="sphere" size="0.1" mass="1" friction="1"/></body>'
    tilt, slid, went = math.atan(0.2), [], []
    m = _scene(ball, tilt)
    law = 0.5 * (5.0 / 7.0) * 9.81 * math.sin(tilt) * READ_S ** 2
    for stuck in (False, True):
        d, _state = _stepped(m, READ_S, stuck)
        went.append(float(d.qpos[0]))
        at = int(d.contact.efc_address[0]) + 1
        slid.append(float(np.hypot(d.efc_vel[at], d.efc_vel[at + 1])))
    report.check('a ball rolls down a slope at 5/7 g sin, its rows stuck or not',
                 all(abs(x / law - 1.0) < 0.01 for x in went) and abs(went[1] / went[0] - 1.0) < 1e-3,
                 "%.4f m MuJoCo's, %.4f m stuck, the law's %.4f m" % (went[0], went[1], law))
    report.check("stuck, its contact slides slower than MuJoCo's own lets it", slid[1] < 0.01 * slid[0],
                 "%.3g mm/s stuck, %.3g mm/s MuJoCo's" % (1e3 * slid[1], 1e3 * slid[0]))


#: The twins' scene: a block thrown tumbling, a ball and a capsule that roll and spin (their contacts of 4 and 6
#: rows) and a block at rest; stepped this long, s.
TUMBLE = ('<body pos="0 0 0.5"><freejoint/><geom type="box" size="0.1 0.06 0.04" mass="1"/></body>'
          '<body pos="1 0 0.1"><freejoint/><geom type="sphere" size="0.1" mass="1" condim="4"/></body>'
          '<body pos="2 0 0.06"><freejoint/><geom type="capsule" fromto="0 -0.1 0 0 0.1 0" size="0.05" mass="1" '
          'condim="6"/></body>' + BLOCK.replace('pos="0 0', 'pos="3 0'))
TWIN_S = 1.5


def _thrown(m):
    """MuJoCo's data of the twins' scene `m`, its bodies thrown: the block spinning and falling, the ball and the
    capsule sent rolling askew."""
    mujoco = importlib.import_module('mujoco')
    d = mujoco.MjData(m)
    d.qvel[0:6] = (1.0, 0.3, 0.0, 3.0, 5.0, 1.0)
    d.qvel[6:12] = (0.8, 0.4, 0.0, 0.0, 0.0, 6.0)
    d.qvel[12:18] = (0.5, 0.2, 0.0, 0.0, 0.0, 4.0)
    return d


def test_the_kernels_in_c_are_their_twins(report, _c):
    """Two of a scene alike - a block thrown tumbling, a ball and a capsule rolling and spinning, a block at rest on
    a slope -, one's friction rows stuck in C, the other's by their python twins: MuJoCo's state and every slip the
    same bits at every step, contacts of 3, 4 and 6 rows among them. And with fewer contacts kept than bear, the
    same bits still."""
    mujoco = importlib.import_module('mujoco')
    from machine.world import stiction
    from tools.cores import world
    m = _scene(TUMBLE, math.atan(0.2))
    tally = {'steps': 0, 'apart': 0, 'most': 0, 'dims': set(), 'short': 0}

    def few(kept):
        """The contacts' slips of `m` with `kept` contacts kept: wbc_stuck_t laid on arrays that short."""
        whole = stiction.laid(m)
        pair, at, slip, row, held, borne = (a[:kept].copy() for a in (*whole[1:5], whole[7], whole[10]))
        c, sprung, yielded = world.stuck(pair, at, slip, row, held, borne, whole[8], whole[9], STEP_S, stiction.FADE)
        return c, pair, at, slip, row, sprung, yielded, held, whole[8], whole[9], borne
    for kept in (None, 3):
        twins = [_thrown(m), _thrown(m)]
        states = [stiction.laid(m), stiction.laid(m)] if kept is None else [few(kept), few(kept)]
        for _ in range(round(TWIN_S / STEP_S)):
            for d, state, in_c in zip(twins, states, (True, False)):
                stiction.stepped(mujoco, m, d, state, in_c)
            a, b = twins
            tally['apart'] += any(getattr(a, n).tobytes() != getattr(b, n).tobytes() for n in ('qpos', 'qvel', 'qacc'))
            tally['apart'] += any(x.tobytes() != y.tobytes() for x, y in zip(
                (*states[0][1:5], states[0][7]), (*states[1][1:5], states[1][7])))
            tally['apart'] += states[0][0].n != states[1][0].n
            tally['steps'] += 1
            tally['most'] = max(tally['most'], a.ncon)
            tally['short'] += kept is not None and a.ncon > kept
            tally['dims'] |= set(a.contact.dim[:a.ncon].tolist())
    report.check('the kernels in C are their twins to the bit, %(steps)d steps, %(most)d contacts at most, of '
                 '%(dims)s rows, %(short)d steps with fewer kept than bore' % tally,
                 not tally['apart'] and tally['dims'] >= {3, 4, 6} and tally['most'] >= 8 and tally['short'] > 100,
                 '%(apart)d apart' % tally)


def test_where_no_row_grips_the_step_is_mujocos(report, _c):
    """A block on a slope without friction - its contacts of one row -, stepped by MuJoCo's own step and by its two
    halves with the kernels between: the same bits, every step."""
    mujoco = importlib.import_module('mujoco')
    from machine.world import stiction
    m = _scene(BLOCK, HELD, condim=1)
    a, b, state, apart = mujoco.MjData(m), mujoco.MjData(m), stiction.laid(m), 0
    for _ in range(round(READ_S / STEP_S)):
        mujoco.mj_step(m, a)
        stiction.stepped(mujoco, m, b, state)
        apart += any(getattr(a, n).tobytes() != getattr(b, n).tobytes() for n in ('qpos', 'qvel', 'qacc'))
    report.check("where no row grips, the step is MuJoCo's own to the bit", not apart and abs(float(a.qpos[0])) > 0.1,
                 '%d apart; the block slid %.3f m' % (apart, float(a.qpos[0])))


ROSTER = (test_a_load_its_cone_holds_rests, test_a_load_past_its_cone_slides_as_it_did,
          test_a_load_between_holds_at_rest_and_slides_once_moved, test_a_ball_rolls_as_freely,
          test_the_kernels_in_c_are_their_twins, test_where_no_row_grips_the_step_is_mujocos)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, None)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
