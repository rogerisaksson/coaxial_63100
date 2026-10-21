#!/usr/bin/env python3
"""Her situation read from her sensors (`machine.situation`): her text's bounds and caps, the readings
off what she senses, each situation entered unannounced on the world told within a second."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A situation entered unannounced is told within TOLD_S s at HELD of WOBBLES (her lay turned, rad).
TOLD_S, HELD, WOBBLES = 1.0, 3, (0.0, 2e-4, -2e-4, 4e-4)
#: Each scene (tools/sim/scenarios.py): the body that has her in it, bearing this share of her
#: weight by the world's own contacts - a sole landed on it, a seat sat on, a wall pushed MET_N -,
#: and what her law tells there.
SCENES = {'ice': ('ice', 0.3, 'slippery'), 'slope': ('slope_ramp', 0.3, 'uphill'),
          'chair': ('chair', 0.5, 'seated'), 'wall': ('wall', 0.04, 'held')}


def test_her_text_names_her_situations(report):
    """Her text's situations, each bounds on her readings and caps on her rhythm: the slippery floor
    under 0.25 of grip with the ice walk's caps, a slope past 0.08 either way, a seat bearing half
    her weight, lying, held by a push on her, stepping over an edge met; one without a bound, with
    a bound of 0, on no reading of hers or neither below nor above is refused."""
    from machine import situation
    table = situation.text()
    names = table['names']
    report.check('her text names %s' % ', '.join(names),
                 {'slippery', 'uphill', 'downhill', 'seated', 'lying', 'held', 'stepping'} <= set(names))
    k, g = names.index('slippery'), situation.READINGS.index('grip')
    report.check("the slippery floor is grip below 0.25, its caps the ice walk's, the envelope's top",
                 table['hi'][k, g] == 0.25 and np.isnan(table['lo'][k]).all() and table['caps'][k][2] == 0.2
                 and table['grips'].max() == 0.25 and np.isnan(table['caps'][names.index('uphill')]).all())
    refused = []
    for bad in ('(defsituation s (cpg_config (amplitude mode_foot_rock 0.0)))', '(defsituation s (grip below 0))',
                '(defsituation s (speed above 1.0))', '(defsituation s (grip under 0.2))'):
        try:
            situation.text(bad)
            refused.append('')
        except SyntaxError as error:
            refused.append(str(error))
    report.check('one without a bound, of 0, on no reading of hers or neither below nor above is refused',
                 all(len(r) > 12 for r in refused), ' | '.join(r[:50] for r in refused))


def test_her_readings_place_her(report):
    """Her readings off what she senses: level on her law's grip, upright, borne and touched by
    nothing, she is in no situation; at a grip of 0.1 on the slippery floor, at 0.225 half in; up a
    10 deg ramp uphill and down it downhill; half her weight on a seat seated, lying on it lying and
    not seated; 20 N on her torso held; an edge 3 cm up stepping."""
    from machine import edge, modal, situation
    from machine.wbc import MU
    table = situation.text()
    y, fr, a = np.zeros(modal.SENSED), np.zeros(modal.FRAME), np.zeros(modal.ASK)
    y[modal.QUAT] = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
    fr[modal.ROT] = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]], float).ravel()
    a[modal.STANCE] = 1.0
    touched, weight, own = np.zeros((len(edge.SKIN), 3)), 540.0, np.full(2, MU)
    r0 = situation.readings(y, fr, a, own, touched, weight)
    into = situation.read(table, r0)
    report.check('level, upright, on her own grip, borne and touched by nothing she is in no situation: %s' % r0.round(3),
                 (into == 0.0).all() and situation.told(table, into) == '' and np.allclose(r0, (MU, 0.0, 0.0, 1.0, 0.0, 0.0)))

    def told(**put):
        r = r0.copy()
        for name, v in put.items():
            r[situation.READINGS.index(name)] = v
        deep = situation.read(table, r)
        return situation.told(table, deep), deep
    report.check('at a grip of 0.1 she is on the slippery floor, at 0.225 half in',
                 told(grip=0.1)[0] == 'slippery' and abs(told(grip=0.225)[1][table['names'].index('slippery')] - 0.5) < 1e-9)
    fr[modal.GROUND] = -math.radians(10.0)
    up = situation.readings(y, fr, a, own, touched, weight)
    fr[modal.GROUND] = math.radians(10.0)
    down = situation.readings(y, fr, a, own, touched, weight)
    fr[modal.GROUND] = 0.0
    report.check('up a 10 deg ramp she is uphill, down it downhill: rise %.3f a metre' % up[1],
                 situation.told(table, situation.read(table, up)) == 'uphill'
                 and situation.told(table, situation.read(table, down)) == 'downhill'
                 and abs(up[1] - math.tan(math.radians(10.0))) < 1e-9)
    report.check('half her weight on a seat seated, lying on it lying and not seated',
                 told(borne=0.6)[0] == 'seated' and told(borne=0.6, upright=0.1)[0] == 'lying'
                 and told(borne=0.6, upright=0.1)[1][table['names'].index('seated')] == 0.0)
    touched[edge.SKIN.index('torso')] = (0.0, 0.0, 20.0)
    pushed = situation.readings(y, fr, a, own, touched, weight)
    report.check('20 N on her torso: held, pushed %.3f of her weight' % pushed[4],
                 situation.told(table, situation.read(table, pushed)) == 'held' and abs(pushed[4] - 20.0 / weight) < 1e-12)
    a[modal.RAISE] = 0.03
    edged = situation.readings(y, fr, a, own, 0.0 * touched, weight)
    report.check('an edge 3 cm up: stepping', situation.told(table, situation.read(table, edged)) == 'stepping')


def _entered(name, wobble):
    """(entered s, told s, fell s) of scene `name` laid turned `wobble` rad: when the world had her
    in it by its own contacts, when her law named it."""
    from machine import edge, modal, physics, situation, worker
    from tools.sim import scenarios
    body, share, said = SCENES[name]
    scene, plan, _seconds, _goal, _aim = scenarios.SCENARIOS[name]
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    s = worker.session()
    scenarios.placed(s, scene)
    world = s['g']['world']
    m, d = world.model, world.data
    q, c, sn = d.qpos[3:7].copy(), math.cos(wobble / 2.0), math.sin(wobble / 2.0)
    d.qpos[3:7] = (q[0] * c - q[2] * sn, q[1] * c + q[3] * sn, q[2] * c + q[0] * sn, q[3] * c - q[1] * sn)
    world._mj.mj_forward(m, d)
    hers = np.array([edge._rooted(m, b, world.pelvis) for b in range(m.nbody)], int)
    kinds, force, weight = {m.body(body).id: body}, np.zeros(6), float(s['g']['fr'][modal.WEIGHT])
    entered = told = None
    k, plan = 0, sorted(plan, key=lambda cmd: cmd[0])
    while (s['g']['t'] < (20.0 if entered is None else entered + TOLD_S + 0.2) and s['epoch'] == 0
           and s['fallen_at'] is None):
        while k < len(plan) and s['g']['t'] >= plan[k][0]:
            worker.apply(s, plan[k][1])
            k += 1
        worker.stepped(s)
        g = s['g']
        if entered is None and scenarios.borne(world, hers, kinds, force)[1].get(body, 0.0) > share * weight:
            entered = round(g['t'], 3)
        if entered is not None and told is None and situation.told(g['situations'], g['into']) == said:
            told = round(g['t'], 3)
    return entered, told, s['fallen_at']


def _situation(name):
    """A test of scene `name` entered unannounced: her law tells it within TOLD_S at HELD of WOBBLES."""
    def test(report):
        got = [_entered(name, w) for w in WOBBLES]
        within = [e is not None and t is not None and t - e <= TOLD_S for e, t, _f in got]
        report.check('%s entered unannounced, %s told within %.0f s at %d of %d lays' % (
            name, SCENES[name][2], TOLD_S, sum(within), len(WOBBLES)), sum(within) >= HELD,
            ', '.join('%s -> %s%s' % (e, t, '' if f is None else ' fell %.2f' % f) for e, t, f in got))
    test.__name__ = 'test_%s_told' % name
    test.__doc__ = _situation.__doc__
    return test


ROSTER = [test_her_text_names_her_situations, test_her_readings_place_her] + [_situation(n) for n in SCENES]


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
