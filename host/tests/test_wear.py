#!/usr/bin/env python3
"""Her identification on-line, under a load (`machine.wear`): on her walk a load put in her hand read
with no alarm; a knee's friction doubled and a hip's torque an amp down a fifth named and nothing
else, the load still read; on the boxed stand a box's play doubled named; the hours run and the
service left in what she tells."""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report
from machine import mjcf, physics

#: Her walk's phases, s: as built, carrying LOAD_KG in her left hand, then a knee's friction
#: FRICTION_X and a hip's torque an amp TORQUE_X of themselves besides; a load read within this
#: of itself; the stand's campaign, s, with a knee's play PLAY_X.
NOMINAL_S, LOADED_S, WORN_S = 10.0, 12.0, 14.0
LOAD_KG, WITHIN, FRICTION_X, TORQUE_X = 0.5, 0.3, 2.0, 0.8
CAMPAIGN_S, PLAY_X = 24.0, 2.0
#: The jeans' hems' mass through the walk, kg: next to none. Swinging on her shanks they read her
#: knees' torque an amp 12-21 % high and their friction 0.5-0.8 N m low, held at rest in her law's
#: model (identification.md); 0.12 as she wears them.
HEM_KG = 1e-3


def _laid():
    """Her session walking at 0.69 m/s from 1 s on the gym's floor, her hems HEM_KG, and the filter
    on her build."""
    from machine import wbc, wear, worker
    physics.ROOM, physics.SIGHT, mjcf.STAND = 0.0, 0.0, 0.0
    was, mjcf.HEM_KG = mjcf.HEM_KG, HEM_KG
    try:
        s = worker.session()
        w = wear.laid(wbc.Body(identified=False))
    finally:
        mjcf.STAND, mjcf.HEM_KG = 1.0, was
    return s, w


def _walked(s, w, until, rows, spent):
    """Her walk on to `until` s, each window of her rows through the filter; `rows` [ys, taus] her
    edge arrays and torques so far - a pass reads her edge at its start and drives her over it,
    ys[k] step k's start and taus[k] its torque -, `spent` the filter's seconds appended."""
    from machine import wear, worker
    g, world = s['g'], s['g']['world']
    ys, taus = rows
    told = len(ys) > 0
    n = int(round(wear.WINDOW_S / physics.STEP_S))
    while g['t'] < until and s['fallen_at'] is None:
        if g['t'] >= 1.0 and not told:
            worker.apply(s, {'twist': (0.69, 0.0, 0.0)})
            told = True
        worker.stepped(s)
        ys.append(g['sense'].copy())
        taus.append(world.motor.copy())
        if len(taus) % n == 1 and len(taus) > n:
            t0 = time.perf_counter()
            got = wear.window(w, ys[-n - 1:], taus[-n - 1:-1], physics.STEP_S)
            wear.step(w, got)
            spent.append(time.perf_counter() - t0)


def _changed(world, change):
    """`change` on the world's model, her state kept."""
    m, d = world.model, world.data
    qpos, qvel = d.qpos.copy(), d.qvel.copy()
    change(m)
    world._mj.mj_setConst(m, d)
    d.qpos[:], d.qvel[:] = qpos, qvel
    world._mj.mj_forward(m, d)


def test_a_load_is_read_and_her_wear_named(report):
    """Walking as built nothing is named and no load read; LOAD_KG put in her left hand is read
    within WITHIN of itself in LOADED_S with no alarm; her left knee's friction FRICTION_X and her
    left hip's torque an amp TORQUE_X besides, each is named within WORN_S and nothing else, the
    load still read; the hours run in what she tells, service due."""
    from machine import wear
    from machine.figure import JOINTS
    s, w = _laid()
    g, world = s['g'], s['g']['world']
    m = world.model
    ys, taus, spent = [], [], []
    _walked(s, w, NOMINAL_S, (ys, taus), spent)
    built = wear.told(w)
    report.check('walking as built %.0f s: nothing named, no load read, %d-%d rows read a drive a window' % (
        NOMINAL_S, min(built['read']), max(built['read'])),
                 s['fallen_at'] is None and not built['alarms'] and max(abs(v[0]) for v in built['load'].values()) < WITHIN * LOAD_KG,
                 'alarms %s; load %s; residual %.2f-%.2f N m' % (built['alarms'], {k: round(v[0], 2) for k, v in built['load'].items()},
                                                                 min(built['residual']), max(built['residual'])))
    hand = m.body('left_hand').id
    _changed(world, lambda m: m.body_mass.__setitem__(hand, m.body_mass[hand] + LOAD_KG))
    _walked(s, w, NOMINAL_S + LOADED_S, (ys, taus), spent)
    loaded = wear.told(w)
    report.check('%.1f kg in her left hand: read %.2f kg there in %.0f s, %.2f in the right, nothing named' % (
        LOAD_KG, loaded['load']['left_hand'][0], LOADED_S, loaded['load']['right_hand'][0]),
                 s['fallen_at'] is None and abs(loaded['load']['left_hand'][0] - LOAD_KG) < WITHIN * LOAD_KG
                 and abs(loaded['load']['right_hand'][0]) < WITHIN * LOAD_KG and not loaded['alarms'],
                 'alarms %s; it hangs %s m from the hand\'s origin' % (loaded['alarms'], np.round(loaded['load']['left_hand'][1], 3)))
    knee, hip = world.vadr[JOINTS.index('left_knee')], m.joint('left_hip').id
    act = int(np.flatnonzero(m.actuator_trnid[:, 0] == hip)[0])

    def worn(m):
        m.dof_frictionloss[knee] *= FRICTION_X
        m.actuator_gear[act, 0] *= TORQUE_X
    _changed(world, worn)
    _walked(s, w, NOMINAL_S + LOADED_S + WORN_S, (ys, taus), spent)
    told = wear.told(w)
    named = sorted(name for name, _v, _b in told['alarms'])
    report.check('her left knee\'s friction x%.0f and her left hip\'s torque an amp x%.1f: named in %.0f s and nothing else, the load still %.2f kg' % (
        FRICTION_X, TORQUE_X, WORN_S, told['load']['left_hand'][0]),
                 s['fallen_at'] is None and named == ['left_hip.kt', 'left_knee.f']
                 and abs(told['load']['left_hand'][0] - LOAD_KG) < WITHIN * LOAD_KG,
                 'alarms %s; the knee\'s f %+.2f N m, the hip\'s kt %+.3f; windows %.2f s each' % (
                     told['alarms'], told['drift']['left_knee']['f'], told['drift']['left_hip']['kt'], np.mean(spent)))
    report.check('%.4f hours run, service due now on %s; %.0f h left as built' % (told['hours'], told['next'], built['left_h']),
                 abs(told['hours'] - g['t'] / 3600.0) < 2 * wear.WINDOW_S / 3600.0 and told['left_h'] == 0.0
                 and told['next'] in named and abs(built['left_h'] - w['service']['hours']) < 0.1)


def test_a_box_played_out_is_named(report):
    """On the boxed stand (`physics.BOXED`), her left knee's play PLAY_X: the campaign's windows
    read it and name it, no other box's play; the wind-ups within their band."""
    from machine import campaign, wbc, wear
    from machine.figure import JOINTS
    mjcf.STAND, physics.BOXED = 1.0, 1.0
    try:
        world = physics.World()
        world.reset(campaign.pose(), where=(0.0, mjcf.STAND_M, 0.0))
        i = JOINTS.index('left_knee')
        world.boxes.gap[i] *= PLAY_X
        ys, taus, touched = campaign.excite(world, campaign.setpoints(world, CAMPAIGN_S, np.random.default_rng(1)))
    finally:
        physics.BOXED = 0.0
    w = wear.laid(wbc.Body(identified=False))
    wear.boxed(w, ys, taus, physics.STEP_S)
    told = wear.told(w)
    played = [name for name, _v, _b in told['alarms'] if name.endswith('.play')]
    report.check('her left knee\'s play x%.0f on the stand: named, %+.0f %% of its own, and no other box\'s; %d boxes read, touched %d' % (
        PLAY_X, 100 * told['boxes'].get('left_knee', (0.0, 0.0))[0], len(told['boxes']), touched),
                 played == ['left_knee.play'] and not [n for n, _v, _b in told['alarms'] if n.endswith('.wind_up')],
                 'alarms %s; wind-ups %s' % (told['alarms'], {k: round(v[1], 2) for k, v in told['boxes'].items()}))


ROSTER = (test_a_load_is_read_and_her_wear_named, test_a_box_played_out_is_named)


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
