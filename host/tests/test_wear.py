#!/usr/bin/env python3
"""Her identification on-line, under a load (`machine.wear`): on her walk with her jeans on, a load
put in her hand read with no alarm and laid into her law, the filter in her worker and on the page;
a knee's friction doubled, a hip's torque an amp down a fifth and a slipped encoder zero named and
nothing else, the load still read; on the boxed stand a box's play doubled and a slipped encoder
zero named; standing on her insoles, a load on her back read and laid into her law; the hours run
and the service left in what she tells."""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report
from machine import mjcf, physics

#: Her walk's phases, s: as built, carrying LOAD_KG in her left hand, then a knee's friction
#: FRICTION_X, a hip's torque an amp TORQUE_X of themselves and an elbow's encoder slipped ZERO_RAD
#: besides; a load read within this of itself; the stand's campaign, s, with a knee's play PLAY_X;
#: her back's load, kg, read standing on her insoles.
NOMINAL_S, LOADED_S, WORN_S = 10.0, 12.0, 14.0
LOAD_KG, WITHIN, FRICTION_X, TORQUE_X, ZERO_RAD = 0.5, 0.3, 2.0, 0.8, 0.02
CAMPAIGN_S, PLAY_X = 24.0, 2.0
TORSO_KG, SWAY_S = 1.5, 6.0


def _laid():
    """Her session on the gym's floor with her jeans on (`mjcf.HEM_KG`), and the filter in her
    worker (`s['wear']`)."""
    from machine import worker
    physics.ROOM, physics.SIGHT, mjcf.STAND = 0.0, 0.0, 0.0
    try:
        s = worker.session()
    finally:
        mjcf.STAND = 1.0
    return s, s['wear']


def _walked(s, until, walked=True):
    """Her worker stepped on to `until` s, her walk at 0.69 m/s from 1 s when `walked`."""
    from machine import worker
    g = s['g']
    told = g['t'] >= 1.0
    while g['t'] < until and s['fallen_at'] is None:
        if walked and g['t'] >= 1.0 and not told:
            worker.apply(s, {'twist': (0.69, 0.0, 0.0)})
            told = True
        worker.stepped(s)


def _changed(world, change):
    """`change` on the world's model, her state kept."""
    m, d = world.model, world.data
    qpos, qvel = d.qpos.copy(), d.qvel.copy()
    change(m)
    world._mj.mj_setConst(m, d)
    d.qpos[:], d.qvel[:] = qpos, qvel
    world._mj.mj_forward(m, d)


def test_a_load_is_read_and_her_wear_named(report):
    """Walking as built with her jeans on (`mjcf.HEM_KG`) nothing is named and no load read; LOAD_KG
    put in her left hand is read within WITHIN of itself in LOADED_S with no alarm and laid into her
    law (`g['body']`, `g['fr']`), the worker's state and the page's SERVICE box showing it; her left
    knee's friction FRICTION_X, her left hip's torque an amp TORQUE_X and her right elbow's encoder
    zero ZERO_RAD besides, each is named within WORN_S and nothing else, the load still read; the
    hours run in what she tells, service due."""
    from machine import modal, worker
    from machine.figure import JOINTS
    from terminal.views import show_humanoid
    s, w = _laid()
    g, world = s['g'], s['g']['world']
    m = world.model
    hand = m.body('left_hand').id
    hand0 = float(g['body'].m.body_mass[hand])
    weight0 = float(g['fr'][modal.WEIGHT])
    _walked(s, NOMINAL_S)
    built = worker.told(s)['wear']
    report.check('walking as built %.0f s (hems %.2f kg): nothing named, no load read, %d-%d rows read a drive a window' % (
        NOMINAL_S, mjcf.HEM_KG, min(built['read']), max(built['read'])),
                 s['fallen_at'] is None and not built['alarms'] and max(abs(v[0]) for v in built['load'].values()) < WITHIN * LOAD_KG,
                 'alarms %s; load %s; residual %.2f-%.2f N m' % (built['alarms'], {k: round(v[0], 2) for k, v in built['load'].items()},
                                                                 min(built['residual']), max(built['residual'])))
    _changed(world, lambda m: m.body_mass.__setitem__(hand, m.body_mass[hand] + LOAD_KG))
    _walked(s, NOMINAL_S + LOADED_S)
    now = worker.told(s)
    loaded = now['wear']
    hand_read = float(loaded['load']['left_hand'][0])
    law_dm = float(g['body'].m.body_mass[hand]) - hand0
    law_dw = (float(g['fr'][modal.WEIGHT]) - weight0) / modal.G
    boxes = show_humanoid.boxes({'recording': None, 'recorded': None}, now, 'cpu')
    shown = '\n'.join('%s\n%s' % (b.title, b.renderable.plain) for b in boxes)
    report.check('%.1f kg in her left hand: read %.2f kg in %.0f s, %.2f in the right, nothing named; %.2f kg laid into her law and on the page' % (
        LOAD_KG, hand_read, LOADED_S, loaded['load']['right_hand'][0], law_dm),
                 s['fallen_at'] is None and abs(hand_read - LOAD_KG) < WITHIN * LOAD_KG
                 and abs(loaded['load']['right_hand'][0]) < WITHIN * LOAD_KG and not loaded['alarms']
                 and abs(law_dm - hand_read) < 1e-6 and abs(law_dw - hand_read) < 0.1
                 and 'SERVICE' in shown and ('%+.2f kg L' % hand_read) in shown,
                 'alarms %s; law dm %.3f, dw %.3f; hangs %s m' % (
                     loaded['alarms'], law_dm, law_dw, np.round(loaded['load']['left_hand'][1], 3)))
    knee, hip = world.vadr[JOINTS.index('left_knee')], m.joint('left_hip').id
    elbow = JOINTS.index('right_elbow')
    act = int(np.flatnonzero(m.actuator_trnid[:, 0] == hip)[0])

    def worn(m):
        m.dof_frictionloss[knee] *= FRICTION_X
        m.actuator_gear[act, 0] *= TORQUE_X
    _changed(world, worn)
    world.boxes.zero[elbow] = ZERO_RAD
    _walked(s, NOMINAL_S + LOADED_S + WORN_S)
    told = worker.told(s)['wear']
    named = sorted(name for name, _v, _b in told['alarms'])
    report.check('her left knee\'s friction x%.0f, left hip\'s kt x%.1f and right elbow\'s zero %+.0f mrad: named in %.0f s and nothing else, the load still %.2f kg' % (
        FRICTION_X, TORQUE_X, 1e3 * ZERO_RAD, WORN_S, told['load']['left_hand'][0]),
                 s['fallen_at'] is None and named == ['left_hip.kt', 'left_knee.f', 'right_elbow.zero']
                 and abs(told['load']['left_hand'][0] - LOAD_KG) < WITHIN * LOAD_KG
                 and abs(told['drift']['right_elbow']['zero'] - ZERO_RAD) < 1e-3,
                 'alarms %s; knee f %+.2f N m, hip kt %+.3f, elbow zero %+.4f rad' % (
                     told['alarms'], told['drift']['left_knee']['f'], told['drift']['left_hip']['kt'],
                     told['drift']['right_elbow']['zero']))
    from machine import wear
    report.check('%.4f hours run, service due now on %s; %.0f h left as built' % (told['hours'], told['next'], built['left_h']),
                 abs(told['hours'] - g['t'] / 3600.0) < 2 * wear.WINDOW_S / 3600.0 and told['left_h'] == 0.0
                 and told['next'] in named and abs(built['left_h'] - w['service']['hours']) < 0.1)


def test_a_box_played_out_and_a_zero_slipped_are_named(report):
    """On the boxed stand (`physics.BOXED`), her left knee's play PLAY_X and her right hip's encoder
    zero ZERO_RAD: the campaign's windows read and name both, no other box's play or zero; the
    wind-ups within their band."""
    from machine import campaign, wbc, wear
    from machine.figure import JOINTS
    mjcf.STAND, physics.BOXED = 1.0, 1.0
    try:
        world = physics.World()
        world.reset(campaign.pose(), where=(0.0, mjcf.STAND_M, 0.0))
        world.boxes.gap[JOINTS.index('left_knee')] *= PLAY_X
        world.boxes.zero[JOINTS.index('right_hip')] = ZERO_RAD
        ys, taus, touched = campaign.excite(world, campaign.setpoints(world, CAMPAIGN_S, np.random.default_rng(1)))
    finally:
        physics.BOXED = 0.0
    w = wear.laid(wbc.Body(identified=False))
    wear.boxed(w, ys, taus, physics.STEP_S)
    told = wear.told(w)
    played = [name for name, _v, _b in told['alarms'] if name.endswith('.play')]
    slipped = [name for name, _v, _b in told['alarms'] if name.endswith('.zero')]
    zero_read = told['drift']['right_hip']['zero']
    report.check('her left knee\'s play x%.0f and right hip\'s zero %+.0f mrad on the stand: both named (zero %+.1f mrad), no other box\'s; %d boxes read, touched %d' % (
        PLAY_X, 1e3 * ZERO_RAD, 1e3 * zero_read, len(told['boxes']), touched),
                 played == ['left_knee.play'] and slipped == ['right_hip.zero']
                 and abs(zero_read - ZERO_RAD) < 0.2 * ZERO_RAD
                 and not [n for n, _v, _b in told['alarms'] if n.endswith('.wind_up')],
                 'alarms %s; wind-ups %s' % (told['alarms'], {k: round(v[1], 2) for k, v in told['boxes'].items()}))


def test_a_load_on_her_back_is_read_off_her_insoles(report):
    """Standing on her insoles (`sway`), TORSO_KG put on her back (`torso`) is read off her insoles'
    rows (`wear.torso_load`, `wear.window`) within WITHIN of itself and laid into her law."""
    from machine import modal, worker
    s, _w = _laid()
    g, world = s['g'], s['g']['world']
    m = world.model
    torso = m.body('torso').id
    torso0 = float(g['body'].m.body_mass[torso])
    weight0 = float(g['fr'][modal.WEIGHT])
    _changed(world, lambda m: m.body_mass.__setitem__(torso, m.body_mass[torso] + TORSO_KG))
    worker.apply(s, {'say': 1})
    _walked(s, SWAY_S, walked=False)
    told = worker.told(s)['wear']
    read_kg = float(told['load']['torso'][0])
    law_dm = float(g['body'].m.body_mass[torso]) - torso0
    law_dw = (float(g['fr'][modal.WEIGHT]) - weight0) / modal.G
    report.check('%.1f kg on her back standing on her insoles: read %.2f kg off her insoles and laid into her law (%.2f kg)' % (
        TORSO_KG, read_kg, law_dm),
                 s['fallen_at'] is None and abs(read_kg - TORSO_KG) < WITHIN * TORSO_KG
                 and abs(law_dm - read_kg) < 1e-6 and abs(law_dw - read_kg) < 1e-6,
                 'read %.3f kg at %s m; law dm %.3f, dw %.3f' % (
                     read_kg, np.round(told['load']['torso'][1], 3), law_dm, law_dw))


ROSTER = (test_a_load_is_read_and_her_wear_named,
          test_a_box_played_out_and_a_zero_slipped_are_named,
          test_a_load_on_her_back_is_read_off_her_insoles)


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
