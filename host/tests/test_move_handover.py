#!/usr/bin/env python3
"""Warm move bank, bit-exact atomic latent transitions, and bumpless mode hot-swap verification."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_warm_move_bank_and_reset_keep_slots(report):
    """Session startup pre-warms all `joints.MOVES`, C structs, and gait tables and preserves them across reset."""
    from machine.body import figure
    from machine.law import modal
    from machine.process import bank, joints, sprung, worker
    from machine.world import physics

    prev_room, prev_sight = physics.ROOM, physics.SIGHT
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    try:
        s = worker.session()
        g = s['g']
        bk = g.get('bank')
        moves = bk['moves'] if bk is not None else {}
        have = set(moves)
        want = {(name, '') for name in joints.MOVES}
        report.check('all moves pre-warmed at session start',
                     bk is not None and want <= have and g.get('bodily') is not None
                     and len(bk['tables']) >= len(s['gym']),
                     'moves=%s tables=%d' % (sorted(m[0] for m in have), len(bk['tables']) if bk is not None else 0))

        ids_before = {k: id(v['pass']) for k, v in moves.items()}
        worker.apply(s, {'joints': 'scoot'})
        slot = g['played']
        swapped_id = id(slot['pass']) if slot is not None else 0
        # Hot-swap directly from scoot to chair_mount (Move -> Move C0 handoff).
        worker.apply(s, {'joints': 'chair_mount'})
        slot2 = g['played']
        cont_err = float(max(abs(slot2['pose'] - slot['cmd']))) if slot is not None and slot2 is not None else 999.0
        joints.ended(g)
        sprung.relaid(g, where=(0.0, 0.0))
        ids_after = {k: id(v['pass']) for k, v in moves.items()}
        report.check('relaid preserves C move bank and Move->Move swap is C0 continuous',
                     ids_before == ids_after and swapped_id == ids_before[('scoot', '')]
                     and cont_err < 1e-12 and g.get('played') is None
                     and len(figure.JOINTS) == (len(slot2['cmd']) if slot2 is not None else 0),
                     'cont_err=%.2e relaid_match=%s rest=%s' % (
                         cont_err, ids_before == ids_after, g['loads'][0] + g['loads'][1] > 0.5 * g['fr'][modal.WEIGHT]))
    finally:
        physics.ROOM, physics.SIGHT = prev_room, prev_sight


def test_latent_multi_row_collision_atomic_in_c_and_python(report):
    """Colliding latent rows inside one `dt = 2 ms` tick fire at most once in both Python and C11."""
    import numpy as np
    from machine.law import latent
    from machine.texts import ortho
    from tools.cores import motion as c_motion

    c_law = c_motion.latent_laid()
    rest = np.zeros(latent.P)
    rest[latent.K.start:latent.K.start + latent.N] = 100.0
    rest[latent.D.start:latent.D.start + latent.N] = 10.0

    # Three rows spaced 0.5 ms apart that all land in a single dt=2 ms window.
    tab = np.zeros((3, latent.TABLE))
    tab[:, latent.SAID.start + latent.MANIFOLD.start:latent.SAID.start + latent.MANIFOLD.stop] = 1.0
    tab[0, latent.AT], tab[0, latent.VALS.start + latent.MANIFOLD.start] = 1.0002, 0.50
    tab[1, latent.AT], tab[1, latent.VALS.start + latent.MANIFOLD.start + 1] = 1.0007, 0.80
    tab[1, latent.AT], tab[1, latent.VALS.start + latent.MANIFOLD.start + ortho.PALMAR] = 1.0007, 0.20
    tab[2, latent.AT], tab[2, latent.VALS.start + latent.MANIFOLD.start + 1] = 1.0012, 1.00
    tab[:, latent.LERP] = 0.001
    tab[:, latent.SHAPED] = latent.SHAPES[0]

    st_py, st_c = latent.state(rest), latent.state(rest)
    out_py, out_c = np.zeros(latent.OUT), np.zeros(latent.OUT)
    latent.step(st_py, tab, 1.0015, 0.002, out_py)
    c_motion.stepped(c_law, st_c, tab, 1.0015, 0.002, out_c)
    man_py = out_py[latent.MANIFOLDED]
    man_c = out_c[latent.MANIFOLDED]
    diff = float(np.max(np.abs(out_py - out_c)))
    report.check('multi-row collision selects latest row bit-exact in Python and C11',
                 diff == 0.0 and abs(float(man_c[1]) - 1.0) < 1e-12
                 and float(man_c.sum()) <= 1.0 + 1e-12 and float(man_c[ortho.PALMAR]) == 0.0,
                 'diff=%.2e man_c=%s' % (diff, [round(float(x), 4) for x in man_c]))


def test_bumpless_live_hotswaps_across_modes(report):
    """Live MuJoCo hot-swaps across walk/turn/stand and sit_chair/rise_chair hold manifold sum and peak qvel."""
    import numpy as np
    from machine.process import sprung, worker
    from machine.world import physics

    prev_room, prev_sight = physics.ROOM, physics.SIGHT
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    try:
        s = worker.session()
        g = s['g']
        for _ in range(400):
            worker.stepped(s)
        man_max = 0.0
        qvel_max = 0.0
        for cmd, secs in (({'say': 'walk'}, 1.2), ({'say': 'walk_back'}, 1.0),
                          ({'say': 'turn_left'}, 0.9), ({'say': 'stand'}, 1.4)):
            worker.apply(s, cmd)
            steps_n = int(round(secs * sprung.RATE_HZ))
            for _ in range(steps_n):
                worker.stepped(s)
                man_max = max(man_max, float(g['told'].sum()))
                qvel_max = max(qvel_max, float(np.abs(g['world'].data.qvel[6:]).max()))
                if s['fallen_at'] is not None:
                    break

        worker.apply(s, {'scenario': 'chair_behind'})
        for _ in range(400):
            worker.stepped(s)
        for cmd, secs in (({'say': 'sit_chair'}, 7.0), ({'say': 'rise_chair'}, 8.0)):
            worker.apply(s, cmd)
            for _ in range(int(round(secs * sprung.RATE_HZ))):
                worker.stepped(s)
                man_max = max(man_max, float(g['told'].sum()))
                qvel_max = max(qvel_max, float(np.abs(g['world'].data.qvel[6:]).max()))
                if s['fallen_at'] is not None:
                    break

        report.check('live hot-swaps keep balance, manifold sum <= 1.05, and bounded qvel',
                     s['fallen_at'] is None and s['epoch'] == 0 and man_max <= 1.05 and qvel_max < 15.0,
                     'fallen=%s man_max=%.3f qvel_max=%.2f' % (s['fallen_at'], man_max, qvel_max))
    finally:
        physics.ROOM, physics.SIGHT = prev_room, prev_sight


ROSTER = (test_warm_move_bank_and_reset_keep_slots,
          test_latent_multi_row_collision_atomic_in_c_and_python,
          test_bumpless_live_hotswaps_across_modes)


def main(argv=None):
    """Run all move handover tests or those named on the CLI."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
