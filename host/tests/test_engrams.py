#!/usr/bin/env python3
"""Bit-exact pre-dock checkpoints, live motor engram hot-swaps, and parallel Monte Carlo combinatorics."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_checkpoint_bit_exact_across_modes(report):
    """Restoring a checkpoint reproduces identical trajectories to the bit while standing and moving."""
    import numpy as np
    from machine.process import checkpoint, worker
    from machine.world import physics

    prev_room, prev_sight = physics.ROOM, physics.SIGHT
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    try:
        s = worker.session()
        g = s['g']
        worker.apply(s, {'scenario': 'chair_behind'})
        for _ in range(300):
            worker.stepped(s)
        worker.apply(s, {'say': 'sit_chair'})
        for _ in range(4200):
            worker.stepped(s)

        snap = checkpoint.capture(s)
        for _ in range(250):
            worker.stepped(s)
        q_a = g['world'].data.qpos.copy()
        v_a = g['world'].data.qvel.copy()
        st_a = g['st'].copy()
        out_a = g['out'].copy()
        sense_a = g['sense'].copy()
        eso_a = g['eso'].copy()
        played_a = None if g.get('played') is None else g['played']['cmd'].copy()
        t_a, eff_a = g['t'], g['world'].effort

        checkpoint.restore(s, snap)
        for _ in range(250):
            worker.stepped(s)
        q_b = g['world'].data.qpos.copy()
        v_b = g['world'].data.qvel.copy()
        st_b = g['st'].copy()
        out_b = g['out'].copy()
        sense_b = g['sense'].copy()
        eso_b = g['eso'].copy()
        played_b = None if g.get('played') is None else g['played']['cmd'].copy()
        t_b, eff_b = g['t'], g['world'].effort

        exact = (np.array_equal(q_a, q_b) and np.array_equal(v_a, v_b)
                 and np.array_equal(st_a, st_b) and np.array_equal(out_a, out_b)
                 and np.array_equal(sense_a, sense_b) and np.array_equal(eso_a, eso_b)
                 and t_a == t_b and eff_a == eff_b
                 and played_a is not None and played_b is not None and np.array_equal(played_a, played_b))
        report.check('mid-move checkpoint restore reproduces 250 steps bit-exact', exact,
                     'dq=%.2e dv=%.2e dst=%.2e cmd=%.2e' % (
                         float(np.max(np.abs(q_a - q_b))), float(np.max(np.abs(v_a - v_b))),
                         float(np.max(np.abs(st_a - st_b))),
                         0.0 if played_a is None or played_b is None else float(np.max(np.abs(played_a - played_b)))))
    finally:
        physics.ROOM, physics.SIGHT = prev_room, prev_sight


def test_predock_branch_and_live_hotswap(report):
    """Pre-dock checkpoint branches identical starts across C engrams and hot-swaps mid-move."""
    import numpy as np
    from machine.process import checkpoint, engrams, sprung, worker
    from machine.world import physics
    from tools.cores import sensed as c_sensed

    prev_room, prev_sight = physics.ROOM, physics.SIGHT
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    try:
        s = worker.session()
        g = s['g']
        worker.apply(s, {'scenario': 'chair_behind'})
        snap, matched = checkpoint.advance_until(
            s, lambda sess: sess['passes'] >= 400 and sprung.doing(sess['g']) == 'rest', 2.0)
        report.check('advance_until captured settled pre-dock checkpoint', matched,
                     'passes=%d doing=%s' % (s['passes'], sprung.doing(g)))

        traces = {}
        c_backed = True
        for name in ('chair_mount_stock', 'chair_mount_calm'):
            checkpoint.restore(s, snap)
            engrams.swap(g, name, s=s)
            slot = g['bank']['moves'][('chair_mount', '')]
            c_backed = c_backed and isinstance(slot['move'], c_sensed.Move)
            worker.apply(s, {'say': 'sit_chair'})
            for _ in range(4600):
                worker.stepped(s)
                if s['fallen_at'] is not None:
                    break
            traces[name] = (s['fallen_at'] is None, g['world'].data.qpos[1:3].copy())
            engrams.unload_all(g, s=s)

        diverged = float(np.linalg.norm(traces['chair_mount_stock'][1] - traces['chair_mount_calm'][1]))
        report.check('C-backed engram branches stay upright and produce distinct kinematics',
                     c_backed and all(ok for ok, _p in traces.values()) and diverged > 1e-4,
                     'c_backed=%s diverged=%.4f' % (c_backed, diverged))

        # Live hot-swap mid-execution from chair_mount_calm to chair_mount_deep_pull and unload back to stock.
        checkpoint.restore(s, snap)
        engrams.swap(g, 'chair_mount_calm', s=s)
        worker.apply(s, {'say': 'sit_chair'})
        for _ in range(4250):
            worker.stepped(s)
        was_active = isinstance(g.get('played'), dict) and g['played'].get('name') == 'chair_mount'
        info = engrams.swap(g, 'chair_mount_deep_pull', s=s)
        for _ in range(300):
            worker.stepped(s)
        man_ok = float(g['told'].sum()) <= 1.05
        engrams.unload_all(g, s=s)
        stock_restored = (not engrams.active(g)
                          and g['bank']['moves'][('chair_mount', '')]['move'].name == 'chair-mount')
        report.check('live mid-move engram hot-swap and unload keep C0 continuity and restore stock',
                     was_active and info['swapped_live'] and info['pose_jump_deg'] < 1e-9
                     and man_ok and s['fallen_at'] is None and stock_restored,
                     'active=%s live=%s jump=%.2e man=%.3f fallen=%s restored=%s' % (
                         was_active, info['swapped_live'], info['pose_jump_deg'],
                         float(g['told'].sum()), s['fallen_at'], stock_restored))
    finally:
        physics.ROOM, physics.SIGHT = prev_room, prev_sight


def test_engram_catalog_and_stunt_integrity(report):
    """All prepared motor engrams compile cleanly and reference valid human mocap stunts."""
    from machine.process import engrams, worker
    from machine.world import physics
    from tools.cores import sensed as c_sensed
    from tools.sim import stunts

    ref = stunts.reference()
    bad_ref = []
    bad_compile = []
    for name, eng in engrams.catalog().items():
        if eng.stunt_ref is not None and eng.stunt_ref not in ref:
            bad_ref.append((name, eng.stunt_ref))
        try:
            payload = engrams.compile_engram(eng)
            if isinstance(payload, c_sensed.Move) and len(payload.tables.names) > c_sensed.P:
                bad_compile.append((name, 'phases=%d' % len(payload.tables.names)))
        except Exception as exc:
            bad_compile.append((name, repr(exc)))

    prev_room, prev_sight = physics.ROOM, physics.SIGHT
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    try:
        s = worker.session()
        g = s['g']
        for name in engrams.catalog():
            engrams.load(g, name)
            engrams.swap(g, name, s=s)
        loaded_n = len(engrams.active(g))
        engrams.unload_all(g, s=s)
        clean_after = len(engrams.active(g)) == 0
    finally:
        physics.ROOM, physics.SIGHT = prev_room, prev_sight

    report.check('entire engram catalog compiles, maps to stunts/, and swaps/unloads cleanly',
                 not bad_ref and not bad_compile and loaded_n >= 4 and clean_after,
                 'bad_ref=%s bad_compile=%s slots=%d clean=%s' % (bad_ref, bad_compile, loaded_n, clean_after))


def test_parallel_mc_combinatorics_runner(report):
    """Parallel Monte Carlo runner evaluates checkpointed combinations across worker processes."""
    from tools.sim import engram_mc

    got = engram_mc.run_sweep(scenes=('crouch_dock', 'turn_dock'), quick=True, samples=1, workers=2)
    rows = got['summary']
    valid_metrics = all(
        row['total'] == 1
        and row['max_handover_deg'] <= 1.0
        and not math.isnan(row['max_jerk'])
        and not math.isnan(row['mean_stunt_far'])
        for row in rows
    )
    report.check('parallel Monte Carlo runner produces valid combinatorial matrix across scenes',
                 len(rows) >= 4 and valid_metrics and any(row['pass_rate'] > 0.0 for row in rows),
                 'rows=%d passed=%d' % (len(rows), sum(r['passed'] for r in rows)))


ROSTER = (
    test_checkpoint_bit_exact_across_modes,
    test_predock_branch_and_live_hotswap,
    test_engram_catalog_and_stunt_integrity,
    test_parallel_mc_combinatorics_runner,
)


def main(argv=None):
    """Run all motor engram tests or those named on the CLI."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
