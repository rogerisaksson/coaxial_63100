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

        # Live hot-swap mid-execution past phase 0 (chair_mount_calm -> chair_mount_deep_pull -> stock).
        from machine.moves import sensed as py_sensed
        checkpoint.restore(s, snap)
        engrams.swap(g, 'chair_mount_calm', s=s)
        worker.apply(s, {'say': 'sit_chair'})
        for _ in range(4250):
            worker.stepped(s)
        while g.get('played') is not None and int(g['played']['state'][py_sensed.PHASE]) < 1 and g['t'] < 12.0:
            worker.stepped(s)
        prev_played = g.get('played')
        was_active = isinstance(prev_played, dict) and prev_played.get('name') == 'chair_mount'
        ph_before = int(prev_played['state'][py_sensed.PHASE]) if was_active else -1
        at_before = prev_played.get('at') if was_active else None
        last_before = prev_played.get('last') if was_active else None
        info = engrams.swap(g, 'chair_mount_deep_pull', s=s)
        after_played = g.get('played')
        ph_after = int(after_played['state'][py_sensed.PHASE]) if isinstance(after_played, dict) else -1
        at_kept = isinstance(after_played, dict) and after_played.get('at') == at_before
        last_kept = isinstance(after_played, dict) and after_played.get('last') == last_before
        for _ in range(300):
            worker.stepped(s)
        man_ok = float(g['told'].sum()) <= 1.05
        ph_pre_unload = int(g['played']['state'][py_sensed.PHASE]) if isinstance(g.get('played'), dict) else -1
        engrams.unload_all(g, s=s)
        ph_post_unload = int(g['played']['state'][py_sensed.PHASE]) if isinstance(g.get('played'), dict) else -1
        unload_kept = (ph_pre_unload < 0) or (ph_post_unload == ph_pre_unload)
        stock_restored = (not engrams.active(g)
                          and g['bank']['moves'][('chair_mount', '')]['move'].name == 'chair-mount')
        report.check('live mid-move engram hot-swap preserves phase, timestamps, C0 continuity, and stock unload',
                     was_active and ph_before >= 1 and ph_after == ph_before
                     and info['swapped_live'] and info['phase_preserved'] and info['pose_jump_deg'] < 1e-9
                     and at_kept and last_kept and unload_kept and man_ok
                     and s['fallen_at'] is None and stock_restored,
                     'ph=%d->%d live=%s pres=%s jump=%.2e rate_dj=%.1f at_ok=%s last_ok=%s unload_ok=%s man=%.3f' % (
                         ph_before, ph_after, info['swapped_live'], info['phase_preserved'],
                         info['pose_jump_deg'], info['rate_jump_deg_s'], at_kept, last_kept,
                         unload_kept, float(g['told'].sum())))
    finally:
        physics.ROOM, physics.SIGHT = prev_room, prev_sight


def test_checkpoint_isolation_and_closed_chain_exactness(report):
    """Checkpoint restores bank slots, vision cache, body mass, closed-chain QP state, and rejects opaque leaks."""
    import numpy as np
    from machine.process import checkpoint, engrams, worker
    from machine.world import physics

    prev_room, prev_sight = physics.ROOM, physics.SIGHT
    physics.ROOM, physics.SIGHT = 0.0, 0.0
    try:
        s = worker.session()
        g = s['g']
        worker.apply(s, {'scenario': 'chair_behind'})
        for _ in range(400):
            worker.stepped(s)
        snap = checkpoint.capture(s)
        mass0 = float(g['body'].mass)
        engrams.swap(g, 'chair_mount_calm', s=s)
        engrams.swap(g, 'turn_left_calm', s=s)
        g['vision'].boxes(g, 0.0)
        g['body'].m.body_mass[1] += 1.25
        g['body'].mass += 1.25

        checkpoint.restore(s, snap)
        bank_clean = (not engrams.active(g)
                      and g['bank']['moves'][('chair_mount', '')]['move'].name == 'chair-mount'
                      and not g['vision']._boxed
                      and abs(float(g['body'].mass) - mass0) < 1e-12)

        class _MutableProbe:
            pass

        g['_opaque_probe'] = _MutableProbe()
        rejected = False
        try:
            checkpoint.capture(s)
        except TypeError:
            rejected = True
        finally:
            g.pop('_opaque_probe', None)

        report.check('checkpoint restores bank/vision/body cleanly and rejects unhandled mutable objects',
                     bank_clean and rejected,
                     'bank_clean=%s rejected=%s' % (bank_clean, rejected))

        worker.apply(s, {'scenario': 'bed_foot_behind'})
        for _ in range(400):
            worker.stepped(s)
        worker.apply(s, {'say': 'lie_bed'})
        snap_c = None
        k_chain = g['solves'][1].args[0]
        for _ in range(12000):
            worker.stepped(s)
            if g.get('chained') and (k_chain.chain_nheld[0] > 0 or k_chain.chain_nlet > 0):
                snap_c = checkpoint.capture(s)
                break
        if snap_c is not None:
            nheld_0, nlet_0 = list(k_chain.chain_nheld), int(k_chain.chain_nlet)
            for _ in range(200):
                worker.stepped(s)
            qa, va = g['world'].data.qpos.copy(), g['world'].data.qvel.copy()
            k_chain.chain_nheld[0], k_chain.chain_nlet = 0, 1
            k_chain.chain_let[0][0] = 99
            checkpoint.restore(s, snap_c)
            qp_restored = (list(k_chain.chain_nheld) == nheld_0 and int(k_chain.chain_nlet) == nlet_0)
            for _ in range(200):
                worker.stepped(s)
            qb, vb = g['world'].data.qpos.copy(), g['world'].data.qvel.copy()
            exact_c = qp_restored and np.array_equal(qa, qb) and np.array_equal(va, vb)
            report.check('closed-chain checkpoint restores C QP active sets and steps bit-exact', exact_c,
                         'qp_restored=%s dq=%.2e dv=%.2e' % (
                             qp_restored, float(np.max(np.abs(qa - qb))), float(np.max(np.abs(va - vb)))))
        else:
            report.check('closed-chain checkpoint restores C QP active sets and steps bit-exact', False,
                         'no closed-chain state reached')
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

    cat = engrams.catalog()
    bad_bundles = [
        k for k, bundle in engrams.STUNT_BUNDLES.items()
        if k not in ref or any(b not in cat for b in bundle)
    ]
    report.check('entire engram catalog compiles, maps to stunts/, and swaps/unloads cleanly',
                 not bad_ref and not bad_compile and not bad_bundles and loaded_n >= 4 and clean_after,
                 'bad_ref=%s bad_compile=%s bad_bundles=%s slots=%d clean=%s' % (
                     bad_ref, bad_compile, bad_bundles, loaded_n, clean_after))


def test_stunt_similarity_and_single_stunt_bundle(report):
    """Anatomical stunt similarity analyzer and single-stunt parallel sweep score verified engrams."""
    from machine.process import engrams
    from tools.sim import engram_mc, stunt_similarity

    res = stunt_similarity.evaluate_stunt('crouching_down', engrams.stunt_bundle('crouching_down'))
    sim_ok = (res['met'] and res['stood'] and res['score'] >= 85.0
              and 0.0 <= res['far_sig'] <= 1.0 and not res['bottlenecks'])
    got = engram_mc.run_sweep(
        scenes=('crouching_down',), engrams_arg='crouch_down_mocap,crouch_down_paced',
        samples=1, workers=2, chunk_size=1)
    rows = got['summary']
    sweep_ok = (len(rows) == 2
                and all(not math.isnan(r['mean_score']) and not math.isnan(r['max_jerk']) for r in rows)
                and any(r['passed'] == 1 for r in rows))
    report.check('stunt similarity analyzer and single-stunt parallel bundle sweep pass cleanly',
                 sim_ok and sweep_ok,
                 'score=%.2f far=%.2f rows=%d passed=%d' % (
                     res['score'], res['far_sig'], len(rows), sum(r['passed'] for r in rows)))


def test_parallel_mc_combinatorics_runner(report):
    """Parallel Monte Carlo runner evaluates catalog and parametric variant combinations across workers."""
    from tools.sim import engram_mc

    got = engram_mc.run_sweep(scenes=('crouch_dock', 'turn_dock'), quick=True, samples=1, workers=2, explore=1)
    rows = got['summary']
    valid_metrics = all(
        row['total'] == 1
        and row['max_handover_deg'] <= 1.0
        and not math.isnan(row['max_jerk'])
        and not math.isnan(row['mean_stunt_far'])
        for row in rows
    )
    has_variant = any(any('_var0_' in c for c in row['combo']) for row in rows)
    report.check('parallel Monte Carlo runner evaluates catalog and parametric variants across scenes',
                 len(rows) >= 6 and has_variant and valid_metrics and any(row['pass_rate'] > 0.0 for row in rows),
                 'rows=%d variant=%s passed=%d' % (len(rows), has_variant, sum(r['passed'] for r in rows)))


def test_arm64_arithmetic_and_stunt_sight(report):
    """Engram tools delegate OpenBLAS pinning to arithmetic.hold() and honor --sight on stunt keys."""
    from machine.world import physics
    from tools.dev import arithmetic
    from tools.sim import engram_mc, engram_scenes, stunt_similarity

    arm_clean = arithmetic.held('aarch64') == {} and arithmetic.held('arm64') == {}
    hardcoded = []
    for mod in (engram_mc, engram_scenes, stunt_similarity):
        path = mod.__file__
        assert path is not None
        with open(path, encoding='utf-8') as f:
            if 'Nehalem' in f.read():
                hardcoded.append(os.path.basename(path))

    prev_core = os.environ.get('OPENBLAS_CORETYPE')
    prev_thr = os.environ.get('OPENBLAS_NUM_THREADS')
    try:
        os.environ['OPENBLAS_CORETYPE'] = 'CUSTOM_ARCH_TEST'
        engram_scenes.worker_init()
        kept_core = os.environ.get('OPENBLAS_CORETYPE') == 'CUSTOM_ARCH_TEST'
        set_thr = os.environ.get('OPENBLAS_NUM_THREADS') == '1'
    finally:
        if prev_core is None:
            os.environ.pop('OPENBLAS_CORETYPE', None)
        else:
            os.environ['OPENBLAS_CORETYPE'] = prev_core
        if prev_thr is None:
            os.environ.pop('OPENBLAS_NUM_THREADS', None)
        else:
            os.environ['OPENBLAS_NUM_THREADS'] = prev_thr

    report.check('engram sim tools defer OpenBLAS coretype to arithmetic.hold() without ARM64 override',
                 arm_clean and not hardcoded and kept_core and set_thr,
                 'arm_clean=%s hardcoded=%s kept_core=%s set_thr=%s' % (
                     arm_clean, hardcoded, kept_core, set_thr))

    prev_flags = (physics.ROOM, physics.CAMERA, physics.SIGHT)
    import inspect
    from machine.process import worker
    try:
        s, snap, split_t = engram_mc._setup_stunt_checkpoint('crouching_down', sight=True)
        try:
            eyes_on = s['g']['eyes'] == 1 and s['g'].get('keep') is not None
            flags_on = physics.CAMERA == 1.0 and physics.SIGHT == 1.0
            for _ in range(10):
                worker.stepped(s)
            stepped_ok = s['fallen_at'] is None and split_t > 0.0 and snap is None
        finally:
            if s['g'].get('keep') is not None:
                s['g']['keep'].close()
    finally:
        physics.ROOM, physics.CAMERA, physics.SIGHT = prev_flags
    shard_out = engram_mc._run_shard(('crouching_down', (), (0.0,), [], None, True))
    restored = (physics.ROOM, physics.CAMERA, physics.SIGHT) == prev_flags
    shard_allows_stunt = 'if sight and is_scene:' not in inspect.getsource(engram_mc._run_shard)
    sight_ok = eyes_on and flags_on and stepped_ok and shard_out == [] and restored and shard_allows_stunt
    report.check('engram_mc honors --sight on stunt keys and restores world camera flags',
                 sight_ok,
                 'eyes=%s flags=%s stepped=%s restored=%s shard=%s' % (
                     eyes_on, flags_on, stepped_ok, restored, shard_allows_stunt))


ROSTER = (
    test_checkpoint_bit_exact_across_modes,
    test_predock_branch_and_live_hotswap,
    test_checkpoint_isolation_and_closed_chain_exactness,
    test_engram_catalog_and_stunt_integrity,
    test_stunt_similarity_and_single_stunt_bundle,
    test_parallel_mc_combinatorics_runner,
    test_arm64_arithmetic_and_stunt_sight,
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
