#!/usr/bin/env python3
"""LLM-generated core and engram compilation, live hot-swap, and MuJoCo checkpoint/pose reload."""
import json
import math
import os
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report


def test_compile_and_hotswap_c_python_and_lisp_units(report):
    """LLM-generated C11 cores, traced Python cores, and Lisp engrams compile, validate, swap, and unload."""
    import numpy as np
    from machine.errors import MachineError
    from machine.process import hotswap, worker
    from machine.world import physics

    prev = (physics.ROOM, physics.CAMERA, physics.SIGHT)
    physics.ROOM = physics.CAMERA = physics.SIGHT = 0.0
    try:
        s = worker.session()
        g = s['g']
        for _ in range(120):
            worker.stepped(s)
        snap = hotswap.save_state(s, 'base')
        report.check('save_state captures baseline session state', snap['name'] == 'base' and 'base' in hotswap.list_states(s),
                     str(snap))

        # 1. C11 microkernel core targeting `sloped` (returns (2,) float64 array) + in-place `released` core.
        c_sloped = {
            'type': 'core', 'kind': 'c', 'name': 'llm_sloped_boost', 'hook': 'sloped',
            'entry': 'hs_sloped_boost', 'args': ['f64*', 'f64*'], 'returns': {'shape': [2], 'dtype': 'float64'},
            'source': (
                'void hs_sloped_boost(const double * restrict grip, const double * restrict grips, double * restrict out) {\n'
                '    (void)grips;\n'
                '    out[0] = grip[0] * 0.92;\n'
                '    out[1] = grip[1] * 0.92;\n'
                '}\n'
            ),
        }
        # 2. Python/NumPy core targeting `sured`, compiled to C11 via `bodied` and held bit-for-bit.
        py_sured = {
            'type': 'core', 'kind': 'python', 'name': 'llm_sured_calm', 'hook': 'sured', 'entry': 'sured',
            'source': (
                'from machine.law import modal\n'
                'def sured(imp, posture, dq, brace, sw, bw, d_ax, d_lat):\n'
                '    imp[modal.BEND] *= sw\n'
                '    imp[modal.SWUNG] *= sw\n'
                '    imp[modal.PREFOLD] *= sw\n'
                '    imp[modal.SOLE_ACC] *= sw\n'
                '    imp[modal.FOLD] *= sw\n'
                '    imp[modal.FOLD_RATE] *= sw\n'
                '    imp[modal.STRIDES] *= sw\n'
                '    imp[modal.COM_ACC] = np.stack([sw * imp[modal.COM_ACC][0], sw * imp[modal.COM_ACC][1] - bw * 2.5,\n'
                '                                   sw * imp[modal.COM_ACC][2]])\n'
                '    return sw * (0.98 * posture + 0.92 * dq) + bw * (np.stack([1.0, d_ax, d_lat]) @ brace)\n'
            ),
        }
        # 3. Lisp `ortho` wire engram and Lisp `defmove` C sensed engram.
        lisp_wire = {
            'type': 'engram', 'name': 'llm_lisp_sway', 'slot': 'sway', 'kind': 'wire', 'task': 'sway',
            'spec': {
                'lisp': (
                    '(stream_inject (sequence_id 202) (blend_duration 350ms)'
                    ' (latent_targets (mode_axial_compression 0.03) (mode_lateral_shift 0.02))'
                    ' (cpg_config (enable true) (amplitude mode_lateral_shift 0.14)))'
                ),
            },
        }
        lisp_move = {
            'type': 'engram', 'name': 'llm_lisp_mount', 'slot': 'chair_mount', 'kind': 'sensed', 'task': 'chair_dock',
            'spec': {
                'lisp': (
                    '(defmove llm-chair-mount []\n'
                    '  (:constants (:tol 2.0 :still 0.08)\n'
                    '   :holds true\n'
                    '   :phases [(:name :prep :ends :reached :next :settle\n'
                    '             :goals (:hip [-65.0 90.0] :knee [72.0 95.0] :ankle [-18.0 70.0] :shoulder [12.0 80.0]))\n'
                    '            (:name :settle :ends :stilled :next :done\n'
                    '             :goals (:hip [-86.0 75.0] :knee [88.0 80.0] :ankle [-8.0 60.0]))\n'
                    '            (:name :done :ends :reached :next :done)]))\n'
                ),
            },
        }

        res = hotswap.swap_batch(g, [c_sloped, py_sured, lisp_wire, lisp_move], s=s)
        act = hotswap.active(g)
        report.check('batch compiles C11 core, bodied Python->C11 core, and Lisp wire/defmove engrams',
                     len(res) == 4 and res[0]['compiled_c'] and res[1]['compiled_c']
                     and act['cores'] == {'sloped': 'llm_sloped_boost', 'sured': 'llm_sured_calm'}
                     and act['engrams'] == {'sway': 'llm_lisp_sway', 'chair_mount': 'llm_lisp_mount'},
                     str(act))

        worker.apply(s, {'walk': True})
        for _ in range(160):
            worker.stepped(s)
        q_mod = g['world'].data.qpos.copy()
        walked_ok = s['fallen_at'] is None

        # Reject invalid C syntax and NaN-producing cores without disturbing active state.
        bad_c = {'type': 'core', 'kind': 'c', 'name': 'broken_c', 'hook': 'sloped',
                 'source': 'void broken_c(double *x) { SYNTAX_ERROR; }'}
        nan_py = {'type': 'core', 'kind': 'python', 'name': 'nan_core', 'hook': 'outside',
                  'compile_c': False, 'entry': 'outside',
                  'source': 'def outside(grip, grips):\n    return float("nan")\n'}
        c_rejected = nan_rejected = False
        try:
            hotswap.swap(g, bad_c, s=s)
        except MachineError:
            c_rejected = True
        try:
            hotswap.swap(g, nan_py, s=s)
        except MachineError:
            nan_rejected = True
        report.check('compiler errors and NaN dry-runs are rejected cleanly without mutating live hooks',
                     c_rejected and nan_rejected and hotswap.active(g) == act,
                     'c_rejected=%s nan_rejected=%s' % (c_rejected, nan_rejected))

        # Unload all and verify bit-exact trajectory replay against clean stock from `base`.
        hotswap.unload_all(g, s=s)
        hotswap.reload_state(s, 'base')
        worker.apply(s, {'walk': True})
        for _ in range(160):
            worker.stepped(s)
        q_stock_a = g['world'].data.qpos.copy()

        hotswap.reload_state(s, 'base')
        worker.apply(s, {'walk': True})
        for _ in range(160):
            worker.stepped(s)
        q_stock_b = g['world'].data.qpos.copy()

        diff_mod = float(np.linalg.norm(q_mod - q_stock_a))
        report.check('hot-swapped cores alter live dynamics and unload restores bit-exact stock trajectory',
                     walked_ok and hotswap.active(g) == {'cores': {}, 'engrams': {}}
                     and diff_mod > 1e-6 and np.array_equal(q_stock_a, q_stock_b),
                     'diff_mod=%.3e exact_stock=%s' % (diff_mod, np.array_equal(q_stock_a, q_stock_b)))
    finally:
        physics.ROOM, physics.CAMERA, physics.SIGHT = prev


def test_mujoco_pose_reload_disk_checkpoint_and_parallel_iterate(report):
    """Robot reloads to saved states, disk `.pkl` files, and explicit `(where, yaw, qpos)` poses; iterates in parallel."""
    import numpy as np
    from machine.body import figure
    from machine.process import hotswap, worker
    from machine.world import physics
    from tools.sim import hotswap as hotswap_cli

    prev = (physics.ROOM, physics.CAMERA, physics.SIGHT)
    physics.ROOM = physics.CAMERA = physics.SIGHT = 0.0
    try:
        s = worker.session()
        g = s['g']
        for _ in range(150):
            worker.stepped(s)

        with tempfile.TemporaryDirectory() as tmp:
            ck_path = os.path.join(tmp, 'state.pkl')
            hotswap.save_state(s, 'stand0', path=ck_path)
            for _ in range(100):
                worker.stepped(s)
            q_ref = g['world'].data.qpos.copy()

            # Reload from `.pkl` into a brand-new session and check bit-exact continuation.
            s2 = worker.session()
            hotswap.reload_state(s2, name=None, path=ck_path)
            for _ in range(100):
                worker.stepped(s2)
            q_disk = s2['g']['world'].data.qpos.copy()
            max_err = float(np.max(np.abs(q_ref - q_disk)))
            report.check('disk checkpoint `.pkl` reload reproduces trajectory across fresh session',
                         max_err < 1e-12, 'max_dq=%.3e' % max_err)

            # Reload to an explicit MuJoCo position, heading, and knee joint angle override.
            rel = hotswap.reload_state(
                s, 'stand0', where=(1.25, -0.75), yaw_deg=45.0,
                qpos={'left_knee': 28.0, 'right_knee': 28.0}, zero_vel=True)
            d = g['world'].data
            lk_deg = math.degrees(float(d.qpos[g['world'].qadr[figure.JOINTS.index('left_knee')]]))
            for _ in range(80):
                worker.stepped(s)
            report.check('reload_state places the robot at target MuJoCo where, yaw, and joint pose stably',
                         abs(rel['where'][0] - 1.25) < 1e-6 and abs(rel['where'][2] + 0.75) < 1e-6
                         and abs(math.degrees(rel['heading']) - 45.0) < 1e-4
                         and abs(lk_deg - 28.0) < 1e-4 and s['fallen_at'] is None,
                         'where=%s yaw=%.1f lk=%.1f reload_ms=%.2f' % (
                             [round(x, 3) for x in rel['where']], math.degrees(rel['heading']), lk_deg, rel['reload_ms']))

            # Parallel multi-process `iterate` from serialized checkpoint with 2 distinct C cores.
            cand_a = [{'type': 'core', 'kind': 'c', 'name': 'iter_sloped_a', 'hook': 'sloped',
                       'entry': 'iter_sloped_a', 'args': ['f64*', 'f64*'],
                       'returns': {'shape': [2], 'dtype': 'float64'},
                       'source': 'void iter_sloped_a(const double *g, const double *gs, double *o){(void)gs;o[0]=g[0];o[1]=g[1];}\n'}]
            cand_b = [{'type': 'core', 'kind': 'c', 'name': 'iter_sloped_b', 'hook': 'sloped',
                       'entry': 'iter_sloped_b', 'args': ['f64*', 'f64*'],
                       'returns': {'shape': [2], 'dtype': 'float64'},
                       'source': 'void iter_sloped_b(const double *g, const double *gs, double *o){(void)gs;o[0]=0.85*g[0];o[1]=0.85*g[1];}\n'}]
            rows = hotswap.iterate([cand_a, cand_b], path=ck_path, steps=120, jobs=2,
                                   where=(0.4, -0.3), yaw_deg=20.0, say='walk')
            cli_rc = hotswap_cli.main(['--compile', json.dumps({'cores': cand_a})])
            report.check('parallel multi-process iterate and CLI compile evaluate C core candidates cleanly',
                         cli_rc == 0 and len(rows) == 2 and all(r.get('ok') for r in rows)
                         and rows[0]['swapped'] == ['iter_sloped_a'] and rows[1]['swapped'] == ['iter_sloped_b'],
                         str(rows))

            # Verify array qpos/qvel sync, compiled modal_sensed retention, non-contiguous C writeback,
            # same-name C variants without pruning collision, and set-valued live room checkpoint pack/unpack.
            from machine import libm
            q_rot = g['world'].data.qpos.copy()
            qy = np.array([libm.cos(math.radians(15.0)), 0.0, libm.sin(math.radians(15.0)), 0.0])
            q_rot[3:7] = hotswap._quat_mul(qy, q_rot[3:7])
            qw2, qx2, qy2, qz2 = (float(v) for v in q_rot[3:7])
            want_hd = libm.arctan2(2.0 * (qw2 * qy2 + qx2 * qz2), 1.0 - 2.0 * (qx2 * qx2 + qy2 * qy2))
            hotswap.place_robot(s, qpos=q_rot, qvel=np.array([0.8, 0.0, -0.3]))
            c_kept = getattr(g['modal_sensed'], 'why', None) is None
            hd_ok = abs(float(g['heading']) - want_hd) < 1e-6
            vel_ok = float(np.max(np.abs(g['est']['vel'] - np.array([0.8, 0.0, -0.3])))) < 1e-9
            c_in = {'type': 'core', 'kind': 'c', 'name': 'hs_slice_inc', 'entry': 'hs_slice_inc',
                    'args': ['f64*', 'len:0'], 'returns': 'inplace:0',
                    'source': 'void hs_slice_inc(double *x, int n){for(int i=0;i<n;i++) x[i]+=2.0;}\n'}
            arr_nc = np.zeros(6, dtype=np.float64)
            hotswap.compile_unit(c_in)['callable'](arr_nc[::2])
            nc_ok = np.allclose(arr_nc, [2.0, 0.0, 2.0, 0.0, 2.0, 0.0])
            same_a = dict(cand_a[0], name='hs_same_variant')
            same_b = dict(cand_b[0], name='hs_same_variant')
            r_a, r_b = hotswap.compile_unit(same_a), hotswap.compile_unit(same_b)
            same_ok = os.path.isfile(r_a['lib_path']) and os.path.isfile(r_b['lib_path']) and r_a['lib_path'] != r_b['lib_path']
            g['_gaze'] = {'seen': {'bed'}, 'visited': {'chair'}}
            blob_sets = hotswap.pack_state(s, allow_apart=True)
            hotswap.unpack_state(s2, blob_sets)
            sets_ok = s2['g'].get('_gaze', {}).get('seen') == {'bed'} and s2['g'].get('eyes') == 0
            report.check('place_robot preserves C modal_sensed and syncs array pose/vel; C writeback, variants, and set checkpoints hold',
                         c_kept and hd_ok and vel_ok and nc_ok and same_ok and sets_ok,
                         'c_kept=%s hd=%s vel=%s nc=%s same=%s sets=%s' % (c_kept, hd_ok, vel_ok, nc_ok, same_ok, sets_ok))
    finally:
        physics.ROOM, physics.CAMERA, physics.SIGHT = prev


def test_live_worker_apply_and_tcp_telemetry_hotswap(report):
    """`worker.apply` and `introspect.attach` TCP handle live checkpoint save/reload and core hot-swap."""
    import threading
    from machine.process import hotswap, introspect, worker
    from machine.world import physics

    prev = (physics.ROOM, physics.CAMERA, physics.SIGHT)
    physics.ROOM = physics.CAMERA = physics.SIGHT = 0.0
    try:
        s = worker.session()
        server = introspect.attach(s, 0)
        port = int(server.server_address[1])
        stop = threading.Event()

        def spin():
            while not stop.is_set():
                worker.stepped(s)
                time.sleep(0.0005)

        loop_thread = threading.Thread(target=spin, daemon=True)
        loop_thread.start()
        try:
            r_save = hotswap.remote(port, {'checkpoint': {'action': 'save', 'name': 'live0'}})
            core_spec = {
                'type': 'core', 'kind': 'c', 'name': 'tcp_outside_core', 'hook': 'outside',
                'entry': 'tcp_outside_core', 'args': ['f64*', 'f64*'],
                'returns': {'shape': [2], 'dtype': 'float64'},
                'source': (
                    'void tcp_outside_core(const double *g, const double *gs, double *out) {\n'
                    '    (void)gs; out[0] = g[0]; out[1] = g[1];\n'
                    '}\n'
                ),
            }
            r_swap = hotswap.remote(port, {'hotswap': core_spec})
            r_rel = hotswap.remote(port, {'checkpoint': {'action': 'reload', 'name': 'live0',
                                                         'where': [-0.5, 0.6], 'yaw_deg': -30.0}})
            r_bad = hotswap.remote(port, {'hotswap': {'type': 'core', 'kind': 'c', 'name': 'bad_tcp',
                                                      'hook': 'outside', 'source': 'int nope( { '}})
        finally:
            stop.set()
            loop_thread.join(timeout=5.0)
            server.shutdown()

        # Also test direct `worker.apply` command interface.
        worker.apply(s, {'checkpoint': {'action': 'save', 'name': 'direct0'}})
        worker.apply(s, {'hotswap': core_spec})
        active_after = hotswap.active(s['g'])
        worker.apply(s, {'hotswap': {'action': 'unload', 'all': True},
                         'checkpoint': {'action': 'reload', 'name': 'direct0', 'where': [0.2, 0.3]}})
        d = s['g']['world'].data
        report.check('live TCP telemetry and worker.apply execute compile, hot-swap, MuJoCo reload, and error refusal',
                     r_save.get('ok') and r_swap.get('ok') and r_rel.get('ok')
                     and not r_bad.get('ok') and 'why' in r_bad
                     and active_after['cores'].get('outside') == 'tcp_outside_core'
                     and hotswap.active(s['g']) == {'cores': {}, 'engrams': {}}
                     and abs(float(d.qpos[0]) - 0.2) < 1e-6 and abs(float(d.qpos[2]) - 0.3) < 1e-6,
                     'tcp_swap=%s tcp_rel=%s bad_ok=%s' % (r_swap.get('ok'), r_rel.get('ok'), r_bad.get('ok')))
    finally:
        physics.ROOM, physics.CAMERA, physics.SIGHT = prev


def test_tiered_l1_l2_l3_cache_prefetch_loader(report):
    """`tools.cores.prefetch` stages the 1 kHz loop in L1, law cores in L2, and engrams/code in L3 bit-identically."""
    import ctypes
    import hashlib
    from machine.process import bank, hotswap, worker
    from machine.world import physics
    from tools.cores import prefetch

    prev = (physics.ROOM, physics.CAMERA, physics.SIGHT)
    physics.ROOM = physics.CAMERA = physics.SIGHT = 0.0
    try:
        info = prefetch.caches()
        report.check('prefetch.caches detects valid hardware L1/L2/L3 cache hierarchy',
                     32 <= info.line <= 256 and 0 < info.l1d <= info.l2 <= info.l3, str(info))

        s = worker.session(walks=True)
        g = s['g']
        loader: prefetch.Loader = g['_prefetch']
        st0 = loader.stats()
        report.check('session stages L1, L2, and L3 within hardware capacity budgets and pins C libraries',
                     isinstance(loader, prefetch.Loader)
                     and 0 < st0['l1_bytes'] <= int(info.l1d * prefetch.BUDGET_SHARE)
                     and 0 < st0['l2_bytes'] <= int(info.l2 * prefetch.BUDGET_SHARE)
                     and 0 < st0['l3_bytes'] <= int(info.l3 * prefetch.BUDGET_SHARE)
                     and st0['pinned_libs'] >= 2
                     and st0['lines_all'] >= st0['lines_hot'] >= st0['lines_l1'] > 0,
                     str(st0))

        # Activating a pre-warmed C move promotes its wbc_sensed_t table and buffers into L1; release demotes it back.
        slot = g['bank']['moves'][('chair_mount', '')]
        tb_addr = ctypes.addressof(slot['move']._laid()[1]) & ~(info.line - 1)
        l1_before = set(loader.addrs[loader.cuts[3]:loader.cuts[5]].tolist())
        bank.activate(g, {'joints': 'chair_mount'})
        l1_active = set(loader.addrs[loader.cuts[3]:loader.cuts[5]].tolist())
        bank.release(g)
        l1_released = set(loader.addrs[loader.cuts[3]:loader.cuts[5]].tolist())
        report.check('activating a C move promotes its table and buffers from L3 to L1 and release restores L3',
                     tb_addr not in l1_before and tb_addr in l1_active and tb_addr not in l1_released,
                     'before=%s active=%s released=%s' % (tb_addr in l1_before, tb_addr in l1_active, tb_addr in l1_released))

        # Hot-swapping a C11 core and Lisp engram updates prefetch status and pins the new shared library.
        c_spec = {
            'type': 'core', 'kind': 'c', 'name': 'pf_sloped_core', 'hook': 'sloped',
            'entry': 'pf_sloped_core', 'args': ['f64*', 'f64*'], 'returns': {'shape': [2], 'dtype': 'float64'},
            'source': (
                'void pf_sloped_core(const double *g, const double *gs, double *out) {\n'
                '    (void)gs; out[0] = g[0]; out[1] = g[1];\n'
                '}\n'
            ),
        }
        worker.stepped(s)
        prefetch.refresh(g)
        sloped_c = getattr(g['sloped'], 'c', None)
        sloped_addr = (int(sloped_c.words.ctypes.data) & ~(info.line - 1)) if sloped_c is not None else 0
        had_sloped = bool(sloped_addr) and sloped_addr in set(loader.addrs.tolist())
        hotswap.swap(g, c_spec, s=s)
        status = hotswap.apply_hotswap(s, {'action': 'status'})
        pf_stat = status.get('prefetch') or {}
        hotswap.unload_all(g, s=s)
        restored_sloped = (not had_sloped) or sloped_addr in set(loader.addrs.tolist())
        cdll_calls, orig_cdll = [], ctypes.CDLL
        try:
            ctypes.CDLL = lambda *a, **kw: (cdll_calls.append(a[0]), orig_cdll(*a, **kw))[1]
            for _ in range(3):
                prefetch.refresh(g)
        finally:
            ctypes.CDLL = orig_cdll
        bad_cuts = (ctypes.c_int32 * 6)(-1, 0, 0, 0, 0, 0)
        bad_lines = int(prefetch._lib().pf_prefetch_all(loader._ap, loader._lp, bad_cuts, info.line))
        report.check('hotswap status exposes live prefetch telemetry, unload_all restores stock spans, and refresh does not reload CDLLs',
                     pf_stat.get('pinned_libs', 0) >= st0['pinned_libs'] and pf_stat.get('lines_hot', 0) > 0
                     and restored_sloped and not cdll_calls and bad_lines == 0,
                     'pf=%s restored=%s cdll=%d bad=%d' % (pf_stat, restored_sloped, len(cdll_calls), bad_lines))

        # Bit-exactness: 300 passes with active prefetch + wake() match 300 passes with _prefetch disabled bit-for-bit.
        s_a = worker.session(walks=True)
        rob_a = worker.Robot(s_a)
        h_a = hashlib.sha1()
        for k in range(300):
            if k % 50 == 0:
                rob_a.wake()
            worker.stepped(s_a)
            d_a = s_a['g']['world'].data
            for arr in (d_a.qpos, d_a.qvel, d_a.ctrl):
                h_a.update(arr.tobytes())

        s_b = worker.session(walks=True)
        s_b['g']['_prefetch'] = None
        h_b = hashlib.sha1()
        for _ in range(300):
            worker.stepped(s_b)
            d_b = s_b['g']['world'].data
            for arr in (d_b.qpos, d_b.qvel, d_b.ctrl):
                h_b.update(arr.tobytes())

        report.check('L1/L2/L3 prefetch loader and slice wakeups preserve bit-exact simulation trajectories',
                     h_a.hexdigest() == h_b.hexdigest(),
                     '%s vs %s' % (h_a.hexdigest()[:16], h_b.hexdigest()[:16]))
    finally:
        physics.ROOM, physics.CAMERA, physics.SIGHT = prev


ROSTER = (
    test_compile_and_hotswap_c_python_and_lisp_units,
    test_mujoco_pose_reload_disk_checkpoint_and_parallel_iterate,
    test_live_worker_apply_and_tcp_telemetry_hotswap,
    test_tiered_l1_l2_l3_cache_prefetch_loader,
)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (`tools.dev.focus`)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
