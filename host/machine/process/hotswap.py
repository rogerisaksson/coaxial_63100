"""Live hot-swap, MuJoCo checkpoint/pose reload, parallel iteration, and IPC for cores and engrams."""
from concurrent.futures import ProcessPoolExecutor
import copy
import json
import math
import os
import pickle
import socket
import time
from typing import Any

from machine import libm
from machine.errors import MachineError
from machine.process import checkpoint, engrams
from machine.process.engram_catalog import Engram
from machine.process.hotswap_cores import (
    _CORE_CACHE, _ensure_core_bank, _validate_and_swap_core,
    compile_batch, compile_unit, normalize_specs,
)

_MP_STATE: dict[str, Any] = {}


def swap(g, unit: Any, keep: str = '', s=None) -> dict[str, Any]:
    """Compile (if needed), dry-run validate, and hot-swap `unit` (core or engram) into live `g`."""
    if isinstance(unit, str) and not unit.strip().startswith(('{', '[')) and not os.path.isfile(unit):
        if unit in _CORE_CACHE:
            return _validate_and_swap_core(g, copy.copy(_CORE_CACHE[unit]))
        res = engrams.swap(g, unit, keep=keep, s=s)
        return {'type': 'engram', **res}
    if isinstance(unit, dict) and callable(unit.get('callable')) and unit.get('type') == 'core':
        return _validate_and_swap_core(g, unit)
    if isinstance(unit, dict) and isinstance(unit.get('engram'), Engram) and 'payload' in unit:
        res = engrams.swap(g, unit['engram'], keep=keep or unit.get('keep', ''), s=s)
        return {'type': 'engram', **res}
    rec = compile_unit(unit, g=g)
    if rec['type'] == 'engram':
        res = engrams.swap(g, rec['engram'], keep=keep or rec.get('keep', ''), s=s)
        return {'type': 'engram', 'compile_ms': rec['compile_ms'], **res}
    out = _validate_and_swap_core(g, rec)
    out['compile_ms'] = rec['compile_ms']
    return out


def swap_batch(g, specs: Any, s=None) -> list[dict[str, Any]]:
    """Compile and atomically hot-swap a batch of cores and/or engrams; roll back batch on failure."""
    compiled = compile_batch(specs, g=g)
    bk = _ensure_core_bank(g)
    prev_hooks = {r['hook']: (r['hook'] in bk['stock_cores'], bk['stock_cores'].get(r['hook']),
                              bk['active_cores'].get(r['hook']), g.get(r['hook']))
                  for r in compiled if r['type'] == 'core' and r.get('hook') in g}
    applied_engrams: list[tuple[str, str, str | None]] = []
    results: list[dict[str, Any]] = []
    try:
        for rec in compiled:
            if rec['type'] == 'engram':
                slot, keep = rec['engram'].slot, rec.get('keep', '')
                applied_engrams.append((slot, keep, bk['active'].get((slot, keep))))
            results.append(swap(g, rec, s=s))
    except Exception:
        for hook, (had_stock, st_fn, prev_name, prev_fn) in prev_hooks.items():
            g[hook] = prev_fn
            if not had_stock:
                bk['stock_cores'].pop(hook, None)
            elif st_fn is not None:
                bk['stock_cores'][hook] = st_fn
            if prev_name is None:
                bk['active_cores'].pop(hook, None)
            else:
                bk['active_cores'][hook] = prev_name
        for slot, keep, prev_eng in reversed(applied_engrams):
            if prev_eng is None:
                engrams.unload(g, slot, keep=keep, s=s)
            else:
                engrams.swap(g, prev_eng, keep=keep, s=s)
        raise
    return results


def unload(g, target: str, keep: str = '', s=None):
    """Restore stock core `target` (hook name) and/or stock engram `target` (slot name)."""
    bk = _ensure_core_bank(g)
    if target in bk['stock_cores']:
        g[target] = bk['stock_cores'].pop(target)
        bk['active_cores'].pop(target, None)
    engrams.unload(g, target, keep=keep, s=s)


def unload_all(g, s=None):
    """Unload all active hot-swapped cores and engrams back to stock definitions."""
    bk = g.get('bank')
    if bk:
        for hook in list(bk.get('active_cores', ())):
            if hook in bk.get('stock_cores', {}):
                g[hook] = bk['stock_cores'].pop(hook)
            bk['active_cores'].pop(hook, None)
    engrams.unload_all(g, s=s)


def active(g) -> dict[str, dict[str, str]]:
    """Return active hot-swapped `{'cores': {hook: name}, 'engrams': {slot: name}}` on `g`."""
    bk = g.get('bank') or {}
    return {'cores': dict(bk.get('active_cores', {})), 'engrams': engrams.active(g)}


def pack_state(s, snap=None, *, allow_apart: bool = True) -> dict[str, Any]:
    """Export session `s` (or captured `snap`) into a process- and disk-picklable dict."""
    from machine.world import physics
    if snap is None:
        snap = checkpoint.capture(s, allow_apart=allow_apart)
    m = s['g']['world'].model
    world_pack = dict(snap['world'], model_geom=(
        m.body_pos.copy(), m.body_quat.copy(), m.geom_pos.copy(), m.geom_quat.copy(), m.geom_size.copy()))
    g_pack = dict(snap['g'])
    bank_snap = snap['g']['bank']
    g_pack['bank'] = {
        'slots': {k: {'arrays': v['arrays'], 'scalars': v['scalars'], 'gains': v['gains']}
                  for k, v in bank_snap['slots'].items()},
        'handover': bank_snap['handover'],
        'programs': bank_snap['programs'],
        'active': bank_snap['active'],
        'played': bank_snap['played'],
    }
    return {
        'world': world_pack, 'g': g_pack, 's': snap['s'],
        'meta': {'every': int(s['every']), 'walks': bool(s['walks']),
                 'scenario': str(s['g'].get('scenario', 'flat')),
                 'physics': (float(physics.ROOM), float(physics.CAMERA), float(physics.SIGHT))},
    }


def unpack_state(s, packed: dict[str, Any]):
    """Restore session `s` from a `pack_state` dict, rebuilding `MjData` and slot bindings."""
    import numpy as np
    from machine.process import bank
    g, world = s['g'], s['g']['world']
    scen = packed.get('meta', {}).get('scenario', 'flat')
    if scen != g.get('scenario', 'flat'):
        world.scenario(scen)
        g['scenario'] = scen
    m = world.model
    if 'model_geom' in packed['world']:
        bpos, bquat, gpos, gquat, gsize = packed['world']['model_geom']
        np.copyto(m.body_pos, bpos)
        np.copyto(m.body_quat, bquat)
        np.copyto(m.geom_pos, gpos)
        np.copyto(m.geom_quat, gquat)
        np.copyto(m.geom_size, gsize)
    world_snap = {k: v for k, v in packed['world'].items() if k != 'model_geom'}
    bk = engrams._ensure_bank(g)
    bank_pack = packed['g']['bank']
    slots = {}
    for k, sv in bank_pack['slots'].items():
        if k not in bk['moves']:
            bk['moves'][k] = bank._build_slot(g, world, k[0], k[1])
        slots[k] = {'ref': bk['moves'][k], 'arrays': sv['arrays'], 'scalars': sv['scalars'], 'gains': sv['gains']}
    g_snap = dict(packed['g'], bank={
        'slots': slots, 'handover': bank_pack['handover'], 'programs': bank_pack['programs'],
        'active': bank_pack['active'], 'stock': dict(bk.get('stock', {})),
        'active_cores': dict(bk.get('active_cores', {})), 'stock_cores': dict(bk.get('stock_cores', {})),
        'hooks': {}, 'played': bank_pack['played'],
    })
    checkpoint.restore(s, {'world': world_snap, 'g': g_snap, 's': packed['s']})
    if g.get('keep') is None:
        g['eyes'] = 0


def save_state(s, name: str = 'default', *, path: str | None = None, allow_apart: bool = True) -> dict[str, Any]:
    """Save a named in-memory checkpoint on `s` (and optionally export to `path` on disk)."""
    snap = checkpoint.capture(s, allow_apart=allow_apart)
    s.setdefault('_checkpoints', {})[str(name)] = snap
    if path is not None:
        blob = pack_state(s, snap, allow_apart=allow_apart)
        parent = os.path.dirname(os.path.abspath(path))
        if parent:
            os.makedirs(parent, exist_ok=True)
        with open(path, 'wb') as f:
            pickle.dump(blob, f, protocol=pickle.HIGHEST_PROTOCOL)
    d = s['g']['world'].data
    return {'name': str(name), 't': float(s['g']['t']), 'passes': int(s['passes']),
            'where': d.qpos[0:3].tolist(), 'path': path}


def list_states(s) -> list[str]:
    """Return sorted names of checkpoints stored in `s['_checkpoints']`."""
    return sorted((s.get('_checkpoints') or {}).keys())


def _quat_mul(a, b):
    """Hamilton product of `(w, x, y, z)` unit quaternions."""
    import numpy as np
    aw, ax, ay, az = float(a[0]), float(a[1]), float(a[2]), float(a[3])
    bw, bx, by, bz = float(b[0]), float(b[1]), float(b[2]), float(b[3])
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=float)


def place_robot(s, *, where=None, yaw: float | None = None, yaw_deg: float | None = None,
                quat=None, qpos=None, qvel=None, degrees: bool = True, zero_vel: bool = False):
    """Reposition and/or re-pose the robot in MuJoCo while keeping sensors, EKF, and drives synced."""
    import numpy as np
    from machine.body.figure import JOINTS
    from machine.law import modal
    from machine.process.sensed import sensed as proc_sensed
    from machine.senses import edge, inekf
    from machine.world import steps
    g, world = s['g'], s['g']['world']
    mj, m, d, body = world._mj, world.model, world.data, g['body']
    steps.behind(world)
    p0 = d.qpos[0:3].copy()
    h0 = float(g['heading'])
    if yaw is None and yaw_deg is not None:
        yaw = math.radians(float(yaw_deg))
    dp = np.zeros(3)
    if where is not None:
        wv = [float(x) for x in where]
        new_p = np.array([wv[0], float(p0[1]), wv[1]], float) if len(wv) == 2 else np.array(wv[:3], float)
        dp = new_p - p0
        d.qpos[0:3] = new_p
    dyaw = 0.0
    if quat is not None:
        qv = np.asarray(quat, dtype=float)
        qv = qv / max(1e-12, float(np.linalg.norm(qv)))
        d.qpos[3:7] = qv
        qw, qx, qy, qz = qv
        dyaw = libm.arctan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) - h0
    elif yaw is not None:
        dyaw = float(yaw) - h0
        qy = np.array([libm.cos(0.5 * dyaw), 0.0, libm.sin(0.5 * dyaw), 0.0])
        q_new = _quat_mul(qy, d.qpos[3:7])
        d.qpos[3:7] = q_new / max(1e-12, float(np.linalg.norm(q_new)))
        if abs(dyaw) > 1e-12:
            cd, sd = libm.cos(dyaw), libm.sin(dyaw)
            vx, vy, vz = float(d.qvel[0]), float(d.qvel[1]), float(d.qvel[2])
            d.qvel[0], d.qvel[1], d.qvel[2] = vx * cd + vz * sd, vy, -vx * sd + vz * cd
    if zero_vel:
        d.qvel[:] = 0.0
        if d.qacc_warmstart.size:
            d.qacc_warmstart[:] = 0.0
    if qpos is not None:
        if isinstance(qpos, dict):
            for jname, val in qpos.items():
                j_idx = JOINTS.index(str(jname))
                d.qpos[world.qadr[j_idx]] = math.radians(float(val)) if degrees else float(val)
        else:
            arr = np.asarray(qpos, dtype=float)
            d.qpos[:len(arr)] = arr
            if quat is None and yaw is None and len(arr) >= 7:
                qn = float(np.linalg.norm(d.qpos[3:7]))
                if qn > 1e-12:
                    d.qpos[3:7] /= qn
                qw, qx, qy, qz = (float(x) for x in d.qpos[3:7])
                dyaw = libm.arctan2(2.0 * (qw * qy + qx * qz), 1.0 - 2.0 * (qx * qx + qy * qy)) - h0
    if qvel is not None:
        if isinstance(qvel, dict):
            for jname, val in qvel.items():
                j_idx = JOINTS.index(str(jname))
                d.qvel[world.vadr[j_idx]] = math.radians(float(val)) if degrees else float(val)
        else:
            arr = np.asarray(qvel, dtype=float)
            d.qvel[:len(arr)] = arr
    mj.mj_forward(m, d)
    if qpos is not None or zero_vel:
        q_all = d.qpos[world.qadr]
        q_drv = q_all[g['joints']]
        world.was[:] = q_all
        world.target[g['joints']] = q_drv
        g['ref_q'][:] = q_drv
        g['eso'][0][:] = q_drv
        if zero_vel:
            g['ref_v'][:] = 0.0
            g['eso'][1:] = 0.0
        played = g.get('played')
        if isinstance(played, dict) and 'pose' in played:
            played['pose'][:] = np.degrees(q_all)
            if qpos is not None:
                played['cmd'][:] = played['pose']
                played['target'][:] = np.radians(played['cmd'][played['joints']])
    edge.read(world, g['sense'])
    edge.contacted(world, g['sense'])
    g['est']['pos'][:] = d.qpos[0:3]
    if zero_vel:
        g['est']['vel'][:] = 0.0
    elif qvel is not None and not isinstance(qvel, dict) and len(qvel) >= 3:
        g['est']['vel'][:] = d.qvel[0:3]
    elif abs(dyaw) > 1e-12:
        cd, sd = libm.cos(dyaw), libm.sin(dyaw)
        ev = g['est']['vel']
        ev[0], ev[2] = float(ev[0]) * cd + float(ev[2]) * sd, -float(ev[0]) * sd + float(ev[2]) * cd
    from machine.process import sprung
    g['inekf'] = inekf.laid(d.qpos[0:3], d.qpos[3:7])
    g['beat'] = sprung._beat().laid(g)
    hd = h0 + dyaw
    sh, ch = libm.sin(hd), libm.cos(hd)
    g['heading'] = hd
    if g.get('has_heading', 0.0):
        g['target_heading'] = float(g.get('target_heading', 0.0)) + dyaw
    g['fr'][modal.HEADING] = hd
    g['fr'][modal.ROT] = np.array([sh, 0.0, ch, ch, 0.0, -sh, 0.0, 1.0, 0.0])
    g['s'] = proc_sensed(body, g['est'], g['sense'], g['limbs'])
    modal.sensed(g['s'], g['loads'], g['y'])
    g['y'][modal.CELLS] = g['sense'][edge.CELLS]
    if np.any(dp != 0.0) or abs(dyaw) > 1e-12 or qpos is not None:
        g['fr'][modal.FLOOR_AT] = g['y'][modal.SOLE].reshape(2, 3)[:, [0, 2]].mean(axis=0)
    s['fallen_at'], s['shutdown'], s['refused'] = None, None, None
    s['worn'].emptied()


def reload_state(s, name: str | None = 'default', *, snap=None, path: str | None = None,
                 scenario: str | None = None, where=None, yaw: float | None = None,
                 yaw_deg: float | None = None, quat=None, qpos=None, qvel=None,
                 degrees: bool = True, zero_vel: bool = False, say: str | None = None,
                 command: dict[str, Any] | None = None) -> dict[str, Any]:
    """Reload `s` from a named/packed checkpoint and optionally place the robot in MuJoCo."""
    from machine.process import worker
    t0 = time.perf_counter()
    if snap is not None:
        checkpoint.restore(s, snap)
    elif path is not None:
        with open(path, 'rb') as f:
            blob = pickle.load(f)
        unpack_state(s, blob)
    elif name is not None:
        saved = (s.get('_checkpoints') or {}).get(str(name))
        if saved is None:
            raise MachineError('unknown checkpoint %r (saved: %s)' % (name, ', '.join(list_states(s)) or 'none'))
        checkpoint.restore(s, saved)
    if scenario is not None:
        s['g']['world'].scenario(str(scenario))
        s['g']['scenario'] = str(scenario)
    if any(x is not None for x in (where, yaw, yaw_deg, quat, qpos, qvel)) or zero_vel:
        place_robot(s, where=where, yaw=yaw, yaw_deg=yaw_deg, quat=quat,
                    qpos=qpos, qvel=qvel, degrees=degrees, zero_vel=zero_vel)
    if say is not None:
        worker.apply(s, {'say': say})
    if command is not None:
        worker.apply(s, dict(command))
    d = s['g']['world'].data
    return {
        'reloaded': name if path is None else path, 't': float(s['g']['t']),
        'passes': int(s['passes']), 'where': d.qpos[0:3].tolist(),
        'heading': float(s['g']['heading']), 'reload_ms': round(1e3 * (time.perf_counter() - t0), 3),
    }


def _eval_on_session(s, cand_specs, steps: int, reload_kw: dict[str, Any]) -> dict[str, Any]:
    """Reload `s`, hot-swap `cand_specs`, step `steps` passes, and return candidate metrics."""
    from machine.process import worker
    t0 = time.perf_counter()
    rel = reload_state(s, **reload_kw)
    g = s['g']
    unload_all(g, s=s)
    swapped = swap_batch(g, cand_specs, s=s) if cand_specs else []
    surety_min = float(g['surety'])
    eff0 = float(g['world'].effort)
    ran = 0
    for _ in range(int(steps)):
        worker.stepped(s)
        ran += 1
        surety_min = min(surety_min, float(g['surety']))
        if s['fallen_at'] is not None:
            break
    d = g['world'].data
    res = {
        'swapped': [x.get('name') or x.get('engram') for x in swapped],
        'ok': s['fallen_at'] is None and s['shutdown'] is None,
        'fallen_at': s['fallen_at'], 'steps': ran, 'surety_min': round(surety_min, 4),
        'effort': round(float(g['world'].effort) - eff0, 4),
        'where': d.qpos[0:3].tolist(), 'reload_ms': rel['reload_ms'],
        'wall_ms': round(1e3 * (time.perf_counter() - t0), 2),
    }
    unload_all(g, s=s)
    return res


def _mp_init(packed_bytes: bytes):
    """Initialize one worker process session from serialized checkpoint bytes."""
    os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')
    from machine.process import worker
    from machine.world import physics
    from tools.dev import arithmetic
    arithmetic.hold()
    blob = pickle.loads(packed_bytes)
    meta = blob['meta']
    physics.ROOM, physics.CAMERA, physics.SIGHT = meta['physics']
    s = worker.session(every=meta['every'], walks=meta['walks'])
    if s['g'].get('keep') is not None:
        s['g']['keep'].close()
        s['g']['keep'] = None
    unpack_state(s, blob)
    save_state(s, '_iter_base', allow_apart=True)
    _MP_STATE['s'] = s


def _mp_eval(task):
    """Worker entrypoint for parallel multi-core candidate iteration."""
    idx, cand_specs, steps, reload_kw = task
    s = _MP_STATE['s']
    try:
        out = _eval_on_session(s, cand_specs, steps, dict(reload_kw, name='_iter_base'))
        out['index'] = idx
        return out
    except Exception as exc:
        return {'index': idx, 'ok': False, 'error': str(exc)}


def iterate(candidates: list[Any], *, s=None, checkpoint_name: str = 'default',
            path: str | None = None, steps: int = 500, jobs: int = 0, **reload_kw) -> list[dict[str, Any]]:
    """Evaluate multiple LLM core/engram candidates from one MuJoCo state in-memory or in parallel."""
    from tools.dev import census
    compile_batch([item for cand in candidates for item in normalize_specs(cand)],
                  g=None if s is None else s['g'])
    workers = census.physical_cores() if jobs == 0 else max(1, int(jobs))
    if workers == 1 and s is not None:
        out = []
        for idx, cand in enumerate(candidates):
            try:
                r = _eval_on_session(s, cand, steps, dict(reload_kw, name=checkpoint_name, path=path))
                r['index'] = idx
                out.append(r)
            except Exception as exc:
                out.append({'index': idx, 'ok': False, 'error': str(exc)})
        reload_state(s, name=checkpoint_name, path=path)
        return out
    if path is not None:
        with open(path, 'rb') as f:
            packed_bytes = f.read()
    elif s is not None:
        saved = (s.get('_checkpoints') or {}).get(checkpoint_name)
        snap = checkpoint.capture(s, allow_apart=True) if saved is None else saved
        packed_bytes = pickle.dumps(pack_state(s, snap, allow_apart=True), protocol=pickle.HIGHEST_PROTOCOL)
    else:
        raise MachineError('iterate requires either session `s` or checkpoint `path`')
    clean_kw = {k: v for k, v in reload_kw.items() if k not in ('snap', 'path', 'name')}
    tasks = [(idx, cand, int(steps), clean_kw) for idx, cand in enumerate(candidates)]
    import multiprocessing
    n_proc = min(workers, max(1, len(tasks)))
    with ProcessPoolExecutor(max_workers=n_proc, mp_context=multiprocessing.get_context('spawn'),
                             initializer=_mp_init, initargs=(packed_bytes,)) as pool:
        return list(pool.map(_mp_eval, tasks))


def apply_checkpoint(s, cmd: Any) -> dict[str, Any]:
    """Execute a `{'checkpoint': ...}` command dict on live worker session `s`."""
    if isinstance(cmd, str):
        cmd = {'action': 'save', 'name': cmd}
    action = str(cmd.get('action', 'reload')).lower()
    if action == 'save':
        return save_state(s, cmd.get('name', 'default'), path=cmd.get('path'), allow_apart=True)
    if action == 'reload':
        kw = {k: cmd[k] for k in ('path', 'scenario', 'where', 'yaw', 'yaw_deg', 'quat',
                                  'qpos', 'qvel', 'degrees', 'zero_vel', 'say', 'command') if k in cmd}
        return reload_state(s, cmd.get('name', 'default' if 'path' not in cmd else None), **kw)
    if action == 'list':
        return {'checkpoints': list_states(s)}
    if action == 'delete':
        (s.get('_checkpoints') or {}).pop(str(cmd.get('name', 'default')), None)
        return {'deleted': str(cmd.get('name', 'default')), 'checkpoints': list_states(s)}
    raise MachineError('unknown checkpoint action %r' % (action,))


def apply_hotswap(s, cmd: Any) -> dict[str, Any]:
    """Execute a `{'hotswap': ...}` command dict on live worker session `s`."""
    g = s['g']
    if isinstance(cmd, dict):
        action = str(cmd.get('action', 'swap')).lower()
        if action == 'unload':
            if cmd.get('all') or 'target' not in cmd:
                unload_all(g, s=s)
            else:
                unload(g, str(cmd['target']), keep=str(cmd.get('keep', '')), s=s)
            return {'action': 'unload', 'active': active(g)}
        if action == 'status':
            return {'action': 'status', 'active': active(g), 'checkpoints': list_states(s)}
        if 'spec' in cmd:
            cmd = cmd['spec']
    swapped = swap_batch(g, cmd, s=s)
    return {'action': 'swap', 'swapped': swapped, 'active': active(g)}


def tcp_command(s, command: dict[str, Any], told):
    """Handle a TCP telemetry client's `hotswap` and/or `checkpoint` request on session `s`."""
    try:
        res: dict[str, Any] = {'t': float(s['g']['t']), 'analyzer': 'hotswap', 'event': 'hotswap', 'ok': True}
        if 'checkpoint' in command:
            res['checkpoint'] = apply_checkpoint(s, command['checkpoint'])
        if 'hotswap' in command:
            res['hotswap'] = apply_hotswap(s, command['hotswap'])
        s['hotswap_last'], s['refused'] = res, None
        told(res)
    except Exception as exc:
        s['refused'] = str(exc)
        told({'t': float(s['g']['t']), 'analyzer': 'hotswap', 'event': 'refused',
              'ok': False, 'why': str(exc)})


def remote(port: int, payload: dict[str, Any], host: str = '127.0.0.1', timeout: float = 15.0) -> dict[str, Any]:
    """Send a live `hotswap` or `checkpoint` payload to an attached telemetry port and return reply."""
    end = time.monotonic() + timeout
    with socket.create_connection((host, int(port)), timeout=timeout) as sock, sock.makefile('rb') as f:
        sock.sendall(json.dumps(payload).encode('utf-8') + b'\n')
        while time.monotonic() < end:
            sock.settimeout(max(0.05, end - time.monotonic()))
            try:
                line = f.readline()
            except TimeoutError:
                break
            if not line:
                break
            rec = json.loads(line)
            if rec.get('analyzer') == 'hotswap':
                return rec
    raise MachineError('no hotswap response received from %s:%d within %.1fs' % (host, port, timeout))
