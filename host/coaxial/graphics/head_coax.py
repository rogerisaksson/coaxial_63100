"""Compact decimated mechanical head and face meshes for the coaxial humanoid build."""
import functools
import hashlib
import os
from typing import Any

_CACHE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))), '.cache', 'compiled')


def _sig():
    """Return content digest over the head CAD generator modules and this adapter."""
    render_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                              'tools', 'render')
    h = hashlib.sha256()
    paths = [os.path.abspath(__file__)] + [
        os.path.join(render_dir, name) for name in sorted(os.listdir(render_dir))
        if name.startswith('face_') and name.endswith('.py')]
    for path in paths:
        if os.path.isfile(path):
            h.update(os.path.basename(path).encode('utf-8'))
            with open(path, 'rb') as handle:
                h.update(handle.read())
    return h.hexdigest()[:16]


@functools.lru_cache(maxsize=1)
def bare_mesh():
    """Return welded bare gimbal/servo head mesh `(c, t, u, m)` in `head` frame, cached on disk."""
    from coaxial.model.blocks import numpy as np
    path = os.path.join(_CACHE_DIR, 'head_coax_bare_%s.npz' % _sig())
    if os.path.isfile(path):
        with np.load(path, allow_pickle=False) as data:
            return data['c'], data['t'], data['u'], data['m']
    from tools.render import face_audit as fa, face_built as fb
    bare = fb.head_parts(**fb.PRESETS['neutral'], plates=False)
    got = fa._weld_part_mesh(bare, (0.0, 0.0, 0.0), voxel_m=0.0032)
    os.makedirs(_CACHE_DIR, exist_ok=True)
    arrays: dict[str, Any] = {'c': got[0], 't': got[1], 'u': got[2], 'm': got[3]}
    tmp = '%s.%d.npz' % (path, os.getpid())
    np.savez(tmp, **arrays)
    os.replace(tmp, path)
    return got


def terminal_parts(torso_at, neck_at, head_at, dressed=False):
    """Return calibrated 4-view studio head, neck and collar parts, omitting collar when dressed."""
    from tools.render import face_audit
    got = face_audit.terminal_parts(torso_at, neck_at, head_at)
    return [p for p in got if not (dressed and p[0] == 'mecha_collar')]


def apply(base_meshes, extra):
    """Return `(meshes, extra)` with legacy head/neck meshes and hair slots cleared for calibrated CAD."""
    from coaxial.model.blocks import numpy as np
    empty = (np.zeros((0, 3)), np.zeros((0, 3), int), np.zeros((0, 2)), np.zeros(0, np.uint32))
    kept = [p for p in extra if p[0] not in ('hair', 'hair_fall') and p[1] != 'head']
    return dict(base_meshes, head=empty, neck=empty), kept
