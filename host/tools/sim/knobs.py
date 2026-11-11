"""A handover spec's knobs: JSON, TOML or key=value overrides on her texts and gait tables.

    texts = knobs.arranged(handover.parsed('sea=143,track=12,edge=0.1'))   # the texts given (tune.given), or None

Supports flat `key=value` pairs, inline JSON, or `.json`/`.toml` files (and `COAXIAL_PARAMS` from the environment),
plus `behaviors` (`{behavior_name: {clause: number}}`) across any Lisp behavior/trait/derivation and `gait` overrides.
"""
import itertools
import json
import math
import os
import tomllib
from typing import Any

GAIT_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'lisp', 'humanoid', 'gait_base.lisp')
POSTURES_PATH = os.path.join(os.path.dirname(GAIT_PATH), 'postures_base.lisp')


def parse_spec(raw: Any = None) -> dict[str, Any]:
    """Parse `raw` (`dict`, `.toml`/`.json` file path, inline JSON, or `k=v,k=v`) into a dict."""
    if not raw:
        return {}
    if isinstance(raw, dict):
        return dict(raw)
    text = str(raw).strip()
    if os.path.isfile(text):
        with open(text, 'rb') as f:
            return dict(tomllib.load(f) if text.endswith('.toml') else json.loads(f.read().decode('utf-8')))
    if text.startswith('{'):
        return dict(json.loads(text))
    return dict(kv.split('=', 1) for kv in text.split(',') if '=' in kv)


def merged(base: Any = None, over: Any = None) -> dict[str, Any]:
    """Deep-merge two parameter dicts (`over` wins on leaf keys)."""
    out: dict[str, Any] = {str(k): merged(v, None) if isinstance(v, dict) else v for k, v in (base or {}).items()}
    for k, v in (over or {}).items():
        out[str(k)] = merged(out.get(str(k)), v) if isinstance(v, dict) else v
    return out


def expand_variants(cfg: dict[str, Any]) -> list[tuple[str, dict[str, Any]]]:
    """Expand `cfg` (`variants` list and/or Cartesian `grid` dict) into `[(name, spec_dict)]`."""
    base = {k: v for k, v in cfg.items() if k not in ('variants', 'grid', 'plays', 'wobbles', 'ensemble', 'window')}
    raw_vars: list[dict[str, Any]] = list(cfg.get('variants') or [])
    out = [(str(v.get('name', 'v%d' % i)), merged(base, {k: x for k, x in v.items() if k != 'name'}))
           for i, v in enumerate(raw_vars)]
    grid: dict[str, Any] = dict(cfg.get('grid') or {})
    if grid:
        keys = list(grid)
        vals = [v if isinstance(v, list) else [v] for v in grid.values()]
        for combo in itertools.product(*vals):
            tag = ','.join('%s=%s' % (k.split('.')[-1], c) for k, c in zip(keys, combo))
            spec = merged(base, {})
            for k, c in zip(keys, combo):
                if '.' in k:
                    bname, clause = k.split('.', 1)
                    spec.setdefault('behaviors', {}).setdefault(bname, {})[clause] = c
                else:
                    spec[k] = c
            out.append((tag, spec))
    return out or [('base', base)]


def literal(src, key, start=0, end=None):
    """(where the number after `key` begins and ends in `src`, from `start`, and its value)."""
    pos = src.find(key, start) if end is None else src.find(key, start, end)
    if pos < 0:
        raise KeyError(key)
    j = pos + len(key)
    while j < len(src) and src[j] == ' ':
        j += 1
    z = j
    while z < len(src) and src[z] in '0123456789.-+':
        z += 1
    return j, z, float(src[j:z])


def _form_span(src, name):
    """`(start, end)` of top-level `(defbehavior|derive|deftrait|define-... <name> ...)` in `src`, or None."""
    for head in ('(defbehavior ', '(derive ', '(deftrait ', '(define-'):
        idx = 0
        while (pos := src.find(head, idx)) >= 0:
            rest = src[pos + len(head):]
            tok = rest.split(None, 1)[0].rstrip(')') if rest else ''
            if tok == name or (head == '(define-' and name in rest[:48].split()):
                depth = 0
                for k in range(pos, len(src)):
                    depth += (src[k] == '(') - (src[k] == ')')
                    if depth == 0:
                        return pos, k + 1
            idx = pos + len(head)
    return None


def _edit_behaviors(texts, edits, behaviors):
    """Record `(path, j, z) -> val` edits for `{behavior_name: {clause: val}}` across `texts`."""
    for bname, clauses in (behaviors or {}).items():
        hit = next(((p, span) for p, s in texts.items() if (span := _form_span(s, bname)) is not None), None)
        if hit is None:
            raise KeyError('unknown behavior %r' % bname)
        path, (lo, hi) = hit
        src = texts[path]
        for clause, val in clauses.items():
            needle = clause if clause.startswith(('(', ':')) else '(' + clause + ' '
            j, z, _ = literal(src, needle, lo, hi)
            edits[(path, j, z)] = float(val)


def _apply_gait_map(gmap: dict[str, Any]) -> None:
    """Apply `gait` overrides (`{'rhythm': {'turn_sway': 0.4}}` or `{'stride.HEEL_RAD': 0.5}`)."""
    from machine.law import poise, rhythm, rocker, se2, stride, vault
    mods: dict[str, Any] = {'rhythm': rhythm.R, 'steer': se2.STEER, 'stride': stride, 'rocker': rocker, 'vault': vault, 'poise': poise}
    for k, v in (gmap or {}).items():
        if isinstance(v, dict) and k in mods:
            tgt = mods[k]
            for sk, sv in v.items():
                if isinstance(tgt, dict):
                    tgt[sk] = float(sv)
                else:
                    setattr(tgt, sk, float(sv))
        elif '.' in k and not isinstance(v, dict):
            mname, attr = k.split('.', 1)
            tgt = mods[mname]
            if isinstance(tgt, dict):
                tgt[attr] = float(v)
            else:
                setattr(tgt, attr, float(v))


def arranged(spec: Any = None):
    """The texts and the stride's numbers as `spec` (and `COAXIAL_PARAMS`) have them (`tune.given`), or None."""
    from machine.law import rocker, se2, stride, vault
    from machine.texts import ortho, tune
    cfg: dict[str, Any] = merged(parse_spec(spec), parse_spec(os.environ.get('COAXIAL_PARAMS')))
    if not cfg:
        return None
    spec = cfg
    texts = tune.texts()
    GAIT, POSTURES = (next(k for k in texts if os.path.normcase(os.path.abspath(k)) == os.path.normcase(p)) for p in (GAIT_PATH, POSTURES_PATH))
    gait, postures = texts[GAIT], texts[POSTURES]
    edits: dict[tuple[str, int, int], float] = {}
    behaviors: dict[str, Any] = dict(cfg.get('behaviors') or {})
    for k, v in cfg.items():
        if '.' in k and not isinstance(v, dict) and k.split('.', 1)[0] not in ('rhythm', 'steer', 'stride', 'rocker', 'vault', 'poise'):
            bname, clause = k.split('.', 1)
            behaviors.setdefault(bname, {})[clause] = v
    if behaviors:
        _edit_behaviors(texts, edits, behaviors)
    gmap: dict[str, Any] = dict(cfg.get('gait') or {})
    for k, v in cfg.items():
        if '.' in k and not isinstance(v, dict) and k.split('.', 1)[0] in ('rhythm', 'steer', 'stride', 'rocker', 'vault', 'poise'):
            gmap[k] = v
    if gmap:
        _apply_gait_map(gmap)
    if 'ax' in spec:
        j, z, _ = literal(se2.WALK_ON, '(mode_axial_compression ')
        se2.WALK_ON = se2.WALK_ON[:j] + str(spec['ax']) + se2.WALK_ON[z:]
        se2._WALK_ON_WIRE = ortho.compiled(se2.WALK_ON)[0]['']
        j, z, _ = literal(postures, '(mode_axial_compression ', postures.find('(defbehavior init_walk'))
        edits[(POSTURES, j, z)] = float(spec['ax'])
    walk = gait.find('(sequence_id 204)')
    if 'rock' in spec:
        j, z, was = literal(gait, '(amplitude mode_foot_rock ', walk)
        edits[(GAIT, j, z)] = round(was * float(spec['rock']), 3)
    if 'drive' in spec:
        j, z, _ = literal(gait, '(amplitude mode_sagittal_drive ', walk)
        edits[(GAIT, j, z)] = float(spec['drive'])
    jog = postures.find('(defbehavior jog')
    for key, name in (('mhz', '(frequency '), ('mdrive', '(amplitude mode_sagittal_drive '), ('mturn', '(amplitude mode_pelvis_turn '),
                      ('max', '(mode_axial_compression ')):
        if key in spec:
            at = jog
            for _phase in range(2):
                j, z, _ = literal(postures, name, at)
                edits[(POSTURES, j, z)] = float(spec[key])
                at = z
    if 'k' in spec:
        j, z, _ = literal(gait, '(stiffness mode_axial_compression ')
        edits[(GAIT, j, z)] = float(spec['k'])
        j, z, _ = literal(gait, '(damping mode_axial_compression ')
        edits[(GAIT, j, z)] = float(spec.get('c', 0.2 * float(spec['k'])))
    # a stride constant is bound by name where rocker and vault import it: set in each (the C table takes stride's)
    for key, name, scale in (('heel', 'HEEL_RAD', math.pi / 180.0), ('heelrate', 'HEEL_RAD_S', 1.0), ('flatk', 'FLAT_K', 1.0),
                             ('fold', 'FOLD_RAD', 1.0), ('dip', 'DIP_M', 1.0), ('ls', 'LAND_SHARE', 1.0), ('rise', 'RISE_U', 1.0),
                             ('edge', 'EDGE_RAD', 1.0)):
        if key in spec:
            for mod in (stride, rocker, vault):
                if hasattr(mod, name):
                    setattr(mod, name, scale * float(spec[key]))
    if 'swing' in spec:
        stride.SWING = float(spec['swing'])
        stride.PHI_W = math.pi * stride.SWING
    if 'swing_fast' in spec:
        stride.SWING_FAST = float(spec['swing_fast'])
    if 'pace_fast' in spec:
        stride.PACE_FAST_M_S = float(spec['pace_fast'])
    if 'rund' in spec:
        from machine.law import rhythm
        rhythm.R['run_drive'] = float(spec['rund'])
    if 'sea' in spec:
        from machine.body import drives
        drives.SERIES['ankle'] = float(spec['sea'])
    if 'wind' in spec:
        from machine.body import drives
        drives.AHEAD_WIND = float(spec['wind'])
    if 'track' in spec:
        from machine.body import gearbox
        gearbox.TRACK = float(spec['track'])
    if 'rates' in spec:
        from machine.body import gearbox
        gearbox.RATE_S = float(spec['rates'])
    if 'charge' in spec:
        from machine.process import worker
        worker.HOP_CHARGE = float(spec['charge'])
    if 'wfold' in spec:
        from machine.stack import wrench
        wrench.W_FOLD = float(spec['wfold'])
    return tune.given(edits) if edits else None
