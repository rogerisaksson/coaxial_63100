"""Mechatronics as data: the parts library (motors, gearboxes, inverters) and a robot's build of it.

    lib = mechatronics.library()            # {'motors': .., 'gearboxes': .., 'inverters': ..}
    build = mechatronics.robot(ROBOT)       # machine/humanoids/<name>.toml, its parts the library's
    mechatronics.robots()                   # every build file's name

A robot's file names, for each joint kind, its stack's motor, gearbox and inverter from the
library, the servo its board runs, the structure between its gearbox and its limb, and what has
no drive; `machine.body.drives` and `machine.body.servos` read it, `tools/cores/manifest.py` writes its
orthokinematic manifest of it, her text (machine/<robot>.lisp) includes that. ROBOT is the
process's: COAXIAL_ROBOT in the environment names another build, its text, manifest and
corrections (`machine.identify.identified`) with it.
"""
import hashlib
import json
import os
import tempfile
import tomllib
from typing import Any

from machine.errors import MachineError

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(HERE))
CACHE = os.path.join(REPO, '.cache', 'compiled')
LIBRARY, ROBOTS = os.path.join(HERE, 'library'), os.path.join(HERE, 'humanoids')
#: Every lisp text, out of the package: hers (texts, suite/, kernel/, builds/) and the crafts'.
LISP = os.path.join(os.path.dirname(HERE), 'lisp')
HUMANOID, AIRCRAFT = os.path.join(LISP, 'humanoid'), os.path.join(LISP, 'aircraft')
#: Classification schema following the gynoid_<morphology>_<scale> convention:
#: - gynoid_ectomorph_petite: Small-framed and fine-boned (waif/gamine, < 165 cm); yields an
#:   extremely low limb moment of inertia, but minimal joint volume forces tendon-driven
#:   architectures or low-torque actuators (the Ava archetype).
#: - gynoid_ectomorph_tall: Slender, statuesque column form (> 175 cm); long lever arms provide
#:   extended reach but demand high structural flexural rigidity and strictly minimized distal
#:   mass at the knees and ankles.
#: - gynoid_mesomorph_medium: Anthropometric standard (160-168 cm); a pragmatic compromise
#:   between statistical human proportions and sufficient internal/external envelope so the
#:   structural chassis is robust rather than overly fragile,
#:   accommodating all structural 3D prints, actuators, harmonic drives, battery packs, TPU
#:   gel pads, and clothing (165 cm, 55.0 kg, 24 cm sole in machine.body.figure).
#: - gynoid_hourglass_full: Hyper-gynoid geometry (WHR <= 0.70); introduces a severe structural
#:   bottleneck for power buses and spinal actuators at the waist, while offering generous
#:   packaging volume for cells and high-torque hip actuators in the pelvis and thorax.
#: - gynoid_athletic_heavy: High-dynamic, muscular profile; expanded thigh, gluteal, and shoulder
#:   envelopes dictated by high-torque-density QDD actuators, structural stiffness under impact
#:   loads, and thermal dissipation requirements.
#: - gynoid_skeletal_ultralight: Hollowed truss or exoskeleton chassis stripped of synthetic soft
#:   tissue; reduces feminine cues strictly to pelvic geometry, Q-angle, and lumbar lordosis to
#:   minimize total system mass.
#: Her own build - the committed C model's (tools/cores/model.py) -, and the robot this process builds:
#: COAXIAL_ROBOT names one of `robots()`, else hers.
HERS = 'gynoid_mesomorph_medium'
ROBOT = os.environ.get('COAXIAL_ROBOT', HERS)
#: Each part's file in the library, and what a stack names of it in turn.
PARTS = ('motors', 'gearboxes', 'inverters')
#: Canonical whole-body latent mode names (`ortho.MODES`, `compile`, `modal`).
MODES = ('axial_compression', 'lateral_shift', 'sagittal_drive', 'pitch_attitude', 'sagittal_shift',
         'limb_lift', 'stance_width', 'pelvis_list', 'pelvis_turn', 'arm_swing', 'foot_rock', 'arm_reach', 'arm_fold')

_READ = {}
_COMPILED = {}


def _toml(path):
    st = os.stat(path)
    sig = (st.st_mtime_ns, st.st_size)
    hit = _READ.get(path)
    if hit is None or hit[0] != sig:
        with open(path, 'rb') as f:
            _READ[path] = (sig, tomllib.load(f))
    return _READ[path][1]


def _to_json(v):
    if isinstance(v, tuple):
        return {'__tuple__': [_to_json(x) for x in v]}
    if isinstance(v, list):
        return [_to_json(x) for x in v]
    if isinstance(v, dict):
        return {str(k): _to_json(x) for k, x in v.items()}
    return v


def _from_json(v) -> Any:
    if isinstance(v, dict):
        if '__tuple__' in v and len(v) == 1:
            return tuple(_from_json(x) for x in v['__tuple__'])
        return {k: _from_json(x) for k, x in v.items()}
    if isinstance(v, list):
        return [_from_json(x) for x in v]
    return v


def library_sources():
    """Sorted paths of all declarative `.toml` files in `machine/library`."""
    return tuple(sorted(os.path.join(LIBRARY, f) for f in os.listdir(LIBRARY) if f.endswith('.toml')))


def robot_sources(name=ROBOT):
    """All `.toml` construction files governing humanoid `name` (its chain of `extends` + library)."""
    chain, cur = [], name
    while cur:
        p = os.path.join(ROBOTS, cur + '.toml')
        chain.append(p)
        cur = _toml(p).get('extends')
    return library_sources() + tuple(sorted(chain))


def craft_sources():
    """All `.toml` construction files governing aircraft and drones (`crafts.toml` + library)."""
    return library_sources() + (os.path.join(HERE, 'aircraft', 'crafts.toml'),)


def compiled(key: str, paths, builder) -> dict[str, Any]:
    """Load compiled parameterization `key` from `.cache/compiled/<key>.json`, or run `builder()` on
    first run or whenever any source file in `paths` changes (by stat signature + SHA-256 digest)."""
    ordered = tuple(sorted({os.path.abspath(p) for p in paths}))
    sig = tuple((st.st_mtime_ns, st.st_size) for st in (os.stat(p) for p in ordered))
    mem = _COMPILED.get(key)
    if mem is not None and mem[0] == sig:
        return mem[1]
    h = hashlib.sha256()
    for p in ordered:
        h.update(os.path.basename(p).encode('utf-8') + b'\0')
        with open(p, 'rb') as f:
            h.update(f.read())
        h.update(b'\0')
    digest = h.hexdigest()
    target = os.path.join(CACHE, key + '.json')
    if os.path.isfile(target):
        try:
            with open(target, encoding='utf-8') as f:
                saved = json.load(f)
            if isinstance(saved, dict) and saved.get('digest') == digest:
                payload = _from_json(saved['payload'])
                _COMPILED[key] = (sig, payload)
                return payload
        except (OSError, ValueError, KeyError):
            pass
    payload = builder()
    os.makedirs(CACHE, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=CACHE, prefix=key + '.', suffix='.tmp')
    try:
        with os.fdopen(fd, 'w', encoding='utf-8') as f:
            json.dump({'digest': digest, 'payload': _to_json(payload)}, f, sort_keys=True)
        os.replace(tmp, target)
    except OSError:
        if os.path.exists(tmp):
            os.unlink(tmp)
    _COMPILED[key] = (sig, payload)
    return payload


def library():
    """{part: {name: its data}} of every part in the library."""
    return {part: _toml(os.path.join(LIBRARY, part + '.toml')) for part in PARTS}


def materials():
    """Structural laminates, metals, stock carbon tubes and thermal material constants (`materials.toml`)."""
    return _toml(os.path.join(LIBRARY, 'materials.toml'))


def environment(name: str | None = ROBOT):
    """Physical environment, atmosphere, contact solver, flight pack and weather (`world.toml`)."""
    base = _toml(os.path.join(LIBRARY, 'world.toml'))
    over = robot(name).get('world') if name else None
    return _merged(base, over) if isinstance(over, dict) else dict(base)


def crafts():
    """Declarative physical construction of every aircraft and variant (`aircraft/crafts.toml`)."""
    raw = _toml(os.path.join(HERE, 'aircraft', 'crafts.toml'))
    out = {'shared': dict(raw['shared'])}
    for name, cfg in raw.items():
        if name == 'shared' or not isinstance(cfg, dict):
            continue
        if 'extends' in cfg:
            base = out.get(cfg['extends']) or raw[cfg['extends']]
            out[name] = _merged(base, {k: v for k, v in cfg.items() if k != 'extends'})
        else:
            out[name] = dict(cfg)
    return out


def robots():
    """Every robot a build file names (machine/humanoids)."""
    return sorted(f[:-5] for f in os.listdir(ROBOTS) if f.endswith('.toml'))


def _merged(base, over):
    """`base` with `over` laid on it: a table merged key by key, anything else `over`'s."""
    out = dict(base)
    for k, v in over.items():
        out[k] = _merged(base[k], v) if isinstance(v, dict) and isinstance(base.get(k), dict) else v
    return out


def robot(name=ROBOT) -> dict:
    """A robot's build (machine/humanoids/`name`.toml), each stack's parts found in the library. A build that
    `extends` another is that one with its own tables laid on it - only what is its own -, its `swap` the
    motors it takes for that one's in every stack it does not name."""
    path = os.path.join(ROBOTS, name + '.toml')
    if not os.path.isfile(path):
        raise MachineError('no robot %s: %s' % (name, ', '.join(sorted(
            f[:-5] for f in os.listdir(ROBOTS) if f.endswith('.toml')))))
    build, lib = _toml(path), library()
    if 'extends' in build:
        base, swap = robot(build['extends']), build.get('swap', {})
        base = dict(base, stacks={kind: [swap.get(stack[0], stack[0])] + list(stack[1:])
                                  for kind, stack in base['stacks'].items()})
        build = _merged(base, {k: v for k, v in build.items() if k not in ('extends', 'swap')})
    for kind, stack in build['stacks'].items():
        for part, named in zip(PARTS, stack):
            if named not in lib[part]:
                raise MachineError('%s: %s %s is no %s in the library (%s)' % (
                    name, kind, named, part[:-1], ', '.join(lib[part])))
    return build
