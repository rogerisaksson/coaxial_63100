"""Mechatronics as data: the parts library (motors, gearboxes, inverters) and a robot's build of it.

    lib = mechatronics.library()            # {'motors': .., 'gearboxes': .., 'inverters': ..}
    build = mechatronics.robot(ROBOT)       # machine/humanoids/<name>.toml, its parts the library's
    mechatronics.robots()                   # every build file's name

A robot's file names its drives' parts from the library - the gearbox and the inverter of every
joint kind's stack once, a motor a group of kinds (`[drives]`) -, the structure between its
gearbox and its limb, and what has no drive; `machine.body.drives` and `machine.body.servos` read
it, `tools/cores/manifest.py` writes its orthokinematic manifest of it, the robot's text
(machine/<robot>.lisp) includes that. ROBOT is the
process's: COAXIAL_ROBOT in the environment names another build, its text, manifest and
corrections (`machine.identify.identified`) with it.
"""
import hashlib
import json
import math
import os
import tempfile
import tomllib
from typing import Any

from machine.errors import MachineError
from motor import constants

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(HERE))
CACHE = os.path.join(REPO, '.cache', 'compiled')
LIBRARY, ROBOTS = os.path.join(HERE, 'library'), os.path.join(HERE, 'humanoids')
#: Every lisp text, out of the package: the robot's own (texts, suite/, kernel/, builds/) and the crafts'.
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
#: The robot's own build - the committed C model's (tools/cores/model.py) -, and the robot this process
#: builds: COAXIAL_ROBOT names one of `robots()`, else its own.
STOCK = 'gynoid_mesomorph_medium'
ROBOT = os.environ.get('COAXIAL_ROBOT', STOCK)
CRAFTS = os.environ.get('COAXIAL_CRAFTS', os.path.join(HERE, 'aircraft', 'crafts.toml'))
#: Each part's file in the library, and what a stack names of it in turn.
PARTS = ('motors', 'gearboxes', 'inverters')
#: Clothing's insulation over bare air (`[cloth]`, ISO 9920) on a joint a build dresses in nothing: the identity.
BARE = 1.0
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


def _robot_path(name: str = ROBOT, near: str | None = None) -> str:
    if name.endswith('.toml') or '/' in name or os.sep in name:
        p = os.path.abspath(os.path.join(near, name) if near and not os.path.isabs(name) else name)
        if os.path.isfile(p):
            return p
    if near:
        sib = os.path.join(near, name + '.toml')
        if os.path.isfile(sib):
            return os.path.abspath(sib)
    p = os.path.join(ROBOTS, name + '.toml')
    if os.path.isfile(p):
        return p
    raise MachineError('no robot %s: %s' % (name, ', '.join(robots())))


def cache_id(name: str = ROBOT) -> str:
    """Filesystem-safe cache stem for humanoid build `name` or `.toml` path."""
    p = _robot_path(name)
    stem = os.path.splitext(os.path.basename(p))[0]
    if os.path.normcase(os.path.dirname(p)) == os.path.normcase(os.path.abspath(ROBOTS)):
        return stem
    return '%s_%s' % (stem, hashlib.sha256(p.encode('utf-8')).hexdigest()[:12])


def robot_sources(name=ROBOT):
    """All `.toml` construction files governing humanoid `name` (its chain of `extends` + library)."""
    chain, cur, near = [], name, None
    while cur:
        p = _robot_path(cur, near)
        chain.append(p)
        cur, near = _toml(p).get('extends'), os.path.dirname(p)
    return library_sources() + tuple(sorted(chain))


def craft_sources(spec: str | None = None):
    """All `.toml` and static solver files governing aircraft and drones (`crafts.toml` + library + solver)."""
    p = os.path.abspath(spec) if spec else os.path.abspath(CRAFTS)
    solv_dir = os.path.join(HERE, 'solver')
    return library_sources() + (p, constants.__file__, os.path.join(solv_dir, 'loops.py'),
                                os.path.join(solv_dir, 'contacts.py'), os.path.join(HERE, 'aircraft', 'solved.py'))


_BODY_MODS = ('mechatronics', 'figure', 'linkage', 'skeleton', 'structural', 'guards',
              'drives', 'build', 'carried', 'looped')


def body_sources(name=ROBOT):
    """All `.toml` and Python solver files governing humanoid `name`'s static body compilation."""
    body_dir, solv_dir = os.path.join(HERE, 'body'), os.path.join(HERE, 'solver')
    py = tuple(os.path.join(body_dir, m + '.py') for m in _BODY_MODS) + (
        os.path.join(solv_dir, 'inertia.py'), os.path.join(solv_dir, 'loops.py'),
        os.path.join(solv_dir, 'contacts.py'), constants.__file__)
    return robot_sources(name) + py


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


def _scale_inverter(base, over, merged):
    if 'amps' in over and float(base.get('amps', 0.0)) > 0.0:
        s_i = float(over['amps']) / float(base['amps'])
        if 'switch_q_c' not in over and 'switch_q_c' in base:
            merged['switch_q_c'] = round(float(base['switch_q_c']) * s_i, 12)
        if 'switch_j_k' not in over and 'switch_j_k' in base:
            merged['switch_j_k'] = round(float(base['switch_j_k']) * s_i, 6)
        if 'flight_amps' not in over and 'flight_amps' in base:
            merged['flight_amps'] = round(float(base['flight_amps']) * s_i, 2)
        if 'wep_amps' not in over and 'wep_amps' in base:
            merged['wep_amps'] = min(float(merged['amps']), round(float(base['wep_amps']) * s_i, 2))
    return merged


def _resolve(raw, part=None):
    out = {}

    def one(name):
        if name in out:
            return out[name]
        cfg = raw[name]
        if isinstance(cfg, dict) and 'extends' in cfg:
            base = one(cfg['extends'])
            over = {k: v for k, v in cfg.items() if k != 'extends'}
            merged = _merged(base, over)
            out[name] = _scale_inverter(base, over, merged) if part == 'inverters' else merged
        else:
            out[name] = dict(cfg) if isinstance(cfg, dict) else cfg
        return out[name]

    for k in raw:
        one(k)
    return out


def library():
    """{part: {name: its data}} of every part in the library."""
    return {part: _resolve(_toml(os.path.join(LIBRARY, part + '.toml')), part) for part in PARTS}


def primary_inverter(rbt=None, lib=None) -> str:
    """Name of the highest-current inverter across `rbt['stacks']`."""
    r = robot() if rbt is None else rbt
    lb = library() if lib is None else lib
    return max((s[2] for s in r['stacks'].values()), key=lambda n: float(lb['inverters'][n]['amps']))


def materials():
    """Structural laminates, metals, stock carbon tubes and thermal material constants (`materials.toml`)."""
    return _toml(os.path.join(LIBRARY, 'materials.toml'))


def control():
    """The control design's targets, stated once for every machine (`control.toml`): a loop's damping ratio,
    the phase its delays may take, its factors over a motion, a fall and an outer loop, its observer's over
    it and the excursion its authority is judged at."""
    return _toml(os.path.join(LIBRARY, 'control.toml'))


def laminate_j_k(inv, mat) -> float:
    """Laminate heat capacity (J/K) from copper-FR4 areal heat capacity and disc diameter."""
    return float(inv.get('laminate_j_k',
                         round(float(mat['laminate_j_m2_k']) * math.pi * (0.5 * float(inv['disc_m'])) ** 2, 1)))


def into_k_w(inv, mat) -> float:
    """Switch-to-laminate prepreg thermal resistance (K/W) from areal conductance and disc diameter."""
    return float(inv.get('into_k_w',
                         round(1.0 / (float(mat['prepreg_w_m2_k']) * math.pi * (0.5 * float(inv['disc_m'])) ** 2), 1)))


def v_frac(inv) -> float:
    """Usable bus-voltage fraction after complementary half-bridge gate dead-time blanking."""
    return round(1.0 - 2.0 * float(inv['deadtime_s']) * float(inv['pwm_hz']), 4)


def dry_loss(inv, volts: float) -> float:
    """Three-phase half-bridge capacitive switching loss Q_sw * f_pwm * V_bus (W)."""
    return float(inv['switch_q_c']) * float(inv['pwm_hz']) * float(volts)


def fan_curve(inv, hs, amb: float) -> tuple[float, float, float]:
    """(fan_temp_on C, fan_temp_full C, fan_idle_duty) from continuous flight switch loss vs junction ceiling."""
    t_sw, t_lam = float(inv['ceiling_c'][0]), float(inv['ceiling_c'][1])
    i_fl, v_bus = float(inv['flight_amps']), float(inv['volts'])
    p_sw = 1.5 * i_fl * i_fl * float(inv['rds_ohm']) + dry_loss(inv, v_bus)
    dt_sink = p_sw * float(hs['r_switches_to_sink'])
    return (round(amb + 0.25 * (t_lam - amb), 1), float(round(t_sw - dt_sink, -1)), 0.0)


def environment(name: str | None = ROBOT):
    """Physical environment, atmosphere, contact solver, flight pack and weather (`world.toml`)."""
    base = _toml(os.path.join(LIBRARY, 'world.toml'))
    over = robot(name).get('world') if name else None
    return _merged(base, over) if isinstance(over, dict) else dict(base)


def crafts(spec: str | None = None):
    """Declarative physical construction of every aircraft and variant (`aircraft/crafts.toml` or `spec`):
    a craft its own laid on `[shared]`, a variant (`extends`) its own on its craft."""
    raw = _toml(os.path.abspath(spec) if spec else CRAFTS)
    shared = raw['shared']
    return _resolve({name: cfg if name == 'shared' or 'extends' in cfg else _merged(shared, cfg)
                     for name, cfg in raw.items()})


def robots():
    """Every robot a build file names (machine/humanoids)."""
    return sorted(f[:-5] for f in os.listdir(ROBOTS) if f.endswith('.toml'))


_EXCLUSIVE = {
    'mounts': ({'at'}, {'z_m'}, {'x_m', 'dy_m'}, {'along_m'}),
    'boards': ({'stack_on', 'gap_m'}, {'on', 'at', 'face'}),
}


def _clean_modes(base, over, groups):
    chosen = [g for g in groups if over.keys() & g]
    if not chosen:
        return base
    drop = set().union(*(g for g in groups if g not in chosen))
    return {k: v for k, v in base.items() if k not in drop}


def _merged(base, over, section=None):
    """`base` with `over` laid on it: a table merged key by key, a key `over` sets to false gone, anything else
    `over`'s; mutually exclusive placement keys in `[mounts]` and `[boards]` replace cleanly."""
    out = dict(base)
    groups = _EXCLUSIVE.get(section or '')
    for k, v in over.items():
        if v is False:
            out.pop(k, None)
            continue
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            b_sub = _clean_modes(base[k], v, groups) if groups else base[k]
            out[k] = _merged(b_sub, v, k if section is None else None)
        else:
            out[k] = v
    return out


def _stacks(base, drives):
    """A build's stacks, {kind: [motor, gearbox, inverter]}: `base`'s with its `[drives]` laid on them - a
    motor the kinds' it names (`motors`), a kind new to the build in its place there; the gearbox and the
    inverter it states every stack's; then a gearbox or an inverter the kinds' named under it (`gearboxes`,
    `inverters`)."""
    out = {kind: list(stack) for kind, stack in base.items()}
    for motor, kinds in drives.get('motors', {}).items():
        for kind in kinds:
            out.setdefault(kind, [motor, None, None])[0] = motor
    for at, one, table in ((1, 'gearbox', 'gearboxes'), (2, 'inverter', 'inverters')):
        if one in drives:
            for stack in out.values():
                stack[at] = drives[one]
        for part, kinds in drives.get(table, {}).items():
            for kind in kinds:
                out[kind][at] = part
    return out


def robot(name=ROBOT, near: str | None = None) -> dict:
    """A robot's build (`machine/humanoids/<name>.toml` or a `.toml` path): its tables and its `stacks`,
    {kind: [motor, gearbox, inverter]} laid of its `[drives]` (`_stacks`), each part found in the library.
    A build that `extends` another is that one with its own tables laid on it - only what is its own -, its
    `swap` the motors it takes for that one's in every stack it names no motor for. A joint the build's
    `[cloth]` dresses in nothing is bare (BARE), laid after those the table names, in the stacks' order."""
    path = _robot_path(name, near)
    build, lib = _toml(path), library()
    own, stacks = build.get('drives', {}), {}
    if 'extends' in build:
        base, swap = robot(build['extends'], os.path.dirname(path)), build.get('swap', {})
        stacks = {kind: [swap.get(stack[0], stack[0])] + list(stack[1:]) for kind, stack in base['stacks'].items()}
        build = _merged(base, {k: v for k, v in build.items() if k not in ('extends', 'swap')})
    build = dict({k: v for k, v in build.items() if k != 'drives'}, stacks=_stacks(stacks, own))
    for kind, stack in build['stacks'].items():
        for part, named in zip(PARTS, stack):
            if named not in lib[part]:
                raise MachineError('%s: %s %s is no %s in the library (%s)' % (
                    name, kind, named, part[:-1], ', '.join(lib[part])))
    worn = build.get('cloth', {})
    return dict(build, cloth={**worn, **{kind: BARE for kind in build['stacks'] if kind not in worn}})
