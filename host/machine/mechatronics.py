"""Mechatronics as data: the parts library (motors, gearboxes, inverters) and a robot's build of it.

    lib = mechatronics.library()            # {'motors': .., 'gearboxes': .., 'inverters': ..}
    build = mechatronics.robot(ROBOT)       # machine/robots/<name>.toml, its parts the library's
    mechatronics.robots()                   # every build file's name

A robot's file names, for each joint kind, its stack's motor, gearbox and inverter from the
library, the servo its board runs, the structure between its gearbox and its limb, and what has
no drive; `machine.drives` and `machine.servos` read it, `tools/cores/manifest.py` writes its
orthokinematic manifest of it, her text (machine/<robot>.lisp) includes that. ROBOT is the
process's: COAXIAL_ROBOT in the environment names another build, its text, manifest and
corrections (`machine.identified`) with it.
"""
import os
import tomllib

from machine.errors import MachineError

HERE = os.path.dirname(os.path.abspath(__file__))
LIBRARY, ROBOTS = os.path.join(HERE, 'library'), os.path.join(HERE, 'robots')
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
#:   gel pads, and clothing (165 cm, 55.0 kg, 24 cm sole in machine.figure).
#: - gynoid_hourglass_full: Hyper-gynoid geometry (WHR <= 0.70); introduces a severe structural
#:   bottleneck for power buses and spinal actuators at the waist, while offering generous
#:   packaging volume for cells and high-torque hip actuators in the pelvis and thorax.
#: - gynoid_athletic_heavy: High-dynamic, muscular profile; expanded thigh, gluteal, and shoulder
#:   envelopes dictated by high-torque-density QDD actuators, structural stiffness under impact
#:   loads, and thermal dissipation requirements.
#: - gynoid_skeletal_ultralight: Hollowed truss or exoskeleton chassis stripped of synthetic soft
#:   tissue; reduces feminine cues strictly to pelvic geometry, Q-angle, and lumbar lordosis to
#:   minimize total system mass.
#: The robot this process builds: COAXIAL_ROBOT names one of `robots()`, else her.
ROBOT = os.environ.get('COAXIAL_ROBOT', 'gynoid_mesomorph_medium')
#: Each part's file in the library, and what a stack names of it in turn.
PARTS = ('motors', 'gearboxes', 'inverters')

_READ = {}


def _toml(path):
    if path not in _READ:
        with open(path, 'rb') as f:
            _READ[path] = tomllib.load(f)
    return _READ[path]


def library():
    """{part: {name: its data}} of every part in the library."""
    return {part: _toml(os.path.join(LIBRARY, part + '.toml')) for part in PARTS}


def robots():
    """Every robot a build file names (machine/robots)."""
    return sorted(f[:-5] for f in os.listdir(ROBOTS) if f.endswith('.toml'))


def robot(name=ROBOT):
    """A robot's build (machine/robots/`name`.toml), each stack's parts found in the library."""
    path = os.path.join(ROBOTS, name + '.toml')
    if not os.path.isfile(path):
        raise MachineError('no robot %s: %s' % (name, ', '.join(sorted(
            f[:-5] for f in os.listdir(ROBOTS) if f.endswith('.toml')))))
    build, lib = _toml(path), library()
    for kind, stack in build['stacks'].items():
        for part, named in zip(PARTS, stack):
            if named not in lib[part]:
                raise MachineError('%s: %s %s is no %s in the library (%s)' % (
                    name, kind, named, part[:-1], ', '.join(lib[part])))
    return build
