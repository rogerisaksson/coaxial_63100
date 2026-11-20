"""Every flying craft's 3-axis loops solved of its own blocks (`aircraft.solved`).

    for machine, loop, kp, kd, solved_loop, needs, ceilings in attitude.rows():   # live beside solved

Every text that flies (`machine.aircraft.variants`) is compiled to its own blocks by kind
(`aerolisp.compile_manifest`) and its pitch, roll and yaw loops are solved by the kinds it has
(`machine.aircraft.solved`, `machine.solver.loops`) - no craft is named here, and every craft and
variant flies on its solved loops. Tilt (`pitch`, `roll`) stands at the least of its ceilings
(`loops.stiffest`); heading (`yaw`) is a first-order rate loop on multirotors (`loops.rate`) and a
second-order attitude loop at its stiffest need (`loops.design`) on 3D attitude crafts.
"""
import os

from machine import aircraft
from machine.aircraft import aerolisp, compile as craft, solved
from machine.body import mechatronics

DEAD_STEPS = solved.DEAD_STEPS
AXES = solved.AXES


def _loops(name, manifest, targets, pull_cfg, ref_radius):
    """A vehicle's rows (`rows`) of its compiled text: its pitch, its roll and its yaw."""
    aero, dynamics, kinematics = manifest['aerodynamics'], manifest['dynamics'], manifest['kinematics']
    today = dynamics['impedance']
    axes = solved.solve(aero, dynamics, kinematics, targets, pull_cfg, ref_radius, manifest.get('stunts'))
    return [(name, loop, today['stiffness']['attitude_' + loop], today['damping']['attitude_' + loop],
             axes[loop][0], axes[loop][1], axes[loop][2]) for loop, _key in AXES]


def rows(crafts=None):
    """`[(machine, loop, live stiffness 1/s^2, live damping 1/s, solved Loop, needs, ceilings)]` for
    every text that flies across `pitch`, `roll` and `yaw`; `crafts` a resolved crafts table compiled in
    the stock one's place (`compile.blocks`)."""
    targets, out = mechatronics.control(), []
    built = craft._compiled() if crafts is None else craft._build_aircraft(crafts)
    blocks = None if crafts is None else {
        k: (kind, {sk: dict(sv) if isinstance(sv, dict) else sv for sk, sv in val.items()})
        for k, (kind, val) in built['blocks'].items()}
    pull_cfg, ref_radius = built['pull'], built['blocks']['quad_x_16x14'][1]['disc_radius']
    for kind in aircraft.CRAFTS:
        for name in aircraft.variants(kind):
            with open(os.path.join(aircraft.LISP, name + '.lisp'), encoding='utf-8') as f:
                text = f.read()
            manifest = aerolisp.compile_manifest(text, blocks=blocks, pull_cfg=pull_cfg)
            out += _loops(name, manifest, targets, pull_cfg, ref_radius)
    return out
