#!/usr/bin/env python3
"""The rig held to the library: what a sheet states is read of its file and stated nowhere else.

The power stage's model and the board's scaling read the library's inverter, the shunt and the
switches' output capacitance stated there alone; the stage's switching rate, its period and its
envelope stand in no module as a number of that module's own, and a core's harness reads the
library as the file it is; the rig imported loads nothing of the robot's body, the robot's law or
the crafts; a motor the library and the rig's profiles both describe is one motor.
The solver's own laws: test_metacompiler; the crafts' spec: test_metacompiler_crafts."""
import ast
import glob
import json
import os
import subprocess
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from structure_kit import HOST, Report, sources

#: The power stage's names its model reads from the library, each the library's key or its solver.
STAGE = {'FSW': 'pwm_hz', 'RDS_ON': 'rds_ohm', 'RTH_JC': 'rth_jc', 'T_J_MAX': 'tj_max_c', 'SHUNT': 'shunt_ohm'}
#: The names a module holds the stage's switching rate or its period under, and the thermal core's envelope's.
RATES, ENVELOPE = ('PWM_HZ', 'FSW', 'TS'), ('LIMIT_C', 'BOARD_LIMIT_C', 'WINDING_LIMIT_C', 'LEG_LIMIT_C')
#: The drive's and the thermal core's harnesses imported first, and what they loaded of the board's package then.
CORES = ('import sys; from tools.cores import drive, thermal; '
         'print(" ".join(sorted(m for m in sys.modules if m.split(".")[0] == "coaxial")) or "none")')
#: A library motor's keys beside its rig profile's: the stand-in model's and the drive's record's.
PROFILED = (('ohm', 'r', 'motor_r'), ('lq_h', 'lq', 'motor_lq'), ('flux_vs', 'lambda', 'motor_lambda'),
            ('pole_pairs', 'pole_pairs', 'motor_pole_pairs'))
#: The rig imported in a process of its own, and what it has loaded of the robot's packages then.
RIG = ('import sys, coaxial; from coaxial.model import inverter, thermal; from coaxial.simulated import sensors; '
       'from tools.emu import world; '
       'print(" ".join(sorted(m for m in sys.modules if m.startswith(("machine.body", "machine.law", '
       '"machine.aircraft")))) or "none")')


def _literals(path, names):
    """Those of `names` a module's top level binds to a bare number, a tuple's own too."""
    with open(path, encoding='utf-8') as f:
        tree = ast.parse(f.read())
    bare = []
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        target, value = node.targets[0], node.value
        pairs = (zip(target.elts, value.elts)
                 if isinstance(target, ast.Tuple) and isinstance(value, ast.Tuple) else [(target, value)])
        bare += [t.id for t, v in pairs if isinstance(t, ast.Name) and t.id in names
                 and isinstance(v, ast.Constant) and isinstance(v.value, (int, float))]
    return bare


def test_the_power_stage_is_the_librarys(r):
    """`coaxial.model.inverter` and `thermal` state no number the library's inverter states: each
    is read of its file, and the share of the link the model gives the modulator is the solver's
    (`mechatronics.v_frac`)."""
    from coaxial.devices import scaling
    from coaxial.model import inverter, thermal
    from coaxial.simulated import values
    from coaxial.simulated.drive import capture
    from machine.body import mechatronics
    inv = mechatronics.library()['inverters']['coaxial_63100']
    bare = _literals(os.path.join(HOST, 'coaxial', 'model', 'inverter.py'), set(STAGE))
    r.check('the inverter model binds none of the library\'s numbers to a literal', not bare, bare)
    apart = [n for n, key in STAGE.items() if getattr(inverter, n) != float(inv[key])]
    r.check('its switching rate, resistances and junction limit are the library\'s',
            not apart and inverter.V_FRAC == mechatronics.v_frac(inv) and inverter.TS == 1.0 / float(inv['pwm_hz']),
            apart)
    r.check('the board\'s phase shunt as its scaling states it is the library\'s',
            scaling.PHASE_ONBOARD.r_shunt == scaling.ShuntParams().r_shunt == float(inv['shunt_ohm']),
            scaling.PHASE_ONBOARD.r_shunt)
    again = sorted('%s:%d' % (path.replace(os.sep, '/'), node.lineno) for path, _text, tree in sources(beside=False)
                   if path.replace(os.sep, '/') != 'coaxial/model/inverter.py' for node in ast.walk(tree)
                   if isinstance(node, ast.Constant) and node.value in (inverter.SHUNT, inverter.CJO))
    r.check("no module states the shunt or the switches' output capacitance again, the stand-in's amps a code are one",
            not again and values.AMPS_PER_CODE == capture.DriveCapture.APC
            and thermal.LOSSES['coss_cjo'] == inverter.CJO, again)
    switch, laminate, stator, winding = (float(c) for c in inv['ceiling_c'])
    ceilings = thermal.CEILING_C
    r.check('the thermal model\'s ceilings and its die\'s resistance are the library\'s',
            thermal.CEILING_DEFAULT_C == switch
            and all(ceilings[n] == laminate for n in thermal.LAMINATE)
            and all(ceilings[n] in (stator, winding) for n in thermal.MOTOR)
            and all(ceilings[n] == float(inv['tj_max_c']) and thermal.RTH_DIE[n] == float(inv['rth_jc'])
                    for n in thermal.DRIVERS), ceilings)


def _bound(tree, rate, names):
    """The lines where a module binds `rate` to a name, scales by it - a rate goes times or over a time,
    where an offset is added - or binds one of `names` to any bare number."""
    lines = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Assign):
            target, value = node.targets[0], node.value
            pairs = (list(zip(target.elts, value.elts))
                     if isinstance(target, ast.Tuple) and isinstance(value, ast.Tuple) else [(target, value)])
        elif isinstance(node, ast.BinOp) and isinstance(node.op, (ast.Mult, ast.Div, ast.FloorDiv)):
            pairs = [(None, node.left), (None, node.right)]
        else:
            continue
        for target, value in pairs:
            if (isinstance(value, ast.Constant) and isinstance(value.value, (int, float))
                    and not isinstance(value.value, bool)
                    and (float(value.value) == rate or getattr(target, 'id', getattr(target, 'attr', None)) in names)):
                lines.add(node.lineno)
    return lines


def test_the_stages_rate_and_envelope_are_stated_once(r):
    """The inverter's switching rate, its period and its nodes' ceilings stand in the library alone:
    no module binds the rate to a name, scales by it or holds a rate's name on a bare number; the
    stand-in's gate drivers and drive, the bench's pulse and the drive core's harness count by the
    library's; the thermal core's envelope is the library's ceilings and its junction's limit; the
    bootloader's line rate the emulator opens at is the master's transport's."""
    from coaxial.devices import boot
    from coaxial.model import inverter
    from coaxial.simulated import power
    from coaxial.simulated.drive import device
    from machine.body import mechatronics
    from tools.bench import pulse
    from tools.cores import drive, thermal as core
    from tools.emu import emulator
    inv = mechatronics.library()['inverters']['coaxial_63100']
    rate = float(inv['pwm_hz'])
    bound = sorted('%s:%d' % (path.replace(os.sep, '/'), line)
                   for path, _text, tree in sources(beside=False) for line in _bound(tree, rate, RATES))
    r.check("no module binds the stage's switching rate, scales by it or names a rate on a bare number",
            not bound, bound)
    rates = (power.SimulatedGateDrivers.PWM_HZ, device.SimulatedDrive.PWM_HZ, device.SimulatedDrive.FS, pulse.PWM_HZ)
    r.check("the stand-in's gate drivers and drive, the bench's pulse and the drive core count by the library's",
            all(held == rate for held in rates) and device.SimulatedDrive.TS == drive.TS == 1.0 / rate, rates)
    switch, laminate, _stator, winding = (float(c) for c in inv['ceiling_c'])
    envelope = tuple(getattr(core, name) for name in ENVELOPE)
    bare = _literals(os.path.join(HOST, 'tools', 'cores', 'thermal.py'), set(ENVELOPE))
    r.check("the thermal core's envelope is the library's ceilings and its junction's limit, none a literal",
            not bare and envelope == (switch, laminate, winding, float(inv['tj_max_c'])), (bare, envelope))
    out = subprocess.run([sys.executable, '-X', 'utf8', '-c', CORES], env=dict(os.environ, PYTHONPATH=HOST),
                         cwd=HOST, capture_output=True, text=True)
    r.check("the cores' harnesses read the library as a file: imported first they load none of the board's package",
            out.returncode == 0 and out.stdout.split() == ['none'], (out.stdout.split()[:4], out.stderr[-200:]))
    r.check("the bootloader's line rate the emulator opens at is the master's transport's, stated there alone",
            emulator.BOOT_BAUD == boot.BOOT_BAUD
            and not _literals(os.path.join(HOST, 'tools', 'emu', 'emulator.py'), {'BOOT_BAUD'}), emulator.BOOT_BAUD)


def test_the_rig_reads_the_specs_as_files(r):
    """The board's package, its models, its stand-in's sensors and the emulator's worlds read the
    library and the world as the files they are: a process that imports them has loaded nothing of
    the robot's body, the robot's law or the crafts (the atlas takes a page to what a module it
    imports runs: a rig page come to the robot's body breaks the pages apart, test_views_apps)."""
    out = subprocess.run([sys.executable, '-X', 'utf8', '-c', RIG], env=dict(os.environ, PYTHONPATH=HOST),
                         cwd=HOST, capture_output=True, text=True)
    loaded = out.stdout.split()
    r.check("the rig imported loads nothing of the robot's body, the robot's law or the crafts",
            out.returncode == 0 and loaded == ['none'], (loaded[:6], out.stderr[-200:]))


def test_a_motor_and_its_profile_are_one(r):
    """A motor the library and the rig both describe (`coaxial/profiles/<its name>.json`, its drive's
    record and its stand-in model) is one motor: its resistance, its q inductance, its flux and its
    pole pairs the same in both, its bearing drag the model's; the crafts' motor's rotor is the can
    the library's mass and diameter give; every profile's share of the link is the solver's."""
    from coaxial.devices.drive import PROFILES
    from machine.aircraft import compile as craft
    from machine.body import mechatronics
    lib, shared = mechatronics.library(), mechatronics.crafts()['shared']
    share = mechatronics.v_frac(lib['inverters'][shared['inverter']])
    both, apart, shares, rotor = [], [], [], None
    for path in sorted(glob.glob(os.path.join(PROFILES, '*.json'))):
        name = os.path.splitext(os.path.basename(path))[0]
        with open(path, encoding='utf-8') as f:
            profile = json.load(f)
        shares += [name] * (float(profile['drive']['drv_v_frac']) != share)
        motor = lib['motors'].get(name)
        if motor is None:
            continue
        both.append(name)
        apart += [(name, key) for key, model, drive in PROFILED
                  if not float(motor[key]) == float(profile['model'][model]) == float(profile['drive'][drive])]
        apart += [(name, 'viscous_nms')] * (float(motor['viscous_nms']) != float(profile['model']['b']))
        rotor = float(profile['model']['j']) if name == shared['motor'] else rotor
    r.check('a motor the library and the rig both describe is one: %s' % ', '.join(both), bool(both) and not apart, apart)
    r.check('the crafts\' motor\'s rotor is the can its mass and diameter give', rotor == craft.ROTOR_CAN_J,
            (rotor, craft.ROTOR_CAN_J))
    r.check('every profile\'s share of the link is the solver\'s', not shares, shares)


ROSTER = (test_the_power_stage_is_the_librarys, test_the_stages_rate_and_envelope_are_stated_once,
          test_the_rig_reads_the_specs_as_files, test_a_motor_and_its_profile_are_one)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
