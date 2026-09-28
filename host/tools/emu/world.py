"""A machine's world for the emulator: board/emu/worlds/<name>.json, as the plant's commands.

    python tools/emu/emulator.py --world quad --nodes 4    # four boards on the quad's rotors
    Coaxial63100(port='emulator://?world=ebike', ...)

A world names its body (ground, lift, vehicle) and a node a board: its motor, a profile in
coaxial/profiles (its `model` part, drive_model's terms), and its load (free, joint, rotor,
wheel). The world core's library (world/, built for this host) is what the emulated boards'
plants step (board/emu/Coaxial63100_Plant.cs); this module only reads the files and says them
to the monitor.
"""
import glob
import json
import math
import os
import sys

from coaxial.simulated.values import DCBUS_V
from tools import REPO
from tools.cores.build import build, find_cc

WORLDS = os.path.join(REPO, 'board', 'emu', 'worlds')
PROFILES = os.path.join(REPO, 'host', 'coaxial', 'profiles')
#: Where the plant hangs: TIM1_CH1, PE9.
PLANT = 'sysbus.gpioPortE.plant'
#: Where the STO chain hangs: KEEPALIVE, PA10.
STO = 'sysbus.gpioPortA.sto'
#: Where the front end hangs: AFE_ON, PB2.
AFE = 'sysbus.gpioPortB.afe'
#: The link a board without a world sits on, V: the stand-in's. A world's plant sets its own.
LINK_VOLTS = DCBUS_V

SOURCES = [os.path.join(REPO, 'world', 'src', name)
           for name in ('world.c', 'world_emu.c', 'world_heat.c', 'world_sto.c')] + [
    os.path.join(REPO, 'drive', 'src', name)
    for name in ('drive.c', 'drive_math.c', 'drive_model.c', 'drive_observer.c')] + [
    os.path.join(REPO, 'thermal', 'src', name) for name in ('thermal.c', 'thermal_ident.c')]
INCLUDES = [os.path.join(REPO, part) for part in ('world/inc', 'drive/inc', 'thermal/inc')]

#: world.h's enums by the names the files use.
BODIES = {'ground': 0, 'lift': 1, 'vehicle': 2}
LOADS = {'free': 0, 'joint': 1, 'rotor': 2, 'wheel': 3}
#: What a file leaves out, in world.h's order.
BODY = (('mass', 0.0), ('gravity', 9.81), ('slope_deg', 0.0), ('crr', 0.0), ('cda', 0.0),
        ('rho', 1.2), ('rider', 0.0), ('kick', 0.0), ('duty', 0.0))
LOAD = (('gear', 1.0), ('inertia', 0.0), ('mass', 0.0), ('arm', 0.0), ('damping', 0.0),
        ('k_drag', 0.0), ('k_thrust', 0.0), ('radius', 0.0), ('angle_deg', 0.0))
MOTOR = ('r', 'ld', 'lq', 'lambda', 'pole_pairs', 'j', 'b', 'vdc', 'noise')


def names():
    """The worlds there are."""
    return sorted(os.path.splitext(os.path.basename(p))[0]
                  for p in glob.glob(os.path.join(WORLDS, '*.json')))


def load(name):
    """A world by name, or a refusal naming them."""
    path = os.path.join(WORLDS, name + '.json')
    if not os.path.exists(path):
        raise ValueError('no world %r - there are %s' % (name, ', '.join(names())))
    with open(path, encoding='utf-8') as f:
        return json.load(f)


def library():
    """The world core built for this host, once a process under a name of its own: a Renode
    holding one build's library keeps it locked, and a body's limbs load the one."""
    if _BUILT:
        return _BUILT[0]
    sweep()
    _BUILT.append(build(find_cc(), SOURCES, INCLUDES, 'world_emu_%d' % os.getpid())[0])
    return _BUILT[0]


def sweep():
    """The world libraries of processes that have ended, removed. Every one not locked was: Linux
    locks none, and a test group's sweep unlinked another's before its Renode loaded it - the
    Renode exited on a DllNotFoundException (CI, 2026-09-27 and 28)."""
    for path in glob.glob(os.path.join(REPO, 'build', 'hosttest', 'world_emu_*')):
        pid = os.path.splitext(os.path.basename(path))[0].rsplit('_', 1)[-1]
        if pid.isdigit() and not _running(int(pid)):
            try:
                os.remove(path)
            except OSError:
                pass                     # held by its Renode, still running


def _running(pid):
    """Whether process `pid` still runs: on Windows by its handle's exit code - os.kill there
    ends it - elsewhere by signal 0."""
    if sys.platform == 'win32':
        import ctypes
        kernel = ctypes.WinDLL('kernel32', use_last_error=True)
        handle = kernel.OpenProcess(0x1000, False, pid)     # PROCESS_QUERY_LIMITED_INFORMATION
        if not handle:
            return ctypes.get_last_error() == 5             # ERROR_ACCESS_DENIED: another's
        code = ctypes.c_ulong()
        known = kernel.GetExitCodeProcess(handle, ctypes.byref(code))
        kernel.CloseHandle(handle)
        return not known or code.value == 259               # STILL_ACTIVE
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    except PermissionError:
        return True
    return True


_BUILT = []


def _decimal(value):
    """A number as the monitor reads one: no exponent - `2e-05` overflowed its
    integer tokenizer and Renode exited (2026-09-25)."""
    text = ('%.12f' % value).rstrip('0')
    return text + '0' if text.endswith('.') else text


def _values(given, table):
    out = []
    for key, default in table:
        value = float(given.get(key, default))
        out.append(math.radians(value) if key.endswith('_deg') else value)
    return out


def _motor(name):
    with open(os.path.join(PROFILES, name + '.json'), encoding='utf-8') as f:
        model = json.load(f)['model']
    return [float(model[key]) for key in MOTOR]


def commands(world, node, lib, first):
    """The monitor's lines for the board at `node` (0-based) in `world`: the world itself set
    up by the `first`."""
    nodes = world['nodes']
    out = []
    if first:
        body = world.get('body', {})
        out += ['%s World "%s" %d' % (PLANT, lib.replace(os.sep, '/'), len(nodes)),
                '%s Body %d %s' % (PLANT, BODIES[body.get('kind', 'ground')],
                                   ' '.join(_decimal(v) for v in _values(body, BODY)))]
    if node >= len(nodes):
        return out
    load_ = nodes[node].get('load', {})
    out += ['%s Node %d' % (PLANT, node),
            '%s Load %d %s' % (PLANT, LOADS[load_.get('kind', 'free')],
                               ' '.join(_decimal(v) for v in _values(load_, LOAD))),
            '%s Motor %s' % (PLANT, ' '.join(_decimal(v) for v in _motor(nodes[node]['motor'])))]
    return out
