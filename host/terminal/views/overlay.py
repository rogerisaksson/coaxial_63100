"""The humanoid page's callouts' inks and its data overlay (D): each drive and each bus.

Her standing, faced, every drive called out on its side (`data_labels`, `data_legend`) and
each bus's traffic in the side column (`traffic`), off the running process's state.
"""
from machine import gait
from machine.routines import TYPES
from terminal.ui.stage import hud

#: A callout's inks: its ground, the torque's, the power's driving and braking, the legend's
#: words, a number on a dark patch and on a light; its width inside its frame, cells.
BOX_GROUND, TORQUE_INK, DRIVE_INK, BRAKE_INK, NUMBER_INK, DARK_INK, LIGHT_INK = (
    (38, 46, 58), (255, 184, 80), (96, 214, 255), (255, 96, 128), (214, 220, 228),
    (16, 18, 22), (236, 240, 244))
CALLOUT_W = 4

#: The data overlay (D): her standing, faced, her arms out in a T, each drive called out on its
#: side narrow and tall - its name in its frame, its angle deg and torque N m, its heat C and
#: state (ON, derated, OFF) under - and each bus's traffic in the side column, its wire's share
#: of BAUD both ways. A row across, 21 cells, the callouts covered her.
DATA_W, BAUD = 7, 9216000
#: Her drawing's share of the view's zoom under the callouts, docked around her.
SMALL = 0.7
SHORT = {'hip_yaw': 'HIPY', 'hip_roll': 'HIPR', 'hip': 'HIP', 'knee': 'KNEE', 'ankle': 'ANKL',
         'ankle_roll': 'ANKR', 'foot': 'TOES', 'shoulder': 'SHLD', 'elbow': 'ELBW',
         'wrist': 'WRST', 'gripper': 'GRIP', 'spine': 'SPIN', 'spine_roll': 'SPNR',
         'waist': 'WAIS', 'neck': 'NECK', 'head': 'HEAD'}
ON_INK, DERATED_INK, OFF_INK = (96, 220, 120), (255, 184, 80), (255, 96, 128)
STAND, STANDING = dict(gait.stand(), arms_out=84.0, left_elbow=0.0, right_elbow=0.0), gait.standing()


def data_labels(now):
    """{joint: rows} for `gynoid.render`: each drive's row, the data overlay's."""
    out = {}
    for joint, (celsius, _spent, derate, on) in now['heat'].items():
        name = SHORT.get(joint.split('_', 1)[-1] if joint.startswith(('left_', 'right_')) else joint,
                         joint[:4].upper())
        state = (OFF_INK, 'x') if not on else (DERATED_INK, '~') if derate < 0.99 else (ON_INK, '*')
        top = '%3.0f %3.0f' % (now['angles'].get(joint, 0.0), now['torque'][joint])
        under = '%3.0fC  ' % celsius
        out[joint] = [name, [(c, NUMBER_INK, BOX_GROUND) for c in top[-DATA_W:]],
                      [(c, NUMBER_INK, BOX_GROUND) for c in under[:DATA_W - 1]]
                      + [(state[1], state[0], BOX_GROUND)]]
    return out


def data_legend(width):
    """The data overlay's last row: its columns, and its states' marks in their inks."""
    cells = [(c, NUMBER_INK, None) for c in ' D: DEG N m | C  ']
    for ink, mark, word in ((ON_INK, '*', 'ON'), (DERATED_INK, '~', 'DERATED'),
                            (OFF_INK, 'x', 'GATES OFF')):
        cells += [(mark, ink, None)] + [(c, NUMBER_INK, None) for c in ' %s  ' % word]
    return cells[:width]


def traffic(state, now):
    """The side column's BUSES box: each bus's bytes a second out and in, its wire's share,
    its bad frames - from the counts' change since the last frame."""
    rows, was = [], state.get('traffic')
    for k, (joints, out, got, bad) in enumerate(now.get('buses', [])):
        name = next((s.name for s in TYPES['gynoid'].body if joints[0] in s.actuators), '?')
        rows.append(('MODBUS #%d' % (k + 1), '%s, %d DRIVES' % (name.replace('_', ' ').upper(),
                                                              len(joints))))
        if was is not None and now['t'] > was[0] and k < len(was[1]):
            dt = now['t'] - was[0]
            up, down = (out - was[1][k][0]) / dt, (got - was[1][k][1]) / dt
            rows.append('OUT %3.0f  IN %3.0f kB/s  %2.0f%%  %d BAD' % (
                up / 1e3, down / 1e3, 100.0 * (up + down) * 10.0 / BAUD, bad))
        else:
            rows.append('-')
    state['traffic'] = (now['t'], [(out, got) for _j, out, got, _b in now.get('buses', [])])
    return hud('BUSES', rows or [('BUSES', 'NONE IN THIS WORLD')])
