"""A mocap take as humanoid G-code: the general rig's program written, and hers converted beside it.

    python tools/sim/gcode.py lying/Lying\\ Down\\ from\\ Sitting      # programs/lying_down_from_sitting[.general].gcode
    python tools/sim/gcode.py --report lying/Sitting\\ on\\ Bed        # the calibration and the conversion's report

The take's joints (`fbx.take`) go through `machine.texts.gcode.rig` to the general rig's words a frame, written
with the take's name, rate and leg; `gcode.hers` converts that to her joints within her ranges and speeds, the
report in her program's header (dropped words, clamped joints, the slowest move's stretch). A program is named
as a stunt is (`scenarios.STUNTS`): the take's file name in lower case with underscores. `--program` writes one
of her own programs (`joints.PROGRAMS`) as G-code, a move a beat's end; `--all` every take under stunts/, a line
each, into the host's gcode/.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np  # behind the OpenBLAS cap
from machine.texts import gcode
from tools.sim import fbx, stunts


def named(take):
    """A program's name of a take `folder/Name`: the take's file name in lower case with underscores."""
    return take.split('/')[-1].lower().replace(' ', '_')


def general(take):
    """(header, times, rows) of a take `folder/Name` on the general rig: its joints' places a frame (`fbx.take`)
    through `gcode.rig`, the rate and leg in the header."""
    times, joints = fbx.take(os.path.join(stunts.TAKES, take + '.fbx'))
    times = np.asarray(times, float) - float(times[0])
    rows = gcode.rig(joints)
    hz = (len(times) - 1) / float(times[-1]) if len(times) > 1 and times[-1] > 0 else 0.0
    header = {'humanoid': 'gcode 1', 'rig': 'general', 'take': take, 'hz': '%.3f' % hz, 'leg': '%.3f' % gcode.leg(joints)}
    return header, times, rows


def written(take, out=gcode.DIR):
    """Both programs of `take` written under `out`: (the general's path, hers)."""
    os.makedirs(out, exist_ok=True)
    header, times, rows = general(take)
    name = named(take)
    paths = (os.path.join(out, name + '.general.gcode'), os.path.join(out, name + '.gcode'))
    with open(paths[0], 'w', encoding='utf-8', newline='\n') as f:
        f.write(gcode.text(header, times, gcode.WORDS, rows))
    mine, t, joints, deg = gcode.hers(header, times, rows)
    with open(paths[1], 'w', encoding='utf-8', newline='\n') as f:
        f.write(gcode.text(mine, t, joints, deg))
    return paths


def exported(name, out=gcode.DIR):
    """Her program `name` (`joints.PROGRAMS`) written as G-code under `out`: a move a beat's end, its pose the
    beat's joints; the path."""
    from machine.process import joints
    os.makedirs(out, exist_ok=True)
    beats = joints.PROGRAMS[name]
    names = sorted({j for _b, _s, goals in beats for j in goals})
    times, rows, pose, t = [], [], {}, 0.0
    for _beat, seconds, goals in beats:
        t += seconds
        pose.update(goals)
        times.append(t)
        rows.append([pose.get(j, float('nan')) for j in names])
    path = os.path.join(out, name + '.gcode')
    with open(path, 'w', encoding='utf-8', newline='\n') as f:
        f.write(gcode.text({'humanoid': 'gcode 1', 'rig': 'hers', 'program': name}, times, names, np.array(rows)))
    return path


def takes():
    """Every take under `stunts.TAKES`, `folder/Name`, in order."""
    out = []
    for folder in sorted(os.listdir(stunts.TAKES)):
        here = os.path.join(stunts.TAKES, folder)
        if os.path.isdir(here):
            out += [folder + '/' + f[:-4] for f in sorted(os.listdir(here)) if f.endswith('.fbx')]
    return out


def line(take, out=gcode.DIR):
    """One line of `take` written (`written`): its name, moves, seconds, what was clamped and slowed - or why it
    could not be."""
    try:
        _general, mine = written(take, out)
        header, times, _words, _values = gcode.read(mine)
        return '%-28s %4d moves %6.2f s stretched %s slowed %s clamped %s' % (
            named(take), len(times), times[-1] if len(times) else 0.0, header.get('stretched'), header.get('slowed'), header.get('clamped'))
    except (KeyError, ValueError, OSError) as error:
        return '%-32s refused: %s' % (named(take), error)


def report(take):
    """Lines: the calibration (a word, her joint, its gain) and the conversion's header of `take`."""
    table, at, _base = gcode.calibrated()
    lines = ['%-4s %-16s gain %6.2f  at base %7.2f' % (w, j, g, at[gcode.WORDS.index(w)]) for w, (j, g) in table.items()]
    header, times, rows = general(take)
    mine, t, joints, deg = gcode.hers(header, times, rows)
    lines += ['%s %s' % kv for kv in mine.items()]
    lines += ['%d moves, %.2f s (the take %.2f s), %d of her joints' % (len(t), t[-1] if len(t) else 0.0, times[-1] if len(times) else 0.0, len(joints))]
    return lines


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
    parser.add_argument('takes', nargs='*', help='takes, folder/Name (under stunts/)')
    parser.add_argument('--program', action='append', default=[], help='one of her programs written as G-code')
    parser.add_argument('--all', action='store_true', help='every take under stunts/, a line each')
    parser.add_argument('--out', default=gcode.DIR, help='where the programs are written')
    parser.add_argument('--report', action='store_true', help='the calibration and the conversion, no files')
    args = parser.parse_args(argv)
    for name in args.program:
        print(exported(name, args.out))
    for take in (takes() if args.all else []):
        print(line(take, args.out))
    for take in args.takes:
        if args.report:
            print('\n'.join(report(take)))
        else:
            print('\n'.join(written(take, args.out)))


if __name__ == '__main__':
    main()
