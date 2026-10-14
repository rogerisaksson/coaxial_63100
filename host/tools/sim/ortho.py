#!/usr/bin/env python3
"""Orthokinematics (`machine.ortho`) run on her latent loop as the page's gym runs it, traced.

    python tools/sim/ortho.py hop_asymmetric              # an entry of machine/gynoid_mesomorph_medium.lisp
    python tools/sim/ortho.py crouch -v                   # a row every 25 ms; -v 5, every 5
    python tools/sim/ortho.py "#101" "#102@3" "#103@6" --to 8    # each at its second
    python tools/sim/ortho.py --walk --to 40 --look              # her walk from rest (`sprung.WALK`): its look
    python tools/sim/ortho.py --file my.lisp hop --every 4       # a file's; the loop every 4th pass

She stands AT_S at rest on the loop (`machine.sprung`: sensed, `machine.latent`, `machine.impedance`,
`wrench.step`); each entry named - a payload '#' and its id, a behaviour by its name - is begun at
its second, AT_S and then 2 s apart where none is given; she is followed `--to` s past the last.
A row: what she is doing, each mode as asked and as she is, her centre of mass's height and
speed up, each sole's load, which soles stand, her speed along. The last line: whether she
stood; each mode as asked, at its furthest, as the last asked ended and at the run's end; her
soles' loads then; her greatest speed up; her seconds off the floor; how far she went, how fast
over her last GOING_S, and her soles' landings. `--look`: her walk from the last entry's blend
on as tools/sim/look.py measures one - its speed and J/m, how far off a woman's band it is and
what of it is out (`normal.BAND`), its words and the gaits it is by name, what of her form it
breaks (`looks.FORM`) -; `--fbx` writes it a take (tools/sim/mocap.py).
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: She stands this long before anything is asked, s; followed TO_S past the last asked; her
#: speed along is hers over her last GOING_S. Fallen past HELD_DEG of her pelvis's tilt, or
#: with her centre of mass under LOW of where it stood.
AT_S, TO_S, GOING_S, HELD_DEG, LOW = 1.0, 2.0, 3.0, 25.0, 0.5
#: A sole bears nothing under this, N.
AIR_N = 5.0


def went(plan, to=TO_S, trace=0, every=None):
    """{stood, t, tilt, modes {mode: (asked, furthest, as the last asked ended, at the end)},
    loads then and at the end (N), up m/s, air s, went m, speed m/s, landings, us a pass,
    rows}: `plan` [(s, label, wire, names)] (`sprung.entries`) - each begun at its second -
    from a stand on the loop, followed `to` s past the last; `trace` a row every that many ms;
    the loop solved every `every` passes, the page's (`sprung.STACK_EVERY`) with none."""
    import time
    import numpy as np
    from machine import latent, modal, ortho, sprung
    from machine.errors import MachineError
    g = sprung.laid(every or sprung.STACK_EVERY)
    for at, label, wire, names in sorted(plan, key=lambda item: item[0]):
        sprung.said(g, label, wire, names, at)
    rows = [(at, row) for at, _label, wire, _names in plan for row in ortho.rows(wire)]
    last = max(at + 1e-3 * (row[1] + row[2]) for at, row in rows)
    # a behaviour has ended as its row back to rest comes, a payload as it is blended in
    ended = max(at + 1e-3 * (row[1] + row[2] * (1 - row[3] % 2)) for at, row in rows)
    n, first, home = ortho.N, min(item[0] for item in plan), float(g['y'][modal.COM][1])
    rec = {'far': np.zeros(n), 'asked': np.zeros(n), 'ended': np.zeros(n), 'z': np.zeros(n), 'up': 0.0,
           'air': 0.0, 'rows': [], 'loads': (0.0, 0.0), 'track': [], 'landings': 0,
           'stood': (True, True), 'tilt': 0.0, 'failed': None}
    along, began = g['fr'][modal.ROT][:3], time.perf_counter()
    while g['t'] < last + to and rec['tilt'] <= HELD_DEG and g['y'][modal.COM][1] > LOW * home:
        now = g['t']
        try:
            s, loads, ask, out = sprung.passed(g)
        except MachineError as error:
            rec['failed'] = '%s at %.3f s' % (error, now)
            break
        z = modal.read(g['y'], g['fr'], g['a'][modal.STANCE])[0]
        rec['z'], rec['tilt'] = z, max(rec['tilt'], sprung.tilt(g))
        if now <= ended:
            rec['asked'], rec['ended'], rec['loads'] = g['st'][latent.TO][latent.Z].copy(), z, tuple(loads)
        if now >= first:
            rec['far'] = np.where(np.abs(z) > np.abs(rec['far']), z, rec['far'])
        rec['up'] = max(rec['up'], float(s['vcom'][1]))
        rec['air'] += 0.001 * (max(loads) < AIR_N)
        rec['landings'] += sum(bool(stands) and not was for stands, was in zip(ask['stance'], rec['stood']))
        rec['stood'] = ask['stance']
        rec['track'].append(float(s['com'] @ along))
        if trace and int(round(now * 1000)) % trace == 0:
            rec['rows'].append('%6.3f %-10s asked %s | is %s | y %.3f vy %+5.2f | N %4.0f %4.0f %s | v %+.2f' % (
                now, sprung.doing(g), ' '.join('%+.2f' % v for v in out[latent.REF]),
                ' '.join('%+.2f' % v for v in z), s['com'][1], s['vcom'][1], loads[0], loads[1],
                ''.join('S' if stands else '-' for stands in ask['stance']), float(s['vcom'] @ along)))
    track, loads = rec['track'], g['loads']
    tail = track[-int(GOING_S * 1000):]
    return {'stood': rec['failed'] is None and g['t'] >= last + to - 1e-6, 'failed': rec['failed'],
            't': round(g['t'], 3), 'tilt': round(rec['tilt'], 1),
            'us': round(1e6 * (time.perf_counter() - began) / max(1, len(track))),
            'modes': {m: tuple(round(float(v), 3) for v in row) for m, *row in zip(
                ortho.MODES, rec['asked'], rec['far'], rec['ended'], rec['z'])},
            'loads': tuple(round(v) for v in rec['loads']), 'rest': tuple(round(v) for v in loads),
            'up': round(rec['up'], 3), 'air': round(rec['air'], 3),
            'went': round(track[-1] - track[0], 3) if track else 0.0,
            'speed': round((tail[-1] - tail[0]) / (len(tail) * 0.001), 3) if len(tail) > 1 else 0.0,
            'landings': rec['landings'], 'rows': rec['rows']}


def looked(plan, to=TO_S, take=None, every=None):
    """Her walk on the loop as the eye sees it: {stood, t, speed m/s, energy J/m, off (a woman's
    band), out [(measure, value, bound)], words [(word, how far)], named [gait], broken [(measure,
    value, bound)] of `looks.FORM`, measures (`strides.WALK`'s), band (`normal.BAND`'s)} - the
    rows the page records (`humanoid_keys.row`), `look.RATE_HZ` a second, walking from the last
    of `plan` blended in; written a take at `take`. Empty of measures where she walked under
    3 s."""
    import math
    from machine import latent, ortho, sprung
    from machine.errors import MachineError
    from machine.figure import JOINTS
    from terminal.views.humanoid_keys import HEADER, row
    from tools.sim import fbx, look, looks, normal, strides
    g = sprung.laid(every or sprung.STACK_EVERY)
    for at, label, wire, names in sorted(plan, key=lambda item: item[0]):
        sprung.said(g, label, wire, names, at)
    walks = max(at + 1e-3 * (r[1] + r[2]) for at, _label, wire, _names in plan for r in ortho.rows(wire))
    world = g['world']
    d, rows, told, stood = world.data, [], -1.0, True
    while g['t'] < walks + to and stood:
        try:
            sprung.passed(g)
        except MachineError:
            stood = False
        stood = stood and sprung.tilt(g) <= sprung.DOWN_DEG and d.qpos[1] >= sprung.DOWN_M
        if g['t'] - told >= 1.0 / look.RATE_HZ:
            told = g['t']
            angles = {j: math.degrees(float(d.qpos[world.qadr[i]])) for i, j in enumerate(JOINTS)}
            now = {'t': g['t'], 'stage': 'walk' if g['t'] >= walks else 'stand', 'speed': float(d.qvel[2]),
                   'phase': float(g['st'][latent.THETA]) / math.tau, 'loads': tuple(float(v) for v in g['loads']),
                   'where': tuple(d.qpos[0:3].tolist()), 'turn': tuple(d.qpos[3:7].tolist()),
                   'angles': angles, 'set': angles, 'watts': world.drawn()}
            rows.append(dict(zip(HEADER, row(now, 60.0)), loose=world.loose(),
                             feet=world.gap(look.LEFT_FOOT, look.RIGHT_FOOT),
                             lifted=world.lifted(look.LEFT_FOOT), landed=True,
                             trunk=world.gap(('torso', 'head', 'upper_arm', 'forearm', 'hand'),
                                             ('thigh', 'shank')),
                             ahead=look._ahead(world), down='', laid=None, **look._held(world)))
    out: dict = {'stood': stood, 't': round(g['t'], 3)}
    walk = strides.steady(rows)
    if len(walk) < 3 * look.RATE_HZ:
        return out
    got = strides.measured(walk)
    band = normal.measured(*fbx.joints(walk))
    off, wide = normal.off(band)
    if take:
        fbx.wrote(take, walk)
    out.update(speed=(float(walk[-1]['z']) - float(walk[0]['z'])) / (float(walk[-1]['t']) - float(walk[0]['t'])),
               energy=got.get('energy', math.nan), off=off, out=wide, words=normal.said(band),
               named=normal.named(band), broken=looks.broken(got), measures=got, band=band)
    return out


def planned(asked, text=None):
    """[(s, label, wire, names)] of `asked` ['label' or 'label@seconds', ..] among `text`'s
    entries (machine/gynoid_mesomorph_medium.lisp's): AT_S, then 2 s apart, where no second is given."""
    from machine import sprung
    gym = {entry[0]: entry for entry in sprung.entries(text)}
    plan, at = [], AT_S
    for word in asked:
        label, _, when = word.partition('@')
        if label not in gym:
            raise KeyError('no entry %s: %s' % (label, ', '.join(gym)))
        at = float(when) if when else at
        plan.append((at,) + gym[label])
        at += 2.0
    return plan


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('what', nargs='*', help="entries: a behaviour's name or '#' and a payload's id, "
                        'each alone or @ its second')
    parser.add_argument('--walk', action='store_true', help='her walk from rest (`sprung.WALK`)')
    parser.add_argument('--file', help="a text's entries, not machine/gynoid_mesomorph_medium.lisp's")
    parser.add_argument('--to', type=float, default=TO_S, help='s followed past the last asked')
    parser.add_argument('--every', type=int, help='the loop solved every that many passes; the page has its own')
    parser.add_argument('-v', nargs='?', type=int, const=25, default=0, metavar='MS',
                        help='a row every MS ms (25)')
    parser.add_argument('--look', action='store_true', help="her walk's look: its band, its words, its form")
    parser.add_argument('--fbx', help='with --look: her walk written a take here')
    args = parser.parse_args(argv)
    text = None
    if args.file:
        with open(args.file, encoding='utf-8') as f:
            text = f.read()
    from machine import sprung
    asked = ['%s@%g' % entry for entry in sprung.WALK] * args.walk + args.what
    if not asked:
        parser.error('an entry, or --walk')
    try:
        plan = planned(asked, text)
    except KeyError as error:
        parser.error(error.args[0])
    if args.look:
        got = looked(plan, args.to, args.fbx, args.every)
        print('%s %.1f s' % ('stood' if got['stood'] else 'DOWN', got['t']))
        if 'off' in got:
            print('  %.2f m/s, %.0f J/m' % (got['speed'], got['energy']))
            print("  off a woman's walk %.2f%s" % (got['off'], ''.join(' | %s %.3g (%g)' % o for o in got['out'])))
            print('  in words: %s%s' % (', '.join('%s %.2f' % w for w in got['words']) or 'none',
                                        ''.join(' - ' + n for n in got['named'])))
            print('  her form: %s' % (', '.join('%s %.3g (%g)' % b for b in got['broken']) or 'held'))
            m = got['measures']
            print('  a landing: %.2f of her weight, %.2f after, a fall of %.0f mm; her head nods %.0f deg' % (
                m['landing load'], m['unloading'], m['dip'], m['head nod']))
        return 0 if got['stood'] else 1
    got = went(plan, args.to, args.v, args.every)
    for row in got.pop('rows'):
        print(row)
    print(json.dumps(got))
    return 0 if got['stood'] else 1


if __name__ == '__main__':
    sys.exit(main())
