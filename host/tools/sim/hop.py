#!/usr/bin/env python3
"""A hop on the whole-body stack: thrown, flown and landed, her soles released by `machine.flight`.

    python tools/sim/hop.py                  # one hop standing: flight, apex, landing, stood
    python tools/sim/hop.py --push 18 -v     # the throw's ask, m/s^2; traced 10 ms a row
    python tools/sim/hop.py --core           # on the loop in C (`tools.cores.wbc.stack_step`)
    python tools/sim/hop.py --wrench         # on the step without an active set (`machine.wrench`)
    python tools/sim/hop.py --hops 3         # that many, one after another
    ... --load 0.12 0.6 --push-s 0.2         # the load's depth m and time s, the throw's s
    ... --held                               # her soles never released: the stack as it was
    ... --timed                              # her throw asked to its time's end, no spring
    ... --upright                            # her trunk asked upright through the throw

The ask is the stack's own (`wbc.step`): her stand's law (`tools.sim.wbc.law`) about her soles'
middle; her height let down LOAD_M over LOAD_S, her posture crouched with it and leant until
her centre of mass is over her soles (`crouched`); then thrown PUSH_S - her centre of mass on
a spring to RELEASE_M under her standing height, no stronger than `--push` m/s^2 -, her
posture back the way it came, her trunk's turn unasked; then her stand's law again. Nothing of it names the flight: her soles leave the floor bearing nothing,
`flight.step` asks each to its landing and stands it again as it bears. She flies while
neither bears `flight.TOUCH_N`.
`tools.sim.wbc.stand`'s world, no buses.
"""
import argparse
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The hop's phases: stood AT_S, her height let down LOAD_M over LOAD_S, thrown PUSH_S, and
#: SETTLE_S to land and stand before the next; her throw asked PUSH m/s^2 up.
AT_S, LOAD_M, LOAD_S, PUSH_S, SETTLE_S, PUSH = 0.5, 0.10, 0.5, 0.15, 1.5, 14.0

#: Her throw is a spring to RELEASE_M under her standing height, RELEASE_K 1/s^2: her legs
#: follow her to knees of 22-31 deg (9-12 rad/s there). Asked to its time's end she left with
#: 4.3-5.0 kg m^2/s about her - the loop in C 13.6 deg over, down at 56 mm; the spring 4.6.
RELEASE_M, RELEASE_K = 0.0175, 700.0

#: A flight is one that lasts FLOWN passes: her soles' loads cross `flight.TOUCH_N` twice
#: in the 3 ms they leave the floor in.
FLOWN = 20


def _eased(u):
    u = max(0.0, min(1.0, u))
    return u * u * (3.0 - 2.0 * u)


def _ahead(posture, names):
    """Her centre of mass ahead of her ankles along her soles, m, in a posture (rad, `names`)."""
    from machine import figure
    degrees = {n: math.degrees(float(q)) for n, q in zip(names, posture)}
    placed = figure.frames(degrees, (0.0, 0.0, 0.0), figure.IDENT)
    com = figure.com(degrees, (0.0, 0.0, 0.0), figure.IDENT)
    feet = [placed[seg[0]] for seg in figure.SEGMENTS if seg[0].endswith('_foot')]
    return sum(figure.apply(figure.t(turn), figure.sub(com, at))[2] for at, turn in feet) / len(feet)


def crouched(posture, names, deg):
    """`posture` with each knee flexed `deg`, its hip and its ankle half that - her pelvis let
    down over her ankles -, then leant at the ankles until her centre of mass is over her soles
    where it was: straight down it lay 26 mm behind their middle at a 46 deg crouch. The
    movements by their terms (`machine.terms`)."""
    from machine import terms

    def bent(lean):
        out = posture.copy()
        for limb in (0, 1):
            for part, term, by in (('hip', 'flexion', 0.5 * deg), ('knee', 'flexion', deg),
                                   ('ankle', 'dorsiflexion', 0.5 * deg + lean)):
                kind, sign, _low, _high = terms.TERMS[(part, term)]
                out[names.index(terms.joint(limb, kind))] += sign * math.radians(by)
        return out

    stood = _ahead(posture, names)
    lean, step = 0.0, 1.0
    off = _ahead(bent(lean), names) - stood
    for _ in range(6):
        other = _ahead(bent(lean + step), names) - stood
        if abs(other - off) < 1e-9:
            break
        lean, step, off = lean + step, -other * step / (other - off), other
    return bent(lean)


def hop(push=PUSH, hops=1, core=False, trace=False, fly=True, inner='pd', free=True, stack=False,
        spring=True, wrench=False):
    """{stood, flights, flight_s, apex_mm, clear_mm, landed_n, tilt, stances, ..}: `hops` hops
    standing, her throw `push` m/s^2. `fly` False, her soles are never released - the stack as
    it was; `free` False, her trunk is asked upright through the throw; `spring` False, her throw
    is asked to its time's end; `wrench`, on `machine.wrench`'s step; `trace` a row every 10 ms,
    `stack` the stand's own rows."""
    from machine import balance, flight
    from machine.gait import SHANK, THIGH
    from tools.sim import wbc as sim
    period = LOAD_S + PUSH_S + SETTLE_S
    rec = {'air': 0, 'apex': 0.0, 'clear': 0.0, 'landed': 0.0, 'flights': 0, 'run': 0,
           'home': 0.0, 'stances': [], 'rows': []}
    thighs = []
    knee = 2.0 * math.degrees(math.acos(1.0 - LOAD_M / (THIGH + SHANK)))

    def asked(st, s, loads, now, bears, body):
        if not thighs:
            thighs.extend(body.m.body(side + '_thigh').id for side in ('left', 'right'))
            rec['home'] = float(s['com'][1])
        soles = [f['sole'] for f in s['feet']]
        ask = sim.law(s, st, (soles[0][[0, 2]] + soles[1][[0, 2]]) / 2.0)
        t = (now - AT_S) % period if AT_S <= now < AT_S + hops * period else -1.0
        acc = ask['com_acc'].copy()
        if 0.0 <= t < LOAD_S:
            u = _eased(t / LOAD_S)
            acc[1] = (balance.HEIGHT_KD[0] * (st['height'] - LOAD_M * u - s['com'][1])
                      - balance.HEIGHT_KD[1] * s['vcom'][1])
            ask['posture'] = crouched(st['posture'], sim.NAMES, knee * u)
        elif LOAD_S <= t < LOAD_S + PUSH_S:
            # her posture back up the way it came down; her trunk's turn unasked: asked upright
            # the stack put her weight on her heels' edges to turn her legs under it, and she
            # left the floor with 4.7 kg m^2/s about her, 14 deg over as she landed
            let = RELEASE_K * (st['height'] - RELEASE_M - s['com'][1])
            acc[1] = max(-flight.G, min(push, let)) if spring else push
            ask['posture'] = crouched(st['posture'], sim.NAMES,
                                      knee * (1.0 - _eased((t - LOAD_S) / PUSH_S)))
            if free:
                ask['turns'] = ()
        ask['com_acc'] = acc
        if fly:
            ask = flight.step(ask, loads, s, [body.d.xpos[i] for i in thighs])
        flying = now >= AT_S and max(loads) < flight.TOUCH_N
        low = min(float(f['pts'][:, 1].min()) for f in s['feet'])
        if flying:
            rec['air'] += 1
            rec['run'] += 1
            rec['apex'] = max(rec['apex'], float(s['com'][1]) - rec['home'])
            rec['clear'] = max(rec['clear'], low)
        else:
            rec['flights'] += rec['run'] >= FLOWN
            rec['run'] = 0
        if not flying and rec['air']:
            rec['landed'] = max(rec['landed'], loads[0] + loads[1])
        stance = ''.join('S' if k else '-' for k in ask['stance'])
        if not rec['stances'] or rec['stances'][-1] != stance:
            rec['stances'].append(stance)
        if trace and int(round(now * 1000)) % 10 == 0 and now >= AT_S + LOAD_S - 0.05:
            w, x, _y, z = (float(v) for v in s['turns'][0]['quat'])
            rec['rows'].append(
                '%6.3f  com %+6.1f mm %+5.2f m/s  ask %+6.1f  loads %5.0f %5.0f  laid %5.0f %5.0f  %s  '
                'low %5.1f mm  pitch %+5.1f roll %+5.1f  knees %3.0f %3.0f  k %+5.2f' % (
                    now, 1e3 * (float(s['com'][1]) - rec['home']), float(s['vcom'][1]), float(acc[1]),
                    loads[0], loads[1], bears[0], bears[1], stance, 1e3 * low,
                    math.degrees(2.0 * x * w), math.degrees(2.0 * z * w),
                    math.degrees(float(s['q'][body.knees[0]])),
                    math.degrees(float(s['q'][body.knees[1]])), float(s['k'][0])))
        return ask

    out = sim.stand(seconds=AT_S + hops * period, steps=False, core=core, asked=asked, inner=inner,
                    trace=stack, wrench=wrench)
    for row in rec['rows']:
        print(row)
    return {'stood': out['stood'], 'flights': rec['flights'], 'flight_s': round(rec['air'] * 0.001, 3),
            'apex_mm': round(1e3 * rec['apex'], 1), 'clear_mm': round(1e3 * rec['clear'], 1),
            'landed_n': round(rec['landed']), 'tilt': out['tilt'], 'drift_mm': out['drift_mm'],
            'stances': ' '.join(rec['stances']), 'top': out['top'], 'failed': out['failed'],
            'down': out['down'], 't': out['t'], 'us': out['us']}


def main(argv=None):
    global LOAD_M, LOAD_S, PUSH_S
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--push', type=float, default=PUSH, metavar='M_S2')
    parser.add_argument('--hops', type=int, default=1)
    parser.add_argument('--load', nargs=2, type=float, metavar=('M', 'S'),
                        help='the load: how far her height is let down, over how long')
    parser.add_argument('--push-s', type=float, metavar='S')
    parser.add_argument('--core', action='store_true', help='the loop in C')
    parser.add_argument('--wrench', action='store_true', help='the step without an active set')
    parser.add_argument('--held', action='store_true', help='her soles never released: the stack as it was')
    parser.add_argument('--timed', action='store_true', help="her throw asked to its time's end")
    parser.add_argument('--upright', action='store_true',
                        help='her trunk asked upright through the throw, as the stand has it')
    parser.add_argument('--inner', default='pd', choices=('pd', 'eso', 'none'),
                        help="the drives' own loop under the stack (tools/sim/wbc.py)")
    parser.add_argument('-v', '--verbose', action='store_true')
    parser.add_argument('--stack', action='store_true', help="the stand's own trace, 25 ms a row")
    args = parser.parse_args(argv)
    if args.load:
        LOAD_M, LOAD_S = args.load
    if args.push_s:
        PUSH_S = args.push_s
    print(json.dumps(hop(args.push, args.hops, args.core, args.verbose, not args.held, args.inner,
                         not args.upright, args.stack, not args.timed, args.wrench)))


if __name__ == '__main__':
    main()
