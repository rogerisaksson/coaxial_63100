"""Each craft text flown as built, its own and its variants: its card to its landing, a process each.

    python tools/sim/flights.py                 # every kind: its own text and its variants, a row a text
    python tools/sim/flights.py --variants      # the variants alone (their kinds' suites fly their own)
    row = flights.flown('helicopter')           # the text this process flies as that kind, flown

A craft's module reads its text as it loads (`machine.aircraft.text`, COAXIAL_<KIND>), so each text flies in a
process of its own, its kind's card as its suite flies it (tests/test_quad.py, test_helicopter.py, test_aerobat.py)
to the card's end or LIMIT_S: landed, the card's last stage reached and nothing struck.
"""
import json
import math
import os
import subprocess
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: The most a card may take, s, by kind.
LIMIT_S = {'quad': 120.0, 'helicopter': 115.0, 'ornithopter': 115.0, 'aerobat': 160.0}


def _tilt(state):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(state['turn'][1][1])))))


def _quad():
    """(stages, seconds, landed, tilt) of the quad's card (`test_quad.flown`)."""
    from machine.aircraft import aerobatics, aerokin
    from tools.sim import streams
    card = aerobatics.CARD
    flight, route, law = streams.Flight(), aerobatics.routine(card), aerokin.fresh()
    t, stages, tilt, landed = 0.0, [], 0.0, False
    while t < LIMIT_S['quad']:
        name, row = streams.asked(flight, route, law, t, 0.01)
        if stages and stages[-1] == card[-1][0] and name == card[0][0]:
            landed = True
            break
        streams.flew(flight, route, law, row, 0.01)
        stages += [name] if not stages or stages[-1] != name else []
        tilt, t = max(tilt, _tilt(flight.sky.state())), t + 0.01
    return stages, t, landed, tilt


def _rotary(kind):
    """(stages, seconds, landed, tilt) of the helicopter's or the ornithopter's card (`test_helicopter`)."""
    import importlib
    from machine.aircraft import aerobatics, aerokin
    craft, dt = importlib.import_module('machine.aircraft.' + kind), 1.0 / 120.0
    sky, core, law = craft.Sky(), craft.Core(), aerokin.fresh()
    route, stages, tilt = aerobatics.routine(craft.CARD), [], 0.0
    for k in range(int(LIMIT_S[kind] / dt)):
        stage, row = aerobatics.fly(route, k * dt, ('held', 'fit') if law['held'] else ('fit',))
        state = sky.state()
        ans = core.step(law, row, state, dt, craft.PULL_MAX)
        sky.step(ans.speeds + ans.controls, dt)
        stages += [stage] if not stages or stages[-1] != stage else []
        tilt = max(tilt, _tilt(state))
        if stages[-2:] == ['land', 'idle']:
            return stages, k * dt, True, tilt
    return stages, LIMIT_S[kind], False, tilt


def _aerobat():
    """(stages, seconds, landed, tilt) of the aerobat's card on its airfield (`test_aerobat._routine`)."""
    from machine.aircraft import aerobat, aerobatics, aerokin, airfield
    dt = 1.0 / 120.0
    sky, core, law = aerobat.Sky(airfield.solids()), aerobat.Core(), aerokin.fresh()
    sky.stand(False)
    route, stages, tilt = aerobatics.routine(aerobat.CARD), [], 0.0
    for k in range(int(LIMIT_S['aerobat'] / dt)):
        state = sky.state()
        route['seen'] = dict(aerokin.seen(law, state, dt, aerobat.PULL_MAX, aerobat.PULL_MAX), spent=False)
        stage, row = aerobatics.fly(route, k * dt, ('held', 'fit') if law['held'] else ('fit',))
        ans = core.step(law, row, state, dt, aerobat.PULL_MAX)
        sky.step(ans.speeds + ans.controls, dt)
        stages += [stage] if not stages or stages[-1] != stage else []
        tilt = max(tilt, _tilt(state))
        if sky.hit is not None:
            return stages, k * dt, False, tilt
        if stages[-2:] == ['land', 'idle']:
            return stages, k * dt, True, tilt
    return stages, LIMIT_S['aerobat'], False, tilt


def flown(kind):
    """{kind, text, vehicle, mass, stages, seconds, landed, tilt}: the text this process flies as `kind`, flown."""
    import importlib
    from machine import aircraft
    module = importlib.import_module('machine.aircraft.' + ('quad' if kind == 'quad' else kind + '.airframe'))
    stages, seconds, landed, tilt = (_quad() if kind == 'quad' else _aerobat() if kind == 'aerobat'
                                     else _rotary(kind))
    return {'kind': kind, 'text': os.path.basename(aircraft.text(kind))[:-5], 'vehicle': module.MANIFEST['vehicle'],
            'mass': float(module.MASS_KG), 'stages': len(stages), 'seconds': round(seconds, 2), 'landed': landed,
            'tilt': round(tilt, 1)}


def rows(variants_only=False):
    """[flown's row] of every kind's texts, a process a text: its variants alone where `variants_only`."""
    from machine import aircraft
    out = []
    for kind in aircraft.CRAFTS:
        for name in aircraft.variants(kind)[1 if variants_only else 0:]:
            run = subprocess.run([sys.executable, '-X', 'utf8', os.path.abspath(__file__), '--one', kind], cwd=HOST,
                                 capture_output=True, text=True,
                                 env=dict(os.environ, PYTHONPATH=HOST, **{'COAXIAL_' + kind.upper(): name}))
            got = [line for line in run.stdout.splitlines() if line.startswith('{')]
            out.append(json.loads(got[-1]) if got else {'kind': kind, 'text': name, 'landed': False,
                                                         'error': (run.stdout + run.stderr)[-300:]})
    return out


def main():
    if sys.argv[1:2] == ['--one']:
        print(json.dumps(flown(sys.argv[2])))
        return 0
    got = rows('--variants' in sys.argv[1:])
    print('%-12s %-24s %-18s %6s %6s %8s %6s %s' % ('kind', 'text', 'vehicle', 'kg', 'stages', 's', 'tilt', 'landed'))
    for r in got:
        print('%-12s %-24s %-18s %6.2f %6d %8.2f %6.1f %s' % (r['kind'], r['text'], r.get('vehicle', '-'), r.get('mass', 0.0),
                                                          r.get('stages', 0), r.get('seconds', 0.0), r.get('tilt', 0.0),
                                                          r['landed'] if 'error' not in r else r['error']))
    return 0 if all(r['landed'] for r in got) else 1


if __name__ == '__main__':
    sys.exit(main())
