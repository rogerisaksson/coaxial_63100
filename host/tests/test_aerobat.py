"""The aerobat on its airfield: its routine among what stands there, its gate course in wind."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report


def _routine():
    """The card flown as the page flies it on the airfield, its gates down: {stage: [rows]},
    the stages in order, the sky."""
    from machine import aerobat, aerobatics, aerokin, airfield, se3mpc
    dt, by_stage, stages = 1.0 / 120.0, {}, []
    tail, foot = se3mpc.points(aerobat)[0][:, :3], aerobat.SKID_M + aerobat.FOOT_M
    sky, core, law = aerobat.Sky(airfield.solids()), aerobat.Core(), aerokin.fresh()
    sky.stand(False)
    route = aerobatics.routine(aerobat.CARD)
    for k in range(int(140.0 / dt)):
        stage, row = aerobatics.fly(route, k * dt, ('held', 'fit') if law['held'] else ('fit',))
        state = sky.state()
        ans = core.step(law, row, state, dt, aerobat.PULL_MAX)
        sky.step(ans.speeds + ans.controls, dt)
        shown = aerokin.FALL if stage == 'burn' and law['doing'] == aerokin.FALL else stage
        if not stages or stages[-1] != shown:
            stages.append(shown)
            if stages[-2:] == ['land', 'idle']:
                break
        turn, vel = state['turn'], state['vel']
        by_stage.setdefault(stage, []).append({
            'h': state['h'], 'v': state['v'], 'r11': float(turn[1][1]), 'lift': state['lift'],
            'tilt': math.degrees(math.acos(max(-1.0, min(1.0, float(turn[1][1]))))),
            'held': law['held'], 'z': float(state['at'][2]), 'vz': float(vel[2]), 'hdg': law['heading'][0],
            'w': core.wingborne, 'tail': state['h'] + foot + float(min(tail @ turn[1])),
            'belly': float(turn[1][2]), 'ahead': float(vel @ turn[:, 1])})
        if sky.hit is not None:
            break
    return by_stage, stages, sky


def test_aerobat_routine(report):
    """`aerobat` flies its card on its airfield: off the runway, a teardrop, the knife-edge back
    down it, its 3D figures, its circuit and down on the runway."""
    from machine import aerobat
    c, stages, sky = _routine()
    hov, ft, burn, hold, land, ke = (c[k] for k in ('prop-hang', 'full tilt', 'burn', 'hold', 'land', 'knife-edge'))
    gl, gd = [p['z'] for p in c['lift'] if p['h'] < 0.03], [p['z'] for p in land if p['h'] < 0.03]
    report.check('aerobat rolls its runway, flies a knife-edge back down it and its 3D figures',
                 c['spool'][0]['tilt'] > 55.0 and len(gl) > 20 and max(gl) - min(gl) > 5.0
                 and sorted(p['tail'] for p in ke)[len(ke) // 2] < 0.05
                 and all(abs(p['belly']) < 0.1 and p['ahead'] > 8.0 for p in ke)
                 and abs(hov[-1]['h'] - 1.5) < 0.06 and hov[-1]['held'] and c['harrier'][-1]['h'] > 7.5
                 and min(p['tilt'] for p in c['waterfall']) < 65.0 and max(p['tilt'] for p in c['waterfall']) > 115.0
                 and min(p['r11'] for p in c['snap-roll']) < -0.80 and max(p['v'] for p in ft) > 9.0,
                 'stages %s' % stages[-4:])
    report.check('aerobat arrests its burn over the floor and lands on its runway',
                 max(p['tilt'] for p in ft + burn) < 10.0 and max(p['h'] for p in burn) > 18.0
                 and min(p['lift'] for p in burn if p['r11'] > 0.5) < 2.0 and stages.count('fall') == 1
                 and abs(hold[-1]['h'] - aerobat.FLOOR_M) < 0.05 and c['final'][-1]['w'] > 0.5
                 and len(gd) > 20 and min(p['v'] for p in land if p['h'] < 0.03) > -1.0
                 and max(p['h'] for p in land[-60:]) < 0.025 and abs(land[-1]['vz']) < 0.02
                 and land[-1]['tilt'] > 55.0 and land[-1]['hdg'] > 0.60 and sky.hit is None,
                 'hit %s, touch %.2f m/s' % (sky.hit, min(p['v'] for p in land if p['h'] < 0.03)))


def test_aerobat_airfield_laps(report):
    """The aerobat laps its airfield's gate course in gusty wind through every gate's opening,
    and lands on its runway along it."""
    from machine import aerobat, aerobatics, aerokin, airfield, pull, quad
    dt, tick, half = 0.02, 0.002, airfield.GATE_M / 2.0
    sky, core, law = aerobat.Sky(airfield.solids()), aerobat.Core(), aerokin.fresh()
    route, air = aerobatics.routine(airfield.CARD), quad.air(7, 'gusty')
    top = aerobat.K_THRUST * pull.held(pull.I_MAX) ** 2 / aerobat.MASS_KG
    stages, misses, crossed = [], [], 0
    sides: list[bool | None] = [None] * len(airfield.GATES)
    for k in range(int(110.0 / dt)):
        st = sky.state()
        route['seen'] = dict(aerokin.seen(law, st, dt, aerobat.COURSE_SPHERE, top), spent=False)
        stage, row = aerobatics.fly(route, k * dt, ('held', 'fit') if law['held'] else ('fit',))
        if not stages or stages[-1] != stage:
            stages.append(stage)
        along = core.plan(law, row, st, dt, aerobat.COURSE_SPHERE)
        for _ in range(10):
            ans = core.step(law, row, sky.state(), tick, aerobat.COURSE_SPHERE, None, along)
            quad.blown(air, tick)
            sky.step(ans.speeds + ans.controls, tick, air)
        x, y, z = (float(v) for v in sky.state()['at'])
        for g, (gx, gy, gz, heading, _size) in enumerate(airfield.GATES):
            s, c = math.sin(math.radians(heading)), math.cos(math.radians(heading))
            side = (x - gx) * s + (z - gz) * c > 0.0
            across = (x - gx) * c - (z - gz) * s
            if sides[g] is False and side and abs(across) < 2.0 * airfield.GATE_M:
                crossed += 1
                misses += [g] * int(abs(across) > half - aerobat.HALF_M or y > airfield.GATE_M - 0.3)
            sides[g] = side
        if sky.hit is not None or (len(stages) > 3 and stage == 'idle'):
            break
    st, rc, rs = sky.state(), math.cos(airfield.RUNWAY_RAD), math.sin(airfield.RUNWAY_RAD)
    ahead = float(st['at'][0]) * rs + float(st['at'][2]) * rc
    report.check('aerobat laps its airfield in gusty wind through every gate and lands on its runway',
                 sky.hit is None and route['lap']['laps'] == airfield.LAPS and not misses
                 and crossed >= airfield.LAPS * len(airfield.GATES) and stages[-2:] == ['land', 'idle']
                 and st['h'] < 0.025 and math.hypot(*st['vel']) < 0.03 and -aerobat.RUNWAY_M < ahead < 0.0
                 and law['heading'][0] * rc + law['heading'][1] * rs > 0.98,
                 'hit %s, laps %s, crossed %d, missed %s, %s, %.1f m along' % (
                     sky.hit, route['lap']['laps'], crossed, misses, stages[-2:], ahead))


ROSTER = (
    test_aerobat_routine,
    test_aerobat_airfield_laps,
)


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
