"""The aerobat on its airfield: its routine among what stands there, its gate course in wind."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report


def _routine():
    """The card flown as the page flies it on the airfield, its gates down: {stage: [rows]},
    the stages in order, the sky."""
    from machine import aerobat, aerobatics, aerokin, airfield, pull, se3mpc
    dt, by_stage, stages = 1.0 / 120.0, {}, []
    top = aerobat.PULL_MAX
    rest, foot = se3mpc.points(aerobat)[2], aerobat.SKID_M + aerobat.FOOT_M
    rc, rs = math.cos(airfield.RUNWAY_RAD), math.sin(airfield.RUNWAY_RAD)
    sky, core, law = aerobat.Sky(airfield.solids()), aerobat.Core(), aerokin.fresh()
    sky.stand(False)
    route = aerobatics.routine(aerobat.CARD)
    for k in range(int(160.0 / dt)):
        state = sky.state()
        route['seen'] = dict(aerokin.seen(law, state, dt, aerobat.PULL_MAX, top), spent=False)
        stage, row = aerobatics.fly(route, k * dt, ('held', 'fit') if law['held'] else ('fit',))
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
            'held': law['held'], 'along': float(state['at'][0]) * rs + float(state['at'][2]) * rc,
            'across': float(state['at'][0]) * rc - float(state['at'][2]) * rs,
            'vz': float(vel[2]), 'hdg': law['heading'][0], 'span': abs(float(turn[1][0])),
            'tip': state['h'] + foot + float(min(rest @ turn[1])),
            'w': core.wingborne, 'knife': row.get('knife', 0.0), 'roll': row.get('roll', 0.0),
            'roll_ang': core.roll_ang, 'sp0': abs(float(state['spin'][0])),
            'belly': float(turn[1][2]), 'ahead': float(vel @ turn[:, 1])})
        if sky.hit is not None:
            break
    return by_stage, stages, sky


def _opening_in_wind():
    """The routine's opening (`idle` .. `prop-hang`) flown with `core.plan` at 50 Hz and 10x
    `core.step` at 500 Hz in `weather.air(3)`, as the live page flies it."""
    from machine import aerobat, aerobatics, aerokin, airfield, weather
    dt, tick = 0.02, 0.002
    rc, rs = math.cos(airfield.RUNWAY_RAD), math.sin(airfield.RUNWAY_RAD)
    sky, core, law = aerobat.Sky(airfield.solids()), aerobat.Core(), aerokin.fresh()
    sky.stand(False)
    route, air = aerobatics.routine(aerobat.CARD[:6]), weather.air(3)
    max_roll, max_across, stage = 0.0, 0.0, 'idle'
    for k in range(int(34.0 / dt)):
        st = sky.state()
        route['seen'] = dict(aerokin.seen(law, st, dt, aerobat.PULL_MAX, aerobat.PULL_MAX), spent=False)
        stage, row = aerobatics.fly(route, k * dt, ('held', 'fit') if law['held'] else ('fit',))
        along = core.plan(law, row, st, dt, aerobat.PULL_MAX)
        for _ in range(10):
            ans = core.step(law, row, sky.state(), tick, aerobat.PULL_MAX, None, along)
            weather.blown(air, tick)
            sky.step(ans.speeds + ans.controls, tick, air)
        if core.roll_ang > 0.0:
            max_roll = max(max_roll, core.roll_ang)
            st = sky.state()
            max_across = max(max_across, abs(float(st['at'][0]) * rc - float(st['at'][2]) * rs))
        if sky.hit is not None or (stage == 'prop-hang' and law['held']):
            break
    return sky.hit, stage, bool(law['held']), max_roll, max_across


def test_aerobat_routine(report):
    """`aerobat` flies its card on its airfield: off the runway, a teardrop at 60 deg/s, the
    knife-edge back down it at 20 m/s - its wings within 20 deg of upright, its lower tip a few
    centimetres over the runway -, a low rolling harrier pass back up the runway, its 3D figures,
    its circuit and down on the runway."""
    from machine import aerobat
    c, stages, sky = _routine()
    hit_w, stage_w, held_w, roll_w, across_w = _opening_in_wind()
    hov, ft, burn, hold, land = (c[k] for k in ('prop-hang', 'full tilt', 'burn', 'hold', 'land'))
    ke = [p for p in c['knife pass'] if abs(p['knife']) > 0.99]
    rh = [p for p in c['rolling harrier'] if p['roll_ang'] > 0.0]
    turns = [p for p in c['knife pass'] + c['rolling harrier'] if abs(p['knife']) < 0.1 and p['roll_ang'] == 0.0]
    gl, gd = [p['along'] for p in c['lift'] if p['h'] < 0.03], [p['along'] for p in land if p['h'] < 0.03]
    report.check('aerobat rolls its runway, flies a knife-edge and low rolling harrier back and forth, and its 3D figures',
                 c['spool'][0]['tilt'] > 55.0 and len(gl) > 20 and max(gl) - min(gl) > 3.0
                 and sorted(p['tip'] for p in ke)[len(ke) // 2] < 0.06
                 and all(p['tip'] > 0.0 and p['span'] > 0.94 and abs(p['belly']) < 0.35 and p['ahead'] > 16.0
                         for p in ke)
                 and max(p['sp0'] for p in turns) < 2.7 and 'rolling harrier' in stages
                 and len(rh) > 250 and max(p['roll_ang'] for p in rh) > 7.8
                 and max(p['belly'] for p in rh) > 0.90 and min(p['belly'] for p in rh) < -0.90
                 and all(0.9 < p['h'] < 2.7 and abs(p['across']) < 1.5 and p['tip'] > 0.5 for p in rh)
                 and hit_w is None and stage_w == 'prop-hang' and held_w and roll_w > 7.8 and across_w < 1.5
                 and abs(hov[-1]['h'] - aerobat.STUNTS['hover']['height']) < 0.06 and hov[-1]['held']
                 and c['harrier'][-1]['h'] > aerobat.STUNTS['climbing_harrier']['height'] - 0.5
                 and min(p['tilt'] for p in c['waterfall']) < 65.0 and max(p['tilt'] for p in c['waterfall']) > 115.0
                 and min(p['r11'] for p in c['snap-roll']) < -0.80 and max(p['v'] for p in ft) > 9.0,
                 'stages %s, wind opening (%s, %s, roll %.2f, across %.2f m)' % (
                     stages[-4:], hit_w, stage_w, roll_w, across_w))
    report.check('aerobat arrests its burn over the floor and lands on its runway',
                 max(p['tilt'] for p in ft + burn) < 10.0 and max(p['h'] for p in burn) > 12.0
                 and min(p['lift'] for p in burn if p['r11'] > 0.5) < 2.0 and stages.count('fall') == 1
                 and abs(hold[-1]['h'] - aerobat.FLOOR_M) < 0.05 and c['final'][-1]['w'] > 0.5
                 and len(gd) > 20 and min(p['v'] for p in land if p['h'] < 0.03) > -1.0
                 and max(p['h'] for p in land[-60:]) < 0.025 and abs(land[-1]['vz']) < 0.02
                 and land[-1]['tilt'] > 55.0 and land[-1]['hdg'] > 0.60 and sky.hit is None,
                 'hit %s, touch %.2f m/s' % (sky.hit, min(p['v'] for p in land if p['h'] < 0.03)))


def test_aerobat_airfield_laps(report):
    """The aerobat laps its airfield's course in gusty wind - through the air gate, under the
    limbo bar, up through the hoop, round each pylon on its line's side, through the knife gate
    on its side - and lands on its runway along it."""
    from machine import aerobat, aerobatics, aerokin, airfield, pull, weather
    dt, tick, foot = 0.02, 0.002, aerobat.SKID_M + aerobat.FOOT_M
    sky, core, law = aerobat.Sky(airfield.solids()), aerobat.Core(), aerokin.fresh()
    route, air = aerobatics.routine(airfield.CARD), weather.air(7, 'gusty')
    top = aerobat.PULL_MAX
    stages, misses, crossed = [], [], 0
    sides: list[bool | None] = [None] * len(airfield.GATES)
    for k in range(int(130.0 / dt)):
        st = sky.state()
        route['seen'] = dict(aerokin.seen(law, st, dt, aerobat.COURSE_SPHERE, top), spent=False)
        stage, row = aerobatics.fly(route, k * dt, ('held', 'fit') if law['held'] else ('fit',))
        if not stages or stages[-1] != stage:
            stages.append(stage)
        along = core.plan(law, row, st, dt, aerobat.COURSE_SPHERE)
        for _ in range(10):
            ans = core.step(law, row, sky.state(), tick, aerobat.COURSE_SPHERE, None, along)
            weather.blown(air, tick)
            sky.step(ans.speeds + ans.controls, tick, air)
        st = sky.state()
        (x, y, z), belly = (float(v) for v in st['at']), abs(float(st['turn'][1][2]))
        for g, (gx, gy, gz, heading, wide, high) in enumerate(airfield.GATES):
            s, c = math.sin(math.radians(heading)), math.cos(math.radians(heading))
            side, across = (x - gx) * s + (z - gz) * c > 0.0, (x - gx) * c - (z - gz) * s
            if sides[g] is False and side and abs(across) < 10.0:
                crossed += 1
                lx, lz = airfield.placed(*airfield.TRACK[int(airfield.S_GATES[g])][:2])
                kept = ((lx - gx) * c - (lz - gz) * s) * across > 0.0 and abs(across) > wide / 2 + aerobat.HALF_M
                knife = wide == airfield.SLOT_M
                inside = (abs(across) < wide / 2 - (0.3 if knife else aerobat.HALF_M)
                          and gy - high / 2 + 0.3 < st['h'] + foot < gy + high / 2 - 0.3 and (belly < 0.35 or not knife))
                misses += [g] * int(not (kept if wide == airfield.PYLON_M else inside))
            sides[g] = side
        if sky.hit is not None or (len(stages) > 3 and stage == 'idle'):
            break
    st, rc, rs = sky.state(), math.cos(airfield.RUNWAY_RAD), math.sin(airfield.RUNWAY_RAD)
    ahead = float(st['at'][0]) * rs + float(st['at'][2]) * rc
    report.check('aerobat laps its airfield in gusty wind through every gate and lands on its runway',
                 sky.hit is None and route['lap']['laps'] == airfield.LAPS and not misses
                 and crossed >= airfield.LAPS * len(airfield.GATES) and stages[-2:] == ['land', 'idle']
                 and st['h'] < 0.025 and math.hypot(*st['vel']) < 0.03
                 and airfield.RUNWAY_ALONG[0] < ahead < airfield.RUNWAY_ALONG[1]
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
