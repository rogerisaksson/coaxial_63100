"""The rotor demo on the stand-in, each stage by its physics, and the bead that rides the can."""
import math
import sys
import time

from tools.dev.focus import chosen
from views_kit import Report


def test_the_demo_actually_loads_the_motor(report):
    """The stand-in's rotor demo to the end of its load stage, as the bench asked for it: up
    clockwise against a propeller to near a kilowatt, coast, brake, the same the other way, a
    load held at speed. Each stage does what its physics says - the coast on J dw/dt = -b w -
    k w|w| - the bead never turns back against the rotor and speeds and slows with it, and
    the load warms the winding and spends the switches' margin (2026-09-28).
    """
    import math
    import re
    from coaxial.draw import cross_section
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import motions
    from coaxial.simulated.drive.plant import CATCH_UP_S
    from tools.render import page

    seconds = 0.0
    for name, stage_s, _rpm, _load in motions.CYCLE:
        seconds += stage_s
        if name == 'load':
            break
    rows, beads, rates = [], [], []
    real, real_bead, real_at = view.compose, cross_section._bead, view.bead_at

    def bead(frame, seat, pointer_deg, glyph=None, rate=None):
        beads.append(pointer_deg)
        return real_bead(frame, seat, pointer_deg, glyph, rate)

    def bead_at(bead, true, rate, top, dt):
        rates.append((rate, dt))
        return real_at(bead, true, rate, top, dt)

    def compose(rig, origin, console, v):
        n, m = len(beads), len(rates)
        out = real(rig, origin, console, v)
        pairs = max(1.0, v['params'].get('motor_pole_pairs') or 1.0)
        st = v['state'] or {}
        rows.append((time.perf_counter(), v.get('stage'),
                     st.get('omega_hat', 0.0) / pairs * 60.0 / math.tau,
                     v.get('iq') or 0.0, beads[-1] if len(beads) > n else None,
                     (v['j'], v['b'], motions.prop_k()),
                     1.5 * (st.get('vd', 0.0) * st.get('id', 0.0)
                            + st.get('vq', 0.0) * st.get('iq', 0.0)),
                     # The rpm the bead moved at and over what dt: the feed's thread replaces
                     # the state while compose draws, and one read after it, across a
                     # reversal, called a step back on CI (8e14db8).
                     rates[-1][0] / 6.0 if len(rates) > m else None,
                     rates[-1][1] if len(rates) > m else None))
        return out

    view.compose, cross_section._bead, view.bead_at = compose, bead, bead_at
    try:
        art = page.frame('rotor_observer', 150, 44, frames=int(seconds * FPS_CAP))
    finally:
        view.compose, cross_section._bead, view.bead_at = real, real_bead, real_at
    stages = []
    for row in rows:
        if not stages or stages[-1][0] != row[1]:
            stages.append((row[1], []))
        stages[-1][1].append(row)
    # The cycle's two: a slow runner draws into the next one, its up judged at 37 A (8e513c2).
    ups = [st for name, st in stages if name == 'up' and abs(st[-1][2]) > 2000.0][:2]
    report.check('up past 2 000 rpm each way against the propeller, on the clamp',
                 len(ups) >= 2 and ups[0][-1][2] > 0.0 > ups[1][-1][2]
                 and all(abs(st[-1][3]) >= 40.0 for st in ups),
                 ', '.join('%.0f rpm on %.0f A' % (st[-1][2], st[-1][3]) for st in ups))
    report.check('and near a kilowatt into the motor at the top',
                 bool(ups) and max(r[6] for r in ups[0]) >= 800.0,
                 '%.0f W' % max(r[6] for r in ups[0]) if ups else 'none')
    def starved(index):
        """Whether the page stalled through stage `index` and into the next: the time between
        frames past the stand-in plant's catch-up - and motions.sweep's step, clamped alike - is
        time the plant and the reference never turned, and past 5 % of the stage, the brake's
        own tolerance, the stage's physics is not the page's to judge. CI's runner braked
        993 -> 221 rpm, another host 993 -> -69 beside a heavy suite (2026-09-28)."""
        times = [row[0] for row in stages[index][1]]
        if index + 1 < len(stages):
            times.append(stages[index + 1][1][0][0])
        dropped = sum(max(0.0, b - a - CATCH_UP_S) for a, b in zip(times, times[1:]))
        # And the page's rate: the speed loop steps a frame, designed at 20 a second - under
        # half of it, CI's load held -595 rpm on 10 A with nothing dropped (78a1038).
        rate = (len(times) - 1) / max(1e-9, times[-1] - times[0])
        return dropped > 0.05 * max(1e-9, times[-1] - times[0]) or rate < FPS_CAP / 2.0

    for index, (name, st) in enumerate(stages):
        if name in ('coast', 'brake') and index + 1 == len(stages):
            continue                      # the window ends in it: not whole
        if name in ('coast', 'brake') and starved(index):
            report.skip('the %s\'s physics' % name, 'the page stalled past the plant\'s catch-up')
            continue
        if name == 'coast' and abs(st[0][2]) > 500.0:
            # J dw/dt = -b w - k w|w|, closed form, from where the coast began.
            j, b, k = st[0][5]
            w0 = abs(st[0][2]) * math.tau / 60.0
            t = st[-1][0] - st[0][0]
            want = b * w0 / ((b + k * w0) * math.exp(b * t / j) - k * w0) / w0
            got = st[-1][2] / st[0][2]
            report.check('the coast is the drag and the propeller, within a quarter of its drop',
                         abs((1.0 - got) - (1.0 - want)) <= 0.25 * (1.0 - want),
                         '%.0f -> %.0f rpm: x%.2f, want x%.2f' % (st[0][2], st[-1][2], got, want))
        if name == 'brake' and abs(st[0][2]) > 100.0:
            report.check('the brake lands, under 5 % of where it began',
                         abs(st[-1][2]) <= 0.05 * abs(st[0][2]),
                         '%.0f -> %.0f rpm' % (st[0][2], st[-1][2]))
    loads = [(index, st) for index, (name, st) in enumerate(stages) if name == 'load']
    # The load starts where the brake and the up before it left the rotor: starved there, it
    # starts off - CI's runner held -711 rpm on 10 A (2c15fe4).
    if loads and any(starved(index) for index in range(max(0, loads[-1][0] - 2),
                                                       loads[-1][0] + 1)):
        report.skip('the load\'s physics', 'the page stalled past the plant\'s catch-up')
    else:
        report.check('the load holds 1 000 rpm to 90 %, on its current',
                     bool(loads) and abs(loads[-1][1][-1][2]) >= 900.0
                     and abs(loads[-1][1][-1][3]) >= 5.0,
                     '%.0f rpm on %.1f A' % (loads[-1][1][-1][2], loads[-1][1][-1][3])
                     if loads else 'none')
    back, xs, ys = 0, [], []
    for was, row in zip(rows, rows[1:]):
        if was[4] is None or row[4] is None or row[7] is None:
            continue
        step, rpm, dt = row[4] - was[4], row[7], row[8]
        # Its speed on the screen, deg/s: its own step over its own dt. Over the rows' clock a
        # frame stalled inside compose put the stall in the next frame's step and not in its
        # time - 0.80 under a 0.4 s stall every tenth frame. At its clamp (bead_at's 0.25 s) a
        # step is the clamp's, not the rule's.
        if abs(rpm) > 30.0 and 0.0 < dt < CATCH_UP_S:
            back += step * rpm < 0.0
            xs.append(abs(rpm))
            ys.append(abs(step) / dt)

    def ranks(v):
        order = sorted(range(len(v)), key=lambda k: v[k])
        out = [0.0] * len(v)
        for rank, k in enumerate(order):
            out[k] = float(rank)
        return out
    rx, ry = ranks(xs), ranks(ys)
    mx, my = sum(rx) / len(rx), sum(ry) / len(ry)
    rho = (sum((a - mx) * (b - my) for a, b in zip(rx, ry))
           / math.sqrt(sum((a - mx) ** 2 for a in rx) * sum((b - my) ** 2 for b in ry)))
    report.check('the bead never turns back against the rotor', back == 0, '%d times' % back)
    report.check('and speeds and slows with it, rank correlation 0.9 or more', rho >= 0.9,
                 '%.2f' % rho)
    text = re.sub(r'\x1b\[[0-9;]*m', '', art)
    winding = re.search(r'WINDING +([0-9.]+)', text)   # %5.1f: a space at two digits
    soa = re.search(r'SWITCH SOA ([0-9.]+) %', text)
    report.check('the winding is warm at the load\'s end',
                 winding is not None and float(winding.group(1)) >= 35.0,
                 '%s C, floor 35, 10 K over the room' % (winding.group(1) if winding else '-'))
    report.check('and the switches have spent a fifth of their margin',
                 soa is not None and float(soa.group(1)) >= 20.0,
                 '%s %%, floor 20' % (soa.group(1) if soa else '-'))


def test_the_bead_is_round_at_every_angle(report):
    """The pointer is `POINTER_GLYPH`, and it rides the rim."""
    from coaxial.draw import cross_section

    for aspect in (2.0, 2.4):
        was, seats = None, set()
        for deg in range(0, 360, 5):
            art = cross_section.render(0.0, 24, 28, 46, 18, aspect=aspect,
                                 pointer_deg=float(deg)).split(chr(10))
            at = [(r, line.index(cross_section.POINTER_GLYPH))
                  for r, line in enumerate(art)
                  if cross_section.POINTER_GLYPH in line]
            if len(at) != 1:
                was = '%d degrees: %d marks' % (deg, len(at))
                break
            if len(art[at[0][0]]) != len(art[0]):
                was = '%d degrees: its row came out a different length' % deg
                break
            seats.add(at[0])
        report.check('at aspect %.1f, one mark at every angle round the can'
                     % aspect, was is None, was or '72 angles')
        report.check('and it travels rather than sitting in a few seats',
                     len(seats) > 60, '%d distinct cells' % len(seats))

    # It rides the rim in the drawing's own space.
    for aspect in (2.0, 2.4):
        stretch = aspect / 4.0 * 2.0
        cx, r, _, _ = cross_section.layout(46, 18, 0, 0, rows=18)
        cy = 18 * 4 / 2.0 - 0.5
        seat = r.can + cross_section.POINTER_SEAT
        out = []
        for deg in range(0, 360, 5):
            phi = math.radians(deg)
            ax = cx + seat * math.cos(phi)
            ay = cy - seat * math.sin(phi) / stretch
            out.append(math.hypot(ax - cx, (cy - ay) * stretch))
        report.check('at aspect %.1f it sits one radius out at every angle'
                     % aspect, max(out) - min(out) < 1e-9,
                     '%.3f to %.3f against a rim at %.3f'
                     % (min(out), max(out), r.can))

    # The nearest cell centre, not the one the point fell inside.
    cx, r, _, _ = cross_section.layout(46, 18, 0, 0, rows=18)
    cy = 18 * 4 / 2.0 - 0.5
    seat = r.can + cross_section.POINTER_SEAT

    def worst(pick):
        out = 0.0
        for deg in range(360):
            phi = math.radians(deg)
            ax, ay = cx + seat * math.cos(phi), cy - seat * math.sin(phi)
            col, row = pick(ax, ay)
            out = max(out, math.hypot(col * 2 + 0.5 - ax, row * 4 + 1.5 - ay))
        return out

    near = worst(lambda x, y: (int(math.floor((x - 0.5) / 2 + 0.5)),
                               int(math.floor((y - 1.5) / 4 + 0.5))))
    cut = worst(lambda x, y: (int(x) // 2, int(y) // 4))
    report.check('the nearest cell centre beats the one it fell inside',
                 near < cut - 0.5, '%.2f dots against %.2f' % (near, cut))


def test_the_bead_trails_its_speed(report):
    """The wake behind the bead: its length is the speed, its side the
    direction, and it fades from the bead's orange into the can's teal.
    """
    import re

    from coaxial.draw import cross_section
    from machine import ansi

    inks = {ansi.code(cross_section.INK[c]) for c in cross_section.TRAIL}

    def wake(rate):
        lines = cross_section.motor(0.0, width=60, height=30, pointer_deg=0.0,
                              pointer_rate=rate, colour=True)
        rows = []
        for row, line in enumerate(lines):
            for hit in re.finditer('(' + chr(27) + r'\[38;[25];[\d;]+m)([^' + chr(27)
                                   + ']*)', line):
                if hit.group(1) in inks:
                    rows += [row] * len(hit.group(2))
        bead = next(row for row, line in enumerate(lines)
                    if cross_section.POINTER_GLYPH in line)
        return rows, bead

    still, _ = wake(0.0)
    slow, bead = wake(360.0)
    fast, _ = wake(1200.0)
    back, _ = wake(-360.0)
    report.check('no wake at rest', not still, '%d cells' % len(still))
    report.check('and a longer one the faster the can turns',
                 0 < len(slow) < len(fast),
                 '%d cells at 360, %d at 1200' % (len(slow), len(fast)))
    report.check('behind the bead: at three o\'clock, below it turning '
                 'counter-clockwise and above it turning clockwise',
                 bool(slow and back)
                 and sum(slow) / len(slow) > bead > sum(back) / len(back),
                 'rows %.1f and %.1f about the bead\'s %d'
                 % (sum(slow) / max(1, len(slow)),
                    sum(back) / max(1, len(back)), bead))
    report.check('the bead wears the palette\'s orange, the north pole\'s',
                 cross_section.INK[cross_section.POINTER] == ansi.AMBER
                 == cross_section.INK[cross_section.NORTH])


ROSTER = (test_the_demo_actually_loads_the_motor, test_the_bead_is_round_at_every_angle,
          test_the_bead_trails_its_speed)

def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
