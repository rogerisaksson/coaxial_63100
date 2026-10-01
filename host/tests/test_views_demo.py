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
    import collections
    import math
    import re
    from coaxial.draw import cross_section
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import motions
    from coaxial.simulated.drive.plant import CATCH_UP_S
    from tools.render import page

    seconds = 0.0
    for _segment, name, stage_s, _rpm, _load, _how in motions.CYCLE:
        seconds += stage_s
        if name == 'load':
            break
    rows, beads, drawn, owned, params = [], [], [], [], {}
    real, real_bead, real_render = view.compose, cross_section._bead, cross_section.render
    real_lines = cross_section.Frame.lines

    def lines(self, ink, colour=False, tint=None):
        # What each cell of the motor went to this frame.
        owned.append(collections.Counter(c for row in self.owner for c in row))
        return real_lines(self, ink, colour, tint)

    def bead(frame, seat, pointer_deg, glyph=None, sweep=0.0):
        beads.append(pointer_deg)
        return real_bead(frame, seat, pointer_deg, glyph, sweep)

    def render(rotor_deg, slots=24, poles=28, *a, **k):
        # The can, the mark, both shutters and the phases as one call drew them: the feed's
        # thread replaces the state while compose draws.
        drawn.append((rotor_deg, k.get('pointer_deg'), k.get('sweep', 0.0), k.get('blur', 0.0),
                      poles, k.get('amps') or (), k.get('full') or 0.0))
        return real_render(rotor_deg, slots, poles, *a, **k)

    def compose(rig, origin, console, v):
        n, k, d = len(beads), len(owned), len(drawn)
        out = real(rig, origin, console, v)
        # The windings judged on the current the frame was drawn on: the state read after it
        # was the feed's newer one on CI's runner, a coasting frame lit whole (4c55f7b).
        amps, full = drawn[-1][5:7] if len(drawn) > d else ((), 0.0)
        pairs = max(1.0, v['params'].get('motor_pole_pairs') or 1.0)
        params.update(v['params'])
        st = v['state'] or {}
        rows.append((time.perf_counter(), v.get('stage'),
                     st.get('omega_hat', 0.0) / pairs * 60.0 / math.tau,
                     v.get('iq') or 0.0, beads[-1] if len(beads) > n else None,
                     (v['j'], v['b'], v.get('drag', motions.prop_k())),
                     1.5 * (st.get('vd', 0.0) * st.get('id', 0.0)
                            + st.get('vq', 0.0) * st.get('iq', 0.0)),
                     None, None,
                     owned[-1] if len(owned) > k else None,
                     max((abs(a) for a in amps), default=0.0), full,
                     bool((v.get('budget') or {}).get('throttling'))))
        return out

    view.compose, cross_section._bead, cross_section.render = compose, bead, render
    cross_section.Frame.lines = lines
    try:
        art = page.frame('rotor_observer', 150, 44, frames=int(seconds * FPS_CAP))
    finally:
        view.compose, cross_section._bead, cross_section.render = real, real_bead, real_render
        cross_section.Frame.lines = real_lines
    stages = []
    for row in rows:
        if not stages or stages[-1][0] != row[1]:
            stages.append((row[1], []))
        stages[-1][1].append(row)
    # SPIN, the special case: the unloaded rotor spooled at the clamp, each way, its top held on
    # little current - a slow runner draws into the next up, judged at 37 A (8e513c2).
    top = max(abs(stage[3] or 0.0) for stage in motions.CYCLE if stage[5] == 'free')
    ups = [st for name, st in stages if name == 'up' and abs(st[-1][2]) > 0.6 * top][:2]
    report.check('the unloaded spin spools to 90 %% of %.0f rpm each way on the clamp, its top '
                 'on little current' % top,
                 len(ups) >= 2 and ups[0][-1][2] > 0.0 > ups[1][-1][2]
                 and all(max(abs(r[3]) for r in st) >= 40.0 and abs(st[-1][2]) >= 0.9 * top
                         and abs(st[-1][3]) < 10.0 for st in ups),
                 ', '.join('%.0f rpm, %.0f A at most, %.0f A at the top' % (
                     st[-1][2], max(abs(r[3]) for r in st), st[-1][3]) for st in ups))
    # Into the motor at its top on the clamp, 1.5 lambda w I (w electrical, the top it spooled to,
    # I the clamp it held): its 0.15 s peak, a frame at a time and the voltage +-30 % frame to
    # frame, read 879 W at 20 frames/s, 775 at 10, 787 and 735 on CI's runner (2026-10-01).
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    clamp = max((abs(r[3]) for r in ups[0]), default=0.0) if ups else 0.0
    spun = abs(ups[0][-1][2]) * math.tau / 60.0 if ups else 0.0
    watts = 1.5 * (params.get('motor_lambda') or 0.0) * pairs * spun * clamp
    report.check('and near a kilowatt into the motor as it spools', watts >= 750.0,
                 '%.0f W: %.0f A at %.0f rad/s' % (watts, clamp, spun))
    # A loaded spin-up spools, slow and then faster and faster: at a constant rate its first
    # half second ran 552 rpm/s against a peak of 1 619, the current stepped on and off
    # (2026-09-28).
    spools = [st for name, st in stages if name == 'spool']
    if spools:
        up, t0 = spools[0], spools[0][0][0]
        early = abs(next((r[2] for r in up if r[0] - t0 >= 0.5), up[-1][2]) - up[0][2]) / 0.5
        windows = [abs(b[2] - a[2]) / (b[0] - a[0]) for a, b in zip(up, up[5:]) if b[0] > a[0]]
        peak = max(windows, default=0.0)
        report.check('a loaded spin-up starts slow: its first half second under a quarter of '
                     'its peak acceleration',
                     peak > 0.0 and early < 0.25 * peak,
                     '%.0f rpm/s, the peak %.0f' % (early, peak))
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
    # What is drawn, frame by frame: the magnets through a shutter - sharp at rest, streaks,
    # a band - never thinned to a ring nor flipping at a threshold (it flipped 7 times in a
    # ramp), and the windings lit by the current against the clamp, gone on a coast's none:
    # against the vector's own size 0.02 A lit them as 50 did (2026-09-28).
    counted = [(name, row) for name, st in stages for row in st if row[9] is not None]
    magnets = [row[9][cross_section.NORTH] + row[9][cross_section.SOUTH] for _, row in counted]
    steps = [abs(b - a) / max(1, a) for a, b in zip(magnets, magnets[1:])]
    report.check('the magnets drawn in every frame, turning or not: none under half the most, '
                 'none a quarter off the frame before',
                 bool(steps) and min(magnets) >= 0.5 * max(magnets) and max(steps) <= 0.25,
                 '%d to %d cells, the largest step %.2f' % (min(magnets), max(magnets),
                                                            max(steps, default=0.0))
                 if magnets else 'none')

    def windings(row):
        return sum(row[9][c] for c in cross_section.PHASE_CLASS)
    dark = [windings(row) for name, row in counted
            if name == 'coast' and row[11] and row[10] < 0.02 * row[11]]
    lit = [windings(row) for _, row in counted if row[10] >= 10.0]
    report.check('the windings gone while it coasts, whole on the current - its brightness',
                 bool(dark) and max(dark) == 0 and bool(lit) and min(lit) >= 0.9 * max(lit),
                 'coasting %d frames, at most %d cells lit; driven %d frames, %d to %d'
                 % (len(dark), max(dark, default=-1), len(lit), min(lit, default=-1),
                    max(lit, default=-1)))
    loads = [(index, st) for index, (name, st) in enumerate(stages) if name == 'load']
    # The load starts where the brake and the up before it left the rotor: starved there, it
    # starts off - CI's runner held -711 rpm on 10 A (2c15fe4).
    if loads and any(starved(index) for index in range(max(0, loads[-1][0] - 2),
                                                       loads[-1][0] + 1)):
        report.skip('the load\'s physics', 'the page stalled past the plant\'s catch-up')
    else:
        # Unless the envelope holds it back: the dyno runs the switches into their SOA, and a
        # throttled clamp let 1 500 rpm sag to 1 299 on its 50 A (2026-09-28).
        held = next(stage[3] for stage in motions.CYCLE if stage[1] == 'load') or 0.0
        load = loads[-1][1] if loads else []
        throttled = any(r[12] for r in load)
        report.check('the load holds %.0f rpm to 90 %%, on its current, unless the envelope '
                     'throttles it' % held,
                     bool(load) and abs(load[-1][3]) >= 5.0
                     and (abs(load[-1][2]) >= 0.9 * held or throttled),
                     '%.0f rpm on %.1f A%s' % (load[-1][2], load[-1][3],
                                               ', throttled' if throttled else '')
                     if load else 'none')
    # The mark is on the rotor: one place among the magnets every frame, streaked through
    # their shutter - on a pace of its own it drifted off them - and at a crawl never 0.4 of a
    # pitch at once: the angle over the pole pairs skipped one each electrical turn, the
    # stand-in's injection flipped half of one at rest, a held rotor rang 12 degrees over a
    # slow frame (2026-09-28).
    pitch = 720.0 / drawn[0][4] if drawn else 360.0
    places = [((mark - can) % pitch, mark, sweep) for can, mark, sweep, *_ in drawn
              if mark is not None]
    off = max((abs((p - places[0][0] + pitch / 2.0) % pitch - pitch / 2.0)
               for p, _m, _s in places), default=None) if places else None
    apart = max((abs(sweep - blur * 360.0 / poles) for _c, _m, sweep, blur, poles, *_ in drawn),
                default=0.0)
    crawl = max((abs(b[1] - a[1]) for a, b in zip(places, places[1:])
                 if abs(a[2]) < 1.0 and abs(b[2]) < 1.0), default=0.0)
    report.check('the mark rides the rotor among its magnets, through their shutter, and '
                 'at a crawl never skips a pitch',
                 off is not None and off < 1e-6 and apart < 1e-9 and crawl < 0.4 * pitch,
                 '%d frames: %.1e degrees off its place, the sweeps %.1e apart, the largest '
                 'step at a crawl %.1f' % (len(drawn), off or 0.0, apart, crawl))
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

    def wake(sweep):
        lines = cross_section.motor(0.0, width=60, height=30, pointer_deg=0.0,
                                    sweep=sweep, colour=True)
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
    slow, bead = wake(5.0)
    fast, _ = wake(25.0)
    back, _ = wake(-5.0)
    report.check('no wake at rest', not still, '%d cells' % len(still))
    report.check('and a longer one the further the can turns in the shutter',
                 0 < len(slow) < len(fast),
                 '%d cells over 5 degrees, %d over 25' % (len(slow), len(fast)))
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
