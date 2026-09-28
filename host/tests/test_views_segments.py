"""The rotor demo's segments on the stand-in, each started on its own and judged by what it is."""
import math
import sys
import time

from tools.dev.focus import chosen
from views_kit import Report


def run(segment, extra=2.0):
    """The rotor page from `segment` on, its length and `extra` seconds: a row a frame - the wall
    time, the segment, the stage, rpm, the current's size, the stand-in's true shaft, degrees
    unwrapped, whether the envelope throttled, the mark as drawn and the pole pairs - and
    whether the page kept half its frame rate."""
    from coaxial.draw import cross_section
    from terminal.ui.screen import FPS_CAP
    from terminal.views import show_rotor_observer as view
    from terminal.views.rotor import motions
    from tools.render import page

    rows, real, was, shaft = [], view.compose, motions.START_AT, [0.0, None]
    real_render, marks = cross_section.render, []

    def render(*a, **k):
        marks.append(k.get('pointer_deg'))
        return real_render(*a, **k)

    def compose(rig, origin, console, v):
        n = len(marks)
        out = real(rig, origin, console, v)
        st = v['state'] or {}
        pairs = max(1.0, v['params'].get('motor_pole_pairs') or 1.0)
        theta = rig.board.drive.model.read().get('theta', 0.0)
        step = ((theta - shaft[1] + math.pi) % math.tau - math.pi) if shaft[1] is not None else 0.0
        shaft[0], shaft[1] = shaft[0] + math.degrees(step) / pairs, theta
        rows.append((time.monotonic(), v.get('segment'), v.get('stage'),
                     st.get('omega_hat', 0.0) / pairs * 60.0 / math.tau,
                     math.hypot(st.get('id', 0.0), st.get('iq', 0.0)), shaft[0],
                     bool((v.get('budget') or {}).get('throttling')),
                     marks[-1] if len(marks) > n else None, pairs))
        return out
    seconds = sum(stage[2] for stage in motions.CYCLE if stage[0] == segment) + extra
    view.compose, motions.START_AT, cross_section.render = compose, segment, render
    try:
        page.frame('rotor_observer', 150, 44, frames=int(seconds * FPS_CAP))
    finally:
        view.compose, motions.START_AT, cross_section.render = real, was, real_render
    rows = [r for r in rows if r[1] == segment]
    stages = []
    for row in rows:
        if not stages or stages[-1][0] != row[2]:
            stages.append((row[2], []))
        stages[-1][1].append(row)
    rate = (len(rows) - 1) / max(1e-9, rows[-1][0] - rows[0][0]) if len(rows) > 1 else 0.0
    return stages, rate >= FPS_CAP / 2.0, rate


def paced(report, name, kept, rate):
    """Whether the page kept half its rate for `name`'s physics - the loop steps a frame."""
    if not kept:
        report.skip(name, 'the page drew %.1f frames a second, under half its rate' % rate)
    return kept


def test_servo_moves_and_stops(report):
    """SERVO: out, stop, back, stop - each move up to its speed its way, each stop at rest."""
    stages, kept, rate = run('SERVO')
    if not paced(report, 'the servo\'s moves', kept, rate):
        return
    moves = [st for name, st in stages if name == 'move']
    stops = [st for name, st in stages if name == 'stop']
    reached = [max((r[3] for r in st), key=abs) for st in moves]
    ends = [abs(st[-1][3]) for st in stops]
    report.check('each move reaches 70 % of its 900 rpm, its way; each stop ends under a tenth',
                 len(reached) >= 2 and all(abs(a) >= 630.0 for a in reached)
                 and reached[0] * reached[1] < 0.0 and bool(ends) and max(ends) <= 90.0,
                 'moves %s rpm, stops at %s' % (['%.0f' % a for a in reached],
                                               ['%.0f' % e for e in ends]))


def test_stepper_steps_its_way(report):
    """STEPPER: a held vector stepped 15 degrees at a time, the rotor after it each way."""
    from terminal.views.rotor import motions
    stages, kept, rate = run('STEPPER')
    if not paced(report, 'the stepper\'s steps', kept, rate):
        return
    runs = [st for name, st in stages if name in ('steps', 'back')]
    asked = [(s[3] or 0.0) * 6.0 * s[2] for s in motions.CYCLE
             if s[0] == 'STEPPER' and s[1] in ('steps', 'back')]
    went = [st[-1][5] - st[0][5] for st in runs]
    # Within 0.7-1.3 of the command each way: a rotor that slipped past its vector went 1.8
    # times the turn back and still passed a floor alone (2026-09-28).
    report.check('the rotor steps each run\'s commanded turn to 30 %, its way',
                 len(went) == len(asked) == 2
                 and all(0.7 <= g / a <= 1.3 for g, a in zip(went, asked) if a),
                 'went %s of %s degrees' % (['%.0f' % g for g in went],
                                            ['%.0f' % a for a in asked]))
    # The mark after the vector, a step at a time: drawn off the angle over the pole pairs it
    # skipped a pitch, 51 degrees, at each electrical turn, twice a run (2026-09-28).
    drawn = [[r[7] for r in st if r[7] is not None] for st in runs]
    leaps = [max((abs(b - a) for a, b in zip(m, m[1:])), default=0.0) for m in drawn]
    turns = [m[-1] - m[0] if m else 0.0 for m in drawn]
    pitch = 360.0 / (runs[0][0][8] if runs and runs[0] else 1.0)
    report.check('and the mark goes with it, never 0.4 of a pitch at once',
                 len(turns) == len(asked) == 2 and max(leaps) < 0.4 * pitch
                 and all(0.7 <= t / a <= 1.3 for t, a in zip(turns, asked) if a),
                 'the mark %s of %s degrees, its largest step %.1f' % (
                     ['%.0f' % t for t in turns], ['%.0f' % a for a in asked], max(leaps)))


def test_fixed_wing_climbs_blips_and_glides(report):
    """FIXED WING: a slow climb against the propeller, cruise, a blip over it, a glide."""
    stages, kept, rate = run('FIXED WING')
    if not paced(report, 'the fixed wing\'s flight', kept, rate):
        return
    by = {}
    for name, st in stages:
        by.setdefault(name, []).append(st)
    climb, glide = by.get('climb', [[]])[0], by.get('glide', [[]])[0]
    cruise_rpm = abs(climb[-1][3]) if climb else 0.0
    # Each blip from where it began: the envelope throttling the cruise, it sags under the
    # climb's top, and a blip judged against that top read as none (2 419 of 2 740).
    # A blip the envelope held - the clamp throttled to the cruise's own current - lifts
    # nothing, and says so: judged, it is the throttle's to show (2026-09-28).
    blips = by.get('blip', [])
    lifts = [max(abs(r[3]) for r in st) - abs(st[0][3]) for st in blips]
    held = [any(r[6] for r in st) for st in blips]
    report.check('the climb comes to 85 % of cruise, and each blip lifts it past where it '
                 'was unless the envelope holds the clamp',
                 cruise_rpm >= 0.85 * 2800.0 and bool(lifts)
                 and all(lift > 0.0 or throttled for lift, throttled in zip(lifts, held)),
                 'climbed to %.0f rpm, the blips %s' % (cruise_rpm, ', '.join(
                     '%+.0f rpm%s' % (lift, ' throttled' if throttled else '')
                     for lift, throttled in zip(lifts, held))))
    # Past its first 0.3 s: the frames before the setpoint lands carry the cruise's current.
    let_go = [r for r in glide if r[0] - glide[0][0] >= 0.3] if glide else []
    report.check('and the glide slows on the drag and the propeller alone',
                 bool(let_go) and abs(glide[-1][3]) < abs(glide[0][3])
                 and max(r[4] for r in let_go) < 2.0,
                 '%.0f -> %.0f rpm, %.1f A at most' % (glide[0][3], glide[-1][3],
                                                       max(r[4] for r in let_go))
                 if let_go else 'none')


def test_quad_stabs_either_way(report):
    """QUAD: a hover and stabs either side of it - each stab moves the speed its way."""
    from terminal.views.rotor import motions
    stages, kept, rate = run('QUAD')
    if not paced(report, 'the quad\'s stabs', kept, rate):
        return
    targets = [float(s[3] or 0.0) for s in motions.CYCLE if s[0] == 'QUAD' and s[1] == 'stab']
    moved = [st[-1][3] - st[0][3] for name, st in stages if name == 'stab']
    report.check('each stab moves the speed toward its target',
                 len(moved) == len(targets) and all(
                     m * (t - 2000.0) > 0.0 for m, t in zip(moved, targets)),
                 ', '.join('%+.0f rpm toward %.0f' % mt for mt in zip(moved, targets)))


def test_joint_holds_against_its_load(report):
    """JOINT: the vector held against gravity's load, the current flowing all the while."""
    from terminal.views.rotor import motions
    stages, kept, rate = run('JOINT', extra=1.0)
    if not paced(report, 'the joint\'s hold', kept, rate):
        return
    held = [r for name, st in stages if name == 'hold' for r in st][10:]
    if not held:
        report.check('the joint holds', False, 'no frames of its hold')
        return
    wander = max(r[5] for r in held) - min(r[5] for r in held)
    report.check('the joint holds its angle within 20 degrees on the vector\'s current',
                 wander < 20.0 and min(r[4] for r in held) >= 0.8 * motions.HOLD_A,
                 '%.1f degrees of travel, %.1f A at least' % (wander, min(r[4] for r in held)))


ROSTER = (test_servo_moves_and_stops, test_stepper_steps_its_way,
          test_fixed_wing_climbs_blips_and_glides, test_quad_stabs_either_way,
          test_joint_holds_against_its_load)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped'
          % (report.passed, report.failed, getattr(report, 'skipped', 0)))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
