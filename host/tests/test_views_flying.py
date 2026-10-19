"""The crafts' flight run apart (`terminal.views.quad.flying`) and played back on their page."""
import sys

from tools.dev.focus import chosen
from views_kit import Report


def test_the_flight_apart_keeps_to_what_is_shown(report):
    """The page's flight in its own process (`flying.Flying`): its first state at its start, none
    past LEAD_S of what the page has shown while it shows nothing, a state a STEP_S as the page
    shows it in real time and none ahead of the wall; A begins it again, a new epoch."""
    import time
    from machine import apart as world_apart, quad
    from terminal.views.quad import flight as view
    from terminal.views.quad import flying
    lead = world_apart.LEAD_S
    apart = flying.Flying(quad, view, 4)
    try:
        said = apart.first()
        time.sleep(0.5)
        said += apart.said()
        held = max(s['t'] for s in said)
        report.check('shown nothing, it waits LEAD_S past its start',
                     said[0]['t'] == 0.0 and held <= lead + view.STEP_S + 1e-9,
                     '%d states to %.2f s' % (len(said), held))
        began, ahead, spans, times = time.monotonic(), 0.0, set(), [said[-1]['t']]
        while time.monotonic() - began < 1.5:
            apart.shown(held + time.monotonic() - began)
            time.sleep(0.05)
            for s in apart.said():
                spans.add(round(s['t'] - times[-1], 9))
                times.append(s['t'])
                ahead = max(ahead, s['t'] - (held + time.monotonic() - began))
        ran = times[-1] - held
        report.check('shown in real time, a state a STEP_S, none LEAD_S past it nor ahead of the wall',
                     spans == {view.STEP_S} and ahead <= lead + view.STEP_S
                     and 1.2 <= ran <= 1.5 + lead + view.STEP_S,
                     '%.2f s flown on 1.5 s of the wall, %.3f s at most past the shown, steps %s'
                     % (ran, ahead, sorted(spans)))
        apart.send(reset=True)
        epochs = set()
        until = time.monotonic() + 3.0
        while time.monotonic() < until and 1 not in epochs:
            apart.shown(held + time.monotonic() - began)
            time.sleep(0.05)
            epochs |= {s['epoch'] for s in apart.said()}
        report.check('A begins it again: a new epoch said', 1 in epochs, str(sorted(epochs)))
    finally:
        apart.close(wait=3.0)


def test_the_page_plays_it_back_never_ahead(report):
    """The page's playback of the flight (`flying.blend`, no catch-up): its time on at the wall's
    while a frame comes in STEADY_S, a stall played as STEADY_S, never past the newest state; between
    two states the turn a rotation and a rotor's angle on at its speed."""
    import math
    import numpy as np
    from machine.apart import LEAD_S
    from terminal.views.playback import STEADY_S, Playback
    from terminal.views.quad import flying

    def state(t, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        turn = np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
        zero = np.zeros(3)
        return {'t': t, 'epoch': 0, 'frame': {'h': t, 'v': 1.0, 'a': 0.0, 'at': zero + t, 'vel': zero,
                                               'spin': zero, 'acc': zero, 'turn': turn},
                'rotors': [{'w': 100.0, 'angle': (100.0 * t) % math.tau}]}
    playback, made = Playback.steady(flying.blend), [0]

    def flown(upto):
        """The states the flight says up to `upto`: LEAD_S past what is shown, as it is held."""
        while 0.02 * made[0] <= upto + 1e-9:
            playback.push([state(0.02 * made[0], 0.5 * made[0])])
            made[0] += 1
    flown(LEAD_S)
    shown, wall, past = [], 0.0, 0.0
    for gap in [0.05] * 6 + [0.6] + [0.05] * 4:
        wall += gap
        shown.append(playback.at(wall))
        past = max(past, shown[-1]['t'] - 0.02 * (made[0] - 1))
        flown(shown[-1]['t'] + LEAD_S)
    steps = [b['t'] - a['t'] for a, b in zip(shown, shown[1:])]
    report.check('on at the wall\'s, a stall played as STEADY_S, never past the newest',
                 all(abs(d - 0.05) < 1e-9 for d in steps[:5] + steps[6:])
                 and abs(steps[5] - STEADY_S) < 1e-9 and past <= 0.0,
                 ' '.join('%.3f' % d for d in steps))
    one = shown[3]
    turn = one['frame']['turn']
    report.check('between two states the turn a rotation, a rotor on at its speed',
                 np.allclose(turn @ turn.T, np.eye(3)) and abs(np.linalg.det(turn) - 1.0) < 1e-9
                 and abs((one['rotors'][0]['angle'] - 100.0 * one['t'] + math.pi) % math.tau - math.pi) < 1e-6,
                 't %.3f, angle %.4f' % (one['t'], one['rotors'][0]['angle']))


def test_the_frames_leave_on_their_beat(report):
    """`screen.run_view`'s beat: a frame's draw 20 to 45 ms at random, yet each frame's bytes
    leave at the instant it drew (`screen.drawn_at`) and the frames leave on an even beat - a
    page playing a flight back shows it as it is seen."""
    import io
    import random
    import statistics
    import time
    import rich.console
    from rich.text import Text
    from terminal.ui import screen
    writes, drawn, dice = [], [], random.Random(3)

    class Sink(io.StringIO):
        def write(self, s):
            writes.append((time.perf_counter(), len(s)))
            return len(s)

        def isatty(self):
            return True

    def draw():
        drawn.append(screen.drawn_at())
        time.sleep(dice.uniform(0.02, 0.045))
        return Text('%.6f' % drawn[-1])
    screen.run_view(rich.console.Console(file=Sink(), force_terminal=True, width=80, height=10,
                                         color_system='truecolor'), False, 0.05, 40, draw)
    sent = [t for t, n in writes if n > 200][:len(drawn)]
    off = [t - d for t, d in zip(sent, drawn)][5:]
    gaps = [b - a for a, b in zip(sent[5:], sent[6:])]
    change = sorted(abs(b - a) for a, b in zip(gaps, gaps[1:]))
    report.check('each frame leaves at the instant it drew, on an even beat',
                 len(sent) == len(drawn) and max(abs(o) for o in off) <= 0.008
                 and change[int(0.9 * (len(change) - 1))] <= 0.005,
                 'left %.1f ms from its instant at most, %.1f ms apart, gap to gap %.1f ms at p90' % (
                     1e3 * max(abs(o) for o in off), 1e3 * statistics.median(gaps),
                     1e3 * change[int(0.9 * (len(change) - 1))]))


def test_the_drawing_is_a_frame_ahead(report):
    """`ahead.Ahead`: a render in a process of its own, each frame given what the frame before
    asked - the first its own."""
    from terminal.ui.ahead import Ahead
    ahead = Ahead('builtins', 'dict')
    try:
        got = [ahead.drawn(k=k) for k in range(4)]
    finally:
        ahead.close()
    report.check('each frame the drawing the frame before asked, the first its own',
                 got == [{'k': 0}, {'k': 0}, {'k': 1}, {'k': 2}], str(got))



ROSTER = (test_the_flight_apart_keeps_to_what_is_shown, test_the_page_plays_it_back_never_ahead,
          test_the_frames_leave_on_their_beat, test_the_drawing_is_a_frame_ahead)


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
