#!/usr/bin/env python3
"""Flight intent as text, compiled and flown: `machine.aerolisp`, `machine.intents`, aero/.

The power loop's four streams as payloads; a stream laid over the one before it; what is no
stream refused; the four flown on ideal rotors in MuJoCo (`tools/sim/streams.py`) and on four
stand-in boards; a hover's stream in the tour's air; a hover asked from upsets."""
import math
import struct
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The streams' payloads as they compile on a frame of 100 m/s^2 at the most: (sequence,
#: blend ms, the targets - None a NaN -, the stiffnesses, the dampings, the oscillator on, its
#: Hz, its amplitudes, the constraints' mask, their values).
N = None
COMPILED = (
    (201, 0, (9.81, 0.0, 0.0, 0.0, N, N), (0, 5000, 5000, 0, 0, 0), (80,) * 6, 0, 0.0,
     (0,) * 6, 1, (1.5, N)),
    (202, 400, (18.5, -math.pi / 4.0, 0.0, 0.0, N, N), (0, 5000, 1500, 0, 0, 0),
     (80, 80, 25, 80, 80, 80), 1, 2.0, (0, 0, math.radians(35.0), 0, 0, 0), 0, (N, N)),
    (203, 100, (2.0, N, 0.0, 0.0, -314.0, N), (0, 5000, 1500, 0, 4000, 0),
     (80, 80, 25, 80, 80, 80), 0, 0.0, (0,) * 6, 0, (N, N)),
    (204, 50, (100.0, math.radians(80.0), 0.0, 0.0, 0.0, N), (0, 8000, 1500, 0, 4000, 0),
     (80, 100, 25, 80, 80, 80), 0, 0.0, (0,) * 6, 2, (N, 2.5)),
)

#: Stream 203 alone, as its author's compiler packs it: 116 bytes, hex.
ALONE_203 = ('cb00000064000000' '00000040' + '0000c07f' * 3 + '00009dc3' '0000c07f'
             + '00000000' * 4 + '00007a45' '00000000' + '00000000' * 6
             + '00000000' '00000000' + '00000000' * 6 + '00000000')


def _same(got, want):
    return all((math.isnan(g) and w is None) or (w is not None and abs(g - w) < 1e-6)
               for g, w in zip(got, want))


def test_the_power_loop_compiles(report):
    """`intents.POWER_LOOP`: four payloads, each laid over the one before it - a target, a
    stiffness, a damping and the oscillator stand where a stream names none, `ignore` is a
    NaN, degrees are rad, MAX_SYS_THRUST the frame's number, a stream's constraints its own
    with their heights; a payload packs to aero_payload_t's 124 bytes."""
    from machine import aerolisp, intents
    from tools.cores import aero
    payloads = aerolisp.compiled(intents.POWER_LOOP, {'MAX_SYS_THRUST': 100.0})
    report.check('four streams, four payloads', len(payloads) == len(COMPILED),
                 ' '.join('%d' % p.sequence_id for p in payloads))
    for p, want in zip(payloads, COMPILED):
        got = (p.sequence_id, p.blend_duration_ms, p.cpg_enabled, p.task_constraints_mask)
        report.check('stream %d as its rows' % want[0],
                     got == (want[0], want[1], want[5], want[8])
                     and _same(p.latent_targets, want[2]) and _same(p.stiffness, want[3])
                     and _same(p.damping, want[4]) and abs(p.cpg_frequency - want[6]) < 1e-9
                     and _same(p.cpg_amplitudes, want[7]) and _same(p.task_constraints, want[9]),
                     'targets %s' % ' '.join('%.3g' % x for x in p.latent_targets))
        report.check('stream %d packs to the core\'s payload' % want[0],
                     len(p.pack()) == 124 == struct.calcsize(aero.PACK)
                     and aero.payload(p).sequence_id == want[0]
                     and abs(aero.payload(p).damping[2] - want[4][2]) < 1e-6)


def test_a_stream_alone_is_its_authors_bytes(report):
    """Stream 203 compiled on its own, as its author's compiler was shown it: the same 116
    bytes, the two constraint heights - NaN, none asked - after them."""
    from machine import aerolisp
    stream = {'sequence_id': 203, 'blend_duration': '100ms', 'cpg_config': {'enable': False},
              'latent_targets': [('mode_collective_thrust', 2.0),
                                 ('mode_attitude_pitch', 'ignore'),
                                 ('mode_pitch_rate', '-314.0rad_s')],
              'impedance': [('stiffness', 'mode_pitch_rate', 4000.0)]}
    blob = aerolisp.AeroLispCompiler().compile(stream).pack()
    report.check('its 116 bytes as written', blob[:116].hex() == ALONE_203, blob[:24].hex())
    report.check('the heights after them: none', blob[116:].hex() == '0000c07f' * 2,
                 blob[116:].hex())


def test_what_is_no_stream_is_refused(report):
    """A mode, a block, a unit, a constraint or a symbol there is not, and a text whose
    brackets do not close: each refused in words, none compiled to something else."""
    from machine import aerolisp
    cases = (
        ('a mode', '(stream_inject (latent_targets (mode_attitude_bank 3deg)))', 'no mode'),
        ('a block', '(stream_inject (waypoints (go 3m)))', 'no block'),
        ('a unit', '(stream_inject (latent_targets (mode_attitude_pitch 3furlongs)))',
         'no value'),
        ('a symbol', '(stream_inject (latent_targets (mode_collective_thrust FULL)))',
         'no value'),
        ('a constraint', '(stream_inject (task_constraints (stay_dry 1m)))', 'no task'),
        ('an impedance', '(stream_inject (impedance (mass mode_yaw_rate 3)))', 'stiffness'),
        ('a form', '(inject (sequence_id 1))', 'stream_inject'),
        ('an open bracket', '(stream_inject (sequence_id 1)', 'never closed'),
    )
    for name, text, says in cases:
        try:
            aerolisp.compiled(text)
            said = None
        except SyntaxError as error:
            said = str(error)
        report.check('%s there is not: refused' % name, said is not None and says in said,
                     said or 'compiled')
    payload = aerolisp.compiled('(stream_inject (blend_duration 0.4s) (latent_targets '
                                '(mode_yaw_rate 90deg) (mode_attitude_roll 0.5rad)))')[0]
    report.check('s, deg and rad compiled to ms and rad',
                 payload.blend_duration_ms == 400
                 and abs(payload.latent_targets[3] - math.pi / 2.0) < 1e-9
                 and payload.latent_targets[2] == 0.5)


#: The catch is flown till the rotors' axis has stood this near the one asked, degrees, and
#: no longer than this, s: a frame turning 80 rad/s is stopped and brought round in 0.62.
NEAR_DEG, CATCH_S = 10.0, 0.8


def _swung(dive, passes):
    """(how far ahead the rotors' axis points at the mean, the least and the most aside,
    degrees, its swings from one side to the other) over the dive's last `passes`."""
    ahead, aside = ([math.degrees(math.asin(float(s['turn'][row][1])))
                     for _t, s, _f, _b in dive['passes'][-passes:]] for row in (2, 0))
    return (sum(ahead) / len(ahead), min(aside), max(aside),
            sum(1 for a, b in zip(aside, aside[1:]) if a < 0.0 <= b))


def test_the_power_loop_is_flown(report):
    """The four streams on ideal rotors from the floor, each its seconds - the catch till the
    frame is caught -, and the hover asked again: it stands at its height; dives ahead, its
    rotors' axis swung either side; turns over past its back; is caught, that axis the 80
    degrees off up that were asked; stands at its height again - nothing struck, whatever the
    turn's length of those tried."""
    from machine import aerolisp, intents, quad
    from tools.cores import aero
    from tools.sim import streams
    compiled = aerolisp.compiled(intents.POWER_LOOP, streams.symbols())
    for turn in (intents.POWER_LOOP_S[2], 0.03, 0.05, 0.12):
        payloads = [aero.payload(p) for p in compiled]
        again = aero.payload(compiled[0])
        again.sequence_id = 205
        flight = streams.Flight()
        hover, dive, over = streams.flown(flight, payloads[:3],
                                          list(intents.POWER_LOOP_S[:2]) + [turn])
        catch, = streams.flown(flight, payloads[3:], [CATCH_S], near=NEAR_DEG)
        stood, = streams.flown(flight, [again], [4.0])
        if turn == intents.POWER_LOOP_S[2]:
            report.check('201: from the floor to 1.5 m, level and still',
                         abs(hover['ends']['h'] - 1.5) < 0.01 and hover['off'] < 0.1
                         and hover['heights'][1] < 1.52,
                         '%.3f m, %.2f deg off, %.2f m at the most' % (
                             hover['ends']['h'], hover['off'], hover['heights'][1]))
            ahead, least, most, swings = _swung(dive, int(round(1.0 / flight.dt)))
            report.check('202: its rotors\' axis 45 degrees ahead and what it leans into its own '
                         'wind, swung either side twice a second',
                         40.0 < ahead < 55.0 and swings == 2 and 28.0 < most < 42.0
                         and -42.0 < least < -28.0,
                         '%.1f deg ahead at the mean, %.1f to %.1f aside, %d swings a second' % (
                             ahead, least, most, swings))
            report.check('202: forward and up on it',
                         float(dive['ends']['vel'][2]) > 12.0 and dive['ends']['h'] > 2.0,
                         '%.1f m/s ahead, %.2f m up' % (dive['ends']['vel'][2], dive['ends']['h']))
        report.check('a turn of %.2f s: every rotor at an end of its reach, over past its back '
                     'where it was long enough' % turn,
                     over['bound'] == 1.0 and (catch['offs'][1] > 150.0 or turn < 0.05),
                     '%.0f deg off at the most' % catch['offs'][1])
        report.check('a turn of %.2f s: caught, its axis the 80 degrees asked off up' % turn,
                     catch['caught'] is not None and abs(catch['off'] - 80.0) < NEAR_DEG,
                     'in %s s, %.1f deg off, rates to %.1f rad/s' % (
                         '%.2f' % catch['caught'] if catch['caught'] else 'over %.1f' % CATCH_S,
                         catch['off'], catch['spin']))
        report.check('a turn of %.2f s: standing at its height again, nothing struck' % turn,
                     abs(stood['ends']['h'] - 1.5) < 0.02 and stood['ends']['hit'] is None
                     and max(abs(float(x)) for x in stood['ends']['spin']) < 0.1
                     and stood['heights'][0] > quad.FLOOR_M,
                     '%.3f m, rates %.2f rad/s, %.2f m at the lowest' % (
                         stood['ends']['h'], max(abs(float(x)) for x in stood['ends']['spin']),
                         stood['heights'][0]))


def test_the_power_loop_on_four_boards(report):
    """The four streams and the hover again on four stand-in boards, the core's pass each of
    the world's steps inside the flight's: stood, turned over, caught and stood again as on
    the ideal rotors; no drive tripped, no node of a board past its throttle's point - asked
    all the rotors give, each board holds its own clamp."""
    from coaxial.devices.thermal import THROTTLE_AT
    from machine import aerolisp, intents
    from terminal.views.quad import flight as view
    from tools.cores import aero
    from tools.sim import streams
    compiled = aerolisp.compiled(intents.POWER_LOOP, streams.symbols())
    payloads = [aero.payload(p) for p in compiled]
    again = aero.payload(compiled[0])
    again.sequence_id = 205
    flight = streams.Boards()
    try:
        hover, dive, _over = streams.flown(flight, payloads[:3], list(intents.POWER_LOOP_S[:3]))
        catch, = streams.flown(flight, payloads[3:], [CATCH_S], near=NEAR_DEG)
        stood, = streams.flown(flight, [again], [4.0])
    finally:
        flight.close()
    ahead, least, most, swings = _swung(dive, int(round(1.0 / flight.dt)))
    report.check('201 and 202 on the boards: to 1.5 m, then its axis ahead, swung, and gone',
                 abs(hover['ends']['h'] - 1.5) < 0.01 and 40.0 < ahead < 55.0 and swings == 2
                 and most > 25.0 and least < -25.0 and float(dive['ends']['vel'][2]) > 12.0,
                 '%.3f m; %.1f deg ahead, %.1f to %.1f aside, %.1f m/s ahead' % (
                     hover['ends']['h'], ahead, least, most, dive['ends']['vel'][2]))
    report.check('203 and 204: over past its back, caught, its axis the 80 degrees asked off up',
                 catch['offs'][1] > 150.0 and catch['caught'] is not None
                 and abs(catch['off'] - 80.0) < NEAR_DEG,
                 '%.0f deg off at the most, caught in %s s, %.1f off' % (
                     catch['offs'][1], '%.2f' % catch['caught'] if catch['caught'] else '-',
                     catch['off']))
    report.check('standing at its height again, nothing struck',
                 abs(stood['ends']['h'] - 1.5) < 0.02 and stood['ends']['hit'] is None
                 and max(abs(float(x)) for x in stood['ends']['spin']) < 0.1,
                 '%.3f m, rates %.2f rad/s' % (
                     stood['ends']['h'], max(abs(float(x)) for x in stood['ends']['spin'])))
    boards = [row['boards'] for row in (hover, dive, _over, catch, stood)]
    report.check('no drive tripped, no node past its throttle\'s point, the clamp kept',
                 sum(b['tripped'] for b in boards) == 0
                 and max(b['soa'] for b in boards) <= THROTTLE_AT + 1.0 / 255.0
                 and max(b['amps'] for b in boards) < view.I_MAX + 3.0
                 and catch['boards']['hold'] < view.I_MAX,
                 'SOA %.2f and %.0f A at the most, %.0f A held at the least in the catch' % (
                     max(b['soa'] for b in boards), max(b['amps'] for b in boards),
                     catch['boards']['hold']))


def test_a_hover_keeps_still_in_the_air(report):
    """A hover's stream through the tour's five kinds of air, 100 s, no place asked: the frame
    leans into the wind and keeps its speed along the floor under 1 m/s, its height to the
    centimetre; with no lean left it (`aero.LEAN` 0) it stands level and goes with the wind."""
    from machine import aerolisp, quad
    from tools.cores import aero
    from tools.sim import streams
    payload = aero.payload(aerolisp.compiled(streams.HOVER % 5.0)[0])
    got = {}
    for lean in (aero.LEAN, 0.0):
        kept, aero.LEAN = aero.LEAN, lean
        try:
            flight = streams.Flight(air=quad.air(3), at=(0.0, 5.0, 0.0))
        finally:
            aero.LEAN = kept
        passes = streams.flown(flight, [payload], [100.0])[0]['passes']
        got[lean] = (max(math.hypot(float(s['vel'][0]), float(s['vel'][2]))
                         for _t, s, _f, _b in passes),
                     max(streams.tilt(s['turn'])[2] for _t, s, _f, _b in passes),
                     max(abs(s['h'] - 5.0) for _t, s, _f, _b in passes), passes[-1][1]['hit'])
    speed, off, height, hit = got[aero.LEAN]
    report.check('leaning into the wind: its speed along the floor under 1 m/s',
                 speed < 1.0 and 3.0 < off < 15.0 and hit is None,
                 '%.2f m/s and %.1f deg at the most' % (speed, off))
    report.check('its height to the centimetre', height < 0.02, '%.3f m off at the most' % height)
    report.check('with no lean left it: level, and gone with the wind',
                 got[0.0][0] > 2.0 and got[0.0][1] < 1.0,
                 '%.2f m/s and %.2f deg at the most' % got[0.0][:2])


def test_a_hover_is_stood_from_an_upset(report):
    """A hover asked of the frame tilted 30, 90 and 150 degrees and turning 0, 6 and 9 rad/s
    about its rotors' axis: it stands from each, losing under a metre of its height."""
    from tools.sim import streams
    for off in streams.TILTS:
        rows = [(turning,) + streams.upset(off, turning) for turning in streams.TURNS]
        report.check('%3.0f degrees off: stood from every turn, under a metre lost' % off,
                     all(stood is not None and hit is None and low > streams.UPSET_M - 1.0
                         for _w, stood, _rest, low, hit in rows),
                     ', '.join('%.0f rad/s %s' % (w, '%.2f s' % stood if stood else 'never')
                               for w, stood, _rest, _low, _hit in rows))


ROSTER = (test_the_power_loop_compiles, test_a_stream_alone_is_its_authors_bytes,
          test_what_is_no_stream_is_refused, test_the_power_loop_is_flown,
          test_the_power_loop_on_four_boards, test_a_hover_keeps_still_in_the_air,
          test_a_hover_is_stood_from_an_upset)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
