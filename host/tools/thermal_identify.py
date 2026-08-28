"""System identification for the thermal observer: drive states, fit, compare.

Four states, each a step change in what the board dissipates. The NTC is
logged through all of them - including the AFE-off ones, which used to be
blind, because the observer now borrows the rail for a sample and gives it
back (`board_power.h`).

    1 passive  AFE off -> the drivers HAVE SUPPLY (the gate is inverted), no PWM
    2 afe      AFE on  -> drivers unpowered, sensors alive, no traffic
    3 traffic  AFE on  + DAQ at full tilt and data pumped off the board
    4 switch   AFE off + three legs at 50 %

WHAT COMES OUT, AND WHAT DOES NOT
The transient gives tau directly, with no power measurement involved. The
differences between plateaux give the power of each subsystem RELATIVE to the
others. What neither gives is an absolute K/W: that needs one trusted power
number, and this bench does not have one - the supply's shunt is not trusted
by its owner. So absolute figures are printed as conditional on the assumed
passive power and labelled that way. Invariant 10: the board reports, the
host judges, and a number nobody measured says so.

TAU IS FITTED FROM THE WHOLE CURVE, NOT THE PLATEAU
dT/dt = (T_inf - T) / tau is linear in T, so a regression of the rate against
the temperature gives both tau and the asymptote without a nonlinear solver
and without waiting for the plateau. An earlier fit took the mean of a
3.5-minute window far from the asymptote and called it the equilibrium; it
was wrong, and the runs that were meant to start cold all started warm.

    python tools/thermal_identify.py                    # all four, 25 min each
    python tools/thermal_identify.py --minutes 8        # quicker, less settled
    python tools/thermal_identify.py --states passive,afe
    python tools/thermal_identify.py --apply            # push the fit to the board
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from screen import say                                       # noqa: E402

from coaxial import Coaxial63100                             # noqa: E402
from coaxial.errors import NoReplyError, RigError            # noqa: E402
from coaxial.thermal import STATE_IS as WHAT
from coaxial.thermal import (CFG, NTC_OFFSET, STATES,
                             tau_minutes)     # noqa: E402
from thermal_validate import CAMERA                          # noqa: E402


#: The one power number this bench has, and it is not trusted - the owner
#: says so. Everything absolute below is conditional on it and prints as such.
ASSUMED_PASSIVE_W = 24.0 * 0.050


def ntc(rig, tries=8):
    """The NTC now, taking the rail if it has to. None if it stayed quiet.

    NOT the observer's stored sample. Measured 2026-08-28: reading
    `thermal.state()['ntc']` gave 36.36 C on every one of eleven samples in
    the switching state - the observer refuses to sample while the stage is
    armed, so the field was the same stale value each time and the run looked
    like a board that had stopped responding to heat.

    The tool measures; the observer estimates. It has to take its own
    reading, and it can: the caller has already brought the stage down.
    """
    for _ in range(tries):
        try:
            was_on = rig.board.afe.is_on()
            if not was_on:
                rig.board.afe.enable()
                time.sleep(0.5)          # the reference, measured
            try:
                return rig.board.analog.ntc_temperature()['celsius']
            finally:
                if not was_on:
                    insist(rig.board.afe.disable)
        except (NoReplyError, RigError):
            time.sleep(0.4)
    return None


def insist(what, tries=10, pause=0.4):
    """Run it until the link answers. Raises if it never does.

    THE LINK GOES QUIET NOW AND THEN - FINDINGS has it open, and 600 requests
    ruled out four causes. Measured 2026-08-28: a 60-minute run died at
    minute 47 because the disarm/arm around a sample had no tolerance while
    the NTC read beside it had. Everything that touches the gate stage goes
    through here.
    """
    last = None
    for _ in range(tries):
        try:
            return what()
        except (NoReplyError, RigError) as exc:
            last = exc
            time.sleep(pause)
    raise RigError('the link stayed silent through %d tries: %s'
                   % (tries, last))


def sample_while_switching(rig, load):
    """Disarm, read, arm again. The firmware refuses a sample while armed.

    That refusal is right - AFE_ON high takes the drivers' supply away, and
    six inputs switching into unpowered drivers is not a measurement worth
    having. So the stage comes down first, in hardware, through MOE.

    The disarm is the step that must not be skipped and the arm is the one
    that must not be half-done, so both are stubborn. Losing the reading is
    one lost point; losing the arm leaves the run reporting a state it is no
    longer in.
    """
    insist(rig.gates.disarm)
    got = ntc(rig)
    insist(lambda: rig.gates.arm(bypass_sto=True, ignore_interlock=True))
    insist(lambda: rig.write(analog=load))
    return got


def enter(rig, state):
    """Put the board in `state`. Returns the load to re-apply after a sample."""
    if state == 'switch':
        insist(rig.board.afe.disable)
        insist(lambda: rig.gates.arm(bypass_sto=True, ignore_interlock=True))
        load = {'Phase %s' % leg: 0.50 for leg in ('U', 'V', 'W')}
        insist(lambda: rig.write(analog=load))
        return load

    if state == 'passive':
        insist(rig.board.afe.disable)
        return None

    insist(rig.board.afe.enable)
    if state == 'traffic':
        rig.configure(accumulate=1, digital=True)
        rig.start()
    return None


def leave(rig, state, load):
    for undo in _undo_steps(rig, state, load):
        try:
            undo()
        except (NoReplyError, RigError):
            pass            # one failed step must not skip the next


def _undo_steps(rig, state, load):
    if state == 'switch':
        return [lambda: rig.write(analog=dict.fromkeys(load, 0.0)),
                rig.gates.disarm]
    if state == 'traffic':
        return [rig.stop]
    return []


def observer(rig):
    """The observer's nodes and its own thermometers, or None if quiet."""
    for _ in range(6):
        try:
            return rig.board.thermal.state()
        except (NoReplyError, RigError):
            time.sleep(0.3)
    return None


def hold(rig, state, seconds, every):
    """Run one state and log (t, ntc). Returns (series, node series)."""
    load = enter(rig, state)
    series, nodes, start, last = [], {}, time.time(), 0.0
    try:
        while True:
            now = time.time() - start
            if now >= seconds:
                break
            if now - last >= every or not series:
                got = (sample_while_switching(rig, load) if load
                       else ntc(rig))
                last = now
                if got is not None:
                    series.append((now, got))
                    print('  %-8s %6.1f s   NTC %6.2f C'
                          % (state, now, got), flush=True)

                # The observer's own estimates, per node, on the same clock.
                # This is what is being judged: the NTC above is only what
                # it had to work from.
                st = observer(rig)
                if st is not None:
                    for name, value in st['nodes'].items():
                        nodes.setdefault(name, []).append((now, value))
            time.sleep(1.0)
    finally:
        leave(rig, state, load)
    return series, nodes


def fit(series):
    """(tau_seconds, asymptote) from the whole curve, or None.

    dT/dt = (T_inf - T)/tau is linear in T. Regressing the rate against the
    temperature gives -1/tau as the slope, so neither a plateau nor a solver
    is needed.
    """
    rates = []
    for (t0, a), (t1, b) in zip(series, series[1:]):
        dt = t1 - t0
        if dt > 0:
            rates.append(((a + b) / 2.0, (b - a) / dt))
    if len(rates) < 6:
        return None

    n = float(len(rates))
    mx = sum(t for t, _ in rates) / n
    my = sum(r for _, r in rates) / n
    sxx = sum((t - mx) ** 2 for t, _ in rates)
    sxy = sum((t - mx) * (r - my) for t, r in rates)
    if sxx <= 0.0 or sxy >= 0.0:
        return None                     # not a decaying exponential

    slope = sxy / sxx
    return (-1.0 / slope, (my - slope * mx) / -slope)


def report(runs):
    """What was measured, what it implies, and what it disagrees with."""
    print('\n%s\n%s' % ('measured', '-' * 60))
    print('  %-9s %7s %7s %9s %9s' % ('state', 'first', 'last', 'tau min',
                                      'asymptote'))
    fits = {}
    for state, series in runs.items():
        got = fit(series)
        fits[state] = got
        print('  %-9s %7.2f %7.2f %9s %9s'
              % (state, series[0][1], series[-1][1],
                 '%.1f' % (got[0] / 60.0) if got else '-',
                 '%.2f' % got[1] if got else '-'))

    print('\n%s\n%s' % ('against the camera, 2026-08-28', '-' * 60))
    print('  the NTC sits in the drivers\' hot spot, so the board is the NTC')
    print('  less the compensation - %.2f K offset, and the driver coupling.'
          % NTC_OFFSET)
    for state, series in runs.items():
        seen = CAMERA.get(STATE_TO_CAMERA.get(state, state), {})
        if 'ntc' in seen:
            print('  %-9s NTC model %6.2f   camera %6.2f   %+.2f K'
                  % (state, series[-1][1], seen['ntc'],
                     series[-1][1] - seen['ntc']))
        else:
            print('  %-9s camera recorded no NTC for this state' % state)

    _report_powers(fits)
    return fits


def _report_powers(fits):
    print('\n%s\n%s' % ('power, RELATIVE - the absolute rests on an untrusted '
                        'shunt', '-' * 60))
    base = fits.get('passive')
    if not base:
        print('  no passive fit, so nothing to difference against')
        return

    for state, label in (('afe', 'the AFE chain'),
                         ('traffic', 'full DAQ and link, over AFE'),
                         ('switch', 'switching alone')):
        got = fits.get(state)
        if not got:
            continue
        against = fits['afe'] if state == 'traffic' and fits.get('afe') else base
        rise = got[1] - against[1]
        watt = rise / CFG['board_to_ambient']
        print('  %-9s %+6.2f K -> %5.3f W   %s  (at %.2f K/W, ASSUMED)'
              % (state, rise, watt, label, CFG['board_to_ambient']))

    print('\n  passive itself is %.2f W ASSUMED (%.0f mA at %.0f V, untrusted)'
          % (ASSUMED_PASSIVE_W, 50, 24))


def apply_fit(rig, fits):
    """Push what was measured without a power number: the board's own tau."""
    got = fits.get('passive') or next((f for f in fits.values() if f), None)
    if not got:
        print('\nnothing fitted, so nothing applied')
        return

    # tau = capacity * to_ambient. Only their product was measured, so the
    # resistance is left as it was and the capacity carries the change -
    # the resistance is the one with a measurement of its own behind it.
    capacity = got[0] / CFG['board_to_ambient']
    print('\napplying: board capacity %.1f J/K (tau %.1f min at %.2f K/W)'
          % (capacity, got[0] / 60.0, CFG['board_to_ambient']))
    rig.board.thermal.set_board(CFG['board_to_ambient'], capacity)


#: What the camera saw, mapped onto the observer's node names. `dead` is the
#: board itself; `bridge` is the half-bridge, which the model splits into the
#: drivers and the phases and cannot tell apart.
#: The camera's own state names. They are not the rig's: two of the four
#: differ, and looking them up by the rig's name returned nothing while
#: reporting it as "the camera recorded none for this state" - a mismatch
#: that reads exactly like a measurement nobody took.
STATE_TO_CAMERA = {'passive': 'passive', 'afe': 'afe on',
                   'traffic': 'traffic', 'switch': 'switching'}


CAMERA_AS_NODES = {
    'dead': 'board', 'mcu': 'mcu', 'regulators': 'regulators',
    'afe': 'afe', 'bridge': 'drivers',
}


def asymptote(series):
    """Where a node is heading, from the transient. None if it will not fit."""
    got = fit(series)
    return got[1] if got else None


def against_camera(node_runs):
    """The observer's predicted equilibrium against what the camera saw.

    The ASYMPTOTE, not the last sample: a run of a few minutes against a
    6.8-minute constant is nowhere near equilibrium, and the camera numbers
    are equilibrium ones. The fit is what makes the two comparable.

    `hotswap` is not here because the model has no such node - it ran
    unloaded through the whole campaign and has no measurement behind it.
    """
    print('\n%s\n%s' % ('the observer against the camera, 2026-08-28',
                         '-' * 66))
    print('  %-9s %-11s %9s %9s %8s' % ('state', 'node', 'predicted',
                                        'camera', 'error'))
    errors = []
    for state, nodes in node_runs.items():
        seen = CAMERA.get(STATE_TO_CAMERA.get(state, state), {})
        for where, node in CAMERA_AS_NODES.items():
            if where not in seen or node not in nodes:
                continue
            want = asymptote(nodes[node])
            if want is None:
                print('  %-9s %-11s %9s %9.2f %8s'
                      % (state, node, 'no fit', seen[where], '-'))
                continue
            err = want - seen[where]
            errors.append(abs(err))
            print('  %-9s %-11s %9.2f %9.2f %+8.2f'
                  % (state, node, want, seen[where], err))

    if errors:
        print('\n  %d comparisons, mean |error| %.2f K, worst %.2f K'
              % (len(errors), sum(errors) / len(errors), max(errors)))
        print('  The model was FITTED to these same four states, so this is '
              'not\n  a prediction of anything new - it is whether the '
              'observer running\n  live reproduces the calibration it was '
              'given.')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('--minutes', type=float, default=25.0,
                   help='per state; 25 = 3.7 tau = 97 %% of the way there')
    p.add_argument('--states', default=','.join(STATES))
    p.add_argument('--every', type=float, default=10.0,
                   help='seconds between logged samples')
    p.add_argument('--apply', action='store_true',
                   help='write the fitted constants back to the board')
    a = p.parse_args()

    want = [s.strip() for s in a.states.split(',') if s.strip()]
    for state in want:
        if state not in STATES:
            raise SystemExit('unknown state %r - pick from %s'
                             % (state, ', '.join(STATES)))

    print('tau is %.1f min in the model; holding %.0f min per state (%.1f tau)'
          % (tau_minutes(), a.minutes, a.minutes / tau_minutes()))
    print('%d state(s), about %.0f min\n' % (len(want), len(want) * a.minutes))

    with Coaxial63100(port=a.port, power_afe=False) as rig:
        say('ok' if rig.origin.real else 'warn', 'link', rig.origin.label)
        runs, node_runs = {}, {}
        try:
            for state in want:
                print('\n--- %s: %s ---' % (state, WHAT[state]), flush=True)
                got, nodes = hold(rig, state, a.minutes * 60.0, a.every)
                if len(got) < 6:
                    print('  only %d samples - not enough to fit' % len(got))
                    continue
                runs[state] = got
                node_runs[state] = nodes
        except KeyboardInterrupt:
            print('\nstopped - reporting what was gathered')

        if not runs:
            raise SystemExit('no state produced enough samples')
        fits = report(runs)
        against_camera(node_runs)
        if a.apply:
            apply_fit(rig, fits)
    return 0


if __name__ == '__main__':
    sys.exit(main())
