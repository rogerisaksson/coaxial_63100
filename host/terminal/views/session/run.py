"""The session dash's run: its keys (PAD), the plan, the sweep, the way out."""
import time
from contextlib import suppress

from coaxial import Coaxial63100
from machine.errors import MachineError
from machine.rig.modes import HARDWARE, SIMULATED
from terminal.ui import bindings
from terminal.ui.screen import park, QUIET, say, steady
from terminal.views.session.state import ACTIVITIES, DEFAULT_PHASES


#: Seconds the teardown list stays on screen before the process exits.
TEARDOWN_HOLD = 2.0

#: What one arrow press moves it. Coarse enough to get somewhere by hand,
#: fine enough that the step is not the experiment. One TIM1 tick of ARR 2375
#: is 0.042 %, so this is 119 ticks and nowhere near the resolution floor.
DUTY_STEP = 0.05


def toggle_rail(session):
    """The rail, by name."""
    if 'switching' in session.running:
        session.note = 'switching owns the rail - stop it first'
        return
    afe = session.rig.board.afe
    got = steady(afe.state)
    if got is None:
        return
    steady(afe.off if got['on'] else afe.on)
    session.note = ''


def _duty(by):
    """The act that moves the duty `by`."""
    return bindings.did(lambda session: session.set_duty(session.duty + by))


def _running(activity):
    """An activity's bar word: its name, [ON] while it runs."""
    return lambda session: activity.name.upper() + (' [ON]' if activity.name in session.running else '')


#: The dash's keys, rows of the terminal's one table (`terminal.ui.bindings`) on its session: an
#: activity's letter it, the rail, the duty - the arrows or + and -.
PAD = bindings.page(tuple(('RUN', act.key.upper(), 'the %s started, or stopped' % act.name, ((act.key.upper(), _running(act)),),
              bindings.cased(act.key, bindings.did(lambda session, act=act: session.toggle(act))))
             for act in ACTIVITIES) + (
    ('RIG', 'A', "the front end's rail on, or off", (('A', 'AFE'),), bindings.cased('a', bindings.did(toggle_rail))),
    ('RIG', 'UP DOWN / + -', 'the duty up, down %.0f %%' % (100 * DUTY_STEP),
     (('+ -', lambda session: 'DUTY %.0f%%' % (100 * session.duty)),),
     {**dict.fromkeys(('up', '+', '='), _duty(DUTY_STEP)), **dict.fromkeys(('down', '-', '_'), _duty(-DUTY_STEP))}),
), column=False)


class Plan:

    """A sequence of leg sets and duties, each for so many seconds."""

    def __init__(self, text, duty=None):
        self.steps = []
        for chunk in (c.strip() for c in text.split(',') if c.strip()):
            legs, _, seconds = chunk.partition(':')
            legs, _, at = legs.partition('@')
            legs = tuple(c for c in legs.upper() if c in DEFAULT_PHASES)
            self.steps.append((legs, float(at) if at else duty,
                               float(seconds or 30)))
        self.at, self.started = 0, None

    def done(self):
        return self.at >= len(self.steps)

    def _begin(self, session, now):
        """The step at hand starts now: its duty when it names one, its
        legs."""
        legs, duty, _seconds = self.steps[self.at]
        self.started = now
        if duty is not None:
            session.duty = min(1.0, max(0.0, duty))
        session.set_legs(legs)

    def advance(self, session, now):
        """Apply the step that should be running, and say if it changed."""
        if self.done():
            return False
        if self.started is None:
            self._begin(session, now)
            return True
        _legs, _duty, seconds = self.steps[self.at]
        if now - self.started >= seconds:
            self.at, self.started = self.at + 1, None
            return self.advance(session, now)
        return False

    def caption(self, now):
        if self.done():
            return 'plan done'
        legs, duty, seconds = self.steps[self.at]
        left = seconds - (now - (self.started or now))
        return ('step %d/%d  %s at %.0f %%  %.0f s left'
                % (self.at + 1, len(self.steps), ''.join(legs) or 'none',
                   100.0 * (duty if duty is not None else 0.0),
                   max(0.0, left)))


def teardown(session, console, drawn, hold=True):
    """List what is being put back, under the last frame, and hold it there."""
    park(drawn, console)
    say('wait', 'stopping', 'putting back what the session started')

    try:
        undone = session.stop_all()
    except (MachineError, OSError) as exc:
        # The library's raise, or the port's: the lines matter more than the
        # exception - this is the only place that says what was put back.
        say('fail', 'stopping', str(exc)[:60])
        undone = []

    for name, what in undone:
        say('ok', name, what)
    if not undone:
        say('ok', 'nothing ran', 'no activity was started this session')

    if session.imu_started:
        with suppress(*QUIET):
            session.rig.board.imu.configure({0x05: 0})
            say('ok', 'rotation vector', 'disabled - the session asked '
                                         'for it')

    # Put back, not just claimed: the session may have raised the rail on the
    # way in, or the user toggled it with A.
    rail = steady(session.rig.board.afe.state)
    others = [u for u in (rail or {}).get('users', ()) if u != 'host']
    if (session.afe_found is not None and rail is not None
            and rail['on'] != session.afe_found and not others):
        steady(session.rig.board.afe.on if session.afe_found
               else session.rig.board.afe.off)
    say('ok', 'AFE_ON', 'back the way the session found it')
    say('ok', 'board', 'nothing the session started is still running')
    if console and hold:
        time.sleep(TEARDOWN_HOLD)


def sweep(rig):
    """What the board still has running, stopped, as (name, what) pairs."""
    done = []

    gates = steady(rig.gates.state)
    if gates is not None and gates['pwm_enabled']:
        steady(rig.write, analog=dict.fromkeys(
            ['Phase %s' % leg for leg in DEFAULT_PHASES], 0.0))
        steady(rig.gates.off)
        done.append(('gate stage', 'was armed - duty to zero, MOE clear'))

    task = steady(rig.board.daq.state)
    if task is not None and task.get('running'):
        steady(rig.stop)
        done.append(('acquisition', 'was running - task stopped'))

    rails = steady(rig.board.power.state)
    afe = (rails or {}).get('afe') or {}
    held = [u for u in afe.get('users', ()) if u != 'host']
    if held:
        done.append(('AFE_ON', 'held by %s - left alone, it is theirs'
                     % ', '.join(held)))
    return done


def leave(port, simulated):
    """Stop whatever the terminal left running, and say so - or say nothing."""
    try:
        with Coaxial63100(port=port, execution_mode=SIMULATED if simulated else HARDWARE,
                          power_afe=False) as rig:
            found = sweep(rig)
    except QUIET as exc:
        say('warn', 'leaving', 'the board did not answer: %s' % str(exc)[:60])
        return 0
    except (MachineError, OSError) as exc:
        # A port another process holds, a broker socket: shown, and the
        # session still ends the way it should.
        say('warn', 'leaving', '%s: %s' % (type(exc).__name__, str(exc)[:50]))
        return 0

    if not found:
        return 0
    say('wait', 'leaving', 'the terminal left something running')
    for name, what in found:
        say('warn', name, what)
    say('ok', 'board', 'put back')
    return 0


def start_activities(session, names):
    """Toggle the activities named on the command line, before any
    keystroke: a session run in the background for somebody else to watch
    has nobody to press one."""
    by_name = dict((act.name, act) for act in ACTIVITIES)
    for name in (n.strip() for n in names.split(',') if n.strip()):
        if name not in by_name:
            say('fail', 'start', '%s is not an activity' % name)
            continue
        session.toggle(by_name[name])
