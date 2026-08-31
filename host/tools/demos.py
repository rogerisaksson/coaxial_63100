"""One dashboard over one rig. `coaxial_tty.ps1` runs this.

Six blocks in three columns - the analog channels, the thermal budget and
the half-bridges over DIO, the IMU and the angle sensor - under one line
saying what the system is doing. Nothing to page through: every standalone
view in `demos/` opens its own rig and owns the port, so switching the gate
drivers and watching the heat meant two processes and one serial port, which
is why `show_thermal.py` had to grow a `--switch` of its own.

ONE SNAPSHOT PER FRAME. Every block reads from the same round of calls, or
two columns disagree about whether the stage is switching - which is the one
thing a dashboard must not do.

WHAT THIS BOARD CANNOT DO, and the session says so rather than hiding it:
AFE_ON is inverted, so the gate drivers have supply only while the analog
front end does not - and AFE_ON is what powers the IMU, the angle sensor, the
ADC reference and every channel behind it. So there is no watching the IMU
react to switching. It is unpowered for the duration. The thermal observer is
the one thing that keeps answering, because it runs on power and time between
samples, and that is what it was built for.

    python tools/demos.py
    python tools/demos.py --simulated
"""
import argparse
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

from screen import (ASH, LABEL, gauge, Keys, park, QUIET,  # noqa: E402
                    say, SODIUM, steady, tint, TO_MENU)

import screen as _screen                                   # noqa: E402
_screen.CHATTER = False     # the boot bar replaced the scroll

from coaxial import Coaxial63100, angle, scaling, thermal  # noqa: E402

#: Samples per channel in the dash's analog read. 64 costs 88 ms of round
#: trip against a 500 ms frame; 16 costs 30 and the dash is a glance, not a
#: measurement - the meter bridge view is the one that averages.
ADC_SAMPLES = 16

#: How often a switching run stands down to let the observer measure.
#:
#: THE BOARD CANNOT DO THIS ITSELF. AFE_ON high removes the gate drivers'
#: supply, so `Board_PowerPoll` refuses the rail while MOE is set - a sample
#: mid-switch would drop the drivers with six inputs moving. The only way to
#: take one is to stop switching for it, and stopping is a policy, which
#: belongs to whoever asked for the run and not to the board (invariant 10).
#:
#: 30 s and not 10: each sample costs an arm/disarm pair and the reference's
#: 500 ms settle, so 10 s spends 7 % of the run not switching against 2 %,
#: and 36 MOE edges in six minutes against 12. The board's time constant is
#: 6.8 minutes, so 30 s is 13 samples a tau - resolution nobody is short of.
#: The drivers node at 5.3 s cannot be tracked by sampling at any of these
#: rates; its anchor is the AFE die, which needs the same rail.
SAMPLE_EVERY_S = 30.0

#: FIELD, from the A1335's register map.
ANGLE_REG_FIELD = 0x2A

#: Where a running session publishes its last snapshot, so a second
#: terminal can look without a second serial port. One file, replaced
#: atomically: a socket would need a port, a protocol and a cleanup path,
#: and none of that is needed to LOOK at a dashboard.
# The published-snapshot watcher lived here until the broker made it
# redundant: a second session now attaches through the broker and sees the
# board live, instead of reading a file of what another process last saw.
#: Older than this and the shared snapshot is not a reading of anything.
#: Two frames at the session's default rate, plus the slack a burst costs.
STALE_S = 3.0

#: Either side of a rail change, before anything is believed. The board's
#: own settle for the same reason, and the same number.
SETTLE_S = 0.5

#: Seconds the teardown list stays on screen before the process exits.
TEARDOWN_HOLD = 2.0

#: Cells in a budget bar. Ten reads as a percentage without being counted.
BAR = 10

#: Every node. Six-of-ten sorted by load was tried and dropped a leg the
#: moment two others warmed - driver W vanished from the very dashboard
#: that exists to show one leg heating alone.
THERMAL_ROWS = 10

#: Below this the A1335 has no magnet in front of it and the angle is noise.
#: The same number dial.py uses, and it is the host's judgement, not the
#: board's - the board reports the gauss and says nothing about it.
WEAK_GAUSS = 30


#: Where the gate drivers start. Half is the duty that puts equal volt-seconds
#: on both halves of every leg, so with all three at it there is no voltage
#: between phases and no phase current - the state the whole thermal
#: calibration was taken in.
DEFAULT_DUTY = 0.50

#: What one arrow press moves it. Coarse enough to get somewhere by hand,
#: fine enough that the step is not the experiment. One TIM1 tick of ARR 2375
#: is 0.042 %, so this is 119 ticks and nowhere near the resolution floor.
DUTY_STEP = 0.05

#: Which legs. Named because a leg at a time is how a per-leg thermal
#: question gets asked, and three at once is how a bulk one does.
DEFAULT_PHASES = ('U', 'V', 'W')


def _start_imu(rig):
    """Enable the rotation vector for the dashboard. True if it took."""
    if steady(rig.board.imu.settled, seconds=6.0) is not True:
        return False
    try:
        with rig.board.imu.configuring():
            rig.board.imu.feature(0x05, 20000)
        return True
    except QUIET:
        return False


class Session:

    """The rig, what is running on it, and what to put back."""

    def __init__(self, rig):
        self.rig = rig
        self.running = {}          # activity name -> what to undo, in order
        self.note = ''
        self.duty = DEFAULT_DUTY
        self.phases = DEFAULT_PHASES
        self.legs = DEFAULT_PHASES     # which of them are switching now
        self.plan = None
        # Both fetched once. The record does not change while a session
        # is open, and reading FIELD costs a hold - which stops the angle
        # loop, so doing it every frame would halve its rate to answer a
        # question whose answer is a magnet somebody has to fit.
        self.scaling = steady(rig.board.analog.scaling)
        self.field = self._field()
        #: The rail as the session found it - main() sets it after probing,
        #: teardown puts it back. None: never learned, leave it alone.
        self.afe_found = None
        #: Whether the session enabled the IMU's rotation vector, and so
        #: owes its disable on the way out.
        self.imu_started = False

    def _field(self):
        """The magnetic field at the A1335, in gauss. None if it refused."""
        if steady(self.rig.board.angle.hold) is None:
            return None
        try:
            got = steady(self.rig.board.angle.read, ANGLE_REG_FIELD)
        finally:
            steady(self.rig.board.angle.resume)
        return None if got is None else angle.gauss(got['value'])

    def set_duty(self, duty):
        """Move the duty, and push it if the stage is already switching.

        Held here rather than in the activity because it outlives it: set it
        down, stop switching, start again, and it is still what you chose.
        """
        self.duty = min(1.0, max(0.0, duty))
        if 'switching' in self.running and self.push() is None:
            return
        self.note = 'duty %.0f %%' % (100.0 * self.duty)

    def push(self):
        """Write the duty to the legs that are live, and zero to the rest.

        All three stay ARMED whatever the plan says. Arming is the thing that
        should be asked for by name, and re-arming per step would put a
        MOE edge into the middle of a thermal run for no reason - a leg at
        zero duty is not switching, which is what the plan means by off.
        """
        load = dict(('Phase %s' % leg,
                     self.duty if leg in self.legs else 0.0)
                    for leg in DEFAULT_PHASES)
        if steady(self.rig.write, analog=load) is None:
            self.note = 'the duty write did not take'
            return None
        return load

    def sample(self):
        """Stand down, let the observer measure, and go back to switching.

        Ordered so the gates are never live without their supply: MOE clear
        first, then the rail up, then the reading, then the rail down, then
        armed again. Every step through `steady`, and a failure leaves the
        stage DISARMED rather than half-way - the safe end of the sequence.
        """
        rig = self.rig
        if steady(rig.gates.disarm) is None:
            self.note = 'could not stand down for a sample'
            return False

        taken = None
        if steady(rig.board.afe.enable) is not None:
            time.sleep(SETTLE_S)
            taken = steady(rig.board.thermal.state)
            steady(rig.board.afe.disable)
            time.sleep(SETTLE_S)

        if steady(rig.gates.arm, bypass_sto=True,
                  ignore_interlock=True) is None:
            self.note = 'sampled, but the stage would not re-arm'
            return False
        self.push()

        if taken and taken.get('ntc') is not None:
            self.note = 'sampled: NTC %.2f C' % taken['ntc']
        return True

    def set_legs(self, legs):
        """Which legs switch. The rest are held at zero duty."""
        self.legs = tuple(legs)
        if 'switching' in self.running:
            self.push()

    def toggle(self, activity):
        if activity.name in self.running:
            self.stop(activity.name)
        else:
            undo = activity.start(self)
            if undo is not None:
                self.running[activity.name] = undo
                self.note = '%s started' % activity.name

    def stop(self, name):
        # One try per step: a failed first undo must not skip the rest, which
        # is how a disarm gets missed and a stage is left switching.
        for undo in self.running.pop(name, ()):
            try:
                undo()
            except QUIET as exc:
                self.note = '%s: %s' % (name, exc)
        self.note = '%s stopped' % name

    def stop_all(self):
        """Stop everything, and say what each one was. For the way out.

        Named rather than counted: 'everything is stopped' was one line for
        a stage that might have been switching 100 A, and the way out has to
        read like the way in - one line per thing, so what was put back is
        seen rather than trusted.
        """
        said = []
        for name in list(self.running):
            self.stop(name)
            said.append((name, UNDONE.get(name, 'stopped')))
        return said


# ---- activities: what the board DOES ------------------------------------

class Switching:

    """Three legs at a duty, and the AFE off because the gate is inverted."""

    name = 'switching'
    key = 's'
    what = 'gate drivers, three legs'
    undone = 'three legs to zero duty, stage disarmed'

    def start(self, session):
        rig = session.rig
        if steady(rig.board.afe.disable) is None:
            session.note = 'could not put the AFE down - not arming'
            return None
        if steady(rig.gates.arm, bypass_sto=True, ignore_interlock=True) is None:
            session.note = 'the stage refused to arm'
            return None

        load = session.push()
        if load is None:
            steady(rig.gates.disarm)
            session.note = 'armed but the duty did not take - disarmed again'
            return None

        return (lambda: rig.write(analog=dict.fromkeys(load, 0.0)),
                rig.gates.disarm)


class Acquiring:

    """A DAQ task, so the link has load and the counters move."""

    name = 'acquiring'
    key = 'd'
    what = 'DAQ running, every channel'
    undone = 'acquisition task stopped'

    def start(self, session):
        rig = session.rig
        if steady(rig.configure, accumulate=8, digital=True) is None:
            session.note = 'the board refused the configuration'
            return None
        if steady(rig.start) is None:
            return None
        return (rig.stop,)


ACTIVITIES = (Switching(), Acquiring())

#: What each one puts back, for the teardown lines. Off the classes, so
#: a new activity carries its own sentence rather than needing one here.
UNDONE = dict((a.name, a.undone) for a in ACTIVITIES)


# ---- the dash: six blocks, one snapshot ---------------------------------

def snapshot(session):
    """One round of reads, each guarded. ~270 ms of round trips at 2 Hz.

    Taken together rather than per block so every column shows the same
    instant: two blocks reading the gate stage a frame apart disagreed about
    whether it was switching, which is the one thing a dashboard must not do.
    """
    rig = session.rig
    board = rig.board
    got = {
        'afe': steady(board.afe.state),
        'gates': steady(rig.gates.state),
        'thermal': steady(board.thermal.state),
        'budget': steady(board.thermal.budget),
        'imu': steady(board.imu.state),
        'angle': steady(board.angle.state),
    }
    # Only while the rail is up. With it down read_all REFUSES (invariant
    # 9), and refusing through steady()'s four retries cost 0.6 s a frame
    # and drew the block as 'did not answer' - which flickered to values
    # whenever the observer borrowed the rail for its 30 s sample. The
    # block says the actual state instead.
    afe_on = bool(got['afe'] and got['afe']['on'])
    got['analog'] = (steady(board.analog.read_all,
                            nr_of_samples=ADC_SAMPLES) if afe_on else None)
    got['daq'] = steady(board.daq.state)
    got['ring'] = steady(board.capture.state)
    got['scaling'] = session.scaling
    got['field'] = session.field
    return got


def block(title, rows):
    """One dashboard instrument - the stage's hud, ANSI rows carried as-is.

    The name stays `block` so the six builders below read unchanged; what
    a block IS comes from the stage now, like every other view.
    """
    from screen import hud
    return hud(title, [row.strip() if isinstance(row, str) else row
                       for row in rows])


def adc_block(got):
    """Every analog channel, cooked by the board's own record."""
    table = got.get('analog')
    if table is None:
        afe = got.get('afe')
        if afe is not None and not afe['on']:
            # Not a fault: the session leaves the rail down to save power
            # and heat, and the observer borrows it on its own schedule.
            return block('ANALOG', [
                tint('  AFE_ON down - no reference', LABEL),
                tint('  observer borrows it for samples', LABEL)])
        return block('ANALOG', ['  did not answer'])

    params = got.get('scaling')
    rows = []
    for r in table['channels']:
        to = scaling.converter(r.get('unit'), r['differential'],
                               signal=r.get('signal'), params=params)
        rows.append('  %-8s %9.3f %s'
                    % (r['signal'][:8], to(r['mean_raw']),
                       scaling.symbol(r.get('unit'), r.get('signal'))))
    return block('ANALOG', rows)


def thermal_block(got):
    """Each node as a fraction of its own ceiling. A byte, drawn.

    The three THERMOMETERS are on the dash line instead: they are the widest
    thing there is and a column sized by them squeezed the two beside it.
    They come from the observer either way - the die sensors are linear parts
    calibrated at the factory, and that curve is in the MCU's system memory,
    so the board is the only side that can convert their codes at all.
    """
    spend = got.get('budget')
    if spend is None:
        return block('THERMAL', ['  the observer did not answer'])

    # ALL TEN, in the firmware's own order, so a leg keeps its row whether
    # it is heating or not - sorted-and-cut dropped driver W the moment two
    # other nodes warmed, on the dashboard that exists to show one leg
    # heating alone. The gauge turns sodium at 0.85, where the board acts.
    used = spend['used']
    rows = []
    for name in (n for n in thermal.ALL_NODES if n in used):
        rows.append('  %-10s %s %3.0f %%'
                    % (thermal.pretty(name), gauge(used[name], BAR),
                       100.0 * used[name]))
    return block('THERMAL', rows)


def bridges_block(got):
    """The six gate signals, by leg, and what the stage is doing."""
    state = got.get('gates')
    if state is None:
        return block('HALF-BRIDGES', ['  the stage did not answer'])

    pins = state.get('pins') or {}
    # `requested_ticks` is a CCR count, so the duty is it over the period.
    # ARR is `period - 1`; dividing by `period` is off by one tick and by
    # nothing anybody could see, but the arithmetic should say what it means.
    span = float(max(1, state.get('period', 1) - 1))
    rows = []
    for leg, ticks in zip(('U', 'V', 'W'),
                          state.get('requested_ticks', (0, 0, 0))):
        rows.append('  %s   H %s  L %s   %3.0f %%'
                    % (leg,
                       'ON ' if pins.get(leg + 'H') else 'off',
                       'ON ' if pins.get(leg + 'L') else 'off',
                       100.0 * ticks / span))
    rows.append('  MOE %-4s  break %s'
                % ('set' if state['pwm_enabled'] else 'clear',
                   'LATCHED' if state['fault'] else 'clear'))
    rows.append('  dead time %d ns' % state['deadtime_ns'])
    return block('HALF-BRIDGES', rows)


def dio_block(got):
    """The three pins the board drives, and the break it only reads."""
    afe = got.get('afe')
    state = got.get('gates')
    if afe is None:
        return block('DIO', ['  did not answer'])

    rows = ['  AFE_ON  %-4s %s'
            % ('high' if afe['on'] else 'low',
               ', '.join(afe['users']) or 'nobody holds it')]
    if state is not None:
        rows.append('  PE15    %-4s break, read only'
                    % ('high' if not afe['pe15'] else 'low'))
        rows.append('  KEEPALIVE  %d edges' % state['keepalive'])
    return block('DIO', rows)


def imu_block(got):
    """The BNO085: is it reporting, and how fast."""
    state = got.get('imu')
    if state is None:
        return block('IMU', ['  did not answer'])

    q = state.get('quaternion')
    asked = state.get('feature') or {}
    rows = ['  loop %-8s  %d reports' % (state['loop'], state['updates']),
            '  report %s' % _report(asked),
            '  errors %-6d %s' % (state['errors'],
                                  state.get('last_fault', ''))]
    if q:
        rows.append('  q %+.3f %+.3f %+.3f %+.3f'
                    % (q['i'], q['j'], q['k'], q['real']))
    else:
        rows.append('  no quaternion yet')
    return block('IMU', rows)


def angle_block(got):
    """The A1335, and whether there is a magnet to measure."""
    state = got.get('angle')
    if state is None:
        return block('ANGLE', ['  did not answer'])

    rows = ['  loop %-8s  %d readings' % (state['loop'], state['updates']),
            '  errors %-6d %s' % (state['errors'], state['error'])]
    field = got.get('field')
    if field is not None and field < WEAK_GAUSS:
        rows.append('  %d gauss - no magnet, so' % field)
        rows.append('  the angle is noise')
    elif state.get('degrees') is None:
        # The loop reads a register the host asked it to, and only ANG
        # decodes to an angle. Whatever else it is watching, it is not this.
        rows.append('  reading %s, not ANG' % state.get('register_name', '?'))
    else:
        rows.append('  %7.2f deg' % state['degrees'])
    return block('ANGLE', rows)


def _interval(us):
    if not us:
        return 'nothing asked for'
    return '%d ms' % (us // 1000) if us >= 1000 else '%d us' % us


def _report(asked):
    """What the part was asked to send, or that nothing was."""
    if not asked.get('interval_us'):
        return 'none asked for'
    return '%d @ %s' % (asked.get('report_id', 0),
                        _interval(asked['interval_us']))


def _degc(value):
    """A thermometer that did not answer says so rather than reading 0."""
    return '--' if value is None else '%.1f' % value


def acquisition_block(got):
    """The DAQ task and the capture ring - the capture view, as one box.

    It was a whole menu entry beside the session showing the same board;
    what was unique there - the task's rate and drops, the ring and its
    sources - is this box."""
    daq, ring = got.get('daq'), got.get('ring')
    if daq is None and ring is None:
        return block('ACQUISITION', ['  did not answer'])

    rows = []
    if daq is not None:
        rows.append('task %s  acc %d' % (daq.get('clock', '?'),
                                         daq.get('accumulate', 0)))
        rows.append('buffered %d  dropped %d' % (daq.get('available', 0),
                                                 daq.get('dropped', 0)))
    if ring is not None:
        rows.append('ring %s' % (', '.join(ring.get('sources', ()))
                                 or 'not armed'))
        rows.append('held %d of %d  thin %d'
                    % (ring.get('count', 0), ring.get('depth', 0),
                       ring.get('thinned', 0)))
    return block('ACQUISITION', rows)


def dash(session, got):
    """One line over the six blocks: what the system is doing right now."""
    afe, gates = got.get('afe'), got.get('gates')
    spend, therm = got.get('budget'), got.get('thermal')

    bits = []
    if afe is not None:
        # Sodium marks power flowing; off is just the street.
        bits.append('AFE %s' % (tint('on', SODIUM) if afe['on']
                                else tint('off', ASH)))
    if gates is not None:
        bits.append('gates %s' % (tint('SWITCHING', SODIUM)
                                  if gates['pwm_enabled']
                                  else tint('idle', ASH)))
    if spend is not None:
        bits.append('worst %s %.0f %%'
                    % (thermal.pretty(spend['worst_node']),
                       100.0 * spend['worst']))
    if therm is not None:
        # All three thermometers here rather than in the thermal column: they
        # are the widest line there is, and a column sized by them squeezed
        # the two beside it. This line has the width.
        bits.append('NTC %s  A1335 die %s  MCU die %s C'
                    % (_degc(therm['ntc']), _degc(therm['afe']),
                       _degc(therm['mcu'])))
    if session.running:
        bits.append('running: %s' % ', '.join(sorted(session.running)))
    if session.plan is not None:
        bits.append(session.plan.caption(time.time()))
    return '  ' + '   '.join(bits)


def act_on(session, typed, by_key):
    """What one keystroke does.

    Its own function because the draw loop was already as deep as this tree
    allows - the structure suite refuses a nest past seven, and a binding
    table growing inside a redraw is how a loop gets there.
    """
    if typed == 'up':
        session.set_duty(session.duty + DUTY_STEP)
    elif typed == 'down':
        session.set_duty(session.duty - DUTY_STEP)
    elif typed in ('a', 'A'):
        # The rail, by name. Not while switching: the gate is inverted, so
        # raising it mid-run would drop the drivers with six inputs moving -
        # the switching activity owns the rail for as long as it runs.
        afe = session.rig.board.afe
        if 'switching' in session.running:
            session.note = 'switching owns the rail - stop it first'
        else:
            got = steady(afe.state)
            if got is not None:
                steady(afe.disable if got['on'] else afe.enable)
                session.note = ''
    elif typed in by_key:
        session.toggle(by_key[typed])


def frame(session, console, note):
    """The dashboard on the stage: dash strip, six instruments, key bar."""
    from rich.text import Text

    from screen import panels_of

    got = snapshot(session)
    rows = [[Text.from_ansi(dash(session, got))],
            [adc_block(got), thermal_block(got), bridges_block(got)],
            [dio_block(got), imu_block(got), angle_block(got),
             acquisition_block(got)]]
    keys = [(a.key.upper(), a.name.upper()
             + (' [ON]' if a.name in session.running else ''))
            for a in ACTIVITIES]
    keys += [('A', 'AFE'), ('+ -', 'DUTY %.0f%%' % (100 * session.duty)),
             ('Q', 'EXIT'), ('ESC', 'MENU')]
    if note:
        keys.append(('', note))
    return panels_of(console, session.rig.origin, 'SESSION', rows,
                     keys)


class Plan:

    """A sequence of leg sets and duties, each for so many seconds.

    `U:45,V:45,UVW@0.75:90` - which legs switch, at what duty, for how long.
    The duty is optional and carries over from the session's own.

    It exists to give the OBSERVER a known excitation that changes. Two axes,
    and they say different things: LEGS should scale the loss by how many are
    switching, and DUTY should barely move it at all - a transition happens
    twice a period whatever the duty is, so a model whose loss tracks duty is
    modelling conduction that is not there.

    Wall clock, not frames: the draw rate is what the link allows on the day,
    and a plan measured in frames would be a different plan each run.
    """

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

    def advance(self, session, now):
        """Apply the step that should be running, and say if it changed."""
        if self.done():
            return False
        legs, duty, seconds = self.steps[self.at]
        if self.started is None:
            self.started = now
            if duty is not None:
                session.duty = min(1.0, max(0.0, duty))
            session.set_legs(legs)
            return True
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
    """List what is being put back, under the last frame, and hold it there.

    NOT after `clear`. Clearing first put the list alone on a blank screen,
    where anything drawn afterwards - a shell prompt, a terminal profile
    closing the pane - took it with it, and it was reported missing four
    times. Left under the dashboard it cannot be lost: nothing writes after.

    The hold is for the same reason. Two seconds is long enough to read six
    lines and short enough not to be in the way - and nothing on the way to
    the menu, which repaints over the list.
    """
    park(drawn, console)
    say('wait', 'stopping', 'putting back what the session started')

    try:
        undone = session.stop_all()
    except Exception as exc:                       # noqa: BLE001
        # The lines matter more than the exception: this is the only place
        # that says what was put back, and a stop that raised is exactly
        # when somebody needs to read it.
        say('fail', 'stopping', str(exc)[:60])
        undone = []

    for name, what in undone:
        say('ok', name, what)
    if not undone:
        say('ok', 'nothing ran', 'no activity was started this session')

    if session.imu_started:
        try:
            with session.rig.board.imu.configuring():
                session.rig.board.imu.feature(0x05, 0)
            say('ok', 'rotation vector', 'disabled - the session asked '
                                         'for it')
        except QUIET:
            pass

    # PUT BACK, not just claimed: the session may have raised the rail on
    # the way in, or the user toggled it with A. Held by somebody else is
    # theirs to keep - the observer mid-sample, another session measuring.
    rail = steady(session.rig.board.afe.state)
    if session.afe_found is not None and rail is not None:
        held = [u for u in rail.get('users', ()) if u != 'host']
        if rail['on'] != session.afe_found and not held:
            steady(session.rig.board.afe.enable if session.afe_found
                   else session.rig.board.afe.disable)
    say('ok', 'AFE_ON', 'back the way the session found it')
    say('ok', 'board', 'nothing the session started is still running')
    if console and hold:
        time.sleep(TEARDOWN_HOLD)


def sweep(rig):
    """What the board still has running, stopped, as (name, what) pairs.

    For LEAVING THE MENU, which holds no rig of its own: a view that was
    killed, or a session that ended badly, can leave a stage armed or a task
    filling a ring, and the way out of the demos has to say so rather than
    let the shell prompt be the last word.

    Reads before it acts, so it can name what it found. Empty means nothing
    was running, and that is worth one line too.
    """
    done = []

    gates = steady(rig.gates.state)
    if gates is not None and gates['pwm_enabled']:
        steady(rig.write, analog=dict.fromkeys(
            ['Phase %s' % leg for leg in DEFAULT_PHASES], 0.0))
        steady(rig.gates.disarm)
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
    """Stop whatever the demos left running, and say so - or say nothing.

    SILENT WHEN THERE IS NOTHING. A clean exit should be an exit, not four
    lines confirming that four things nobody started are not running. The
    lines are for the case that needs them: a view that was killed, or a
    session that ended badly, leaving a stage armed with nothing on screen
    to say it.

    Costs one port open either way. That is the price of the difference
    between knowing and assuming.

    IT CANNOT RAISE. This is the way out, and a board that will not answer
    on the way out is something to report rather than a traceback over the
    prompt - measured, a reset board turned quitting the menu into one.
    Exit 0 either way: leaving is not a thing that fails.
    """
    try:
        with Coaxial63100(port=port, simulated_device=simulated,
                          power_afe=False) as rig:
            found = sweep(rig)
    except QUIET as exc:
        say('warn', 'leaving', 'the board did not answer: %s' % str(exc)[:60])
        return 0
    except Exception as exc:                          # noqa: BLE001
        say('warn', 'leaving', '%s: %s' % (type(exc).__name__, str(exc)[:50]))
        return 0

    if not found:
        return 0
    say('wait', 'leaving', 'the demos left something running')
    for name, what in found:
        say('warn', name, what)
    say('ok', 'board', 'put back')
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--hz', type=float, default=2.0)
    p.add_argument('--frames', type=int, default=0)
    p.add_argument('--sample-every', type=float, default=SAMPLE_EVERY_S,
                   metavar='S',
                   help='stand the stage down this often so the observer can '
                        'measure; 0 leaves it running blind')
    p.add_argument('--sequence', default='', metavar='U:45,VW:60,...',
                   help='legs to switch and for how long, in order; ends the '
                        'run when the last step is done')
    p.add_argument('--start', default='', metavar='NAME',
                   help='begin with these activities running, comma separated: '
                        + ', '.join(a.name for a in ACTIVITIES))
    p.add_argument('--duty', type=float, default=DEFAULT_DUTY,
                   help='what the gate stage starts at, 0..1')
    p.add_argument('--leave', action='store_true',
                   help='report and stop whatever the demos left running')
    a = p.parse_args()

    if a.leave:
        return leave(a.port, a.simulated)

    # power_afe stays False here - the session raises the rail ITSELF,
    # conditionally, a few lines down. The constructor cannot know whether
    # another session is mid-run: forcing the rail up on open would drop an
    # armed stage's drivers before the first frame, which is the one load
    # this dashboard exists to watch.
    from screen import boot
    with boot('LINKING SESSION') as ready,          Coaxial63100(port=a.port, simulated_device=a.simulated,
                      power_afe=False) as rig:
        ready()
        say('ok' if rig.origin.real else 'warn', 'link', rig.origin.label)
        session_afe_found = None
        stage = steady(rig.gates.state)
        rail = steady(rig.board.afe.state)
        if rail is not None:
            session_afe_found = rail['on']
        if (rail is not None and not rail['on']
                and stage is not None and not stage['pwm_enabled']):
            # The resting state is the rail UP - values on the dash from
            # the first frame - and A toggles it. Never over a run: a
            # stage armed by another session keeps the rail where the run
            # needs it.
            steady(rig.board.afe.enable)
            say('ok', 'AFE_ON', 'up for the session - A toggles it, and '
                                'it goes back on the way out')
        else:
            say('ok', 'AFE_ON', 'left as found - the session owns it from '
                                'here')

        # With the rail up the IMU box can carry live values - the part
        # only produces once a report is asked for, so the session asks
        # (and puts it back on the way out). Best effort: a part mid-boot
        # leaves the box at 'none asked for', not the session dead.
        imu_started = rail is not None and _start_imu(rig)

        from screen import curtain, stage

        dashboard = stage()
        console = dashboard.is_terminal

        session = Session(rig)
        session.afe_found = session_afe_found
        session.imu_started = imu_started
        session.duty = min(1.0, max(0.0, a.duty))
        leaving, count = None, 0
        sampled = time.time()
        by_key = dict((act.key, act) for act in ACTIVITIES)

        # Started here rather than by a keystroke, because a session run in
        # the background for somebody else to watch has nobody to press one.
        by_name = dict((act.name, act) for act in ACTIVITIES)
        for name in (n.strip() for n in a.start.split(',') if n.strip()):
            if name in by_name:
                session.toggle(by_name[name])
            else:
                say('fail', 'start', '%s is not an activity' % name)

        if a.sequence:
            session.plan = Plan(a.sequence, a.duty)

        try:
            with curtain(dashboard) as live, Keys(console) as keys:
                while True:
                    count += 1
                    live.update(frame(session, console, session.note),
                                refresh=True)

                    # The observer is blind while the stage is armed, so
                    # a run that never stands down is a run it estimates
                    # from end to end. Not on the frame count: the draw rate
                    # is whatever the link allows on the day.
                    if (a.sample_every > 0 and 'switching' in session.running
                            and time.time() - sampled >= a.sample_every):
                        session.sample()
                        sampled = time.time()

                    if session.plan is not None:
                        if session.plan.advance(session, time.time()):
                            session.note = session.plan.caption(time.time())
                        elif session.plan.done():
                            leaving = 'quit'
                            break

                    if a.frames and count >= a.frames:
                        break
                    leaving, _moved = keys.poll()
                    if leaving:
                        break

                    for typed in keys.taken():
                        act_on(session, typed, by_key)

                    time.sleep(1.0 / max(a.hz, 0.2))
        except KeyboardInterrupt:
            pass
        finally:
            print()
            teardown(session, console, 0, hold=leaving != 'menu')

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
