"""The demos, in one session. `demo.ps1` runs this.

There is nothing to pick and no mode to enter: opening the demos IS the
session. Every standalone view in `demos/` opens its own rig and owns the
port, so switching the gate drivers and watching the heat meant two processes
and one serial port - which is why `show_thermal.py` had to grow a `--switch`
of its own. Here one rig is held and the panels come and go over it.

THE ACTIVITY OUTLIVES THE PANEL. That is the whole point: start the drivers
switching, move to the thermal panel, and the stage keeps running because the
session owns it and not the view. Panels only draw.

WHAT THIS BOARD CANNOT DO, and the session says so rather than hiding it:
AFE_ON is inverted, so the gate drivers have supply only while the analog
front end does not - and AFE_ON is what powers the IMU, the angle sensor, the
ADC reference and every channel behind it. So there is no watching the IMU
react to switching. It is unpowered for the duration. The thermal observer is
the one thing that keeps answering, because it runs on power and time between
samples, and that is what it was built for.

    python tools/demos.py
    python tools/demos.py --simulated
    python tools/demos.py --panel gates
"""
import argparse
import os
import sys
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])

import show_gate_drivers                                   # noqa: E402
import show_thermal                                        # noqa: E402
from screen import (TO_MENU, Keys, banner, clear, paint,   # noqa: E402
                    say)

from coaxial import Coaxial63100                           # noqa: E402
from coaxial.errors import DeviceStateError, NoReplyError, RigError  # noqa: E402

#: Lines the frame spends on chrome: a blank, the banner, a blank, the key
#: bar, a blank. The first blank keeps the banner off the terminal's top row,
#: where the shell's own decoration sits over it.
CHROME = 5

QUIET = (NoReplyError, RigError, DeviceStateError)

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


def steady(fn, *args, **kwargs):
    """Run it, tolerating the link's occasional silence. None if it stayed."""
    for _ in range(4):
        try:
            return fn(*args, **kwargs)
        except QUIET:
            time.sleep(0.15)
    return None


class Session:

    """The rig, what is running on it, and what to put back."""

    def __init__(self, rig):
        self.rig = rig
        self.running = {}          # activity name -> what to undo, in order
        self.note = ''
        self.duty = DEFAULT_DUTY
        self.phases = DEFAULT_PHASES

    def set_duty(self, duty):
        """Move the duty, and push it if the stage is already switching.

        Held here rather than in the activity because it outlives it: set it
        down, stop switching, start again, and it is still what you chose.
        """
        self.duty = min(1.0, max(0.0, duty))
        if 'switching' in self.running:
            load = dict(('Phase %s' % leg, self.duty) for leg in self.phases)
            if steady(self.rig.write, analog=load) is None:
                self.note = 'the duty write did not take'
                return
        self.note = 'duty %.0f %%' % (100.0 * self.duty)

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
        for name in list(self.running):
            self.stop(name)


# ---- activities: what the board DOES ------------------------------------

class Switching:

    """Three legs at a duty, and the AFE off because the gate is inverted."""

    name = 'switching'
    key = 's'
    what = 'gate drivers, three legs'

    def start(self, session):
        rig = session.rig
        if steady(rig.board.afe.disable) is None:
            session.note = 'could not put the AFE down - not arming'
            return None
        if steady(rig.gates.arm, bypass_sto=True, ignore_interlock=True) is None:
            session.note = 'the stage refused to arm'
            return None

        load = dict(('Phase %s' % leg, session.duty) for leg in session.phases)
        if steady(rig.write, analog=load) is None:
            steady(rig.gates.disarm)
            session.note = 'armed but the duty did not take - disarmed again'
            return None

        return (lambda: rig.write(analog=dict.fromkeys(load, 0.0)),
                rig.gates.disarm)


class Acquiring:

    """A DAQ task, so the panels have traffic to show and the link has load."""

    name = 'acquiring'
    key = 'd'
    what = 'DAQ running, every channel'

    def start(self, session):
        rig = session.rig
        if steady(rig.configure, accumulate=8, digital=True) is None:
            session.note = 'the board refused the configuration'
            return None
        if steady(rig.start) is None:
            return None
        return (rig.stop,)


ACTIVITIES = (Switching(), Acquiring())


# ---- panels: what you LOOK at -------------------------------------------

def thermal_panel(session, console, reserve):
    """The observer. The one panel that still means something while switching."""
    rig = session.rig
    state = steady(rig.board.thermal.state)
    if state is None:
        return ['  the observer did not answer']

    out = ['  ' + show_thermal.summary(state)]
    spend = steady(rig.board.thermal.budget)
    if spend is not None:
        out.append(show_thermal.budget_line(spend))
    out.append('')
    out.extend(show_thermal.picture(state, console, reserve + len(out)))
    return out


def gates_panel(session, console, reserve):
    """TIM1, the six signals and the break."""
    del console, reserve
    state = steady(session.rig.gates.state)
    if state is None:
        return ['  the gate stage did not answer']
    return ['  ' + row for row in show_gate_drivers.gate_rows(state, 76)]


def sensors_panel(session, console, reserve):
    """The parts behind AFE_ON - and whether they have any supply at all."""
    del console, reserve
    rig = session.rig
    afe = steady(rig.board.afe.state)
    if afe is None:
        return ['  the AFE did not answer']

    out = ['  AFE_ON %-5s   held by %s'
           % (afe['on'], ', '.join(afe['users']) or 'nobody')]
    if not afe['on']:
        out.append('')
        out.append('  Everything below is UNPOWERED. The gate is inverted, so')
        out.append('  the drivers have supply only while these do not.')
        return out

    imu = steady(rig.board.imu.state)
    ang = steady(rig.board.angle.state)
    out.append('')
    if imu is not None:
        out.append('  imu    %-8s updates %-8d errors %-6d quaternion %s'
                   % (imu['loop'], imu['updates'], imu['errors'],
                      'yes' if imu['quaternion'] else 'no'))
    if ang is not None:
        out.append('  angle  %-8s updates %-8d errors %-6d reg 0x%02X'
                   % (ang['loop'], ang['updates'], ang['errors'], ang['reg']))
    return out


PANELS = (
    ('1', 'thermal', 'where the heat is, and the budget left', thermal_panel),
    ('2', 'gates', 'TIM1, the six signals, the break', gates_panel),
    ('3', 'sensors', 'the parts behind AFE_ON', sensors_panel),
)


def key_bar(session, active):
    """One line: the panels, the activities, and which are running."""
    panels = '  '.join(
        ('[%s]%s' % (key, name) if key == active else ' %s %s' % (key, name))
        for key, name, _what, _fn in PANELS)
    doing = '  '.join(
        '%s%s%s' % ('[' if a.name in session.running else ' ', a.key,
                    ']' if a.name in session.running else ' ') + a.name
        for a in ACTIVITIES)
    # ESC is named, like every standalone view names it. It was not, and the
    # only visible way out was Q - which closes the whole thing, correctly -
    # so the session read as the one view you cannot come back from.
    return ('  %s     %s     duty %3.0f %% (up/down)     q quit   ESC menu'
            % (panels, doing, 100.0 * session.duty))


def act_on(session, typed, active, by_key, panel_keys):
    """What one keystroke does. Returns the panel to draw next.

    Its own function because the draw loop was already as deep as this tree
    allows - the structure suite refuses a nest past seven, and a binding
    table growing inside a redraw is how a loop gets there.
    """
    if typed == 'up':
        session.set_duty(session.duty + DUTY_STEP)
    elif typed == 'down':
        session.set_duty(session.duty - DUTY_STEP)
    elif typed in panel_keys:
        session.note = ''
        return typed
    elif typed in by_key:
        session.toggle(by_key[typed])
    return active


def frame(session, active, console, note):
    reserve = CHROME + (1 if note else 0)
    draw = dict((key, fn) for key, _n, _w, fn in PANELS)[active]
    body = draw(session, console, reserve)
    lines = ['', banner(session.rig.origin, 'demos - %s' % active,
                        console, session.rig.origin.label), '']
    lines += body
    lines += ['', key_bar(session, active)]
    if note:
        lines.append('  %s' % note)
    return lines


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--hz', type=float, default=2.0)
    p.add_argument('--frames', type=int, default=0)
    p.add_argument('--panel', default='thermal',
                   help='which panel to open on, by name')
    a = p.parse_args()

    # power_afe=False: the session decides the rail, not the constructor.
    # Opening with the AFE forced up would stop any switching before the
    # first frame, which is the load this exists to watch.
    with Coaxial63100(port=a.port, simulated_device=a.simulated,
                      power_afe=False) as rig:
        say('ok' if rig.origin.real else 'warn', 'link', rig.origin.label)
        say('ok', 'AFE_ON', 'left as found - the session owns it from here')

        console = sys.stdout.isatty()
        if console:
            if os.name == 'nt':
                os.system('')
            sys.stdout.write(chr(27) + '[2J')

        session = Session(rig)
        by_name = dict((name, key) for key, name, _w, _f in PANELS)
        active = by_name.get(a.panel, '1')
        shown, leaving, count = [], None, 0
        by_key = dict((act.key, act) for act in ACTIVITIES)
        panel_keys = frozenset(key for key, _n, _w, _f in PANELS)

        try:
            with Keys(console) as keys:
                while True:
                    count += 1
                    lines = frame(session, active, console, session.note)
                    sys.stdout.write(paint(shown, lines, console))
                    sys.stdout.flush()
                    shown = lines

                    if a.frames and count >= a.frames:
                        break
                    leaving, _moved = keys.poll()
                    if leaving:
                        break

                    for typed in keys.taken():
                        active = act_on(session, typed, active,
                                        by_key, panel_keys)

                    time.sleep(1.0 / max(a.hz, 0.2))
        except KeyboardInterrupt:
            pass
        finally:
            clear(console)
            session.stop_all()
            say('ok', 'board', 'everything the session started is stopped')

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
