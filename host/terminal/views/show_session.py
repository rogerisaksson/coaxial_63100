"""One dashboard over one rig. `coaxial_tty.ps1` runs this.

Six blocks in three columns - the analog channels, the thermal budget and
the half-bridges over DIO, the IMU and the angle sensor - under one line
saying what the system is doing. Nothing to page through: every standalone
view in `terminal/` opens its own rig and owns the port, so switching the gate
drivers in one view and watching the heat in another would take two processes
on one serial port (`show_thermal_observer.py --switch` does both).

One snapshot per frame: every block reads from the same round of calls, or
two columns disagree about whether the stage is switching.

The session shows what this board cannot do. AFE_ON is inverted: the gate
drivers have supply only while the analog front end does not, and AFE_ON
powers the IMU, the angle sensor, the ADC reference and every channel behind
it. The IMU is unpowered while the stage switches. The thermal observer keeps
answering: it runs on power and time between samples.

    python terminal/views/show_session.py
    python terminal/views/show_session.py --simulated
"""
import argparse
import sys
import time

from coaxial import Coaxial63100
from terminal.loader import TO_MENU
from terminal.ui import screen as _screen
from terminal.ui.screen import mode_of, run_view, say, steady
from terminal.ui.stage import boot, stage
from terminal.views.session.blocks import frame
from terminal.views.session.run import (Plan, act_on, leave, start_activities,
                                        teardown)
from terminal.views.session.state import (ACTIVITIES, DEFAULT_DUTY, Session,
                                          _start_imu)

_screen.CHATTER = False     # the boot bar replaced the scroll


#: How often a switching run stands down to let the thermal observer measure.
#:
#: The board cannot do this itself: AFE_ON high removes the gate drivers'
#: supply, so `Board_PowerPoll` refuses the rail while MOE is set - a sample
#: mid-switch would drop the drivers with six inputs moving. A sample needs
#: the switching stopped, and stopping is a policy of whoever asked for the
#: run, not of the board (invariant 10).
#:
#: 30 s and not 10: each sample costs an arm/disarm pair and the reference's
#: 500 ms settle, so 10 s spends 7 % of the run not switching against 2 %,
#: and 36 MOE edges in six minutes against 12. The board's time constant is
#: 6.8 minutes, so 30 s is 13 samples a tau - resolution nobody is short of.
#: The drivers node at 5.3 s cannot be tracked by sampling at any of these
#: rates; its anchor is the AFE die, which needs the same rail.
SAMPLE_EVERY_S = 30.0


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--hz', type=float, default=2.0)
    p.add_argument('--frames', type=int, default=0)
    p.add_argument('--sample-every', type=float, default=SAMPLE_EVERY_S,
                   metavar='S',
                   help='stand the stage down this often so the thermal observer can '
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
                   help='report and stop whatever the terminal left running')
    a = p.parse_args()

    if a.leave:
        return leave(a.port, a.simulated)

    # power_afe=False: the session raises the rail itself, conditionally,
    # below.
    with (boot('LINKING SESSION') as ready,
          Coaxial63100(port=a.port, execution_mode=mode_of(a),
                       power_afe=False) as rig):
        ready()
        say('ok' if rig.origin.real else 'warn', 'link', rig.origin.label)
        session_afe_found = None
        gates = steady(rig.gates.state)
        rail = steady(rig.board.afe.state)
        if rail is not None:
            session_afe_found = rail['on']
        if (rail is not None and not rail['on']
                and gates is not None and not gates['pwm_enabled']):
            # The resting state is the rail up (values on the dash from the
            # first frame); A toggles it.
            steady(rig.board.afe.on)
            say('ok', 'AFE_ON', 'up for the session - A toggles it, and '
                                'it goes back on the way out')
        else:
            say('ok', 'AFE_ON', 'left as found - the session owns it from '
                                'here')

        # With the rail up the IMU box can carry live values - the part only
        # produces once a report is asked for, so the session asks (and puts it
        # back on the way out).
        imu_started = rail is not None and _start_imu(rig)

        dashboard = stage()
        console = dashboard.is_terminal

        session = Session(rig)
        session.afe_found = session_afe_found
        session.imu_started = imu_started
        session.duty = min(1.0, max(0.0, a.duty))
        by_key = dict((act.key, act) for act in ACTIVITIES)
        start_activities(session, a.start)
        if a.sequence:
            session.plan = Plan(a.sequence, a.duty)
        sampled = [time.time()]

        def draw():
            return frame(session, dashboard, session.note)

        def tick():
            # The thermal observer is blind while the stage is armed, so a run
            # that never stands down is a run it estimates from end to end.
            if (a.sample_every > 0 and 'switching' in session.running
                    and time.time() - sampled[0] >= a.sample_every):
                session.sample()
                sampled[0] = time.time()
            if session.plan is None:
                return False
            if session.plan.advance(session, time.time()):
                session.note = session.plan.caption(time.time())
                return False
            return session.plan.done()      # the last step ran: the run ends

        def on_input(typed, _moved):
            for key in typed:
                act_on(session, key, by_key)

        leaving = None
        try:
            # The arrows are the duty's here, and the dashboard is a grid of
            # instruments with no column to scroll.
            leaving = run_view(dashboard, console, 1.0 / max(a.hz, 0.2),
                               a.frames, draw, on_input, tick,
                               scroll_keys=False)
        finally:
            print()
            teardown(session, console, 0, hold=leaving != 'menu')

    return TO_MENU if leaving == 'menu' else 0


if __name__ == '__main__':
    sys.exit(main())
