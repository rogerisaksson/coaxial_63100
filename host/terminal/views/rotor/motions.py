"""The demo's own drive: its cycle on the speed loop, the burst, the load."""
import math

from machine.parts import Slew, SpeedPI
from motor.pmsm import RAD_S_PER_RPM


#: The heavy start (B, and every BURST_EVERY_S on the stand-in): 43 A for
#: 1 s, bounded by heat, not the clamp. Measured on the stand-in 2026-09-06:
#: 0.57 of the span on a cold board at the 80 % floor, 0.82 warm and stable;
#: 1.4-1.8 s reached no higher (0.68, 0.71). 38 A into a 40 A clamp reached
#: 0.70, so the clamp went to 50.
BURST_A = 43.0

BURST_S = 1.0

BURST_ACCEL = 12000.0

#: After the heavy start, the burn: 3 s at 20 A against BURST_LOAD_NM at half the
#: no-load speed, where back-EMF times current reaches the kW bar.

BURST_HOLD_S = 3.0

BURST_HOLD_A = 20.0

#: Sized so the burn settles at half the no-load speed (torque x fade =
#: b w + load): 0.8 N.m sat at 778 of 3902 rpm, 0.42 sits near 1950.
BURST_LOAD_NM = 0.42

#: The load loop: 30 A peak, 40 s period (the legs' constant is seconds,
#: the board's minutes), rewritten when it moves 0.2 A.
LOAD_PEAK_A = 30.0

LOAD_PERIOD_S = 40.0

LOAD_GRAIN = 0.2


def no_load_rpm(view):
    """What the link will spin this motor to with nothing on the shaft."""
    params = view['params']
    lam = params.get('motor_lambda') or 0.0
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    vdc = (view['state'] or {}).get('vdc') or 0.0
    if lam <= 0.0 or vdc <= 0.0:
        return 0.0
    return vdc / (math.sqrt(3.0) * lam) / pairs * 60.0 / math.tau


def heavy_start(rig, view):
    """A second at the clamp, then the burn."""
    drive = rig.board.drive
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    left = view['burst_until'] - view['clock'].now()
    if left > BURST_HOLD_S:
        # Breaking away: everything the clamp allows, at the top of the speed
        # range, and no load in the way of it.
        drive.model.configure(load=0.0)
        drive.write(id_ref=0.0, iq_ref=BURST_A, accel=BURST_ACCEL,
                    omega_target=no_load_rpm(view) / 60.0 * math.tau * pairs)
        view['iq'] = BURST_A
        return
    # Burning: half the no-load speed, and a load to make the volts and the
    # amps happen at the same time.
    drive.model.configure(load=BURST_LOAD_NM)
    drive.write(id_ref=0.0, iq_ref=BURST_HOLD_A, accel=BURST_ACCEL,
                omega_target=no_load_rpm(view) / 120.0 * math.tau * pairs)
    view['iq'] = BURST_HOLD_A


def turn_the_handle(rig, view):
    """Whichever of the three is driving this frame, and only one of them."""
    if view['state']['mode'] == 'off':
        return
    now = view['clock'].now()
    if now < view['burst_until']:
        heavy_start(rig, view)
        view['bursting'] = True
        return
    if view['bursting']:
        view['bursting'] = False
        view['leaning'] = False
        rig.board.drive.model.configure(load=0.0)
        rig.board.drive.write(id_ref=0.0, iq_ref=view['iq'])
    if view['load']:
        load_loop(rig, view)
    if view['spin']:
        sweep(rig, view)


def load_loop(rig, view):
    """D current up and back down, continuously, the shape the speed loop
    has.
    """
    now = view['clock'].now()
    phase = ((now - view['load_at']) % LOAD_PERIOD_S) / LOAD_PERIOD_S
    ramp = 2.0 * phase if phase < 0.5 else 2.0 * (1.0 - phase)
    view['load_amps'] = LOAD_PEAK_A * ramp
    view['load_rising'] = phase < 0.5
    # Only when it has moved enough to matter: a setpoint is a round trip, and
    # one a frame against a board is the link's whole budget.
    if abs(view['load_amps'] - view['load_written']) >= LOAD_GRAIN:
        view['load_written'] = view['load_amps']
        rig.board.drive.write(id_ref=view['load_amps'])


#: The load stage's torque as the q amps it takes: the board's continuous rating, 19.1 A
#: against a 105 C laminate and 22.0 A against a 125 C junction (FINDINGS, 2026-09-05).
LOAD_A = 20.0


#: The demo, the bench's word (2026-09-28): up clockwise, let go, brake, the same the other way,
#: then a load held at speed. A stage: its name, seconds, the speed it ramps to, rpm (None: no
#: current, the rotor on its own drag) and the load on the stand-in's shaft, q amps. On the
#: demo's flywheel (J 8e-3, b 5e-4): up at 52 rad/s^2 on ~9 A, a coast of J/b = 16 s taking
#: 1 500 rpm to ~1 100 in 5 s, the brake back to rest in 2 s.
CYCLE = (
    ('align', 1.5, 0.0, 0.0),
    ('up', 6.0, 3300.0, 0.0),
    ('coast', 5.0, None, 0.0),
    ('brake', 3.0, 0.0, 0.0),
    ('up', 6.0, -3300.0, 0.0),
    ('coast', 5.0, None, 0.0),
    ('brake', 3.0, 0.0, 0.0),
    ('up', 3.0, 1000.0, 0.0),
    ('load', 6.0, 1000.0, LOAD_A),
    ('brake', 3.0, 0.0, 0.0),
)

#: A propeller on the stand-in's shaft through the cycle, torque k w|w|: the kilowatt at the
#: cycle's top. At the 24.8 V link vq tops out at vdc/sqrt 3 = 14.3 V, so a kilowatt is ~48 A
#: at the ceiling, 3 300 rpm: 0.8 kW on the shaft there, ~1 kW in.
PROP_KW = 0.8

#: A ramp takes this share of its stage, and the speed holds for the rest.
RAMP_SHARE = 0.67

#: The no-load speed where the record gives none to work it out from, rpm.
TOP_RPM = 3800.0

#: The first stage's pull onto the frame at 0, A: the estimate starts on its polarity.
HOLD_A = 12.0

#: The speed loop's bandwidth, Hz - a third of a feed pass's phase at 20 a second, where 3 Hz
#: was one - and its current through zero speed, A: a 10 A reversal held on the emulated
#: flywheel, the clamp through zero ran it to 1e5 rad/s. The drive's clamp past SEND_FROM
#: times the back-EMF's w_hi, where the estimate is the back-EMF's.
SPEED_HZ = 1.0
SPIN_A = 10.0
SEND_FROM = 1.5

#: An estimate past this share of the no-load speed is lost: the link cannot spin the motor
#: there.
LOST = 1.5


def speed(view):
    """The drive's own estimate, rad/s electrical: what it steers by. The chain beside it lost
    lock at the clamp's acceleration on the emulated flywheel and read 0 at 2 500."""
    return (view.get('state') or {}).get('omega_hat') or 0.0


def stage_at(view, now):
    """The stage `now` is in: its index, name, seconds into it, its length, the speed it ramps
    to (a share of the no-load speed, or None) and its load."""
    total = sum(stage[1] for stage in CYCLE)
    into = (now - view['spin_at']) % total
    for index, (name, seconds, rpm, load) in enumerate(CYCLE):
        if into < seconds:
            return index, name, into, seconds, rpm, load
        into -= seconds
    name, seconds, rpm, load = CYCLE[-1]
    return len(CYCLE) - 1, name, seconds, seconds, rpm, load


def prop_k():
    """The propeller's k, N m per (rad/s)^2: PROP_KW at the cycle's top."""
    top = max(abs(stage[2] or 0.0) for stage in CYCLE) * RAD_S_PER_RPM
    return PROP_KW * 1e3 / top ** 3


def _loop(view):
    """The speed loop the demo steps: the reference's ramp and the PI on the estimate, mechanical
    rad/s in, q amps out - designed on the rotor the demo turns (`view['j']`, `view['b']`), the
    propeller fed forward where the stand-in carries one."""
    p = view['params']
    kt = 1.5 * max(1.0, p.get('motor_pole_pairs') or 1.0) * (p.get('motor_lambda') or 0.005)
    return {'ramp': Slew(rate=0.0),
            'pi': SpeedPI(SPEED_HZ, SPIN_A, kt, view['j'], view['b'],
                          prop_k() if view['source'] == 'model' else 0.0)}


def sweep(rig, view):
    """The demo, a stage at a time: the reference ramped to the stage's speed and the loop's q
    current after it, no current while the rotor coasts, and the stage's load on the stand-in's
    shaft. The first stage holds the rotor onto the frame at 0; after it the drive is
    sensorless."""
    drive = rig.board.drive
    now = view['clock'].now()
    index, name, into, seconds, rpm, load = stage_at(view, now)
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    top = (no_load_rpm(view) or TOP_RPM) * RAD_S_PER_RPM
    w_hat = speed(view) / pairs
    loop = view.get('speed_loop') or view.setdefault('speed_loop', _loop(view))
    ramp, pi = loop['ramp'], loop['pi']
    p = view['params']
    kt = 1.5 * pairs * (p.get('motor_lambda') or 0.005)
    through = SEND_FROM * (p.get('drv_w_hi') or 0.0) / pairs
    pi.limit = SPIN_A if abs(w_hat) < through else max(SPIN_A, p.get('drv_i_max') or SPIN_A)
    if abs(w_hat) > LOST * top and name != 'align':
        # The demo's operator: the estimate lost, the cycle again from the pull onto the frame.
        view['said'] = 'estimate lost at %.0f rpm - aligned again' % (w_hat / RAD_S_PER_RPM)
        view['spin_at'], view['stage_index'] = now, None
        index, name, into, seconds, rpm, load = stage_at(view, now)
    if index != view.get('stage_index'):
        view['stage_index'], view['stage'] = index, name
        view['leaning'] = False
        view['stage_load'], view['load_full'] = 0.0, load * kt
        if name == 'align':
            drive.hold()
        elif view.get('stage_mode') != 'sensorless':
            drive.on('sensorless')
        view['stage_mode'] = 'hold' if name == 'align' else 'sensorless'
        # From where the rotor is: the ramp starts at its speed, the PI with no history.
        ramp.y = w_hat
        pi.reset()
        pi.was = w_hat
        if rpm is not None:
            ramp.configure(rate=abs(rpm * RAD_S_PER_RPM - w_hat)
                           / max(0.1, RAMP_SHARE * seconds))
        view['speed_at'] = now
    dt = min(0.25, max(0.0, now - view.get('speed_at', now)))
    view['speed_at'] = now
    if rpm:
        # The stage's load grows with the rotor's speed to its stage's, a dynamometer's: laid on
        # whole at the stage's start, a rotor the up left under the clamp's step - 10 A through
        # zero - lost to it and ran backwards, -635 rpm on a loaded host (2026-09-28).
        view['stage_load'] = view.get('load_full', 0.0) * min(
            1.0, abs(w_hat) / (abs(rpm) * RAD_S_PER_RPM))
    if view['source'] == 'model':
        # The propeller at the shaft's own speed, and the stage's load on top: the model's
        # load opposes positive turning, so the propeller's sign is the speed's.
        w = drive.model.read()['omega'] / pairs
        drive.model.configure(load=prop_k() * w * abs(w) + view.get('stage_load', 0.0))
    if name == 'align':
        drive.write(id_ref=HOLD_A, iq_ref=0.0, omega_target=0.0, theta=0.0)
        view['iq'] = 0.0
        return
    if rpm is None:
        # Let go: no torque, the bridge still switching - the rotor on its drag and the
        # propeller, the observer on the back-EMF. With the stage off the emulated board's
        # estimate fell from 3 308 to 238 rpm while the flywheel turned on, and stuck there
        # through the brake (2026-09-28). The ramp rides the estimate for the brake.
        ramp.y = pi.was = w_hat
        drive.write(id_ref=0.0, iq_ref=0.0)
        view['iq'] = 0.0
        return
    w_ref = ramp.step(dt, x=rpm * RAD_S_PER_RPM)['y'] if dt else ramp.y
    view['iq'] = pi.step(dt, setpoint=w_ref, measured=w_hat)['command'] if dt else view['iq']
    drive.write(id_ref=0.0, iq_ref=view['iq'])
