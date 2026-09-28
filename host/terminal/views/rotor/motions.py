"""The demo's own drive: its cycle on the speed loop, the burst, the load."""
import math

from machine.parts import SpeedPI
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

#: An emulated board's world takes the stage's torque when it moves this far, N m: a monitor's
#: round trip a frame on Renode.
WORLD_GRAIN_NM = 0.01


def no_load_rpm(view):
    """What the link will spin this motor to with nothing on the shaft."""
    params = view['params']
    lam = params.get('motor_lambda') or 0.0
    pairs = max(1.0, params.get('motor_pole_pairs') or 1.0)
    vdc = (view['state'] or {}).get('vdc') or 0.0
    if lam <= 0.0 or vdc <= 0.0:
        return 0.0
    return vdc / (math.sqrt(3.0) * lam) / pairs * 60.0 / math.tau


def lay(rig, view, torque):
    """`torque` against the shaft, N m: the stand-in's model's load, or on an emulated board's
    world the stage's beside the propeller its drag (`world_drag`)."""
    if view['source'] == 'model':
        rig.board.drive.model.configure(load=torque)
    else:
        world_load(view, torque)


def world_load(view, torque):
    """The demo's loads on an emulated board's world, as the stand-in's model takes them off the
    page: the propeller its drag on the world's own shaft, the stage's torque laid when it moves
    a grain. Without them native's demo drew the flywheel's current alone."""
    laid = view.get('world_drag')
    if laid is None or abs(torque - view.get('world_torque', math.inf)) < WORLD_GRAIN_NM:
        return
    laid(prop_k(), torque)
    view['world_torque'] = torque


def heavy_start(rig, view):
    """A second at the clamp, then the burn."""
    drive = rig.board.drive
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    left = view['burst_until'] - view['clock'].now()
    if left > BURST_HOLD_S:
        # Breaking away: everything the clamp allows, at the top of the speed
        # range, and no load in the way of it.
        lay(rig, view, 0.0)
        drive.write(id_ref=0.0, iq_ref=BURST_A, accel=BURST_ACCEL,
                    omega_target=no_load_rpm(view) / 60.0 * math.tau * pairs)
        view['iq'] = BURST_A
        return
    # Burning: half the no-load speed, and a load to make the volts and the
    # amps happen at the same time.
    lay(rig, view, BURST_LOAD_NM)
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
        lay(rig, view, 0.0)
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

#: A joint's load held on the vector, q amps: gravity at the end of a limb, the rotor a few
#: degrees off its angle under HOLD_A.
JOINT_LOAD_A = 6.0


#: The demo in segments, the bench's words (2026-09-28): an application each, its stages
#: driven by the speed loop, a held vector or one stepped. A stage: its segment, its name,
#: seconds, the speed it ramps to - rpm, None: no current, the rotor on its drag - the load on
#: the stand-in's shaft in q amps, and how. On the demo's flywheel (J 8e-3, b 5e-4): up at 52
#: rad/s^2 on ~9 A, a coast of J/b = 16 s taking 1 500 rpm to ~1 100 in 5 s, the brake back to
#: rest in 2 s; a climb against the propeller 40 A at its top, the joint's vector 12 A all the
#: while.
CYCLE = (
    ('SPIN', 'align', 1.5, 0.0, 0.0, 'hold'),
    ('SPIN', 'up', 6.0, 3300.0, 0.0, 'speed'),
    ('SPIN', 'coast', 5.0, None, 0.0, 'speed'),
    ('SPIN', 'brake', 3.0, 0.0, 0.0, 'speed'),
    ('SPIN', 'up', 6.0, -3300.0, 0.0, 'speed'),
    ('SPIN', 'coast', 5.0, None, 0.0, 'speed'),
    ('SPIN', 'brake', 3.0, 0.0, 0.0, 'speed'),
    ('SPIN', 'up', 3.0, 1000.0, 0.0, 'speed'),
    ('SPIN', 'load', 6.0, 1000.0, LOAD_A, 'speed'),
    ('SPIN', 'brake', 3.0, 0.0, 0.0, 'speed'),
    ('SERVO', 'move', 1.5, 900.0, 0.0, 'speed'),
    ('SERVO', 'stop', 1.8, 0.0, 0.0, 'speed'),
    ('SERVO', 'move', 1.5, -900.0, 0.0, 'speed'),
    ('SERVO', 'stop', 1.8, 0.0, 0.0, 'speed'),
    ('SERVO', 'move', 1.5, 900.0, 0.0, 'speed'),
    ('SERVO', 'stop', 1.8, 0.0, 0.0, 'speed'),
    ('STEPPER', 'hold', 1.0, 0.0, 0.0, 'hold'),
    ('STEPPER', 'steps', 5.0, 2.0, 0.0, 'step'),
    ('STEPPER', 'back', 5.0, -2.0, 0.0, 'step'),
    ('FIXED WING', 'climb', 10.0, 2800.0, 0.0, 'speed'),
    ('FIXED WING', 'cruise', 3.0, 2800.0, 0.0, 'speed'),
    ('FIXED WING', 'blip', 0.5, 3300.0, 0.0, 'speed'),
    ('FIXED WING', 'cruise', 2.0, 2800.0, 0.0, 'speed'),
    ('FIXED WING', 'blip', 0.5, 3300.0, 0.0, 'speed'),
    ('FIXED WING', 'glide', 4.0, None, 0.0, 'speed'),
    ('FIXED WING', 'land', 3.0, 0.0, 0.0, 'speed'),
    ('QUAD', 'spool', 3.0, 2000.0, 0.0, 'speed'),
    ('QUAD', 'stab', 0.4, 2600.0, 0.0, 'speed'),
    ('QUAD', 'hover', 0.8, 2000.0, 0.0, 'speed'),
    ('QUAD', 'stab', 0.4, 1400.0, 0.0, 'speed'),
    ('QUAD', 'hover', 0.8, 2000.0, 0.0, 'speed'),
    ('QUAD', 'stab', 0.4, 2600.0, 0.0, 'speed'),
    ('QUAD', 'hover', 0.8, 2000.0, 0.0, 'speed'),
    ('QUAD', 'stab', 0.4, 1400.0, 0.0, 'speed'),
    ('QUAD', 'hover', 1.0, 2000.0, 0.0, 'speed'),
    ('QUAD', 'land', 3.0, 0.0, 0.0, 'speed'),
    ('JOINT', 'hold', 8.0, 0.0, JOINT_LOAD_A, 'hold'),
)

#: Where the demo starts: a segment's name, or None for the cycle's first (`--segment`).
START_AT = None

#: A stepper's step, electrical degrees, the share of its interval the vector eases over and the
#: current it is held on, A. 15 mechanical degrees - 105 electrical, past the 90 where the held
#: vector's torque turns - lost the rotor: 180 asked, 580 turned, and the way back went on
#: forward; the stand-in's rotor damps at 0.002 of critical, so a step set down whole rings on
#: (2026-09-28). A servo's stop is 1.8 s: 858 rpm through the 10 A clamp at zero takes 1.4,
#: and at 1.0 s the first stop ended at 167 rpm.
STEP_E_DEG = 45.0
STEP_EASE = 0.3
STEP_A = 8.0

#: A propeller on the stand-in's shaft through the cycle, torque k w|w|: the kilowatt at the
#: cycle's top. At the 24.8 V link vq tops out at vdc/sqrt 3 = 14.3 V, so a kilowatt is ~48 A
#: at the ceiling, 3 300 rpm: 0.8 kW on the shaft there, ~1 kW in.
PROP_KW = 0.8

#: A ramp takes this share of its stage, and the speed holds for the rest.
RAMP_SHARE = 0.67

#: How a ramp spools, shares of it: its acceleration up from none over SPOOL_RISE - slow, then
#: faster and faster - held, and eased off over SPOOL_LAND onto its speed. At a constant rate
#: the current stepped on at the first frame (488 rpm/s at 0.25 s), the lag held under the 10 A
#: clamp through zero caught up at 1 379 rpm/s where it opened, and the top stopped dead: on
#: and off (bench, 2026-09-28).
SPOOL_RISE, SPOOL_LAND = 0.5, 0.15

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


def start_of(segment):
    """Seconds into the cycle a segment begins, 0 for None."""
    at = 0.0
    for stage in CYCLE:
        if stage[0] == segment:
            return at
        at += stage[2]
    return 0.0


def stage_at(view, now):
    """The stage `now` is in: its index, segment, name, seconds into it, its length, the speed it
    ramps to (rpm, or None), its load and how it is driven."""
    total = sum(stage[2] for stage in CYCLE)
    into = (now - view['spin_at'] + start_of(START_AT)) % total
    for index, (segment, name, seconds, rpm, load, how) in enumerate(CYCLE):
        if into < seconds:
            return index, segment, name, into, seconds, rpm, load, how
        into -= seconds
    segment, name, seconds, rpm, load, how = CYCLE[-1]
    return len(CYCLE) - 1, segment, name, seconds, seconds, rpm, load, how


def prop_k():
    """The propeller's k, N m per (rad/s)^2: PROP_KW at the cycle's top."""
    top = max(abs(stage[3] or 0.0) for stage in CYCLE) * RAD_S_PER_RPM
    return PROP_KW * 1e3 / top ** 3


def spool(x):
    """The share of a ramp's speed change made `x` of the way through it: its acceleration up
    from none over SPOOL_RISE, held, and eased off over SPOOL_LAND."""
    x, rise, land = min(1.0, max(0.0, x)), SPOOL_RISE, SPOOL_LAND
    peak = 1.0 / (1.0 - 0.5 * (rise + land))
    if x < rise:
        return peak * x * x / (2.0 * rise)
    if x < 1.0 - land:
        return peak * (0.5 * rise + x - rise)
    return 1.0 - peak * (1.0 - x) ** 2 / (2.0 * land)


def _loop(view):
    """The speed loop the demo steps: the PI on the estimate after the spooled reference,
    mechanical rad/s in, q amps out - designed on the rotor the demo turns (`view['j']`,
    `view['b']`), the propeller fed forward where the stand-in carries one."""
    p = view['params']
    kt = 1.5 * max(1.0, p.get('motor_pole_pairs') or 1.0) * (p.get('motor_lambda') or 0.005)
    return {'pi': SpeedPI(SPEED_HZ, SPIN_A, kt, view['j'], view['b'],
                          prop_k() if view['source'] == 'model' or view.get('world_drag')
                          else 0.0)}


def sweep(rig, view):
    """The demo, a stage at a time: the reference ramped to the stage's speed and the loop's q
    current after it, no current while the rotor coasts, and the stage's load on the stand-in's
    shaft. The first stage holds the rotor onto the frame at 0; after it the drive is
    sensorless."""
    drive = rig.board.drive
    now = view['clock'].now()
    index, segment, name, into, seconds, rpm, load, how = stage_at(view, now)
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    top = (no_load_rpm(view) or TOP_RPM) * RAD_S_PER_RPM
    w_hat = speed(view) / pairs
    loop = view.get('speed_loop') or view.setdefault('speed_loop', _loop(view))
    pi = loop['pi']
    p = view['params']
    kt = 1.5 * pairs * (p.get('motor_lambda') or 0.005)
    through = SEND_FROM * (p.get('drv_w_hi') or 0.0) / pairs
    pi.limit = SPIN_A if abs(w_hat) < through else max(SPIN_A, p.get('drv_i_max') or SPIN_A)
    if abs(w_hat) > LOST * top and how == 'speed':
        # The demo's operator: the estimate lost, the cycle again from the pull onto the frame.
        view['said'] = 'estimate lost at %.0f rpm - aligned again' % (w_hat / RAD_S_PER_RPM)
        view['spin_at'], view['stage_index'] = now + start_of(START_AT), None
        index, segment, name, into, seconds, rpm, load, how = stage_at(view, now)
    if index != view.get('stage_index'):
        view['stage_index'], view['stage'], view['segment'] = index, name, segment
        view['stage_load'], view['load_full'] = 0.0, load * kt
        mode = 'sensorless' if how == 'speed' else 'hold'
        if mode != view.get('stage_mode'):
            if mode == 'hold':
                drive.hold()
            else:
                drive.on('sensorless')
        view['stage_mode'] = mode
        # A vector held or stepped from where the rotor stands; the cycle's first pulls it onto
        # the frame at 0, the estimate starting on its polarity.
        view['held_theta'] = (0.0 if index == 0
                              else (view.get('state') or {}).get('theta_hat') or 0.0)
        # From where the rotor is: the spool from its speed, its clock at 0, the PI with no
        # history.
        view['spool_from'], view['spool_x'] = w_hat, 0.0
        pi.reset()
        pi.was = w_hat
        view['speed_at'] = now
    dt = min(0.25, max(0.0, now - view.get('speed_at', now)))
    view['speed_at'] = now
    if how == 'hold':
        view['stage_load'] = view.get('load_full', 0.0)       # gravity's: whatever the speed
    elif rpm:
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
    else:
        world_load(view, view.get('stage_load', 0.0))
    if how == 'hold':
        drive.write(id_ref=HOLD_A, iq_ref=0.0, omega_target=0.0, theta=view['held_theta'])
        view['iq'] = 0.0
        return
    if how == 'step':
        # A step a time: the vector eased over STEP_EASE of its interval, then held - a
        # stepper's staircase, microstepped at the page's rate.
        turn = rpm or 0.0
        every = STEP_E_DEG / max(1e-6, abs(turn) * 6.0 * pairs)
        k, part = divmod(into / every, 1.0)
        eased = min(1.0, part / STEP_EASE)
        steps = k + eased * eased * (3.0 - 2.0 * eased)
        drive.write(id_ref=STEP_A, iq_ref=0.0, omega_target=0.0,
                    theta=view['held_theta'] + math.copysign(
                        math.radians(STEP_E_DEG) * steps, turn))
        view['iq'] = 0.0
        return
    if rpm is None:
        # Let go: no torque, the bridge still switching - the rotor on its drag and the
        # propeller, the observer on the back-EMF. With the stage off the emulated board's
        # estimate fell from 3 308 to 238 rpm while the flywheel turned on, and stuck there
        # through the brake (2026-09-28). The spool and the PI ride the estimate to the brake.
        view['spool_from'] = pi.was = w_hat
        drive.write(id_ref=0.0, iq_ref=0.0)
        view['iq'] = 0.0
        return
    # The spool's clock waits for the rotor: on while the reference leads the estimate by less
    # than the error that alone commands the loop's reach. On regardless, the lag held under the
    # 10 A through zero came back at 2 110 rpm/s where the clamp opened (2026-09-28).
    w0, x = view.get('spool_from', w_hat), view.get('spool_x', 0.0)
    w_ref = w0 + (rpm * RAD_S_PER_RPM - w0) * spool(x)
    if abs(w_ref - w_hat) < pi.limit * kt / (math.tau * SPEED_HZ * view['j']):
        view['spool_x'] = x + dt / max(0.1, RAMP_SHARE * seconds)
    view['iq'] = pi.step(dt, setpoint=w_ref, measured=w_hat)['command'] if dt else view['iq']
    drive.write(id_ref=0.0, iq_ref=view['iq'])
