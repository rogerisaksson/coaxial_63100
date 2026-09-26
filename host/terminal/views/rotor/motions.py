"""The demo's own drive: bursts, the sweep, the load, the rock between."""
import math


#: The demo cycle's period, seconds.
SWEEP_S = 16.0

#: The heavy start (B, and every BURST_EVERY_S on the stand-in): 43 A for
#: 1 s, bounded by heat, not the clamp. Measured on the stand-in 2026-09-06:
#: 0.57 of the span on a cold board at the 80 % floor, 0.82 warm and stable;
#: 1.4-1.8 s reached no higher (0.68, 0.71). 38 A into a 40 A clamp reached
#: 0.70, so the clamp went to 50.
BURST_A = 43.0

BURST_S = 1.0

BURST_ACCEL = 12000.0

#: A burst every 45 s on the stand-in, long against the loops' 20 and 40 s;
#: then the burn: 3 s at 20 A against BURST_LOAD_NM at half the no-load
#: speed, where back-EMF times current reaches the kW bar.
BURST_EVERY_S = 45.0

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
    # The burst is part of the sequence, not only a key.
    # On the model only: the burst's load is the model's, and its clamp from any speed.
    if (view['demo'] and view['spin'] and view['source'] == 'model'
            and now - view['burst_at'] > BURST_EVERY_S):
        view['burst_at'] = now
        view['burst_until'] = now + BURST_S + BURST_HOLD_S
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


#: The demo cycle as fractions of SWEEP_S: hold, rock, send at the clamp,
#: brake. The send heats the legs so the margins move; the brake gets as
#: long as the send.
CYCLE_HOLD, CYCLE_ROCK, CYCLE_SEND = 0.14, 0.46, 0.73

#: What the rock peaks at, and what the first hold aligns with.
ROCK_RPM = 200.0

HOLD_A = 12.0

#: The rock's current, A: through zero speed the estimate rides the injection alone, and it
#: held a 5 A reversal; the clamp reversed through zero ran it to 1e5 rad/s on the emulated
#: flywheel.
ROCK_A = 5.0

#: The rock's speed-loop gain, A per rpm of error per second.
ROCK_GAIN = 0.01

#: Where the send takes the clamp, times the back-EMF's w_hi, and its current below: a 10 A
#: reversal through zero held on the emulated flywheel.
SEND_FROM = 1.5

SPIN_A = 10.0

#: An estimate past this share of the no-load speed is lost: the link cannot spin the motor
#: there.
LOST = 1.5

#: The brake pulls the whole clamp above this and proportionally less below,
#: so the rotor lands on zero: a quarter of the electrical no-load speed.
BRAKE_FULL_RAD_S = 700.0

#: Below this share of the clamp the brake lets go: the rotor has landed.
BRAKE_LANDED = 0.03


def speed(view):
    """The drive's own estimate, rad/s electrical: what it steers by. The chain beside it lost
    lock at the clamp's acceleration on the emulated flywheel and read 0 at 2 500."""
    return (view.get('state') or {}).get('omega_hat') or 0.0


def braking(view, clamp):
    """The clamp against the turning, proportionally less below BRAKE_FULL_RAD_S; none once
    landed."""
    turning = speed(view)
    share = min(1.0, abs(turning) / BRAKE_FULL_RAD_S)
    return -math.copysign(clamp * share, turning) if share > BRAKE_LANDED else 0.0


def cycle_phase(view):
    """Where in the demo cycle we are, and how far into that phase."""
    turn = ((view['clock'].now() - view['spin_at']) % SWEEP_S) / SWEEP_S
    if turn < CYCLE_HOLD:
        return 'hold', turn / CYCLE_HOLD
    if turn < CYCLE_ROCK:
        return 'rock', (turn - CYCLE_HOLD) / (CYCLE_ROCK - CYCLE_HOLD)
    if turn < CYCLE_SEND:
        return 'send', (turn - CYCLE_ROCK) / (CYCLE_SEND - CYCLE_ROCK)
    return 'brake', (turn - CYCLE_SEND) / (1.0 - CYCLE_SEND)


def sweep(rig, view):
    """The demo cycle: hold, rock, send, brake, and round again. The first hold pulls the rotor
    onto the frame at 0, so the estimate starts on its polarity; after it the drive stays
    sensorless and no full current crosses zero speed."""
    drive = rig.board.drive
    stage, into = cycle_phase(view)
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    clamp = view['params'].get('drv_i_max') or 5.0
    lost = LOST * no_load_rpm(view) / 60.0 * math.tau * pairs
    if view.get('aligned') and 0.0 < lost < abs(speed(view)):
        # The demo's operator: the estimate lost, the rotor pulled onto the frame at 0 again,
        # to the stage's end.
        view['aligned'] = False
        view['said'] = 'estimate lost at %.0f rad/s - aligned again' % speed(view)
        drive.hold()
    if stage != view['stage']:
        # A stage held through is an aligned rotor.
        view['aligned'] = view['stage'] is not None
        view['stage'] = stage
        view['leaning'] = False
        drive.model.configure(load=0.0)
        drive.on('sensorless') if view['aligned'] else drive.hold()
    if not view['aligned']:
        drive.write(id_ref=HOLD_A, iq_ref=0.0, omega_target=0.0, theta=0.0)
        view['iq'] = 0.0
        return
    if stage == 'hold':
        view['iq'] = braking(view, ROCK_A)
        drive.write(id_ref=0.0, iq_ref=view['iq'])
        return
    if stage == 'rock':
        # One swing each way: two in five seconds gave the integrator 2.8 s a
        # side and it never left 25 rpm.
        target = ROCK_RPM * math.sin(math.tau * into)
        view['iq'] = _toward(view, target, ROCK_A)
        drive.write(id_ref=0.0, iq_ref=view['iq'],
                    omega_target=abs(target) / 60.0 * math.tau * pairs)
        return
    if stage == 'send':
        # Through zero on SPIN_A; the clamp above the back-EMF's speed, where the estimate is
        # the back-EMF's.
        if speed(view) < SEND_FROM * (view['params'].get('drv_w_hi') or 0.0):
            view['iq'] = SPIN_A
            drive.write(id_ref=0.0, iq_ref=SPIN_A)
            return
        drive.write(id_ref=0.0, iq_ref=clamp, accel=BURST_ACCEL,
                    omega_target=no_load_rpm(view) / 60.0 * math.tau * pairs)
        view['iq'] = clamp
        return
    # The brake: the same current the other way until the rotor stops, then
    # none.
    view['iq'] = braking(view, clamp)
    drive.write(id_ref=0.0, iq_ref=view['iq'], omega_target=0.0)


def _toward(view, rpm, clamp):
    """The speed loop's integrator, one frame."""
    now = view['clock'].now()
    dt = min(0.5, max(0.0, now - view['sweep_at']))
    view['sweep_at'] = now
    pairs = max(1.0, view['params'].get('motor_pole_pairs') or 1.0)
    turning = speed(view) / pairs * 60.0 / math.tau
    return max(-clamp, min(clamp,
                           view['iq'] + ROCK_GAIN * (rpm - turning) * dt))
