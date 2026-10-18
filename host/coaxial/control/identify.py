"""A drive identified on its board: held as a stepper, swung at tones by the board's own loop, read.

    rows = identify.campaign(rig, seconds=16.0)     # t, q, ia, ib off the board's DAQ, its hold's field
    got = identify.fitted(rows, gear=36.0)          # {J, b, f, gravity, residual, kt, poles}

The board plays the setpoint rows itself (device 12, a Direct regulator on `theta`); its DAQ
streams two phase currents and the A1335 in one stamped record (`RATE_HZ`); the rotor frame's
q current is the torque, kt (1.5 p lambda) times it, the A1335's angle at the window's middle
the rotor's. `machine.sysid.motor` fits J, b, f and the gravity on the shaft. The record's
drive part is the motor's (`drive.configure(**profile['drive'])`): a profile loaded whole on a
native rig lays its model on the world's plant, and the load's inertia and damping with it.
"""
import math

from coaxial.acquire.task import SENSOR_FIELDS
from coaxial.comm.hostclock import clock_of
from coaxial.devices.scaling import from_calibration
from machine import sysid
from machine.campaign import TONES, tone
from machine.parts import Direct

#: The hold's current, A, ramped up in RAMP steps; the first tone's swing, rad, the rest as
#: much slower (`campaign.tone`), at TONES_HZ - the stand's first tones and 2.9, the inertia's
#: torque growing as the tone squared: without it the hip's J read 1.65e-4 for 2.13e-4 -; the
#: board's loop, ticks a second, and a row's length, s; the DAQ's records a second - at 100 the
#: host keeps up at 12 ops a second; a record's mean current turns with the shaft, its vector
#: shrunk by the sine over the turn (`arrays` undoes it).
HOLD_A, RAMP, SWING_RAD, TONES_HZ = 2.0, 6, 1.2, TONES[0] + (2.9,)
LOOP_HZ, ROW_S, RATE_HZ = 1000, 0.01, 100.0
#: The DAQ's two phase channels, the third their sum's negative; the shaft's sensor bit.
PHASES = (0, 1)
SHAFT = 1 << [row['name'] for row in SENSOR_FIELDS].index('shaft angle')
ROWS = [('t', 'f8'), ('q', 'f8'), ('ia', 'f8'), ('ib', 'f8')]


class _Slots:
    prefilter = measure = estimator = None
    regulator = Direct(1000.0)


def held(rig, amps=HOLD_A):
    """The drive held as a stepper at `amps`: (theta0 rad el, the A1335's degrees there, its
    poll's period, s - a record's snapshot is the last poll, half a period old in the mean)."""
    board, drive, clock = rig.board, rig.drive, clock_of(rig)
    board.afe.on()
    board.transport.sleep(0.05)
    rig.gates.on()
    theta0 = drive.state()['theta_hat']
    drive.write(id_ref=amps / RAMP, iq_ref=0.0, theta=theta0, omega_target=0.0)
    drive.hold()
    for k in range(1, RAMP + 1):
        drive.write(id_ref=amps * k / RAMP)
        board.transport.sleep(0.05)
    polled, since = board.angle.state()['updates'], clock.now()
    board.transport.sleep(0.5)
    angle = board.angle.state()
    return theta0, angle['degrees'], (clock.now() - since) / max(angle['updates'] - polled, 1)


def played(rig, thetas, rate=RATE_HZ):
    """`thetas` (N,) rad el played by the board's loop a ROW_S each, its DAQ read throughout:
    (records, layout)."""
    board, ctrl, daq = rig.board, rig.ctrl, rig.board.daq
    ctrl.off()
    ctrl.load(_Slots())
    ctrl.wire('angle', 'theta', LOOP_HZ)
    layout = daq.configure(list(PHASES), clock='software', sample_rate=rate, sensors=SHAFT)
    rows = [(ROW_S, float(th)) for th in thetas] + [(0.0, float(thetas[-1]))]
    sent, free = 0, ctrl.state()['free']
    while sent < len(rows) and free >= 40:
        ctrl.rows(rows[sent:sent + 40])
        sent, free = sent + 40, free - 40
    daq.start()
    ctrl.on()
    records, passes = [], 0
    try:
        while True:
            records += daq.acquire(255, layout)
            passes += 1
            if passes % 4 and sent < len(rows):
                continue
            state = ctrl.state()
            if sent < len(rows) and state['free'] >= 40:
                ctrl.rows(rows[sent:sent + 40])
                sent += 40
            elif sent >= len(rows) and not state['playing'] and state['rows'] == 0:
                break
        for _ in range(30):
            got = daq.acquire(255, layout)
            records += got
            if not got:
                break
    finally:
        daq.stop()
        ctrl.off()
    return records, layout


def arrays(rig, records, layout, degrees0, poles, poll=0.0):
    """ROWS off `records`: t s on the board's clock, q the shaft from `degrees0` (rad, unwrapped)
    at the window's middle - the snapshot at its close is the A1335's last `poll`, half a period
    old -, ia and ib the window's mean currents, A (Clarke of two phases), grown back by the sine
    over the vector's turn through the window at `poles` times the shaft's speed."""
    import numpy as np
    hz = float(rig.board.system.clock()['sysclk_hz'])
    phase = from_calibration(rig.board.calibration.read())['phase']
    names = [f['signal'] for f in layout['fields']]
    at = np.unwrap(np.array([r['at'] for r in records], float), period=2.0 ** 32) / hz
    count = np.maximum(np.array([r['samples'] for r in records], float), 1.0)
    iu, iv = (phase.amps(np.array([r[n] for r in records], float) / count) for n in names)
    counts = np.array([(r['sensors']['shaft angle'][0] & 0xFFFF) & 0x0FFF for r in records], float)
    q = np.unwrap(counts * 2.0 * math.pi / 4096.0 - math.radians(degrees0))
    out = np.zeros(len(records), np.dtype(ROWS))
    out['t'] = at - at[0]
    window = float(np.median(np.diff(out['t'])))
    out['q'] = np.interp(out['t'] - 0.5 * window + 0.5 * poll, out['t'], q)
    shrunk = np.sinc(0.5 * poles * np.gradient(out['q'], out['t']) * window / math.pi)
    out['ia'], out['ib'] = iu / shrunk, (2.0 * iv + iu) / math.sqrt(3.0) / shrunk
    return out


def torque(rows, kt, poles, theta0):
    """(N,) N m: kt times the rotor frame's q current, the rotor at `poles` times the shaft from
    the hold's field `theta0` (rad el) - the A1335's zero there."""
    import numpy as np
    th = poles * rows['q'] + theta0
    return kt * (-rows['ia'] * np.sin(th) + rows['ib'] * np.cos(th))


def campaign(rig, seconds=16.0, amps=HOLD_A, swing=SWING_RAD, rate=RATE_HZ, seed=1):
    """One drive's campaign on its board: held at `amps`, swung `swing` rad at the tones for
    `seconds`, read; {rows (ROWS), tau (N,), kt, poles, theta0}."""
    import numpy as np
    rig.drive.configure(drv_i_max=max(amps + 1.0, rig.drive.params().get('drv_i_max', 0.0)))
    p = rig.drive.params()
    poles, kt = p['motor_pole_pairs'], 1.5 * p['motor_pole_pairs'] * p['motor_lambda']
    theta0, degrees0, poll = held(rig, amps)
    try:
        t = np.arange(int(round(seconds / ROW_S)) + 1) * ROW_S
        phase = np.random.default_rng(seed).uniform(0.0, 2.0 * math.pi, len(TONES_HZ))
        records, layout = played(rig, theta0 + poles * tone(t, TONES_HZ, phase, swing), rate)
    finally:
        rig.drive.off()
        rig.gates.off()
    rows = arrays(rig, records, layout, degrees0, poles, poll)
    return {'rows': rows, 'tau': torque(rows, kt, poles, theta0), 'kt': kt, 'poles': poles, 'theta0': theta0, 'poll': poll}


def fitted(got, gear=1.0):
    """`campaign`'s rows fitted (`sysid.motor`), the gravity's sine on the shaft over `gear`."""
    rows = got['rows']
    return dict(sysid.motor(rows['t'], rows['q'], got['tau'], gear), kt=got['kt'], poles=got['poles'])
