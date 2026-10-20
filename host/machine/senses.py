"""What a craft's law reads of its frame: its IMU, an RTK receiver on the bus, a star camera.

    eyes = senses.eyes(seed)                 # the instruments, none read yet
    seen = senses.read(eyes, state, dt)      # the frame `dt` s on as they give it
    seen = senses.seen(eyes, state)          # as last read, nothing stepped
    seen = senses.told(eyes, state, dt)      # either, or the truth where `eyes` is None

The IMU is the board's, the receiver a Modbus RTU slave on its 10 Mbit RS485 bus, the camera
the receiver's stand-in where the signal is lost. The attitude is the IMU's fused with the
fixes, off by a slow tilt; its rates and specific force noisy. Place and speed are off by what
those errors integrate to - the frame's own kept: integrated afresh, they parted from MuJoCo's
substeps by 1.6 cm through a knife-in -, each fix pulling them in by a Kalman filter's steady
gains against the error it was taken at: the tilt on a pull leaves them 0.35 m off a m/s^2,
a knife-in's 1-2 cm; with the drift a third state, its estimate chased the pulls 0.045 m/s^2
off and a loss drifted 1 m. Lost, they are dead-reckoned on the star camera's attitude: the
drift is the speed's error at the loss. A gust is felt at once in the specific force; the
instruments set how well the frame holds its place.
"""
import collections
import math
import random

#: The attitude, the IMU's fused with the fixes: tilted ATT_RAD rms over ATT_S, an RTK-aided
#: MEMS INS's roll and pitch (Xsens MTi-680G: 0.2 deg) - on the BNO085's own 1 deg the height
#: ran 1.8 cm high through a knife-in and its tip struck. The gyros' noise, rad/s/sqrt(Hz), the
#: specific force's, m/s^2/sqrt(Hz): Bosch's BMI055 class.
ATT_RAD, ATT_S, GYRO, ACC = math.radians(0.2), 10.0, math.radians(0.014), 150e-6 * 9.81
#: The RTK receiver's fix: a second, s behind - its poll on the bus 0.1 ms of it -, its
#: place's noise, m rms, its speed's, m/s.
GNSS = (20.0, 0.06, 0.02, 0.03)
#: The star camera's attitude: a second, s behind, rad rms.
STAR = (5.0, 0.1, math.radians(20.0 / 3600.0))
#: The signal lost about every LOSE_S, for LOST_S.
LOSE_S, LOST_S = 90.0, (8.0, 15.0)
#: The seconds of the estimate kept to meet a fix taken behind it.
KEEP_S = 0.5
GRAVITY = 9.81


def _gains(rate, pos_m, vel_m, acc):
    """The steady gains of a Kalman filter on place and speed fixed `rate` a second with `pos_m`
    and `vel_m` of noise, its speed off by `acc` m/s^2 held a fix: 2x2, (place, speed) of the
    misses (place, speed)."""
    t, q = 1.0 / rate, acc * acc
    q = ((q * t ** 4 / 4.0, q * t ** 3 / 2.0), (q * t ** 3 / 2.0, q * t * t))
    p, k = ((pos_m * pos_m, 0.0), (0.0, vel_m * vel_m)), []
    for _ in range(200):
        m = ((p[0][0] + 2.0 * t * p[0][1] + t * t * p[1][1] + q[0][0], p[0][1] + t * p[1][1] + q[0][1]),
             (p[0][1] + t * p[1][1] + q[1][0], p[1][1] + q[1][1]))
        s = (m[0][0] + pos_m * pos_m, m[0][1], m[1][0], m[1][1] + vel_m * vel_m)
        det = s[0] * s[3] - s[1] * s[2]
        inv = ((s[3] / det, -s[1] / det), (-s[2] / det, s[0] / det))
        k = [[m[r][0] * inv[0][c] + m[r][1] * inv[1][c] for c in range(2)] for r in range(2)]
        p = tuple(tuple(m[r][c] - k[r][0] * m[0][c] - k[r][1] * m[1][c] for c in range(2)) for r in range(2))
    return k


#: The fixes' gains: the speed off by the tilt on gravity.
GAINS = _gains(GNSS[0], GNSS[2], GNSS[3], GRAVITY * ATT_RAD)


def eyes(seed=1, lose=True):
    """A craft's instruments, none read yet - the place's and speed's errors 'p' and 'v', the
    tilt 'err' -; its signal lost now and then where `lose`."""
    dice = random.Random(seed)
    return {'dice': dice, 't': 0.0, 'err': [0.0] * 3, 'p': [0.0] * 3, 'v': [0.0] * 3, 'was': None,
            'hist': collections.deque(), 'fixes': collections.deque(), 'gnss': 0.0, 'star': 0.0,
            'lost': False, 'until': dice.uniform(0.5, 1.5) * LOSE_S if lose else math.inf,
            'seen': None}


def _fixed(eyes, t):
    """The fixes due by `t` laid on the errors, each against what they were when it was taken
    and that carried on to now - the errors kept since with it."""
    hist, fixes, (kpp, kpv), (kvp, kvv) = eyes['hist'], eyes['fixes'], *GAINS
    while fixes and fixes[0][0] <= t + 1e-12:
        _due, took, fp, fv, cam = fixes.popleft()
        then = next((h for h in reversed(hist) if h[0] <= took + 1e-9), hist[0])
        if cam is not None:
            shift = [e0 - c for e0, c in zip(then[3], cam)]
            for h in [h for h in hist if h[0] > took] + [(t, None, None, eyes['err'])]:
                h[3][:] = [e - s for e, s in zip(h[3], shift)]
            continue
        dp = [kpp * (f - p) + kpv * (w - v) for f, w, p, v in zip(fp, fv, then[1], then[2])]
        dv = [kvp * (f - p) + kvv * (w - v) for f, w, p, v in zip(fp, fv, then[1], then[2])]
        for h in [h for h in hist if h[0] > took] + [(t, eyes['p'], eyes['v'], None)]:
            h[1][:] = [x + a + b * (h[0] - took) for x, a, b in zip(h[1], dp, dv)]
            h[2][:] = [x + b for x, b in zip(h[2], dv)]


def read(eyes, state, dt):
    """The frame `state` (`Sky.state`) as its instruments give it `dt` s on: its attitude and
    rates the IMU's, its place, speed and height the estimate's, 'lost' the signal; the rest its
    own."""
    turn = [[float(c) for c in r] for r in state['turn']]
    vel = [float(x) for x in state['vel']]
    dice = eyes['dice']
    g, was = dice.gauss, eyes['was'] or vel
    t = eyes['t'] = eyes['t'] + dt
    if t >= eyes['until']:
        eyes['lost'] = not eyes['lost']
        eyes['until'] = t + (dice.uniform(*LOST_S) if eyes['lost'] else dice.uniform(0.5, 1.5) * LOSE_S)
    if eyes['lost']:
        eyes['err'][:] = [e + GYRO * math.sqrt(dt) * g(0.0, 1.0) for e in eyes['err']]
    else:
        keep = math.exp(-dt / ATT_S)
        eyes['err'][:] = [e * keep + ATT_RAD * math.sqrt(1.0 - keep * keep) * g(0.0, 1.0)
                          for e in eyes['err']]
    # The specific force as the tilt and the IMU's noise have it, m/s^2 off in the world.
    noise = ACC / math.sqrt(2.0 * dt)
    (x, y, z), p, v = eyes['err'], eyes['p'], eyes['v']
    f = [(c - w) / dt + GRAVITY * (a == 1) for a, (c, w) in enumerate(zip(vel, was))]
    off = (y * f[2] - z * f[1], z * f[0] - x * f[2], x * f[1] - y * f[0])
    eyes['was'] = vel
    for a in range(3):
        v[a] += (off[a] + noise * g(0.0, 1.0)) * dt
        p[a] += v[a] * dt
    hist = eyes['hist']
    hist.append((t, list(p), list(v), list(eyes['err'])))
    while hist[0][0] < t - KEEP_S:
        hist.popleft()
    if eyes['lost'] and t >= eyes['star']:
        rate, behind, cam = STAR
        eyes['star'] = t + 1.0 / rate
        eyes['fixes'].append((t + behind, t, (), (), [cam * g(0.0, 1.0) for _ in range(3)]))
    elif not eyes['lost'] and t >= eyes['gnss']:
        rate, behind, pos_m, vel_m = GNSS
        eyes['gnss'] = t + 1.0 / rate
        eyes['fixes'].append((t + behind, t, [pos_m * g(0.0, 1.0) for _ in range(3)],
                              [vel_m * g(0.0, 1.0) for _ in range(3)], None))
    _fixed(eyes, t)
    x, y, z = eyes['err']
    noise = GYRO / math.sqrt(2.0 * dt)
    t0, t1, t2 = turn[0], turn[1], turn[2]
    turn_out = [
        [t0[0] - z * t1[0] + y * t2[0], t0[1] - z * t1[1] + y * t2[1], t0[2] - z * t1[2] + y * t2[2]],
        [z * t0[0] + t1[0] - x * t2[0], z * t0[1] + t1[1] - x * t2[1], z * t0[2] + t1[2] - x * t2[2]],
        [-y * t0[0] + x * t1[0] + t2[0], -y * t0[1] + x * t1[1] + t2[1], -y * t0[2] + x * t1[2] + t2[2]],
    ]
    seen = dict(state, at=[float(c) + d for c, d in zip(state['at'], p)], lost=eyes['lost'],
                vel=[c + d for c, d in zip(vel, v)], h=float(state['h']) + p[1], v=vel[1] + v[1],
                turn=turn_out, spin=[float(s) + noise * g(0.0, 1.0) for s in state['spin']])
    eyes['seen'] = seen
    return seen


def seen(eyes, state):
    """The frame as last read (`read`), nothing stepped; as it is where nothing was read yet."""
    return eyes['seen'] if eyes['seen'] is not None else dict(state, lost=False)


def told(eyes, state, dt=0.0):
    """The frame as a law reads it on `eyes`: `dt` s on (`read`), as last read at none (`seen`);
    its truth where `eyes` is None."""
    if eyes is None:
        return state
    return read(eyes, state, dt) if dt else seen(eyes, state)


def _bus_row_svg(k, title, mbps, nodes):
    y0, col = 112 + k * 108, ('#38bdf8', '#a78bfa', '#34d399', '#f472b6', '#fbbf24')[k % 5]
    out = [
        '<g font-family="monospace">',
        '<path d="M 220 280 L 250 %d" stroke="%s" stroke-width="2.5" fill="none"/>' % (y0 + 28, col),
        '<line x1="250" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="3" stroke-dasharray="6,3"/>'
        % (y0 + 28, 260 + len(nodes) * 134, y0 + 28, col),
        '<rect x="250" y="%d" width="215" height="20" rx="4" fill="#0f172a" stroke="%s" stroke-width="1.2"/>' % (y0, col),
        '<text x="258" y="%d" fill="%s" font-size="11" font-weight="bold">BUS %d: %s (%.0f Mbit/s)</text>'
        % (y0 + 14, col, k + 1, title, mbps),
    ]
    for u, (name, sub, spec, active) in enumerate(nodes, 1):
        nx, fill = 260 + (u - 1) * 134, ('#1e293b' if active else '#0f172a')
        out += [
            '<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="1.8"/>' % (nx + 60, y0 + 28, nx + 60, y0 + 40, col),
            '<rect x="%d" y="%d" width="124" height="54" rx="5" fill="%s" stroke="%s" stroke-width="1.3"/>' % (nx, y0 + 40, fill, col),
            '<text x="%d" y="%d" fill="#f8fafc" font-size="10" font-weight="bold">#%d %s</text>' % (nx + 6, y0 + 54, u, name),
            '<text x="%d" y="%d" fill="#94a3b8" font-size="9">%s</text>' % (nx + 6, y0 + 69, sub),
            '<text x="%d" y="%d" fill="%s" font-size="9">%s</text>' % (nx + 6, y0 + 83, col, spec),
        ]
    return '\n'.join(out + ['</g>'])


def aircraft_bus_layout_svg():
    """Render the live 10 Mbit/s RS485 Modbus RTU aircraft drive, servo & avionics bus topology
    as a deterministic SVG diagram for `docs/architecture/aircraft_bus_layout.svg`."""
    from machine import edge, flightpack, pull, quad, rtu
    from machine.aerobat import airframe as aero_af
    from machine.helicopter import airframe as heli_af
    from machine.ornithopter import airframe as orni_af
    mbps, reg = edge.BAUD / 1e6, rtu.SETPOINT_REG
    quad_nodes = [(n, 'r%02d / reg %#06x' % (i, reg + 2 * i), '63100 / %.0f-%.0f A' % (pull.I_MAX, pull.I_WEP), True)
                  for i, n in enumerate(('rotor_fl', 'rotor_fr', 'rotor_rr', 'rotor_rl'))]
    heli_nodes = [
        ('main_drive', 'd00 / reg %#06x' % reg, '63100 / %.0f rad/s' % heli_af.GOV_RAD_S, True),
        ('tail_drive', 'd01 / reg %#06x' % (reg + 2), '63100 / %.0f rad/s' % heli_af.TAIL_GOV_RAD_S, True),
        ('swash_coll', 's00 / reg %#06x' % (reg + 4), 'pitch +/-%.0f deg' % heli_af.COLL_DEG, True),
        ('swash_cyc', 's01 / reg %#06x' % (reg + 6), 'tilt +/-%.0f deg' % heli_af.CYC_DEG, True),
        ('tail_servo', 's02 / reg %#06x' % (reg + 8), 'yaw +/-%.0f deg' % heli_af.TAIL_DEG, True),
    ]
    orni_nodes = [
        ('fore_drive', 'd00 / reg %#06x' % reg, '63100 / %.0f rad/s' % orni_af.GOV_RAD_S, True),
        ('aft_drive', 'd01 / reg %#06x' % (reg + 2), '63100 / 1:%.1f gear' % orni_af.GEAR_RATIO, True),
        ('wing_stroke', 's00 / reg %#06x' % (reg + 4), 'flap +/-%.0f deg' % orni_af.STROKE_DEG, True),
        ('wing_pitch', 's01 / reg %#06x' % (reg + 6), 'twist +/-%.0f deg' % orni_af.PITCH_DEG, True),
        ('wing_fold', 's02 / reg %#06x' % (reg + 8), 'sweep %.0f deg' % orni_af.FOLD_DEG, True),
    ]
    aero_nodes = [
        ('prop_drive', 'd00 / reg %#06x' % reg, '63100 / %.0f rad/s' % aero_af.W_TOP, True),
        ('vect_drive', 'd01 / reg %#06x' % (reg + 2), '63100 / %.0f rad/s' % aero_af.VECT_GOV_RAD_S, True),
        ('elevator', 's00 / reg %#06x' % (reg + 4), '3D +/-%.0f deg' % aero_af.ELEV_DEG, True),
        ('ailerons', 's01 / reg %#06x' % (reg + 6), '3D +/-%.0f deg' % aero_af.AIL_DEG, True),
        ('rudder', 's02 / reg %#06x' % (reg + 8), '3D +/-%.0f deg' % aero_af.RUD_DEG, True),
    ]
    sense_nodes = [
        ('bmi055_imu', 'senses.GYRO/ACC', '%.1f deg RTK INS' % math.degrees(ATT_RAD), True),
        ('rtk_gnss', 'senses.GNSS', '%.0f Hz / %.0f cm rms' % (GNSS[0], GNSS[2] * 100.0), True),
        ('star_cam', 'senses.STAR', '%.0f Hz dead-reckon' % STAR[0], False),
        ('flightpack', '%dS LiPo bus' % flightpack.PACK_CELLS, '%.0f V / %.2f ohm' % (flightpack.open_volts(1.0), flightpack.PACK_OHM), True),
        ('device_14', 'cmd_aero.c', '%.0f Hz C11 core' % (1.0 / quad.STEP_S), True),
    ]
    buses = (('quad', quad_nodes), ('helicopter', heli_nodes), ('ornithopter', orni_nodes),
             ('aerobat', aero_nodes), ('avionics', sense_nodes))
    banks = ('SETPOINT %#06x' % rtu.SETPOINT_REG, 'KP       %#06x' % rtu.KP_REG,
             'FEED     %#06x' % rtu.FEED_REG, 'LIMIT    %#06x' % rtu.LIMIT_REG,
             'STATE    %#06x' % rtu.STATE_REG, 'AERO_OP  0x000e')
    head = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1220 665" width="100%" height="100%">',
        '<rect width="1220" height="665" rx="10" fill="#020617"/>',
        '<g font-family="monospace">',
        '<text x="24" y="34" fill="#f8fafc" font-size="16" font-weight="bold">'
        'COAXIAL 63100 -- %.0f Mbit/s RS485 Modbus RTU Aircraft Drive, Servo &amp; Avionics Buses (4 Airframes, 10 SIL Drives)</text>' % mbps,
        '<text x="24" y="54" fill="#94a3b8" font-size="11">'
        'Generated from machine.senses (GNSS=%.0f Hz, STAR=%.0f Hz), machine.pull (I_MAX=%.0f A), flightpack (%dS %.0f V), and craft airframes</text>'
        % (GNSS[0], STAR[0], pull.I_MAX, flightpack.PACK_CELLS, flightpack.open_volts(1.0)),
        '<rect x="20" y="160" width="200" height="245" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="2"/>',
        '<text x="32" y="186" fill="#38bdf8" font-size="12" font-weight="bold">FLIGHT LAW &amp; MASTER</text>',
        '<text x="32" y="204" fill="#e2e8f0" font-size="10">aero/ + SO(3) craft laws</text>',
        '<text x="32" y="220" fill="#94a3b8" font-size="10">CoaxialDrives (240-500 Hz)</text>',
        '<line x1="32" y1="232" x2="208" y2="232" stroke="#334155" stroke-width="1"/>',
        '<text x="32" y="250" fill="#f8fafc" font-size="10" font-weight="bold">Modbus &amp; Wire Banks:</text>',
    ] + ['<text x="32" y="%d" fill="#cbd5e1" font-size="10">%s</text>' % (268 + i * 18, b) for i, b in enumerate(banks)] + ['</g>']
    rows = [_bus_row_svg(k, name, mbps, nodes) for k, (name, nodes) in enumerate(buses)]
    return '\n'.join(head + rows + ['</svg>\n'])


def _layer_row_svg(k, title, cards):
    y_line, col = 130 + k * 114, ('#38bdf8', '#a78bfa', '#34d399', '#fbbf24', '#f472b6')[k % 5]
    y_tag, y_card = y_line - 28, y_line + 12
    out = [
        '<g font-family="monospace">',
        '<path d="M 225 295 L 255 %d" stroke="%s" stroke-width="2.5" fill="none"/>' % (y_line, col),
        '<line x1="255" y1="%d" x2="1256" y2="%d" stroke="%s" stroke-width="3" stroke-dasharray="6,3"/>' % (y_line, y_line, col),
        '<rect x="255" y="%d" width="350" height="20" rx="4" fill="#0f172a" stroke="%s" stroke-width="1.2"/>' % (y_tag, col),
        '<text x="263" y="%d" fill="%s" font-size="11" font-weight="bold">LAYER %d: %s</text>' % (y_tag + 14, col, k + 1, title),
    ]
    for u, (head, l1, l2, l3) in enumerate(cards, 1):
        nx = 265 + (u - 1) * 200
        out += [
            '<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="1.8"/>' % (nx + 92, y_line, nx + 92, y_card, col),
            '<rect x="%d" y="%d" width="184" height="62" rx="5" fill="#1e293b" stroke="%s" stroke-width="1.3"/>' % (nx, y_card, col),
            '<text x="%d" y="%d" fill="#f8fafc" font-size="10" font-weight="bold">#%d %s</text>' % (nx + 8, y_card + 14, u, head),
            '<text x="%d" y="%d" fill="#94a3b8" font-size="9">%s</text>' % (nx + 8, y_card + 29, l1),
            '<text x="%d" y="%d" fill="#cbd5e1" font-size="9">%s</text>' % (nx + 8, y_card + 43, l2),
            '<text x="%d" y="%d" fill="%s" font-size="9">%s</text>' % (nx + 8, y_card + 56, col, l3),
        ]
    return '\n'.join(out + ['</g>'])


def aircraft_software_layout_svg():
    """Render the live 5-layer aircraft flight control & simulation architecture as a deterministic
    SVG diagram for `docs/architecture/aircraft_software_layout.svg`."""
    from machine import flightpack, pull, quad
    from machine.aerobat import airframe as aero_af
    from machine.helicopter import airframe as heli_af
    quad_hz = round(1.0 / quad.STEP_S)
    shared = (
        ('#38bdf8', 'cmd / frames  IPC (60 Hz)'),
        ('#a78bfa', 'seen[state]   senses.told'),
        ('#a78bfa', 'air[wx,wy,wz] weather.wind_at'),
        ('#34d399', 'payload[66B]  aerolisp/aerokin'),
        ('#34d399', 'alpha, pull   wingborne.plane'),
        ('#fbbf24', 'acc, target_R helicopter.kit'),
        ('#fbbf24', 'w[4], ctrl[4] aero/ &amp; laws'),
        ('#f472b6', 'aeh_world_t   10x RK2 substep'),
        ('#f472b6', '%dS %.0f V LiPo  %.0f-%.0f A 63100' % (flightpack.PACK_CELLS, flightpack.open_volts(1.0), pull.I_MAX, pull.I_WEP)),
    )
    layers = (
        ('terminal, views &amp; flight worker (60 Hz)', (
            ('show_&lt;craft&gt;.py', 'quad / heli / orni / aerobat', 'chase &amp; spectator cam', 'out: mode, weather, card'),
            ('views/flying.py', 'Flying subprocess worker', 'playback.py LAG_S = 60 ms', 'in: 60 Hz state snapshots'),
            ('aerolisp.py &amp; *.lisp', '4 craft manifests + shared', 'stream_inject &amp; defblock', 'out: CompiledAeroPayload'),
            ('grounds &amp; airfield', 'gates, limbo, hoop, slalom', 'L1 arcs &amp; sweep.py', 'out: laid lines &amp; solids'),
            ('coaxial/ &amp; rig.py', '0x6E wire / device 14 aero', 'motion.spun &amp; started', 'Kalman &amp; thermal twin'),
        )),
        ('sensing, weather &amp; observers (240-%d Hz)' % quad_hz, (
            ('senses.py (eyes)', 'BMI055 IMU + %.0f Hz RTK' % GNSS[0], '%.0f Hz star camera backup' % STAR[0], 'out: seen(at,vel,turn,spin)'),
            ('weather.py (wind_at)', 'Dryden gusts &amp; eddies', 'calm / breeze / gale / tour', 'out: air(wx, wy, wz)'),
            ('aero.c ESO (quad)', '3-axis attitude &amp; pull ESO', 'wind vs push separation', 'in: aero_sense_t (%d Hz)' % quad_hz),
            ('observer.py (heli/orni)', 'SO(3) turn_eso (wo=%.0f)' % heli_af.WO_TURN, 'pull_eso &amp; wind_eso', 'in: seen &amp; controls (240 Hz)'),
            ('aerobat/observer.py', 'SO(3) turn_eso (wo=%.0f)' % aero_af.WO_TURN, 'slipstream &amp; pull_eso', 'out: d_tau, d_pull'),
        )),
        ('trajectory, kinematics &amp; guidance (240-%d Hz)' % quad_hz, (
            ('aerobatics.py &amp; cards', 'HOVER + figure setpoints', 'routines &amp; gate courses', 'out: row(pos,vel,lean,rates)'),
            ('aerokin.py (vacuum)', 'branch-free quad kinematics', 'collective + tilt + rates', 'out: AeroPayload (%d Hz)' % quad_hz),
            ('course.py &amp; airfield.py', 'L1 arc guidance &amp; gates', 'runway takeoff &amp; flare', 'out: path row &amp; trigger'),
            ('wingborne.py (plane)', 'post-stall + flat-plate lift', '10-step alpha bisection', 'out: alpha, target_R, pull'),
            ('helicopter/kit.py', 'shared SO(3) track &amp; step', 'hover, orbit &amp; line track', 'out: acc_cmd, target_R'),
        )),
        ('SO(3) flight laws &amp; allocation (240-%d Hz)' % quad_hz, (
            ('aero.c &amp; aero_turn.c', 'branch-free C11 core', 'impedance + CPG oscillator', 'tilt-prioritized SO(3) wrench'),
            ('aero_share.c &amp; pull.py', 'active-set thrust share', '%.0f A / %.0f A WEP reach' % (pull.I_MAX, pull.I_WEP), 'out: 4x rotor rad/s'),
            ('helicopter/law.py', 'collective + cyclic swash', 'tail anti-torque + piro', 'out: coll, cyc_p, cyc_r, tail'),
            ('ornithopter/law.py', 'stroke + pitch + roll_d', 'dive wing fold &amp; soar', 'out: 4x flapping controls'),
            ('aerobat/law.py', 'propeller + slipstream tail', 'knife-edge, harrier, flare', 'out: pitch, elev, ail, rud'),
        )),
        ('C harness, MuJoCo sky &amp; 63100 drives (1 kHz)', (
            ('aero/test/harness.c', 'aeh_step_heli/orni/aerobat', '10-substep RK2 aerodynamics', '0 direct qpos/qvel writes'),
            ('quad/heli/orni/aerobat', 'MuJoCo 6-DOF rigid body', 'skids, wheels &amp; contacts', 'out: state(at,vel,turn,spin)'),
            ('CoaxialDrives &amp; motion', '2-4x 63100 FOC boards', 'theta_hat phase-locked spin', 'out: actual rotor speeds'),
            ('flightpack.py (%dS LiPo)' % flightpack.PACK_CELLS, '%.1f V open, %.2f ohm sag' % (flightpack.open_volts(1.0), flightpack.PACK_OHM), 'regenerative braking charge', 'out: bus volts &amp; amps'),
            ('STM32H743 &amp; Stand-In', '35 kHz FOC + cmd_aero.c', 'native:// &amp; emulator://', 'see: aircraft_bus_layout.svg'),
        )),
    )
    head = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1280 685" width="100%" height="100%">',
        '<rect width="1280" height="685" rx="10" fill="#020617"/>',
        '<g font-family="monospace">',
        '<text x="24" y="34" fill="#f8fafc" font-size="16" font-weight="bold">'
        'COAXIAL 63100 -- Aircraft Flight Stack &amp; Real-Time SO(3) Control Architecture</text>',
        '<text x="24" y="54" fill="#94a3b8" font-size="11">'
        '5-Layer Pipeline: Terminal UI (60 Hz) &lt;-&gt; Sensing &amp; Weather &lt;-&gt; Trajectory &amp; Wingborne &lt;-&gt; SO(3) Laws (240-%d Hz) &lt;-&gt; C Harness &amp; 63100 Drives</text>' % quad_hz,
        '<rect x="20" y="150" width="205" height="295" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="2"/>',
        '<text x="32" y="176" fill="#38bdf8" font-size="12" font-weight="bold">FLIGHT ORCHESTRATOR</text>',
        '<text x="32" y="194" fill="#e2e8f0" font-size="10">views.flying / apart</text>',
        '<text x="32" y="210" fill="#94a3b8" font-size="10">Sky + Law + CoaxialDrives</text>',
        '<line x1="32" y1="222" x2="213" y2="222" stroke="#334155" stroke-width="1"/>',
        '<text x="32" y="240" fill="#f8fafc" font-size="10" font-weight="bold">Shared State &amp; Payloads:</text>',
    ] + ['<text x="32" y="%d" fill="%s" font-size="9">%s</text>' % (258 + i * 18, c, t) for i, (c, t) in enumerate(shared)] + [
        '<text x="32" y="424" fill="#94a3b8" font-size="9">0 direct qpos/qvel hacks</text>',
        '</g>',
    ]
    rows = [_layer_row_svg(k, title, cards) for k, (title, cards) in enumerate(layers)]
    return '\n'.join(head + rows + ['</svg>\n'])

