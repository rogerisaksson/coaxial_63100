"""Her body's one edge: what a robot of her build measures of the world, what her drives are told.

    y = edge.laid(world)                 # (SENSED,), its layout once
    edge.felt(world, y)                  # her pelvis's IMU and her joints' encoders, every pass
    edge.read(world, y)                  # and her soles' loads and her segments' contacts
    edge.drive(world, joints, target=, feed=, kp=, limit=)    # her boards told, the next pass

Her law reads this array alone - an IMU on her pelvis (its turn as a rotation vector, its spin and
its specific force, its own frame), each joint's angle and rate and each motor's angle through its
gearbox, each sole's load cell, each segment's skin: the force on it of whatever it touches that
is not her, in its own frame -, each insole's centre of pressure, never the world's state; and writes her drives' setpoints, feed,
stiffness and clamps alone (test_structure).
"""
import ctypes
import functools
import math
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.figure import BALL, HEEL, JOINTS, SEGMENTS

G = np.array([0.0, -9.81, 0.0])
J = len(JOINTS)
#: Her segments with skin, `SEGMENTS`' order.
SKIN = tuple(seg[0] for seg in SEGMENTS)
#: Her soles' segments, each with an insole: (left foot, left toes, right foot, right toes).
INSOLES = ('left_foot', 'left_toes', 'right_foot', 'right_toes')
#: A foot's insole's cells: its heel's half and its ball's, split at the sole's middle.
FEET, MIDDLE_M = ('left_foot', 'right_foot'), 0.5 * (BALL - HEEL)
#: The array: the IMU's turn (w, x, y, z, world from pelvis), spin (rad/s) and specific force
#: (m/s^2), its frame; each joint's angle (rad) and rate (rad/s), each motor's angle seen through
#: its gearbox (rad); each sole's load (N); each segment's touch (N, its frame); each insole's
#: centre of pressure (m, its segment's frame) and its load (N); each foot's heel and ball cell's
#: load (N); where on each segment its touch is, its centre (m, its frame).
QUAT, GYRO, ACCEL = slice(0, 4), slice(4, 7), slice(7, 10)
Q, QD, QM = (slice(10 + k * J, 10 + (k + 1) * J) for k in range(3))
LOAD = slice(10 + 3 * J, 12 + 3 * J)
TOUCH = slice(12 + 3 * J, 12 + 3 * J + 3 * len(SKIN))
COP = slice(TOUCH.stop, TOUCH.stop + 3 * len(INSOLES))
PRESS = slice(COP.stop, COP.stop + len(INSOLES))
CELLS = slice(PRESS.stop, PRESS.stop + 2 * len(FEET))
AT = slice(CELLS.stop, CELLS.stop + 3 * len(SKIN))
SENSED = AT.stop
#: Her touch past this, N, a segment's: MuJoCo's soft contact holds a resting hand at a few.
TOUCHED_N = 5.0


def laid(world):
    """Her array for `world`, read: and the world's ids it reads by, kept on the world."""
    m = world.model
    hers = np.array([_rooted(m, b, world.pelvis) for b in range(m.nbody)])
    world.skin = np.full(m.nbody, -1, int)
    for k, name in enumerate(SKIN):
        world.skin[m.body(name).id] = k
    world.skin_bodies = np.array([m.body(name).id for name in SKIN])
    world.hers = hers
    world.insole = np.full(m.nbody, -1, int)
    for k, name in enumerate(INSOLES):
        world.insole[m.body(name).id] = k
    world.foot = np.full(m.nbody, -1, int)
    for k, name in enumerate(FEET):
        world.foot[m.body(name).id] = k
    from tools.cores import wbc as cores
    kept = {'gb': np.ascontiguousarray(m.geom_bodyid, dtype=np.int32),
            'hers': np.ascontiguousarray(hers, dtype=np.bool_),
            'sole_side': np.array([int(s[1]) if s.any() else -1 for s in world.soles], dtype=np.int32),
            'insole': np.ascontiguousarray(world.insole, dtype=np.int32),
            'foot': np.ascontiguousarray(world.foot, dtype=np.int32),
            'skin': np.ascontiguousarray(world.skin, dtype=np.int32),
            'loads': np.zeros(2), 'touch': np.zeros((len(SKIN), 3)), 'pressed': np.zeros(len(INSOLES)),
            'at': np.zeros((len(INSOLES), 3)), 'cells': np.zeros(2 * len(FEET)), 'felt_n': np.zeros(len(SKIN)),
            'felt_at': np.zeros((len(SKIN), 3))}
    c = cores.Contacts(nbody=m.nbody, nskin=len(SKIN), ninsoles=len(INSOLES), nfeet=len(FEET), middle_m=MIDDLE_M)
    for name in ('gb', 'sole_side', 'insole', 'foot', 'skin'):
        setattr(c, name, cores.pointer(kept[name], np.int32))
    c.hers = cores.pointer(kept['hers'], np.bool_)
    for name, a in (('xmat', world.data.xmat), ('xpos', world.data.xpos)):
        setattr(c, name, cores.pointer(a))
    for name in ('loads', 'touch', 'pressed', 'at', 'cells', 'felt_n', 'felt_at'):
        setattr(c, name, cores.pointer(kept[name]))
    #: The contacts' bins and the C that fills them (wbc_pass.c), the contacts' layout laid at
    #: the first contact.
    lib = cores.library()
    world.contacts = (c, kept, lib.wbh_pass_contacts, lib.wbh_pass_sensed)
    return read(world, np.zeros(SENSED))


def _rooted(m, body, pelvis):
    while body > 0:
        if body == pelvis:
            return True
        body = m.body_parentid[body]
    return False


def felt(world, y):
    """`y`'s IMU and encoders as the world stands now (wbc_pass.c, on the world's laid pass)."""
    _read()(ctypes.byref(world.passing), _cores().address(y), QUAT.start, GYRO.start, ACCEL.start, Q.start, QD.start,
            QM.start)
    return y


@functools.cache
def _read():
    """The C read, bound once."""
    return _cores().library().wbh_pass_felt


def read(world, y):
    """`y` whole: the IMU and encoders (`felt`), her soles' loads, her segments' touch and
    where on each it is."""
    felt(world, y)
    d = world.data
    c, kept, contacts, sensed = world.contacts
    ncon = d.ncon
    # the contacts between a body of hers and one not, in C: each's force its rows of the
    # solver's (the cone elliptic, `mjcf.CONE`), a bin a segment in the contacts' order
    if ncon:
        con = d.contact
        if not c.stride:
            base = con.pos.ctypes.data
            c.stride, c.off_geom, c.off_frame = con.pos.strides[0], con.geom.ctypes.data - base, con.frame.ctypes.data - base
            c.off_dim, c.off_efc = con.dim.ctypes.data - base, con.efc_address.ctypes.data - base
        contacts(ctypes.byref(c), ncon, con.pos.ctypes.data, d.efc_force.ctypes.data)
    else:
        contacts(ctypes.byref(c), 0, None, None)
    # a shove is a hand on her: what the world applies to her bodies, her skin feels - at
    # their centres of mass, where the world applies it
    if d.xfrc_applied.any():
        touch, felt_n, felt_at = kept['touch'], kept['felt_n'], kept['felt_at']
        sb = world.skin_bodies
        hands = d.xfrc_applied[sb, :3]
        for k, body in enumerate(sb):
            hand = hands[k]
            if hand.any():
                hn = float(np.linalg.norm(hand))
                rm_t = d.xmat[body].reshape(3, 3).T
                touch[k] += rm_t @ hand
                felt_n[k] += hn
                felt_at[k] += hn * rm_t @ (d.xipos[body] - d.xpos[body])
    sensed(ctypes.byref(c), y.ctypes.data, LOAD.start, TOUCH.start, PRESS.start, COP.start, CELLS.start, AT.start)
    return y


def touched(y):
    """(SKIN,) each segment's touch, N."""
    return np.linalg.norm(y[TOUCH].reshape(-1, 3), axis=1)


def tilt(y):
    """Her pelvis's tilt from upright as her IMU has it, deg."""
    q = y[QUAT]
    return math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * float(q[1] ** 2 + q[3] ** 2)))))


def yaw(y):
    """Her pelvis's turn about up as her IMU has it, rad."""
    q = y[QUAT]
    return 2.0 * math.atan2(float(q[2]), float(q[0]))


#: 10 Mbit/s RS485 Modbus RTU bus per extremity (`axis`, `left_arm`, `left_leg`, `right_arm`, `right_leg`).
BAUD = 10_000_000
LIMBS = (
    ('axis', ('spine', 'spine_roll', 'waist', 'neck', 'head')),
    ('left_arm', ('left_shoulder', 'left_elbow', 'left_wrist', 'left_gripper')),
    ('left_leg', ('left_hip_yaw', 'left_hip_roll', 'left_hip', 'left_knee', 'left_ankle', 'left_ankle_roll', 'left_foot')),
    ('right_arm', ('right_shoulder', 'right_elbow', 'right_wrist', 'right_gripper')),
    ('right_leg', ('right_hip_yaw', 'right_hip_roll', 'right_hip', 'right_knee', 'right_ankle', 'right_ankle_roll', 'right_foot')),
)
_SCALE = None


class LimbBus:

    """One extremity's 10 Mbit/s RS485 Modbus RTU SIL bus: commands cross as CRC-16/MODBUS frames
    into its SIL coaxial_63100 boards (`modbus_rtu.c`, `modbus_slave.c`) and can only address the
    boards wired to this extremity."""

    def __init__(self, link, name, joints, lib, baud=BAUD):
        self.link, self.name, self.joints, self.baud = int(link), name, tuple(joints), int(baud)
        self.indices = np.array([JOINTS.index(j) for j in self.joints], dtype=np.int32)
        self._lib = lib
        self._counts = np.zeros(4, dtype=np.uint64)
        self._wire = ctypes.c_double(0.0)
        self._buf = (ctypes.c_uint8 * 64)()
        self._len = ctypes.c_int(0)

    def _pull(self):
        self._lib.wbh_sil_stats(self.link, self._counts.ctypes.data, ctypes.byref(self._wire),
                                ctypes.byref(self._buf), ctypes.byref(self._len))

    @property
    def tx_bytes(self):
        self._pull()
        return int(self._counts[0])

    @property
    def frames(self):
        self._pull()
        return int(self._counts[1])

    @property
    def comm_errors(self):
        self._pull()
        return int(self._counts[2])

    @property
    def server_messages(self):
        self._pull()
        return int(self._counts[3])

    @property
    def wire_s(self):
        self._pull()
        return float(self._wire.value)

    @property
    def last_frame(self):
        self._pull()
        return bytes(self._buf[:self._len.value])

    def transceive(self, frame, world):
        """Verify CRC-16/MODBUS on `frame` via the SIL boards' `mb_rtu_t` + `mb_slave_t`, account
        10 Mbit/s 8N1 wire time, and apply the register write solely to this extremity's boards."""
        raw = bytes(frame)
        rc = int(self._lib.wbh_sil_transceive(ctypes.byref(world.passing), self.link, raw, len(raw)))
        if rc == -1:
            raise ValueError('invalid Modbus RTU broadcast on %s' % self.name)
        if rc == -2:
            raise ValueError('Modbus RTU unit span mismatch on %s' % self.name)
        if rc < 0:
            raise ValueError('Modbus RTU CRC-16 fault on %s' % self.name)


@functools.cache
def _cores():
    """`tools.cores.wbc`, imported on first use."""
    from tools.cores import wbc
    return wbc


def _buses_of(world):
    buses = getattr(world, 'limb_buses', None)
    if buses is None:
        lib = _cores().library()
        counts = np.array([len(jnts) for _, jnts in LIMBS], dtype=np.int32)
        flat = np.array([JOINTS.index(j) for _, jnts in LIMBS for j in jnts], dtype=np.int32)
        lib.wbh_sil_init(ctypes.byref(world.passing), len(LIMBS), counts.ctypes.data, flat.ctypes.data)
        buses = world.limb_buses = tuple(LimbBus(k, name, jnts, lib) for k, (name, jnts) in enumerate(LIMBS))
        world._sil_drive = lib.wbh_sil_drive
        world._idx_cache = {}
    return buses


def _c_arr(arr):
    """(`arr` laid contiguous in doubles, its address), (None, None) for none."""
    if arr is None:
        return None, None
    a = arr if (isinstance(arr, np.ndarray) and arr.dtype == np.float64 and arr.flags.c_contiguous) else np.ascontiguousarray(arr, dtype=np.float64)
    return a, _cores().address(a)


def drive(world, joints, target=None, feed=None, kp=None, limit=None):
    """Her SIL boards told for the next pass over each extremity's 10 Mbit/s RS485 Modbus RTU bus
    (`LIMBS`), each at `joints` (indices of JOINTS): their setpoints (rad), torques fed forward
    (N m), stiffness (N m/rad) and clamps (N m)."""
    _buses_of(world)
    cache = world._idx_cache
    key = id(joints) if isinstance(joints, np.ndarray) else tuple(joints)
    idx = cache.get(key)
    if idx is None:
        idx = cache[key] = np.ascontiguousarray(joints, dtype=np.int32)
    t_a, t_p = _c_arr(target)
    f_a, f_p = _c_arr(feed)
    k_a, k_p = _c_arr(kp)
    l_a, l_p = _c_arr(limit)
    world._sil_drive(ctypes.byref(world.passing), len(idx), _cores().address(idx), t_p, f_p, k_p, l_p)


def _bus_row_svg(k, name, joints, rtu, drives):
    y0 = 112 + k * 108
    mbps = BAUD / 1e6
    col = ('#38bdf8', '#a78bfa', '#34d399', '#f472b6', '#fbbf24')[k % 5]
    end_x = 260 + len(joints) * 134
    out = [
        '<g font-family="monospace">',
        '<path d="M 220 %d L 250 %d" stroke="%s" stroke-width="2.5" fill="none"/>' % (280, y0 + 28, col),
        '<line x1="250" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="3" stroke-dasharray="6,3"/>' % (y0 + 28, end_x, y0 + 28, col),
        '<rect x="250" y="%d" width="175" height="20" rx="4" fill="#0f172a" stroke="%s" stroke-width="1.2"/>' % (y0, col),
        '<text x="258" y="%d" fill="%s" font-size="11" font-weight="bold">BUS %d: %s (%.0f Mbit/s)</text>' % (y0 + 14, col, k + 1, name, mbps),
    ]
    for u, j in enumerate(joints, 1):
        nx = 260 + (u - 1) * 134
        pas = drives.passive(j)
        pk = 0.0 if pas else float(drives.peak(j))
        reg = rtu.SETPOINT_REG + 2 * (u - 1)
        spec = 'passive' if pas else ('%s / %.0f Nm' % (drives.of(j)[0], pk))
        fill = '#1e293b' if not pas else '#0f172a'
        out += [
            '<line x1="%d" y1="%d" x2="%d" y2="%d" stroke="%s" stroke-width="1.8"/>' % (nx + 60, y0 + 28, nx + 60, y0 + 40, col),
            '<rect x="%d" y="%d" width="124" height="54" rx="5" fill="%s" stroke="%s" stroke-width="1.3"/>' % (nx, y0 + 40, fill, col),
            '<text x="%d" y="%d" fill="#f8fafc" font-size="10" font-weight="bold">#%d %s</text>' % (nx + 6, y0 + 54, u, j),
            '<text x="%d" y="%d" fill="#94a3b8" font-size="9">j%02d / reg %#06x</text>' % (nx + 6, y0 + 69, JOINTS.index(j), reg),
            '<text x="%d" y="%d" fill="%s" font-size="9">%s</text>' % (nx + 6, y0 + 83, col, spec),
        ]
    out.append('</g>')
    return '\n'.join(out)


def bus_layout_svg():
    """Render the live 10 Mbit/s RS485 Modbus RTU per-extremity SIL bus architecture (`LIMBS`,
    `BAUD`, `rtu`, `drives`) as a deterministic SVG diagram for `docs/architecture/bus_layout.svg`."""
    from machine import drives, rtu
    total_boards = sum(len(j) for _, j in LIMBS)
    driven_boards = sum(1 for _, jnts in LIMBS for j in jnts if not drives.passive(j))
    banks = ('SETPOINT %#06x' % rtu.SETPOINT_REG, 'KP       %#06x' % rtu.KP_REG,
             'FEED     %#06x' % rtu.FEED_REG, 'LIMIT    %#06x' % rtu.LIMIT_REG,
             'STATE    %#06x' % rtu.STATE_REG, 'GATE     %#06x' % rtu.GATE_REG)
    head = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1220 665" width="100%" height="100%">',
        '<rect width="1220" height="665" rx="10" fill="#020617"/>',
        '<g font-family="monospace">',
        '<text x="24" y="34" fill="#f8fafc" font-size="16" font-weight="bold">'
        'COAXIAL 63100 -- %dx %.0f Mbit/s RS485 Modbus RTU Extremity Buses (%d SIL Boards, %d Driven)</text>'
        % (len(LIMBS), BAUD / 1e6, total_boards, driven_boards),
        '<text x="24" y="54" fill="#94a3b8" font-size="11">'
        'Generated from machine.edge (LIMBS, BAUD=%d), machine.rtu (CRC-16/MODBUS), and wbc_pass.c (mb_rtu_t + mb_slave_t)</text>' % BAUD,
        '<rect x="20" y="160" width="200" height="245" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="2"/>',
        '<text x="32" y="186" fill="#38bdf8" font-size="12" font-weight="bold">SOLVER &amp; SIL MASTER</text>',
        '<text x="32" y="204" fill="#e2e8f0" font-size="10">machine.motion -&gt; edge.drive</text>',
        '<text x="32" y="220" fill="#94a3b8" font-size="10">wbc_sil_drive (1 kHz)</text>',
        '<line x1="32" y1="232" x2="208" y2="232" stroke="#334155" stroke-width="1"/>',
        '<text x="32" y="250" fill="#f8fafc" font-size="10" font-weight="bold">Modbus RTU Banks (0x10):</text>',
    ]
    for i, b in enumerate(banks):
        head.append('<text x="32" y="%d" fill="#cbd5e1" font-size="10">%s</text>' % (268 + i * 18, b))
    head.append('</g>')
    rows = [_bus_row_svg(k, name, jnts, rtu, drives) for k, (name, jnts) in enumerate(LIMBS)]
    return '\n'.join(head + rows + ['</svg>\n'])


def software_layout_svg():
    """Render the live 5-layer gynoid software systems engineering & real-time control architecture
    as a deterministic SVG diagram for `docs/architecture/software_layout.svg`."""
    from machine import latent, modal, senses, sprung
    law_hz = round(sprung.RATE_HZ / sprung.STACK_EVERY)
    rate_khz = round(sprung.RATE_HZ / 1000.0)
    mbps = round(BAUD / 1e6)
    shared = (
        ('#38bdf8', 'cmd / states  IPC (60 Hz)'),
        ('#a78bfa', 'y_in[%d]     edge.SENSED' % SENSED),
        ('#a78bfa', 'y[%d], fr[%d] modal.SENSED' % (modal.SENSED, modal.FRAME)),
        ('#34d399', 'st[%d], tab  latent.STATE' % latent.STATE),
        ('#34d399', 'out[%d]       latent.OUT' % latent.OUT),
        ('#fbbf24', 'a[%d]         modal.ASK' % modal.ASK),
        ('#fbbf24', 'got[qacc,tau] wrench.solve'),
        ('#f472b6', 'eso[3,21]     %d kHz observer' % rate_khz),
        ('#f472b6', '%dx RS485 RTU  %d Mbit/s SIL' % (len(LIMBS), mbps)),
    )
    layers = (
        ('terminal, outer setpoint &amp; rig front door (60 Hz)', (
            ('show_humanoid.py', 'humanoid_keys / WASDQE', 'chase &amp; orbit cam', 'out: twist, say, charge'),
            ('worker.py &amp; se2.heard', 'outer command -&gt; setpoint', 'asked, twist, heading, tab', 'zero inner mode strings'),
            ('playback.py &amp; apart.py', 'LAG_S = 60 ms / SLICE_S', 'quat slerp &amp; pose lerp', 'in: 60 Hz state snapshots'),
            ('sight.py / scanner.py', 'stereo.py disparity', 'room occupancy &amp; props', 'out: chair / bed / table'),
            ('coaxial/ &amp; mcp/', 'rig.py / 0x6E wire / 8763', 'afe, gates, boot, acquire', 'Kalman &amp; thermal twin'),
        )),
        ('sensing, continuum touch &amp; outward situation (%d Hz, C)' % law_hz, (
            ('edge.py (y_in[%d])' % SENSED, 'IMU + 27 encoders + qm', '4 analog insoles + 28 skin', 'edge.felt &amp; edge.read'),
            ('estimate.py &amp; sense', 'bodied.on_body C-compiled', 'com, vcom, M, h, feet J6', 'out: qpos(34), qvel(33)'),
            ('borne.py &amp; grip.py', 'branchless support span', 'online sole friction mu', 'out: prop, contacts, held'),
            ('touch.py &amp; se2.floored', 'branchless skin admittance', 'insole floor height/pitch', 'out: twist, RAISE, GROUND'),
            ('situation.py (outward)', '6 continuous readings', 'into(S) -&gt; state[situation]', 'outward to UI / LLM only'),
        )),
        ('branchless inner SE(3) &amp; modal continuum (%d Hz, C)' % law_hz, (
            ('ortho.py &amp; *.lisp', 'gait / postures / stunts', '244-byte latent.WIRE rows', 'continuous table schedule'),
            ('latent.py &amp; se2.py', '11 modes + CPG phase THETA', 'continuous twist &amp; gripped', 'out: out[%d] (REF..DS)' % latent.OUT),
            ('support.py (couple)', 'upper-body admittance dq', 'ESO disturbance bleed', 'out: dq(21), PI @ phi'),
            ('impedance.py &amp; poise', 'spanned ZMP + h_axial', 'capture point xi = c+v/om', 'out: a[%d] (STANCE, ACC)' % modal.ASK),
            ('stride.py &amp; modal.py', 'beat, meets, heels, clears', 'SE(3) turns &amp; knee folds', 'out: ask dict for WBC'),
        )),
        ('whole-body solver, surety field &amp; health (%d Hz)' % law_hz, (
            ('wrench.py / wbc_wrench', 'reduced contact-wrench QP', '6D sole + prop contacts', '0/9668 C vs Python diff'),
            ('motion.surety (ukemi)', 'Wendland attitude &amp; support', 'convex Euler-Lagrange brace', 'out: qacc(33), tau(21)'),
            ('reach.py &amp; leap.py', 'analytic leg IK guards', 'airborne capture landing', 'reach.within &amp; reach.land'),
            ('heat.py &amp; drives.py', '2-node Cu/Stator thermal', 'WEP_S = 8.0 s power tank', 'voltage ellipse &amp; SOA'),
            ('sysid.py &amp; wear.py', 'RNE base regressor window', 'payload, friction, lash', 'out: identified.toml'),
        )),
        ('inner ESO, SIL buses &amp; MCU (%d kHz)' % rate_khz, (
            ('wbh_pass_drives (C)', '3rd-order ESO (wo=100)', 'qacc -&gt; ref_v, ref_q, kp', 'dt-scaled leak &amp; bleed'),
            ('wbc_sil.c &amp; rtu.py', '%dx %d Mbit/s RS485 buses' % (len(LIMBS), mbps), 'CRC-16/MODBUS + 27 slaves', 'see: bus_layout.svg'),
            ('physics.py (MuJoCo)', 'figure + mjcf + sneaker', '27 joints, shells &amp; pads', '1 kHz contact &amp; insoles'),
            ('STM32H743 Firmware', 'Cortex-M7 475 MHz (src/)', '35 kHz FOC + CORDIC + ADC', 'DRV8353RS + 12x IPT015N10'),
            ('Stand-In Emulator', 'world_libm.h (277 ns/step)', 'native:// &amp; emulator://', 'bit-identical telemetry'),
        )),
    )
    head = [
        '<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 1280 685" width="100%" height="100%">',
        '<rect width="1280" height="685" rx="10" fill="#020617"/>',
        '<g font-family="monospace">',
        '<text x="24" y="34" fill="#f8fafc" font-size="16" font-weight="bold">'
        'COAXIAL 63100 -- Software Systems Engineering &amp; Real-Time Control Architecture</text>',
        '<text x="24" y="54" fill="#94a3b8" font-size="11">'
        '5-Layer Data-Oriented Pipeline: Outer Setpoint UI (60 Hz) &lt;-&gt; Sensing &amp; Outward Situation (%d Hz) &lt;-&gt; Branchless SE(3)/Modal Continuum (%d Hz) &lt;-&gt; Inner ESO &amp; SIL (%d kHz)</text>'
        % (law_hz, law_hz, rate_khz),
        '<rect x="20" y="150" width="205" height="295" rx="8" fill="#0f172a" stroke="#38bdf8" stroke-width="2"/>',
        '<text x="32" y="176" fill="#38bdf8" font-size="12" font-weight="bold">RUNTIME ORCHESTRATOR</text>',
        '<text x="32" y="194" fill="#e2e8f0" font-size="10">machine.worker / apart</text>',
        '<text x="32" y="210" fill="#94a3b8" font-size="10">sprung.laid + motion.laid</text>',
        '<line x1="32" y1="222" x2="213" y2="222" stroke="#334155" stroke-width="1"/>',
        '<text x="32" y="240" fill="#f8fafc" font-size="10" font-weight="bold">Shared State Arrays:</text>',
    ] + ['<text x="32" y="%d" fill="%s" font-size="9">%s</text>' % (258 + i * 18, c, s) for i, (c, s) in enumerate(shared)] + [
        '<text x="32" y="424" fill="#94a3b8" font-size="9">0 branches in inner core</text>',
        '</g>',
    ]
    rows = [senses._layer_row_svg(k, title, cards) for k, (title, cards) in enumerate(layers)]
    return '\n'.join(head + rows + ['</svg>\n'])



