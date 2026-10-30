"""A drive's heat as its board keeps it: three nodes and the envelope on them, thermal.c's law.

The switches, the laminate under them and the winding, each a node lumped; the envelope's derate
and trip on them.

    heat = Heat([drives.heat(j), ..])   # the drives at AMBIENT_C, their gates on
    heat.load(k, torque)                # a tick's torque, N m, into drive k's mean square
    heat.step(dt, air, rds)             # dt s on: the nodes on their losses, the envelope
    heat.derate[k], heat.gates[k]       # the share of its clamp drive k gives, if anything
    heat.arm(k); heat.warm(k, celsius)  # the gates on again; its nodes warmed to at least
    heat.short(k); heat.shorted[k]      # its phases shorted through the low sides
    heat.off(k)                         # its gates off: nothing switched, nothing held
    heat.report(k)                      # (celsius, spent, derate, status): its reply's
    heat.envelope = False               # a fantasy board: its envelope counted, never binding

A drive is a board behind its outrunner and a cycloid (`machine.body.drives`): its torque an amp,
its winding, its board's losses as the 63 V 100 A board's at its current scaled to that board's,
its laminate's path through the housing. `air` scales the laminate's and the winding's paths to
the air (1 as built; a blocked one runs hot), `rds` the switches' on-resistance (1 as built; a
gate drive sagging runs the FETs in their SOA). Heat runs HASTE times the clock.
"""

#: The switches' on-resistance and the shunts', ohm (IAUCN10S7N021, two WSHM2818 in parallel);
#: the housekeeping's always (MCU, regulators, AFE): thermal.c's loss table; idle, a board draws
#: about 1.4 W. 2nd-order silicon temperature coefficients of RDS_OHM (thermal.c).
RDS_OHM, RDS_ALPHA, RDS_BETA, SHUNT_OHM, HOUSEKEEPING_W = 1.8e-3, 4.6e-3, 1.2e-5, 3.5e-3, 1.33


def dry_loss(volts=48.0):
    """The three legs' dry switching loss, W: 2.4 W at 24 V to 4.4 W at 44 V (0.1 W/V)."""
    return 2.4 + 0.1 * (float(volts) - 24.0)


#: Switching watts with gates on, unshorted, at nominal 48 V bus (bench fit: 2.4 W at 24 V to 4.4 W at 44 V).
SWITCHING_W = dry_loss(48.0)

#: The nodes: the switches, the laminate under them, the coaxial aluminium stator-housing, the winding.
#: The switches', laminate's, and default coaxial stator-housing's heat capacity, J/K, and paths, K/W.
NODES = ('switch', 'laminate', 'stator', 'winding')
SWITCH_J_K, LAMINATE_J_K, INTO_K_W = 0.35, 16.0, 4.0
STATOR_J_K_DEFAULT, MOUNT_K_W_DEFAULT, CLOTH_AIR_DEFAULT = 240.0, 0.60, 1.35

#: Each node's ceiling, C (the record's: the drivers' copper, the laminate, the stator, the winding),
#: and the FETs' junction limit over the switches by RTH_JC K/W a FET's watts (the sheet's).
CEILING_C, TJ_MAX_C, RTH_JC = (125.0, 105.0, 120.0, 120.0), 175.0, 0.69

#: The room, C; heat's seconds a second of the clock (the stand-in's: a 7 min board constant
#: shows in 40 s).
AMBIENT_C, HASTE = 25.0, 10.0

#: The envelope (thermal.c): the derate 1 at THROTTLE_AT of a node's span spent and 0 at its
#: ceiling, a node's hold inside LOOKAHEAD_S counted as spent; down at once, up RECOVER_PER_S a
#: heat second. A trip drops the gates and trims every ceiling to TRIP_MARGIN of its span, given
#: back TRIP_RECOVER_PER_S a heat second.
THROTTLE_AT, LOOKAHEAD_S, RECOVER_PER_S = 0.90, 2.0, 0.05
TRIP_MARGIN, TRIP_RECOVER_PER_S = 0.70, 0.30 / 1800.0

#: The status word: the gates on, the phases shorted, and the worst node's index above
#: WORST_SHIFT.
GATES_ON, SHORTED, WORST_SHIFT = 0x1, 0x2, 4


class Heat:

    """Drives' heat and envelopes, an array a quantity, a drive an index."""

    def __init__(self, drives, volts=48.0):
        import ctypes
        import numpy as np
        from tools.cores import wbc as cores
        n = len(drives)
        self.volts = float(volts)
        self.switching_w = dry_loss(volts)
        #: Each drive's torque an amp, winding ohm, board scale, winding J/K and K/W, laminate
        #: K/W (`drives.heat`), plus coaxial stator J/K, mount K/W, and clothing insulation.
        self.kt, self.r, self.scale, self.winding_j_k, self.winding_k_w, self.laminate_k_w = (
            [float(d[i]) for d in drives] for i in range(6))
        self.stator_j_k = [float(d[6]) if len(d) > 6 else STATOR_J_K_DEFAULT for d in drives]
        self.mount_k_w = [float(d[7]) if len(d) > 7 else MOUNT_K_W_DEFAULT for d in drives]
        self.cloth_air = [float(d[8]) if len(d) > 8 else CLOTH_AIR_DEFAULT for d in drives]
        self.ratio = [float(d[9]) if len(d) > 9 else 80.0 for d in drives]
        self.capacity = [(SWITCH_J_K, LAMINATE_J_K, s_c, w_c)
                         for s_c, w_c in zip(self.stator_j_k, self.winding_j_k)]
        self.t = np.full((n, len(NODES)), AMBIENT_C)
        self.sq, self.ticks = np.zeros(n), np.zeros(n, int)
        self.krpm, self.limb_rad_s = np.zeros(n), np.zeros(n)
        self.derate, self.gates, self.trips = np.ones(n), np.ones(n, bool), np.zeros(n, int)
        self.shorted = np.zeros(n, bool)
        self.spent, self.worst, self.die = np.zeros(n), np.zeros(n, int), np.full(n, AMBIENT_C)
        #: The trip cap and the heat second it was set at.
        self.cap, self.cap_at = np.ones(n), np.zeros(n)
        #: Whether the envelope derates and trips the drives, or only counts what they spend.
        self.envelope = True
        #: The step in C (wbc_pass.c, `tools.cores.wbc.Heat`): its struct pointing at the arrays
        #: above and the parameters below, laid once; each drive's hottest a step.
        self._np, self._ctypes = np, ctypes
        self._kt = np.asarray(self.kt, float)
        self._scale = np.asarray(self.scale, float)
        self._active = self._scale > 0.0
        self._ratio_k = np.asarray(self.ratio, float) * (60.0 / (2.0 * 3.141592653589793 * 1000.0))
        self._cloth_air = np.asarray(self.cloth_air, float)
        self._r = np.asarray(self.r, float)
        self._g_ls = 1.0 / np.asarray(self.mount_k_w, float)
        self._g_ws = 1.0 / np.maximum(0.15, 0.35 * np.asarray(self.winding_k_w, float))
        self._lam_k_w = np.asarray(self.laminate_k_w, float)
        self._win_k_w = np.asarray(self.winding_k_w, float)
        self._cap = np.asarray(self.capacity, float).reshape(n, len(NODES))
        self._ceil = np.asarray(CEILING_C, float)
        self._house_w = np.where(self._active, HOUSEKEEPING_W, 0.0)
        self._peak = np.zeros(n)
        h = self._h = cores.Heat(n=n, at=0.0, haste=HASTE, ambient_c=AMBIENT_C, rds_ohm=RDS_OHM, rds_alpha=RDS_ALPHA,
                                 rds_beta=RDS_BETA, shunt_ohm=SHUNT_OHM, switching_w=self.switching_w,
                                 into_k_w=INTO_K_W, rth_jc=RTH_JC, lookahead_s=LOOKAHEAD_S, throttle_at=THROTTLE_AT,
                                 recover_per_s=RECOVER_PER_S, trip_margin=TRIP_MARGIN,
                                 trip_recover_per_s=TRIP_RECOVER_PER_S, tj_max_c=TJ_MAX_C)
        for name, a in (('kt', self._kt), ('scale', self._scale), ('ratio_k', self._ratio_k),
                        ('cloth_air', self._cloth_air), ('r', self._r), ('g_ls', self._g_ls), ('g_ws', self._g_ws),
                        ('lam_k_w', self._lam_k_w), ('win_k_w', self._win_k_w), ('capacity', self._cap),
                        ('ceiling', self._ceil), ('house_w', self._house_w), ('t', self.t), ('sq', self.sq),
                        ('krpm', self.krpm), ('limb_rad_s', self.limb_rad_s), ('derate', self.derate),
                        ('spent', self.spent), ('die', self.die), ('cap', self.cap), ('cap_at', self.cap_at)):
            setattr(h, name, cores.pointer(a))
        for name, a in (('ticks', self.ticks), ('worst', self.worst), ('trips', self.trips)):
            setattr(h, name, cores.pointer(a, np.int64))
        for name, a in (('active', self._active), ('gates', self.gates), ('shorted', self.shorted)):
            setattr(h, name, cores.pointer(a, np.bool_))
        lib = cores.library()
        self._load, self._step, self._hottest = lib.wbh_pass_heat_load, lib.wbh_pass_heat, lib.wbh_pass_heat_peak

    @property
    def at(self):
        """Heat seconds since the start."""
        return self._h.at

    @at.setter
    def at(self, seconds):
        self._h.at = seconds

    @classmethod
    def for_joints(cls, joints=None, volts=48.0):
        """Construct a coaxial 4-node `Heat` observer for all joints in `figure.JOINTS`."""
        from machine.body import drives
        from machine.body.figure import JOINTS
        names = JOINTS if joints is None else joints
        specs = []
        for j in names:
            if drives.passive(j):
                specs.append((1.0, 0.0, 0.0, 100.0, 10.0, 10.0, STATOR_J_K_DEFAULT, MOUNT_K_W_DEFAULT, 1.0, 1.0))
            else:
                specs.append(drives.heat(j) + drives.stator(j) + (drives.ratio(j),))
        inst = cls(specs, volts=volts)
        for k, j in enumerate(names):
            if drives.passive(j):
                inst.off(k)
        return inst

    def load(self, k, torque, joint_rad_s=0.0):
        amps = torque / self.kt[k]
        self.sq[k] += amps * amps
        self.ticks[k] += 1
        w = abs(float(joint_rad_s))
        self.limb_rad_s[k] = w
        self.krpm[k] = w * self.ratio[k] * (60.0 / (2.0 * 3.141592653589793 * 1000.0))

    def arm(self, k):
        """The gates on again: the host's ask; the envelope trips them again if still hot."""
        self.gates[k], self.shorted[k] = True, False

    def short(self, k):
        """The phases shorted through the low sides: the host's ask; nothing with the gates
        tripped."""
        self.shorted[k] = True

    def off(self, k):
        """The gates off: the host's ask."""
        self.gates[k], self.shorted[k] = False, False

    def warm(self, k, celsius):
        """Drive k's nodes at `celsius` at least: run hard before."""
        self.t[k] = [max(t, celsius) for t in self.t[k]]

    def step(self, dt, air=None, rds=None):
        """`dt` s of the clock on: each drive's losses on its mean square since the last step,
        its coaxial 4-node network integrated, its envelope's derate and trip (wbc_pass.c)."""
        if not len(self.t):
            return
        np, ctypes = self._np, self._ctypes
        air = None if air is None else np.ascontiguousarray(air, dtype=np.float64)
        rds = None if rds is None else np.ascontiguousarray(rds, dtype=np.float64)
        self._step(ctypes.byref(self._h), float(dt), None if air is None else air.ctypes.data,
                   None if rds is None else rds.ctypes.data, int(bool(self.envelope)))

    def step_joints(self, torques, qd, dt):
        """Load and step all joints over `dt` s (real-time clock, `HASTE=1`) and return peak C per joint."""
        if not len(self.t):
            return []
        np, ctypes = self._np, self._ctypes
        torques, qd = np.ascontiguousarray(torques, dtype=np.float64), np.ascontiguousarray(qd, dtype=np.float64)
        self._load(ctypes.byref(self._h), torques.ctypes.data, qd.ctypes.data)
        self.step(float(dt) / HASTE)
        self._hottest(ctypes.byref(self._h), self._peak.ctypes.data)
        return self._peak

    def report(self, k):
        """(celsius, spent, derate, status) of drive k: its worst node's temperature, how much
        of its envelope is spent, its derate, the gates, the short and the worst node's index."""
        return (float(self.t[k][self.worst[k]]), float(self.spent[k]), float(self.derate[k]),
                (GATES_ON if self.gates[k] else 0) | (SHORTED if self.shorted[k] else 0)
                | int(self.worst[k]) << WORST_SHIFT)


def held(joint, volts=48.0):
    """(N m, node) a joint's drive holds for ever at its envelope's THROTTLE_AT: its nodes'
    steady state (no hold left to count) bisected on the torque, the node that binds there
    (NODES)."""
    from machine.body import drives
    kt, r, scale, _wj, winding_k_w, laminate_k_w = drives.heat(joint)
    _sj, mount_k_w, _ca = drives.stator(joint)
    sw_w = dry_loss(volts)
    g_ls = 1.0 / mount_k_w
    g_ws = 1.0 / max(0.15, 0.35 * winding_k_w)
    g_la, g_sa, g_wa = 0.35 / laminate_k_w, 0.65 / laminate_k_w, 1.0 / winding_k_w

    def spent(torque):
        amps = torque / kt
        board = (amps * scale) ** 2
        switch = 1.5 * board * RDS_OHM + sw_w
        p_b = switch + 1.5 * board * SHUNT_OHM + HOUSEKEEPING_W
        p_w = r * amps * amps
        a11, a12 = g_la + g_ls, -g_ls
        a22 = g_sa + g_ls + g_ws - g_ws * g_ws / (g_wa + g_ws)
        rhs2 = p_w * g_ws / (g_wa + g_ws)
        det = a11 * a22 - a12 * a12
        dt_l = (a22 * p_b - a12 * rhs2) / det
        dt_s = (-a12 * p_b + a11 * rhs2) / det
        dt_w = (p_w + g_ws * dt_s) / (g_wa + g_ws)
        laminate, stator, winding = AMBIENT_C + dt_l, AMBIENT_C + dt_s, AMBIENT_C + dt_w
        nodes = (laminate + switch * INTO_K_W, laminate, stator, winding)
        return max(((t - AMBIENT_C) / (top - AMBIENT_C), name)
                   for t, top, name in zip(nodes, CEILING_C, NODES))
    lo, hi = 0.0, drives.peak(joint) * 4.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if spent(mid)[0] < THROTTLE_AT else (lo, mid)
    return lo, spent(hi)[1]


#: Default finned aluminium heatsink block (conducting heat from power switches and shunts) and
#: temperature-controlled cooling fan parameters (`finned_aluminium_fan_sink` in `dynamics.lisp`).
DEFAULT_COOLING = {
    'sink_mass': 0.085, 'sink_capacity': 180.0,
    'r_switches_to_sink': 3.0, 'r_shunts_to_sink': 1.5,
    'r_sink_to_air_natural': 2.5, 'r_sink_to_air_forced': 0.25,
    'fan_temp_on': 45.0, 'fan_temp_full': 80.0,
    'fan_idle_duty': 0.0, 'fan_max_airflow': 14.0, 'fan_max_rpm': 8000.0,
}


def fan_duty(temp_c, cooling=None):
    """The temperature-controlled cooling fan's duty `[fan_idle_duty, 1.0]` at `temp_c` C."""
    c = cooling or DEFAULT_COOLING
    on_c, full_c, idle = float(c['fan_temp_on']), float(c['fan_temp_full']), float(c['fan_idle_duty'])
    if temp_c <= on_c:
        return idle
    if temp_c >= full_c:
        return 1.0
    return idle + (1.0 - idle) * (float(temp_c) - on_c) / max(1e-6, full_c - on_c)


def fan_airflow(temp_c, cooling=None):
    """Forced airflow speed (m/s) blown across the aluminium cooling fins by the fan at `temp_c`."""
    c = cooling or DEFAULT_COOLING
    return fan_duty(temp_c, c) * float(c['fan_max_airflow'])


def sink_r_to_air(temp_c, airspeed_m_s=0.0, cooling=None):
    """Effective thermal resistance (K/W) from the aluminium cooling fins to ambient air."""
    c = cooling or DEFAULT_COOLING
    g_nat = 1.0 / max(1e-6, float(c['r_sink_to_air_natural']))
    g_for = 1.0 / max(1e-6, float(c['r_sink_to_air_forced']))
    max_flow = max(1e-6, float(c['fan_max_airflow']))
    share = min(1.0, (fan_airflow(temp_c, c) + 0.35 * max(0.0, float(airspeed_m_s))) / max_flow)
    return 1.0 / (g_nat + share * (g_for - g_nat))


def cooling_state(ambient_c=AMBIENT_C, cooling=None):
    """Fresh thermal state for the power switches, shunts, finned aluminium block, and fan."""
    c = cooling or DEFAULT_COOLING
    amb = float(ambient_c)
    duty = fan_duty(amb, c)
    return {
        'ambient_c': amb, 'switches_c': amb, 'shunts_c': amb, 'sink_c': amb,
        'fan_duty': duty, 'fan_rpm': duty * float(c['fan_max_rpm']),
        'fan_airflow_m_s': fan_airflow(amb, c),
        'r_sink_air_k_w': sink_r_to_air(amb, 0.0, c),
        'q_switches_to_sink_w': 0.0, 'q_shunts_to_sink_w': 0.0, 'q_sink_to_air_w': 0.0,
    }


def step_cooling(state, switch_watts, shunt_watts, dt, airspeed_m_s=0.0, cooling=None):
    """Step heat conduction from power switches and shunts into the finned aluminium block and
    convection to air under the temperature-controlled cooling fan across `dt` s."""
    c = cooling or DEFAULT_COOLING
    r_sw = max(1e-6, float(c['r_switches_to_sink']))
    r_sh = max(1e-6, float(c['r_shunts_to_sink']))
    c_sink = max(1.0, float(c['sink_capacity']))
    c_sw, c_sh = 4.5, 3.6
    amb = float(state.get('ambient_c', AMBIENT_C))
    t_sw = float(state.get('switches_c', amb))
    t_sh = float(state.get('shunts_c', amb))
    t_sink = float(state.get('sink_c', amb))
    p_sw, p_sh = max(0.0, float(switch_watts)), max(0.0, float(shunt_watts))
    sub_n = max(1, int((float(dt) + 0.049999) / 0.05))
    sub_dt = max(1e-9, float(dt) / sub_n)
    a_sw, a_sh = sub_dt / (c_sw * r_sw), sub_dt / (c_sh * r_sh)
    g_sw, g_sh = 1.0 / (r_sw * (1.0 + a_sw)), 1.0 / (r_sh * (1.0 + a_sh))
    g_cap = c_sink / sub_dt
    q_sw = q_sh = q_air = 0.0
    for _ in range(sub_n):
        ctrl_t = max(t_sink, 0.5 * (t_sw + t_sink), 0.5 * (t_sh + t_sink))
        r_air = sink_r_to_air(ctrl_t, airspeed_m_s, c)
        g_air = 1.0 / r_air
        b_sw, b_sh = t_sw + sub_dt * p_sw / c_sw, t_sh + sub_dt * p_sh / c_sh
        t_sink = (g_cap * t_sink + g_sw * b_sw + g_sh * b_sh + g_air * amb) / (
            g_cap + g_sw + g_sh + g_air)
        t_sw = (b_sw + a_sw * t_sink) / (1.0 + a_sw)
        t_sh = (b_sh + a_sh * t_sink) / (1.0 + a_sh)
        q_sw, q_sh, q_air = (t_sw - t_sink) / r_sw, (t_sh - t_sink) / r_sh, (t_sink - amb) / r_air
    ctrl_t = max(t_sink, 0.5 * (t_sw + t_sink), 0.5 * (t_sh + t_sink))
    duty = fan_duty(ctrl_t, c)
    state.update(
        switches_c=t_sw, shunts_c=t_sh, sink_c=t_sink,
        fan_duty=duty, fan_rpm=duty * float(c['fan_max_rpm']),
        fan_airflow_m_s=fan_airflow(ctrl_t, c),
        r_sink_air_k_w=sink_r_to_air(ctrl_t, airspeed_m_s, c),
        q_switches_to_sink_w=q_sw, q_shunts_to_sink_w=q_sh, q_sink_to_air_w=q_air)
    return state


def step_board_cooling(state, amps, volts=48.0, dt=0.02, airspeed_m_s=0.0, cooling=None,
                       switching=True):
    """Step a 63100 board's finned aluminium heatsink and fan at `amps` A and `volts` V."""
    sq = float(amps) ** 2
    sw_w = 1.5 * sq * RDS_OHM + (dry_loss(volts) if switching else 0.0)
    sh_w = 1.5 * sq * SHUNT_OHM + HOUSEKEEPING_W
    return step_cooling(state, sw_w, sh_w, dt, airspeed_m_s, cooling)

