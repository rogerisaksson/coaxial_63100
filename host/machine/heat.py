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

A drive is a board behind its outrunner and a cycloid (`machine.drives`): its torque an amp,
its winding, its board's losses as the 63 V 100 A board's at its current scaled to that board's,
its laminate's path through the housing. `air` scales the laminate's and the winding's paths to
the air (1 as built; a blocked one runs hot), `rds` the switches' on-resistance (1 as built; a
gate drive sagging runs the FETs in their SOA). Heat runs HASTE times the clock.
"""

#: The switches' on-resistance and the shunts', ohm (IAUCN10S7N021, two WSHM2818 in parallel);
#: the housekeeping's always (MCU, regulators, AFE): thermal.c's loss table; idle, a board draws
#: about 1.4 W.
RDS_OHM, SHUNT_OHM, HOUSEKEEPING_W = 1.8e-3, 3.5e-3, 1.33


def dry_loss(volts=48.0):
    """The three legs' dry switching loss, W: 2.4 W at 24 V to 4.4 W at 44 V (0.1 W/V)."""
    return 2.4 + 0.1 * (float(volts) - 24.0)


#: Switching watts with gates on, unshorted, at nominal 48 V bus (bench fit: 2.4 W at 24 V to 4.4 W at 44 V).
SWITCHING_W = dry_loss(48.0)

#: The nodes: the switches, the laminate under them, the winding. The switches' and the
#: laminate's heat capacity, J/K, and the switches' path into the laminate, K/W: lumped from the
#: board's network (a leg's driver 0.117 J/K on 12 K/W into its patch; the three patches 16 J/K);
#: bare in still air the laminate stands 35 K over the room at 1 W a leg, 11.7 K/W. The winding's
#: and the laminate's path on to the air are the drive's (`drives.heat`).
NODES = ('switch', 'laminate', 'winding')
SWITCH_J_K, LAMINATE_J_K, INTO_K_W = 0.35, 16.0, 4.0

#: Each node's ceiling, C (the record's: the drivers' copper, the laminate, the winding), and
#: the FETs' junction limit over the switches by RTH_JC K/W a FET's watts (the sheet's).
CEILING_C, TJ_MAX_C, RTH_JC = (125.0, 105.0, 120.0), 175.0, 0.69

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

    """Drives' heat and envelopes, a list a quantity, a drive an index."""

    def __init__(self, drives, volts=48.0):
        n = len(drives)
        self.volts = float(volts)
        self.switching_w = dry_loss(volts)
        #: Each drive's torque an amp, winding ohm, board scale, winding J/K and K/W, laminate
        #: K/W (`drives.heat`); the nodes' capacities.
        self.kt, self.r, self.scale, self.winding_j_k, self.winding_k_w, self.laminate_k_w = (
            [float(d[i]) for d in drives] for i in range(6))
        self.capacity = [(SWITCH_J_K, LAMINATE_J_K, c) for c in self.winding_j_k]
        self.t = [[AMBIENT_C] * len(NODES) for _ in range(n)]
        self.sq, self.ticks = [0.0] * n, [0] * n
        self.derate, self.gates, self.trips = [1.0] * n, [True] * n, [0] * n
        self.shorted = [False] * n
        self.spent, self.worst, self.die = [0.0] * n, [0] * n, [AMBIENT_C] * n
        #: The trip cap and the heat second it was set at.
        self.cap, self.cap_at, self.at = [1.0] * n, [0.0] * n, 0.0
        #: Whether the envelope derates and trips the drives, or only counts what they spend.
        self.envelope = True

    def load(self, k, torque):
        amps = torque / self.kt[k]
        self.sq[k] += amps * amps
        self.ticks[k] += 1

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

    def step(self, dt, air, rds):
        """`dt` s of the clock on: each drive's losses on its mean square since the last step,
        its nodes integrated, its envelope's derate and trip."""
        h = dt * HASTE
        self.at += h
        for k, t in enumerate(self.t):
            sq = self.sq[k] / self.ticks[k] if self.ticks[k] else 0.0
            self.sq[k], self.ticks[k] = 0.0, 0
            board = sq * self.scale[k] ** 2
            fet = 0.5 * board * RDS_OHM * rds[k]
            power = (3.0 * fet + (self.switching_w if self.gates[k] and not self.shorted[k] else 0.0),
                     1.5 * board * SHUNT_OHM + HOUSEKEEPING_W, self.r[k] * sq)
            into = (t[0] - t[1]) / INTO_K_W
            net = (power[0] - into,
                   power[1] + into - (t[1] - AMBIENT_C) * air[k] / self.laminate_k_w[k],
                   power[2] - (t[2] - AMBIENT_C) * air[k] / self.winding_k_w[k])
            for i, c in enumerate(self.capacity[k]):
                t[i] += h * net[i] / c
            self.die[k] = t[0] + fet * RTH_JC
            self._envelope(k, t, net, h)

    def _envelope(self, k, t, net, h):
        cap = min(1.0, self.cap[k] + (self.at - self.cap_at[k]) * TRIP_RECOVER_PER_S)
        spent, worst = 0.0, 0
        for i, top in enumerate(CEILING_C):
            limit = AMBIENT_C + cap * (top - AMBIENT_C)
            used = max(0.0, min(1.0, (t[i] - AMBIENT_C) / (limit - AMBIENT_C)))
            if net[i] > 0.0:
                hold = max(0.0, (limit - t[i]) * self.capacity[k][i] / net[i])
                used = max(used, min(1.0, 1.0 - hold / LOOKAHEAD_S))
            if used > spent:
                spent, worst = used, i
        self.spent[k], self.worst[k] = spent, worst
        if not self.envelope:
            return
        want = 1.0 if spent <= THROTTLE_AT else max(0.0, (1.0 - spent) / (1.0 - THROTTLE_AT))
        self.derate[k] = min(want, self.derate[k] + RECOVER_PER_S * h)
        if self.gates[k] and (self.die[k] >= TJ_MAX_C
                              or any(x >= top for x, top in zip(t, CEILING_C))):
            self.gates[k] = False
            self.trips[k] += 1
            self.cap[k], self.cap_at[k] = TRIP_MARGIN, self.at

    def report(self, k):
        """(celsius, spent, derate, status) of drive k: its worst node's temperature, how much
        of its envelope is spent, its derate, the gates, the short and the worst node's index."""
        return (self.t[k][self.worst[k]], self.spent[k], self.derate[k],
                (GATES_ON if self.gates[k] else 0) | (SHORTED if self.shorted[k] else 0)
                | self.worst[k] << WORST_SHIFT)


def held(joint, volts=48.0):
    """(N m, node) a joint's drive holds for ever at its envelope's THROTTLE_AT: its nodes'
    steady state (no hold left to count) bisected on the torque, the node that binds there
    (NODES)."""
    from machine import drives
    kt, r, scale, _wj, winding_k_w, laminate_k_w = drives.heat(joint)
    sw_w = dry_loss(volts)

    def spent(torque):
        amps = torque / kt
        board = (amps * scale) ** 2
        switch = 1.5 * board * RDS_OHM + sw_w
        laminate = AMBIENT_C + (switch + 1.5 * board * SHUNT_OHM + HOUSEKEEPING_W) * laminate_k_w
        nodes = (laminate + switch * INTO_K_W, laminate, AMBIENT_C + r * amps * amps * winding_k_w)
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

