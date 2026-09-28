"""A drive's heat as its board keeps it: three nodes and the envelope on them, thermal.c's law.

The switches, the laminate under them and the winding, each a node lumped; the envelope's derate
and trip on them.

    heat = Heat([drives.heat(j), ..])   # the drives at AMBIENT_C, their gates on
    heat.load(k, torque)                # a tick's torque, N m, into drive k's mean square
    heat.step(dt, air, rds)             # dt s on: the nodes on their losses, the envelope
    heat.derate[k], heat.gates[k]       # the share of its clamp drive k gives, if anything
    heat.arm(k); heat.warm(k, celsius)  # the gates on again; its nodes warmed to at least
    heat.report(k)                      # (celsius, spent, derate, status): its reply's
    heat.envelope = False               # a fantasy board: its envelope counted, never binding

A drive is a board behind its outrunner and a cycloid (`machine.drives`): its torque an amp,
its winding, its board's losses as the 63 V 100 A board's at its current scaled to that board's,
its laminate's path through the housing. `air` scales the laminate's and the winding's paths to
the air (1 as built; a blocked one runs hot), `rds` the switches' on-resistance (1 as built; a
gate drive sagging runs the FETs in their SOA). Heat runs HASTE times the clock.
"""

#: The switches' on-resistance and the shunts', ohm (IAUCN10S7N021, two WSHM2818 in parallel);
#: the switching's watts with the gates on, and the housekeeping's always (MCU, regulators,
#: AFE): thermal.c's loss table.
RDS_OHM, SHUNT_OHM, SWITCHING_W, HOUSEKEEPING_W = 1.8e-3, 3.5e-3, 1.2, 1.33

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

#: The status word: the gates on, and the worst node's index above WORST_SHIFT.
GATES_ON, WORST_SHIFT = 0x1, 4


class Heat:

    """Drives' heat and envelopes, a list a quantity, a drive an index."""

    def __init__(self, drives):
        n = len(drives)
        #: Each drive's torque an amp, winding ohm, board scale, winding J/K and K/W, laminate
        #: K/W (`drives.heat`); the nodes' capacities.
        self.kt, self.r, self.scale, self.winding_j_k, self.winding_k_w, self.laminate_k_w = (
            [float(d[i]) for d in drives] for i in range(6))
        self.capacity = [(SWITCH_J_K, LAMINATE_J_K, c) for c in self.winding_j_k]
        self.t = [[AMBIENT_C] * len(NODES) for _ in range(n)]
        self.sq, self.ticks = [0.0] * n, [0] * n
        self.derate, self.gates, self.trips = [1.0] * n, [True] * n, [0] * n
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
        self.gates[k] = True

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
            power = (3.0 * fet + (SWITCHING_W if self.gates[k] else 0.0),
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
        of its envelope is spent, its derate, the gates and the worst node's index."""
        return (self.t[k][self.worst[k]], self.spent[k], self.derate[k],
                (GATES_ON if self.gates[k] else 0) | self.worst[k] << WORST_SHIFT)
