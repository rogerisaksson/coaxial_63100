"""The online identification, as the board runs it - `thermal_ident.c`
mirrored in Python so the stand-in identifies the same way and a page in
simulated mode shows the same states for the same reasons.

THE GRAPH HAS SIXTY NUMBERS AND THE BOARD HAS THREE THERMOMETERS. Four
scales ride on the record's network - air, capacity, spread, ntc - each a
multiplier on the derived default, and a cooldown shows the thermometers
two of them: the face's path to the air and the laminate's mass. Those
two move (`ONLINE`); spread and ntc are carried at one, since a cooldown
puts no power through the legs' edges and a scale the data cannot see
absorbs what the others leave over (FINDINGS, 2026-09-05: to a clamp).
Beside them THE ROOM, in degrees C: the board has no ambient sensor, and
a cooldown tells a cold room from a good air path - the room's
sensitivity is the same at every rise, the air path's grows with it -
where an idle board tells neither (the still rule). Estimated any other
way it ran off: FINDINGS has the two tries.

HOW: a SHADOW of the observer runs open loop from the last sample with
the sensitivity of every node to every quantity beside it, by finite
differences of the flows. When a thermometer answers, the shadow's
prediction of it against the reading is the innovation, the
sensitivities the regressor, and a Kalman step moves the quantities -
gated at three sigma, a little process noise a sample, the covariance
floored while UNCERTAIN. The shadow is then seated on the observer,
MEASUREMENT-CONSISTENT: the element set to its reading, each die's node
to what it implies and the patch under it by the package's lag algebra,
so the next innovation is the reading's change over the interval against
the model's and the state's error is not in it.

The observer's own anchor is here too (`anchor`), the same rules
`thermal.c` applies: a pull sized to the seconds since the last sample,
the dies' corrections shared with the laminate no thermometer reaches,
the thermistor compared with its modelled element and the V patch
corrected through the share and the lag, one for one when the element
is held at the patch. The room is not the anchor's to estimate.

Every number here has the same name and value as in the C, and
`test_simulated` holds the stand-in's walk UNCERTAIN → CONVERGING →
STABLE against a ground truth the way `test_thermal_core` holds the C's.
"""
import math

from . import thermal

SCALES = thermal.IDENT_SCALES
AIR, CAPACITY, SPREAD, NTC, AMBIENT = range(5)
#: How many quantities, and how many of them the record keeps.
PARAMS, RECORD = 5, 4
STATES = thermal.IDENT_STATES
UNCERTAIN, CONVERGING, STABLE = STATES

#: Which the samples move, and what each starts believed to: the air
#: path and the spread are what a situation or a layout can double, the
#: laminate's capacity was measured, the thermistor's seat comes off the
#: pick and place, and the room is ten kelvin either way - A WEIGHT: in
#: the Kalman step the prior decides who takes an innovation neither
#: quantity predicted, and a room that steps 45 K must be able to take it.
#: At three kelvin the room moved 0.6 K a gated sample and the air scale
#: took the rest, 2.9 for a truth of 0.8; at ten the room is found within
#: 5 K both ways and the air scale stays near the truth, at the price of
#: some early state error landing in the room (measured 2026-09-05).
ONLINE = (True, True, False, False, True)
PRIOR_SIGMA = (0.5, 0.2, 0.5, 0.3, 10.0)
#: The sigma floor while UNCERTAIN, as a share of the prior: half for
#: the air path and the room, which are what a situation changes, a
#: quarter for the rest - or the capacity took a third of a fan's
#: correction (measured on the stand-in, 2026-09-05: 0.65 for a truth
#: of 1.0) and its floor sat on the STABLE threshold.
FLOOR_SHARE = (0.5, 0.25, 0.25, 0.25, 0.5)
#: Below what sigma each is CONVERGING, and STABLE: three tenths of a
#: scale, and a tenth - 0.15 for the air path, since with the room
#: identified beside it one transient leaves it known to about 0.12 and a
#: machine holding a steady current after a change has one transient to
#: learn from (the rotor page never showed STABLE at a tenth, 2026-09-06);
#: three and six kelvin of room - one cycle leaves it at 2.3 from the
#: ten-kelvin prior, and three is as much as the envelope can use.
SIGMA_CONVERGING = (0.30, 0.30, 0.30, 0.30, 6.0)
SIGMA_STABLE = (0.15, 0.10, 0.10, 0.10, 3.0)
#: The process noise a sample: half a percent on a scale, 22 mK on the
#: room - a couple of kelvin an hour at thirty-second samples, which is
#: what a room does; at 0.14 K a sample the room's variance never closed
#: and the air scale's sigma floored at 0.14 through their correlation,
#: so a box was never STABLE. A room that steps is the UNCERTAIN floor's
#: business. And the covariance's ceiling - a scale never less known than
#: its own size, the room than its prior.
DRIFT_VAR = (2.5e-5, 2.5e-5, 2.5e-5, 2.5e-5, 5.0e-4)
VAR_MAX = (1.0, 1.0, 1.0, 1.0, 100.0)
SCALE_MIN, SCALE_MAX = 0.25, 4.0
AMBIENT_MIN_C, AMBIENT_MAX_C = -40.0, 85.0

#: The filter's constants - `thermal_ident.c`'s, by name.
NOISE_GAIN = 3.0            # measurement noise as a multiple of the floor
EXCITATION_MIN = 1.0e-4     # below this the sample says nothing
INNOVATION_FOLLOW = 0.2
GATE_SIGMAS = 3.0
#: A thermometer that moved less than this many floors since the seat is
#: a STILL board - nothing burning, nothing moving - and its sample moves
#: neither the quantities nor their covariance: at idle the readings
#: agree with the shadow whatever the air scale or the room, and
#: confidence from that would be confidence from silence.
STILL_GAIN = 3.0
#: The error is judged against what the thermometer did: shrunk by the
#: floor over the floor plus this share of the reading's movement since
#: the seat, so a model predicting a four-kelvin swing to a few percent
#: is predicting, and a 0.3 K miss at rest is still three floors.
MOVE_SHARE = 0.05
RATIO_STABLE, RATIO_UNCERTAIN = 2.0, 3.0

#: The floor the margin rises from when the caller gives none: the
#: record's default, `thermal.IDENT_MARGIN_FLOOR`.
MARGIN_FLOOR = thermal.IDENT_MARGIN_FLOOR
STABLE_RUNS = 5
MIN_HORIZON_S, MAX_HORIZON_S = 20.0, 600.0
BLIND_S, SETTLE_SAMPLES = 90.0, 2
EPS_T, EPS_S, EPS_AMB, SLICE_S = 0.5, 0.02, 0.5, 0.25

#: The observer's anchor - `thermal.c`'s THERMAL_ANCHOR_HZ and the
#: thermistor's two rules.
ANCHOR_HZ = 0.05
NTC_INVERT_MAX_S = 90.0
NTC_AT_LEG_K = 2.0

#: The nodes that are the board - everything but the motor - and the two
#: dies that read their own node.
BOARD_NODES = tuple(n for n in thermal.ALL_NODES if n not in thermal.MOTOR)
DIES = ('mcu', 'afe')
LEG_PATCHES = ('patch_u', 'patch_v', 'patch_w')


def apply(scale, base):
    """`base` with the scales on it: the air path and the capacity of
    every laminate node, the sources' ten edges, the thermistor's share.
    The room is not the network's and is not here. A shallow copy that
    replaces only what it changes."""
    cfg = dict(base)
    air, cap, spread, ntc = scale[:RECORD]
    cfg['board_to_ambient'] = base['board_to_ambient'] * air
    to_ambient, capacity = dict(base['to_ambient']), dict(base['capacity'])
    for node in thermal.LAMINATE:
        if base['area_share'].get(node, 0.0) > 0.0:
            to_ambient[node] = base['to_ambient'][node] * air
            capacity[node] = base['capacity'][node] * cap
    cfg['to_ambient'], cfg['capacity'] = to_ambient, capacity
    edges = list(base['edges'])
    for e in range(10):
        edges[e] = base['edges'][e] * spread
    cfg['edges'] = edges
    cfg['ntc_sees'] = base.get('ntc_sees', thermal.NTC_SEES_DRIVERS) * ntc
    cfg['ntc_tau_s'] = base.get('ntc_tau_s', thermal.NTC_TAU_S)
    return cfg


def rates(temps, cfg, power, speed_rpm, ambient=thermal.AMBIENT):
    """K/s per node: the net flows over the capacities."""
    net = thermal.net_flows(temps, power, cfg, ambient, speed_rpm)
    out = {}
    for node in thermal.ALL_NODES:
        cap = cfg['capacity'].get(node, 0.0)
        out[node] = net[node] / cap if cap > 0.0 else 0.0
    return out


def ntc_follow(temps, ntc, cfg, dt):
    """One step of the thermistor's element - `thermal_ntc_follow`:
    first order toward the weighted average of its two patches, and
    never past either. Returns `(ntc, held)`, `held` the node it is
    pinned at or None."""
    f = min(1.0, max(0.0, cfg.get('ntc_sees', thermal.NTC_SEES_DRIVERS)))
    tau = cfg.get('ntc_tau_s', thermal.NTC_TAU_S)
    centre, leg = temps['board'], temps[thermal.NTC_PATCH]
    target = centre + f * (leg - centre)
    if tau > 0.0 and dt > 0.0:
        ntc += (target - ntc) * min(1.0, dt / tau)
    elif tau <= 0.0:
        ntc = target
    low_at, high_at = (('board', thermal.NTC_PATCH) if centre < leg
                       else (thermal.NTC_PATCH, 'board'))
    if ntc < temps[low_at]:
        return temps[low_at], low_at
    if ntc > temps[high_at]:
        return temps[high_at], high_at
    return ntc, None


def anchor(temps, ntc, cfg, power, seen, speed_rpm, since_s,
           ambient=thermal.AMBIENT):
    """Fold the thermometers into the observer - `thermal.c`'s `anchor`.

    `seen` is `{'ntc', 'afe', 'mcu'}`, None where nothing answered;
    `since_s` the seconds since the last sample, which sizes the pull;
    `ambient` the room the observer runs against - the identification's,
    not this function's to move. `temps` is corrected in place; returns
    `(ntc, settled)`.
    """
    k = 1.0 - math.exp(-ANCHOR_HZ * since_s)
    net = thermal.net_flows(temps, power, cfg, ambient, speed_rpm)
    held, common, dies = set(), 0.0, 0
    for node in DIES:
        reading = seen.get(node)
        if reading is None:
            continue
        watt = power.get(node, 0.0)
        at = reading - watt * cfg['rth_die'].get(node, 0.0)
        temps[node] += k * (at - temps[node])
        held.add(node)
        edge = thermal.sink_edge(node)
        if edge is not None:
            # THE NODE LAGS ITS PATCH: patch = node - P R + C R dT/dt.
            patch, r = thermal.EDGES[edge][1], cfg['edges'][edge]
            cap = cfg['capacity'].get(node, 0.0)
            rate = net[node] / cap if cap > 0.0 else 0.0
            implied = at - watt * r + cap * r * rate
            common += implied - temps[patch]
            temps[patch] += k * (implied - temps[patch])
            held.add(patch)
        dies += 1
    if dies:
        # THE REST OF THE LAMINATE MOVES WITH THE DIES.
        common /= dies
        for node in BOARD_NODES:
            if node not in held and cfg['capacity'].get(node, 0.0) > 0.0:
                temps[node] += k * common
        reading = seen.get('ntc')
        f = cfg.get('ntc_sees', thermal.NTC_SEES_DRIVERS)
        if reading is not None and f > 0.01:
            # THE THERMISTOR AGAINST THE ELEMENT AS MODELLED; the miss
            # through the share and the lag, one for one at the leg.
            miss = reading - ntc
            leg, centre = temps[thermal.NTC_PATCH], temps['board']
            at_leg = leg >= centre and (reading >= leg - NTC_AT_LEG_K
                                        or ntc >= leg - NTC_AT_LEG_K)
            fresh = since_s <= NTC_INVERT_MAX_S
            tau, lag_gain = cfg.get('ntc_tau_s', thermal.NTC_TAU_S), 1.0
            if tau > 0.0 and since_s > 0.0:
                lag_gain = min(8.0, max(1.0, 0.5 * tau / since_s))
            through = 1.0 if at_leg else ((lag_gain / f) if fresh else 0.0)
            move = k * miss * through
            ntc += k * miss
            for patch in LEG_PATCHES:          # one layout, mirrored
                if patch not in held:
                    temps[patch] += move
        return ntc, True
    reading = seen.get('ntc')
    if reading is not None:
        # Degraded: no die, the whole laminate on the thermistor's miss.
        miss = reading - ntc
        ntc += k * miss
        for node in BOARD_NODES:
            if cfg['capacity'].get(node, 0.0) > 0.0:
                temps[node] += k * miss
    return ntc, False


def _unit(x):
    return 0.0 if x < 0.0 else (1.0 if x > 1.0 else x)


def doubt_terms(innovation_k, sigma, noise_k=0.1):
    """Each normalised term of the doubt by name - `thermal_ident_doubt`
    taken apart, so a page can say which one holds the margin down:
    'innovation' from the floor to the UNCERTAIN ratio, and each online
    quantity's sigma from its STABLE threshold to its prior, 'air',
    'capacity' and 'room'. `sigma` is the five sigmas in wire order."""
    terms = {'innovation': _unit((innovation_k / noise_k - 1.0)
                                 / (RATIO_UNCERTAIN - 1.0))}
    for k in range(PARAMS):
        if ONLINE[k]:
            span = PRIOR_SIGMA[k] - SIGMA_STABLE[k]
            name = SCALES[k] if k < RECORD else 'room'
            terms[name] = _unit((sigma[k] - SIGMA_STABLE[k]) / span)
    return terms


def _clamped(scale):
    out = [min(SCALE_MAX, max(SCALE_MIN, s)) for s in scale[:RECORD]]
    out.append(min(AMBIENT_MAX_C, max(AMBIENT_MIN_C, scale[AMBIENT])))
    return out


class Identifier:

    """The quantities, their covariance, the shadow and its sensitivities."""

    def __init__(self, noise_k=0.1, ambient=thermal.AMBIENT):
        self.scale = _clamped([1.0] * RECORD + [float(ambient)])
        self.p = [[0.0] * PARAMS for _ in range(PARAMS)]
        self._set_covariance(0.5)
        self.noise_k = noise_k if noise_k > 0.0 else 0.1
        self.innovation_k = self.noise_k
        self.state = UNCERTAIN
        self.updates = 0
        self.stable_runs = 0
        self.primed = False
        self.shadow_t = None
        self.shadow_ntc = None
        self.shadow_cfg = None
        self.shadow_ambient = self.scale[AMBIENT]
        self.s = [dict.fromkeys(thermal.ALL_NODES, 0.0) for _ in range(PARAMS)]
        self.s_ntc = [0.0] * PARAMS
        self.horizon_s = 0.0
        self.since_sample_s = 0.0
        self.seated = set()
        self.seat_reading = {}
        self.settle_left = 0
        self._pending = 0.0

    def _set_covariance(self, sigma):
        self.p = [[0.0] * PARAMS for _ in range(PARAMS)]
        for k in range(PARAMS):
            s = min(sigma, PRIOR_SIGMA[k]) if k != AMBIENT else PRIOR_SIGMA[k]
            self.p[k][k] = s * s

    # -- what it is worth ---------------------------------------------

    @property
    def ambient(self):
        """The room as identified, degrees C: what the observer runs
        against."""
        return self.scale[AMBIENT]

    def sigma(self, k):
        var = self.p[k][k]
        return math.sqrt(var) if var > 0.0 else 0.0

    def doubt(self):
        """How far the model is doubted, 0..1 - `thermal_ident_doubt`:
        the worse of the innovation normalised from the thermometers'
        floor to the ratio that says UNCERTAIN, and each online quantity's
        sigma from where STABLE calls it known to its prior. A fresh
        board is doubted whole and an idle one stays so; a cooldown is
        what lowers it."""
        return max(doubt_terms(self.innovation_k,
                               [self.sigma(k) for k in range(PARAMS)],
                               self.noise_k).values())

    def margin(self, floor=MARGIN_FLOOR):
        """`thermal_ident_margin`: the floor while the model is doubted
        whole, one when not at all, the doubt between - continuous since
        2026-09-06; it was three steps on the state."""
        f = _unit(floor)
        return f + (1.0 - f) * (1.0 - self.doubt())

    def apply(self, base):
        return apply(self.scale, base)

    @staticmethod
    def online(k):
        return ONLINE[k]

    def _known(self, threshold):
        return all(self.sigma(k) < threshold[k] for k in range(PARAMS)
                   if ONLINE[k])

    # -- the shadow ---------------------------------------------------

    def _reseat(self, temps, ntc, base, power, speed_rpm, seen):
        self.shadow_t = dict(temps)
        self.shadow_ntc = ntc
        self.shadow_ambient = self.scale[AMBIENT]
        self.shadow_cfg = apply(self.scale, base)
        f0 = rates(self.shadow_t, self.shadow_cfg, power, speed_rpm,
                   self.shadow_ambient)
        self.s = [dict.fromkeys(thermal.ALL_NODES, 0.0) for _ in range(PARAMS)]
        self.s_ntc = [0.0] * PARAMS
        self.horizon_s = 0.0
        self.seated = set()
        self.seat_reading = {}
        if not seen:
            return
        if seen.get('ntc') is not None:
            self.shadow_ntc = seen['ntc']
            self.seated.add('ntc')
            self.seat_reading['ntc'] = seen['ntc']
        for node in DIES:
            reading = seen.get(node)
            if reading is None:
                continue
            watt = power.get(node, 0.0)
            at = reading - watt * self.shadow_cfg['rth_die'].get(node, 0.0)
            self.shadow_t[node] = at
            edge = thermal.sink_edge(node)
            if edge is not None:
                patch = thermal.EDGES[edge][1]
                cap = self.shadow_cfg['capacity'].get(node, 0.0)
                per_r = -watt + cap * f0[node]
                self.shadow_t[patch] = at + per_r * self.shadow_cfg['edges'][edge]
                # The seat's own dependence on the spread, in the regressor.
                self.s[SPREAD][patch] = per_r * base['edges'][edge]
            self.seated.add(node)
            self.seat_reading[node] = reading

    def _propagate(self, base, power, speed_rpm, dt):
        t, cfg, ambient = self.shadow_t, self.shadow_cfg, self.shadow_ambient
        f0 = rates(t, cfg, power, speed_rpm, ambient)
        for k in range(PARAMS):
            sk = self.s[k]
            largest = max(abs(v) for v in sk.values())
            eps = EPS_T / largest if largest > 1.0e-6 else 1.0
            probe = dict((n, t[n] + eps * sk[n]) for n in thermal.ALL_NODES)
            f1 = rates(probe, cfg, power, speed_rpm, ambient)
            ds = dict((n, (f1[n] - f0[n]) / eps) for n in thermal.ALL_NODES)
            # df/dq_k: the room nudged half a kelvin, a scale by a fraction.
            if k == AMBIENT:
                f1 = rates(t, cfg, power, speed_rpm, ambient + EPS_AMB)
                over = EPS_AMB
            else:
                nudged = list(self.scale)
                nudged[k] *= 1.0 + EPS_S
                f1 = rates(t, apply(nudged, base), power, speed_rpm, ambient)
                over = EPS_S * self.scale[k]
            for n in thermal.ALL_NODES:
                sk[n] += (ds[n] + (f1[n] - f0[n]) / over) * dt
        for n in thermal.ALL_NODES:
            t[n] += f0[n] * dt
        self.shadow_ntc, held = ntc_follow(t, self.shadow_ntc, cfg, dt)
        tau = cfg.get('ntc_tau_s', thermal.NTC_TAU_S)
        share = min(1.0, dt / tau) if tau > 0.0 else 1.0
        f = min(1.0, max(0.0, cfg.get('ntc_sees', thermal.NTC_SEES_DRIVERS)))
        for k in range(PARAMS):
            if held is not None:
                self.s_ntc[k] = self.s[k][held]
                continue
            target = ((1.0 - f) * self.s[k]['board']
                      + f * self.s[k][thermal.NTC_PATCH])
            if k == NTC and self.scale[k] > 0.0:
                target += (f / self.scale[k]) * (t[thermal.NTC_PATCH]
                                                 - t['board'])
            self.s_ntc[k] += (target - self.s_ntc[k]) * share

    # -- the filter ---------------------------------------------------

    def _update(self, h_all, innovation):
        h = [h_all[k] if ONLINE[k] else 0.0 for k in range(PARAMS)]
        if sum(v * v for v in h) < EXCITATION_MIN:
            return False
        r = NOISE_GAIN * self.noise_k
        ph = [sum(self.p[k][j] * h[j] for j in range(PARAMS))
              for k in range(PARAMS)]
        hph = sum(h[k] * ph[k] for k in range(PARAMS))
        denom = hph + r * r
        if innovation * innovation > GATE_SIGMAS * GATE_SIGMAS * denom:
            denom = innovation * innovation / (GATE_SIGMAS * GATE_SIGMAS)
        for k in range(PARAMS):
            self.scale[k] += ph[k] / denom * innovation
        for k in range(PARAMS):
            for j in range(PARAMS):
                self.p[k][j] -= ph[k] * ph[j] / denom
            self.p[k][k] = min(VAR_MAX[k], self.p[k][k])
        self.scale = _clamped(self.scale)
        return True

    def _room_reset(self):
        """`room_reset`: on the way to UNCERTAIN the room's variance goes
        back to its whole prior and its correlation with every scale is
        cut, so a room step is charged to the room first."""
        for k in range(PARAMS):
            self.p[AMBIENT][k] = 0.0
            self.p[k][AMBIENT] = 0.0
        self.p[AMBIENT][AMBIENT] = PRIOR_SIGMA[AMBIENT] ** 2

    def _judge(self):
        ratio = self.innovation_k / self.noise_k
        if self.state == STABLE:
            if ratio > RATIO_UNCERTAIN:
                self.state, self.stable_runs = UNCERTAIN, 0
                for k in range(PARAMS):
                    self.p[k][k] += (FLOOR_SHARE[k] * PRIOR_SIGMA[k]) ** 2
                self._room_reset()
        elif self.state == CONVERGING:
            if ratio > RATIO_UNCERTAIN:
                self.state, self.stable_runs = UNCERTAIN, 0
                self._room_reset()
            elif self._known(SIGMA_STABLE) and ratio < RATIO_STABLE:
                self.stable_runs += 1
                if self.stable_runs >= STABLE_RUNS:
                    self.state = STABLE
            else:
                self.stable_runs = 0
        else:
            if self._known(SIGMA_CONVERGING) and ratio < RATIO_UNCERTAIN:
                self.state, self.stable_runs = CONVERGING, 0
            elif ratio >= RATIO_UNCERTAIN:
                # NOT PREDICTING, SO NOT SURE: the online quantities stay free.
                for k in range(PARAMS):
                    floor = (FLOOR_SHARE[k] * PRIOR_SIGMA[k]) ** 2
                    if ONLINE[k] and self.p[k][k] < floor:
                        self.p[k][k] = floor

    # -- one step beside the observer ---------------------------------

    def step(self, temps, ntc, base, power, speed_rpm, seen, dt):
        """The observer's state after its own step, the record's base,
        this slice's power and speed, the thermometers (`{'ntc', 'afe',
        'mcu'}` or None) and the slice. True when a sample moved the
        quantities: the caller re-applies the scales to the observer's
        network, and the room to its `ambient` after every step."""
        seen = seen or {}
        if not self.primed:
            self._reseat(temps, ntc, base, power, speed_rpm, seen)
            self.primed = True
            return False
        any_seen = any(seen.get(k) is not None for k in ('ntc', 'afe', 'mcu'))
        self._pending += dt
        self.horizon_s += dt
        self.since_sample_s += dt
        if self._pending >= SLICE_S or any_seen:
            left = self._pending
            while left > 1.0e-9:
                slice_s = min(SLICE_S, left)
                self._propagate(base, power, speed_rpm, slice_s)
                left -= slice_s
            self._pending = 0.0
        blind = any_seen and self.since_sample_s > BLIND_S
        if any_seen:
            self.since_sample_s = 0.0
        if blind:
            self._reseat(temps, ntc, base, power, speed_rpm, None)
            self.settle_left = SETTLE_SAMPLES
            return False
        if any_seen and self.settle_left > 0 and self.horizon_s >= MIN_HORIZON_S:
            self.settle_left -= 1
            self._reseat(temps, ntc, base, power, speed_rpm, seen)
            return False
        if any_seen and self.horizon_s >= MIN_HORIZON_S:
            # A STILL BOARD TEACHES NOTHING: every seated thermometer
            # within STILL_GAIN floors of what it read at the seat is a
            # board with nothing burning, and the sample moves neither
            # the quantities nor their covariance - though its prediction
            # error still says whether the model predicts.
            stirred = max([abs(seen[name] - self.seat_reading[name])
                           for name in self.seated
                           if seen.get(name) is not None] or [0.0])
            still = stirred < STILL_GAIN * self.noise_k
            moved, judged, worst = False, False, 0.0
            channels = [('ntc', self.shadow_ntc, list(self.s_ntc))]
            for node in DIES:
                over = power.get(node, 0.0) * self.shadow_cfg['rth_die'].get(node, 0.0)
                channels.append((node, self.shadow_t[node] + over,
                                 [self.s[k][node] for k in range(PARAMS)]))
            for name, predicted, h in channels:
                reading = seen.get(name)
                if reading is None or name not in self.seated:
                    continue
                e = reading - predicted
                if not still:
                    moved = self._update(h, e) or moved
                allowed = self.noise_k + MOVE_SHARE * abs(
                    reading - self.seat_reading[name])
                worst = max(worst, abs(e) * self.noise_k / allowed)
                judged = True
            if judged:
                self.innovation_k += INNOVATION_FOLLOW * (worst - self.innovation_k)
                if not still:
                    for k in range(PARAMS):
                        if ONLINE[k]:
                            self.p[k][k] += DRIFT_VAR[k]
                self._judge()
            if moved:
                self.updates += 1
            self._reseat(temps, ntc, base, power, speed_rpm, seen)
            return moved
        if self.horizon_s >= MAX_HORIZON_S:
            self._reseat(temps, ntc, base, power, speed_rpm, None)
        return False
