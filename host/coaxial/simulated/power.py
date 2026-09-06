"""The power stage stood down: thermal observer, power rails and the
gate drivers with the real arming policy."""
import copy
import math
import random
import time

from .. import motor
from .. import thermal
from .. import thermal_ident
from ..thermal_device import THROTTLE_AT
from ..errors import RigError
from ..gates import GateControl


class SimulatedThermal:
    """The thermal observer without a board: the same twenty-node graph
    `thermal.c` integrates, on `coaxial.thermal`'s tables, so a view
    running -Simulated draws the network the board runs and the envelope
    rehearses the same play.

    Keeps the node order and the field shape of `0x6E` device 8 so a view
    running -Simulated does not crash. Every number is invented, and
    `ntc: None` mirrors the real behaviour: with AFE_ON low there is no
    measurement at all.
    """

    NODES = thermal.ALL_NODES

    #: The ceiling each node is judged against - the record's defaults:
    #: laminate lower than the rest because it is what everything else sits
    #: on, the motor's three at the winding's.
    LIMIT = dict(thermal.CEILING_C)
    DEFAULT_LIMIT = thermal.CEILING_DEFAULT_C

    #: Which nodes the current clamp cannot cool - `soa_undriven_mask` in
    #: the calibration record, and the same three for the same reason.
    #:
    #: A THROTTLE NEEDS AN ACTUATOR. The clamp scales the phase current,
    #: so it moves the legs, the hot swap and the motor, and nothing at all
    #: on the MCU, the regulators or the front end: those draw the same
    #: watts at zero duty as at full. Weighed into the worst node they put
    #: a floor under the margin that no derating can lift - measured here
    #: 2026-09-04, an idle board settles at 49.1 C on the MCU and 51.1 C
    #: on the regulators, 0.30 of the budget gone on a cold bench.
    UNDRIVEN = ('mcu', 'regulators', 'afe')

    #: How much longer than wall time this stand-in heats. The board's
    #: own constant is about seven minutes, and a view nobody watches for
    #: seven minutes shows a flat line: at ten, a load step is visible in
    #: half a minute. It is the CLOCK that is sped up and nothing else -
    #: the network, the capacities and the ceilings are the real ones out
    #: of `coaxial.thermal`, so what a step settles at is right even
    #: though it gets there sooner.
    HASTE = 10.0

    #: How fast the tracked rms follows the samples, seconds. Long enough
    #: that a single sample cannot move it - one instant of a rotating
    #: three-phase current is a vector, not an amplitude - and short
    #: enough that a load step is in the temperatures within a second.
    RMS_TAU = 0.5

    #: The window of remaining hold the throttle insists on, seconds.
    #: `soa_lookahead_ms` in the board's record, and the same two seconds
    #: it defaults to.
    LOOKAHEAD_S = 2.0

    #: How fast the derate may recover, per second of model time.
    #: `THERMAL_DERATE_RECOVER_PER_S` in the firmware.
    DERATE_RECOVER_PER_S = 0.05

    #: The longest slice the integrator takes, seconds of model time.
    #: `THERMAL_STEP_MS` in the firmware, and the same number for the same
    #: reason: the envelope is evaluated once per step, and a step longer
    #: than the throttle's ramp steps straight over it. A fifth of the
    #: stiffest node's constant, too - a leg's silicon at 1.4 s - so the
    #: explicit step is an integration and not an oscillation.
    STEP_S = 0.1

    #: THE WINDING'S ENVELOPE, the record's CAL_VERSION 12 defaults: the
    #: motor profile's placeholder pair and an estimated ceiling, over the
    #: stand-in record's phase resistance. Split into the graph as the
    #: firmware splits it: a quarter of the K/W from the copper into the
    #: iron, the rest the iron's air path.
    WINDING_R = motor.BENCH_MOTOR.r
    WINDING_K_PER_W = motor.WINDING_K_PER_W
    WINDING_J_PER_K = motor.WINDING_J_PER_K
    WINDING_LIMIT_C = 120.0

    #: The hot swap's two pass FETs in series - the bridge's own part,
    #: `thermal_losses` says 3.6 mOhm - and the link current they see is
    #: the phases' at the duty, estimated as half the rms here since the
    #: stand-in's sampler has no duty to weigh by.
    HOTSWAP_R = 3.6e-3

    #: THE GROUND TRUTH'S SITUATIONS: what a box, a fan or a heat sink
    #: does to the board the stand-in pretends to be, as scales on its
    #: air path and its laminate. The bench: "a simulated ground truth,
    #: to see how well the estimator identifies online, and a way to lay
    #: another thermal situation over it" - and, in simulated mode, "a
    #: switch at random, moderate intervals so ROTOR OBSERVER and THERMAL
    #: OBSERVER show the logic in action".
    #: Each situation: a scale on the air path, one on the laminate's
    #: capacity, and the ROOM - `outdoors` is the whole assembly, motor
    #: and electronics, carried to -20 C in a light wind, the bench's
    #: "from 25 C indoors to -20 C outdoors and back in again"; the
    #: observer is not told the room and reads its way there from its
    #: own losses, as the board does.
    SITUATIONS = {'bench': {'air': 1.0, 'capacity': 1.0, 'ambient': 25.0},
                  'box': {'air': 2.0, 'capacity': 1.0, 'ambient': 25.0},
                  'fan': {'air': 0.5, 'capacity': 1.0, 'ambient': 25.0},
                  'heatsink': {'air': 0.35, 'capacity': 1.6, 'ambient': 25.0},
                  'stuffy': {'air': 1.5, 'capacity': 1.0, 'ambient': 25.0},
                  'outdoors': {'air': 0.8, 'capacity': 1.0, 'ambient': -20.0},
                  # The bench's robot: a warehouse, a freezer room, and
                  # out into the Thai midsummer.
                  'warehouse': {'air': 1.0, 'capacity': 1.0, 'ambient': 20.0},
                  'freezer': {'air': 0.9, 'capacity': 1.0, 'ambient': -25.0},
                  'thai': {'air': 1.2, 'capacity': 1.0, 'ambient': 45.0}}
    #: Wall seconds between switches when they are on: long enough for
    #: STABLE to be reached between them at HASTE, short enough to watch.
    SWITCH_EVERY_S = (180.0, 360.0)

    #: The thermometers' own noise, ±kelvin, and their floor as the
    #: identifier is told it - the board's 30 mK NTC and 125 mK dies.
    NOISE_K = 0.05
    IDENT_NOISE_K = 0.1

    #: The floor the envelope's margin rises from, the record's default
    #: (`thermal.IDENT_MARGIN_FLOOR`); `set_margin_floor` moves it as
    #: thermal op 12 does on the board.
    MARGIN_FLOOR = thermal.IDENT_MARGIN_FLOOR

    #: THE LOAD CYCLE a page in simulated mode lays on, model seconds:
    #: six minutes at 30 A and fourteen cooling - the walk the suites
    #: take the identification through - so the board's regions warm and
    #: cool on the map while the state earns its span. Two minutes of
    #: wall time a cycle at HASTE. The bench, 2026-09-06: "make the
    #: THERMAL OBSERVER page show the board's temperatures from a
    #: simulated load cycle".
    CYCLE_AMPS, CYCLE_ON_S, CYCLE_OFF_S = 30.0, 360.0, 840.0

    def __init__(self, sample=None, situation='bench', seed=7):
        self._seconds = 0
        #: THE BOARD'S CADENCE, thirty seconds (THERMAL_SAMPLE_EVERY_MS),
        #: three of wall time at HASTE. It was five: judged every twenty
        #: seconds a late cooldown's samples moved under the still rule's
        #: 0.3 K and the room never separated from the air path, so the
        #: stand-in stayed CONVERGING where the board goes STABLE.
        self._every_s = 30.0
        self._settle_s = 0.3
        #: WHERE IT LOOKS, not what it is told. A sampler that answers
        #: the phase currents and whether the bridge is switching - the
        #: two things this board has - and nothing about how hot
        #: anything is or is going to be.
        self._sample = sample or (lambda: {'amps': (0.0, 0.0, 0.0),
                                           'switching': False})
        #: An rms per phase, tracked across samples.
        self._rms = 0.0
        #: WHAT IT DROPS WHEN A NODE REACHES ITS CEILING. The board wires
        #: the gate drivers' own disable here.
        self._gate = None
        self._trips = 0
        #: What the effective duty is, asked of whatever owns the compares.
        self._duty = lambda: (0.0, 0.0, 0.0)
        #: Where the derate goes. The board wires the drive's clamp.
        self._derate_to = None
        self._last_power = None
        self._last_net = None
        self._derate_held = 1.0
        self._derate_at = None
        self._node = {n: thermal.AMBIENT for n in self.NODES}
        #: THE READING, LAGGED. See `thermal.NTC_TAU_S`.
        self._ntc = thermal.AMBIENT
        self._at = None
        #: The rotor's speed the air paths see, rpm: what the drive says,
        #: or nothing.
        self._speed_rpm = 0.0
        self._speed_of = lambda: 0.0
        #: THE GRAPH'S PARAMETERS, a copy this stand-in can move - the
        #: mirror's tables with the winding's record fields laid over, as
        #: `board_thermal.c` lays them.
        self._base = copy.deepcopy(thermal.CFG)
        self._base['capacity']['winding'] = self.WINDING_J_PER_K
        self._base['edges'][thermal.EDGE_WINDING_STATOR] = \
            0.25 * self.WINDING_K_PER_W
        self._base['to_ambient']['stator'] = 0.75 * self.WINDING_K_PER_W
        self._base['ntc_sees'] = thermal.NTC_SEES_DRIVERS
        self._base['ntc_tau_s'] = thermal.NTC_TAU_S
        # THE GROUND TRUTH: a second board, the base with a situation laid
        # over it, integrated on the same power and read through three
        # noisy thermometers every sample. `_node` above is the OBSERVER
        # - what the page shows, what the envelope acts on - anchored on
        # those readings exactly as `thermal.c` anchors, and identified
        # beside by the same identifier (`thermal_ident.py`). The
        # observer starts on the base; the identification finds the
        # situation, and the state says how far it has got.
        self._random = random.Random(seed)
        self._truth = {n: thermal.AMBIENT for n in self.NODES}
        self._truth_ntc = thermal.AMBIENT
        #: The room the truth stands in, and the observer's ESTIMATE of
        #: it - `thermal.c` infers ambient from the laminate's losses,
        #: there being no sensor for it, and so does the mirror's anchor.
        self._truth_ambient = thermal.AMBIENT
        self._ambient = thermal.AMBIENT
        self._situation = None
        self._truth_cfg = None
        self._switching = False
        self._switch_at = None
        self._switches = 0
        self._model_s = 0.0
        self._switched_s = 0.0
        self._sampled_s = 0.0
        self._since_seen_s = 0.0
        self._seen = {}
        self._settled = False
        self._ident = thermal_ident.Identifier(self.IDENT_NOISE_K,
                                               thermal.AMBIENT)
        # NOTHING BETWEEN RUNS. The record was a file (`COAXIAL_SIM_NVM`)
        # until 2026-09-06, as the flash sector was to the board; the
        # bench's rule took both out - every start is at the floor and
        # earns its span.
        self._margin_floor = self.MARGIN_FLOOR
        #: The load cycle, `(amps, on_s, off_s, began_model_s)` or None:
        #: what the live path samples instead of the drive while one runs.
        self._cycle = None
        self._cycle_trip = None     # the cycle index a trip ended early
        self._cfg = self._ident.apply(self._base)
        self.situation(situation)

    def _advance(self):
        """The network integrated forward to now, in steps it can take.

        SUB-STEPPED, and it has to be. One explicit step across a whole
        poll gap was what made the temperatures move in stairs, and a
        first-order step is only first order while it is small against
        the constant it is stepping. Reading it twice in a row is
        harmless: the second read finds no elapsed time and does nothing.
        """
        now = time.time()
        was, self._at = self._at, now
        if was is None:
            return
        elapsed = min(now - was, 5.0)
        if elapsed <= 0.0:
            return
        # THE SITUATION SWITCHES ON THE WALL CLOCK, when switching is on:
        # the page's viewer is what the interval is measured against.
        if self._switching and self._switch_at is not None \
                and now >= self._switch_at:
            self.situation('random')
        self.fast_forward(elapsed * self.HASTE, live=True)

    def fast_forward(self, model_seconds, seen=None, live=False):
        """Run the truth, the observer and the identification `model_seconds`
        on: the live path's own loop, and a test's way of taking the
        stand-in through a cooldown without waiting for one."""
        left = float(model_seconds)
        while left > 0.0:
            step = min(self.STEP_S, left)
            left -= step
            # SAMPLED EVERY SLICE, not once for the gap: the envelope
            # below writes the clamp into the drive, and the next slice
            # has to see what that did to the current.
            self._integrate(step, seen if seen is not None
                            else self._cycle_sample())
            # THE ENVELOPE INSIDE THE LOOP, not after it. `board_thermal.c`
            # runs the budget every THERMAL_STEP_MS - one step, one look.
            if live:
                self._envelope()

    def _envelope(self):
        """THE ONE PLACE THIS CLASS ACTS RATHER THAN REPORTS.

        A trip drops the stage, and the estimate is reported either way.
        `board_thermal.c` does exactly this after every step of the real
        observer. The limits come from the record; nothing here decides
        one (invariant 10). Latched by construction: dropping the stage is
        a thing it does, and only an arm brings the gates back.
        """
        # FIRST IT DERATES. Past the throttle point the drive's clamp is
        # scaled toward zero, so the stage keeps driving on less. One
        # clamp over every node the clamp reaches - the winding is one of
        # them since it is a node of the graph.
        # The factor is worked out whether or not a drive is wired to
        # take it: the load cycle reads it too, since a load the envelope
        # cannot clamp would cook the hypothetical board past the
        # ceilings it is there to act on.
        applied = self._derate_applied(self.derate())
        if self._derate_to is not None:
            self._derate_to(applied)
        # THEN, only if that was not enough - AND ON EVERY NODE, not just
        # the ones the clamp reaches.
        if self._gate is None or not self._tripped():
            return
        if self._gate():
            self._trips += 1

    def _integrate(self, dt, seen):
        """One explicit step over the whole graph: `thermal.net_flows` is
        the same arithmetic `thermal.c` steps, so the stand-in's
        temperatures are that model integrated rather than a second one.
        """
        power = self._power(dt, seen)
        self._last_power = power
        self._speed_rpm = float(self._speed_of() or 0.0)
        # THE TRUTH FIRST, on its own network, its thermistor by the same
        # rule as the observer's below.
        net = thermal.net_flows(self._truth, power, self._truth_cfg,
                                self._truth_ambient, self._speed_rpm)
        for name in self.NODES:
            capacity = self._truth_cfg['capacity'].get(name, 0.0)
            if capacity > 0.0:
                self._truth[name] += net[name] * dt / capacity
        self._truth_ntc = thermal_ident.ntc_follow(
            self._truth, self._truth_ntc, self._truth_cfg, dt)[0]
        # THEN THE OBSERVER, on the base with the identified scales and
        # the room as it believes it to be.
        net = thermal.net_flows(self._node, power, self._cfg,
                                self._ambient, self._speed_rpm)
        self._last_net = net
        for name in self.NODES:
            capacity = self._cfg['capacity'].get(name, 0.0)
            if capacity > 0.0:
                self._node[name] += net[name] * dt / capacity
        # A SAMPLE every `_every_s` of model time: the truth's three
        # thermometers with their own noise, and the observer anchored on
        # them with a pull sized to the interval, as the board does.
        self._model_s += dt
        self._since_seen_s += dt
        sample = None
        if self._every_s > 0.0 \
                and self._model_s - self._sampled_s >= self._every_s:
            self._sampled_s = self._model_s
            sample = self._read_truth(power)
            self._seen = dict(sample)
            self._ntc, self._settled = thermal_ident.anchor(
                self._node, self._ntc, self._cfg, power, sample,
                self._speed_rpm, self._since_seen_s, self._ambient)
            self._since_seen_s = 0.0
        # THE READING FOLLOWS THE PATCHES, it does not jump with them:
        # toward the weighted average of the two it sits between, at the
        # laminate's own lag, and never past either of them - a passive
        # link in a chain cannot read outside the pair (docs/papers, 2.3).
        self._ntc = thermal_ident.ntc_follow(self._node, self._ntc,
                                             self._cfg, dt)[0]
        # THE IDENTIFICATION BESIDE IT, on the same power and slice; a
        # sample that moves the scales re-applies them at once.
        if self._ident.step(self._node, self._ntc, self._base, power,
                            self._speed_rpm, sample, dt):
            self._cfg = self._ident.apply(self._base)
        # THE ROOM IS THE IDENTIFICATION'S, as on the board: no sensor
        # reads it, and the observer's rise is against what it believes.
        self._ambient = self._ident.ambient

    def _read_truth(self, power):
        """What the three thermometers read off the truth this sample:
        the element, and each die its node plus its watts through R_th -
        with ±NOISE_K of quantisation, deterministic from the seed."""
        def noisy(value):
            return value + (self._random.random() - 0.5) * 2.0 * self.NOISE_K

        out = {'ntc': noisy(self._truth_ntc)}
        for die in thermal_ident.DIES:
            out[die] = noisy(self._truth[die] + power.get(die, 0.0)
                             * self._truth_cfg['rth_die'].get(die, 0.0))
        return out

    # -- the truth's situation ------------------------------------------

    def situation(self, name=None, switching=None):
        """Lay a situation over the ground truth - `SITUATIONS` by name,
        or 'random' for one that is not the present one - and, with
        `switching`, turn the random switches on or off. Returns what
        the truth is now. The observer is not told: finding out is its
        job."""
        if switching is not None:
            self._switching = bool(switching)
            self._switch_at = (time.time() + self._random.uniform(*self.SWITCH_EVERY_S)
                               if self._switching else None)
        if name is not None:
            if name == 'random':
                choices = [n for n in self.SITUATIONS if n != self._situation]
                name = self._random.choice(choices)
            if name not in self.SITUATIONS:
                raise RigError('a situation is one of %s, or random'
                               % ', '.join(self.SITUATIONS))
            laid = self.SITUATIONS[name]
            if self._situation is not None:
                self._switches += 1
            self._situation = name
            self._switched_s = self._model_s
            self._truth_ambient = float(laid['ambient'])
            self._truth_cfg = thermal_ident.apply(
                (laid['air'], laid['capacity'], 1.0, 1.0), self._base)
            if self._switching:
                self._switch_at = time.time() + self._random.uniform(
                    *self.SWITCH_EVERY_S)
        return self.truth()

    def settle(self, seen=None):
        """Both boards at their equilibria for `seen`'s power - the truth on
        its network, the observer on its own - as a board is after an hour
        of idling: where a test starts that asks what idling teaches."""
        power = self._power(1.0, seen or self._sample())
        # An hour of idling has told the identification the room as well.
        self._ident.scale[thermal_ident.AMBIENT] = self._truth_ambient
        self._ambient = self._truth_ambient
        self._truth = thermal.steady(power, self._truth_cfg,
                                     ambient=self._truth_ambient)
        self._node = thermal.steady(power, self._cfg, ambient=self._ambient)
        for temps in (self._truth, self._node):
            for name in self.NODES:
                temps.setdefault(name, thermal.AMBIENT)
        centre, leg = self._truth['board'], self._truth[thermal.NTC_PATCH]
        self._truth_ntc = thermal.expected_ntc(centre, leg - centre)
        centre, leg = self._node['board'], self._node[thermal.NTC_PATCH]
        self._ntc = thermal.expected_ntc(centre, leg - centre)
        self._last_net = None

    def truth(self):
        """The ground truth as a page may show it beside the estimate:
        its situation, the scales that make it, how long it has stood,
        and the load the cycle has on it now - absent on a board, which
        has no truth to tell."""
        laid = self.SITUATIONS[self._situation]
        return {'situation': self._situation, 'air': laid['air'],
                'capacity': laid['capacity'], 'ambient': self._truth_ambient,
                'switches': self._switches,
                'since_s': self._model_s - self._switched_s,
                'switching': self._switching,
                'load_a': self._cycle_sample()['amps'][0] if self._cycle
                else None}

    # -- the load cycle ----------------------------------------------

    def load_cycle(self, amps=CYCLE_AMPS, on_s=CYCLE_ON_S, off_s=CYCLE_OFF_S):
        """Drive a load on and off from the model's own clock: `on_s` at
        `amps` on all three phases, switching, then `off_s` idle, over
        and over - what a page in simulated mode lays on so the map's
        regions warm and cool and the identification has cooldowns to
        learn from. `amps` zero or None stops it and the drive's own
        sample is read again. Returns what runs."""
        if not amps or float(amps) <= 0.0:
            self._cycle = None
            return {'amps': 0.0, 'on_s': 0.0, 'off_s': 0.0}
        if not (float(on_s) > 0.0 and float(off_s) > 0.0):
            raise RigError('a cycle is seconds on and seconds off, both '
                           'above zero - the walk is 360 and 840')
        self._cycle = (float(amps), float(on_s), float(off_s), self._model_s)
        self._cycle_trip = None
        return {'amps': float(amps), 'on_s': float(on_s),
                'off_s': float(off_s)}

    def _cycle_sample(self):
        """What the sampler sees this slice: the cycle's phase while one
        runs, the drive's sample otherwise. UNDER THE ENVELOPE: the amps
        are the cycle's times the clamp the envelope applies, so past the
        throttle point the run carries less, and a trip ends the run -
        the stage is down until the next on-phase, which stands for the
        re-arm a bench would do."""
        if self._cycle is None:
            return self._sample()
        amps, on_s, off_s, began = self._cycle
        since = self._model_s - began
        index = int(since // (on_s + off_s))
        phase = since - index * (on_s + off_s)
        if phase < on_s and index != self._cycle_trip:
            if self._tripped():
                self._cycle_trip = index
            else:
                amps *= self._derate_held
                return {'amps': (amps, amps, amps), 'switching': True}
        return {'amps': (0.0, 0.0, 0.0), 'switching': False}

    def _power(self, dt, seen):
        """Watts per node, worked out from the sample. The observer's job.

        `i^2 R` on what the shunts actually carried, across the
        resistance the current crosses - `inverter` holds the FET's
        Rds(on) and the shunt - plus the housekeeping, which does not
        care whether anything switches; the hot swap's two FETs on the
        link current; the winding's copper on the same mean square. The
        rms is tracked rather than taken from one sample: three phase
        currents at an instant are a vector, and a vector says nothing
        about how long it has been that big.
        """
        from .. import inverter

        amps = seen.get('amps') or (0.0, 0.0, 0.0)
        now = math.sqrt(sum(a * a for a in amps) / 3.0)
        self._rms += (now - self._rms) * min(1.0, dt / self.RMS_TAU)
        if not seen.get('switching'):
            self._rms *= max(0.0, 1.0 - dt / self.RMS_TAU)
        watt = thermal.phase_power(self._rms,
                                   inverter.RDS_ON + inverter.SHUNT,
                                   switching=bool(seen.get('switching')))
        watt['hotswap'] = (0.5 * self._rms) ** 2 * self.HOTSWAP_R
        watt['winding'] = 3.0 * self._rms * self._rms * self.WINDING_R
        return watt

    def state(self):
        self._seconds += 1
        self._advance()
        centre = self._node['board']
        power = self._last_power or {}
        seen = self._seen
        # MEASURED where a sample has been taken - the truth's thermometers
        # - and the observer's own element where none has; the board
        # reports both what the thermistor says and what the model expects.
        ntc = seen.get('ntc', self._ntc)
        return {
            'ntc': ntc,
            'nodes': dict(self._node),
            'ambient': self._ambient,          # ESTIMATED, as the board's
            'expected_ntc': self._ntc,
            'seconds': self._seconds,
            'settled': self._settled or not seen,
            'sample_every_s': self._every_s,
            'sample_settle_s': self._settle_s,
            'afe': seen.get('afe', self._node['afe']),
            'mcu': seen.get('mcu', self._node['mcu']),
            'seen_s_ago': (self._model_s - self._sampled_s) if seen else 0.4,
            'steps': 1200,
            'error': self._ntc - ntc,
            # MINOR 13: each leg's FET junction over its node - half the
            # node's watts through R_th,JC - and the speed the air saw.
            'junction_over': [0.5 * power.get(n, 0.0)
                              * self._cfg['rth_die'].get(n, 0.0)
                              for n in thermal.DRIVERS],
            'speed_rpm': int(self._speed_rpm),
        }

    def set_sample(self, every_s, settle_s=0.3):
        self._every_s, self._settle_s = every_s, settle_s
        return True

    def _limit(self, name):
        """One node's ceiling as the envelope acts on it: the record's,
        its span over the room trimmed by the identification's margin -
        the floor while the model is doubted whole, one when not at all
        - as `board_thermal.c` trims it, so the silicon and the laminate
        are not run to ceilings computed on a network just proved wrong."""
        top = self.LIMIT.get(name, self.DEFAULT_LIMIT)
        margin = self._ident.margin(self._margin_floor)
        return thermal.AMBIENT + margin * (top - thermal.AMBIENT)

    def _used(self):
        """Each node as a fraction of its own ceiling. One definition:
        `budget()` answers it and `_envelope()` acts on it."""
        used = {}
        for name in self.NODES:
            top = self._limit(name)
            used[name] = max(0.0, (self._node[name] - thermal.AMBIENT)
                             / (top - thermal.AMBIENT))
        return used

    def _worst(self):
        """The worst node the clamp can reach, and every node's spend."""
        used = self._used()
        driven = [n for n in used if n not in self.UNDRIVEN] or list(used)
        name = max(driven, key=lambda n: used[n])
        return used[name], name, used

    def _tripped(self):
        """Whether ANY node is at its ceiling, driven or not."""
        return max(self._used().values(), default=0.0) >= 1.0

    def derate(self, worst=None):
        """What the current clamp should be multiplied by, 1 down to 0:
        one at the throttle point and zero at the ceiling, linear
        between, on the worse of where the worst node is and how far
        into the window any driven node's hold has come - `thermal.c`'s
        own arithmetic."""
        spent = self._worst()[0] if worst is None else worst
        spent = max(spent, self._soon())
        return self._ramp(spent)

    @staticmethod
    def _ramp(spent):
        """One at the throttle point, zero at the ceiling, linear between.
        ONE DEFINITION for every node, as `thermal.c`'s `derate_of`."""
        band = 1.0 - THROTTLE_AT
        if spent <= THROTTLE_AT or band <= 0.0:
            return 1.0
        return max(0.0, 1.0 - (spent - THROTTLE_AT) / band)

    def _derate_applied(self, want):
        """The factor after the recovery slew. Down is immediate.

        ASYMMETRIC ON PURPOSE, and `board_thermal.c` does the same: cut
        the clamp and the ramp goes away, so the next look sees no ramp
        and asks for full current again - measured, that oscillated
        between 1.00 and 0.00 every hundred milliseconds. Recovering over
        seconds gives the node time to cool first.
        """
        now = time.time()
        was, self._derate_at = self._derate_at, now
        if want <= self._derate_held or was is None:
            self._derate_held = want
        else:
            step = self.DERATE_RECOVER_PER_S * min(5.0, now - was) * self.HASTE
            self._derate_held = min(want, self._derate_held + step)
        self._derate_held = max(0.0, min(1.0, self._derate_held))
        return self._derate_held

    def _hold(self, name):
        """Seconds this node can stay at its net power before its ceiling
        - the soak over what is going into it - or None when it is not
        heading there. The same net flows the step integrated."""
        net = (self._last_net or {}).get(name, 0.0)
        capacity = self._cfg['capacity'].get(name, 0.0)
        top = self._limit(name)
        if net <= 0.0 or capacity <= 0.0 or top <= thermal.AMBIENT:
            return None
        togo = top - self._node[name]
        return (togo * capacity / net) if togo > 0.0 else 0.0

    def _soon(self):
        """How far into the last `LOOKAHEAD_S` of hold the worst driven
        node is - THE SAME ARITHMETIC `thermal.c` DOES: time left, not a
        projected temperature, so a node at ambient has its whole soak
        in front of it and a burst runs."""
        worst = 0.0
        for name in self.NODES:
            if name in self.UNDRIVEN:
                continue
            hold = self._hold(name)
            if hold is None:
                continue
            worst = max(worst, min(1.0, 1.0 - hold / self.LOOKAHEAD_S))
        return worst

    def node_derate(self, name):
        """One node's OWN factor - its spend and its hold alone."""
        used = self._used().get(name, 0.0)
        hold = self._hold(name)
        soon = min(1.0, 1.0 - hold / self.LOOKAHEAD_S) if hold is not None \
            else 0.0
        return self._ramp(max(used, soon))

    def soak_j(self):
        """Joules each node can still absorb before its ceiling:
        `capacity x (limit - t)`, never negative."""
        out = {}
        for name in self.NODES:
            top = self._limit(name)
            out[name] = max(0.0, self._cfg['capacity'].get(name, 0.0)
                            * (top - self._node[name]))
        return out

    def budget(self):
        self._advance()
        worst, name, used = self._worst()
        return {'used': used, 'worst': worst, 'worst_node': name,
                'seconds_to_limit': None,
                'throttling': worst >= THROTTLE_AT,
                'tripped': self._tripped(), 'trips': self._trips,
                'derate': self._derate_held, 'soak_j': self.soak_j(),
                'duty': list(self._duty() or (0.0, 0.0, 0.0)),
                # MINOR 12: the winding's estimate, spend and OWN factor,
                # from the node it is, beside `derate` - the stage's.
                'winding_c': self._node['winding'],
                'winding_used': used['winding'],
                'winding_derate': self.node_derate('winding')}

    def network(self):
        """The graph as the stand-in holds it - the same shape
        `Thermal.network()` reads off a board."""
        nodes = {}
        for name in self.NODES:
            nodes[name] = {'capacity': self._cfg['capacity'].get(name, 0.0),
                           'to_ambient': self._cfg['to_ambient'].get(name, 0.0),
                           'area_share': self._cfg['area_share'].get(name, 0.0),
                           'rth_die': self._cfg['rth_die'].get(name, 0.0),
                           'forced': self._cfg['forced'].get(name, 0.0)}
        edges = [(a, b, r) for (a, b, _r), r
                 in zip(thermal.EDGES, self._cfg['edges'])]
        return {'nodes': nodes, 'edges': edges}

    def set_limit(self, node, limit_c, throttle_at=THROTTLE_AT):
        return True

    def set_winding(self, limit_c, k_per_w, j_per_k):
        if k_per_w <= 0.0 or j_per_k <= 0.0:
            raise RigError('the winding needs a positive K/W and J/K; '
                           'a zero ceiling is how it is disabled')
        self.LIMIT = dict(self.LIMIT, winding=float(limit_c))
        self.WINDING_K_PER_W = float(k_per_w)
        self.WINDING_J_PER_K = float(j_per_k)
        self._cfg['capacity']['winding'] = float(j_per_k)
        self._cfg['edges'][thermal.EDGE_WINDING_STATOR] = 0.25 * float(k_per_w)
        self._cfg['to_ambient']['stator'] = 0.75 * float(k_per_w)
        return True

    def set_node(self, node, to_board, capacity):
        """One node's first path out and its capacity - the sink edge for
        a source, the air for a patch, as `thermal_set_node` does."""
        if to_board <= 0.0 or capacity <= 0.0:
            raise RigError('a K/W and a heat capacity are both positive')
        edge = thermal.sink_edge(node)
        if edge is not None:
            self._cfg['edges'][edge] = float(to_board)
        else:
            self._cfg['to_ambient'][node] = float(to_board)
        self._cfg['capacity'][node] = float(capacity)
        return True

    def set_edge(self, edge, k_per_w):
        """One edge's K/W by index; None opens it."""
        self._cfg['edges'][int(edge)] = 0.0 if k_per_w is None \
            else float(k_per_w)
        return True

    def set_board(self, to_ambient, capacity):
        """The bulk's two numbers, shared out by area as the core does."""
        if to_ambient <= 0.0 or capacity <= 0.0:
            raise RigError('both are positive')
        self._cfg['board_to_ambient'] = float(to_ambient)
        self._cfg['board_capacity'] = float(capacity)
        for name in thermal.LAMINATE:
            share = self._cfg['area_share'][name]
            self._cfg['to_ambient'][name] = float(to_ambient) / share
            self._cfg['capacity'][name] = float(capacity) * share
        return True

    def identification(self):
        """The identification as the stand-in runs it - the wire's shape
        (`Thermal.identification`) off the same identifier the board
        runs, plus `truth`: the situation the ground truth is in, which
        no board can report and a page in simulated mode shows beside
        the estimate so the logic can be seen working."""
        self._advance()
        ident = self._ident
        got = {'state': ident.state,
               'scales': dict((s, ident.scale[k])
                              for k, s in enumerate(thermal.IDENT_SCALES)),
               'sigma': dict((s, ident.sigma(k))
                             for k, s in enumerate(thermal.IDENT_SCALES)),
               'online': [s for k, s in enumerate(thermal.IDENT_SCALES)
                          if thermal_ident.ONLINE[k]],
               'innovation_k': ident.innovation_k,
               'margin': ident.margin(self._margin_floor),
               'ambient': ident.ambient,
               'ambient_sigma': ident.sigma(thermal_ident.AMBIENT),
               'updates': ident.updates,
               # The wire's two since MINOR 14, and since 16 always
               # none: the board keeps nothing it identified.
               'saves': 0, 'since_save_s': None,
               'margin_floor': self._margin_floor,
               'truth': self.truth()}
        return got

    def reset_identification(self):
        """Forget what was identified: scales to one, UNCERTAIN, the
        margin back at the floor. Nothing is written anywhere."""
        self._ident = thermal_ident.Identifier(self.IDENT_NOISE_K,
                                               self._ambient)
        self._cfg = self._ident.apply(self._base)
        return True

    def set_margin_floor(self, floor):
        """The floor the margin rises from, a fraction (0, 1] of every
        ceiling's span - thermal op 12, refused in the board's words."""
        if not 0.0 < float(floor) <= 1.0:
            raise RigError('the floor is a fraction of the span, 1 .. '
                           '1 000 000 ppm - 800 000 is the bench\'s; zero '
                           'would trip the stage at boot')
        self._margin_floor = float(floor)
        return True


class SimulatedPower:
    """Rail reference counts without a board.

    Nothing switches, so the count is whatever was last asked for and `on`
    follows it exactly - which is the one thing the real board does not
    promise. It reads the pin back precisely so the two can disagree.
    """

    def __init__(self):
        self._mask = 0

    def state(self):
        from ..power import named
        return {'afe': {'on': self._mask != 0, 'users': named(self._mask),
                        'mask': self._mask, 'count': bin(self._mask).count('1'),
                        'blocked': False, 'leased': []}}

    def release_all(self):
        self._mask = 0
        return True


class SimulatedGateDrivers(GateControl):
    """TIM1, the injected triple and the STO chain, without any of them.

    The numbers are the real board's registers as configured: ARR 2375 for
    50 kHz off 237.5 MHz, DTG 19 for 80 ns. Enabling is refused until the
    break is bypassed, as the real board refuses it until the STO chain
    releases - and nothing here can release it.
    """

    PERIOD = 2376
    DEADTIME = 19
    TRIGGER = 2360

    def __init__(self):
        self._deadtime = self.DEADTIME
        self._deadtime_ns = self.DEADTIME * 4210 // 1000
        self._skew = 0
        self._armed = False
        self._enabled = False
        self._duty = (0, 0, 0)
        self._hold_until = None
        self._trigger = self.TRIGGER
        self._updates = 0
        self._keepalive = 0
        self._bypassed = False

    def state(self):
        self._keepalive += 214000        # the measured idle toggle rate
        if self._armed:
            self._updates += 50000
        left = 0
        if self._hold_until is not None:
            remaining = self._hold_until - time.monotonic()
            if remaining <= 0.0:
                self._duty = (0, 0, 0)
                self._hold_until = None
            else:
                left = max(1, int(remaining * 50000.0))
        at = self._cnt()
        return {
            'pwm_ready': True, 'pwm_enabled': self._enabled,
            'fault': not self._bypassed,
            'sync_ready': True, 'sync_armed': self._armed, 'afe_on': True,
            'pilot_ok': True, 'level_ok': True,
            'period': self.PERIOD, 'deadtime': self.DEADTIME,
            'duty': self._duty, 'trigger': self._trigger,
            'phase': (1433, -8136, 390), 'at': 1385,
            'updates': self._updates, 'overruns': 0,
            'keepalive': self._keepalive,
            'worst_gap_cycles': 24700,
            'pilot_raw': 15149, 'pilot_microvolts': 763000,
            'level_raw': 1305, 'level_microvolts': 65000,
            'break_bypassed': self._bypassed,
            # TICKS, like the board: it sends Q16.16 of a CCR count and
            # the host divides that back. `_duty` is already ticks.
            'requested_ticks': tuple(float(d) for d in self._duty),
            'pins': self._gates(at),
            'pins_at': at,
            'deadtime_ns': self._deadtime_ns,
            'deadtime_skew': self._skew,
            'periods_left': left,
            'deadtime_floor': self.DEADTIME_FLOOR,
            'gate_shorts': (),
        }

    #: DTG counts for 20 ns at 237.5 MHz, rounded up - the same floor the
    #: board computes, because the 2EDL8034 has no interlock either way.
    DEADTIME_FLOOR = 5
    DTG_MAX = 127
    DTS_PS = 4210

    def dead_time(self, nanoseconds=None, skew=0):
        if nanoseconds is None:
            return {'nanoseconds': self._deadtime_ns, 'skew': self._skew,
                    'floor': self.DEADTIME_FLOOR}

        counts = max(self.DEADTIME_FLOOR,
                     int(nanoseconds) * 1000 // self.DTS_PS)
        if counts + abs(int(skew)) > self.DTG_MAX:
            raise RigError("that dead time plus its skew is past DTG's "
                           "linear range - ask for 535 ns or less")
        if counts - abs(int(skew)) < self.DEADTIME_FLOOR:
            raise RigError('that skew would take one of the two dead times '
                           'under the 20 ns floor - raise the dead time '
                           'first, or skew it less')
        self._deadtime, self._skew = counts, int(skew)
        self._deadtime_ns = counts * self.DTS_PS // 1000
        return self.dead_time()

    #: Counts per read, chosen coprime with PERIOD so repeated reads walk
    #: the whole period instead of landing in one half of it. A wall-clock
    #: counter looked right and was not: sixty reads in a millisecond moved
    #: it seven ticks, and every sample showed the same side conducting.
    CNT_STEP = 617

    def _cnt(self):
        """Somewhere in the period, and somewhere else next time."""
        self._at = (getattr(self, '_at', 0) + self.CNT_STEP) % self.PERIOD
        return self._at

    def _gates(self, at):
        """The six signals a real one would show at this count.

        Complementary and never both on, because that is the property the
        dead time gives the real gate drivers and a stand-in that could show a leg
        conducting through would teach a reader the wrong thing. With MOE
        clear every output is low, which is both FETs off.
        """
        out = {}
        for leg, duty in zip(('U', 'V', 'W'), self._duty):
            high = self._enabled and at < duty
            out[leg + 'L'] = bool(self._enabled and not high)
            out[leg + 'H'] = bool(high)
        return {k: out[k] for k in ('UL', 'UH', 'VL', 'VH', 'WL', 'WH')}

    def reset_worst_gap(self):
        return True

    def bypass_break(self, on=True):
        self._bypassed = bool(on)
        return True

    def enable(self):
        # Refuses for the reason the real board refuses: the break is
        # latched because nFAULT is low, and clearing the latch does not
        # help while it stays low. Bypassing the break input is what gets
        # past it there, so it is what gets past it here.
        from ..errors import RigError
        if not self._bypassed:
            raise RigError('the board refused to enable the gate drivers - check '
                           'fault, and whether the STO chain has released '
                           '(simulated)')
        self._enabled = True
        return True

    def disable(self):
        self._enabled = False
        self._duty = (0, 0, 0)
        return True

    def duty(self, ticks, periods=0):
        from ..errors import RigError
        ticks = tuple(int(t) for t in ticks)
        if not self._enabled:
            raise RigError('the gate drivers are not enabled (simulated)')
        if len(ticks) != 3 or any(t > self.PERIOD - 1 for t in ticks):
            raise RigError('the board refused %r - past ARR (simulated)'
                           % (ticks,))
        self._duty = ticks
        # The counted hold, wall-paced like the rest of the stand-in: the
        # virtual interrupt zeroes the compares when the count runs out,
        # and state() is where the expiry is noticed - the board's own
        # shape, seen from the link.
        self._hold_until = (time.monotonic() + periods / 50000.0
                            if periods else None)
        return True

    def duty_fine(self, fractions):
        from ..errors import RigError
        fractions = tuple(fractions)
        if len(fractions) != 3:
            raise ValueError('%d duties, not 3' % len(fractions))
        if not self._enabled:
            raise RigError('the gate drivers are not enabled (simulated)')
        period = self.PERIOD - 1
        self._duty = tuple(max(0.0, min(1.0, f)) * period for f in fractions)
        return True

    def alternate(self, ticks_a, ticks_b):
        from ..errors import RigError
        ticks_a, ticks_b = tuple(int(t) for t in ticks_a), tuple(int(t) for t in ticks_b)
        if len(ticks_a) != 3 or len(ticks_b) != 3:
            raise ValueError('two triples of 3 compare values')
        if not self._enabled:
            raise RigError('the gate drivers are not enabled (simulated)')
        if any(t > self.PERIOD - 1 for t in ticks_a + ticks_b):
            raise RigError('the board refused %r / %r - past ARR (simulated)'
                           % (ticks_a, ticks_b))
        # The stand-in holds A: the real board's state shows whichever
        # triple the last update wrote.
        self._duty = ticks_a
        return True

    def arm(self):
        self._armed = True
        return True

    def disarm(self):
        self._armed = False
        return True

    def trigger(self, ticks=None):
        if ticks is not None:
            self._trigger = min(int(ticks), self.PERIOD - 1)
            drive = getattr(self, '_drive', None)
            if drive is not None:
                drive.trigger(self._trigger)
        return self._trigger

    def clear_fault(self):
        return True
