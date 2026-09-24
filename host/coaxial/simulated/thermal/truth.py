"""The stand-in's ground truth: the network integrated, the rooms laid on, the tour, the load."""
import copy
import math
import time
from typing import Any

from coaxial.errors import RigError
from coaxial.kalman import thermal_ident
from coaxial.model import inverter, thermal
from motor import catalog


class ThermalTruth:

    """The board the estimate chases: its network, its situations and its load cycle."""

    # What the class this mixes into brings.
    HASTE: Any
    IDENT_NOISE_K: Any
    NODES: Any
    WINDING_J_PER_K: Any
    WINDING_K_PER_W: Any
    _derate_held: Any
    _envelope: Any
    _every_s: Any
    _random: Any
    _sample: Any
    _speed_of: Any
    _tripped: Any

    #: The tracked rms's time constant, s: one sample (a vector, not an
    #: amplitude) cannot move it; a load step shows within a second.
    RMS_TAU = 0.5

    #: The longest slice, model s (`THERMAL_STEP_MS`): the envelope runs once a
    #: step, and a fifth of the stiffest constant (a leg's 1.4 s) keeps the
    #: explicit step stable.
    STEP_S = 0.1

    #: The winding's envelope, the record's CAL_VERSION 12 defaults: the
    #: motor profile's placeholder pair and an estimated ceiling, over the
    #: stand-in record's phase resistance. Split into the graph as the
    #: firmware splits it: a quarter of the K/W from the copper into the
    #: iron, the rest the iron's air path.
    WINDING_R = catalog.BENCH_MOTOR.r

    #: The hot swap's two pass FETs in series - the bridge's own part,
    #: `thermal_losses` says 3.6 mOhm - and the link current they see is
    #: the phases' at the duty, estimated as half the rms here since the
    #: stand-in's sampler has no duty to weigh by.
    HOTSWAP_R = 3.6e-3

    #: The ground truth's situations: scales on its air path and laminate
    #: capacity, and its room. The observer is not told the room and reads it
    #: from its own losses, as the board does.
    SITUATIONS = {'bench': {'air': 1.0, 'capacity': 1.0, 'ambient': 25.0},
                  'box': {'air': 2.0, 'capacity': 1.0, 'ambient': 25.0},
                  'fan': {'air': 0.5, 'capacity': 1.0, 'ambient': 25.0},
                  'heatsink': {'air': 0.35, 'capacity': 1.6, 'ambient': 25.0},
                  'stuffy': {'air': 1.5, 'capacity': 1.0, 'ambient': 25.0},
                  'outdoors': {'air': 0.8, 'capacity': 1.0, 'ambient': -20.0},
                  # The bench's robot's rooms, named for their temperature
                  # (2026-09-06).
                  'temperate': {'air': 1.0, 'capacity': 1.0, 'ambient': 20.0},
                  'cold': {'air': 0.9, 'capacity': 1.0, 'ambient': -25.0},
                  'toasty': {'air': 1.2, 'capacity': 1.0, 'ambient': 45.0}}

    #: Wall seconds between switches when they are on: long enough for
    #: STABLE to be reached between them at HASTE, short enough to watch.
    SWITCH_EVERY_S = (180.0, 360.0)

    #: The tour: temperate 20 C, cold -25, toasty 45, round again. It moves on
    #: when the room is earned - STABLE held TOUR_STABLE_S (10 wall s at HASTE),
    #: no sooner than TOUR_MIN_S - or at TOUR_MAX_S. Measured under the page's
    #: cycle: STABLE at minute 10 temperate, 23-25 cold, 38-48 toasty, so the cap
    #: is 50 min (at 45 toasty sometimes hit it; CI, 2026-09-06).
    TOUR = ('temperate', 'cold', 'toasty')

    TOUR_STABLE_S = 100.0

    TOUR_MIN_S = 300.0

    TOUR_MAX_S = 3000.0

    #: The thermometers' own noise, ±kelvin, and their floor as the
    #: identifier is told it - the board's 30 mK NTC and 125 mK dies.
    NOISE_K = 0.05

    #: The page's load cycle, model s: 6 min at 30 A, 14 cooling - two wall
    #: minutes at HASTE (bench 2026-09-06).
    CYCLE_AMPS, CYCLE_ON_S, CYCLE_OFF_S = 30.0, 360.0, 840.0

    def _lay_base(self):
        """The graph's parameters, a copy this stand-in can move: the
        mirror's tables with the winding's record fields laid over, as
        `board_thermal.c` lays them.
        """
        self._base = copy.deepcopy(thermal.CFG)
        self._base['capacity']['winding'] = self.WINDING_J_PER_K
        self._base['edges'][thermal.EDGE_WINDING_STATOR] = \
            thermal.WINDING_INTO_IRON * self.WINDING_K_PER_W
        self._base['to_ambient']['stator'] = \
            (1.0 - thermal.WINDING_INTO_IRON) * self.WINDING_K_PER_W
        self._base['ntc_sees'] = thermal.NTC_SEES_DRIVERS
        self._base['ntc_tau_s'] = thermal.NTC_TAU_S

    def _start_in_room(self):
        """The board starts in its room, as a board does: the truth at the
        room it was switched on in, the observer at its thermistor's
        reading and the identification's room at the same -
        `Board_ThermalInit` starts on the NTC.
        """
        start = self._truth_ambient
        for temps in (self._truth, self._node):
            for name in self.NODES:
                temps[name] = start
        self._truth_ntc = self._ntc = start
        self._ambient = start
        self._ident = thermal_ident.Identifier(self.IDENT_NOISE_K, start)
        self._cfg = self._ident.apply(self._base)

    def _advance(self):
        """The network integrated forward to now, in steps it can take."""
        if self._driven:
            return                  # the caller has the clock
        now = time.time()
        was, self._at = self._at, now
        if was is None:
            return
        elapsed = min(now - was, 5.0)
        if elapsed <= 0.0:
            return
        # The situation switches on the wall clock, when switching is on: the
        # page's viewer is what the interval is measured against.
        if self._switching and not self._tour and self._switch_at is not None \
                and now >= self._switch_at:
            self.situation('random')
        self._run(elapsed * self.HASTE, None, True)

    def fast_forward(self, model_seconds, seen=None, live=False):
        """Run the truth, the observer and the identification
        `model_seconds` on - a test's or a notebook's way of taking the
        stand-in through a cooldown without waiting for one.
        """
        self._driven = True
        self._run(model_seconds, seen, live)

    def _run(self, model_seconds, seen, live):
        """The loop itself, the live path's and `fast_forward`'s."""
        left = float(model_seconds)
        while left > 0.0:
            step = min(self.STEP_S, left)
            left -= step
            # Sampled every slice, not once for the gap: the envelope below
            # writes the clamp into the drive, and the next slice has to see
            # what that did to the current.
            self._integrate(step, seen if seen is not None
                            else self._cycle_sample())
            # The envelope inside the loop, not after it.
            if live:
                self._envelope()

    def _integrate(self, dt, seen):
        """One explicit step over the whole graph: `thermal.net_flows` is
        the same arithmetic `thermal.c` steps, so the stand-in's
        temperatures are that model integrated rather than a second one.
        """
        power = self._power(dt, seen)
        self._last_power = power
        self._speed_rpm = float(self._speed_of() or 0.0)
        # The truth first, on its own network, its thermistor by the same rule
        # as the observer's below.
        net = thermal.net_flows(self._truth, power, self._truth_cfg,
                                self._truth_ambient, self._speed_rpm)
        truth_cfg = self._laid()
        for name in self.NODES:
            capacity = truth_cfg['capacity'].get(name, 0.0)
            if capacity > 0.0:
                self._truth[name] += net[name] * dt / capacity
        self._truth_ntc = thermal_ident.ntc_follow(
            self._truth, self._truth_ntc, self._truth_cfg, dt)[0]
        # Then the observer, on the base with the identified scales and the
        # room as it believes it to be.
        net = thermal.net_flows(self._node, power, self._cfg,
                                self._ambient, self._speed_rpm)
        self._last_net = net
        for name in self.NODES:
            capacity = self._cfg['capacity'].get(name, 0.0)
            if capacity > 0.0:
                self._node[name] += net[name] * dt / capacity
        # A sample every `_every_s` of model time: the truth's three
        # thermometers with their own noise, and the observer anchored on them
        # with a pull sized to the interval, as the board does.
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
        # The reading follows the two patches it sits between, at the
        # laminate's lag, never outside them (docs/papers, 2.3).
        self._ntc = thermal_ident.ntc_follow(self._node, self._ntc,
                                             self._cfg, dt)[0]
        # The identification beside it, on the same power and slice; a sample
        # that moves the scales re-applies them at once.
        if self._ident.step(self._node, self._ntc, self._base, power,
                            self._speed_rpm, sample, dt):
            self._cfg = self._ident.apply(self._base)
        # The room is the identification's, as on the board: no sensor reads
        # it, and the observer's rise is against what it believes.
        self._ambient = self._ident.ambient
        if self._tour:
            self._tour_step(dt)

    def _read_truth(self, power):
        """What the three thermometers read off the truth this sample: the
        element, and each die its node plus its watts through R_th - with
        ±NOISE_K of quantisation, deterministic from the seed.
        """
        def noisy(value):
            return value + (self._random.random() - 0.5) * 2.0 * self.NOISE_K

        out = {'ntc': noisy(self._truth_ntc)}
        for die in thermal_ident.DIES:
            out[die] = noisy(self._truth[die] + power.get(die, 0.0)
                             * self._laid()['rth_die'].get(die, 0.0))
        return out

    def _laid(self):
        """The truth's configuration for the situation laid on, which every
        stand-in has from construction.
        """
        if self._truth_cfg is None:
            raise RigError('no situation laid on the stand-in')
        return self._truth_cfg

    def situation(self, name=None, switching=None):
        """Lay a situation over the ground truth - `SITUATIONS` by name,
        'random' for one that is not the present one, or 'tour' for the
        rooms in turn (`TOUR`), moved on by the identification's own
        earned margin - and, with `switching`, turn the random switches
        on or off.
        """
        if switching is not None:
            self._switching = bool(switching)
            self._switch_at = (time.time() + self._random.uniform(*self.SWITCH_EVERY_S)
                               if self._switching else None)
        if name is not None:
            self._tour = name == 'tour'
            self._earned_s = 0.0
            self._lay(self._resolve(name))
        return self.truth()

    def _resolve(self, name):
        """A situation by name: the tour's next stop for 'tour', one that is
        not the present one for 'random', or a raise naming them all.
        """
        if name == 'tour':
            return self._next_stop()
        if name == 'random':
            return self._random.choice(
                [n for n in self.SITUATIONS if n != self._situation])
        if name not in self.SITUATIONS:
            raise RigError('a situation is one of %s, random, or tour'
                           % ', '.join(self.SITUATIONS))
        return name

    def _lay(self, name):
        """The situation onto the truth: its air path, capacity and room."""
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

    def _next_stop(self):
        """The tour's next room: the one after the present, the first from
        anywhere off the tour.
        """
        at = self.TOUR.index(self._situation) if self._situation in self.TOUR \
            else -1
        return self.TOUR[(at + 1) % len(self.TOUR)]

    def _tour_step(self, dt):
        """Move the tour on once the room is earned - STABLE held for
        TOUR_STABLE_S, no sooner than TOUR_MIN_S after the last move - or
        after TOUR_MAX_S whatever the state did.
        """
        stable = self._ident.state == thermal_ident.STABLE
        self._earned_s = (self._earned_s + dt) if stable else 0.0
        stood = self._model_s - self._switched_s
        if ((self._earned_s >= self.TOUR_STABLE_S and stood >= self.TOUR_MIN_S)
                or stood >= self.TOUR_MAX_S):
            self._lay(self._next_stop())
            self._earned_s = 0.0

    def settle(self, seen=None):
        """Both boards at their equilibria for `seen`'s power - the truth on
        its network, the observer on its own - as a board is after an
        hour of idling: where a test starts that asks what idling
        teaches.
        """
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
        """The ground truth as a page may show it beside the estimate: its
        situation, the scales that make it, how long it has stood, and
        the load the cycle has on it now - absent on a board, which has
        no truth to tell.
        """
        name = self._situation or 'bench'
        laid = self.SITUATIONS[name]
        return {'situation': name, 'air': laid['air'],
                'capacity': laid['capacity'], 'ambient': self._truth_ambient,
                'switches': self._switches,
                'since_s': self._model_s - self._switched_s,
                'switching': self._switching, 'tour': self._tour,
                'load_a': self._cycle_sample()['amps'][0] if self._cycle
                else None}

    def load_cycle(self, amps=CYCLE_AMPS, on_s=CYCLE_ON_S, off_s=CYCLE_OFF_S):
        """Drive a load on and off from the model's own clock: `on_s` at
        `amps` on all three phases, switching, then `off_s` idle, over
        and over - what a page in simulated mode lays on so the map's
        regions warm and cool and the identification has cooldowns to
        learn from.
        """
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
        runs, the drive's sample otherwise.
        """
        if self._cycle is None:
            return self._sample()
        amps, on_s, off_s, began = self._cycle
        since = self._model_s - began
        index = int(since // (on_s + off_s))
        phase = since - index * (on_s + off_s)
        idle = {'amps': (0.0, 0.0, 0.0), 'switching': False}
        if phase >= on_s or index == self._cycle_trip:
            return idle
        if self._tripped():
            self._cycle_trip = index
            return idle
        amps *= self._derate_held
        return {'amps': (amps, amps, amps), 'switching': True}

    def _power(self, dt, seen):
        """Watts per node, worked out from the sample. The observer's job."""

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
