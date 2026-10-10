"""The thermal observer without a board: the board's own C on the host, beside a truth in its room.

thermal/src/thermal_run.c - the observer, its identification and its envelope as the board runs
them - stepped on world/src/world_heat.c's truth, a slot of world/src/world_stand.c's
(tools.cores.stand): one code with the part and the emulated worlds (the user, 2026-10-10).
This class is the stand-in board's edge: the load gathered off its drive, the room and its tour
laid, the clamp and the stage acted on, the views named as the board's device names them.
"""
import ctypes
import gc
import math
import random
import time
import weakref
from typing import Callable, Optional

from coaxial.comm.wire import BYTE_FRACTION, CENTI, MICRO, MILLI, centi, micro, milli
from coaxial.devices.thermal import THROTTLE_AT, ThermalControl, application
from coaxial.errors import RigError
from coaxial.model import inverter, rooms, thermal, thermal_app
from motor import catalog, pmsm
from tools.cores import stand

#: The slots no stand-in holds, the lowest first.
_FREE = list(range(stand.SLOTS))


def _slot():
    """A free slot - after a collection, where cycles hold the last of them."""
    if not _FREE:
        gc.collect()
    if not _FREE:
        raise RigError('every one of the %d stand-in boards a process holds is taken'
                       % stand.SLOTS)
    return _FREE.pop(0)


def _none(value):
    """A thermometer's reading in the wire's hundredths, None where it answered nothing."""
    return None if math.isnan(value) else _c(value)


def _c(value):
    """`value` as the wire carries it in hundredths."""
    return centi(value) / CENTI


def _m(value):
    """In thousandths."""
    return milli(value) / MILLI


def _u(value):
    """In millionths."""
    return micro(value) / MICRO


class SimulatedThermal(ThermalControl):
    """The thermal observer without a board: the twenty-node network, its identification and its
    envelope as thermal_run.c runs them on the part, beside a truth laid in a room."""

    NODES = thermal.ALL_NODES

    #: The ceiling each node is judged against - the record's defaults: laminate lower than the
    #: rest because it is what everything else sits on, the motor's three at the winding's.
    LIMIT = dict(thermal.CEILING_C)
    DEFAULT_LIMIT = thermal.CEILING_DEFAULT_C

    #: The clock, thermal s per wall s (`configure(clock=)`, op 13 on a board).
    HASTE = thermal.HASTE

    WINDING_K_PER_W = pmsm.WINDING_K_PER_W
    WINDING_J_PER_K = pmsm.WINDING_J_PER_K
    WINDING_R = catalog.BENCH_MOTOR.r

    #: The floor the envelope's margin rises from, the record's default.
    MARGIN_FLOOR = thermal.IDENT_MARGIN_FLOOR

    #: Nodes the current clamp cannot cool (`soa_undriven_mask`): judged, not throttled on.
    UNDRIVEN = ('mcu', 'regulators', 'afe')

    #: The window of hold the throttle keeps, s (`soa_lookahead_ms`).
    LOOKAHEAD_S = 2.0

    #: The thermometers' noise, +-K: the board's 30 mK NTC and 125 mK dies.
    NOISE_K = 0.05

    #: The ground truth's situations and their tour (coaxial.model.rooms).
    SITUATIONS = rooms.SITUATIONS
    TOUR = rooms.TOUR

    #: Wall seconds between switches when they are on: long enough for STABLE between them.
    SWITCH_EVERY_S = (180.0, 360.0)

    #: The default load cycle, model s: 6 min at 30 A, 14 cooling (bench 2026-09-06).
    CYCLE_AMPS, CYCLE_ON_S, CYCLE_OFF_S = 30.0, 360.0, 840.0

    #: The tour judged this often while it runs, model s: a minute's call once took it whole.
    TOUR_STEP_S = 5.0

    def __init__(self, sample=None, situation='bench', seed=7):
        self._slot = _slot()
        weakref.finalize(self, _FREE.append, self._slot)
        self._lib = stand.library()
        #: The board's cadence, 30 s (THERMAL_SAMPLE_EVERY_MS), and its settle.
        self._every_s, self._settle_s = 30.0, 0.5
        #: The sampler: phase currents and whether the bridge switches - no temperatures.
        self._sample = sample or (lambda: {'amps': (0.0, 0.0, 0.0), 'switching': False})
        #: The stand-in board wires these: its AFE rail, the drive's hold on the meter, what
        #: drops the stage, the compares' duty, the drive's clamp, the rotor's rpm.
        self._afe_on: Callable[[], bool] = lambda: False
        self._meter_locked: Callable[[], bool] = lambda: False
        self._gate: Optional[Callable[[], bool]] = None
        self._duty: Callable[[], tuple] = lambda: (0.0, 0.0, 0.0)
        self._derate_to: Optional[Callable[[float], None]] = None
        self._speed_of: Callable[[], float] = lambda: 0.0
        self._app = thermal_app.STILL
        self._random = random.Random(seed)
        self._margin_floor = self.MARGIN_FLOOR
        self._throttle_at = THROTTLE_AT
        self._situation: Optional[str] = None
        self._switching = False
        self._switch_at = None
        self._tour = False
        self._earned_s = 0.0
        self._switches = 0
        self._switched_s = 0.0
        #: Whose clock: the wall's until `fast_forward` takes it.
        self._driven = False
        self._at = None
        self._cycle = None
        name = self._resolve(situation)
        self._lib.world_stand_init(self._slot, thermal_app.APPLICATIONS.index(self._app),
                                   self.WINDING_K_PER_W, self.WINDING_J_PER_K, self.WINDING_R,
                                   self.SITUATIONS[name]['ambient'], self.NOISE_K, seed)
        self._lib.world_stand_every(self._slot, int(round(self._every_s * 1000.0)))
        self._lay_envelope()
        self._tour = situation == 'tour'
        self._lay(name)

    # ---- the edge: the row, the run, the views ------------------------------------------

    def _row(self, seen, acting):
        """The load as the board gathers it (world_stand.h's row, `stand.ROW`): `seen` a sample
        fixed by the caller, or the drive's."""
        sample = seen if seen is not None else self._sample()
        amps = tuple(sample.get('amps') or (0.0, 0.0, 0.0))
        duty = tuple(sample.get('duty') or self._duty() or (0.0, 0.0, 0.0))
        return stand.floats((float(bool(self._afe_on())), float(bool(sample.get('switching'))))
                            + duty + amps
                            + (float(sample.get('link') or 0.0),
                               float(sample.get('link_amps', -1.0)),
                               float(self._speed_of() or 0.0), inverter.T_DEAD,
                               float(bool(self._meter_locked())), float(bool(acting))))

    def _run(self, model_seconds, seen, live):
        """`model_seconds` of the pair, the envelope acting on the board where `live`; on a
        tour in TOUR_STEP_S, each judged."""
        left = float(model_seconds)
        while True:
            part = min(left, self.TOUR_STEP_S) if self._tour else left
            out = (ctypes.c_float * 3)()
            self._lib.world_stand_run(self._slot, self._row(seen, live), part, out)
            if live and self._derate_to is not None:
                self._derate_to(out[0])
            if live and out[1] and self._gate is not None:
                self._gate()
            if self._tour:
                self._tour_step(part)
            left -= part
            if left <= 0.0:
                return

    def _view(self, what):
        return stand.view(self._slot, what)

    @property
    def _model_s(self):
        """The observer's clock, model s."""
        return self._view(stand.STATE)[28] / 1000.0

    def _advance(self):
        """The pair on to now, on the wall's clock at HASTE, unless a caller has the clock."""
        if self._driven:
            return
        now = time.time()
        was, self._at = self._at, now
        if was is None:
            return
        elapsed = min(now - was, 5.0)
        if elapsed <= 0.0:
            return
        # The situation switches on the wall clock, when switching is on.
        if self._switching and not self._tour and self._switch_at is not None \
                and now >= self._switch_at:
            self.situation('random')
        self._run(elapsed * self.HASTE, None, True)

    def fast_forward(self, model_seconds, seen=None, live=False):
        """The truth, the observer and the identification `model_seconds` on - a test's or a
        notebook's way through a cooldown without waiting for one."""
        self._driven = True
        self._run(model_seconds, seen, live)

    def _judge(self):
        """The envelope judged where the observer stands, acting: no time passes."""
        self._run(0.0, None, True)

    def _set_phase_r(self, ohm):
        """The winding's resistance, ohm, in the observer's losses and the truth's."""
        self._lib.world_stand_phase_r(self._slot, float(ohm))

    def _air(self):
        """(the told airspeed while it holds, the truth's own), m/s."""
        v = self._view(stand.STATE)
        return v[31], v[32]

    def _place(self, node, celsius):
        """An observer's node at `celsius`: a test's way to a hot leg."""
        self._lib.world_stand_place(self._slot, self.NODES.index(node), float(celsius))

    def settle(self, seen=None):
        """Both boards at their equilibria for `seen`'s power - as a board is after an hour of
        idling: where a test starts that asks what idling teaches."""
        self._lib.world_stand_settle(self._slot, self._row(seen, False), 7200.0)

    # ---- what the board's device answers --------------------------------------------------

    def state(self):
        self._advance()
        v = self._view(stand.STATE)
        expected, settled, sampled = _c(v[21]), bool(v[22]), bool(v[26])
        ntc = _c(v[23]) if sampled and not math.isnan(v[23]) else expected
        return {'ntc': ntc, 'nodes': {n: _c(t) for n, t in zip(self.NODES, v[:20])},
                'ambient': _c(v[20]), 'expected_ntc': expected, 'seconds': int(v[28] / 1000.0),
                'settled': settled or not sampled,
                'sample_every_s': self._every_s, 'sample_settle_s': self._settle_s,
                'afe': _none(v[24]), 'mcu': _none(v[25]),
                'seen_s_ago': _m(v[27] / 1000.0) if sampled else 0.4,
                'steps': int(v[29]), 'error': _c(expected - ntc),
                'junction_over': [_c(x) for x in v[33:36]], 'speed_rpm': int(v[30])}

    def budget(self):
        self._advance()
        v = self._view(stand.BUDGET)
        used = {n: v[k] / BYTE_FRACTION for k, n in enumerate(self.NODES)}
        millis = v[22]
        return {'used': used, 'worst': v[20] / BYTE_FRACTION,
                'worst_node': self.NODES[int(v[21])],
                'seconds_to_limit': millis / 1000.0 if millis >= 0 else None,
                'throttling': bool(v[23]), 'tripped': bool(v[24]), 'trips': int(v[50]),
                'derate': _u(v[25]), 'soak_j': {n: _m(j) for n, j in zip(self.NODES, v[27:47])},
                'duty': [_u(d) for d in (self._duty() or (0.0, 0.0, 0.0))],
                # MINOR 12: the winding's estimate, spend and own factor.
                'winding_c': _c(v[47]), 'winding_used': v[48] / BYTE_FRACTION,
                'winding_derate': _u(v[49])}

    def derate(self):
        """What the current clamp is asked to be multiplied by now, before its recovery's
        slew - thermal_budget's."""
        return _u(self._view(stand.BUDGET)[26])

    def network(self):
        """The graph the observer runs, the shape `Thermal.network()` reads off a board."""
        v = self._view(stand.NETWORK)
        fields = ('capacity', 'to_ambient', 'area_share', 'rth_die', 'forced')
        nodes = {n: dict(zip(fields, v[5 * k:5 * k + 5])) for k, n in enumerate(self.NODES)}
        edges = [(self.NODES[int(v[100 + 3 * e])], self.NODES[int(v[101 + 3 * e])],
                  v[102 + 3 * e]) for e in range(len(thermal.EDGES))]
        return {'nodes': nodes, 'edges': edges}

    def identification(self):
        """The identification in the wire's shape (`Thermal.identification`), plus `truth` -
        the ground truth's situation, which no board can report."""
        self._advance()
        v = self._view(stand.IDENT)
        scales, sigma, online = {}, {}, []
        for k, name in enumerate(thermal.IDENT_SCALES):
            scales[name], sigma[name] = _m(v[1 + 3 * k]), _m(v[2 + 3 * k])
            if v[3 + 3 * k]:
                online.append(name)
        rest = 1 + 3 * len(thermal.IDENT_SCALES)
        innovation, margin, updates, ambient, ambient_sigma, floor, cap = v[rest:rest + 7]
        return {'state': thermal.IDENT_STATES[int(v[0])], 'scales': scales, 'sigma': sigma,
                'online': online, 'innovation_k': _m(innovation), 'margin': _u(margin),
                'ambient': _c(ambient), 'ambient_sigma': _c(ambient_sigma),
                'updates': int(updates),
                # The board keeps nothing it identified (MINOR 16).
                'saves': 0, 'since_save_s': None,
                'margin_floor': _u(floor), 'trip_cap': _u(cap), 'application': self._app,
                'truth': self.truth()}

    def _thermistor(self):
        """The NTC's element in the truth now, C: what the channel reads."""
        self._advance()
        return self._view(stand.TRUTH)[20]

    def _die(self, name):
        """A die in the truth now, C: its node plus its watts through R_th,JC - what the part on
        it reads live (the A1335's TSEN)."""
        self._advance()
        return self._view(stand.TRUTH)[21 if name == 'mcu' else 22]

    # ---- the record's thermal fields, as device 9 writes them -----------------------------

    def _lay_envelope(self):
        """The record's envelope into the run: each node's ceiling, the undriven, the throttle,
        the window, the floor."""
        limits = stand.floats([float(self.LIMIT.get(n, self.DEFAULT_LIMIT)) for n in self.NODES])
        undriven = sum(1 << self.NODES.index(n) for n in self.UNDRIVEN)
        self._lib.world_stand_envelope(self._slot, limits, undriven, self._throttle_at,
                                       self.LOOKAHEAD_S, self._margin_floor)

    def _set_sample(self, every_s, settle_s=0.3):
        self._every_s, self._settle_s = every_s, settle_s
        self._lib.world_stand_every(self._slot, int(round(float(every_s) * 1000.0)))
        return True

    def _wep(self, seconds):
        if not 0.0 <= seconds <= 2.0:
            raise RigError('WEP is 0 .. 2 s, the trip standing')
        self._lib.world_stand_wep(self._slot, int(round(seconds * self.HASTE * 1000.0)))
        if seconds > 0.0 and self._derate_to is not None:
            self._derate_to(1.0)
        return True

    def _set_clock(self, haste):
        if haste != int(haste) or not 1 <= haste <= 1000:
            raise RigError('the thermal clock is whole thermal seconds a second, 1 .. 1000, '
                           'not %r' % haste)
        self.HASTE = float(haste)
        return True

    def _set_margin_floor(self, floor):
        if not 0.0 < float(floor) <= 1.0:
            raise RigError('the floor is a fraction of the span, 1 .. 1 000 000 ppm - 800 000 '
                           'is the bench\'s; zero would trip the stage at boot')
        self._margin_floor = float(floor)
        self._lay_envelope()
        return True

    def _set_limit(self, node, limit_c, throttle_at=THROTTLE_AT):
        self.LIMIT = dict(self.LIMIT, **{node: float(limit_c)})
        if 0.0 < throttle_at < 1.0:
            self._throttle_at = float(throttle_at)
        self._lay_envelope()
        return True

    def _set_application(self, name):
        """What the board is mounted in, thermal op 15: both networks laid anew on it, the
        identification started over."""
        application(name)
        self._app = name
        self._lib.world_stand_application(self._slot, thermal_app.APPLICATIONS.index(name))
        return True

    def _network(self, what, node, a, b):
        if not self._lib.world_stand_network(self._slot, what, node, float(a), float(b)):
            raise RigError('a K/W and a heat capacity are both positive')
        return True

    def _set_node(self, node, to_board, capacity):
        """One node's first path out and its capacity, as `thermal_set_node` does."""
        return self._network(0, self.NODES.index(node), to_board, capacity)

    def _set_edge(self, edge, k_per_w):
        """One edge's K/W by index; None opens it."""
        return self._network(1, int(edge), 0.0 if k_per_w is None else k_per_w, 0.0)

    def _set_board(self, to_ambient, capacity):
        """The bulk's two numbers, shared out by area as the core does."""
        return self._network(2, 0, to_ambient, capacity)

    def _set_winding(self, limit_c, k_per_w, j_per_k):
        if k_per_w <= 0.0 or j_per_k <= 0.0:
            raise RigError('the winding needs a positive K/W and J/K; a zero ceiling is how it '
                           'is disabled')
        self.LIMIT = dict(self.LIMIT, winding=float(limit_c))
        self.WINDING_K_PER_W, self.WINDING_J_PER_K = float(k_per_w), float(j_per_k)
        self._network(3, 0, k_per_w, j_per_k)
        self._lay_envelope()
        return True

    def _reset_identification(self):
        """Forget what was identified: scales to one, UNCERTAIN, the margin at the floor."""
        self._lib.world_stand_identify(self._slot)
        return True

    # ---- the stand-in's own: its air, its room, its load ----------------------------------

    def airspeed(self, m_s, truth=None):
        """The frame's airspeed across the board as the host knows it, m/s, held a wall second
        on the observer's clock (op 16); the truth's own air `truth`, the told where none."""
        if not 0.0 <= float(m_s) <= thermal_app.AIRSPEED_MAX_M_S:
            raise RigError('an airspeed is 0 .. 100 000 mm/s, held a second - and the observer '
                           'starts with the board')
        self._lib.world_stand_airspeed(
            self._slot, float(m_s), int(round(thermal_app.AIRSPEED_HOLD_S * self.HASTE * 1000.0)),
            float(m_s if truth is None else truth))
        return True

    def situation(self, name=None, switching=None):
        """Lay a situation over the ground truth - `SITUATIONS` by name, 'random' for one that is
        not the present one, or 'tour' for the rooms in turn (`TOUR`), moved on by the
        identification's own earned margin - and, with `switching`, turn the random switches
        on or off."""
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
        """A situation by name: the tour's next stop for 'tour', one that is not the present
        one for 'random', or a raise naming them all."""
        if name == 'tour':
            return rooms.next_stop(self._situation)
        if name == 'random':
            return self._random.choice([n for n in self.SITUATIONS if n != self._situation])
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
        self._lib.world_stand_room(self._slot, float(laid['ambient']), float(laid['air']),
                                   float(laid['capacity']))
        if self._switching:
            self._switch_at = time.time() + self._random.uniform(*self.SWITCH_EVERY_S)

    def _tour_step(self, dt):
        """The tour `dt` model s on (coaxial.model.rooms.step)."""
        stable = self.identification_state() == 'STABLE'
        now = self._model_s
        self._earned_s, stop = rooms.step(self._earned_s, stable, dt, now - self._switched_s,
                                          self._situation)
        if stop:
            self._lay(stop)

    def _earned(self):
        """The margin the identification earns alone, before any trip cap."""
        return _u(self._view(stand.IDENT)[-1])

    def identification_state(self):
        """The identification's state alone, by its name."""
        return thermal.IDENT_STATES[int(self._view(stand.IDENT)[0])]

    def truth(self):
        """The ground truth as a page may show it beside the estimate: its situation, the
        scales that make it, how long it has stood, the load the cycle has on it now and what
        it asks this phase - absent on a board, which has no truth to tell."""
        name = self._situation or 'bench'
        laid = self.SITUATIONS[name]
        v = self._view(stand.TRUTH)
        return {'situation': name, 'air': laid['air'], 'capacity': laid['capacity'],
                'ambient': v[23], 'switches': self._switches,
                'since_s': self._model_s - self._switched_s, 'switching': self._switching,
                'tour': self._tour,
                'load_a': v[24] if self._cycle else None,
                'asked_a': v[25] if self._cycle else None}

    def load_cycle(self, amps=CYCLE_AMPS, on_s=CYCLE_ON_S, off_s=CYCLE_OFF_S):
        """A load on and off by the model's own clock: `on_s` at `amps` on all three phases,
        switching, then `off_s` idle, over and over - what a page in simulated mode lays on so
        the map's regions warm and cool and the identification has cooldowns to learn from."""
        if not amps or float(amps) <= 0.0:
            self._cycle = None
            self._lib.world_stand_cycle(self._slot, 0.0, 0.0, 0.0)
            return {'amps': 0.0, 'on_s': 0.0, 'off_s': 0.0}
        if not (float(on_s) > 0.0 and float(off_s) > 0.0):
            raise RigError('a cycle is seconds on and seconds off, both above zero - the '
                           'default is 360 and 840')
        self._cycle = (float(amps), float(on_s), float(off_s))
        self._lib.world_stand_cycle(self._slot, *self._cycle)
        return {'amps': float(amps), 'on_s': float(on_s), 'off_s': float(off_s)}
