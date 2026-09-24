"""The thermal observer without a board: the network thermal.c integrates, on the host."""
import random
from typing import Callable, Optional

from coaxial.devices.thermal_device import ThermalControl
from coaxial.kalman import thermal_ident
from coaxial.model import motor, thermal
from coaxial.simulated.thermal.envelope import ThermalEnvelope
from coaxial.simulated.thermal.record import ThermalRecord
from coaxial.simulated.thermal.truth import ThermalTruth


class SimulatedThermal(ThermalTruth, ThermalEnvelope, ThermalRecord, ThermalControl):
    """The thermal observer without a board: the same twenty-node graph
    `thermal.c` integrates, on `coaxial.model.thermal`'s tables, so a view
    running -Simulated draws the network the board runs and the envelope
    rehearses the same play.
    """

    NODES = thermal.ALL_NODES

    #: The ceiling each node is judged against - the record's defaults:
    #: laminate lower than the rest because it is what everything else sits
    #: on, the motor's three at the winding's.
    LIMIT = dict(thermal.CEILING_C)

    #: The stand-in's clock runs this much faster than the wall: the board's
    #: ~7 min constant shows a load step in half a minute. Only the clock - the
    #: network, capacities and ceilings are `coaxial.model.thermal`'s.
    HASTE = 10.0

    WINDING_K_PER_W = motor.WINDING_K_PER_W
    WINDING_J_PER_K = motor.WINDING_J_PER_K
    WINDING_LIMIT_C = 120.0

    IDENT_NOISE_K = 0.1

    #: The floor the envelope's margin rises from, the record's default
    #: (`thermal.IDENT_MARGIN_FLOOR`); `configure(margin_floor=)` moves it as
    #: thermal op 12 does on the board.
    MARGIN_FLOOR = thermal.IDENT_MARGIN_FLOOR

    def __init__(self, sample=None, situation='bench', seed=7):
        self._seconds = 0
        #: The board's cadence, 30 s (THERMAL_SAMPLE_EVERY_MS). At 5 s a late
        #: cooldown moved under the still rule's 0.3 K and the room never separated
        #: from the air path.
        self._every_s = 30.0
        self._settle_s = 0.3
        #: The sampler: phase currents and whether the bridge switches - what the
        #: board has - and no temperatures.
        self._sample = sample or (lambda: {'amps': (0.0, 0.0, 0.0),
                                           'switching': False})
        #: An rms per phase, tracked across samples.
        self._rms = 0.0
        #: What drops the stage at a ceiling: the board wires the gate drivers.
        self._gate: Optional[Callable[[], bool]] = None
        self._trips = 0
        #: What the effective duty is, asked of whatever owns the compares.
        self._duty: Callable[[], tuple] = lambda: (0.0, 0.0, 0.0)
        #: Where the derate goes. The board wires the drive's clamp.
        self._derate_to: Optional[Callable[[float], None]] = None
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
        self._lay_base()
        # THE GROUND TRUTH: a second board, the base with a situation laid over
        # it, integrated on the same power and read through three noisy
        # thermometers every sample.
        self._random = random.Random(seed)
        self._truth = {n: thermal.AMBIENT for n in self.NODES}
        self._truth_ntc = thermal.AMBIENT
        #: The room the truth stands in, and the observer's ESTIMATE of
        #: it - `thermal.c` infers ambient from the laminate's losses,
        #: there being no sensor for it, and so does the mirror's anchor.
        self._truth_ambient = thermal.AMBIENT
        self._ambient = thermal.AMBIENT
        self._situation: Optional[str] = None
        self._truth_cfg: Optional[dict] = None
        self._switching = False
        self._switch_at = None
        self._tour = False
        self._earned_s = 0.0
        self._switches = 0
        self._model_s = 0.0
        self._switched_s = 0.0
        self._sampled_s = 0.0
        self._since_seen_s = 0.0
        self._seen = {}
        self._settled = False
        self._ident = thermal_ident.Identifier(self.IDENT_NOISE_K,
                                               thermal.AMBIENT)
        # NOTHING BETWEEN RUNS.
        self._margin_floor = self.MARGIN_FLOOR
        #: The trip cap and when it was set, model seconds; one when no
        #: trip is in force.
        self._trip_cap = 1.0
        self._trip_at = 0.0
        #: Whose clock: the wall's until `fast_forward` takes it. A reader advancing
        #: on the wall mid-walk made CI differ, twice (2026-09-06).
        self._driven = False
        #: The load cycle, `(amps, on_s, off_s, began_model_s)` or None:
        #: what the live path samples instead of the drive while one runs.
        self._cycle = None
        self._cycle_trip = None     # the cycle index a trip ended early
        self._cfg = self._ident.apply(self._base)
        self.situation(situation)
        self._start_in_room()

    def state(self):
        self._seconds += 1
        self._advance()
        centre = self._node['board']
        power = self._last_power or {}
        seen = self._seen
        # MEASURED where a sample has been taken - the truth's thermometers -
        # and the observer's own element where none has; the board reports both
        # what the thermistor says and what the model expects.
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
            # MINOR 13: each leg's FET junction over its node - half the node's
            # watts through R_th,JC - and the speed the air saw.
            'junction_over': [0.5 * power.get(n, 0.0)
                              * self._cfg['rth_die'].get(n, 0.0)
                              for n in thermal.DRIVERS],
            'speed_rpm': int(self._speed_rpm),
        }

    def _set_sample(self, every_s, settle_s=0.3):
        self._every_s, self._settle_s = every_s, settle_s
        return True

