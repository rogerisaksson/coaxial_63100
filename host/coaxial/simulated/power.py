"""The power stage stood down: thermal observer, power rails and the
gate drivers with the real arming policy."""
import math
import time

from .. import thermal
from ..thermal_device import THROTTLE_AT
from ..errors import RigError
from ..gates import GateControl


class SimulatedThermal:
    """The thermal observer without a board.

    Keeps the node order and the field shape of `0x6E` device 8 so a view
    running -Simulated does not crash. Every number is invented, and
    `ntc: None` mirrors the real behaviour: with AFE_ON low there is no
    measurement at all.
    """

    NODES = thermal.ALL_NODES

    #: The ceiling each node is judged against. `board` lower than the
    #: rest because the laminate is what everything else sits on.
    LIMIT = {'board': 105.0}
    DEFAULT_LIMIT = 125.0

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
    #: it defaults to. Not a distance to project a temperature - see
    #: `_soon` for why that shape was taken out.
    LOOKAHEAD_S = 2.0

    #: How fast the derate may recover, per second of model time.
    #: `THERMAL_DERATE_RECOVER_PER_S` in the firmware.
    DERATE_RECOVER_PER_S = 0.05

    #: The longest slice the integrator takes, seconds of model time.
    #: `THERMAL_STEP_MS` in the firmware, and the same number for the same
    #: reason: the envelope is evaluated once per step, and a step longer
    #: than the throttle's ramp steps straight over it.
    #:
    #: IT WAS ONE SECOND, chosen as a fifth of the fastest node's constant
    #: - a driver is 0.35/3 J/K across 45.6 K/W, about five seconds - and
    #: that is the right rule for INTEGRATING and the wrong one for
    #: ACTING. The ramp is the last `lookahead_s * (1 - throttle_at)` of a
    #: node's hold, 300 ms at the record's numbers, so a one second step
    #: cannot land inside it. Measured 2026-09-03 on the stand-in at 90 A:
    #: 20.0 C on one poll and 158.6 C on the next, 33 K past a 125 C
    #: ceiling, derate 1.00 then 0.00, and the trip - the drive killed
    #: rather than throttled, which is the one thing the envelope is
    #: there to avoid.
    STEP_S = 0.1

    def __init__(self, sample=None):
        self._seconds = 0
        self._every_s = 5.0
        self._settle_s = 0.3
        #: WHERE IT LOOKS, not what it is told. A sampler that answers
        #: the phase currents and whether the bridge is switching - the
        #: two things this board has - and nothing about how hot
        #: anything is or is going to be. Without one it sees a stage
        #: that is not switching, which is what an unplugged observer
        #: should see.
        self._sample = sample or (lambda: {'amps': (0.0, 0.0, 0.0),
                                           'switching': False})
        #: An rms per phase, tracked across samples. A single sample of a
        #: three-phase current says where the vector is pointing, not how
        #: big it has been: the observer squares and leaks, which is what
        #: a firmware one does between its own reads.
        self._rms = 0.0
        #: WHAT IT DROPS WHEN A NODE REACHES ITS CEILING. The board wires
        #: the gate drivers' own disable here, which is the same path the
        #: break uses; without one this observer reports and does not act,
        #: which is what it did before and is not what the board does.
        self._gate = None
        self._trips = 0
        #: What the effective duty is, asked of whatever owns the
        #: compares. The board wires the gate drivers' own; without one
        #: there is no stage and the answer is zeros.
        self._duty = lambda: (0.0, 0.0, 0.0)
        #: Where the derate goes. The board wires the drive's clamp.
        self._derate_to = None
        self._last_power = None
        self._derate_held = 1.0
        self._derate_at = None
        self._node = {n: thermal.AMBIENT for n in self.NODES}
        #: THE READING, LAGGED. See `thermal.NTC_TAU_S`: the algebra had
        #: no mass, so a modelled thermistor followed a small fast node
        #: instantly and the page showed one climbing like silicon.
        #: No offset: the 6.0 K is an instrument disagreement, recorded
        #: rather than added to a temperature. See `thermal.NTC_OFFSET`.
        self._ntc = thermal.AMBIENT
        self._at = None

    def _advance(self):
        """The network integrated forward to now, in steps it can take.

        SUB-STEPPED, and it has to be. One explicit step across a whole
        poll gap was what made the temperatures move in stairs: the view
        reads this every couple of seconds, ten times hurried is twenty
        seconds of model time, and the fastest node's constant is five -
        so `dt / tau` clamped at one and the node JUMPED to wherever the
        power put it, waited, and jumped again. A first-order step is
        only first order while it is small against the constant it is
        stepping.

        The same lesson `_advance_model` has next door for the rotor, and
        for the same reason: an Euler step the size of the thing it is
        integrating is not an integration.

        Reading it twice in a row is also harmless now - the second read
        finds no elapsed time and does nothing, where before it was the
        one that got the whole step and its neighbour got none.
        """
        now = time.time()
        was, self._at = self._at, now
        if was is None:
            return
        elapsed = min(now - was, 5.0)
        if elapsed <= 0.0:
            return
        left = elapsed * self.HASTE
        while left > 0.0:
            step = min(self.STEP_S, left)
            left -= step
            # SAMPLED EVERY SLICE, not once for the gap. It was once - "what
            # the drive is doing is what it is doing now, and sampling it
            # again inside the loop would be reading the same value and
            # pretending it was news" - and that was true only while nothing
            # inside the loop could change the current. The envelope below
            # changes it: it writes the clamp into the drive, and `_dq`
            # applies it. Frozen, the model went on integrating the
            # pre-throttle current for the rest of the gap and cooked the
            # node the throttle had already backed off - measured at 90 A,
            # the stage tripped at a reported spend of 0.756 because the
            # peak happened between two polls.
            self._integrate(step, self._sample())
            # THE ENVELOPE INSIDE THE LOOP, not after it. `board_thermal.c`
            # runs the budget every THERMAL_STEP_MS - one step, one look -
            # and a stand-in that integrated a whole poll gap and then
            # looked once was not rehearsing that. It was a model that
            # could only ever see the aftermath: at 90 A it ran 2.5 s of
            # model time at full current, overshot the ceiling by 33 K,
            # and the first evaluation it made had nothing left to do but
            # trip. The throttle needs to be asked while there is still
            # something to throttle.
            self._envelope()

    def _envelope(self):
        """THE ONE PLACE THIS CLASS ACTS RATHER THAN REPORTS.

        A trip drops the stage, and the estimate is reported either way.
        `board_thermal.c` does exactly this after every step of the real
        observer - `if (s_budget.tripped && Board_PwmIsEnabled())
        Board_PwmDisable();` - and a stand-in whose observer watched a
        node go past its ceiling and did nothing would be a stand-in you
        could not rehearse the envelope against. The limits come from the
        record; nothing here decides one (invariant 10).

        Latched by construction, as it is there: dropping the stage is
        not a state this class holds, it is a thing it does, and only an
        arm brings the gates back.
        """
        # FIRST IT DERATES. Past the throttle point the drive's clamp is
        # scaled toward zero, so the stage keeps driving on less - which
        # is what an envelope is for, and what `board_thermal.c` does one
        # line before it considers dropping anything. Without it the
        # envelope was a cliff and the page went dead at the first trip.
        worst = self._worst()[0]
        if self._derate_to is not None:
            self._derate_to(self._derate_applied(self.derate(worst)))
        # THEN, only if that was not enough.
        if self._gate is None or worst < 1.0:
            return
        if self._gate():
            self._trips += 1

    def _integrate(self, dt, seen):
        """One first-order step per node, on the model's own network.

        TWO LEVELS, as the network is: the board relaxes toward what the
        total power puts it at, and every source relaxes toward the board
        plus its own rise. `coaxial.thermal`'s CFG holds both the K/W and
        the J/K, so this is that model integrated rather than a second
        one - a stand-in whose temperatures disagreed with `steady()`
        would be worth less than no temperatures at all.
        """
        power = self._power(dt, seen)
        #: Kept so the lookahead can project without sampling again.
        self._last_power = power
        cfg = thermal.CFG
        total = sum(power.values())
        board = self._node['board']
        # AT THE RISE IT IS CARRYING. The path off the board is
        # convection and radiation, and neither is linear in the
        # rise - `thermal.board_to_ambient_at` has the two shapes.
        # Frozen at the calibration rise the stand-in ran the board
        # ten to fifteen kelvin hot at the powers a burst makes.
        r_board = thermal.board_to_ambient_at(board - thermal.AMBIENT)
        tau_board = cfg['board_capacity'] * r_board
        board += (thermal.AMBIENT + total * r_board - board) * \
            min(1.0, dt / tau_board)
        self._node['board'] = board
        for name in thermal.NODES:
            r = cfg['to_board'][name]
            tau = cfg['capacity'][name] * r
            target = board + power.get(name, 0.0) * r
            self._node[name] += (target - self._node[name]) * \
                min(1.0, dt / tau)
        # THE READING FOLLOWS THE NODES, it does not jump with them.
        # `thermal.NTC_TAU_S` has why: the algebra had no mass and a
        # modelled thermistor climbed like the silicon it watches.
        want = thermal.expected_ntc(
            board, self._node[thermal.NTC_NEIGHBOUR] - board)
        self._ntc += (want - self._ntc) * min(1.0, dt / thermal.NTC_TAU_S)

    def _power(self, dt, seen):
        """Watts per node, worked out from the sample. The observer's job.

        `i^2 R` on what the shunts actually carried, across the
        resistance the current crosses - `inverter` holds the FET's
        Rds(on) and the shunt - plus the housekeeping, which does not
        care whether anything switches. `coaxial.thermal.phase_power` is
        that split, and it is the same function the bench arithmetic and
        the notebooks use.

        The rms is tracked rather than taken from one sample: three
        phase currents at an instant are a vector, and a vector says
        nothing about how long it has been that big. It leaks toward the
        instantaneous magnitude at `RMS_TAU`, which is what a firmware
        observer does between reads.
        """
        from .. import inverter

        amps = seen.get('amps') or (0.0, 0.0, 0.0)
        now = math.sqrt(sum(a * a for a in amps) / 3.0)
        self._rms += (now - self._rms) * min(1.0, dt / self.RMS_TAU)
        if not seen.get('switching'):
            self._rms *= max(0.0, 1.0 - dt / self.RMS_TAU)
        return thermal.phase_power(self._rms,
                                   inverter.RDS_ON + inverter.SHUNT,
                                   switching=bool(seen.get('switching')))

    def state(self):
        self._seconds += 1
        self._advance()
        board = self._node['board']
        return {
            # LAGGED, where `expected_ntc` below is the algebra it heads
            # for. The board reports both for the same reason: what the
            # thermistor says and what the model expects it to say are two
            # facts, and their difference is the observer's report card.
            'ntc': self._ntc,
            'nodes': dict(self._node),
            'ambient': thermal.AMBIENT,
            'expected_ntc': thermal.expected_ntc(
                board, self._node[thermal.NTC_NEIGHBOUR] - board),
            'seconds': self._seconds,
            'settled': True,
            'sample_every_s': self._every_s,
            'sample_settle_s': self._settle_s,
            'afe': self._node['afe'],
            'mcu': self._node['mcu'],
            'seen_s_ago': 0.4,
            'steps': 1200,
            'error': 0.0,
        }

    def set_sample(self, every_s, settle_s=0.3):
        self._every_s, self._settle_s = every_s, settle_s
        return True

    def _used(self):
        """Each node as a fraction of its own ceiling.

        One definition: `budget()` answers it and `_envelope()` acts on
        it, and the two disagreeing about how close a node is would be a
        stage that trips at a number nobody reported.
        """
        used = {}
        for name in self.NODES:
            top = self.LIMIT.get(name, self.DEFAULT_LIMIT)
            used[name] = max(0.0, (self._node[name] - thermal.AMBIENT)
                             / (top - thermal.AMBIENT))
        return used

    def _worst(self):
        used = self._used()
        name = max(used, key=lambda n: used[n])
        return used[name], name, used

    def derate(self, worst=None):
        """What the current clamp should be multiplied by, 1 down to 0.

        One at the throttle point and zero at the ceiling, linear
        between - `thermal.c`'s own arithmetic, which is why the two
        agree about when a stage backs off and by how much.
        """
        spent = self._worst()[0] if worst is None else worst
        # ON THE WORSE OF NOW AND SOON, as `thermal.c` does. A node at
        # 45 A crosses the whole throttle band between two polls, so a
        # throttle reading only the present never sees it: measured, the
        # derate stayed at 1.0 through a crossing from a fifth of the
        # budget to over the ceiling, and the stage tripped instead.
        spent = max(spent, self._soon())
        band = 1.0 - THROTTLE_AT
        if spent <= THROTTLE_AT or band <= 0.0:
            return 1.0
        return max(0.0, 1.0 - (spent - THROTTLE_AT) / band)

    def _derate_applied(self, want):
        """The factor after the recovery slew. Down is immediate.

        ASYMMETRIC ON PURPOSE, and `board_thermal.c` does the same. The
        factor is part of a loop: cut the clamp and the ramp goes away,
        so the next look sees no ramp and asks for full current again.
        Measured, that oscillated between 1.00 and 0.00 every hundred
        milliseconds with the node at nine tenths of its ceiling - a
        stage chattering at its own poll rate, which is worse for the
        silicon than the derate was for. Recovering over a few seconds
        gives the node time to actually cool first.
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

    def _soon(self):
        """How far into the last `LOOKAHEAD_S` of hold the worst node is.

        THE SAME ARITHMETIC `thermal.c` DOES, and it has to be: a
        stand-in that throttles on a different rule from the board is a
        rehearsal of the wrong play. `hold` is the soak over the power
        spending it - seconds this node can stay here - and the fraction
        is how much of the window has gone.

        IT WAS A PROJECTED TEMPERATURE, forward `LOOKAHEAD_S` at the
        present rate, and that shape fails on the case this board is
        for. Measured 2026-09-03: 100 A puts 18.4 W into a driver node
        of 0.12 J/K, 0.67 s from ambient to its ceiling, and a two
        second projection lands past it from a cold board - the clamp
        went to zero before the burst began. Time does not do that: a
        node at ambient has its whole soak in front of it however much
        power is on it, so the burst runs and what closes the clamp is
        the hold falling into the window.
        """
        power = self._last_power or {}
        cfg = thermal.CFG
        worst = 0.0
        for name in self.NODES:
            top = self.LIMIT.get(name, self.DEFAULT_LIMIT)
            capacity = (cfg['board_capacity'] if name == 'board'
                        else cfg['capacity'].get(name, 0.0))
            if capacity <= 0.0 or top <= thermal.AMBIENT:
                continue
            r = (thermal.board_to_ambient_at(self._node['board']
                                             - thermal.AMBIENT)
                 if name == 'board' else cfg['to_board'].get(name, 0.0))
            reference = (thermal.AMBIENT if name == 'board'
                         else self._node['board'])
            net = (power.get(name, 0.0) - (self._node[name] - reference) / r
                   if r > 0.0 else power.get(name, 0.0))
            if net <= 0.0:
                continue                # not heading anywhere warmer
            togo = top - self._node[name]
            hold = (togo * capacity / net) if togo > 0.0 else 0.0
            worst = max(worst, min(1.0, 1.0 - hold / self.LOOKAHEAD_S))
        return worst

    def soak_j(self):
        """Joules each node can still absorb before its ceiling.

        `capacity x (limit - t)`, and never negative: a node past its
        ceiling has no budget rather than a debt. It is the quantity a
        control system plans a burst with - divide by the power it means
        to spend and the answer is seconds, at any power rather than
        only the present one.
        """
        cfg = thermal.CFG
        out = {}
        for name in self.NODES:
            top = self.LIMIT.get(name, self.DEFAULT_LIMIT)
            capacity = (cfg['board_capacity'] if name == 'board'
                        else cfg['capacity'].get(name, 0.0))
            out[name] = max(0.0, capacity * (top - self._node[name]))
        return out

    def budget(self):
        self._advance()
        worst, name, used = self._worst()
        return {'used': used, 'worst': worst, 'worst_node': name,
                'seconds_to_limit': None,
                'throttling': worst >= THROTTLE_AT,
                'tripped': worst >= 1.0, 'trips': self._trips,
                'derate': self._derate_held, 'soak_j': self.soak_j(),
                'duty': list(self._duty() or (0.0, 0.0, 0.0))}

    def set_limit(self, node, limit_c, throttle_at=0.85):
        return True

    def set_node(self, node, to_board, capacity):
        return True

    def set_board(self, to_ambient, capacity):
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
