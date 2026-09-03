"""The power stage stood down: thermal observer, power rails and the
gate drivers with the real arming policy."""
import time

from .. import thermal
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

    def __init__(self, watts=None):
        self._seconds = 0
        self._every_s = 5.0
        self._settle_s = 0.3
        #: What is being dissipated, asked for rather than told: the
        #: drive changes what it is doing between two reads of this and
        #: nothing would have carried the news.
        self._watts = watts or (lambda: dict(thermal.POWER_SWITCHING))
        self._node = {n: thermal.AMBIENT for n in self.NODES}
        self._at = None

    def _advance(self):
        """One first-order step per node, on the model's own network.

        TWO LEVELS, as the network is: the board relaxes toward what the
        total power puts it at, and every source relaxes toward the board
        plus its own rise. `coaxial.thermal`'s CFG holds both the K/W and
        the J/K, so this is that model integrated rather than a second
        one - a stand-in whose temperatures disagreed with `steady()`
        would be worth less than no temperatures at all.
        """
        now = time.time()
        was, self._at = self._at, now
        if was is None:
            return
        dt = min(now - was, 5.0) * self.HASTE
        power = self._watts()
        cfg = thermal.CFG
        total = sum(power.values())
        r_board = cfg['board_to_ambient']
        tau_board = cfg['board_capacity'] * r_board
        board = self._node['board']
        board += (thermal.AMBIENT + total * r_board - board) * \
            min(1.0, dt / tau_board)
        self._node['board'] = board
        for name in thermal.NODES:
            r = cfg['to_board'][name]
            tau = cfg['capacity'][name] * r
            target = board + power.get(name, 0.0) * r
            self._node[name] += (target - self._node[name]) * \
                min(1.0, dt / tau)

    def state(self):
        self._seconds += 1
        self._advance()
        board = self._node['board']
        return {
            'ntc': thermal.expected_ntc(
                board, self._node[thermal.NTC_NEIGHBOUR] - board),
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

    def budget(self):
        self._advance()
        used = {}
        for name in self.NODES:
            top = self.LIMIT.get(name, self.DEFAULT_LIMIT)
            used[name] = max(0.0, (self._node[name] - thermal.AMBIENT)
                             / (top - thermal.AMBIENT))
        worst = max(used, key=lambda n: used[n])
        return {'used': used, 'worst': used[worst], 'worst_node': worst,
                'seconds_to_limit': None,
                'throttling': used[worst] >= 0.85,
                'tripped': used[worst] >= 1.0, 'trips': 0}

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
