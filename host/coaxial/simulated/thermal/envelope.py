"""The stand-in's envelope: each node's spend, the derate, the hold, the trip, the budget."""
from coaxial.devices.thermal_device import THROTTLE_AT
from coaxial.model import thermal


class ThermalEnvelope:

    """Where the envelope acts: the one place the observer changes what the drive may do."""

    DEFAULT_LIMIT = thermal.CEILING_DEFAULT_C

    #: Nodes the current clamp cannot cool (`soa_undriven_mask`): they draw the
    #: same watts at any duty. Counted in, an idle board spent 0.30 of its budget
    #: on a cold bench (MCU 49.1 C, regulators 51.1 C, 2026-09-04).
    UNDRIVEN = ('mcu', 'regulators', 'afe')

    #: The window of remaining hold the throttle insists on, seconds.
    #: `soa_lookahead_ms` in the board's record, and the same two seconds
    #: it defaults to.
    LOOKAHEAD_S = 2.0

    #: How fast the derate may recover, per second of model time.
    #: `THERMAL_DERATE_RECOVER_PER_S` in the firmware.
    DERATE_RECOVER_PER_S = 0.05

    #: THE TRIP CAP (`board_thermal.c`): after a trip the margin holds at 70 %,
    #: recovering a percent a model minute; every trip restarts it (bench
    #: 2026-09-06: "graceful degradation").
    TRIP_MARGIN = 0.70

    TRIP_RECOVER_PER_S = 0.30 / 1800.0

    def _envelope(self):
        """THE ONE PLACE THIS CLASS ACTS RATHER THAN REPORTS."""
        # FIRST IT DERATES.
        applied = self._derate_applied(self.derate())
        if self._derate_to is not None:
            self._derate_to(applied)
        # THEN, only if that was not enough - AND ON EVERY NODE, not just the
        # ones the clamp reaches.
        if self._gate is None or not self._tripped():
            return
        if self._gate():
            self._trips += 1
            # AND THE ENVELOPE SHRINKS, as on the board: the trip cap from now,
            # recovering a percent a minute of model time.
            self._trip_cap = self.TRIP_MARGIN
            self._trip_at = self._model_s

    def _trip_cap_now(self):
        """The trip cap as it stands: set at a trip, given back at
        TRIP_RECOVER_PER_S, one with no trip in hand.
        """
        if self._trip_cap >= 1.0:
            return 1.0
        return min(1.0, self._trip_cap
                   + (self._model_s - self._trip_at) * self.TRIP_RECOVER_PER_S)

    def _margin(self):
        """The margin the envelope acts on now: the identification's for its
        doubt, or the trip cap as it stands - whichever keeps more in
        hand.
        """
        return min(self._ident.margin(self._margin_floor), self._trip_cap_now())

    def _limit(self, name):
        """One node's ceiling as the envelope acts on it: the record's, its
        span over the reference trimmed by the margin - the floor while
        the model is doubted whole, one when not at all, the trip cap
        after a trip - as `board_thermal.c` trims it, so the silicon and
        the laminate are not run to ceilings computed on a network just
        proved wrong.
        """
        top = self.LIMIT.get(name, self.DEFAULT_LIMIT)
        return thermal.AMBIENT + self._margin() * (top - thermal.AMBIENT)

    def _used(self):
        """Each node as a fraction of its own ceiling, FROM THE ROOM the
        observer believes it stands in, clamped to 0..1 -
        `thermal_budget` in the C, line for line.
        """
        used = {}
        for name in self.NODES:
            span = self._limit(name) - self._ambient
            if not span > 0.0:
                used[name] = 0.0
                continue
            part = (self._node[name] - self._ambient) / span
            used[name] = max(0.0, min(1.0, part))
        return used

    def _worst(self):
        """The worst node the clamp can reach, and every node's spend."""
        used = self._used()
        driven = [n for n in used if n not in self.UNDRIVEN] or list(used)
        name = max(driven, key=lambda n: used[n])
        return used[name], name, used

    def _tripped(self):
        """Whether ANY node is at the RECORD'S ceiling, driven or not -
        `thermal_budget`'s `trip_c`, untrimmed.
        """
        for name in self.NODES:
            top = self.LIMIT.get(name, self.DEFAULT_LIMIT)
            if top > self._ambient and self._node[name] >= top:
                return True
        return False

    def derate(self, worst=None):
        """What the current clamp should be multiplied by, 1 down to 0: one
        at the throttle point and zero at the ceiling, linear between, on
        the worse of where the worst node is and how far into the window
        any driven node's hold has come - `thermal.c`'s own arithmetic.
        """
        spent = self._worst()[0] if worst is None else worst
        spent = max(spent, self._soon())
        return self._ramp(spent)

    @staticmethod
    def _ramp(spent):
        """One at the throttle point, zero at the ceiling, linear between."""
        band = 1.0 - THROTTLE_AT
        if spent <= THROTTLE_AT or band <= 0.0:
            return 1.0
        return max(0.0, 1.0 - (spent - THROTTLE_AT) / band)

    def _derate_applied(self, want):
        """The factor after the recovery slew. Down is immediate."""
        # On model time: on the wall's, two runs of one notebook walk moved
        # the tour at different minutes (2026-09-06).
        now = self._model_s
        was, self._derate_at = self._derate_at, now
        if want <= self._derate_held or was is None:
            self._derate_held = want
        else:
            step = self.DERATE_RECOVER_PER_S * min(5.0 * self.HASTE, now - was)
            self._derate_held = min(want, self._derate_held + step)
        self._derate_held = max(0.0, min(1.0, self._derate_held))
        return self._derate_held

    def _hold(self, name):
        """Seconds this node can stay at its net power before its ceiling -
        the soak over what is going into it - or None when it is not
        heading there.
        """
        net = (self._last_net or {}).get(name, 0.0)
        capacity = self._cfg['capacity'].get(name, 0.0)
        top = self._limit(name)
        if net <= 0.0 or capacity <= 0.0 or top <= self._ambient:
            return None
        togo = top - self._node[name]
        return (togo * capacity / net) if togo > 0.0 else 0.0

    def _soon(self):
        """How far into the last `LOOKAHEAD_S` of hold the worst driven node
        is - THE SAME ARITHMETIC `thermal.c` DOES: time left, not a
        projected temperature, so a node at ambient has its whole soak in
        front of it and a burst runs.
        """
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
        """Joules each node can still absorb before its ceiling: `capacity x
        (limit - t)`, never negative.
        """
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
                # MINOR 12: the winding's estimate, spend and OWN factor, from
                # the node it is, beside `derate` - the stage's.
                'winding_c': self._node['winding'],
                'winding_used': used['winding'],
                'winding_derate': self.node_derate('winding')}
