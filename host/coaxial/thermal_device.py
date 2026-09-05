"""The board's thermal observer, behind `0x6E` device 8.

**Measured and estimated are never mixed.** `ntc` is a measurement and only
exists while AFE_ON is high - with it low the channel reads exact mid-scale,
which is not a temperature (invariant 9). The node temperatures are estimates
from power and time. The reply keeps them in separate fields and so does this.

The gate is inverted, so AFE_ON low is exactly when the gate drivers have
supply. While switching there is therefore **no** NTC measurement and the
model runs open on power and time alone. That is the normal case for the
interesting load, not an edge case - `seconds` says how long it has run, and
an estimate under a few minutes has not settled against the network's
6.8-minute constant.
"""
from . import protocol
from .subsystem import Subsystem
from .thermal import ALL_NODES

#: Where the board starts backing off, as a fraction of a node's ceiling.
#: `set_limit`'s default and therefore what is in the record unless a
#: bench wrote something else - named here so a page drawing a margin
#: draws it against the same number the board acts on, rather than one of
#: its own. The board still owns the action; this is only where the bar
#: changes colour.
#:
#: NINETY, FROM EIGHTY-FIVE, on the bench's word: the ramp from here to
#: the ceiling is where the clamp comes off, and at eighty-five it took
#: the last sixth of every node's budget away from a burst the board is
#: there to survive. The record's `soa_throttle_ppm` carries the same
#: number, and a bench that wants more warning writes a smaller one.
THROTTLE_AT = 0.90
from .wire import Reader, pack

#: Op codes. Prefixed because every device has its own op 0 and the bare
#: names collide between modules; `protocol.DEVICE_THERMAL` owns the device
#: byte, so it does not belong here as well.
THERMAL_OP_STATE = 0
THERMAL_OP_SET_NODE = 1
THERMAL_OP_SET_BOARD = 2
THERMAL_OP_SET_SAMPLE = 3
THERMAL_OP_BUDGET = 4
THERMAL_OP_SET_LIMIT = 5
THERMAL_OP_SET_WINDING = 6
THERMAL_OP_NODES = 7
THERMAL_OP_EDGES = 8
THERMAL_OP_SET_EDGE = 9


class Thermal(Subsystem):

    """What each region of the board is at: one measurement, the rest model."""

    def _op(self, op, payload=b'', **kwargs):
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_THERMAL, op]) + payload, **kwargs)

    def state(self):
        """The thermal observer's state.

        `ntc` is None while AFE_ON is low - that is, while the gate drivers
        have supply and switching is running. Not a fault but what the board
        can do: the sensor and the drivers share one switch.

        `error` is the model's own NTC minus the measured one, when both
        exist. It is the only number that says whether the parameters hold.
        """
        r = Reader(self._op(THERMAL_OP_STATE))
        measured = bool(r.u8())
        ntc_centi = r.i32()

        count = r.u8()
        nodes = {}
        for i in range(count):
            value = r.i32() / 100.0
            nodes[ALL_NODES[i] if i < len(ALL_NODES) else 'node%d' % i] = value

        got = {
            'ntc': (ntc_centi / 100.0) if measured else None,
            'nodes': nodes,
            'ambient': r.i32() / 100.0,
            'expected_ntc': r.i32() / 100.0,
            'seconds': r.u32(),
            'settled': bool(r.u8()),
            'sample_every_s': r.u32() / 1000.0,
            'sample_settle_s': r.u32() / 1000.0,
        }

        # The other two thermometers. Each is a die, so each anchors the node
        # it sits ON - not the board. `None` where it did not answer.
        #
        # The flag comes first on the wire and the value always follows, so
        # both are read either way: skipping the read on a false flag would
        # leave the reader one field behind for everything after it.
        for name in ('afe', 'mcu'):
            measured = bool(r.u8())
            value = r.i32() / 100.0
            got[name] = value if measured else None

        got['seen_s_ago'] = r.u32() / 1000.0

        # `seconds` is wall clock and its rate is 1.0 whatever the thermal observer
        # does. `steps` is what a benchmark can watch fall.
        got['steps'] = r.u32()
        got['error'] = ((got['expected_ntc'] - got['ntc'])
                        if got['ntc'] is not None else None)
        # MINOR 13: each leg's FET junction over its node - half the node's
        # watts through R_th,JC - and the rotor speed the air paths were
        # evaluated at. Absent on older firmware, and absent is honest.
        if r.remaining >= 16:
            got['junction_over'] = [r.i32() / 100.0 for _ in range(3)]
            got['speed_rpm'] = r.i32()
        return got

    def network(self):
        """The graph the board runs: every node's capacity, air path, share
        of the face, junction-per-watt and forced-convection gain, and
        every edge's two nodes and K/W - what the record overlays on the
        core's defaults, as the observer holds it now. MINOR 13; a board
        without it refuses the op, and the raise says so.
        """
        nodes = {}
        first = 0
        while True:
            r = Reader(self._op(THERMAL_OP_NODES, pack(('u8', first))))
            count, first, n = r.u8(), r.u8(), r.u8()
            for i in range(first, first + n):
                name = ALL_NODES[i] if i < len(ALL_NODES) else 'node%d' % i
                nodes[name] = {'capacity': r.i32() / 1e3,
                               'to_ambient': r.i32() / 1e3,
                               'area_share': r.i32() / 1e6,
                               'rth_die': r.i32() / 1e3,
                               'forced': r.i32() / 1e3}
            first += n
            if first >= count or n == 0:
                break
        r = Reader(self._op(THERMAL_OP_EDGES))
        edges = []
        for _ in range(r.u8()):
            a, b = r.u8(), r.u8()
            edges.append((ALL_NODES[a] if a < len(ALL_NODES) else 'node%d' % a,
                          ALL_NODES[b] if b < len(ALL_NODES) else 'node%d' % b,
                          r.i32() / 1e3))
        return {'nodes': nodes, 'edges': edges}

    def set_edge(self, edge, k_per_w):
        """One edge's K/W, by index in the table `network()` lists; None
        opens it. Written to the observer and to the record's RAM copy."""
        milli = -1 if k_per_w is None else int(round(k_per_w * 1000))
        return self._ack(THERMAL_OP_SET_EDGE, pack(('u8', int(edge)),
                                                   ('i32', milli)))

    def budget(self):
        """What is left of the thermal budget, per node.

        `used` is a FRACTION, 0 at ambient and 1 at the node's ceiling. A
        temperature cannot say how close a part is without its limit beside
        it, so the board sends the fraction and keeps degrees on `state()`.

        `seconds_to_limit` is what a burst plans on: not how hot it is now,
        but how long it may stay at this power. None when it is not heading
        for a limit at all.

        `tripped` means the board has already dropped MOE. That is an action,
        not a verdict - the estimates are reported either way, and the limits
        came from the calibration record rather than from the firmware.
        """
        r = Reader(self._op(THERMAL_OP_BUDGET))
        used = {}
        for i in range(r.u8()):
            name = ALL_NODES[i] if i < len(ALL_NODES) else 'node%d' % i
            used[name] = r.u8() / 255.0

        worst = r.u8() / 255.0
        index = r.u8()
        millis = r.i32()
        got = {
            'used': used,
            'worst': worst,
            'worst_node': (ALL_NODES[index] if index < len(ALL_NODES)
                           else 'node%d' % index),
            'seconds_to_limit': (millis / 1000.0) if millis >= 0 else None,
            'throttling': bool(r.u8()),
            'tripped': bool(r.u8()),
            'trips': r.u32(),
        }
        # MINOR 11 appended the three below. Read only if they are there:
        # a board on an older firmware answers a shorter frame, and the
        # host selects on the protocol MAJOR alone (invariant 4) so it
        # cannot refuse one for being short.
        if r.remaining >= 4:
            got['derate'] = r.i32() / 1e6
            got['soak_j'] = {}
            for i in range(len(used)):
                name = ALL_NODES[i] if i < len(ALL_NODES) else 'node%d' % i
                got['soak_j'][name] = r.i32() / 1e3
            got['duty'] = [r.i32() / 1e6 for _ in range(3)]
        # MINOR 12: the winding - the one node that is not on the board.
        # Its estimate, its spend against the record's ceiling, and its
        # OWN factor; `derate` above is what the stage got, the smaller
        # of it and the board's, so the two say which envelope holds
        # the stage back. Absent on older firmware, and absent is
        # honest: the page then estimates the winding itself and says
        # so.
        if r.remaining >= 9:
            got['winding_c'] = r.i32() / 100.0
            got['winding_used'] = r.u8() / 255.0
            got['winding_derate'] = r.i32() / 1e6
        return got

    def set_winding(self, limit_c, k_per_w, j_per_k):
        """The winding's envelope: its ceiling in degrees C - zero
        disables it - and its K/W to the air and J/K.

        The board holds these; it does not invent them (invariant 10).
        The record's defaults are the motor profile's placeholder pair
        and an estimated ceiling; a bench with a thermocouple on the
        winding writes what it measured over them.
        """
        return self._ack(THERMAL_OP_SET_WINDING, pack(
            ('i32', int(round(limit_c * 1000))),
            ('i32', int(round(k_per_w * 1000))),
            ('i32', int(round(j_per_k * 1000)))))

    def set_limit(self, node, limit_c, throttle_at=THROTTLE_AT):
        """One node's ceiling in degrees C, and where derating starts.

        The board holds a limit; it does not invent one. Zero disables that
        node's ceiling, which is how a node with no measurement behind its
        limit should be left rather than guessed at.
        """
        index = ALL_NODES.index(node) if isinstance(node, str) else int(node)
        return self._ack(THERMAL_OP_SET_LIMIT, pack(
            ('u8', index),
            ('i32', int(round(limit_c * 1000)),
            ('i32', int(round(throttle_at * 1000000))))))

    def set_sample(self, every_s, settle_s=0.3):
        """How often the thermal observer borrows AFE_ON for an NTC reading.

        It only borrows when nothing else has the rail up. While another
        subsystem holds it the NTC is read every step instead, and this
        setting does not apply.

        `every_s` 0 stops sampling: the model then runs open on power and
        time, which is what it does under a switching run anyway - the rail
        is shared with the gate drivers and an acquire is refused while the
        stage is armed.
        """
        return self._ack(THERMAL_OP_SET_SAMPLE, pack(
            ('u32', int(round(every_s * 1000)),
            ('u32', int(round(settle_s * 1000))))))

    def set_node(self, node, to_board, capacity):
        """Set one node's first path out (K/W) and heat capacity (J/K).

        The path is the node's edge into the laminate under it for a
        source, its air path for a patch or the rotor, its edge into the
        iron for the winding - `thermal_sink_edge`'s rule. The calibration
        behind the defaults was taken **dry** - nothing on the phases,
        nothing drawn through the hot swap. Both change the moment current
        flows: at 100 A the shunt alone makes 35 W against the whole dry
        budget's 1.2 W.

        Re-fitting is one division per node: `(T_zone - T_patch) / P`, with
        T from a camera against a dead patch of soldermask.
        """
        index = ALL_NODES.index(node) if isinstance(node, str) else int(node)
        return self._ack(THERMAL_OP_SET_NODE, pack(
            ('u8', index),
            ('i32', int(round(to_board * 1000)),
            ('i32', int(round(capacity * 1000))))))

    def set_board(self, to_ambient, capacity):
        """The board's own two numbers: K/W to ambient and J/K.

        `to_ambient` is the one figure with a clean measurement behind it -
        8.33 K/W from the passive state against the supply's 50 mA - and also
        the one that moves if the board is ever mounted behind a stator
        instead of lying on a bench. Still air is not a rotor.
        """
        return self._ack(THERMAL_OP_SET_BOARD, pack(
            ('i32', int(round(to_ambient * 1000)),
            ('i32', int(round(capacity * 1000))))))
