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
THROTTLE_AT = 0.85
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
        return got

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
        return {
            'used': used,
            'worst': worst,
            'worst_node': (ALL_NODES[index] if index < len(ALL_NODES)
                           else 'node%d' % index),
            'seconds_to_limit': (millis / 1000.0) if millis >= 0 else None,
            'throttling': bool(r.u8()),
            'tripped': bool(r.u8()),
            'trips': r.u32(),
        }

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
        """Set one node's spreading resistance (K/W) and heat capacity (J/K).

        The calibration behind the defaults was taken **dry** - nothing on the
        phases, nothing drawn through the hot swap. Both change the moment
        current flows: the phase node gains a conduction loss it has never
        had, and at 100 A the shunt alone makes 35 W against the whole dry
        budget's 1.2 W.

        Re-fitting is one division per node: `(T_zone - T_board) / P`, with T
        from a camera against a dead patch of soldermask.
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
