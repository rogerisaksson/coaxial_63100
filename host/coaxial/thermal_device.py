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
from .errors import RigError
from .protocol import ThermalOp
from .subsystem import Device
from .thermal import ALL_NODES, IDENT_SCALES, IDENT_STATES, PHASES
from .wire import Reader, label, micro, milli, pack, pages

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

#: `since_save_s` on the wire when the record was never written this boot
#: - always, since MINOR 16: the board keeps nothing it identified.
_NEVER_SAVED = 0xFFFFFFFF

#: An edge's K/W on the wire when the edge is open.
OPEN_EDGE = -1


def _node(index):
    return label(ALL_NODES, index, 'node')


def _index(node):
    """A node by name or by index, as the wire wants it."""
    return ALL_NODES.index(node) if isinstance(node, str) else int(node)


def _thermometer(r):
    """A flag and a centi-degree value, both always on the wire; the value
    counts only when the flag says it was measured.

    Both are read either way: skipping the read on a false flag would
    leave the reader one field behind for everything after it.
    """
    measured = bool(r.u8())
    value = r.centi()
    return value if measured else None


def _node_row(page):
    return {'capacity': page.milli(),
            'to_ambient': page.milli(),
            'area_share': page.micro(),
            'rth_die': page.milli(),
            'forced': page.milli()}


def _edge(r):
    a, b = r.u8(), r.u8()
    return (_node(a), _node(b), r.milli())


class Thermal(Device, device=protocol.DEVICE_THERMAL):

    """What each region of the board is at: one measurement, the rest model."""

    def state(self):
        """The thermal observer's state.

        `ntc` is None while AFE_ON is low - that is, while the gate drivers
        have supply and switching is running. Not a fault but what the board
        can do: the sensor and the drivers share one switch.

        `error` is the model's own NTC minus the measured one, when both
        exist. It is the only number that says whether the parameters hold.
        """
        r = Reader(self._op(ThermalOp.STATE))
        ntc = _thermometer(r)
        nodes = {_node(i): r.centi() for i in range(r.u8())}

        got = {
            'ntc': ntc,
            'nodes': nodes,
            'ambient': r.centi(),
            'expected_ntc': r.centi(),
            'seconds': r.u32(),
            'settled': bool(r.u8()),
            'sample_every_s': r.milli('u32'),
            'sample_settle_s': r.milli('u32'),
        }

        # The other two thermometers. Each is a die, so each anchors the node
        # it sits ON - not the board. `None` where it did not answer.
        for name in ('afe', 'mcu'):
            got[name] = _thermometer(r)

        got['seen_s_ago'] = r.milli('u32')

        # `seconds` is wall clock and its rate is 1.0 whatever the thermal observer
        # does. `steps` is what a benchmark can watch fall.
        got['steps'] = r.u32()
        got['error'] = ((got['expected_ntc'] - ntc)
                        if ntc is not None else None)
        # MINOR 13: each leg's FET junction over its node - half the node's
        # watts through R_th,JC - and the rotor speed the air paths were
        # evaluated at. Absent on older firmware, and absent is honest.
        if r.remaining >= 16:
            got['junction_over'] = [r.centi() for _ in PHASES]
            got['speed_rpm'] = r.i32()
        return got

    def network(self):
        """The graph the board runs: every node's capacity, air path, share
        of the face, junction-per-watt and forced-convection gain, and
        every edge's two nodes and K/W - what the record overlays on the
        core's defaults, as the observer holds it now. MINOR 13; a board
        without it refuses the op, and the raise says so.
        """
        nodes = {_node(i): _node_row(page)
                 for page in pages(self._nodes_from) for i in page.indices()}
        r = Reader(self._op(ThermalOp.EDGES))
        edges = [_edge(r) for _ in range(r.u8())]
        return {'nodes': nodes, 'edges': edges}

    def _nodes_from(self, first):
        return self._op(ThermalOp.NODES, pack(('u8', first)))

    def set_edge(self, edge, k_per_w):
        """One edge's K/W, by index in the table `network()` lists; None
        opens it. Written to the observer and to the record's RAM copy."""
        on_wire = OPEN_EDGE if k_per_w is None else milli(k_per_w)
        return self._ack(ThermalOp.SET_EDGE,
                         pack(('u8', int(edge)), ('i32', on_wire)))

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
        r = Reader(self._op(ThermalOp.BUDGET))
        used = {_node(i): r.fraction() for i in range(r.u8())}

        worst = r.fraction()
        index = r.u8()
        millis = r.i32()
        got = {
            'used': used,
            'worst': worst,
            'worst_node': _node(index),
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
            got['derate'] = r.micro()
            got['soak_j'] = {_node(i): r.milli() for i in range(len(used))}
            got['duty'] = [r.micro() for _ in PHASES]
        # MINOR 12: the winding - the one node that is not on the board.
        # Its estimate, its spend against the record's ceiling, and its
        # OWN factor; `derate` above is what the stage got, the smaller
        # of it and the board's, so the two say which envelope holds
        # the stage back. Absent on older firmware, and absent is
        # honest: the page then estimates the winding itself and says
        # so.
        if r.remaining >= 9:
            got['winding_c'] = r.centi()
            got['winding_used'] = r.fraction()
            got['winding_derate'] = r.micro()
        return got

    def identification(self, **kwargs):
        """The online identification beside the observer (MINOR 14).

        `scales` are multipliers on the record's network - one is the
        derived default; `online` names the ones the samples move (air and
        capacity), the rest ride at the record's values. `state` is
        UNCERTAIN, CONVERGING or STABLE: the model not trusted, tightening,
        or predicting at the thermometers' floor with every online scale
        known to a tenth - a word, since MINOR 16. `margin` is what the
        envelope keeps in hand NOW: continuous between `margin_floor`
        (the record's, 0.8 unless a bench set it) and one, on how far the
        model is doubted; the ceilings' spans are multiplied by it on the
        board, so a page drawing the margin draws the number the board
        acts on. `saves` is 0 and `since_save_s` None on MINOR 16 and
        later - the board keeps nothing it identified; older firmware
        reports what it wrote.
        """
        r = Reader(self._op(ThermalOp.IDENT, **kwargs))
        state = label(IDENT_STATES, r.u8(), 'state')
        mask = r.u8()
        names = [label(IDENT_SCALES, k, 'scale') for k in range(r.u8())]
        scales, sigma = {}, {}
        for name in names:
            scales[name] = r.milli()
            sigma[name] = r.milli()
        got = {'state': state, 'scales': scales, 'sigma': sigma,
               'online': [n for k, n in enumerate(names) if (mask >> k) & 1],
               'innovation_k': r.milli(),
               'margin': r.micro(),
               'updates': r.u32(), 'saves': r.u32()}
        since = r.u32()
        got['since_save_s'] = None if since == _NEVER_SAVED else since
        # MINOR 15: the room as identified beside the scales, and how
        # sure - the board has no ambient sensor. Absent on older
        # firmware, and absent is honest.
        if r.remaining >= 8:
            got['ambient'] = r.centi()
            got['ambient_sigma'] = r.centi()
        # MINOR 16: the floor the margin rises from.
        if r.remaining >= 4:
            got['margin_floor'] = r.micro()
        # MINOR 17: the trip cap as it stands, one with no trip in hand.
        # `margin` is the least of it and the identification's own.
        if r.remaining >= 4:
            got['trip_cap'] = r.micro()
        return got

    def reset_identification(self):
        """Forget what was identified: scales to one, UNCERTAIN, the
        margin back at the floor. Nothing is written anywhere."""
        return self._ack(ThermalOp.IDENT_RESET)

    def set_margin_floor(self, floor):
        """The least of every ceiling's span the envelope keeps while the
        identification has no evidence for its model, a fraction (0, 1];
        the margin rises from here to one as the evidence comes in.

        Through the record, so `cal.save()` persists it beside the
        ceilings - a limit the board is given, not one it invents. The
        bench's default is 0.8; the board refuses zero in its own words,
        since every ceiling would be at 25 C the moment it booted.
        """
        return self._ack(ThermalOp.SET_MARGIN, pack(('i32', micro(floor))))

    def situation(self, name=None, switching=None):
        """A board has no ground truth to put in a situation: that is
        the stand-in's (`SimulatedThermal.situation`), where a box, a
        fan or a heat sink is laid over a hypothetical board for the
        identification to find. Here it is refused in words, so a page
        that asks on the wrong rig hears why rather than AttributeError.
        """
        raise RigError('a board has no ground truth to put in a situation - '
                       'the stand-in has (simulated_device=True): box, fan, '
                       'heatsink, stuffy, bench, or random')

    def load_cycle(self, amps=None, on_s=None, off_s=None):
        """A board's load is the drive's and the bench's to put through
        it - `tools/switch.py`, `tools/pulse.py`, the drive - not the
        observer's to lay on from here: that is the stand-in's
        (`SimulatedThermal.load_cycle`), where a page in simulated mode
        cycles 30 A on and off so the map's regions warm and cool.
        Refused in words on a board, so the page hears why."""
        raise RigError('a board has no load to lay on from the observer - '
                       'the drive and tools/switch.py put current through '
                       'it; the stand-in (simulated_device=True) cycles one')

    def set_winding(self, limit_c, k_per_w, j_per_k):
        """The winding's envelope: its ceiling in degrees C - zero
        disables it - and its K/W to the air and J/K.

        The board holds these; it does not invent them (invariant 10).
        The record's defaults are the motor profile's placeholder pair
        and an estimated ceiling; a bench with a thermocouple on the
        winding writes what it measured over them.
        """
        return self._ack(ThermalOp.SET_WINDING, pack(
            ('i32', milli(limit_c)), ('i32', milli(k_per_w)),
            ('i32', milli(j_per_k))))

    def set_limit(self, node, limit_c, throttle_at=THROTTLE_AT):
        """One node's ceiling in degrees C, and where derating starts.

        The board holds a limit; it does not invent one. Zero disables that
        node's ceiling, which is how a node with no measurement behind its
        limit should be left rather than guessed at.
        """
        return self._ack(ThermalOp.SET_LIMIT, pack(
            ('u8', _index(node)), ('i32', milli(limit_c)),
            ('i32', micro(throttle_at))))

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
        return self._ack(ThermalOp.SET_SAMPLE, pack(
            ('u32', milli(every_s)), ('u32', milli(settle_s))))

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
        return self._ack(ThermalOp.SET_NODE, pack(
            ('u8', _index(node)), ('i32', milli(to_board)),
            ('i32', milli(capacity))))

    def set_board(self, to_ambient, capacity):
        """The board's own two numbers: K/W to ambient and J/K.

        `to_ambient` is the one figure with a clean measurement behind it -
        8.33 K/W from the passive state against the supply's 50 mA - and also
        the one that moves if the board is ever mounted behind a stator
        instead of lying on a bench. Still air is not a rotor.
        """
        return self._ack(ThermalOp.SET_BOARD, pack(
            ('i32', milli(to_ambient)), ('i32', milli(capacity))))
