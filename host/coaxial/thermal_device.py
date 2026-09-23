"""The board's thermal observer, behind `0x6E` device 8."""
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
        """The thermal observer's state."""
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

        # The other two thermometers.
        for name in ('afe', 'mcu'):
            got[name] = _thermometer(r)

        got['seen_s_ago'] = r.milli('u32')

        # `seconds` is wall clock and its rate is 1.0 whatever the thermal
        # observer does.
        got['steps'] = r.u32()
        got['error'] = ((got['expected_ntc'] - ntc)
                        if ntc is not None else None)
        # MINOR 13: each leg's FET junction over its node - half the node's
        # watts through R_th,JC - and the rotor speed the air paths were
        # evaluated at.
        if r.remaining >= 16:
            got['junction_over'] = [r.centi() for _ in PHASES]
            got['speed_rpm'] = r.i32()
        return got

    def network(self):
        """The graph the board runs: every node's capacity, air path, share of
        the face, junction-per-watt and forced-convection gain, and every
        edge's two nodes and K/W - what the record overlays on the core's
        defaults, as the observer holds it now. MINOR 13; a board without it
        refuses the op, and the raise says so.
        """
        nodes = {_node(i): _node_row(page)
                 for page in pages(self._nodes_from) for i in page.indices()}
        r = Reader(self._op(ThermalOp.EDGES))
        edges = [_edge(r) for _ in range(r.u8())]
        return {'nodes': nodes, 'edges': edges}

    def _nodes_from(self, first):
        return self._op(ThermalOp.NODES, pack(('u8', first)))

    def set_edge(self, edge, k_per_w):
        """One edge's K/W, by index in the table `network()` lists; None opens
        it. Written to the observer and to the record's RAM copy.
        """
        on_wire = OPEN_EDGE if k_per_w is None else milli(k_per_w)
        return self._ack(ThermalOp.SET_EDGE,
                         pack(('u8', int(edge)), ('i32', on_wire)))

    def budget(self):
        """What is left of the thermal budget, per node."""
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
        # MINOR 11 appended the three below.
        if r.remaining >= 4:
            got['derate'] = r.micro()
            got['soak_j'] = {_node(i): r.milli() for i in range(len(used))}
            got['duty'] = [r.micro() for _ in PHASES]
        # MINOR 12: the winding - the one node that is not on the board.
        if r.remaining >= 9:
            got['winding_c'] = r.centi()
            got['winding_used'] = r.fraction()
            got['winding_derate'] = r.micro()
        return got

    def identification(self, **kwargs):
        """The online identification beside the observer (MINOR 14)."""
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
        # MINOR 15: the room as identified beside the scales, and how sure -
        # the board has no ambient sensor.
        if r.remaining >= 8:
            got['ambient'] = r.centi()
            got['ambient_sigma'] = r.centi()
        # MINOR 16: the floor the margin rises from.
        if r.remaining >= 4:
            got['margin_floor'] = r.micro()
        # MINOR 17: the trip cap as it stands, one with no trip in hand.
        if r.remaining >= 4:
            got['trip_cap'] = r.micro()
        return got

    def reset_identification(self):
        """Forget what was identified: scales to one, UNCERTAIN, the margin
        back at the floor. Nothing is written anywhere.
        """
        return self._ack(ThermalOp.IDENT_RESET)

    def set_margin_floor(self, floor):
        """The least of every ceiling's span the envelope keeps while the
        identification has no evidence for its model, a fraction (0, 1]; the
        margin rises from here to one as the evidence comes in.
        """
        return self._ack(ThermalOp.SET_MARGIN, pack(('i32', micro(floor))))

    def situation(self, name=None, switching=None):
        """A board has no ground truth to put in a situation: that is the
        stand-in's (`SimulatedThermal.situation`), where a box, a fan or a
        heat sink is laid over a hypothetical board for the identification
        to find. Here it is refused in words, so a page that asks on the
        wrong rig hears why rather than AttributeError.
        """
        raise RigError('a board has no ground truth to put in a situation - '
                       'the stand-in has (simulated_device=True): box, fan, '
                       'heatsink, stuffy, bench, or random')

    def load_cycle(self, amps=None, on_s=None, off_s=None):
        """A board's load is the drive's and the bench's to put through it -
        `tools/switch.py`, `tools/pulse.py`, the drive - not the observer's
        to lay on from here: that is the stand-in's
        (`SimulatedThermal.load_cycle`), where a page in simulated mode
        cycles 30 A on and off so the map's regions warm and cool. Refused
        in words on a board, so the page hears why.
        """
        raise RigError('a board has no load to lay on from the observer - '
                       'the drive and tools/switch.py put current through '
                       'it; the stand-in (simulated_device=True) cycles one')

    def set_winding(self, limit_c, k_per_w, j_per_k):
        """The winding's envelope: its ceiling in degrees C - zero disables it
        - and its K/W to the air and J/K.
        """
        return self._ack(ThermalOp.SET_WINDING, pack(
            ('i32', milli(limit_c)), ('i32', milli(k_per_w)),
            ('i32', milli(j_per_k))))

    def set_limit(self, node, limit_c, throttle_at=THROTTLE_AT):
        """One node's ceiling in degrees C, and where derating starts."""
        return self._ack(ThermalOp.SET_LIMIT, pack(
            ('u8', _index(node)), ('i32', milli(limit_c)),
            ('i32', micro(throttle_at))))

    def set_sample(self, every_s, settle_s=0.3):
        """How often the thermal observer borrows AFE_ON for an NTC reading."""
        return self._ack(ThermalOp.SET_SAMPLE, pack(
            ('u32', milli(every_s)), ('u32', milli(settle_s))))

    def set_node(self, node, to_board, capacity):
        """Set one node's first path out (K/W) and heat capacity (J/K)."""
        return self._ack(ThermalOp.SET_NODE, pack(
            ('u8', _index(node)), ('i32', milli(to_board)),
            ('i32', milli(capacity))))

    def set_board(self, to_ambient, capacity):
        """The board's own two numbers: K/W to ambient and J/K."""
        return self._ack(ThermalOp.SET_BOARD, pack(
            ('i32', milli(to_ambient)), ('i32', milli(capacity))))
