"""The stand-in's thermal record: the network as held, each setter, the identification."""
from typing import Any

from coaxial.devices.thermal import THROTTLE_AT
from coaxial.errors import RigError
from coaxial.kalman import thermal_ident
from coaxial.model import thermal


class ThermalRecord:

    """The record's thermal fields, read and written as the board's device 9 does."""

    # What the class this mixes into brings.
    IDENT_NOISE_K: Any
    NODES: Any
    _advance: Any
    _ambient: Any
    _base: Any
    _margin: Any
    _trip_cap_now: Any
    truth: Any

    def network(self):
        """The graph as the stand-in holds it - the same shape
        `Thermal.network()` reads off a board.
        """
        nodes = {}
        for name in self.NODES:
            nodes[name] = {'capacity': self._cfg['capacity'].get(name, 0.0),
                           'to_ambient': self._cfg['to_ambient'].get(name, 0.0),
                           'area_share': self._cfg['area_share'].get(name, 0.0),
                           'rth_die': self._cfg['rth_die'].get(name, 0.0),
                           'forced': self._cfg['forced'].get(name, 0.0)}
        edges = [(a, b, r) for (a, b, _r), r
                 in zip(thermal.EDGES, self._cfg['edges'])]
        return {'nodes': nodes, 'edges': edges}

    def _set_limit(self, node, limit_c, throttle_at=THROTTLE_AT):
        return True

    def _set_winding(self, limit_c, k_per_w, j_per_k):
        if k_per_w <= 0.0 or j_per_k <= 0.0:
            raise RigError('the winding needs a positive K/W and J/K; '
                           'a zero ceiling is how it is disabled')
        self.LIMIT = dict(self.LIMIT, winding=float(limit_c))
        self.WINDING_K_PER_W = float(k_per_w)
        self.WINDING_J_PER_K = float(j_per_k)
        self._cfg['capacity']['winding'] = float(j_per_k)
        self._cfg['edges'][thermal.EDGE_WINDING_STATOR] = \
            thermal.WINDING_INTO_IRON * float(k_per_w)
        self._cfg['to_ambient']['stator'] = \
            (1.0 - thermal.WINDING_INTO_IRON) * float(k_per_w)
        return True

    def _set_node(self, node, to_board, capacity):
        """One node's first path out and its capacity - the sink edge for a
        source, the air for a patch, as `thermal_set_node` does.
        """
        if to_board <= 0.0 or capacity <= 0.0:
            raise RigError('a K/W and a heat capacity are both positive')
        edge = thermal.sink_edge(node)
        if edge is not None:
            self._cfg['edges'][edge] = float(to_board)
        else:
            self._cfg['to_ambient'][node] = float(to_board)
        self._cfg['capacity'][node] = float(capacity)
        return True

    def _set_edge(self, edge, k_per_w):
        """One edge's K/W by index; None opens it."""
        self._cfg['edges'][int(edge)] = 0.0 if k_per_w is None \
            else float(k_per_w)
        return True

    def _set_board(self, to_ambient, capacity):
        """The bulk's two numbers, shared out by area as the core does."""
        if to_ambient <= 0.0 or capacity <= 0.0:
            raise RigError('both are positive')
        self._cfg['board_to_ambient'] = float(to_ambient)
        self._cfg['board_capacity'] = float(capacity)
        for name in thermal.LAMINATE:
            share = self._cfg['area_share'][name]
            self._cfg['to_ambient'][name] = float(to_ambient) / share
            self._cfg['capacity'][name] = float(capacity) * share
        return True

    def identification(self):
        """The identification in the wire's shape (`Thermal.identification`),
        off the board's identifier, plus `truth` - the ground truth's
        situation, which no board can report.
        """
        self._advance()
        ident = self._ident
        got = {'state': ident.state,
               'scales': dict((s, ident.scale[k])
                              for k, s in enumerate(thermal.IDENT_SCALES)),
               'sigma': dict((s, ident.sigma(k))
                             for k, s in enumerate(thermal.IDENT_SCALES)),
               'online': [s for k, s in enumerate(thermal.IDENT_SCALES)
                          if thermal_ident.ONLINE[k]],
               'innovation_k': ident.innovation_k,
               'margin': self._margin(),
               'ambient': ident.ambient,
               'ambient_sigma': ident.sigma(thermal_ident.AMBIENT),
               'updates': ident.updates,
               # The wire's two since MINOR 14, and since 16 always none: the
               # board keeps nothing it identified.
               'saves': 0, 'since_save_s': None,
               'margin_floor': self._margin_floor,
               'trip_cap': self._trip_cap_now(),
               'truth': self.truth()}
        return got

    def _reset_identification(self):
        """Forget what was identified: scales to one, UNCERTAIN, the margin
        back at the floor.
        """
        self._ident = thermal_ident.Identifier(self.IDENT_NOISE_K,
                                               self._ambient)
        self._cfg = self._ident.apply(self._base)
        return True

    def _set_margin_floor(self, floor):
        """The floor the margin rises from, a fraction (0, 1] of every
        ceiling's span - thermal op 12, refused in the board's words.
        """
        if not 0.0 < float(floor) <= 1.0:
            raise RigError('the floor is a fraction of the span, 1 .. '
                           '1 000 000 ppm - 800 000 is the bench\'s; zero '
                           'would trip the stage at boot')
        self._margin_floor = float(floor)
        return True
