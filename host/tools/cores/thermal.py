"""The thermal/ core through ctypes: the observer, the identification and a ground truth."""
import ctypes
import math
import os

from tools import REPO

THERMAL = os.path.join(REPO, 'thermal')
SOURCES = [os.path.join(THERMAL, 'test', 'harness.c'),
           os.path.join(THERMAL, 'src', 'thermal.c'),
           os.path.join(THERMAL, 'src', 'thermal_ident.c')]

#: The nodes, in the order `thermal.h` declares them. Named here so a
#: failure says `phase_u` and not `3`; the count is asked of the C.
NODES = ('driver_u', 'driver_v', 'driver_w', 'phase_u', 'phase_v', 'phase_w',
         'mcu', 'regulators', 'afe', 'board', 'hotswap',
         'patch_u', 'patch_v', 'patch_w', 'patch_left', 'patch_bottom',
         'patch_right', 'winding', 'stator', 'rotor')

#: The laminate: the centre patch, which an older host reads as `board`,
#: and the six round it. One ceiling for all of them.
LAMINATE = ('board', 'patch_u', 'patch_v', 'patch_w', 'patch_left',
            'patch_bottom', 'patch_right')

#: BUDGET_ORDER, as `harness.c` flattens it.
BUDGET = ('worst', 'worst_node', 'millis', 'throttling', 'tripped', 'derate')

#: The envelope this tree's calibration record carries, stated rather than
#: read: 125 C on every node but the board's copper and the winding, 105 on
#: the copper, 120 on the winding, throttling from 90 % of the span and
#: looking two seconds ahead. Another board's record carries other numbers;
#: the arithmetic is the same.
LIMIT_C, BOARD_LIMIT_C, WINDING_LIMIT_C = 125.0, 105.0, 120.0
THROTTLE_AT, LOOKAHEAD_S = 0.90, 2.0

AMBIENT = 20.0

#: The nodes a current clamp cannot cool: `soa_undriven_mask` in the
#: calibration record, and the record's default. A test passes another mask
#: to change the verdict.
UNDRIVEN = ('mcu', 'regulators', 'afe')


class Model:

    """One observer behind the harness, standing where a test puts it."""

    def __init__(self, lib, celsius=AMBIENT):
        f, p, i = ctypes.c_float, ctypes.c_void_p, ctypes.c_int
        fp = ctypes.POINTER(f)
        lib.thm_new.restype = p
        lib.thm_new.argtypes = [f]
        lib.thm_at.restype = f
        lib.thm_at.argtypes = [p, i]
        lib.thm_capacity.restype = f
        lib.thm_capacity.argtypes = [p, i]
        lib.thm_ntc.restype = f
        lib.thm_ntc.argtypes = [p]
        lib.thm_edge_r.restype = f
        lib.thm_edge_r.argtypes = [p, i]
        lib.thm_set_edge.argtypes = [p, i, f]
        lib.thm_set_board.argtypes = [p, f, f]
        lib.thm_set_rad_board_stator.argtypes = [p, f]
        lib.thm_to_ambient_at.restype = f
        lib.thm_to_ambient_at.argtypes = [p, i, f, f]
        lib.thm_step_at.argtypes = [p, fp, f, f, f, f, f]
        lib.thm_node_derate.restype = f
        lib.thm_node_derate.argtypes = [p, fp, fp, f, f, fp, i]
        lib.thm_junction.restype = f
        lib.thm_junction.argtypes = [p, fp, i]
        lib.thm_coss_energy.restype = f
        lib.thm_coss_energy.argtypes = [f]
        lib.thm_power_r.argtypes = [fp, fp, f, fp]
        self.lib = lib
        self.n = lib.thm_nodes()
        self.slots = lib.thm_budget_slots()
        self.h = ctypes.c_void_p(lib.thm_new(ctypes.c_float(celsius)))
        assert self.n == len(NODES), 'the C has %d nodes, this file %d' % (
            self.n, len(NODES))

    def _floats(self, values):
        return (ctypes.c_float * len(values))(*values)

    def place(self, node, celsius):
        self.lib.thm_place(self.h, ctypes.c_int(NODES.index(node)),
                           ctypes.c_float(celsius))

    def at(self, node):
        return self.lib.thm_at(self.h, NODES.index(node))

    def capacity(self, node):
        return self.lib.thm_capacity(self.h, NODES.index(node))

    def set_node(self, node, to_board, capacity):
        return bool(self.lib.thm_set_node(self.h,
                                          ctypes.c_int(NODES.index(node)),
                                          ctypes.c_float(to_board),
                                          ctypes.c_float(capacity)))

    def ntc(self):
        return self.lib.thm_ntc(self.h)

    def step(self, watt, dt_s, seen=(math.nan, math.nan, math.nan),
             speed_rpm=0.0):
        self.lib.thm_step_at(self.h, self._floats(self._watt(watt)),
                             seen[0], seen[1], seen[2], speed_rpm, dt_s)

    def _watt(self, watt):
        return [float(watt.get(name, 0.0)) for name in NODES]

    def limits(self):
        return [BOARD_LIMIT_C if name in LAMINATE
                else WINDING_LIMIT_C if name == 'winding' else LIMIT_C
                for name in NODES]

    def edge_r(self, edge):
        return self.lib.thm_edge_r(self.h, edge)

    def set_edge(self, edge, k_per_w):
        return bool(self.lib.thm_set_edge(self.h, edge, k_per_w))

    def set_board(self, to_ambient, capacity):
        return bool(self.lib.thm_set_board(self.h, to_ambient, capacity))

    def radiate_to_stator(self, w_per_k):
        self.lib.thm_set_rad_board_stator(self.h, w_per_k)

    def to_ambient_at(self, node, rise_k, speed_rpm=0.0):
        return self.lib.thm_to_ambient_at(self.h, NODES.index(node), rise_k,
                                          speed_rpm)

    def junction(self, watt, node):
        return self.lib.thm_junction(self.h, self._floats(self._watt(watt)),
                                     NODES.index(node))

    def node_derate(self, node, watt=None, throttle_at=THROTTLE_AT,
                    lookahead_s=0.0, limits=None, undriven=UNDRIVEN):
        mask = [1.0 if name in (undriven or ()) else 0.0 for name in NODES]
        return self.lib.thm_node_derate(
            self.h, self._floats(self._watt(watt or {})),
            self._floats(limits or self.limits()), throttle_at, lookahead_s,
            self._floats(mask), NODES.index(node))

    def budget(self, watt=None, throttle_at=THROTTLE_AT, lookahead_s=0.0,
               limits=None, undriven=UNDRIVEN, trip=None):
        """The envelope's verdict."""
        out = (ctypes.c_float * self.slots)()
        mask = [1.0 if name in (undriven or ()) else 0.0 for name in NODES]
        if trip is None:
            self.lib.thm_budget(self.h, self._floats(self._watt(watt or {})),
                                self._floats(limits or self.limits()),
                                ctypes.c_float(throttle_at),
                                ctypes.c_float(lookahead_s),
                                self._floats(mask), out)
        else:
            self.lib.thm_budget_capped(
                self.h, self._floats(self._watt(watt or {})),
                self._floats(limits or self.limits()), self._floats(trip),
                ctypes.c_float(throttle_at), ctypes.c_float(lookahead_s),
                self._floats(mask), out)
        got: dict = dict(zip(BUDGET, list(out)[:len(BUDGET)]))
        got['worst_node'] = NODES[int(got['worst_node'])]
        got['throttling'] = bool(got['throttling'])
        got['tripped'] = bool(got['tripped'])
        got['used'] = dict(zip(NODES, list(out)[6:6 + self.n]))
        got['soak_j'] = dict(zip(NODES, list(out)[6 + self.n:]))
        return got


def power(lib, phase_amps=(0.0, 0.0, 0.0), duty=(0.0, 0.0, 0.0),
          link_volts=24.0, link_amps=-1.0, switching=True, afe_on=False,
          phase_c=None, phase_sq=(0.0, 0.0, 0.0), speed_rpm=0.0,
          t_dead_s=0.0, r_phase=0.0):
    """The estimator's answer, per node, watts."""
    load = (list(phase_amps) + list(duty)
            + [link_volts, link_amps, 1.0 if switching else 0.0,
               1.0 if afe_on else 0.0] + list(phase_sq)
            + [speed_rpm, t_dead_s])
    assert len(load) == lib.thm_load_slots()
    out = (ctypes.c_float * len(NODES))()
    temps = None
    if phase_c is not None:
        temps = (ctypes.c_float * 3)(*phase_c)
    lib.thm_power_r.argtypes = [ctypes.POINTER(ctypes.c_float),
                                ctypes.POINTER(ctypes.c_float),
                                ctypes.c_float,
                                ctypes.POINTER(ctypes.c_float)]
    lib.thm_power_r((ctypes.c_float * len(load))(*load), temps, r_phase, out)
    return dict(zip(NODES, list(out)))


def losses(lib):
    names = ('rds_on', 'rds_alpha', 'r_shunt', 'r_hotswap', 'switching_watt',
             'switch_volts', 'driver_share', 'mcu_watt', 'ldo_watt',
             'afe_watt', 'f_sw', 'coss_cjo', 'coss_m', 'coss_vj',
             't_switch_s', 'v_sd', 'q_g', 'v_drive', 'buck_eff', 'r_phase',
             'k_iron')
    assert len(names) == lib.thm_loss_slots()
    out = (ctypes.c_float * len(names))()
    lib.thm_losses(out)
    return dict(zip(names, list(out)))


def edges(lib):
    """Every edge as `(a, b)` node names, in the C's order."""
    return [(NODES[lib.thm_edge_end(e, 0)], NODES[lib.thm_edge_end(e, 1)])
            for e in range(lib.thm_edges())]


#: The identification's scales and states, in wire order.
SCALES = ('air', 'capacity', 'spread', 'ntc')
STATES = ('UNCERTAIN', 'CONVERGING', 'STABLE')


class Ident:

    """An observer with the identification beside it, behind the harness:
    the scales multiply the observer's defaults, and `run` steps both."""

    def __init__(self, lib, observer, noise_k=0.1):
        f, p, i = ctypes.c_float, ctypes.c_void_p, ctypes.c_int
        fp = ctypes.POINTER(f)
        lib.thm_ident_new.restype = p
        lib.thm_ident_new.argtypes = [p, f]
        lib.thm_ident_run.restype = i
        lib.thm_ident_run.argtypes = [p, p, fp, f, f, f, f, f]
        lib.thm_ident_scale.restype = f
        lib.thm_ident_scale.argtypes = [p, i]
        lib.thm_ident_sigma.restype = f
        lib.thm_ident_sigma.argtypes = [p, i]
        lib.thm_ident_state.restype = i
        lib.thm_ident_state.argtypes = [p]
        lib.thm_ident_innovation.restype = f
        lib.thm_ident_innovation.argtypes = [p]
        lib.thm_ident_updates.restype = i
        lib.thm_ident_updates.argtypes = [p]
        lib.thm_ident_margin.restype = f
        lib.thm_ident_margin.argtypes = [p, f]
        lib.thm_ident_doubt.restype = f
        lib.thm_ident_doubt.argtypes = [p]
        self.lib = lib
        self.net = observer
        self.h = ctypes.c_void_p(lib.thm_ident_new(observer.h, noise_k))

    def run(self, watt, dt_s, seen=(math.nan, math.nan, math.nan),
            speed_rpm=0.0):
        return bool(self.lib.thm_ident_run(
            self.h, self.net.h,
            self.net._floats(self.net._watt(watt)),
            seen[0], seen[1], seen[2], speed_rpm, dt_s))

    def scale(self, which):
        return self.lib.thm_ident_scale(self.h, SCALES.index(which))

    def sigma(self, which):
        return self.lib.thm_ident_sigma(self.h, SCALES.index(which))

    def state(self):
        return STATES[self.lib.thm_ident_state(self.h)]

    def innovation(self):
        return self.lib.thm_ident_innovation(self.h)

    def updates(self):
        return self.lib.thm_ident_updates(self.h)

    def margin(self, floor=0.8):
        """What the envelope keeps of every span for this much evidence,
        `floor`..1 - the number the board acts on."""
        return self.lib.thm_ident_margin(self.h, floor)

    def doubt(self):
        """How far the model is doubted, 0..1: the worse of the
        innovation and the covariance, normalised."""
        return self.lib.thm_ident_doubt(self.h)

    def room_sigma(self):
        """How sure of the room, kelvin - the fifth quantity's sigma."""
        return self.lib.thm_ident_sigma(self.h, 4)

    def ambient(self):
        """The room as identified - the fifth quantity, degrees C."""
        self.lib.thm_ident_ambient.restype = ctypes.c_float
        self.lib.thm_ident_ambient.argtypes = [ctypes.c_void_p]
        return self.lib.thm_ident_ambient(self.h)


class GroundTruth:

    """A board the identification does not know: the same graph with a
    situation laid over it, read through the three thermometers with
    their own noise, at the board's own sampling.
    """

    def __init__(self, lib, air=1.0, capacity=49.0, seed=7):
        self.lib = lib
        self.model = Model(lib)
        self.capacity = capacity
        self.situation(air)
        self.seed = seed

    def situation(self, air):
        self.model.set_board(8.33 * air, self.capacity)

    def noise(self):
        """Deterministic, +-0.05 K: a thermometer's own quantisation."""
        self.seed = (self.seed * 1103515245 + 12345) & 0x7fffffff
        return (self.seed / float(0x7fffffff) - 0.5) * 0.1

    def cycle(self, ident, watt, run_s, cool_s, sample_s=30.0, dt_s=1.0,
              trace=None):
        """One run then one cooldown: the thermometers absent during the
        run (AFE_ON is low while anything switches) and read every
        `sample_s` of the cooldown, as the board reads them."""
        blind = (math.nan, math.nan, math.nan)
        for _ in range(int(run_s / dt_s)):
            self.model.step(watt, dt_s)
            ident.run(watt, dt_s, blind)
        for step in range(int(cool_s / dt_s)):
            self.model.step({}, dt_s)
            seen = blind
            if (step + 1) % int(sample_s / dt_s) == 0:
                seen = (self.model.ntc() + self.noise(),
                        self.model.junction({}, 'afe') + self.noise(),
                        self.model.junction({}, 'mcu') + self.noise())
            if ident.run({}, dt_s, seen) and trace is not None:
                trace.append((ident.state(), ident.scale('air'),
                              ident.sigma('air'), ident.innovation(),
                              ident.margin(), ident.room_sigma()))
