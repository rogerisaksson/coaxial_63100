"""Her DC link: her packs and her boards' capacitance under her drives' power, bled by her boards.

    link = dclink.laid()                       # at her packs' open-circuit volts (`drives.PACK_V`)
    dclink.step(link, tau, qvel, vadr, dt)     # her drives' torques at their speeds a step on: link['v']
    dclink.stepped(world, dt)                  # the world's, its step `compiled`; her boards' buses driven

Each step her drives' electrical power - the work they do, braking negative, their copper, each
board's housekeeping and switching - is drawn from her packs through their resistance; what comes
back they take no faster than their BMS's charge, the rest charges her boards' DC link. Solved
implicit in the link's volts (its RC, 0.52 ms, is under a step): the packs' linear share or their
charge's limit, whichever leaves the link higher. Past the text's bleed onset
(`shunt-regenerative-bus-overvoltage`) each board drives its d axis in proportion to its clamp, at
the current its drive's peak leaves its torque's - copper heat, no torque: the text's sinks
(SINKS), unloaded, have the most. Its sinks alone (880 W) let a hop down's landing to 60.78 V.
"""
import math

from machine.body import drives, heat, mechatronics
from machine.texts import kernel
from machine.body.figure import JOINTS
from machine.law.supervisor import NODE_KINDS

_PACK = mechatronics.robot()['pack']
_SHUNT = kernel.node('shunt-regenerative-bus-overvoltage', 'dc-bus')
#: Her packs: their resistance, ohm, their BMS's most charge, A; her boards' DC link, F.
PACK_OHM = _PACK['series'] * _PACK['cell_ohm'] / _PACK['parallel'] / _PACK['packs']
CHARGE_A = _PACK['cell_charge_a'] * _PACK['parallel'] * _PACK['packs']
DRIVEN = [j for j in JOINTS if not drives.passive(j)]
LINK_F = _PACK['board_link_f'] * len(DRIVEN)
#: The bleed's onset and clamp, V; the text's sinks.
ONSET_V, CLAMP_V = _SHUNT['bleed-threshold-v'], _SHUNT['max-voltage-v']
SINKS = [j for j in DRIVEN if drives.kind(j) in {k for node in _SHUNT['priority-sink-nodes'] for k in NODE_KINDS[node]}]


def laid():
    """The link at rest at her packs' open-circuit volts: {v, peak, bled (J), id (A, JOINTS order)}, each
    written in place - v, peak and bled 0-d."""
    import numpy as np
    driven = np.array([not drives.passive(j) for j in JOINTS])
    kt = np.array([drives.kt(j) if d else 1.0 for j, d in zip(JOINTS, driven)])
    ohm = np.array([drives.motors(j) * drives.r_ohm(j) if d else 0.0 for j, d in zip(JOINTS, driven)])
    return {'v': np.array(drives.PACK_V, float), 'peak': np.array(drives.PACK_V, float), 'bled': np.zeros(()),
            'id': np.zeros(len(JOINTS)), 'kt': kt, 'ohm': ohm,
            'amps': np.array([drives.peak(j) / drives.kt(j) if d else 0.0 for j, d in zip(JOINTS, driven)]),
            'loss': ohm / (kt * kt), 'house_w': len(DRIVEN) * (heat.HOUSEKEEPING_W + heat.SWITCHING_W)}


def compiled():
    """`step` compiled on her arrays (`tools.cores.bodied`), its `vadr` and `dt` constants: a world's own."""
    from tools.cores import bodied
    return bodied.on_args(step, 'dclink', (0, 1, 2))


def step(link, tau, qvel, vadr, dt):
    """`link` `dt` s on under her drives' torques `tau` at their speeds `qvel[vadr]`."""
    import numpy as np
    speed, v = qvel[vadr], link['v'][()]
    spare = np.sqrt(np.maximum(link['amps'] ** 2 - (tau / link['kt']) ** 2, 0.0))
    np.multiply(spare, np.clip((v - ONSET_V) / (CLAMP_V - ONSET_V), 0.0, 1.0), out=link['id'])
    bleed = link['ohm'] @ (link['id'] * link['id'])
    load = (tau @ speed + link['loss'] @ (tau * tau) + bleed + link['house_w']) / v
    linear = (LINK_F * v / dt + drives.PACK_V / PACK_OHM - load) / (LINK_F / dt + 1.0 / PACK_OHM)
    limited = v + dt / LINK_F * (-CHARGE_A - load)
    np.maximum(linear, limited, out=link['v'])
    np.maximum(link['peak'], link['v'], out=link['peak'])
    link['bled'] += bleed * dt


def stepped(world, dt):
    """`world`'s link `dt` s on under her drives' torques and speeds (`world.linked`, `compiled`); her
    boards' current loops on their buses driven from it (their back-EMF's headroom, `buses`)."""
    world.linked(world.link, world.motor, world.data.qvel, world.vadr, dt)
    if world.buses is not None:
        world.block.volts[0] = world.link['v'] / math.sqrt(3.0)
