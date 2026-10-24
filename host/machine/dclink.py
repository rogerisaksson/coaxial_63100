"""Her DC link: her packs and her boards' capacitance under her drives' power, bled by her boards.

    link = dclink.laid()                       # at her packs' open-circuit volts (`drives.PACK_V`)
    dclink.step(world, dt)                     # her drives' torques and speeds a step on: world.link['v']

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

from machine import drives, heat, kernel, mechatronics
from machine.figure import JOINTS
from machine.supervisor import NODE_KINDS

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
    """The link at rest at her packs' open-circuit volts: {v, peak, bled (J), id (A, JOINTS order)}."""
    import numpy as np
    driven = np.array([not drives.passive(j) for j in JOINTS])
    kt = np.array([drives.kt(j) if d else 1.0 for j, d in zip(JOINTS, driven)])
    ohm = np.array([drives.motors(j) * drives.r_ohm(j) if d else 0.0 for j, d in zip(JOINTS, driven)])
    return {'v': drives.PACK_V, 'peak': drives.PACK_V, 'bled': 0.0, 'id': np.zeros(len(JOINTS)), 'kt': kt, 'ohm': ohm,
            'amps': np.array([drives.peak(j) / drives.kt(j) if d else 0.0 for j, d in zip(JOINTS, driven)]),
            'loss': ohm / (kt * kt), 'house_w': len(DRIVEN) * (heat.HOUSEKEEPING_W + heat.SWITCHING_W)}


def step(world, dt):
    """`world`'s link `dt` s on under her drives' torques and speeds; her boards' current loops on
    their buses driven from it (their back-EMF's headroom, `buses`)."""
    import numpy as np
    link, tau, speed = world.link, world.motor, world.data.qvel[world.vadr]
    v = link['v']
    spare = np.sqrt(np.maximum(link['amps'] ** 2 - (tau / link['kt']) ** 2, 0.0))
    link['id'] = spare * np.clip((v - ONSET_V) / (CLAMP_V - ONSET_V), 0.0, 1.0)
    bleed = float(link['ohm'] @ (link['id'] * link['id']))
    load = (float(tau @ speed) + float(link['loss'] @ (tau * tau)) + bleed + link['house_w']) / v
    linear = (LINK_F * v / dt + drives.PACK_V / PACK_OHM - load) / (LINK_F / dt + 1.0 / PACK_OHM)
    limited = v + dt / LINK_F * (-CHARGE_A - load)
    link['v'] = max(linear, limited)
    link['peak'] = max(link['peak'], link['v'])
    link['bled'] += bleed * dt
    if world.buses is not None:
        world.block.volts[0] = link['v'] / math.sqrt(3.0)
    return link
