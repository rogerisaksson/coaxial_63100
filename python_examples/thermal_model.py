# %% [markdown]
# # coaxial 63100 - the thermal observer, and how it is calibrated
# `# %%` cells: opens as a notebook, runs as a script. Needs no board.
#
# The same node network `Thermal/Src/thermal.c` runs in firmware, here in
# Python so the parameters can be fitted against measurements without a
# reflash.
#
# **Not FEM.** A mesh does not fit in a main loop. Ten nodes do:
#
#     driver U/V/W ---+       three each, one per leg - switching U
#     phase  U/V/W ---+       alone heats U alone, which is what the
#     mcu          ---+---    camera showed and one lumped node per
#     regs         ---+       band could not say
#     afe          ---+
#                     |
#                   board --- ambient
#
# A node is a ZONE, not a part - `regulators` is the whole supply corner.
# Constant sources inside a zone cancel in every difference.

# %%
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'host'))

# One definition, several consumers: this notebook, tools/show_thermal.py and
# tools/thermal_validate.py all read the model from the library.
from coaxial.thermal import (CFG, DRIVER_RISE_SWITCHING, NODES, NTC_OFFSET,
                             NTC_SEES_DRIVERS, POWER_SWITCHING,
                             board_from_ntc, calibrate, expected_ntc,
                             steady, tau_minutes)
from coaxial import ansi

ansi.utf8_stdout()   # the map below is glyphs, not ASCII
from coaxial.thermalmap import render

# %% [markdown]
# ## What was measured
# Four states against a dead patch of black soldermask (emissivity ~0.95),
# room 20 C. The differences are the measurement; the absolute level rests on
# the supply's 50 mA, and that supply's shunt is not trusted.

# %%
CAMERA = {
    'passive':   {'dead': 30.0, 'mcu': 45.0, 'regulators': 38.0,
                  'bridge': 31.0, 'afe': 31.0, 'ntc': 36.0},
    'afe on':    {'dead': 31.1, 'mcu': 45.3, 'regulators': 39.2, 'afe': 37.0},
    'traffic':   {'dead': 31.4, 'mcu': 45.0, 'regulators': 39.0, 'afe': 37.3},
    'switching': {'dead': 40.0, 'mcu': 57.3, 'regulators': 60.0,
                  'bridge': 50.1, 'afe': 40.0, 'ntc': 55.6},
}

print('%-11s %7s %8s %11s %8s %6s' %
      ('state', 'dead', 'mcu', 'regulators', 'bridge', 'afe'))
for tag, m in CAMERA.items():
    dead = m['dead']
    print('%-11s %7.1f %8s %11s %8s %6s'
          % (tag, dead,
             '%+.1f' % (m['mcu'] - dead),
             '%+.1f' % (m['regulators'] - dead),
             '%+.1f' % (m['bridge'] - dead) if 'bridge' in m else '-',
             '%+.1f' % (m['afe'] - dead)))

delta = CAMERA['switching']['dead'] - CAMERA['passive']['dead']
print('\n4-1 on the dead surface: %+.1f K -> %.2f W at %.2f K/W'
      % (delta, delta / CFG['board_to_ambient'], CFG['board_to_ambient']))

# %% [markdown]
# ## The NTC compensation
# Two terms of different character: a constant that is mounting, and one that
# scales with driver power. Solved against both camera states.

# %%
print('offset        %+.2f K   (passive: no driver was warming)' % NTC_OFFSET)
print('sees_drivers  %.3f     (above 1: closer to the heat than the node)'
      % NTC_SEES_DRIVERS)
for tag, rise in (('passive', 0.0), ('switching', DRIVER_RISE_SWITCHING)):
    print('  %-10s model %.2f, camera %.2f -> bulk %.2f (dead %.2f)'
          % (tag, expected_ntc(CAMERA[tag]['dead'], rise), CAMERA[tag]['ntc'],
             board_from_ntc(CAMERA[tag]['ntc'], rise), CAMERA[tag]['dead']))

# %% [markdown]
# ## The network
# The same numbers `thermal_defaults()` carries. **Only `board_to_ambient`
# and `board_capacity` have a clean measurement behind them.**

# %%
print('board_to_ambient %5.2f K/W   board_capacity %5.1f J/K   tau %.1f min'
      % (CFG['board_to_ambient'], CFG['board_capacity'], tau_minutes()))
for name in NODES:
    print('  %-11s to_board %5.1f K/W   capacity %.2f J/K'
          % (name, CFG['to_board'][name], CFG['capacity'][name]))

print('\nmodelled equilibrium, switching (%.2f W total):'
      % sum(POWER_SWITCHING.values()))
for name, t in sorted(steady(POWER_SWITCHING).items(), key=lambda kv: -kv[1]):
    print('  %-11s %6.2f C' % (name, t))

# %% [markdown]
# ## Re-fitting from a camera
# One division per zone, no fitting: `to_board = (T_zone - T_board) / P`.
# Measure against the **dead surface**, not the NTC.
#
# The camera sees ONE bridge band; the model holds three legs. With all
# three switching, each leg carries a third of the power at the same
# temperature - so the same photograph divides out to the per-leg
# resistance directly, and three of those in parallel are what the band
# measured.

# %%
s = CAMERA['switching']
got = calibrate(dict({'mcu': s['mcu'], 'regulators': s['regulators']},
                     **{leg: s['bridge'] for leg in
                        ('driver_u', 'driver_v', 'driver_w')}),
                s['dead'])
print('to_board from the switching state, K/W:')
for name, r in sorted(got.items()):
    print('  %-11s %6.1f   (in the model %.1f)'
          % (name, r, CFG['to_board'][name]))

# %% [markdown]
# ## The board as a picture
# Diffuse on purpose - heat in a laminate spreads, and sharp zones would be a
# lie about the physics. `N` marks the NTC, `+` the DC link connectors.

# %%
seen = {leg: s['bridge'] for leg in
        ('driver_u', 'driver_v', 'driver_w',
         'phase_u', 'phase_v', 'phase_w')}
seen.update({'regulators': s['regulators'], 'afe': s['afe'], 'mcu': s['mcu']})
print(render(seen, board_c=s['dead']))

# %% [markdown]
# ## What this does not cover
# * **The calibration is dry.** Nothing on the phases, nothing through the hot
#   swap. At 100 A the phase shunt alone makes 35 W against the whole dry
#   budget's 1.2 W, so the phase node's resistance is the first thing that
#   will need re-fitting.
# * **The phase legs, every capacity but the board's, the MCU's position and
#   all the LAYOUT coordinates are assumed**, not measured.
#   `tools/thermal_validate.py` prints which is which.
# * **The supply's current reading is not trusted** - a cheap shunt, and the
#   absolute power scale rests on it.
