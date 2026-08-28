# %% [markdown]
# # coaxial 63100 - where the switching losses go
# `# %%` cells: opens as a notebook, runs as a script. Needs no board.
#
# **The headline: with nothing connected to the phases, edge speed costs
# nothing.** Overlap loss is the only term rise time enters, and it carries a
# factor of load current. C_oss and gate drive do not care how fast the edge
# is - they are set by charge and voltage.
#
# That flips once a motor is on the end, and by an order of magnitude. This
# file exists so that trade is written down rather than re-derived.
#
# Device parameters are **parsed from the LTspice model** in
# `electronic_simulations/`, not copied here. A second copy is the one that
# goes stale (invariant 7).

# %%
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ASC = os.path.join(ROOT, 'electronic_simulations', 'motor_inverters',
                   'half_bridge', 'half_bridge.asc')

PART = 'IAUCN10S7N021'


def spice_model(path, name):
    """The named .MODEL's parameters, off the schematic that simulates it."""
    text = open(path, encoding='utf-8', errors='replace').read()
    m = re.search(r'\.MODEL\s+%s\s+VDMOS\((.*?)\)' % re.escape(name),
                  text, re.S)
    if not m:
        raise SystemExit('no .MODEL %s in %s' % (name, path))
    body = m.group(1).replace('\\n', ' ')
    out = {}
    for key, val in re.findall(r'(\w+)\s*=\s*([0-9.eE+-]+[a-zA-Z]?)', body):
        out[key.lower()] = _si(val)
    return out


def _si(text):
    """SPICE suffixes: 1.8m is milli, 21p pico, 15.6n nano. Not 1.8e-3 only."""
    scale = {'t': 1e12, 'g': 1e9, 'meg': 1e6, 'k': 1e3,
             'm': 1e-3, 'u': 1e-6, 'n': 1e-9, 'p': 1e-12, 'f': 1e-15}
    text = text.strip()
    for suffix in ('meg', 't', 'g', 'k', 'm', 'u', 'n', 'p', 'f'):
        if text.lower().endswith(suffix):
            head = text[:-len(suffix)]
            try:
                return float(head) * scale[suffix]
            except ValueError:
                break
    return float(text)


FET = spice_model(ASC, PART)
print('%s, from %s' % (PART, os.path.relpath(ASC, ROOT)))
for k in ('cjo', 'vj', 'm', 'cgs', 'cgdmin', 'cgdmax', 'qg', 'rg', 'ron'):
    print('  %-7s %s' % (k, FET.get(k)))

# %% [markdown]
# ## The board's own numbers
# TIM1 is centre-aligned, ARR 2375 off 237.5 MHz - see docs/TODO.md. The gate
# rail is the 15 V that D1 (CDZV15B) clamps to.

# %%
F_SW = 50e3            # Hz, exact: ARR 2375 off 237.5 MHz, centre-aligned
V_GATE = 15.0          # V, the +15V7 rail through D1's clamp
LEGS = 3               # U, V, W

# %% [markdown]
# ## C_oss loss
# `C_ds(v) = Cjo / (1 + v/Vj)**m`, the junction law the VDMOS model uses.
# Integrating it gives the charge into the node and the energy stored there.
#
# With no load the switch node swings rail-to-rail once per period. Everything
# the rail delivers into that capacitance comes back out as heat in the
# channels, so the dissipation per cycle is `Q_node * V` - not `1/2 C V**2`,
# which is only the part that ends up stored.


# %%
def q_oss(V, fet=FET):
    """Charge into one FET's C_ds, 0..V."""
    u = 1.0 + V / fet['vj']
    return fet['cjo'] * fet['vj'] * (u ** (1 - fet['m']) - 1) / (1 - fet['m'])


def e_oss(V, fet=FET):
    """Energy stored in one FET's C_ds at V."""
    u = 1.0 + V / fet['vj']
    a, b = 2 - fet['m'], 1 - fet['m']
    return fet['cjo'] * fet['vj'] ** 2 * (
        u ** a / a - u ** b / b - (1 / a - 1 / b))


def p_coss(V, legs=LEGS, f=F_SW):
    """Both FETs of each leg, node charged and dumped once per period."""
    return 2 * q_oss(V) * V * f * legs


def p_gate(legs=LEGS, f=F_SW, vg=V_GATE):
    """Qg * Vg * f, two FETs per leg. Independent of duty and of link."""
    return FET['qg'] * vg * f * 2 * legs


def p_overlap(V, amps, t_rise, legs=LEGS, f=F_SW):
    """The ONLY term rise time enters - and it scales with load current.

    Rise and fall taken equal, which is the optimistic half: the model's
    turn-off is slower than its turn-on.
    """
    return 0.5 * V * amps * (2 * t_rise) * f * legs


# %% [markdown]
# ## No load - the bench today
# Nothing on the phases, and all three legs at the same duty put no volts
# between them, so there is no phase current whatever the duty is.

# %%
print('\nno load, %d leg(s), %.0f kHz' % (LEGS, F_SW / 1e3))
print('%6s %10s %10s %10s' % ('link', 'C_oss', 'gate', 'total'))
for V in (24.1, 30.6, 63.0):
    print('%5.1fV %9.3fW %9.3fW %9.3fW'
          % (V, p_coss(V), p_gate(), p_coss(V) + p_gate()))

# %% [markdown]
# ## What the edge speed is worth, under load
# 8 ns is the figure to beat. At the board's rating the difference between a
# fast edge and a lazy one is the difference between a warm leg and a
# redesign.

# %%
print('\noverlap loss, %d leg(s) at 63 V / 100 A' % LEGS)
for t_r in (8e-9, 20e-9, 80e-9):
    print('  t_rise %4.0f ns -> %7.2f W'
          % (t_r * 1e9, p_overlap(63.0, 100.0, t_r)))

print('\nand at no load, the same sweep:')
for t_r in (8e-9, 80e-9):
    print('  t_rise %4.0f ns -> %7.2f W'
          % (t_r * 1e9, p_overlap(30.6, 0.0, t_r)))

# %% [markdown]
# ## Measured, 2026-08-28
# One leg (U) switching, nothing connected, ten minutes each. The NTC is not
# the junction - it sits elsewhere on the board - so the ratio below is a
# coupling figure, not a thermal resistance.
#
# The 30 V run started 2 C above the 24 V run's baseline and was still
# shedding heat, which compresses its delta. That is why the two coupling
# numbers differ; they should not be averaged.

# %%
MEASURED = [
    # link V, minutes, NTC start, NTC end
    (24.109, 10.0, 40.91, 44.87),
    (30.644, 10.0, 42.88, 47.34),
]

print('\nmeasured, one leg:')
print('%6s %8s %8s %10s %12s' % ('link', 'dT', 'model W', 'C/W', 'note'))
for V, _mins, t0, t1 in MEASURED:
    watts = p_coss(V, legs=1) + p_gate(legs=1)
    print('%5.1fV %+7.2fC %8.3f %9.1f  %s'
          % (V, t1 - t0, watts, (t1 - t0) / watts,
             'warm start' if t0 > 41 else 'cold-ish start'))

# %% [markdown]
# ## Where the heat lands - which is not where the watts are
# **None of the gate-drive power heats the FET channel.** It is dissipated in
# whatever resistance the gate charge flows through: the driver's own output
# switch, the FET's internal Rg, and R9. D5 (Schottky across R9) bypasses R9
# on turn-off, so the two paths are not symmetric.
#
# Resistances are the models' own - `SW_PU_MOD`/`SW_PD_MOD` from
# `2EDL8034F5.lib`, `Rg` from the VDMOS model, R9 off the schematic.

# %%
R_PU, R_PD = 3.0, 2.0          # 2EDL8034F5.lib output switches
R9, RG_INT = 0.47, FET['rg']   # D5 shorts R9 on turn-off
V_FWD = 0.8                    # DBOOT_MOD forward drop

PATHS = {'turn-on':  {'driver': R_PU, 'R9': R9,  'FET Rg': RG_INT},
         'turn-off': {'driver': R_PD, 'R9': 0.0, 'FET Rg': RG_INT}}


def gate_split(legs=LEGS, f=F_SW, vg=V_GATE):
    """Gate-drive watts by where they are dissipated, whole board."""
    half = 0.5 * FET['qg'] * vg
    out = {}
    for path in PATHS.values():
        total_r = sum(path.values())
        for where, r in path.items():
            out[where] = out.get(where, 0.0) + half * (r / total_r)
    return {k: v * f * 2 * legs for k, v in out.items()}


def driver_watts(legs=LEGS, f=F_SW, vg=V_GATE, i_q=1.5e-3):
    """Per driver IC: its share of the gate path, bootstrap, quiescent.

    `i_q` is an estimate - the .lib models switches, not supply current.
    NOT included: the high-side level shifter, which has no model here and
    scales with link voltage. That is the largest known gap.
    """
    return (gate_split(legs, f, vg)['driver']
            + FET['qg'] * V_FWD * f * legs
            + i_q * vg * legs) / legs


# %%
#: Package thermal resistance to ambient, order of magnitude. The FET's drain
#: pad conducts into copper; the driver's PG-DSO-8 has no pad at all.
THETA_FET, THETA_DRV = 50.0, 140.0

split = gate_split()
print('\ngate-drive %.3f W across %d legs, by where it is dissipated:'
      % (sum(split.values()), LEGS))
for where, w in sorted(split.items(), key=lambda kv: -kv[1]):
    print('  %-8s %6.1f mW' % (where, w * 1e3))

fet_pkg = (p_coss(24.1) + split['FET Rg']) / (2 * LEGS)
drv_pkg = driver_watts()
print('\nper package at 24.1 V:')
print('  FET    %6.1f mW  x %3.0f C/W -> %4.1f C rise  (pad into copper)'
      % (fet_pkg * 1e3, THETA_FET, fet_pkg * THETA_FET))
print('  driver %6.1f mW  x %3.0f C/W -> %4.1f C rise  (PG-DSO-8, no pad)'
      % (drv_pkg * 1e3, THETA_DRV, drv_pkg * THETA_DRV))

# %% [markdown]
# **Measured with a thermal camera, 2026-08-28: the gate drivers are the
# hottest parts on the board.** The arithmetic above says why, and it is not
# because they dissipate most - the FET packages dissipate about twice as
# much. The 2EDL8034 is a PG-DSO-8 with no thermal pad, while the FET's drain
# pad conducts into copper. Hottest is a statement about thermal path, not
# about watts, and reading a camera the other way would send someone looking
# for a fault in the drivers that is not there.

# %% [markdown]
# ## What this does not cover
# * **Body-diode conduction in the dead time.** Zero here because there is no
#   load current to commutate. It is not zero once a motor is connected.
# * **Core and copper loss** in whatever is on the end. Not the board's.
# * **Reverse recovery.** Same reason - no current in the diode to recover.
# * **The high-side level shifter.** No model here, and it scales with link
#   voltage and frequency. This is the largest known gap: measured, three legs
#   heated the board about twice as fast as the terms above predict.
# * **The 8 ns itself is unvalidated for this layout.** docs/TODO.md item 4:
#   the dead time is simulated and nothing has been on a scope. `Cjo` is the
#   single parameter the whole no-load budget rests on.
