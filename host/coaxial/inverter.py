"""The power stage's numbers, in one importable place.

What a simulation of this drive needs to know about the hardware it
pretends to be: the switching rate, the dead time, the FET's charge
curves, the loop the current rings in and the sense chain's floor. Every
figure is traced - schematic, datasheet, LTSpice (`electronic_simulations`
submodule, `half_bridge.asc`) or FINDINGS - and **none is a measurement on
this board** unless its comment says which one. docs/HARDWARE.md is the
prose behind the dead-time and sense figures.

The derived functions are the arithmetic the Monte Carlo and its notebook
share: dead-time voltage error, the knee current where the FET's output
charge stops softening it, the ring after an edge and what it leaves of
the sampling window.
"""
import math

FSW = 50e3                    #: TIM1 centre-aligned, ARR 2375 (HARDWARE.md)
TS = 1.0 / FSW

T_DEAD = 33.7e-9              #: DTG 8, bench-trimmed against the supply's OCP
T_DEAD_SIM = 65.4e-9          #: the simulation's worst corner - the tension
                              #: between the two is stated in HARDWARE.md
T_MIN_PULSE = 76e-9           #: 18 ticks: TPW 40 ns + DTG 8 -> 0.38 % duty
V_FRAC = 0.95                 #: of the link the modulator may use

# IAUCN10S7N021, from the vendor VDMOS model in half_bridge.asc.
#
# TYPICAL, NOT MAXIMUM, and the datasheet has both: 1.8 mOhm typ against
# 2.1 max at Vgs 10 V and Id 88 A (Rev 1.2, p.4); at Vgs 7 V it is 2.0 typ
# and 2.4 max. An SOA envelope built on the typical under-books the
# conduction by 17 % against a part that is within spec, which is the
# wrong direction for a limit - flagged rather than changed, because the
# LTspice model this tree traces is the typical one and the two would
# then disagree.
RDS_ON = 1.8e-3

#: Junction to case, K/W. `datasheets/mosfet/IAUCN10S7N021-Datasheet.pdf`
#: Rev 1.2 p.4, maximum - the only figure the sheet gives for it.
#:
#: THE DIE THE THERMAL MODEL HAS NO NODE FOR. `thermal`'s `driver_*` is
#: the copper a camera can see, and this is how far the junction sits
#: above its own case: at 100 A each FET carries about 9 W over a period,
#: so 6.2 K. The SOA ceiling of 125 C on the copper is therefore about
#: 131 C at the junction against the sheet's 175 C limit - 44 K of margin,
#: so the ceiling is CONSERVATIVE rather than optimistic, which is the
#: opposite of what was feared before this number was looked up.
RTH_JC = 0.69

#: Junction to ambient, K/W, typical. Same sheet, and the conditions
#: matter: "device on 2s2p FR4 PCB defined in accordance with JEDEC
#: standards (JESD51-5, -7). PCB is vertical in still air."
#:
#: AN INDEPENDENT CHECK ON THE SPREADING RESISTANCE, and it disagrees
#: with the model. One FET's whole path to air on a JEDEC board is
#: 25.9 K/W; the model's own path for one leg is `to_board` 45.6 plus
#: `board_to_ambient` 8.33, about 54 K/W - so the model's SPREADING TERM
#: ALONE is nearly twice the datasheet's entire junction-to-air, on a
#: board that carries heavier copper than 2s2p. FINDINGS has what that
#: means for the campaign.
RTH_JA_JEDEC = 25.9

#: The junction limit the sheet actually states, degrees C. Not the SOA
#: ceiling - that lives in the calibration record and is the board's to
#: hold (invariant 10) - but the number the ceiling has to stay under.
T_J_MAX = 175.0
CJO, M, VJ = 15.6e-9, 0.45, 0.7    #: output capacitance at 0 V, and its law
CGS, QG, RG = 5.48e-9, 81e-9, 2.2

L_LOOP = 4e-9                 #: power loop, tight layout: 0.25 nH/mm over
                              #: the half_bridge.asc geometry (L21+L22+L1)
Q_RING = 1.0                  #: ASSUMED crisp damping - TODO 6 is the scope
                              #: trace that replaces this

SHUNT = 3.5e-3                #: two WSHM28187L000FEA in parallel
AFE_V_PER_A = 15.909e-3       #: 4.5455 V/V x 3.5 mohm (HARDWARE.md, traced)
AFE_DELAY = 60e-9             #: sense chain group delay, from the AFE sim
AFE_A_PER_COUNT = 3.3 / 65536 / AFE_V_PER_A    #: 3.2 mA at 16 bits
NOISE_A = (0.35, 0.41)        #: MEASURED phase noise floor, A rms (FINDINGS)


def coss(v):
    """The FET's output capacitance at `v`, the VDMOS junction law."""
    return CJO / (1.0 + v / VJ) ** M


def qoss(v):
    """Output charge to `v`: the junction law integrated."""
    return CJO * VJ / (1.0 - M) * ((1.0 + v / VJ) ** (1.0 - M) - 1.0)


def ring(vdc):
    """The switch-node ring at this link: L_LOOP against both FETs' Coss.

    Q_RING is an assumption, so `tau_s` and `settle_s` (to 1 %) are design
    figures, not measurements.
    """
    c = 2.0 * coss(vdc)
    f = 1.0 / (2.0 * math.pi * math.sqrt(L_LOOP * c))
    tau = Q_RING / (math.pi * f)
    return {'f_hz': f, 'tau_s': tau, 'settle_s': tau * math.log(100.0),
            'z_ohm': math.sqrt(L_LOOP / c)}


def blanking(vdc):
    """Margin left for the current sample at the deepest duty the
    modulator allows: a quarter of the (1 - V_FRAC) window, minus the ring
    settling and the sense chain's delay. Positive means the sample lands
    on settled current."""
    return (1.0 - V_FRAC) * TS / 4.0 - ring(vdc)['settle_s'] - AFE_DELAY


def dead_time_volts(vdc, t_dead=T_DEAD):
    """The dead time's mean voltage error at full current, per phase."""
    return vdc * t_dead / TS


def knee_amps(vdc, t_dead=T_DEAD):
    """The current that just slews the node across the link inside the
    dead time: 2 Qoss / t_dead. Below it the output charge soft-switches
    the error away; the tanh knee the compensation and the model share."""
    return 2.0 * qoss(vdc) / t_dead


def dt_table(vdc, t_dead=T_DEAD, points=8):
    """(step_amps, volts[points]) for the firmware's compensation table:
    v_dt tanh(i / knee) sampled every half knee, held past the last."""
    step = knee_amps(vdc, t_dead) / 2.0
    v = dead_time_volts(vdc, t_dead)
    return step, [v * math.tanh(0.5 * k) for k in range(points)]
