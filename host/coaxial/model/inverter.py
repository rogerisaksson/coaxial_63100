"""The power stage's numbers, in one importable place: the library's inverter, its bench's beside it."""
import math
import os
import tomllib

#: The board as the parts library states it (`machine/library/inverters.toml`), read as the file it is: what a
#: sheet or the layout gives is stated there alone, and no page of the rig comes to the robot's body for it.
with open(os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                       'machine', 'library', 'inverters.toml'), 'rb') as _f:
    _INV = tomllib.load(_f)['coaxial_63100']

FSW = float(_INV['pwm_hz'])    #: TIM1 centre-aligned, ARR 2375 (HARDWARE.md)
TS = 1.0 / FSW

T_DEAD = 63.2e-9              #: DTG 15: 33.7 ns shot through dry, 42 did not
T_DEAD_SIM = 65.4e-9          #: the simulation's worst corner
T_MIN_PULSE = 105e-9          #: 25 ticks: TPW 40 ns + DTG 15 -> 0.53 % duty
GATE_UVLO_V = 7.5             #: the 2EDL8034's (motor_inverters/half_bridge/2EDL8034F5.lib)
#: Of the link the modulator may use: what the library's dead time leaves of a period, blanked at both
#: of its edges (the static solver's law, `mechatronics.v_frac`, held to this one by test_metacompiler_rig).
V_FRAC = round(1.0 - 2.0 * float(_INV['deadtime_s']) * FSW, 4)

# IAUCN10S7N021, from the vendor VDMOS model in half_bridge.asc.
RDS_ON = float(_INV['rds_ohm'])

#: Junction to case, K/W. `datasheets/mosfet/IAUCN10S7N021-Datasheet.pdf`
#: Rev 1.2 p.4, maximum - the only figure the sheet gives for it.
#:
#: The die the network has no node for: `thermal`'s `driver_*` is the copper a
#: camera can see, and the junction sits this far over its case - at 100 A a
#: FET's 9 W over a period, 6.2 K. The envelope judges a leg on it against
#: T_J_MAX (`thermal.judged`): a 125 C ceiling on the copper left 44 K of the
#: sheet unused.
RTH_JC = float(_INV['rth_jc'])

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

#: The junction limit the sheet states, degrees C: a leg's ceiling in the
#: calibration record, judged on the junction.
T_J_MAX = float(_INV['tj_max_c'])
#: The inverter's nodes' ceilings, degrees C: the switches' copper, the laminate and the stator.
CEILING_SWITCH_C, CEILING_LAMINATE_C, CEILING_STATOR_C = (
    float(c) for c in _INV['ceiling_c'][:3])
CJO, M, VJ = 15.6e-9, 0.45, 0.7    #: output capacitance at 0 V, and its law
QG, RG = 81e-9, 2.2

L_LOOP = 4e-9                 #: power loop, tight layout: 0.25 nH/mm over
                              #: the half_bridge.asc geometry (L21+L22+L1)
Q_RING = 1.0                  #: ASSUMED crisp damping - TODO 6 is the scope
                              #: trace that replaces this

SHUNT = float(_INV['shunt_ohm'])    #: two WSHM28187L000FEA in parallel
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
    """The switch-node ring at this link: L_LOOP against both FETs' Coss."""
    c = 2.0 * coss(vdc)
    f = 1.0 / (2.0 * math.pi * math.sqrt(L_LOOP * c))
    tau = Q_RING / (math.pi * f)
    return {'f_hz': f, 'tau_s': tau, 'settle_s': tau * math.log(100.0),
            'z_ohm': math.sqrt(L_LOOP / c)}


def blanking(vdc):
    """Margin left for the current sample at the deepest duty the modulator
    allows: a quarter of the (1 - V_FRAC) window, minus the ring settling
    and the sense chain's delay.
    """
    return (1.0 - V_FRAC) * TS / 4.0 - ring(vdc)['settle_s'] - AFE_DELAY


def dead_time_volts(vdc, t_dead=T_DEAD):
    """The dead time's mean voltage error at full current, per phase."""
    return vdc * t_dead / TS


def knee_amps(vdc, t_dead=T_DEAD):
    """The current that just slews the node across the link inside the dead
    time: 2 Qoss / t_dead.
    """
    return 2.0 * qoss(vdc) / t_dead


def dt_table(vdc, t_dead=T_DEAD, points=8):
    """(step_amps, volts[points]) for the firmware's compensation table: v_dt
    tanh(i / knee) sampled every half knee, held past the last.
    """
    step = knee_amps(vdc, t_dead) / 2.0
    v = dead_time_volts(vdc, t_dead)
    return step, [v * math.tanh(0.5 * k) for k in range(points)]
