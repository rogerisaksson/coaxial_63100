"""The motors known here: a nameplate, the bench's aim, the stand-in's truth."""
from motor.pmsm import flux_from_kv, Parameters

#: The nameplate, off Hobbywing's own specification page: the
#: manufacturer's numbers, none measured here. Two of them bound what the
#: drive may ask for: the motor's 112.5 A burst sits just above the
#: board's 100 A rating, so the inverter is the limit, and a 12S pack at
#: full charge is 50.4 V
#: against a 63 V rating and 78.15 V of divider scale.
RATINGS = {
    'slots_poles': '24N28P',        # 24 stator slots, 28 magnet poles
    'kv': 190.0,
    'cells': '10-12S LiPo',
    'i_noload': 3.0, 'v_noload': 44.4,
    'i_max': 112.5, 't_i_max': 42.0,
    'p_max': 4985.0,
    'mass_kg': 0.582, 'shaft_mm': 8.0,
    'stator': '51.7 x 30 mm',
    'source': 'hobbywing.com Platinum 5220/5230 specification page',
}

#: The motor this bench is aimed at, not measured here: every electrical
#: constant is an estimate, for the notebook to replace first. What comes
#: from where:
#:
#:   poles   the sheet: 24N28P is 28 poles, 14 pairs. It is not derivable
#:           from any torque measurement - see KT_NM_PER_AMP - and it sets
#:           the whole electrical frequency, so it had to come from the
#:           winding.
#:   b       the sheet, arithmetic: 3.0 A at 44.4 V spins it at 190*44.4 =
#:           8436 rpm, so 133 W of iron and friction against 883 rad/s is
#:           0.151 N.m of drag, b = 1.71e-4. Copper at 3 A is 0.4 W, lost
#:           in the rounding. The guess this replaced was 4.3x too low.
#:   lambda  the label's 190 KV through `flux_from_kv` at 14 pole pairs.
#:   R       estimate, the size class. The sheet states no winding
#:           resistance. R is what an identification finds first, because
#:           it is the easiest thing to see.
#:   Ld, Lq  estimate, the size class again, with the ~1.3 saliency an
#:           outrunner of this construction tends to show. This is the
#:           number the sensorless observer lives or dies on, and it is the
#:           least trustworthy one here.
#:   J       estimate. 582 g total, of which the bell is the part that
#:           turns; a thin ring at 30 mm would be 3e-4 and a hub pulls it
#:           down, so this is a middling guess and not arithmetic.
PLATINUM_5230SL = Parameters(
    name='Hobbywing Platinum 5230SL 190KV',
    r=0.030,
    ld=22e-6,
    lq=29e-6,
    lam=flux_from_kv(190.0, 14),
    poles=14,                       # 24N28P: 28 poles, 14 pairs
    j=1.2e-4,
    b=1.71e-4,
    # Estimated.
    sat=0.3, i_sat=40.0,
    measured=False,
    source='poles and friction from the manufacturer sheet (see RATINGS); '
           'R, Ld, Lq and J estimated from the size class - replace with '
           'an identification against the real machine')


#: The machine behind the stand-in's drive device, not the
#: 5230SL: its constants were picked so every commissioning step recovers a
#: number it can be checked against, which is a different job from
#: resembling the motor on the bench. It lives here rather than as literals
#: on `SimulatedDrive` so that a third copy of a machine cannot drift away
#: from the two above.
BENCH_MOTOR = Parameters(
    name='stand-in bench motor',
    r=0.05, ld=20e-6, lq=30e-6, lam=0.005, poles=7, j=2e-5, b=1e-5,
    sat=0.3, i_sat=4.0,        # SimulatedDrive.SAT / I_SAT
    measured=False,
    source='invented for the stand-in, chosen to be recoverable')

#: What the sheet pins: torque per ampere of q current, from
#: 2.78 N.m at the top of the 190KV curve:
#:
#:     Kt = 1.5 * P * lambda = T / iq
#:
#: and it is the product that the sheet determines. iq comes out 63.9 A at
#: full throttle whether the machine has 5 pole pairs or 10, because
#: lambda from a KV goes as 1/P and the P cancels - so this document
#: cannot tell you the pole count, and neither can any torque measurement.
#: The observer needs P on its own, because electrical speed is P times
#: mechanical. It came from the winding and not from any measurement of
#: torque: 24N28P is 28 poles, 14 pairs. Kt is unchanged by that - 1.5 *
#: 14 * 0.00207 is the same 0.0435 as 1.5 * 7 * 0.00415 - which is the
#: cancellation stated above, seen from the other side.
KT_NM_PER_AMP = 0.0435
