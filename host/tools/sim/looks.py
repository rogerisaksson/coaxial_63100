"""A walk's look as the eye judges it, its landing and its power: the scoreboard's cost of a walk.

    looks.cost(measures)        # of a walk's measures {name: value}
    looks.shown(measures)       # them, a line

The measures: `strides.WALK`'s by name (LOOKS), and the landing's, the drives' load and the
power as `gait_montecarlo.trial` reads them (LANDS).
"""
import math

#: The thigh's reach ahead of upright at the landing past its reach behind at the lift by more
#: than BALANCE_DEG, BALANCE_K a degree; the head fore and aft past SURGE_MM, SURGE_K a mm; the
#: feet nearer than CLEAR_MM as they pass, CLEAR_K a mm. Her legs a little further back, straight,
#: graceful - not the trudge, the feet far out in front and none behind (2026-09-28): the knee
#: landing bent past KNEE_DEG, the thigh swung out past where it lands by more than OVER_DEG,
#: KNEE_K and OVER_K a degree.
BALANCE_DEG, BALANCE_K = 0.0, 0.2
KNEE_DEG, KNEE_K, OVER_DEG, OVER_K = 10.0, 0.5, 3.0, 0.5

#: The step taken out: the thigh short of REACH_DEG behind upright at the lift, REACH_K a degree,
#: the heaviest of the look - weighed light, the searches came to tiptoeing, the legs always in
#: front, the easiest balance (2026-09-28).
REACH_DEG, REACH_K = 10.0, 1.0
SURGE_MM, SURGE_K = 30.0, 0.15

#: The upper body's bob: the torso pitching past TORSO_DEG, TORSO_K a degree. The feet: the
#: stance's toes out of TOE_OUT degrees or the swinging foot's turned in, TOE_K a degree; the
#: ankle rolled past PRONATE_DEG under the shin, PRONATE_K a degree - pigeon-toed, the swinging
#: foot in 6.5, and overpronated at 4.9, to the eye (2026-09-28).
TORSO_DEG, TORSO_K = 2.0, 1.0
TOE_OUT, TOE_K, PRONATE_DEG, PRONATE_K = (5.0, 15.0), 0.5, 3.0, 0.5
CLEAR_MM, CLEAR_K = 5.0, 0.2

#: The rear foot's toes back along the floor at their lift past SLIP_MM, SLIP_K a mm: the step
#: ends with the foot lifted and moving forward at once (the user, 2026-10-04) - 92 mm with the
#: heel rising by the phase alone (`stance.HEEL_UP_DEG`).
SLIP_MM, SLIP_K = 2.0, 0.5

#: The landing, heavy - a soft walk keeps the drives whole, copper lost before anything broken
#: (2026-09-28): the sole's peak from its touch past IMPACT_N, IMPACT_K a newton; the ankle
#: falling past TOUCH_MS as it touches, TOUCH_K a m/s.
IMPACT_N, IMPACT_K = 700.0, 0.01
TOUCH_MS, TOUCH_K = 0.15, 20.0

#: The landing's loading rate, the clonk heard: the sole's steepest rise over a millisecond in
#: the impact's window past RATE_KN_S kN/s, RATE_K a kN/s - she should be as quiet as a person,
#: only her clothes heard against her (2026-09-28).
RATE_KN_S, RATE_K = 20.0, 0.02

#: The drives' load: the share of drive-ms at a drive's peak past LOAD_PCT %, LOAD_K a percent -
#: at 0.85 strides/s 0.47 % of all, the ankles 1.78, the knees 1.40, the hips 0.95: at each
#: landing and as each leg is snapped into its swing (2026-09-30).
LOAD_PCT, LOAD_K = 0.0, 4.0

#: Her power walking, W - the page's sum (`machine.running`): the work done, the copper's heat,
#: the boards' own - ENERGY_K a watt past ENERGY_W; 482 W at 0.85 strides/s (2026-10-01).
ENERGY_W, ENERGY_K = 300.0, 0.05

#: Her walk's form on a flat, smooth floor (the user, 2026-10-05): a condition, never weighed
#: against a fall - (measure, least, most), `strides.measured`'s names. The stance leg a strut,
#: its knee all but straight while her foot bears her alone behind the plumb line; the leg on
#: behind the plumb line; the pelvis rolled up over the standing hip; the toes not back as they
#: lift (2026-10-04); no parry asked of a plain floor; her head still and her feet quiet, as the
#: walk approved 2026-10-03 had them (945a6a8: bob 20.5, fore and aft 34.1, aside 25.0 mm). The
#: landing knee stood 24 deg on that walk, 1-3 on a take of a woman's (`mocap.py`): its bound
#: comes down as the landing does (docs/TODO.md).
FORM = (('knee behind plumb', None, 10.0), ('knee at landing', None, 30.0),
        ('leg behind plumb', 12.0, None), ('hip over stance', 3.0, None),
        ('toes back at lift', None, 2.0), ('catches', None, 0.0), ('head bob', None, 30.0),
        ('head fore-aft', None, 45.0), ('head aside', None, 35.0), ('strike', None, 450.0))

#: A walk's price (`priced`): its energy a metre, J/m over ENERGY_J_M, and FORM_K a measure's
#: share past its bound - a walk off its form costs more than any energy saves; fallen, FELL.
ENERGY_J_M, FORM_K, FELL = 100.0, 20.0, 1000.0


def broken(measures):
    """[(measure, value, bound)]: FORM's conditions `measures` does not meet; one unmeasured is
    not met."""
    out = []
    for name, least, most in FORM:
        v = measures.get(name, math.nan)
        if v != v or (least is not None and v < least) or (most is not None and v > most):
            out.append((name, v, least if least is not None else most))
    return out


def priced(measures):
    """A steady walk's price: its energy a metre and what it is off its form."""
    if measures.get('fell'):
        return FELL
    off = sum(1.0 if v != v else abs(v - bound) / max(1.0, abs(bound))
              for _name, v, bound in broken(measures))
    return measures.get('energy', math.nan) / ENERGY_J_M + FORM_K * off


#: The look's measures, by `strides.WALK`'s names, and the landing's and the power's.
LOOKS = ('thigh ahead at landing', 'thigh behind at lift', 'head fore-aft', 'feet clear',
         'torso pitch', 'toe out', 'toe out swinging', 'ankle roll', 'knee at landing',
         'thigh most ahead', 'toes back at lift')
LANDS = ('impact', 'touch', 'rate', 'load', 'power')


def cost(looks):
    """The cost of a walk's measures {name: value} (LOOKS, LANDS); one not measured costs
    nothing."""
    (ahead, behind, surge, clear, torso, out, swinging, roll, knee, most, slip, impact, touch,
     rate, load, power) = (looks.get(n, math.nan) for n in LOOKS + LANDS)
    terms = (BALANCE_K * max(0.0, ahead - behind - BALANCE_DEG),
             REACH_K * max(0.0, REACH_DEG - behind),
             SURGE_K * max(0.0, surge - SURGE_MM), CLEAR_K * max(0.0, CLEAR_MM - clear),
             TORSO_K * max(0.0, torso - TORSO_DEG),
             TOE_K * (max(0.0, TOE_OUT[0] - out) + max(0.0, out - TOE_OUT[1])
                      + max(0.0, -swinging)),
             PRONATE_K * max(0.0, roll - PRONATE_DEG), KNEE_K * max(0.0, knee - KNEE_DEG),
             OVER_K * max(0.0, most - ahead - OVER_DEG), SLIP_K * max(0.0, slip - SLIP_MM),
             IMPACT_K * max(0.0, impact - IMPACT_N), TOUCH_K * max(0.0, touch - TOUCH_MS),
             RATE_K * max(0.0, rate - RATE_KN_S), LOAD_K * max(0.0, load - LOAD_PCT),
             ENERGY_K * max(0.0, power - ENERGY_W))
    return sum(t for t in terms if t == t)


def shown(looks):
    """A walk's measures, a line."""
    return ('ahead %.1f behind %.1f deg, surge %.1f, clear %.1f mm, torso %.1f, toes %.1f'
            ' swinging %.1f, roll %.1f, knee %.1f, most %.1f deg, slip %.0f mm, impact %.0f N,'
            ' touch %.2f m/s, rate %.0f kN/s, load %.2f %%, power %.0f W'
            % tuple(looks.get(n, math.nan) for n in LOOKS + LANDS))
