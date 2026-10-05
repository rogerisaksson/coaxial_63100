"""The gynoid's gaits: a gait a row of `machine.going`'s setpoints, her way between two a mix.

    going.ask = gaits.between(0.3)              # 0 the walk's row, 1 the run's
"""

#: A gait a row of setpoints: speed, m/s; step, the seconds from a landing to the other foot's;
#: stand, a foot's stance; the bounce - `up` m over its landing and rising `rise` m/s, `bounce` s
#: on; land, the foot's pitch as it comes down, deg, the heel up; knee, deg, as it lands; lean,
#: her trunk ahead of plumb, deg; fold, the swinging knee's, deg; track, m, her feet out from
#: her capture point; off, the heel's rise as she leaves, deg; list, the pelvis's roll over the
#: standing leg, deg; under, m ahead of the ankle, the sole's point put under her; folded, the
#: share of its swing the knee's fold takes; reach, m ahead of its hip the free foot may wait.
#: The run's is `machine.runner`'s: asked 1.5 m/s, 1.47 at 516 J/m drawn (2026-10-05).
RUN = {'speed': 1.5, 'step': 0.40, 'stand': 0.30, 'up': 0.0, 'rise': 0.49, 'bounce': 0.30,
       'land': 14.0, 'knee': 22.0, 'lean': 6.0, 'fold': 55.0, 'track': 0.035, 'off': 50.0,
       'list': 0.0, 'under': 0.117, 'folded': 1.0, 'reach': 0.36}
#: The walk's, found: 2 240 rows priced as the scoreboard prices a walk (`looks.priced`), a foot
#: always down and the strut the form's - 1 357 walked their 10 s. The best, 0.99 m/s at 360
#: J/m, priced 25; this row is it to three digits: 0.95 m/s at 366 J/m, its knee landing at 17
#: deg, priced 92 - her toes 8.6 mm back at their lift where 1.9 -, and to four digits another
#: way a stumble, 336: found on one walk, a row's price is chance (2026-10-05).
WALK = {'speed': 0.76, 'step': 0.528, 'stand': 0.581, 'up': 0.10, 'rise': 0.0, 'bounce': 0.253,
        'land': -12.9, 'knee': 19.4, 'lean': 5.1, 'fold': 10.0, 'track': 0.048, 'off': 1.7,
        'list': 0.0, 'under': 0.037, 'folded': 0.65, 'reach': 0.248}
#: Between them, a row her way from the one to the other passes through (`between`), found: 432
#: rows, 14 up through both passages; this one walk, run and walk again, 22 s, on 2 timings of
#: 10 - half-way between the two she was down at 2.8 s (2026-10-05).
MID = {'speed': 1.42, 'step': 0.433, 'stand': 0.458, 'up': 0.014, 'rise': 0.058, 'bounce': 0.255,
       'land': -8.1, 'knee': 11.9, 'lean': 3.9, 'fold': 22.6, 'track': 0.029, 'off': 31.5,
       'list': 0.0, 'under': 0.112, 'folded': 0.56, 'reach': 0.271}



def mix(a, b, k):
    """The row `k` of the way from `a` to `b`."""
    return {name: a[name] + (b[name] - a[name]) * k for name in a}


def between(k):
    """The row `k` of her way from the walk's, 0, through MID to the run's, 1."""
    return mix(WALK, MID, 2.0 * k) if k <= 0.5 else mix(MID, RUN, 2.0 * k - 1.0)
