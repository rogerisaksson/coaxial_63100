"""The gynoid's gaits: a gait a row of `machine.going`'s setpoints, her way between two a mix.

    going.ask = gaits.between(0.3)              # -1 her stand, 0 the walk's row, 1 the run's
"""

#: A gait a row of setpoints: speed, m/s; step, the seconds from a landing to the other foot's;
#: stand, a foot's stance; the bounce - `up` m over its landing and rising `rise` m/s, `bounce` s
#: on; land, the foot's pitch as it comes down, deg, the heel up; knee, deg, as it lands; lean,
#: her trunk ahead of plumb, deg; fold, the swinging knee's, deg; track, m, her feet out from
#: her capture point; off, the heel's rise as she leaves, deg; list, the pelvis's roll over the
#: standing leg, deg; under, m ahead of the ankle, the sole's point put under her; folded, the
#: share of its swing the knee's fold takes; reach, m ahead of its hip the free foot may wait.
#: The run's is `machine.runner`'s: asked 1.5 m/s, 1.44-1.47 at 508-522 J/m drawn (2026-10-05).
RUN = {'speed': 1.5, 'step': 0.40, 'stand': 0.30, 'up': 0.0, 'rise': 0.49, 'bounce': 0.30,
       'land': 14.0, 'knee': 22.0, 'lean': 6.0, 'fold': 55.0, 'track': 0.035, 'off': 50.0,
       'list': 0.0, 'under': 0.117, 'folded': 1.0, 'reach': 0.36}
#: The walk's, found: 2 240 rows priced as the scoreboard prices a walk (`looks.priced`), a foot
#: always down and the strut the form's - 1 357 walked their 10 s - before her stand was in the
#: law. On the law as it is: 0.81 m/s at 392 J/m, its knee landing at 18 deg, priced 72
#: (2026-10-05).
WALK = {'speed': 0.76, 'step': 0.528, 'stand': 0.581, 'up': 0.10, 'rise': 0.0, 'bounce': 0.253,
        'land': -12.9, 'knee': 19.4, 'lean': 5.1, 'fold': 10.0, 'track': 0.048, 'off': 1.7,
        'list': 0.0, 'under': 0.037, 'folded': 0.65, 'reach': 0.248}
#: Her jog, the run's row at the walk's speed: her way from the walk to the run passes it
#: (`between`), the two gaits mixed at one speed. On its speed alone the run's row goes 0.54 m/s
#: at 936 J/m asked 0.6, 0.74 at 704 asked 0.8, 0.99 at 602, 1.19 at 547 (2026-10-05). Before
#: it a row found between, at 1.42 m/s, passed walk to run and back on 2 timings of 10.
JOG = dict(RUN, speed=0.8)
#: Standing: the walk's row at no speed.
STAND = dict(WALK, speed=0.0)


def mix(a, b, k):
    """The row `k` of the way from `a` to `b`."""
    return {name: a[name] + (b[name] - a[name]) * k for name in a}


def between(k):
    """The row `k` of her way: -1 her stand, 0 the walk's, 0.5 her jog, 1 the run's."""
    if k <= 0.0:
        return mix(STAND, WALK, 1.0 + max(-1.0, k))
    return mix(WALK, JOG, 2.0 * k) if k <= 0.5 else mix(JOG, RUN, 2.0 * min(1.0, k) - 1.0)
