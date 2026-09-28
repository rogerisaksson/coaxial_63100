"""What befalls her walking, laid where the walk will meet it: the floor's events and a drive's.

    if was < events.at('sill') <= director.walker.phase:    # the left leg's phase crossing
        events.lay('sill', director, world)

EVENTS: a hole, a sill, a slip patch or a loose rug on the floor ahead (`World.terrain`), a lace
caught under the other foot (`World.tug`), a knee's board run into its SOA or warmed
(`World.glitch`). Each is laid as the left leg's phase crosses `at`: at its toe-off the hole,
the slip patch and the rug's heel-end under where the walk lands that foot, the sill SILL_AHEAD_M
ahead of its toes as it lifts, the lace pulling that foot back LACE_N for LACE_S; at GLITCH_AT
of its stance the knee's board in its SOA for SOA_S, or warmed. A spread step `k` moves a floor
event STEP_M along the walk and a glitch GLITCH_STEP of the stride; `strides` on lays a floor
event that many strides further, met at the same phase.
"""
from machine import figure, floor, gait

EVENTS = ('hole', 'sill', 'slip', 'rug', 'stairs', 'lace', 'soa', 'hot')

#: The events laid on the floor, and those that befall her where she is.
FLOOR, NOW = ('hole', 'sill', 'slip', 'rug', 'stairs'), ('lace', 'soa', 'hot')

SILL_AHEAD_M, RUG_HEEL_M, STEP_M = 0.15, 0.15, 0.03
GLITCH_AT, GLITCH_STEP, SOA_S = 0.25, 0.05, 0.5

#: A lace stepped on: the lifting foot pulled back, N for s. At 120 N for 0.15 s she walked on,
#: tipped 7 degrees; at 250 N for 0.2 s and past it she fell (2026-09-28).
LACE_N, LACE_S = 300.0, 0.2


def at(event, k=0):
    """The left leg's phase `event` is laid at, a spread step `k` on."""
    if event in ('soa', 'hot'):
        return GLITCH_AT + k * GLITCH_STEP
    return gait.TOE_OFF


def lay(event, director, world, k=0, strides=0):
    """`event` laid where the walk will meet it, the left leg's phase crossing `at` now - a
    floor event `strides` strides on."""
    bus, walker = director.machine.loop.bus, director.walker
    on = strides * gait.STRIDE_M * walker.stride
    landing = (bus['pelvis.pose.z'] + (1.0 - gait.TOE_OFF) * gait.STRIDE_M * walker.stride
               + gait.planted(0.0, walker.stride)[0] + k * STEP_M + on)
    if event in ('soa', 'hot'):
        world.glitch('left_knee', event, SOA_S)
    elif event == 'lace':
        world.tug('left_foot', (0.0, 0.0, -LACE_N), LACE_S)
    else:
        world.terrain(event, {
            'hole': landing + (gait.BALL - gait.HEEL) / 2.0, 'slip': landing,
            'stairs': landing + (gait.BALL - gait.HEEL) / 2.0 - floor.RUN_M / 2.0,
            'rug': landing - RUG_HEEL_M,
            'sill': (walker.balls['left'][2] + 2.0 * figure.CONTACTS[1][2][2]
                     + SILL_AHEAD_M + k * STEP_M + on),
        }[event])
