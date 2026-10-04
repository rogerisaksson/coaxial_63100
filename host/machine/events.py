"""What befalls her walking, laid where the walk will meet it: the floor's events and a drive's.

    if was < events.at('sill') <= director.walker.phase:    # the left leg's phase crossing
        events.lay('sill', director, world)

EVENTS: a hole, a sill, a slip patch or a loose rug on the floor ahead (`World.terrain`), her left
lace snagged on her right shoe (`World.lace`), a knee's board run into its SOA or warmed
(`World.glitch`), a shove from her side (`World.push`). Each is laid as the left leg's phase crosses `at`: at its toe-off the hole, the
slip patch and the rug's heel-end under where the walk lands that foot, the sill SILL_AHEAD_M ahead
of its toes as it lifts; the lace at LACE_AT of its swing, passing the right foot; at GLITCH_AT of
its stance the knee's board in its SOA for SOA_S, or warmed; at SHOVE_AT of its stride a shove,
toward her left on an even `k`. A spread step `k` moves a floor event STEP_M along the walk and a
glitch or a shove GLITCH_STEP of the stride; `strides` on lays a floor event that
many strides further, met at the same phase. STANDING befalls her as she stands (`rig`,
`befall`).
"""
import math

from machine import figure, floor, gait

EVENTS = ('hole', 'sill', 'slip', 'rug', 'stairs', 'lace', 'soa', 'hot', 'nudge', 'shove')

#: The events laid on the floor, and those that befall her where she is.
FLOOR, NOW = ('hole', 'sill', 'slip', 'rug', 'stairs'), ('lace', 'soa', 'hot', 'nudge', 'shove')

#: The shoves, newtons for SHOVE_S along her side: a nudge she parries - 0.13 m/s, 60 N's on her
#: 55 kg as a woman, 38 on her 35 as built: at 60 she held 0 of 16, at 38 9 (2026-10-01); a shove
#: past saving, the page's P (`terminal.views.show_humanoid`), held 0 of 48 (2026-10-01).
SHOVES, SHOVE_S, SHOVE_AT = {'nudge': 38.0, 'shove': 120.0}, 0.12, 0.25

SILL_AHEAD_M, RUG_HEEL_M, STEP_M, LACE_AT = 0.15, 0.15, 0.03, 0.5
GLITCH_AT, GLITCH_STEP, SOA_S = 0.25, 0.05, 0.5


def at(event, k=0):
    """The left leg's phase `event` is laid at, a spread step `k` on."""
    if event in ('soa', 'hot'):
        return GLITCH_AT + k * GLITCH_STEP
    if event in SHOVES:
        return SHOVE_AT + k * GLITCH_STEP
    if event == 'lace':
        return gait.TOE_OFF + LACE_AT * (1.0 - gait.TOE_OFF) + k * GLITCH_STEP
    return gait.TOE_OFF


def lay(event, director, world, k=0, strides=0):
    """`event` laid where the walk will meet it, the left leg's phase crossing `at` now - a
    floor event `strides` strides on."""
    walker = director.walker
    bus = walker.view(director.machine.loop.bus)
    h = walker.heading
    on = strides * gait.STRIDE_M * walker.stride
    landing = (bus['pelvis.pose.z'] + (1.0 - gait.TOE_OFF) * gait.STRIDE_M * walker.stride
               + gait.planted(0.0, walker.stride)[0] + k * STEP_M + on)
    if event in ('soa', 'hot'):
        world.glitch('left_knee', event, SOA_S)
    elif event in SHOVES:
        side = SHOVES[event] * (1.0 if k % 2 == 0 else -1.0)
        world.push((side * math.cos(h), 0.0, -side * math.sin(h)), SHOVE_S)
    elif event == 'lace':
        world.lace()
    else:
        world.terrain(event, {
            'hole': landing + (gait.BALL - gait.HEEL) / 2.0, 'slip': landing,
            'stairs': landing + (gait.BALL - gait.HEEL) / 2.0 - floor.RUN_M / 2.0,
            'rug': landing - RUG_HEEL_M,
            'sill': (walker.balls['left'][2] + figure.TOE_M
                     + SILL_AHEAD_M + k * STEP_M + on),
        }[event], bus['pelvis.pose.x'], h)


#: Standing (the user, 2026-10-04): a nudge or a shove from her side ('nudge', 'shove') or
#: along her way ('nudge_on', 'shove_on'); a brick taken from under a foot, the feet abreast
#: ('brick') or the left BRICK_STAGGER_M ahead ('brick_on'); a nudge on a balance board, stiff
#: ('board', rocking about her way; 'board_on', across it) or free ('rocker', 'rocker_on').
#: Toward her left, from behind, the left brick, on an even `k`. `rigged` says how she lands
#: for one, `rig` sets its floor under her landed, `befall` lays it. A push's way is an angle
#: in her plane (`machine.dcm`'s: 0 toward her left, 90 ahead), PUSH_DEG on: a knob, the
#: circle swept by it.
STANDING = ('nudge', 'nudge_on', 'shove', 'shove_on', 'brick', 'brick_on', 'board', 'board_on',
            'rocker', 'rocker_on')
BRICK_STAGGER_M, PUSH_DEG = 0.1, 0.0


def rigged(event):
    """(up, stagger) for `event`'s rig: how high above the floor she lands, m, and how far her
    left foot is ahead of her right."""
    return (2.0 * floor.BRICK[1] if event.startswith('brick') else 0.0,
            BRICK_STAGGER_M if event == 'brick_on' else 0.0)


def rig(event, director, world):
    """`event`'s floor under her, landed: the bricks under her soles, the board under her
    feet."""
    frame = director.arrival.frames[0][2]
    h = math.radians(frame.get('yaw', 0.0))
    soles = [floor._on((frame[side][0][0], frame[side][0][2]), h, 0.0, 0.0,
                       (gait.BALL - gait.HEEL) / 2.0) for side in ('left', 'right')]
    if event.startswith('brick'):
        floor.bricks(world, [(s[0], s[2]) for s in soles], h)
    elif event.startswith(('board', 'rocker')):
        x, z = ((soles[0][i] + soles[1][i]) / 2.0 for i in (0, 2))
        g = h + (math.pi / 2.0 if event.endswith('_on') else 0.0)
        world.terrain('board', x * math.sin(g) + z * math.cos(g), x * math.cos(g) - z * math.sin(g), g)
        floor.board(world, floor.BOARD_K if event.startswith('board') else 0.0, floor.BOARD_C)


def befall(event, director, world, k=0):
    """`event` befalls her standing: a step `k` of its spread."""
    turn = director._pelvis(director.machine.loop.bus)[1]
    h = math.atan2(turn[0][2], turn[2][2])
    if event.startswith('brick'):
        floor.take(world, 'brick_right' if k % 2 else 'brick_left')
        return
    n = SHOVES['shove' if event.startswith('shove') else 'nudge']
    a = math.radians(PUSH_DEG + (90.0 if event.endswith('_on') else 0.0) + 180.0 * (k % 2))
    left, ahead = n * math.cos(a), n * math.sin(a)
    world.push((left * math.cos(h) + ahead * math.sin(h), 0.0,
                ahead * math.cos(h) - left * math.sin(h)), SHOVE_S)
