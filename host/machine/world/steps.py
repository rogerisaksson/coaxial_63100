"""Her world's steps between MuJoCo's in C, where nothing else is asked of them (`tools.cores.world`).

    steps.advanced(world, now, room)        # `physics.World.advance` on to `now`: two calls in C a step about MuJoCo's
    steps.paired(world, a, b, below)        # whether MuJoCo's contacts pair a geom `a` marks with one `b` marks below
"""


def _laid(world):
    """Her world's steps (`tools.cores.world.laid`), laid at her first and anew after her reset (`world._steps`)."""
    steps = world._steps
    if steps is None:
        from tools.cores import world as c_world
        steps = world._steps = c_world.laid(world)
    return steps


def advanced(world, now, room):
    """`world` on to `now` as `physics.World.advance` steps it where nothing else is asked of its steps - no buses, no
    push, no harness, no lace -: its setpoints' rates and stamp, then a step at a time her drives' torques and her
    gearboxes (C), MuJoCo's step (its own binding: its errors Python's), the boxes moved, their meters, her link and,
    her `room` laid, her room's switch (C), the switch's state in from her world and out again."""
    _at, rated, before, after, _paired, s, _kept = _laid(world)
    d = world.data
    if room:
        s.switched, s.lamp = getattr(world, 'switched', s.switch_s), int(getattr(world, 'lamp', False))
    rated(now - world.stamp)
    world.stamp = now
    start, step, m = d.time, world._mj.mj_step, world.model
    while d.time < now - 1e-9:
        before(d.time - start)
        step(m, d)
        after()
    if room:
        world.switched, world.lamp = s.switched, bool(s.lamp)
    world.at = d.time
    p = world.passing
    world.effort, world.work, world.brake, world.heat = p.effort, p.work, p.brake, p.heat


def paired(world, a, b, below):
    """Whether any of MuJoCo's contacts now pairs a geom `a` marks with one `b` marks under `below` m up - `a`, `b`
    their bytes' addresses - read off its data in C (wbc_world_paired), no view of them made."""
    return bool(_laid(world)[4](a, b, below))
