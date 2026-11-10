"""Her world's steps between MuJoCo's in C, where nothing else is asked of them (`tools.cores.world`).

    steps.advanced(world, now, room)        # `physics.World.advance` on to `now`: two calls in C a step about MuJoCo's
    steps.paired(world, a, b, below)        # whether MuJoCo's contacts pair a geom `a` marks with one `b` marks below
    steps.ahead(world)                      # MuJoCo's step's first half begun beside her law's pass
    steps.behind(world)                     # ... joined: whether one was begun
"""


def alone(world):
    """Whether nothing else is asked of `world`'s steps - no buses, no push, no harness, no lace -: C's between
    MuJoCo's (`advanced`)."""
    return (world.in_c and world.buses is None and world.hung is None and not world.laced and not world.pushed
            and world.data.time >= world.push_until)


def ahead(world):
    """MuJoCo's step's first half (`mj_step1`: its position and velocity stages - her kinematics, her
    collisions, her constraints laid) begun on a thread of her C's (`tools.cores.world.staged`, no interpreter in
    it) beside what follows, where her steps are C's (`alone`). Of her state as her last step left it alone, it reads
    nothing her law, her drives or her boxes write before her step; what it writes - her poses, her contacts, her
    constraints - none of them reads once her edge has (`edge.felt`, `edge.contacted`, her look). With `mj_step2` at
    her step (`advanced`) it is MuJoCo's own `mj_step`, operation for operation: 75 us of its 119 behind her law's
    pass. Joined before her pass ends (`behind`): a pass that raised leaves it joined, her next step whole. Its error
    there is MuJoCo's own, her process ended with its word - `mj_step2`'s and a whole step's stay Python's."""
    from tools.cores import world as c_world
    staged = c_world.staged()
    if staged is not None and alone(world):
        world._ahead = bool(staged[0](staged[2], world.model._address, world.data._address))


def behind(world):
    """Whether a step's first half was begun (`ahead`), joined here: waited for where it still runs."""
    if not world._ahead:
        return False
    from tools.cores import world as c_world
    staged, world._ahead = c_world.staged(), False
    return staged is not None and bool(staged[1]())


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
    start, mj, m = d.time, world._mj, world.model
    while d.time < now - 1e-9:
        before(d.time - start)
        (mj.mj_step2 if behind(world) else mj.mj_step)(m, d)     # its first half begun beside her law, or whole
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
