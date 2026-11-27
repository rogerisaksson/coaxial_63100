"""The robot's world steps between MuJoCo's in C where nothing else asks of them (`tools.cores.world`).

    steps.advanced(world, now, room)        # `physics.World.advance` on to `now`: two calls in C a step about MuJoCo's
    steps.paired(world, a, b, below)        # whether MuJoCo's contacts pair a geom `a` marks with one `b` marks below
    steps.ahead(world)                      # MuJoCo's step's first half begun beside the robot's law's pass
    steps.behind(world)                     # ... joined: whether one was begun
"""


def alone(world):
    """Whether nothing else is asked of `world`'s steps - no buses, no push, no harness, no lace -: C's between
    MuJoCo's (`advanced`)."""
    return (world.in_c and world.buses is None and world.hung is None and not world.laced and not world.pushed
            and world.data.time >= world.push_until)


def ahead(world):
    """MuJoCo's step's first half (`mj_step1`: its position and velocity stages - the robot's kinematics, its
    collisions, its constraints laid) begun on a thread of its C's (`tools.cores.world.staged`, no
    interpreter in it) beside what follows, where the robot's steps are C's (`alone`). Of the robot's state as the
    robot's last step left it alone, it reads nothing the robot's law, its drives or its boxes write
    before its step; what it writes - the robot's poses, its contacts, its constraints - none
    of them reads once its edge has (`edge.felt`, `edge.contacted`, its look). With `mj_step2` at the
    robot's step (`advanced`) it is MuJoCo's own `mj_step`, operation for operation: 75 us of its 119 behind the
    robot's law's pass. Joined before the robot's pass ends (`behind`): a pass that raised leaves it joined, the
    robot's next step whole. An error there is MuJoCo's own, the robot's process ended with its word - `mj_step2`'s
    and a whole step's stay Python's."""
    if alone(world):
        ahead_fn = _laid(world)[5]._split[4]
        if ahead_fn is not None:
            world._ahead = bool(ahead_fn())


def behind(world):
    """Whether a step's first half was begun (`ahead`), joined here: waited for where it still runs."""
    if not world._ahead:
        return False
    world._ahead = False
    behind_fn = _laid(world)[5]._split[5]
    return behind_fn is not None and bool(behind_fn())


def _laid(world):
    """The robot's world's steps (`tools.cores.world.laid`), laid at its first and anew after its reset
    (`world._steps`)."""
    steps = world._steps
    if steps is None:
        from tools.cores import world as c_world
        steps = world._steps = c_world.laid(world)
    return steps


def advanced(world, now, room):
    """`world` on to `now` as `physics.World.advance` steps it where nothing else is asked of its steps - no buses, no
    push, no harness, no lace -: its setpoints' rates and stamp, then a step at a time MuJoCo's step's first half
    where none was begun beside (the robot's drives and gearboxes stepped beside it where one was), the contacts'
    friction rows sprung to where they stuck (C, `stiction`), MuJoCo's step's second half (its own binding, as its
    first: its errors Python's), the contacts' slips yielded and, its `room` laid, its room's switch read before the
    next step's first half begins beside the boxes' move, their meters and the robot's link (C)."""
    _at, rated, before, after, _paired, s, _kept = _laid(world)
    join_before, split_after = s._fused
    d = world.data
    if room:
        s.switched, s.lamp = getattr(world, 'switched', s.switch_s), int(getattr(world, 'lamp', False))
    rated(now - world.stamp)
    world.stamp = now
    start, mj, m = d.time, world._mj, world.model
    try:
        while d.time < now - 1e-9:
            since = d.time - start
            if world._ahead:
                world._ahead = False
                join_before(since)
            else:
                mj.mj_step1(m, d)
                before(since)
            mj.mj_step2(m, d)
            if d.time < now - 1e-9 and split_after is not None:
                world._ahead = bool(split_after())
            else:
                after()
    finally:
        if world._ahead:
            behind(world)
    if room:
        world.switched, world.lamp = s.switched, bool(s.lamp)
    world.at = d.time
    p = world.passing
    world.effort, world.work, world.brake, world.heat = p.effort, p.work, p.brake, p.heat


def paired(world, a, b, below):
    """Whether any of MuJoCo's contacts now pairs a geom `a` marks with one `b` marks under `below` m up - `a`, `b`
    their bytes' addresses - read off its data in C (wbc_world_paired), no view of them made."""
    return bool(_laid(world)[4](a, b, below))
