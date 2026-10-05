"""Her state marked and gone back to: the world, her boards, the loop and the director.

    mark = replay.mark(body, director)      # between passes
    replay.back(mark)                       # all of it as it was: a fall undone, a law swapped in

A mark holds MuJoCo's state, the model's arrays an event moves, every plain value of the machine's
and the director's objects, and what the host last heard of each board. The boards are other
processes: each keeps its own of every mark and goes back to the one asked at its next step
(`buses.Segment.tick`; the block's `marked`, `backed` and `back_to`). Held at the mark's setpoints as at a reset instead, their
rates read anew from two frames, the walk gone back to drew 382 W over 3 s where 333 walked on
(2026-10-05).
"""
import collections
import copy

#: The model's arrays a run moves: a short's damping, the lace, the floor's events.
MODEL = ('dof_damping', 'tendon_range', 'geom_size', 'geom_pos', 'geom_quat', 'geom_friction',
         'geom_contype', 'geom_conaffinity', 'body_pos', 'body_quat')

#: The block's fields the world writes as it runs; its counters stay the processes'.
BLOCK = ('limit', 'ctrl', 'air', 'rds', 'scale', 'rotor', 'gains', 'envelope', 'free_at', 'warm',
         'hold', 'time', 'q', 'qd')

#: The packages whose objects a mark walks.
OURS = ('machine', 'tools', 'terminal', 'coaxial')

_SCALARS = (int, float, bool, str, bytes, complex, type(None))


def _plain(v, np):
    """Whether `v` is data alone: numbers, strings, arrays and containers of them."""
    if isinstance(v, _SCALARS) or isinstance(v, (np.ndarray, np.generic)):
        return True
    if isinstance(v, (tuple, list, set, frozenset, collections.deque)):
        return all(_plain(x, np) for x in v)
    if isinstance(v, dict):
        return all(_plain(k, np) and _plain(x, np) for k, x in v.items())
    return False


def _ours(v):
    return (type(v).__module__.split('.')[0] in OURS and hasattr(v, '__dict__')
            and not isinstance(v, type))


def _within(v, depth=0):
    """Our objects in `v`, itself or held in its containers."""
    if _ours(v):
        yield v
    elif depth < 4 and isinstance(v, (tuple, list, set, frozenset, collections.deque)):
        for x in v:
            yield from _within(x, depth + 1)
    elif depth < 4 and isinstance(v, dict):
        for x in v.values():
            yield from _within(x, depth + 1)


def _into(v, was, np):
    """`v` holding `was` again: a container or an array in place, whoever else holds it; anything
    else `was`'s own copy, or itself where it was kept by reference."""
    if was is None:
        return v
    if isinstance(v, np.ndarray) and v.shape == was.shape:
        v[...] = was
    elif isinstance(v, dict):
        v.clear()
        v.update(copy.deepcopy(was))
    elif isinstance(v, list):
        v[:] = copy.deepcopy(was)
    elif isinstance(v, (set, collections.deque)):
        v.clear()
        (v.update if isinstance(v, set) else v.extend)(copy.deepcopy(was))
    else:
        return copy.deepcopy(was)
    return v


def mark(body, director):
    """Her state now, between passes."""
    from machine import buses, walkplan
    world = body.nodes['pelvis'].world
    np, mj = world._np, world._mj
    live = (buses.Bus, buses.Buses, buses.Block)
    held, queue = {}, [body, director]
    while queue:
        o = queue.pop()
        if id(o) in held or isinstance(o, live):
            continue
        attrs = {}
        for k, v in vars(o).items():
            attrs[k] = (copy.deepcopy(v), v) if _plain(v, np) else (None, v)
            if attrs[k][0] is None and v is not None:
                queue.extend(_within(v))
        held[id(o)] = (o, attrs)
    # Her boards take their own of it now: left to their next step, a reset before it (a
    # second `Director.begin`) left the mark with none, and her going back to it a dead bus.
    world.block.marked[0] += 1
    world.buses.step()
    spec = mj.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mj.mj_stateSize(world.model, spec))
    mj.mj_getState(world.model, world.data, state, spec)
    return {'world': world, 'held': held, 'state': state, 'id': world.block.marked[0],
            'model': {k: getattr(world.model, k).copy() for k in MODEL},
            'block': {k: list(getattr(world.block, k)) for k in BLOCK},
            'heard': [dict(bus.heard) for bus in world.buses.each],
            'plan': dict(walkplan._BEFORE)}


def back(m):
    """Her state as `mark` took it."""
    from machine import gait, walkplan
    world = m['world']
    np, mj = world._np, world._mj
    for o, attrs in m['held'].values():
        for k in [k for k in vars(o) if k not in attrs]:
            delattr(o, k)
        for k, (was, v) in attrs.items():
            setattr(o, k, _into(v, was, np))
    for k, v in m['model'].items():
        getattr(world.model, k)[...] = v
    mj.mj_setState(world.model, world.data, m['state'], mj.mjtState.mjSTATE_INTEGRATION)
    mj.mj_forward(world.model, world.data)
    world.buses.drain()
    for k, v in m['block'].items():
        getattr(world.block, k)[:] = np.array(v)
    world.block.back_to[0] = m['id']
    world.block.backed[0] += 1
    for bus, heard in zip(world.buses.each, m['heard']):
        bus.heard, bus.gating = dict(heard), {}
    walkplan._BEFORE.update(m['plan'])
    walkplan._TABLES.clear()
    gait._FITS.clear()


def tracked(m):
    """What a mark holds, a line a class: how many plain values, which it keeps by reference."""
    out = collections.OrderedDict()
    for o, attrs in m['held'].values():
        name = '%s.%s' % (type(o).__module__, type(o).__name__)
        kept = sorted(k for k, (was, v) in attrs.items() if was is None and v is not None)
        n, refs = out.get(name, (0, set()))
        out[name] = (n + len(attrs) - len(kept), refs | set(kept))
    return ['%-34s %3d plain | kept: %s' % (k, n, ' '.join(sorted(r))) for k, (n, r) in out.items()]
