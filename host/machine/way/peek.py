"""Her docks foreseen: the faces her wayfinder reaches on each thing, before she is told to go.

Asked of a planner of their own one thing after another, nearest first, again and again; the page rings
them (`balloons.docked`).

    peek.peeked(s)            # a pass: an answer taken where it is ready, the next thing asked
    peek.told(s)              # [{'kind', 'slot', 'at', 'chosen'}] each thing's docks, her errand's over its own

`dock.docked` took 43-157 ms a thing she reaches and up to 3.6 s one she does not (her page's room, 12
things): a round of them some seconds, the loop never waiting on one (`planner.Planner.ready`).
"""
import functools


def _goes(g):
    """[(verb, box)] each thing among hers a walked verb goes to (`furniture.GOES`), nearest her first."""
    from machine.way import affordance, dock, furniture
    things = furniture._things(g, None)
    at = (float(g['world'].data.qpos[0]), float(g['world'].data.qpos[2]))
    got = [(affordance.distance(p, at), verb, p) for verb in dock.docks() for p in things
           if furniture.GOES[verb][0](p, things)]
    return [(verb, p) for _d, verb, p in sorted(got, key=lambda h: h[0])]


def _key(verb, box):
    """A thing's key: its verb and its slot among hers, its centre where it has none."""
    return verb, int(box[4]) if len(box) > 4 else tuple(round(float(v), 1) for v in box[1])


def level(verb, box):
    """The height a dock on `box` is drawn at, m: a seat's or a bed's top, a door's floor."""
    from machine.way import affordance
    return float(affordance.bottom(box) if verb == 'exit_door' else affordance.top(box))


def _shown(verb, box, laid):
    """`laid`'s (`dock.docked`) docks as the page draws them: each face reached at its `level`, the laid one."""
    import math
    top, reach = level(verb, box), laid['reach']
    off = [abs(math.remainder(float(h) - float(laid['hn']), math.tau)) for h, _e in reach]
    return {'kind': verb, 'slot': _key(verb, box)[1], 'at': [(float(e[0]), top, float(e[1])) for _h, e in reach],
            'chosen': min(range(len(off)), key=off.__getitem__)}


@functools.cache
def _maneuver():
    """machine.texts.maneuver, imported at a first peek and not looked up again each pass."""
    from machine.texts import maneuver
    return maneuver


def peeked(s):
    """A pass of her peeks (`s['peeker']`, a planner of their own): its answer taken where ready - a thing
    she reaches none of dropped -, the next thing asked; a round over, things gone dropped; a new epoch
    all of them."""
    maneuver = _maneuver()
    g, planner = s['g'], s.get('peeker')
    if planner is None:
        return
    st = s.setdefault('peek', {'epoch': None, 'docks': {}, 'todo': [], 'asked': None})
    if st['epoch'] != s['epoch']:
        st.update(epoch=s['epoch'], docks={}, todo=[])
    told = planner.ready()
    if told is not None and st['asked'] is not None and st['asked'][0] == st['epoch']:
        _epoch, verb, box = st['asked']
        laid = told.get('laid')
        st['docks'][_key(verb, box)] = _shown(verb, box, laid) if laid and laid.get('reach') else None
    if planner.asked_at is not None:
        return
    if not st['todo']:
        st['todo'] = _goes(g)
        live = {_key(verb, box) for verb, box in st['todo']}
        st['docks'] = {k: d for k, d in st['docks'].items() if k in live}
    if not st['todo']:
        return
    verb, box = st['todo'].pop(0)
    standoff = maneuver.standoff(verb, g['suite']) if verb in maneuver.PLAYED else 0.0
    d = g['world'].data
    planner.ask(g['t'], {'kind': 'peek', 'at': (float(d.qpos[0]), float(d.qpos[2])), 'things': list(g['things']),
                         'floor': min(float(f['sole'][1]) for f in g['s']['feet']), 'dock': (verb, box, standoff)})
    st['asked'] = (st['epoch'], verb, box)


def told(s):
    """[{'kind', 'slot', 'at', 'chosen'}] her peeks' docks (`peeked`), her errand's (`dock.shown`) in place
    of its thing's."""
    from machine.way import dock
    mine = dock.shown(s['g'].get('_errand'))
    peeks = [d for d in (s.get('peek') or {}).get('docks', {}).values() if d]
    return [d for d in peeks if not mine or (d['kind'], d['slot']) != (mine['kind'], mine['slot'])] + ([mine] if mine else [])
