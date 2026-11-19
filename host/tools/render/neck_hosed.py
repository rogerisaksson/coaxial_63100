"""The neck's leads: the head's motor's in a hose across the neck's joint, the neck's motor's by it.

    told = neck_hosed.checked()        # the hose's bends, its holds, its air through the neck's travel
    parts = neck_hosed.posed(nod)      # {name: tris}, the torso's frame, the neck nodded `nod` deg

The two drives' boards are the twin in the bay on the spine's tube (`neck_mount`): what goes up from it are motors'
leads, no bus. The head's motor turns the head with some 5 A at the most the robot was seen to ask (7 N m through
its gearbox): its three phases a square millimetre each - the bus's core, twice over - and its sensor's lead go in
the robot's hose (`materials.toml` [hoses]), as a bus does. From a pipe on the bay's guard the hose climbs the
block's back, clipped, and bends over its crossbar's band to an elbow lying on that band beside the neck's motor's
back, its mouth up under the neck's axis; free from there, across the axis (`hung`), to a pipe under the crown's
floor, the leads on through the floor to the motor standing on it. The hose is straight with the neck upright; its
still mouth stands as far under the axis as leaves it an arc at the travel's farther end (`under`), no S. The
neck's motor's leads cross no joint: a braided sleeve from the bay's other pipe straight up the two bands' backs,
beside the hose, to a cap round the neck's band's end onto its motor's back. Frame: the torso's (x left, y up, z
ahead); m.
"""
import functools
import math

from tools.render import elbow_mount as em
from tools.render import knee_mount as km
from tools.render import neck_laid as nl
from tools.render import neck_mount as nm
from tools.render import tees

#: The sleeve round the neck's motor's leads - three phases' and its sensor's -: its radius, m, and its ink; the
#: cap over its end: how far over the neck's axis the leads go in under it, its height, how far it stands off
#: the band's back and off the motor's, m.
SLEEVE_M, SLEEVE = 0.0052, (40, 42, 48)
INTO_M, CAP_M = 0.010, (0.018, 0.014, 0.011)


def plane():
    """The hose's plane beside the neck's motor's back, the torso's x, m: a clip's ring clear of that back."""
    return nl.laid()['motor'][0] - 0.5 * nl.AIR_M - (tees.HOSE_R + tees.CLIP[0] + tees.CLIP[1])


def under(reach):
    """How far under the neck's axis the hose's still mouth stands, m, its turning one `reach` over it: where the
    hose, straight with the neck upright, is at the travel's farther end as much longer than its mouths stand
    apart as an arc of that turn is than its chord - an arc there, no S."""
    from machine.body import drives
    half = math.radians(max(abs(t) for t in drives.TRAVEL['neck']))
    short = 1.0 - math.sin(0.5 * half) / (0.5 * half)
    q = (1.0 - (1.0 - short) * (1.0 - short)) / (2.0 * (1.0 - math.cos(half)))
    share = 0.5 * (1.0 - math.sqrt(1.0 - 4.0 * q))
    return reach * share / (1.0 - share)


def _fitting(end, way):
    """The fitting a hose goes on, its end at `end`, the hose leaving along `way`."""
    import numpy as np
    end, way = np.asarray(end, float), np.asarray(way, float)
    return km._rod(end - tees.FITTING[0] * way, end, tees.FITTING[1], tees.PIPE_INK)


def _stub(root, way):
    """([(tri, colour)], its mouth): a straight pipe out of a face at `root` along `way`, a collar round its root
    and a fitting at its end."""
    import numpy as np
    root, way = np.asarray(root, float), np.asarray(way, float)
    end = root + (tees.STUB_M + tees.PIPE_LEG) * way
    return (km._rod(root - 0.003 * way, root + 0.0012 * way, tees.COLLAR_R, tees.PIPE_INK)
            + km._rod(root, end, tees.PIPE[1], tees.PIPE_INK) + _fitting(end, way)), end


def _elbow(mouth, up, back):
    """([(tri, colour)], its other end): a pipe's quarter bend - its mouth at `mouth`, the hose leaving it along
    `up`; a leg down from it, the bend round toward `back`, a leg on and the fitting the hose from there goes on."""
    import numpy as np
    mouth, up, back = (np.asarray(q, float) for q in (mouth, up, back))
    rb, r = tees.PIPE
    start = mouth - tees.PIPE_LEG * up
    centre = start + rb * back
    path = [mouth] + [centre - rb * (math.cos(t) * back + math.sin(t) * up) for t in np.linspace(0.0, 0.5 * math.pi, 13)]
    end = path[-1] + tees.PIPE_LEG * back
    tris = km._wire(path + [end], r, tees.PIPE_INK)
    return tris + _fitting(mouth, up) + _fitting(end, back), end


@functools.lru_cache(maxsize=None)
def fitted():
    """{'a', 'b': the hose's still mouth and its turning one, the neck upright, 'in': the elbow's other end, 'bay',
    'twin': the mouths of the bay's two pipes - the head's hose's in the hose's plane, the neck's sleeve's two
    fittings' width nearer the trunk's middle, both behind the crossbar's band by the hose's air -, 'elbow', 'hold',
    'stubs', 'nods': the tris of the elbow on the block, the clip that holds it on the crossbar's band, the bay's
    two pipes and the one under the crown's floor}."""
    import numpy as np
    lay = nl.laid()
    (ny, nz), x = lay['neck'], plane()
    low = lay['plate'][0]
    floor, b = _stub((x, low, nz), (0.0, -1.0, 0.0))
    clear = lay['bar'][0] + lay['bore'] + tees.FITTING[1] + tees.PIPE[0] + tees.PIPE_LEG + 0.5 * nl.AIR_M
    a = np.array([x, max(ny - under(float(b[1]) - ny), clear), nz])      # no lower than its elbow lies on the band
    elbow, come = _elbow(a, (0.0, 1.0, 0.0), (0.0, 0.0, -1.0))
    at = nm.bayed()[0]
    guard = np.array([v for tri, _c in nm.still()['bay guard'] for v in tri])
    wall = float(np.hypot(guard[:, 0], guard[:, 1] - at[1]).max())
    behind = lay['bar'][1] - lay['bore'] - nl.AIR_M - tees.HOSE_R
    pipes, mouths = [], []
    for over in (x, x + 2.0 * tees.FITTING[1] + 0.5 * nl.AIR_M):           # the guard's wall's top there
        pipe, mouth = _stub((over, at[1] + math.sqrt(wall * wall - over * over), behind), (0.0, 1.0, 0.0))
        pipes, mouths = pipes + pipe, mouths + [mouth]
    held = come + (0.0, 0.0, 0.5 * tees.PIPE_LEG)       # the elbow's leg over the band, and the band's top under it
    top = lay['bar'][0] + math.sqrt(lay['bore'] * lay['bore'] - (held[2] - lay['bar'][1]) * (held[2] - lay['bar'][1]))
    hold = tees.clip(held, (0.0, 0.0, 1.0), (x, top - 0.001, held[2]), tees.PIPE[1])
    return {'a': a, 'b': np.asarray(b), 'in': come, 'bay': mouths[0], 'twin': mouths[1], 'elbow': elbow, 'hold': hold,
            'stubs': pipes, 'nods': floor}


@functools.lru_cache(maxsize=None)
def span():
    """The hose's span across the neck's joint in the neck's frame (`hung`): from the elbow's mouth on the block
    to the pipe's under the crown's floor, each aimed at the other across the axis with the neck upright; the
    neck's motor and its band, cylinders it keeps off. `hung` lays a hose where its joint stands at 0 and turns it
    from there - and a hose laid straight, a little longer than its mouths stand apart, folds on itself: so the
    span's 0 is the neck nodded back to its travel's end, its turns the neck's nod past that (`hung_at`)."""
    import numpy as np
    from machine.body import drives
    lay, fit = nl.laid(), fitted()
    origin = (0.0, lay['neck'][0], lay['neck'][1])
    lo, hi = drives.TRAVEL['neck']
    back = km._rx(lo)
    drums = [(lay['motor'][0], lay['motor'][1], lay['motor'][2]), (lay['band'][0], lay['band'][1], lay['band'][2])]
    return {'name': 'neck', 'kind': 'bus', 'travel': (0.0, hi - lo),
            'from': ('still', tuple(float(v) for v in fit['a'] - origin), (0.0, 1.0, 0.0)),
            'to': ('turning', tuple(float(v) for v in back @ (fit['b'] - origin)),
                   tuple(float(v) for v in back @ np.array([0.0, -1.0, 0.0]))),
            'solids': [('still', (0.5 * (x0 + x1), 0.0, 0.0), (1.0, 0.0, 0.0), r + nl.AIR_M, 0.5 * (x1 - x0))
                       for x0, x1, r in drums]}


def hung_at(nod):
    """(k, 3): the hose where it hangs free with the neck nodded `nod` deg, the torso's frame."""
    from machine.body import drives
    from tools.render import hung
    lay = nl.laid()
    return hung.path(span(), float(nod) - drives.TRAVEL['neck'][0]) + (0.0, lay['neck'][0], lay['neck'][1])


@functools.lru_cache(maxsize=None)
def lines():
    """{'hose': the head's hose's still stretch - from the bay's pipe up the crossbar's band's back and over it
    to the elbow's fitting -, 'sleeve': the neck's leads', straight up from the bay's other pipe to their cap}:
    each (k, 3)."""
    import numpy as np
    lay, fit = nl.laid(), fitted()
    up, ahead = np.array([0.0, 1.0, 0.0]), np.array([0.0, 0.0, 1.0])
    hose = tees.run([(fit['bay'], up), (np.array([fit['bay'][0], lay['bar'][0], fit['bay'][2]]), up),
                     (fit['in'], ahead)])
    top = np.array([fit['twin'][0], lay['neck'][0] + INTO_M, fit['twin'][2]])
    count = int(np.linalg.norm(top - fit['twin']) / tees.PITCH) + 1
    return {'hose': hose, 'sleeve': np.linspace(fit['twin'], top, count)}


@functools.lru_cache(maxsize=None)
def still():
    """{name: tris} of what stays with the torso: 'pipes' - the bay's two, the elbow on the block and its hold -,
    'head hose' - its still stretch and its clips -, 'neck leads' - their sleeve, its clips and the cap over its
    end on the neck's motor's back."""
    lay, line = nl.laid(), lines()
    (ny, nz), back, out = lay['neck'], lay['motor'][0], lay['band'][2]
    on = [t for name in ('block', 'spine', 'bay guard') for t in nm.still()[name]]
    hose = tees.hose(line['hose'])
    sleeve = km._rod(line['sleeve'][0], line['sleeve'][-1], SLEEVE_M, SLEEVE)
    end = line['sleeve'][-1]
    low, high = ny + INTO_M - 0.002, ny + INTO_M - 0.002 + CAP_M[0]
    rear = nz - math.sqrt(out * out - (low - ny) * (low - ny))           # the band's back where the cap begins
    wide = [(back - CAP_M[2], rear - CAP_M[1]), (end[0] + SLEEVE_M + 0.002, rear - CAP_M[1]),
            (end[0] + SLEEVE_M + 0.002, rear - 0.0005), (back - CAP_M[2], rear - 0.0005)]     # (x, z): behind the band
    side = [(back - CAP_M[2], rear - 0.0005), (back - 0.0005, rear - 0.0005), (back - 0.0005, rear + CAP_M[1]),
            (back - CAP_M[2], rear + CAP_M[1])]                           # round the band's end, onto the motor's back
    cap = [t for outline in (wide, side) for t in em._rounded(outline, low, high, em.FRAME, axis=1, r=0.002)]
    fit = fitted()
    return {'pipes': fit['elbow'] + fit['hold'] + fit['stubs'], 'head hose': hose + tees.clipped(line['hose'], on),
            'neck leads': sleeve + tees.clipped(line['sleeve'], on, SLEEVE_M) + cap}


def posed(nod=0.0):
    """{name: tris}, the neck nodded `nod` deg: what stays, the pipe under the crown's floor nodded with it, and
    the hose between hung as the joint has turned it (`hung_at`)."""
    lay = nl.laid()
    out = dict(still())
    out['floor pipe'] = nm._spun(fitted()['nods'], nod, (0.0, lay['neck'][0], lay['neck'][1]), 'x')
    out['neck hose'] = tees.hose(hung_at(nod))
    return out


@functools.lru_cache(maxsize=None)
def aired():
    """The least air round the hose where it hangs free, through the neck's travel (`hung.turns`), m: off what
    stays - the block, the neck's drive, the shoulders, the sleeve and its cap - and off what nods, nodded as far."""
    import numpy as np
    from scipy.spatial import KDTree
    from machine.body import drives
    from tools.render import hung
    lay = nl.laid()
    origin = np.array([0.0, lay['neck'][0], lay['neck'][1]])
    fixed = KDTree(km._points([t for name in ('block', 'neck drive', 'left shoulder', 'right shoulder')
                                for t in nm.still()[name]] + still()['neck leads'], 0.001))
    nods = km._points([t for name in ('neck plate', 'crown', 'shoe', 'head drive') for t in nm.nodding()[name]],
                      0.001) - origin
    least = math.inf
    for nod in (turn + drives.TRAVEL['neck'][0] for turn in hung.turns(span())):
        path = hung_at(nod)
        free = path[np.linalg.norm(path - path[0], axis=1) > tees.FITTING[0]]       # past the elbow's own fitting
        moved = KDTree(nods @ km._rx(float(nod)).T + origin)
        least = min(least, float(fixed.query(free)[0].min()), float(moved.query(free)[0].min()))
    return least - tees.HOSE_R


def checked():
    """{'bend': (the least radius the hose bends at through the neck's travel, the hose's own least) mm, 'laid':
    the least its stretch laid still bends at, mm - the sleeve lies straight -, 'free': (the longest a laid
    stretch lies free between holds, the longest that may be) mm, 'air': the hose's least where it hangs free,
    through the travel, mm, 'crossing': triangle pairs of the hoses, the sleeve, its cap and the pipes through the
    mount's parts, the neck upright - a clip stands on what carries it, and a pipe in it -, 'skin': how far inside
    the likeness's head's skin, turned past it, the floor's pipe and the hose under it keep, mm - None with no
    likeness beside the tree}: what the suite holds."""
    from coaxial.graphics import likeness
    from machine.body import mechatronics
    from tools.render import clips, hung
    parts, drawn, fit = nm.posed(), posed(), fitted()

    def crossing(tris, but=()):
        own = [t for t in tris if tuple(t[1]) != tees.CLIP_INK]
        return clips.crossed(own, [t for name, part in parts.items() if name not in but for t in part])
    crossed = (crossing(drawn['head hose'] + drawn['neck leads'] + drawn['neck hose'] + fit['elbow'])
               + crossing(fit['stubs'], ('bay', 'bay guard')) + crossing(drawn['floor pipe'], ('crown',)))
    laid_ = tees.tightest(drawn['head hose'])
    hose = float(mechatronics.materials()['hoses']['bus'][4])
    return {'bend': (round(1e3 * hung.least(span()), 1), 1e3 * hose), 'laid': round(1e3 * laid_, 1),
            'free': (round(1e3 * tees.loosest(drawn['head hose'] + drawn['pipes']), 1), round(1e3 * tees.unheld(), 1)),
            'air': round(1e3 * aired(), 2), 'crossing': crossed,
            'skin': (round(1e3 * nm.skin(drawn['floor pipe'] + drawn['neck hose'], parts['carrier'])[0], 2)
                     if likeness.FOUND else None)}
