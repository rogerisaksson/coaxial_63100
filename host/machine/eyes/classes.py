"""What the robot's eyes take a thing for: its class by look and box (the text's semantic classifier).

    names, whole = classes.of(kinds, centre, half, R, clear)   # each one's class, the thing each is a piece of

A thing is a box with a kind (`registry`'s): the robot's eyes' look - the palette's nearest colour (`sight.LOOKS`)
-, the stand-in's (`world.props()`), or a class already - a scene's chair, its registry's slot -, which stays. A
look that tells its class alone is it (LOOKED: a door's, a bed's, a lamp's); a material many things are made of -
wood, timber, stone, glass - is told by its box against the robot's figure and the text's slabs and kernels
(`lie_group_slam_core.lisp`'s `:semantic-affordance-classifier`), every thing's arrays at once, each table in its
order, the first that holds: a top seen clear over it by its height and what it meets (TOPS: a floor, a stair, a
step, a platform, a sofa, a chair, a table, a bench, a stool, a cabinet, a shelf), the rest by its faces (FACES: a
window and the posts framing it, a railing, a ceiling, a wall, a post); a piece of a raised top of its look - a
chair's back on its edge, a leg or a face under it - the top's class, the top its whole. What none tells is a
thing, named none.

Of what the robot tells, a building (`houses`, `wholes.buildings`): its shell - walls, windows and doors, joined across what the
robot's eyes may not have seen of it - with an opening in it, enclosing a floor, and what roofs it over; a cabin
of one storey, else a house. Outside it what lies by it is its veranda - a floor - or its terrace - a platform -;
a floor apart from it or a top sunk under the robot's floor the ground.

What is one thing, many to one (`wholes.grouped`): what of one class touches, slabs across each other apart - a window's
panes and the posts between them, a stair's treads, not two walls at a corner -; a building's walls on one side one
wall; a piece of furniture its top's, two chairs side by side two.
"""
import os

os.environ.setdefault('OPENBLAS_NUM_THREADS', '1')

import numpy as np

from machine.body import figure
from machine.eyes.sight import ALPHA
from machine.eyes.voxels import CELL_M, FAR_M
from machine.texts import kernel

#: The text's elevation windows over the floor by affordance (`evaluate-situational-affordances`), and its slabs' tops
#: (`configure-foliated-sdf-bands`), m.
_WINDOWS = {k['affordance']: tuple(1e-3 * z for z in k['elevation-window-z-mm'])
            for k in kernel.node('evaluate-situational-affordances', 'affordances')['morphological-kernels']
            if 'elevation-window-z-mm' in k}
_SLAB_TOPS = [1e-3 * s.args['z-window-mm'][1] for s in kernel.node('configure-foliated-sdf-bands', 'sdf-bands')['slabs']]

#: What a look tells alone (`sight.LOOKS`; the stand-in's lamp lit): its class, None a material many things are made
#: of, told by their boxes; a kind no look is stays, a class already - one of BORNE's where its box bears it out.
LOOKED = {'glass': None, 'wood': None, 'timber': None, 'stone': None, 'bed': 'bed', 'door': 'door', 'lamp': 'lamp',
          'lit': 'lamp', 'switch': 'switch', 'roof': 'roof', 'rail': 'railing'}
#: A face seen from one side fills a voxel across, from both two: a wall, a pane, a chair's back or a leg is no
#: thicker, a seat (0.44 m) is - m.
THIN = 2.0 * CELL_M
#: Over the floor, m: flush, under the robot's ankle, it walks on it (`voxels.STAND_M`); in the text's
#: lowest slab, the robot's feet's, it steps onto it, and onto a rise of it from one the robot does; a
#: top's piece stands on it with its foot no higher over it than the mid-pelvic slab; past the highest, the
#: robot's head's, a thing is overhead.
FLUSH, STEP, OVER, OVERHEAD = figure.ANKLE_H, _SLAB_TOPS[0], _SLAB_TOPS[1], _SLAB_TOPS[2]
#: A seat's top (`:sit-on-fixture`) and a bed's (`:lie-supine-on-fixture`), m; past what the robot braces on
#: (`:recruit-brace-support`) a face parts the room and a top is a cabinet's or a shelf's.
SEAT, BED, BRACE = _WINDOWS[':sit-on-fixture'], _WINDOWS[':lie-supine-on-fixture'], _WINDOWS[':recruit-brace-support'][1]
#: The robot's lap sat at its popliteal height (`floor.CHAIR_LOW_H`), its knee and its
#: thigh's radius - a table's top is over it, a seat's under -; the robot's lumbar over that seat
#: (`affordance.LUMBAR_M`), a chair's back reaching it - m.
LAP = figure.ANKLE_H + figure.SHANK + [c for c in figure.BODY if c[0] == 'thigh'][-1][1]
LUMBAR = (next(s for s in figure.SEGMENTS if s[0] == 'torso')[3][1]
          + next(c for c in figure.BODY if c[0] == 'torso')[2][1] - (figure.SEAT_Y - figure.SEAT_R))
#: Two of the robot's soles (heel, ball and toes, 24 cm), m: a top shallower takes one foot - a stair's tread -, one
#: as deep it stands on whole - a platform -, a seat as deep seats none alone - a low table's.
STANCE = 2.0 * (figure.HEEL + figure.BALL + figure.TOE_M)
#: A look that tells its class only where its box (top, bottom) bears it out, and the material it is taken for where
#: not: a door's standing from the lowest slab past the robot's brace - shaded timber reads as one, pixel by pixel -;
#: a bed's lying under its brace - a glass gable reads as one against the room behind it.
BORNE = {'door': (lambda top, bottom: bottom <= OVER and top > BRACE, 'timber'),
         'bed': (lambda top, bottom: BED[0] <= top <= BED[1], 'glass')}

#: A top's class by its features (`of`), the first that holds: flush with the floor; a shallow top the robot steps
#: onto, a rise off another - a stair's tread - or alone; one it stands on whole, or as deep and solid under
#: it within the robot's brace - its body, or a face under it along half its length; a seat with a back, twice as
#: long as deep for more than the robot; a top over its lap it braces on, or one at a seat's height
#: too deep to sit on alone; a seat twice as long as deep; a seat; past its brace a solid top, else one
#: held up.
TOPS = (('floor', lambda f: np.abs(f['top']) <= FLUSH),
        ('stairs', lambda f: f['steps'] & (f['short'] < STANCE) & f['flight']),
        ('step', lambda f: f['steps'] & (f['short'] < STANCE)),
        ('platform', lambda f: f['steps'] | (f['solid'] & (f['short'] >= STANCE) & (f['top'] <= BRACE))),
        ('sofa', lambda f: f['wide'] & f['seat'] & f['backed'] & (f['long'] >= 2.0 * np.maximum(f['short'], STANCE))),
        ('chair', lambda f: f['wide'] & f['seat'] & f['backed']),
        ('table', lambda f: f['wide'] & (((f['top'] > LAP) & (f['top'] <= BRACE))
                                         | (f['seat'] & (f['short'] >= STANCE)))),
        ('bench', lambda f: f['wide'] & f['seat'] & (f['long'] >= 2.0 * np.maximum(f['short'], STANCE))),
        ('stool', lambda f: f['wide'] & f['seat']),
        ('cabinet', lambda f: f['wide'] & f['solid'] & (f['top'] > BRACE)),
        ('shelf', lambda f: f['wide'] & (f['top'] > BRACE)))
#: The class of a thing no top by its faces, the first that holds: seen through past the robot's brace, or a post
#: thin both ways framing such a pane; seen through under it; overhead and wider than THIN; past the robot's brace,
#: longer than THIN or a sliver beside an opaque one standing through its brace - a wall's fragment of
#: another colour -, else as thin both ways.
FACES = (('window', lambda f: (f['see'] | (f['framed'] & (f['long'] <= THIN + 1e-9))) & (f['top'] > BRACE)),
         ('railing', lambda f: f['see']),
         ('ceiling', lambda f: (f['bottom'] > OVERHEAD) & (f['short'] > THIN + 1e-9)),
         ('wall', lambda f: (f['top'] > BRACE) & ((f['long'] > THIN + 1e-9) | f['walled'])),
         ('post', lambda f: f['top'] > BRACE))
#: The tops that are furniture: a piece of one other than its top - a chair's back, a leg - is no thing of its own.
FURNITURE = ('sofa', 'chair', 'table', 'bench', 'stool', 'cabinet', 'shelf')
#: A building's shell, joined where its things meet; an opening in it makes it one - a garden's wall has none -; what
#: roofs it over beside it its too; what stands it a storey high, a cabin's walls and doors under two of the robot.
SHELL, OPENINGS, ROOFS, STOREY = ('wall', 'window', 'door'), ('window', 'door'), ('roof', 'ceiling'), ('wall', 'door')
#: What the robot sees through, in its map in pieces where its eyes met its glass: one in one
#: plane where no wall stands between.
THROUGH = ('window', 'railing')


def meet(centre, e):
    """(n, n) whether boxes' footprints - centres and half extents along the room's axes (n, 3) - meet a cell apart."""
    gap, reach = np.abs(centre[:, None, ::2] - centre[None, :, ::2]), e[:, None, ::2] + e[None, :, ::2]
    return (gap <= reach + CELL_M + 1e-9).all(axis=2) & ~np.eye(len(centre), dtype=bool)


def planar(lo, hi):
    """(n, n) whether boxes - corners `lo`, `hi` (n, 3) - lie in one wall's plane: each THIN across one floor axis, the
    two a cell apart across it."""
    middle, flat = 0.5 * (lo + hi)[:, ::2], 0.5 * (hi - lo)[:, ::2] <= 0.5 * THIN + 1e-9
    return (flat[:, None] & flat[None, :] & (np.abs(middle[:, None] - middle[None, :]) <= CELL_M + 1e-9)).any(axis=2)


def houses(names, beside, lo, hi):
    """[(class, low corner (3,), high corner (3,), members (k,))] of the buildings things told `names` make -
    `beside` (n, n) which meet, `lo`, `hi` (n, 3) their boxes' corners -: each its SHELL's things joined
    across what the robot's eyes may not have seen of it, the robot's map's reach (`voxels.FAR_M`: a door's
    gap, a wall's far end), or in one wall's plane, an opening among them and more - an opening alone, a lone
    door, is in nothing -, enclosing a floor - its box deeper than THIN both ways: a facade alone has no
    inside -, what roofs them over (ROOFS beside them) its too; a cabin where its STOREY stands under two of
    the robot (OVERHEAD twice), else a house; its box its shell's."""
    n = len(names)
    shell, opening, roof, storey = (np.array([k in kinds for k in names], bool) for kinds in (SHELL, OPENINGS, ROOFS, STOREY))
    middle, half = 0.5 * (lo + hi)[:, ::2], 0.5 * (hi - lo)[:, ::2]
    near = (np.abs(middle[:, None] - middle[None, :]) - half[:, None] - half[None, :] <= FAR_M + 1e-9).all(axis=2)
    plane = planar(lo, hi)
    label = np.where(shell, np.arange(n), n)
    while True:
        low = np.where(shell, np.minimum(label, np.where((near | plane) & shell[None, :], label[None, :], n).min(axis=1)), n)
        if (low == label).all():
            break
        label = low
    out = []
    for k in np.unique(label[opening]):
        part = label == k
        if part.sum() < 2 or ((hi[part].max(axis=0) - lo[part].min(axis=0))[::2] <= THIN + 1e-9).any():
            continue
        out.append(('cabin' if hi[part & storey, 1].max(initial=0.0) <= 2.0 * OVERHEAD else 'house', lo[part].min(axis=0),
                    hi[part].max(axis=0), np.flatnonzero(part | (roof & (beside & part[None, :]).any(axis=1)))))
    return out


def _first(table, f, n):
    """(n,) each thing's name in `table` [(name, test of the features `f`)], the first whose test holds; 'thing'
    where none does."""
    held = np.array([np.broadcast_to(test(f), (n,)) for _name, test in table] + [np.ones(n, bool)])
    return np.array([name for name, _test in table] + ['thing'], object)[held.argmax(axis=0)]


#: The classes `of` tells, as wbc_classes.h numbers them: the tops, the faces, a thing, what lies by a building.
CLASSES = (tuple(name for name, _test in TOPS) + tuple(name for name, _test in FACES)
           + ('thing', 'veranda', 'terrace', 'ground'))
#: Each kind's code and flags for the C (`tools.cores.classes.BITS`), told at its first look; the told name each
#: code is; the C laid at its first look, False where no host compiler builds it.
_CODES: dict = {}
_TOLD: list = []
_CORE: list = [None]


def _coded(kind):
    """(code, flags) of `kind` for the C: what its look tells alone (LOOKED), seen through (`sight.ALPHA`), furniture,
    a building's shell, an opening, a roof, a storey's, a door's or a bed's."""
    got = _CODES.get(kind)
    if got is None:
        from tools.cores.classes import BITS
        told = LOOKED.get(kind, kind)
        flags = sum(BITS[name] for name, held in (('raw', told is None), ('see', ALPHA.get(kind, 1.0) < 1.0),
                                                   ('furn', kind in FURNITURE), ('shell', told in SHELL),
                                                   ('open', told in OPENINGS), ('roof', told in ROOFS),
                                                   ('storey', told in STOREY), ('door', kind == 'door'),
                                                   ('bed', kind == 'bed')) if held)
        got = _CODES[kind] = (len(_TOLD), flags)
        _TOLD.append(told)
    return got


def core():
    """`of` in C (`tools.cores.classes`) - `wholes.grouped` beside it (`.grouped`) -, laid at its first look with the text's heights as this module lays them
    - each sum here, as numpy sums it -; None where the C is not to be had (it stays numpy)."""
    if _CORE[0] is None:
        try:
            from tools.cores import classes as c_classes, registry as c_registry
            timber, glass = _coded('timber'), _coded('glass')
            _CORE[0] = c_classes.laid(flush=FLUSH, step=STEP, over=OVER, overhead=OVERHEAD, seat0=SEAT[0], seat1=SEAT[1],
                                      bed0=BED[0], bed1=BED[1], brace=BRACE, lap=LAP, lumbar=LUMBAR, stance=STANCE,
                                      thin_eps=THIN + 1e-9, cell=CELL_M, eps=1e-9, brace_half=BRACE + 0.5 * CELL_M,
                                      far_eps=FAR_M + 1e-9, half_thin_eps=0.5 * THIN + 1e-9, cell_eps=CELL_M + 1e-9,
                                      lane=c_registry.lanes()[2], timber=timber[0], timber_flags=timber[1],
                                      glass=glass[0], glass_flags=glass[1], spare=0)
        except (OSError, RuntimeError, ValueError) as e:
            import sys
            print('classes.of stays numpy: %s' % e, file=sys.stderr)
            _CORE[0] = False
    return _CORE[0] or None


def of(kinds, centre, half, R, clear, in_c=True):
    """(names [str], whole (n,)) of `n` things - kinds, centres and half sizes (n, 3), turns (n, 3, 3), whether the
    space over each is seen clear (n,) -: each one's class, and the thing each is a piece of, its own index where it
    is whole. In C where `in_c` (`tools.cores.classes`, wbc_classes.c: the lodge's 70 things 1.1 ms of numpy a
    look, every look of a thing that moved), numpy's own below its twin (`_of`), bit for bit."""
    from tools.cores.classes import MOST
    told = core() if in_c and 0 < len(kinds) <= MOST else None
    if told is None:
        return _of(kinds, centre, half, R, clear)
    coded = [_coded(str(k)) for k in kinds]
    cls, of_, whole = told(centre, half, R, clear, [c for c, _f in coded], [f for _c, f in coded])
    return [CLASSES[k] if k >= 0 else _TOLD[o] for k, o in zip(cls.tolist(), of_.tolist())], whole.astype(int)


def _of(kinds, centre, half, R, clear):
    """`of` in numpy, its twin."""
    n = len(kinds)
    if not n:
        return [], np.zeros(0, int)
    e = np.einsum('nij,nj->ni', np.abs(R), half)               # half extents along the room's axes
    top, bottom = centre[:, 1] + e[:, 1], centre[:, 1] - e[:, 1]
    kinds = [BORNE[k][1] if k in BORNE and not BORNE[k][0](t, b) else k for k, t, b in zip(kinds, top, bottom)]
    told = [LOOKED.get(k, k) for k in kinds]
    raw, see = np.array([t is None for t in told]), np.array([ALPHA.get(k, 1.0) < 1.0 for k in kinds])
    look = np.unique(np.array(kinds, str), return_inverse=True)[1]
    long, short = 2.0 * half[:, ::2].max(axis=1), 2.0 * half[:, ::2].min(axis=1)
    surface, wide = raw & (clear > 0.0), short > THIN + 1e-9
    gap, reach, beside = np.abs(centre[:, None, ::2] - centre[None, :, ::2]), e[:, None, ::2], meet(centre, e)
    within = (gap + reach <= e[None, :, ::2] + CELL_M + 1e-9).all(axis=2) & ~np.eye(n, dtype=bool)
    # a piece of a raised top of its look: on its edge, its foot from the top's surface into the slab over it - a
    # chair's back -, or under it within its footprint - a leg, a face of what holds it up
    wood = raw | np.isin(kinds, FURNITURE)
    top_ = wood & (clear > 0.0) & wide & (top > FLUSH)
    pair = (wood & ~see & ~top_)[:, None] & top_[None, :] & (look[:, None] == look[None, :])
    on = pair & beside & (bottom[:, None] >= top[None, :] - CELL_M - 1e-9) & (bottom[:, None] <= top[None, :] + OVER)
    under = pair & within & (top[:, None] <= bottom[None, :] + CELL_M + 1e-9)
    # the robot's backrest at a seat's edge, the text's (`:verify-high-thoracic-slab-occupancy`,
    # `affordance.back_of`'s): a piece of its look beside it from no more than a cell over its top past the robot's
    # lumbar, its foot anywhere under - a chair's back is one with its rear legs, a low chair's with the wall it
    # stands against
    backs = (pair & beside & (bottom[:, None] <= top[None, :] + CELL_M + 1e-9)
             & (top[:, None] >= top[None, :] + LUMBAR) & (top[:, None] <= BRACE + 0.5 * CELL_M)
             & (long[:, None] <= long[None, :] + (THIN + 1e-9)))
    held = on | under | backs
    # what the robot steps onto: a top no piece in the lowest slab, then each beside one it does a rise
    # of it off it
    free = surface & ~held.any(axis=1)
    rise = beside & free[:, None] & free[None, :] & (np.abs(top[:, None] - top[None, :]) <= STEP)
    steps = free & (top <= STEP)
    while True:
        more = steps | (rise & steps[None, :]).any(axis=1)
        if (more == steps).all():
            break
        steps = more
    f = {'top': top, 'bottom': bottom, 'long': long, 'short': short, 'wide': wide, 'see': see, 'steps': steps,
         'flight': (rise & steps[None, :] & (np.abs(top[:, None] - top[None, :]) > FLUSH)).any(axis=1),
         'solid': (bottom <= FLUSH) | (np.where(under & ~surface[:, None], long[:, None], 0.0).max(axis=0) >= 0.5 * long),
         'seat': (top >= SEAT[0]) & (top <= SEAT[1]), 'framed': (beside & see[None, :]).any(axis=1),
         'walled': (beside & (~see & (long > THIN + 1e-9) & (bottom <= BRACE) & (top > BRACE))[None, :]).any(axis=1),
         'backed': backs.any(axis=0)}
    names, tops = np.array(told, object), _first(TOPS, f, n)
    face = raw & ~(surface & (tops != 'thing'))
    names[surface], names[face] = tops[surface], _first(FACES, f, n)[face]
    whole = np.where(held.any(axis=1), held.argmax(axis=1), np.arange(n))
    names, built = names[whole], houses(names[whole], beside, centre - e, centre + e)
    if built:
        # outside a building by it a floor its veranda, a platform its terrace; a floor apart, a platform sunk, the ground
        flo, fhi = (np.array([b[j] for b in built])[:, ::2] for j in (1, 2))
        xz, exz = centre[:, None, ::2], e[:, None, ::2]
        out = ~((xz >= flo[None]) & (xz <= fhi[None])).all(axis=2).any(axis=1)
        by = out & ((xz + exz >= flo[None] - CELL_M - 1e-9) & (xz - exz <= fhi[None] + CELL_M + 1e-9)).all(axis=2).any(axis=1)
        floor, platform = names == 'floor', names == 'platform'
        names = np.where(by & floor, 'veranda', np.where(by & platform & (top >= -FLUSH), 'terrace', np.where(
            out & ((floor & ~by) | (platform & (top < -FLUSH))), 'ground', names)))
    return names.tolist(), whole


def stacked(things):
    """(kinds, centres (n, 3), half sizes (n, 3), turns (n, 3, 3), clear (n,)) of boxes `things`, as `registry` takes
    them: a box without a turn upright, without a word of what is over it clear; the robot's things' numbers
    (`registry.Things`, their arrays beside their rows) copied, not built again of their rows' floats - 141 us of
    the lodge's 93 a state told its page."""
    arrays = getattr(things, 'arrays', None)
    if arrays is not None and len(arrays[0]) == len(things):
        centre, half, R = (np.array(a, float) for a in arrays[:3])
        return ([str(t[0]) for t in things], centre, half, R, np.array([float(t[5]) if len(t) > 5 else 1.0 for t in things]))
    return ([str(t[0]) for t in things], np.array([t[1] for t in things], float).reshape(-1, 3),
            np.array([t[2] for t in things], float).reshape(-1, 3),
            np.array([t[3] if len(t) > 3 else np.eye(3) for t in things], float).reshape(-1, 3, 3),
            np.array([float(t[5]) if len(t) > 5 else 1.0 for t in things]))
