"""A printed piece shaped by its loads: material only where they ask it, and nothing hung over its bed.

    python -X utf8 tools/render/topo.py            # a trial piece - a bracket on its bed - told

`knee_topo`'s method (SIMP on voxels, a density filter, optimality steps) made general and printable. A piece is a
case (`case`): its room - voxels on its bed, layers up y -, the voxels that must stay (seats, bosses), those it is
held by and its loads, a wrench on a patch of voxels each. Between the density filter and the stiffness stands the
printing filter (`printed`): a voxel no denser than the densest of the nine under it - a smooth least of a smooth
most, layer by layer off the bed, its sensitivities carried back down (`_back`) -, so what is kept stands on what
lies under it within 45 degrees; what the case keeps is solid whatever that filter says, and what is printed may
stand on it. A case `extruded` - a load-bearing slab, printed up its thickness - has each column of voxels one
density in place of that filter: a prism, its pockets through, nothing over its bed. The compliances are solved by
conjugate gradients, each case started from its last round's answer: a direct solve does not end at these sizes.
`shaped` runs a case - kept under .cache/topo by the case's content - and cuts the densities at the highest level
whose solid, a kept voxel left hanging propped from under (`propped`) and what else hangs pruned off, still carries
the loads: the principal stress within the printed polymer's allowable, the stress up the layers within ACROSS of
it, each load taken either way - what a robot bears turns round -, read in the voxels that were the optimiser's to
shape. Told: the solid's voxels, its mass, its margin, what it lost. `faced` draws them - an extruded one's prisms
smooth (`outlined`) -, `knee_topo.stl` writes them. `whole` reads the room all solid the same way: a room that does
not carry solid is its sketch's to fix, not the optimiser's.
"""
import hashlib
import math
import os
import sys
from typing import Any

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import knee_topo as kt  # noqa: E402

#: The penalty, the optimality step's largest move, its rounds, the void's share of the solid's modulus; the
#: printing filter's power, its least's rounding and the density its most is exact at; the levels a solid is cut at.
PENALTY, MOVE, ROUNDS, VOID = 3.0, 0.2, 60, 1e-3
SHARP, EPS, HALF, LEVELS = 40.0, 1e-4, 0.5, (0.5, 0.4, 0.3, 0.2)


def case(name, room, kept, held, loads, h, share, extruded=False):
    """A piece to shape: its `room`, the voxels `kept` whatever the loads and those `held` still - bool arrays
    [across, up, deep] off the bed, `h` mm a voxel -, its `loads`, a case each: [(a patch of voxels, force N (3,),
    moment N mm (3,), the point mm the force acts at and the moment is about)], summed -, the `share` of what is
    its to shape that it may keep, and, `extruded`, the shape a prism up its bed: each column of voxels one
    density, a slab with its pockets through (`outlined`). Of the room only what hangs together with what holds it
    is the piece's: a part cut off carries nothing."""
    import numpy as np
    from scipy import ndimage
    labelled: Any = ndimage.label(room)
    parts = np.asarray(labelled[0], dtype=np.int64)
    room = np.isin(parts, np.unique(parts[held & room & (parts > 0)]))
    return {'name': name, 'room': room, 'kept': kept & room, 'held': held & room, 'loads': loads, 'h': float(h),
            'share': float(share), 'extruded': bool(extruded)}


def filled(drawing, way, h, fine=3):
    """(room [across, up, deep], turn, corner, (x, y, z)): a print's `drawing` [(tri, colour)] filled to `h`-mm
    voxels on its bed - grown along `way`, its frame's rows across, up and deep (`turn`), its low corner there, mm -
    and the voxels' middles in the drawing's frame, mm: its faces marked in cells `fine` to a voxel's side, the room
    round them flooded from outside, what the flood never reached the solid, a voxel solid where half its cells are."""
    import numpy as np
    from scipy import ndimage
    from tools.render import knee_mount as km
    up = np.asarray(way, float)
    across = np.cross(up, (1.0, 0.0, 0.0) if abs(up[0]) < 0.9 else (0.0, 1.0, 0.0))
    turn = np.array((across / np.linalg.norm(across), up, np.cross(across / np.linalg.norm(across), up)))
    cell = h / fine
    points = 1e3 * km._points(drawing, 1e-3 * cell / 3.0) @ turn.T
    corner = points.min(axis=0)
    at = np.floor((points - corner) / cell).astype(int) + fine
    faces = np.zeros(fine * (-(-(at.max(axis=0) + 1 + fine) // fine)), bool)
    faces[tuple(at.T)] = True
    labelled: Any = ndimage.label(~faces)
    flood = np.asarray(labelled[0])
    solid = flood != flood[0, 0, 0]
    n = np.array(solid.shape) // fine
    room = solid.reshape(n[0], fine, n[1], fine, n[2], fine).mean(axis=(1, 3, 5))[1:-1, 1:-1, 1:-1] >= 0.5
    middles = (corner + (np.indices(room.shape).transpose(1, 2, 3, 0) + 0.5) * h) @ turn
    return room, turn, corner, (middles[..., 0], middles[..., 1], middles[..., 2])


def printed(x):
    """(the printed densities, what `_back` needs) of `x` [layer, across, deep], layer 0 on the bed."""
    import numpy as np
    q = SHARP + math.log(9.0) / math.log(HALF)
    out, keep = np.empty_like(x), []
    out[0] = x[0]
    for k in range(1, len(x)):
        pad = np.pad(np.maximum(out[k - 1], VOID), 1, constant_values=VOID)
        under = np.stack([pad[1 + a:pad.shape[0] - 1 + a, 1 + b:pad.shape[1] - 1 + b]
                          for a in (-1, 0, 1) for b in (-1, 0, 1)])
        total = (under ** SHARP).sum(axis=0)
        most = total ** (1.0 / q)
        root = np.sqrt((x[k] - most) ** 2 + EPS)
        out[k] = 0.5 * (x[k] + most - root + math.sqrt(EPS))
        keep.append((under, total, most, root, q))
    return out, keep


def _back(x, keep, sens):
    """d / d x of what `sens` is d / d printed: the printing filter's adjoint, the top layer first."""
    import numpy as np
    lam, out = sens.copy(), np.empty_like(x)
    for k in range(len(x) - 1, 0, -1):
        under, total, most, root, q = keep[k - 1]
        out[k] = lam[k] * 0.5 * (1.0 - (x[k] - most) / root)
        part = lam[k] * 0.5 * (1.0 + (x[k] - most) / root) * (SHARP / q) * total ** (1.0 / q - 1.0)
        down, n = np.zeros((x.shape[1] + 2, x.shape[2] + 2)), 0
        for a in (-1, 0, 1):
            for b in (-1, 0, 1):
                down[1 + a:down.shape[0] - 1 + a, 1 + b:down.shape[1] - 1 + b] += part * under[n] ** (SHARP - 1.0)
                n += 1
        lam[k - 1] += down[1:-1, 1:-1]
    out[0] = lam[0]
    return out


def stood(solid):
    """`solid` [layer, across, deep] with what hangs pruned off: a voxel stays with one of the nine under it."""
    import numpy as np
    out = solid.copy()
    for k in range(1, len(out)):
        pad = np.pad(out[k - 1], 1)
        under = np.zeros_like(out[k])
        for a in (-1, 0, 1):
            for b in (-1, 0, 1):
                under |= pad[1 + a:pad.shape[0] - 1 + a, 1 + b:pad.shape[1] - 1 + b]
        out[k] &= under
    return out


def propped(grid, need, room):
    """`grid` [layer, across, deep] with what holds up each voxel of `need` added: under one standing on none of
    the nine under it, the nearest of the nine in the room under it, and so on down - a prop, solid."""
    import numpy as np
    out, need = grid.copy(), need.copy()
    ways = ((0, 0), (-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1))
    for k in range(len(out) - 1, 0, -1):
        pad, rpad = np.pad(out[k - 1], 1), np.pad(room[k - 1], 1)
        under = np.zeros_like(out[k])
        for a, b in ways:
            under |= pad[1 + a:pad.shape[0] - 1 + a, 1 + b:pad.shape[1] - 1 + b]
        left = out[k] & need[k] & ~under
        for a, b in ways:
            hit = left & rpad[1 + a:rpad.shape[0] - 1 + a, 1 + b:rpad.shape[1] - 1 + b]
            spad = np.pad(hit, 1)
            prop = spad[1 - a:spad.shape[0] - 1 - a, 1 - b:spad.shape[1] - 1 - b]
            out[k - 1] |= prop
            need[k - 1] |= prop
            left &= ~hit
    return out


def outlined(solid, h, fine=4, blur=0.6):
    """(n, 3, 3) mm in the bed's frame: a `solid` [across, up, deep] of `h`-mm voxels drawn as the prisms it
    stacks, smooth - each run of layers alike one prism, its outline the half-way line of its layer blurred
    `blur` of a voxel on a grid `fine` to a voxel's side (`contourpy`), its walls up it, its ends' faces those of
    its outline's points' triangles whose middles lie inside."""
    import contourpy
    import numpy as np
    from scipy import ndimage
    from scipy.spatial import Delaunay
    layers, out, k = solid.transpose(1, 0, 2), [], 0
    while k < len(layers):
        j = k + 1
        while j < len(layers) and np.array_equal(layers[j], layers[k]):
            j += 1
        if layers[k].any():
            field = ndimage.gaussian_filter(np.kron(np.pad(layers[k], 2).astype(float), np.ones((fine, fine))),
                                            blur * fine)
            across, deep = ((np.arange(n) + 0.5) / fine - 2.0 for n in field.shape)
            loops = [h * np.asarray(line) for line in contourpy.contour_generator(x=deep, y=across, z=field).lines(0.5)]
            z0, z1 = k * h, j * h
            for p in loops:                              # (deep, across) mm round each outline
                for (d0, a0), (d1, a1) in zip(p[:-1], p[1:]):
                    out += [((a0, z0, d0), (a1, z0, d1), (a1, z1, d1)), ((a0, z0, d0), (a1, z1, d1), (a0, z1, d0))]
            points = np.concatenate(loops)
            simplices = Delaunay(points).simplices
            middle = points[simplices].mean(axis=1) / h * fine + 2.0 * fine - 0.5
            inside = ndimage.map_coordinates(field, [middle[:, 1], middle[:, 0]], order=1) > 0.5
            for s in simplices[inside]:
                (d, a) = points[s].T
                out += [tuple((a[i], z, d[i]) for i in range(3)) for z in (z0, z1)]
        k = j
    return np.array(out, float).reshape(-1, 3, 3)


def faced(got, c):
    """(n, 3, 3) mm in the bed's frame: a shaped case's solid drawn smooth (`outlined` if extruded, else `surfaced`)."""
    return outlined(got['solid'], got['h']) if c.get('extruded') else kt.surfaced(got['solid'], c['kept'], got['h'])


def wrench(points, force, moment, about):
    """Forces (n, 3) on `points` (n, 3) that sum to `force` and turn `moment` about `about`: one push shared
    alike, and a turn about their middle."""
    import numpy as np
    points, force = np.asarray(points, float), np.asarray(force, float)
    middle = points.mean(axis=0)
    r = points - middle
    inertia = (r * r).sum() * np.eye(3) - r.T @ r
    left = np.asarray(moment, float) - np.cross(middle - np.asarray(about, float), force)
    spin = np.linalg.pinv(inertia) @ left
    return force / len(points) + np.cross(spin, r)


def _meshed(c):
    """(mesh, basis, cells, the fixed freedoms, the cases' load vectors) of a case's room."""
    import numpy as np
    room, h = c['room'], c['h']
    top = np.array(room.shape) - 1

    def keep(x, y, z):
        at = [np.minimum((v / h).astype(int), n) for v, n in zip((x, y, z), top)]
        return room[at[0], at[1], at[2]]
    mesh, basis, cells = kt.grid((0.0, 0.0, 0.0), tuple(h * n for n in room.shape), h, keep)
    dofs = basis.nodal_dofs
    assert dofs is not None
    fixed = dofs[:, np.unique(mesh.t[:, c['held'][tuple(cells.T)]])].ravel()
    loads = []
    for parts in c['loads']:
        f = np.zeros(basis.N)
        for patch, force, moment, about in parts:
            nodes = np.unique(mesh.t[:, patch[tuple(cells.T)]])
            f[dofs[:, nodes].T.ravel()] += wrench(mesh.p[:, nodes].T, force, moment, about).ravel()
        loads.append(f)
    return mesh, basis, cells, fixed, loads


def _moved(basis, k0, young, loads, free, starts):
    """[displacements] of the load cases by conjugate gradients, each from `starts`' (left there for the next)."""
    import numpy as np
    from scipy.sparse import coo_matrix, diags
    from scipy.sparse.linalg import cg
    ed = basis.element_dofs
    n = ed.shape[1]
    rows = np.broadcast_to(ed[:, None, :], (24, 24, n)).ravel()
    cols = np.broadcast_to(ed[None, :, :], (24, 24, n)).ravel()
    stiff = coo_matrix(((k0[:, :, None] * young[None, None, :]).ravel(), (rows, cols)),
                       shape=(basis.N, basis.N)).tocsr()[free][:, free]
    pre = diags(1.0 / stiff.diagonal())                  # Jacobi's: the diagonal's inverse
    out = []
    for k, f in enumerate(loads):
        starts[k] = cg(stiff, f[free], x0=starts[k], rtol=1e-4, maxiter=8000, M=pre)[0]
        u = np.zeros(basis.N)
        u[free] = starts[k]
        out.append(u)
    return out


def optimise(c, log=None):
    """(the printed densities a voxel, mesh, basis, cells, fixed, loads) of a case: the least summed compliance
    its share of the room gives, printed."""
    import numpy as np
    from machine.body import structural
    mesh, basis, cells, fixed, loads = _meshed(c)
    shape, e0 = c['room'].shape, 1e3 * structural.PAHT_CF[0]
    free = np.setdiff1d(np.arange(basis.N), fixed)
    kept = c['kept'][tuple(cells.T)]
    weights, sums = kt.neighbours(cells, kt.REACH)
    k0, ed = kt.unit(c['h']), basis.element_dofs

    def lain(v):                                         # the voxels' values as [layer, across, deep], and back
        g = np.zeros((shape[1], shape[0], shape[2]))
        g[cells[:, 1], cells[:, 0], cells[:, 2]] = v
        return g

    def taken(g):
        return g[cells[:, 1], cells[:, 0], cells[:, 2]]
    column = np.unique(cells[:, 0] * shape[2] + cells[:, 2], return_inverse=True)[1]
    design = (~kept).astype(float)
    count = np.maximum(np.bincount(column, weights=design), 1.0)

    def columned(soft):                                  # a prism up the bed: a column's own voxels at their mean
        return np.where(kept, 1.0, (np.bincount(column, weights=soft * design) / count)[column]), None

    def columned_back(_soft, _keep, d):                  # its adjoint: each one its share of its column's sum
        return (np.bincount(column, weights=d * design) / count)[column] * design

    def stood_up(soft):                                  # what is kept is solid: what stands on it is printed
        grid, keep = printed(lain(soft))
        return np.where(kept, 1.0, taken(grid)), keep

    def stood_back(soft, keep, d):
        return taken(_back(lain(soft), keep, lain(np.where(kept, 0.0, d))))
    physical, back = (columned, columned_back) if c.get('extruded') else (stood_up, stood_back)
    x, starts, phys = np.where(kept, 1.0, c['share']), [None] * len(loads), None
    for step in range(ROUNDS):
        soft = np.where(kept, 1.0, (weights @ x) / sums)
        phys, keep = physical(soft)
        young = e0 * (VOID + (1.0 - VOID) * phys ** PENALTY)
        sens, compliance = np.zeros(len(x)), 0.0
        for u in _moved(basis, k0, young, loads, free, starts):
            ce = np.einsum('ie,ij,je->e', u[ed], k0, u[ed])
            compliance += float((young * ce).sum())
            sens -= PENALTY * e0 * (1.0 - VOID) * phys ** (PENALTY - 1.0) * ce
        sens = weights @ (back(soft, keep, sens) / sums)
        dv = np.maximum(weights @ (back(soft, keep, np.where(kept, 0.0, 1.0)) / sums), 1e-12)
        lo, hi, new = 0.0, 1e12, x
        while (hi - lo) / (hi + lo + 1e-30) > 1e-4:
            mid = 0.5 * (lo + hi)
            new = np.where(kept, 1.0, np.clip(x * np.sqrt(np.maximum(-sens, 0.0) / (dv * mid)),
                                              np.maximum(x - MOVE, 0.0), np.minimum(x + MOVE, 1.0)))
            trial = physical(np.where(kept, 1.0, (weights @ new) / sums))[0]
            lo, hi = (mid, hi) if trial[~kept].mean() > c['share'] else (lo, mid)
        change = float(np.abs(new - x)[~kept].max())
        x = new
        if log:
            log('  round %2d: compliance %.4g, change %.3f' % (step, compliance, change))
        if change < 0.01:
            break
    assert phys is not None
    return phys, mesh, basis, cells, fixed, loads


def _digest(c):
    """A case's content as a name: its arrays, its loads, its voxel, its share and the method's own numbers."""
    import numpy as np
    h = hashlib.sha256(repr((c['h'], c['share'], PENALTY, MOVE, ROUNDS, VOID, SHARP, EPS, LEVELS, kt.REACH,
                             'either way', 'kept whole', 'kept solid, propped', c.get('extruded', False))).encode())
    for part in (c['room'], c['kept'], c['held']):
        h.update(np.packbits(part).tobytes() + repr(part.shape).encode())
    for patch, force, moment, about in (part for parts in c['loads'] for part in parts):
        h.update(np.packbits(patch).tobytes() + np.asarray((force, moment, about), float).round(6).tobytes())
    return h.hexdigest()[:16] + '-%d' % len(c['loads'])


def _path(c):
    from machine.body import mechatronics
    return os.path.join(os.path.dirname(mechatronics.CACHE), 'topo', '%s-%s.npz' % (c['name'], _digest(c)))


def ready(c):
    """Whether this very case was shaped before and is kept."""
    return os.path.isfile(_path(c))


def _margin(c, mesh, basis, fixed, loads, solid, kept):
    """How many times over a case's `solid` (bool a voxel) carries its loads, each either way: the principal stress
    within the printed polymer's allowable, the stress up the layers within ACROSS of it, read in what is not kept."""
    import numpy as np
    from machine.body import structural
    h, e0, allow = c['h'], 1e3 * structural.PAHT_CF[0], structural.PAHT_CF[2]
    free, k0, young = np.setdiff1d(np.arange(basis.N), fixed), kt.unit(h), np.where(solid, e0, e0 * VOID)
    moved = _moved(basis, k0, young, loads, free, [None] * len(loads))
    worst = kt.stresses(basis, mesh, k0, young, loads, fixed, h, e0, moved + [-u for u in moved])[solid & ~kept]
    return float(min(allow / worst[:, 0].max(), kt.ACROSS_SHARE * allow / max(worst[:, 3].max(), 1e-9)))


def whole(c):
    """{'margin', 'grams'}: a case's room all solid - the most the optimiser could keep -, read as `shaped` reads a
    cut: where it does not carry, no cut is asked to."""
    import numpy as np
    mesh, basis, cells, fixed, loads = _meshed(c)
    return {'margin': _margin(c, mesh, basis, fixed, loads, np.ones(len(cells), bool), c['kept'][tuple(cells.T)]),
            'grams': float(len(cells)) * c['h'] ** 3 * 1e-9 * kt.DENSITY * 1e3}


def shaped(c, log=None, fresh=False):
    """{'solid': [across, up, deep] the voxels kept, 'h', 'grams', 'room': the room's grams, 'margin', 'level',
    'lost': the kept voxels the cut let hang, 'points', 'cells': its voxels' corners for `knee_topo.faces`}: a
    case shaped (`optimise`) and cut at the first level that carries its loads with nothing told to stay lost, else
    at the last - from .cache/topo where this very case was shaped before, unless `fresh`."""
    import numpy as np
    path = _path(c)
    if os.path.isfile(path) and not fresh:
        with np.load(path) as got:
            return {k: (got[k] if got[k].ndim else got[k].item()) for k in got.files}
    phys, mesh, basis, cells, fixed, loads = optimise(c, log)
    shape, h, kept = c['room'].shape, c['h'], c['kept'][tuple(cells.T)]
    out = None
    room, keeps = (np.zeros((shape[1], shape[0], shape[2]), bool) for _ in range(2))
    room[cells[:, 1], cells[:, 0], cells[:, 2]] = True
    keeps[cells[:, 1], cells[:, 0], cells[:, 2]] = kept
    for level in LEVELS:
        grid = np.zeros((shape[1], shape[0], shape[2]), bool)
        grid[cells[:, 1], cells[:, 0], cells[:, 2]] = (phys >= level) | kept
        for _ in range(len(grid)):                       # what is kept, held up where it hangs, till none falls
            lost = keeps & ~stood(grid)
            if not lost.any():
                break
            grid = propped(stood(grid) | keeps, lost, room)
        solid = stood(grid)[cells[:, 1], cells[:, 0], cells[:, 2]]
        margin = _margin(c, mesh, basis, fixed, loads, solid, kept)
        grams = float(solid.sum()) * h ** 3 * 1e-9 * kt.DENSITY * 1e3
        if log:
            log('  cut at %.1f: %.0f g, margin x%.2f, %d kept voxels lost' % (level, grams, margin,
                                                                               int((kept & ~solid).sum())))
        out = {'solid': np.zeros(shape, bool), 'h': h, 'grams': grams, 'margin': float(margin), 'level': level,
               'room': float(len(cells)) * h ** 3 * 1e-9 * kt.DENSITY * 1e3, 'lost': int((kept & ~solid).sum()),
               'points': mesh.p, 'cells': mesh.t[:, solid]}
        out['solid'][tuple(cells[solid].T)] = True
        if margin >= 1.0 and not out['lost']:
            break
    assert out is not None
    os.makedirs(os.path.dirname(path), exist_ok=True)
    np.savez_compressed(path, **out)
    return out


def trial(h=4.0):
    """A bracket to try the method on: a room 48 x 32 x 24 mm on its bed, held by its one end's face, a push down
    its other end's top edge."""
    import numpy as np
    room = np.ones((int(48 / h), int(32 / h), int(24 / h)), bool)
    held, patch = np.zeros_like(room), np.zeros_like(room)
    held[0], patch[-1, -1] = True, True
    return case('trial', room, held | patch, held, [[(patch, (0.0, -400.0, 0.0), (0.0, 0.0, 0.0), (46.0, 30.0, 12.0))]],
                h, 0.4)


def main():
    import numpy as np
    got = shaped(trial(), log=print, fresh=True)
    hung = int((got['solid'].transpose(1, 0, 2) & ~stood(got['solid'].transpose(1, 0, 2))).sum())
    print('kept %.1f g of %.1f g at %.1f, margin x%.2f, %d voxels hung, %d kept voxels lost' % (
        got['grams'], got['room'], got['level'], got['margin'], hung, got['lost']))
    for j in range(got['solid'].shape[1] - 1, -1, -1):
        print('   ' + ''.join('#' if got['solid'][i, j].any() else '.' for i in range(got['solid'].shape[0])))
    return 0 if hung == 0 and np.isfinite(got['margin']) else 1


if __name__ == '__main__':
    sys.exit(main())
