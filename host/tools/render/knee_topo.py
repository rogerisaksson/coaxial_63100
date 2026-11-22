"""The knee's printed parts by topology optimisation: material laid only where the loads need it.

    python -X utf8 tools/render/knee_topo.py OUT_DIR [--part arm|block] [--h MM] [--volume SHARE]

SIMP on voxels (penalised density, a density filter, optimality-criteria steps; the classic method, written on the
same voxel stiffness as knee_loads.fem): within the part's room - what nothing else needs through the knee's travel -
the material is given where the loads' summed compliance falls most for the share of the room asked. The loads are
knee_loads': the swing (the gearbox's shock), the lateral (the hip roll's shock down the leg), the torsion (the hip
yaw's shock) and the axial (`legload.py`'s most, measured), each its own case, times the structural margin; the part
is held where it is screwed on (the output hub, or the stator's base). The density field is then cut at the lowest
level the printed polymer still carries (the highest principal stress, and across the layers ACROSS of it), and kept
as voxels: an STL to print and the triangles knee_mount.py draws.
"""
import argparse
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import knee_loads as kl  # noqa: E402

#: The penalty, the filter's reach in voxels, the optimality step's largest move, its iterations, the void's share
#: of the solid's modulus, Poisson's ratio.
PENALTY, REACH, MOVE, ROUNDS, VOID, NU = 3.0, 1.6, 0.2, 60, 1e-6, 0.35
#: The printed polymer across its layers as a share of along (knee_loads.ACROSS); its density, kg/m3 (CF-filled
#: PC and PA datasheets: 1.2-1.3 t/m3).
ACROSS_SHARE, DENSITY = kl.ACROSS, 1250.0


def unit(h):
    """The 24 x 24 stiffness of one h-mm voxel of unit modulus, in its local functions' order."""
    import numpy as np
    from skfem import Basis, BilinearForm, ElementHex1, ElementVector, MeshHex
    from skfem.helpers import ddot, div, sym_grad
    one = Basis(MeshHex.init_tensor(np.array([0.0, h]), np.array([0.0, h]), np.array([0.0, h])),
                ElementVector(ElementHex1()))
    local = one.element_dofs[:, 0]

    @BilinearForm
    def shear(u, v, _w):
        return 2.0 * ddot(sym_grad(u), sym_grad(v))

    @BilinearForm
    def bulk(u, v, _w):
        return div(u) * div(v)
    lam, mu = NU / ((1 + NU) * (1 - 2 * NU)), 1.0 / (2 * (1 + NU))
    return mu * shear.assemble(one).toarray()[np.ix_(local, local)] + lam * bulk.assemble(one).toarray()[
        np.ix_(local, local)]


def grid(lo, hi, h, keep):
    """(mesh, basis, cells (n, 3) ints) of the voxels whose middles `keep(x, y, z)` holds, between corners lo, hi mm."""
    import numpy as np
    from skfem import Basis, ElementHex1, ElementVector, MeshHex
    axes = [np.arange(a, b + 1e-9, h) for a, b in zip(lo, hi)]
    mesh = MeshHex.init_tensor(*axes)
    mid = mesh.p[:, mesh.t].mean(axis=1)
    kept = keep(*mid)
    mesh = mesh.remove_elements(np.nonzero(~kept)[0])
    if hasattr(mesh, 'remove_unused_nodes'):
        mesh = mesh.remove_unused_nodes()
    cells = np.floor((mesh.p[:, mesh.t].mean(axis=1).T - np.asarray(lo)) / h).astype(int)
    return mesh, Basis(mesh, ElementVector(ElementHex1())), cells


def neighbours(cells, reach):
    """The density filter: sparse weights between voxels within `reach` voxels, and their sums."""
    import numpy as np
    from scipy.sparse import coo_matrix
    index = -np.ones(cells.max(axis=0) + 1 + 2 * int(math.ceil(reach)), int)
    pad = int(math.ceil(reach))
    index[tuple((cells + pad).T)] = np.arange(len(cells))
    rows, cols, vals = [], [], []
    span = range(-pad, pad + 1)
    for di in span:
        for dj in span:
            for dk in span:
                w = reach - math.sqrt(di * di + dj * dj + dk * dk)
                if w <= 0:
                    continue
                other = index[tuple((cells + pad + (di, dj, dk)).T)]
                ok = other >= 0
                rows.append(np.nonzero(ok)[0])
                cols.append(other[ok])
                vals.append(np.full(int(ok.sum()), w))
    weights = coo_matrix((np.concatenate(vals), (np.concatenate(rows), np.concatenate(cols))),
                         shape=(len(cells), len(cells))).tocsr()
    return weights, np.asarray(weights.sum(axis=1)).ravel()


def solved(basis, k0, young, loads, fixed):
    """[displacements] for each load case, the stiffness assembled from each voxel's modulus `young`."""
    import numpy as np
    from scipy.sparse import coo_matrix
    from scipy.sparse.linalg import splu
    ed = basis.element_dofs
    n = ed.shape[1]
    rows = np.broadcast_to(ed[:, None, :], (24, 24, n)).ravel()
    cols = np.broadcast_to(ed[None, :, :], (24, 24, n)).ravel()
    stiff = coo_matrix(((k0[:, :, None] * young[None, None, :]).ravel(), (rows, cols)),
                       shape=(basis.N, basis.N)).tocsc()
    free = np.setdiff1d(np.arange(basis.N), fixed)
    lu = splu(stiff[free][:, free].tocsc())
    out = []
    for f in loads:
        u = np.zeros(basis.N)
        u[free] = lu.solve(f[free])
        out.append(u)
    return out


def optimise(basis, cells, design, k0, loads, fixed, volume, e0, log=None):
    """The voxels' filtered densities (1 where not `design`) for the least summed compliance at `volume`."""
    import numpy as np
    weights, sums = neighbours(cells, REACH)
    x = np.where(design, volume, 1.0)
    ed = basis.element_dofs
    for step in range(ROUNDS):
        phys = np.where(design, (weights @ x) / sums, 1.0)
        young = e0 * (VOID + (1.0 - VOID) * phys ** PENALTY)
        sens = np.zeros(len(x))
        compliance = 0.0
        for u in solved(basis, k0, young, loads, fixed):
            ue = u[ed]
            ce = np.einsum('ie,ij,je->e', ue, k0, ue)
            compliance += float((young * ce).sum())
            sens -= PENALTY * e0 * (1.0 - VOID) * phys ** (PENALTY - 1.0) * ce
        sens = weights @ (sens / sums)
        dv = weights @ (np.ones(len(x)) / sums)
        lo, hi, new = 0.0, 1e12, x
        while (hi - lo) / (hi + lo + 1e-30) > 1e-4:
            mid = 0.5 * (lo + hi)
            new = np.clip(x * np.sqrt(np.maximum(-sens, 0.0) / (dv * mid)), np.maximum(x - MOVE, 0.0),
                          np.minimum(x + MOVE, 1.0))
            new = np.where(design, new, 1.0)
            if ((weights @ new) / sums)[design].mean() > volume:
                lo = mid
            else:
                hi = mid
        change = float(np.abs(new - x)[design].max())
        x = new
        if log:
            log('  step %2d: compliance %.4g, change %.3f' % (step, compliance, change))
        if change < 0.01:
            break
    return np.where(design, (weights @ x) / sums, 1.0)


def stresses(basis, mesh, k0, young, loads, fixed, h, e0, moved=None):
    """[(principal, across x, across z, across y)] per voxel, MPa, the most over the load cases - their
    displacements `moved` where another solver gave them, else solved here."""
    import numpy as np
    lam, mu = e0 * NU / ((1 + NU) * (1 - 2 * NU)), e0 / (2 * (1 + NU))
    dofs = basis.nodal_dofs
    assert dofs is not None
    corner = mesh.p[:, mesh.t] - mesh.p[:, mesh.t].mean(axis=1, keepdims=True)
    worst = None
    for u in solved(basis, k0, young, loads, fixed) if moved is None else moved:
        grad = np.einsum('ian,jan->nij', u[dofs][:, mesh.t], np.sign(corner)) / (4.0 * h)
        strain = 0.5 * (grad + np.swapaxes(grad, 1, 2))
        s = 2.0 * mu * strain + (lam * np.trace(strain, axis1=1, axis2=2))[:, None, None] * np.eye(3)
        now = np.c_[np.linalg.eigvalsh(s)[:, -1], s[:, 0, 0], s[:, 2, 2], s[:, 1, 1]]
        worst = now if worst is None else np.maximum(worst, now)
    assert worst is not None, 'no load case'
    return worst


def faces(points, cells):
    """The voxels' outer faces as triangles (n, 3, 3), in the points' units: a face two voxels share is inside."""
    import numpy as np
    sides = ((0, 3, 5, 1), (2, 4, 7, 6), (0, 2, 6, 3), (1, 5, 7, 4), (0, 1, 4, 2), (3, 6, 7, 5))
    quads = np.concatenate([cells[list(f)].T for f in sides])
    _u, inverse, counts = np.unique(np.sort(quads, axis=1), axis=0, return_inverse=True, return_counts=True)
    outer = quads[counts[inverse.ravel()] == 1]
    return points.T[np.concatenate([outer[:, [0, 1, 2]], outer[:, [0, 2, 3]]])]


def surfaced(solid, kept, h, fine=2, sigma=1.15, rounds=6):
    """(n, 3, 3) mm: a closed 2-manifold surface net on a tricubic signed distance field of `solid`, relaxed
    tangentially and projected onto the zero level set, the bed flat at y = 0 and `kept` seats held."""
    import numpy as np
    from scipy import ndimage
    h = float(h)
    if not solid.any():
        return np.zeros((0, 3, 3))
    step = h / fine
    s_ext = np.pad(solid, ((2, 2), (0, 2), (2, 2)), constant_values=False)
    s_ext = np.concatenate([np.repeat(s_ext[:, :1, :], 2, axis=1), s_ext], axis=1)
    k_ext = np.pad(kept, ((2, 2), (0, 2), (2, 2)), constant_values=False)
    k_ext = np.concatenate([np.repeat(k_ext[:, :1, :], 2, axis=1), k_ext], axis=1)
    edt_in = np.asarray(ndimage.distance_transform_edt(s_ext), float) - 0.5
    edt_out = np.asarray(ndimage.distance_transform_edt(~s_ext), float) - 0.5
    d = np.where(k_ext, np.maximum(np.where(s_ext, edt_in, -edt_out), 0.45), np.where(s_ext, edt_in, -edt_out))
    phi = np.asarray(ndimage.zoom(ndimage.gaussian_filter(d, sigma), fine, order=3), float) * h
    phi = np.minimum(phi, (((np.arange(int(phi.shape[1])) + 0.5) / fine - 2.0) * h)[None, :, None])
    ins = phi > 0.0
    e0, e1, e2 = ins[:-1] != ins[1:], ins[:, :-1] != ins[:, 1:], ins[:, :, :-1] != ins[:, :, 1:]
    cell = np.zeros(np.array(phi.shape) - 1, bool)
    for a in (0, 1):
        for b in (0, 1):
            cell |= e0[:, a:cell.shape[1] + a, b:cell.shape[2] + b]
            cell |= e1[a:cell.shape[0] + a, :, b:cell.shape[2] + b]
            cell |= e2[a:cell.shape[0] + a, b:cell.shape[1] + b, :]
    cid, ijk = -np.ones(cell.shape, int), np.argwhere(cell)
    if not len(ijk):
        return np.zeros((0, 3, 3))
    cid[cell] = np.arange(len(ijk))
    p, parts = (ijk.astype(float) + 1.0) * step - 2.0 * h, []
    for axis, mask in ((0, e0[:, 1:-1, 1:-1]), (1, e1[1:-1, :, 1:-1]), (2, e2[1:-1, 1:-1, :])):
        ii, jj, kk = np.nonzero(mask)
        if axis == 0:
            jj, kk = jj + 1, kk + 1
            q = np.c_[cid[ii, jj - 1, kk - 1], cid[ii, jj, kk - 1], cid[ii, jj, kk], cid[ii, jj - 1, kk]]
        elif axis == 1:
            ii, kk = ii + 1, kk + 1
            q = np.c_[cid[ii - 1, jj, kk - 1], cid[ii - 1, jj, kk], cid[ii, jj, kk], cid[ii, jj, kk - 1]]
        else:
            ii, jj = ii + 1, jj + 1
            q = np.c_[cid[ii - 1, jj - 1, kk], cid[ii, jj - 1, kk], cid[ii, jj, kk], cid[ii - 1, jj, kk]]
        flip = ins[ii, jj, kk][:, None]
        parts += [np.where(flip, q[:, [0, 1, 2]], q[:, [0, 2, 1]]), np.where(flip, q[:, [0, 2, 3]], q[:, [0, 3, 2]])]
    tris = np.vstack(parts)
    edges = np.vstack([tris[:, [0, 1]], tris[:, [1, 2]], tris[:, [2, 0]],
                       tris[:, [1, 0]], tris[:, [2, 1]], tris[:, [0, 2]]])
    deg = np.maximum(np.bincount(edges[:, 0], minlength=len(p)).astype(float)[:, None], 1.0)
    for _ in range(rounds):
        bed = p[:, 1] <= 0.15 * step
        g = (p + 2.0 * h) / step - 0.5
        grad = np.column_stack([ndimage.map_coordinates(phi, (g + 0.5 * np.eye(3)[k]).T, order=1, mode='nearest')
                                - ndimage.map_coordinates(phi, (g - 0.5 * np.eye(3)[k]).T, order=1, mode='nearest')
                                for k in range(3)]) / step
        n2 = np.sum(grad * grad, axis=1, keepdims=True) + 1e-8
        acc = np.zeros_like(p)
        np.add.at(acc, edges[:, 0], p[edges[:, 1]])
        lap = acc / deg - p
        tang = lap - (np.sum(lap * grad, axis=1, keepdims=True) / n2) * grad
        p = p + np.where(bed[:, None], 0.35 * np.c_[lap[:, 0], np.zeros(len(p)), lap[:, 2]], 0.45 * tang)
        g = (p + 2.0 * h) / step - 0.5
        val = ndimage.map_coordinates(phi, g.T, order=1, mode='nearest')
        dp = np.clip(val[:, None] / n2, -0.5 * step, 0.5 * step) * grad
        p = p - np.where(bed[:, None], np.c_[dp[:, 0], np.zeros(len(p)), dp[:, 2]], dp)
        p[:, 1] = np.where(bed, 0.0, np.maximum(p[:, 1], 0.0))
    return p[tris]


def stl(path, mesh, solid):
    """The solid voxels' outer faces as a binary STL, mm."""
    import numpy as np
    p = faces(mesh.p, mesh.t[:, solid])
    normal = np.cross(p[:, 1] - p[:, 0], p[:, 2] - p[:, 0])
    normal /= np.linalg.norm(normal, axis=1, keepdims=True) + 1e-30
    rec = np.zeros(len(p), dtype=[('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')])
    rec['n'], rec['v'] = normal, p
    with open(path, 'wb') as f:
        f.write(b'knee_topo'.ljust(80, b' '))
        f.write(np.uint32(len(p)).tobytes())
        f.write(rec.tobytes())
    return p


def arm_case(sz, h):
    """(lo, hi, keep, design, loads, held) of the output arm: its room in the knee's frame (mm, the gearbox toward
    +x), the voxels it may fill and those it must, its load cases and where the hub holds it."""
    import numpy as np
    from tools.render import knee_mount as km
    can, ring, _m0, g0, g1, _thigh, _shank = (v * 1e3 for v in km._stack())
    off = kl.apart()
    rb = ring + 5.0 + 1.0 + km.CASSETTE[0] * 1e3                     # the cassette's turning ring
    inner = rb + km.CASSETTE[1] * 1e3
    face = g1 + 3.0                                                  # the hub's face
    boss = -(inner + 12.0)                                           # the shank's boss's face
    width = sz['arm'] * 2.0                                          # room for the optimiser past the hand's arm
    start = face - h * math.ceil((face - (off - kl.oval()[0] - 1.0)) / h)   # the hub's face on a voxel plane
    lo, hi = (start, boss, -kl.oval()[1] - 8.0), (face + width, inner + 8.0, kl.oval()[1] + 8.0)

    def keep(x, y, z):
        r = np.hypot(y, z)
        past = x >= face                                             # beyond the hub's face: the whole disc
        under = (x >= max(g0 + 2.5, off - kl.oval()[0])) & (r >= inner)  # round the cassette, short of the block
        oval = (np.abs(x - off) / kl.oval()[0]) ** kl.POWER + (np.abs(z) / kl.oval()[1]) ** kl.POWER <= 1.0
        return ((past | under) & (y >= boss) & ((y > -inner) | past)) | (oval & (y >= boss) & (y <= -inner))

    def solid(x, y, z):
        oval = (np.abs(x - off) / kl.oval()[0]) ** kl.POWER + (np.abs(z) / kl.oval()[1]) ** kl.POWER <= 1.0
        boss_part = oval & (y <= -inner) & (y >= boss)
        hub_part = (x >= face) & (x < face + 2.0 * h) & (np.hypot(y, z) <= kl.HUB[1] + 6.0)
        return boss_part | hub_part
    return lo, hi, keep, solid, face, boss, off


def main(argv=None):
    import numpy as np
    from machine.body import structural
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    parser.add_argument('--h', type=float, default=4.0, help='the voxel, mm')
    parser.add_argument('--volume', type=float, default=0.3, help='the share of the room the material may fill')
    parser.add_argument('--sizes', help='the sizes knee_loads.py --fem --json kept')
    args = parser.parse_args(argv)
    if args.sizes:
        import json
        with open(args.sizes, encoding='utf-8') as f:
            sz = json.load(f)
    else:
        sz = kl.sizes()
    h, ld = args.h, sz['loads']
    lo, hi, keep, solid_at, face, boss, off = arm_case(sz, h)
    mesh, basis, cells = grid(lo, hi, h, keep)
    mid = mesh.p[:, mesh.t].mean(axis=1)
    design = ~solid_at(*mid)
    dofs = basis.nodal_dofs
    assert dofs is not None
    p = mesh.p
    held = np.nonzero(np.isclose(p[0], face, atol=1e-6) & (np.hypot(p[1], p[2]) <= kl.HUB[1] + 6.0))[0]
    fixed = dofs[:, held].ravel()
    foot = (np.abs(p[0] - off) / kl.oval()[0]) ** kl.POWER + (np.abs(p[2]) / kl.oval()[1]) ** kl.POWER <= 1.0
    ring = np.nonzero(np.isclose(p[1], boss, atol=1e-6) & foot)[0]          # the flange's seat on the boss
    sx, sz_ = p[0, ring] - off, p[2, ring]
    cases = []
    for kind in ('swing', 'lateral', 'torsion', 'axial'):
        f = np.zeros(basis.N)
        if kind == 'swing':
            f[dofs[1, ring]] = ld['swing'] * sz_ / (sz_ ** 2).sum()
        elif kind == 'lateral':
            f[dofs[1, ring]] = ld['lateral'] * sx / (sx ** 2).sum()
        elif kind == 'torsion':
            turn = ld['torsion'] / (sx ** 2 + sz_ ** 2).sum()
            f[dofs[0, ring]], f[dofs[2, ring]] = -turn * sz_, turn * sx
        else:
            f[dofs[1, ring]] = -ld['axial'] / len(ring)
        cases.append(f)
    e0, allow = 1e3 * structural.PAHT_CF[0], structural.PAHT_CF[2]
    k0 = unit(h)
    print('arm: %d voxels of %.0f mm (%d to shape), %d held, %d loaded' % (len(cells), h, int(design.sum()),
                                                                          len(held), len(ring)))
    if not len(held) or not len(ring):
        print("the arm is neither held nor loaded on its voxels' planes")
        return 1
    density = optimise(basis, cells, design, k0, cases, fixed, args.volume, e0, log=print)
    kept = None
    for level in (0.5, 0.4, 0.3, 0.2, 0.1):
        solid = density >= level
        young = np.where(solid, e0, e0 * VOID)
        worst = stresses(basis, mesh, k0, young, cases, fixed, h, e0)[solid]
        margin = {o: min(allow / worst[:, 0].max(), ACROSS_SHARE * allow / max(worst[:, i].max(), 1e-9))
                  for o, i in (('x', 1), ('z', 2))}
        best = max(margin, key=lambda o: margin[o])
        mass = float(solid.sum()) * h ** 3 * 1e-9 * DENSITY
        print('  cut at %.1f: %d voxels, %.0f g, margin x%.2f (layers across %s)' % (level, int(solid.sum()),
                                                                                    mass * 1e3, margin[best], best))
        if margin[best] >= 1.0:
            kept = (level, solid, margin[best], best, mass)
            break
    if kept is None:
        print('no cut carries the loads: more volume asked for')
        return 1
    level, solid, margin, layers, mass = kept
    path = os.path.join(args.out, 'knee_arm.stl')
    stl(path, mesh, solid)
    np.savez_compressed(os.path.join(args.out, 'knee_arm.npz'), points=mesh.p, cells=mesh.t[:, solid], h=h,
                        density=density, level=level, margin=margin, layers=layers, mass=mass)
    print('kept at %.1f: %.0f g, margin x%.2f, layers across %s -> %s' % (level, mass * 1e3, margin, layers, path))
    return 0


if __name__ == '__main__':
    sys.exit(main())
