"""The chest's yokes: carbon sheets stacked round the roll journals, in the room the pelvis leaves.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/trunk_yokes.py

The cross hands the chest's push to the pelvis through its roll journals: the rods' pulls summed, and the twist
about the cross's own third axis that their lean leaves - up to 350 N m leant 35 deg and bowed back 30, a couple
of 5 to 6 kN across the two rings. Leant and bowed back, the pelvis's plates leave a ring's arm behind the cross
a neck 13 to 17 mm wide: printed PAHT-CF carried the couple there at x0.23 as drawn, at x0.30 in the widest room
the plates leave. So each yoke is a stack of stock carbon sheets (`structural.CARBON`), bonded, all cut to one
outline (`outline`): the ring round the journal's flanged polymer bushing, its tangents at the roll's end - the
plates' inner faces 2 mm off -, 45 deg out from where the plates' tops fall away below the stack, to a foot as
wide as the cage, each sheet's top under the cage's round. `drawn` gives the two stacks; `rings` the cross's push on
each ring at `trunk_laid.cases` and under the spine's thrust and twist; `held` what the waists, the bond between
the sheets, the rings' walls and the bushings hold of the most, by hand; `laminated` each stack solved as a
laminate on voxels - the round of its waist, its bonds peeled and sheared. The cage the feet stand under is the
calf's, drawn for its boards, not yet for the trunk's loads.
"""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import clamped as cl  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import hip_frame as hf  # noqa: E402
from tools.render import knee_clamped as kc  # noqa: E402
from tools.render import trunk_laid as tl  # noqa: E402
from tools.render import trunk_rods as tr  # noqa: E402

#: A carbon sheet's thickness - stock -, m; the bond's shear between two - a structural epoxy's lap shear on its
#: makers' sheets, 25 to 30 MPa, taken low -, MPa.
SHEET_M, BOND_MPA = 0.004, 20.0
#: The ring's radius round the journal - the pelvis's plates' inner faces (`trunk_clamped.ARM_X`) 2 mm off -, the
#: journal's radius and the bore's, round its flanged polymer bushing's 1.5 mm wall, m.
RING_R, JOURNAL_R, BORE_R = 0.019, 0.010, 0.0115
#: The stacks: {name: (the way off the block - ahead, behind -, the near face off its middle, m, its sheets, their
#: flare line - a sheet's half width its height less this, m)}: where below the stack the pelvis's plates' tops
#: fall away leant and bowed, one outline to all its sheets - sheets cut each to its own room stepped at the waist,
#: where the stack is bent the most, and a voxel solve read the steps' edges at 276 MPa. Swept over the travel by
#: 1.25 deg, the room 2 mm off the plates was (1 mm voxels, half width / height, mm): behind, the sheet next the
#: block 8/21 7/23 9/25 11/27, the sixth 12/21 15/23 18/25, a seventh 6/25 5/27 4/29 where the notch rises to the
#: bar - so six; ahead, the fifth 12/17 13/19 15/21, the first 14/14 15/15 18/17: each line a mm in from the least.
STACKS = {'front yoke': (1.0, 0.019, 5, 0.007), 'rear yoke': (-1.0, 0.019, 6, 0.017)}
#: A foot's half width - the cage's -, the round on the outline's corners, m.
FOOT_M, ROUND_M = 0.016, 0.003
#: The laminate past its plane (`laminated`): its Poisson's ratio in the plane, its modulus through the stack and
#: its shear modulus across the bonds, MPa, its Poisson's ratio from the plane through the stack - a woven carbon
#: laminate's handbook values (8-10 GPa, 3-5 GPa, 0.25-0.35).
LAMINA = (0.1, 9e3, 4e3, 0.3)


def _tangent():
    """(cos, sin) of the roll's end - the build's travel: where the ring's tangent stands."""
    roll = math.radians(tl.travel()[0][1])
    return math.cos(roll), math.sin(roll)


def _narrowest(flare):
    """(height, half width), m: where a sheet's ring's tangent meets its flare line."""
    c, s = _tangent()
    y = (RING_R + flare * c) / (s + c)
    return y, y - flare


def _top(z0, z1):
    """A sheet's top over z from `z0` to `z1`, m: 1 mm under the cage's round where it comes lowest."""
    outside, (hub_y, hub_z) = kc.laid()['cage'][3] + 0.001, tl.HUB[1:]
    return min(hub_y - math.sqrt(outside * outside - (z - hub_z) ** 2) for z in (z0, z1))


def _sheets(name):
    """[(z from, z to, flare line)] of a stack's sheets, from the block out, m."""
    way, near, sheets, flare = STACKS[name]
    return [tuple(sorted((way * (near + k * SHEET_M), way * (near + (k + 1) * SHEET_M)))) + (flare,)
            for k in range(sheets)]


def outline(flare, top):
    """A sheet's outline (x, y), the chest's frame, m, round from its ring's bottom: the ring, its tangent at the
    roll's end, the flare 45 deg out to the foot's side, the top at `top`; its corners rounded."""
    import numpy as np
    c, s = _tangent()
    y, w = _narrowest(flare)
    assert top > y, 'a sheet\'s top over its narrowest'
    up = min(top, flare + FOOT_M)
    right = ([(RING_R * math.cos(a), RING_R * math.sin(a)) for a in np.linspace(-0.5 * math.pi, math.atan2(s, c), 13)]
             + [(w, y), (up - flare, up)] + ([(FOOT_M, top)] if up < top - 1e-9 else []))
    return cl.filleted(np.array(right + [(-x, y) for x, y in right[:0:-1]]), ROUND_M)


def drawn():
    """{'front yoke', 'rear yoke': [(tri, colour)]} in the chest's frame: each sheet a slab of carbon."""
    return {name: [t for z0, z1, flare in _sheets(name)
                   for t in hf._slab(outline(flare, _top(z0, z1)), z0, z1, em.CARBON, 2, False)] for name in STACKS}


def rings():
    """[{'front yoke', 'rear yoke': the force, N (3,), the cross puts on that ring}], the chest's frame: a case each of
    `trunk_laid.cases`, then the spine's thrust and twist - the rods' pulls summed shared by the rings' middles'
    lever, the twist about the cross's own third axis a couple across them."""
    import numpy as np
    from tools.render import knee_loads as kl
    mid = {name: way * (near + 0.5 * sheets * SHEET_M) for name, (way, near, sheets, _flare) in STACKS.items()}
    front, rear = mid['front yoke'], mid['rear yoke']
    pins, pushed = (np.array(tl.BALL), np.array(tl.BALL) * (-1.0, 1.0, 1.0)), []
    for (roll, pitch), moment in tl.cases():
        pulls = tl.pulled(roll, pitch, moment)
        assert pulls is not None, 'every pose of the travel is reached (trunk_laid.reached)'
        cross, chest = tl.turn(roll, pitch)
        way = cross @ np.array((0.0, 1.0, 0.0))
        pushed.append((chest.T @ sum(pulls), chest.T @ (sum(np.cross(pin, f) for pin, f in zip(pins, pulls)) @ way * way)))
    spine = kl.loads('spine')
    pushed.append((np.array((0.0, spine['axial'], 0.0)), np.array((0.0, -1e-3 * spine['torsion'], 0.0))))
    out = []
    for force, twist in pushed:
        couple = np.cross(twist, (0.0, 0.0, 1.0)) / (front - rear)
        out.append({'front yoke': force * -rear / (front - rear) + couple, 'rear yoke': force * front / (front - rear) - couple})
    return out


def held():
    """{check: the least margin over the cases}: each yoke's waist at its narrowest - pulled, bent in its sheets'
    plane and across the stack -, the bond between its sheets there, its ring's wall beside the bore, its journal's
    bushing."""
    import numpy as np
    from machine.body import structural
    loads, out = rings(), {}
    for name, (_way, _near, sheets, flare) in STACKS.items():
        (at, half), depth = _narrowest(flare), sheets * SHEET_M
        area = 2.0 * half * depth
        plane, across = area * 2.0 * half / 6.0, area * depth / 6.0
        worst = dict.fromkeys(('waist', 'bond', 'ring', 'bushing'), 0.0)
        for f in (got[name] for got in loads):
            (fx, fy, fz), whole = np.abs(f), float(np.linalg.norm(f))
            worst['waist'] = max(worst['waist'], fy / area + fx * at / plane + fz * at / across)
            worst['bond'] = max(worst['bond'], 1.5 * fz / area)
            worst['ring'] = max(worst['ring'], whole / (2.0 * (RING_R - BORE_R) * depth))
            worst['bushing'] = max(worst['bushing'], whole / (2.0 * JOURNAL_R * depth))
        allowed = {'waist': structural.CARBON[2], 'bond': BOND_MPA, 'ring': structural.CARBON[2],
                   'bushing': tr.BUSHING_MPA}
        out.update({'%s %s' % (name, k): round(allowed[k] / (1e-6 * v), 2) for k, v in worst.items()})
    return out


def _stiffness(h):
    """(the 6 x 6 stiffness of the laminate, MPa - Voigt, z through the stack -, the 24 x 24 of an h-mm voxel)."""
    import numpy as np
    from machine.body import structural
    from skfem import Basis, BilinearForm, ElementHex1, ElementVector, MeshHex
    from skfem.helpers import sym_grad
    (e, nu), (ez, gz, nuz) = (1e3 * structural.CARBON[0], LAMINA[0]), LAMINA[1:]
    soft = np.zeros((6, 6))
    soft[:2, :2] = np.array([[1.0, -nu], [-nu, 1.0]]) / e
    soft[:2, 2] = soft[2, :2] = -nuz / e
    soft[2, 2], soft[3, 3], soft[4, 4], soft[5, 5] = 1.0 / ez, 1.0 / gz, 1.0 / gz, 2.0 * (1.0 + nu) / e
    stiff = np.linalg.inv(soft)
    one = Basis(MeshHex.init_tensor(*(np.array([0.0, h]),) * 3), ElementVector(ElementHex1()))

    def strains(g):
        return [g[0, 0], g[1, 1], g[2, 2], 2.0 * g[1, 2], 2.0 * g[0, 2], 2.0 * g[0, 1]]

    @BilinearForm
    def form(u, v, _w):
        a, b = strains(sym_grad(u)), strains(sym_grad(v))
        return sum(stiff[i, j] * a[j] * b[i] for i in range(6) for j in range(6))
    local = one.element_dofs[:, 0]
    return stiff, form.assemble(one).toarray()[np.ix_(local, local)]


def laminated(h=1.0):
    """{name: (the margin in the sheets' plane, at the bonds peeled, at the bonds sheared)}: each stack as drawn - its
    bore open, its feet held 2 mm down from their tops - a laminate on `h`-mm voxels under `rings`' pushes on its
    bore."""
    import numpy as np
    from machine.body import structural
    from tools.render import topo
    stiff, k0 = _stiffness(h)
    loads, out, wide = rings(), {}, max(RING_R, FOOT_M)
    for name in STACKS:
        sheets = _sheets(name)
        z0, z1 = min(s[0] for s in sheets), max(s[1] for s in sheets)
        axes = (np.arange(-wide, wide, 1e-3 * h), np.arange(-RING_R, max(_top(a, b) for a, b, _f in sheets), 1e-3 * h),
                np.arange(z0, z1 - 1e-9, 1e-3 * h))
        x, y, z = (1e3 * (v + 0.5e-3 * h) for v in np.meshgrid(*axes, indexing='ij'))
        room, foot = np.zeros(x.shape, bool), np.zeros(x.shape, bool)
        for a, b, flare in sheets:
            band = (z >= 1e3 * a) & (z <= 1e3 * b)
            room |= band & _inside(1e3 * outline(flare, _top(a, b)), x, y)
            foot |= band & (y >= 1e3 * _top(a, b) - 2.0)
        room &= np.hypot(x, y) >= 1e3 * BORE_R
        bore, held = room & (np.hypot(x, y) <= 1e3 * BORE_R + 1.5), room & foot
        at = np.array((0.0, 0.0, 0.5e3 * (z0 + z1))) - 1e3 * np.array([v[0] for v in axes])
        case = topo.case('trunk ' + name, room, held | bore, held, [[(bore, got[name], np.zeros(3), at)] for got in loads],
                         h, 1.0)
        mesh, basis, cells, fixed, vectors = topo._meshed(case)
        free, dofs = np.setdiff1d(np.arange(basis.N), fixed), basis.nodal_dofs
        corner = np.sign(mesh.p[:, mesh.t] - mesh.p[:, mesh.t].mean(axis=1, keepdims=True))
        worst = np.zeros((len(cells), 3))
        for u in topo._moved(basis, k0, np.ones(len(cells)), vectors, free, [None] * len(vectors)):
            g = np.einsum('ian,jan->nij', u[dofs][:, mesh.t], corner) / (4.0 * h)
            g = 0.5 * (g + np.swapaxes(g, 1, 2))
            s = np.stack([g[:, 0, 0], g[:, 1, 1], g[:, 2, 2], 2 * g[:, 1, 2], 2 * g[:, 0, 2], 2 * g[:, 0, 1]], 1) @ stiff.T
            spread = np.hypot(0.5 * (s[:, 0] - s[:, 1]), s[:, 5])
            worst = np.maximum(worst, np.c_[np.abs(0.5 * (s[:, 0] + s[:, 1])) + spread, np.abs(s[:, 2]),
                                            np.hypot(s[:, 3], s[:, 4])])
        w = worst[~case['kept'][tuple(cells.T)]].max(axis=0)
        out[name] = (structural.CARBON[2] / w[0], BOND_MPA / w[1], BOND_MPA / w[2])
    return out


def _inside(poly, x, y):
    """Whether each point (x, y) lies inside the polygon `poly` (k, 2): crossings counted."""
    import numpy as np
    hit = np.zeros(x.shape, bool)
    for (a, b), (c, d) in zip(poly, np.roll(poly, 1, axis=0)):
        hit ^= ((b > y) != (d > y)) & (x < (c - a) * (y - b) / (d - b + 1e-30) + a)
    return hit


def main():
    import numpy as np
    from machine.body import structural
    density = structural.CF_KG_M3
    for name in STACKS:
        grams = 0.0
        for z0, z1, flare in _sheets(name):
            pts = outline(flare, _top(z0, z1))
            area = 0.5 * abs(float(np.dot(pts[:, 0], np.roll(pts[:, 1], 1)) - np.dot(pts[:, 1], np.roll(pts[:, 0], 1))))
            grams += 1e3 * area * (z1 - z0) * density
            y, w = _narrowest(flare)
            print('%s sheet z %+5.1f..%+5.1f mm: narrowest %4.1f mm across at %4.1f mm, top %4.1f mm' % (
                name, 1e3 * z0, 1e3 * z1, 2e3 * w, 1e3 * y, 1e3 * _top(z0, z1)))
        print('%s: %.0f g of carbon' % (name, grams))
    for check, margin in held().items():
        print('  %-22s x%.2f' % (check, margin))
    for name, (plane, peel, shear) in laminated().items():
        print('%s as a laminate: x%.2f in the sheets\' plane, the bonds x%.2f peeled, x%.2f sheared' % (
            name, plane, peel, shear))
    return 0


if __name__ == '__main__':
    sys.exit(main())
