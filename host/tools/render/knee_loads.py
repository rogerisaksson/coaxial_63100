"""The knee mount's loads and sizes: a hand calculation from the specs, checked by finite elements.

    python -X utf8 tools/render/knee_loads.py [--fem] [--h MM]

The loads are what the knees carried, measured, all at once, times the structural margin (`structural.MARGIN`): the
swing (no less than the knee's gearbox's shock torque, `drives.shock`), the lateral, the torsion and the axial
(MEASURED). The printed polymer is `materials.toml`'s paht_cf along its layers and ACROSS of that across them. Each
size comes out of its check: the collar round the tube's mouth where the spigot alone falls short, the flange's
thickness, the output arm's thickness, each joint's screws the least size that neither parts at the margin nor slips
at once (preload, friction). `--fem` holds the printed spigot part to linear elasticity on voxels (scikit-fem): the
tube bonded on, the loads on its far end, the flange's face held down as its preload keeps it and its sleeves held.
"""
import argparse
import json
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: Printed interlayer strength as a share of in-layer (CF-filled PC/PA datasheets: 40-60 %); the bond's line and
#: the least printed wall, mm.
ACROSS, BOND, WALL = 0.5, 0.2, 3.0
#: 12.9 socket screws: preload at 70 % of proof (970 MPa) on the stress area, N, and the head's radius and height,
#: mm; friction on steel sleeves; a toughened epoxy on a printed face: plastic shear (the face's, below the
#: adhesive's), shear modulus, MPa, and plastic strain.
SCREWS = {'M4': (0.7 * 970.0 * 8.78, 3.5, 4.0), 'M5': (0.7 * 970.0 * 14.2, 4.25, 5.0),
          'M6': (0.7 * 970.0 * 20.1, 5.0, 6.0)}
FRICTION, TAU_P, G_ADH, GAMMA_P = 0.15, 20.0, 1000.0, 0.3
#: The flange about a tube r round: its oval's half-widths r and these across and along the swing, its power, its
#: four screws r and these off the axis either way, mm; the spigot's insertion and the collar's length as shares of
#: the tube's width; the two flanges' gap where they pass at the joint's last bend, mm. The output hub's and the
#: stator base's screws: count, circle radius mm.
OVAL_PAST, POWER, BOLTS_PAST, SPIGOT_SHARE, COLLAR_SHARE, GAP = (3.0, 17.0), 4.0, (-6.0, 8.0), 1.25, 0.375, 3.0
HUB, BASE = (8, 20.0), (8, 22.0)
#: Each mounted joint: the segment that comes in, the one that goes out, the cables' bore, mm - the knee's the 48 V
#: pair, an RS485 pair and the foot's sensors with a 30 A two-pole bullet plug pulled through (11.4 mm across its
#: corners), the elbow's the hand's sensors alone (its wrist and gripper are passive) with a four-pole 1.25 mm plug
#: pulled through.
MOUNTED = {'knee': ('thigh', 'shank', 14.0), 'elbow': ('upper_arm', 'forearm', 7.0)}
#: What each mounted joint of the build bore through the scenarios it stood (tools/sim/legload.py --table, 49 of
#: 69): the most of its mean over 20 ms on end - a blow shorter counts by its impulse, not by the peak the solver's
#: contact shapes -, no joint counted in a scenario that wedged it. What a mount is not sized for is told apart
#: there: those peaks, a fall's loads, a wedged limb's.
#: The knee: about its axis 186 N m (play_walk_leap, past the gearbox's 128 through the stop), across it 55 and
#: about its length 20 (running_jump), along it 756 N (play_walk_leap, 2.67 weights). Apart: its peaks - 197 N m,
#: 89 across (8 ms), 1015 N (10 ms); cross_slope, whose 189 N m across was taken before, is a fall.
KNEE = {'swing': 186.0, 'lateral': 55.0, 'torsion': 20.0, 'axial': 756.0}
#: The elbow: about its axis 52 N m and across it 126 (turning_in_bed: the robot on its arm), about its length 25
#: (crawl_backwards), along it 493 N (mattress). Apart: bed_leg wedges it - 353 N m across kept a second, its arm
#: under it, where the robot's weight at its hand makes 108; chair's 192 across, taken before, lasted under a
#: millisecond; the stock robot's bed_leg lays 222 N m across its elbow.
ELBOW = {'swing': 52.0, 'lateral': 126.0, 'torsion': 25.0, 'axial': 493.0}
#: The shoulder: about its own axis 67 N m (mattress - its drive's own is more), across it 191 (slip: the robot
#: caught on its hand), about the arm's length 115 (turning_in_bed), along the arm 467 N (mattress). Apart:
#: bed_leg wedges it; hop_up's 250 across, taken before, was a peak.
SHOULDER = {'swing': 67.0, 'lateral': 191.0, 'torsion': 115.0, 'axial': 467.0}
#: The neck: its nod 14 N m, across it 17, its twist 4 and 1113 N down it; the head: its turn 3 N m, its nod 20,
#: across 13 and 1150 N down its axis - the bed's all (mattress, turning_in_bed: the robot laid on its head's
#: side), their peaks 2631 and 2718 N for 7 ms. The spine: 106 N m (standing_jump), across it 135 (slip), about
#: its length 41 and 1001 N down it (chair_low).
NECK = {'swing': 14.0, 'lateral': 17.0, 'torsion': 4.0, 'axial': 1113.0}
HEAD = {'swing': 3.0, 'lateral': 20.0, 'torsion': 13.0, 'axial': 1150.0}
SPINE = {'swing': 106.0, 'lateral': 135.0, 'torsion': 41.0, 'axial': 1001.0}
MEASURED = {'knee': KNEE, 'elbow': ELBOW, 'shoulder': SHOULDER, 'neck': NECK, 'head': HEAD, 'spine': SPINE}


def tube(joint='knee'):
    """The incoming tube at the joint: outer radius and wall, mm (`[skeleton] hung`, the stock table)."""
    from machine.body import mechatronics, skeleton
    radii = skeleton.HUNG[MOUNTED[joint][0]][1]
    r = 1e3 * float(radii[-1] if isinstance(radii, tuple) else radii)
    stock = {round(float(d), 1): float(w) for d, w in mechatronics.materials()['tubes']['stock']}
    return r, 1e3 * stock[round(2.0 * r, 1)]


def stated():
    """How far ahead of the knee's axis the thigh's tube passes as the build states its run (`[skeleton] hung`), m:
    the line's distance from the axis - what `knee_clamped.ahead` searches for, laid in the spec."""
    from machine.body import skeleton
    from machine.body.figure import THIGH
    a, b, _r = skeleton.runs('thigh')[0]
    (y1, z1), (y2, z2) = (a[1] + THIGH, a[2]), (b[1] + THIGH, b[2])
    return abs((z2 - z1) * y1 - (y2 - y1) * z1) / math.hypot(y2 - y1, z2 - z1)


def loads(joint='knee'):
    """The joint's loads at the margin: the swing (no less than its drive's peak), the lateral and the torsion,
    N mm; the axial, N."""
    from machine.body import drives, structural
    k, got = structural.MARGIN, MEASURED[joint]
    return {'swing': k * 1e3 * max(got['swing'], drives.peak(joint)), 'lateral': k * 1e3 * got['lateral'],
            'torsion': k * 1e3 * got['torsion'], 'axial': k * got['axial']}


def oval(joint='knee'):
    """The flange's oval's half-widths across and along the swing, mm."""
    r = tube(joint)[0]
    return r + OVAL_PAST[0], r + OVAL_PAST[1]


def bolts(joint='knee'):
    """The flange's four screws (x, z), mm."""
    r = tube(joint)[0]
    x, z = r + BOLTS_PAST[0], r + BOLTS_PAST[1]
    return ((x, z), (-x, z), (-x, -z), (x, -z))


def lengths(joint='knee'):
    """The spigot's insertion and the collar's length, mm."""
    d = 2.0 * tube(joint)[0]
    return SPIGOT_SHARE * d, COLLAR_SHARE * d


def apart(joint='knee'):
    """How far each tube's axis sits off the stack's middle, mm: the thigh's one way, the shank's the other, so the
    two flanges just pass at the knee's last bend and the leg's line runs through the joint."""
    return oval(joint)[0] + GAP / 2.0


def spigot(joint='knee'):
    """The spigot's radius, its bore's radius and the bore's offset along the knee's axis, mm: the bore as far
    out as the least wall lets it, on the swing's neutral axis."""
    r, wall = tube(joint)
    rs, rb = r - wall - BOND, MOUNTED[joint][2] / 2.0
    return rs, rb, rs - rb - WALL


def _discs(collar, joint='knee'):
    r, _wall = tube(joint)
    rs, rb, e = spigot(joint)
    discs = [(rs, 0.0, 1.0), (rb, -e, -1.0)]
    return discs + ([(r + BOND + collar, 0.0, 1.0), (r + BOND, 0.0, -1.0)] if collar else [])


def root(collar, ld, joint='knee'):
    """The highest principal stress, MPa, at the tube's mouth in the printed spigot (and collar), all loads at once."""
    discs = _discs(collar, joint)
    area = sum(s * math.pi * r * r for r, _x, s in discs)
    xc = sum(s * math.pi * r * r * x for r, x, s in discs) / area
    ix = sum(s * math.pi * r ** 4 / 4.0 for r, _x, s in discs)
    iz = sum(s * (math.pi * r ** 4 / 4.0 + math.pi * r * r * (x - xc) ** 2) for r, x, s in discs)
    c = max(r for r, _x, s in discs if s > 0)
    shear = ld['torsion'] * c / (ix + iz)

    def principal(t):
        normal = ld['swing'] * c * math.sin(t) / ix + ld['lateral'] * (c * math.cos(t) - xc) / iz + ld['axial'] / area
        return normal / 2.0 + math.hypot(normal / 2.0, shear)
    return max(principal(math.radians(d)) for d in range(360))


def _least(sizes_ok):
    """The smallest screw whose check holds, else the largest."""
    return next((s for s in SCREWS if sizes_ok(s)), list(SCREWS)[-1])


def sizes(collar=None, flange=None, joint='knee'):
    """{'collar', 'flange', 'arm': mm, 'screws': {joint: size}, 'margins': {name: margin}, 'loads': loads()}: each
    size its check's least, or as given."""
    from machine.body import structural
    ld, k = loads(joint), structural.MARGIN
    pattern, (spigot_l, collar_l) = bolts(joint), lengths(joint)
    e_print, allow = 1e3 * structural.PAHT_CF[0], structural.PAHT_CF[2]
    r, wall = tube(joint)
    if collar is None:
        collar = next((w for w in [0] + list(range(int(WALL), 13)) if root(w, ld, joint) <= allow), 12)
    rs, rb, _e = spigot(joint)
    i_sp = math.pi / 4.0 * (rs ** 4 - rb ** 4)
    i_col = math.pi / 4.0 * ((r + BOND + collar) ** 4 - (r + BOND) ** 4) if collar else 0.0
    moment, et_tube = math.hypot(ld['swing'], ld['lateral']), 1e3 * structural.CARBON[0] * wall
    bonds = []
    for i, radius, length in ((i_sp, rs, spigot_l), (i_col, r + BOND + collar / 2.0, collar_l)):
        if i:
            share = i / (i_sp + i_col)
            steady = share * (ld['torsion'] / (2.0 * math.pi * radius ** 2 * length)
                              + ld['axial'] / (2.0 * math.pi * radius * length))    # the bond's even shear, MPa
            tau, et = TAU_P - steady, e_print * i / (math.pi * radius ** 3)     # a ring's wall in bending
            capacity = math.sqrt(2.0 * tau * BOND * (tau / G_ADH / 2.0 + GAMMA_P) * 2.0 * et * (1.0 + et / et_tube))
            bonds.append(capacity / (moment * share / (math.pi * radius ** 2)))
    reach = math.hypot(*pattern[0])
    pull = max(ld['swing'] * abs(z) / sum(b[1] ** 2 for b in pattern) + ld['lateral'] * abs(x)
               / sum(b[0] ** 2 for b in pattern) for x, z in pattern) + ld['axial'] / len(pattern)
    slide = ld['torsion'] / (len(pattern) * reach)

    def held(preload, tension, shear):
        """(parting, slipping at once): preload against the tension at the margin, friction against the shear."""
        return preload / (0.8 * tension), FRICTION * (preload - 0.8 * tension / k) / (shear / k)
    flange_screw = _least(lambda s: min(held(SCREWS[s][0], pull, slide)) >= 1.0)
    lever = reach - (r + BOND + collar if collar else r)
    least = math.ceil(math.sqrt(6.0 * pull * lever / (2.0 * math.pi * reach / len(pattern) * ACROSS * allow)))
    flange = least if flange is None else flange
    depth = 2.0 * oval(joint)[1]                                    # the arm printed flat: its bendings in its layers

    def arm_stress(t):
        normal = 6.0 * ld['swing'] / (t * depth ** 2) + 6.0 * ld['lateral'] / (depth * t * t) + ld['axial'] / (depth * t)
        return normal / 2.0 + math.hypot(normal / 2.0, 3.0 * ld['torsion'] / (depth * t * t))
    arm = next(t for t in range(5, 80) if arm_stress(t) <= allow)
    across = math.hypot(ld['lateral'] + ld['axial'] * apart(joint), ld['torsion'])   # the tube's axial off the circle

    def ring(n, radius, s):
        return held(SCREWS[s][0], 2.0 * across / (n * radius), ld['swing'] / (n * radius) + ld['axial'] / n)
    screws = {'flange': flange_screw, 'hub': _least(lambda s: min(ring(*HUB, s)) >= 1.0),
              'base': _least(lambda s: min(ring(*BASE, s)) >= 1.0)}
    margins = {'spigot root': allow / root(collar, ld, joint), 'bond': min(bonds),
               'flange parting': held(SCREWS[flange_screw][0], pull, slide)[0],
               'flange slip at 1x': held(SCREWS[flange_screw][0], pull, slide)[1],
               'hub slip at 1x': ring(*HUB, screws['hub'])[1], 'base slip at 1x': ring(*BASE, screws['base'])[1],
               'heads clear mm': reach - SCREWS[flange_screw][1] - (r + BOND + collar)}
    return {'collar': collar, 'flange': flange, 'arm': arm, 'screws': screws, 'margins': margins, 'loads': ld,
            'joint': joint}


def fem(sz, h=2.0):
    """{'mouth', 'collar', 'flange': the highest principal stress, MPa} in the printed part on `h`-mm voxels."""
    import numpy as np
    from scipy.sparse import coo_matrix
    from skfem import Basis, BilinearForm, ElementHex1, ElementVector, MeshHex, condense, solve
    from skfem.helpers import ddot, div, sym_grad
    from machine.body import structural
    joint = sz.get('joint', 'knee')
    r, wall = tube(joint)
    rs, rb, e = spigot(joint)
    pattern, (spigot_l, collar_l), half = bolts(joint), lengths(joint), oval(joint)
    sleeve_r = SCREWS[sz['screws']['flange']][1]
    collar, deck, run = sz['collar'], float(sz['flange']), spigot_l + 10.0
    span = [np.arange(-q - h, q + h + 1e-9, h) for q in half]
    mesh = MeshHex.init_tensor(span[0], np.arange(0.0, deck + run + 1e-9, h), span[1])
    x, y, z = mesh.p[:, mesh.t].mean(axis=1)
    rr, bore = np.hypot(x, z), np.hypot(x + e, z) <= rb
    sleeve = np.min([np.hypot(x - bx, z - bz) for bx, bz in pattern], axis=0) <= sleeve_r
    inside = (np.abs(x) / half[0]) ** POWER + (np.abs(z) / half[1]) ** POWER <= 1.0
    up = y - deck
    printed = ((y < deck) & inside & ~bore & ~sleeve) | ((up >= 0) & (up < spigot_l) & (rr <= rs + BOND / 2) & ~bore)
    if collar:
        printed |= (up >= 0) & (up < collar_l) & (rr > r + BOND / 2) & (rr <= r + BOND + collar)
    carbon = (up >= h) & (rr > rs + BOND / 2) & (rr <= r + BOND / 2)      # its butt end left off the flange
    steel = (y < deck) & sleeve
    keep = printed | carbon | steel
    mesh = mesh.remove_elements(np.nonzero(~keep)[0])
    if hasattr(mesh, 'remove_unused_nodes'):
        mesh = mesh.remove_unused_nodes()
    kind = np.where(printed, 0, np.where(carbon, 1, 2))[keep]
    solids = (structural.PAHT_CF, structural.CARBON, structural.STEEL)
    young = np.array([1e3 * s[0] for s in solids])[kind]
    nu = 0.35
    lam, mu = young * nu / ((1 + nu) * (1 - 2 * nu)), young / (2 * (1 + nu))
    basis = Basis(mesh, ElementVector(ElementHex1()))
    one = Basis(MeshHex(mesh.p[:, mesh.t[:, 0]], np.arange(8)[:, None]), ElementVector(ElementHex1()))
    local = one.element_dofs[:, 0]

    @BilinearForm
    def shear(u, v, _w):
        return 2.0 * ddot(sym_grad(u), sym_grad(v))

    @BilinearForm
    def bulk(u, v, _w):
        return div(u) * div(v)
    k_mu, k_lam = (f.assemble(one).toarray()[np.ix_(local, local)] for f in (shear, bulk))   # every voxel alike
    ed = basis.element_dofs
    rows, cols = np.broadcast_to(ed[:, None, :], (24, 24, ed.shape[1])), np.broadcast_to(ed[None, :, :],
                                                                                        (24, 24, ed.shape[1]))
    data = k_mu[:, :, None] * mu[None, None, :] + k_lam[:, :, None] * lam[None, None, :]
    stiff = coo_matrix((data.ravel(), (rows.ravel(), cols.ravel())), shape=(basis.N, basis.N)).tocsr()
    p = mesh.p
    top = np.nonzero(np.isclose(p[1], p[1].max()))[0]
    ld, force, dofs = sz['loads'], np.zeros(basis.N), basis.nodal_dofs
    assert dofs is not None
    zt, xt = p[2, top], p[0, top]
    turn = ld['torsion'] / (zt ** 2 + xt ** 2).sum()
    force[dofs[1, top]] = (ld['swing'] * zt / (zt ** 2).sum() + ld['lateral'] * xt / (xt ** 2).sum()
                           + ld['axial'] / len(top))
    force[dofs[0, top]], force[dofs[2, top]] = -turn * zt, turn * xt
    face = np.isclose(p[1], 0.0)                 # preloaded past parting: the face stays down, the sleeves hold
    held = np.nonzero(face & (np.min([np.hypot(p[0] - bx, p[2] - bz) for bx, bz in pattern], axis=0) <= sleeve_r + 1e-9))[0]
    fixed = np.unique(np.r_[dofs[1, np.nonzero(face)[0]], dofs[:, held].ravel()])
    disp = solve(*condense(stiff, force, D=fixed))
    assert isinstance(disp, np.ndarray)
    corner = p[:, mesh.t] - p[:, mesh.t].mean(axis=1, keepdims=True)      # each voxel's strain at its middle
    grad = np.einsum('ian,jan->nij', disp[dofs][:, mesh.t], np.sign(corner)) / (4.0 * h)
    strain = 0.5 * (grad + np.swapaxes(grad, 1, 2))
    mean = 2.0 * mu[:, None, None] * strain + (lam * np.trace(strain, axis1=1, axis2=2))[:, None, None] * np.eye(3)
    peak = np.linalg.eigvalsh(mean)[..., -1]
    cx, cy, cz = mesh.p[:, mesh.t].mean(axis=1)
    lone = kind == 0
    near = np.min([np.hypot(cx - bx, cz - bz) for bx, bz in pattern], axis=0) <= sleeve_r + 1.5 * h
    ring = np.hypot(cx, cz)
    regions = {'mouth': lone & (cy >= deck) & (cy < deck + 8.0) & (ring <= rs + h), 'flange': lone & (cy < deck) & ~near}
    if collar:
        regions['collar'] = lone & (cy >= deck) & (ring > r)
    return {name: (float(peak[at].max()), {'x': float(mean[at, 0, 0].max()), 'z': float(mean[at, 2, 2].max())})
            for name, at in regions.items()}


def judged(out):
    """(layers, {region: margin}): the print's layers across x or z, whichever leaves the least margin highest -
    the highest principal stress against the in-layer allowable, the stress across the layers against ACROSS of it."""
    from machine.body import structural
    allow = structural.PAHT_CF[2]
    best = {o: {k: min(allow / p, ACROSS * allow / max(n[o], 1e-9)) for k, (p, n) in out.items()} for o in 'xz'}
    layers = max(best, key=lambda o: min(best[o].values()))
    return layers, best[layers]


def settled(h=1.5, rounds=8, joint='knee'):
    """sizes() raised a millimetre at a time where the finite elements find the hand calculation short."""
    sz = sizes(joint=joint)
    for _ in range(rounds):
        layers, margins = judged(fem(sz, h))
        worst = min(margins, key=lambda k: margins[k])
        if margins[worst] >= 1.0:
            break
        sz = sizes(collar=sz['collar'] + (worst != 'flange'), flange=sz['flange'] + (worst == 'flange'), joint=joint)
    sz['fem'] = {'layers': layers, 'margins': margins, 'h': h}
    return sz


def main(argv=None):
    from machine.body import structural
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--fem', action='store_true')
    parser.add_argument('--joint', default='knee', choices=sorted(MOUNTED))
    parser.add_argument('--h', type=float, default=1.5)
    parser.add_argument('--json', help='where to keep the settled sizes (knee_mount.py --sizes)')
    args = parser.parse_args(argv)
    sz = sizes(joint=args.joint)
    ld = sz['loads']
    print('loads at x%.1f: swing %.0f, lateral %.0f, torsion %.0f N m, axial %.0f N; tubes %.1f mm off the middle' % (
        structural.MARGIN, ld['swing'] / 1e3, ld['lateral'] / 1e3, ld['torsion'] / 1e3, ld['axial'], apart(args.joint)))
    print('sizes: collar %d mm, flange %d mm, arm %d mm; spigot r %.1f, bore r %.1f at %.1f mm off; screws %s' % (
        (sz['collar'], sz['flange'], sz['arm']) + spigot(args.joint) + (sz['screws'],)))
    print('margins: ' + ', '.join('%s x%.2f' % kv for kv in sz['margins'].items()))
    if args.fem:
        sz = settled(args.h, joint=args.joint)
        if args.json:
            with open(args.json, 'w', encoding='utf-8', newline='') as f:
                json.dump(sz, f, indent=1)
        print('settled by finite elements (%.1f mm voxels, layers across %s): collar %d mm, flange %d mm; %s' % (
            args.h, sz['fem']['layers'], sz['collar'], sz['flange'],
            ', '.join('%s x%.2f' % kv for kv in sz['fem']['margins'].items())))
    return 0


if __name__ == '__main__':
    sys.exit(main())
