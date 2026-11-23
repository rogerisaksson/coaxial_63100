"""The shoulder's abduction: pectoral drive, clamped crank and carbon pushrod to a clevis.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/shoulder_rolled.py OUT.png

Each shoulder's abduction drive (MN6007 + B with coaxial_63100) sits in a bilateral chest seat (`PEC_AT`, facing +z),
held to spine and crossbar by the support-free PAHT-CF mount (`shoulder_shaped`). An outboard slit-band crank
(`sized.band`) clamped on the output ring drives a carbon pushrod on cardan crosses outside the pectoral stack up to
Stage 1's horn (`X_ABD`), swinging the arm across `TRAVEL` (-30 to +95 deg past T-pose) without crossing itself or
anything stationary. Frame: `shoulder_laid`'s.
"""
import argparse
import functools
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import clamped as cl  # noqa: E402
from tools.render import covered as cv  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import hip_frame as hf  # noqa: E402
from tools.render import knee_loads as kl  # noqa: E402
from tools.render import knee_mount as km  # noqa: E402
from tools.render import leaves  # noqa: E402
from tools.render import mounting as m  # noqa: E402
from tools.render import shoulder_laid as sl  # noqa: E402
from tools.render import sized  # noqa: E402
from tools.render import tees  # noqa: E402

#: Abduction drive kind (`MN6007 + B`) and travel off hanging rest, deg.
KIND, TRAVEL = 'shoulder', (-30.0, 95.0)
#: Stage-1 abduction x in `shoulder_laid`'s frame (inboard of arm saddle).
X_ABD = 0.049
#: Pectoral drive axis `(x, y, z_back)` in `torso` m.
PEC_AT = (0.0575, 0.218, 0.066)
#: Four-bar geometry, m: crank, horn, pushrod and pin radii/z in `torso`.
CRANK_R, Z_CRANK, HORN_R, Z_HORN, ROD_R, PIN_R = 0.056, 0.142, 0.033, 0.104, 0.006, 0.008
#: Stage-1 clevis & gimbal dimensions, m (`shoulder_laid` frame).
AIR_M, CHEEK_T, FORK_T, HUB_R, WIDE_M, CLEVIS_PIN_R, BUSH_M = 0.0036, 0.0045, 0.005, 0.016, 0.030, 0.005, 0.001
STOP_M, LAP_M, REACH_M = 0.010, 0.010, 0.041
POLYMER = (226, 222, 206)


def _torso_frame():
    """(turn (3, 3) sketch->torso, arm (3,) pivot, p_abd (3,) Stage-1 pivot in torso)."""
    import numpy as np
    from machine.body import figure
    seg = next(s for s in figure.SEGMENTS if s[0] == 'left_upper_arm')
    arm = np.array([float(v) for v in seg[3]])
    c, s = math.cos(math.radians(float(seg[4]))), math.sin(math.radians(float(seg[4])))
    turn = np.array([[-c, -s, 0.0], [-s, c, 0.0], [0.0, 0.0, -1.0]])
    return turn, arm, np.array([X_ABD, 0.0, 0.0]) @ turn + arm


def to_sketch(tris):
    """`tris` in `torso` mapped into `shoulder_laid`'s frame."""
    turn, arm, _p = _torso_frame()
    return [((tri - arm) @ turn.T, col) for tri, col in tris]


def to_torso(tris, right=False):
    """`tris` in `shoulder_laid`'s mapped into `torso` (mirrored across x when `right`)."""
    turn, arm, _p = _torso_frame()
    out = [(tri @ turn + arm, col) for tri, col in tris]
    return [((t * (-1.0, 1.0, 1.0))[::-1], col) for t, col in out] if right else out


def stack():
    """{'board', 'motor', 'gear': (radius, length)} of the pectoral drive, m."""
    return sl.stack()


def gap():
    """Inner z-offset of Stage 1's cheeks off the pitch axis, m."""
    lay = sl.laid()
    return lay['bore'] + sl.DRUM_M + sl.RIB_UP + AIR_M


@functools.lru_cache(maxsize=None)
def linkage():
    """Solved 3D four-bar parameters between the pectoral crank and Stage 1's front horn (`torso`, m)."""
    import numpy as np
    _turn, _arm, p_abd = _torso_frame()
    cx0, cy0, _z0 = PEC_AT
    px, py = float(p_abd[0]), float(p_abd[1])
    phi = math.atan2(py - cy0, px - cx0)
    rolls = np.radians(np.linspace(TRAVEL[0], TRAVEL[1], 231))
    dz = Z_HORN - Z_CRANK
    best = None
    for off_deg in np.linspace(-18.0, 18.0, 37):
        beta0 = phi - 0.5 * math.pi - math.radians(0.5 * sum(TRAVEL)) + math.radians(off_deg)
        hx, hy = px + HORN_R * np.cos(beta0 + rolls), py + HORN_R * np.sin(beta0 + rolls)
        for length in np.linspace(0.105, 0.175, 141):
            l_xy2 = length * length - dz * dz
            if l_xy2 <= 1e-4:
                continue
            ux, uy = hx - cx0, hy - cy0
            d = np.hypot(ux, uy)
            ca = (d * d + CRANK_R * CRANK_R - l_xy2) / (2.0 * d * CRANK_R)
            if float(np.abs(ca).max()) > 0.96:
                continue
            th = np.arctan2(uy, ux) - np.arccos(ca)
            if not (np.diff(th) > 1e-4).all():
                continue
            cx, cy = cx0 + CRANK_R * np.cos(th), cy0 + CRANK_R * np.sin(th)
            u_rod = np.c_[hx - cx, hy - cy, np.full_like(cx, dz)] / length
            ec = float(np.abs((u_rod * np.c_[-np.sin(th), np.cos(th), np.zeros_like(th)]).sum(axis=1)).min())
            eh = float(np.abs((u_rod * np.c_[-np.sin(beta0 + rolls), np.cos(beta0 + rolls),
                                             np.zeros_like(rolls)]).sum(axis=1)).min())
            sc = min(ec, eh)
            if best is None or sc > best[0]:
                best = (sc, float(length), float(beta0), ec, eh, float(th[0]), float(th[-1]))
    assert best is not None, 'no four-bar geometry reaches TRAVEL'
    sc, length, beta0, ec, eh, th0, th1 = best
    return {'length': length, 'beta0': beta0, 'dz': dz, 'eff': sc, 'eff_crank': ec, 'eff_horn': eh,
            'crank_span': (math.degrees(th0), math.degrees(th1))}


def crank_at(roll=0.0):
    """(crank angle rad, crank pin (3,) in torso m, horn pin (3,) in torso m, unit rod vector (3,)) at `roll` deg."""
    import numpy as np
    lk = linkage()
    _turn, _arm, p_abd = _torso_frame()
    cx0, cy0, _z0 = PEC_AT
    beta = lk['beta0'] + math.radians(roll)
    horn = np.array([float(p_abd[0]) + HORN_R * math.cos(beta), float(p_abd[1]) + HORN_R * math.sin(beta), Z_HORN])
    ux, uy = horn[0] - cx0, horn[1] - cy0
    d = math.hypot(ux, uy)
    l_xy2 = lk['length'] ** 2 - lk['dz'] ** 2
    ca = max(-1.0, min(1.0, (d * d + CRANK_R * CRANK_R - l_xy2) / (2.0 * d * CRANK_R)))
    th = math.atan2(uy, ux) - math.acos(ca)
    crank = np.array([cx0 + CRANK_R * math.cos(th), cy0 + CRANK_R * math.sin(th), Z_CRANK])
    rod = horn - crank
    return th, crank, horn, rod / float(np.linalg.norm(rod))


@functools.lru_cache(maxsize=None)
def pectoral():
    """Axial layout of the pectoral abduction stack in `torso` m along +z at `PEC_AT[:2]`."""
    st = stack()
    cx, cy, z0 = PEC_AT
    b0, b1 = z0, z0 + st['board'][1]
    m0, m1 = b1 + cv.WALL_M + 0.001, b1 + cv.WALL_M + 0.001 + st['motor'][1]
    h0, h1 = m1, m1 + em.OUTPUT_SHARE * st['gear'][1]
    r0, r1 = h1, m1 + st['gear'][1]
    own = carried()[1]
    band_w = 1e-3 * sized.band(own, 1e3 * st['motor'][0], 1e3 * (h1 - m0 - 0.003))
    crank_z0, crank_z1 = r0 + 0.0015, r1 - 0.0005
    crank_w = 1e-3 * sized.band(own, 1e3 * st['gear'][0], 1e3 * (crank_z1 - crank_z0))
    face = b0 - em.ROTOR_AIR - cv.WALL_M
    guard = cv.span(face, m0, st['board'][0], st['motor'][0] + max(band_w, 0.004))
    return {'xy': (cx, cy), 'board': (b0, b1), 'motor': (m0, m1), 'house': (h0, h1), 'ring': (r0, r1),
            'band': (m0 + 0.0015, h1 - 0.001, st['motor'][0] + max(band_w, 0.005)),
            'crank': (crank_z0, crank_z1, st['gear'][0] + max(crank_w, 0.0045)), 'guard': guard}


@functools.lru_cache(maxsize=None)
def carried():
    """(Shoulder loads at margin, drive peak torque at margin N mm, clevis cheek push N, max pushrod force N)."""
    from machine.body import drives, structural
    ld = kl.loads('shoulder')
    own = structural.MARGIN * 1e3 * drives.peak('left_' + KIND)
    span_z = 2e3 * (gap() + 0.5 * CHEEK_T)
    twist, swing = ld['torsion'] / span_z, ld['swing'] / span_z
    push = math.hypot(twist, swing + 0.5 * ld['axial'])
    rod_f = own / (1e3 * CRANK_R * linkage()['eff_crank'])
    return ld, own, push, rod_f


@functools.lru_cache(maxsize=None)
def laid():
    """Stage-1 clevis & gimbal layout in `shoulder_laid`'s sketch frame (`X_ABD` pivot), m."""
    g, lay_s = gap(), sl.laid()
    cheek = (g, g + CHEEK_T)
    fork = (cheek[1] + 0.0036, cheek[1] + 0.0036 + FORK_T)
    bridge_x = (sl.FOOT_X + lay_s['coil'][1] + sl.MOUTH_M + 0.008,
                sl.FOOT_X + lay_s['coil'][1] + sl.MOUTH_M + 0.014)
    root_x = X_ABD + 0.0555
    _turn, arm, _p = _torso_frame()
    horn_z = float(arm[2]) - Z_HORN
    return {'x_abd': X_ABD, 'gap': g, 'cheek': cheek, 'fork': fork, 'pin': CLEVIS_PIN_R,
            'bridge': bridge_x, 'root': root_x, 'horn_z': horn_z, 'top': lay_s['top'], 'down': lay_s['down']}


def held():
    """{name: margin} of the pectoral crank, carbon pushrod, gimbal cheeks, crossbar fork, pins, stops and hose."""
    from machine.body import mechatronics, structural
    ld, own, push, rod_f = carried()
    pec, lay, lk = pectoral(), laid(), linkage()
    carbon, steel, allow = structural.CARBON[2], structural.STEEL[2], structural.PAHT_CF[2]
    rod_mm, l_mm = 1e3 * ROD_R, 1e3 * lk['length']
    euler = math.pi ** 2 * 1e3 * structural.ROD[0] * (math.pi * rod_mm ** 4 / 4.0) / (l_mm * l_mm)
    rod_stress = rod_f / (math.pi * rod_mm * rod_mm)
    pin_lever = 1e3 * max(abs(Z_CRANK - 0.5 * (pec['crank'][0] + pec['crank'][1])), 0.012)
    pin_bend = 32.0 * rod_f * pin_lever / (math.pi * (2e3 * PIN_R) ** 3)
    c_pin = 1e3 * lay['pin']
    c_shear = (push + 0.5 * rod_f) / (math.pi * c_pin * c_pin)
    bush_p = (push + 0.5 * rod_f) / (2.0 * c_pin * 1e3 * FORK_T)
    wide = 1e3 * WIDE_M
    cheek_s = 6.0 * (0.5 * ld['torsion'] * 0.35 + 0.5 * own) / (1e3 * CHEEK_T * wide * wide)
    fork_s = 6.0 * (0.5 * ld['swing'] + 0.5 * ld['torsion'] * 0.4) / (1e3 * FORK_T * (wide + 4.0) ** 2)
    horn_s = 6.0 * own / (5.0 * 26.0 * 26.0)
    hoop = sized._hoop(own, 1e3 * stack()['gear'][0])
    pinch = next((p for p, _h in sized.PINCHES if sized.PINCH * p >= hoop), sized.PINCHES[-1][0])
    stop_f = own / (1e3 * (HUB_R + 0.5 * STOP_M))
    return {'crank band': sized.PINCH * pinch / hoop, 'crank pin': structural.ROD[2] / pin_bend,
            'rod buckle': euler / rod_f, 'rod carbon': structural.ROD[2] / rod_stress,
            'horn': carbon / horn_s, 'cheeks': carbon / cheek_s, 'fork': carbon / fork_s,
            'clevis pins': (0.58 * steel) / c_shear, 'bushings': 80.0 / bush_p,
            'stop': allow * 1e6 * STOP_M * LAP_M / stop_f,
            'span bend': span_least() / float(mechatronics.materials()['hoses']['bus'][4])}


def _pectoral_drive():
    """(`hc._drive`, rot_z, cx, cy, can, ring) of the left pectoral stack."""
    import numpy as np
    from tools.render import hip_clamped as hc
    pec, st = pectoral(), stack()
    cx, cy = pec['xy']
    can, ring, r_b = st['motor'][0], st['gear'][0], st['board'][0]
    drv = hc._drive({'board': pec['board'] + (r_b,), 'motor': pec['motor'] + (can,),
                     'gear': (pec['house'][0], pec['ring'][1], ring)},
                    pec['band'][2] - can, 0.5 * math.pi, leave=0.5 * math.pi)
    rot_z = np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])
    return drv, rot_z, cx, cy, can, ring


def _pectoral_still():
    """[(tri, col)] of the stationary left pectoral drive, guard, lid and mount."""
    from tools.render import shoulder_shaped as ss
    drv, rot_z, cx, cy, _can, _ring = _pectoral_drive()
    tris = [((t @ rot_z) + (cx, cy, 0.0), c) for k in ('still', 'leads', 'guard', 'lid') for t, c in drv[k]]
    mount = ss.drawn() or ss.sketched()
    return tris + mount


def _crank_parts(roll=0.0):
    """{'ring', 'band', 'arm', 'pin', 'rod', 'ends': tris} of the rotating crank and pushrod."""
    import numpy as np
    pec = pectoral()
    drv, rot_z, cx, cy, _can, ring = _pectoral_drive()
    z0, z1, out_r = pec['crank']
    th, c_pin, h_pin, u_rod = crank_at(roll)
    ring_tris = [((t @ rot_z) + (cx, cy, 0.0), c) for t, c in drv['ring']]
    band = [((t @ rot_z) + (cx, cy, 0.0), col)
            for t, col in cl.turned([(z0, ring + 3e-4, out_r), (z1, ring + 3e-4, out_r)], th + 0.5 * math.pi)]
    root = [(cx + (out_r - 0.001) * math.cos(th + a), cy + (out_r - 0.001) * math.sin(th + a))
            for a in np.radians(np.linspace(-36.0, 36.0, 9))]
    arm_2d = cl.filleted(leaves.hull(root + leaves.ring((float(c_pin[0]), float(c_pin[1])), PIN_R + 0.006)))
    arm = em._rounded(arm_2d, z0, z1, em.FRAME, axis=2, r=0.0015, steps=1)
    pin = km._rod((c_pin[0], c_pin[1], z0 + 0.0005), (c_pin[0], c_pin[1], Z_CRANK + 0.0065), PIN_R, em.CARBON)
    rod = km._rod(c_pin + 0.014 * u_rod, h_pin - 0.014 * u_rod, ROD_R, em.CARBON)
    ends = []
    for pt, ax_z in ((c_pin, 1.0), (h_pin, -1.0)):
        ends += km._rod(pt - (0.0, 0.0, 0.0055), pt + (0.0, 0.0, 0.0055), PIN_R + 0.0036, hf.STEEL)
        ends += km._rod(pt, pt + ax_z * 0.015 * u_rod, ROD_R + 0.003, em.FRAME)
        ends += km._rod(pt - (0.0, 0.0, 0.006), pt + (0.0, 0.0, 0.006), PIN_R + 0.001, POLYMER)
    return {'ring': ring_tris, 'band': band, 'arm': arm, 'pin': pin, 'rod': rod, 'ends': ends}


def _crank_and_rod(roll=0.0):
    """[(tri, col)] of the rotating output ring, clamped crank and pushrod at `roll` deg."""
    return [t for v in _crank_parts(roll).values() for t in v]


def chest(roll=0.0, right=False):
    """{name: tris} of one side's pectoral drive, mount, crank and pushrod."""
    tris = _pectoral_still() + _crank_and_rod(roll)
    if right:
        tris = [((t * (-1.0, 1.0, 1.0))[::-1], col) for t, col in tris]
    return {'pectoral': tris}


@functools.lru_cache(maxsize=None)
def parts():
    """{name: tris} at the shoulder in `shoulder_laid`'s frame (`roll = 0`): 'still', 'yoke', 'ring'."""
    import numpy as np
    lay, lk = laid(), linkage()
    (c0, c1), (f0, f1), (bx0, bx1), root = lay['cheek'], lay['fork'], lay['bridge'], lay['root']
    arm_pts_f = cl.filleted(leaves.hull(leaves.ring((X_ABD, 0.0), HUB_R)
                                        + [(root, -0.015), (root + 0.003, -0.015),
                                           (root + 0.003, 0.015), (root, 0.015)]))
    arm_pts_r = cl.filleted(leaves.hull(leaves.ring((X_ABD, 0.0), HUB_R)
                                        + [(0.094, -0.015), (0.098, -0.015),
                                           (0.098, 0.015), (0.094, 0.015)]))
    fork_tris = (em._rounded(arm_pts_r, f0, f1, em.CARBON, axis=2, r=0.001)
                 + em._rounded(arm_pts_f, -f1, -f0, em.CARBON, axis=2, r=0.001))
    from tools.render import foot
    web_xz = cl.filleted([(0.094, f0), (0.098, f1), (root + 0.003, 0.020),
                          (root + 0.003, -0.020), (root + 0.003, -f1), (root, -f0), (root, 0.018)], r=0.0015)
    sock = [(t[:, (1, 0, 2)][::-1], col) for t, col in foot._slab(web_xz, -0.015, 0.015, em.FRAME)]
    stops = []
    r_st = HUB_R + 0.5 * STOP_M
    for deg, way in ((TRAVEL[0], -1.0), (TRAVEL[1], 1.0)):
        b = math.radians(deg) + way * STOP_M / r_st
        at = (X_ABD - r_st * math.sin(b), -r_st * math.cos(b))
        stops += km._rod((at[0], at[1], -f1), (at[0], at[1], -c1 - 0.002), 0.45 * STOP_M, em.FRAME)
    # Stage-1 gimbal cheeks stay at x >= X_ABD - HUB_R = +0.027 m, 5 mm inboard of Stage 2's arm saddle!
    cheek_pts = cl.filleted(leaves.hull(leaves.ring((X_ABD, 0.0), HUB_R - 0.001)
                                        + [(bx0, -0.015), (bx1, -0.015), (bx1, 0.015), (bx0, 0.015)]))
    cheeks = (em._rounded(cheek_pts, c0, c1, em.CARBON, axis=2, r=0.001)
              + em._rounded(cheek_pts, -c1, -c0, em.CARBON, axis=2, r=0.001))
    bridge = em._rounded([(-0.016, -c1), (-0.016, c1), (0.016, c1), (0.016, -c1)], bx0, bx1, em.FRAME, axis=0, r=0.002)
    stop_rib = em._rounded([(X_ABD - 0.5 * STOP_M, -r_st - 0.5 * STOP_M),
                            (X_ABD + 0.5 * STOP_M, -r_st - 0.5 * STOP_M),
                            (X_ABD + 0.5 * STOP_M, -HUB_R - 0.003),
                            (X_ABD - 0.5 * STOP_M, -HUB_R - 0.003)],
                           -f0 + 0.0015, -c1 + 0.001, em.FRAME, axis=2, r=0.0005)
    _turn, arm, p_abd = _torso_frame()
    h_torso = np.array([float(p_abd[0]) + HORN_R * math.cos(lk['beta0']),
                        float(p_abd[1]) + HORN_R * math.sin(lk['beta0']), Z_HORN])
    h_sk = (h_torso - arm) @ _turn.T
    horn_2d = cl.filleted(leaves.hull(leaves.ring((X_ABD, 0.0), 0.014) + leaves.ring((float(h_sk[0]), float(h_sk[1])), 0.011)))
    hz = lay['horn_z']
    horn = (em._rounded(horn_2d, hz + 0.0065, hz + 0.0115, em.CARBON, axis=2, r=0.001)
            + km._rod((h_sk[0], h_sk[1], hz - 0.0065), (h_sk[0], h_sk[1], hz + 0.011), PIN_R, em.CARBON))
    lz0, lz1 = _lever_z()
    pins = []
    for sgn, z_out in ((1.0, lz1), (-1.0, hz + 0.0055)):
        z_a, z_b = min(sgn * (c0 - 0.001), z_out), max(sgn * (c0 - 0.001), z_out)
        pins += [(t[:, (1, 2, 0)] + (X_ABD, 0.0, 0.0), col) for t, col in em._drum(lay['pin'], z_a, z_b, em.BOLT)]
        pins += [(t[:, (1, 2, 0)] + (X_ABD, 0.0, 0.0), col)
                 for t, col in em._drum(lay['pin'] + BUSH_M, sgn * f0, sgn * f1, POLYMER)]
    lever = em._rounded([(X_ABD - 0.006, -REACH_M - 0.005), (X_ABD + 0.006, -REACH_M - 0.005),
                         (X_ABD + 0.006, 0.006), (X_ABD - 0.006, 0.006)], lz0, lz1, em.FRAME, axis=2, r=0.0015)
    return {'still': fork_tris + sock + stops, 'yoke': cheeks + bridge + stop_rib + horn + lever, 'ring': pins}


def _lever_z():
    """(From, to) along +z (behind) of the bus span's lever on the rear clevis pin, m."""
    f1 = laid()['fork'][1]
    return f1 + 0.0025, f1 + 0.0065


def rolled(tris, roll):
    """`tris` turned `roll` deg about Stage 1's z-axis through `(X_ABD, 0, 0)`."""
    import numpy as np
    c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
    rz = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
    piv = np.array([X_ABD, 0.0, 0.0])
    return [((tri - piv) @ rz.T + piv, colour) for tri, colour in tris]


#: Turn from `hung`'s frame `(a, b, c) -> (-c, b, a)` (shifted by `(X_ABD, 0, 0)`) to `shoulder_laid`'s frame.
BACK = ((0.0, 0.0, -1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0))


@functools.lru_cache(maxsize=None)
def span():
    """Bus hose across Stage 1's rear clevis pin (`hung`), facing across `(X_ABD, 0)` at mid-travel."""
    lay, (l0, l1) = laid(), _lever_z()
    x, near = l1 + tees.FITTING[0], lay['pin'] + 0.005
    a = math.radians(0.5 * (TRAVEL[0] + TRAVEL[1]))
    return {'name': 'abduction', 'kind': 'bus', 'travel': (-TRAVEL[1], -TRAVEL[0]),
            'from': ('still', (x, REACH_M * math.cos(a), -REACH_M * math.sin(a)), (0.0, -math.cos(a), math.sin(a))),
            'to': ('turning', (x, -REACH_M, 0.0), (0.0, 1.0, 0.0)),
            'solids': [('still', (0.5 * sum(lay['fork']), 0.0, 0.0), (1.0, 0.0, 0.0), near, 0.5 * FORK_T),
                       ('turning', (0.5 * (l0 + l1), 0.0, 0.0), (1.0, 0.0, 0.0), near, 0.5 * (l1 - l0))]}


def span_least():
    """Least bend radius of the rear Stage-1 bus span across `TRAVEL`, m (`hung.least`)."""
    from tools.render import hung
    return hung.least(span())


def posed(roll=0.0, chest_parts=False):
    """{name: tris}: Stage-1 clevis, turned gimbal and bus span at `roll` deg (plus pectoral & pushrod when asked)."""
    import numpy as np
    from tools.render import hung
    got = parts()
    line = hung.path(span(), -roll) @ np.array(BACK).T + np.array([X_ABD, 0.0, 0.0])
    out = {'still': got['still'], 'yoke': rolled(got['yoke'] + got['ring'], roll), 'span': tees.hose(line)}
    if chest_parts:
        out['pectoral'] = to_sketch(chest(roll)['pectoral'])
    return out


@functools.lru_cache(maxsize=None)
def aired(step=5.0, ease=3.0):
    """Least air, m, between what turns with `roll` (gimbal + Stage 2) and the fixed crossbar fork."""
    import numpy as np
    from tools.render import shoulder_mount as sm
    piv = np.array([X_ABD, 0.0, 0.0])
    # Exclude stop pins at the ends when checking within `ease` of the stops? No, stop pins only meet at TRAVEL ends!
    fixed_tris = [t for t in parts()['still'] if np.hypot(t[0][:, 0] - X_ABD, t[0][:, 1]).min() > laid()['pin'] + BUSH_M + 0.002]
    fixed = km._points(fixed_tris + sm.still()['crossbar'], 0.002)
    h = 0.006
    keys = km._key(np.floor(fixed / h).astype(np.int64))
    order = np.argsort(keys)
    near = laid()['pin'] + BUSH_M + 0.003
    mov_pts = km._points([t for t in parts()['yoke']]
                         + [t for k, v in sm.still().items() if k != 'crossbar' for t in v]
                         + [t for v in sm.turning().values() for t in v if t[0][:, 1].min() > -laid()['down']], 0.002)
    # Exclude points inside the pin hub where clevis pin runs, and check across travel:
    keep = (np.hypot(mov_pts[:, 0] - X_ABD, mov_pts[:, 1]) > near) & (mov_pts[:, 2] > laid()['horn_z'] + 0.013)
    moving = mov_pts[keep] - piv
    out = 9.0
    for roll in np.arange(TRAVEL[0] + ease, TRAVEL[1] - ease + 0.1, step):
        c, s = math.cos(math.radians(roll)), math.sin(math.radians(roll))
        rz = np.array([[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]])
        out = min(out, float(km._least(keys[order], fixed[order], moving @ rz.T + piv, h)))
    return out


@functools.lru_cache(maxsize=None)
def crossed():
    """Triangle crossings across `TRAVEL`: bilateral pectoral overlap plus moving crank/rod vs stationary stack."""
    from tools.render import clips, shoulder_mount as sm
    still = _pectoral_still()
    total = 0
    for roll in (TRAVEL[0], -15.0, 0.0, 45.0, 90.0, TRAVEL[1]):
        cr = _crank_parts(roll)
        mov = [t for k, v in cr.items() if k != 'ring' for t in v]
        left = still + [t for v in cr.values() for t in v]
        right = [((t * (-1.0, 1.0, 1.0))[::-1], c) for t, c in left]
        sh_p = sm.posed(0.0, roll)
        sh_turn = to_torso(sh_p['housing'] + sh_p['arm saddle'])
        total += clips.crossed(left, right) + clips.crossed(mov, still) + clips.crossed(cr['rod'], sh_turn)
    return total


def checked():
    """{'held', 'air', 'travel', 'reaches', 'bend', 'length', 'lever', 'crossing'}: what the suite holds."""
    from machine.body import mechatronics
    from tools.render import hung
    return {'held': round(min(held().values()), 3), 'air': round(1e3 * aired(), 2), 'travel': TRAVEL,
            'reaches': TRAVEL[1] + sl.pivot()[1],
            'bend': (round(1e3 * span_least(), 1), 1e3 * float(mechatronics.materials()['hoses']['bus'][4])),
            'length': round(1e3 * hung.long(span()), 1), 'lever': round(linkage()['eff'], 3),
            'crossing': crossed()}


def main(argv=None):
    from PIL import Image, ImageDraw
    from tools.render import shoulder_mount as sm
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    args = parser.parse_args(argv)
    views = [(0.0, 28.0, 14.0), (0.0, -155.0, 14.0), (TRAVEL[1], 25.0, 12.0), (TRAVEL[0], 35.0, 12.0)]
    sheet = Image.new('RGB', (760 * len(views), 800), (18, 18, 18))
    for i, (roll, yaw, tilt) in enumerate(views):
        tris = [t for part in sm.posed(0.0, roll, chest=True).values() for t in part]
        sheet.paste(m.draw(tris, yaw=yaw, pitch=tilt, px=760, half=0.19, centre=(0.04, -0.05, -0.04)), (760 * i, 40))
    ImageDraw.Draw(sheet).text((12, 10), 'pectoral pushrod abduction: rest ahead | behind | T+ %.0f | in %.0f deg'
                               % (TRAVEL[1], TRAVEL[0]), fill=(235, 235, 235))
    sheet.save(args.out)
    for name, margin in held().items():
        print('  %-12s %.2f' % (name, margin))
    print(args.out, checked())
    return 0


if __name__ == '__main__':
    sys.exit(main())
