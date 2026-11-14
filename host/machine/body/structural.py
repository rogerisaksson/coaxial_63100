"""Static compliance of her skeleton, linkages and carbon ankle leaf."""
import math

from machine.body import linkage, mechatronics, skeleton

_MAT, _BUILD = mechatronics.materials(), mechatronics.robot()
_SOLIDS, _SHELL, _BUS = _MAT['solids'], _BUILD['shell'], _BUILD['bus']
#: Materials (`materials.toml`): (E GPa, G GPa, allowable MPa).
CARBON, HM_CARBON, ROD, PAHT_CF, STEEL = (
    tuple(float(x) for x in _SOLIDS[k]) for k in ('carbon', 'hm_carbon', 'rod', 'paht_cf', 'steel'))
#: Tube wall share of radius, flex evaluation load fraction, flex limit (deg), and clamp margin (`[shell]`).
WALL, FLEX_AT, FLEX_DEG, MARGIN = (
    float(_SHELL[k]) for k in ('wall_share', 'flex_at', 'flex_deg', 'margin'))
#: Four-bar PAHT-CF plate width and hip-fork carbon crown width, m (`[shell]`).
PLATE_W, CROWN_W = float(_SHELL['plate_w_m']), float(_SHELL['crown_w_m'])
#: Stock roll-wrapped carbon tubes, (outer radius, wall) m, minimum wall and laminate density (`materials.toml`).
_MIN_WALL_M = float(_MAT['min_wall_m'])
TUBES = tuple((float(d) / 2e3, float(t)) for d, t in _MAT['tubes']['stock'])
ROOM, CF_KG_M3 = float(_SHELL['room_m']), float(_MAT['cf_kg_m3'])
#: Series structural load path from each joint kind's gearbox to its driven segment.
PATH = {
    'spine': ('spine frame 0', 'pelvis frame 0'),
    'spine_roll': ('spine_roll crank', 'spine_roll horn', 'spine_roll rod'),
    'waist': ('torso frame 0',),
    'neck': ('neck frame 0',),
    'head': ('neck frame 0',),
    'shoulder': ('torso frame 1', 'upper_arm run 0'),
    'elbow': ('upper_arm run 2', 'forearm run 0'),
    'wrist': ('forearm run 1',),
    'gripper': ('forearm run 1',),
    'hip_yaw': ('hip fork 0', 'hip fork 1', 'hip fork 2'),
    'hip_roll': ('hip cradle 0', 'hip fork 3'),
    'hip': ('thigh run 0', 'thigh run 1'),
    'knee': ('shank run 0', 'shank run 1'),
    'ankle': ('ankle rod', 'foot run 2'),
    'ankle_roll': ('ankle_roll rod',),
    'foot': ('foot run 2',),
}


def tube_section(r, t):
    """(I, J, A) m^4, m^4, m^2 of a hollow circular tube of outer radius `r` and wall `t`."""
    ri = r - t
    return (math.pi * (r ** 4 - ri ** 4) / 4.0, math.pi * (r ** 4 - ri ** 4) / 2.0,
            math.pi * (r * r - ri * ri))


def tube(r):
    """(I, J, A) of a carbon tube `r` round: stock wall (`TUBES`), else `WALL * r` (>= `_MIN_WALL_M`)."""
    return tube_section(r, next((t for rr, t in TUBES if math.isclose(rr, r)), max(WALL * r, _MIN_WALL_M)))


def plate(t, w):
    """Area moment `I` m^4 of a rectangular plate `t` thick and `w` wide bent in its plane."""
    return t * w ** 3 / 12.0


def members(peak=None):
    """[(name, kind, length m, load, (I, J, A), material, c)] across bones, frames, gimbals and rods."""
    from machine.body import drives
    if peak is None:
        peak = {k: 0.0 for k in drives.STACKS}
    out = []
    ends = {'thigh': ('hip', 'knee'), 'shank': ('knee', 'ankle'), 'upper_arm': ('shoulder', 'elbow'),
            'forearm': ('elbow', 'wrist'), 'foot': ('ankle', 'foot')}
    rods_n = sum(peak[k] / drives.motors('left_' + k) / min(g[2], g[4])
                 for k, (g, _s) in linkage.RODS.items())
    for seg, (a, b) in ends.items():
        for k, (p, q, r) in enumerate(skeleton.runs(seg)):
            length = math.dist(p, q)
            if seg == 'foot' or (seg == 'shank' and k >= 2):
                out.append(('%s run %d' % (seg, k), 'pull', length, rods_n, tube(r), CARBON, r))
            else:
                out.append(('%s run %d' % (seg, k), 'bend', length, max(peak[a], peak[b]), tube(r),
                            HM_CARBON, r))
    rings = skeleton.TRUNK['pelvis'][1]
    couple = peak['spine'] / abs(rings[0][0][2] - rings[1][0][2])
    for frame, (runs, _rings) in skeleton.TRUNK.items():
        for k, pts in enumerate(runs):
            r = skeleton.TRUNK_R[frame][min(k, len(skeleton.TRUNK_R[frame]) - 1)]
            length = (math.dist(pts[0], pts[-1]) if frame == 'pelvis'
                      else sum(math.dist(p, q) for p, q in zip(pts, pts[1:])))
            mat = STEEL if frame == 'spine_roll' else HM_CARBON
            sec = tube(r) if frame != 'spine_roll' else (
                math.pi * r ** 4 / 4.0, math.pi * r ** 4 / 2.0, math.pi * r * r)
            load = {'pelvis': couple * (0.5 if k < 2 else 1.0) * length,
                    'spine_roll': couple * length, 'spine': peak['spine'],
                    'torso': peak['waist'] + peak['shoulder'] if k == 0 else peak['shoulder'],
                    'neck': peak['neck']}[frame]
            out.append(('%s frame %d' % (frame, k), 'bend',
                        length / 2.0 if frame == 'torso' and k == 1 else length, load, sec, mat, r))
    r, pts = skeleton.BOOM
    out.append(('boom', 'twist', math.dist(pts[0], pts[1]), peak['hip'], tube(r), HM_CARBON, r))
    for k, (a, b, r) in enumerate(skeleton.gimbal('hip_yaw')[0]):
        mode, load_nm = ('twist', peak['hip_yaw']) if k == 0 else (
            'bend', peak['hip'] * 0.5 if r >= 0.01 else peak['hip_yaw'])
        if k and r <= skeleton.CROWN_R:
            i = plate(2.0 * r, CROWN_W)
            out.append(('hip fork %d' % k, mode, math.dist(a, b), load_nm,
                        (i, i, 2.0 * r * CROWN_W), HM_CARBON, CROWN_W / 2.0))
        else:
            out.append(('hip fork %d' % k, mode, math.dist(a, b), load_nm, tube(r), CARBON, r))
    cradle_span = 2.0 * skeleton.BEARING[2]
    for k, (a, b, r) in enumerate(skeleton.gimbal('hip_roll')[0]):
        length = math.dist(a, b)
        out.append(('hip cradle %d' % k, 'bend', length, peak['hip'] / cradle_span * length,
                    (math.pi * r ** 4 / 4.0, math.pi * r ** 4 / 2.0, math.pi * r * r), STEEL, r))
    for name, _rides, shape in skeleton.held('left_'):
        if shape[0] == 'tube':
            kind = drives.kind(name.replace('left_', '').rstrip('+'))
            _t, a, b, r = shape
            out.append((name.replace('left_', '') + ' strut', 'pull', math.dist(a, b),
                        peak.get(kind, 0.0) / max(0.02, math.hypot(a[1], a[2])),
                        (math.pi * r ** 4 / 4.0, 0.0, math.pi * r * r), PAHT_CF, r))
    for kind, (_a, _b, r_c, _t0, horn, _b0, _z, _s) in linkage.PLANAR.items():
        pull = peak[kind] / horn
        for part, arm in (('crank', r_c), ('horn', horn)):
            i = plate(2.0 * linkage.PLATE_R, PLATE_W)
            out.append(('%s %s' % (kind, part), 'bend', arm, pull * arm,
                        (i, i, 2.0 * linkage.PLATE_R * PLATE_W), PAHT_CF, PLATE_W / 2.0))
        length = math.dist(linkage.planar(kind, 0.0)[0], linkage.planar(kind, 0.0)[1])
        out.append(('%s rod' % kind, 'pull', length, pull, tube(linkage.ROD_R), ROD, 0.0))
    for kind, (geo, _stroke) in linkage.RODS.items():
        length, _x, crank, _t, horn, _b = geo
        pull = peak[kind] / drives.motors('left_' + kind) / min(crank, horn)
        out.append(('%s rod' % kind, 'pull', length, pull,
                    (math.pi * linkage.ROD_R ** 4 / 4.0, 0.0, math.pi * linkage.ROD_R ** 2), ROD, 0.0))
    return out


def judge(_name, kind, length, load, section, material, c):
    """(stress ratio, flex deg) of a member under `load` against its material's allowable."""
    i, j, area = section
    e, g, allow = material[0] * 1e9, material[1] * 1e9, material[2] * 1e6
    if kind == 'bend':
        return load * c / i / allow, math.degrees(FLEX_AT * load * length / (e * i))
    if kind == 'twist':
        return load * c / j / (allow * 0.6), math.degrees(FLEX_AT * load * length / (g * j))
    euler = math.pi ** 2 * e * i / max(length, 0.02) ** 2
    return max(load / area / allow, load / euler), 0.0


def compliance(name, kind, length, _load, section, material, _c):
    """rad per N m of joint torque wound in one member: L/(EI), L/(GJ), or rod stretch L/(EA h^2)."""
    i, j, area = section
    e, g = material[0] * 1e9, material[1] * 1e9
    if kind == 'bend':
        return length / (e * i)
    if kind == 'twist':
        return length / (g * j)
    stem = name.split()[0]
    horn = (min(linkage.RODS[stem][0][2:5:2]) if stem in linkage.RODS
            else linkage.PLANAR.get(stem, (0, 0, 0, 0, 0.02))[4])
    return length / (e * area * horn * horn)


def wind(rows):
    """{joint kind: rad per N m} summed in series along `PATH` over member rows or `(member, judged)`."""
    raw = [r[0] if isinstance(r[0], tuple) else r for r in rows]
    by = {m[0]: compliance(*m) for m in raw}
    return {kind: sum(by.get(n, 0.0) for n in names) for kind, names in PATH.items()}


def wind_mrad():
    """{joint kind: mrad per N m} of structural wind-up between each gearbox and its driven segment."""
    return {kind: round(1e3 * w, 3) for kind, w in wind(members()).items()}


def leaf_series(leaf):
    """{joint kind: N m/rad} rotational series stiffness of a pre-curved carbon leaf (`[leaf]`), Castigliano."""
    if not leaf:
        return {}
    joint = leaf['joint']
    e = float(leaf.get('e_gpa', HM_CARBON[0])) * 1e9
    w, t = float(leaf['width_m']), float(leaf['thick_m'])
    up, ahead, r, t0 = linkage.RODS[joint][0][:4]
    pin = (linkage.ROD_AT[joint][1], up - r * math.cos(t0), ahead + r * math.sin(t0))
    bend, b0 = linkage.BENDS[joint], linkage.ball(joint, 0.0, 0.0)
    seg_a, seg_b, chord = math.dist(pin, bend), math.dist(bend, b0), math.dist(pin, b0)
    along = (seg_a * seg_a + chord * chord - seg_b * seg_b) / (2.0 * chord)
    arch = float(leaf.get('arch_m', math.sqrt(max(0.0, seg_a * seg_a - along * along))))
    length = float(leaf.get('length_m', seg_a + seg_b))
    arm = abs(math.dist(pin, linkage.ball(joint, 1e-6, 0.0))
              - math.dist(pin, linkage.ball(joint, -1e-6, 0.0))) / 2e-6
    area, i_sec = w * t, w * t ** 3 / 12.0
    comp_axial = 8.0 * length * arch ** 2 / (15.0 * e * i_sec) + length / (e * area)
    return {joint: round(arm ** 2 / comp_axial, 1)}


def leaf_track(series_map, armature_fn, rate_s=None, observe_x=None):
    """SEA rotor torque-loop gain from leaf natural frequency vs observer/rate bandwidth ceiling."""
    if not series_map:
        return 0.0
    bus_hz, law_div = float(_BUS['bus_hz']), float(_BUS['law_div'])
    r_s = (law_div + 2.0) / bus_hz if rate_s is None else float(rate_s)
    obs_x = round(2.0 + 1.0 / law_div, 1) if observe_x is None else float(observe_x)
    omega_sea = 0.90 / (obs_x * r_s)
    gains = []
    for kind, k_rot in series_map.items():
        j_rot = max(armature_fn('left_' + kind), 1e-6)
        omega_leaf = math.sqrt(k_rot / j_rot)
        gains.append(0.0 if omega_leaf >= 0.70 * omega_sea else max(0.0, (omega_sea / omega_leaf) ** 2 - 1.0))
    return max(gains)
