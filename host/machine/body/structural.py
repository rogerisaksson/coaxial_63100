"""Static compliance of the robot's skeleton, linkages and carbon ankle leaf; its chassis' masses."""
import math

from machine.body import linkage, mechatronics, skeleton

_MAT, _BUILD, _LIB = mechatronics.materials(), mechatronics.robot(), mechatronics.library()
_SOLIDS, _SHELL = _MAT['solids'], _BUILD['shell']
_MIN_BACKLASH_DEG = min(float(_LIB['gearboxes'][s[1]]['backlash_deg']) for s in _BUILD['stacks'].values())
#: Materials (`materials.toml`): (E GPa, G GPa, allowable MPa).
CARBON, HM_CARBON, ROD, PAHT_CF, STEEL = (
    tuple(float(x) for x in _SOLIDS[k]) for k in ('carbon', 'hm_carbon', 'rod', 'paht_cf', 'steel'))
#: Tube wall share of radius (`[shell]`), von Mises shear/flex share, half-backlash flex limit, and safety factor.
WALL = float(_SHELL['wall_share'])
FLEX_AT = float(_SHELL.get('flex_at', round(1.0 / math.sqrt(3.0), 1)))
FLEX_DEG = float(_SHELL.get('flex_deg', 0.5 * _MIN_BACKLASH_DEG))
MARGIN = float(_SHELL.get('margin', 1.5))
#: Four-bar PAHT-CF plate width and hip-fork carbon crown width, m (`[shell]`).
PLATE_W, CROWN_W = float(_SHELL['plate_w_m']), float(_SHELL['crown_w_m'])
#: Stock roll-wrapped carbon tubes, (outer radius, wall) m, minimum wall and laminate density (`materials.toml`).
_MIN_WALL_M = float(_MAT['min_wall_m'])
TUBES = tuple((float(d) / 2e3, float(t)) for d, t in _MAT['tubes']['stock'])
ROOM, CF_KG_M3 = float(_SHELL['room_m']), float(_MAT['cf_kg_m3'])
#: Steel, kg/m^3 (`materials.toml`): the spine roll's rods and a hip cradle's trunnions.
STEEL_KG_M3 = float(_MAT['steel_kg_m3'])
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
    rods_n = sum(peak[k] / drives.motors(linkage.joints(k)[0]) / min(g[2], g[4])
                 for k, (g, _s) in linkage.RODS.items() if linkage.ROD_AT[k][0] in ('ankle', 'foot'))
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
        pull = peak[kind] / drives.motors(linkage.joints(kind)[0]) / min(crank, horn)
        out.append(('%s rod' % kind, 'pull', length, pull,
                    (math.pi * linkage.ROD_R ** 4 / 4.0, 0.0, math.pi * linkage.ROD_R ** 2), ROD, 0.0))
    return out


def _solid(r):
    return math.pi * r ** 4 / 4.0, math.pi * r ** 4 / 2.0, math.pi * r * r


def _piece(a, b, section, kg_m3):
    """(kg, centre, 3x3 inertia about it) of a straight member from `a` to `b` of `section` (I, J, A)."""
    length, (_i, j, area) = math.dist(a, b), section
    kg = area * length * kg_m3
    w = tuple((q - p) / length for p, q in zip(a, b))
    about, across = kg * j / area, kg * (j / (2.0 * area) + length * length / 12.0)
    return kg, tuple((p + q) / 2.0 for p, q in zip(a, b)), [
        [(across if r == c else 0.0) + (about - across) * w[r] * w[c] for c in range(3)] for r in range(3)]


def _ring(centre, axis, r, half):
    """A ring `r` round and `half` either side of `centre` along `axis`: a stock tube's short length."""
    return _piece(tuple(p - a * half for p, a in zip(centre, axis)), tuple(p + a * half for p, a in zip(centre, axis)),
                  tube(r), CF_KG_M3)


def chassis():
    """[(body, kg, centre, 3x3 inertia about it)]: the chassis each segment or stage carries, its frame, sized
    as `members` sizes it - each limb's bones, the trunk's frames and the boom stock tubes (`tube`), the spine
    roll's rods solid steel, each hip's fork (its crown in plates) and its cradle (trunnions of solid steel), a
    ring a tube's short length. A drive's collars and struts are its fasteners' (`drives`), the rods what a
    limb holds (`build.HOLDS`)."""
    out = []
    for side, s in (('left_', 1.0), ('right_', -1.0)):
        out += [(side + seg,) + _piece(a, b, tube(r), CF_KG_M3) for seg in skeleton.HUNG for a, b, r in skeleton.runs(seg, s)]
        for stage in ('hip_yaw', 'hip_roll'):
            tubes, rings = skeleton.gimbal(stage)
            for k, (a, b, r) in enumerate(tubes):
                a, b, crown = (s * a[0],) + tuple(a[1:]), (s * b[0],) + tuple(b[1:]), plate(2.0 * r, CROWN_W)
                out.append((side + stage,) + (
                    _piece(a, b, _solid(r), STEEL_KG_M3) if stage == 'hip_roll' else
                    _piece(a, b, (crown, crown, 2.0 * r * CROWN_W) if k and r <= skeleton.CROWN_R else tube(r), CF_KG_M3)))
            out += [(side + stage,) + _ring((s * c[0],) + tuple(c[1:]), skeleton.AXES[axis], r, half)
                    for c, axis, r, half in rings]
    for frame, (runs, rings) in skeleton.TRUNK.items():
        for k, pts in enumerate(runs):
            r = skeleton.TRUNK_R[frame][min(k, len(skeleton.TRUNK_R[frame]) - 1)]
            section, kg_m3 = (_solid(r), STEEL_KG_M3) if frame == 'spine_roll' else (tube(r), CF_KG_M3)
            out += [(frame,) + _piece(a, b, section, kg_m3) for a, b in zip(pts, pts[1:])]
        out += [(frame,) + _ring(c, skeleton.AXES[axis], r, half) for c, axis, r, half in rings]
    r, pts = skeleton.BOOM
    return out + [('pelvis',) + _piece(a, b, tube(r), CF_KG_M3) for a, b in zip(pts, pts[1:])]


def judge(_name, kind, length, load, section, material, c):
    """(stress ratio, flex deg) of a member under `load` against its material's allowable."""
    i, j, area = section
    e, g, allow = material[0] * 1e9, material[1] * 1e9, material[2] * 1e6
    if kind == 'bend':
        return load * c / i / allow, math.degrees(FLEX_AT * load * length / (e * i))
    if kind == 'twist':
        return load * c / j / (allow * FLEX_AT), math.degrees(FLEX_AT * load * length / (g * j))
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
    return {kind: sum(by.get(n, 0.0) for n in names + (
        ('%s rod' % kind,) if kind in linkage.RODS and '%s rod' % kind not in names else ()))
            for kind, names in PATH.items()}


def wind_mrad():
    """{joint kind: mrad per N m} of structural wind-up between each gearbox and its driven segment."""
    return {kind: round(1e3 * w, 3) for kind, w in wind(members()).items()}


def _reach(joint, pin, q, roll):
    """How far a rod's length changes a radian its ball's joint turns - `q` its pitch, `roll` its roll, the step."""
    return abs(math.dist(pin, linkage.ball(joint, q, roll)) - math.dist(pin, linkage.ball(joint, -q, -roll))) / 2e-6


def leaf_series(leaf):
    """{joint kind: N m/rad} rotational series stiffness of each pre-curved leaf (`[leaf]`: its `joint` one
    rod's kind or a list of them, each size one for all or one a leaf), Castigliano - about the joint's pitch, a
    pair's roll rod's about its roll (`linkage.PAIRS`). A leaf alone is a bow pinned at its eyes; `leaves` of them
    side by side between two blocks are each held at both roots, a bump between them. Where both rods of a pair
    are leaves they turn the pitch together and the roll against each other: each joint's stiffness both rods'."""
    if not leaf:
        return {}
    joints = leaf['joint'] if isinstance(leaf['joint'], list) else [leaf['joint']]
    e = float(leaf.get('e_gpa', _SOLIDS[leaf['material']][0] if 'material' in leaf else HM_CARBON[0])) * 1e9
    stack = int(leaf.get('leaves', 1))

    def each(key, i, default=0.0):
        v = leaf.get(key, default)
        return float(v[i] if isinstance(v, list) else v)
    out, rods = {}, {}
    for i, joint in enumerate(joints):
        w, t = each('width_m', i), each('thick_m', i)
        up, ahead, r, t0 = linkage.RODS[joint][0][:4]
        pin = (linkage.ROD_AT[joint][1], up - r * math.cos(t0), ahead + r * math.sin(t0))
        bend, b0 = linkage.BENDS[joint], linkage.ball(joint, 0.0, 0.0)
        seg_a, seg_b, chord = math.dist(pin, bend), math.dist(bend, b0), math.dist(pin, b0)
        along = (seg_a * seg_a + chord * chord - seg_b * seg_b) / (2.0 * chord)
        arch = each('arch_m', i, math.sqrt(max(0.0, seg_a * seg_a - along * along)))
        length = each('length_m', i, seg_a + seg_b)
        q, roll = (0.0, 1e-6) if joint in linkage.PAIRS.values() else (1e-6, 0.0)
        arm = abs(math.dist(pin, linkage.ball(joint, q, roll)) - math.dist(pin, linkage.ball(joint, -q, -roll))) / 2e-6
        area, i_sec = w * t, w * t ** 3 / 12.0
        if stack > 1:
            comp_axial = (length * arch * arch / (8.0 * e * i_sec) + length / (e * area)) / stack
        else:
            comp_axial = 8.0 * length * arch ** 2 / (15.0 * e * i_sec) + length / (e * area)
        out[joint], rods[joint] = round(arm ** 2 / comp_axial, 1), (1.0 / comp_axial, pin)
    for pitch, rolled in linkage.PAIRS.items():
        if pitch in rods and rolled in rods:
            for joint, turn in ((pitch, (1e-6, 0.0)), (rolled, (0.0, 1e-6))):
                far = {rod: _reach(rod, pin, *turn) for rod, (_k, pin) in rods.items() if rod in (pitch, rolled)}
                out[joint] = round(sum(rods[rod][0] * reach * reach for rod, reach in far.items()), 1)
    return out


def leaf_tracks(series_map, armature_fn, wind_fn=None, k_grav=0.0, rate_s=None, observe_x=None):
    """{joint kind: gain} SEA rotor torque-loop gains from observer bandwidth and stance gravity stiffness."""
    if not series_map:
        return {}
    from machine.body import drives
    bus_hz, law_div = drives.BUS_HZ, float(drives.LAW_DIV)
    r_s = (law_div + 2.0) / bus_hz if rate_s is None else float(rate_s)
    obs_x = round(2.0 + 1.0 / law_div, 1) if observe_x is None else float(observe_x)
    omega_sea = 0.90 / (obs_x * r_s)
    out = {}
    for kind, k_rot in series_map.items():
        j_name = linkage.joints(kind)[0]
        j_rot = max(armature_fn(j_name), 1e-6)
        k_wnd = 0.0 if wind_fn is None else float(wind_fn(j_name))
        k_eff = (k_wnd * k_rot) / (k_wnd + k_rot) if k_wnd > 0.0 and k_rot > 0.0 else k_rot
        omega_leaf = math.sqrt(k_rot / j_rot)
        g_bw = 0.0 if omega_leaf >= 0.70 * omega_sea else max(0.0, (omega_sea / omega_leaf) ** 2 - 1.0)
        g_grav = max(0.0, float(k_grav) / max(k_eff, 1e-6) - 1.0) if k_eff > 0.0 else 0.0
        out[kind] = round(max(g_bw, g_grav), 4)
    return out


def leaf_track(series_map, armature_fn, rate_s=None, observe_x=None):
    """SEA rotor torque-loop gain ceiling across sprung joints (`leaf_tracks`)."""
    return max(leaf_tracks(series_map, armature_fn, rate_s=rate_s, observe_x=observe_x).values(), default=0.0)
