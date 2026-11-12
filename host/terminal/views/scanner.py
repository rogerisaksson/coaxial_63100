"""Vision and object affordance scanner for the gynoid's stereo eyes and local Ollama model.

    scanner = VisionScanner()
    detected = scanner.scan(now, look)
    shown = scanner.seen(now)                  # what her eyes' view holds, for her vision's panel
    cells = docked_vision(scanner, now, width, height)
    action = auto_action(detected)

Names what her stereo visual field holds by its class (`machine.eyes.classes`: a wall, a window, a
door, a chair, a table, a bed, stairs, a slope) using local Ollama inference when available or deterministic spatial geometry, and compiles
affordance interaction shortcuts displayed beneath her picture-in-picture viewport inset.
"""
import math
import threading

from coaxial.graphics.callouts import FRAME_CORNERS, FRAME_INK, packed
from coaxial.graphics import inset
from machine.world import room
from machine.eyes import classes, sight, stereo, voxels


#: What F says by a support she sees (`gym.offers`): its verb, and the verb's name.
AFFORDANCES = {'chair': ('sit_chair', 'sit'), 'armchair': ('sit_chair', 'sit'),
               'stool': ('sit_chair', 'sit'), 'bench': ('sit_chair', 'sit'), 'sofa': ('sit_chair', 'sit'),
               'bed': ('lie_bed', 'lie down'), 'mattress': ('lie_floor', 'lie down'),
               'door': ('exit_door', 'go out')}
PRIMARY_ACTION = {kind: verb for kind, (verb, _name) in AFFORDANCES.items()}
#: A box's turn where a thing has none: her map's are upright. Her sight's reach, m. Her vision's panel holds
#: her eyes' view about her heading alone - ACROSS rad either side (`sight.FOVY_DEG` high, W / H as wide), out
#: to her map's reach (`voxels.FAR_M`): held to the page's own view's heading, 75 deg and anything of use
#: within 1.2 m wherever, it named what no look of hers had shown.
UPRIGHT, SIGHT_M = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)), 8.0
ACROSS = math.atan(math.tan(math.radians(sight.FOVY_DEG / 2.0)) * stereo.W / stereo.H)

_PROP_RULES = {
    'bed': lambda py, _sx, _sz, sc: 'mattress' if (py < 0.5 * room.BED[1] or sc == 'mattress') else 'bed',
    'stairs': lambda py, _sx, _sz, sc: 'stairs_down' if (py < 0.0 or sc == 'stairs_down') else 'stairs',
    'slope': lambda py, _sx, _sz, sc: (sc if sc in ('slope_down', 'cross_slope', 'asphalt')
                                       else ('slope_down' if py < -room.THICK else 'slope')),
}


def _in_cone(dx, dz, dist, kind, reach, cones):
    if reach is not None or (kind in AFFORDANCES and dist < 1.2):
        return True
    return any(abs((math.atan2(dx, dz) - a + math.pi) % math.tau - math.pi) < math.radians(75.0)
               and (dx * math.sin(a) + dz * math.cos(a)) > -0.4 for a in cones)

#: Inks for affordance plates
PLATE_TITLE_INK = (0, 215, 215)
PLATE_KEY_INK = (255, 175, 0)
PLATE_LABEL_INK = (214, 220, 228)
PLATE_BG = (8, 14, 16)
#: The bus plate's own faults (CRC), and her wear's service alarms (`wear.told`): a drive's drift
#: past its band, no bus fault.
CRC_INK, WEAR_INK = (255, 80, 80), (200, 120, 255)


def auto_action(detected):
    """The verb the nearest support detected asks, None with none."""
    return next((PRIMARY_ACTION[d['kind']] for d in detected if d['kind'] in PRIMARY_ACTION), None)


def _snap_key(now):
    src = now.get('things') or now.get('props')
    return (id(now), now.get('t'), now.get('scenario'), now.get('epoch'),
            tuple(now.get('where') or ()), now.get('heading'),
            id(src), len(src) if isinstance(src, (tuple, list)) else 0)


class VisionScanner:

    """Stereo camera vision scanner with local Ollama model integration; what she has told apart
    since her scene was laid kept (`identified`)."""

    def __init__(self):
        self._ollama_checked = False
        self._ollama_model = None
        self._laid, self._known, self._places = None, set(), []
        self._snap, self._scans, self._ident_key, self._ident = None, {}, None, []

    def model_name(self):
        """Return the name of the local Ollama model if connected, else 'local/stereo'."""
        self._check_ollama()
        return self._ollama_model or 'stereo (sight)'

    def _check_ollama(self):
        if self._ollama_checked:
            return
        self._ollama_checked = True

        def _probe():
            try:
                from coaxial_ollama import decide
                self._ollama_model = decide.available(timeout=0.35)
            except Exception:
                self._ollama_model = None

        threading.Thread(target=_probe, daemon=True).start()

    def identified(self, now):
        """What her eyes' view has shown (`seen`) since her scene was laid - its scenario, her epoch -,
        wherever it stands now within SIGHT_M, nearest first: a thing once seen is known, her sway at
        its edge none of its; held to the wider `scan`, F offered things no look of hers had shown."""
        laid = (now.get('scenario'), now.get('epoch')) if now else None
        if laid != self._laid:
            self._laid, self._known, self._places, self._ident_key = laid, set(), [], None
        snap = _snap_key(now) if now else None
        if snap is not None and self._ident_key == (snap, len(self._known), len(self._places)):
            return list(self._ident)
        for d in self.seen(now):
            self._known |= d.get('slots', {d['index']})
            lo = (d['pos'][0] - d['extent'][0], d['pos'][2] - d['extent'][2])
            hi = (d['pos'][0] + d['extent'][0], d['pos'][2] + d['extent'][2])
            if not _met(d, self._places):
                self._places.append((d['kind'], lo, hi))
        out = []
        for d in self.scan(now, reach=SIGHT_M):
            slots = d.get('slots', {d['index']})
            if slots & self._known or _met(d, self._places):
                self._known |= slots
                out.append(d)
        self._ident_key, self._ident = (snap, len(self._known), len(self._places)), out
        return list(out)

    def seen(self, now):
        """What her eyes' view holds (`scan`'s within FAR_M, ACROSS of her heading): her vision's panel."""
        heading = float((now or {}).get('heading') or 0.0)
        return [d for d in self.scan(now, reach=voxels.FAR_M)
                if d.get('shown') or abs(math.remainder(
                    math.atan2(d['near'][0] - now['where'][0], d['near'][2] - now['where'][2]) - heading,
                    math.tau)) <= ACROSS]

    def scan(self, now, look=None, reach=None):
        """Her things in her sight, one a thing of its pieces - her registry's, by what each is and the one thing
        each is of as her worker read them (`now['named']`, `now['groups']`, `classes.read`), else the props
        by theirs, a scenario's variants by its rules (`_PROP_RULES`), a piece of furniture's and what she
        cannot tell none; and each building she is outside of, its index from -1 down -: within 75 deg of her
        heading or the heading the view looks along (`look`, rad), SIGHT_M, or with `reach` every one that
        near, nearest first; a thing's distance and kind its nearest piece's, that piece's place `near`, the box
        round its pieces its `pos` and `extent` (half sizes along the world's axes), its lowest slot - a prop's
        place in her state's - its `index`."""
        if not now:
            return []
        snap = _snap_key(now)
        if snap != self._snap:
            self._snap, self._scans = snap, {}
        cached = self._scans.get((look, reach))
        if cached is not None:
            return list(cached)
        things = list(now.get('things') or ())
        props = things or (now.get('props') or ())
        named, groups, built = ((now.get('named') or (), now.get('groups') or (), now.get('buildings') or ())
                                if things else classes.read(props))
        where = now.get('where', (0.0, 0.0, 0.0))
        scen = str(now.get('scenario', ''))
        gx, gz = where[0], where[2]
        cones = [float(now.get('heading') or 0.0)] + ([look] if look is not None else [])
        frustum = any(isinstance(p, (tuple, list)) and len(p) > 6 and float(p[6]) > 0.0 for p in things)

        pieces = {}
        for at, prop in enumerate(props):
            if not isinstance(prop, (tuple, list)) or len(prop) < 3 or len(prop[1]) < 3:
                continue
            raw_kind = named[at] if at < len(named) else None
            px, py, pz = float(prop[1][0]), float(prop[1][1]), float(prop[1][2])
            sz = prop[2] if hasattr(prop[2], '__len__') and len(prop[2]) >= 3 else (0.0, 0.0, 0.0)
            turn = prop[3] if len(prop) > 3 else UPRIGHT
            dx, dz = px - gx, pz - gz
            dist = math.hypot(dx, dz)
            shown = frustum and len(prop) > 6 and float(prop[6]) == 0.0
            if raw_kind is None or dist > (reach or SIGHT_M) or not (shown or _in_cone(dx, dz, dist, raw_kind, reach, cones)):
                continue
            pieces.setdefault(groups[at] if at < len(groups) else at, []).append({
                'kind': raw_kind,
                'index': prop[4] if things and len(prop) > 4 else at,
                'dist': dist,
                'pos': (px, py, pz),
                'extent': tuple(sum(abs(float(a)) * float(h) for a, h in zip(row, sz)) for row in turn),
                'shown': shown,
            })
        found: list[dict] = [_one(ds, scen) for ds in pieces.values()]

        # a building she is outside of (`now['buildings']`, `classes.read`): its distance its shell's
        for k, (kind, centre, half) in enumerate(built):
            dx, dz = float(centre[0]) - gx, float(centre[2]) - gz
            dist = math.hypot(max(abs(dx) - float(half[0]), 0.0), max(abs(dz) - float(half[2]), 0.0))
            if dist <= 0.0 or dist > (reach or SIGHT_M) or not _in_cone(dx, dz, dist, kind, reach, cones):
                continue
            found.append({'kind': kind, 'index': -1 - k, 'dist': dist, 'pos': tuple(centre), 'near': tuple(centre),
                          'extent': tuple(half), 'label': '%s (%.1fm)' % (kind, dist), 'affordances': []})

        found.sort(key=lambda item: (not bool(item['affordances'] and item['dist'] < 3.4), item['dist']))
        self._scans[(look, reach)] = found
        return list(found)


def _met(d, places):
    """Whether `d`'s footprint over the floor meets one of `places` [(kind, (x0, z0), (x1, z1))] of its kind."""
    lo = (d['pos'][0] - d['extent'][0], d['pos'][2] - d['extent'][2])
    hi = (d['pos'][0] + d['extent'][0], d['pos'][2] + d['extent'][2])
    return any(k == d['kind'] and a[0] <= hi[0] and lo[0] <= b[0] and a[1] <= hi[1] and lo[1] <= b[1]
               for k, a, b in places)


def _one(pieces, scen=''):
    """One thing of its `pieces` (`scan`'s, of one group): its nearest piece's kind (by `_PROP_RULES` on the
    whole's box), distance and label, that piece's place `near`, the box round them all its `pos` and `extent`,
    the lowest slot its `index`."""
    near = min(pieces, key=lambda d: d['dist'])
    lo = [min(d['pos'][k] - d['extent'][k] for d in pieces) for k in range(3)]
    hi = [max(d['pos'][k] + d['extent'][k] for d in pieces) for k in range(3)]
    pos = tuple(0.5 * (a + b) for a, b in zip(lo, hi))
    ext = tuple(0.5 * (b - a) for a, b in zip(lo, hi))
    raw = near['kind']
    kind = _PROP_RULES[raw](pos[1], min(ext[0], ext[2]), max(ext[0], ext[2]), scen) if raw in _PROP_RULES else raw
    return dict(near, kind=kind, index=min(d['index'] for d in pieces), slots=frozenset(d['index'] for d in pieces),
                near=near['pos'], pos=pos, extent=ext, label='%s (%.1fm)' % (kind, near['dist']),
                affordances=[AFFORDANCES[kind]] if kind in AFFORDANCES else [],
                shown=any(d.get('shown') for d in pieces))


def _render_plate_row(row_idx, top, left, width, text, code, key_frame):
    out = {}
    r = top + row_idx
    out[(r, left)] = (ord('│'), key_frame)
    out[(r, left + width - 1)] = (ord('│'), key_frame)
    padded = text.ljust(width - 2)[:width - 2]
    for c_idx, ch in enumerate(padded):
        out[(r, left + 1 + c_idx)] = (ord(ch), code)
    return out


def _build_plate_rows(offered, max_rows=7):
    """A row an offer (`gym.offered`'s): its key, its word and its thing; one saying none."""
    return ([('[%s] %s  %s' % (key, word, thing), packed(PLATE_KEY_INK, PLATE_BG))
             for key, _verb, word, _at, thing in offered[:max_rows]]
            or [('(nothing to act on)', packed(PLATE_LABEL_INK, PLATE_BG))])


def affordance_plate(offered, top, left, width):
    """Render the framed affordance interaction plate beneath the picture-in-picture inset:
    `offered` (`gym.offered`'s), the balloons' keys."""
    key_frame = packed(FRAME_INK)
    lines = _build_plate_rows(offered)
    height = len(lines) + 2
    out = {}

    title = ' ACTIONS '
    head_edge = (title + '─' * (width - 2 - len(title)))[:width - 2]
    out[(top, left)] = (ord(FRAME_CORNERS[0]), key_frame)
    out[(top, left + width - 1)] = (ord(FRAME_CORNERS[1]), key_frame)
    for c, ch in enumerate(head_edge):
        out[(top, left + 1 + c)] = (ord(ch), key_frame)

    for r_idx, (text, code) in enumerate(lines):
        out.update(_render_plate_row(r_idx + 1, top, left, width, text, code, key_frame))

    bottom_r = top + height - 1
    out[(bottom_r, left)] = (ord(FRAME_CORNERS[2]), key_frame)
    out[(bottom_r, left + width - 1)] = (ord(FRAME_CORNERS[3]), key_frame)
    for c in range(width - 2):
        out[(bottom_r, left + 1 + c)] = (ord('─'), key_frame)

    return out


BUS_TAGS = {'axis': 'AXS  ', 'left_arm': 'L·ARM', 'left_leg': 'L·LEG', 'right_arm': 'R·ARM', 'right_leg': 'R·LEG'}


def _board_cell(now, joint, alarmed, passive, pk_default):
    if passive:
        return ord('▱'), packed((80, 145, 160), PLATE_BG)
    if joint in alarmed:
        return ord('▰'), packed(WEAR_INK, PLATE_BG)
    tau = abs(float((now.get('torque') or {}).get(joint, 0.0))) if now else 0.0
    pk = max(1.0, float((now.get('peak') or {}).get(joint, pk_default))) if now else pk_default
    deg_c = float((now.get('temp') or {}).get(joint, 25.0)) if now else 25.0
    load = max(0.0, min(1.0, math.sqrt(tau / pk)))
    heat = max(0.0, min(1.0, (deg_c - 35.0) / 70.0))
    use_h = heat > load and heat > 0.35
    u, target_rgb = (heat, (255, 110, 64)) if use_h else (load, (255, 184, 80))
    ink = tuple(int(a + (b - a) * u) for a, b in zip((64, 214, 200), target_rgb))
    return ord('▰'), packed(ink, PLATE_BG)


def _limb_bus_row(r, left, width, inner, b_name, joints, now, alarms, key_frame, trunk_code):
    from machine.body import drives
    out = {(r, left): (ord('│'), key_frame), (r, left + width - 1): (ord('│'), key_frame),
           (r, left + 6): (ord('├'), packed(FRAME_INK, PLATE_BG))}
    tag = BUS_TAGS.get(b_name) or b_name[:5].upper().ljust(5)
    out.update({(r, left + 1 + c): (ord(ch), packed(PLATE_TITLE_INK, PLATE_BG)) for c, ch in enumerate(tag)})
    torques = (now.get('torque') or {}) if now else {}
    pk_share = 0.0
    for u, j in enumerate(joints[:9]):
        pas = drives.passive(j)
        pk = 1.0 if pas else max(1.0, float(drives.peak(j)))
        pk_share = max(pk_share, (0.0 if pas else min(1.0, abs(float(torques.get(j, 0.0))) / pk)))
        out[(r, left + 7 + u)] = _board_cell(now, j, alarms, pas, pk)
    for u in range(len(joints), 9):
        out[(r, left + 7 + u)] = (ord('─'), trunk_code)
    tail = ('%4.0f%%' % (pk_share * 100.0)).rjust(inner - 15)[:inner - 15]
    out.update({(r, left + 16 + c): (ord(ch), packed(PLATE_LABEL_INK, PLATE_BG)) for c, ch in enumerate(tail)})
    return out


def bus_plate(now, top, left, width):
    """Render the framed 10 Mbit/s RS485 Modbus RTU per-extremity SIL bus & 27-board status graphic."""
    from machine.senses import edge
    inner = width - 2
    if inner < 18:
        return {}
    key_frame, trunk_code = packed(FRAME_INK), packed((48, 88, 102), PLATE_BG)
    alarms = {str(a[0]).split('.')[0] for a in ((now.get('wear') or {}).get('alarms') or ())} if now else set()
    buses_now = (now.get('buses') or {}) if now else {}
    crc = sum(int(v[1]) for v in buses_now.values() if len(v) > 1)
    out = {(top, left): (ord(FRAME_CORNERS[0]), key_frame), (top, left + width - 1): (ord(FRAME_CORNERS[1]), key_frame)}
    title = ' RS485 10M SIL '
    for c in range(inner):
        ch = title[c] if c < len(title) else '─'
        out[(top, left + 1 + c)] = (ord(ch), packed(PLATE_TITLE_INK) if c < len(title) else key_frame)
    for idx, (b_name, joints) in enumerate(edge.LIMBS):
        out.update(_limb_bus_row(top + 1 + idx, left, width, inner, b_name, joints, now, alarms, key_frame, trunk_code))
    bot = top + len(edge.LIMBS) + 1
    notes = ((' SHUTDOWN ', packed(CRC_INK)),) * bool(now and now.get('shutdown')) + (
        (' WEAR %d ' % len(alarms), packed(WEAR_INK)),) * bool(alarms) + (
        (' CRC %d ' % crc, packed(CRC_INK)) if crc else (' CRC OK ', packed(PLATE_KEY_INK)),)
    line = [(ord('─'), key_frame)] * inner
    at = inner - 1
    for note, ink in reversed(notes):
        at = max(1, at - len(note))
        line[at:at + len(note)] = [(ord(ch), ink) for ch in note][:inner - at]
    out[(bot, left)] = (ord(FRAME_CORNERS[2]), key_frame)
    out[(bot, left + width - 1)] = (ord(FRAME_CORNERS[3]), key_frame)
    out.update({(bot, left + 1 + c): cell for c, cell in enumerate(line[:inner])})
    return out


def _fallback_seen(now, detected):
    import numpy as np
    vw, vh = sight.VIEW
    thumb = np.full((vh, vw, 3), (15, 22, 28), dtype=np.uint8)
    where = (now.get('where') if now else None) or (0.0, 0.0, 0.0)
    hulls = []
    for item in detected[:3]:
        k = item['kind']
        bx = max(2, min(vw - 6, int(vw / 2 + (item['pos'][0] - where[0]) * 8)))
        by = max(2, min(vh - 6, int(vh / 2 - (item['pos'][2] - where[2]) * 4)))
        hulls.append((k, sight.INK.get(k, sight.INK['thing']), [(bx, by), (bx + 5, by), (bx + 5, by + 4), (bx, by + 4)]))
    return thumb, hulls


def docked_vision(scanner, now, width, height, offered=()):
    """Render the docked stereo eyes PiP, affordance plate (`offered`, `gym.offered`'s), and
    10 Mbit/s RS485 SIL bus graphic."""
    detected = scanner.seen(now)
    seen = (now.get('seen') if now else None) or _fallback_seen(now, detected)
    cells = inset.eyes(seen, width, height)
    if not cells:
        return {}
    max_r = max(r for r, _c in cells)
    min_c = min(c for _r, c in cells)
    box_w = max(c for _r, c in cells) - min_c + 1
    plate_top = max_r + 1
    if plate_top + 5 < height:
        cells.update(affordance_plate(offered, plate_top, min_c, box_w))
    bus_top = max(r for r, _c in cells) + 1
    if plate_top + 5 < height and bus_top + 7 < height - 1:
        cells.update(bus_plate(now, bus_top, min_c, box_w))
    return cells

