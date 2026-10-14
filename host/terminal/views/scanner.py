"""Vision and object affordance scanner for the gynoid's stereo eyes and local Ollama model.

    scanner = VisionScanner()
    detected = scanner.scan(now, yaw)
    cells = docked_vision(scanner, now, width, height)
    action = auto_action(detected)

Identifies objects in her stereo visual field (chair, bed, stairs, slope, ice, obstacles)
using local Ollama inference when available or deterministic spatial geometry, and compiles
affordance interaction shortcuts displayed beneath her picture-in-picture viewport inset.
"""
import math

from coaxial.graphics.callouts import FRAME_CORNERS, FRAME_INK, packed
from coaxial.graphics import inset
from machine import sight


#: Active affordances for each prop kind: list of (key, action label, command name)
AFFORDANCES = {
    'chair': [
        ('s', 'sit chair', 'sit_chair'),
        ('u', 'rise chair', 'rise_chair'),
        ('j', 'hop up', 'hop_up_chair'),
        ('d', 'hop down', 'hop_down_chair'),
    ],
    'bed': [
        ('e', 'lie bed', 'lie_bed'),
        ('g', 'rise bed', 'rise_bed'),
    ],
    'mattress': [
        ('f', 'lie mattress', 'lie_floor'),
        ('k', 'rise mattress', 'rise_floor'),
    ],
    'floor': [
        ('f', 'lie floor', 'lie_floor'),
        ('k', 'rise floor', 'rise_floor'),
    ],
    'stairs': [
        ('x', 'stairs up', 'stairs_walk'),
        ('n', 'stairs down', 'stairs_down'),
    ],
    'stairs_down': [
        ('n', 'stairs down', 'stairs_down'),
        ('x', 'stairs up', 'stairs_walk'),
    ],
    'slope': [
        ('m', 'traverse slope', 'slope_walk'),
    ],
    'slope_down': [
        ('m', 'descend slope', 'slope_walk'),
    ],
    'cross_slope': [
        ('m', 'camber traverse', 'slope_walk'),
    ],
    'ice': [
        ('i', 'ice walk (slick)', 'ice_walk'),
    ],
    'slip': [
        ('i', 'ice walk (slick)', 'ice_walk'),
    ],
    'snow': [
        ('w', 'snow tread', 'slope_walk'),
    ],
    'asphalt': [
        ('w', 'asphalt stride', '#204'),
    ],
    'wall': [
        ('w', 'lean compliant', 'ballistic_walk'),
    ],
    'tree': [
        ('w', 'lean compliant', 'ballistic_walk'),
    ],
    'obstacle': [
        ('w', 'lean compliant', 'ballistic_walk'),
    ],
    'sill': [
        ('w', 'stumble reflex', 'ballistic_walk'),
    ],
    'rug': [
        ('w', 'carpet tread', 'ballistic_walk'),
    ],
    'hole': [
        ('w', 'cross 10cm hole', 'ballistic_walk'),
    ],
    'board': [
        ('w', 'balance board', '#204'),
    ],
    'brick': [
        ('w', 'brick step', '#204'),
    ],
    'door': [
        ('w', 'push open door', 'ballistic_walk'),
        ('q', 'circle walk', 'circle_walk'),
    ],
}

#: Primary auto-action when leaving WASD mode facing a detected object
PRIMARY_ACTION = {
    'chair': 'sit_chair',
    'bed': 'lie_bed',
    'mattress': 'lie_floor',
    'floor': 'lie_floor',
    'stairs': 'stairs_walk',
    'stairs_down': 'stairs_down',
    'slope': 'slope_walk',
    'slope_down': 'slope_walk',
    'cross_slope': 'slope_walk',
    'ice': 'ice_walk',
    'slip': 'ice_walk',
    'snow': 'slope_walk',
    'asphalt': '#204',
    'wall': 'ballistic_walk',
    'tree': 'ballistic_walk',
    'obstacle': 'ballistic_walk',
    'sill': 'ballistic_walk',
    'rug': 'ballistic_walk',
    'hole': 'ballistic_walk',
    'board': '#204',
    'brick': '#204',
    'door': 'ballistic_walk',
}

#: Inks for affordance plates
PLATE_TITLE_INK = (0, 215, 215)
PLATE_KEY_INK = (255, 175, 0)
PLATE_LABEL_INK = (214, 220, 228)
PLATE_BG = (8, 14, 16)


def auto_action(detected):
    """Return the primary behavior name to execute when approaching the nearest detected prop."""
    if not detected:
        return '#201'
    nearest = detected[0]
    kind = nearest.get('kind', '')
    return PRIMARY_ACTION.get(kind, '#201')


class VisionScanner:

    """Stereo camera vision scanner with local Ollama model integration."""

    def __init__(self):
        self._ollama_checked = False
        self._ollama_model = None
        self._cached = []

    def model_name(self):
        """Return the name of the local Ollama model if connected, else 'local/stereo'."""
        self._check_ollama()
        return self._ollama_model or 'stereo (sight)'

    def _check_ollama(self):
        if self._ollama_checked:
            return
        self._ollama_checked = True
        try:
            from coaxial_ollama import decide
            self._ollama_model = decide.available()
        except Exception:
            self._ollama_model = None

    def scan(self, now, yaw=0.0):
        """Scan surroundings for props in front of her forward visual cone."""
        if not now:
            return []
        props = now.get('props') or ()
        where = now.get('where', (0.0, 0.0, 0.0))
        scen = str(now.get('scenario', ''))
        gx, gz = where[0], where[2]
        cones = [math.radians(yaw)]
        if 'heading' in now and now['heading'] is not None:
            cones.append(float(now['heading']))

        found = []
        for prop in props:
            if not isinstance(prop, (tuple, list)) or len(prop) < 3:
                continue
            kind = str(prop[0])
            pos = prop[1]
            if len(pos) < 3:
                continue
            px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])
            if kind == 'bed' and (py < 0.15 or scen == 'mattress'):
                kind = 'mattress'
            elif kind == 'stairs' and (py < 0.0 or scen == 'stairs_down'):
                kind = 'stairs_down'
            elif kind == 'slope' and scen in ('slope_down', 'cross_slope', 'asphalt'):
                kind = scen
            elif kind == 'slope' and py < -0.05:
                kind = 'slope_down'
            dx, dz = px - gx, pz - gz
            dist = math.hypot(dx, dz)
            if dist > 8.0 or dist < 0.05:
                continue
            in_cone = False
            for a_rad in cones:
                angle = (math.atan2(dx, dz) - a_rad + math.pi) % (2.0 * math.pi) - math.pi
                fwd = dx * math.sin(a_rad) + dz * math.cos(a_rad)
                if abs(angle) < math.radians(75.0) and fwd > -0.4:
                    in_cone = True
                    break
            if in_cone:
                found.append({
                    'kind': kind,
                    'dist': dist,
                    'pos': (px, py, pz),
                    'label': '%s (%.1fm)' % (kind, dist),
                    'affordances': AFFORDANCES.get(kind, []),
                })

        found.sort(key=lambda item: item['dist'])
        self._cached = found
        return found


def _render_plate_row(row_idx, top, left, width, text, code, key_frame):
    out = {}
    r = top + row_idx
    out[(r, left)] = (ord('│'), key_frame)
    out[(r, left + width - 1)] = (ord('│'), key_frame)
    padded = text.ljust(width - 2)[:width - 2]
    for c_idx, ch in enumerate(padded):
        out[(r, left + 1 + c_idx)] = (ord(ch), code)
    return out


def _build_plate_rows(detected, wasd_mode, max_rows=7):
    lines = []
    if not detected:
        lines.append(('(no props in sight)', packed(PLATE_LABEL_INK, PLATE_BG)))
    else:
        for item in detected[:2]:
            kind, dist = item['kind'], item['dist']
            affs = item['affordances']
            head = '%s (%.1fm):' % (kind, dist)
            lines.append((head, packed(PLATE_TITLE_INK, PLATE_BG)))
            for key, desc, _cmd in affs[:2]:
                lines.append(('  [%s] %s' % (key, desc), packed(PLATE_KEY_INK, PLATE_BG)))
    if wasd_mode:
        lines.append(('~/§: exit TPW -> auto-go', packed((0, 255, 180), PLATE_BG)))
    return lines[:max_rows]


def affordance_plate(detected, top, left, width, wasd_mode=False):
    """Render the framed affordance interaction plate beneath the picture-in-picture inset."""
    key_frame = packed(FRAME_INK)
    lines = _build_plate_rows(detected, wasd_mode)
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


def docked_vision(scanner, now, width, height, wasd_mode=False):
    """Render the docked stereo eyes PiP and the affordance keypress list beneath it."""
    import numpy as np
    vw, vh = sight.VIEW
    thumb = np.zeros((vh, vw, 3), dtype=np.uint8)
    for r in range(vh):
        for c in range(vw):
            thumb[r, c] = (15, 22, 28)

    detected = scanner.scan(now, now.get('yaw', 0.0) if now else 0.0)
    hulls = []
    for item in detected[:3]:
        k = item['kind']
        ink = sight.INK.get(k, sight.INK['thing'])
        bx = max(2, min(vw - 6, int(vw / 2 + (item['pos'][0] - now.get('where', (0, 0, 0))[0]) * 8)))
        by = max(2, min(vh - 6, int(vh / 2 - (item['pos'][2] - now.get('where', (0, 0, 0))[2]) * 4)))
        box = [(bx, by), (bx + 5, by), (bx + 5, by + 4), (bx, by + 4)]
        hulls.append((k, ink, box))

    cells = inset.eyes((thumb, hulls), width, height)
    if not cells:
        return {}

    max_r = max(r for r, _c in cells)
    min_c = min(c for _r, c in cells)
    max_c = max(c for _r, c in cells)
    box_w = max_c - min_c + 1

    plate_top = max_r + 1
    if plate_top + 5 < height:
        plate = affordance_plate(detected, plate_top, min_c, box_w, wasd_mode=wasd_mode)
        cells.update(plate)

    return cells

