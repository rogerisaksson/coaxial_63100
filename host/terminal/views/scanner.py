"""Vision and object affordance scanner for the gynoid's stereo eyes and local Ollama model.

    scanner = VisionScanner()
    detected = scanner.scan(now, look)
    cells = docked_vision(scanner, now, width, height)
    action = auto_action(detected)

Identifies objects in her stereo visual field (chair, bed, stairs, slope, ice, obstacles)
using local Ollama inference when available or deterministic spatial geometry, and compiles
affordance interaction shortcuts displayed beneath her picture-in-picture viewport inset.
"""
import math
import threading

from coaxial.graphics.callouts import FRAME_CORNERS, FRAME_INK, packed
from coaxial.graphics import inset
from machine import sight


#: What F says by the support nearest her (`gym.offered`): its verb, and the verb's name.
AFFORDANCES = {'chair': ('sit_chair', 'sit'), 'bed': ('lie_bed', 'lie down'),
               'mattress': ('lie_floor', 'lie down')}
PRIMARY_ACTION = {kind: verb for kind, (verb, _name) in AFFORDANCES.items()}

#: Inks for affordance plates
PLATE_TITLE_INK = (0, 215, 215)
PLATE_KEY_INK = (255, 175, 0)
PLATE_LABEL_INK = (214, 220, 228)
PLATE_BG = (8, 14, 16)


def auto_action(detected):
    """The verb the nearest support detected asks, None with none."""
    return next((PRIMARY_ACTION[d['kind']] for d in detected if d['kind'] in PRIMARY_ACTION), None)


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

        def _probe():
            try:
                from coaxial_ollama import decide
                self._ollama_model = decide.available(timeout=0.35)
            except Exception:
                self._ollama_model = None

        threading.Thread(target=_probe, daemon=True).start()

    def scan(self, now, look=None, reach=None):
        """The props in her sight - within 75 deg of her heading or the heading the view looks
        along (`look`, rad), 8 m -, or with `reach` every one that near, nearest first."""
        if not now:
            return []
        things = now.get('things') or ()
        props = [((t[0], (t[1][0], t[3] * 0.5, t[1][1]), (t[2][0], t[3] * 0.5, t[2][1])))
                 for t in things if len(t) >= 4] or (now.get('props') or ())
        where = now.get('where', (0.0, 0.0, 0.0))
        scen = str(now.get('scenario', ''))
        gx, gz = where[0], where[2]
        cones = [float(now.get('heading') or 0.0)] + ([look] if look is not None else [])

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
            if dist > (reach or 8.0) or dist < 0.05:
                continue
            in_cone = reach is not None
            for a_rad in () if in_cone else cones:
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
                    'affordances': [AFFORDANCES[kind]] if kind in AFFORDANCES else [],
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


def _build_plate_rows(detected, max_rows=7):
    lines, detected = [], [d for d in detected if d['affordances']]
    if not detected:
        lines.append(('(nothing to act on)', packed(PLATE_LABEL_INK, PLATE_BG)))
    else:
        for item in detected[:2]:
            kind, dist = item['kind'], item['dist']
            affs = item['affordances']
            head = '%s (%.1fm):' % (kind, dist)
            lines.append((head, packed(PLATE_TITLE_INK, PLATE_BG)))
            for _verb, name in affs:
                lines.append(('  [F] %s' % name, packed(PLATE_KEY_INK, PLATE_BG)))
    return lines[:max_rows]


def affordance_plate(detected, top, left, width):
    """Render the framed affordance interaction plate beneath the picture-in-picture inset."""
    key_frame = packed(FRAME_INK)
    lines = _build_plate_rows(detected)
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


def docked_vision(scanner, now, width, height):
    """Render the docked stereo eyes PiP and the affordance keypress list beneath it."""
    import numpy as np
    detected = scanner.scan(now)
    seen = now.get('seen') if now else None
    if seen is None:
        vw, vh = sight.VIEW
        thumb = np.full((vh, vw, 3), (15, 22, 28), dtype=np.uint8)
        hulls = []
        for item in detected[:3]:
            k = item['kind']
            ink = sight.INK.get(k, sight.INK['thing'])
            bx = max(2, min(vw - 6, int(vw / 2 + (item['pos'][0] - now.get('where', (0, 0, 0))[0]) * 8)))
            by = max(2, min(vh - 6, int(vh / 2 - (item['pos'][2] - now.get('where', (0, 0, 0))[2]) * 4)))
            box = [(bx, by), (bx + 5, by), (bx + 5, by + 4), (bx, by + 4)]
            hulls.append((k, ink, box))
        seen = (thumb, hulls)

    cells = inset.eyes(seen, width, height)
    if not cells:
        return {}

    max_r = max(r for r, _c in cells)
    min_c = min(c for _r, c in cells)
    max_c = max(c for _r, c in cells)
    box_w = max_c - min_c + 1

    plate_top = max_r + 1
    if plate_top + 5 < height:
        plate = affordance_plate(detected, plate_top, min_c, box_w)
        cells.update(plate)

    return cells

