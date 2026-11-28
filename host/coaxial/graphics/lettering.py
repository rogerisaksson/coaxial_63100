"""Small lettering on the robot's guards: a drive's type, its joint in katakana, on a header; up close.

A label is words drawn in a face of the host's - a Eurostile where it keeps one (`faced`),
else Pillow's own; katakana in a face of the host's that holds it, else left out - a cap
CAP_M high: the runs of its ink, a raster's row at a time (`lettered`), laid across the
header of a limb's guard and on the back's plate (`guards`) from an anchor on it, a triangle of
its rim. A run's
end is told by that triangle - along its two edges and off it - and carried by it as the
body is posed (`marks.layers`): read only where its joint's drive fills the view, and short of
that its box filtered to its tone - as much of it as its ink covers. Lettered across a body's
every limb in caps a limb wide, a name was smeared over it; a word laid on its side down a
limb, a cap 8 dots high, read as a box; read from 7 dots a cap, medium braille text was
clutter over the guards. The faces are the host's, never the tree's.

    told = lettering.laid(parts, spans)     # (tri, a, b (n, 3) each, way, size, kind, tone (n,)) or None
"""
import math
import os

from coaxial.graphics import guards
from coaxial.graphics.livery import FRAME
from coaxial.graphics.panels import LIMBS
from machine.body import drives
from machine.body.guards import CAP_M

#: Where a system keeps its faces, and a face's file by a word in its name: a Eurostile's, a
#: bold cut first, and one that holds katakana.
FOLDERS = ('%LOCALAPPDATA%/Microsoft/Windows/Fonts', '%WINDIR%/Fonts', '~/.fonts', '~/.local/share/fonts',
           '/usr/share/fonts', '/usr/local/share/fonts', '~/Library/Fonts', '/Library/Fonts')
FACES = {'latin': ('eurostile',), 'kana': ('yugothb', 'meiryob', 'msgothic', 'notosanscjk', 'notosansjp', 'ipag', 'takao')}
#: A cap CAP_M high and RASTER rows of it, as tall as LIKE in its face - read where a drive fills
#: the view, a cap stands 30 to 65 dots high, a row a dot at the most: at 12 rows a glyph stood
#: in stripes -; a label LIFT_M off its header, its words GAP of a cap apart.
RASTER, LIFT_M, GAP = 64, 0.002, 0.5
LIKE = {'latin': 'H', 'kana': '\u30ed'}
#: A guard's label by the end of its segment's name: the joint its drive turns and the joint in
#: katakana - kata, hiji, koshi, hiza, senaka. Across its header stands the drive's type, and the
#: katakana where the header holds both; where it holds not the type, the type alone, smaller -
#: drawn only closer. Told down the thighs alone, a label faced away from the page's own view,
#: from behind: the back's plate bears its spine's.
LABELS = {'upper_arm': ('shoulder', '\u30ab\u30bf'), 'forearm': ('elbow', '\u30d2\u30b8'),
          'thigh': ('hip', '\u30b3\u30b7'), 'shank': ('knee', '\u30d2\u30b6'), 'torso': ('spine', '\u30bb\u30ca\u30ab')}
#: An anchor lies within NEAR_M of its header's middle: a guard in a sleeve has no header left.
#: Of two as near it is the one further out, FACE times as much as it is nearer: a guard's
#: rim lies on its face and under it.
NEAR_M, FACE = 0.03, 4.0

_KEPT = {}


def faced(kind='latin'):
    """The file of a face this host keeps for `kind` ('latin' a Eurostile, 'kana' one that holds
    katakana), None where it keeps none or COAXIAL_FACE says 'none': the first by name, a bold
    cut before the rest, under FOLDERS."""
    if os.environ.get('COAXIAL_FACE') == 'none':
        return None
    if kind not in _KEPT:
        got = [os.path.join(where, file) for folder in FOLDERS
               for where, _dirs, files in os.walk(os.path.expanduser(os.path.expandvars(folder)))
               for file in files if file.lower().endswith(('.ttf', '.otf', '.ttc'))
               and any(word in file.lower().replace(' ', '') for word in FACES[kind])]
        _KEPT[kind] = min(got, default=None, key=lambda path: (
            'bold' not in os.path.basename(path).lower(), os.path.basename(path).lower()))
    return _KEPT[kind]


def lettered(word, kind='latin'):
    """(runs (n, 3), columns): `word` drawn in the host's face for `kind` (`faced`) - Pillow's
    own where it keeps no Eurostile -, LIKE's glyph RASTER rows high: each run of its ink along
    a row its (row, first column, the column past its last), LIKE's top row 0; None without
    Pillow or a face it can draw in, or for katakana with no face that holds it."""
    from coaxial.model.blocks import numpy as np
    face = faced(kind)
    if (word, kind, face) not in _KEPT:
        try:
            from PIL import Image, ImageDraw, ImageFont
        except ImportError:
            return None
        if face is None and kind != 'latin':
            return None

        def cut(size):
            return ImageFont.truetype(face, size) if face else ImageFont.load_default(size)
        try:
            _left, top, _right, bottom = cut(100).getbbox(LIKE[kind])
        except OSError:                        # a Pillow built with no FreeType: no face to draw in
            return None
        font = cut(100.0 * RASTER / (bottom - top))
        _left, top, _right, bottom = font.getbbox(LIKE[kind])
        left, _top, right, _bottom = font.getbbox(word)
        image = Image.new('L', (int(math.ceil(right - left)) + 1, int(math.ceil(bottom - top)) + RASTER))
        ImageDraw.Draw(image).text((-left, RASTER // 2 - top), word, 255, font)
        step = np.diff(np.pad(np.asarray(image) > 127, ((0, 0), (1, 1))).astype(np.int8), axis=1)
        rows, first = np.nonzero(step == 1)
        _KEPT[(word, kind, face)] = (np.stack([rows - RASTER // 2, first, np.nonzero(step == -1)[1]], 1).astype(float),
                                     image.width)
    return _KEPT[(word, kind, face)]


def laid(parts, spans):
    """(tri (n, 3), a (n, 3), b (n, 3), way (n,), size (n,), kind (n,), tone (n,)): the lettering
    of the guards among `parts` [(name, parent, joints, offset, rest, mesh)] ('guard_' and a limb
    or the trunk: `guards.laid`'s), `spans` each part's corners among all laid end to end. A run
    goes from `a` to `b`, each told by its anchor `tri` - three corners of the guard's header's
    rim: along its first edge, along its second, off it out of the guard, `way` 1 where its
    corners' turn faces out and -1 where in -, `size` its joint's drive's diameter, m, what reads
    it (`marks.read`), `kind` 0 a glyph's ink and 1 a row of its label's box - a raster's row a
    row, across the label -, `tone` the share of its label's box its ink covers. Across a guard's
    header (LABELS) stand its joint's drive's type (`drives.of`) and the joint's name in
    katakana, read from without; None where nothing is lettered."""
    from coaxial.model.blocks import numpy as np
    rows = []
    for (name, _parent, _joints, _offset, _rest, (corners, triangles, _uv, materials)), (start, _end) in zip(parts, spans):
        kind, _, seg = name.partition('_')
        side, _, limb = seg.partition('_')
        side, limb = (side, limb) if limb in guards.LIMB else ('', seg)
        spec = next((guard for guard in guards.LIMB.get(limb, guards.TRUNK) if guard[5] in ('pad', 'plate')), None)
        if kind != 'guard' or limb not in LABELS or spec is None or not len(triangles):
            continue
        joint, kana = LABELS[limb]
        joint, (high, low) = (side + '_' + joint, (-spec[1] * LIMBS[limb][0], -spec[2] * LIMBS[limb][0])) if side else (
            joint, (spec[2], spec[1]))
        centre = high - guards.HEADER_M / 2.0 if spec[5] == 'pad' else (high + low) / 2.0
        # A label's anchor: the triangle of its header's rim nearest the header's middle, on its
        # face, bare of clothes.
        mid, x = corners[triangles].mean(axis=1), -1.0 if side == 'right' else 1.0
        off = np.abs((np.arctan2(mid[:, 2], x * mid[:, 0]) - math.radians(spec[3]) + math.pi) % (2.0 * math.pi) - math.pi)
        reach = np.hypot(mid[:, 0], mid[:, 2])
        far = np.where((materials[triangles] == FRAME).all(axis=1),
                       np.hypot(mid[:, 1] - centre, reach * off), np.inf)
        near = far < NEAR_M                    # of those, the one furthest out: told among all, a cup's beat a back's
        k = int(np.argmin(np.where(near, far + FACE * (reach[near].max(initial=0.0) - reach), np.inf)))
        words = [(drives.of(joint)[0], 'latin')] * (not drives.passive(joint)) + [(kana, 'kana')]
        told = [got for word, face in words for got in [lettered(word, face)] if got and len(got[0])]
        if not near.any() or not told:
            continue
        # What the header holds - a plate between its rivets: the katakana gives way first; the type
        # alone is drawn smaller.
        room = 2.0 * (math.radians(spec[4]) * float(reach[k]) - (
            guards.BAND_M if spec[5] == 'pad' else (high - low) / 2.0 + guards.RIVET_R))

        def span(got):
            return (sum(columns for _runs, columns in got) + GAP * RASTER * (len(got) - 1)) * CAP_M / RASTER
        while len(told) > 1 and span(told) > room:
            told = told[:-1]
        shrink = min(1.0, room / span(told))
        scale = shrink * CAP_M / RASTER
        p, e1, e2 = corners[triangles[k, 0]], *(corners[triangles[k, i]] - corners[triangles[k, 0]] for i in (1, 2))
        out = np.cross(e1, e2) / np.linalg.norm(np.cross(e1, e2))
        way = 1.0 if out @ (mid[k] * (1.0, 0.0, 1.0)) > 0.0 else -1.0
        out = out * way
        up = np.array([0.0, 1.0, 0.0]) - out[1] * out                  # up the limb, along the header there
        up = up / np.linalg.norm(up)
        across, at = np.cross(up, out), mid[k] + LIFT_M * out
        half = (sum(wide for _runs, wide in told) + GAP * RASTER * (len(told) - 1)) / 2.0
        col, ends, kinds, inked = -half, ([], []), [], 0.0
        for runs, wide in told:                # a word beside the last, read from without
            for c in (0, 1):
                ends[c].append(at + scale * (np.outer(col + runs[:, 1 + c], across) + np.outer(RASTER / 2.0 - runs[:, 0] - 0.5, up)))
            col, kinds, inked = col + wide + GAP * RASTER, kinds + [0] * len(runs), inked + float((runs[:, 2] - runs[:, 1]).sum())
        box = RASTER / 2.0 - np.arange(RASTER) - 0.5                  # its box: a row of it each row of a glyph
        for c, x in ((0, -half), (1, half)):
            ends[c].append(at + scale * (np.outer(np.full(RASTER, x), across) + np.outer(box, up)))
        kinds += [1] * RASTER
        a, b = (np.linalg.solve(np.stack([e1, e2, out], 1), (np.vstack(ends[c]) - p).T).T for c in (0, 1))
        rows.append((np.tile(triangles[k] + start, (len(a), 1)), a, b, np.full(len(a), way),
                     np.full(len(a), drives.of(joint)[1].diameter), np.array(kinds),
                     np.full(len(a), inked / (2.0 * half * RASTER))))
    return tuple(np.concatenate([row[i] for row in rows]) for i in range(7)) if rows else None
