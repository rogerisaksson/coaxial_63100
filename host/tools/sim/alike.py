#!/usr/bin/env python3
"""How alike the robot bare of its clothes is to the pictures of robots by the tree: image analysis.

    python tools/sim/alike.py                  # each picture's measures, the robot's, and the verdict
    python tools/sim/alike.py --png OUT.png    # the same laid side by side: each as seen, and as its classes

A picture of a robot is cut out of its ground - white, a colour, a ramp - and its banner; the
robot is drawn bare of its clothes, its arms out (T-posed), from the front as its page draws
it - dots on black, a dot of it that is not lit black as the ground: its darkest. The robot's
head is a figure's likeness and no part of what is held to the pictures. Both are measured alike
(`measured`): the shares of what is seen that are dark, grey, light and of each hue - its
brightest a picture's own white -, how much of its outline is air (struts and guards with
gaps between, where a body in a suit has none), how much of it is edge (cut-outs, struts, a
pad's cells) and how thick its parts are (plates on a frame, where a frame alone is sticks).
The robot is alike (`alike`) where its palette lies no further from a picture's than the
pictures' lie apart, it wears no hue no picture has, and its air, its edge and its girth are
a picture's least by SHARE. The pictures are never the tree's: where none lie beside it the
robot is held to what they measured (PICTURED, by their names).
"""
import argparse
import glob
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from coaxial.model.blocks import numpy as np

#: A picture is read HIGH rows tall. The ground is what a flood from the picture's border reaches, a dot to
#: its neighbour no more than RAMP apart in any channel, from the border's dots within SEED of the border's
#: middle colour - a white ground, a blue one, a grey one ramping to white; told by whiteness alone (0.97) a
#: blue ground was 70 % of a robot -, and of a ground with a hue every dot as saturated as SAT within NEAR deg
#: of that hue: the blue a robot's edges blended and its arms closed off was 8 % of one. A banner is the rows
#: at the picture's foot more than half within SEED of their own middle colour and not the ground's.
HIGH, RAMP, SEED, NEAR = 400, 0.035, 0.08, 25.0
#: The classes: a dot with a hue is SAT saturated and no darker than DIM of the white; one
#: without is dark under DARK of the white and light over LIGHT. The hues, deg, each from its
#: first to its second round the wheel.
SAT, DIM, DARK, LIGHT = 0.3, 0.2, 0.4, 0.75
HUES = (('red', 345.0, 20.0), ('yellow', 20.0, 70.0), ('green', 70.0, 170.0), ('blue', 170.0, 270.0), ('magenta', 270.0, 345.0))
CLASSES = ('dark', 'grey', 'light') + tuple(name for name, _a, _b in HUES)
#: Air is what closing an outline CLOSE of its height adds to it; edge is told in blocks BLOCK
#: rows a side, two neighbours STEP of the white apart.
CLOSE, BLOCK, STEP = 0.02, 4, 0.2
#: A hue is a picture's where it is HAS of what is seen in one - a gynoid's magenta is an
#: eighth of it; at a fiftieth a highlight's blue was a hue, and a robot with its drives in
#: cyan passed -; the robot may wear FOREIGN of hues that are none's, and has SHARE of the
#: pictures' least air, edge and girth at the least: dots on black a third as fine as a
#: photograph hold no more, and its guards close no limb.
HAS, FOREIGN, SHARE = 0.05, 0.04, 0.5
#: The view the robot is measured in, cells, and its ink's classes' colours as a picture's map.
CELLS = (220, 100)
INKS = {'dark': (40, 40, 48), 'grey': (128, 128, 136), 'light': (236, 236, 240), 'red': (230, 40, 40), 'yellow': (240, 220, 40),
        'green': (60, 220, 60), 'blue': (60, 110, 240), 'magenta': (240, 60, 200)}
#: What of the robot is its head: a part by its name's start or its segment.
HEAD = ('like_head', 'like_hair', 'like_fall', 'shell_head', 'shell_jaw', 'shell_nose', 'shell_lips', 'head')


def pictures():
    """The pictures of robots beside the tree: COAXIAL_ROBOTS' folder's, else those under
    robot/ beside the figure the robot wears (`likeness.FOUND`); none where there are none."""
    from coaxial.graphics import likeness
    folder = os.environ.get('COAXIAL_ROBOTS') or (
        os.path.join(os.path.dirname(os.path.dirname(likeness.FOUND)), 'robot') if likeness.FOUND else '')
    return sorted(path for kind in ('*.jpg', '*.jpeg', '*.png') for path in glob.glob(os.path.join(folder, kind)))


def _hue(rgb):
    """Each colour's hue, deg round the wheel."""
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    return np.degrees(np.arctan2(np.sqrt(3.0) * (g - b), 2.0 * r - g - b)) % 360.0


def _sat(rgb):
    """Each colour's saturation: its channels' spread of its brightest."""
    return (rgb.max(axis=-1) - rgb.min(axis=-1)) / np.maximum(rgb.max(axis=-1), 1e-6)


def _ground(rgb):
    """What of `rgb` (rows, columns, 3) is its ground: flooded from its border (RAMP, SEED); what lies
    within half SEED of the flooded ground's middle colour - ground an arm closed off -; and of a ground
    with a hue, every dot as saturated within NEAR deg of it."""
    border = np.concatenate([rgb[0], rgb[-1], rgb[:, 0], rgb[:, -1]])
    middle = np.median(border, axis=0)
    rim = np.zeros(rgb.shape[:2], bool)
    rim[0], rim[-1], rim[:, 0], rim[:, -1] = True, True, True, True
    out = rim & (np.abs(rgb - middle).max(axis=2) < SEED)
    down, across = (np.abs(np.diff(rgb, axis=k)).max(axis=2) < RAMP for k in (0, 1))
    while True:
        was = out.copy()
        out[1:] |= out[:-1] & down
        out[:-1] |= out[1:] & down
        out[:, 1:] |= out[:, :-1] & across
        out[:, :-1] |= out[:, 1:] & across
        if (out == was).all():
            middle = np.median(rgb[out], axis=0)
            out |= np.abs(rgb - middle).max(axis=2) < SEED / 2.0
            if _sat(middle) >= SAT:
                off = np.abs((_hue(rgb) - _hue(middle) + 180.0) % 360.0 - 180.0)
                out |= (off < NEAR) & (_sat(rgb) >= SAT)
            return out


def cut_out(path):
    """(colours (rows, columns, 3) 0..1, what is the robot (rows, columns)) of the picture at
    `path`, HIGH rows tall: its banner cut off, its ground not the robot (`_ground`)."""
    from PIL import Image
    with Image.open(path) as picture:
        rgb = np.asarray(picture.convert('RGB'), float) / 255.0
    border = np.median(np.concatenate([rgb[0], rgb[:, 0], rgb[:, -1]]), axis=0)
    row_middle = np.median(rgb, axis=1)
    banner = ((np.abs(rgb - row_middle[:, None]).max(axis=2) < SEED).mean(axis=1) > 0.5) & (
        np.abs(row_middle - border).max(axis=1) > SEED)
    rgb = rgb[:len(rgb) - int(np.argmin(banner[::-1]))] if banner[-1] else rgb
    small = Image.fromarray((rgb * 255.0).astype(np.uint8)).resize((max(1, round(rgb.shape[1] * HIGH / len(rgb))), HIGH), Image.Resampling.BOX)
    rgb = np.asarray(small, float) / 255.0
    return rgb, ~_ground(rgb)


def t_posed():
    """The robot's joints with its arms out: its stand, its upper arms' rest spread to level."""
    from machine.body import figure
    rest = next(abs(seg[4]) for seg in figure.SEGMENTS if seg[0] == 'left_upper_arm')
    return dict(figure.stand(), arms_out=90.0 - rest)


def drawn(like=None, see=None, cells=CELLS, yaw=0.0):
    """(colours, what is the robot, what of that is its body) of the robot bare of its clothes,
    T-posed, as `gynoid.render` draws it from `yaw` in `cells` without a card: a dot a row and
    a column - a lit dot in its cell's ink, one the robot covers and does not light black -,
    the robot's own dots, none of its floor's; its body what of it is not its head (HEAD)."""
    from coaxial.graphics import gynoid, lit as lights
    from coaxial.graphics.raster import BRAILLE_BITS, DOTS_X, DOTS_Y
    from machine.rig import ansi
    lines = gynoid.render(t_posed(), cells[0], cells[1], yaw=yaw, pitch=0.0, dressed=False, like=like, see=see)
    who = gynoid.body(False, see, like if not see else None, not see)
    pos, norm = who.pose(t_posed(), dense=True)
    looked = gynoid._looked(cells[0], cells[1], yaw, 0.0, 1.0, (0.0, 0.0))
    whole = lights.splat((pos, norm, who.dense[3], who.dense[2]), *looked)[0]
    head = np.zeros(len(pos), bool)
    for (name, parent, *_rest), (lo, hi) in zip(who.parts, who.dense_spans):
        head[lo:hi] = name.startswith(HEAD) or parent == 'head'
    body = lights.splat((pos[~head], norm[~head], who.dense[3][~head], who.dense[2][~head]), *looked)[0]
    rgb = np.zeros((cells[1] * DOTS_Y, cells[0] * DOTS_X, 3))
    for r, row in enumerate(ansi.parse('\n'.join(lines))):
        for c, (ch, fg, _bg) in enumerate(row[:cells[0]]):
            bits = ord(ch) - 0x2800
            if fg and 0 < bits < 256:
                for lane in range(DOTS_X):
                    for k in range(DOTS_Y):
                        if bits & BRAILLE_BITS[lane][k]:
                            rgb[r * DOTS_Y + k, c * DOTS_X + lane] = np.array(fg) / 255.0
    covered = whole > 0.0
    return rgb * covered[..., None], covered, (body > 0.0) & (body >= whole)


def _grown(mask, r):
    """`mask` with every dot within `r` rows and columns of one of its own."""
    out = np.pad(mask, r).astype(int).cumsum(axis=0).cumsum(axis=1)
    out = np.pad(out, ((1, 0), (1, 0)))
    k = 2 * r + 1
    return (out[k:, k:] - out[:-k, k:] - out[k:, :-k] + out[:-k, :-k]) > 0


def _closed(mask, r):
    """`mask` closed `r` rows and columns: grown, and what is no dot's own grown back."""
    return ~_grown(~_grown(mask, r), r)


def _runs(mask):
    """Each dot's run down its column: how many of `mask`'s stand unbroken with it, 0 off it."""
    count, down = np.zeros(mask.shape[1], int), np.zeros(mask.shape, int)
    for i, row in enumerate(mask):
        count = np.where(row, count + 1, 0)
        down[i] = count
    for i in range(len(mask) - 2, -1, -1):
        down[i] = np.where(mask[i] & mask[i + 1], down[i + 1], down[i])
    return down


def classed(rgb, mask):
    """Each dot's class's place in CLASSES, -1 where it is none of `mask`'s: by its hue where it
    has one, else by how bright it is of the picture's own white - its dots' 95th of a hundred."""
    high, low = rgb.max(axis=2), rgb.min(axis=2)
    white = float(np.percentile(high[mask], 95.0)) if mask.any() else 1.0
    bright, sat = np.minimum(high / max(white, 1e-6), 1.0), (high - low) / np.maximum(high, 1e-6)
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    hue = np.degrees(np.arctan2(np.sqrt(3.0) * (g - b), 2.0 * r - g - b)) % 360.0
    out = np.where(bright < DARK, 0, np.where(bright < LIGHT, 1, 2))
    for k, (_name, first, last) in enumerate(HUES, 3):
        within = (hue >= first) & (hue < last) if first < last else (hue >= first) | (hue < last)
        out = np.where((sat >= SAT) & (bright >= DIM) & within, k, out)
    return np.where(mask, out, -1), bright


def measured(rgb, mask, body=None):
    """{'palette': {class: share}, 'air': share, 'edge': share, 'girth': share} of what `mask`
    tells of the colours `rgb` (rows, columns, 3), `body` what of it is held (all of it
    unless told): the shares of its body's dots in each of CLASSES; of its outline closed
    CLOSE of its height, what the closing added - gaps between its parts; of its blocks BLOCK
    a side, those a neighbour is STEP of the white brighter or darker than; and how thick its
    body's parts are, of its height - a dot's the lesser of its run along its row and down
    its column, their mean."""
    body = mask if body is None else body
    kind, bright = classed(rgb, body)
    palette = {name: float((kind == k).sum()) / max(int(body.sum()), 1) for k, name in enumerate(CLASSES)}
    whole = _closed(mask, 1)
    rows = np.flatnonzero(whole.any(axis=1))
    tall = int(rows[-1] - rows[0] + 1) if len(rows) else 1
    shut = _closed(whole, max(1, round(CLOSE * tall)))
    high, wide = (n - n % BLOCK for n in mask.shape)

    def blocks(told):
        return told[:high, :wide].reshape(high // BLOCK, BLOCK, wide // BLOCK, BLOCK).mean(axis=(1, 3))
    there, tone = blocks(body * 1.0), blocks(bright * body) / np.maximum(blocks(body * 1.0), 1e-9)
    full = there >= 0.5
    pairs = [(full[:, 1:] & full[:, :-1], np.abs(tone[:, 1:] - tone[:, :-1])), (full[1:] & full[:-1], np.abs(tone[1:] - tone[:-1]))]
    edge = sum(float((gap[both] > STEP).sum()) for both, gap in pairs) / max(sum(int(both.sum()) for both, _gap in pairs), 1)
    solid = _closed(body, 1)
    thick = np.minimum(_runs(solid), _runs(solid.T).T)[solid]
    return {'palette': palette, 'air': float(shut.sum() - whole.sum()) / max(int(shut.sum()), 1), 'edge': edge,
            'girth': float(thick.mean()) / tall if len(thick) else 0.0}


def apart(a, b):
    """How far two palettes lie apart: half the sum of their classes' shares' differences."""
    return 0.5 * sum(abs(a[name] - b[name]) for name in CLASSES)


#: What the pictures beside the tree measured (`measured`, tools/sim/alike.py), by their names: white plates
#: on a black frame walking (robot 1), gunmetal plates and magenta actuators (robot 2), white plates over a
#: black frame on a blue ground from behind (robot_3) and seated (robot_4), white plates and red actuators
#: (robot_5 from behind, robot_6 walking), a black suit and white plates from behind (robot_7). A host that
#: keeps the pictures holds those it names to these (test_views_bared).
PICTURED = {
    'robot 1.png': {'palette': {'dark': 0.434, 'grey': 0.321, 'light': 0.243, 'red': 0.0, 'yellow': 0.0, 'green': 0.0,
                                'blue': 0.001, 'magenta': 0.0}, 'air': 0.122, 'edge': 0.281, 'girth': 0.107},
    'robot 2.png': {'palette': {'dark': 0.5, 'grey': 0.21, 'light': 0.115, 'red': 0.0, 'yellow': 0.0, 'green': 0.006,
                                'blue': 0.028, 'magenta': 0.142}, 'air': 0.089, 'edge': 0.188, 'girth': 0.15},
    'robot_3.png': {'palette': {'dark': 0.539, 'grey': 0.228, 'light': 0.233, 'red': 0.0, 'yellow': 0.0, 'green': 0.0,
                                'blue': 0.0, 'magenta': 0.0}, 'air': 0.144, 'edge': 0.221, 'girth': 0.097},
    'robot_4.png': {'palette': {'dark': 0.487, 'grey': 0.257, 'light': 0.221, 'red': 0.001, 'yellow': 0.001, 'green': 0.001,
                                'blue': 0.031, 'magenta': 0.0}, 'air': 0.105, 'edge': 0.213, 'girth': 0.179},
    'robot_5.png': {'palette': {'dark': 0.328, 'grey': 0.321, 'light': 0.232, 'red': 0.109, 'yellow': 0.009, 'green': 0.0,
                                'blue': 0.0, 'magenta': 0.0}, 'air': 0.08, 'edge': 0.237, 'girth': 0.11},
    'robot_6.png': {'palette': {'dark': 0.347, 'grey': 0.312, 'light': 0.206, 'red': 0.129, 'yellow': 0.007, 'green': 0.0,
                                'blue': 0.0, 'magenta': 0.0}, 'air': 0.134, 'edge': 0.247, 'girth': 0.089},
    'robot_7.png': {'palette': {'dark': 0.637, 'grey': 0.173, 'light': 0.178, 'red': 0.0, 'yellow': 0.0, 'green': 0.0,
                                'blue': 0.011, 'magenta': 0.0}, 'air': 0.116, 'edge': 0.209, 'girth': 0.148}}


def alike(robot, told=None):
    """[what is not alike]: the robot's measures (`measured`) held to the pictures' `told`
    (PICTURED unless told) - its palette no further from the nearest than they lie apart, no
    more than FOREIGN of it in hues none of them has, its air, its edge and its girth SHARE of
    their least at the least. Empty where it is alike."""
    told = list(told or PICTURED.values())
    spread = max((apart(a['palette'], b['palette']) for a in told for b in told), default=0.0)
    near = min(apart(robot['palette'], p['palette']) for p in told)
    foreign = sum(robot['palette'][name] for name in CLASSES[3:] if all(p['palette'][name] < HAS for p in told))
    out = []
    if near > spread:
        out.append('its palette lies %.2f from the nearest picture, the pictures %.2f apart' % (near, spread))
    if foreign > FOREIGN:
        out.append("%.2f of it is in hues no picture has" % foreign)
    for what in ('air', 'edge', 'girth'):
        least = min(p[what] for p in told)
        if robot[what] < SHARE * least:
            out.append('its %s is %.3f, under %.2f of the least picture\'s %.3f' % (what, robot[what], SHARE, least))
    return out


def _said(name, got):
    shares = ' '.join('%s %.2f' % (key, got['palette'][key]) for key in CLASSES if got['palette'][key] >= 0.005)
    return '  %-12s air %.3f  edge %.3f  girth %.3f  %s' % (name, got['air'], got['edge'], got['girth'], shares)


def _sheet(shown, path):
    """The comparison as a picture at `path`: each of `shown` [(name, colours, mask)] as seen
    and, under that, as its classes' inks."""
    from PIL import Image
    tall = max(rgb.shape[0] for _n, rgb, _m in shown)
    sheet = Image.new('RGB', (sum(rgb.shape[1] + 8 for _n, rgb, _m in shown), 2 * tall + 8), (24, 24, 28))
    at = 0
    for _name, rgb, mask in shown:
        kind, _bright = classed(rgb, mask)
        inks = np.array([INKS[name] for name in CLASSES] + [(0, 0, 0)], np.uint8)[kind]
        sheet.paste(Image.fromarray((rgb * 255.0).astype(np.uint8)), (at, 0))
        sheet.paste(Image.fromarray(inks), (at, tall + 8))
        at += rgb.shape[1] + 8
    sheet.save(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--png', help='the comparison laid side by side as a picture here')
    parser.add_argument('--see', choices=('mechanism',), help='the layer under its shell, not its shell')
    args = parser.parse_args(argv)
    from coaxial.graphics import likeness
    shown = [(os.path.basename(path), *cut_out(path)) for path in pictures()]
    told = [measured(rgb, mask) for _name, rgb, mask in shown]
    for (name, _rgb, _mask), got in zip(shown, told):
        print(_said(name, got))
    rgb, mask, body = drawn(likeness.FOUND, args.see)
    robot = measured(rgb, mask, body)
    print(_said('the robot', robot))
    faults = alike(robot, told or None) if told or PICTURED else ['no picture beside the tree, none measured']
    print('  alike' if not faults else '  NOT alike: ' + '; '.join(faults))
    if args.png:
        across = np.flatnonzero(mask.any(axis=0))
        cut = slice(max(int(across[0]) - 8, 0), int(across[-1]) + 9)
        _sheet(shown + [('the robot', rgb[:, cut], body[:, cut])], args.png)
    return 1 if faults else 0


if __name__ == '__main__':
    sys.exit(main())
