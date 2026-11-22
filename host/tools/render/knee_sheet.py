"""The knee's mount's sheet: its clamps together, from behind, bent to their stop, apart, as printed.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/knee_sheet.py OUT.png [--ahead M]

The knee as knee_clamped lays it, the thigh's tube `ahead` of the knee's axis (else as the build states it,
`leg_clamped.stated`); each printed piece on its bed face (`printed`).
"""
import argparse
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.render import covered as cv  # noqa: E402
from tools.render import elbow_mount as em  # noqa: E402
from tools.render import knee_clamped as kc  # noqa: E402
from tools.render import knee_stop  # noqa: E402
from tools.render import mounting as m  # noqa: E402

#: The bed's ink.
BED = (70, 74, 84)


def printed(piece, ahead=0.0):
    """One printed piece (0 the stator clamp, 1 the multi-clamp, 2 the collar, 3 the board's guard, 4 its lid) as it
    lies on its bed - the stator clamp and the multi-clamp on their front faces, the guard and its lid on their back
    faces, the collar on its flange - grown up from y = 0, on a plate standing for the bed; a tube's seat, a print
    of its own, not on it."""
    import numpy as np
    _p, _c, still, turning = kc.bodies(ahead=ahead)
    lay = kc.laid()
    bed_t, bed_s, bed_c = lay['bed']
    up = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])     # +x up
    down = np.array([[0.0, 1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])   # -x up
    turn, lift, parts = ((down, lay['saddle'][1], still), (down, bed_s, turning), (down, bed_c, turning),
                         (up, -bed_t, still), (up, -bed_t - cv.IN_M, still))[piece]
    laid_down = [(tri @ turn.T + (0.0, lift, 0.0), c) for k in kc.PIECES[piece] if k in parts for tri, c in parts[k]
                 if tuple(c) != em.PLATE]
    pts = np.concatenate([tri for tri, _c in laid_down])
    lo, hi = pts.min(axis=0) - 0.015, pts.max(axis=0) + 0.015
    plate = np.array([(lo[0], lo[2]), (hi[0], lo[2]), (hi[0], hi[2]), (lo[0], hi[2])])
    return em._rounded(plate, -0.003, 0.0, BED, r=0.001, steps=1) + laid_down


def main(argv=None):
    from PIL import Image, ImageDraw
    from machine.body import drives
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('out')
    parser.add_argument('--ahead', type=float, default=None, help='the bone line ahead of the axis, m (else stated)')
    args = parser.parse_args(argv)
    from tools.render import leg_clamped
    lead = leg_clamped.stated() if args.ahead is None else args.ahead
    limit, why = knee_stop._stopped(round(1e3 * lead))
    lay = kc.laid()
    px, views = 700, []
    parts, cables, _s, turning = kc.bodies(ahead=lead)
    centre, half = kc._framed(parts)
    views.append(('together', m.draw(parts + cables, px=px, half=half, centre=centre)))
    views.append(('from behind', m.draw(parts + cables, yaw=-50.0, pitch=18.0, px=px, half=half, centre=centre)))
    near = [t for k in ('cage', 'pitch drive', 'roll drive', 'collar', 'collar screws', 'pitch band')
            for t in turning[k]]
    centre, half = kc._framed(near, pad=1.2)
    views.append(('the cage round the inverters, its phase slots', m.draw(near, yaw=15.0, pitch=10.0, px=px, half=half,
                                                                           centre=centre)))
    parts, cables, _s, _t = kc.bodies(flex=limit, ahead=lead)
    centre, half = kc._framed(parts)
    views.append(('bent to its stop %.0f deg (travel %.0f)' % (limit, drives.TRAVEL['knee'][1]),
                  m.draw(parts + cables, yaw=-70.0, pitch=18.0, px=px, half=half, centre=centre)))
    parts, _c, _s, _t = kc.bodies(True, ahead=lead)
    centre, half = kc._framed(parts)
    views.append(('apart: the pitch drive in through the cage, the roll drive in on its collar, screwed on',
                  m.draw(parts, yaw=60.0, pitch=22.0, px=px, half=half, centre=centre)))
    for piece, name in enumerate(('stator clamp as printed, front face on the bed',
                                  'multi-clamp as printed, front face on the bed',
                                  'collar as printed, its flange down', "the board's guard as printed, its face down",
                                  "the guard's lid as printed, flat")):
        parts = printed(piece, lead)
        centre, half = kc._framed(parts)
        views.append((name, m.draw(parts, yaw=30.0, pitch=32.0, px=px, half=half, centre=centre)))
    sheet = Image.new('RGB', (px * 4, 3 * (px + 30) + 30), (18, 18, 18))
    pen = ImageDraw.Draw(sheet)
    pen.text((12, 8), "knee, round 8: the thigh's tube from the hip's axis, %.0f mm ahead of the knee's, the ankle's"
             " drives back to back %.1f mm under the knee's, their inverters in a slotted black cage"
             % (1e3 * lead, -1e3 * lay['pitch'][1][0]),
             fill=(235, 235, 235))
    for i, (name, img) in enumerate(views):
        x, y = px * (i % 4), 30 + (px + 30) * (i // 4)
        sheet.paste(img, (x, y + 30))
        pen.text((x + 12, y + 10), name, fill=(200, 200, 200))
    sheet.save(args.out)
    print(args.out, 'ahead %.0f mm, stop %.1f deg, set by %s' % (1e3 * lead, limit, why))
    print('looked at:', ', '.join('%d mm %.1f deg' % (k, v[0]) for k, v in sorted(knee_stop._STOPS.items())))
    print('drives moved down %.1f mm, bone x %.1f mm, beds x %s mm' % (
        1e3 * lay['drop'], 1e3 * lay['bone'], ' / '.join('%.1f' % (1e3 * b) for b in lay['bed'])))
    print('band walls mm:', ', '.join('%s %.1f' % (k, 1e3 * v) for k, v in lay['walls'].items()))
    return 0


if __name__ == '__main__':
    sys.exit(main())
