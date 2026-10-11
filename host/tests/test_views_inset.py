"""The picture in the viewport's corner: its frame, her eyes' labels, the quad's camera on its gimbal."""
import math
import sys

from tools.dev.focus import chosen
from views_kit import Report


def test_her_eyes_picture_is_framed(report):
    """The picture in the viewport's corner (`coaxial.graphics.inset`): the instruments'
    frame whole round it - four corners, its title in the top edge, its note in the bottom one
    -, each hull's kind a label on it, none over another; nothing where the viewport has no
    room; W takes it away and gives it back."""
    import numpy as np
    from coaxial.graphics import callouts, inset
    from machine import sight
    from terminal.views import humanoid_keys, viewpoint
    wide, rows = sight.VIEW
    thumb = np.full((rows, wide, 3), 90, np.uint8)
    box = [(40, 60), (200, 60), (200, 200), (40, 200)]
    hulls = [('wall', sight.INK['wall'], [(0, 0), (319, 0), (319, 239), (0, 239)]),
             ('chair', sight.INK['chair'], box), ('table', sight.INK['table'], box),
             ('lamp', sight.INK['lamp'], [(42, 62), (120, 62), (120, 120), (42, 120)])]
    cells = inset.eyes((thumb, hulls), 120, 40)
    text = {at: chr(code) for at, (code, _key) in cells.items()}
    top, left = min(r for r, _c in cells), min(c for _r, c in cells)
    bottom, right = max(r for r, _c in cells), max(c for _r, c in cells)
    corners = ''.join(text[at] for at in ((top, left), (top, right), (bottom, left),
                                          (bottom, right)))
    edges = [''.join(text[(r, c)] for c in range(left, right + 1)) for r in (top, bottom)]
    sides = {text[(r, c)] for r in range(top + 1, bottom) for c in (left, right)}
    frame = callouts.packed(callouts.FRAME_INK)
    report.check('the instruments\' frame whole round the picture, in their ink',
                 corners == callouts.FRAME_CORNERS and sides == {'│'}
                 and bottom - top == rows // 2 + 1 and right - left == wide + 1
                 and all(cells[at][1] == frame for at in ((top, left), (bottom, right),
                                                          (top + 1, left), (top + 1, right))),
                 '%s, %d by %d cells' % (corners, right - left + 1, bottom - top + 1))
    report.check('its title in the top edge, what she has told in the bottom one',
                 inset.EYES in edges[0] and '%d told' % len(hulls) in edges[1], ' | '.join(edges))
    inside = [''.join(text[(r, c)] if text[(r, c)] != '▀' else ' '
                      for c in range(left + 1, right)) for r in range(top + 1, bottom)]
    words = ' '.join(inside).split()
    report.check('each hull\'s kind a label on the picture, none over another',
                 sorted(words) == sorted(kind for kind, _ink, _hull in hulls), ' '.join(words))
    report.check('nothing where the viewport has no room for it',
                 not inset.eyes((thumb, hulls), wide + 1, 40)
                 and not inset.eyes((thumb, hulls), 120, rows // 2 + 1))
    state = {'eyes': True}
    keys = humanoid_keys.table(viewpoint.KEYS, ('strong',), ('torque',))
    keys['w'](state)
    off = state['eyes']
    keys['W'](state)
    report.check('W takes the picture away and gives it back', off is False and state['eyes'])


def test_its_camera_rides_a_gimbal(report):
    """The carried camera (`machine.gimbal`, `coaxial.graphics.onboard`): its sight comes
    round to the way the frame goes and keeps level whatever the frame does - rolled on its
    side the level frame's, on its back at rest its own -, its tilt within its axis' ends; in
    the picture the horizon one row from edge to edge, the gate flown to next in its ink on the
    course and not off it; C takes the picture away and gives it back."""
    from coaxial.graphics import onboard, scenery, shapes
    from coaxial.graphics.lit import project
    from coaxial.graphics.raster import DOTS_Y
    from machine import gimbal, grounds
    from terminal.views import show_quad
    level = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    rolled = shapes.turn_about('z', 80.0).tolist()
    over = shapes.turn_about('z', 180.0).tolist()

    def flown(turn, vel, aim=gimbal.REST, seconds=3.0):
        for _ in range(int(seconds / 0.01)):
            aim = gimbal.step(aim, turn, vel, 0.01)
        return aim
    along, across = (0.0, 0.0, 12.0), (12.0, 0.0, 0.0)
    flat, side = flown(level, along), flown(rolled, along)
    report.check('rolled 80 degrees on its way the sight is the level frame\'s',
                 math.dist(flat, side) < 1e-3,
                 'pan %+.2f and %+.2f deg, tilt %+.2f and %+.2f' % tuple(
                     math.degrees(a) for a in (flat[0], side[0], flat[1], side[1])))
    turned = flown(level, across, flat)
    report.check('its way turned 90 degrees, the sight comes round to it',
                 abs(math.degrees(turned[0]) - 90.0) < 1.0, '%+.1f deg' % math.degrees(turned[0]))
    kept = flown(over, (0.0, 0.0, 0.0), turned)
    report.check('on its back at rest the pan keeps its own',
                 abs(kept[0] - turned[0]) < 1e-9, '%+.1f deg' % math.degrees(kept[0]))
    up, down = flown(level, (0.0, 30.0, 0.0)), flown(level, (0.0, -30.0, 0.0))
    report.check('straight up and straight down the tilt stands at its axis\' ends',
                 abs(up[1] - gimbal.TILTS[1]) < 1e-3 and abs(down[1] - gimbal.TILTS[0]) < 1e-3,
                 '%+.0f and %+.0f deg' % (math.degrees(up[1]), math.degrees(down[1])))
    eye = gimbal.lens((0.0, 6.0, 0.0), rolled)
    m, centre = onboard.sight(eye, side)
    far = [(600.0 * math.sin(a), 0.0, 600.0 * math.cos(a)) for a in (-0.6, 0.0, 0.6)]
    _sx, sy, _w = project(far, m, onboard.lens(), centre)
    blue = [tone[2] for tone in onboard.air(side[1]).tolist()]
    sky = max(range(1, len(blue)), key=lambda k: blue[k - 1] - blue[k])
    report.check('the horizon one row of the picture from edge to edge, the sky over it',
                 max(sy) - min(sy) < 0.5 and abs(sky - sum(sy) / len(sy) / DOTS_Y) <= 1.0,
                 'far ground at dot rows %s, the air turns to ground at row %d' % (
                     ' '.join('%.1f' % y for y in sy), sky))
    gate = 1
    x, y, z = grounds.GATES[gate][:3]

    def inked(body, ink):
        want = (ink[0] << 16) | (ink[1] << 8) | ink[2]
        return sum(1 for row in body for _glyph, key in row if key & 0xFFFFFF == want)
    before = (x, y, z - 6.0)
    lit = inked(onboard.picture(before, (0.0, 0.0), gate), scenery.INKS['next'])
    none = inked(onboard.picture(before, (0.0, 0.0)), scenery.INKS['next'])
    report.check('the gate flown to next in its ink on the course, none of it off',
                 lit >= 8 and none == 0, '%d cells on, %d off' % (lit, none))
    held = {'carried': True}
    keys = show_quad.hands(held, lambda k: None)
    keys['c']()
    off = held['carried']
    keys['C']()
    report.check('C takes the picture away and gives it back', off is False and held['carried'])
    frame = {'at': (0.0, 6.0, 0.0), 'turn': rolled}
    cells = show_quad.carried(frame, side, gate, True, 110, 38)
    report.check('the picture is docked inside the viewport, its note the gate and how far',
                 cells and all(0 <= r < 38 and 0 <= c < 110 for r, c in cells)
                 and 'GATE 1' in ''.join(chr(cells[k][0]) for k in sorted(cells))
                 and not show_quad.carried(frame, side, gate, True, 30, 38)
                 and not show_quad.carried(frame, side, gate, True, 110, 10),
                 '%d cells' % len(cells))


ROSTER = (test_her_eyes_picture_is_framed, test_its_camera_rides_a_gimbal)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed, %d skipped' % (report.passed, report.failed, report.skipped))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
