"""Screens carbon and a hard shell are drawn through; a body beside the figure, its guards' surface."""
import os
import sys
import tempfile

from test_views_likeness import _figure, _tube
from tools.dev.focus import chosen
from views_kit import Report


def _drawn(lines, ink):
    """The dots of each braille cell of `lines` (`gynoid.render`'s) drawn in `ink` as the fold steps it."""
    from coaxial.graphics import lit
    from machine.rig import ansi
    stepped = tuple(c // lit.INK_STEP * lit.INK_STEP for c in ink)
    return [bin(ord(ch) - 0x2800).count('1') for row in ansi.parse('\n'.join(lines)) for ch, fg, _bg in row
            if fg == stepped and 0x2800 < ord(ch) < 0x2900]


def test_carbon_and_hard_shells_drawn_through_their_screens(report):
    """A carbon fibre's paint (`lit.TWILL`): glossy, its dots through a twill's screen, either hand by the side a
    surface faces. A hard shell (`lit.HARD`), a plate's, every dot. A mark over them alone in its cells
    (`lit.ALONE`)."""
    import numpy as np
    from coaxial.graphics import lit, livery, panels, raster
    half = np.array([lit.KEY]) + (0.0, 0.0, float(np.linalg.norm(lit.KEY)))
    plain, carbon, other, hard = (lit._light(at, np.array([ink]), np.zeros((1, 2)))[0].astype(int) for at, ink in (
        (half, lit.paint((60, 60, 60))), (half, lit.paint((60, 60, 60), twill=True)),
        (half * (-1.0, 1.0, 1.0), lit.paint((60, 60, 60), twill=True)), (half, lit.paint((60, 60, 60), hard=True))))
    tube, turned = (lit._light(at, np.array([livery.TUBE]), np.zeros((1, 2)))[0].astype(int) for at in (half, half * (-1.0, 1.0, 1.0)))
    report.check("carbon fibre gleams under its clear coat and tells its twill's hand by the side it faces, a paint the "
                 "blue noise, a hard shell none; the shell under clothes is carbon; the livery's tube carbon as a "
                 "hard shell - its hand a quarter over -, its rims, its foam and its drives hard shells' paints",
                 carbon[1] > plain[1] + 40 and (plain[3], carbon[3], other[3], hard[3]) == (255, 128, 0, 224)
                 and bool(panels.SHELL & lit.TWILL) and bool(livery.TUBE & lit.HARD) and bool(livery.TUBE & lit.TWILL)
                 and all(bool(ink & lit.HARD) and not ink & lit.TWILL for ink in (
                     livery.FRAME, livery.PAD, livery.FOAM, livery.STEEL, livery.BOLT, livery.SERVO, livery.GEAR))
                 and not livery.VOID & lit.HARD and (tube[3], turned[3]) == (192, 64),
                 '%s a paint, %s carbon, %s faced the other way, %s a tube' % (plain.tolist(), carbon.tolist(), other.tolist(), tube.tolist()))
    high, wide = 6, 8
    depth = np.ones((high * raster.DOTS_Y, wide * raster.DOTS_X))
    y, x = np.mgrid[:depth.shape[0], :depth.shape[1]]

    def folded(shade, hand, props=()):
        """The dots `lit.braille` draws of a raster covered whole in `shade`, its dots' screen `hand`."""
        rgb = np.full(depth.shape + (4,), shade, np.uint8)
        rgb[..., 3] = hand
        lines = lit.braille(depth, rgb, np.zeros(depth.shape), wide, high, colour=False, props=props)
        bits, out = np.array([[max(ord(ch) - 0x2800, 0) for ch in line] for line in lines]), np.zeros(depth.shape, bool)
        for lane in range(raster.DOTS_X):
            for row in range(raster.DOTS_Y):
                out[row::raster.DOTS_Y, lane::raster.DOTS_X] = (bits & raster.BRAILLE_BITS[lane][row]) > 0
        return out
    inner = (slice(1, -1), slice(1, -1))                # a raster's rim is its silhouette, always drawn
    right, left, bright, noise = folded(40, 0), folded(40, 128), folded(255, 0), folded(40, 255)
    report.check("a twill's dots: two of four along a row and a dot aside a row down, either hand; a highlight fills "
                 'its rib; a paint keeps the blue noise',
                 bool((right == ((x + y) % lit.RIB < 2))[inner].all()) and bool((left == ((x - y) % lit.RIB < 2))[inner].all())
                 and bool(bright.all()) and not bool((noise == right)[inner].all()),
                 '%d and %d of %d dots, %d in the noise' % (right[inner].sum(), left[inner].sum(), right[inner].size, noise[inner].sum()))
    report.check('a hard shell is drawn every dot, dark as it may be', bool(folded(40, 224).all()),
                 '%d of %d dots' % (folded(40, 224).sum(), depth.size))
    solid, mirror = folded(40, 64), folded(40, 192)
    report.check("a carbon plate's dots: every dot but its rib's - one of four along a row, a dot aside a row down, "
                 'either hand -, and a highlight fills that',
                 bool((solid == ((x + y) % lit.RIB < lit.RIB - 1))[inner].all())
                 and bool((mirror == ((x - y) % lit.RIB < lit.RIB - 1))[inner].all()) and bool(folded(255, 64).all()),
                 '%d and %d of %d dots' % (solid[inner].sum(), mirror[inner].sum(), solid[inner].size))
    plain3 = lit.braille(depth, np.full(depth.shape + (3,), 40, np.uint8), np.zeros(depth.shape), wide, high, colour=False)
    plain4 = lit.braille(depth, np.full(depth.shape + (4,), (40, 40, 40, 255), np.uint8), np.zeros(depth.shape), wide, high, colour=False)
    report.check('a colour of three a dot, no screen told, is drawn as one of four in the blue noise', plain3 == plain4)
    mark = np.zeros(depth.shape, bool)
    mark[9, 5] = True
    alone, over = (int(folded(255, 255, [(mark, (0, 255, 0), how)])[8:12, 4:6].sum()) for how in (lit.ALONE, True))
    report.check('a mark is alone in its cell: of a lit cell its one dot drawn; a prop over all leaves the cell lit',
                 (alone, over) == (1, 8), '%d dot alone, %d over' % (alone, over))
    share = np.zeros(depth.shape)
    share[4:8, 2:4] = 0.5                              # the cell at row 1, column 1, all its dots half
    rgb = np.full(depth.shape + (4,), 96, np.uint8)
    rgb[..., 3] = 224
    plain, tinted = (lit.braille(depth, rgb, np.zeros(depth.shape), wide, high, props=props) for props in ([], [(share, (232, 232, 232), lit.TINT)]))
    inks = [[fg for _ch, fg, _bg in row] for row in lit.ansi.parse('\n'.join(tinted))]
    base = [[fg for _ch, fg, _bg in row] for row in lit.ansi.parse('\n'.join(plain))]
    dots = [[ch for ch, _fg, _bg in row] for row in lit.ansi.parse('\n'.join(tinted))] == [[ch for ch, _fg, _bg in row] for row in lit.ansi.parse('\n'.join(plain))]
    report.check("a TINT prop lights no dot and draws its cells toward its ink by its mean over them: half of each dot, "
                 "half the way; a cell it covers not, as it was",
                 dots and abs(inks[1][1][0] - (base[1][1][0] + 0.5 * (232 - base[1][1][0]))) <= lit.INK_STEP
                 and inks[1][1][0] > base[1][1][0] and inks[3][5] == base[3][5],
                 '%s tinted, %s plain' % (inks[1][1], base[1][1]))


def _mannequin(path, crown=1.75):
    """A body stood arms down, laid as its maker lays it - its height along z, its face toward -y, a hundredth its
    size -, then stood, thinned and written to `path` (`tools.sim.sewn`): legs to 0.9 m, hips, a waist, a chest with
    a bust, sloping shoulders, a neck and a head; an arm a side hung from 1.4 m out past its hips, clear of its trunk
    under 1.3."""
    import numpy as np
    from coaxial.graphics.shapes import ellipsoid
    from tools.sim import sewn
    made = [_tube((x, 0.0, 0.0), 0.0, 0.9, ((0.04, 0.05), (0.075, 0.085))) for x in (0.085, -0.085)]
    made += [ellipsoid((x, 0.03, 0.07), (0.04, 0.03, 0.11), 0)[:2] for x in (0.085, -0.085)]
    made += [_tube(0.0, 0.84, 1.1, ((0.16, 0.1), (0.1, 0.075))), _tube(0.0, 1.1, 1.4, ((0.1, 0.075), (0.17, 0.085))),
             _tube(0.0, 1.4, 1.46, ((0.17, 0.085), (0.05, 0.05))), _tube(0.0, 1.46, 1.56, (0.045, 0.045)),
             ellipsoid((0.0, 1.65, 0.01), (0.075, 0.1, 0.09), 0)[:2]]
    made += [ellipsoid((x, 1.27, 0.06), (0.06, 0.06, 0.06), 0)[:2] for x in (0.065, -0.065)]
    for side in (1.0, -1.0):
        c, t = _tube(0.0, 0.76, 1.4, ((0.03, 0.03), (0.042, 0.042)))
        made.append((c + np.outer(side * (0.195 + 0.13 * (1.4 - c[:, 1])), (1.0, 0.0, 0.0)), t))
    base = np.cumsum([0] + [len(c) for c, _t in made[:-1]])
    corners, triangles = np.vstack([c for c, _t in made]), np.vstack([t + k for (_c, t), k in zip(made, base)])
    laid = corners @ np.array([[1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, -1.0, 0.0]]) * 0.01
    sewn.wrote(path, [sewn.thinned(sewn.stood(laid, crown), triangles, np.zeros(len(triangles), int), 0.012)])


def test_a_body_beside_the_figure_is_the_guards_surface(report):
    """A mannequin's body beside the figure the robot wears (`form`): its joints found on it as it stands arms down;
    laid on the robot's skeleton it is the surface the robot's guards lie on, and its hands the robot's - nothing
    else of it is drawn, no shell; dressed, its guards are under its clothes."""
    import numpy as np
    from coaxial.graphics import form, gynoid, livery, plated, wavefront
    from machine.body import figure
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, 'models', 'figure', 'figure.obj')
        os.makedirs(os.path.dirname(path))
        _figure(path)
        _mannequin(os.path.join(folder, 'models', 'mannequin', 'formed', 'body.obj'))
        found = form.beside(path) or ''
        v, groups, _at = wavefront.read(found)
        j = form.joints(v, np.vstack([t for _names, t in groups]))
        parts = {name: mesh for name, _parent, *_rest, mesh in form.worn(path)}
        body, worn = gynoid.body(dressed=False, like=path), gynoid.body(like=path)
    shoulder = j['arm'][0]
    report.check('the body beside the figure is found, stood upright, and its joints on it: its legs meet at its '
                 'crotch, an arm and its trunk at its armpit, its shoulder over that on its arm, its bust under it',
                 os.path.basename(found) == 'body.obj' and abs(float(v[:, 1].max()) - 1.75) < 0.01 and abs(j['crotch'] - 0.86) < 0.05
                 and 1.26 < j['armpit'] < 1.4 and j['armpit'] < shoulder[1] < 1.48 and 0.14 < shoulder[0] < 0.24
                 and 1.18 < j['bust'] < 1.34 and int(j['arms'].sum()) > 100,
                 'crotch %.3f, armpit %.3f, shoulder (%.3f, %.3f), bust %.3f' % (
                     j['crotch'], j['armpit'], shoulder[0], shoulder[1], j['bust']))
    sides = ('left_', 'right_')
    drawn = {p[0] for p in body.parts if len(p[5][1])}
    names = [p[0] for p in body.parts]
    hand = body.parts[names.index('form_left_hand')][5][3]
    report.check("of the body its hands alone are drawn, carbon, in the mechanism's hands' place; the robot's own "
                 "shoes, the figure's head and hair, a drive at each joint - no shell of a trunk or a limb",
                 {n for n in drawn if n.startswith(('form_', 'like_', 'shell_'))} - {'like_fall'} == {
                     'form_left_hand', 'form_right_hand', 'like_head', 'like_hair'} | {
                         'shell_' + s + n for s in sides for n in ('foot', 'toes')}
                 and not drawn & set(plated.HANDS) and {'drive_left_elbow', 'drive_right_knee', 'drive_waist'} <= drawn
                 and bool((hand == livery.TUBE).all()), sorted(n for n in drawn if n.startswith(('form_', 'like_'))))
    guard, ink = body.parts[names.index('guard_left_thigh')][5][0], body.parts[names.index('guard_left_thigh')][5][3]
    thigh = parts['form_left_thigh'][0]
    face = guard[ink == livery.VOID]                     # its honeycomb's bars, the face outermost
    level = np.abs(thigh[:, 1] - float(face[:, 1].mean())) < 0.03
    girth, stands = float(np.hypot(thigh[level, 0], thigh[level, 2]).max()), float(np.hypot(face[:, 0], face[:, 2]).max())
    report.check("its guards lie on the body's surface: a thigh's pad stands where the body's thigh is, no further "
                 'in than its girth there and no more than a guard\'s clearance out',
                 girth - 0.01 < stands < girth + 0.04, 'a pad %.3f m off its axis, the thigh %.3f' % (stands, girth))
    far = dict(figure.stand(), spine=40.0, left_knee=90.0, left_elbow=100.0, right_shoulder=-60.0)
    report.check('bent at its joints it is whole', bool(np.isfinite(body.pose(far)[0]).all()))
    dressed = {p[0] for p in worn.parts if len(p[5][1])}
    report.check('dressed, what its clothes gird is not drawn - a thigh\'s guard, its trunk\'s -; a forearm\'s past '
                 'its sleeve and its hands are',
                 not dressed & {'guard_left_thigh', 'guard_right_thigh', 'guard_torso'}
                 and {'guard_left_forearm', 'form_left_hand', 'form_right_hand'} <= dressed,
                 sorted(n for n in dressed if n.startswith(('form_', 'guard_'))))


ROSTER = (test_carbon_and_hard_shells_drawn_through_their_screens, test_a_body_beside_the_figure_is_the_guards_surface)


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
