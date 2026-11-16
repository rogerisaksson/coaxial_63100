"""Small lettering on the robot's guards: read where its drive fills the view, else filtered to a tone."""
import os
import sys

from test_views_marks import _drawn
from tools.dev.focus import chosen
from views_kit import Report


def _ends(body, pose, told, mine):
    """The ends (2n, 3), world, of the runs `mine` of a body's lettering `told` (`lettering.laid`'s) with the body at
    `pose`: each by its anchor's three corners as they are posed - along its two edges and off it."""
    import numpy as np
    pos, _norm = body.pose(pose)
    tri, first, last, turn = (each[mine] for each in told[:4])
    p, e1, e2 = pos[tri[:, 0]], pos[tri[:, 1]] - pos[tri[:, 0]], pos[tri[:, 2]] - pos[tri[:, 0]]
    off = np.cross(e1, e2)
    off = off / np.linalg.norm(off, axis=1, keepdims=True) * turn[:, None]
    return np.vstack([p + c[:, :1] * e1 + c[:, 1:2] * e2 + c[:, 2:] * off for c in (first, last)])


def test_lettering_across_a_guards_header_read_up_close(report):
    """A guard is lettered across its header, and the back's plate (`lettering`): its joint's drive's type and the
    joint in katakana where the header holds both, a cap `lettering.CAP_M` high - smaller where it holds not the type -;
    carried by the guard as the limb is posed; read (`marks.layers`) only where its joint's drive spans the view, and
    short of that its box filtered to the share its ink covers (`lit.TINT`). With no face of the host's the glyphs are
    Pillow's own and katakana is left out."""
    import numpy as np
    from coaxial.graphics import guards, gynoid, lettering, lit, livery, marks
    from machine.body import drives, figure
    was = os.environ.get('COAXIAL_FACE')
    os.environ['COAXIAL_FACE'] = 'none'        # no face of the host's: every host letters alike
    try:
        runs, wide = lettering.lettered('8') or (np.zeros((0, 3)), 0)
        over = lettering.RASTER / 16.0                 # a round glyph overshoots a flat cap by a few of a hundred
        report.check("a glyph is the runs of its ink, a raster's row each: a cap `lettering.RASTER` rows high - a round "
                     "one past it by a sixteenth at the most -, a run within the glyph's width; katakana with no face that "
                     "holds it is none",
                     len(runs) >= lettering.RASTER and -over <= float(runs[:, 0].min()) <= over
                     and lettering.RASTER - 1.0 - over <= float(runs[:, 0].max()) <= lettering.RASTER + over
                     and bool((runs[:, 2] > runs[:, 1]).all()) and float(runs[:, 2].max()) <= wide
                     and lettering.lettered('\u30d2', 'kana') is None,
                     '%d runs over rows %d..%d, %d columns' % (len(runs), runs[:, 0].min(initial=0), runs[:, 0].max(initial=0), wide))
        body = gynoid.body(dressed=False, plates=True)
        told = lettering.laid(body.parts, body.spans) or tuple(np.zeros((0, k)) for k in (3, 3, 3, 1, 1, 1, 1))
        tri, size, kind, tone = told[0].astype(int), told[4], told[5], told[6]
        owner = np.searchsorted([start for start, _end in body.spans], tri[:, 0], side='right') - 1
        names = {body.parts[k][0] for k in np.unique(owner)}
        anchors = {int(body.materials[k]) for k in np.unique(tri)}
        thighs = np.isin(owner, [k for k, p in enumerate(body.parts) if p[0].endswith('_thigh') and p[0].startswith('guard_')])
        report.check("every limb's guard is lettered, a side each, and the back's plate - seen from behind as the page "
                     "looks -, every run by three corners of its own guard's rim; read by its joint's drive's size - a "
                     "thigh's its hip's -; its box a row a raster's row, its tone the share its ink covers",
                     names == {'guard_' + s + n for s in ('left_', 'right_') for n in ('upper_arm', 'forearm', 'thigh', 'shank')}
                     | {'guard_torso'} and anchors == {livery.FRAME} and bool((tri.max(initial=0) < len(body.corners)))
                     and bool(np.allclose(size[thighs], drives.of('left_hip')[1].diameter)) and bool((size > 0.05).all())
                     and int((kind == 1).sum()) == len(names) * lettering.RASTER and bool(((tone > 0.1) & (tone < 0.7)).all())
                     and int((kind == 0).sum()) > 100,
                     '%d runs on %d guards; drives %.3f to %.3f m; tones %.2f to %.2f' % (
                         int((kind == 0).sum()), len(names), size.min(initial=0.0), size.max(initial=0.0),
                         tone.min(initial=0.0), tone.max(initial=0.0)))
        stand = figure.stand()
        thigh = (owner == [p[0] for p in body.parts].index('guard_left_thigh')) & (kind == 0)
        at = _ends(body, stand, told, thigh)
        tall, across = float(np.ptp(at[:, 1])), float(np.hypot(*np.ptp(at[:, [0, 2]], axis=0)))
        guard = body.pose(stand)[0][slice(*body.spans[[p[0] for p in body.parts].index('guard_left_thigh')])]
        report.check("a thigh's label is its hip's drive's type, across its guard's header: a cap tall, its glyphs "
                     "side by side, within HEADER_M of the guard's head",
                     0.7 * lettering.CAP_M < tall < 1.4 * lettering.CAP_M and across > tall
                     and float(guard[:, 1].max()) - guards.HEADER_M - 0.01 < float(at[:, 1].min())
                     and float(at[:, 1].max()) < float(guard[:, 1].max()) + 0.005,
                     "'%s': %.3f m tall, %.3f across" % (drives.of('left_hip')[0], tall, across))
        moved = _ends(body, dict(stand, left_hip=-90.0), told, thigh).mean(axis=0) - at.mean(axis=0)
        report.check('a label is carried by its guard: with the hip bent 90 deg the thigh wears it where the thigh is',
                     float(np.linalg.norm(moved)) > 0.03, 'moved %.3f m' % float(np.linalg.norm(moved)))
        frames = body._frames(stand)
        dense, laid = body.pose(stand, dense=True, _frames=frames), body.pose(stand, _frames=frames)

        middle, yaw = at.mean(axis=0), 36.0           # the thigh's label, looked at from where it faces
        view = np.asarray(gynoid.view(yaw, 0.0)).reshape(3, 3)
        pan = tuple(float((middle - np.asarray(gynoid.CENTRE)) @ view[k]) for k in (0, 1))
        cam = gynoid._looked(110, 50, yaw, 0.0, 1.0, pan)[1]
        fills = min(cam['width'], cam['height']) / (cam['scale'] / cam['distance'] * drives.of('left_hip')[1].diameter)

        def lettered(zoom):
            """(glyphs' dots, the most tone a dot of a box tints by): the thigh's label at `zoom`, centred, by `marks.layers`."""
            m, fine, centre = gynoid._looked(110, 50, yaw, 0.0, zoom, pan if zoom > 1.0 else (0.0, 0.0))
            depth, _rgb = lit.splat((*dense, body.dense[3], body.dense[2]), m, fine, centre)
            got = marks.layers(body, laid, m, fine, centre, depth)
            return (sum(int(dots.sum()) for dots, _hue, how in got if how == lit.ALONE),
                    max((float(dots.max()) for dots, _hue, how in got if how == lit.TINT), default=0.0))
        far, mid, short, close = (lettered(zoom) for zoom in (1.0, 8.0, fills / 1.1, fills * 1.1))
        cells = len(_drawn(gynoid.render(stand, 110, 50, yaw=yaw, pitch=0.0, zoom=fills * 1.1, pan=pan, dressed=False), marks.LETTER))
        report.check("a label is read only where its joint's drive spans the view's shorter side: from afar, at a zoom "
                     "of 8 and just short of that its box tinted by its tone, no glyph; just past it its glyphs, in its "
                     "ink", far[0] == mid[0] == short[0] == 0 and min(far[1], mid[1], short[1]) > 0.1 and close[0] > 100
                     and cells > 20, 'fills at zoom %.1f: %s from afar, %s at 8, %s short, %s past; %d cells' % (
                         fills, far, mid, short, close, cells))
    finally:
        os.environ.pop('COAXIAL_FACE', None)
        os.environ.update({} if was is None else {'COAXIAL_FACE': was})


ROSTER = (test_lettering_across_a_guards_header_read_up_close,)


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
