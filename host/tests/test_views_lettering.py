"""Small lettering on the robot's guards: a drive's type across a guard's header, read up close alone."""
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
    tri, first, last, turn, _least = (each[mine] for each in told)
    p, e1, e2 = pos[tri[:, 0]], pos[tri[:, 1]] - pos[tri[:, 0]], pos[tri[:, 2]] - pos[tri[:, 0]]
    off = np.cross(e1, e2)
    off = off / np.linalg.norm(off, axis=1, keepdims=True) * turn[:, None]
    return np.vstack([p + c[:, :1] * e1 + c[:, 1:2] * e2 + c[:, 2:] * off for c in (first, last)])


def test_lettering_across_a_guards_header_read_up_close(report):
    """A guard is lettered across its header, and the back's plate (`lettering`): its joint's drive's type and the
    joint in katakana where the header holds both, a cap `lettering.CAP_M` high - smaller where it holds not the type -;
    carried by the guard as the limb is posed; drawn (`marks.layers`) from the scale a cap is legible at, none of it
    from afar. With no face of the host's the glyphs are Pillow's own and katakana is left out."""
    import numpy as np
    from coaxial.graphics import guards, gynoid, lettering, lit, livery, marks
    from machine.body import drives, figure
    was = os.environ.get('COAXIAL_FACE')
    os.environ['COAXIAL_FACE'] = 'none'        # no face of the host's: every host letters alike
    try:
        runs, wide = lettering.lettered('8') or (np.zeros((0, 3)), 0)
        report.check("a glyph is the runs of its ink, a raster's row each: a cap `lettering.RASTER` rows high, a run "
                     "within the glyph's width; katakana with no face that holds it is none",
                     len(runs) >= lettering.RASTER and -1.0 <= float(runs[:, 0].min()) <= 1.0
                     and lettering.RASTER - 2.0 <= float(runs[:, 0].max()) <= lettering.RASTER
                     and bool((runs[:, 2] > runs[:, 1]).all()) and float(runs[:, 2].max()) <= wide
                     and lettering.lettered('\u30d2', 'kana') is None,
                     '%d runs over rows %d..%d, %d columns' % (len(runs), runs[:, 0].min(initial=0), runs[:, 0].max(initial=0), wide))
        body = gynoid.body(dressed=False, plates=True)
        told = lettering.laid(body.parts, body.spans) or tuple(np.zeros((0, k)) for k in (3, 3, 3, 1, 1))
        tri, least = told[0].astype(int), told[4]
        owner = np.searchsorted([start for start, _end in body.spans], tri[:, 0], side='right') - 1
        names = {body.parts[k][0] for k in np.unique(owner)}
        anchors = {int(body.materials[k]) for k in np.unique(tri)}
        thighs = np.isin(owner, [k for k, p in enumerate(body.parts) if p[0].endswith('_thigh') and p[0].startswith('guard_')])
        report.check("every limb's guard is lettered, a side each, and the back's plate - seen from behind as the page "
                     "looks -, every run by three corners of its own guard's rim, from the scale a cap is legible at: a "
                     "thigh's at a cap CAP_M high, none before that",
                     names == {'guard_' + s + n for s in ('left_', 'right_') for n in ('upper_arm', 'forearm', 'thigh', 'shank')}
                     | {'guard_torso'} and anchors == {livery.FRAME} and bool((tri.max(initial=0) < len(body.corners)))
                     and bool(np.allclose(least[thighs], lettering.LEGIBLE / lettering.CAP_M))
                     and bool((least >= lettering.LEGIBLE / lettering.CAP_M - 1e-6).all()) and len(tri) > 100,
                     '%d runs on %d guards, from %.0f to %.0f dots a metre' % (
                         len(tri), len(names), least.min(initial=0.0), least.max(initial=0.0)))
        stand = figure.stand()
        thigh = owner == [p[0] for p in body.parts].index('guard_left_thigh')
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

        def lettered(yaw, zoom=1.0, pan=0.0):
            """The lettering's dots of the robot standing, from `yaw`: `marks.layers` on the dots' own depth."""
            m, fine, centre = gynoid._looked(110, 50, yaw, 8.0, zoom, (0.0, pan))
            depth, _rgb = lit.splat((*dense, body.dense[3], body.dense[2]), m, fine, centre)
            return sum(int(dots.sum()) for dots, hue, _how in marks.layers(body, laid, m, fine, centre, depth) if hue == marks.LETTER)
        far = sum(lettered(yaw) for yaw in (0.0, 45.0, 315.0))
        near = max(lettered(yaw, 6.0, pan) for yaw in (0.0, 45.0, 315.0) for pan in (-0.05, -0.15))
        cells = max(len(_drawn(gynoid.render(stand, 110, 50, yaw=yaw, zoom=6.0, pan=(0.0, pan), dressed=False), marks.LETTER))
                    for yaw in (0.0, 45.0, 315.0) for pan in (-0.05, -0.15))
        report.check('from afar a guard wears no lettering; up close a label is drawn, cells of it in its ink',
                     far == 0 and near > 20 and cells > 5, '%d dots from afar, %d up close in %d cells' % (far, near, cells))
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
