"""The robot's likeness: a figure's head and clothes on it, garments sewn elsewhere laid on it."""
import os
import sys
import tempfile

from tools.dev.focus import chosen
from views_kit import Report


def _tube(at, low, high, radii, axis='y', cloth=False):
    """(corners, triangles) of a tube about `at` from `low` to `high` along `axis`, `radii` (rx, rz) there - or
    ((rx, rz) at `low`, (rx, rz) at `high`) -; `cloth` open at its ends."""
    from coaxial.graphics.shapes import finer, lain, loft
    first, last = radii if isinstance(radii[0], tuple) else (radii, radii)
    c, t, _u, _m = lain(loft(finer([(low, *first), (high, *last)], min(low, high), max(low, high), 0.005), 0), axis)
    return (c[:-2] + at, t[(t < len(c) - 2).all(axis=1)]) if cloth else (c + at, t)


def _figure(path):
    """A T-posed figure written to `path` as an OBJ, m: jeans, a sleeved shirt to 1.42 m up, bare shoulders that slope
    from 0.15 aside there to its neck, 0.06 aside at 1.46, bare hands, feet and head, hair."""
    from coaxial.graphics.shapes import ellipsoid
    legs = [_tube((x, 0.0, 0.0), 0.12, 0.86, (0.07, 0.07)) for x in (0.09, -0.09)]
    feet = [_tube((x, 0.0, 0.0), 0.02, 0.12, (0.04, 0.04)) for x in (0.09, -0.09)]
    sleeves = [_tube((0.0, 1.38, 0.0), s * 0.15, s * 0.50, (0.04, 0.04), 'x') for s in (1.0, -1.0)]
    hands = [_tube((0.0, 1.38, 0.0), s * 0.50, s * 0.80, (0.035, 0.035), 'x') for s in (1.0, -1.0)]
    shoulders, neck = _tube(0.0, 1.42, 1.46, ((0.15, 0.10), (0.06, 0.04))), _tube(0.0, 1.46, 1.53, (0.04, 0.04))
    head, hair = (ellipsoid((0.0, 1.62, z), (0.08, 0.11, 0.09), 0)[:2] for z in (0.0, -0.03))
    groups = (('Jeans', legs + [_tube(0.0, 0.86, 1.02, (0.17, 0.10))]), ('Shirt', sleeves + [_tube(0.0, 1.0, 1.42, (0.15, 0.10))]),
              ('Body Std_Skin', feet + hands + [shoulders, neck, head]), ('Hair', [hair]))
    lines, base = [], 1
    for name, meshes in groups:
        lines.append('g ' + name)
        for corners, triangles in meshes:
            lines += ['v %.5f %.5f %.5f' % tuple(p) for p in corners] + ['f %d %d %d' % tuple(t + base) for t in triangles]
            base += len(corners)
    with open(path, 'w', encoding='utf-8', newline='') as out:
        out.write('\n'.join(lines) + '\n')


def _garments(folder):
    """A tee and trousers as a cloth simulator sews them, on a larger body 50 mm ahead, its arms hung 45 deg,
    thinned (`tools.sim.sewn`) under worn/ in `folder`: a piece a mesh, sewn where 20 corners meet; a thread on
    the tee shares 8. The trousers' seat is closed between their legs, open at their waist."""
    import numpy as np
    from tools.sim import sewn

    def sewn_to(piece, to, shared):
        return np.vstack([piece[0], to[0][:shared]]), piece[1]
    trunk, seat = _tube(0.0, 1.10, 1.60, (0.15, 0.11), cloth=True), _tube(0.0, 0.94, 1.12, (0.185, 0.11))
    hips = (seat[0], seat[1][(seat[1] < len(seat[0]) - 1).all(axis=1)])
    trunk[0][:, [0, 2]] *= 1.0 + 0.04 * np.sin(16.0 * np.arctan2(trunk[0][:, :1], trunk[0][:, 2:]))    # its folds
    c, t = _tube(0.0, 0.0, 0.09, (0.065, 0.065), 'x', cloth=True)
    hung = c @ np.array([[0.7071, 0.7071, 0.0], [-0.7071, 0.7071, 0.0], [0.0, 0.0, 1.0]]).T
    pieces = [trunk, hips, sewn_to(_tube((0.0, 1.3, 0.11), 0.0, 0.02, (0.002, 0.002), cloth=True), trunk, 8)]
    pieces += [sewn_to((hung * (s, 1.0, 1.0) + (s * 0.15, 1.50, 0.0), t), trunk, 20) for s in (1.0, -1.0)]
    pieces += [sewn_to(_tube((x, 0.0, 0.0), 0.16, 0.94, (0.075, 0.075), cloth=True), hips, 20) for x in (0.11, -0.11)]
    made = [sewn.thinned(c + (0.0, 0.0, 0.05), t, p, sewn.PITCH_M) for c, t, p in sewn.garments(pieces)]
    sewn.wrote(os.path.join(folder, 'worn', 'sewn.obj'), made)
    return pieces, made


def test_a_figures_likeness_on_the_robots_skeleton(report):
    """A T-posed figure read from a file (`likeness`): its joints found on it, its head and its hair worn on the robot
    bare of its clothes (`plated`) and, dressed, its clothes - their cloth alone, on the segments they ride -; what
    they gird not drawn; nothing moved by the bend at rest, bent whole."""
    import numpy as np
    from coaxial.graphics import gynoid, likeness, lit, livery, plated, wavefront
    from coaxial.graphics.shapes import moved
    from machine.body import figure
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, 'figure.obj')
        _figure(path)
        v, groups, _at = wavefront.read(path)
        j = likeness.joints(v[np.unique(np.concatenate([t.ravel() for names, t in groups if 'hair' not in names]))])
        body, robot = gynoid.body(like=path), gynoid.body(dressed=False, like=path)
    report.check('its crotch, armpit, ankle and arms found where it was built',
                 abs(j['crotch'] - 0.86) < 0.02 and abs(j['pit'] - 0.15) < 0.03 and 0.03 < j['ankle'] < 0.13
                 and abs(j['arm'][0] - 1.38) < 0.01 and abs(j['tip'] - 0.80) < 0.01,
                 {k: round(float(j[k]), 3) for k in ('crotch', 'pit', 'ankle', 'tip')})
    worn = {p[0][5:]: i for i, p in enumerate(body.parts) if p[0].startswith('like_')}
    frames = body._frames({})
    still, _n = body.pose({}, root=((0.0, 0.0, 0.0), figure.IDENT), _frames=frames)
    rigid, _n = moved(body.corners, body.normals, body.spans, frames)
    sleeve, shoulder = still[slice(*body.spans[worn['left_upper_arm']])], frames[worn['left_upper_arm']][1]
    cloth = [body.parts[i][5][3] for name, i in worn.items() if 'like_' + name not in plated.WORN]
    sides = ('left_', 'right_')
    report.check("dressed, the robot plated wears the figure's head, hair and hands and its clothes' cloth alone - its "
                 "trunk's, its hips', its legs', its sleeves' -; a sleeve hangs under its shoulder, out past its hips",
                 set(worn) - {'fall'} == {'head', 'hair', 'torso', 'pelvis'} | {s + n for s in sides for n in (
                     'upper_arm', 'thigh', 'shank', 'hand')}
                 and body.parts[worn['hair']][1] == 'head' and all(bool(((ink & lit.WOVEN) > 0).all()) for ink in cloth)
                 and not any(p[0].startswith('cloth_') for p in body.parts)
                 and sleeve[:, 1].min() < shoulder[1] - 0.05 and sleeve[:, 0].max() > figure.HIP_HALF + 0.05,
                 '%d parts of the figure, a sleeve to %.2f m under its shoulder' % (len(worn), shoulder[1] - sleeve[:, 1].min()))
    skin = {p[0]: int((p[5][3] == lit.SKIN).sum()) for p in body.parts if len(p[5][3])}
    drawn = {p[0] for p in body.parts if len(p[5][1])}
    hand = body.parts[worn['left_hand']][5][3]
    report.check("its face alone skin; past its sleeves its arms the robot's own - a forearm's guard, its mechanism -, "
                 "its hands the figure's in a tube's carbon, the bones of its own given way",
                 skin['like_head'] > 100 and sum(skin.values()) == skin['like_head']
                 and {'guard_left_forearm', 'guard_right_forearm', 'like_left_hand', 'like_right_hand'} <= drawn
                 and not drawn & set(plated.HANDS) and float((hand == livery.TUBE).mean()) > 0.5
                 and not any(n.startswith('like_') and n.endswith(('forearm', 'neck')) for n in drawn),
                 'skin corners: %s; %d of %d corners of a hand carbon' % (
                     {k: n for k, n in skin.items() if n}, (hand == livery.TUBE).sum(), len(hand)))
    drives = {n[6:] for n in drawn if n.startswith('drive_')}
    under = int(((body.materials & lit.UNDER) > 0).sum())
    # This figure's collar stands over the robot's neck's drives: under it from every side, they are not drawn.
    report.check('its drives seen where it is bare - its elbows - and none drawn under its clothes: a knee, a hip, its '
                 "spine; nor a guard a garment girds - a thigh's, a shin's, the trunk's -, and what is left across a "
                 "garment's end is a layer under it",
                 {'left_elbow', 'right_elbow'} <= drives and not drives & {'right_knee', 'left_hip', 'spine', 'waist'}
                 and not drawn & {'guard_torso', 'guard_left_thigh', 'guard_right_thigh', 'guard_left_shank'}
                 and under > 0, '%s; %d corners a layer under' % (sorted(drives), under))
    far = dict(figure.stand(), spine=60.0, left_hip=-110.0, left_knee=140.0, left_shoulder=150.0, left_elbow=120.0)
    bent, _n = body.pose(far)
    report.check('at rest its bends move no corner; bent far, every corner whole and within its reach',
                 float(np.abs(still - rigid).max()) < 1e-9 and bool(np.isfinite(bent).all())
                 and float(np.linalg.norm(bent - bent.mean(0), axis=1).max()) < 1.3 and len(body.bends) >= 9,
                 '%.1e m at rest, %d bends' % (float(np.abs(still - rigid).max()), len(body.bends)))
    plates = {p[0] for p in robot.parts}
    report.check("bare of its clothes (`plated`), the robot wears the figure's head, hair and hands on its mechanism, "
                 'its guards on it, and nothing else of the figure; no head as built',
                 {'like_head', 'like_hair', 'like_left_hand'} <= plates and {n for n in plates if n.startswith('like_')} <= set(
                     plated.WORN) and 'guard_torso' in plates
                 and not plates & {'shell_head', 'shell_jaw', 'shell_nose', 'shell_left_eye'},
                 sorted(n for n in plates if n.startswith('like_')))
    was = os.environ.get('COAXIAL_LIKENESS')
    os.environ['COAXIAL_LIKENESS'] = 'none'
    try:
        report.check("COAXIAL_LIKENESS 'none': no likeness, the robot as built", likeness.found() is None)
        os.environ.pop('COAXIAL_LIKENESS')
        with tempfile.TemporaryDirectory() as folder:
            # Beside a repo: figures 1 and 10 posed - a column, no arms out -, figure 2 T-posed.
            for number, posed in ((1, True), (2, False), (10, True)):
                path = os.path.join(folder, 'stunts', '3d_models', 'female_%d' % number, 'figure.obj')
                os.makedirs(os.path.dirname(path))
                _figure(path)
                if posed:
                    with open(path, encoding='utf-8') as file:
                        lines = [line for line in file if not line.startswith('v ') or abs(float(line.split()[1])) < 0.2]
                    with open(path, 'w', encoding='utf-8', newline='') as file:
                        file.write(''.join(lines))
            got = likeness.found(os.path.join(folder, 'repo'))
        report.check("beside the repo the first figure that stands T-posed is found, its folders in their numbers' "
                     'order: a posed one is a reference, none to wear',
                     got is not None and os.path.basename(os.path.dirname(got)) == 'female_2', got)
    finally:
        os.environ.pop('COAXIAL_LIKENESS', None)
        os.environ.update({} if was is None else {'COAXIAL_LIKENESS': was})


def test_sewn_garments_laid_on_a_figure(report):
    """Garments a cloth simulator sewed on another body, thinned (`tools.sim.sewn`) and laid on a figure
    (`garment.dressed`): hem on hem, clear of what they lie on, on its shoulders - no plate at its collar's
    height -, sleeves along the arms, their folds shaded; cloth matt (`lit.WOVEN`)."""
    import numpy as np
    from coaxial.graphics import fall, garment, likeness, lit, wavefront
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, 'figure.obj')
        _figure(path)
        pieces, made = _garments(folder)
        v, groups, _at = wavefront.read(path)
        sewn = likeness._sewn(path)
    part = {names.split()[0]: v[np.unique(t)] for names, t in groups}
    j = likeness.joints(np.vstack([part[n] for n in ('jeans', 'shirt', 'body')]))
    worn = {names: (c, shade) for names, c, _t, shade in garment.dressed(sewn, part['shirt'], part['jeans'], part['body'], j)}
    report.check('7 pieces: a tee of 3 - its sleeves a piece each - and trousers sewn, a thread no garment; thinned to '
                 'fewer corners, a cell each, and read back as they were',
                 len(made) == 2 and [len(t) for _c, t, _p in sewn] == [len(t) for _c, t, _p in made]
                 and [len(np.unique(p)) for _c, _t, p in sewn] == [3, 3]
                 and sum(len(c) for c, _t, _p in made) < 0.8 * sum(len(c) for c, _t in pieces),
                 '%d corners of %d' % (sum(len(c) for c, _t, _p in made), sum(len(c) for c, _t in pieces)))
    tee, jeans, hem = worn['tee'][0], worn['jeans'][0], float(part['shirt'][:, 1].min())
    shade, leg = np.asarray(worn['tee'][1], float), np.abs(jeans[np.abs(jeans[:, 1] - 0.5) < 0.02, 0])
    report.check("the trousers on the figure's: hem on hem, waist on waist, a leg EASE_M about its leg",
                 abs(jeans[:, 1].min() - 0.12) < 0.02 and abs(jeans[:, 1].max() - 1.02) < 0.02
                 and abs(leg.max() - 0.16 - fall.EASE_M) < 0.01 and abs(leg.min() - 0.02 + fall.EASE_M) < 0.01,
                 '%.3f..%.3f m up, a leg %.3f..%.3f m aside' % (jeans[:, 1].min(), jeans[:, 1].max(), leg.min(), leg.max()))
    reach = j['pit'] + garment.SLEEVE * (j['elbow'] - j['pit'])
    over, under = fall.extent(tee, hem + 0.01), fall.extent(jeans, hem + 0.01)
    report.check("the tee: its hem on what the figure wore's, its collar where the neck leaves the shoulders, its "
                 'sleeves SLEEVE of the upper arms, clear of the trousers under it',
                 abs(tee[:, 1].min() - hem) < 0.02 and abs(tee[:, 1].max() - garment.neck(part['body'], j)[0]) < 0.03
                 and abs(np.abs(tee[:, 0]).max() - reach) < 0.04
                 and over is not None and under is not None and over[0] > under[0] and over[1] > under[1],
                 'hem %.3f, top %.3f, sleeves to %.3f of %.3f' % (tee[:, 1].min(), tee[:, 1].max(), np.abs(tee[:, 0]).max(), reach))
    # The figure's shoulders fall from 1.46 m up at its neck, 0.06 aside, to 1.42 at 0.15: over their
    # tops' outer ends a tee laid by rows stood level at its collar's height, a plate 15-30 mm off them.
    over = tee[(np.abs(tee[:, 0]) > 0.10) & (np.abs(tee[:, 0]) < 0.14) & (np.abs(tee[:, 2]) < 0.03) & (tee[:, 1] > 1.40)]
    lift = over[:, 1] - 1.42 - (0.15 - np.abs(over[:, 0])) / 0.09 * 0.04
    report.check('over its armpits the tee lies on its shoulders, no plate at its collar\'s height: none of it over '
                 'their tops stands 15 mm off them',
                 len(over) >= 4 and float(lift.max()) < 0.015, '%d corners, %.0f..%.0f mm off the shoulders' % (
                     len(over), 1e3 * float(lift.min(initial=0.0)), 1e3 * float(lift.max(initial=0.0))))
    top = max(sewn, key=lambda g: g[0][:, 1].max())[0]
    low = top[:, 1] < 1.4
    fold = np.sin(16.0 * np.arctan2(top[low, 0], top[low, 2] - 0.05))
    valley, ridge = float(shade[low][fold < -0.5].mean()), float(shade[low][fold > 0.5].mean())
    report.check('its folds shade it: a valley darker than a ridge, each within SHADES',
                 valley < ridge - 0.05 and garment.SHADES[0] <= shade.min() and shade.max() <= garment.SHADES[1],
                 'a valley %.2f, a ridge %.2f' % (valley, ridge))
    at = np.array([lit.KEY]) + (0.0, 0.0, float(np.linalg.norm(lit.KEY)))
    gloss, matt = (lit._light(at, np.array([lit.paint((200, 40, 40), woven)]), np.zeros((1, 2)))[0].astype(int)
                   for woven in (False, True))
    report.check('cloth is matt: woven, a paint wears no highlight where one gleams',
                 gloss[1] > matt[1] + 40 and matt[1] * 4 < matt[0], '%s gleaming, %s woven' % (gloss.tolist(), matt.tolist()))


ROSTER = (test_a_figures_likeness_on_the_robots_skeleton, test_sewn_garments_laid_on_a_figure)


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
