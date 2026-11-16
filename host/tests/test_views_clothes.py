"""The robot's clothes: nothing of it stands out of them, they do not tear, a hem keeps its place on a shoe."""
import math
import os
import sys
import tempfile

from test_views_likeness import _figure, _garments
from tools.dev.focus import chosen
from views_kit import Report


def test_nothing_stands_out_of_its_clothes(report):
    """What a garment girds is not drawn, and what is left across a garment's end is drawn a layer further from the
    eye (`lit.UNDER`, a card's shader the same): cloth within LAYER_M behind it hides it. Cloth bends at the joints it
    spans (`drape`): a garment on one side whole, one across both sides by both sides' bends between their joints.
    Posed, nothing of the robot stands out of its clothes (`tools.sim.shown.stood_out`: told where it stands, from no
    side) and they do not tear (`shown.torn`); a hem at the ankle keeps its place on the shoe."""
    import numpy as np
    from coaxial.graphics import gynoid, likeness, lit, plated
    from machine.body import figure
    from tools.sim import shown
    m, fine, centre = gynoid._looked(40, 20, 0.0, 0.0, 1.0, (0.0, 0.0))
    eye = np.asarray(m).reshape(3, 3)[2]
    inks = np.array([lit.paint((200, 0, 0), True), lit.paint((0, 200, 0)) | lit.UNDER])

    def seen(ahead):
        """The paint a dot shows of cloth and of what it covers `ahead` m before it."""
        at = np.asarray(centre) + np.outer((0.0, ahead), eye)
        _depth, rgb = lit.splat((at, np.tile(eye, (2, 1)), inks, np.zeros((2, 2))), m, fine, centre)
        rgb = rgb[..., :3].reshape(-1, 3)
        return rgb[rgb.astype(int).sum(axis=1).argmax()].astype(int)
    near, far = seen(lit.LAYER_M / 3.0), seen(2.0 * lit.LAYER_M)
    report.check('what a garment covers, 10 mm before its cloth, is not seen; 60 mm before it - a knee out of a hem - it is',
                 near[0] > near[1] and far[1] > far[0], '%s, %s' % (near.tolist(), far.tolist()))
    # A sleeve: cloth 50 mm about a bone that runs down from its joint, 0.3 m of it.
    turn, down = np.arange(48) * (math.pi / 24.0), np.arange(0.0, 0.2999, 0.005)
    sleeve = np.stack([0.05 * np.tile(np.cos(turn), len(down)), -np.repeat(down, 48), 0.05 * np.tile(np.sin(turn), len(down))], 1)
    told = shown.girded(sleeve, np.zeros(3), np.array([0.0, -1.0, 0.0]))
    within, past, edge, beyond = told(np.array([[0.04, -0.15, 0.0], [0.07, -0.15, 0.0], [0.07, -0.35, 0.0], [0.07, -0.295, 0.0],
                                                [0.0, -0.15, 0.058], [0.04, -0.31, 0.0]]))
    report.check("about a sleeve's bone: a point within its cloth reaches short of it, one outside it 20 mm past "
                 "it; one beyond its end, or within a slab of that, is held to no cloth; 8 mm past it is past OUT_M; "
                 "what stands in its last slab or the one past it stands by its end, inside its reach or out of it",
                 within.tolist() == [True, True, False, False, True, False] and past[0] < 0.0 and abs(past[1] - 0.02) < 0.002
                 and shown.OUT_M < past[4] < 0.01 and edge.tolist() == [False, False, False, True, False, True]
                 and beyond[5] < 0.0 < beyond[3],
                 '%s, past by %s mm; by its end %s' % (within.tolist(), np.round(1e3 * past[[0, 1, 4]], 1).tolist(), edge.tolist()))
    poses = [(name, angles) for name, angles in shown.tried() if name in (
        'stand', 'squat', 'sit', 'bow', 'stride', 'shoulder +135', 'hip -90', 'knee +90', 'swing', 'raise', 'reach')]
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, 'figure.obj')
        _figure(path)
        _garments(folder)
        body = gynoid.body(like=path)
    names, starts = [p[0] for p in body.parts], [lo for lo, _hi in body.spans]

    def bent(cloth_name, goal):
        """[(the corners, their weights or None)] of each row of the body's bends (`drape.table`) of `cloth_name` into `goal`."""
        out = []
        for row in body.bends:
            full, mid, blend = row[3][0], row[3][3], row[3][7]
            idx = np.concatenate([full, mid])
            if len(idx) and names[int(np.searchsorted(starts, idx[0], side='right') - 1)] == cloth_name and names[row[0]] == goal:
                out.append((idx, None if blend is None else np.concatenate(blend[:2])))
        return out
    knee = bent('like_left_shank', 'left_thigh')
    seat = bent('like_pelvis', 'left_thigh') + bent('like_pelvis', 'right_thigh')
    x = body.corners[:, 0]
    shared = np.intersect1d(seat[0][0], seat[1][0]) if len(seat) == 2 else np.zeros(0, int)
    both = [dict(zip(idx.tolist(), weight.tolist())) for idx, weight in seat] if len(seat) == 2 else [{}, {}]
    sums = np.array([both[0][k] + both[1][k] for k in shared.tolist()])
    middle = shared[np.abs(x[shared]) < 0.002]
    report.check("a shin's cloth bends at its knee whole, both halves of it; a seat across both hips takes each "
                 "thigh's bend by how near its hip a corner stands - half of each on its middle, the two weights one",
                 len(knee) == 1 and knee[0][1] is None and bool((x[knee[0][0]] < -0.01).any()) and bool((x[knee[0][0]] > 0.01).any())
                 and len(shared) > 0 and bool(np.allclose(sums, 1.0)) and len(middle) > 0
                 and all(abs(both[0][k] - 0.5) < 0.02 for k in middle.tolist()),
                 '%d corners of a knee bent; %d shared by the seat, %d on its middle' % (
                     len(knee[0][0]) if knee else 0, len(shared), len(middle)))
    out = {name: got for name, angles in poses for got in [shown.stood_out(body, angles)] if got}
    tears = {name: got for name, angles in poses for got in [shown.torn(body, angles)] if got}
    held = shown.held(body)
    cloth = (body.dense[3] & lit.WOVEN) > 0
    report.check("the likeness dressed, in %d poses - a squat, a seat, a bow, a stride, its arms raised -: no point "
                 'of the robot stands out of its clothes, nor its trousers out of its tee, and no edge of them grows '
                 'past TORN times its length; held to them what a garment girds as it stands, what stands by its '
                 'opening and what it wears under another' % len(poses),
                 not out and not tears and len(poses) == 11 and int((held & ~cloth).sum()) > 20 and int((held & cloth).sum()) > 20,
                 out or tears or '%d points of the robot and %d of its trousers held' % ((held & ~cloth).sum(), (held & cloth).sum()))
    if likeness.FOUND:
        worn = gynoid.body(like=likeness.FOUND)
        out = {name: got for name, angles in poses for got in [shown.stood_out(worn, angles)] if got}
        tears = {name: got for name, angles in poses for got in [shown.torn(worn, angles)] if got}
        report.check('this host keeps a figure: in its likeness, the same poses, nothing stands out of its clothes '
                     'and they do not tear', not out and not tears and int(shown.held(worn).sum()) > 100,
                     out or tears or '%d points held' % shown.held(worn).sum())
        wn = [p[0] for p in worn.parts]
        lo, hi = worn.spans[wn.index('like_left_shank')]
        hem = lo + np.flatnonzero(worn.corners[lo:hi, 1] <= plated.ANKLE)
        stand, foot = figure.stand(), wn.index('left_foot')

        def on_shoe(angles):
            """The hem's corners in the foot's frame, the robot at `angles`."""
            frames = worn._frames(angles, shown.ROOT)
            turn, spot = frames[foot]
            return (worn.pose(angles, root=shown.ROOT, _frames=frames)[0][hem] - spot) @ turn
        moved = max(float(np.abs(on_shoe(angles) - on_shoe(stand)).max()) for angles in (
            dict(figure.SQUAT), dict(stand, left_hip=-90.0, left_knee=120.0, left_ankle=-40.0), dict(stand, left_ankle=25.0)))
        report.check("its jeans' hem lies at the ankle and keeps its place on the shoe, the shin leant over the foot "
                     "or the toes down (`plated.BLENDS`: rigid on the shin, the shoe's instep stood out of their front)",
                     len(hem) >= 3 and abs(float(worn.corners[hem, 1].mean()) - plated.ANKLE) < 0.005 and moved < 1e-6,
                     '%d corners of its hem, moved %.1e m on the shoe' % (len(hem), moved))
    else:
        report.skip("in the figure's likeness nothing stands out of its clothes nor tears; a hem keeps its place on the shoe",
                    'no figure on this host')


def test_nothing_stands_through_what_covers_the_robot(report):
    """Each drive is drawn as its cover, its drum under it by the shell's wall; bent far, a garment's
    corners between two frames keep their reach from the joint; dressed, the shell under the clothes
    is not drawn and the limbs past them are."""
    import numpy as np
    from coaxial.graphics import drums, gynoid
    from coaxial.graphics.shapes import excess, moved, ringed
    from machine.body import build, figure
    bare = {name: mesh for name, _parent, _at, mesh in drums.drums()}
    under = {name: float(excess(bare[name][0], ringed(mesh)).max()) for name, _parent, _at, mesh in drums.covers()}
    report.check("every drive's drum under its cover by the shell's wall at the least",
                 len(under) == len(bare) > 0 and max(under.values()) <= -build.WALL_M,
                 '%d drives, the nearest %.1f mm under' % (len(under), -1e3 * max(under.values())))
    body, shell = gynoid.body(), gynoid.body(dressed=False)
    far = dict(figure.stand(), spine=85.0, **{side + joint: deg for side in ('left_', 'right_') for joint, deg in (
        ('hip', -120.0), ('hip_yaw', 20.0), ('knee', 150.0), ('shoulder', 150.0))})
    frames = body._frames(far)
    rigid, _n = moved(body.corners, body.normals, body.spans, frames)
    bent, _n = body.pose(far, root=((0.0, 0.0, 0.0), figure.IDENT), _frames=frames)
    reach = [abs(math.dist(bent[k], frames[hung][1]) - math.dist(rigid[k], frames[hung][1]))
             for _goal, hung, _joints, laid, _dense in body.bends if laid[7] is None for k in laid[3]]
    sums = {}                                  # a garment across both sides: its joints' places weighted as a corner is
    for _goal, hung, _joints, laid, _dense in body.bends:
        if laid[7] is not None:
            held = laid[7][2]
            at = sums.setdefault(id(held), (held, np.zeros((len(held), 4))))[1]
            for idx, weight in ((laid[0], laid[7][0]), (laid[3], laid[7][1])):
                at[held.searchsorted(idx)] += weight[:, None] * np.append(frames[hung][1], 1.0)
    across = [abs(math.dist(bent[k], j) - math.dist(rigid[k], j)) for held, at in sums.values()
              for k, j in zip(held.tolist(), at[:, :3] / at[:, 3:])]
    report.check('bent 85 to 150 deg, each corner between two frames keeps its reach from their joint - one of a '
                 "garment across both sides from its joints' places weighted as it is",
                 len(reach) > 400 and max(reach) < 1e-9 and len(across) > 100 and max(across) < 1e-9,
                 '%d corners, %.1e m at the most; %d across, %.1e m' % (len(reach), max(reach), len(across), max(across)))
    drawn = {part[0]: (len(part[5][1]), len(other[5][1])) for part in body.parts for other in shell.parts
             if part[0] == other[0]}
    report.check('dressed, its pelvis, thighs, shanks, cheeks and bust not drawn; its forearms and feet whole; '
                 'its arms and torso past their clothes alone',
                 all(drawn[name][0] == 0 for name in ('pelvis', 'left_thigh', 'right_shank', 'right_cheek', 'left_bust'))
                 and all(drawn[name][0] == drawn[name][1] for name in ('left_forearm', 'right_foot', 'head'))
                 and all(0 < drawn[name][0] < drawn[name][1] for name in ('left_upper_arm', 'torso')),
                 {name: drawn[name] for name in ('pelvis', 'left_upper_arm', 'right_shank', 'torso', 'left_forearm')})


ROSTER = (test_nothing_stands_out_of_its_clothes, test_nothing_stands_through_what_covers_the_robot)


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
