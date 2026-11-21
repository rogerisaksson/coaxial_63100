"""The robot bare of its clothes: carbon tubes, bolted drives, honeycomb guards; held to the pictures."""
import math
import os
import sys

from tools.dev.focus import chosen
from views_kit import Report


def _islands(triangles, count):
    """Each corner's island among `count`: the least corner its triangles join it to."""
    import numpy as np
    label = np.arange(count)
    for _ in range(count):
        was = label.copy()
        np.minimum.at(label, triangles.ravel(), np.repeat(label[triangles].min(axis=1), 3))
        if (label == was).all():
            break
    return label


def _off(points, tubes):
    """How far each of `points` stands off the nearest of `tubes` [(a, b, radius)], m: under 0 within one."""
    import numpy as np
    best = np.full(len(points), np.inf)
    for a, b, r in tubes:
        a, b = np.asarray(a, float), np.asarray(b, float)
        w = np.clip((points - a) @ (b - a) / ((b - a) @ (b - a)), 0.0, 1.0)
        best = np.minimum(best, np.linalg.norm(points - (a + w[:, None] * (b - a)), axis=1) - r)
    return best


def test_the_mechanism_in_its_livery_bolted(report):
    """Bare of its clothes the robot is drawn as its mechanism (`plated.framed`): every part of it a hard shell's in
    its livery's paints - black carbon tubes, anodised drives -, each drive bolted, a ring of bolts on either face of
    its can; its own head and its shoes on it, no shell over it, and its seat a plate a side. Posed as its mechanism
    is."""
    import numpy as np
    from coaxial.graphics import drums, gynoid, lit, livery, mechanism, plated
    from machine.body import figure
    body = gynoid.body(dressed=False, plates=True)
    names, frame = [p[0] for p in body.parts], [p[0] for p in mechanism.parts()]
    worn = {name: body.parts[names.index(name)][5][3] for name in frame}
    shells = {name for name in names if name.startswith('shell_')}
    report.check("its mechanism's every part in its order, its bolts, its guards and its seat beside; of a shell its "
                 'own head and its shoes alone - no trunk, no hips, no limb',
                 [n for n in names if not n.startswith(('bolts_', 'guard_', 'shell_')) and not n.endswith('_seat')] == frame
                 and names.count('left_seat') == names.count('right_seat') == 1
                 and shells == {'shell_head', 'shell_jaw', 'shell_nose', 'shell_lips'} | {
                     'shell_' + s + n for s in ('left_', 'right_') for n in ('ear', 'eye', 'foot', 'toes')},
                 '%d parts of the mechanism, %d shells' % (len(frame), len(shells)))
    servos = [name for name in frame if name.startswith(('drive_', 'inv_'))]
    tubes = [name for name in frame if name.startswith('hung')]
    report.check("its mechanism is hard shells, every part's every corner; a drive's motor and its board anodised, its "
                 "gearbox darker, a tube black carbon - a twill's under a clear coat",
                 all(bool(((ink & lit.HARD) > 0).all()) for ink in worn.values()) and len(servos) > 30
                 and all(bool((worn[name] == livery.SERVO).all()) for name in servos)
                 and all(bool((worn[name] == livery.GEAR).all()) for name in frame if name.startswith('gear_'))
                 and len(tubes) > 20 and all(livery.TUBE in worn[name] for name in tubes) and bool(livery.TUBE & lit.TWILL),
                 '%d parts of drives anodised, %d tubes carbon' % (len(servos), len(tubes)))
    bolted = [name[len('bolts_'):] for name in names if name.startswith('bolts_')]
    heads, can = body.parts[names.index('bolts_left_knee')][5], body.parts[names.index('drive_left_knee')][5]
    axis = np.array(drums.AXES['left_knee'][0], float)
    along, radius = heads[0] @ axis, float(np.linalg.norm(can[0] - np.outer(can[0] @ axis, axis), axis=1).max())
    ring = np.linalg.norm(heads[0] - np.outer(along, axis), axis=1)
    base, top = (heads[0].reshape(-1, 12, 3)[:, part].mean(axis=1) for part in (slice(0, 6), slice(6, 12)))
    over, under = plated.faces(axis, can[0], can[1], base)
    face = np.r_[over[:plated.BOLTS], under[plated.BOLTS:]]
    way = np.r_[np.ones(plated.BOLTS), -np.ones(plated.BOLTS)]
    report.check('each drive is bolted: BOLTS heads on either face of its can, a hexagon each, on a circle CIRCLE of '
                 'its radius, each on its face where it stands and BOLT_M proud of it',
                 sorted(bolted) == sorted(name[len('drive_'):] for name in frame if name.startswith('drive_'))
                 and len(heads[0]) == 2 * plated.BOLTS * 12 and bool((heads[3] == livery.BOLT).all())
                 and abs(float(ring.mean()) - plated.CIRCLE * radius) < 0.002
                 and float(np.abs(base @ axis - face).max()) < 1e-9
                 and float(np.abs((top - base) @ axis - way * plated.BOLT_M).max()) < 1e-9
                 and float((base @ axis).max()) <= float((can[0] @ axis).max()) + 1e-9
                 and float((base @ axis).min()) >= float((can[0] @ axis).min()) - 1e-9,
                 '%d drives bolted, a knee\'s ring %.4f m of a can %.4f, its heads %.4f to %.4f along it of %.4f to %.4f'
                 % (len(bolted), ring.mean(), radius, (base @ axis).min(), (base @ axis).max(), (can[0] @ axis).min(),
                    (can[0] @ axis).max()))
    walk = dict(figure.stand(), left_hip=-25.0, right_hip=20.0, left_knee=30.0, left_shoulder=-25.0, right_elbow=40.0)
    stood, _n = body.pose(figure.stand())
    walked, _n = body.pose(walk)
    report.check('posed as its mechanism is - its cranks and its rods with its joints -, its lowest sole on the floor',
                 body.see == 'plated' and bool(np.isfinite(walked).all()) and abs(float(stood[:, 1].min())) < 0.01
                 and abs(float(walked[:, 1].min())) < 0.01 and float(np.abs(walked - stood).max()) > 0.05,
                 'its lowest point %.4f m standing, %.4f walking' % (stood[:, 1].min(), walked[:, 1].min()))


def test_guards_of_honeycomb_stood_off_the_tubes(report):
    """On the mechanism its guards (`guards.laid`): an open honeycomb in a rim on each limb's segment and on the trunk -
    its bars black and nothing behind them, the seat's net -, stood off the tubes on struts with air between; none
    closes its limb, a leg's inside and its back wear none, and its hips, its seat and the small of its back are open;
    a knee's cup clears its knee's drum."""
    import math
    import numpy as np
    from coaxial.graphics import guards, gynoid, livery, panels
    from machine.body import skeleton
    body = gynoid.body(dressed=False, plates=True)
    names = [p[0] for p in body.parts]
    worn = sorted(name[len('guard_'):] for name in names if name.startswith('guard_'))
    report.check("a guard on each limb's four segments, a side each, and on the trunk; none on its hips, its neck, a "
                 'hand or a foot',
                 worn == sorted(['torso'] + [s + n for s in ('left_', 'right_') for n in guards.LIMB]), worn)
    air, free, cells, meets, opens, reach = {}, {}, {}, [], [], {}
    for seg in worn:
        corners, triangles, _uv, ink = body.parts[names.index('guard_' + seg)][5]
        side, _, limb = seg.partition('_')
        part = limb if limb in guards.LIMB else seg
        rows_g = ([(-a * panels.LIMBS[limb][0], -b * panels.LIMBS[limb][0], m) for _n, a, b, m, *_ in guards.LIMB[limb]]
                  if limb in guards.LIMB else [(hi, lo, m) for _n, lo, hi, m, *_ in guards.TRUNK])
        req = ({livery.FRAME, livery.VOID, livery.BOLT}
               | ({livery.STEEL} if any(guards.held(part, top, foot, mid) is None for top, foot, mid in rows_g) else set())
               | ({livery.TUBE} if guards.hoops(part) else set()))
        x, hardware = -1.0 if side == 'right' else 1.0, np.isin(ink, (livery.STEEL, livery.TUBE))
        island = _islands(triangles, len(corners))
        roots, counts = np.unique(island, return_counts=True)
        rings = np.array([corners[island == r] for r, n in zip(roots, counts) if n == 12 and ink[r] == livery.VOID])
        mid = rings[:, :6].mean(axis=1)                  # a cell's bars on its face or its back: out, then its opening
        out, inner = (np.linalg.norm(rings[:, k:k + 6] - mid[:, None], axis=2).mean(axis=1) for k in (0, 6))
        apart = np.linalg.norm(rings[:, None, :6, None] - rings[None, :, None, :6], axis=-1)
        apart[np.arange(len(rings)), np.arange(len(rings))] = 9.0
        meets.append(apart.min(axis=(1, 2, 3)))          # a corner shared with a neighbour's ring: 0
        opens.append(inner / out)
        cells[seg] = len(rings) if set(np.unique(ink).tolist()) >= req else 0
        if limb in guards.LIMB:
            tubes = skeleton.runs(limb, x)
            air[seg] = float(_off(corners[~hardware], tubes).min())
            reach[seg] = float(_off(corners[ink == livery.STEEL], tubes).min())
            about = np.degrees(np.arctan2(corners[~hardware, 2], x * corners[~hardware, 0])) % 360.0
            free[seg] = set(range(24)) - set((about // 15.0).astype(int).tolist())
            low, high = (corners[~hardware, 1].min() / panels.LIMBS[limb][0],
                         corners[~hardware, 1].max() / panels.LIMBS[limb][0])
            span = max(0.92, max(float(b) for _n, _a, b, *_ in guards.LIMB[limb]) + 0.05)
            cells[seg] *= bool(-span < low and high < 0.09)
    meet, opening = np.concatenate(meets), np.concatenate(opens)
    report.check("a guard is a rim, an open honeycomb in it - black bars, nothing behind them -, rivets and struts; its "
                 "cells a honeycomb - a ring of bars each, open within, meeting its neighbours' -, short of its limb's ends",
                 all(n >= 4 for n in cells.values()) and float(np.median(meet)) < 1e-4
                 and 0.6 < float(opening.min()) and float(opening.max()) < 0.95,
                 "%s cells; a ring's corner %.2f mm off its nearest neighbour's at the median; open %.2f..%.2f of a "
                 'cell' % (sorted(cells.values()), 1e3 * np.median(meet), opening.min(), opening.max()))
    report.check("air between a guard and its limb's tubes, 3 mm at the least, and its struts reach a tube",
                 min(air.values()) > 0.003 and max(reach.values()) < 0.001,
                 'air %.0f..%.0f mm' % (1e3 * min(air.values()), 1e3 * max(air.values())))
    inside, behind = set(range(11, 14)), set(range(16, 20))
    report.check("no guard closes its limb: a third of the ways round it and more are bare; a leg's inside and its "
                 'back wear none',
                 all(len(ways) >= 8 for ways in free.values())
                 and all(inside | behind <= free[s + n] for s in ('left_', 'right_') for n in ('thigh', 'shank')),
                 {seg: len(ways) for seg, ways in free.items()})
    trunk, ink = body.parts[names.index('guard_torso')][5][0], body.parts[names.index('guard_torso')][5][3]
    trunk = trunk[~np.isin(ink, (livery.STEEL, livery.TUBE))]
    dorsal = min(float(lo) for _n, lo, _hi, mid, *_ in guards.TRUNK if math.sin(math.radians(float(mid))) < -0.5) - 0.005
    ventral = min(float(lo) for _n, lo, *_ in guards.TRUNK) - 0.005
    report.check('its hips, its seat and the small of its back are open: no guard on its hips, the trunk\'s behind '
                 'none under its shoulder blades\' slat, ahead none under its belly\'s',
                 'guard_pelvis' not in names and dorsal > 0.10 and ventral > 0.03
                 and float(trunk[trunk[:, 2] < -0.02, 1].min()) > dorsal
                 and float(trunk[:, 1].min()) > ventral,
                 'behind from %.3f m up the trunk, ahead from %.3f' % (trunk[trunk[:, 2] < -0.02, 1].min(), trunk[:, 1].min()))
    pos, _n = body.pose({})
    cup, drum = (pos[slice(*body.spans[names.index(name)])] for name in ('guard_left_shank', 'drive_left_knee'))
    cup = cup[np.abs(cup[:, 1] - drum[:, 1].mean()) < 0.03]
    report.check("a guard clears what it lies over: a knee's cup stands ahead of its knee's drum",
                 float(cup[:, 2].max()) > float(drum[:, 2].max()) + 0.01,
                 'the cup %.3f m ahead, the drum %.3f' % (cup[:, 2].max(), drum[:, 2].max()))


def test_a_motor_turned_after_its_makers_model(report):
    """A motor's model (`bells`: an OBJ, its axis z, its foot at 0) is told as its turned outline - a foot under a
    bell, a hub on its face; the knee's motor is drawn after its model where one is found - within its drum, its
    hub toward its gearbox -, else as its drum."""
    import os
    import tempfile
    import numpy as np
    from coaxial.graphics import bells, drums, gynoid
    from coaxial.graphics.shapes import loft
    from machine.body import drives
    rings = [(0.0, 0.8, 0.8), (0.25, 0.8, 0.8), (0.25, 1.0, 1.0), (0.8, 1.0, 1.0), (0.8, 0.3, 0.3), (1.0, 0.3, 0.3)]
    corners, triangles, _uv, _ink = loft(rings, 0, poles=(0.0, 1.0))       # up its y: a model's up its z
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, 'stepped.obj')
        with open(path, 'w', encoding='utf-8', newline='') as file:
            file.writelines('v %.6f %.6f %.6f\n' % (x, -z, y) for x, y, z in 0.04 * corners)
            file.writelines('f %d %d %d\n' % tuple(t + 1) for t in triangles)
        got, tall = bells.outline(path)
    z, r = np.array(got).T
    first, last = int(np.argmax(r)), len(r) - 1 - int(np.argmax(r[::-1]))           # its rim, up
    under = np.interp((0.5, 0.9), r[:first + 1], z[:first + 1])                      # out under its foot and its bell
    over = np.interp((0.15, 0.6), r[last:][::-1], z[last:][::-1])                    # in over its face to its hub
    near = 2.0 / bells.BINS
    report.check('a model told as its turned outline: its foot, its bell and its hub, a dozen points at the most, its '
                 'height over its width',
                 len(got) <= 12 and abs(tall - 0.5) < 1e-4 and float(np.abs(under - (0.0, 0.25)).max()) < near
                 and float(np.abs(over - (1.0, 0.8)).max()) < near,
                 '%d points, its foot and its bell %s under, its hub and its face %s over, %.6f tall a width' % (
                     len(got), under.round(3).tolist(), over.round(3).tolist(), tall))
    body = gynoid.body(dressed=False, plates=True)
    names = [p[0] for p in body.parts]
    can = body.parts[names.index('drive_left_knee')][5][0]
    axis = np.array(drums.AXES['left_knee'][0], float)
    _part, radius, _at, long = next(p for p in drives.along('left_knee') if p[0] == 'motor')
    along = can @ axis
    out = np.linalg.norm(can - np.outer(along, axis), axis=1)
    face = out[along > along.max() - 1e-9].max()
    report.check("the knee's motor after its model, within its drum and its hub toward its gearbox; with none, its drum",
                 float(out.max()) <= radius + 1e-9 and float(np.ptp(along)) <= long + 1e-9
                 and (face < 0.5 * radius if bells.MODELS else face > 0.8 * radius),
                 '%d models; its face out to %.4f m of %.4f' % (len(bells.MODELS), face, radius))


def test_arms_out_and_light_on_hard_shells(report):
    """T-posed (`arms_out`), an upper arm and what hangs on it lie out level, a side each; a hard shell's rim light is
    a faint one (`lit.HARD_RIM`); a cell's ink is the mean of the dots it may draw - black paint's are no part of it."""
    import numpy as np
    from coaxial.graphics import gynoid, lit, raster
    from tools.sim import alike
    body = gynoid.body(dressed=False, plates=True)
    names = [p[0] for p in body.parts]
    pos, _n = body.pose(alike.t_posed())
    out = {name: pos[slice(*body.spans[names.index(name)])] for name in (
        'guard_left_upper_arm', 'guard_right_upper_arm', 'hung2_left_upper_arm', 'guard_left_forearm')}
    left, right = out['guard_left_upper_arm'], out['guard_right_upper_arm']
    report.check('arms out, an upper arm\'s guard and its tube lie out along the arm at its shoulder\'s height, a '
                 "side each - hung on an arm turned once, not twice -, the forearm's past them",
                 float(left[:, 0].min()) > 0.1 and float(right[:, 0].max()) < -0.1 and float(np.ptp(left[:, 1])) < 0.12
                 and abs(float(left[:, 1].mean()) - float(right[:, 1].mean())) < 0.005
                 and float(np.ptp(out['hung2_left_upper_arm'][:, 1])) < 0.08
                 and float(out['guard_left_forearm'][:, 0].min()) > float(left[:, 0].mean()),
                 'x %.2f..%.2f left, %.2f..%.2f right' % (left[:, 0].min(), left[:, 0].max(), right[:, 0].min(), right[:, 0].max()))
    edge = np.array([[0.995, 0.0, 0.1]])               # a surface turned away: its rim's light at the most
    plain, hard = (lit._light(edge, np.array([lit.paint((40, 40, 40), hard=how)]), np.zeros((1, 2)))[0].astype(int)
                   for how in (False, True))
    report.check("a hard shell's rim is a faint one: at a surface's edge a dark paint is blue, a hard shell's grey",
                 plain[2] > plain[0] + 60 and hard[2] - hard[0] < 0.5 * (plain[2] - plain[0]),
                 '%s a paint, %s a hard shell' % (plain[:3].tolist(), hard[:3].tolist()))
    high, wide = 2, 2
    depth = np.ones((high * raster.DOTS_Y, wide * raster.DOTS_X))
    rgb = np.zeros(depth.shape + (4,), np.uint8)
    rgb[..., 3] = 224
    rgb[::2, :, :3] = 200                               # a dot in two foam, the other its backing's black
    inks = [fg for row in lit.ansi.parse('\n'.join(lit.braille(depth, rgb, np.zeros(depth.shape), wide, high)))
            for _ch, fg, _bg in row if fg]
    report.check("a cell half a pad's foam and half its black backing is drawn in the foam's ink, the backing's dots "
                 'not drawn',
                 len(inks) == 4 and all(min(fg) > 200 for fg in inks), inks[:1])


def test_the_robot_bare_is_held_to_the_pictures(report):
    """The robot bare of its clothes, T-posed, is measured as a picture of a robot is (`tools.sim.alike`) - the shares
    of what is seen that are dark, grey, light and of each hue, its air, its edge, its girth - and held to what the
    pictures beside the tree measured: wrong colours and wrong styling fail."""
    import numpy as np
    from coaxial.graphics import likeness
    from tools.sim import alike
    rgb, mask = np.zeros((120, 60, 3)), np.zeros((120, 60), bool)
    mask[10:110, 10:20] = mask[10:110, 24:34] = True    # two bars, a gap of four between them
    rgb[10:60, 10:34], rgb[60:110, 10:20], rgb[60:110, 24:34] = 1.0, 0.1, (1.0, 0.2, 0.8)
    got = alike.measured(rgb * mask[..., None], mask)
    report.check('a picture measured: half of two bars light, a quarter dark, a quarter magenta; the gap between '
                 'them its air, a bar its girth - ten of a hundred rows',
                 abs(got['palette']['light'] - 0.5) < 0.01 and abs(got['palette']['dark'] - 0.25) < 0.01
                 and abs(got['palette']['magenta'] - 0.25) < 0.01 and abs(got['air'] - 4.0 / 24.0) < 0.01
                 and abs(got['girth'] - 0.1) < 0.005 and got['edge'] > 0.0,
                 {k: round(v, 3) for k, v in got['palette'].items() if v} | {k: round(got[k], 3) for k in ('air', 'edge', 'girth')})
    rgb, mask, body = alike.drawn(None)
    robot = alike.measured(rgb, mask, body)
    faults = alike.alike(robot)
    report.check('the robot as built, bare and T-posed, is alike: its palette no further from a picture\'s than the '
                 'pictures lie apart, no hue of its own, their air, edge and girth by half at the least',
                 not faults and len(alike.PICTURED) > 1, faults or alike._said('the robot', robot).strip())
    swapped = alike.alike(alike.measured(rgb[..., [1, 0, 2]], mask, body))
    solid = alike.alike(alike.measured(rgb, alike._closed(mask, 12), None))
    frame = alike.alike(alike.measured(*alike.drawn(None, 'mechanism')))
    report.check('wrong colours and wrong styling fail: its drives in another hue, a body with no air in it, the '
                 "mechanism in its frames' own paints",
                 any('hues' in fault for fault in swapped) and any('air' in fault for fault in solid) and len(frame) > 0,
                 '; '.join(swapped + solid + frame))
    found = {os.path.basename(path): alike.measured(*alike.cut_out(path)) for path in alike.pictures()}
    named = sorted(set(found) & set(alike.PICTURED))
    if named and likeness.FOUND:
        told = alike.PICTURED
        apart = max(max(abs(found[n][k] - told[n][k]) for k in ('air', 'edge', 'girth')) for n in named)
        apart = max(apart, max(alike.apart(found[n]['palette'], told[n]['palette']) for n in named))
        worn = alike.measured(*alike.drawn(likeness.FOUND))
        faults = alike.alike(worn, list(found.values()))
        report.check('this host keeps the pictures: those it names measure what is told of them (PICTURED), and the '
                     "robot in the figure's likeness is alike to every one it keeps", apart < 0.005 and not faults,
                     '%d named within %.3f of what is told, untold %s; %s' % (
                         len(named), apart, ', '.join(sorted(set(found) - set(told))) or 'none', faults or 'alike'))
    else:
        report.skip('the pictures beside the tree measure what is told of them', 'no pictures on this host')


ROSTER = (test_the_mechanism_in_its_livery_bolted, test_guards_of_honeycomb_stood_off_the_tubes,
          test_a_motor_turned_after_its_makers_model, test_arms_out_and_light_on_hard_shells,
          test_the_robot_bare_is_held_to_the_pictures)


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
