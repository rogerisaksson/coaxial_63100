"""Her gaze and her stereo through it (`machine.eyes.gaze`, `gimbal`, `epipolar`, `squint`): her eyes turned and
placed as built - squinted, their pushrods' shares and play, their levers - and as her calibration told her them;
the stereo's depth held to the renderer's own looking ahead and turned across and down; her squint told from her
pictures look by look as her gaze turns and turns back; her front looked round once and her gaze held at rest; a
way's places scanned in turn, a place fixed wherever she stands, her gaze let go; her walk to her dock led by her way."""
import itertools
import math
import sys

import numpy as np

from tools.dev.focus import chosen
from gynoid_kit import Report

#: The gazes her stereo is held to the renderer's depth at, (yaw, pitch) deg: ahead, across both ways to her
#: eyes' reach, and down; out to STEREO_M, the room's depth.
GAZES, STEREO_M = ((0, 0, 0), (17, 0, 0), (-35, 0, 0), (50, 0, 0), (-50, 25, 0), (17, 25, 0), (0, 35, 0), (0, 10, 6)), 4.0


def _room():
    from machine.world import physics
    physics.ROOM = physics.SIGHT = 1.0
    from machine import Machine
    from machine.rig.modes import DYNAMIC
    body = Machine.discover('gynoid', execution_mode=DYNAMIC)
    world = next(n.world for n in body.others + list(a.node for a in body.actuators.values())
                 if hasattr(n, 'world'))
    return body, world


def _stood(world, heading=0.0):
    """Her posed standing in the room's middle, headed `heading` rad: nothing walks her."""
    from machine.body import figure
    world.reset(figure.stand(), where=(0.0, figure.standing(), 0.0),
                turn=(math.cos(heading / 2.0), 0.0, math.sin(heading / 2.0), 0.0))


def _angle(R):
    """The angle of rotation `R`, rad."""
    return math.acos(max(-1.0, min(1.0, (float(np.trace(R)) - 1.0) / 2.0)))


def _head(world):
    """Her head's pose as the world has it: she is posed, her estimate not running."""
    hb = world.model.body('head').id
    return {'pos': np.array(world.data.xpos[hb]), 'quat': np.array(world.data.xquat[hb])}


def _squinted(world, eyes):
    """deg (3,): her right eye's turn off her left's as she holds them (`sight.Eyes.held`, her squint told) past their
    turn as built and played (`world.gimbals`), in its frame."""
    from machine.eyes import gimbal
    hl, hr = (gimbal.eye(s, eyes.held[s], True) for s in ('left', 'right'))
    Rl, Rr = (gimbal.eye(s, world.gimbals[s]) for s in ('left', 'right'))
    D = ((hr @ eyes.squint.turn()).T @ hl) @ (Rr.T @ Rl).T
    return np.degrees(np.array([D[2, 1] - D[1, 2], D[0, 2] - D[2, 0], D[1, 0] - D[0, 1]]) / 2.0)


def _errors(eyes, world, a, disp, kept, laid):
    """(n,) her points' distances off her left eye against the renderer's along the pixel of her left eye's picture
    each lies on (`laid.at`), relative, out to STEREO_M."""
    from machine.eyes import epipolar, stereo
    rows, cols = np.nonzero(kept)
    rng = np.linalg.norm(epipolar.points(rows, cols, disp[rows, cols], laid.beta, laid.fv, laid.B), axis=1)
    v, u = (np.clip(np.rint(m[rows, cols] / float(1 << epipolar.BITS)).astype(int), 0, n - 1)
            for m, n in zip(laid.at[0], (stereo.H, stereo.W)))
    z = eyes.truth(world)[v, u]
    truth = z * np.sqrt(1.0 + ((u - stereo.W / 2.0) / epipolar.F) ** 2 + ((v - stereo.H / 2.0) / epipolar.F) ** 2)
    ok = (z > 0.3) & (z < STEREO_M)
    return np.abs(rng[ok] - truth[ok]) / truth[ok]


def test_the_stereo_holds_to_the_depth(report):
    """Her eyes turned by her gaze, each as built and played (`gaze.aimed`), their pictures laid by her eyes as she
    holds them and matched (`sight.Eyes.stereo`): her depth against the renderer's on the pixels kept, ahead and
    turned; at rest the card's disparity against the CPU's; held a parallel pair, unsquinted, it holds not."""
    from machine.eyes import epipolar, gaze, gimbal, sight, stereo
    body, world = _room()
    try:
        _stood(world)
        eyes = sight.Eyes(world)
        for gz_deg in GAZES:
            gz = tuple(math.radians(v) for v in gz_deg)
            gaze.aimed(world, gz)
            pictured, disp, kept = eyes.stereo(world, _head(world), gz)
            a, b, laid = pictured.left, pictured.right, eyes.laid
            err = _errors(eyes, world, a, disp, kept, laid)
            report.check('looking %+d deg across, %+d down, %d converged: her depth within 3 %% of the renderer\'s at the '
                         'median, 10 %% at the ninth decile, a twentieth of her pixels or more kept' % gz_deg,
                         len(err) >= 0.05 * kept.size and np.median(err) < 0.03 and np.percentile(err, 90) < 0.10,
                         'median %.1f %%, 90th %.1f %%, %d kept' % (100 * np.median(err), 100 * np.percentile(err, 90), len(err)))
            if gz_deg == (0, 0, 0):
                (disp_, kept_), (c_disp, c_kept) = stereo.disparity(a, b), stereo.cpu_disparity(a, b)
                most = float(np.max(np.abs(disp_[kept_ & c_kept] - c_disp[kept_ & c_kept])))
                report.check('the card\'s disparity agrees with the CPU\'s within 0.05 px on 99 %% of her laid pixels',
                             (kept_ == c_kept).mean() > 0.99 and most < 0.05,
                             '%.2f %% agree, max %.4f px' % (100.0 * (kept_ == c_kept).mean(), most))
            if gz_deg == (17, 0, 0):
                raw, square = eyes.see(world), gimbal.turned(gz[:2]) @ gimbal.mounted()
                held = epipolar.matched(raw.left, raw.right, square, square)
                err = _errors(eyes, world, *held[:1], *held[2:])
                report.check('held a square pair, unsquinted, turned 17 deg across: her depth off past 5 % at the median',
                             np.median(err) > 0.05, 'median %.1f %%' % (100 * np.median(err)))
    finally:
        body.close()


def test_her_squint_told_from_her_pictures(report):
    """Her gaze turned and turned back, her pushrods' play crossing at each turn (`gimbal.played`), a look a gaze held
    twice: her squint told look by look from her pictures (`squint.Squint`), her depth within 3 % of the renderer's at
    the median at every look - its first at a gaze too -, her right eye's turn off her left's as she holds it within
    0.05 deg of theirs as built and played after each."""
    from machine.eyes import gaze, sight
    body, world = _room()
    try:
        _stood(world)
        eyes, worst, off = sight.Eyes(world), 0.0, 0.0
        for gz_deg in ((0, 0), (0, 0), (17, 0), (17, 0), (-17, 0), (-17, 0), (-50, 25), (-50, 25), (17, 25), (0, 0)):
            gz = tuple(math.radians(v) for v in gz_deg)
            gaze.aimed(world, gz)
            pictured, disp, kept = eyes.stereo(world, _head(world), gz)
            err = _errors(eyes, world, pictured.left, disp, kept, eyes.laid)
            worst, off = max(worst, float(np.median(err))), max(off, float(np.abs(_squinted(world, eyes)).max()))
        report.check('her depth within 3 % of the renderer\'s at the median at every look, its first at a gaze too',
                     worst < 0.03, 'at most %.1f %%' % (100 * worst))
        report.check('her right eye\'s turn off her left\'s as she holds it within 0.05 deg of theirs as built after each',
                     off < 0.05, 'at most %.3f deg' % off)
    finally:
        body.close()


def test_her_eyes_as_built_and_told(report):
    """Her eyes' cameras turned and placed by her gaze each as built and played (`gimbal.eye`, `gimbal.place`): her
    squint more across the further she looks, her pushrods' shares; her calibration's eyes within its share of how they
    are built off square (`gimbal.CHURCH`)."""
    from machine.eyes import gaze, gimbal
    body, world = _room()
    try:
        _stood(world)
        hb = world.model.body('head').id
        worst, off, told = 0.0, {}, 0.0
        for gz_deg in GAZES:
            gz = tuple(math.radians(v) for v in gz_deg)
            gaze.aimed(world, gz)
            head, at = world.data.xmat[hb].reshape(3, 3), world.data.xpos[hb]
            for side in ('left', 'right'):
                i, a = world.model.camera('eye_' + side).id, world.gimbals[side]
                cam = world.data.cam_xmat[i].reshape(3, 3)
                worst = max(worst, float(np.abs(head.T @ cam - gimbal.eye(side, a)).max()),
                            float(np.abs(head.T @ (world.data.cam_xpos[i] - at) - gimbal.place(side, a)).max()))
                half = gz[2] * (0.5 if side == 'right' else -0.5)
                square = gimbal.turned((gz[0] + half, gz[1])) @ gimbal.mounted()
                built, held = (gimbal.eye(side, gimbal.angles(side, gz, t), t) for t in (False, True))
                told = max(told, _angle(held.T @ built) / max(abs(gimbal.CHURCH[side]) * _angle(square.T @ built), 1e-12))
            off[gz_deg] = _angle(gimbal.eye('left', gimbal.angles('left', gz)).T @ gimbal.eye('right', gimbal.angles('right', gz)))
        report.check('her eyes\' cameras look and sit as her gaze turns each as built and played', worst < 1e-6,
                     'within %.1e' % worst)
        report.check('her eyes squint more looking 50 deg across than ahead (her pushrods\' shares)',
                     off[(50, 0, 0)] > off[(0, 0, 0)] + math.radians(0.3),
                     '%.2f deg ahead, %.2f at 50' % (math.degrees(off[(0, 0, 0)]), math.degrees(off[(50, 0, 0)])))
        report.check('her calibration tells each eye within its share of how it is built off square, every gaze',
                     told <= 1.01, '%.3f of it at most' % told)
    finally:
        body.close()


def _g():
    """A gaze's world: her head 1.5 m up at the room's middle, looking along its z; no look yet."""
    return {'t': 0.0, 's': {'head': {'pos': np.array([0.0, 1.5, 0.0]), 'quat': np.array([1.0, 0.0, 0.0, 0.0])}}}


def _looked(g, until, dt=1.0 / 120.0):
    """[(her gaze, whether her eyes stood on what they looked at)] of each look LOOK_S apart till `until` s, her eyes
    settled (`gaze.settled`), her servos moved each `dt` as her vision moves them (`vision.look`, her refresh)."""
    from machine.eyes import gaze, sight
    out, last = [], -math.inf
    while g['t'] < until:
        gz = gaze.moved(g)
        if g['t'] - last >= sight.LOOK_S and gaze.settled(g):
            last = g['t']
            out.append((gz, g['_gaze']['on']))
            gaze.asked(g, gz)
        g['t'] += dt
    return out


def test_her_gaze_explores_then_holds(report):
    """Waking, what she has not seen where she stands: each sector her eyes reach (`gaze.SECTORS`) looked at once, a
    look each where her saccade landed, then her gaze held at rest - her eyes do not wander."""
    from machine.eyes import gaze, sight
    g = _g()
    head = g['s']['head']
    reach = {s for s in gaze.SECTORS if gaze._reached(head, gaze._shown(head, s))}
    looks = _looked(g, (len(reach) + 6) * sight.LOOK_S)
    off = [gz for gz, _there in looks if max(abs(v) for v in gz[:2]) > 1e-9]
    report.check('each sector her eyes reach looked at once, a look each', g['_gaze']['seen'] >= reach and len(off) == len(reach),
                 '%d of %d sectors seen, %d looks off rest' % (len(g['_gaze']['seen'] & reach), len(reach), len(off)))
    rest = looks[len(off):]
    report.check('then her gaze held at rest', bool(rest) and all(max(abs(v) for v in gz[:2]) < 1e-9 for gz, _t in rest)
                 and not g['_gaze']['exploring'], '%d looks at rest' % len(rest))


def test_her_saccades_and_fixations(report):
    """Her eyes from one fixation to the next as a person's: a saccade's time the main sequence's no shorter than her
    servos turn (`gaze.saccade`), its pace a minimum jerk's (its peak 1.875 of its mean), no look in its flight
    (`gaze.settled`); a fixation held on its world point as her head sways as her walk sways it - across 8.2 deg at
    2.40 Hz, down 5.3 -, her gaze off it no more than her head turns in a refresh; her vergence to a point 0.8 m off
    within its lag (TAU)."""
    from machine.eyes import gaze
    g, dt = _g(), 1.0 / 120.0
    gaze.rest(g)
    while not gaze.settled(g):
        gaze.moved(g)
        g['t'] += dt
    gaze.fix(g, (2.0, 1.5, 1.2))
    trail, flying = [], 0
    for _ in range(int(0.6 / dt)):
        trail.append(gaze.moved(g)[:2])
        flying += int(not gaze.settled(g))
        g['t'] += dt
    pace = np.degrees(np.hypot(*np.diff(np.array(trail), axis=0).T)) / dt
    a = math.degrees(math.hypot(*np.subtract(trail[-1], trail[0])))
    T = gaze.saccade(math.radians(a))
    report.check('her saccade the main sequence\'s time, no shorter than her servos turn, its peak a minimum jerk\'s',
                 abs(flying * dt - T) <= dt and abs(pace.max() / (a / T) - 1.875) < 0.05,
                 '%.1f deg in %.0f ms (%.0f asked), peak %.0f deg/s = %.3f of its mean' % (a, 1e3 * flying * dt, 1e3 * T, pace.max(), pace.max() / (a / T)))
    slip, sway = [], []
    t0 = g['t']
    for _ in range(int(3.0 / dt)):
        u = 2.0 * math.pi * 2.40 * (g['t'] - t0)
        yaw, pitch = math.radians(4.1) * math.sin(u), math.radians(2.65) * math.sin(2.0 * u)
        R = gimbal_turn(yaw, pitch)
        g['s']['head'] = {'pos': np.array([0.0, 1.5 + 0.02 * math.sin(2.0 * u), 0.0]), 'quat': _quatted(R)}
        gz = gaze.moved(g)
        want = gaze.toward(g['s']['head'], (2.0, 1.5, 1.2))
        slip.append(math.degrees(max(abs(gz[0] - want[0]), abs(gz[1] - want[1]))))
        sway.append(math.degrees(abs(yaw)))
        g['t'] += dt
    turned_most = math.degrees(math.radians(4.1) * 2.0 * math.pi * 2.40) * dt
    report.check('a fixation held on its world point as her head sways, off it no more than her head turns in a refresh',
                 max(slip) <= turned_most * 1.01, 'off %.2f deg at most against %.2f a refresh, her head %.1f deg' % (max(slip), turned_most, max(sway)))
    gaze.fix(g, (0.0, 1.45, 0.8))
    g['s']['head'] = _g()['s']['head']
    want, t0, half, was = gaze.toward(g['s']['head'], (0.0, 1.45, 0.8))[2], g['t'], None, g['_gaze']['at'][2]
    while g['t'] - t0 < 1.0:
        gz = gaze.moved(g)
        if half is None and gz[2] >= was + (want - was) * (1.0 - math.exp(-1.0)):
            half = g['t'] - t0
        g['t'] += dt
    report.check('her vergence to a point 0.8 m off, its lag TAU', half is not None and abs(half - gaze.TAU) <= 2.0 * dt
                 and abs(gz[2] - want) < 1e-3,
                 '%.2f deg asked, %.2f there, a lag of %s s' % (math.degrees(want), math.degrees(gz[2]), half))


def gimbal_turn(yaw, pitch):
    """(3, 3) a head turned across `yaw` and down `pitch` rad (`gimbal.turned`)."""
    from machine.eyes import gimbal
    return gimbal.turned((yaw, pitch))


def _quatted(R):
    """The quaternion of rotation `R` (`gaze._quat`)."""
    from machine.eyes import gaze
    return gaze._quat(R)


def test_her_gaze_scans_then_fixes(report):
    """Told a way, each of its places looked at HELD looks in turn, scanned a look after the last; a place fixed held
    as her head moves; let go, at rest."""
    from machine.eyes import gaze, sight
    g = _g()
    gaze.rest(g)
    places = [(1.0, 0.0, 2.0), (-1.2, 0.0, 3.0), (0.3, 0.5, 4.0)]
    gaze.scan(g, places)
    looks, scanned = [], None
    while scanned is None and g['t'] < 20.0:
        looks += _looked(g, g['t'] + sight.LOOK_S)
        scanned = len(looks) if gaze.scanned(g) else None
    head = g['s']['head']
    at = [sum(1 for gz, there in looks if there and max(abs(a - b) for a, b in zip(gz[:2], gaze.toward(head, p)[:2])) < 1e-6)
          for p in places]
    report.check('each place of her way looked at HELD looks, in turn, scanned a look after the last',
                 at == [gaze.HELD] * len(places) and scanned is not None,
                 'looks a place %s, scanned at look %s of %d' % (at, scanned, len(looks)))
    gaze.fix(g, (2.0, 0.4, 1.0))
    g['s']['head'] = {'pos': np.array([0.5, 1.5, -0.4]), 'quat': np.array([math.cos(0.3), 0.0, math.sin(0.3), 0.0])}
    _looked(g, g['t'] + 1.0)
    held = gaze.moved(g)
    want = gaze.toward(g['s']['head'], (2.0, 0.4, 1.0))
    report.check('a place fixed held as her head moves, her vergence within its lag',
                 max(abs(a - b) for a, b in zip(held[:2], want[:2])) < 1e-6 and abs(held[2] - want[2]) < 1e-3,
                 'gaze %s, the place %s deg' % (np.round(np.degrees(held), 2).tolist(), np.round(np.degrees(want), 2).tolist()))
    gaze.rest(g)
    _looked(g, g['t'] + 1.0)
    at = gaze.moved(g)
    report.check('let go, her gaze at rest, her vergence within its lag', max(abs(v) for v in at[:2]) < 1e-9 and abs(at[2]) < 1e-3,
                 '%s' % (at,))


def _curve(pts, pace=1.0):
    """(n, 4) a way through `pts` [(x, z)] as her planner's curve has it: x, z, its length walked to each, her pace."""
    pts = np.asarray(pts, float)
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    return np.column_stack([pts, s, np.full(len(pts), pace)])


def test_her_gaze_leads_her_walk(report):
    """Walking her way to her dock (`gaze.fix` led by a way): her gaze on her dock while a look at it shows the ground
    her way takes her to `lead` s on; where it shows it not - her way round, her dock beside her -, that ground HELD
    looks between her dock's HELD, in turn."""
    from machine.eyes import gaze, sight
    g = _g()
    g['s']['feet'] = [{'sole': np.zeros(3)}, {'sole': np.zeros(3)}]
    head, lead = g['s']['head'], 2.0
    runs = {}
    for name, dock, pts in (('straight', (0.0, 0.5, 4.5), [(0.0, 0.0), (0.0, 4.0)]),
                            ('round', (2.5, 0.5, 0.5), [(0.0, 0.0), (0.0, 3.0), (2.5, 3.0), (2.5, 1.0)])):
        gaze.fix(g, dock, _curve(pts), lead)
        looks = _looked(g, g['t'] + 6 * gaze.HELD * sight.LOOK_S)
        on = [('dock' if max(abs(a - b) for a, b in zip(gz[:2], gaze.toward(head, dock)[:2])) < 1e-6 else 'ground'
               if max(abs(a - b) for a, b in zip(gz[:2], gaze.toward(head, (0.0, 0.0, lead))[:2])) < 1e-6 else 'else')
              for gz, _there in looks]
        runs[name] = [(k, sum(1 for _ in grp)) for k, grp in itertools.groupby(on)]
    report.check('her way ahead in her dock\'s view: her gaze on her dock throughout',
                 [k for k, _n in runs['straight']] == ['dock'], '%s' % runs['straight'])
    report.check('her way round, her dock beside her: her dock and her way\'s ground `lead` s on, HELD looks each in turn',
                 len(runs['round']) >= 4 and all(n == gaze.HELD for _k, n in runs['round'][:-1])
                 and [k for k, _n in runs['round']] == ['dock', 'ground'] * (len(runs['round']) // 2) + ['dock'] * (len(runs['round']) % 2),
                 '%s' % runs['round'])


def _walked(g, until, dt=1.0 / 120.0):
    """[(the time, her anchor)] at each look LOOK_S apart till `until` s, her head walked each `dt` by her twist asked
    (`g['twist']`: ahead, to her left, turning) along her heading, her servos moved as her vision moves them."""
    from machine.eyes import gaze, sight
    out, last = [], -math.inf
    while g['t'] < until:
        head = g['s']['head']
        vx, vy, wz = g['twist']
        a = gaze._heading(head)
        head['pos'] = head['pos'] + dt * np.array([vx * math.sin(a) + vy * math.cos(a), 0.0, vx * math.cos(a) - vy * math.sin(a)])
        head['quat'] = _quatted(gimbal_turn(a + wz * dt, 0.0))
        gz = gaze.moved(g)
        if g['t'] - last >= sight.LOOK_S and gaze.settled(g):
            last = g['t']
            gaze.asked(g, gz)
            out.append((g['t'], g['_gaze']['anchor']))
        g['t'] += dt
    return out


def test_her_gaze_follows_her_walk(report):
    """Walking with no errand (her twist asked): her gaze on the ground she walks onto HELD looks on, held there as she
    walks up to it, a saccade on once a look there shows that ground no more - a sawtooth, each anchor ahead of the
    last -; on a turn's arc, on it; stood, let go to rest, and past NEAR_M from where she explored, exploring anew."""
    from machine.eyes import gaze, sight
    g = _g()
    g['s']['feet'] = [{'sole': np.zeros(3)}, {'sole': np.zeros(3)}]
    g['twist'] = (1.0, 0.0, 0.0)
    looks = _walked(g, 4.0)
    anchors = [a for _t, a in looks]
    starts = [a for k, a in enumerate(anchors) if a is not None and (k == 0 or a != anchors[k - 1])]
    T = gaze.HELD * sight.LOOK_S
    report.check('walking ahead, her gaze on the ground she walks onto HELD looks on, each anchor ahead of the last',
                 len(starts) >= 3 and all(b[2] > a[2] for a, b in zip(starts, starts[1:]))
                 and all(abs(a[0]) < 1e-9 and abs(a[1]) < 1e-9 for a in starts),
                 '%d anchors, z %s m' % (len(starts), [round(a[2], 2) for a in starts]))
    g['twist'] = (1.0, 0.0, 0.4)
    head = g['s']['head']
    x, z, a, n = float(head['pos'][0]), float(head['pos'][2]), gaze._heading(head), 4000
    for _ in range(n):
        x, z, a = x + T / n * math.sin(a + 0.2 * T / n), z + T / n * math.cos(a + 0.2 * T / n), a + 0.4 * T / n
    arc = gaze._anchored(g)
    report.check('on a turn\'s arc, her anchor where the arc takes her', arc is not None and math.hypot(arc[0] - x, arc[2] - z) < 1e-3,
                 'anchor (%.3f, %.3f), her arc (%.3f, %.3f)' % (arc[0], arc[2], x, z) if arc else 'none')
    g['twist'] = (0.0, 0.0, 0.0)
    _walked(g, g['t'] + 1.0)
    st = g['_gaze']
    report.check('stood, her anchor let go and her gaze exploring anew where she stands', st['anchor'] is None and st['exploring'],
                 'anchor %s, exploring %s' % (st['anchor'], st['exploring']))


ROSTER = [test_the_stereo_holds_to_the_depth, test_her_squint_told_from_her_pictures, test_her_eyes_as_built_and_told,
          test_her_gaze_explores_then_holds, test_her_saccades_and_fixations, test_her_gaze_scans_then_fixes,
          test_her_gaze_leads_her_walk, test_her_gaze_follows_her_walk]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
