#!/usr/bin/env python3
"""The build's hoses as its figure carries them: the sketches' spans laid in its segments' frames.

    python -X utf8 tools/render/hosed.py --write     # machine/world/hoses/<build>.toml
    page = hosed.text()                              # what the file holds, laid anew
    told = hosed.checked()                           # the file, the hoses' bends, their fittings held, what is drawn

A sketch's hose across a joint (`knee_hose.span`, `elbow_mount.span`: its fittings, in the joint's frame with it
straight) is laid where the robot's figure carries it (`figure.SEGMENTS`): its still side on the
segment above the joint, its turning side on the one below, as the sketch stands on the drive (`spans`: its x the
drive's own or turned about, its middle where the drive's is; its z mirrored where the figure turns the segment below
the other way than that x) - the one below as it stands with the joint straight,
whatever turn the figure rests that segment at. The hose is as long as its fittings stand apart at their farthest
through the joint's own travel as the figure turns it, and `hung.SPARE_M`; laid only where its least bending radius
through the sketch's travel
(`hung.least`) keeps the hose's own (`materials.toml` [hoses]). `machine.world.hoses` reads the file; a test holds
it to this.
"""
import math
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

NOTICE = "The build's hoses across its joints: written by host/tools/render/hosed.py, not by hand."
#: The time the figure's knees and elbows are swept through their travel in, s, and the frames it is drawn in.
SWEPT_S, FRAMES = 1.0, 30
#: Each hose's joint: (the segment above it, the one below, the drive).
JOINTS = {'knee': ('left_thigh', 'left_shank', 'left_knee'), 'elbow': ('left_upper_arm', 'left_forearm', 'left_elbow')}


def path(robot=None):
    """Where the build's hoses are written."""
    from machine.body import mechatronics
    from machine.world import hoses
    return os.path.join(hoses.LAID, (robot or mechatronics.ROBOT) + '.toml')


def spans():
    """{name: (the sketch's span, (way, mid) its x against the drive's own)}: the build's hoses as sketched."""
    from tools.render import elbow_mount as em, knee_hose
    return {'knee': (knee_hose.span(), (1.0, 0.0)), 'elbow': (em.span(), em.axis())}


def laid():
    """{name: {'kind', 'length', 'from', 'to'}}: each hose in the figure's segments' frames, m."""
    import numpy as np
    from machine.body import drives, figure, mechatronics
    from machine.errors import MachineError
    from tools.render import hung, knee_mount as km
    segments, made, out = {s[0]: s for s in figure.SEGMENTS}, mechatronics.materials()['hoses'], {}
    for name, (span, (way, mid)) in sorted(spans().items()):
        above, below, joint = JOINTS[name]
        (_j, axis, sign), offset, rest = segments[below][2][0], np.array(segments[below][3]), segments[below][4]
        if segments[below][1] != above or axis != 'x':
            raise MachineError('%s turns %s about %s, not about x as its sketch' % (joint, below, axis))
        least, own = hung.least(span), float(made[span['kind']][4])
        if least < own:
            raise MachineError("the %s's hose bends at %.1f mm, under its %.1f" % (name, 1e3 * least, 1e3 * own))
        c, s = math.cos(math.radians(rest)), math.sin(math.radians(rest))
        rested = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])       # the figure's rest turn, about z

        def put(side, at, point=True):
            """A sketch's point - or, not a `point`, a way - on `side` in its segment's frame: its z by the figure's
            turn's sign, so the sketch bends the way the figure does."""
            q = np.array([way * (at[0] + mid * point), at[1], sign * at[2]])
            return q + offset * point if side == 'still' else rested.T @ q

        def end(fitting):
            side, at, leaves = fitting
            return [above if side == 'still' else below, put(side, at), put(side, leaves, False)]
        ends = end(span['from']), end(span['to'])
        lo, hi = drives.TRAVEL[drives.kind(joint)]
        apart = max(float(np.linalg.norm(offset + rested @ km._rx(sign * turn) @ ends[1][1] - ends[0][1]))
                    for turn in np.r_[np.arange(lo, hi, 1.0), hi])
        out[name] = {'kind': span['kind'], 'length': round(apart + hung.SPARE_M, 4), 'from': ends[0], 'to': ends[1]}
    return out


def text(done=None):
    """The file's text."""
    def told(vector):
        return '[%s]' % ', '.join('%.6f' % (round(float(q), 6) + 0.0) for q in vector)

    def fitting(end):
        return '["%s", %s, %s]' % (end[0], told(end[1]), told(end[2]))
    lines = ['# ' + NOTICE,
             '# Each: its kind (materials.toml [hoses]), its length m, the fitting it leaves and the one it goes into -',
             "# [segment, its mouth, the way the hose leaves it], m in the segment's frame. The left side's; the",
             "# right's its mirror."]
    for name, hose in sorted((done or laid()).items()):
        lines += ['', '[%s]' % name, 'kind = "%s"' % hose['kind'], 'length = %.4f' % hose['length'],
                  'from = ' + fitting(hose['from']), 'to = ' + fitting(hose['to'])]
    return '\n'.join(lines) + '\n'


def outputs():
    """[path]: each build's hoses' file there is."""
    from machine.body import mechatronics
    return [path(robot) for robot in mechatronics.robots() if os.path.exists(path(robot))]


def checked():
    """{'laid': the build's file is this tool's output, 'bends': {hose: (the least radius it bends at through its
    joint's travel, the hose's own least) mm}, 'held': (how far a hose's end stands off its fitting at the most as
    the page carries them - `machine.world.hoses`, its links by the robot's world's step, the figure's knees and
    elbows swept through their travel in SWEPT_S s, FRAMES frames -, a hose's radius) mm, 'drawn': {'leg', 'arm':
    (the least radius any hose its sketch draws bends at, the longest stretch one laid still lies free between
    holds) mm} read off what is drawn (`tees.tightest`, `tees.loosest`), 'free': the longest that may be, mm}."""
    import numpy as np
    from coaxial.graphics import gynoid
    from machine.body import drives, mechatronics, peripherals
    from machine.world import hoses
    from tools.render import elbow_mount as em, hung, knee_hose, leg_clamped as lc, tees
    with open(path(), encoding='utf-8') as f:
        same = f.read() == text()
    made = mechatronics.materials()['hoses']
    bends = {name: (round(1e3 * hung.least(span), 1), 1e3 * float(made[span['kind']][4]))
             for name, (span, _axis) in spans().items()}
    leg = dict(knee_hose.knee(0.0), **knee_hose.ankle(lc.under(), lc.shank()['fork']))
    arm, cables, _still, _turned = em.bodies()
    still = [t for name, tris in leg.items() if name != 'knee hose' for t in tris]       # all but the one hung
    drawn = {'leg': (tees.tightest(still + leg['knee hose']), tees.loosest(still)),
             'arm': (tees.tightest(arm + cables), tees.loosest(arm))}
    who, laid_ = gynoid.body(False, None, None, True), hoses.spans()
    index = {seg: [part[0] for part in who.parts].index(seg) for s in laid_ for seg in (s['from'][0], s['to'][0])}

    def posed(share):
        frames = who._frames({side + joint: share * drives.TRAVEL[joint][1] for side in ('left_', 'right_')
                              for joint in ('knee', 'elbow')}, None)
        return {seg: frames[i] for seg, i in index.items()}
    world, off = hoses.World(laid_, posed(0.0), step=float(peripherals.world()['step_s'])), 0.0
    for k in range(1, FRAMES + 1):
        poses = posed(k / FRAMES)
        world.follow(poses, SWEPT_S / FRAMES)
        for span, line in zip(laid_, world.lines()):
            for end, at in ((span['from'], line[0]), (span['to'], line[-1])):
                turn, put = poses[end[0]]
                off = max(off, float(np.linalg.norm(turn @ np.array(end[1]) + put - at)))
    return {'laid': same, 'bends': bends,
            'held': (round(1e3 * off, 2), 1e3 * min(float(made[s['kind']][0]) for s in laid_)),
            'drawn': {k: tuple(round(1e3 * q, 1) for q in v) for k, v in drawn.items()},
            'free': round(1e3 * tees.unheld(), 1)}


if __name__ == '__main__':
    from tools.cores import written
    if '--write' not in sys.argv[1:]:
        sys.exit(__doc__)
    page = text()
    os.makedirs(os.path.dirname(path()), exist_ok=True)
    written(path(), page)
    print('%s: %d lines' % (path(), page.count('\n')))
