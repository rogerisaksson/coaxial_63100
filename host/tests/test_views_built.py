"""The robot's build as sketched: a joint's mount holds its loads, turns its travel and keeps its hose."""
import os
import sys
from typing import Any

from tools.dev.focus import chosen
from views_kit import Report

#: The build the sketches draw: each check a process of its own on it.
BUILD = 'gynoid_mesomorph_medium_coax'


def _told(code):
    """What `code` prints last, as JSON, run in a process of its own on the build; {'failed': its fault} else."""
    import json
    import subprocess
    host = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    run = subprocess.run([sys.executable, '-X', 'utf8', '-c', 'import json; ' + code], cwd=host, capture_output=True,
                         text=True, env=dict(os.environ, PYTHONPATH=host, COAXIAL_ROBOT=BUILD))
    return json.loads(run.stdout.strip().splitlines()[-1]) if run.returncode == 0 else {'failed': run.stderr[-400:]}


def test_the_shoulder_holds_turns_and_keeps_its_coil(report):
    """The coaxial build's shoulder as sketched (`tools.render.shoulder_mount`, sized by `shoulder_laid`): every
    member holds the shoulder's loads at the margin by its hand checks - the hub's screws, the foot's studs, the
    carbon plate, the crossbar and the arm's tube among them; what turns with the arm keeps a millimetre off what
    stays with the torso through the travel, short of the stops where horn and rib meet, and crosses none of it
    with the arm hanging; the bus's hose, a coil round the crossbar, bends no tighter than the hose's own least
    radius and crosses no part at five turns of the arm; the travel takes the farthest the robot swung an arm back
    while it stood and the farthest ahead short of a fall; and the arm's guards, laid clear of the whole arm,
    cross nothing of the shoulder's mount (`arm_worn.capped`) - with no likeness beside the tree there is no figure
    to lay them on."""
    told: Any = _told('from tools.render import shoulder_mount as s; print(json.dumps(s.checked()))')
    report.check("every member holds the shoulder's loads at the margin", (told.get('held') or 0.0) >= 1.0,
                 told.get('failed') or told.get('held'))
    report.check('what turns keeps a millimetre off what stays through the travel, and crosses none of it hanging',
                 (told.get('air') or 0.0) >= 1.0 and told.get('crossing') == 0, (told.get('air'), told.get('crossing')))
    bend, own = told.get('bend') or (0.0, 1.0)
    report.check("the coil bends no tighter than the hose's own least radius and crosses no part at five turns",
                 bend >= own and told.get('coil') == 0, (told.get('bend'), told.get('coil')))
    (lo, hi), (back, ahead) = told.get('travel') or (0.0, 0.0), told.get('asked') or (-1.0, 1.0)
    report.check('the travel takes the farthest the robot swung an arm back while it stood, and ahead short of a fall',
                 lo <= back and hi >= ahead, (told.get('travel'), told.get('asked')))
    from coaxial.graphics import likeness
    if not likeness.FOUND:                               # the guards are laid on the figure's body beside the tree
        report.skip("the arm's guards cross nothing of the shoulder's mount", 'no likeness beside the tree')
        return
    capped: Any = _told('from tools.render import arm_worn as a; print(json.dumps({"crossing": a.capped()}))')
    report.check("the arm's guards cross nothing of the shoulder's mount", capped.get('crossing') == 0, capped)


def test_the_neck_holds_clears_and_keeps_its_hose(report):
    """The coaxial build's neck and head as sketched (`tools.render.neck_mount`, sized by `neck_laid`; their
    motors' leads `neck_hosed`): every member holds the neck's and the head's loads at the margin by its hand
    checks - the two hubs' screws, the yoke's carbon plate, its crown's floor, the shoe under the head's thrust,
    the three bands' screws and the spine's foot among them; what nods keeps a millimetre off what stays through
    the neck's travel and crosses none of it upright; the hose that takes the head's motor's leads across the
    neck's joint bends no tighter than its own least radius, hung through that travel and where it is laid still;
    hung, it keeps a millimetre off the mount's parts, laid, no stretch of it lies free longer than a hose may, and
    it, the neck's leads' sleeve and their pipes cross no part; and what nods or turns keeps a printed wall inside
    the likeness's head's outer skin - the skin turned through the head's travel past what does not turn with it -,
    with no likeness beside the tree there is no skin to keep inside."""
    told: Any = _told('from tools.render import knee_loads as k, neck_hosed as h, neck_mount as n; '
                      'print(json.dumps({"mount": n.checked(), "hose": h.checked(), "wall": k.WALL}))')
    mount, hose = told.get('mount') or {}, told.get('hose') or {}
    report.check("every member holds the neck's and the head's loads at the margin", (mount.get('held') or 0.0) >= 1.0,
                 told.get('failed') or mount.get('held'))
    report.check('what nods keeps a millimetre off what stays through the travel, and crosses none of it upright',
                 (mount.get('air') or 0.0) >= 1.0 and mount.get('crossing') == 0, (mount.get('air'), mount.get('crossing')))
    bend, own = hose.get('bend') or (0.0, 1.0)
    report.check("the hose across the neck's joint bends no tighter than its own least radius, hung and laid",
                 bend >= own and (hose.get('laid') or 0.0) >= own, (hose.get('bend'), hose.get('laid')))
    free, most = hose.get('free') or (1.0, 0.0)
    report.check('hung, the hose keeps a millimetre clear; laid, no stretch lies free too long; nothing crosses',
                 (hose.get('air') or 0.0) >= 1.0 and free <= most and hose.get('crossing') == 0,
                 (hose.get('air'), hose.get('free'), hose.get('crossing')))
    if mount.get('skin') is None or hose.get('skin') is None:     # the skin is the likeness's, beside the tree
        report.skip("what nods or turns keeps a printed wall inside the head's outer skin", 'no likeness beside the tree')
        return
    report.check("what nods or turns keeps a printed wall inside the head's outer skin",
                 min(list(mount['skin']) + [hose['skin']]) >= told.get('wall', 1e9), (mount['skin'], hose['skin']))


ROSTER = (test_the_shoulder_holds_turns_and_keeps_its_coil, test_the_neck_holds_clears_and_keeps_its_hose)


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
