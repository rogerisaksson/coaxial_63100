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


ROSTER = (test_the_shoulder_holds_turns_and_keeps_its_coil,)


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
