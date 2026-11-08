"""Her scoot on her fists along a seat, found as a puppet: the egress stroke the chair text binds.

    python -X utf8 tools/sim/fist.py --scene chair_behind --say sit_chair --start 8.0        # one puppet, its rows
    python -X utf8 tools/sim/fist.py --push -20,-30,-40 --elbow 10,20                          # a grid: the way a stroke

Sat on a seat too high for her feet (`transfer-egress-scoot`), her law's twist on planted fists never moved her: a
121 N push costs the shear's regularisation 5.9 against the twist's 0.36, and where it pushed the shoulders sat at
their clamps. A stroke instead, as the bed pull's (`puppet.heel`): the fists planted beside her hips, the elbows
straightened so the fists bear and the seat unloads, the shoulders extended so the planted fists carry her pelvis
forward, the seat set down, the fists lifted and planted again - till her feet bear `feet` N or her pelvis passes
`mark` m along the seat. Played on the servos from `start` s (`puppet.play`), the rows say the pelvis's way along the
seat a stroke, the slip across, the fall. Measured on the chair from her sit at 8 s: sat plumb her knuckles sit 6 cm
over the seat beside her hips, so her straight arms press 105 N at most (under the seat's 0.6 friction on the rest of
her) and the fists slip instead of her; pressed past that she hangs on them (330 N) and a push swings her 10 cm a
stroke, once 25-40 cm in one; an elbow angle between is a lottery on the arm's stiffness, and a press ended on the
fists' load overshoots the same way; leaning the hips to load the fists (`--lean`) holds through the push but the
pelvis slides back as she sits upright again, so strokes net 0-12 cm either way. Her law's own lean of the egress
stage, a flat 0.25 rad in the text, met the chair once (slid to her soles and rose) and slid nothing at either wobble.
"""
import argparse
import itertools
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools.sim import puppet
from tools.sim.puppet import JI, SIDES, Regime, both


class Fist(Regime):
    """The seat scoot on her fists: `plant_sh`/`elbow_in` deg the fists planted beside her hips, the elbows out to
    `elbow_out` (the press), the shoulders back to `push_sh` at `rate` deg/s (the push), the seat set, the fists
    lifted; over when her feet bear `feet` N, her pelvis passes `mark` m along, or `most` strokes are played."""
    first = 'plant'

    def __init__(self, plant_sh=5.0, push_sh=-30.0, elbow_in=15.0, elbow_out=0.0, rate=60.0, feet=40.0, mark=None,
                 most=4, press=160.0, lean_hip=-105.0, sit_hip=-90.0, **params):
        super().__init__(plant_sh=plant_sh, push_sh=push_sh, elbow_in=elbow_in, elbow_out=elbow_out, rate=rate,
                         feet=feet, mark=mark, most=most, press=press, lean_hip=lean_hip, sit_hip=sit_hip, **params)
        self.strokes = 0

    def done(self, sense):
        p = self.params
        return (sense['left_foot'] + sense['right_foot'] >= p['feet']
                or (p['mark'] is not None and sense['pelvis'] <= p['mark']))

    def hips(self, goal, sense, rate=40.0):
        for s in SIDES:
            self.ramp(s + '_hip', goal, rate, sense['dt'])

    def p_plant(self, el, sense):
        self.set(**both(shoulder=self.params['plant_sh'], elbow=self.params['elbow_in']))
        self.hips(self.params['sit_hip'], sense)
        return 'press' if (sense['hands'] >= 40.0 and el > 0.3) or el > 0.8 else None

    def p_press(self, el, sense):
        # the elbows straighten till her fists bear `press` N of her: the seat keeps the rest (an angle alone is a
        # lottery on the arm's stiffness - 5 deg out her fists bore 105 N and slipped, straight 330 N and she hung)
        # - and her trunk leans over them (the hips past the sit's): plumb, her straight arms barely reach the seat
        self.hips(self.params['lean_hip'], sense)
        if sense['hands'] < self.params['press']:
            for s in SIDES:
                self.ramp(s + '_elbow', self.params['elbow_out'], self.params['rate'], sense['dt'])
        return 'push' if sense['hands'] >= self.params['press'] or el > 1.0 else None

    def p_push(self, el, sense):
        # the lean stays where the press ended it: leaning on through the push her fists took her whole weight
        for s in SIDES:
            self.ramp(s + '_shoulder', self.params['push_sh'], self.params['rate'], sense['dt'])
        there = abs(float(self.cmd[JI['left_shoulder']]) - self.params['push_sh']) < 1.0
        return 'set' if there or el > 1.5 or self.done(sense) else None

    def p_set(self, el, sense):
        self.set(**both(elbow=self.params['elbow_in'] + 15.0))
        self.hips(self.params['sit_hip'], sense, 60.0)
        if el > 0.25:
            self.strokes += 1
            return 'end' if self.done(sense) or self.strokes >= self.params['most'] else 'lift'
        return None

    def p_lift(self, el, sense):
        self.set(**both(shoulder=self.params['plant_sh'] + 10.0, elbow=self.params['elbow_in'] + 30.0))
        self.hips(self.params['sit_hip'], sense, 60.0)
        return 'plant' if el > 0.3 else None


def strokes(rows):
    """(the pelvis's way along a stroke, m) from `rows`: between the starts of each `push`."""
    at = [r['along'] for i, r in enumerate(rows) if r['phase'] == 'push' and (i == 0 or rows[i - 1]['phase'] != 'push')]
    return [round(b - a, 3) for a, b in zip(at, at[1:])]


def main(argv=None):
    ap = argparse.ArgumentParser(description=(__doc__ or '').split('\n')[0])
    ap.add_argument('--scene', default='chair_behind')
    ap.add_argument('--say', default='sit_chair')
    ap.add_argument('--prop', default='chair', help="the furniture's body (`puppet.Scene`)")
    ap.add_argument('--start', type=float, default=8.0, help='the s the fists take over her servos')
    ap.add_argument('--until', type=float, default=16.0)
    ap.add_argument('--plant', default='5', help='the shoulders planted, deg (a grid: comma list)')
    ap.add_argument('--push', default='-30', help='the shoulders pushed back to, deg')
    ap.add_argument('--elbow', default='15', help='the elbows planted, deg')
    ap.add_argument('--out', default='0', help='the elbows pressed out to, deg')
    ap.add_argument('--rate', type=float, default=60.0, help='deg/s')
    ap.add_argument('--feet', type=float, default=40.0, help='over when her feet bear this, N')
    ap.add_argument('--mark', type=float, default=None, help='over when her pelvis passes this along the seat toward its front edge (-half), m')
    ap.add_argument('--most', type=int, default=4, help='strokes at most')
    ap.add_argument('--press', default='160', help='the press ends with her fists bearing this, N (a grid: comma list)')
    ap.add_argument('--lean', default='-105', help='the hips through the press and push, deg: her trunk over her fists')
    a = ap.parse_args(argv)
    grid = [[float(v) for v in getattr(a, k).split(',')] for k in ('plant', 'push', 'elbow', 'out', 'press', 'lean')]
    for plant_sh, push_sh, elbow_in, elbow_out, press, lean in itertools.product(*grid):
        regime = Fist(plant_sh=plant_sh, push_sh=push_sh, elbow_in=elbow_in, elbow_out=elbow_out, rate=a.rate,
                      feet=a.feet, mark=a.mark, most=a.most, press=press, lean_hip=lean)
        rows, _sc, _s = puppet.play(a.scene, a.say, regime, prop=a.prop, start=a.start, until=a.until)
        for k, r in enumerate(rows):
            if k + 1 == len(rows) or rows[k + 1]['phase'] != r['phase']:
                print('%6.2f %-6s along %+.3f across %+.3f up %.3f trunk %5.1f hands %4.0f feet %4.0f' % (
                    r['t'], r['phase'], r['along'], r['across'], r['up'], r['trunk'], r['hands'], r['feet']), flush=True)
        print('plant %+.0f push %+.0f elbow %.0f out %.0f press %.0f lean %.0f: strokes %s, end %+.3f, fallen %s' % (
            plant_sh, push_sh, elbow_in, elbow_out, press, lean, strokes(rows), rows[-1]['along'], rows[-1]['fallen']),
            flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
