"""Her movements' terms (`machine.terms`): its rows her figure's laid anew; a term moves her in
MuJoCo as its name says, either limb the one sign; its stops the figure's; what she has not is
absent; the leg's own IK bends its joints the terms' way."""
import importlib
import math
import sys

from tools.dev.focus import chosen
from gynoid_kit import Report

IDENT = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
#: How far a term is asked to see what it moves, deg.
ASKED_DEG = 10.0


def test_the_rows_are_her_figure(report):
    """machine/term_rows.py is what tools/cores/terms.py lays from her figure now."""
    from tools.cores import terms as laid
    with open(laid.PATH, encoding='utf-8') as f:
        on_disk = f.read()
    report.check('the rows on disk are her figure laid anew', laid.render(laid.rows()) == on_disk,
                 'python tools/cores/terms.py --write')


def _posed(mujoco, model, data, segment, kind, vector, angles):
    """A segment's point or axis, world, in MuJoCo with the joints at `angles` {joint: deg}."""
    data.qpos[:] = model.qpos0
    for joint, deg in angles.items():
        data.qpos[model.joint(joint).qposadr[0]] = math.radians(deg)
    mujoco.mj_kinematics(model, data)
    body = data.body(segment)
    turn = body.xmat.reshape(3, 3)
    tip = [float(sum(turn[r][c] * vector[c] for c in range(3))) for r in range(3)]
    return [float(p) + t for p, t in zip(body.xpos, tip)] if kind == 'point' else tip


def test_a_term_moves_her_as_its_name_says(report):
    """Every term asked ASKED_DEG of each limb - or of her trunk - in the compiled figure: its
    segment's point or axis goes the way anatomy says (`tools.cores.terms.ANATOMY`), nine
    tenths of its motion at the least; its opposite the other way; a term toward a side, toward
    the limb's whose mirror it took."""
    from machine import mjcf, reach_rows, term_rows, terms
    from tools.cores import terms as laid
    mujoco = importlib.import_module('mujoco')
    model = mujoco.MjModel.from_xml_string(mjcf.mjcf())
    data = mujoco.MjData(model)
    pair, least, asked = laid.limbs(), math.inf, 0
    for part, stem, name, opposite, kind, vector, way in laid.ANATOMY:
        if terms.find(part, name) is None or stem is None or way is None:
            continue
        of_limb = stem in pair[0]
        for limb, mirror in enumerate(reach_rows.LIMBS):
            segment = pair[limb][stem] if of_limb else stem
            toward = (way[0] * mirror, way[1], way[2])
            for term, sense in ((name, 1.0), (opposite, -1.0)):
                if term is None:
                    continue
                joint, sign, _low, _high = terms.TERMS[(part, term)]
                sided = (part, term) in term_rows.TOWARD
                deg = sign * ASKED_DEG * (mirror if sided else 1.0)
                at = [_posed(mujoco, model, data, segment, kind, vector, angles)
                      for angles in ({}, {terms.joint(limb, joint): deg})]
                went = sum((b - a) * w for a, b, w in zip(at[0], at[1], toward)) / math.radians(ASKED_DEG)
                least, asked = min(least, sense * went), asked + 1
    report.check('every term moves her its own way in MuJoCo, either limb', asked > 40 and least > 0.9,
                 '%d asked, %.3f of the motion at the least' % (asked, least))


def test_its_stops_are_the_figures(report):
    """A term's stops are her figure's as declared - `mjcf.STOPS`, the hip's yaw, the ankle's
    rods' strokes -, through the term's sign; with none declared, none."""
    from machine import linkage, mjcf, terms
    roll = linkage.ROLLED['ankle']
    declared = dict(mjcf.STOPS, hip_yaw=(-mjcf.HIP_YAW_DEG, mjcf.HIP_YAW_DEG),
                    ankle=linkage.RODS['ankle'][1], ankle_roll=(-roll, roll))
    wrong, held, free = [], 0, []
    for (part, name), (joint, sign, low, high) in terms.TERMS.items():
        if joint in declared:
            held += 1
            if (low, high) != tuple(sorted(sign * v for v in declared[joint])):
                wrong.append((part, name, low, high))
        elif joint not in ('neck', 'head'):
            free.append((part, name, low, high))
    report.check("a term's stops are her figure's through its sign", held > 10 and not wrong,
                 '%d held; wrong: %s' % (held, wrong or 'none'))
    report.check('where nothing is declared, none', all(low is None and high is None for _p, _n, low, high in free),
                 ', '.join('%s %s' % (p, n) for p, n, _l, _h in free))


def test_what_she_has_not_is_absent(report):
    """A movement anatomy names and no driven joint of hers makes is not found: her shoulder's
    abduction, her forearm's pronation, her wrist's flexion - held -, her toes' - sprung; no
    term is on a joint without a drive."""
    from machine import drives, term_rows, terms
    found = [t for t in term_rows.ABSENT if terms.find(*t) is not None]
    named = all(t in term_rows.ABSENT for t in (('shoulder', 'abduction'), ('forearm', 'pronation'),
                                                 ('wrist', 'flexion'), ('toes', 'extension')))
    undriven = [(part, name) for (part, name), (joint, *_rest) in terms.TERMS.items()
                if any(drives.passive(terms.joint(limb, joint)) for limb in range(2))]
    report.check('what she has not is absent', named and not found and not undriven,
                 '%d absent, %d of them found, %d terms on a joint with no drive'
                 % (len(term_rows.ABSENT), len(found), len(undriven)))


def test_the_legs_ik_bends_the_terms_way(report):
    """`figure.leg`, either limb: the foot ahead of the hip is hip flexion, behind it extension;
    a foot drawn up, knee flexion; out to its side, abduction; turned out, lateral rotation;
    its toes up, dorsiflexion - each joint's angle times its term's sign."""
    from machine import figure, reach_rows, terms
    from machine.gait import ANKLE_H
    pelvis, wrong = (0.0, 0.9, 0.0), []
    for limb, mirror in enumerate(reach_rows.LIMBS):
        hip = figure.hip(mirror, pelvis, IDENT)
        under = (hip[0], ANKLE_H, hip[2])
        poses = {
            ('hip', 'flexion'): ((under[0], ANKLE_H + 0.1, under[2] + 0.3), IDENT),
            ('hip', 'extension'): ((under[0], ANKLE_H + 0.1, under[2] - 0.3), IDENT),
            ('knee', 'flexion'): ((under[0], ANKLE_H + 0.3, under[2]), IDENT),
            ('hip', 'abduction'): ((under[0] + mirror * 0.3, ANKLE_H + 0.1, under[2]), IDENT),
            ('hip', 'lateral-rotation'): (under, figure.ry(mirror * 0.3)),
            ('ankle', 'dorsiflexion'): (under, figure.rx(-0.3)),
        }
        for (part, name), (ankle, foot) in poses.items():
            joint, sign, _low, _high = terms.TERMS[(part, name)]
            angle = dict(zip((n[1:] for n in figure.LEG), figure.leg(mirror, pelvis, IDENT, ankle, foot)))[joint]
            if not sign * angle > 0.02:
                wrong.append((limb, part, name, round(math.degrees(angle), 1)))
    report.check("the leg's IK bends each joint its term's way, either limb", not wrong, '%s' % (wrong or '12 of 12'))


ROSTER = [test_the_rows_are_her_figure, test_a_term_moves_her_as_its_name_says,
          test_its_stops_are_the_figures, test_what_she_has_not_is_absent,
          test_the_legs_ik_bends_the_terms_way]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
