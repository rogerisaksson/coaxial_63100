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
    env = dict(os.environ, PYTHONPATH=host, COAXIAL_ROBOT=BUILD)
    env.setdefault('OPENBLAS_NUM_THREADS', '1')
    run = subprocess.run([sys.executable, '-X', 'utf8', '-c', 'import json; ' + code], cwd=host, capture_output=True,
                         text=True, env=env)
    return json.loads(run.stdout.strip().splitlines()[-1]) if run.returncode == 0 else {'failed': run.stderr[-400:]}


def test_the_shoulder_holds_turns_and_keeps_its_coil(report):
    """The coaxial shoulder (`tools.render.shoulder_mount`, `shoulder_laid`): every member holds the shoulder's loads
    at the margin; what turns keeps a millimetre off what stays through travel and crosses none hanging; its bus coil
    bends no tighter than the hose's own least radius and crosses nothing; travel spans a human swing back to ahead;
    its abduction (`shoulder_rolled`) holds at the margin, clears what stays, never crosses across travel, rises past a
    T and rotates inward after shoulder pitch to clasp hands front and back; its bus (`shoulder_hosed`) holds; and arm
    guards clear the shoulder's mount."""
    from coaxial.graphics import likeness
    got: Any = _told('from coaxial.graphics import likeness; '
                     'from tools.render import arm_worn as a, robot_built as b, shoulder_hosed as h, '
                     'shoulder_mount as s, shoulder_rolled as r; '
                     'print(json.dumps({"mount": s.checked(), "raised": r.checked(), "bus": h.checked(), '
                     '"capped": {"crossing": a.capped()} if likeness.FOUND else None, '
                     '"clasp": b.clasped() if likeness.FOUND else None}))')
    told: Any = got.get('mount') or got
    report.check("every member holds the shoulder's loads at the margin", (told.get('held') or 0.0) >= 1.0,
                 got.get('failed') or told.get('held'))
    report.check('what turns keeps a millimetre off what stays through the travel, and crosses none of it hanging',
                 (told.get('air') or 0.0) >= 1.0 and told.get('crossing') == 0, (told.get('air'), told.get('crossing')))
    bend, own = told.get('bend') or (0.0, 1.0)
    report.check("the coil bends no tighter than the hose's own least radius and crosses no part at five turns",
                 bend >= own and told.get('coil') == 0, (told.get('bend'), told.get('coil')))
    (lo, hi), (back, ahead) = told.get('travel') or (0.0, 0.0), told.get('asked') or (-1.0, 1.0)
    report.check("the travel takes a human's swing back, and the farthest the robot swung an arm ahead short of a fall",
                 lo <= back and hi >= ahead, (told.get('travel'), told.get('asked')))
    raised: Any = got.get('raised') or {}
    report.check("the abduction holds at the margin, clears what stays, and never crosses across its travel",
                 (raised.get('held') or 0.0) >= 1.0 and (raised.get('air') or 0.0) >= 1.0
                 and raised.get('crossing') == 0,
                 got.get('failed') or (raised.get('held'), raised.get('air'), raised.get('crossing')))
    bend, own = raised.get('bend') or (0.0, 1.0)
    report.check("the arm rises past a T on its abduction, the bus's span across it no tighter than the hose's own",
                 (raised.get('reaches') or 0.0) >= 90.0 and bend >= own, (raised.get('reaches'), raised.get('bend')))
    bus: Any = got.get('bus') or {}
    (bend, own), (free, most) = bus.get('bend') or (0.0, 1.0), bus.get('free') or (1.0, 0.0)
    report.check("the bus's hoses through the shoulder bend no tighter than their own, held, a millimetre off what "
                 "turns by them", bend >= own and free <= most and (bus.get('air') or 0.0) >= 1.0, got.get('failed') or bus)
    if not likeness.FOUND:                               # the guards are laid on the figure's body beside the tree
        report.skip("the arm's guards cross nothing of the shoulder's mount", 'no likeness beside the tree')
        return
    capped: Any = got.get('capped') or {}
    report.check("the arm's guards cross nothing of the shoulder's mount", capped.get('crossing') == 0, capped)
    clasp: Any = got.get('clasp') or {}
    report.check('turning the shoulders ahead or back and drawing the pectoral drives brings the hands together cleanly',
                 bool(clasp.get('in_travel')) and clasp.get('front_gap', 9.0) <= 2.0
                 and clasp.get('back_gap', 9.0) <= 2.0 and clasp.get('front_lap', 9.0) <= 5.0
                 and clasp.get('back_lap', 9.0) <= 5.0 and clasp.get('crossing') == 0, clasp)


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


def test_a_mount_is_sized_for_what_it_bore_not_for_a_peak(report):
    """The rule the sketches' loads are read by (`tools.sim.legload.table`, `knee_loads.MEASURED`), on rows made
    up: a load counts by its mean over MEAN_S on end - a steady one in full, a blow of a millisecond by its
    impulse, never by its peak -; a joint that kept for KEPT_S more than the robot's weight at its limb's far end
    explains is wedged, and nothing it bore in that scenario counts, whichever of its six told; what a fall
    loaded it with is told apart, and so is the highest peak of the scenarios it stood."""
    import numpy as np
    from tools.sim import legload as ll
    t = np.arange(0.0, 3.0, 0.001)
    v = np.zeros((len(t), 6))
    v[:, 0], v[1500, 1] = 100.0, 2000.0                  # a steady 100 N; a blow of 2000 N for a millisecond: 2 N s
    mean, kept = ll._over(t, v, ll.MEAN_S, np.mean), ll._over(t, v, ll.KEPT_S, np.min)
    report.check('a steady load counts in full, a blow of a millisecond by its impulse, and is not kept',
                 abs(mean[0] - 100.0) < 1e-9 and abs(mean[1] - 2.0 / (ll.MEAN_S + 0.001)) < 1.0 and kept[1] == 0.0
                 and abs(kept[0] - 100.0) < 1e-9, (mean[:2].round(1).tolist(), kept[:2].tolist()))

    def row(name, stood, mean, kept, most):
        joint = {'force': most[:3], 'moment': most[3:], 'mean': mean, 'kept': kept, 'reach': 0.4, 'at': [1.0] * 6,
                 'lasted': [0.01] * 6}
        return dict({'name': name, 'stood': stood, 'weight': 280.0}, **{j: joint for j in ll.OUTER})
    walked = [50.0, 300.0, 40.0, 60.0, 5.0, 20.0]
    rows = [row('walk', True, walked, [10.0, 250.0, 5.0, 5.0, 1.0, 2.0], [80.0, 600.0, 70.0, 90.0, 9.0, 40.0]),
            row('wedge', True, [900.0, 100.0, 50.0, 30.0, 5.0, 300.0], [10.0, 20.0, 5.0, 5.0, 1.0, 200.0], [1000.0] * 6),
            row('fall', False, [500.0] * 6, [0.0] * 6, [3000.0] * 6)]
    got: Any = ll.table(rows)['elbow']
    report.check('a wedged joint counts for nothing in its scenario, whichever of its six told',
                 got['mean'] == walked and got['in'] == ['walk'] * 6 and [w[:2] for w in got['wedged']] == [[5, 'wedge']],
                 (got['mean'], got['wedged']))
    report.check("a fall's loads and the highest peak stood through are told apart from what a mount is sized for",
                 got['fell'] == [3000.0] * 6 and got['spike'][1][:2] == [600.0, 'walk'], (got['fell'], got['spike'][1]))


def test_the_trunk_reaches_its_travel_and_prints_standing(report):
    """The coaxial build's trunk as first drafted (`tools.render.trunk_laid`, `trunk_clamped`): the calf's drive
    pair on the chest over one cardan joint low by the hips, a rod each down to the pelvis's pins. The pair
    reaches every pose of the build's own travel - its roll's ends with its pitch's - with no rod pulling nearer
    its crank's dead centre than the layout holds; a rod's crosses, forks and swivel (`trunk_rods`) hold the most
    a rod is asked, and the chest's carbon yokes (`trunk_yokes`) the most the cross pushes them with; each of its
    solid printed parts stands on its bed with nothing hung over it, and all its prints fit a plate; and its guards
    - the page's cups, sternum, belly slats and back (`trunk_worn`) - are laid and cross nothing the chest carries:
    with no likeness beside the tree there is no figure to lay them on."""
    told: Any = _told('from tools.render import prints as p, solids as s, trunk_laid as t, trunk_rods as r, '
                      'trunk_yokes as y; got = list(t.reached().values()); '
                      'print(json.dumps({"reached": [v is not None for v in got], "least": t.LEVER, '
                      '"lever": min([v["lever"] for v in got if v] or [0.0]), "held": r.held(), "yokes": y.held(), '
                      '"hung": s.hung(s.TRUNK), "noise": s.NOISE_CM2, "fit": p.fitted(p.trunk())}))')
    reached = told.get('reached') or [False]
    report.check("the pair reaches every pose of the build's travel, no rod nearer its crank's dead centre than held",
                 all(reached) and (told.get('lever') or 0.0) >= told.get('least', 1.0) - 1e-9,
                 told.get('failed') or (sum(reached), len(reached), told.get('lever')))
    held = told.get('held') or {'none told': 0.0}
    report.check("a rod's crosses, forks and swivel hold the most a rod is asked", min(held.values()) >= 1.0, held)
    yokes = told.get('yokes') or {'none told': 0.0}
    report.check("the chest's carbon yokes hold the most the cross pushes them with", min(yokes.values()) >= 1.0, yokes)
    hung = told.get('hung') or {'none told': 1e9}
    report.check("each of the trunk's solid prints stands on its bed with nothing hung over it",
                 max(hung.values()) <= told.get('noise', 0.0), hung)
    fit: Any = told.get('fit') or {'over': ['none told']}
    report.check("the trunk's prints fit a plate", fit['over'] == [] and fit.get('pieces', 0) > 0, fit)
    from coaxial.graphics import likeness
    if not likeness.FOUND:                               # the guards are laid on the figure's body beside the tree
        report.skip("the trunk's guards are laid and cross nothing the chest carries", 'no likeness beside the tree')
        return
    worn: Any = _told('from tools.render import trunk_worn as w; '
                      'print(json.dumps({"laid": len(w.guards()), "crossing": w.crossing()}))')
    report.check("the trunk's guards are laid and cross nothing the chest carries",
                 (worn.get('laid') or 0) > 0 and worn.get('crossing') == 0, worn)


def test_a_print_shaped_by_its_loads_stands_on_its_bed(report):
    """The optimiser the prints are shaped with (`tools.render.topo`), on its own trial piece - a bracket held by
    one end's face, pushed down at its other end's top edge: what the optimiser keeps is less than the room it
    was given, carries the load within the printed polymer's allowable - the room all solid (`topo.whole`) no less
    -, keeps every voxel told to stay, and hangs no voxel over the bed - each stands on one of the nine under it;
    shaped extruded, as a load-bearing slab is,
    it is a prism up its bed - each column's own voxels all kept or all gone, its pockets through -, it keeps
    every voxel told to stay, and its outline is drawn smooth (`topo.outlined`)."""
    told: Any = _told('import numpy as np; from tools.render import solids, topo; g = topo.shaped(topo.trial(), fresh=True); '
                      'layers = g["solid"].transpose(1, 0, 2); c = dict(topo.trial(), extruded=True); '
                      'e = topo.shaped(c, fresh=True); s, own = e["solid"], ~c["kept"]; '
                      'pg = solids.printable(topo.faced(g, topo.trial())); pe = solids.printable(topo.faced(e, c)); '
                      'sm = round(topo.surfaced_margin(g, topo.trial()), 3); '
                      'print(json.dumps({"grams": g["grams"], "room": g["room"], "margin": g["margin"], '
                      '"whole": topo.whole(topo.trial())["margin"], "pg": pg, "pe": pe, "sm": sm, '
                      '"lost": g["lost"], "hung": int((layers & ~topo.stood(layers)).sum()), '
                      '"prism": bool((((s & own).any(axis=1) == (s | ~own).all(axis=1)) | ~own.any(axis=1)).all()), '
                      '"extruded lost": e["lost"], "faces": len(topo.outlined(s, e["h"])), "margin x": e["margin"]}))')
    report.check('what the optimiser keeps is less than its room and carries the load within the allowable, the '
                 'room all solid no less',
                 0.0 < (told.get('grams') or 0.0) < 0.6 * (told.get('room') or 0.0)
                 and 1.0 <= (told.get('margin') or 0.0) <= (told.get('whole') or 0.0),
                 told.get('failed') or (told.get('grams'), told.get('room'), told.get('margin'), told.get('whole')))
    report.check('every voxel told to stay is kept, and none hangs over the bed',
                 told.get('lost') == 0 and told.get('hung') == 0, (told.get('lost'), told.get('hung')))
    pg: Any = told.get('pg') or {}
    pe: Any = told.get('pe') or {}
    report.check('surfaced trial parts stay closed on a flat bed with <=45-deg overhangs and carry their loads',
                 pg.get('over_cm2', 1.0) <= 0.05 and pg.get('worst_deg', 90.0) <= 45.0 and pg.get('open', 1) == 0
                 and pg.get('bed_cm2', 0.0) >= 2.0 and (told.get('sm') or 0.0) >= 1.0
                 and pe.get('over_cm2', 1.0) <= 0.05 and pe.get('worst_deg', 90.0) <= 45.0
                 and pe.get('open', 1) == 0 and pe.get('bed_cm2', 0.0) >= 0.4,
                 (pg, pe, told.get('sm')))
    report.check('extruded, it is a prism up its bed that carries the load, keeps what it was told to and is drawn smooth',
                 bool(told.get('prism')) and told.get('extruded lost') == 0 and (told.get('faces') or 0) > 0
                 and (told.get('margin x') or 0.0) >= 1.0,
                 (told.get('prism'), told.get('extruded lost'), told.get('faces'), told.get('margin x')))


def _check_rooms(report, label, exprs):
    """Verify `exprs` {name: (module, expr)} carry their loads all solid (`topo.whole`) and, where cached, shaped."""
    mods = ', '.join(sorted({mod for mod, _e in exprs.values()}))
    items = ', '.join('%r: %s' % (k, e) for k, (_m, e) in exprs.items())
    got: Any = _told('from tools.render import solids, topo, %s; out = {}\n'
                     'for k, c in {%s}.items():\n'
                     '    w = topo.whole(c)\n'
                     '    s = topo.shaped(c) if topo.ready(c) else None\n'
                     '    layers = s["solid"].transpose(1, 0, 2) if s else None\n'
                     '    p = solids.printable(topo.faced(s, c)) if s else None\n'
                     '    sm = round(topo.surfaced_margin(s, c), 3) if s else None\n'
                     '    out[k] = {"whole": round(w["margin"], 3), "shaped": None if s is None else '
                     '{"grams": round(float(s["grams"]), 1), "room": round(float(s["room"]), 1), '
                     '"margin": round(float(s["margin"]), 3), "lost": int(s["lost"]), '
                     '"hung": int((layers & ~topo.stood(layers)).sum()), "print": p, "surf_margin": sm}}\n'
                     'print(json.dumps(out))' % (mods, items))
    wholes = {k: (got.get(k) or {}).get('whole') or 0.0 for k in exprs}
    report.check('%s carry their loads all solid at the margin' % label,
                 'failed' not in got and min(wholes.values()) >= 1.0, got if 'failed' in got else wholes)
    shaped = {k: got[k]['shaped'] for k in exprs if isinstance((got.get(k) or {}).get('shaped'), dict)}
    if shaped:
        report.check('cached %s weigh less than their rooms, hold, keep their seats and hang nothing' % label,
                     all(0.0 < d['grams'] < d['room'] and d['margin'] >= 1.0 and d['lost'] == 0 and d['hung'] == 0
                         for d in shaped.values()), shaped)
        report.check('cached surfaced %s stand closed on a flat bed within 45 deg and carry their loads' % label,
                     all(d['print']['over_cm2'] <= 0.05 and d['print']['worst_deg'] <= 45.0
                         and d['print']['open'] == 0 and d['print']['bed_cm2'] >= 8.0 and d['surf_margin'] >= 1.0
                         for d in shaped.values()), shaped)


def test_the_hood_room_carries_its_loads(report):
    """The shoulder abduction hood's room (`shoulder_shaped`) carries its loads all solid and, where cached, shaped."""
    _check_rooms(report, "the shoulder hood's room", {'hood': ('shoulder_shaped', 'shoulder_shaped.case()')})


def test_the_trunk_and_hip_rooms_carry_their_loads(report):
    """The trunk's and hip's rooms (`trunk_shaped`, `leg_shaped`) carry their loads all solid and, where cached, shaped."""
    _check_rooms(report, "the trunk's and hip's rooms",
                 {'trunk yoke': ('trunk_shaped', 'trunk_shaped.cases()["yoke"]'),
                  'trunk lever': ('trunk_shaped', 'trunk_shaped.cases()["lever"]'),
                  'hip yoke': ('leg_shaped', 'leg_shaped.cases()["yoke"]'),
                  'hip block': ('leg_shaped', 'leg_shaped.cases()["block"]')})


def test_the_thigh_and_knee_rooms_carry_their_loads(report):
    """The thigh's and knee's rooms (`leg_shaped`) carry their loads all solid and, where cached, shaped."""
    _check_rooms(report, "the thigh's and knee's rooms",
                 {'thigh arm': ('leg_shaped', 'leg_shaped.cases()["thigh arm"]'),
                  'thigh knuckle': ('leg_shaped', 'leg_shaped.cases()["thigh knuckle"]'),
                  'stator clamp': ('leg_shaped', 'leg_shaped.cases()["stator clamp"]')})


def _check_prints(report, label, names_expr, rows_expr):
    """Verify every piece in `names_expr` closes its shell, hangs <= NOISE_CM2 and fits on plates (`rows_expr`)."""
    got: Any = _told('from tools.render import prints as p, solids as s; names = %s; '
                     'told = {n: s.told(n) for n in names}; '
                     'print(json.dumps({"n": len(names), "noise": s.NOISE_CM2, "told": told, '
                     '"fit": p.fitted(%s)}))' % (names_expr, rows_expr))
    told = got.get('told') or {}
    noise = got.get('noise', 0.0)
    over = {n: row['over'] for n, row in told.items() if row['over'] > noise}
    open_ = {n: row['shell'] for n, row in told.items() if row['shell'] >= 0.9}
    report.check('every %s print stands on its bed within 45 degrees and closes its shell' % label,
                 'failed' not in got and len(told) == got.get('n', -1) and not over and not open_,
                 got.get('failed') or {'over': over, 'open': open_})
    fit: Any = got.get('fit') or {'over': ['none told']}
    report.check('every %s print fits within the printer plates' % label,
                 fit.get('over') == [] and fit.get('pieces', 0) > 0, fit)


def test_every_limb_print_stands_and_fits_a_plate(report):
    """Every printed PAHT-CF piece of the legs (`solids.STANDING`) closes its shell, stands <=45 deg and fits plates."""
    _check_prints(report, 'leg', 's.STANDING', 'p.PIECES')


def test_every_upper_print_stands_and_fits_a_plate(report):
    """Every printed PAHT-CF piece of the arms, shoulders, neck and bay (`solids.UPPER`) stands <=45 deg and fits plates."""
    _check_prints(report, 'upper-body', 's.UPPER', 'p.upper()')


ROSTER = (test_the_shoulder_holds_turns_and_keeps_its_coil, test_the_neck_holds_clears_and_keeps_its_hose,
          test_a_mount_is_sized_for_what_it_bore_not_for_a_peak, test_the_trunk_reaches_its_travel_and_prints_standing,
          test_a_print_shaped_by_its_loads_stands_on_its_bed, test_the_hood_room_carries_its_loads,
          test_the_trunk_and_hip_rooms_carry_their_loads, test_the_thigh_and_knee_rooms_carry_their_loads,
          test_every_limb_print_stands_and_fits_a_plate, test_every_upper_print_stands_and_fits_a_plate)


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
