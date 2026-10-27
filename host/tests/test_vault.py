#!/usr/bin/env python3
"""Onto and off a stool over her hips (`machine.bedside`; the kernel's overheight_*.lisp): the texts read whole, her
dip hung from her fists, the seat's edge and the stool under her, her way down to the floor."""
import math
import os
import re
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report
from test_groundwork import LANDMARKS, MOVES, _laid, _part, _skin

#: The texts' landmarks by the segment of hers that carries each.
CARRIED = {**LANDMARKS, 'posterior-femoral': 'thigh'}

#: The overheight texts, machine/ortho_kernel/.
TEXTS = ('overheight_vault_and_dip_core.lisp', 'overheight_vault_specializers.lisp', 'overheight_stool_patches.lisp')
#: The dip's axial range its comment gives (`drive-anterior-overhang-dip-descent`); its column sampled; pitches round
#: a turn; a joint nudged, rad.
DIP, SAMPLES, TURN, NUDGE = (-0.6, 0.95), 41, 7200, 1e-4


def _plan():
    from machine import dsp, kernel
    return dsp.schedule(kernel.read()['compile-overheight-vault-and-dip-soa'])


def _shots():
    """The schedule's gaits: the mount, the dismount."""
    return [seg[1] for seg in _plan() if seg[0] == 'oneshot']


def _calls(shot, stage):
    """The keys of each of a shot's calls of `stage`, in order."""
    return [keys for name, keys in shot['_trace'] if name == stage]


def _dip():
    from machine import kernel
    return kernel.read()['high-vault-1jet-posture-atlas']['postures']['carpal-dip-suspension']


def test_the_texts_read_whole(report):
    """The texts read with the kernel: one posture, its origin, three columns and its hang; every manifold they name a
    patch, every landmark a segment of hers; their stages; the mount, 4 s seated, the dismount."""
    from machine import dsp, kernel
    got, plan = kernel.read(), _plan()
    patches = {**got['anatomical-manifold-registry'], **got['groundwork-manifold-extensions']}
    folder = os.path.join(os.path.dirname(dsp.__file__), 'ortho_kernel')
    texts = []
    for name in TEXTS:
        with open(os.path.join(folder, name), encoding='utf-8') as f:
            texts.append(f.read())
    dip = _dip()
    report.check('one posture: its origin, %d columns, its hang' % len(dip['modal-basis-columns']),
                 list(got['high-vault-1jet-posture-atlas']['postures']) == ['carpal-dip-suspension']
                 and len(dip['modal-basis-columns']) == 3 and dip['gravity-hang-law']['enabled'] is True)
    named = {m for t in texts
             for m in re.findall(r':([a-z0-9-]+-(?:bilateral|continuum|lateral|sphinx))(?![a-z0-9-])', t)}
    marks = {m for t in texts for m in re.findall(r':([a-z0-9-]+)-[lr](?![a-z0-9-])', t)}
    report.check('%d manifolds named, each a patch; landmarks %s, each a segment of hers' % (
        len(named), ', '.join('%s (her %s)' % (m, CARRIED.get(m)) for m in sorted(marks))),
        named <= set(patches) and marks <= set(CARRIED), str(sorted(named - set(patches))))
    stages = [n for t in texts for n in re.findall(r'\(def(?:primitive|modifier|patch) ([a-z0-9-]+)', t)]
    report.check('%d stages' % (len(stages) - 1), all(isinstance(got[n], dsp.Stage) for n in stages
                                                      if n != 'compile-overheight-vault-and-dip-soa'))
    report.check('the schedule: %s' % ', '.join(seg[1]['_trace'][-1][0] if seg[0] == 'oneshot' else '%g s' % seg[1]
                                               for seg in plan),
                 [seg[0] for seg in plan] == ['oneshot', 'dwell', 'oneshot'] and plan[1][1] == 4.0
                 and [g['_trace'][-1][0] for g in _shots()] == ['patch-mount-overheight-stool-with-arm-vault',
                                                                'patch-dismount-overheight-stool-with-arm-dip'])


def _posed(a):
    """Her figure on the dip's origin moved `a` along its axial column."""
    dip = _dip()
    origin, column = dip['origin-q0'], dip['modal-basis-columns']['mode-axial-compression']
    return _laid({j: origin.get(j, 0.0) + a * column.get(j, 0.0) for j in {**origin, **column}})


def _hung(a):
    """Her figure on the dip's origin `a` along its axial column, pitched so her centre stands over her fists - a
    point each, no moment - nearest upright: (deg, the floor's up in her frame, her fists' bottom, her pitch rad)."""
    import numpy as np
    from machine import figure, groundwork
    deg = _posed(a)
    fists = np.mean([np.subtract(figure.fist(deg, side=s), (0.0, figure.FIST_R, 0.0)) for s in ('left', 'right')],
                    axis=0)
    centre = np.subtract(figure.com(deg, (0.0, 0.0, 0.0), figure.IDENT), fists)
    phi = np.linspace(-np.pi, np.pi, TURN + 1)
    up = groundwork.pitched(phi)
    off = np.stack([np.zeros_like(phi), -np.sin(phi), np.cos(phi)], axis=1) @ centre
    cross = np.flatnonzero((np.sign(off[:-1]) != np.sign(off[1:])) & (up[:-1] @ centre > 0.0))
    k = cross[np.argmin(np.abs(phi[cross]))]
    return deg, up[k], fists, phi[k]


def _torque(deg, up, joint, force):
    """N m: her left `joint`'s static torque holding `force` N up at her left fist (virtual work, her pelvis held)."""
    import numpy as np
    from machine import figure
    moved = [np.array(figure.fist(dict(deg, **{'left_' + joint: deg['left_' + joint] + s * math.degrees(NUDGE)})))
             for s in (-1.0, 1.0)]
    return abs(force * float(up @ (moved[1] - moved[0])) / (2.0 * NUDGE))


def _shares():
    """The hands' shares of her weight the vault's anchors take (`inject-adjoint-cone`'s weight laws)."""
    return [c['gain-law'].args['max-share'] for g in _shots() for i, c in g['adjoint-cones'].items()
            if i.startswith('vault-')]


def test_her_dip_hung_from_her_fists(report):
    """The dip's joints hers or ones she lacks, its lockout at her elbow's stop. Her fists bear her centre: hung from
    them she leans back, her pelvis rising from the origin to her lockout by the vault's lift and falling over the
    dip's range by its travel; her shoulder and elbow at the anchors' share of her within their peak, past what they
    hold for ever; her heel hooked bearing its share, her arms the rest."""
    import numpy as np
    from machine import drives, figure, heat, kernel
    from machine.term_rows import ABSENT
    dip, shots = _dip(), _shots()
    origin, column = dip['origin-q0'], dip['modal-basis-columns']['mode-axial-compression']
    lockout = kernel.node('drive-capstan-bevel-arm-vault', 'bevel-vault')['coupled-1jet-vault-trajectory'][
        'lockout-axial-command']
    stop = (math.radians(drives.TRAVEL['elbow'][0]) - origin['cubital']) / column['cubital']
    lacks = sorted(n for n in origin if MOVES[n] in ABSENT)
    report.check('she lacks %s; her elbow\'s %g deg stop at %.3f along the column, the lockout %.2f' % (
        ', '.join(lacks), drives.TRAVEL['elbow'][0], stop, lockout), lacks == ['glenohumeral-abduct']
        and stop <= lockout < stop + 0.01)
    weight, share = figure.mass() * 9.81, max(_shares())
    peak, held = drives.peak('left_elbow'), heat.held('left_elbow')[0]
    rows = []
    for a in np.linspace(lockout, DIP[1], SAMPLES):
        deg, up, fists, phi = _hung(a)
        rows.append((a, math.degrees(phi), -(up @ fists), *(_torque(deg, up, j, 0.5 * share * weight)
                                                          for j in ('shoulder', 'elbow'))))
    rows = np.array(rows)
    height = {a: -(_hung(a)[1] @ _hung(a)[2]) for a in (0.0, lockout, *DIP)}
    lift, travel = height[lockout] - height[0.0], height[DIP[0]] - height[DIP[1]]
    told = (kernel.node('drive-capstan-bevel-arm-vault', 'bevel-vault')['vertical-arch-clearance-m'],
            shots[1]['overhang-dip']['max-vertical-dip-stroke-m'])
    report.check('hung from her fists she leans back %.0f-%.0f deg; her lockout lifts her pelvis %.0f mm from the '
                 'origin, the text %.0f; the dip lowers it %.0f mm, the text %.0f' % (
                     rows[:, 1].min(), rows[:, 1].max(), 1e3 * lift, 1e3 * told[0], 1e3 * travel, 1e3 * told[1]),
                 abs(told[0] - lift) < 5e-4 and abs(told[1] - travel) < 5e-4)
    hook = kernel.node('specialize-vault-with-unilateral-leg-hook', 'adjoint-cones')['unilateral-bevel-leg-hook'][
        'gain-law'].args['max-share']
    most, hooked = rows[:, 3:].max(axis=0), (1.0 - hook) / share * rows[:, 3:].max(axis=0)
    report.check('at the anchors\' %.2f of her: shoulder %.1f, elbow %.1f N m at most, hooked %.1f and %.1f; they peak '
                 'at %.1f, hold %.1f' % (share, *most, *hooked, peak, held),
                 most.max() < peak and drives.peak('left_shoulder') == peak)


def test_the_edge_and_the_stool(report):
    """At the seat's edge her thighs bear 1 - alpha of her (`bedside.unweighted`), all of her far from it; the edge's
    drag as her thighs wrap it from the dip to sitting (`bedside.capstan`). On the patch's stool, its seat at her hips,
    her weight on its seat and that drag at its top (`bedside.cop`) keep its centre of pressure inside its base by the
    cone's margin; the shear that tips it under the text's cap. Her fists reach along the seat past the replants."""
    import numpy as np
    from machine import bedside, figure, kernel
    shots = _shots()
    vault = kernel.node('xform-overheight-arm-vault-ingress', 'bevel-vault')
    alpha, sigma = vault['capstan-edge-unweighting-alpha'], vault['bevel-transition-halfwidth-m']
    s = np.array([0.0, sigma, 4.0 * sigma])
    w = bedside.unweighted(s, alpha, sigma)
    deg, up, _fists, _phi = _hung(0.0)
    placed = figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT)
    thigh = np.subtract(placed['left_shank'][0], placed['left_thigh'][0])
    wrap = math.asin(abs(up @ thigh) / np.linalg.norm(thigh))
    mu = kernel.read()['anatomical-manifold-registry']['posterior-femoral-bilateral']['default-mu']
    weight = figure.mass() * 9.81
    drag = mu * w[0] * weight * float(bedside.capstan(mu, wrap))
    report.check('her thighs bear %s of her at %s mm from the edge; wrapping it %.0f deg, its drag %.1f N' % (
        ', '.join('%.2f' % x for x in w), ', '.join('%.0f' % (1e3 * x) for x in s), math.degrees(wrap), drag),
        abs(w[0] - (1.0 - alpha)) < 1e-12 and w[-1] > 0.999)
    stool = _calls(shots[0], 'specialize-vault-for-light-tippy-stool')[-1]
    cone = shots[0]['fixture-cone']
    base, mass = 1e-3 * stool['base-radius-mm'], stool['estimated-stool-mass-kg']
    rim = 0.5e-3 * _calls(shots[0], 'specialize-vault-hand-anchor-topology')[-1]['seat-width-mm']
    seat = figure.standing() - figure.HIP_DROP
    x, push = bedside.cop(np.array([[0.0, seat], [rim, seat]]),
                          np.array([[0.0, -(1.0 - w[0]) * weight], [-drag, -w[0] * weight]]), mass * 9.81)
    tips = (base - cone['object-floor-zmp-margin-m']) / seat
    cap = cone['axial-pinning-law']['max-shear-to-normal-ratio-on-fixture']
    fist = shots[1]['adjoint-cones']['vault-carpal-flat-anchors']['cop-bounds'][1]
    report.check('her %.0f kg stool, its seat %.3f m at her hips: its centre %.0f mm from its middle, %.0f mm in from '
                 'its base\'s margin; it tips at %.3f of its load in shear, the text caps %.2f; her fists\' pressure '
                 'within %s mm' % (mass, seat, 1e3 * x, 1e3 * (base - cone['object-floor-zmp-margin-m'] - abs(x)), tips,
                                   cap, fist),
                 abs(x) < base - cone['object-floor-zmp-margin-m'] and push > weight
                 and fist == (-1e3 * figure.FIST_R, 1e3 * figure.FIST_R))
    sat = _laid(kernel.read()['canonical-posture-soa-atlas']['postures']['ischial-seated']['origin-q0'])
    points, radii, of, _c = _skin(sat)
    floor = min(p[1] - r for p, r, o in zip(points, radii, of) if _part(o) == 'pelvis')
    straight = dict(sat, left_elbow=0.0, left_shoulder=0.0)
    shoulder = figure.frames(straight, (0.0, 0.0, 0.0), figure.IDENT)['left_upper_arm'][0]
    arm = np.linalg.norm(np.subtract(figure.fist(straight), shoulder)) + figure.FIST_R
    reach = math.sqrt(arm ** 2 - (shoulder[1] - floor) ** 2)
    steps = [k['hand-step-back-mm'] for g in shots for k in _calls(g, 'ratchet-carpal-seat-replant')]
    report.check('seated her fists reach along the seat %.0f mm either side of her shoulders; the replants %s mm' % (
        1e3 * reach, ', '.join('%.0f' % x for x in sorted(set(steps)))), 1e-3 * max(steps) < 2.0 * reach)


def test_her_way_down(report):
    """Sat on a seat her hip is over it; her forefeet take her at the handover's reach (rho of her leg): without the
    dip she steps down from no higher than that less her hip, dipped from that less her hip at the dip's end - from
    there undipped a drop."""
    import numpy as np
    from machine import figure, kernel, somatic
    shots = _shots()
    sat = _laid(kernel.read()['canonical-posture-soa-atlas']['postures']['ischial-seated']['origin-q0'])
    points, radii, of, _c = _skin(sat)
    hip = figure.frames(sat, (0.0, 0.0, 0.0), figure.IDENT)['left_thigh'][0][1] - min(
        p[1] - r for p, r, o in zip(points, radii, of) if _part(o) == 'pelvis')
    rho = shots[1]['overhang-dip']['ground-capture-handover']['require-reachability-rho-below']
    reach = rho * somatic.LEG_M
    deg, up, fists, _phi = _hung(DIP[1])
    dipped = up @ np.subtract(figure.frames(deg, (0.0, 0.0, 0.0), figure.IDENT)['left_thigh'][0], fists)
    drop = hip - dipped
    report.check('her hip %.0f mm over the seat, %.0f at the dip\'s end; her forefeet take her at %.3f m (rho %.2f): '
                 'she steps down from %.3f m, dipped from %.3f - undipped a %.0f mm drop, %.2f m/s, %.0f J' % (
                     1e3 * hip, 1e3 * dipped, reach, rho, reach - hip, reach - dipped, 1e3 * drop,
                     math.sqrt(2.0 * 9.81 * drop), figure.mass() * 9.81 * drop), drop > 0.0)


ROSTER = [test_the_texts_read_whole, test_her_dip_hung_from_her_fists, test_the_edge_and_the_stool, test_her_way_down]


def main(argv=None):
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
