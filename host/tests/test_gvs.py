#!/usr/bin/env python3
"""Tests for invariant kinematic evaluation and CI guillotine on SO(3).

    python -X utf8 host/tests/test_gvs.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np

from gynoid_kit import Report
from tools.sim import gvs


def test_hof_scaling_axiom_strips_dimensions(report):
    """Hof's Scaling Axiom: system mass M, leg rest length L0, and gravity g are
    systematically eliminated, yielding a dimensionless analytical space."""
    mass, leg, g = 55.0, 0.77, 9.81
    scale = gvs.hof_scaling(mass=mass, leg=leg, g=g)
    t0_expected = math.sqrt(leg / g)
    v0_expected = math.sqrt(g * leg)
    f0_expected = mass * g
    tau0_expected = mass * g * leg
    om0_expected = math.sqrt(g / leg)

    report.check('time normalizer T0 is sqrt(L0/g)',
                 abs(scale['t0'] - t0_expected) < 1e-9,
                 't0 = %.4f s' % scale['t0'])
    report.check('force normalizer F0 is Mg',
                 abs(scale['f0'] - f0_expected) < 1e-9,
                 'F0 = %.2f N' % scale['f0'])
    report.check('torque normalizer tau0 is MgL0',
                 abs(scale['tau0'] - tau0_expected) < 1e-9,
                 'tau0 = %.2f N m' % scale['tau0'])
    report.check('dimensionless natural frequency is exactly 1.0',
                 abs(scale['om0'] * scale['t0'] - 1.0) < 1e-9,
                 'om0 * t0 = %.4f' % (scale['om0'] * scale['t0']))

    # Telemetry stripping
    times = [0.0, 0.5, 1.0]
    forces = [0.0, f0_expected, 2.0 * f0_expected]
    torques = [0.0, tau0_expected, -tau0_expected]
    stripped = gvs.strip_hardware(times, forces=forces, torques=torques, mass=mass, leg=leg, g=g)

    report.check('reaction forces normalized to Mg: 0, 1, 2',
                 np.allclose(stripped['f_star'], [0.0, 1.0, 2.0]),
                 '%s' % stripped['f_star'])
    report.check('joint torques normalized to MgL0: 0, 1, -1',
                 np.allclose(stripped['tau_star'], [0.0, 1.0, -1.0]),
                 '%s' % stripped['tau_star'])


def test_temporal_rigidity_capture_point_phase(report):
    """Temporal Rigidity: clock time and DTW are prohibited; synchronization is parameterized
    solely over forward Capture Point progression xi* = c*_along + v*_along."""
    times = np.linspace(0.0, 2.0, 100)
    leg, g = 0.77, 9.81
    scale = gvs.hof_scaling(leg=leg, g=g)

    # Steady progression: forward speed v = 0.6 m/s
    v_const = 0.6
    com_steady = v_const * times
    tau, xi_steady = gvs.capture_point(times, com_steady, leg=leg, g=g)
    v_star = v_const / scale['v0']

    report.check('dimensionless capture point is xi* = c* + v*',
                 np.allclose(xi_steady, com_steady / leg + v_star, atol=1e-3),
                 'xi*[0] = %.3f, xi*[-1] = %.3f' % (xi_steady[0], xi_steady[-1]))

    # Hesitation / stall: velocity drops to zero halfway through
    com_stalled = np.piecewise(times, [times < 1.0, times >= 1.0],
                               [lambda t: v_const * t, lambda t: v_const * 1.0])
    _, xi_stalled = gvs.capture_point(times, com_stalled, leg=leg, g=g)
    phase_stalled = gvs.phase_from_capture_point(xi_stalled)

    # In the second half, forward progress stalls, so mechanical phase cannot advance
    phase_delta_first_half = phase_stalled[49] - phase_stalled[0]
    phase_delta_second_half = phase_stalled[-1] - phase_stalled[50]

    report.check('solver hesitation freezes phase progression: second half progress is ~0',
                 phase_delta_first_half > 0.8 and phase_delta_second_half < 0.2,
                 'first half: %.2f, second half: %.2f' % (phase_delta_first_half, phase_delta_second_half))


def test_so3_geodesic_distance_lie_log_map(report):
    """Geodesic distance on SO(3) via Lie algebra logarithmic map:
    Delta theta = || log(R_norm^T R_robot) ||^vee = arccos((tr(R_norm^T R_robot) - 1) / 2)."""
    # 1. Identity / zero error
    r_eye = np.eye(3)
    d0 = gvs.geodesic_distance_so3(r_eye, r_eye)
    report.check('distance between identical rotations is 0.0',
                 abs(d0) < 1e-12, 'd0 = %g' % d0)

    # 2. Rotation around arbitrary axis by angle alpha
    axis = np.array([1.0, 2.0, 3.0])
    axis /= np.linalg.norm(axis)
    angles = [0.1, 0.5, 1.2, math.pi / 2, 2.5]
    errors = []
    for alpha in angles:
        # Rodrigues formula
        k_mat = np.array([[0, -axis[2], axis[1]],
                          [axis[2], 0, -axis[0]],
                          [-axis[1], axis[0], 0]])
        r_rot = np.eye(3) + math.sin(alpha) * k_mat + (1 - math.cos(alpha)) * (k_mat @ k_mat)
        d = gvs.geodesic_distance_so3(r_eye, r_rot)
        errors.append(abs(d - alpha))

    report.check('Lie log map distance matches rotation angle exactly on SO(3)',
                 max(errors) < 1e-6,
                 'max error = %g rad' % max(errors))


def test_gvs_and_gps_aggregation(report):
    """GVS RMS error per joint and aggregated Gait Profile Score (GPS)."""
    ref_path = os.path.join(os.path.dirname(__file__), 'takes', 'walk.fbx')
    report.check('reference take walk.fbx exists', os.path.isfile(ref_path))

    ref_times, ref_joints, ref_rots = gvs.extract_take_rotations(ref_path)
    ref_com = ref_joints['Hips'][:, 2]

    # Evaluate walk.fbx against itself
    gvs_dict, gps_score = gvs.compute_gvs(ref_times, ref_com, ref_rots,
                                          ref_times, ref_com, ref_rots)

    report.check('self-evaluation GVS values are all under 3.5 deg',
                 all(score < math.radians(3.5) for score in gvs_dict.values()),
                 ', '.join('%s: %.2f deg' % (k, math.degrees(v)) for k, v in gvs_dict.items()))
    report.check('self-evaluation GPS is under 3.0 deg',
                 gps_score < math.radians(3.0),
                 'GPS = %.2f deg' % math.degrees(gps_score))


def test_ci_guillotine_binary_gate(report):
    """CI Guillotine Binary Gate: if GPS > MAX_TOLERANCE exit(1)."""
    # Passing score
    pass_gvs = {'Hips': math.radians(2.0), 'LeftLeg': math.radians(3.0)}
    pass_gps = math.radians(2.5)
    code_pass = gvs.report_and_gate(pass_gvs, pass_gps, tolerance=math.radians(15.0))
    report.check('passing score (2.5 deg <= 15 deg) returns exit code 0',
                 code_pass == 0, 'code = %d' % code_pass)

    # Failing score (trips guillotine)
    fail_gvs = {'Hips': math.radians(18.0), 'LeftLeg': math.radians(25.0)}
    fail_gps = math.radians(21.8)
    code_fail = gvs.report_and_gate(fail_gvs, fail_gps, tolerance=math.radians(15.0))
    report.check('failing score (21.8 deg > 15 deg) trips guillotine and returns exit code 1',
                 code_fail == 1, 'code = %d' % code_fail)


def test_rig_invariant_anatomical_frames_and_capture_gates(report):
    """Rig-invariant anatomical SO(3) frames and dimensionless Capture Point invariant gates."""
    ref_path = os.path.join(os.path.dirname(__file__), 'takes', 'walk.fbx')
    ref_times, ref_joints, ref_rots = gvs.extract_take_rotations(ref_path)
    com_along, v_along, anat_rots, leg = gvs.anatomical_frames(ref_times, ref_joints)

    # Every anatomical frame is a proper orthogonal rotation matrix in SO(3)
    all_so3 = all(
        np.allclose( np.matmul(r.transpose(0, 2, 1), r), np.eye(3), atol=1e-5)
        and np.allclose(np.linalg.det(r), 1.0, atol=1e-5)
        for r in anat_rots.values()
    )
    report.check('anatomical_frames constructs proper SO(3) matrices (R^T R = I, det R = +1)',
                 all_so3 and leg > 0.4 and len(v_along) == len(ref_times),
                 'leg = %.3f m, joints = %d' % (leg, len(anat_rots)))

    metrics, violations = gvs.capture_gates(ref_times, com_along, ref_joints, leg=leg, gps_rad=0.05)
    report.check('reference mocap walk passes all dimensionless Capture Point gates',
                 not violations and metrics['step_star'] >= 0.45 and metrics['walk_ratio'] >= 1.0,
                 'step* = %.3f, wr = %.2f, violations = %s' % (
                     metrics['step_star'], metrics['walk_ratio'], violations))


def test_data_oriented_ballistic_wire_and_module_addressing(report):
    """Data-oriented addressing in `tools.sim.ballistic` across flat `<4I59f` wire rows,
    behaviour phases, and branchless module constants."""
    from machine import impedance, ortho, sprung, stride
    from tools.sim import ballistic
    from tools.sim import ortho as sim_ortho

    plan = sim_ortho.planned(['%s@%g' % entry for entry in sprung.WALK] + ['ballistic_walk@30'])
    # `#204` is entry 3, row 0; `amp.2` (sagittal_drive) is column 4 + ortho.AMP + 2
    col_drive = 4 + ortho.AMP + 2
    sym_val = ballistic.now(plan, '#204.amp.2')
    raw_val = ballistic.now(plan, 'wire[3,0,%d]' % col_drive)
    report.check('flat wire[e,r,c] and symbolic #204.amp.2 address the exact same float cell',
                 abs(sym_val - raw_val) < 1e-6 and abs(sym_val - 0.6) < 1e-6,
                 'sym = %.3f, raw = %.3f' % (sym_val, raw_val))

    snap = ballistic._snapshot()
    try:
        mutated = ballistic.put(plan, {
            'wire[3,0,%d]' % col_drive: 0.72,
            '#203.blend': 0.55,
            'ballistic_walk.apa.amp.2': 0.25,
            'stride.AHEAD_M_S': 0.31,
            'impedance.GOING_S': 0.52,
            'stride.SWING': 0.38,
        })
        report.check('put() mutates wire rows, behaviour phases, and synced module constants',
                     abs(ballistic.now(mutated, '#204.amp.2') - 0.72) < 1e-6
                     and abs(ballistic.now(mutated, '#203.blend') - 0.55) < 1e-6
                     and abs(ballistic.now(mutated, 'ballistic_walk.apa.amp.2') - 0.25) < 1e-6
                     and abs(stride.AHEAD_M_S - 0.31) < 1e-9
                     and abs(impedance.GOING_S - 0.52) < 1e-9
                     and abs(stride.PHI_W - math.pi * 0.38) < 1e-9,
                     'PHI_W = %.4f' % stride.PHI_W)
    finally:
        ballistic._restore(snap)

    report.check('_restore() cleanly restores all module constants',
                 abs(stride.AHEAD_M_S - 0.36) < 1e-9 and abs(impedance.GOING_S - 0.28) < 1e-9,
                 'AHEAD_M_S = %.2f, GOING_S = %.2f' % (stride.AHEAD_M_S, impedance.GOING_S))

    cands = ballistic.lhs_candidates({'a': (0.0, 1.0), 'b': (10.0, 20.0)}, 8, seed=11)
    bins_a = sorted(int(c['a'] * 8) for c in cands)
    bins_b = sorted(int((c['b'] - 10.0) / 10.0 * 8) for c in cands)
    report.check('Latin Hypercube sample stratifies every parameter axis into N equal bins',
                 bins_a == list(range(8)) and bins_b == list(range(8)),
                 'bins_a = %s' % bins_a)


def test_initiation_gates_reject_swaying_baby_steps_and_stiff_arms(report):
    """Invariant gates must fail when gait initiation hesitates/sways in place, takes
    tripping baby steps, or holds arms and feet stiff instead of stepping ballistically."""
    ref_path = os.path.join(os.path.dirname(__file__), 'takes', 'walk.fbx')
    ref_times, ref_joints, _ = gvs.extract_take_rotations(ref_path)
    _, _, _, leg = gvs.anatomical_frames(ref_times, ref_joints)

    # Construct a hesitant trajectory that stands & sways for 5s, then takes tiny steps with stiff arms
    n_stand = 150
    t_stand = np.linspace(0.0, 5.0, n_stand, endpoint=False)
    t_walk = 5.0 + ref_times
    times = np.concatenate([t_stand, t_walk])
    bad_joints = {}
    h0 = ref_joints['Hips'][:1]
    hips_moved = ref_joints['Hips'].copy()
    hips_moved[:, [0, 2]] = h0[:, [0, 2]] + 0.20 * (ref_joints['Hips'][:, [0, 2]] - h0[:, [0, 2]])
    for k, pts in ref_joints.items():
        if not isinstance(pts, np.ndarray) or pts.ndim < 2:
            bad_joints[k] = pts
            continue
        first_row = np.repeat(pts[:1], n_stand, axis=0)
        moved = pts.copy()
        # Compress horizontal displacement to baby steps (20% of normal)
        moved[:, [0, 2]] = h0[:, [0, 2]] + 0.20 * (pts[:, [0, 2]] - h0[:, [0, 2]])
        if k in ('LeftArm', 'RightArm', 'LeftForeArm', 'RightForeArm'):
            # Pin arms stiffly relative to hips
            moved = hips_moved + (pts[:1] - h0)
        if k in ('LeftToeBase', 'RightToeBase'):
            ankle = 'LeftFoot' if k.startswith('Left') else 'RightFoot'
            ankle_moved = ref_joints[ankle].copy()
            ankle_moved[:, [0, 2]] = h0[:, [0, 2]] + 0.20 * (ref_joints[ankle][:, [0, 2]] - h0[:, [0, 2]])
            moved = ankle_moved + (pts[:1] - ref_joints[ankle][:1])
        bad_joints[k] = np.concatenate([first_row, moved], axis=0)

    com_bad = bad_joints['Hips'][:, 2]
    metrics, violations = gvs.capture_gates(times, com_bad, bad_joints, leg=leg, gps_rad=math.radians(18.0))
    broken_names = {name for name, _v, _b in violations}
    report.check('swaying 5s before stepping fails init_tau gate (> 6.5)',
                 'init_tau' in broken_names and metrics['init_tau'] > 6.5,
                 'init_tau = %.2f' % metrics['init_tau'])
    report.check('baby steps fail first_step_star and step_star gates',
                 'first_step_star' in broken_names and 'step_star' in broken_names,
                 'first_step* = %.3f, step* = %.3f' % (metrics['first_step_star'], metrics['step_star']))
    report.check('stiff arms and flat feet fail arm_deg and heel_deg gates',
                 'arm_deg' in broken_names and 'heel_deg' in broken_names,
                 'arm = %.1f deg, heel = %.1f deg' % (metrics['arm_deg'], metrics['heel_deg']))

    pass_gvs = {'Hips': math.radians(5.0)}
    code = gvs.report_and_gate(pass_gvs, math.radians(5.0), tolerance=math.radians(15.0),
                               metrics=metrics, violations=violations)
    report.check('report_and_gate trips CI guillotine (returns 1) on invariant gate violations even if GPS <= 15 deg',
                 code == 1, 'code = %d' % code)


def test_limping_and_transition_artifacts_rejected(report):
    """Invariant gates must detect and reject transition artifacts: aborted first step in place,
    knee hyperextension (< -0.5 deg), and limping gait (bilateral foot lift asymmetry > 0.12)."""
    ref_path = os.path.join(os.path.dirname(__file__), 'takes', 'walk.fbx')
    ref_times, ref_joints, _ = gvs.extract_take_rotations(ref_path)
    _, _, _, leg = gvs.anatomical_frames(ref_times, ref_joints)

    # Construct a trajectory with:
    # 1) an aborted first step in place (left foot lifts 6 cm and lands at the exact same spot)
    # 2) asymmetric foot clearance (right foot barely lifts -> limping gait)
    # 3) knee hyperextension on the stance leg (left knee pushed backward behind hip-ankle line)
    n_abort = 30
    t_abort = np.linspace(0.0, 0.5, n_abort, endpoint=False)
    times = np.concatenate([t_abort, 0.5 + ref_times])
    limp_joints = {}
    for k, pts in ref_joints.items():
        if not isinstance(pts, np.ndarray) or pts.ndim < 2:
            limp_joints[k] = pts
            continue
        abort_seg = np.repeat(pts[:1], n_abort, axis=0)
        if k in ('LeftFoot', 'LeftToeBase'):
            # Lift 6 cm in place and land back down at the same horizontal position
            lift_arc = 0.06 * np.sin(np.linspace(0.0, math.pi, n_abort))
            abort_seg[:, 1] += lift_arc
        walk_seg = pts.copy()
        if k in ('RightFoot', 'RightToeBase'):
            # Suppress right foot vertical clearance to 25% of normal (limping asymmetry)
            y_min = np.min(walk_seg[:, 1])
            walk_seg[:, 1] = y_min + 0.25 * (walk_seg[:, 1] - y_min)
        if k == 'LeftLeg':
            # Push left knee 6 cm backward (-Z) behind hip-ankle line to induce hyperextension (< -0.5 deg)
            walk_seg[:25, 2] -= 0.06
        limp_joints[k] = np.concatenate([abort_seg, walk_seg], axis=0)

    com_limp = limp_joints['Hips'][:, 2]
    metrics, violations = gvs.capture_gates(times, com_limp, limp_joints, leg=leg, gps_rad=math.radians(8.0))
    broken_names = {name for name, _v, _b in violations}
    report.check('aborted first step in place fails first_step_star gate (< 0.25)',
                 'first_step_star' in broken_names and metrics['first_step_star'] < 0.08,
                 'first_step* = %.3f' % metrics['first_step_star'])
    report.check('knee hyperextension fails knee_min_deg gate (< -0.5 deg)',
                 'knee_min_deg' in broken_names and metrics['knee_min_deg'] < -0.5,
                 'knee_min = %.2f deg' % metrics['knee_min_deg'])
    report.check('limping foot clearance asymmetry fails lift_asym gate (> 0.12)',
                 'lift_asym' in broken_names and metrics['lift_asym'] > 0.12,
                 'lift_asym = %.3f' % metrics['lift_asym'])


ROSTER = [test_hof_scaling_axiom_strips_dimensions,
          test_temporal_rigidity_capture_point_phase,
          test_so3_geodesic_distance_lie_log_map,
          test_gvs_and_gps_aggregation,
          test_ci_guillotine_binary_gate,
          test_rig_invariant_anatomical_frames_and_capture_gates,
          test_data_oriented_ballistic_wire_and_module_addressing,
          test_initiation_gates_reject_swaying_baby_steps_and_stiff_arms,
          test_limping_and_transition_artifacts_rejected]


def main():
    report = Report()
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
