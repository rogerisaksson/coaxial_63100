#!/usr/bin/env python3
"""Tests for invariant kinematic evaluation and CI guillotine on SO(3).

    python -X utf8 host/tests/test_gvs.py
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np  # noqa: E402

from gynoid_kit import Report  # noqa: E402
from tools.sim import gvs  # noqa: E402


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


ROSTER = [test_hof_scaling_axiom_strips_dimensions,
          test_temporal_rigidity_capture_point_phase,
          test_so3_geodesic_distance_lie_log_map,
          test_gvs_and_gps_aggregation,
          test_ci_guillotine_binary_gate]


def main():
    report = Report()
    for test in ROSTER:
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
