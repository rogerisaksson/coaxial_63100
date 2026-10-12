#!/usr/bin/env python3
"""Invariant kinematic evaluation and CI guillotine on SO(3).

    python host/tools/sim/gvs.py --eval               # evaluate simulated walk against walk.fbx
    python host/tools/sim/gvs.py --record RECORD.csv  # evaluate recorded walk against walk.fbx
    python host/tools/sim/gvs.py TAKE.fbx [REF.fbx]   # evaluate FBX take against reference

Hof's Scaling Axiom:
    System mass M, leg rest length L0, and gravity g are systematically eliminated.
    tau = t / sqrt(L0/g), F* = F / (Mg), tau* = tau / (MgL0), r* = r / L0.

Temporal Rigidity (Capture Point Phase Variable):
    Clock time and Dynamic Time Warping (DTW) are strictly prohibited.
    Synchronization against the biological reference is parameterized solely over
    forward Capture Point progression: xi* = c*_along + v*_along.

Geodesic Diff Engine on SO(3):
    Delta theta_i = || log(R_norm,i^T R_robot,i) ||^vee = arccos((tr(R_norm,i^T R_robot,i) - 1) / 2)
    GVS_i = RMS(Delta theta_i) over the gait cycle.

Falsifiable Aggregation & CI Guillotine:
    GPS = sqrt(mean(GVS_i^2))
    if GPS > MAX_TOLERANCE: sys.exit(1)
"""
import argparse
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

import numpy as np  # noqa: E402

from machine import figure  # noqa: E402
from tools.sim import fbx  # noqa: E402

G = 9.81
L0_DEFAULT = figure.THIGH + figure.SHANK  # 0.77 m
MASS_DEFAULT = figure.MASS_KG             # 55.0 kg
MAX_TOLERANCE_DEG = 15.0                  # Maximum allowable GPS in degrees
MAX_TOLERANCE_RAD = math.radians(MAX_TOLERANCE_DEG)

#: Key joints evaluated for clinical Gait Profile Score (GPS)
EVAL_JOINTS = (
    'Hips',
    'LeftUpLeg',
    'LeftLeg',
    'LeftFoot',
    'RightUpLeg',
    'RightLeg',
    'RightFoot',
)

#: All available HumanIK joints mapped from figure.SEGMENTS
ALL_JOINTS = tuple(fbx.bone(seg[0]) for seg in figure.SEGMENTS)


def hof_scaling(mass=MASS_DEFAULT, leg=L0_DEFAULT, g=G):
    """(t0, v0, f0, tau0, om0): Hof's dimensionless scaling factors."""
    t0 = math.sqrt(leg / g)
    v0 = math.sqrt(g * leg)
    f0 = mass * g
    tau0 = mass * g * leg
    om0 = math.sqrt(g / leg)
    return {'t0': t0, 'v0': v0, 'f0': f0, 'tau0': tau0, 'om0': om0,
            'mass': mass, 'leg': leg, 'g': g}


def strip_hardware(times, places=None, velocities=None, forces=None, torques=None,
                   mass=MASS_DEFAULT, leg=L0_DEFAULT, g=G):
    """Strip physical context from telemetry data using Hof's scaling axiom.
    Returns dimensionless dictionary: tau, r*, v*, F*, tau*."""
    scale = hof_scaling(mass, leg, g)
    out = {'tau': np.asarray(times, float) / scale['t0'], 'scale': scale}
    if places is not None:
        out['r_star'] = np.asarray(places, float) / scale['leg']
    if velocities is not None:
        out['v_star'] = np.asarray(velocities, float) / scale['v0']
    if forces is not None:
        out['f_star'] = np.asarray(forces, float) / scale['f0']
    if torques is not None:
        out['tau_star'] = np.asarray(torques, float) / scale['tau0']
    return out


def capture_point(times, com_along, vcom_along=None, leg=L0_DEFAULT, g=G):
    """Dimensionless forward Capture Point progression xi* = c*_along + v*_along.
    If vcom_along is not supplied, it is computed via numeric gradient."""
    times = np.asarray(times, float)
    com = np.asarray(com_along, float)
    scale = hof_scaling(leg=leg, g=g)
    tau = times / scale['t0']
    c_star = com / scale['leg']
    if vcom_along is None:
        v_star = np.gradient(c_star, tau)
    else:
        v_star = np.asarray(vcom_along, float) / scale['v0']
    # In dimensionless Hof units, om0* = 1.0, hence xi* = c* + v*
    return tau, c_star + v_star


def phase_from_capture_point(xi_star, window_samples=None):
    """Parameterize phase theta in [0, 1) strictly over forward Capture Point progression.
    No clock time, no Dynamic Time Warping (DTW).
    Identifies step/stride periods from monotonic progression and unwraps phase."""
    n = len(xi_star)
    if n < 4:
        return np.linspace(0.0, 1.0, n, endpoint=False)
    # Total progress of Capture Point over the sequence
    total_progress = xi_star[-1] - xi_star[0]
    if abs(total_progress) < 1e-6:
        return np.zeros(n)
    # Monotonic normalized phase over forward mechanical progression
    progress = (xi_star - xi_star[0]) / total_progress
    return np.clip(progress, 0.0, 1.0)


def geodesic_distance_so3(r1, r2):
    """Geodesic distance on SO(3) via Lie algebra logarithmic map:
        Delta theta = || log(R1^T R2) ||^vee = arccos(clip((tr(R1^T R2) - 1) / 2, -1, 1))
    Shapes: r1, r2 of shape (3, 3) or (N, 3, 3).
    Returns scalar or (N,) array of geodesic distances in radians."""
    r1, r2 = np.asarray(r1, float), np.asarray(r2, float)
    if r1.ndim == 2:
        m = r1.T @ r2
        tr = np.trace(m)
        cos_angle = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
        return float(np.arccos(cos_angle))
    # Batch computation (N, 3, 3)
    # m = r1^T @ r2: (N, 3, 3)
    m = np.einsum('nij,njk->nik', np.transpose(r1, (0, 2, 1)), r2)
    tr = np.trace(m, axis1=1, axis2=2)
    cos_angle = np.clip((tr - 1.0) / 2.0, -1.0, 1.0)
    return np.arccos(cos_angle)


def extract_take_rotations(path):
    """(times, joints_places, rotations_dict) from an FBX take.
    rotations_dict maps joint name -> (N, 3, 3) SO(3) orientation matrices."""
    with open(path, 'rb') as f:
        data = f.read()
    top = {n[0]: n for n in fbx.nodes(data)}
    settings = fbx._props(top['GlobalSettings'])
    metres = float(settings.get('UnitScaleFactor', [1.0])[0]) / 100.0
    up = int(settings.get('UpAxis', [1])[0])
    objects = {n[1][0]: n for n in top['Objects'][2]}
    models = {i: n for i, n in objects.items() if n[0] == 'Model'}
    parent, channel, curve = {}, {}, {}
    for c in top['Connections'][2]:
        kind, a, b = c[1][:3]
        what = objects.get(a, ('',))[0]
        if kind == 'OO' and a in models and b in models:
            parent[a] = b
        elif kind == 'OP' and what == 'AnimationCurveNode' and b in models:
            channel[(b, c[1][3])] = a
        elif kind == 'OP' and what == 'AnimationCurve':
            curve[(b, c[1][3][-1])] = a
    keys = {}
    for (node, axis), i in curve.items():
        keys[(node, axis)] = (fbx._keys(objects[i], 'KeyTime') / float(fbx.KTIME),
                              fbx._keys(objects[i], 'KeyValueFloat').astype(float))
    times = np.unique(np.concatenate([t for t, _v in keys.values()]))

    def lcl(i, name):
        node, rest = channel.get((i, name)), fbx._props(models[i]).get(name, [0.0, 0.0, 0.0])
        if node is not None:
            given = fbx._props(objects[node])
            rest = [given.get('d|' + a, [rest[k]])[0] for k, a in enumerate('XYZ')]
        return np.stack([np.interp(times, *keys[(node, a)]) if (node, a) in keys
                         else np.full(len(times), float(rest[j]))
                         for j, a in enumerate('XYZ')], 1)

    placed = {}

    def place(i):
        if i not in placed:
            props = fbx._props(models[i])
            pre = fbx._turns(np.array([props.get('PreRotation', [0, 0, 0])], float), 'xyz')
            post = fbx._turns(np.array([props.get('PostRotation', [0, 0, 0])], float), 'xyz')
            order = fbx.ORDERS[int(props.get('RotationOrder', [0])[0])]
            turn = pre @ fbx._turns(lcl(i, 'Lcl Rotation'), order) @ np.transpose(post, (0, 2, 1))
            at = lcl(i, 'Lcl Translation')
            if i in parent:
                up_turn, up_at = place(parent[i])
                turn, at = up_turn @ turn, np.einsum('nij,nj->ni', up_turn, at) + up_at
            placed[i] = (turn, at)
        return placed[i]

    joints_places = {}
    rotations_dict = {}
    for i, n in models.items():
        name = n[1][1].split('\x00')[0].split(':')[-1]
        turn, at = place(i)
        joints_places[name] = at * metres
        rotations_dict[name] = turn

    for name in list(joints_places):
        key = name.lower().replace(' ', '').replace('_', '')
        key = next((key[len(lead):] for lead in ('ccbase', 'bip001', 'bip01', 'bip')
                    if key.startswith(lead)), key)
        std_name = fbx.RIGS.get(key, name)
        if std_name not in joints_places:
            joints_places[std_name] = joints_places[name]
            rotations_dict[std_name] = rotations_dict[name]

    if up == 2:
        joints_places = {k: np.stack([p[:, 0], p[:, 2], -p[:, 1]], 1)
                         for k, p in joints_places.items()}
    return times, joints_places, rotations_dict


def sample_rotations_at_phase(ref_phases, ref_rotations, target_phases):
    """Sample SO(3) rotations at target phases strictly along mechanical phase."""
    n_targets = len(target_phases)
    out = np.zeros((n_targets, 3, 3))
    # Find nearest or interpolated orientation along the phase progression
    indices = np.clip(np.searchsorted(ref_phases, target_phases), 0, len(ref_phases) - 1)
    for k, idx in enumerate(indices):
        out[k] = ref_rotations[idx]
    return out


def canonical_stride(ref_times, ref_com, ref_rots, joints=EVAL_JOINTS, leg=L0_DEFAULT):
    """Extract canonical steady-state gait cycle (phase in [0, 1) and rotations) from reference."""
    _, xi_star = capture_point(ref_times, ref_com, leg=leg)
    n = len(ref_times)
    # Canonical steady stride in walk.fbx is frames 143 to 213 (69-70 frames, ~1.18 s)
    if n >= 220:
        k0, k1 = 143, 213
    else:
        k0, k1 = 0, n - 1
    xi_stride = xi_star[k0:k1 + 1]
    d_xi = float(xi_stride[-1] - xi_stride[0])
    phase = (xi_stride - xi_stride[0]) / (d_xi if abs(d_xi) > 1e-6 else 1.0)
    rots = {j: ref_rots[j][k0:k1 + 1] for j in joints if j in ref_rots}
    return phase, rots, d_xi


def align_initial_phase(phase_canon, canon_rots, xi_rob, rob_rots, d_xi, grid_size=50):
    """Find the single initial phase offset theta0 in [0, 1) that aligns the initial contact.
    No time warping is performed; this is an invariant origin alignment on the circle S^1."""
    best_theta0, best_err = 0.0, float('inf')
    n_sample = min(len(xi_rob), 70)
    for shift in np.linspace(0.0, 1.0, grid_size, endpoint=False):
        phase_test = ((xi_rob[:n_sample] - xi_rob[0]) / d_xi + shift) % 1.0
        err = 0.0
        for j in ('Hips', 'LeftUpLeg'):
            if j in canon_rots and j in rob_rots:
                r_ref = sample_rotations_at_phase(phase_canon, canon_rots[j], phase_test)
                err += float(np.mean(geodesic_distance_so3(r_ref, rob_rots[j][:n_sample])))
        if err < best_err:
            best_err, best_theta0 = err, shift
    return best_theta0


def compute_gvs(ref_times, ref_com, ref_rots, robot_times, robot_com, robot_rots,
                joints=EVAL_JOINTS, leg=L0_DEFAULT):
    """Compute Gait Variable Score (GVS) for each joint and aggregate Gait Profile Score (GPS).
    Returns (gvs_dict, gps_score)."""
    # 1. Extract canonical reference stride and parameterize via Capture Point progression
    phase_canon, canon_rots, d_xi = canonical_stride(ref_times, ref_com, ref_rots, joints, leg)
    _, xi_rob = capture_point(robot_times, robot_com, leg=leg)
    theta0 = align_initial_phase(phase_canon, canon_rots, xi_rob, robot_rots, d_xi)
    phase_rob = ((xi_rob - xi_rob[0]) / (d_xi if abs(d_xi) > 1e-6 else 1.0) + theta0) % 1.0

    gvs = {}
    for joint in joints:
        if joint not in canon_rots or joint not in robot_rots:
            continue
        r_ref = canon_rots[joint]
        r_rob = robot_rots[joint]
        # Look up reference rotation at robot's current Capture Point phase
        r_ref_synced = sample_rotations_at_phase(phase_canon, r_ref, phase_rob)
        # Geodesic difference on SO(3)
        delta_theta = geodesic_distance_so3(r_ref_synced, r_rob)
        # GVS is RMS error in radians
        rms = float(np.sqrt(np.mean(delta_theta ** 2)))
        gvs[joint] = rms

    if not gvs:
        return {}, float('inf')

    # GPS is quadratic mean of all joint GVS values
    gps = float(np.sqrt(np.mean([val ** 2 for val in gvs.values()])))
    return gvs, gps


def evaluate_simulation(ref_path, duration=12.0, joints=EVAL_JOINTS, tolerance=MAX_TOLERANCE_RAD):
    """Run simulation on the latent loop, evaluate against reference FBX take,
    execute CI guillotine check."""
    from machine import sprung
    from machine.errors import MachineError
    from tools.sim import ortho as run
    ref_times, ref_joints, ref_rots = extract_take_rotations(ref_path)
    ref_com = ref_joints['Hips'][:, 2]  # Ahead along Z in walk.fbx

    plan = run.planned(['#201@0.2', '#204@0.5'])
    g = sprung.laid(sprung.STACK_EVERY)
    for at, label, wire, names in sorted(plan, key=lambda item: item[0]):
        sprung.said(g, label, wire, names, at)

    world = g['world']
    d = world.data
    robot_times = []
    robot_com = []
    robot_rots = {j: [] for j in joints}

    walk_start = 1.5  # Steady walking phase directly after APA initiation
    while g['t'] < walk_start + duration and sprung.tilt(g) <= sprung.DOWN_DEG:
        try:
            sprung.passed(g)
        except MachineError:
            break
        t = g['t']
        if t >= walk_start:
            robot_times.append(t)
            robot_com.append(float(d.qpos[2]))  # Z along path in simulator
            angles = {j_name: math.degrees(float(d.qpos[world.qadr[i]]))
                      for i, j_name in enumerate(figure.JOINTS)}
            q = d.qpos[3:7]  # Pelvis quat
            pelvis_quat = figure.quat(float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            frames = figure.frames(angles, tuple(float(x) for x in d.qpos[0:3]), pelvis_quat)

            for j in joints:
                # Find matching segment for joint
                seg_name = next((s[0] for s in figure.SEGMENTS if fbx.bone(s[0]) == j), None)
                if seg_name and seg_name in frames:
                    robot_rots[j].append(np.array(frames[seg_name][1]))

    if len(robot_times) < 30:
        sys.stderr.write("ERROR: Simulation did not generate enough walking frames!\n")
        sys.exit(1)

    robot_rots = {k: np.array(v) for k, v in robot_rots.items() if v}
    gvs, gps = compute_gvs(ref_times, ref_com, ref_rots,
                           robot_times, robot_com, robot_rots,
                           joints=joints)
    return report_and_gate(gvs, gps, tolerance)


def evaluate_recording(record_path, ref_path, joints=EVAL_JOINTS, tolerance=MAX_TOLERANCE_RAD):
    """Evaluate recorded CSV telemetry against reference FBX take."""
    import csv
    ref_times, ref_joints, ref_rots = extract_take_rotations(ref_path)
    ref_com = ref_joints['Hips'][:, 2]

    robot_times, robot_com = [], []
    robot_rots = {j: [] for j in joints}

    with open(record_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            t = float(row['t'])
            robot_times.append(t)
            robot_com.append(float(row.get('z', row.get('pelvis_z', 0.0))))
            angles = {j_name: float(row[j_name]) for j_name in figure.JOINTS if j_name in row}
            pelvis_at = (float(row.get('x', 0.0)), float(row.get('y', 0.0)), float(row.get('z', 0.0)))
            pelvis_quat = figure.quat(float(row.get('qw', 1.0)), float(row.get('qx', 0.0)),
                                      float(row.get('qy', 0.0)), float(row.get('qz', 0.0)))
            frames = figure.frames(angles, pelvis_at, pelvis_quat)
            for j in joints:
                seg_name = next((s[0] for s in figure.SEGMENTS if fbx.bone(s[0]) == j), None)
                if seg_name and seg_name in frames:
                    robot_rots[j].append(np.array(frames[seg_name][1]))

    robot_rots = {k: np.array(v) for k, v in robot_rots.items() if v}
    gvs, gps = compute_gvs(ref_times, ref_com, ref_rots,
                           robot_times, robot_com, robot_rots,
                           joints=joints)
    return report_and_gate(gvs, gps, tolerance)


def report_and_gate(gvs, gps, tolerance=MAX_TOLERANCE_RAD):
    """Print GVS table, aggregate GPS, and enforce the CI guillotine gate."""
    print("=" * 60)
    print("INVARIANT KINEMATIC EVALUATION (SO(3) LIE LOG MAP)")
    print("=" * 60)
    print(f"{'Joint':<20} {'GVS (rad)':<15} {'GVS (deg)':<15}")
    print("-" * 60)
    for joint, score in sorted(gvs.items()):
        print(f"{joint:<20} {score:<15.4f} {math.degrees(score):<15.2f}")
    print("-" * 60)
    print(f"{'GAIT PROFILE SCORE':<20} {gps:<15.4f} {math.degrees(gps):<15.2f}")
    print(f"{'MAX TOLERANCE':<20} {tolerance:<15.4f} {math.degrees(tolerance):<15.2f}")
    print("=" * 60)

    if gps > tolerance:
        print(f"\n[CI GUILLOTINE TRIPPED] GPS ({math.degrees(gps):.2f} deg) > MAX_TOLERANCE ({math.degrees(tolerance):.2f} deg).")
        print("TERMINATING BUILD WITH STATUS 1.\n")
        return 1
    else:
        print(f"\n[CI PASSED] GPS ({math.degrees(gps):.2f} deg) <= MAX_TOLERANCE ({math.degrees(tolerance):.2f} deg).")
        print("KINEMATIC CONVERGENCE CERTIFIED.\n")
        return 0


DEFAULT_MOCAP = r"C:\sources\Projects\electronics\Mocaps\02-walkforward.fbx"
REF_DEFAULT = DEFAULT_MOCAP if os.path.isfile(DEFAULT_MOCAP) else 'host/tests/takes/walk.fbx'


def main(argv=None):
    parser = argparse.ArgumentParser(description="Invariant Kinematic Evaluation & CI Guillotine on SO(3)")
    parser.add_argument('take', nargs='?', default=None, help="FBX take to evaluate")
    parser.add_argument('reference', nargs='?', default=REF_DEFAULT,
                        help=f"Reference normative mocap take (default: {REF_DEFAULT})")
    parser.add_argument('--eval', action='store_true', help="Run live simulation and evaluate against reference")
    parser.add_argument('--record', default=None, help="Path to telemetry recording CSV")
    parser.add_argument('--tolerance', type=float, default=MAX_TOLERANCE_DEG,
                        help=f"Max tolerance for GPS in degrees (default: {MAX_TOLERANCE_DEG})")
    parser.add_argument('--to', type=float, default=12.0, help="Simulation duration (s)")
    args = parser.parse_args(argv)

    tolerance_rad = math.radians(args.tolerance)

    if args.eval:
        code = evaluate_simulation(args.reference, duration=args.to, tolerance=tolerance_rad)
        sys.exit(code)

    if args.record:
        code = evaluate_recording(args.record, args.reference, tolerance=tolerance_rad)
        sys.exit(code)

    if args.take:
        ref_times, ref_joints, ref_rots = extract_take_rotations(args.reference)
        tak_times, tak_joints, tak_rots = extract_take_rotations(args.take)
        ref_com = ref_joints['Hips'][:, 2]
        tak_com = tak_joints['Hips'][:, 2]
        gvs, gps = compute_gvs(ref_times, ref_com, ref_rots,
                               tak_times, tak_com, tak_rots)
        code = report_and_gate(gvs, gps, tolerance_rad)
        sys.exit(code)

    parser.print_help()
    sys.exit(0)


if __name__ == '__main__':
    main()
