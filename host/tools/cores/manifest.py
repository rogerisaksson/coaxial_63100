#!/usr/bin/env python3
"""Her orthokinematic manifest: what a text's numbers move, as Lisp, of her build and her figure.

    python tools/cores/manifest.py --write     # machine/<robot>.manifest.lisp
    page = manifest.text()                     # what the file holds, laid anew

A text asks a mode's target in its unit, its stiffness and damping in N/m and N s/m of the mass
the mode moves (`machine.modal`). Her manifest gives each mode's unit, mass and resting spring,
her body's statics, and each drive's parts, ratio, peak, speed and range - of her build's file
(machine/robots, `machine.mechatronics`) and her figure laid on her loop (`sprung.laid`).
`ortho` compiles it (`defrobot`, `ortho.robot`); test_mirrors holds the file to this.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO
from tools.cores import written

NOTICE = 'Her orthokinematic manifest: written by host/tools/cores/manifest.py, not by hand.'


def path(robot=None):
    """Where her manifest is written."""
    from machine import mechatronics
    return os.path.join(REPO, 'host', 'machine', '%s.manifest.lisp' % (robot or mechatronics.ROBOT))


def laid():
    """{robot, body, modes, drives}: her manifest's numbers, her figure laid on her loop."""
    from machine import drives, latent, mechatronics, modal, ortho, sprung
    from machine.figure import JOINTS
    g = sprung.laid()
    fr, rest = g['fr'], g['rest']
    body = (('mass', g['body'].mass), ('weight', fr[modal.WEIGHT]), ('com_height', fr[modal.H0]),
            ('omega', fr[modal.OMEGA]), ('leg', modal.LEG_M), ('stance_width', 2.0 * fr[modal.W0]),
            ('knee', math.degrees(fr[modal.KNEE])))
    modes = [('mode_' + name, fr[modal.UNIT][i], fr[modal.MASS][i], rest[latent.K][i], rest[latent.D][i])
             for i, name in enumerate(ortho.MODES)]
    build = mechatronics.robot()
    rows = []
    for joint in JOINTS:
        if drives.passive(joint):
            continue
        kind = drives.kind(joint)
        motor, box, inverter = build['stacks'][kind]
        low, high = (math.degrees(v) for v in sprung._Q_LIM.get(kind, (-2.0, 2.0)))
        rows.append((joint, kind, motor, box, inverter, drives.ratio(joint), drives.peak(joint),
                     drives.speed(joint), low, high))
    return {'robot': mechatronics.ROBOT, 'body': body, 'modes': modes, 'drives': rows}


def text(done=None):
    """The manifest's text."""
    done = done or laid()
    lines = [';; ' + NOTICE,
             ';; Units: m, kg, s, N; a mode\'s unit m or rad, its mass kg, its stiffness N/m and damping',
             ';; N s/m of that mass; a drive\'s peak N m at its joint, speed deg/s, the range her law asks deg.',
             '(defrobot %s' % done['robot'],
             '  (body %s)' % ' '.join('(%s %.4g)' % kv for kv in done['body'])]
    lines += ['  (mode %s (unit %.4g) (mass %.4g) (stiffness %.4g) (damping %.4g))' % row for row in done['modes']]
    lines += ['  (drive %s (kind %s) (motor %s) (gearbox %s) (inverter %s) (ratio %.4g) (peak %.4g) '
              '(speed %.4g) (range %.4g %.4g))' % row for row in done['drives']]
    return '\n'.join(lines) + ')\n'


def _c_impedance_ask_a():
    """First part of `law_impedance_ask`: reading, poise boxes, stride beat, and stance."""
    return '''
void law_impedance_ask(const double *out, const double *y, const double *fr, double *a) {
  const double stood[2] = {a[0], a[1]}, turns = out[77];
  double z[11], zd[11], leg0[2][3], leg1[2], leg2[2], turn[3][2], spun[3][2];
  c_reading(y, fr, stood, z, zd, leg0, leg1, leg2, turn, spun);
  double c[3], v[3], vh[3], soles_rot[2][3], shod[2][3], flat[2][2], low[2][2], high[2][2], lo[2][2], hi[2][2];
  for (int r = 0; r < 3; r++) {
    c[r] = dot3(fr + 3 * r, y + 0); v[r] = dot3(fr + 3 * r, y + 3); vh[r] = dot3(fr + 3 * r, y + 60);
    for (int k = 0; k < 2; k++) {
      soles_rot[k][r] = dot3(fr + 3 * r, y + 6 + 3 * k);
      shod[k][r] = dot3(fr + 3 * r, y + 12 + 3 * k);
    }
  }
  for (int k = 0; k < 2; k++) { flat[k][0] = soles_rot[k][0]; flat[k][1] = soles_rot[k][1]; }
  const double om = sqrt(G / fmax(c[2] - fr[11], LOW_M));
  const double xi[2] = {c[0] + v[0] / om, c[1] + v[1] / om}, *tilt = y + 45, *tilting = y + 47;
  c_boxes(flat, a + 44, tilt, low, high);
  const double b0_s = (stood[0] + TINY) / (stood[0] + stood[1] + 2.0 * TINY);
  const double b1_s = (stood[1] + TINY) / (stood[0] + stood[1] + 2.0 * TINY);
  const double pg0 = fr[56] * c_clip((fabs(fr[56]) - 0.5 * EDGE_RAD) / (0.5 * EDGE_RAD), 0.0, 1.0);
  const double pg1 = fr[57] * c_clip((fabs(fr[57]) - 0.5 * EDGE_RAD) / (0.5 * EDGE_RAD), 0.0, 1.0);
  const double rise = -(b0_s * tan(pg0) + b1_s * tan(pg1));
  const double floors[2] = {
    fr[11] + rise * ((y[6] - fr[58]) * fr[0] + (y[8] - fr[59]) * fr[2]),
    fr[11] + rise * ((y[9] - fr[58]) * fr[0] + (y[11] - fr[59]) * fr[2])
  };
  double prop_lo[2] = {1e30, 1e30}, prop_hi[2] = {-1e30, -1e30};
  for (int ix = 0; ix < 2; ix++) {
    for (int iz = 0; iz < 2; iz++) {
      const double px = y[52 + 3 * ix], pz = y[54 + 3 * iz];
      const double wx = px * fr[0] + pz * fr[2], wy = px * fr[3] + pz * fr[5];
      prop_lo[0] = fmin(prop_lo[0], wx); prop_lo[1] = fmin(prop_lo[1], wy);
      prop_hi[0] = fmax(prop_hi[0], wx); prop_hi[1] = fmax(prop_hi[1], wy);
    }
  }
  const double propped = c_clip(y[51] / PROP_ON, 0.0, 1.0);
  const double beat = out[55 + LIFT], sw_lift = out[66 + LIFT];
  const double tone[3] = {sw_lift * sin(beat), sw_lift * turns * cos(beat), -sw_lift * turns * turns * sin(beat)};
  double ref[11], rate[11], acc[11], kp[11], kd[11], zdd[11], sw_n2 = 0.0;
  for (int n = 0; n < 11; n++) {
    ref[n] = out[0 + n] - ((n == LIFT) ? tone[0] : 0.0);
    rate[n] = out[11 + n] - ((n == LIFT) ? tone[1] : 0.0);
    acc[n] = out[22 + n] - ((n == LIFT) ? tone[2] : 0.0);
    kp[n] = out[33 + n] / fr[45 + n]; kd[n] = out[44 + n] / fr[45 + n];
    sw_n2 += out[66 + n] * out[66 + n];
  }
  const double up[2] = {ref[LIFT], -ref[LIFT]}, still = 1.0 - c_clip(sqrt(sw_n2) / TINY, 0.0, 1.0);
  const double zero2[2] = {0.0, 0.0}, out_of[2] = {BIG * (1.0 - stood[0]), BIG * (1.0 - stood[1])};
  c_boxes(flat, zero2, tilt, lo, hi);
  const double lifted_b[2] = {
    out_of[0] * fmax(1.0 - still, fmax(a[51], (up[0] > LIFT_EPS) ? 1.0 : 0.0)),
    out_of[1] * fmax(1.0 - still, fmax(a[52], (up[1] > LIFT_EPS) ? 1.0 : 0.0))
  };
  double beyond[2], past_v[2];
  for (int d = 0; d < 2; d++) {
    const double l0 = fmin(lo[0][d] + out_of[0], lo[1][d] + out_of[1]), h0 = fmax(hi[0][d] - out_of[0], hi[1][d] - out_of[1]);
    const double sl0 = l0 + propped * (fmin(l0, prop_lo[d]) - l0), sh0 = h0 + propped * (fmax(h0, prop_hi[d]) - h0);
    beyond[d] = xi[d] - c_clip(xi[d], sl0, sh0);
    const double l1 = fmin(lo[0][d] + lifted_b[0], lo[1][d] + lifted_b[1]), h1 = fmax(hi[0][d] - lifted_b[0], hi[1][d] - lifted_b[1]);
    const double sl1 = l1 + propped * (fmin(l1, prop_lo[d]) - l1), sh1 = h1 + propped * (fmax(h1, prop_hi[d]) - h1);
    past_v[d] = xi[d] - c_clip(xi[d], sl1, sh1);
  }
  const double stood_max = fmax(stood[0], stood[1]);
  a[54] = stood_max * c_clip(hypot(past_v[0], past_v[1]) / MARGIN_M, 0.0, 1.0);
  const double drive_sw = out[66 + DRIVE], psi_dr = out[55 + DRIVE], sway_lat = out[66 + LATERAL], lat_psi = out[55 + LATERAL];
  const double stepping = fmax(c_clip(fabs(drive_sw) / STIRS, 0.0, 1.0), a[54]);
  const double way_dir = 1.0 - 2.0 * ((sin(lat_psi - psi_dr) * copysign(1.0, sway_lat) < -BACK) ? 1.0 : 0.0);
  const double swung = 0.5 * turns / PHI_W;
  double sweep[2], phi[2], on_b[2], off_b[2], u_b[2], flying[2], hurry[2], limbs[2];
  const double kp_free[2] = {kp[DRIVE], kp[WIDTH]};
  for (int k = 0; k < 2; k++) {
    const double side = -C_LIMBS[k] * copysign(1.0, drive_sw) * way_dir;
    sweep[k] = side * sin(psi_dr);
    phi[k] = atan2(sweep[k], side * cos(psi_dr));
    on_b[k] = ((phi[k] > PHI_W - C_PI) && (phi[k] < PHI_W)) ? 1.0 : 0.0;
    u_b[k] = c_clip(0.5 * (phi[k] / PHI_W + 1.0) + a[55 + k], 0.0, 1.0);
    off_b[k] = fmax(stepping, a[51 + k]) * on_b[k] * (1.0 - (((a[55 + k] > 0.0) && (u_b[k] >= 1.0)) ? 1.0 : 0.0));
    flying[k] = off_b[k] * ((u_b[k] > 0.0) ? 1.0 : 0.0);
    a[51 + k] = off_b[k];
    const double aside = stood_max * fmax(C_LIMBS[k] * beyond[1], 0.0);
    const double due = log(fmax(REACH_M / fmax(aside, TINY), 1.0)) / om;
    const double hk = fmax(1.0, (1.0 - u_b[k]) / (swung * fmax(due, sqrt(6.0 / kp_free[k]))));
    hurry[k] = 1.0 + flying[k] * (hk - 1.0);
    a[55 + k] = on_b[k] * (a[55 + k] + fr[18] * swung * (hurry[k] - 1.0));
    limbs[k] = fmax(stepping, off_b[k]);
  }
'''


def _c_impedance_ask_b():
    """Second part of `law_impedance_ask`: ZMP stance, stride landing, and capture target."""
    return '''
  const double st_lt = ((stood[0] + stood[1]) < 1.5) ? 1.0 : 0.0;
  a[57] = stood[0] * st_lt * fmax(beyond[1], 0.0) + stood[1] * st_lt * fmax(-beyond[1], 0.0);
  const double raw_cr = way_dir * 2.0 / C_PI * STRIDE_M * fabs(drive_sw) * sin(PHI_W) * turns;
  const double cruise = c_clip(raw_cr, a[22] - AHEAD_M_S, a[22] + AHEAD_M_S);
  const double lat_dc = stepping * (ref[LATERAL] - sway_lat * sin(lat_psi)), across = 2.5 * lat_dc;
  const double dv0 = v[0] - cruise - HALF_M * rate[SAGITTAL], dv1 = v[1] - across - fr[10] * rate[LATERAL];
  const double dv2 = dv0 * dv0 + dv1 * dv1, dv4 = dv2 * dv2;
  double dvh[3] = {vh[0] - v[0] - HALF_M * PITCH_RAD * rate[PITCH], vh[1] - v[1], vh[2] - v[2]};
  for (int i = 0; i < 3; i++) {
    const double h2 = dvh[i] * dvh[i], h4 = h2 * h2;
    dvh[i] = 0.5 * still * dvh[i] * h4 / (h4 + 0.02);
  }
  ref[AXIAL] += stood_max * still * c_clip(0.5 * dv2 * (dv4 / (dv4 + 0.02)) / (G * AXIAL_M), 0.0, 0.25);
  zd[SAGITTAL] = (v[0] - cruise + dvh[0]) / HALF_M;
  zd[LATERAL] += stepping * ((v[1] - across) / fr[10] - zd[LATERAL]) + dvh[1] / fr[10];
  double both[3] = {0.0, 0.0, 0.0};
  for (int n = 0; n < 11; n++) {
    zdd[n] = acc[n] + kp[n] * (ref[n] - z[n]) + kd[n] * (rate[n] - zd[n]);
    for (int i = 0; i < 2; i++) {
      for (int r = 0; r < 3; r++) both[r] += 0.5 * C_HANDS[(n * 2 + i) * 3 + r] * fr[23 + n] * zdd[n];
    }
  }
  const int swaying = (fabs(ref[LATERAL] - lat_dc - z[LATERAL]) < ON_0) && (fabs(rate[LATERAL] - zd[LATERAL]) < ON_1);
  double zmp[2] = {c[0] + both[0] / (om * om), c[1] + both[1] / (om * om)}, zmp_in[2], slo[2], shi[2];
  const double kept = c_clip(out[78], 0.0, 1.0), foot = c_clip(out[78] - 1.0, 0.0, 1.0);
  for (int d = 0; d < 2; d++) {
    const double l0 = fmin(low[0][d], low[1][d]), h0 = fmax(high[0][d], high[1][d]);
    slo[d] = l0 + propped * (fmin(l0, prop_lo[d]) - l0);
    shi[d] = h0 + propped * (fmax(h0, prop_hi[d]) - h0);
  }
  c_into(zmp, xi, slo, shi, om, 1.0 - foot, zmp_in);
  zmp[0] += kept * (zmp_in[0] - zmp[0]); zmp[1] += kept * (zmp_in[1] - zmp[1]);
  const double over_f[2] = {y[37] - floors[0], y[38] - floors[1]};
  const double level[2] = {fmax(over_f[0], y[37] - fr[11]), fmax(over_f[1], y[38] - fr[11])};
  const double borne_n = y[51] * fr[13];
  const int rests = (fmax(y[18], y[19]) >= TOUCH_N) || (borne_n >= TOUCH_N) || (fabs(y[4]) <= REST_M_S);
  int within[2], landed[2];
  for (int k = 0; k < 2; k++) {
    within[k] = (zmp[0] >= low[k][0] - GATE_M) && (zmp[0] <= high[k][0] + GATE_M)
                && (zmp[1] >= low[k][1] - GATE_M) && (zmp[1] <= high[k][1] + GATE_M);
    landed[k] = (off_b[k] < 0.5) && ((y[18 + k] >= TOUCH_N) || ((level[k] <= TOUCH_M) && rests));
  }
  double stance[2], left_s[2];
  for (int k = 0; k < 2; k++) {
    const int early = (sweep[k] > EARLY) && ((y[18 + k] >= TOUCH_N) || (stood[k] > 0.5));
    const int gone = (flying[k] > 0.5) && (landed[1 - k] || (over_f[k] > TOUCH_M)) && !early;
    const int lifting = ((up[k] > LIFT_EPS) && ((within[1 - k] && swaying) || (over_f[k] > TOUCH_M))) || gone;
    stance[k] = (!lifting && ((y[18 + k] >= TOUCH_N) || ((level[k] <= TOUCH_M) && rests))) ? 1.0 : 0.0;
    left_s[k] = stance[k] * (1.0 - off_b[k] * (double)landed[1 - k]);
  }
  const double lmax = fmax(left_s[0], left_s[1]);
  left_s[0] += (1.0 - lmax) * stance[0]; left_s[1] += (1.0 - lmax) * stance[1];
  for (int d = 0; d < 2; d++) {
    const double l0 = fmin(low[0][d] + BIG * (1.0 - left_s[0]), low[1][d] + BIG * (1.0 - left_s[1]));
    const double h0 = fmax(high[0][d] - BIG * (1.0 - left_s[0]), high[1][d] - BIG * (1.0 - left_s[1]));
    slo[d] = l0 + propped * (fmin(l0, prop_lo[d]) - l0);
    shi[d] = h0 + propped * (fmax(h0, prop_hi[d]) - h0);
  }
  c_into(zmp, xi, slo, shi, om, foot * fmin(left_s[0], left_s[1]), zmp_in);
  zmp[0] += kept * (zmp_in[0] - zmp[0]); zmp[1] += kept * (zmp_in[1] - zmp[1]);
  const double grounded = fmax(fmax(stance[0], stance[1]), propped), aloft[2] = {1.0 - stance[0], 1.0 - stance[1]};
  const double b0 = (stance[0] + TINY) / (stance[0] + stance[1] + 2.0 * TINY);
  const double b1 = (stance[1] + TINY) / (stance[0] + stance[1] + 2.0 * TINY);
  const double bgrip = b0 * y[58] + b1 * y[59];
  double along[2] = {om * om * (c[0] - zmp[0]), om * om * (c[1] - zmp[1])};
  const double al_scale = fmin(1.0, bgrip * G / fmax(hypot(along[0], along[1]), TINY));
  along[0] *= al_scale; along[1] *= al_scale;
  const double reach = fmin(REACH_M, bgrip * (c[2] - fr[11])), step_s = C_PI / fmax(turns, TINY), grown = exp(om * step_s);
  const double led = LEADS * fmax((xi[0] - c[0]) - cruise / om, -0.04), s_ac = c_sign(across);
  const double led_lat = s_ac * c_clip(s_ac * (xi[1] - c[1]) - 1.35 * fabs(across) / om, -0.052, 0.042);
  const double wide = 2.0 * fr[10] * (1.0 + ref[WIDTH]);
'''


def _c_impedance_ask_c():
    """Third part of `law_impedance_ask`: heels, clearances, CoM, swing blend, and poses."""
    return '''
  double meet[2][2], tip[2], tipping[2], tipped[2], short_f[2], clear[3][2], free_acc[2][3], frame[3][3];
  const double load_sum = fmax(y[18] + y[19], TINY), kb0 = fr[14] + a[18];
  const double long_l = fr[19] - AXIAL_M * fr[20] * fmax(ref[AXIAL], 0.0), pg_b = b0 * pg0 + b1 * pg1;
  a[53] -= fr[18] * turns / C_TAU * a[53];
  const double high_cl = CLEAR_M * (1.0 + fabs(out[66 + LIFT])) + a[53];
  for (int k = 0; k < 2; k++) {
    const double m0 = xi[0] - (cruise * step_s / (grown - 1.0) - led);
    const double m1 = xi[1] - (-led_lat - C_LIMBS[k] * wide / (grown + 1.0));
    meet[k][0] = c[0] + c_clip(m0 - c[0], -reach, reach);
    meet[k][1] = flat[1 - k][1] + C_LIMBS[k] * fmax(C_LIMBS[k] * (m1 - flat[1 - k][1]), APART_M);
    const double heel = ROCK_RAD * (1.0 + HEEL_UP * c_sign(C_LIMBS[k] * ref[ROCK])) * C_LIMBS[k];
    tip[k] = heel * ref[ROCK]; tipping[k] = heel * rate[ROCK]; tipped[k] = heel * acc[ROCK];
    a[49 + k] = FOLD_RAD * off_b[k] * (1.0 - y[18 + k] / load_sum);
    short_f[k] = LEG_M * (cos(0.5 * kb0) - cos(0.5 * (kb0 + a[49 + k])));
    const double st = sin(tilt[k]), ct = cos(tilt[k]);
    const double hip_x = dot3(y + 39 + 3 * k, fr + 0) - (soles_rot[k][0] + HALF_M * ct);
    const double high_h = floors[k] + ANKLE_H + long_l - DIP_M - (soles_rot[k][2] - HALF_M * st);
    const double lever = hypot(hip_x * BALL - high_h * ANKLE_H, hip_x * ANKLE_H + high_h * BALL);
    const double ls = long_l - short_f[k];
    const double ac_in = c_clip((ls * ls - hip_x * hip_x - high_h * high_h - BALL * BALL - ANKLE_H * ANKLE_H) / (2.0 * lever), -1.0, 1.0);
    const double need_h = c_clip(atan2(-(hip_x * ANKLE_H + high_h * BALL), hip_x * BALL - high_h * ANKLE_H) + acos(ac_in), 0.0, HEEL_RAD) * c_clip(hip_x / BALL, 0.0, 1.0);
    const double rocked = fmax(off_b[k] * fmax(tip[k], 0.0), need_h), lean = tilt[k] - ((k == 0) ? pg0 : pg1);
    const double flk = stepping * stance[k] * (FLAT_K * (rocked - lean) + CONTACT_K * c_clip((rocked - a[46 + k]) / fr[18], -HEEL_RAD_S, HEEL_RAD_S));
    const double ed = c_sign(lean + BIG * rocked);
    a[46 + k] = rocked;
    a[44 + k] = stepping * stance[k] * ed * c_clip(fmax(fabs(lean), rocked) / EDGE_RAD, 0.0, 1.0);
    const double r0 = ed * HALF_M * (flk * st + tilting[k] * tilting[k] * ct);
    const double r2 = ed * HALF_M * (flk * ct - tilting[k] * tilting[k] * st);
    for (int d = 0; d < 3; d++) {
      a[32 + 6 * k + d] = flk * fr[3 + d];
      a[32 + 6 * k + 3 + d] = r0 * fr[0 + d] + r2 * fr[6 + d];
    }
    const double late = stepping * c_clip((phi[k] - PHI_W) / fmax(turns, TINY), 0.0, REACH_S);
    const double on_u = c_clip(u_b[k] / RISE_U, 0.0, 1.0), by_u = c_clip((1.0 - u_b[k]) / FALL_U, 0.0, 1.0);
    const double e00 = on_u * on_u * (3.0 - 2.0 * on_u), e01 = by_u * by_u * (3.0 - 2.0 * by_u);
    const double e10 = 6.0 * on_u * (1.0 - on_u) / RISE_U, e11 = -6.0 * by_u * (1.0 - by_u) / FALL_U;
    const double e20 = 6.0 * (1.0 - 2.0 * on_u) * ((on_u < 1.0) ? 1.0 : 0.0) / (RISE_U * RISE_U);
    const double e21 = 6.0 * (1.0 - 2.0 * by_u) * ((by_u < 1.0) ? 1.0 : 0.0) / (FALL_U * FALL_U);
    const double sw_h = swung * hurry[k], down = fmax(tip[k], tilt[k]);
    clear[0][k] = aloft[k] * (high_cl * flying[k] * e00 * e01 - REACH_M_S * late + flying[k] * (HALF_M * fmax(-sin(tip[k]), 0.0) + (HALF_M + TOE_M) * fmax(sin(down), 0.0)));
    clear[1][k] = aloft[k] * (high_cl * flying[k] * (e10 * e01 + e00 * e11) * sw_h - REACH_M_S * ((late > 0.0) ? 1.0 : 0.0) + flying[k] * (-HALF_M * ((tip[k] < 0.0) ? 1.0 : 0.0) + (HALF_M + TOE_M) * ((tip[k] > 0.0) ? 1.0 : 0.0)) * cos(tip[k]) * tipping[k]);
    clear[2][k] = aloft[k] * (high_cl * flying[k] * (e20 * e01 + e00 * e21) * sw_h * sw_h);
    a[27 + k] = tilt[k] + (aloft[k] * (tipped[k] + kp[ROCK] * (tip[k] + pg_b - tilt[k]) + kd[ROCK] * (tipping[k] - tilting[k])) + TURN_KD * tilting[k]) / TURN_KP;
  }
  const double st_max = fmax(stance[0], stance[1]);
  z[AXIAL] = (fr[19] - (b0 * leg1[0] + b1 * leg1[1])) / (AXIAL_M * fr[20]);
  zd[AXIAL] = -(b0 * leg2[0] + b1 * leg2[1] + st_max * dvh[2]) / (AXIAL_M * fr[20]);
  const double pressed = acc[AXIAL] + kp[AXIAL] * (ref[AXIAL] - z[AXIAL]) + kd[AXIAL] * (rate[AXIAL] - zd[AXIAL]);
  double b_risen = 0.0;
  for (int k = 0; k < 2; k++) {
    const double dv_s = (v[0] - shod[k][0]) * (v[0] - shod[k][0]) + (v[1] - shod[k][1]) * (v[1] - shod[k][1]) + (v[2] - shod[k][2]) * (v[2] - shod[k][2]);
    const double rk = (leg2[k] * leg2[k] - dv_s - (leg0[k][0] * along[0] + leg0[k][1] * along[1]) - leg1[k] * AXIAL_M * fr[20] * pressed) / fmax(leg0[k][2], LOW_LEG * fr[19]);
    b_risen += ((k == 0) ? b0 : b1) * rk;
  }
  const double lifted_c = st_max * b_risen / fr[20] - (1.0 - st_max) * AXIAL_M * pressed;
  const double com[3] = {grounded * along[0], grounded * along[1], grounded * c_clip(lifted_c, -G, THROW_M_S2) - (1.0 - grounded) * G};
  c_swings(y, fr, v, om, shod, floors, rise, ref, rate, acc, kp, kd, clear, stance, cruise, grounded, free_acc, frame);
  const double kd_free[2] = {kd[DRIVE], kd[WIDTH]};
  for (int k = 0; k < 2; k++) {
    const double str_k = limbs[k] * grounded, to_go = fmax((ARRIVE_U - u_b[k]) * 2.0 * PHI_W / fmax(turns * hurry[k], TINY), SOON_S);
    const double g0 = fmin(6.0 / (to_go * to_go), kp_free[0]) * (meet[k][0] - flat[k][0]) - fmin(4.0 / to_go, kd_free[0]) * (shod[k][0] - v[0]);
    const double g1 = fmin(6.0 / (to_go * to_go), kp_free[1]) * (meet[k][1] - flat[k][1]) - fmin(4.0 / to_go, kd_free[1]) * (shod[k][1] - v[1]);
    for (int d = 0; d < 3; d++) {
      const double f_w = free_acc[k][0] * frame[0][d] + free_acc[k][1] * frame[1][d] + free_acc[k][2] * frame[2][d]
                         + com[0] * fr[0 + d] + com[1] * fr[3 + d] + (1.0 - grounded) * com[2] * fr[6 + d];
      const double s_w = g0 * fr[0 + d] + g1 * fr[3 + d] + free_acc[k][2] * fr[6 + d];
      a[5 + 3 * k + d] = (1.0 - str_k) * f_w + str_k * s_w;
    }
  }
  a[22] += fr[18] / GOING_S * (0.5 * (shod[0][0] + shod[1][0]) - a[22]);
  a[0] = stance[0]; a[1] = stance[1];
  for (int d = 0; d < 3; d++) a[2 + d] = com[0] * fr[0 + d] + com[1] * fr[3 + d] + com[2] * fr[6 + d];
  a[48] = stepping; a[58] = propped;
  const double folds[2] = {aloft[0] * (1.0 - limbs[0]), aloft[1] * (1.0 - limbs[1])};
  c_poses(out, y, fr, a, z, ref, rate, acc, kp, kd, turn, spun, up, folds, still);
}
'''


def c_impedance_ask():
    """C11 implementation of `impedance.ask` (`law_impedance_ask`)."""
    return _c_impedance_ask_a() + _c_impedance_ask_b() + _c_impedance_ask_c()


if __name__ == '__main__':
    if '--write' not in sys.argv[1:]:
        sys.exit(__doc__)
    page = text()
    written(path(), page)
    print('%s: %d lines' % (path(), page.count('\n')))

