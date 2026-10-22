/** wbc_surety.c - her directional surety field (wbc_imp.h): machine.motion.surety_field on her law's
    arrays, numpy's and Python's arithmetic operation for operation (wbc_np.h). Portable C11, the
    host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>

#define PI 3.14159265358979323846

/** Her pelvis's tilt from upright as her IMU has it, deg (machine.edge.tilt). */
static double tilt_deg(const double *q)
{
  const double s = pow(q[1], 2.0) + pow(q[3], 2.0);

  return acos(npmax(-1.0, npmin(1.0, 1.0 - 2.0 * s))) * (180.0 / PI);
}

double wbc_imp_surety(const wbc_imp_t *k, const wbc_surety_t *s, double prev, double dt, double ct, double t,
                      double vec[5])
{
  const double *y_in = s->y_in, *y = s->y, *fr = s->fr, *a = s->a, *st = s->st, *rot = fr + k->f_rot;
  const double *q = y_in + s->quat_at, *gyro = y_in + s->gyro_at, *acc = y_in + s->accel_at;
  const double  qw = q[0], qx = q[1], qj = q[2], qk = q[3], borne = a[k->a_borne];
  const double  wide[2] = {0.04, 0.05};
  double        c[3], v[3], sr[2][3], xi[2], flat[2][2], edge[2], tilt[2], lo[2][2], hi[2][2], up_w[3], r_y[3];
  double        loads[2], w[2], lo_1[2], hi_1[2], lmin[2], hmax[2], b_lo[2], b_hi[2], mid[2], span[2], cap[2];
  double        r_lo[2], r_hi[2], out_xi[2], s_dir[2], u_xy[2], e_vec[2], e3[3], d_xy[2], s_vec[2];
  double        om, ap_y, w_gyro, wt, k_ax, w_jump, f_soles, theta, theta_e, v_norm, w_ext, probe, w_sum;
  double        w_both, w_air, w_prop, w_sup, r_gait, w_tilt, h, f_att, raw, diff, step;

  for (int j = 0; j < 3; j++)
  {
    c[j] = npmv3(rot + 3 * j, y + k->y_com);
    v[j] = npmv3(rot + 3 * j, y + k->y_vcom);
  }
  om = sqrt(k->g / npmax(c[2] - fr[k->f_floor], 0.3));
  npmmt(y + k->y_sole, 2, rot, &sr[0][0]);
  for (int b = 0; b < 2; b++)
  {
    xi[b] = c[b] + v[b] / om;
    flat[b][0] = sr[b][0];
    flat[b][1] = sr[b][1];
    edge[b] = a[k->a_edge + b];
    tilt[b] = y[k->y_tilt + b];
  }
  wbc_imp_boxes(k, flat, edge, tilt, lo, hi);
  /* up and the IMU's own y, turned out of its frame */
  up_w[0] = 2.0 * (qx * qj - qw * qk);
  up_w[1] = 1.0 - 2.0 * (qx * qx + qk * qk);
  up_w[2] = 2.0 * (qj * qk + qw * qx);
  r_y[0] = 2.0 * (qx * qj + qw * qk);
  r_y[1] = 1.0 - 2.0 * (qx * qx + qk * qk);
  r_y[2] = 2.0 * (qj * qk - qw * qx);
  ap_y = npdot(r_y, 1, acc, 1, 3);
  w_gyro = sqrt(npdot(gyro, 1, gyro, 1, 3));
  wt = npmax(fr[k->f_weight], 1.0);
  k_ax = npmax(st[s->k_to], st[s->k_from]);
  w_jump = npmin(1.0, npmax(0.0, (k_ax - 15000.0) / 5000.0));
  for (int b = 0; b < 2; b++)
  {
    loads[b] = npmaximum(y[k->y_load + b], y_in[s->load_at + b]);
    w[b] = npmaximum(npclip(loads[b] / (0.25 * wt), 0.0, 1.0), w_jump);
  }
  f_soles = npsum(loads, 2) / wt;
  theta = (isnan(ct) ? tilt_deg(q) : ct) * (PI / 180.0);
  theta_e = npmax(0.0, theta - fabs(a[k->a_want]));
  v_norm = sqrt(npdot(v, 1, v, 1, 3));
  w_ext = npmin(1.0, npmax(0.0, (ap_y / k->g - f_soles) / 0.45))
          * exp(((-pow(w_gyro / 1.2, 2.0)) - pow(v_norm / 0.18, 2.0)) - pow(theta_e / 0.65, 4.0));
  probe = ((0.008 * npmax(0.0, (4.0 * f_soles) * (1.0 - f_soles))) * exp(-pow(w_gyro / 0.6, 2.0)))
          * sin(((2.0 * PI) * 6.0) * t);
  w_sum = (w[0] + w[1]) + 1e-9;
  w_both = npmin(w[0], w[1]);
  w_air = npmax(npmax(npmax(0.0, 1.0 - s->out[s->zmp_out]), 1.0 - s->out[s->held_out]), w_jump);
  w_prop = npmax(npmax(w_ext, borne), w_air);
  w_sup = npmin(1.0, ((0.42 + 0.30 * (w[0] + w[1])) + 0.22 * npmax(w[0], w[1])) + 0.55 * w_prop);
  r_gait = 0.28 * npmin(1.0, st[s->gain_now]);
  w_tilt = hypot(gyro[0], gyro[2]);
  for (int j = 0; j < 2; j++)
  {
    const double vp = v[j] * (1.0 - w_prop);
    double       o;

    lo_1[j] = npdot(w, 1, &lo[0][j], 2, 2) / w_sum;
    hi_1[j] = npdot(w, 1, &hi[0][j], 2, 2) / w_sum;
    lmin[j] = npminimum(lo[0][j], lo[1][j]);
    hmax[j] = npmaximum(hi[0][j], hi[1][j]);
    b_lo[j] = ((1.0 - w_both) * lo_1[j] + w_both * lmin[j]) - w_ext * 0.16;
    b_hi[j] = ((1.0 - w_both) * hi_1[j] + w_both * hmax[j]) + w_ext * 0.16;
    mid[j] = 0.5 * (b_lo[j] + b_hi[j]);
    span[j] = npmaximum(0.5 * (b_hi[j] - b_lo[j]), 0.02);
    cap[j] = w_sup * (0.85 + 0.15 * (span[j] / (span[j] + wide[j])));
    r_lo[j] = npminimum(b_lo[j], lmin[j] - r_gait);
    r_hi[j] = npmaximum(b_hi[j], hmax[j] + r_gait);
    out_xi[j] = (xi[j] - npclip(xi[j], r_lo[j], r_hi[j])) * (1.0 - w_prop);
    o = out_xi[j] / 0.24;
    s_dir[j] = cap[j] * (0.45 + 0.55 * exp(((-(o * o)) - 0.04 * (vp * vp)) - pow(0.22 * w_tilt, 2.0)));
    u_xy[j] = npdot(up_w, 1, rot + 3 * j, 1, 3);
    e_vec[j] = ((xi[j] - mid[j]) * (1.0 - w_ext) + 0.35 * u_xy[j]) + 0.15 * v[j];
  }
  e3[0] = e_vec[0];
  e3[1] = e_vec[1];
  e3[2] = 0.08;
  h = pyhypot(e3, 3);
  f_att = pow(npmax(0.0, 1.0 - pow(theta_e / 1.20, 4.0)), 2.0);
  for (int j = 0; j < 2; j++)
  {
    d_xy[j] = e_vec[j] / h;
    s_vec[j] = npclip(s_dir[j] * f_att, 0.0, 1.0);
  }
  raw = npsum(s_vec, 2) / 2.0;
  diff = raw - prev;
  step = npmin(1.0, dt / 0.012) * npmin(diff, 0.0)
         + (npmin(1.0, dt / 0.12) * npmax(pow(prev, 3.0), 0.08 * borne)) * npmax(diff, 0.0);
  vec[0] = s_vec[0];
  vec[1] = s_vec[1];
  vec[2] = d_xy[0];
  vec[3] = d_xy[1];
  vec[4] = probe;
  return npmin(1.0, npmax(0.0, prev + step));
}
