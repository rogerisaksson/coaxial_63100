/** wbc_touch.c - what her skin and her soles feel (wbc_imp.h): machine.touch.stepped on her law's
    arrays, numpy's and Python's arithmetic operation for operation (wbc_np.h). Portable C11, the
    host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>

/** Python's math.hypot of two. */
static double hypot2(double a, double b)
{
  const double v[2] = {a, b};

  return pyhypot(v, 2);
}

/** machine.touch.against: `tw` less any axis aimed into what she felt (`ox`, `oz`). */
static void against(double ox, double oz, double heading, double along_cos, const double tw[3], double got[3])
{
  const double rem = hypot2(ox, oz), w = npmin(1.0, rem / 0.10);
  const double nx = ox / npmax(rem, 1e-9), nz = oz / npmax(rem, 1e-9), sh = sin(heading), ch = cos(heading);
  const double cx = sh * nx + ch * nz, cy = ch * nx - sh * nz;
  const double kx = 1.0 - w * (1.0 - npmin(1.0, npmax(0.0, (copysign(cx, tw[0]) + along_cos) / (1.0 - along_cos))));
  const double ky = 1.0 - w * (1.0 - npmin(1.0, npmax(0.0, (copysign(cy, tw[1]) + along_cos) / (1.0 - along_cos))));

  got[0] = kx * tw[0];
  got[1] = ky * tw[1];
  got[2] = tw[2];
}

void wbc_touch_step(const wbc_imp_t *k, const wbc_touch_t *t, const double asked[3], double twist[3], double tw_on,
                    double heading, double dt, const double *vel)
{
  const int     n = k->n;
  const double *touch = t->y_in + t->touch_at, *out0 = t->out;
  double       *a = t->a, *out = t->out, *state = t->state;
  double        f[3] = {0.0, 0.0, 0.0}, c_max = 0.0, af, ox, oz, rem, rem_d, ox_d, oz_d, fx, fz, fn, vn, nx, nz, cv;
  double        drv0, w_opp, tw[3], blocked, gain_on, hold, u_act, u_b, need, u_set, u_s, u_drv, turns;
  double        sway[WBC_IMP_N], psi[WBC_IMP_N], z0[WBC_IMP_N], zb[WBC_IMP_N], z_new[WBC_IMP_N], sway_b[WBC_IMP_N];
  double        sway_new[WBC_IMP_N], sine_new[WBC_IMP_N], kb[WBC_IMP_N], db[WBC_IMP_N], aloft[2], clear[2];
  double        pushed[2][2][3], sizes[2][2], lowest[2], floors[2], up[2], w_up[2], w_edge[2], unfelt[2], stance[2];

  /* her core's touch turned out of its segments, summed; the largest of them */
  for (int b = 0; b < 4; b++)
  {
    const double *r = t->xmat + 9 * t->ids[t->core[b]], *tc = touch + 3 * t->core[b];
    const double  size = sqrt((tc[0] * tc[0] + tc[1] * tc[1]) + tc[2] * tc[2]);

    for (int j = 0; j < 3; j++)
    {
      f[j] = f[j] + npmv3(r + 3 * j, tc);
    }
    c_max = b == 0 ? size : npmaximum(c_max, size);
  }
  af = npmin(1.0, npmax(0.0, (c_max - t->touched_n) / t->touched_n));
  fx = f[0];
  fz = f[2];
  ox = state[t->s_felt];
  oz = state[t->s_felt + 1];
  rem = hypot2(ox, oz);
  rem_d = npmax(0.0, rem - dt);
  ox_d = ox * (rem_d / npmax(rem, 1e-9));
  oz_d = oz * (rem_d / npmax(rem, 1e-9));
  fn = hypot2(fx, fz);
  vn = hypot2(vel[0], vel[2]);
  nx = fx / npmax(fn, 1e-9);
  nz = fz / npmax(fn, 1e-9);
  state[t->s_felt] = (1.0 - af) * ox_d + (af * t->felt_s) * nx;
  state[t->s_felt + 1] = (1.0 - af) * oz_d + (af * t->felt_s) * nz;
  cv = (fx * vel[0] + fz * vel[2]) / npmax(fn * vn, 1e-9);
  drv0 = fabs(out0[k->o_sway + k->drive]);
  w_opp = ((af * npmin(1.0, npmax(0.0, (t->along_cos - cv) / 0.15))) * npmin(1.0, npmax(0.0, (drv0 - 0.02) / 0.04)))
          * (1.0 - a[k->a_borne]);
  a[k->a_going] = a[k->a_going] * (1.0 - w_opp);
  against(state[t->s_felt], state[t->s_felt + 1], heading, t->along_cos, asked, tw);
  for (int i = 0; i < 3; i++)
  {
    twist[i] = tw_on * tw[i] + (1.0 - tw_on) * twist[i];
  }
  blocked = tw_on * npmin(1.0, npmax(0.0, (hypot2(asked[0], asked[1]) - hypot2(tw[0], tw[1])) / 0.05));
  gain_on = npmin(1.0, npmax(0.0, (npmax(t->st[t->st_now_gain], t->st[t->st_to_gain]) - 0.02) / 0.02));
  hold = gain_on * npmax(npmax(npmax(0.0, state[t->s_hold] - dt), blocked * 0.5), w_opp * t->resume_s);
  state[t->s_hold] = hold;
  u_act = npmin(1.0, hold / 0.01);
  u_b = npmin(1.0, npmax(0.0, state[t->s_brake] + (2.0 * u_act - 1.0) * (dt / (0.50 * u_act + 0.65 * (1.0 - u_act)))));
  need = a[k->a_need] + npmaximum(a[k->a_swings], a[k->a_swings + 1]);
  u_set = (u_act * npmin(1.0, npmax(0.0, (u_b - 0.95) / 0.05))) * npmin(1.0, npmax(0.0, 1.0 - need / 0.05));
  u_s = npmin(1.0, npmax(0.0, (state[t->s_stand] + u_set * (dt / 0.40)) - (1.0 - u_act) * (dt / 0.20)));
  state[t->s_brake] = u_b;
  state[t->s_stand] = u_s;
  turns = out0[k->o_turns];
  for (int m = 0; m < n; m++)
  {
    sway[m] = out0[k->o_sway + m];
    psi[m] = out0[k->o_psi + m];
    z0[m] = out0[k->o_ref + m] - sway[m] * sin(psi[m]);
    zb[m] = m == k->axial ? 0.04 : 0.0;
    z_new[m] = ((1.0 - u_b) * z0[m] + (u_b * (1.0 - u_s)) * zb[m]) + (u_b * u_s) * t->rest[t->r_z + m];
    sway_b[m] = 0.0;
  }
  sway_b[k->lateral] = 0.12 * (1.0 - u_s);
  sway_b[k->lift] = sway[k->lift] * (1.0 - u_s);
  for (int m = 0; m < n; m++)
  {
    sway_new[m] = (1.0 - u_b) * sway[m] + u_b * sway_b[m];
  }
  u_drv = npmin(1.0, npmax(0.0, (state[t->s_drv] * (1.0 - w_opp) + (1.0 - u_act) * (dt / 0.65)) - u_act * (dt / 0.50)));
  state[t->s_drv] = u_drv;
  sway_new[k->drive] = u_drv * sway[k->drive] + ((1.0 - u_drv) * npmin(sway[k->drive], 0.04)) * (1.0 - u_b);
  for (int m = 0; m < n; m++)
  {
    sine_new[m] = sway_new[m] * sin(psi[m]);
    kb[m] = out0[k->o_ks + m];
    db[m] = out0[k->o_ds + m];
  }
  kb[k->axial] = 3000.0;
  kb[k->lateral] = 3000.0;
  kb[k->sagittal] = 0.0;
  kb[k->rock] = 2400.0;
  db[k->axial] = 800.0;
  db[k->lateral] = 600.0;
  db[k->sagittal] = 260.0;
  db[k->rock] = 240.0;
  for (int m = 0; m < n; m++)
  {
    out[k->o_ref + m] = z_new[m] + sine_new[m];
    out[k->o_rate + m] = (1.0 - u_b) * (out0[k->o_rate + m] - (sway[m] * turns) * cos(psi[m]))
                         + (sway_new[m] * turns) * cos(psi[m]);
    out[k->o_acc + m] = ((1.0 - u_b) * out0[k->o_bend + m]) * (1.0 - npclip(out0[k->o_held], 0.0, 1.0))
                        - (sine_new[m] * turns) * turns;
    out[k->o_sway + m] = sway_new[m];
    out[k->o_ks + m] = ((1.0 - u_b) * out0[k->o_ks + m] + (u_b * (1.0 - u_s)) * kb[m]) + (u_b * u_s) * t->rest[t->r_k + m];
    out[k->o_ds + m] = ((1.0 - u_b) * out0[k->o_ds + m] + (u_b * (1.0 - u_s)) * db[m]) + (u_b * u_s) * t->rest[t->r_d + m];
  }
  /* her soles: each's pushes turned out of its foot and its toes, their sizes, its toes' lowest
     tip, the floor under it (machine.touch.soles) */
  for (int s = 0; s < 2; s++)
  {
    const int     toes = t->ids[t->soles[s][1]];
    const double *r = t->xmat + 9 * toes, *xp = t->xpos + 3 * toes;
    double        tip[2];

    for (int j = 0; j < 2; j++)
    {
      const int     seg = t->soles[s][j];
      const double *rs = t->xmat + 9 * t->ids[seg], *ts = touch + 3 * seg;

      for (int c = 0; c < 3; c++)
      {
        pushed[s][j][c] = npmv3(rs + 3 * c, ts);
      }
      sizes[s][j] = sqrt(npdot(pushed[s][j], 1, pushed[s][j], 1, 3));
    }
    for (int p = 0; p < 2; p++)
    {
      tip[p] = xp[1] + npmm3(t->tips + 3 * p, r + 3, 1);
    }
    lowest[s] = npminimum(tip[0], tip[1]);
    aloft[s] = npclip(1.0 - a[k->a_stance + s], 0.0, 1.0);
    clear[s] = state[t->s_clear + s];
    stance[s] = a[k->a_stance + s];
  }
  wbc_imp_under(k, t->y, t->fr, stance, floors);
  for (int s = 0; s < 2; s++)
  {
    double edge[2];

    up[s] = npminimum(t->y[k->y_lowest + s], lowest[s]) - floors[s];
    w_up[s] = npclip((up[s] - t->edge_m) / 0.004, 0.0, 1.0) * npmaximum(up[s], 0.0);
    for (int j = 0; j < 2; j++)
    {
      const double *p = pushed[s][j];

      edge[j] = npclip((sizes[s][j] - t->touched_n) / t->touched_n, 0.0, 1.0)
                * npclip((hypot(p[0], p[2]) - fabs(p[1])) / t->touched_n, 0.0, 1.0);
    }
    w_edge[s] = npmaximum(edge[0], edge[1]);
    unfelt[s] = npclip(1.0 - npmaximum(sizes[s][0], sizes[s][1]) / t->touched_n, 0.0, 1.0);
  }
  a[k->a_raise] = npmax(a[k->a_raise], npmaximum((clear[0] * w_edge[0]) * w_up[0], (clear[1] * w_edge[1]) * w_up[1]));
  for (int s = 0; s < 2; s++)
  {
    state[t->s_clear + s] = aloft[s] * npmaximum(clear[s], unfelt[s]);
  }
}
