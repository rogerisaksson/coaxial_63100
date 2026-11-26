/** mppi_lanes.c - eight plane rollouts stepped in lockstep SIMD lanes (AVX, AVX2, AVX-512, SVE2, SME2) */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize("no-trapping-math", "no-tree-loop-distribute-patterns", "vect-cost-model=dynamic")
#define MPPI_FLATTEN __attribute__((flatten))
#elif defined(__GNUC__)
#define MPPI_FLATTEN __attribute__((flatten))
#else
#define MPPI_FLATTEN
#endif
#if defined(__ARM_FEATURE_SME2)
#define MPPI_STREAMING __arm_locally_streaming
#else
#define MPPI_STREAMING
#endif

#include <math.h>
#include <stddef.h>

#include "board_units.h"
#include "mppi.h"
#include "wbc_libm_arc.h"

#define MPPI_LANES 8
#define SQ(a) ((a) * (a))

/* C11 fmax(c, x) and fmin(x, c) for finite c as single SIMD compares (ucrt's fmin/fmax are calls). */
static inline double mmax_c(double c, double x)
{
  return (x > c) ? x : c;
}

static inline double mmin_c(double x, double c)
{
  return (x < c) ? x : c;
}

static inline double mpos(double x)
{
  return mmin_c(mmax_c(0.0, x * 1e12), 1.0);
}

MPPI_STREAMING static MPPI_FLATTEN void rollout_block(const double *restrict x0, const double *const *restrict u_lane,
                                                      const double *restrict ref, const double *restrict k,
                                                      const double *restrict c, int nc, int ns,
                                                      const double *restrict r, int nr,
                                                      const double *restrict weight, const double *restrict u0,
                                                      int h, double dt, double w_p, double w_v,
                                                      const double *restrict wind, double *restrict out,
                                                      double *restrict far_out)
{
  _Alignas(64) double px[MPPI_LANES], py[MPPI_LANES], pz[MPPI_LANES];
  _Alignas(64) double vx[MPPI_LANES], vy[MPPI_LANES], vz[MPPI_LANES];
  _Alignas(64) double wx_s[MPPI_LANES], wy_s[MPPI_LANES], wz_s[MPPI_LANES];
  _Alignas(64) double R[9][MPPI_LANES], last[4][MPPI_LANES];
  _Alignas(64) double ut0[MPPI_LANES], ut1[MPPI_LANES], ut2[MPPI_LANES], ut3[MPPI_LANES];
  _Alignas(64) double Fx[MPPI_LANES], Fy[MPPI_LANES], Fz[MPPI_LANES];
  _Alignas(64) double Tx[MPPI_LANES], Ty[MPPI_LANES], Tz[MPPI_LANES];
  _Alignas(64) double wwx[MPPI_LANES], wwy[MPPI_LANES], wwz[MPPI_LANES];
  _Alignas(64) double twx[MPPI_LANES], twy[MPPI_LANES], twz[MPPI_LANES];
  _Alignas(64) double low[MPPI_LANES], acc[MPPI_LANES], far_acc[MPPI_LANES];

  for (int l = 0; l < MPPI_LANES; l++)
  {
    px[l] = x0[0];
    py[l] = x0[1];
    pz[l] = x0[2];
    vx[l] = x0[3];
    vy[l] = x0[4];
    vz[l] = x0[5];
    wx_s[l] = x0[15];
    wy_s[l] = x0[16];
    wz_s[l] = x0[17];
    last[0][l] = u0[0];
    last[1][l] = u0[1];
    last[2][l] = u0[2];
    last[3][l] = u0[3];
    acc[l] = 0.0;
    far_acc[l] = 0.0;
  }
  for (int m = 0; m < 9; m++)
  {
    const double rm = x0[6 + m];

    for (int l = 0; l < MPPI_LANES; l++)
    {
      R[m][l] = rm;
    }
  }
  const double mass = k[MP_MASS], wp2 = w_p * w_p, ix = k[MP_IX], iy = k[MP_IY], iz = k[MP_IZ];
  const double wind_x = wind[0], wind_y = wind[1], wind_z = wind[2], mg = mass * k[MP_GRAVITY];
  const double j_den = k[MP_J0] * mmax_c(1.0, k[MP_PITCH] * fabs(w_p) / TAU_D);
  const double wake_den = mmax_c(k[MP_WAKE_FLOOR], w_p * k[MP_PROP_R]);
  const double c_inf = 0.04 * k[MP_CL_ALPHA_A], inf_fl = k[MP_INFLOW_FLOOR];
  const double k_thr_wp2 = k[MP_K_THRUST] * wp2, stall = k[MP_STALL], blend = k[MP_BLEND];
  const double cl_a = k[MP_CL_ALPHA], cd_l = k[MP_CD_LIFT], rho = k[MP_RHO], area = k[MP_AREA];
  const double neg_rho_side = -rho * k[MP_SIDE], k_cle = 0.12 * k[MP_CL_ALPHA_A] * k[MP_SHARE];
  const double cda = k[MP_CDA], area_a = k[MP_AREA_A], cd0 = k[MP_CD0], cdi = k[MP_CDI];
  const double wq = k[MP_WQ], a_disc = k[MP_A_DISC], arm = k[MP_ARM], span2 = SQ(0.25 * k[MP_SPAN]);
  const double p_base = k[MP_PROP_BASE], p_load = k[MP_PROP_LOAD], p_surf = k[MP_PROP_SURF];
  const double k_drag = k[MP_K_DRAG], k_pitch = k[MP_K_PITCH], k_roll = k[MP_K_ROLL];
  const double stab = k[MP_STAB], fin = k[MP_FIN], ky_wv2 = k[MP_K_YAW] * w_v * w_v;
  const double k_fl = k[MP_K_FLOOR], c_fl = k[MP_C_FLOOR], touch = k[MP_TOUCH], skim = k[MP_SKIM];
  const double wx_w = weight[MW_X], wy_w = weight[MW_Y], wz_w = weight[MW_Z], wsp_w = weight[MW_SPEED];
  const double wb_w = weight[MW_BELLY], wn_w = weight[MW_NOSE], wspin_w = weight[MW_SPIN];
  const double wres_w = weight[MW_RESERVE], wtouch_w = weight[MW_TOUCH], wskim_w = weight[MW_SKIM];
  const double wcrash_w = weight[MW_CRASH], wease_w = weight[MW_EASE], wthr_w = weight[MW_THRUST];

  for (int t = 0; t < h; t++)
  {
    const double *rf = ref + (size_t)MR_REF * (size_t)t;
    const size_t  u_off = (size_t)t * 4U;

    for (int l = 0; l < MPPI_LANES; l++)
    {
      const double *ul = u_lane[l] + u_off;

      ut0[l] = ul[0];
      ut1[l] = ul[1];
      ut2[l] = ul[2];
      ut3[l] = ul[3];
    }
    for (int l = 0; l < MPPI_LANES; l++)
    {
      const double el = ut1[l], ai = ut2[l], ru = ut3[l];
      const double ax = vx[l] - wind_x, ay = vy[l] - wind_y, az = vz[l] - wind_z;
      const double sx = R[0][l], sy = R[3][l], sz = R[6][l];
      const double nx = R[1][l], ny = R[4][l], nz = R[7][l];
      const double bx = R[2][l], by = R[5][l], bz = R[8][l];
      const double along = ax * nx + ay * ny + az * nz, vh = wbc_hypot(ax, az);
      const double air = sqrt(ax * ax + ay * ay + az * az), uxb = ax * sx + ay * sy + az * sz;
      const double jj = mmax_c(0.0, 1.0 - along / j_den);
      const double share = jj * (1.0 + c_inf * mmin_c(vh / wake_den, inf_fl));
      const double lift = k_thr_wp2 * share;
      const double uy = mmax_c(0.0, along), uz = ax * bx + ay * by + az * bz, v2 = uy * uy + uz * uz;
      const double on = mpos(uy) * mpos(v2 - 1e-6);
      const double alpha = wbc_arctan2(uz, on * uy + (1.0 - on));
      const double flat = mmin_c(mmax_c(0.0, (fabs(alpha) - stall) / blend), 1.0);
      double       alo, aq;
      const double ah = lm_reduce(alpha, &alo, &aq);
      const double sa = lm_sin_at(ah, alo, aq, alpha), ca = lm_cos_at(ah, alo, aq), cln = cl_a * alpha;
      const double cl = (1.0 - flat) * cln + flat * 2.0 * sa * ca;
      const double cd = (1.0 - flat) * cd_l * cln * cln + flat * 2.0 * sa * sa;
      const double q_dyn = on * 0.5 * rho * area * v2, sp = sqrt(mmax_c(1e-12, v2));
      const double dx = (uy * nx + uz * bx) / sp, dy = (uy * ny + uz * by) / sp, dz = (uy * nz + uz * bz) / sp;
      const double lx = (uz * nx - uy * bx) / sp, ly = (uz * ny - uy * by) / sp, lz = (uz * nz - uy * bz) / sp;
      const double slip = neg_rho_side * air * uxb;
      const double cle = k_cle * wbc_hypot(el, ai);
      const double dn = 0.5 * rho * (cda + area_a * (cd0 + cdi * cle * cle)) * air;
      const double qw = wq * along, qd = wq * fabs(along);
      const double qs = 0.5 * rho * uy * uy + mmax_c(0.0, lift) / a_disc;
      const double prop = (p_base + p_load * share + p_surf * (el * el + ai * ai)) * k_drag * wp2;
      const double w0 = wx_s[l], w1 = wy_s[l], w2 = wz_s[l];

      Fx[l] = lift * nx + q_dyn * cl * lx - q_dyn * cd * dx + slip * sx - dn * ax;
      Fy[l] = lift * ny + q_dyn * cl * ly - q_dyn * cd * dy + slip * sy - dn * ay - mg;
      Fz[l] = lift * nz + q_dyn * cl * lz - q_dyn * cd * dz + slip * sz - dn * az;
      Tx[l] = k_pitch * qs * el + arm * stab * (qw * uz - qd * arm * w0);
      Ty[l] = ky_wv2 * ru - prop - qd * area * span2 * w1;
      Tz[l] = k_roll * qs * ai - arm * fin * (qw * uxb + qd * arm * w2);
      wwx[l] = sx * w0 + nx * w1 + bx * w2;
      wwy[l] = sy * w0 + ny * w1 + by * w2;
      wwz[l] = sz * w0 + nz * w1 + bz * w2;
      twx[l] = 0.0;
      twy[l] = 0.0;
      twz[l] = 0.0;
    }
    for (int ci = 0; ci < nc; ci++)
    {
      const double *b = c + 5 * ci;
      const double  b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3], b4 = b[4];

      for (int l = 0; l < MPPI_LANES; l++)
      {
        const double a0 = R[0][l] * b0 + R[1][l] * b1 + R[2][l] * b2;
        const double a1 = R[3][l] * b0 + R[4][l] * b1 + R[5][l] * b2;
        const double a2 = R[6][l] * b0 + R[7][l] * b1 + R[8][l] * b2;
        const double pen = mmax_c(0.0, b3 - py[l] - a1);
        const double vp0 = vx[l] + wwy[l] * a2 - wwz[l] * a1;
        const double vp1 = vy[l] + wwz[l] * a0 - wwx[l] * a2;
        const double vp2 = vz[l] + wwx[l] * a1 - wwy[l] * a0;
        const double nf = mpos(pen) * mmax_c(0.0, k_fl * pen - c_fl * vp1);
        const double sl = sqrt(vp0 * vp0 + vp2 * vp2 + 0.01);
        const double f0 = -b4 * nf * vp0 / sl, f1 = nf, f2 = -b4 * nf * vp2 / sl;

        Fx[l] += f0;
        Fy[l] += f1;
        Fz[l] += f2;
        twx[l] += a1 * f2 - a2 * f1;
        twy[l] += a2 * f0 - a0 * f2;
        twz[l] += a0 * f1 - a1 * f0;
      }
    }
    const double rp0 = rf[MR_PLACE], rp1 = rf[MR_PLACE + 1], rp2 = rf[MR_PLACE + 2], rwh = rf[MR_W_HEIGHT];
    const double rs0 = rf[MR_SPEED], rs1 = rf[MR_SPEED + 1], rs2 = rf[MR_SPEED + 2];
    const double rwb = rf[MR_W_BELLY], rb0 = rf[MR_BELLY], rb1 = rf[MR_BELLY + 1], rb2 = rf[MR_BELLY + 2];
    const double rwn = rf[MR_W_NOSE], rn0 = rf[MR_NOSE], rn1 = rf[MR_NOSE + 1], rn2 = rf[MR_NOSE + 2];

    for (int l = 0; l < MPPI_LANES; l++)
    {
      const double r0 = R[0][l], r1 = R[1][l], r2 = R[2][l];
      const double r3 = R[3][l], r4 = R[4][l], r5 = R[5][l];
      const double r6 = R[6][l], r7 = R[7][l], r8 = R[8][l];
      const double t0 = Tx[l] + (r0 * twx[l] + r3 * twy[l] + r6 * twz[l]);
      const double t1 = Ty[l] + (r1 * twx[l] + r4 * twy[l] + r7 * twz[l]);
      const double t2 = Tz[l] + (r2 * twx[l] + r5 * twy[l] + r8 * twz[l]);
      const double w0_was = wx_s[l], w1_was = wy_s[l], w2_was = wz_s[l];
      const double iw0 = ix * w0_was, iw1 = iy * w1_was, iw2 = iz * w2_was;
      const double cr0 = w1_was * iw2 - w2_was * iw1;
      const double cr1 = w2_was * iw0 - w0_was * iw2;
      const double cr2 = w0_was * iw1 - w1_was * iw0;
      const double v0 = vx[l] + dt * Fx[l] / mass;
      const double v1 = vy[l] + dt * Fy[l] / mass;
      const double v2 = vz[l] + dt * Fz[l] / mass;
      const double w0 = w0_was + dt * (t0 - cr0) / ix;
      const double w1 = w1_was + dt * (t1 - cr1) / iy;
      const double w2 = w2_was + dt * (t2 - cr2) / iz;
      const double p0 = px[l] + dt * v0, p1 = py[l] + dt * v1, p2 = pz[l] + dt * v2;
      const double tt = sqrt(w0 * w0 + w1 * w1 + w2 * w2) * dt;
      const double tt_ok = lm_sel(tt < LM_FAR, tt, 0.0);
      double       tlo, tq;
      const double th = lm_reduce(tt_ok, &tlo, &tq);
      const double s1 = lm_sin_at(th, tlo, tq, tt_ok), c1 = 1.0 - lm_cos_at(th, tlo, tq);
      const double inv_tt = dt / mmax_c(1e-12, tt);
      const double e0 = w0 * inv_tt, e1 = w1 * inv_tt, e2 = w2 * inv_tt;
      const double inc0 = 1.0 + c1 * (-e2 * e2 - e1 * e1);
      const double inc1 = -s1 * e2 + c1 * (e0 * e1);
      const double inc2 = s1 * e1 + c1 * (e0 * e2);
      const double inc3 = s1 * e2 + c1 * (e1 * e0);
      const double inc4 = 1.0 + c1 * (-e2 * e2 - e0 * e0);
      const double inc5 = -s1 * e0 + c1 * (e1 * e2);
      const double inc6 = -s1 * e1 + c1 * (e2 * e0);
      const double inc7 = s1 * e0 + c1 * (e2 * e1);
      const double inc8 = 1.0 + c1 * (-e1 * e1 - e0 * e0);
      const double rn0_m = r0 * inc0 + r1 * inc3 + r2 * inc6;
      const double rn1_m = r0 * inc1 + r1 * inc4 + r2 * inc7;
      const double rn2_m = r0 * inc2 + r1 * inc5 + r2 * inc8;
      const double rn3_m = r3 * inc0 + r4 * inc3 + r5 * inc6;
      const double rn4_m = r3 * inc1 + r4 * inc4 + r5 * inc7;
      const double rn5_m = r3 * inc2 + r4 * inc5 + r5 * inc8;
      const double rn6_m = r6 * inc0 + r7 * inc3 + r8 * inc6;
      const double rn7_m = r6 * inc1 + r7 * inc4 + r8 * inc7;
      const double rn8_m = r6 * inc2 + r7 * inc5 + r8 * inc8;
      const double u0_t = ut0[l], u1_t = ut1[l], u2_t = ut2[l], u3_t = ut3[l];
      double       step_c = acc[l];

      vx[l] = v0;
      vy[l] = v1;
      vz[l] = v2;
      wx_s[l] = w0;
      wy_s[l] = w1;
      wz_s[l] = w2;
      px[l] = p0;
      py[l] = p1;
      pz[l] = p2;
      R[0][l] = rn0_m;
      R[1][l] = rn1_m;
      R[2][l] = rn2_m;
      R[3][l] = rn3_m;
      R[4][l] = rn4_m;
      R[5][l] = rn5_m;
      R[6][l] = rn6_m;
      R[7][l] = rn7_m;
      R[8][l] = rn8_m;
      far_acc[l] += lm_sel(!(tt < LM_FAR), 1.0, 0.0);
      step_c += wx_w * SQ(p0 - rp0) + wz_w * SQ(p2 - rp2) + wy_w * rwh * SQ(p1 - rp1);
      step_c += wsp_w * (SQ(v0 - rs0) + SQ(v1 - rs1) + SQ(v2 - rs2));
      step_c += wb_w * rwb * (1.0 - (rn2_m * rb0 + rn5_m * rb1 + rn8_m * rb2));
      step_c += wn_w * rwn * (1.0 - (rn1_m * rn0 + rn4_m * rn1 + rn7_m * rn2));
      step_c += wspin_w * (w0 * w0 + w1 * w1 + w2 * w2);
      step_c += wres_w * (SQ(mmax_c(0.0, fabs(u1_t) - 0.5)) + SQ(mmax_c(0.0, fabs(u2_t) - 0.5))
                          + SQ(mmax_c(0.0, fabs(u3_t) - 0.5)));
      step_c += wease_w * (SQ(u0_t - last[0][l]) + SQ(u1_t - last[1][l])
                           + SQ(u2_t - last[2][l]) + SQ(u3_t - last[3][l]));
      step_c += wthr_w * u0_t * u0_t;
      last[0][l] = u0_t;
      last[1][l] = u1_t;
      last[2][l] = u2_t;
      last[3][l] = u3_t;
      acc[l] = step_c;
      low[l] = 1e3;
    }
    for (int ci = 0; ci < nc; ci++)
    {
      const double *b = c + 5 * ci;
      const double  b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];

      for (int l = 0; l < MPPI_LANES; l++)
      {
        acc[l] += wtouch_w * SQ(mmax_c(0.0, -touch - (py[l] + R[3][l] * b0 + R[4][l] * b1 + R[5][l] * b2 - b3)));
      }
    }
    for (int si = 0; si < ns; si++)
    {
      const double *b = c + 5 * si;
      const double  b0 = b[0], b1 = b[1], b2 = b[2], b3 = b[3];

      for (int l = 0; l < MPPI_LANES; l++)
      {
        const double yh = py[l] + R[3][l] * b0 + R[4][l] * b1 + R[5][l] * b2 - b3;

        low[l] = lm_sel(yh < low[l], yh, low[l]);
      }
    }
    const double rws = wskim_w * rf[MR_W_SKIM], rsk = rf[MR_SKIM];

    for (int l = 0; l < MPPI_LANES; l++)
    {
      acc[l] += rws * SQ(low[l] - rsk);
    }
    for (int ri = 0; ri < nr; ri++)
    {
      const double *b = r + 3 * ri;
      const double  b0 = b[0], b1 = b[1], b2 = b[2];

      for (int l = 0; l < MPPI_LANES; l++)
      {
        acc[l] += wcrash_w * SQ(mmax_c(0.0, skim - (py[l] + R[3][l] * b0 + R[4][l] * b1 + R[5][l] * b2)));
      }
    }
  }
  for (int l = 0; l < MPPI_LANES; l++)
  {
    out[l] = mmin_c(acc[l], 1e18);
    far_out[l] = far_acc[l];
  }
}

void mppi_plane_costs_lanes(const double *restrict x0, const double *restrict u,
                            const double *restrict ref, const double *restrict k,
                            const double *restrict c, int nc, int ns,
                            const double *restrict r, int nr, const double *restrict weight,
                            const double *restrict u0, int n, int h, double dt, double w_p,
                            double w_v, const double *restrict wind, double *restrict cost)
{
  const size_t stride = (size_t)h * 4U;

  for (int base = 0; base < n; base += MPPI_LANES)
  {
    const int           count = ((n - base) < MPPI_LANES) ? (n - base) : MPPI_LANES;
    const double       *u_lane[MPPI_LANES];
    _Alignas(64) double block_out[MPPI_LANES], block_far[MPPI_LANES];

    for (int l = 0; l < MPPI_LANES; l++)
    {
      const int src = (l < count) ? l : 0;

      u_lane[l] = u + (size_t)(base + src) * stride;
    }
    rollout_block(x0, u_lane, ref, k, c, nc, ns, r, nr, weight, u0, h, dt, w_p, w_v, wind, block_out, block_far);
    for (int l = 0; l < count; l++)
    {
      cost[base + l] = (block_far[l] == 0.0)
                           ? block_out[l]
                           : mppi_plane_rollout(x0, u_lane[l], ref, k, c, nc, ns, r, nr, weight, u0, h, dt, w_p, w_v, wind);
    }
  }
}
