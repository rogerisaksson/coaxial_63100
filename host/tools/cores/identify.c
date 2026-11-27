/** identify.c - SIMD C11 capsule self-contact, hem kinematics, and low-pass kernels for sysid. */
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "wbc_libm.h"
#include "wbc_np.h"

#define ID_MAX_BODIES 64
#define ID_MAX_JOINTS 64
#define ID_MAX_CAPS   96
#define ID_MAX_CHAIN  32
#define ID_LANES      8

/** Bidirectional first-order low-pass filter on contiguous `(n, cols)` row-major float64 array. */
void id_lowpass(double *restrict x, int n, int cols, double a)
{
  if (x == NULL || n < 2 || cols < 1)
  {
    return;
  }
  const size_t w = (size_t)cols;
  for (int k = 1; k < n; k++)
  {
    double *restrict       cur = x + (size_t)k * w;
    const double *restrict prv = cur - w;
    for (int c = 0; c < cols; c++)
    {
      cur[c] += a * (prv[c] - cur[c]);
    }
  }
  for (int k = n - 2; k >= 0; k--)
  {
    double *restrict       cur = x + (size_t)k * w;
    const double *restrict nxt = cur + w;
    for (int c = 0; c < cols; c++)
    {
      cur[c] += a * (nxt[c] - cur[c]);
    }
  }
}

static inline void id_quat_rot(double qw, double qx, double qy, double qz, double r[9])
{
  const double xx = qx * qx, yy = qy * qy, zz = qz * qz;
  const double xy = qx * qy, xz = qx * qz, yz = qy * qz;
  const double wx = qw * qx, wy = qw * qy, wz = qw * qz;

  r[0] = 1.0 - 2.0 * (yy + zz);
  r[1] = 2.0 * (xy - wz);
  r[2] = 2.0 * (xz + wy);
  r[3] = 2.0 * (xy + wz);
  r[4] = 1.0 - 2.0 * (xx + zz);
  r[5] = 2.0 * (yz - wx);
  r[6] = 2.0 * (xz - wy);
  r[7] = 2.0 * (yz + wx);
  r[8] = 1.0 - 2.0 * (xx + yy);
}

static void id_pose_block(int m, int stride, int quat_at, int q_at, const double *restrict ys,
                          int nb, const int32_t *restrict parent, const double *restrict b_pos,
                          const double *restrict b_rot, const int32_t *restrict j_span,
                          const int32_t *restrict j_idx, const double *restrict j_ax,
                          const double *restrict j_pos, const uint8_t *restrict j_has_pos,
                          double xpos[ID_MAX_BODIES][3][ID_LANES],
                          double xmat[ID_MAX_BODIES][9][ID_LANES])
{
  for (int l = 0; l < ID_LANES; l++)
  {
    const int     row = (l < m) ? l : 0;
    const double *q = ys + (size_t)row * (size_t)stride + (size_t)quat_at;
    double        r0[9];

    id_quat_rot(q[0], q[1], q[2], q[3], r0);
    xpos[0][0][l] = 0.0;
    xpos[0][1][l] = 0.0;
    xpos[0][2][l] = 0.0;
    for (int d = 0; d < 9; d++)
    {
      xmat[0][d][l] = r0[d];
    }
  }

  for (int k = 1; k < nb; k++)
  {
    const int     pk = (int)parent[k];
    const double *bp = b_pos + 3 * k;
    const double *br = b_rot + 9 * k;
    const int     j0 = (int)j_span[2 * k];
    const int     j1 = (int)j_span[2 * k + 1];

    for (int i = 0; i < 3; i++)
    {
      for (int l = 0; l < ID_LANES; l++)
      {
        xpos[k][i][l] = xpos[pk][i][l]
                        + ((xmat[pk][3 * i][l] * bp[0] + xmat[pk][3 * i + 1][l] * bp[1])
                           + xmat[pk][3 * i + 2][l] * bp[2]);
      }
      for (int j = 0; j < 3; j++)
      {
        const double b0 = br[j], b1 = br[3 + j], b2 = br[6 + j];
        for (int l = 0; l < ID_LANES; l++)
        {
          xmat[k][3 * i + j][l] = (xmat[pk][3 * i][l] * b0 + xmat[pk][3 * i + 1][l] * b1)
                                  + xmat[pk][3 * i + 2][l] * b2;
        }
      }
    }

    for (int jn = j0; jn < j1; jn++)
    {
      const int     ji = (int)j_idx[jn];
      const double *ax = j_ax + 3 * jn;
      const double *jp = j_pos + 3 * jn;
      const uint8_t has_jp = j_has_pos[jn];
      double        dr[9][ID_LANES];
      double        rn[9][ID_LANES];

      for (int l = 0; l < ID_LANES; l++)
      {
        const int    row = (l < m) ? l : 0;
        const double hq = 0.5 * ys[(size_t)row * (size_t)stride + (size_t)(q_at + ji)];
        const double cq = (fabs(hq) < LM_FAR) ? lm_cos_near(hq) : wbc_cos(hq);
        const double sq = (fabs(hq) < LM_FAR) ? lm_sin_near(hq) : wbc_sin(hq);
        double       d_one[9];

        id_quat_rot(cq, sq * ax[0], sq * ax[1], sq * ax[2], d_one);
        for (int d = 0; d < 9; d++)
        {
          dr[d][l] = d_one[d];
        }
      }

      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          for (int l = 0; l < ID_LANES; l++)
          {
            rn[3 * i + j][l] = (xmat[k][3 * i][l] * dr[j][l] + xmat[k][3 * i + 1][l] * dr[3 + j][l])
                               + xmat[k][3 * i + 2][l] * dr[6 + j][l];
          }
        }
      }

      if (has_jp != 0u)
      {
        for (int i = 0; i < 3; i++)
        {
          for (int l = 0; l < ID_LANES; l++)
          {
            const double r_jp = (xmat[k][3 * i][l] * jp[0] + xmat[k][3 * i + 1][l] * jp[1])
                                + xmat[k][3 * i + 2][l] * jp[2];
            const double rn_jp = (rn[3 * i][l] * jp[0] + rn[3 * i + 1][l] * jp[1])
                                 + rn[3 * i + 2][l] * jp[2];
            xpos[k][i][l] = (xpos[k][i][l] + r_jp) - rn_jp;
          }
        }
      }

      for (int d = 0; d < 9; d++)
      {
        for (int l = 0; l < ID_LANES; l++)
        {
          xmat[k][d][l] = rn[d][l];
        }
      }
    }
  }
}

static void id_caps_block(int ncaps, const int32_t *restrict ck, const double *restrict ca,
                          const double *restrict cb, const double xpos[ID_MAX_BODIES][3][ID_LANES],
                          const double xmat[ID_MAX_BODIES][9][ID_LANES],
                          double pa[ID_MAX_CAPS][3][ID_LANES], double pb[ID_MAX_CAPS][3][ID_LANES],
                          double mid[ID_MAX_CAPS][3][ID_LANES])
{
  for (int c = 0; c < ncaps; c++)
  {
    const int     b = (int)ck[c];
    const double * end_a = ca + 3 * c;
    const double * end_b = cb + 3 * c;

    for (int i = 0; i < 3; i++)
    {
      for (int l = 0; l < ID_LANES; l++)
      {
        const double va = xpos[b][i][l]
                          + ((xmat[b][3 * i][l] * end_a[0] + xmat[b][3 * i + 1][l] * end_a[1])
                             + xmat[b][3 * i + 2][l] * end_a[2]);
        const double vb = xpos[b][i][l]
                          + ((xmat[b][3 * i][l] * end_b[0] + xmat[b][3 * i + 1][l] * end_b[1])
                             + xmat[b][3 * i + 2][l] * end_b[2]);
        pa[c][i][l] = va;
        pb[c][i][l] = vb;
        mid[c][i][l] = 0.5 * (va + vb);
      }
    }
  }
}

static int id_segment_over(double p1x, double p1y, double p1z, double q1x, double q1y, double q1z,
                           double p2x, double p2y, double p2z, double q2x, double q2y, double q2z,
                           double rad_sq)
{
  const double d1x = q1x - p1x, d1y = q1y - p1y, d1z = q1z - p1z;
  const double d2x = q2x - p2x, d2y = q2y - p2y, d2z = q2z - p2z;
  const double rx = p1x - p2x, ry = p1y - p2y, rz = p1z - p2z;
  const double a = (d1x * d1x + d1y * d1y) + d1z * d1z;
  const double e = (d2x * d2x + d2y * d2y) + d2z * d2z;
  const double f = (d2x * rx + d2y * ry) + d2z * rz;
  const double c = (d1x * rx + d1y * ry) + d1z * rz;
  const double b = (d1x * d2x + d1y * d2y) + d1z * d2z;
  const double den = a * e - b * b;
  double       s = (den > 1e-12) ? npclip((b * f - c * e) / npmaximum(den, 1e-12), 0.0, 1.0) : 0.0;
  double       t = (e > 1e-12) ? ((b * s + f) / npmaximum(e, 1e-12)) : 0.0;
  const int    lo = (t < 0.0);
  const int    hi = (t > 1.0);

  t = npclip(t, 0.0, 1.0);
  if (lo)
  {
    s = (a > 1e-12) ? npclip((-c) / npmaximum(a, 1e-12), 0.0, 1.0) : 0.0;
  }
  if (hi)
  {
    s = (a > 1e-12) ? npclip((b - c) / npmaximum(a, 1e-12), 0.0, 1.0) : 0.0;
  }
  const double dx = (p1x + s * d1x) - (p2x + t * d2x);
  const double dy = (p1y + s * d1y) - (p2y + t * d2y);
  const double dz = (p1z + s * d1z) - (p2z + t * d2z);
  return ((dx * dx + dy * dy) + dz * dz) < rad_sq;
}

/** Evaluate capsule-pair self-overlaps across `n` edge rows and project onto `ndrv` driven joints. */
int id_self_hit(int n, int stride, int quat_at, int q_at, const double *restrict ys,
                int nb, const int32_t *restrict parent, const double *restrict b_pos,
                const double *restrict b_rot, const int32_t *restrict j_span,
                int nj_tot, const int32_t *restrict j_idx, const double *restrict j_ax,
                const double *restrict j_pos, const uint8_t *restrict j_has_pos,
                int ncaps, const int32_t *restrict ck, const double *restrict ca,
                const double *restrict cb, int npairs, const int32_t *restrict pair_a,
                const int32_t *restrict pair_b, const double *restrict reach_sq,
                const double *restrict rad_sq, int ndrv, const uint8_t *restrict pdrv,
                const uint64_t *restrict pdrv_mask, uint8_t *restrict out)
{
  if (nb > ID_MAX_BODIES || nj_tot > ID_MAX_JOINTS || ncaps > ID_MAX_CAPS || ndrv < 0)
  {
    return -1;
  }
  if (n <= 0 || ndrv == 0)
  {
    return 0;
  }
  memset(out, 0, (size_t)n * (size_t)ndrv);
  if (npairs <= 0)
  {
    return 0;
  }

  _Alignas(64) double xpos[ID_MAX_BODIES][3][ID_LANES];
  _Alignas(64) double xmat[ID_MAX_BODIES][9][ID_LANES];
  _Alignas(64) double pa[ID_MAX_CAPS][3][ID_LANES];
  _Alignas(64) double pb[ID_MAX_CAPS][3][ID_LANES];
  _Alignas(64) double mid[ID_MAX_CAPS][3][ID_LANES];

  for (int base_t = 0; base_t < n; base_t += ID_LANES)
  {
    const int     m = (n - base_t < ID_LANES) ? (n - base_t) : ID_LANES;
    const double *ys_blk = ys + (size_t)base_t * (size_t)stride;
    uint64_t      masks[ID_LANES] = {0u, 0u, 0u, 0u, 0u, 0u, 0u, 0u};

    id_pose_block(m, stride, quat_at, q_at, ys_blk, nb, parent, b_pos, b_rot, j_span,
                  j_idx, j_ax, j_pos, j_has_pos, xpos, xmat);
    id_caps_block(ncaps, ck, ca, cb, xpos, xmat, pa, pb, mid);

    for (int p = 0; p < npairs; p++)
    {
      const int    ia = (int)pair_a[p];
      const int    ib = (int)pair_b[p];
      const double rsq = reach_sq[p];
      double       dm2[ID_LANES];
      int          any_near = 0;

      for (int l = 0; l < ID_LANES; l++)
      {
        const double mx = mid[ia][0][l] - mid[ib][0][l];
        const double my = mid[ia][1][l] - mid[ib][1][l];
        const double mz = mid[ia][2][l] - mid[ib][2][l];
        dm2[l] = (mx * mx + my * my) + mz * mz;
      }
      for (int l = 0; l < m; l++)
      {
        any_near |= (dm2[l] < rsq);
      }
      if (!any_near)
      {
        continue;
      }
      const double csq = rad_sq[p];
      for (int l = 0; l < m; l++)
      {
        if (dm2[l] < rsq
            && id_segment_over(pa[ia][0][l], pa[ia][1][l], pa[ia][2][l],
                               pb[ia][0][l], pb[ia][1][l], pb[ia][2][l],
                               pa[ib][0][l], pa[ib][1][l], pa[ib][2][l],
                               pb[ib][0][l], pb[ib][1][l], pb[ib][2][l], csq))
        {
          if (ndrv <= 64)
          {
            masks[l] |= pdrv_mask[p];
          }
          else
          {
            uint8_t       *dst = out + (size_t)(base_t + l) * (size_t)ndrv;
            const uint8_t *src = pdrv + (size_t)p * (size_t)ndrv;
            for (int d = 0; d < ndrv; d++)
            {
              dst[d] |= src[d];
            }
          }
        }
      }
    }

    if (ndrv <= 64)
    {
      for (int l = 0; l < m; l++)
      {
        const uint64_t mk = masks[l];
        if (mk != 0u)
        {
          uint8_t *dst = out + (size_t)(base_t + l) * (size_t)ndrv;
          for (int d = 0; d < ndrv; d++)
          {
            dst[d] = (uint8_t)((mk >> d) & 1u);
          }
        }
      }
    }
  }
  return 0;
}

static inline void id_cross_lanes(const double u[3][ID_LANES], const double v[3][ID_LANES],
                                  double c[3][ID_LANES])
{
  for (int l = 0; l < ID_LANES; l++)
  {
    c[0][l] = u[1][l] * v[2][l] - u[2][l] * v[1][l];
    c[1][l] = u[2][l] * v[0][l] - u[0][l] * v[2][l];
    c[2][l] = u[0][l] * v[1][l] - u[1][l] * v[0][l];
  }
}

static void id_hem_init_block(int m, int stride, int quat_at, int gyro_at, int accel_at, double dt,
                              const double *restrict ys, double R[9][ID_LANES],
                              double wv[3][ID_LANES], double al[3][ID_LANES], double a[3][ID_LANES])
{
  for (int l = 0; l < ID_LANES; l++)
  {
    const int     row = (l < m) ? l : 0;
    const double *r0_row = ys + (size_t)row * (size_t)stride;
    const double *r1_row = ys + (size_t)(row + 1) * (size_t)stride;
    const double *q0 = r0_row + quat_at;
    const double *q1 = r1_row + quat_at;
    const double *g0 = r0_row + gyro_at;
    const double *g1 = r1_row + gyro_at;
    const double *acc1 = r1_row + accel_at;
    double        rot0[9], rot1[9], dg[3];

    id_quat_rot(q0[0], q0[1], q0[2], q0[3], rot0);
    id_quat_rot(q1[0], q1[1], q1[2], q1[3], rot1);
    dg[0] = (g1[0] - g0[0]) / dt;
    dg[1] = (g1[1] - g0[1]) / dt;
    dg[2] = (g1[2] - g0[2]) / dt;
    for (int d = 0; d < 9; d++)
    {
      R[d][l] = rot0[d];
    }
    for (int i = 0; i < 3; i++)
    {
      wv[i][l] = (rot0[3 * i] * g0[0] + rot0[3 * i + 1] * g0[1]) + rot0[3 * i + 2] * g0[2];
      al[i][l] = (rot0[3 * i] * dg[0] + rot0[3 * i + 1] * dg[1]) + rot0[3 * i + 2] * dg[2];
      a[i][l] = (rot1[3 * i] * acc1[0] + rot1[3 * i + 1] * acc1[1]) + rot1[3 * i + 2] * acc1[2];
    }
  }
}

static void id_hem_link_step(int m, int stride, int q_at, int qd_at, double dt,
                             const double *restrict ys, const double bp[3], const double br[9],
                             int j0, int j1, const int32_t *restrict j_idx,
                             const double *restrict j_ax, double R[9][ID_LANES],
                             double wv[3][ID_LANES], double al[3][ID_LANES], double a[3][ID_LANES])
{
  double dp[3][ID_LANES], wdp[3][ID_LANES], wwdp[3][ID_LANES], aldp[3][ID_LANES], rn[9][ID_LANES];

  for (int i = 0; i < 3; i++)
  {
    for (int l = 0; l < ID_LANES; l++)
    {
      dp[i][l] = (R[3 * i][l] * bp[0] + R[3 * i + 1][l] * bp[1]) + R[3 * i + 2][l] * bp[2];
    }
  }
  id_cross_lanes(al, dp, aldp);
  id_cross_lanes(wv, dp, wdp);
  id_cross_lanes(wv, wdp, wwdp);
  for (int i = 0; i < 3; i++)
  {
    for (int l = 0; l < ID_LANES; l++)
    {
      a[i][l] = (a[i][l] + aldp[i][l]) + wwdp[i][l];
    }
    for (int j = 0; j < 3; j++)
    {
      const double b0 = br[j], b1 = br[3 + j], b2 = br[6 + j];
      for (int l = 0; l < ID_LANES; l++)
      {
        rn[3 * i + j][l] = (R[3 * i][l] * b0 + R[3 * i + 1][l] * b1) + R[3 * i + 2][l] * b2;
      }
    }
  }
  for (int d = 0; d < 9; d++)
  {
    for (int l = 0; l < ID_LANES; l++)
    {
      R[d][l] = rn[d][l];
    }
  }
  for (int jn = j0; jn < j1; jn++)
  {
    const int     ji = (int)j_idx[jn];
    const double *ax = j_ax + 3 * jn;
    double        sw[3][ID_LANES], wsw[3][ID_LANES], dr[9][ID_LANES];
    double        qd_v[ID_LANES], qdd_v[ID_LANES];

    for (int i = 0; i < 3; i++)
    {
      for (int l = 0; l < ID_LANES; l++)
      {
        sw[i][l] = (R[3 * i][l] * ax[0] + R[3 * i + 1][l] * ax[1]) + R[3 * i + 2][l] * ax[2];
      }
    }
    id_cross_lanes(wv, sw, wsw);
    for (int l = 0; l < ID_LANES; l++)
    {
      const int     row = (l < m) ? l : 0;
      const double *r0_row = ys + (size_t)row * (size_t)stride;
      const double *r1_row = ys + (size_t)(row + 1) * (size_t)stride;
      const double  hq = 0.5 * r0_row[q_at + ji];
      const double  qd0 = r0_row[qd_at + ji];
      const double  cq = (fabs(hq) < LM_FAR) ? lm_cos_near(hq) : wbc_cos(hq);
      const double  sq = (fabs(hq) < LM_FAR) ? lm_sin_near(hq) : wbc_sin(hq);
      double        d_one[9];

      qd_v[l] = qd0;
      qdd_v[l] = (r1_row[qd_at + ji] - qd0) / dt;
      id_quat_rot(cq, sq * ax[0], sq * ax[1], sq * ax[2], d_one);
      for (int d = 0; d < 9; d++)
      {
        dr[d][l] = d_one[d];
      }
    }
    for (int i = 0; i < 3; i++)
    {
      for (int l = 0; l < ID_LANES; l++)
      {
        al[i][l] = (al[i][l] + sw[i][l] * qdd_v[l]) + wsw[i][l] * qd_v[l];
        wv[i][l] = wv[i][l] + sw[i][l] * qd_v[l];
      }
      for (int j = 0; j < 3; j++)
      {
        for (int l = 0; l < ID_LANES; l++)
        {
          rn[3 * i + j][l] = (R[3 * i][l] * dr[j][l] + R[3 * i + 1][l] * dr[3 + j][l])
                             + R[3 * i + 2][l] * dr[6 + j][l];
        }
      }
    }
    for (int d = 0; d < 9; d++)
    {
      for (int l = 0; l < ID_LANES; l++)
      {
        R[d][l] = rn[d][l];
      }
    }
  }
}

/** Propagate pelvic IMU and encoder chain kinematics to hem anchor `(ap, om, alb)` across `n` steps. */
int id_hem_anchor(int n, int stride, int quat_at, int gyro_at, int accel_at, int q_at, int qd_at,
                  double dt, const double *restrict ys, int nlinks, const double *restrict link_pos,
                  const double *restrict link_rot, const int32_t *restrict j_span, int nj_tot,
                  const int32_t *restrict j_idx, const double *restrict j_ax,
                  const double *restrict end_pos, double *restrict ap_out,
                  double *restrict om_out, double *restrict alb_out)
{
  if (nlinks < 0 || nlinks > ID_MAX_CHAIN || nj_tot < 0 || nj_tot > ID_MAX_JOINTS || dt <= 0.0)
  {
    return -1;
  }
  if (n <= 0)
  {
    return 0;
  }

  _Alignas(64) double R[9][ID_LANES];
  _Alignas(64) double wv[3][ID_LANES];
  _Alignas(64) double al[3][ID_LANES];
  _Alignas(64) double a[3][ID_LANES];
  _Alignas(64) double dp[3][ID_LANES];
  _Alignas(64) double wdp[3][ID_LANES];
  _Alignas(64) double wwdp[3][ID_LANES];
  _Alignas(64) double aldp[3][ID_LANES];
  _Alignas(64) double atot[3][ID_LANES];

  for (int base_t = 0; base_t < n; base_t += ID_LANES)
  {
    const int     m = (n - base_t < ID_LANES) ? (n - base_t) : ID_LANES;
    const double *ys_blk = ys + (size_t)base_t * (size_t)stride;

    id_hem_init_block(m, stride, quat_at, gyro_at, accel_at, dt, ys_blk, R, wv, al, a);
    for (int lk = 0; lk < nlinks; lk++)
    {
      id_hem_link_step(m, stride, q_at, qd_at, dt, ys_blk, link_pos + 3 * lk, link_rot + 9 * lk,
                       (int)j_span[2 * lk], (int)j_span[2 * lk + 1], j_idx, j_ax, R, wv, al, a);
    }
    for (int i = 0; i < 3; i++)
    {
      for (int l = 0; l < ID_LANES; l++)
      {
        dp[i][l] = (R[3 * i][l] * end_pos[0] + R[3 * i + 1][l] * end_pos[1])
                   + R[3 * i + 2][l] * end_pos[2];
      }
    }
    id_cross_lanes(al, dp, aldp);
    id_cross_lanes(wv, dp, wdp);
    id_cross_lanes(wv, wdp, wwdp);
    for (int i = 0; i < 3; i++)
    {
      for (int l = 0; l < ID_LANES; l++)
      {
        atot[i][l] = (a[i][l] + aldp[i][l]) + wwdp[i][l];
      }
    }
    for (int l = 0; l < m; l++)
    {
      double *ap_r = ap_out + (size_t)(base_t + l) * 3u;
      double *om_r = om_out + (size_t)(base_t + l) * 3u;
      double *ab_r = alb_out + (size_t)(base_t + l) * 3u;
      for (int i = 0; i < 3; i++)
      {
        ap_r[i] = (R[i][l] * atot[0][l] + R[3 + i][l] * atot[1][l]) + R[6 + i][l] * atot[2][l];
        om_r[i] = (R[i][l] * wv[0][l] + R[3 + i][l] * wv[1][l]) + R[6 + i][l] * wv[2][l];
        ab_r[i] = (R[i][l] * al[0][l] + R[3 + i][l] * al[1][l]) + R[6 + i][l] * al[2][l];
      }
    }
  }
  return 0;
}
