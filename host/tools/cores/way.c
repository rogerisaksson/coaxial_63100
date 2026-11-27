/** way.c - Bit-exact C11 wayfinding distance, separating-axis footprint gap, and 8-way A* kernels. */
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "wbc_libm_arc.h"

#define WY_MAX_K 128

void wy_apart(int32_t n_pts, const double *restrict pts, int32_t m_things, const double *restrict centres,
              const double *restrict axes, const double *restrict half, double *restrict out)
{
  for (int32_t i = 0; i < n_pts; i++)
  {
    const double  px = pts[(size_t)i * 2U + 0U];
    const double  pz = pts[(size_t)i * 2U + 1U];
    double       *row = &out[(size_t)i * (size_t)m_things];
    for (int32_t j = 0; j < m_things; j++)
    {
      const double off0 = px - centres[(size_t)j * 2U + 0U];
      const double off1 = pz - centres[(size_t)j * 2U + 1U];
      const size_t ao = (size_t)j * 4U;
      const double t0 = (0.0 + axes[ao + 0U] * off0) + axes[ao + 1U] * off1;
      const double t1 = (0.0 + axes[ao + 2U] * off0) + axes[ao + 3U] * off1;
      const double r0 = fabs(t0) - half[(size_t)j * 2U + 0U];
      const double r1 = fabs(t1) - half[(size_t)j * 2U + 1U];
      const double d0 = lm_sel(r0 <= 0.0, 0.0, r0);
      const double d1 = lm_sel(r1 <= 0.0, 0.0, r1);
      row[j] = wbc_hypot(d0, d1);
    }
  }
}

int32_t wy_gaps(int32_t n_poses, const double *restrict at_pts, const double *restrict ahead,
                const double *restrict left, const double *restrict dirs, int32_t k_dirs, int32_t m_things,
                const uint8_t *restrict meets, const double *restrict toward, const double *restrict centres,
                const double *restrict axes, const double *restrict half, double two_pi, double step_rad,
                double cos_pi_k, double *restrict out)
{
  double at_dot_d[WY_MAX_K];

  if (k_dirs <= 0 || k_dirs > WY_MAX_K)
  {
    return -1;
  }
  for (int32_t n = 0; n < n_poses; n++)
  {
    const double  at0 = at_pts[(size_t)n * 2U + 0U];
    const double  at1 = at_pts[(size_t)n * 2U + 1U];
    const double  ah0 = ahead[(size_t)n * 2U + 0U];
    const double  ah1 = ahead[(size_t)n * 2U + 1U];
    const double  lf0 = left[(size_t)n * 2U + 0U];
    const double  lf1 = left[(size_t)n * 2U + 1U];
    const double *ndirs = &dirs[(size_t)n * (size_t)k_dirs * 2U];
    double       *orow = &out[(size_t)n * (size_t)m_things];

    for (int32_t k = 0; k < k_dirs; k++)
    {
      const double d0 = ndirs[(size_t)k * 2U + 0U];
      const double d1 = ndirs[(size_t)k * 2U + 1U];
      at_dot_d[k] = (0.0 + at0 * d0) + at1 * d1;
    }
    for (int32_t m = 0; m < m_things; m++)
    {
      if (meets[m] == 0U)
      {
        orow[m] = INFINITY;
        continue;
      }
      const double  c0 = centres[(size_t)m * 2U + 0U];
      const double  c1 = centres[(size_t)m * 2U + 1U];
      const size_t  ao = (size_t)m * 4U;
      const double  a00 = axes[ao + 0U], a01 = axes[ao + 1U];
      const double  a10 = axes[ao + 2U], a11 = axes[ao + 3U];
      const double  h0 = half[(size_t)m * 2U + 0U];
      const double  h1 = half[(size_t)m * 2U + 1U];
      const double *tw = &toward[(size_t)m * (size_t)k_dirs];
      double        mine = -INFINITY;

      for (int32_t k = 0; k < k_dirs; k++)
      {
        const double d0 = ndirs[(size_t)k * 2U + 0U];
        const double d1 = ndirs[(size_t)k * 2U + 1U];
        const double sp0 = fabs((0.0 + a00 * d0) + a01 * d1);
        const double sp1 = fabs((0.0 + a10 * d0) + a11 * d1);
        const double least = ((0.0 + c0 * d0) + c1 * d1) - ((0.0 + sp0 * h0) + sp1 * h1);
        const double v = (least - at_dot_d[k]) - tw[k];
        if (k == 0 || v > mine || lm_nan(v))
        {
          mine = lm_nan(mine) ? mine : v;
        }
      }
      const double ax_list[4][2] = {{a00, a01}, {a10, a11}, {-a00, -a01}, {-a10, -a11}};
      const double hb_list[4] = {h0, h1, h0, h1};
      double       theirs = -INFINITY;

      for (int32_t a = 0; a < 4; a++)
      {
        const double ax0 = ax_list[a][0], ax1 = ax_list[a][1];
        const double y_val = (0.0 + ax0 * lf0) + ax1 * lf1;
        const double x_val = (0.0 + ax0 * ah0) + ax1 * ah1;
        double       ang = fmod(wbc_arctan2(y_val, x_val), two_pi);
        if (lm_nan(ang))
        {
          theirs = ang;
          break;
        }
        if (ang != 0.0)
        {
          if (ang < 0.0)
          {
            ang += two_pi;
          }
        }
        else
        {
          ang = 0.0;
        }
        int32_t bin0 = ((int32_t)floor(ang / step_rad)) % k_dirs;
        if (bin0 < 0)
        {
          bin0 += k_dirs;
        }
        const int32_t bin1 = (bin0 + 1) % k_dirs;
        const double  tw_max = (tw[bin0] >= tw[bin1]) ? tw[bin0] : tw[bin1];
        const double  bound = tw_max / cos_pi_k;
        const double  c_ax = (0.0 + c0 * ax0) + c1 * ax1;
        const double  at_ax = (0.0 + at0 * ax0) + at1 * ax1;
        const double  v = ((c_ax - hb_list[a]) - at_ax) - bound;
        if (a == 0 || v > theirs || lm_nan(v))
        {
          theirs = lm_nan(theirs) ? theirs : v;
        }
      }
      orow[m] = lm_nan(mine) ? mine : (lm_nan(theirs) ? theirs : ((mine >= theirs) ? mine : theirs));
    }
  }
  return 0;
}

static inline bool wy_heap_less(double fa, int32_t na, double fb, int32_t nb)
{
  return (fa < fb) || ((fa == fb) && (na < nb));
}

int32_t wy_astar(const double *restrict cost, int32_t nx, int32_t nz, int32_t si, int32_t sj, int32_t gi, int32_t gj,
                 double *restrict spent, int32_t *restrict came, double *restrict heap_f, int32_t *restrict heap_n,
                 int32_t heap_cap, int32_t *restrict out_path)
{
  static const int32_t di[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int32_t dj[8] = {0, 0, 1, -1, 1, -1, 1, -1};
  const double         root2 = sqrt(2.0);
  const double         step_w[8] = {1.0, 1.0, 1.0, 1.0, root2, root2, root2, root2};
  const int32_t        ncells = nx * nz;

  if (si < 0 || si >= nx || sj < 0 || sj >= nz || gi < 0 || gi >= nx || gj < 0 || gj >= nz)
  {
    return 0;
  }
  for (int32_t k = 0; k < ncells; k++)
  {
    spent[k] = INFINITY;
    came[k] = -1;
  }
  const int32_t start_n = si * nz + sj;
  const int32_t goal_n = gi * nz + gj;
  spent[start_n] = 0.0;
  int32_t hlen = 1;
  heap_f[0] = wbc_hypot((double)(si - gi), (double)(sj - gj));
  heap_n[0] = start_n;

  while (hlen > 0)
  {
    const int32_t here = heap_n[0];
    hlen--;
    if (hlen > 0)
    {
      const double  lf = heap_f[hlen];
      const int32_t ln = heap_n[hlen];
      int32_t       idx = 0;
      for (;;)
      {
        int32_t left_c = 2 * idx + 1;
        if (left_c >= hlen)
        {
          break;
        }
        int32_t best_c = left_c;
        int32_t right_c = left_c + 1;
        if (right_c < hlen && wy_heap_less(heap_f[right_c], heap_n[right_c], heap_f[left_c], heap_n[left_c]))
        {
          best_c = right_c;
        }
        if (!wy_heap_less(heap_f[best_c], heap_n[best_c], lf, ln))
        {
          break;
        }
        heap_f[idx] = heap_f[best_c];
        heap_n[idx] = heap_n[best_c];
        idx = best_c;
      }
      heap_f[idx] = lf;
      heap_n[idx] = ln;
    }
    if (here == goal_n)
    {
      int32_t len = 0;
      for (int32_t cur = goal_n; cur >= 0; cur = came[cur])
      {
        len++;
      }
      int32_t pos = len - 1;
      for (int32_t cur = goal_n; cur >= 0; cur = came[cur])
      {
        out_path[(size_t)pos * 2U + 0U] = cur / nz;
        out_path[(size_t)pos * 2U + 1U] = cur % nz;
        pos--;
      }
      return len;
    }
    const int32_t hi = here / nz, hj = here % nz;
    const double  chere = cost[here];
    const double  shere = spent[here];
    for (int32_t dir = 0; dir < 8; dir++)
    {
      const int32_t ni = hi + di[dir], nj = hj + dj[dir];
      if (ni < 0 || ni >= nx || nj < 0 || nj >= nz)
      {
        continue;
      }
      const int32_t nxt = ni * nz + nj;
      const double  cnxt = cost[nxt];
      if (isinf(cnxt))
      {
        continue;
      }
      if (dir >= 4 && (isinf(cost[(hi + di[dir]) * nz + hj]) || isinf(cost[hi * nz + (hj + dj[dir])])))
      {
        continue;
      }
      const double cand = shere + step_w[dir] * 0.5 * (chere + cnxt);
      if (cand < spent[nxt])
      {
        if (hlen >= heap_cap)
        {
          return -1;
        }
        spent[nxt] = cand;
        came[nxt] = here;
        const double nf = cand + wbc_hypot((double)(ni - gi), (double)(nj - gj));
        int32_t      idx = hlen++;
        while (idx > 0)
        {
          const int32_t par = (idx - 1) / 2;
          if (!wy_heap_less(nf, nxt, heap_f[par], heap_n[par]))
          {
            break;
          }
          heap_f[idx] = heap_f[par];
          heap_n[idx] = heap_n[par];
          idx = par;
        }
        heap_f[idx] = nf;
        heap_n[idx] = nxt;
      }
    }
  }
  return 0;
}
