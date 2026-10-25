/** wbc_patch.c - a bearing patch's rows (wbc_step.h): machine.wrench._patch, numpy's arithmetic and numpy's
    own OpenBLAS (wbc_blas.h) operation for operation. The segments one surface bears are a patch
    (machine.borne's cpatch): each contact's point held still, up alone where it slides along the floor's plane
    along the slide its text says it may (p->slide); and the
    patch's normal force split as it is felt - each contact's, along the patch's normal, its share of the forces
    felt along it - pinned at w_pin in shares of her weight, as a clipped force is held. */
#include "wbc_np.h"
#include "wbc_step.h"

#include <math.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})

/** Python's max of two floats: the first unless the second is past it. */
static double pymax(double a, double b)
{
  return b > a ? b : a;
}

void wbc_patch_rows(const wbc_step_t *p, int n, int nu, int nc, const double *full, const double *full0, double *S,
                    double *r, double *r0, double *felt, double *slips)
{
  const wbc_blas_t *l = &p->blas;
  const int         nu0 = p->nu0;
  const double      pin = (p->w_pin * (double)(p->stance[0] || p->stance[1])) / (p->mass * p->g), up[3] = {0.0, 1.0, 0.0};
  double            share[WBC_STEP_C], rows3[3 * WBC_STEP_U];
  int               patched[WBC_STEP_C];

  for (int q = 0; q < nc; q++)
  {
    patched[q] = 0;
  }
  for (int q = 0; q < nc; q++)
  {
    double nrm[3] = {0.0, 0.0, 0.0}, total = 0.0, count = 0.0, scale;

    if (patched[q])
    {
      continue;
    }
    /* the patch's normal, the forces felt on it summed; each contact's share of what is felt along it */
    for (int m = 0; m < nc; m++)
    {
      if (p->cpatch[m] == p->cpatch[q])
      {
        for (int i = 0; i < 3; i++)
        {
          nrm[i] += p->cf[m][i];
        }
        count += 1.0;
      }
    }
    scale = pymax(sqrt(wbc_dot(l, nrm, 1, nrm, 1, 3)), 1e-9);
    for (int i = 0; i < 3; i++)
    {
      nrm[i] = nrm[i] / scale;
    }
    for (int m = 0; m < nc; m++)
    {
      share[m] = 0.0;
      if (p->cpatch[m] == p->cpatch[q])
      {
        share[m] = pymax(wbc_dot(l, p->cf[m], 1, nrm, 1, 3), 0.0);
        felt[m] = share[m];
        total += share[m];
      }
    }
    for (int m = 0; m < nc; m++)
    {
      if (p->cpatch[m] == p->cpatch[q])
      {
        share[m] = total > 0.0 ? share[m] / total : 1.0 / count;
      }
    }
    for (int m = 0; m < nc; m++)
    {
      double left[3], jv[3], way[2], said[2], r3[3], mix[9], sliding;

      if (p->cpatch[m] != p->cpatch[q])
      {
        continue;
      }
      /* its point held still, along the patch's normal alone where it slides across it (machine.wrench._patch) */
      wbc_matmul(l, VIEW(p->cJ[m], n, 1), VIEW(full0, 1, 0), left, 1, 0, 3, n, 1);
      wbc_matmul(l, VIEW(p->cJ[m], n, 1), VIEW(full, nu, 1), rows3, nu, 1, 3, n, nu);
      wbc_matmul(l, VIEW(p->cJ[m], n, 1), VIEW(p->v, 1, 0), jv, 1, 0, 3, n, 1);
      way[0] = jv[0];
      way[1] = jv[2];
      said[0] = p->slide[0];
      said[1] = p->slide[2];
      sliding = (hypot(way[0], way[1]) > p->slide_m_s && wbc_dot(l, way, 1, said, 1, 2) > 0.0) ? 1.0 : 0.0;
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < 3; j++)
        {
          mix[3 * i + j] = (1.0 - sliding) * (double)(i == j) + sliding * (up[i] * up[j]);
        }
        left[i] = (-p->cdJv[m][i]) - left[i];
        r3[i] = left[i] - p->contact_k * jv[i];
      }
      wbc_matmul(l, VIEW(mix, 3, 1), VIEW(rows3, nu, 1), S + 4 * nu * m, nu, 1, 3, 3, nu);
      wbc_matmul(l, VIEW(mix, 3, 1), VIEW(r3, 1, 0), r + 4 * m, 1, 0, 3, 3, 1);
      wbc_matmul(l, VIEW(mix, 3, 1), VIEW(left, 1, 0), r0 + 4 * m, 1, 0, 3, 3, 1);
      for (int i = 0; i < 3; i++)
      {
        for (int j = 0; j < nu; j++)
        {
          S[4 * nu * m + nu * i + j] = p->w_still * S[4 * nu * m + nu * i + j];
        }
        r[4 * m + i] = p->w_still * r[4 * m + i];
        r0[4 * m + i] = p->w_still * r0[4 * m + i];
      }
      slips[3 * m] = sliding;
      slips[3 * m + 1] = way[0];
      slips[3 * m + 2] = way[1];
      /* the patch's normal force split as felt: this contact's less its share of every member's */
      for (int j = 0; j < nu; j++)
      {
        S[4 * nu * m + 3 * nu + j] = 0.0;
      }
      for (int k = 0; k < nc; k++)
      {
        if (p->cpatch[k] == p->cpatch[q])
        {
          const double d = (k == m ? 1.0 : 0.0) - share[m];

          for (int i = 0; i < 3; i++)
          {
            S[4 * nu * m + 3 * nu + nu0 + 3 * k + i] = pin * (nrm[i] * d);
          }
        }
      }
      r[4 * m + 3] = 0.0;
      r0[4 * m + 3] = 0.0;
      patched[m] = 1;
    }
  }
}

void wbc_patch_caps(const wbc_step_t *p, int nc, const double *felt, const double *slips, double *caps, double *borne)
{
  double hands[WBC_STEP_C], off_hands[WBC_STEP_C], others[WBC_STEP_C], total, rest;

  for (int q = 0; q < nc; q++)
  {
    hands[q] = (double)(p->cpatch[q] >= p->carpal);
    off_hands[q] = felt[q] * (1.0 - hands[q]);
    others[q] = 1.0 - hands[q];
    borne[hands[q] > 0.0 ? p->cpatch[q] / p->carpal - 1 : 2] = 1.0;
  }
  total = npsum(off_hands, nc);
  rest = npsum(others, nc);
  for (int q = 0; q < nc; q++)
  {
    const double lim = hands[q] > 0.0 ? p->hand_cap : slips[3 * q] > 0.0 ? fmin(p->fz_cap, p->slid_cap) : p->fz_cap;
    const double share = (0.0 + (total > 0.0 ? off_hands[q] / total : others[q] / npmax(rest, 1.0))) + hands[q];

    caps[q] = isfinite(lim) ? lim * share : INFINITY;
  }
}
