/** wbc_law.c - her latent loop's step and her upper body's coupling on the host (wbc_law.h):
    machine.latent.step and machine.support.couple, numpy's arithmetic and numpy's own OpenBLAS
    (wbc_blas.h) operation for operation. Portable C11, the host's alone. */
#include "wbc_law.h"
#include "wbc_np.h"

#include <math.h>

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})

/** numpy's clip (clip.cpp): nan kept, a bound where the value is not inside it. */
static double clipped(double x, double lo, double hi)
{
  const double m = isnan(x) ? x : (x > lo ? x : lo);

  return isnan(m) ? m : (m < hi ? m : hi);
}

/** numpy's remainder (npy_divmod): fmod, then the divisor's sign. */
static double remainder_of(double a, double b)
{
  double mod = fmod(a, b);

  if (b == 0.0)
  {
    return mod;
  }
  if (mod != 0.0)
  {
    if ((b < 0.0) != (mod < 0.0))
    {
      mod += b;
    }
  }
  else
  {
    mod = copysign(0.0, b);
  }
  return mod;
}

void wbc_law_latent(const wbc_law_t *k, double *st, const double *tab, int ntab, double now, double dt, double *out,
                    double *hit)
{
  const int T = k->table, P = k->p, n = k->n;
  double   *to = st + k->to, *from = st + k->from, *p = st + k->now;
  double    said[WBC_LAW_P], vals[WBC_LAW_P], came, span, u, turns, lerp;

  for (int i = 0; i < ntab; i++)
  {
    const double at = tab[i * T + k->at];

    hit[i] = (double)((at <= now + k->edge_s) && (at > (now - dt) + k->edge_s));
  }
  came = npmin(npsum(hit, ntab), 1.0);
  wbc_matmul(&k->blas, VIEW(hit, 0, 1), VIEW(tab + k->said, T, 1), said, 0, 1, 1, ntab, P);
  wbc_matmul(&k->blas, VIEW(hit, 0, 1), VIEW(tab + k->vals, T, 1), vals, 0, 1, 1, ntab, P);
  for (int j = 0; j < P; j++)
  {
    to[j] += npmin(said[j], 1.0) * (vals[j] - to[j]);
  }
  for (int j = 0; j < P; j++)
  {
    from[j] += came * (p[j] - from[j]);
  }
  st[k->t0] += came * (now - st[k->t0]);
  lerp = wbc_dot(&k->blas, hit, 1, tab + k->lerp, T, ntab);
  st[k->blend] += came * (lerp - st[k->blend]);
  span = npmax(st[k->blend], dt);
  u = clipped(((now - st[k->t0]) + dt) / span, 0.0, 1.0);
  for (int j = 0; j < P; j++)
  {
    p[j] = from[j] + (to[j] - from[j]) * u;
  }
  turns = k->tau * p[k->hz];
  st[k->theta] = remainder_of(st[k->theta] + turns * dt, k->tau);
  for (int m = 0; m < n; m++)
  {
    const double psi = st[k->theta] + p[k->phase + m], sway = p[k->gain] * p[k->amp + m];

    out[k->ref + m] = p[k->z + m] + sway * sin(psi);
    out[k->rate + m] = ((to[k->z + m] - from[k->z + m]) / span) * (double)(u < 1.0) + (sway * turns) * cos(psi);
    out[k->acc + m] = (((-sway) * turns) * turns) * sin(psi);
    out[k->ks + m] = p[k->k + m];
    out[k->ds + m] = p[k->d + m];
    out[k->psi + m] = psi;
    out[k->sway + m] = sway;
  }
  out[k->turns] = turns;
  out[k->held] = p[k->zmp];
  out[k->yawed] = p[k->yaw];
}

void wbc_law_couple(const wbc_law_t *k, const double *q, const double *qd, const double *ref_q, const double *ref_v,
                    double *eso_z, double *out, double *dq)
{
  const double s2 = k->sigma * k->sigma, z0 = k->eso_s * k->eso_s, s4 = s2 * s2, sz4 = z0 * z0, s8 = s4 * s4;
  double       phi[WBC_LAW_D], shift[WBC_LAW_N];

  for (int i = 0; i < k->driven; i++)
  {
    const double err = q[i] - ref_q[i], e2 = err * err;
    const double m = npmax(eso_z[i] * copysign(1.0, err) - k->eso_0, 0.0), z2 = m * m;
    const double e4 = e2 * e2, z4 = z2 * z2, e8 = e4 * e4;

    phi[i] = ((z4 / (z4 + sz4)) * (e8 / (e8 + s8))) * (err + ((k->tau_v * (qd[i] - ref_v[i])) * e2) / (e2 + s2));
  }
  wbc_matmul(&k->blas, VIEW(k->pi, k->driven, 1), VIEW(phi, 1, 0), shift, 1, 0, k->n, k->driven, 1);
  for (int m = 0; m < k->n; m++)
  {
    out[k->ref + m] += shift[m];
  }
  for (int i = 0; i < k->driven; i++)
  {
    eso_z[i] /= 1.0 + (k->bleed * k->gamma[i]) * (phi[i] * phi[i]);
    dq[i] = k->gamma[i] * phi[i];
  }
}
