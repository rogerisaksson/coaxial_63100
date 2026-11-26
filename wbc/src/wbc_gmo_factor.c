/** wbc_gmo_factor.c - the observer's split's factor kept as pushes join and leave. */
#include "wbc_gmo_factor.h"
#include "wbc_libm_arc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* eps top lies on K's diagonal, so no Schur complement of it is smaller than `least`. 53 factorizations of 26 a
 * step from scratch were half the robot's observer's time. */
int wbc_gmo_joined(wbc_gmo_t *restrict o, int at[WBC_GMO_F], int n, int b, double least)
{
  double       *kn = o->k[n];
  const double *hb = o->h[b];
  double        d = hb[b];

  /* each of the row's entries less its terms along its row of K in their order, held in a register */
  for (int y = 0; y < n; y++)
  {
    const double *ky = o->k[y];
    double        s = hb[at[y]];

    for (int x = 0; x < y; x++)
    {
      s -= kn[x] * ky[x];
    }
    const double ky_n = s / ky[y];

    kn[y] = ky_n;
    d -= ky_n * ky_n;
  }
  at[n] = b;
  kn[n] = sqrt(fmax(d, least));
  return n + 1;
}

/* L L' is K less the push's row and column. */
int wbc_gmo_left(wbc_gmo_t *o, int at[WBC_GMO_F], int n, int p)
{
  for (int r = p; r < n - 1; r++)
  {
    at[r] = at[r + 1];
    memcpy(o->k[r], o->k[r + 1], (size_t)(r + 2) * sizeof(double));
  }
  for (int j = p; j < n - 1; j++)
  {
    const double a = o->k[j][j], b = o->k[j][j + 1], r = wbc_hypot(a, b), c = a / r, s = b / r;

    for (int i = j; i < n - 1; i++)
    {
      const double t1 = o->k[i][j], t2 = o->k[i][j + 1];

      o->k[i][j] = c * t1 + s * t2;
      o->k[i][j + 1] = c * t2 - s * t1;
    }
  }
  return n - 1;
}
