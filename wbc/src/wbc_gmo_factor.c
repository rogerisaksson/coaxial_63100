/** wbc_gmo_factor.c - the observer's split's factor kept as pushes join and leave. */
#include "wbc_gmo_factor.h"
#include "wbc_libm_arc.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/* eps top lies on K's diagonal, so no Schur complement of it is smaller than `least`. 53 factorizations of 26 a
 * step from scratch were half the robot's observer's time. */
int wbc_gmo_joined(wbc_gmo_t *o, int at[WBC_GMO_F], int n, int b, double least)
{
  double d = o->h[b][b];

  /* each of the row's entries less its terms in their order, as each before it is had: the rows independent */
  for (int x = 0; x < n; x++)
  {
    o->k[n][x] = o->h[b][at[x]];
  }
  for (int x = 0; x < n; x++)
  {
    const double kx = o->k[n][x] / o->k[x][x];

    o->k[n][x] = kx;
    d -= kx * kx;
    for (int y = x + 1; y < n; y++)
    {
      o->k[n][y] -= kx * o->k[y][x];
    }
  }
  at[n] = b;
  o->k[n][n] = sqrt(fmax(d, least));
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
