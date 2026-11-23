/** wbc_hull.c - the convex hull of the robot's support's floor points (wbc_hull.h): machine.law.barrier.hull, its
    sort as Python's tuples sort and its turns Python's floats' arithmetic operation for operation. Portable C11, the
    host's alone. */
#include "wbc_hull.h"

/* Whether floor point `a` sorts before `b` as Python's tuples do: its x less, or its x equal and its y less. */
static int before(const double *a, const double *b)
{
  return (a[0] < b[0]) || ((a[0] == b[0]) && (a[1] < b[1]));
}

/* The turn from `o` to `a` to `b`, barrier.hull's own product. */
static double turned(const double *o, const double *a, const double *b)
{
  return ((a[0] - o[0]) * (b[1] - o[1])) - ((a[1] - o[1]) * (b[0] - o[0]));
}

/* `p` (2) onto the chain `chain` of `n`, each last point left off while it makes no turn of more than nothing. */
static int chained(double chain[WBC_HULL_MOST][2], int n, const double *p)
{
  int at = n;

  while ((at >= 2) && (turned(chain[at - 2], chain[at - 1], p) <= 0.0))
  {
    at--;
  }
  chain[at][0] = p[0];
  chain[at][1] = p[1];
  return at + 1;
}

int wbc_hull(const double *pts, int n, double *ring)
{
  double sorted[WBC_HULL_MOST][2], lower[WBC_HULL_MOST][2], upper[WBC_HULL_MOST][2];
  int    nl = 0, nu = 0, m = 0;

  if (n > WBC_HULL_MOST)
  {
    return -1;
  }
  /* an insertion a point at a time, each after those it does not sort before: stable, as Python's sort is */
  for (int i = 0; i < n; i++)
  {
    int j = i;

    while ((j > 0) && (before(&pts[2 * i], sorted[j - 1]) != 0))
    {
      sorted[j][0] = sorted[j - 1][0];
      sorted[j][1] = sorted[j - 1][1];
      j--;
    }
    sorted[j][0] = pts[2 * i];
    sorted[j][1] = pts[(2 * i) + 1];
  }
  for (int i = 0; i < n; i++)
  {
    nl = chained(lower, nl, sorted[i]);
  }
  for (int i = n - 1; i >= 0; i--)
  {
    nu = chained(upper, nu, sorted[i]);
  }
  for (int i = 0; i < (nl - 1); i++)
  {
    ring[2 * m]       = lower[i][0];
    ring[(2 * m) + 1] = lower[i][1];
    m++;
  }
  for (int i = 0; i < (nu - 1); i++)
  {
    ring[2 * m]       = upper[i][0];
    ring[(2 * m) + 1] = upper[i][1];
    m++;
  }
  return m;
}
