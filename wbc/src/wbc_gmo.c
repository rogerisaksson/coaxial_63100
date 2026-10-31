/** wbc_gmo.c - her generalized momentum observer (wbc_gmo.h). */
#include "wbc_gmo.h"
#include "wbc_solve.h"

#include <float.h>
#include <math.h>
#include <string.h>

#define NF WBC_GMO_F
#define NH (WBC_N - 6)

static const double gravity[3] = {0.0, -9.81, 0.0};

static void turned(const double r[9], const double v[3], double out[3])
{
  for (int i = 0; i < 3; i++)
  {
    out[i] = r[3 * i] * v[0] + r[3 * i + 1] * v[1] + r[3 * i + 2] * v[2];
  }
}

static double dot(const double a[3], const double b[3])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static double clip01(double x)
{
  return (x < 0.0) ? 0.0 : ((x > 1.0) ? 1.0 : x);
}

void wbc_gmo_felt(wbc_gmo_t *o, const double quat[4], const double accel[3], const double *tau, double dt)
{
  const double w = quat[0], x = quat[1], y = quat[2], z = quat[3];
  const double r[9] = {1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y),
                       2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x),
                       2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)};
  double       a[3];

  turned(r, accel, a);
  for (int i = 0; i < 3; i++)
  {
    o->acc[i] += dt * (a[i] + gravity[i]);
  }
  for (int i = 0; i < WBC_DRIVEN; i++)
  {
    o->tau[i] += dt * tau[o->driven_joint[i]];
  }
  o->span += dt;
}

/** u' over the pass: the hinges' and the turn's by difference, the pelvis's linear in its own frame
    of the world's mean acceleration, less the frame's turning (w x v). */
static void accelerations(const wbc_gmo_t *o, const wbc_frame_t *base, const double u[WBC_N], double span, double dt,
                          double out[WBC_N])
{
  double mean[3], own[3];

  for (int i = 0; i < WBC_N; i++)
  {
    out[i] = (u[i] - o->u[i]) / dt;
  }
  for (int i = 0; i < 3; i++)
  {
    mean[i] = o->acc[i] / span;
  }
  for (int i = 0; i < 3; i++)
  {
    own[i] = base->r[i] * mean[0] + base->r[3 + i] * mean[1] + base->r[6 + i] * mean[2];
  }
  out[3] = own[0] - (u[1] * u[5] - u[2] * u[4]);
  out[4] = own[1] - (u[2] * u[3] - u[0] * u[5]);
  out[5] = own[2] - (u[0] * u[4] - u[1] * u[3]);
}

/** Point p's e-th push in the world: its segment's turn of it, or as laid where `world` says so. */
static void along(const wbc_gmo_t *o, int p, int e, double n[3])
{
  turned(o->body.t[o->link[p]].r, o->edge[p][e], n);
  for (int i = 0; i < 3; i++)
  {
    n[i] += o->world[p] * (o->edge[p][e][i] - n[i]);
  }
}

/** Column c of J' for a push along the world direction n at a point with Jacobian j. */
static void pushed(wbc_gmo_t *o, int c, double j[6][WBC_N], const double n[3], double sign)
{
  for (int i = 0; i < WBC_N; i++)
  {
    o->a[i][c] += sign * (j[3][i] * n[0] + j[4][i] * n[1] + j[5][i] * n[2]);
  }
}

/** The closest points of the segments p1-q1 and p2-q2 (a point a segment of none), into c1, c2. */
static void closest(const double p1[3], const double q1[3], const double p2[3], const double q2[3], double c1[3],
                    double c2[3])
{
  double d1[3], d2[3], r[3], s = 0.0, t = 0.0;

  for (int i = 0; i < 3; i++)
  {
    d1[i] = q1[i] - p1[i];
    d2[i] = q2[i] - p2[i];
    r[i] = p1[i] - p2[i];
  }
  const double a = dot(d1, d1), e = dot(d2, d2), f = dot(d2, r), c = dot(d1, r), b = dot(d1, d2);
  const double denom = a * e - b * b;

  if (a > 0.0 && e > 0.0)
  {
    s = (denom > 0.0) ? clip01((b * f - c * e) / denom) : 0.0;
    t = (b * s + f) / e;
    s = (t < 0.0) ? clip01(-c / a) : ((t > 1.0) ? clip01((b - c) / a) : s);
    t = clip01(t);
  }
  else if (a > 0.0)
  {
    s = clip01(-c / a);
  }
  else if (e > 0.0)
  {
    t = clip01(f / e);
  }
  for (int i = 0; i < 3; i++)
  {
    c1[i] = p1[i] + s * d1[i];
    c2[i] = p2[i] + t * d2[i];
  }
}

/** A point of the world in `link`'s frame. */
static void own(const wbc_body_t *b, int link, const double x[3], double out[3])
{
  const double *r = b->t[link].r, *p = b->t[link].p;
  const double  d[3] = {x[0] - p[0], x[1] - p[1], x[2] - p[2]};

  for (int i = 0; i < 3; i++)
  {
    out[i] = r[i] * d[0] + r[3 + i] * d[1] + r[6 + i] * d[2];
  }
}

/** Her pairs staged (`list`, `listed`) in their order: those whose capsules' spheres - about each axis's
    middle, half its length and its radius - come within her envelope, every one where `whole` says so;
    her capsules' ends `ends` kept as staged. A sphere holds its capsule: a pair left out is an envelope
    apart. */
static void stage(wbc_gmo_t *o, double ends[][2][3])
{
  double mid[WBC_GMO_CAPSULES][3], reach[WBC_GMO_CAPSULES];

  for (int c = 0; c < o->capsules; c++)
  {
    double d[3];

    for (int i = 0; i < 3; i++)
    {
      mid[c][i] = 0.5 * (ends[c][0][i] + ends[c][1][i]);
      d[i] = ends[c][1][i] - ends[c][0][i];
    }
    reach[c] = 0.5 * sqrt(dot(d, d)) + o->capsule_r[c];
  }
  o->listed = 0;
  for (int k = 0; k < o->pairs; k++)
  {
    const int    c1 = o->pair[k][0], c2 = o->pair[k][1];
    const double d[3] = {mid[c2][0] - mid[c1][0], mid[c2][1] - mid[c1][1], mid[c2][2] - mid[c1][2]};

    o->list[o->listed] = k;
    o->listed += o->whole | (sqrt(dot(d, d)) - reach[c1] - reach[c2] < o->horizon);
  }
  memcpy(o->staged, ends, sizeof o->staged);
  o->stagings++;
}

/** Her capsules touching, the deepest WBC_GMO_TOUCH: each one's column, pushed apart along the line
    between their axes' closest points at the middle of their overlap. Searched among the pairs staged
    (`stage`), restaged once any capsule's end has moved a quarter of her envelope: a pair left out stays
    half of it apart, so the touches are every pair's. */
static void touches(wbc_gmo_t *o)
{
  static double j[6][WBC_N], ends[WBC_GMO_CAPSULES][2][3];
  double        depth[WBC_GMO_TOUCH], moved = 0.0;

  for (int c = 0; c < o->capsules; c++)
  {
    wbc_body_point(&o->body, o->capsule_link[c], o->capsule_a[c], ends[c][0]);
    wbc_body_point(&o->body, o->capsule_link[c], o->capsule_b[c], ends[c][1]);
    for (int e = 0; e < 2; e++)
    {
      const double d[3] = {ends[c][e][0] - o->staged[c][e][0], ends[c][e][1] - o->staged[c][e][1],
                           ends[c][e][2] - o->staged[c][e][2]};

      moved = fmax(moved, dot(d, d));
    }
  }
  if (o->whole || o->stagings == 0 || moved >= 0.0625 * o->horizon * o->horizon)
  {
    stage(o, ends);
  }
  for (int t = 0; t < WBC_GMO_TOUCH; t++)
  {
    o->touched[t] = -1;
    depth[t] = 0.0;
  }
  for (int x = 0; x < o->listed; x++)
  {
    const int k = o->list[x], c1 = o->pair[k][0], c2 = o->pair[k][1];
    double    s1[3], s2[3], d[3];

    closest(ends[c1][0], ends[c1][1], ends[c2][0], ends[c2][1], s1, s2);
    for (int i = 0; i < 3; i++)
    {
      d[i] = s2[i] - s1[i];
    }
    const double gap = sqrt(dot(d, d)) - o->capsule_r[c1] - o->capsule_r[c2];
    int          worst = 0;

    for (int t = 1; t < WBC_GMO_TOUCH; t++)
    {
      worst = (depth[t] > depth[worst]) ? t : worst;
    }
    if (gap < depth[worst] && dot(d, d) > 0.0)
    {
      depth[worst] = gap;
      o->touched[worst] = k;
    }
  }
  for (int t = 0; t < WBC_GMO_TOUCH; t++)
  {
    const int c = o->pushes + NH + t, k = o->touched[t];

    if (k < 0)
    {
      continue;
    }
    const int c1 = o->pair[k][0], c2 = o->pair[k][1];
    double    s1[3], s2[3], n[3], x[3], local[3];

    closest(ends[c1][0], ends[c1][1], ends[c2][0], ends[c2][1], s1, s2);
    for (int i = 0; i < 3; i++)
    {
      n[i] = s2[i] - s1[i];
    }
    const double len = sqrt(dot(n, n));

    for (int i = 0; i < 3; i++)
    {
      n[i] /= len;
      x[i] = s1[i] + n[i] * (o->capsule_r[c1] + 0.5 * depth[t]);
    }
    own(&o->body, o->capsule_link[c2], x, local);
    wbc_body_jacobian(&o->body, o->capsule_link[c2], local, j);
    pushed(o, c, j, n, 1.0);
    own(&o->body, o->capsule_link[c1], x, local);
    wbc_body_jacobian(&o->body, o->capsule_link[c1], local, j);
    pushed(o, c, j, n, -1.0);
  }
}

/** Each point's push as gravity would have it, N (`prior`): her weight over the points as far as
    each presses up (`upright` of its pushes' mean's cosine with the vertical), lies low (less its
    height over her lowest point in `reach`s) and stands still (less its speed in `still`s), a
    softmax; even over a point's edges into `expected` - her soles' points there their last pass's
    pushes. */
static void expect(wbc_gmo_t *o)
{
  double logit[WBC_GMO_POINTS], lowest = o->at[0][1], top = -INFINITY, sum = 0.0;

  for (int p = 1; p < WBC_GMO_POINTS; p++)
  {
    lowest = fmin(lowest, o->at[p][1]);
  }
  for (int p = 0; p < WBC_GMO_POINTS; p++)
  {
    double mean[3] = {0.0, 0.0, 0.0};

    for (int e = 0; e < o->edges[p]; e++)
    {
      double n[3];

      along(o, p, e, n);
      for (int i = 0; i < 3; i++)
      {
        mean[i] += n[i];
      }
    }
    logit[p] = o->upright * fmax(mean[1] / sqrt(dot(mean, mean)), 0.0) - (o->at[p][1] - lowest) / o->reach
               - o->speed[p] / o->still;
    top = fmax(top, logit[p]);
  }
  for (int p = 0; p < WBC_GMO_POINTS; p++)
  {
    o->prior[p] = exp(logit[p] - top);
    sum += o->prior[p];
  }
  memset(o->expected, 0, sizeof o->expected);
  for (int p = 0; p < WBC_GMO_POINTS; p++)
  {
    o->prior[p] *= o->weight / sum;
    for (int e = 0; e < o->edges[p]; e++)
    {
      const int a = o->col[p] + e;

      o->expected[a] = o->soled[p] ? o->f[a] : o->prior[p] / o->edges[p];
    }
  }
}

/** Her working set this pass (`live`, `lives`), the unknowns that can push in their order: every
    point's, a hinge's on its stop, a touch found; all of them where `whole` says so. A column of zeros
    never joins the split nor moves a sum: the rest left out, the same bits. */
static void scoped(wbc_gmo_t *o)
{
  int n = 0;

  for (int c = 0; c < NF; c++)
  {
    const int h = c - o->pushes, t = h - NH;
    const int stop = (h >= 0) && (h < NH) && (o->a[6 + h][c] != 0.0);
    const int touch = (t >= 0) && (t < WBC_GMO_TOUCH) && (o->touched[t] >= 0);

    o->live[n] = c;
    n += o->whole | (c < o->pushes) | stop | touch;
  }
  o->lives = n;
}

/** Each unknown's column of J': a point's edges along their directions; a hinge on its stop pushed
    off it; her capsules touching. */
static void columns(wbc_gmo_t *o, const double q[NH])
{
  static double j[6][WBC_N];

  memset(o->a, 0, sizeof o->a);
  for (int p = 0; p < WBC_GMO_POINTS; p++)
  {
    double v[3] = {0.0, 0.0, 0.0};

    wbc_body_jacobian(&o->body, o->link[p], o->local[p], j);
    wbc_body_point(&o->body, o->link[p], o->local[p], o->at[p]);
    for (int e = 0; e < o->edges[p]; e++)
    {
      double n[3];

      along(o, p, e, n);
      pushed(o, o->col[p] + e, j, n, 1.0);
    }
    for (int i = 0; i < WBC_N; i++)
    {
      v[0] += j[3][i] * o->u[i];
      v[1] += j[4][i] * o->u[i];
      v[2] += j[5][i] * o->u[i];
    }
    double w[3] = {v[0] - o->slide[0], v[1] - o->slide[1], v[2] - o->slide[2]};

    o->speed[p] = fmin(sqrt(dot(v, v)), sqrt(dot(w, w)));
  }
  expect(o);
  for (int h = 0; h < NH; h++)
  {
    o->a[6 + h][o->pushes + h] = o->stopped[h] * ((q[h] <= o->stop_lo[h]) - (q[h] >= o->stop_hi[h]));
  }
  touches(o);
  scoped(o);
}

/** The least squares of A f = r over the rows `weight` keeps, each shifted by `shift`, every unknown of
    her working set (`scoped`) pushing, in its order (Lawson-Hanson on the normal equations over it alone,
    the tie-break toward `expected` on their diagonal: what the rows cannot tell, her soles' last pass and
    gravity elsewhere do): a push joins the set where it lowers the residual past rounding, one not felt
    (`gated`) or moving past `still` only where none else would, the set solved, a push it would pull
    stepped back to none and out. */
/** Push `b` joined to the `n` pushing `at` (in the order they joined): `o->k`'s factor L of K = H over them
    grown a row by forward substitution, the rows before it as they were, its new diagonal's square no less
    than `least` - eps top lies on K's diagonal, so no Schur complement of it is smaller. 53 factorizations
    of 26 a step from scratch were half her observer's time. */
static int joined(wbc_gmo_t *o, int at[NF], int n, int b, double least)
{
  double d = o->h[b][b];

  for (int x = 0; x < n; x++)
  {
    double s = o->h[b][at[x]];

    for (int k = 0; k < x; k++)
    {
      s -= o->k[n][k] * o->k[x][k];
    }
    o->k[n][x] = s / o->k[x][x];
    d -= o->k[n][x] * o->k[n][x];
  }
  at[n] = b;
  o->k[n][n] = sqrt(fmax(d, least));
  return n + 1;
}

/** The push at `p` among the `n` pushing `at` left: L's row taken out, its columns from `p` turned back
    lower-triangular by Givens rotations - L L' K less its row and column. */
static int left(wbc_gmo_t *o, int at[NF], int n, int p)
{
  for (int r = p; r < n - 1; r++)
  {
    at[r] = at[r + 1];
    memcpy(o->k[r], o->k[r + 1], (size_t)(r + 2) * sizeof(double));
  }
  for (int j = p; j < n - 1; j++)
  {
    const double a = o->k[j][j], b = o->k[j][j + 1], r = hypot(a, b), c = a / r, s = b / r;

    for (int i = j; i < n - 1; i++)
    {
      const double t1 = o->k[i][j], t2 = o->k[i][j + 1];

      o->k[i][j] = c * t1 + s * t2;
      o->k[i][j + 1] = c * t2 - s * t1;
    }
  }
  return n - 1;
}

static void pushes(wbc_gmo_t *o, const double weight[WBC_N], const double shift[WBC_N])
{
  static double g[NF], w[NF], z[NF], sub[NF], f[NF], ag[WBC_N][NF];
  static int    set[NF], at[NF], gate[NF], gated[NF], on[NF];
  const int     m = o->lives;
  int           rows[WBC_N], weighed = 0;
  double        top = 0.0, most = 0.0;

  /* A row weighed 0, or a push f 0, adds exactly +-0 to a sum that starts at +0: left out, the same bits. */
  for (int i = 0; i < WBC_N; i++)
  {
    rows[weighed] = i;
    weighed += weight[i] != 0.0;
  }
  memset(gated, 0, sizeof gated);
  for (int p = 0; p < WBC_GMO_POINTS; p++)
  {
    for (int e = 0; e < o->edges[p]; e++)
    {
      gated[o->col[p] + e] = o->gated[p] || (o->speed[p] > o->still);
    }
  }
  /* her working set gathered, x an unknown's place in it, f[x] its push: its weighed rows along memory */
  for (int x = 0; x < m; x++)
  {
    gate[x] = gated[o->live[x]];
    for (int k = 0; k < weighed; k++)
    {
      ag[k][x] = o->a[rows[k]][o->live[x]];
    }
  }
  for (int x = 0; x < m; x++)
  {
    g[x] = 0.0;
    for (int k = 0; k < weighed; k++)
    {
      const int i = rows[k];

      g[x] += weight[i] * ag[k][x] * (o->r[i] - shift[i]);
    }
    /* each h[x][y] summed over the weighed rows in their order, the row's y innermost along its memory; a
       weighed row's weight is 1, so (1 a) b is b a bit for bit: h symmetric, its lower half mirrored */
    for (int y = x; y < m; y++)
    {
      o->h[x][y] = 0.0;
    }
    for (int k = 0; k < weighed; k++)
    {
      const double wa = weight[rows[k]] * ag[k][x];

      for (int y = x; y < m; y++)
      {
        o->h[x][y] += wa * ag[k][y];
      }
    }
    for (int y = 0; y < x; y++)
    {
      o->h[x][y] = o->h[y][x];
    }
    top = fmax(top, o->h[x][x]);
    f[x] = 0.0;
    set[x] = 0;
  }
  for (int x = 0; x < m; x++)
  {
    o->h[x][x] += o->eps * top;
    g[x] += o->eps * top * o->expected[o->live[x]];
    most = fmax(most, fabs(g[x]));
  }
  int n = 0;

  for (int round = 0; round < 3 * NF; round++)
  {
    int free_ = -1, held = -1, pushing = 0;

    for (int y = 0; y < m; y++)
    {
      on[pushing] = y;
      pushing += f[y] != 0.0;
    }
    for (int x = 0; x < m; x++)
    {
      const int sums = set[x] ? 0 : pushing; /* a push in the set is never picked: its w unread */

      w[x] = g[x];
      for (int k = 0; k < sums; k++)
      {
        w[x] -= o->h[x][on[k]] * f[on[k]];
      }
      const int fits = !set[x] && w[x] > DBL_EPSILON * NF * most;

      free_ = (fits && !gate[x] && (free_ < 0 || w[x] > w[free_])) ? x : free_;
      held = (fits && gate[x] && (held < 0 || w[x] > w[held])) ? x : held;
    }
    const int best = (free_ >= 0) ? free_ : held;

    if (best < 0)
    {
      break;
    }
    set[best] = 1;
    n = joined(o, at, n, best, o->eps * top);
    for (int inner = 0; inner < NF; inner++)
    {
      int    out = -1;
      double step = 1.0;

      for (int x = 0; x < n; x++)
      {
        sub[x] = g[at[x]];
      }
      wbc_chol_solve(n, NF, &o->k[0][0], sub, z);
      for (int x = 0; x < n; x++)
      {
        const double was = f[at[x]], ratio = (z[x] <= 0.0) ? was / (was - z[x]) : 1.0;

        out = (ratio < step) ? x : out;
        step = (ratio < step) ? ratio : step;
      }
      for (int x = 0; x < n; x++)
      {
        f[at[x]] += step * (z[x] - f[at[x]]);
      }
      if (out < 0)
      {
        break;
      }
      for (int x = n - 1; x >= 0; x--)
      {
        const int drop = (x == out) || (f[at[x]] <= 0.0);

        f[at[x]] = drop ? 0.0 : f[at[x]];
        set[at[x]] = !drop;
        n = drop ? left(o, at, n, x) : n;
      }
    }
  }
  memset(o->f, 0, sizeof o->f);
  for (int x = 0; x < m; x++)
  {
    o->f[o->live[x]] = f[x];
  }
}

/** The pushes with each hinge's row in its friction's band - its loss less what its speed spends
    (`slip`) - read as friction: a row within its band left out, one past it held to the band's
    edge; WBC_GMO_ROUNDS rounds from the last pass's, till none crosses. */
static void split(wbc_gmo_t *o)
{
  double weight[WBC_N], shift[WBC_N], band[NH];

  for (int i = 0; i < 6; i++)
  {
    weight[i] = 1.0;
    shift[i] = 0.0;
  }
  for (int h = 0; h < NH; h++)
  {
    band[h] = o->loss[6 + h] * (1.0 - fabs(tanh(o->u[6 + h] / o->slip)));
  }
  for (int round = 0; round < WBC_GMO_ROUNDS; round++)
  {
    int crossed = 0;

    for (int h = 0; h < NH; h++)
    {
      weight[6 + h] = (double)(o->past[h] != 0);
      shift[6 + h] = o->past[h] * band[h];
    }
    pushes(o, weight, shift);
    for (int h = 0; h < NH; h++)
    {
      double e = o->r[6 + h];

      for (int x = 0; x < o->lives; x++)
      {
        e -= o->a[6 + h][o->live[x]] * o->f[o->live[x]];
      }
      const int past = (e > band[h]) - (e < -band[h]);

      crossed += past != o->past[h];
      o->past[h] = past;
    }
    o->rounds = round + 1;
    if (!crossed)
    {
      break;
    }
  }
}

/** Each manifold's normal force: a point's edges summed in the world, along each of its manifolds'
    normals, pushing. */
static void shared(wbc_gmo_t *o)
{
  memset(o->force, 0, sizeof o->force);
  for (int s = 0; s < WBC_GMO_SHARES; s++)
  {
    const int p = o->share_point[s], c = o->col[p];
    double    world[3] = {0.0, 0.0, 0.0}, axis[3];

    for (int e = 0; e < o->edges[p]; e++)
    {
      double n[3];

      along(o, p, e, n);
      for (int i = 0; i < 3; i++)
      {
        world[i] += o->f[c + e] * n[i];
      }
    }
    turned(o->body.t[o->share_link[s]].r, o->share_axis[s], axis);
    o->force[o->share_manifold[s]] += fmax(dot(world, axis), 0.0);
  }
}

void wbc_gmo_step(wbc_gmo_t *o, const wbc_frame_t *base, const double q[NH], const double u[WBC_N], double dt)
{
  const double span = o->span > dt ? o->span : dt;
  const double k = o->primed * (1.0 - exp(-o->alpha * dt));
  double       ud[WBC_N], tau[WBC_N];

  wbc_body_pose(&o->body, base, q);
  wbc_body_mass(&o->body);
  wbc_body_bias(&o->body, u);
  accelerations(o, base, u, span, dt, ud);
  for (int i = 0; i < WBC_N; i++)
  {
    tau[i] = -o->loss[i] * tanh(u[i] / o->slip);
  }
  for (int i = 0; i < WBC_DRIVEN; i++)
  {
    tau[o->driven_at[i]] += o->tau[i] / span;
  }
  for (int i = 0; i < WBC_N; i++)
  {
    double e = o->body.h[i] - tau[i];

    for (int j = 0; j < WBC_N; j++)
    {
      e += o->body.m[i][j] * ud[j];
    }
    o->r[i] += k * (e - o->r[i]);
  }
  memset(o->tau, 0, sizeof o->tau);
  memset(o->acc, 0, sizeof o->acc);
  memcpy(o->u, u, sizeof o->u);
  o->span = 0.0;
  o->primed = 1.0;
  columns(o, q);
  split(o);
  shared(o);
}
