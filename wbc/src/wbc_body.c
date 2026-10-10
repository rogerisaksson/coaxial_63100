/** wbc_body.c - her chain on the static arrays: poses, Jacobians, mass matrix and bias.

    Twists (w, v) and wrenches (m, f) in body frames; a link's frame in its parent's is
    X exp([S] q), the body form of the product of exponentials; the mass matrix by composite
    rigid bodies, the bias by the Newton-Euler passes with no acceleration and gravity as the
    pelvis's. Lynch and Park, Modern Robotics, chapter 8. */
#include "wbc.h"

#include <math.h>
#include <string.h>

static void cross(const double a[3], const double b[3], double out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

/** out = r v, r by rows. */
static void rotate(const double r[9], const double v[3], double out[3])
{
  for (int i = 0; i < 3; i++)
  {
    out[i] = r[3 * i] * v[0] + r[3 * i + 1] * v[1] + r[3 * i + 2] * v[2];
  }
}

/** out = r^T v. */
static void unrotate(const double r[9], const double v[3], double out[3])
{
  for (int i = 0; i < 3; i++)
  {
    out[i] = r[i] * v[0] + r[3 + i] * v[1] + r[6 + i] * v[2];
  }
}

/** out = a b. */
static void compose(const wbc_frame_t *a, const wbc_frame_t *b, wbc_frame_t *out)
{
  wbc_frame_t c;

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      c.r[3 * i + j] = a->r[3 * i] * b->r[j] + a->r[3 * i + 1] * b->r[3 + j]
                       + a->r[3 * i + 2] * b->r[6 + j];
    }
  }
  rotate(a->r, b->p, c.p);
  for (int i = 0; i < 3; i++)
  {
    c.p[i] += a->p[i];
  }
  *out = c;
}

static void invert(const wbc_frame_t *a, wbc_frame_t *out)
{
  wbc_frame_t c;
  double      p[3];

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      c.r[3 * i + j] = a->r[3 * j + i];
    }
  }
  rotate(c.r, a->p, p);
  for (int i = 0; i < 3; i++)
  {
    c.p[i] = -p[i];
  }
  *out = c;
}

/** exp([s] q) for a unit rotation screw s = (w, v). */
static void twist_exp(const double s[6], double q, wbc_frame_t *out)
{
  const double *w = s;
  const double  c = cos(q), sn = sin(q);
  const double  wx[9] = {0.0, -w[2], w[1], w[2], 0.0, -w[0], -w[1], w[0], 0.0};
  double        ww[9], g[9];

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      ww[3 * i + j] = w[i] * w[j] - ((i == j) ? 1.0 : 0.0);   /* [w]^2 */
    }
  }
  for (int k = 0; k < 9; k++)
  {
    const double one = ((k % 4) == 0) ? 1.0 : 0.0;

    out->r[k] = one + sn * wx[k] + (1.0 - c) * ww[k];
    g[k] = one * q + (1.0 - c) * wx[k] + (q - sn) * ww[k];
  }
  rotate(g, s + 3, out->p);
}

/** The adjoint of a frame, 6x6 by rows: (w, v) in the frame to (w, v) in its parent's. */
static void adjoint(const wbc_frame_t *t, double ad[36])
{
  const double *r = t->r, *p = t->p;
  const double  px[9] = {0.0, -p[2], p[1], p[2], 0.0, -p[0], -p[1], p[0], 0.0};

  memset(ad, 0, 36U * sizeof(double));
  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      ad[6 * i + j] = r[3 * i + j];
      ad[6 * (i + 3) + (j + 3)] = r[3 * i + j];
      ad[6 * (i + 3) + j] = px[3 * i] * r[j] + px[3 * i + 1] * r[3 + j] + px[3 * i + 2] * r[6 + j];
    }
  }
}

static void apply6(const double a[36], const double v[6], double out[6])
{
  for (int i = 0; i < 6; i++)
  {
    double s = 0.0;

    for (int j = 0; j < 6; j++)
    {
      s += a[6 * i + j] * v[j];
    }
    out[i] = s;
  }
}

static void apply6t(const double a[36], const double v[6], double out[6])
{
  for (int j = 0; j < 6; j++)
  {
    double s = 0.0;

    for (int i = 0; i < 6; i++)
    {
      s += a[6 * i + j] * v[i];
    }
    out[j] = s;
  }
}

/** out = a^T g a. */
static void congruence(const double a[36], const double g[36], double out[36])
{
  double ga[36];

  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      double s = 0.0;

      for (int k = 0; k < 6; k++)
      {
        s += g[6 * i + k] * a[6 * k + j];
      }
      ga[6 * i + j] = s;
    }
  }
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      double s = 0.0;

      for (int k = 0; k < 6; k++)
      {
        s += a[6 * k + i] * ga[6 * k + j];
      }
      out[6 * i + j] = s;
    }
  }
}

static double dot6(const double a[6], const double b[6])
{
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4] + a[5] * b[5];
}

/** [ad_v] s: the Lie bracket of two twists. */
static void bracket(const double v[6], const double s[6], double out[6])
{
  double a[3], b[3];

  cross(v, s, out);
  cross(v, s + 3, a);
  cross(v + 3, s, b);
  for (int k = 0; k < 3; k++)
  {
    out[3 + k] = a[k] + b[k];
  }
}

/** [ad_v]^T f, f a wrench (m, f). */
static void bracket_t(const double v[6], const double f[6], double out[6])
{
  double a[3], b[3];

  cross(v, f, a);
  cross(v + 3, f + 3, b);
  for (int k = 0; k < 3; k++)
  {
    out[k] = -a[k] - b[k];
  }
  cross(v, f + 3, a);
  for (int k = 0; k < 3; k++)
  {
    out[3 + k] = -a[k];
  }
}

void wbc_body_pose(wbc_body_t *b, const wbc_frame_t *base, const double q[WBC_N - 6])
{
  static const wbc_frame_t one = {{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}, {0.0, 0.0, 0.0}};

  b->l[0] = one;
  b->t[0] = *base;
  adjoint(&one, b->a[0]);
  for (int i = 1; i < WBC_LINKS; i++)
  {
    wbc_frame_t x, e, inv;

    memcpy(x.r, wbc_x[i], 9U * sizeof(double));
    memcpy(x.p, wbc_x[i] + 9, 3U * sizeof(double));
    twist_exp(wbc_s[i], q[i - 1], &e);
    compose(&x, &e, &b->l[i]);
    compose(&b->t[wbc_parent[i]], &b->l[i], &b->t[i]);
    invert(&b->l[i], &inv);
    adjoint(&inv, b->a[i]);
  }
}

void wbc_body_mass(wbc_body_t *b)
{
  double tmp[36];

  memcpy(b->gc, wbc_g, sizeof(b->gc));
  for (int i = WBC_LINKS - 1; i > 0; i--)
  {
    congruence(b->a[i], b->gc[i], tmp);
    for (int k = 0; k < 36; k++)
    {
      b->gc[wbc_parent[i]][k] += tmp[k];
    }
  }
  memset(b->m, 0, sizeof(b->m));
  for (int i = 0; i < 6; i++)
  {
    for (int j = 0; j < 6; j++)
    {
      b->m[i][j] = b->gc[0][6 * i + j];
    }
  }
  for (int i = 1; i < WBC_LINKS; i++)
  {
    const int d = 5 + i;
    double    f[6], up[6];

    apply6(b->gc[i], wbc_s[i], f);
    b->m[d][d] = dot6(wbc_s[i], f) + wbc_armature[i];
    for (int j = i; j != 0;)
    {
      apply6t(b->a[j], f, up);
      memcpy(f, up, sizeof(f));
      j = wbc_parent[j];
      if (j == 0)
      {
        for (int k = 0; k < 6; k++)
        {
          b->m[k][d] = f[k];
          b->m[d][k] = f[k];
        }
      }
      else
      {
        b->m[5 + j][d] = dot6(wbc_s[j], f);
        b->m[d][5 + j] = b->m[5 + j][d];
      }
    }
  }
}

void wbc_body_bias(wbc_body_t *b, const double u[WBC_N])
{
  double g[3], br[6], gv[6], gvd[6], c[6], up[6];

  memcpy(b->v[0], u, 6U * sizeof(double));
  unrotate(b->t[0].r, wbc_gravity, g);
  for (int k = 0; k < 3; k++)
  {
    b->vd[0][k] = 0.0;
    b->vd[0][3 + k] = -g[k];
  }
  for (int i = 1; i < WBC_LINKS; i++)
  {
    const int    p = wbc_parent[i];
    const double qd = u[5 + i];

    apply6(b->a[i], b->v[p], b->v[i]);
    for (int k = 0; k < 6; k++)
    {
      b->v[i][k] += wbc_s[i][k] * qd;
    }
    apply6(b->a[i], b->vd[p], b->vd[i]);
    bracket(b->v[i], wbc_s[i], br);
    for (int k = 0; k < 6; k++)
    {
      b->vd[i][k] += br[k] * qd;
    }
  }
  for (int i = 0; i < WBC_LINKS; i++)
  {
    apply6(wbc_g[i], b->v[i], gv);
    apply6(wbc_g[i], b->vd[i], gvd);
    bracket_t(b->v[i], gv, c);
    for (int k = 0; k < 6; k++)
    {
      b->f[i][k] = gvd[k] - c[k];
    }
  }
  for (int i = WBC_LINKS - 1; i > 0; i--)
  {
    apply6t(b->a[i], b->f[i], up);
    for (int k = 0; k < 6; k++)
    {
      b->f[wbc_parent[i]][k] += up[k];
    }
    b->h[5 + i] = dot6(wbc_s[i], b->f[i]);
  }
  memcpy(b->h, b->f[0], 6U * sizeof(double));
}

void wbc_body_point(const wbc_body_t *b, int link, const double r[3], double out[3])
{
  rotate(b->t[link].r, r, out);
  for (int k = 0; k < 3; k++)
  {
    out[k] += b->t[link].p[k];
  }
}

void wbc_body_jacobian(const wbc_body_t *b, int link, const double r[3], double j[6][WBC_N])
{
  wbc_frame_t   inv, rel;
  double        ad[36], col[6];
  const double *rot = b->t[link].r;

  memset(j, 0, 6U * WBC_N * sizeof(double));
  invert(&b->t[link], &inv);
  for (int k = link; k >= 0; k = wbc_parent[k])
  {
    compose(&inv, &b->t[k], &rel);
    adjoint(&rel, ad);
    if (k == 0)
    {
      for (int row = 0; row < 6; row++)
      {
        for (int c = 0; c < 6; c++)
        {
          j[row][c] = ad[6 * row + c];
        }
      }
    }
    else
    {
      apply6(ad, wbc_s[k], col);
      for (int row = 0; row < 6; row++)
      {
        j[row][5 + k] = col[row];
      }
    }
  }
  /* Into the world: w' = R w, v' = R (v + w x r). */
  for (int c = 0; c < WBC_N; c++)
  {
    double w[3] = {j[0][c], j[1][c], j[2][c]}, v[3] = {j[3][c], j[4][c], j[5][c]}, wr[3], o[3];

    cross(w, r, wr);
    for (int k = 0; k < 3; k++)
    {
      v[k] += wr[k];
    }
    rotate(rot, w, o);
    for (int k = 0; k < 3; k++)
    {
      j[k][c] = o[k];
    }
    rotate(rot, v, o);
    for (int k = 0; k < 3; k++)
    {
      j[3 + k][c] = o[k];
    }
  }
}
