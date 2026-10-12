/** wbc_body.c - her chain on the static arrays: poses, Jacobians, mass matrix and bias.

    Twists (w, v) and wrenches (m, f) in body frames; a link's frame in its parent's is
    X exp([S] q), the body form of the product of exponentials; the mass matrix by composite
    rigid bodies about the pelvis's origin in the world's axes, the bias by the Newton-Euler
    passes with no acceleration and gravity as the pelvis's. A link is its mass, its centre and
    its inertia about it, a frame its rotation and its origin: no 6x6 is formed. Lynch and
    Park, Modern Robotics, chapter 8. */
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

/** A twist of a frame's parent, in the frame: w' = R^T w, v' = R^T (v + w x p). */
static void into(const wbc_frame_t *t, const double tw[6], double out[6])
{
  double wp[3];

  cross(tw, t->p, wp);
  for (int k = 0; k < 3; k++)
  {
    wp[k] += tw[3 + k];
  }
  unrotate(t->r, tw, out);
  unrotate(t->r, wp, out + 3);
}

/** A wrench of a frame, in its parent's: f' = R f, m' = R m + p x f'. */
static void out_of(const wbc_frame_t *t, const double f[6], double out[6])
{
  double pf[3];

  rotate(t->r, f, out);
  rotate(t->r, f + 3, out + 3);
  cross(t->p, out + 3, pf);
  for (int k = 0; k < 3; k++)
  {
    out[k] += pf[k];
  }
}

/** What link i carries under a twist of its frame: its momentum, about the frame's origin. */
static void carried(int i, const double tw[6], double out[6])
{
  const double *a = wbc_about[i], *c = wbc_com[i];
  double        wc[3], ang[3];

  cross(tw, c, wc);
  for (int k = 0; k < 3; k++)
  {
    out[3 + k] = wbc_mass[i] * (tw[3 + k] + wc[k]);
  }
  cross(c, out + 3, ang);
  out[0] = a[0] * tw[0] + a[1] * tw[1] + a[2] * tw[2] + ang[0];
  out[1] = a[1] * tw[0] + a[3] * tw[1] + a[4] * tw[2] + ang[1];
  out[2] = a[2] * tw[0] + a[4] * tw[1] + a[5] * tw[2] + ang[2];
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
  for (int i = 1; i < WBC_LINKS; i++)
  {
    wbc_frame_t x, e;
    double      near[3];

    memcpy(x.r, wbc_x[i], 9U * sizeof(double));
    memcpy(x.p, wbc_x[i] + 9, 3U * sizeof(double));
    twist_exp(wbc_s[i], q[i - 1], &e);
    compose(&x, &e, &b->l[i]);
    compose(&b->t[wbc_parent[i]], &b->l[i], &b->t[i]);
    b->q[i - 1] = q[i - 1];
    rotate(b->t[i].r, wbc_s[i], b->axis[i]);
    cross(wbc_s[i], wbc_s[i] + 3, near);        /* w x v: the axis's point nearest the origin */
    wbc_body_point(b, i, near, b->anchor[i]);
  }
}

void wbc_body_mass(wbc_body_t *b)
{
  const double *r0 = b->t[0].r, *p0 = b->t[0].p;

  /* each link about the pelvis's origin, the world's axes: R A R^T + m (|c|^2 - c c^T) */
  for (int i = 0; i < WBC_LINKS; i++)
  {
    const double *r = b->t[i].r, *a = wbc_about[i], m = wbc_mass[i];
    double       *g = b->gc[i], c[3], ra[9], cc;

    rotate(r, wbc_com[i], c);
    for (int k = 0; k < 3; k++)
    {
      c[k] += b->t[i].p[k] - p0[k];
      ra[3 * k] = r[3 * k] * a[0] + r[3 * k + 1] * a[1] + r[3 * k + 2] * a[2];
      ra[3 * k + 1] = r[3 * k] * a[1] + r[3 * k + 1] * a[3] + r[3 * k + 2] * a[4];
      ra[3 * k + 2] = r[3 * k] * a[2] + r[3 * k + 1] * a[4] + r[3 * k + 2] * a[5];
      g[1 + k] = m * c[k];
    }
    g[0] = m;
    cc = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    for (int k = 0, at = 4; k < 3; k++)
    {
      for (int n = k; n < 3; n++, at++)
      {
        g[at] = ra[3 * k] * r[3 * n] + ra[3 * k + 1] * r[3 * n + 1] + ra[3 * k + 2] * r[3 * n + 2]
                + m * (((k == n) ? cc : 0.0) - c[k] * c[n]);
      }
    }
  }
  for (int i = WBC_LINKS - 1; i > 0; i--)
  {
    for (int k = 0; k < 10; k++)
    {
      b->gc[wbc_parent[i]][k] += b->gc[i][k];
    }
  }
  memset(b->m, 0, sizeof(b->m));
  for (int i = 1; i < WBC_LINKS; i++)
  {
    const double *g = b->gc[i];
    const int     at = 5 + i;
    double       *s = b->s[i], d[3], hv[3], hw[3], f[6], up[6];

    for (int k = 0; k < 3; k++)
    {
      d[k] = b->anchor[i][k] - p0[k];
      s[k] = b->axis[i][k];
    }
    cross(d, s, s + 3);
    /* all it carries turned on its hinge: the wrench that takes */
    cross(g + 1, s + 3, hv);
    cross(g + 1, s, hw);
    f[0] = g[4] * s[0] + g[5] * s[1] + g[6] * s[2] + hv[0];
    f[1] = g[5] * s[0] + g[7] * s[1] + g[8] * s[2] + hv[1];
    f[2] = g[6] * s[0] + g[8] * s[1] + g[9] * s[2] + hv[2];
    for (int k = 0; k < 3; k++)
    {
      f[3 + k] = g[0] * s[3 + k] - hw[k];
    }
    b->m[at][at] = dot6(s, f) + wbc_armature[i];
    for (int j = wbc_parent[i]; j > 0; j = wbc_parent[j])
    {
      b->m[5 + j][at] = dot6(b->s[j], f);
      b->m[at][5 + j] = b->m[5 + j][at];
    }
    /* on the pelvis, in its own axes */
    unrotate(r0, f, up);
    unrotate(r0, f + 3, up + 3);
    for (int k = 0; k < 6; k++)
    {
      b->m[k][at] = up[k];
      b->m[at][k] = up[k];
    }
  }
  {
    /* the pelvis's block: all she is, in its axes - R0^T I R0, [h] and her mass */
    const double *g = b->gc[0];
    const double  in[9] = {g[4], g[5], g[6], g[5], g[7], g[8], g[6], g[8], g[9]};
    double        ir[9], h[3];

    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        ir[3 * i + j] = in[3 * i] * r0[j] + in[3 * i + 1] * r0[3 + j] + in[3 * i + 2] * r0[6 + j];
      }
    }
    for (int i = 0; i < 3; i++)
    {
      for (int j = 0; j < 3; j++)
      {
        b->m[i][j] = r0[i] * ir[j] + r0[3 + i] * ir[3 + j] + r0[6 + i] * ir[6 + j];
      }
      b->m[3 + i][3 + i] = g[0];
    }
    unrotate(r0, g + 1, h);
    b->m[0][4] = b->m[4][0] = -h[2];
    b->m[0][5] = b->m[5][0] = h[1];
    b->m[1][3] = b->m[3][1] = h[2];
    b->m[1][5] = b->m[5][1] = -h[0];
    b->m[2][3] = b->m[3][2] = -h[1];
    b->m[2][4] = b->m[4][2] = h[0];
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

    into(&b->l[i], b->v[p], b->v[i]);
    for (int k = 0; k < 6; k++)
    {
      b->v[i][k] += wbc_s[i][k] * qd;
    }
    into(&b->l[i], b->vd[p], b->vd[i]);
    bracket(b->v[i], wbc_s[i], br);
    for (int k = 0; k < 6; k++)
    {
      b->vd[i][k] += br[k] * qd;
    }
  }
  for (int i = 0; i < WBC_LINKS; i++)
  {
    carried(i, b->v[i], gv);
    carried(i, b->vd[i], gvd);
    bracket_t(b->v[i], gv, c);
    for (int k = 0; k < 6; k++)
    {
      b->f[i][k] = gvd[k] - c[k];
    }
  }
  for (int i = WBC_LINKS - 1; i > 0; i--)
  {
    out_of(&b->l[i], b->f[i], up);
    for (int k = 0; k < 6; k++)
    {
      b->f[wbc_parent[i]][k] += up[k];
    }
    b->h[5 + i] = dot6(wbc_s[i], b->f[i]) + wbc_passive[i][0] * (b->q[i - 1] - wbc_passive[i][2])
                  + wbc_passive[i][1] * u[5 + i];
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
  const double *r0 = b->t[0].r;
  double        at[3], d[3], col[3];

  memset(j, 0, 6U * WBC_N * sizeof(double));
  wbc_body_point(b, link, r, at);
  /* The pelvis's columns: its w turns the point about it, its v carries it. */
  for (int k = 0; k < 3; k++)
  {
    d[k] = at[k] - b->t[0].p[k];
  }
  for (int c = 0; c < 3; c++)
  {
    const double e[3] = {r0[c], r0[3 + c], r0[6 + c]};

    cross(e, d, col);
    for (int row = 0; row < 3; row++)
    {
      j[row][c] = e[row];
      j[3 + row][c] = col[row];
      j[3 + row][3 + c] = e[row];
    }
  }
  /* A hinge under it turns it about its axis. */
  for (int k = link; k > 0; k = wbc_parent[k])
  {
    for (int n = 0; n < 3; n++)
    {
      d[n] = at[n] - b->anchor[k][n];
    }
    cross(b->axis[k], d, col);
    for (int row = 0; row < 3; row++)
    {
      j[row][5 + k] = b->axis[k][row];
      j[3 + row][5 + k] = col[row];
    }
  }
}

void wbc_body_com(wbc_body_t *b, double com[3], double j[3][WBC_N])
{
  double c[3], d[3], col[3];

  for (int i = 0; i < WBC_LINKS; i++)
  {
    wbc_body_point(b, i, wbc_com[i], c);
    b->sub_m[i] = wbc_mass[i];
    for (int k = 0; k < 3; k++)
    {
      b->sub_mc[i][k] = wbc_mass[i] * c[k];
    }
  }
  for (int i = WBC_LINKS - 1; i > 0; i--)
  {
    const int p = wbc_parent[i];

    b->sub_m[p] += b->sub_m[i];
    for (int k = 0; k < 3; k++)
    {
      b->sub_mc[p][k] += b->sub_mc[i][k];
    }
  }
  for (int k = 0; k < 3; k++)
  {
    com[k] = b->sub_mc[0][k] / b->sub_m[0];
    d[k] = com[k] - b->t[0].p[k];
  }
  memset(j, 0, 3U * WBC_N * sizeof(double));
  /* The pelvis's columns: its w turns the centre about it, -[d] R0; its v carries it, R0. */
  for (int c0 = 0; c0 < 3; c0++)
  {
    const double rc[3] = {b->t[0].r[c0], b->t[0].r[3 + c0], b->t[0].r[6 + c0]};

    cross(d, rc, col);
    for (int row = 0; row < 3; row++)
    {
      j[row][c0] = -col[row];
      j[row][3 + c0] = rc[row];
    }
  }
  /* A hinge turns all it carries about its axis: z x (its share of the centre, from the axis). */
  for (int i = 1; i < WBC_LINKS; i++)
  {
    const double share = b->sub_m[i] / b->sub_m[0];

    for (int k = 0; k < 3; k++)
    {
      d[k] = b->sub_mc[i][k] / b->sub_m[0] - share * b->anchor[i][k];
    }
    cross(b->axis[i], d, col);
    for (int row = 0; row < 3; row++)
    {
      j[row][5 + i] = col[row];
    }
  }
}

void wbc_body_drift(const wbc_body_t *b, int link, const double r[3], double out[6])
{
  const double *w = b->v[link], *v = b->v[link] + 3, *wd = b->vd[link], *vd = b->vd[link] + 3;
  double        a[3], wr[3], t1[3], t2[3];

  /* a = vd + wd x r + w x (v + w x r), into the world; the bias's -g taken back out. */
  cross(w, r, wr);
  for (int k = 0; k < 3; k++)
  {
    t1[k] = v[k] + wr[k];
  }
  cross(w, t1, t2);
  cross(wd, r, wr);
  for (int k = 0; k < 3; k++)
  {
    a[k] = vd[k] + wr[k] + t2[k];
  }
  rotate(b->t[link].r, wd, out);
  rotate(b->t[link].r, a, out + 3);
  for (int k = 0; k < 3; k++)
  {
    out[3 + k] += wbc_gravity[k];
  }
}

void wbc_body_momentum(const wbc_body_t *b, const double com[3], double k[3])
{
  double pm[6], ang[3], lin[3], d[3], c[3];

  k[0] = k[1] = k[2] = 0.0;
  for (int i = 0; i < WBC_LINKS; i++)
  {
    carried(i, b->v[i], pm);
    rotate(b->t[i].r, pm, ang);
    rotate(b->t[i].r, pm + 3, lin);
    for (int n = 0; n < 3; n++)
    {
      d[n] = b->t[i].p[n] - com[n];
    }
    cross(d, lin, c);
    for (int n = 0; n < 3; n++)
    {
      k[n] += ang[n] + c[n];
    }
  }
}
