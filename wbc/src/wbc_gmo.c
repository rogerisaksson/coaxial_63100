/** wbc_gmo.c - her generalized momentum observer (wbc_gmo.h). */
#include "wbc_gmo.h"
#include "wbc_solve.h"

#include <math.h>
#include <string.h>

static const double gravity[3] = {0.0, -9.81, 0.0};

static void turned(const double r[9], const double v[3], double out[3])
{
  for (int i = 0; i < 3; i++)
  {
    out[i] = r[3 * i] * v[0] + r[3 * i + 1] * v[1] + r[3 * i + 2] * v[2];
  }
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

/** Each unknown's column of J': a sole's twist rows, a push's normal along its point's translation. */
static void columns(wbc_gmo_t *o)
{
  static double j[6][WBC_N];

  memset(o->a, 0, sizeof o->a);
  for (int p = 0; p < WBC_GMO_POINTS; p++)
  {
    const int c = o->col[p];
    double    n[3];

    wbc_body_jacobian(&o->body, o->link[p], o->local[p], j);
    wbc_body_point(&o->body, o->link[p], o->local[p], o->at[p]);
    turned(o->body.t[o->link[p]].r, o->normal[p], n);
    for (int i = 0; i < WBC_N; i++)
    {
      for (int k = 0; k < 6 * o->sole[p]; k++)
      {
        o->a[i][c + k] = j[k][i];
      }
      o->a[i][c] += (1 - o->sole[p]) * (j[3][i] * n[0] + j[4][i] * n[1] + j[5][i] * n[2]);
    }
  }
}

/** The least squares of A f = r, each push pulling pinned to none for the next round. */
static void split(wbc_gmo_t *o)
{
  double g[WBC_GMO_F], free_[WBC_GMO_F], pinned[WBC_GMO_F] = {0}, top = 0.0;
  int    push[WBC_GMO_F] = {0};

  for (int p = 0; p < WBC_GMO_POINTS; p++)
  {
    push[o->col[p]] = 1 - o->sole[p];
  }
  for (int a = 0; a < WBC_GMO_F; a++)
  {
    g[a] = 0.0;
    for (int i = 0; i < WBC_N; i++)
    {
      g[a] += o->a[i][a] * o->r[i];
    }
    for (int b = 0; b < WBC_GMO_F; b++)
    {
      double s = 0.0;

      for (int i = 0; i < WBC_N; i++)
      {
        s += o->a[i][a] * o->a[i][b];
      }
      o->h[a][b] = s;
    }
    top = fmax(top, o->h[a][a]);
  }
  for (int a = 0; a < WBC_GMO_F; a++)
  {
    o->h[a][a] += o->eps * top;
  }
  for (int round = 0; round < o->rounds; round++)
  {
    double rhs[WBC_GMO_F];

    for (int a = 0; a < WBC_GMO_F; a++)
    {
      free_[a] = 1.0 - pinned[a];
      rhs[a] = free_[a] * g[a];
    }
    for (int a = 0; a < WBC_GMO_F; a++)
    {
      for (int b = 0; b < WBC_GMO_F; b++)
      {
        o->k[a][b] = free_[a] * o->h[a][b] * free_[b] + (a == b) * pinned[a];
      }
    }
    wbc_cholesky(WBC_GMO_F, WBC_GMO_F, &o->k[0][0]);
    wbc_chol_solve(WBC_GMO_F, WBC_GMO_F, &o->k[0][0], rhs, o->f);
    for (int a = 0; a < WBC_GMO_F; a++)
    {
      pinned[a] = fmax(pinned[a], push[a] * (o->f[a] < 0.0));
    }
  }
  for (int a = 0; a < WBC_GMO_F; a++)
  {
    o->f[a] = push[a] ? fmax(o->f[a], 0.0) : o->f[a];
  }
}

/** Each manifold's normal force: a sole's force along its up, a push's along each of its
    manifolds' normals, pushing. */
static void shared(wbc_gmo_t *o)
{
  memset(o->force, 0, sizeof o->force);
  for (int s = 0; s < WBC_GMO_SHARES; s++)
  {
    const int p = o->share_point[s], c = o->col[p];
    double    world[3], axis[3], along = 0.0;

    if (o->sole[p])
    {
      memcpy(world, &o->f[c + 3], sizeof world);
    }
    else
    {
      turned(o->body.t[o->link[p]].r, o->normal[p], world);
      for (int i = 0; i < 3; i++)
      {
        world[i] *= o->f[c];
      }
    }
    turned(o->body.t[o->share_link[s]].r, o->share_axis[s], axis);
    for (int i = 0; i < 3; i++)
    {
      along += world[i] * axis[i];
    }
    o->force[o->share_manifold[s]] += o->sole[p] ? along : fmax(along, 0.0);
  }
}

void wbc_gmo_step(wbc_gmo_t *o, const wbc_frame_t *base, const double q[WBC_N - 6], const double u[WBC_N], double dt)
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
  columns(o);
  split(o);
  shared(o);
}
