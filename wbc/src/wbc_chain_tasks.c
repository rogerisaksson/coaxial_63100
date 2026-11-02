/** wbc_chain_tasks.c - the closed chains' rows above posture (wbc_step.h): machine.stack.chain.tasks and
    ways, numpy's arithmetic and its OpenBLAS (wbc_blas.h) operation for operation. Portable C11, the host's
    alone; wbc_chain.c solves over them. */
#include "wbc_np.h"
#include "wbc_step.h"

#define VIEW(at, s0, s1) ((wbc_view_t){(at), (s0), (s1)})

/** `row` weighed into R, c into r. */
static void laid(double *R, double *r, int *m, int n, const double *row, double c, double weight)
{
  for (int i = 0; i < n; i++)
  {
    R[*m * n + i] = weight * row[i];
  }
  r[*m] = weight * c;
  *m = *m + 1;
}

int wbc_chain_tasks(const wbc_step_t *p, double *R, double *r, double *want, double *jv)
{
  const wbc_blas_t *l = &p->blas;
  const int         n = p->n;
  int               m = 0;
  double            err[3], a6[6];

  for (int t = 0; t < p->nturns; t++)
  {
    if (t == 0)
    {
      /* her pelvis's turn about its own up and way, not its left (machine.stack.chain.ways): its pitch is her
         posture's; its error the turn taking its left to the asked left, numpy's cross of the two */
      const double *q = p->tquat[0], *a = p->turn[0], *J = p->Jr[0];
      const double  ways[2][3] = {{2.0 * (q[1] * q[2] - q[0] * q[3]), 1.0 - 2.0 * (q[1] * q[1] + q[3] * q[3]), 2.0 * (q[2] * q[3] + q[0] * q[1])},
                                  {2.0 * (q[1] * q[3] + q[0] * q[2]), 2.0 * (q[2] * q[3] - q[0] * q[1]), 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2])}};
      const double  lh[3] = {1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]), 2.0 * (q[1] * q[2] + q[0] * q[3]), 2.0 * (q[1] * q[3] - q[0] * q[2])};
      const double  la[3] = {1.0 - 2.0 * (a[2] * a[2] + a[3] * a[3]), 2.0 * (a[1] * a[2] + a[0] * a[3]), 2.0 * (a[1] * a[3] - a[0] * a[2])};

      err[0] = lh[1] * la[2] - lh[2] * la[1];
      err[1] = lh[2] * la[0] - lh[0] * la[2];
      err[2] = lh[0] * la[1] - lh[1] * la[0];
      for (int b = 0; b < 3; b++)
      {
        err[b] = (p->chain_turn_kp * err[b] - p->chain_turn_kd * p->tw[0][b]) - p->dJr[0][b];
      }
      for (int ax = 0; ax < 2; ax++, m++)
      {
        for (int i = 0; i < n; i++)
        {
          R[m * n + i] = p->chain_w_turn * (ways[ax][0] * J[i] + ways[ax][1] * J[n + i] + ways[ax][2] * J[2 * n + i]);
        }
        r[m] = p->chain_w_turn * (ways[ax][0] * err[0] + ways[ax][1] * err[1] + ways[ax][2] * err[2]);
      }
      continue;
    }
    wbc_step_turn_error(l, p->turn[t], p->tquat[t], err);
    for (int a = 0; a < 3; a++)
    {
      laid(R, r, &m, n, p->Jr[t] + a * n, (p->chain_turn_kp * err[a] - p->chain_turn_kd * p->tw[t][a]) - p->dJr[t][a],
           p->chain_w_turn);
    }
  }
  for (int k = 0; k < 2; k++)
  {
    if (!p->swing[k])
    {
      continue;
    }
    wbc_matmul(l, VIEW(p->J6[k], n, 1), VIEW(p->v, 1, 0), jv, 1, 0, 6, n, 1);
    wbc_step_turn_error(l, p->swing_quat[k], p->quat[k], err);
    for (int a = 0; a < 3; a++)
    {
      a6[a] = p->chain_turn_kp * err[a] - p->chain_turn_kd * jv[a];
      a6[3 + a] = (p->swing_acc[k][a] + p->swing_kd[0] * (p->swing_at[k][a] - p->sole[k][a]))
                  + p->swing_kd[1] * (p->swing_speed[k][a] - jv[3 + a]);
    }
    for (int a = 0; a < 6; a++)
    {
      laid(R, r, &m, n, p->J6[k] + a * n, a6[a] - p->dJv[k][a], p->chain_w_swing);
    }
  }
  for (int i = 0; i < p->nd; i++)
  {
    want[i] = p->chain_posture_kp * (p->posture[i] - p->q[i]) - p->chain_posture_kd * p->v[p->act[i]];
  }
  return m;
}
