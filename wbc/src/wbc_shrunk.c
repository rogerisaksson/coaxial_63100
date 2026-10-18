/** wbc_shrunk.c - what the step without an active set (wbc_wrench.c) hands out: every
    acceleration and the drives' torques at its unknowns; an ask the drives cannot carry shrunk
    as one toward nothing asked, as far as brings every torque within its clamp
    (machine.wrench.shrunk). */
#include "wbc.h"

#include <math.h>

#define NU WBC_U

/** Every acceleration at the unknowns x: the pelvis's six by p, then udot = t base + e x + t0. */
void wbc_wrench_motion(const wbc_wrench_t *w, const double x[NU], double udot[WBC_N])
{
  double d[6];

  for (int i = 0; i < 6; i++)
  {
    d[i] = w->p0[i];
    for (int c = 0; c < NU; c++)
    {
      d[i] += w->p[i][c] * x[c];
    }
  }
  for (int n = 0; n < WBC_N; n++)
  {
    double v = w->t0[n];

    for (int i = 0; i < 6; i++)
    {
      v += w->t[n][i] * d[i];
    }
    for (int c = w->reach[n][0]; c < w->reach[n][1]; c++)
    {
      v += w->e[n][c] * x[c];
    }
    udot[n] = v;
  }
}

/** The drives' torques at the unknowns x and the accelerations udot they give: M udot + h less
    what the floor gives through the standing soles. */
void wbc_wrench_torques(const wbc_body_t *b, const wbc_wrench_t *w, const double stance[2],
                        const double x[NU], const double udot[WBC_N], double tau[WBC_DRIVEN])
{
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int dof = 5 + wbc_driven[j];
    double    v = b->h[dof];

    for (int n = 0; n < WBC_N; n++)
    {
      v += b->m[dof][n] * udot[n];
    }
    for (int k = 0; k < 2; k++)
    {
      for (int i = 0; i < 6; i++)
      {
        v -= w->jsole[k][i][dof] * (stance[k] * x[6 * k + i]);
      }
    }
    tau[j] = v;
  }
}

/** How far from tau0, the torques with nothing asked, toward tau, the ask's, every drive stays
    within its limit: the largest s in [0, 1] with every |tau0 + s (tau - tau0)| <= limit, each
    drive's in closed form, the least of them; 0 where none is. */
double wbc_shrunk(const double tau[WBC_DRIVEN], const double tau0[WBC_DRIVEN],
                  const double limit[WBC_DRIVEN])
{
  double s = 1.0;

  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const double d = tau[j] - tau0[j], l = limit[j];
    double       sj;

    if (d == 0.0)
    {
      sj = (fabs(tau[j]) <= l) ? 1.0 : 0.0;
    }
    else
    {
      const double leaves = ((d > 0.0) ? l - tau0[j] : -l - tau0[j]) / d;
      const double enters = ((d > 0.0) ? -l - tau0[j] : l - tau0[j]) / d;

      sj = (leaves < 1.0) ? leaves : 1.0;
      sj = (sj >= ((enters > 0.0) ? enters : 0.0)) ? sj : 0.0;
    }
    s = (sj < s) ? sj : s;
  }
  return (s < 0.0) ? 0.0 : s;
}
