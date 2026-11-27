/** wbc_shrunk.c - what the step without an active set (wbc_wrench.c) hands out: every
    acceleration and the drives' torques at its unknowns; an ask the drives cannot carry shrunk
    as one toward nothing asked, as far as brings every torque within its clamp
    (machine.stack.wrench.shrunk). */
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
    within its bounds [lo, hi] (0 within them): the largest s in [0, 1] with every tau0 + s (tau -
    tau0) within them, each drive's in closed form, the least of them; 0 where none is. */
double wbc_shrunk(const double tau[WBC_DRIVEN], const double tau0[WBC_DRIVEN], const double lo[WBC_DRIVEN],
                  const double hi[WBC_DRIVEN])
{
  double s = 1.0;

  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const double t0 = (tau0[j] >= lo[j] && tau0[j] <= hi[j]) ? tau0[j] : 0.0, d = tau[j] - t0;
    double       sj;

    if (d == 0.0)
    {
      sj = (tau[j] >= lo[j] && tau[j] <= hi[j]) ? 1.0 : 0.0;
    }
    else
    {
      const double leaves = ((d > 0.0) ? hi[j] - t0 : lo[j] - t0) / d;
      const double enters = ((d > 0.0) ? lo[j] - t0 : hi[j] - t0) / d;

      sj = (leaves < 1.0) ? leaves : 1.0;
      sj = (sj >= ((enters > 0.0) ? enters : 0.0)) ? sj : 0.0;
    }
    s = (sj < s) ? sj : s;
  }
  return (s < 0.0) ? 0.0 : s;
}

/** Each drive's load as its boards carry it: a joint's own torque (N m), or a parallel pair's two
    motor currents (A) as common plus or less differential (machine.stack.shrunk.paired). */
static void paired_vec(const double tau[WBC_DRIVEN], double out[WBC_DRIVEN])
{
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    out[j] = tau[j];
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int pair = wbc_pair[wbc_driven[j]];

    for (int o = 0; (pair > 0) && (o < WBC_DRIVEN); o++)
    {
      if (wbc_driven[o] == pair - 1)
      {
        const double a = tau[j] / wbc_kt[wbc_driven[j]], c = tau[o] / wbc_kt[wbc_driven[o]];

        out[j] = a + c;
        out[o] = a - c;
      }
    }
  }
}

/** Each drive's ceiling matching paired_vec: its torque limit (N m), or a parallel pair's symmetric
    per-motor current ceiling (A) from its drives' limits (machine.stack.shrunk.paired_limit). */
static void paired_lim(const double limit[WBC_DRIVEN], double out[WBC_DRIVEN])
{
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    out[j] = limit[j];
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int pair = wbc_pair[wbc_driven[j]];

    for (int o = 0; (pair > 0) && (o < WBC_DRIVEN); o++)
    {
      if (wbc_driven[o] == pair - 1)
      {
        const double tp = limit[j] / wbc_kt[wbc_driven[j]], tr = limit[o] / wbc_kt[wbc_driven[o]];
        const double top = ((tp > 0.0) && (tr > 0.0)) ? ((tp > tr) ? tp : tr) : 0.0;

        out[j] = out[o] = top;
      }
    }
  }
}

/** The step's shrunk answer into `out`: the accelerations, the soles' wrenches, the drives'
    torques inside their clamps; the drives asked past their clamps and their loads the ask's,
    before it was shrunk. */
void wbc_wrench_out(const wbc_body_t *b, const wbc_wrench_t *w, const wbc_ask_t *ask,
                    const double stance[2], const double whole[NU], const double stood[NU],
                    wbc_out_t *out)
{
  double at[NU], tau[WBC_DRIVEN], tau0[WBC_DRIVEN], carried[WBC_DRIVEN], limit[WBC_DRIVEN], nlimit[WBC_DRIVEN];
  double clamps[WBC_DRIVEN], plim[WBC_DRIVEN], nplim[WBC_DRIVEN], pclamp[WBC_DRIVEN];
  double ptau[WBC_DRIVEN], ptau0[WBC_DRIVEN], lo[WBC_DRIVEN], hi[WBC_DRIVEN];
  double s, s_j, s_m, s0 = 1.0;
  int    past = 0, past0 = 0;

  wbc_wrench_motion(w, whole, out->udot);
  wbc_wrench_torques(b, w, stance, whole, out->udot, tau);
  wbc_wrench_motion(w, stood, out->udot);
  wbc_wrench_torques(b, w, stance, stood, out->udot, tau0);
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const double clamp = wbc_clamp[wbc_driven[j]], peak = wbc_peak[wbc_driven[j]];
    clamps[j] = clamp;
    limit[j] = ((ask->wep && (peak > clamp)) ? peak : clamp) * ask->derate[j];
    nlimit[j] = -limit[j];
    lo[j] = (-limit[j] > ask->lo[j]) ? -limit[j] : ask->lo[j];
    hi[j] = (limit[j] < ask->hi[j]) ? limit[j] : ask->hi[j];
  }
  paired_lim(limit, plim);
  paired_lim(clamps, pclamp);
  paired_vec(tau, ptau);
  paired_vec(tau0, ptau0);
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    nplim[j] = -plim[j];
  }
  s_j = wbc_shrunk(tau, tau0, nlimit, limit);
  s_m = wbc_shrunk(ptau, ptau0, nplim, plim);
  s = (s_m < s_j) ? s_m : s_j;
  for (int c = 0; c < NU; c++)
  {
    at[c] = whole[c] + (s - 1.0) * (whole[c] - stood[c]);
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    past |= (fabs(tau[j]) > limit[j]) || (fabs(ptau[j]) > plim[j]);
    past0 |= (fabs(tau0[j]) > limit[j]) || (fabs(ptau0[j]) > plim[j]);
  }
  if (past && past0)
  {
    double t1[WBC_DRIVEN], pt1[WBC_DRIVEN], z0[WBC_DRIVEN];
    double s0_j, s0_m;

    for (int j = 0; j < WBC_DRIVEN; j++)
    {
      t1[j] = tau[j] + (s - 1.0) * (tau[j] - tau0[j]);
      pt1[j] = ptau[j] + (s - 1.0) * (ptau[j] - ptau0[j]);
      z0[j] = 0.0;
    }
    s0_j = wbc_shrunk(t1, z0, nlimit, limit);
    s0_m = wbc_shrunk(pt1, z0, nplim, plim);
    s0 = (s0_m < s0_j) ? s0_m : s0_j;
    s *= s0;
    if (s0 < 1.0)
    {
      for (int c = 0; c < NU; c++)
      {
        at[c] = s0 * at[c];
      }
    }
  }
  out->alpha[0] = s;
  wbc_wrench_motion(w, at, out->udot);
  if (s0 < 1.0)
  {
    for (int n = 0; n < WBC_N; n++)
    {
      out->udot[n] += (s0 - 1.0) * (w->t0[n] + w->t[n][0] * w->p0[0] + w->t[n][1] * w->p0[1]
                                    + w->t[n][2] * w->p0[2] + w->t[n][3] * w->p0[3]
                                    + w->t[n][4] * w->p0[4] + w->t[n][5] * w->p0[5]);
    }
  }
  wbc_wrench_torques(b, w, stance, at, out->udot, carried);
  if (s0 < 1.0)
  {
    for (int j = 0; j < WBC_DRIVEN; j++)
    {
      carried[j] += (s0 - 1.0) * b->h[5 + wbc_driven[j]];
    }
  }
  for (int k = 0; k < 2; k++)
  {
    for (int i = 0; i < 6; i++)
    {
      out->wrench[k][i] = stance[k] * at[6 * k + i];
    }
    out->bears[k] = out->wrench[k][4];
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const double c = carried[j];
    const double lj = fabs(tau[j]) / clamps[j], lm = fabs(ptau[j]) / pclamp[j];

    out->over[j] = (fabs(tau[j]) > clamps[j] * (1.0 + 1e-6)) || (fabs(ptau[j]) > pclamp[j] * (1.0 + 1e-6));
    out->load[j] = (lm > lj) ? lm : lj;
    out->tau[j] = (c < lo[j]) ? lo[j] : ((c > hi[j]) ? hi[j] : c);
  }
  for (int j = 0; j < WBC_DRIVEN; j++)
  {
    const int pair = wbc_pair[wbc_driven[j]];

    for (int o = 0; (pair > 0) && (o < WBC_DRIVEN); o++)
    {
      if (wbc_driven[o] == pair - 1)
      {
        out->over[j] = out->over[o] = out->over[j] || out->over[o];
      }
    }
  }
}
