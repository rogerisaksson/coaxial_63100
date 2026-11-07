/** wbc_bedexit.c - her way out of a bed over its end (wbc_move.h): machine.moves.bedexit's laid and step on its
    tables, numpy's and Python's arithmetic operation for operation (wbc_np.h), every branch a select. */
#include "wbc_move.h"
#include "wbc_np.h"

void wbc_bedexit_laid(const wbc_bedexit_t *b, const double *q_deg, double t, double *state)
{
  state[WBC_BEDEXIT_PHASE] = 0.0;
  state[WBC_BEDEXIT_AT] = t;
  state[WBC_BEDEXIT_COUNT] = 0.0;
  state[WBC_BEDEXIT_OVER] = 0.0;
  for (int j = 0; j < b->joints; j++)
  {
    state[WBC_BEDEXIT_FROM + j] = q_deg[j];
  }
}

double wbc_bedexit_step(const wbc_bedexit_t *b, double *state, double *cmd, const double *q_deg, double t, double dt,
                        double soles, double shins, double pitch)
{
  const int    k = (int)state[WBC_BEDEXIT_PHASE];
  const double el = t - state[WBC_BEDEXIT_AT], a = npminimum(el / b->length[k], 1.0);
  const int    over = state[WBC_BEDEXIT_OVER] > 0.5;
  int          due, pushed, stretched;
  double       knees = 0.0, folded = 0.0, sat, seated;

  for (int i = 0; i < WBC_BEDEXIT_MOVED; i++)
  {
    const int    j = b->joint[i], p = b->part[i];
    const double how = over ? b->how_over[k][p] : b->how[k][p], goal = over ? b->over[k][p] : b->goal[k][p];
    const double rate = over ? b->rate_over[k][p] : b->rate[k][p], q = q_deg[j];
    const double moved = wbc_anatomy_driven(how, goal, rate, cmd[j], q, state[WBC_BEDEXIT_FROM + j], a, dt);

    cmd[j] = (p == 0 && b->holds[k] > 0.5)
                 ? wbc_anatomy_held(moved, b->trunk_gain, pitch, b->trunk[k], dt, b->hip_lo, b->hip_hi)
                 : moved;
    knees = knees + (p == 1 ? q : 0.0);
    folded = folded + (double)(p == 1);
  }
  sat = state[WBC_BEDEXIT_OVER] * (double)(soles >= b->sole_share) * (double)(knees / folded >= b->knee_seat);
  due = el >= b->length[k] || (k == b->pushed && sat > 0.0);
  pushed = due && k == b->pushed;
  stretched = due && k == b->stretched;
  state[WBC_BEDEXIT_COUNT] = state[WBC_BEDEXIT_COUNT] + (double)pushed;
  state[WBC_BEDEXIT_OVER] = npmaximum(state[WBC_BEDEXIT_OVER], (double)(stretched && shins < b->shin_share));
  seated = sat + (double)(state[WBC_BEDEXIT_COUNT] >= b->strokes);
  state[WBC_BEDEXIT_PHASE] = due ? ((pushed && seated > 0.0) ? (double)b->seated : b->next[k]) : (double)k;
  state[WBC_BEDEXIT_AT] = due ? t : state[WBC_BEDEXIT_AT];
  for (int j = 0; j < b->joints; j++)
  {
    state[WBC_BEDEXIT_FROM + j] = due ? cmd[j] : state[WBC_BEDEXIT_FROM + j];
  }
  return (double)(state[WBC_BEDEXIT_PHASE] >= (double)b->done);
}
