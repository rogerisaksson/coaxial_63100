/** wbc_bedentry.c - her way into a bed over its end to its middle (wbc_move.h): machine.moves.bedentry's laid and
    step on its tables, numpy's and Python's arithmetic operation for operation (wbc_np.h), every branch a select. */
#include "wbc_move.h"
#include "wbc_np.h"

void wbc_bedentry_laid(double *state)
{
  for (int i = 0; i < WBC_BEDENTRY_SLOTS; i++)
  {
    state[i] = 0.0;
  }
}

double wbc_bedentry_step(const wbc_bedentry_t *b, double *state, double *cmd, double dt, double soles, double along,
                         double half, double pitch, double speed)
{
  const int    k = (int)state[WBC_BEDENTRY_PHASE];
  const double way = state[WBC_BEDENTRY_WAY] != 0.0 ? state[WBC_BEDENTRY_WAY] : (along < 0.0 ? -1.0 : 1.0);
  const double remaining = way * along;
  const int    still = speed <= b->still;
  int          reached = 1, kinds[6], due, again;

  for (int i = 0; i < WBC_BEDENTRY_MOVED; i++)
  {
    const int    j = b->joint[i], p = b->part[i];
    const double how = b->how[k][p], x = cmd[j], goal = wbc_anatomy_aimed(how, b->goal[k][p], x);

    cmd[j] = wbc_anatomy_driven(how, goal, b->rate[k][p], x, x, x, 0.0, dt);
    reached = reached && (fabs(goal - cmd[j]) <= b->tol || how == WBC_ANATOMY_HELD);
  }
  kinds[0] = reached;
  kinds[1] = reached || soles >= b->plant;
  kinds[2] = half - remaining >= b->pull_in || (reached && still);
  kinds[3] = reached && pitch <= b->lain;
  kinds[4] = reached && still;
  kinds[5] = reached || remaining <= 0.0;
  due = kinds[b->over[k]];
  state[WBC_BEDENTRY_WAY] = way;
  state[WBC_BEDENTRY_COUNT] = state[WBC_BEDENTRY_COUNT] + b->counts[k] * (double)due;
  again = remaining > b->near && state[WBC_BEDENTRY_COUNT] < b->strokes;
  state[WBC_BEDENTRY_PHASE] = due ? (again ? b->again[k] : b->next[k]) : (double)k;
  return (double)(state[WBC_BEDENTRY_PHASE] >= (double)b->done);
}
