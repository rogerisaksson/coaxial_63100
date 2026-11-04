/** wbc_scoot.c - her scoot on the floor as her twist tells it (wbc_move.h): machine.moves.scoot's laid and step on its
    tables, numpy's and Python's arithmetic operation for operation (wbc_np.h), every branch a select. */
#include "wbc_move.h"
#include "wbc_np.h"

void wbc_scoot_laid(double *state)
{
  for (int i = 0; i < WBC_SCOOT_SLOTS; i++)
  {
    state[i] = 0.0;
  }
}

double wbc_scoot_step(const wbc_scoot_t *s, double *state, double *cmd, double dt, double pitch, double hands,
                      double left, double right, double back, double turn, double heading)
{
  const int k = (int)state[WBC_SCOOT_PHASE];
  const int asked = fabs(back) + fabs(turn) > s->ask, turning = fabs(turn) > s->ask;
  const int    spin = state[WBC_SCOOT_WAY] * state[WBC_SCOOT_WAY + 1] < 0.0;
  const double lean = s->trunk + (s->upright - s->trunk) *
                                     (double)(k == s->stroking && state[WBC_SCOOT_WAY] > 0.0 && state[WBC_SCOOT_WAY + 1] > 0.0);
  int       reached = 1, stroked = 1, kinds[5], over;
  double    nxt, aim, off, share;

  for (int i = 0; i < WBC_SCOOT_MOVED; i++)
  {
    const int    j = s->joint[i], p = s->part[i], hip = p == 0, knee = p == 1;
    const double side = s->side[i], way = state[WBC_SCOOT_WAY + (side < 0.0)], how = s->how[k][p], x = cmd[j];
    const double foot = side > 0.0 ? left : right;
    const int    stretched = way > 0.0, folding = way < 0.0;
    const double start = stretched ? s->fold : folding ? s->stretch : x;
    const double end = start + ((stretched ? s->stretch : folding ? s->fold : x) - start) * fabs(way);
    const double begin = knee ? start : way != 0.0 ? s->folded : x;
    const double goal = (how == 1.0 || how == 5.0) ? s->goal[k][p]
                        : how == 2.0               ? end
                        : how == 3.0               ? begin
                        : how == 4.0               ? (stretched ? npmaximum(x, s->stretch + s->settle) : x)
                                                   : x;
    const double rate = how == 2.0                 ? s->stroke_rate * fabs(way)
                        : (hip && how == 3.0)      ? s->rate[k][p] * (double)(stretched || foot >= s->skim || spin)
                        : how == 5.0               ? s->rate[k][p] * (double)(foot < s->bear)
                                                   : s->rate[k][p];
    const double reach = rate * dt;
    const double moved = how == 0.0 ? x : x + npclip(goal - x, -reach, reach);
    const double held = npclip(moved + s->gain * (pitch - lean) * dt, s->hip_lo, s->hip_hi);
    int          near;

    cmd[j] = (hip && (how == 0.0 || rate == 0.0)) ? held : moved;
    near = fabs(goal - cmd[j]) <= s->tol;
    reached = reached && (near || how == 0.0 || (hip && rate == 0.0));
    stroked = stroked && (near || rate == 0.0 || !knee);
  }
  aim = turning ? heading + s->lead * ((double)(turn > 0.0) - (double)(turn < 0.0))
                : ((asked && state[WBC_SCOOT_AIMED] > 0.5) ? state[WBC_SCOOT_AIM] : heading);
  state[WBC_SCOOT_AIM] = aim;
  state[WBC_SCOOT_AIMED] = 1.0;
  off = aim - heading;
  off = off - 360.0 * floor((off + 180.0) / 360.0);
  share = npclip(off / s->stroke_turn, -s->turn, s->turn);
  kinds[0] = reached;
  kinds[1] = reached && hands >= s->bear && fabs(pitch - s->trunk) <= s->leant && left >= s->bear && right >= s->bear;
  kinds[2] = (left >= s->bear && right >= s->bear) || reached;
  kinds[3] = stroked || !asked;
  kinds[4] = asked;
  over = kinds[s->over[k]];
  nxt = asked ? s->asked[k] : s->idle[k];
  for (int w = 0; w < 2; w++)
  {
    const double side = w == 0 ? 1.0 : -1.0;

    state[WBC_SCOOT_WAY + w] = (over && nxt == (double)s->cycle)
                                   ? npclip(back + side * share * (1.0 - 2.0 * (double)(turning && back > -s->ask)), -1.0, 1.0)
                                                                 : state[WBC_SCOOT_WAY + w];
  }
  state[WBC_SCOOT_PHASE] = over ? nxt : (double)k;
  return 0.0;
}
