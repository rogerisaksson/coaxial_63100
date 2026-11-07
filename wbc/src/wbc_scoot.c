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
  const int k = (int)state[WBC_SCOOT_PHASE], n = (int)state[WBC_SCOOT_STEP];
  const int pushed = state[WBC_SCOOT_WAY] > 0.0 && state[WBC_SCOOT_WAY + 1] > 0.0;
  const int up = pitch <= s->trunk - s->leant;
  const int trunked = s->trunked[k] > 0.5 && (pushed || k != s->pushing) && (left >= s->bear || right >= s->bear);
  const int asked = fabs(back) + fabs(turn) > s->ask, turning = fabs(turn) > s->ask;
  const int last = n == WBC_SCOOT_STEPS - 1;
  const int leaned = fabs(pitch - s->trunk) <= s->leant;
  int       reached = 1, stroked = 1, over, kinds[7];
  double    was[WBC_SCOOT_MOVED], aim, off, share, want[2], nxt;

  for (int i = 0; i < WBC_SCOOT_MOVED; i++)
  {
    was[i] = cmd[s->joint[i]];
  }
  for (int i = 0; i < WBC_SCOOT_MOVED; i++)
  {
    const int    j = s->joint[i], p = s->part[i], sd = s->sided[i], hip = p == 0, knee = p == 1;
    const double way = state[WBC_SCOOT_WAY + sd];
    const double how = (s->masked[k] > 0.5 && p < 2 && s->steps[n][i] < 0.5) ? 0.0 : s->how[k][p];
    const double x = was[i], pair = was[s->partner[i]];
    const double foot = sd == 0 ? left : right;
    const int    stretched = way > 0.0, folding = way < 0.0;
    const double start = stretched ? s->fold : folding ? s->stretch : x;
    const double end = start + ((stretched ? s->stretch : folding ? s->fold : x) - start) * fabs(way);
    const double begin = knee ? start : way != 0.0 ? s->folded : x;
    const double goal = how == 5.0                               ? 0.5 * (x + pair)
                        : (how == 1.0 || how == 4.0 || how == 6.0) ? s->goal[k][p]
                        : how == 2.0                             ? end
                        : how == 3.0                             ? begin
                                                                 : x;
    const double rate = how == 2.0             ? s->stroke_rate * fabs(way)
                        : (hip && how == 3.0) ? s->rate[k][p] * (double)(stretched || foot >= s->skim)
                        : how == 4.0          ? s->rate[k][p] * (double)(foot < s->skim)
                        : how == 6.0          ? s->rate[k][p] * (double)(!up)
                                              : s->rate[k][p];
    const double moved = wbc_anatomy_driven(how == 0.0 ? WBC_ANATOMY_HELD : WBC_ANATOMY_RATE, goal, rate, x, x, x, 0.0, dt);
    const double held = wbc_anatomy_held(moved, s->gain, pitch, s->trunk, dt, s->hip_lo, s->hip_hi);
    int          near;

    cmd[j] = (hip && (how == 0.0 || how == 5.0 || rate == 0.0) && trunked) ? held : moved;
    near = fabs(goal - cmd[j]) <= s->tol;
    reached = reached && (how == 5.0 ? fabs(x - pair) <= s->tol : (near || how == 0.0 || (hip && rate == 0.0)));
    stroked = stroked && (near || rate == 0.0 || !knee);
  }
  aim = turning ? heading + s->lead * ((double)(turn > 0.0) - (double)(turn < 0.0))
                : ((asked && state[WBC_SCOOT_AIMED] > 0.5) ? state[WBC_SCOOT_AIM] : heading);
  state[WBC_SCOOT_AIM] = aim;
  state[WBC_SCOOT_AIMED] = 1.0;
  off = aim - heading;
  off = off - 360.0 * floor((off + 180.0) / 360.0);
  share = npclip(off / s->stroke_turn, -s->turn, s->turn);
  for (int w = 0; w < 2; w++)
  {
    const double v = npclip(back + (w == 0 ? 1.0 : -1.0) * share, -1.0, 1.0);

    want[w] = back > s->ask ? npmaximum(v, s->half * back) : npminimum(v, 0.0);
  }
  kinds[0] = reached;
  kinds[1] = reached && hands >= s->skim && leaned;
  kinds[2] = (s->order[n] == 0 ? left : right) >= s->skim || reached;
  kinds[3] = stroked || !asked;
  kinds[4] = asked;
  kinds[5] = up;
  kinds[6] = leaned;
  over = kinds[s->over[k]];
  nxt = asked ? s->asked[k] : s->idle[k];
  nxt = (k == s->setting_down && last) ? (asked ? (double)s->reaching : (double)s->holding) : nxt;
  nxt = (k == s->reaching && asked && !pushed) ? (double)s->pushing : nxt;
  for (int w = 0; w < 2; w++)
  {
    state[WBC_SCOOT_WAY + w] = (over && s->cycles[k] > 0.5 && nxt == (double)s->sitting && asked)
                                   ? want[w]
                                   : state[WBC_SCOOT_WAY + w];
  }
  state[WBC_SCOOT_STEP] = (over && k == s->setting_down) ? (n + 1.0) * (double)(!last) : (double)n;
  state[WBC_SCOOT_PHASE] = over ? nxt : (double)k;
  return 0.0;
}
