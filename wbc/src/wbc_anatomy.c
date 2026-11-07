/** wbc_anatomy.c - her moves' anatomy (wbc_move.h): each joint driven as its phase says, her trunk held by her hips -
    machine.moves.anatomy's arithmetic operation for operation (wbc_np.h), every branch a select. */
#include "wbc_move.h"
#include "wbc_np.h"

double wbc_anatomy_driven(double how, double goal, double rate, double x, double q, double start, double a, double dt)
{
  const double reach = rate * dt;

  return how == WBC_ANATOMY_BLEND    ? start + a * (goal - start)
         : how == WBC_ANATOMY_RATE   ? x + npclip(goal - x, -reach, reach)
         : how == WBC_ANATOMY_FOLLOW ? q + goal
         : how == WBC_ANATOMY_TOWARD ? q + npclip(goal - q, 0.0, rate)
                                     : x;
}

double wbc_anatomy_held(double x, double gain, double pitch, double trunk, double dt, double lo, double hi)
{
  return npclip(x + gain * (pitch - trunk) * dt, lo, hi);
}
