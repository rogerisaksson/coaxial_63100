/** wbc_anatomy.c - her moves' anatomy (wbc_move.h): each joint driven as its phase says, her trunk held by her hips -
    machine.moves.anatomy's arithmetic operation for operation (wbc_np.h), every branch a select. */
#include "wbc_move.h"
#include "wbc_np.h"

double wbc_anatomy_aimed(double how, double goal, double x)
{
  return how == WBC_ANATOMY_LEAST ? npmaximum(x, goal) : goal;
}

double wbc_anatomy_driven(double how, double goal, double rate, double x, double q, double start, double a, double dt)
{
  const double reach = rate * dt, to = wbc_anatomy_aimed(how, goal, x);

  return how == WBC_ANATOMY_BLEND                                ? start + a * (to - start)
         : (how == WBC_ANATOMY_RATE || how == WBC_ANATOMY_LEAST) ? x + npclip(to - x, -reach, reach)
         : how == WBC_ANATOMY_FOLLOW                             ? q + to
         : how == WBC_ANATOMY_TOWARD                             ? q + npclip(to - q, 0.0, rate)
                                                                 : x;
}

double wbc_anatomy_held(double x, double gain, double pitch, double trunk, double dt, double lo, double hi)
{
  return npclip(x + gain * (pitch - trunk) * dt, lo, hi);
}
