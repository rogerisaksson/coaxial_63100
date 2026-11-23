/** wbc_inekf.c - machine.senses.inekf.stepped's arithmetic around its points and its step (wbc_inekf.h): numpy's
    row sums (npsum) and maximum (npmaximum) where the robot's python lays its arrays, Python's floats, min and sum
    (pysum) where it sums its contacts' roll. Portable C11, the host's alone. */
#include "wbc_inekf.h"
#include "wbc_np.h"

#include <math.h>

/** inekf.CONTACTS: each sole and the robot's core. */
#define CONTACTS 3

void wbc_inekf_laid(const wbc_inekf_t *k, const double *y, const double *shares, int n, double *local, double *weights,
                    double *sh)
{
  double core[2];

  for (int c = 0; c < 2; c++)
  {
    const double *pushed = y + k->touch + (3 * k->core[c]);
    double        sq[3];

    for (int i = 0; i < 3; i++)
    {
      sq[i] = pushed[i] * pushed[i];
    }
    core[c] = sqrt(npsum(sq, 3));
  }
  for (int r = 0; r < k->insoles; r++)
  {
    for (int i = 0; i < 3; i++)
    {
      local[(3 * r) + i] = y[k->cop + (3 * r) + i];
    }
    weights[r] = npmaximum(y[k->press + r], 0.0) + 1e-9;
  }
  for (int c = 0; c < 2; c++)
  {
    for (int i = 0; i < 3; i++)
    {
      local[(3 * (k->insoles + c)) + i] = y[k->at + (3 * k->core[c]) + i];
    }
    weights[k->insoles + c] = core[c] + 1e-9;
  }
  for (int c = 0; c < CONTACTS; c++)
  {
    sh[c] = (c < n) ? shares[c] : 0.0;
  }
  sh[2] = sh[2] * (((core[0] + core[1]) >= k->touched_n) ? 1.0 : 0.0);
}

double wbc_inekf_rolled(const wbc_inekf_t *k, const double *soles, const double *anchors, const double *sh,
                        const double *on, double cap, double gyro2dt, double *d)
{
  pysum_t rolled = {0.0, 0.0, 0};
  double  sum;

  for (int c = 0; c < CONTACTS; c++)
  {
    const double x = soles[3 * c] - anchors[3 * c];
    const double y = soles[(3 * c) + 1] - anchors[(3 * c) + 1];
    const double z = soles[(3 * c) + 2] - anchors[(3 * c) + 2];
    const double r = ((x * x) + (y * y)) + (z * z);
    const double m = (cap < r) ? cap : r;

    pysum_add(&rolled, (m * ((sh[c] >= k->release) ? 1.0 : 0.0)) * ((on[c] > 0.5) ? 1.0 : 0.0));
  }
  sum = pysum_of(&rolled);
  d[0] += gyro2dt;
  d[5] += sum;
  d[10] += sum;
  d[15] += sum;
  return sum;
}
