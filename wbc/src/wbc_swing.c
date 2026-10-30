/** wbc_swing.c - a sole that does not stand (wbc_imp.h): machine.law.swing's free and rested, numpy's
    arithmetic operation for operation (wbc_np.h). Portable C11, the host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>

/** machine.law.swing.free: a sole that does not stand on the limbs' own impedance - its
    acceleration in `frame`, the frame its place is read in. */
void wbc_imp_swings(const wbc_imp_t *k, const double *y, const double *fr, const double v[3], double om,
                    const double shod[2][3], const double floors[2], double rise, const wbc_imp_said_t *s,
                    const double clear[3][2], const double stance[2], double cruise, double grounded,
                    double propped, double got[2][3], double frame[3][3])
{
  const double *rot = fr + k->f_rot, *q = y + k->y_quat, *soles = y + k->y_sole, *sp = y + k->y_spin;
  const double  stood = npmax(stance[0], stance[1]);
  const double  q1 = pow(q[1], 2.0), q2 = pow(q[2], 2.0), q3 = pow(q[3], 2.0);
  const double  hers[3][3] = {
      {2.0 * (q[1] * q[3] + q[0] * q[2]), 2.0 * (q[2] * q[3] - q[0] * q[1]), 1.0 - 2.0 * (q1 + q2)},
      {1.0 - 2.0 * (q2 + q3), 2.0 * (q[1] * q[2] + q[0] * q[3]), 2.0 * (q[1] * q[3] - q[0] * q[2])},
      {2.0 * (q[1] * q[2] - q[0] * q[3]), 1.0 - 2.0 * (q1 + q3), 2.0 * (q[2] * q[3] + q[0] * q[1])}};
  double        arm[2][3], moved[2][3], speed[2][3], at[2][3], off3[3][2][3], w[WBC_IMP_N], e[6], bears[2];
  double        bo[3][3];
  double        sr[2][3], along[2], aloft[2], ds[2], d1, d2, d3, d4, lead[3];

  for (int i = 0; i < 3; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      frame[i][j] = grounded * rot[3 * i + j] + (1.0 - grounded) * hers[i][j];
    }
  }
  for (int b = 0; b < 2; b++)
  {
    for (int j = 0; j < 3; j++)
    {
      arm[b][j] = soles[3 * b + j] - y[k->y_com + j];
    }
  }
  for (int b = 0; b < 2; b++)
  {
    const double cr[3] = {sp[1] * arm[b][2] - sp[2] * arm[b][1], sp[2] * arm[b][0] - sp[0] * arm[b][2],
                          sp[0] * arm[b][1] - sp[1] * arm[b][0]};

    for (int j = 0; j < 3; j++)
    {
      moved[b][j] = (y[k->y_vsole + 3 * b + j] - y[k->y_vcom + j]) - (1.0 - grounded) * cr[j];
    }
  }
  npmmt(&moved[0][0], 2, &frame[0][0], &speed[0][0]);
  npmmt(&arm[0][0], 2, &frame[0][0], &at[0][0]);
  for (int m = 0; m < k->n; m++)
  {
    w[m] = (s->ref[m] - npeye(m, k->axial) * npmin(s->ref[k->axial], 0.0)) - fr[k->f_z0 + m];
  }
  npein(k->n, s->hands, w, e);
  for (int i = 0; i < 6; i++)
  {
    off3[0][i / 3][i % 3] = e[i] - at[i / 3][i % 3];
  }
  npein(k->n, s->hands, s->rate, e);
  for (int i = 0; i < 6; i++)
  {
    off3[1][i / 3][i % 3] = e[i] - speed[i / 3][i % 3];
  }
  npein(k->n, s->hands, s->acc, e);
  for (int i = 0; i < 6; i++)
  {
    off3[2][i / 3][i % 3] = e[i];
  }
  for (int r = 0; r < 3; r++)
  {
    for (int b = 0; b < 2; b++)
    {
      off3[r][b][2] += clear[r][b];
    }
  }
  wbc_imp_shares(k, stance, bears);
  for (int r = 0; r < 3; r++)
  {
    for (int j = 0; j < 3; j++)
    {
      bo[r][j] = npdot(bears, 1, &off3[r][0][j], 3, 2);
    }
  }
  for (int r = 0; r < 3; r++)
  {
    for (int b = 0; b < 2; b++)
    {
      for (int j = 0; j < 3; j++)
      {
        off3[r][b][j] = off3[r][b][j] - (stood * k->between[j]) * bo[r][j];
      }
    }
  }
  npmmt(soles, 2, rot, &sr[0][0]);
  for (int b = 0; b < 2; b++)
  {
    aloft[b] = stood * (1.0 - stance[b]);
    along[b] = sr[b][0];
    ds[b] = soles[3 * b + 1] - y[k->y_lowest + b];
  }
  d1 = npdot(bears, 1, ds, 1, 2);
  d2 = npdot(bears, 1, along, 1, 2);
  d3 = npdot(bears, 1, &shod[0][2], 3, 2);
  d4 = npdot(bears, 1, &shod[0][0], 3, 2);
  for (int b = 0; b < 2; b++)
  {
    off3[0][b][2] -= aloft[b] * (d1 - rise * (along[b] - d2));
  }
  for (int b = 0; b < 2; b++)
  {
    off3[1][b][2] -= aloft[b] * (d3 - rise * (shod[b][0] - d4));
  }
  for (int b = 0; b < 2; b++)
  {
    const double fl[3] = {clear[0][b] - (y[k->y_lowest + b] - floors[b]), clear[1][b] - shod[b][2], clear[2][b]};
    const double held = npmax(grounded - stood, propped * (1.0 - stance[b]));

    for (int r = 0; r < 3; r++)
    {
      off3[r][b][2] += held * (fl[r] - off3[r][b][2]);
    }
  }
  for (int j = 0; j < 3; j++)
  {
    lead[j] = ((grounded * npeye(j, 0)) * (v[0] - cruise)) / om;
  }
  for (int b = 0; b < 2; b++)
  {
    for (int j = 0; j < 3; j++)
    {
      got[b][j] = (off3[2][b][j] + s->kp[k->free_by[j]] * (off3[0][b][j] + lead[j])) + s->kd[k->free_by[j]] * off3[1][b][j];
    }
  }
}

/** machine.law.swing.rested: a sole that neither stands nor strides while anything bears her - a seat,
    her other sole -, her ZMP let go to her text (`held` 0), rests where it is, still on the floor
    under it; `a`'s soles' accelerations and rests laid. */
void wbc_imp_rested(const wbc_imp_t *k, const wbc_imp_said_t *s, double held, double grounded, const double aloft[2],
                    const double limbs[2], const double floors[2], const double *y, const double shod[2][3],
                    const double *rot, double *a)
{
  double kept[2][3], kw[2][3];

  for (int b = 0; b < 2; b++)
  {
    for (int j = 0; j < 3; j++)
    {
      kept[b][j] = (s->kp[k->free_by[j]] * npeye(j, 2)) * (floors[b] - y[k->y_lowest + b])
                   - s->kd[k->free_by[j]] * shod[b][j];
    }
  }
  npmm(&kept[0][0], rot, &kw[0][0]);
  for (int b = 0; b < 2; b++)
  {
    const double rest = (((1.0 - npclip(held, 0.0, 1.0)) * grounded) * aloft[b]) * (1.0 - limbs[b] * grounded);

    for (int j = 0; j < 3; j++)
    {
      double *sa = a + k->a_sole_acc + 3 * b + j;

      *sa = *sa + rest * (kw[b][j] - *sa);
    }
    a[k->a_rests + b] = rest;
  }
}
