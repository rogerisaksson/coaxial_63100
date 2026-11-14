/** wbc_poses.c - under the ask (wbc_imp.h): machine.law.impedance._poses, her turns, her arms and her knees
    into `a` - numpy's arithmetic operation for operation (wbc_np.h), the same bit for bit. Portable C11, the
    host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>

/** machine.law.impedance._poses: her turns, her arms and her knees into `a`; `z` her modes as she
    is, `up` how far each sole is asked up, `folds` which soles' knees are asked a fold, `still`
    how still her rhythm is - her trunk's rates then her head's, shrunk where they are small. */
void wbc_imp_poses(const wbc_imp_t *k, const double *out, const double *y, const double *fr, double *a,
                   const double *z, const wbc_imp_said_t *s, const double turn[3][2], const double rates[3][2],
                   const double up[2], const double folds[2], double still)
{
  const double *sway = out + k->o_sway, *psi = out + k->o_psi, *mom = y + k->y_moment, *rot = fr + k->f_rot;
  const double  turn_dc = s->ref[k->turn] - sway[k->turn] * sin(psi[k->turn]);
  const int     by[3] = {k->pitch, k->list, k->turn}, at[3] = {k->a_want, k->a_rolled, k->a_headed};
  const double  knee = fr[k->f_knee], yawed = out[k->o_yawed];
  double        bent, cosine, wh[3], spun[3][2];

  wbc_mv(&k->blas, rot, 3, 3, y + k->y_hspin, wh);
  {
    const double dwh[3] = {wh[1] - rates[0][1], wh[0] - rates[1][1], 0.0};

    for (int r = 0; r < 3; r++)
    {
      const double sq = dwh[r] * dwh[r], dwh4 = sq * sq, shrunk = (dwh[r] * dwh4) / (dwh4 + 0.04);

      for (int b = 0; b < 2; b++)
      {
        spun[r][b] = rates[r][b] + ((0.5 * still) * (1.0 - k->pelvis[b])) * shrunk;
      }
    }
  }

  for (int b = 0; b < 2; b++)
  {
    const double p = k->pelvis[b];
    const double aim[3] = {fr[k->f_pitch0 + b] + k->pitch_rad * s->ref[k->pitch], (p * k->list_rad) * s->ref[k->list],
                           k->turn_rad * (p * s->ref[k->turn] + ((1.0 - p) * 1.10) * turn_dc)};
    const double rate[3] = {k->pitch_rad * s->rate[k->pitch] + 0.0 * p, (p * k->list_rad) * s->rate[k->list],
                            (p * k->turn_rad) * s->rate[k->turn]};
    const double acc[3] = {k->pitch_rad * s->acc[k->pitch] + 0.0 * p, (p * k->list_rad) * s->acc[k->list],
                           (p * k->turn_rad) * s->acc[k->turn]};

    for (int r = 0; r < 3; r++)
    {
      const double turning
        = (acc[r] + (s->kp[by[r]] * (aim[r] - turn[r][b])) * (1.0 - (a[k->a_borne] * (1.0 - npclip(out[k->o_held], 0.0, 1.0))) * p * (r == 1)))
          + s->kd[by[r]] * (rate[r] - spun[r][b]);

      a[at[r] + b] = turn[r][b] + (turning + k->turn_kd[1] * spun[r][b]) / k->turn_kd[0];
    }
  }
  a[k->a_turned] = (double)((s->kp[k->pitch] + s->kd[k->pitch]) > 0.0);
  a[k->a_swung] = k->arm_rad * (s->ref[k->arms] + (s->acc[k->arms] + k->posture_kd[1] * s->rate[k->arms]) / k->posture_kd[0]);
  a[k->a_reached] = k->arm_rad * (s->ref[k->reach] + (s->acc[k->reach] + k->posture_kd[1] * s->rate[k->reach]) / k->posture_kd[0]);
  a[k->a_elbowed] = k->arm_rad * (s->ref[k->elbows] + (s->acc[k->elbows] + k->posture_kd[1] * s->rate[k->elbows]) / k->posture_kd[0]);
  bent = z[k->axial] + npclip(s->ref[k->axial] - z[k->axial], -k->ahead, k->ahead);
  cosine = cos(0.5 * knee) - ((k->axial_m * bent) * fr[k->f_hips_by]) / k->leg_m;
  a[k->a_bend] = 2.0 * acos(npclip(cosine, -1.0, 1.0)) - knee;
  a[k->a_bend_rate] = ((((((2.0 * k->axial_m) * fr[k->f_hips_by]) * s->rate[k->axial]) * (double)(fabs(cosine) < 1.0)) / k->leg_m)
                       / sqrt(npmax(1.0 - cosine * cosine, k->tiny))) * (1.0 - npclip(out[k->o_held], 0.0, 1.0));
  for (int b = 0; b < 2; b++)
  {
    const double shorter = npclip(cos(0.5 * (knee + a[k->a_bend])) - (k->clear_m * npmax(up[b], 0.0)) / k->leg_m,
                                  -1.0, 1.0);

    a[k->a_fold + b] = ((2.0 * acos(shorter)) - knee) - a[k->a_bend];
    a[k->a_fold_rate + b] = (((((2.0 * k->clear_m) * k->limbs[b]) * s->rate[k->lift]) * (double)(up[b] > 0.0)) / k->leg_m)
                            / sqrt(npmax(1.0 - shorter * shorter, k->tiny));
    a[k->a_folds + b] = folds[b];
  }
  for (int j = 0; j < 3; j++)
  {
    a[k->a_kdot + j] = (-k->momentum_k) * mom[j]
                       - (k->upward[j] * k->yaw_k) * (mom[1] - npclip(mom[1], -yawed, yawed));
  }
}
