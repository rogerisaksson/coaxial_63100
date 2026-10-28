/** wbc_imp.c - her modes' springs on her body (wbc_imp.h): machine.impedance.ask, machine.modal's
    reading under it, numpy's arithmetic operation for operation (wbc_np.h) - the ask the same bit
    for bit. Portable C11, the host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <math.h>
#include <string.h>

#define PI 3.14159265358979323846

typedef wbc_imp_said_t said_t;

/** machine.impedance._poses: her turns, her arms and her knees into `a`; `z` her modes as she
    is, `up` how far each sole is asked up, `folds` which soles' knees are asked a fold, `still`
    how still her rhythm is - her trunk's rates then her head's, shrunk where they are small. */
static void poses(const wbc_imp_t *k, const double *out, const double *y, const double *fr, double *a,
                  const double *z, const said_t *s, const double turn[3][2], const double rates[3][2],
                  const double up[2], const double folds[2], double still)
{
  const double *sway = out + k->o_sway, *psi = out + k->o_psi, *mom = y + k->y_moment, *rot = fr + k->f_rot;
  const double  turn_dc = s->ref[k->turn] - sway[k->turn] * sin(psi[k->turn]);
  const int     by[3] = {k->pitch, k->list, k->turn}, at[3] = {k->a_want, k->a_rolled, k->a_headed};
  const double  knee = fr[k->f_knee], yawed = out[k->o_yawed];
  double        bent, cosine, wh[3], spun[3][2];

  for (int j = 0; j < 3; j++)
  {
    wh[j] = npmv3(rot + 3 * j, y + k->y_hspin);
  }
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

void wbc_imp_ask(const wbc_imp_t *k, const double *out, const double *y, const double *fr, double *a)
{
  const double *rot = fr + k->f_rot, *sway = out + k->o_sway, *psi = out + k->o_psi, *tilt = y + k->y_tilt;
  const double *tilting = y + k->y_tilt_rate, turns = out[k->o_turns];
  const double  stood[2] = {a[k->a_stance], a[k->a_stance + 1]};
  const int     n = k->n, ax = k->axial;
  said_t        s;
  double        z[WBC_IMP_N], zd[WBC_IMP_N], zdd[WBC_IMP_N], leg[2][3], len[2], lrate[2], turn[3][2], spun[3][2];
  double        c[3], v[3], vh[3], om, om2, shod[2][3], sr[2][3], flat[2][2], xi[2], low[2][2], high[2][2];
  double        floors[2], rise, still, dvh[3];
  double        way4[4][3], plo[2], phig[2], propped, lo[2][2], hi[2][2], slo[2], shi[2], beyond[2], edge0[2];
  double        smax, stepping, way, swung, sweep[2], phi[2], on[2], off[2], u[2], flying[2], kpf[2], kdf[2];
  double        aside[2], hurry[2], limbs[2], tone[3], cruise, lat_dc, across, e[6], both[2], up[2], zmp[2];
  double        stance[2], grounded, aloft[2], bears[2], along[2], gd, reach, meet[2][2], tip[2], tipping[2];
  double        tipped[2], fold[2], shorter[2], hip3[2][3], hips[2], rocked[2], flatten[2], edge[2], rounds[2][3];
  double        rr[2][3], axial[2], pressed, risen[2], lifted, com[3], late[2], lift2[2], clr[3][2], clear[3][2];
  double        wide, top[2], land[2], adj[2], over[2], len2[2], apart, held2[2][2], duck[2];
  double        loose[2][3], frame[3][3], ff[2][3], cm[3], gc[3], th[2], g2[2][2], cat[2][3], gg[2][3], pg2[2], pg;
  double        mvs[2], folds[2], l_ask, short_h, up_h[2];
  int           swaying;

  memset(&s, 0, sizeof s);
  wbc_imp_reading(k, y, fr, stood, z, zd, leg, len, lrate, turn, spun);
  for (int j = 0; j < 3; j++)
  {
    c[j] = npmv3(rot + 3 * j, y + k->y_com);
    v[j] = npmv3(rot + 3 * j, y + k->y_vcom);
    vh[j] = npmv3(rot + 3 * j, y + k->y_vhead);
  }
  om = sqrt(k->g / npmax(c[2] - fr[k->f_floor], k->low_m));
  om2 = pow(om, 2.0);
  npmmt(y + k->y_vsole, 2, rot, &shod[0][0]);
  npmmt(y + k->y_sole, 2, rot, &sr[0][0]);
  for (int b = 0; b < 2; b++)
  {
    flat[b][0] = sr[b][0];
    flat[b][1] = sr[b][1];
    edge0[b] = a[k->a_edge + b];
  }
  xi[0] = c[0] + v[0] / om;
  xi[1] = c[1] + v[1] / om;
  wbc_imp_boxes(k, flat, edge0, tilt, low, high);
  rise = wbc_imp_under(k, y, fr, stood, floors);
  /* what bears her off her soles, in her way's frame */
  {
    const double xs[2] = {y[k->y_prop_lo], y[k->y_prop_hi]}, zs[2] = {y[k->y_prop_lo + 2], y[k->y_prop_hi + 2]};
    const double corner[4][3] = {{xs[0], 0.0, zs[0]}, {xs[0], 0.0, zs[1]}, {xs[1], 0.0, zs[0]}, {xs[1], 0.0, zs[1]}};

    npmmt(&corner[0][0], 4, rot, &way4[0][0]);
  }
  /* her support's comparator, the master insert's: latched from its engage, let go under its
     release held its dwell, never within its strike's blanking; its box held under the release */
  {
    const double share = y[k->y_prop] + (y[k->y_palms] + y[k->y_palms + 1]), latched = a[k->a_propped];
    const double struck = (1.0 - latched) * (double)(share >= k->prop_engage);
    const double under = latched * (double)(share < k->prop_release);
    double       freed;

    a[k->a_propped_below] = under * (a[k->a_propped_below] + fr[k->f_dt]);
    a[k->a_propped_since] = (1.0 - struck) * (a[k->a_propped_since] + fr[k->f_dt]);
    freed = (under * (double)(a[k->a_propped_since] >= k->prop_blank_s)) * (double)(a[k->a_propped_below] >= k->prop_dwell_s);
    a[k->a_propped] = (latched + struck) - freed;
    for (int j = 0; j < 2; j++)
    {
      const double lo_j = npmin(npmin(npmin(way4[0][j], way4[1][j]), way4[2][j]), way4[3][j]);
      const double hi_j = npmax(npmax(npmax(way4[0][j], way4[1][j]), way4[2][j]), way4[3][j]);

      a[k->a_propped_box + j] = share >= k->prop_release ? lo_j : a[k->a_propped_box + j];
      a[k->a_propped_box + 2 + j] = share >= k->prop_release ? hi_j : a[k->a_propped_box + 2 + j];
      plo[j] = a[k->a_propped_box + j];
      phig[j] = a[k->a_propped_box + 2 + j];
    }
    propped = a[k->a_propped];
  }
  /* the lift's own rhythm is her stride's clearance */
  {
    const double beat = psi[k->lift];

    tone[0] = sway[k->lift] * sin(beat);
    tone[1] = sway[k->lift] * (turns * cos(beat));
    tone[2] = sway[k->lift] * (((-turns) * turns) * sin(beat));
  }
  for (int m = 0; m < n; m++)
  {
    s.ref[m] = out[k->o_ref + m] - npeye(m, k->lift) * tone[0];
    s.rate[m] = out[k->o_rate + m] - npeye(m, k->lift) * tone[1];
    s.acc[m] = out[k->o_acc + m] - npeye(m, k->lift) * tone[2];
  }
  for (int b = 0; b < 2; b++)
  {
    up[b] = k->limbs[b] * s.ref[k->lift];
  }
  /* her capture point past the soles that stood steps her; still, past those not lifting */
  still = 1.0 - npclip(sqrt(npdot(sway, 1, sway, 1, n)) / k->tiny, 0.0, 1.0);
  edge0[0] = 0.0 * a[k->a_edge];
  edge0[1] = 0.0 * a[k->a_edge + 1];
  wbc_imp_boxes(k, flat, edge0, tilt, lo, hi);
  {
    const double out_of[2] = {k->big * (1.0 - stood[0]), k->big * (1.0 - stood[1])};
    double       l2[2], h2[2], lifting[2], past[2];

    for (int b = 0; b < 2; b++)
    {
      lifting[b] = out_of[b] * npmax(1.0 - still, npmax(a[k->a_swings + b], (double)(up[b] > k->lift_eps)));
    }
    for (int j = 0; j < 2; j++)
    {
      l2[j] = npmin(lo[0][j] + out_of[0], lo[1][j] + out_of[1]);
      h2[j] = npmax(hi[0][j] - out_of[0], hi[1][j] - out_of[1]);
    }
    wbc_imp_spanned(l2, h2, plo, phig, propped, slo, shi);
    for (int j = 0; j < 2; j++)
    {
      beyond[j] = xi[j] - npclip(xi[j], slo[j], shi[j]);
      l2[j] = npmin(lo[0][j] + lifting[0], lo[1][j] + lifting[1]);
      h2[j] = npmax(hi[0][j] - lifting[0], hi[1][j] - lifting[1]);
    }
    wbc_imp_spanned(l2, h2, plo, phig, propped, slo, shi);
    for (int j = 0; j < 2; j++)
    {
      past[j] = xi[j] - npclip(xi[j], slo[j], shi[j]);
    }
    smax = npmax(stood[0], stood[1]);
    a[k->a_need] = (((1.0 - propped) * smax)
                    * npclip(sqrt(npdot(past, 1, past, 1, 2)) / k->margin_m, 0.0, 1.0))
                   * npclip(out[k->o_held], 0.0, 1.0);
  }
  wbc_imp_beat(k, sway[k->drive], psi[k->drive], sway[k->lateral], psi[k->lateral], turns, a + k->a_swings,
               a[k->a_need], a + k->a_lead, &stepping, &way, &swung, sweep, phi, on, off, u, flying);
  a[k->a_swings] = off[0];
  a[k->a_swings + 1] = off[1];
  for (int m = 0; m < n; m++)
  {
    s.kp[m] = out[k->o_ks + m] / fr[k->f_mass + m];
    s.kd[m] = out[k->o_ds + m] / fr[k->f_mass + m];
  }
  {
    const double counted = (double)(((0.0 + stood[0]) + stood[1]) < 1.5);
    const double sw[2] = {stood[0] * counted, stood[1] * counted};
    double       past[2];

    for (int b = 0; b < 2; b++)
    {
      past[b] = npmax(k->limbs[b] * beyond[1], 0.0);
      aside[b] = smax * npmax(past[b], still * fabs(beyond[0]));
    }
    a[k->a_outside] = npdot(sw, 1, past, 1, 2);
  }
  kpf[0] = s.kp[k->free_by[0]];
  kpf[1] = s.kp[k->free_by[1]];
  kdf[0] = s.kd[k->free_by[0]];
  kdf[1] = s.kd[k->free_by[1]];
  wbc_imp_hurried(k, aside, om, u, swung, kpf, hurry);
  for (int b = 0; b < 2; b++)
  {
    hurry[b] = 1.0 + flying[b] * (hurry[b] - 1.0);
    a[k->a_lead + b] = on[b] * (a[k->a_lead + b] + (fr[k->f_dt] * swung) * (hurry[b] - 1.0));
    limbs[b] = npmax(stepping, off[b]);
  }
  cruise = wbc_imp_cruises(k, way, sway[k->drive], turns, a[k->a_going]);
  lat_dc = stepping * (s.ref[k->lateral] - sway[k->lateral] * sin(psi[k->lateral]));
  across = 2.5 * lat_dc;
  /* still, her head's speed past her centre of mass's, shrunk where it is small, and her
     height's spring lifted by what her speed off her rhythm's asks */
  {
    const double dv2 = pow((v[0] - cruise) - k->half_m * s.rate[k->sagittal], 2.0)
                       + pow((v[1] - across) - fr[k->f_w0] * s.rate[k->lateral], 2.0);
    const double lift = npclip(((0.5 * dv2) * ((dv2 * dv2) / ((dv2 * dv2) + 0.02))) / (k->g * k->axial_m), 0.0, 0.25);

    dvh[0] = (vh[0] - v[0]) - (k->half_m * k->pitch_rad) * s.rate[k->pitch];
    dvh[1] = vh[1] - v[1];
    dvh[2] = vh[2] - v[2];
    for (int i = 0; i < 3; i++)
    {
      const double sq = dvh[i] * dvh[i], dvh4 = sq * sq;

      dvh[i] = ((((0.5 * still) * (1.0 - propped)) * dvh[i]) * dvh4) / (dvh4 + 0.02);
    }
    for (int m = 0; m < n; m++)
    {
      s.ref[m] = s.ref[m] + ((npeye(m, k->axial) * smax) * still) * lift;
    }
  }
  {
    const double zl = z[k->lateral];
    const double ts = ((v[0] - cruise) + dvh[0]) / k->half_m - zd[k->sagittal];
    const double tl = (v[1] - across) / fr[k->f_w0] - zd[k->lateral], tl2 = dvh[1] / fr[k->f_w0];

    for (int m = 0; m < n; m++)
    {
      z[m] = z[m] - (propped * npeye(m, k->lateral)) * zl;
      zd[m] = ((zd[m] + npeye(m, k->sagittal) * ts) + (npmax(stepping, propped) * npeye(m, k->lateral)) * tl)
              + npeye(m, k->lateral) * tl2;
    }
  }
  for (int m = 0; m < n; m++)
  {
    zdd[m] = (s.acc[m] + s.kp[m] * (s.ref[m] - z[m])) + s.kd[m] * (s.rate[m] - zd[m]);
    for (int i = 0; i < 6; i++)
    {
      s.hands[6 * m + i] = k->hands[6 * m + i] * fr[k->f_unit + m];
    }
  }
  /* her limbs' places as her modes ask them; their sum's acceleration is her centre of mass's */
  npein(n, s.hands, zdd, e);
  both[0] = ((0.0 + e[0]) + e[3]) / 2.0;
  both[1] = ((0.0 + e[1]) + e[4]) / 2.0;
  swaying = (fabs((s.ref[k->lateral] - lat_dc) - z[k->lateral]) < k->on[0])
            && (fabs(s.rate[k->lateral] - zd[k->lateral]) < k->on[1]);
  npmmt(y + k->y_hip, 2, rot, &hip3[0][0]);
  hips[0] = hip3[0][0];
  hips[1] = hip3[1][0];
  l_ask = fr[k->f_reach0] - (k->axial_m * fr[k->f_hips_by]) * npmax(s.ref[ax], 0.0);
  short_h = propped * npmax((((0.0 + (hip3[0][2] - floors[0])) + (hip3[1][2] - floors[1])) / 2.0) - (k->ankle_h + l_ask), 0.0);
  for (int b = 0; b < 2; b++)
  {
    up_h[b] = up[b] + short_h / k->clear_m;
  }
  {
    const double beat3[3][2] = {{sweep[0], sweep[1]}, {off[0], off[1]}, {flying[0], flying[1]}};
    const double bears3[3][2] = {{y[k->y_load], y[k->y_load + 1]},
                                 {y[k->y_lowest] - floors[0], y[k->y_lowest + 1] - floors[1]},
                                 {y[k->y_lowest] - fr[k->f_floor], y[k->y_lowest + 1] - fr[k->f_floor]}};
    const double prop[2][2] = {{plo[0], plo[1]}, {phig[0], phig[1]}};

    zmp[0] = c[0] + both[0] / om2;
    zmp[1] = c[1] + both[1] / om2;
    wbc_imp_stands(k, zmp, xi, low, high, om, out[k->o_held], up_h, swaying, beat3, bears3, y[k->y_vcom + 1],
                   y[k->y_prop] * fr[k->f_weight], stood, prop, propped, stance);
  }
  {
    /* her legs let go while what bears her off her soles holds her and her text tells what bears her and nothing
       of them, or a leg lifts (machine.impedance._let_go) */
    double said = 0.0, gone;

    for (int i = 1; i < 5; i++)
    {
      said = npmax(said, (double)(out[k->o_manifolded + i] > 0.0));
    }
    gone = propped * (said * (double)(out[k->o_manifolded] <= 0.0));

    for (int b = 0; b < 2; b++)
    {
      const double gb = npmax(gone, propped * (double)(up[b] > k->lift_eps));

      stance[b] *= 1.0 - gb;
      a[k->a_let_go + b] = gb;
    }
  }
  grounded = npmax(npmax(stance[0], stance[1]), propped);
  aloft[0] = 1.0 - stance[0];
  aloft[1] = 1.0 - stance[1];
  wbc_imp_shares(k, stance, bears);
  along[0] = om2 * (c[0] - zmp[0]);
  along[1] = om2 * (c[1] - zmp[1]);
  gd = npdot(bears, 1, y + k->y_grip, 1, 2);
  {
    const double by = npmin(1.0, (gd * k->g) / npmax(sqrt(npdot(along, 1, along, 1, 2)), k->tiny));

    along[0] = along[0] * by;
    along[1] = along[1] * by;
  }
  /* striding: where a sole lands, how it is pitched, how a standing one is turned */
  reach = npmin(k->reach_m, gd * (c[2] - fr[k->f_floor]));
  wide = (2.0 * fr[k->f_w0]) * (1.0 + s.ref[k->width]);
  wbc_imp_landings(k, y, fr, a, rot, c, flat, xi, &cruise, om, turns, wide, across, reach, stood, u, flying, meet, top, land, adj,
                   over, &apart);
  wbc_imp_rocks(k, s.ref[k->rock], s.rate[k->rock], s.acc[k->rock], tip, tipping, tipped);
  wbc_imp_folds(k, off, y + k->y_load, fr[k->f_knee] + a[k->a_bend], fold, shorter);
  a[k->a_prefold] = fold[0];
  a[k->a_prefold + 1] = fold[1];
  wbc_imp_heels(k, sr, tilt, tilting, hips, floors, l_ask, shorter, off, tip,
                stance, stepping, a + k->a_heeled, fr[k->f_dt], fr + k->f_ground, rocked, flatten, edge, rounds);
  npmm(&rounds[0][0], rot, &rr[0][0]);
  for (int b = 0; b < 2; b++)
  {
    a[k->a_heeled + b] = rocked[b];
    a[k->a_edge + b] = edge[b];
    for (int j = 0; j < 3; j++)
    {
      a[k->a_settle + 6 * b + j] = flatten[b] * rot[3 + j];
      a[k->a_settle + 6 * b + 3 + j] = rr[b][j];
    }
  }
  /* up as her height's spring asks of her standing legs' length */
  len2[0] = len[0] + adj[0];
  len2[1] = len[1] + adj[1];
  wbc_imp_shortened(k, len2, lrate, fr, stance, axial);
  {
    const double air0 = propped * aloft[0];
    const double air1 = propped * aloft[1];
    const double h_air[2] = {air0 * npmax(y[k->y_lowest] - floors[0], 0.0),
                             air1 * npmax(y[k->y_lowest + 1] - floors[1], 0.0)};
    const double v_air[2] = {air0 * shod[0][2], air1 * shod[1][2]};
    const double za = (axial[0] - npdot(bears, 1, h_air, 1, 2) / (k->axial_m * fr[k->f_hips_by])) - z[ax];
    const double zda = (axial[1] - (npdot(bears, 1, v_air, 1, 2) + npmax(stance[0], stance[1]) * dvh[2])
                                   / (k->axial_m * fr[k->f_hips_by])) - zd[ax];

    for (int m = 0; m < n; m++)
    {
      z[m] += npeye(m, ax) * za;
      zd[m] += npeye(m, ax) * zda;
    }
  }
  wbc_imp_ducked(out + k->o_ducked, a + k->a_ducking, stance, stood, fr[k->f_dt], duck);
  pressed = (s.acc[ax] + s.kp[ax] * (((s.ref[ax] + duck[0] / k->axial_m) - short_h / (k->axial_m * fr[k->f_hips_by])) - z[ax]))
            + s.kd[ax] * ((s.rate[ax] + duck[1] / k->axial_m) - zd[ax]);
  for (int b = 0; b < 2; b++)
  {
    const double la = npmv2(leg[b], along);
    double       sq[3];

    for (int j = 0; j < 3; j++)
    {
      sq[j] = (v[j] - shod[b][j]) * (v[j] - shod[b][j]);
    }
    risen[b] = (((lrate[b] * lrate[b] - (0.0 + npsum(sq, 3))) - npmax(la, (1.0 - propped) * la))
                - ((len[b] * k->axial_m) * fr[k->f_hips_by]) * pressed)
               / npmax(leg[b][2], k->low_leg * fr[k->f_reach0]);
  }
  smax = npmax(stance[0], stance[1]);
  lifted = ((smax * npdot(bears, 1, risen, 1, 2)) / fr[k->f_hips_by]) - ((1.0 - smax) * k->axial_m) * pressed;
  com[0] = grounded * along[0] + (1.0 - grounded) * k->falls[0];
  com[1] = grounded * along[1] + (1.0 - grounded) * k->falls[1];
  com[2] = grounded * npclip(lifted, -k->g, k->throw_m_s2) + (1.0 - grounded) * k->falls[2];
  wbc_imp_lates(k, phi, turns, stepping, late);
  /* what a sole met her swings clear, for a turn of her rhythm */
  a[k->a_raise] -= ((fr[k->f_dt] * turns) / (2.0 * PI)) * a[k->a_raise];
  for (int b = 0; b < 2; b++)
  {
    lift2[b] = npmax(k->clear_m * (1.0 + fabs(sway[k->lift])) + a[k->a_raise], top[b] + 2.0 * k->clear_m) * (1.0 - propped)
               + k->clear_m * propped;
  }
  th[0] = swung * hurry[0];
  th[1] = swung * hurry[1];
  wbc_imp_clears(k, u, flying, th, lift2, tip, tipping, tilt, late, land, clr);
  for (int r = 0; r < 3; r++)
  {
    for (int b = 0; b < 2; b++)
    {
      clear[r][b] = aloft[b] * (clr[r][b] + npeye(r, 0) * short_h);
    }
  }
  wbc_imp_swings(k, y, fr, v, om, shod, floors, rise, &s, clear, stance, cruise, grounded, propped, loose, frame);
  npmm(&loose[0][0], &frame[0][0], &ff[0][0]);
  for (int j = 0; j < 3; j++)
  {
    cm[j] = (1.0 - grounded * k->between[j]) * com[j];
  }
  for (int j = 0; j < 3; j++)
  {
    gc[j] = npdot(cm, 1, rot + j, 3, 3);
  }
  th[0] = turns * hurry[0];
  th[1] = turns * hurry[1];
  wbc_imp_guarded(k, meet, over, flat, apart, held2);
  wbc_imp_goes(k, held2, flat, shod, v, u, th, kpf, kdf, g2);
  for (int b = 0; b < 2; b++)
  {
    cat[b][0] = g2[b][0];
    cat[b][1] = g2[b][1];
    cat[b][2] = loose[b][2];
  }
  npmm(&cat[0][0], rot, &gg[0][0]);
  for (int b = 0; b < 2; b++)
  {
    const double strides = limbs[b] * grounded;

    for (int j = 0; j < 3; j++)
    {
      a[k->a_sole_acc + 3 * b + j] = (1.0 - strides) * (ff[b][j] + gc[j]) + strides * gg[b][j];
    }
  }
  wbc_imp_rested(k, &s, out[k->o_held], grounded, aloft, limbs, floors, y, shod, rot, a);
  /* its pitch on the rock's own spring, from the floor's under the soles that stand */
  wbc_imp_pitched(k, fr + k->f_ground, pg2);
  pg = npdot(bears, 1, pg2, 1, 2);
  for (int b = 0; b < 2; b++)
  {
    a[k->a_tipped + b] = tilt[b] + ((aloft[b] * ((tipped[b] + s.kp[k->rock] * ((tip[b] + pg) - tilt[b]))
                                                  + s.kd[k->rock] * (tipping[b] - tilting[b])))
                                     + k->turn_kd[1] * tilting[b]) / k->turn_kd[0];
    mvs[b] = npmv3(y + k->y_vsole + 3 * b, rot);
  }
  a[k->a_going] += (fr[k->f_dt] / k->going_s) * (((0.0 + mvs[0]) + mvs[1]) / 2.0 - a[k->a_going]);
  a[k->a_stance] = stance[0];
  a[k->a_stance + 1] = stance[1];
  for (int j = 0; j < 3; j++)
  {
    a[k->a_com_acc + j] = npdot(com, 1, rot + j, 3, 3);
  }
  a[k->a_strides] = stepping;
  a[k->a_borne] = propped;
  folds[0] = aloft[0] * (1.0 - limbs[0]);
  folds[1] = aloft[1] * (1.0 - limbs[1]);
  poses(k, out, y, fr, a, z, &s, turn, spun, up_h, folds, still);
}
