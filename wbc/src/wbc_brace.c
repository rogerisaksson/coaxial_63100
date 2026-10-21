/** wbc_brace.c - her surety's brace on the solve (wbc_imp.h): machine.motion.scaled and braced on her
    law's arrays - the cervical and shoulder dofs' inertia scaled off the solve, the tuck's torques
    and the cervical blend laid over its answer, her drives' inertia and her legs' stiffness -
    numpy's arithmetic operation for operation (wbc_np.h), its matmuls numpy's own BLAS on the
    gathered submatrices in numpy's memory order (wbc_blas.h). Portable C11, the host's alone. */
#include "wbc_imp.h"
#include "wbc_np.h"

#include <string.h>

void wbc_brace_scaled(const wbc_brace_t *b, const double *M, const double *w_up, double *Ms)
{
  const int n = b->n;

  memcpy(Ms, M, (size_t)(n * n) * sizeof(double));
  for (int j = 0; j < b->ncv; j++)
  {
    const int    d = b->acv[j];
    const double keep = 1.0 - w_up[j];

    for (int i = 0; i < 6; i++)
    {
      Ms[i * n + d] *= keep;
      Ms[d * n + i] *= keep;
    }
  }
}

/** numpy's `M[rows, :][:, rows]` and `M[rows, :6]`: the first a (k, k) copy F-ordered as numpy's
    advanced index on the last axis leaves it, the second a (k, 6) copy C-ordered. */
static void gathered(const double *M, int n, const int *rows, int k, double *sq, double *six)
{
  for (int i = 0; i < k; i++)
  {
    const double *row = M + rows[i] * n;

    for (int j = 0; j < k; j++)
    {
      sq[i + k * j] = row[rows[j]];
    }
    for (int j = 0; j < 6; j++)
    {
      six[i * 6 + j] = row[j];
    }
  }
}

/** numpy's `sq @ x + six @ y6 + h[rows]`, sq F-ordered (k, k), six (k, 6), into `got`. */
static void torques(const wbc_blas_t *l, const double *sq, const double *six, int k, const double *x,
                    const double *y6, const double *h, const int *rows, double *got, double *work)
{
  const wbc_view_t a = {sq, 1, k}, xv = {x, 1, 0}, c = {six, 6, 1}, yv = {y6, 1, 0};
  double          *one = work, *two = work + k;

  wbc_matmul(l, a, xv, one, 1, 0, k, k, 1);
  wbc_matmul(l, c, yv, two, 1, 0, k, 6, 1);
  for (int i = 0; i < k; i++)
  {
    got[i] = (one[i] + two[i]) + h[rows[i]];
  }
}

void wbc_brace_blend(const wbc_brace_t *b, const double *M, const double *Ms, const double *q, const double *v,
                     const double *h, const double *posture, double *qacc, double *tau, unsigned char *over,
                     double sw, double bw, double d_ax, double d_lat, double w_hop, const double *w_up,
                     const double *stance, double *jeff, double *leg_kp)
{
  const int         n = b->n, nd = b->nd, ncv = b->ncv;
  const double      lean[3] = {1.0, d_ax, d_lat};
  const wbc_view_t  lv = {lean, 0, 1}, bv = {b->brace, nd, 1};
  double           *q_brace = b->work, *qdd_br = q_brace + nd, *tau_br = qdd_br + nd, *sq = tau_br + nd;
  double           *six = sq + nd * nd, *scratch = six + nd * 6, *qdd_cv = scratch + 2 * nd, *tau_cv = qdd_cv + ncv;
  double           *sq4 = tau_cv + ncv, *six4 = sq4 + ncv * ncv, *scratch4 = six4 + ncv * 6;
  double            kp_cv[4], kd_cv[4];
  int               at = 0;

  /* the tuck's posture by her surety's direction, its torques through her inertia */
  wbc_matmul(&b->blas, lv, bv, q_brace, 0, 1, 1, 3, nd);
  for (int i = 0; i < nd; i++)
  {
    qdd_br[i] = npclip((3.5 * b->kd0) * (q_brace[i] - q[i]) - (1.8 * b->kd1) * v[b->act[i]], -180.0, 180.0);
  }
  gathered(M, n, b->act, nd, sq, six);
  torques(&b->blas, sq, six, nd, qdd_br, qacc, h, b->act, tau_br, scratch);
  for (int i = 0; i < nd; i++)
  {
    tau_br[i] = npclip(tau_br[i], -b->clamp[i], b->clamp[i]);
  }
  for (int i = 0; i < nd; i++)
  {
    qacc[b->act[i]] = sw * qacc[b->act[i]] + bw * qdd_br[i];
  }
  for (int i = 0; i < nd; i++)
  {
    tau[i] = sw * tau[i] + bw * tau_br[i];
  }
  /* the cervical and shoulder dofs held to their posture as far as w_up */
  kp_cv[0] = 3.2 * b->kd0;
  kp_cv[1] = 3.2 * b->kd0;
  kp_cv[2] = (1.0 + 2.2 * w_hop) * b->kd0;
  kp_cv[3] = (1.0 + 2.2 * w_hop) * b->kd0;
  kd_cv[0] = 1.8 * b->kd1;
  kd_cv[1] = 1.8 * b->kd1;
  kd_cv[2] = (1.0 + 0.8 * w_hop) * b->kd1;
  kd_cv[3] = (1.0 + 0.8 * w_hop) * b->kd1;
  for (int j = 0; j < ncv; j++)
  {
    qdd_cv[j] = npclip(kp_cv[j] * (posture[b->cv[j]] - q[b->cv[j]]) - kd_cv[j] * v[b->acv[j]], -130.0, 130.0);
  }
  gathered(Ms, n, b->acv, ncv, sq4, six4);
  torques(&b->blas, sq4, six4, ncv, qdd_cv, qacc, h, b->acv, tau_cv, scratch4);
  for (int j = 0; j < ncv; j++)
  {
    qacc[b->acv[j]] += w_up[j] * (qdd_cv[j] - qacc[b->acv[j]]);
    tau[b->cv[j]] += w_up[j] * (tau_cv[j] - tau[b->cv[j]]);
    over[b->cv[j]] = (unsigned char)(over[b->cv[j]] && (w_up[j] < 0.5));
  }
  for (int i = 0; i < nd; i++)
  {
    jeff[i] = M[b->act[i] * n + b->act[i]];
  }
  /* each leg's stiffness by its stance */
  for (int leg = 0; leg < b->nleg; leg++)
  {
    const double k = 1.0 + (b->standing_kp - 1.0) * stance[leg];

    for (int j = 0; j < b->leg_len[leg]; j++, at++)
    {
      leg_kp[at] = b->free_kp[b->leg_joints[at]] * k;
    }
  }
}
