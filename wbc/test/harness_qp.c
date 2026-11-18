/** harness_qp.c - A flat C API over wbc_qp.c for test_qp.py through ctypes, its workspace laid once. */
#include "wbc_qp.h"
#include "wbc_np.h"

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

static double qp_work[WBC_QP_WORK(WBC_QP_N, WBC_QP_M)];
/** The QR's workspace: a (m, q) copy, a run of m and q taus, at the QP's largest m. */
static double qr_work[WBC_QP_N * WBC_QP_N + 2 * WBC_QP_N];

API int wbh_qp_solve(const wbc_blas_t *l, const double *H, const double *g, const double *A, const double *b, int n,
                     int m, const int *warm, int nwarm, int most, double *x, int *act, int *nact, double *u)
{
  return wbc_qp_solve(l, H, g, A, b, n, m, warm, nwarm, most, x, act, nact, u, qp_work);
}

/** wbc_qr on a C-ordered (m, q) `a`: Q (m, m) and R (m, q), 0 or 1. */
API int wbh_qr(const wbc_blas_t *l, const double *a, int m, int q, double *Q, double *R)
{
  const wbc_view_t view = {a, q, 1};

  return wbc_qr(l, view, m, q, Q, R, qr_work, (int)(sizeof qr_work / sizeof qr_work[0]));
}

/** numpy's pairwise sum of n doubles (wbc_np.h's npsum). */
API double wbh_npsum(const double *a, int n)
{
  return npsum(a, n);
}
