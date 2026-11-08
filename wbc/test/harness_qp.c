/** harness_qp.c - A flat C API over wbc_qp.c for test_qp.py through ctypes, its workspace laid once. */
#include "wbc_qp.h"

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

static double qp_work[WBC_QP_WORK(WBC_QP_N, WBC_QP_M)];

API int wbh_qp_solve(const wbc_blas_t *l, const double *H, const double *g, const double *A, const double *b, int n,
                     int m, const int *warm, int nwarm, int most, double *x, int *act, int *nact, double *u)
{
  return wbc_qp_solve(l, H, g, A, b, n, m, warm, nwarm, most, x, act, nact, u, qp_work);
}
