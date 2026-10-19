/** wbc_blas.h - numpy's own OpenBLAS as numpy calls it (wbc_blas.c): its matmul's choice of
    gemm, syrk, gemv, dot or its own loop by shapes and strides, its linalg's solve and inv on
    Fortran-ordered copies through dgesv - so a product or a solve here is numpy's bit for bit.
    The functions are numpy's, handed over by the host (ILP64). Portable C11, the host's alone. */
#ifndef WBC_BLAS_H
#define WBC_BLAS_H

#include <stddef.h>
#include <stdint.h>

/** numpy's OpenBLAS: cblas_dgemm, cblas_dsyrk, cblas_dgemv, cblas_ddot and LAPACK's dgesv. */
typedef struct
{
  void   (*gemm)(int order, int ta, int tb, int64_t m, int64_t n, int64_t k, double alpha, const double *a,
                 int64_t lda, const double *b, int64_t ldb, double beta, double *c, int64_t ldc);
  void   (*syrk)(int order, int uplo, int trans, int64_t n, int64_t k, double alpha, const double *a, int64_t lda,
                 double beta, double *c, int64_t ldc);
  void   (*gemv)(int order, int trans, int64_t m, int64_t n, double alpha, const double *a, int64_t lda,
                 const double *x, int64_t incx, double beta, double *y, int64_t incy);
  double (*dot)(int64_t n, const double *x, int64_t incx, const double *y, int64_t incy);
  void   (*gesv)(const int64_t *n, const int64_t *nrhs, double *a, const int64_t *lda, int64_t *ipiv, double *b,
                 const int64_t *ldb, int64_t *info);
} wbc_blas_t;

/** An array's view: its first element and its strides, elements. */
typedef struct
{
  const double *at;
  ptrdiff_t     s0, s1;
} wbc_view_t;

/** numpy's `a @ b`, a (m, n) and b (n, p) views, into o (m, p) at o's strides. A vector is a
    view of one row or one column: numpy's 1-d operand, its length n. */
void wbc_matmul(const wbc_blas_t *l, wbc_view_t a, wbc_view_t b, double *o, ptrdiff_t o_m, ptrdiff_t o_p, int m,
                int n, int p);

/** numpy's 1-d `a @ b`, `n` long, at strides sa and sb. */
double wbc_dot(const wbc_blas_t *l, const double *a, ptrdiff_t sa, const double *b, ptrdiff_t sb, int n);

/** numpy's linalg.solve(a, b), a (n, n) and b (n, nrhs) views, into x (n, nrhs) C-ordered: 0, or
    1 where numpy raises LinAlgError (x then nan). `work` 2 n (n + nrhs) doubles. */
int wbc_solve(const wbc_blas_t *l, wbc_view_t a, wbc_view_t b, int n, int nrhs, double *x, double *work);

/** numpy's linalg.inv(a), a (n, n), into x (n, n) C-ordered: 0, or 1 where numpy raises. */
int wbc_inv(const wbc_blas_t *l, wbc_view_t a, int n, double *x, double *work);

#endif
