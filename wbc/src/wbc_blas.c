/** wbc_blas.c - numpy's own OpenBLAS as numpy calls it (wbc_blas.h): its matmul's dispatch, its
    dot, its linalg's solve and inv, transcribed from numpy 2.5's umath/matmul.c.src, arraytypes'
    DOUBLE_dot and umath_linalg.cpp. Portable C11, the host's alone. */
#include "wbc_blas.h"

#include <math.h>

/** cblas's enums. */
enum
{
  ROW = 101,
  COL = 102,
  NOTRANS = 111,
  TRANS = 112,
  UPPER = 121
};

/** The largest solve laid on the stack. */
#define MOST 128

/** numpy's is_blasable2d, element strides: unit inner, outer at least the row. */
static int blasable(ptrdiff_t s1, ptrdiff_t s2, int d2)
{
  return s2 == 1 && s1 >= d2;
}

/** numpy's matmul_inner_noblas: each entry summed from zero, n in order. */
static void noblas(wbc_view_t a, wbc_view_t b, double *o, ptrdiff_t o_m, ptrdiff_t o_p, int m, int n, int p)
{
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < p; j++)
    {
      double *at = o + i * o_m + j * o_p;

      *at = 0.0;
      for (int k = 0; k < n; k++)
      {
        *at += a.at[i * a.s0 + k * a.s1] * b.at[k * b.s0 + j * b.s1];
      }
    }
  }
}

/** numpy's gemv: a (m, n) at strides s_m, s_n times x, into y. */
static void gemv(const wbc_blas_t *l, const double *a, ptrdiff_t s_m, ptrdiff_t s_n, const double *x, ptrdiff_t incx,
                 double *y, ptrdiff_t incy, int m, int n)
{
  if (blasable(s_m, s_n, n))
  {
    l->gemv(COL, TRANS, n, m, 1.0, a, s_m, x, incx, 0.0, y, incy);
  }
  else
  {
    l->gemv(ROW, TRANS, n, m, 1.0, a, s_n, x, incx, 0.0, y, incy);
  }
}

/** numpy's matmul_matrixmatrix: gemm, or syrk for a matrix times its own transpose. */
static void matrixmatrix(const wbc_blas_t *l, wbc_view_t a, wbc_view_t b, double *o, ptrdiff_t ldc, int m, int n,
                         int p)
{
  const int       ta = blasable(a.s0, a.s1, n) ? NOTRANS : TRANS, tb = blasable(b.s0, b.s1, p) ? NOTRANS : TRANS;
  const ptrdiff_t lda = ta == NOTRANS ? a.s0 : a.s1, ldb = tb == NOTRANS ? b.s0 : b.s1;

  if (a.at == b.at && m == p && a.s0 == b.s1 && a.s1 == b.s0 && ta != tb)
  {
    l->syrk(ROW, UPPER, ta, p, n, 1.0, a.at, ta == NOTRANS ? lda : ldb, 0.0, o, ldc);
    for (int i = 0; i < p; i++)
    {
      for (int j = i + 1; j < p; j++)
      {
        o[j * ldc + i] = o[i * ldc + j];
      }
    }
  }
  else
  {
    l->gemm(ROW, ta, tb, m, p, n, 1.0, a.at, lda, b.at, ldb, 0.0, o, ldc);
  }
}

void wbc_mv(const wbc_blas_t *l, const double *m, int r, int c, const double *v, double *o)
{
  const wbc_view_t mv = {m, c, 1}, vv = {v, 1, 1};

  wbc_matmul(l, mv, vv, o, 1, 1, r, c, 1);
}

void wbc_mm3(const wbc_blas_t *l, const double *a, int r, const double *b, int bt, double *o)
{
  const wbc_view_t av = {a, 3, 1}, bv = {b, bt != 0 ? 1 : 3, bt != 0 ? 3 : 1};

  wbc_matmul(l, av, bv, o, 3, 1, r, 3, 3);
}

double wbc_dot(const wbc_blas_t *l, const double *a, ptrdiff_t sa, const double *b, ptrdiff_t sb, int n)
{
  double sum = 0.0;

  if (sa > 0 && sb > 0)
  {
    if (n > 0)
    {
      sum += l->dot(n, a, sa, b, sb);
    }
    return sum;
  }
  for (int i = 0; i < n; i++)
  {
    sum += a[i * sa] * b[i * sb];
  }
  return sum;
}

void wbc_matmul(const wbc_blas_t *l, wbc_view_t a, wbc_view_t b, double *o, ptrdiff_t o_m, ptrdiff_t o_p, int m,
                int n, int p)
{
  const int i1 = blasable(a.s0, a.s1, n) || blasable(a.s1, a.s0, m);
  const int i2 = blasable(b.s0, b.s1, p) || blasable(b.s1, b.s0, n);

  if (m == 0 || n == 0 || p == 0)
  {
    noblas(a, b, o, o_m, o_p, m, n, p);
  }
  else if (m == 1 || n == 1 || p == 1)
  {
    if (m == 1 && p == 1)
    {
      *o = wbc_dot(l, a.at, a.s1, b.at, b.s0, n);
    }
    else if (n == 1 && (p == 1 || m == 1))
    {
      noblas(a, b, o, o_m, o_p, m, n, p);
    }
    else if (m == 1 && i2 && blasable(a.s1, 1, 1))
    {
      gemv(l, b.at, b.s1, b.s0, a.at, a.s1, o, o_p, p, n);
    }
    else if (p == 1 && i1 && blasable(b.s0, 1, 1))
    {
      gemv(l, a.at, a.s0, a.s1, b.at, b.s0, o, o_m, m, n);
    }
    else
    {
      noblas(a, b, o, o_m, o_p, m, n, p);
    }
  }
  else if (i1 && i2 && blasable(o_m, o_p, p))
  {
    matrixmatrix(l, a, b, o, o_m, m, n, p);
  }
  else if (i1 && i2 && blasable(o_p, o_m, m))
  {
    const wbc_view_t bt = {b.at, b.s1, b.s0}, at = {a.at, a.s1, a.s0};

    matrixmatrix(l, bt, at, o, o_p, p, n, m);
  }
  else
  {
    noblas(a, b, o, o_m, o_p, m, n, p);
  }
}

/** dgesv on Fortran-ordered copies of a and b, as umath_linalg's linearize lays them. */
static int gesv(const wbc_blas_t *l, wbc_view_t a, wbc_view_t b, int n, int nrhs, double *x, double *work)
{
  double *af = work, *bf = work + n * n;
  int64_t nn = n, nr = nrhs, info = 0, ipiv[MOST];

  if (n > MOST)
  {
    return 1;
  }
  for (int j = 0; j < n; j++)
  {
    for (int i = 0; i < n; i++)
    {
      af[i + j * n] = a.at[i * a.s0 + j * a.s1];
    }
  }
  for (int j = 0; j < nrhs; j++)
  {
    for (int i = 0; i < n; i++)
    {
      bf[i + j * n] = b.at == NULL ? (double)(i == j) : b.at[i * b.s0 + j * b.s1];
    }
  }
  l->gesv(&nn, &nr, af, &nn, ipiv, bf, &nn, &info);
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < nrhs; j++)
    {
      x[i * nrhs + j] = info == 0 ? bf[i + j * n] : NAN;
    }
  }
  return info != 0;
}

int wbc_solve(const wbc_blas_t *l, wbc_view_t a, wbc_view_t b, int n, int nrhs, double *x, double *work)
{
  return gesv(l, a, b, n, nrhs, x, work);
}

int wbc_inv(const wbc_blas_t *l, wbc_view_t a, int n, double *x, double *work)
{
  const wbc_view_t eye = {NULL, 0, 0};

  return gesv(l, a, eye, n, n, x, work);
}

/** a (m, q) view into Fortran order at `af`, m rows a column. */
static void fortran(wbc_view_t a, int m, int q, double *af)
{
  for (int j = 0; j < q; j++)
  {
    for (int i = 0; i < m; i++)
    {
      af[i + j * m] = a.at[i * a.s0 + j * a.s1];
    }
  }
}

int wbc_potrf(const wbc_blas_t *l, wbc_view_t a, int n, double *x, double *work)
{
  /* umath_linalg's cholesky_lo: dpotrf on the Fortran-ordered copy, its upper triangle zeroed */
  const char uplo = 'L';
  int64_t    nn = n, info = 0;

  fortran(a, n, n, work);
  l->potrf(&uplo, &nn, work, &nn, &info);
  for (int i = 0; i < n; i++)
  {
    for (int j = 0; j < n; j++)
    {
      x[i * n + j] = info != 0 ? NAN : (j > i ? 0.0 : work[i + j * n]);
    }
  }
  return info != 0;
}

/** dgeqrf or dorgqr (`k` under 0) on `af` (m, cols) with its workspace asked first, as umath_linalg's init_geqrf and
    init_gqr_common size it: 0, or 1 failed or `avail` doubles too few. */
static int lapack_qr(const wbc_blas_t *l, int m, int cols, int64_t k, double *af, double *tau, double *lw, int64_t avail)
{
  int64_t mm = m, nn = cols, lwork = -1, info = 0;
  double  asked = 0.0;

  if (k < 0)
  {
    l->geqrf(&mm, &nn, af, &mm, tau, &asked, &lwork, &info);
  }
  else
  {
    l->orgqr(&mm, &nn, &k, af, &mm, tau, &asked, &lwork, &info);
  }
  lwork = (int64_t)asked;
  lwork = lwork > nn ? lwork : (nn > 1 ? nn : 1);
  if (info != 0 || lwork > avail)
  {
    return 1;
  }
  if (k < 0)
  {
    l->geqrf(&mm, &nn, af, &mm, tau, lw, &lwork, &info);
  }
  else
  {
    l->orgqr(&mm, &nn, &k, af, &mm, tau, lw, &lwork, &info);
  }
  return info != 0;
}

int wbc_qr(const wbc_blas_t *l, wbc_view_t a, int m, int q, double *Q, double *R, double *work, int nwork)
{
  /* numpy's qr: qr_r_raw's dgeqrf on the Fortran-ordered copy, R its upper triangle; qr_complete's (qr_reduced's at
     q = m) dorgqr on the m by m copy of the reflectors, its first q columns */
  double       *af = work, *qf = af + m * q, *tau = qf + m * m, *lw = tau + q;
  const int64_t avail = (int64_t)nwork - (int64_t)(m * q + m * m + q);

  if (avail < 1 || q > m)
  {
    return 1;
  }
  fortran(a, m, q, af);
  if (lapack_qr(l, m, q, -1, af, tau, lw, avail))
  {
    return 1;
  }
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < q; j++)
    {
      R[i * q + j] = j >= i ? af[i + j * m] : 0.0;
    }
  }
  for (int j = 0; j < m; j++)
  {
    for (int i = 0; i < m; i++)
    {
      qf[i + j * m] = j < q ? af[i + j * m] : 0.0;
    }
  }
  if (lapack_qr(l, m, m, q, qf, tau, lw, avail))
  {
    return 1;
  }
  for (int i = 0; i < m; i++)
  {
    for (int j = 0; j < m; j++)
    {
      Q[i * m + j] = qf[i + j * m];
    }
  }
  return 0;
}
