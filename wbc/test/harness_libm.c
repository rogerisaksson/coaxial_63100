/** harness_libm.c - wbc_libm's functions for the host: its arrays as wbc_libm.c lays them, and each scalar function
    as another core inlines it, so that test_libm holds both to the Python twin's bits and the benches time them. */
#include "wbc_libm_arc.h"

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

API void wbh_libm_sin(const double *x, double *y, int32_t n)
{
  wbc_libm_sin(x, y, n);
}

API void wbh_libm_cos(const double *x, double *y, int32_t n)
{
  wbc_libm_cos(x, y, n);
}

API void wbh_libm_tan(const double *x, double *y, int32_t n)
{
  wbc_libm_tan(x, y, n);
}

API void wbh_libm_sincos(const double *x, double *s, double *c, int32_t n)
{
  wbc_libm_sincos(x, s, c, n);
}

API void wbh_libm_exp(const double *x, double *y, int32_t n)
{
  wbc_libm_exp(x, y, n);
}

API void wbh_libm_expm1(const double *x, double *y, int32_t n)
{
  wbc_libm_expm1(x, y, n);
}

API void wbh_libm_log(const double *x, double *y, int32_t n)
{
  wbc_libm_log(x, y, n);
}

API void wbh_libm_arctan(const double *x, double *y, int32_t n)
{
  wbc_libm_arctan(x, y, n);
}

API void wbh_libm_arcsin(const double *x, double *y, int32_t n)
{
  wbc_libm_arcsin(x, y, n);
}

API void wbh_libm_arccos(const double *x, double *y, int32_t n)
{
  wbc_libm_arccos(x, y, n);
}

API void wbh_libm_tanh(const double *x, double *y, int32_t n)
{
  wbc_libm_tanh(x, y, n);
}

API void wbh_libm_arctan2(const double *y, const double *x, double *out, int32_t n)
{
  wbc_libm_arctan2(y, x, out, n);
}

API void wbh_libm_hypot(const double *x, const double *y, double *out, int32_t n)
{
  wbc_libm_hypot(x, y, out, n);
}

/** The scalar function `which` (0 sin, 1 cos, 2 tan, 3 exp, 4 expm1, 5 log, 6 arctan, 7 arcsin, 8 arccos, 9 tanh,
    10 arctan2, 11 hypot, 12 sincos's sine, 13 its cosine) of each element, a call at a time as another core inlines
    it. */
API void wbh_libm_scalar(int32_t which, const double *a, const double *b, double *out, int32_t n)
{
  double s, c;

  for (int32_t i = 0; i < n; i++)
  {
    switch (which)
    {
    case 0:
      out[i] = wbc_sin(a[i]);
      break;
    case 1:
      out[i] = wbc_cos(a[i]);
      break;
    case 2:
      out[i] = wbc_tan(a[i]);
      break;
    case 3:
      out[i] = wbc_exp(a[i]);
      break;
    case 4:
      out[i] = wbc_expm1(a[i]);
      break;
    case 5:
      out[i] = wbc_log(a[i]);
      break;
    case 6:
      out[i] = wbc_arctan(a[i]);
      break;
    case 7:
      out[i] = wbc_arcsin(a[i]);
      break;
    case 8:
      out[i] = wbc_arccos(a[i]);
      break;
    case 9:
      out[i] = wbc_tanh(a[i]);
      break;
    case 10:
      out[i] = wbc_arctan2(a[i], b[i]);
      break;
    case 11:
      out[i] = wbc_hypot(a[i], b[i]);
      break;
    case 12:
      wbc_sincos(a[i], &s, &c);
      out[i] = s;
      break;
    default:
      wbc_sincos(a[i], &s, &c);
      out[i] = c;
      break;
    }
  }
}
