/** harness_classes.c - A flat C API over wbc_classes.c and wbc_wholes.c for tools.cores.classes, and over
    wbc_afford.c for tools.cores.afford, through ctypes. */
#include "wbc_afford.h"
#include "wbc_classes.h"
#include "wbc_wholes.h"

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

API int wbh_classes_of(const wbc_classes_t *k, int n, const double *c, const double *h, const double *R,
                       const double *clear, const int32_t *code, const int32_t *flags, int32_t *cls, int32_t *of,
                       int32_t *whole, unsigned char *scratch)
{
  return wbc_classes_of(k, n, c, h, R, clear, code, flags, cls, of, whole, scratch);
}

API int wbh_classes_size(void)
{
  return (int)sizeof(wbc_classes_t);
}

API long long wbh_classes_scratch(int n)
{
  return (long long)wbc_classes_scratch(n);
}

API int wbh_wholes_grouped(const wbc_classes_t *k, int n, const double *c, const double *h, const double *R,
                           const int32_t *code, const int32_t *flags, const int32_t *whole, int32_t *one,
                           int32_t *cabin, double *blo, double *bhi, unsigned char *member, unsigned char *scratch)
{
  return wbc_wholes_grouped(k, n, c, h, R, code, flags, whole, one, cabin, blo, bhi, member, scratch);
}

API long long wbh_wholes_scratch(int n)
{
  return (long long)wbc_wholes_scratch(n);
}

API int wbh_afford_near(int n, const double *c_now, const double *h_now, const double *R_now, const double *c_was,
                        const double *h_was, const double *R_was, const uint8_t *told, double cell, double root2,
                        uint8_t *near)
{
  return wbc_afford_near(n, c_now, h_now, R_now, c_was, h_was, R_was, told, cell, root2, near);
}

API double wbh_afford_clear(const wbc_blas_t *l, int n, const double *centre, const double *half, const double *R,
                            const uint8_t *other, const double *box, double lo, double hi, double reach, double root2)
{
  return wbc_afford_clear(l, n, centre, half, R, other, box, lo, hi, reach, root2);
}
