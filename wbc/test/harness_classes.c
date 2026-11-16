/** harness_classes.c - A flat C API over wbc_classes.c for tools.cores.classes through ctypes. */
#include "wbc_classes.h"

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
