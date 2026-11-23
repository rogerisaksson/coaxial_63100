/** harness_inekf.c - A flat C API over wbc_inekf.c for tools.cores.inekf, through ctypes: the robot's bearing
    points' frames, weights and shares off its edge's array, and the drift its rolling soles add. */
#include "wbc_inekf.h"

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

API int wbh_inekf_size(void)
{
  return (int)sizeof(wbc_inekf_t);
}

API void wbh_inekf_laid(const wbc_inekf_t *k, const double *y, const double *shares, int n, double *local,
                        double *weights, double *sh)
{
  wbc_inekf_laid(k, y, shares, n, local, weights, sh);
}

API double wbh_inekf_rolled(const wbc_inekf_t *k, const double *soles, const double *anchors, const double *sh,
                            const double *on, double cap, double gyro2dt, double *d)
{
  return wbc_inekf_rolled(k, soles, anchors, sh, on, cap, gyro2dt, d);
}
