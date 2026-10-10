/** harness.c - A flat C API over wbc/, so test_wbc_core.py can run the real chain on the host
    through ctypes beside MuJoCo. */
#include "wbc.h"

#include <string.h>
#include <time.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

static wbc_body_t body;

static void frame_of(const double *flat, wbc_frame_t *f)
{
  memcpy(f->r, flat, 9U * sizeof(double));
  memcpy(f->p, flat + 9, 3U * sizeof(double));
}

API int wbh_links(void)
{
  return WBC_LINKS;
}

API int wbh_n(void)
{
  return WBC_N;
}

API const char *wbh_name(int link)
{
  return wbc_name[link];
}

API int wbh_parent(int link)
{
  return wbc_parent[link];
}

/** Every link's frame in the world, 12 doubles each, from the pelvis's and the hinges' rad. */
API void wbh_pose(const double *base, const double *q, double *t)
{
  wbc_frame_t f;

  frame_of(base, &f);
  wbc_body_pose(&body, &f, q);
  for (int i = 0; i < WBC_LINKS; i++)
  {
    memcpy(t + 12 * i, body.t[i].r, 9U * sizeof(double));
    memcpy(t + 12 * i + 9, body.t[i].p, 3U * sizeof(double));
  }
}

API void wbh_mass(double *m)
{
  wbc_body_mass(&body);
  memcpy(m, body.m, sizeof(body.m));
}

API void wbh_bias(const double *u, double *h)
{
  wbc_body_bias(&body, u);
  memcpy(h, body.h, sizeof(body.h));
}

API void wbh_point(int link, const double *r, double *out)
{
  wbc_body_point(&body, link, r, out);
}

API void wbh_jacobian(int link, const double *r, double *j)
{
  wbc_body_jacobian(&body, link, r, (double (*)[WBC_N])j);
}

/** Seconds a step: the pose, the mass matrix, the bias and two links' Jacobians, over reps. */
API double wbh_seconds(const double *base, const double *q, const double *u, int a, int b,
                       int reps)
{
  static double j[6][WBC_N];
  const double  o[3] = {0.0, 0.0, 0.0};
  wbc_frame_t   f;
  clock_t       t0;

  frame_of(base, &f);
  t0 = clock();
  for (int k = 0; k < reps; k++)
  {
    wbc_body_pose(&body, &f, q);
    wbc_body_mass(&body);
    wbc_body_bias(&body, u);
    wbc_body_jacobian(&body, a, o, j);
    wbc_body_jacobian(&body, b, o, j);
  }
  return (double)(clock() - t0) / (double)CLOCKS_PER_SEC / (double)reps;
}
