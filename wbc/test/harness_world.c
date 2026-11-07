/** harness_world.c - her world's steps (wbc_world.c) through ctypes beside harness.c: tools.cores.world's
    calls. */
#include "wbc_world.h"

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

API int wbh_steps_size(void)
{
  return (int)sizeof(wbc_steps_t);
}

API void wbh_world_rated(wbc_steps_t *s, double span)
{
  wbc_world_rated(s, span);
}

API void wbh_world_before(wbc_steps_t *s, double since)
{
  wbc_world_before(s, since);
}

API void wbh_world_after(wbc_steps_t *s)
{
  wbc_world_after(s);
}

API int wbh_world_paired(const wbc_steps_t *s, const unsigned char *a, const unsigned char *b, double below)
{
  return wbc_world_paired(s, a, b, below);
}
