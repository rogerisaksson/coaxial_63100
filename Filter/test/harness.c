/**
  ******************************************************************************
  * @file    harness.c
  * @brief   A flat C API over Filter/, so test_filter_core.py can run the
  *          real chain on the host through ctypes and compare it against a
  *          reference written in Python.
  *
  * Built by the Python suite with the host gcc, never by the firmware build.
  * Test scaffolding; it must not appear in the root CMakeLists.
  ******************************************************************************
  */
#include "filter.h"

#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

typedef struct
{
  filter_design_t  design;
  filter_channel_t channel;
} rig_t;


API rig_t *flt_new(void)
{
  rig_t *r = (rig_t *)calloc(1U, sizeof(rig_t));

  if (r != NULL)
  {
    filter_pass_through(&r->design);
    filter_reset(&r->channel);
  }
  return r;
}


API void flt_free(rig_t *r)
{
  free(r);
}


/** The design as the host holds it: boxcar, decimate, then five floats per
  * section in b0 b1 b2 a1 a2 order - the order coaxial/bessel.py emits. */
API int flt_design(rig_t *r, uint16_t boxcar, uint16_t decimate,
                   uint8_t sections, const float *coeffs)
{
  if ((r == NULL) || (sections > FILTER_MAX_SECTIONS))
  {
    return 0;
  }

  r->design.boxcar = boxcar;
  r->design.decimate = decimate;
  r->design.sections = sections;

  for (uint8_t i = 0U; i < sections; i++)
  {
    r->design.section[i].b0 = coeffs[(i * 5U) + 0U];
    r->design.section[i].b1 = coeffs[(i * 5U) + 1U];
    r->design.section[i].b2 = coeffs[(i * 5U) + 2U];
    r->design.section[i].a1 = coeffs[(i * 5U) + 3U];
    r->design.section[i].a2 = coeffs[(i * 5U) + 4U];
  }

  filter_reset(&r->channel);
  return filter_valid(&r->design) ? 1 : 0;
}


API void flt_reset(rig_t *r)
{
  if (r != NULL)
  {
    filter_reset(&r->channel);
  }
}


API uint32_t flt_ratio(const rig_t *r)
{
  return (r == NULL) ? 1U : filter_ratio(&r->design);
}


/**
  * @brief  Push `n` samples and collect whatever came out.
  * @return how many output samples were written to `out`.
  */
API uint32_t flt_run(rig_t *r, const int32_t *in, uint32_t n, float *out)
{
  uint32_t made = 0U;

  if ((r == NULL) || (in == NULL) || (out == NULL))
  {
    return 0U;
  }

  for (uint32_t i = 0U; i < n; i++)
  {
    float y = 0.0f;

    if (filter_push(&r->design, &r->channel, in[i], &y))
    {
      out[made++] = y;
    }
  }
  return made;
}
