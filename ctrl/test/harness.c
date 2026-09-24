/** harness.c - A flat C API over ctrl/, so test_ctrl_core.py can step the real parts on
    the host through ctypes beside machine.parts. */
#include "ctrl.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

/** The kind whose class is `name`, CTRL_NONE for none. */
API int ctl_kind(const char *name)
{
  for (int k = 1; k < (int)CTRL_KINDS; k++)
  {
    if (strcmp(ctrl_kind_names[k], name) == 0)
    {
      return k;
    }
  }
  return 0;
}

API int ctl_params(int kind)
{
  return ((kind > 0) && (kind < (int)CTRL_KINDS)) ? ctrl_kind_params[kind] : -1;
}

API ctrl_part_t *ctl_part(int kind, const float *params)
{
  ctrl_part_t *part = (ctrl_part_t *)calloc(1U, sizeof(ctrl_part_t));

  if ((part != NULL) && !ctrl_part_set(part, (uint8_t)kind, params))
  {
    free(part);
    part = NULL;
  }
  return part;
}

API ctrl_feedback_t *ctl_feedback(void)
{
  return (ctrl_feedback_t *)calloc(1U, sizeof(ctrl_feedback_t));
}

/** A feedback's slot: 0 prefilter, 1 measure, 2 estimator, 3 regulator. */
API int ctl_slot(ctrl_feedback_t *f, int slot, int kind, const float *params)
{
  ctrl_part_t *slots[4] = { &f->prefilter, &f->measure, &f->estimator, &f->regulator };

  return ((slot >= 0) && (slot < 4)) ? ctrl_part_set(slots[slot], (uint8_t)kind, params) : 0;
}

API void ctl_free(void *p)
{
  free(p);
}

API float ctl_filter(ctrl_part_t *part, float dt, float x)
{
  return ctrl_filter(part, dt, x);
}

API float ctl_estimate(ctrl_part_t *part, float dt, float measured, float command)
{
  return ctrl_estimate(part, dt, measured, command);
}

/** `has_accel` 0: the setpoint's own slope. */
API float ctl_regulate(ctrl_part_t *part, float dt, float setpoint, float measured,
                       float accel, int has_accel, int held)
{
  return ctrl_regulate(part, dt, setpoint, measured, has_accel ? accel : NAN, held != 0);
}

API void ctl_reset(ctrl_part_t *part)
{
  ctrl_part_reset(part);
}

API float ctl_step(ctrl_feedback_t *f, float dt, float setpoint, float measured)
{
  return ctrl_feedback_step(f, dt, setpoint, measured);
}

API void ctl_feedback_reset(ctrl_feedback_t *f)
{
  ctrl_feedback_reset(f);
}

API ctrl_runner_t *ctl_runner(void)
{
  return (ctrl_runner_t *)calloc(1U, sizeof(ctrl_runner_t));
}

/** The runner's feedback, for ctl_slot and ctl_step. */
API ctrl_feedback_t *ctl_runner_feedback(ctrl_runner_t *r)
{
  return &r->f;
}

API int ctl_push(ctrl_runner_t *r, uint16_t ms, float setpoint)
{
  return ctrl_rows_push(r, ms, setpoint) ? 1 : 0;
}

API int ctl_free_rows(ctrl_runner_t *r)
{
  return ctrl_rows_free(r);
}

API float ctl_seconds(ctrl_runner_t *r)
{
  return ctrl_rows_seconds(r);
}

API void ctl_clear(ctrl_runner_t *r)
{
  ctrl_rows_clear(r);
}

API float ctl_tick(ctrl_runner_t *r, float dt, float measured)
{
  return ctrl_runner_step(r, dt, measured);
}

/** played, idle */
API void ctl_counts(ctrl_runner_t *r, uint32_t *out)
{
  out[0] = r->played;
  out[1] = r->idle;
}
