/** harness.c - a flat C API over se3/ for tools/cores/se3.py: a step and its sums run over, for their pace */
#include "se3.h"

/** `calls` steps from the same pose, each into `out`: what one costs, no interpreter between two. */
void seh_steps(se3_real_t *planes, size_t stride, const se3_cfg_t *cfg, const se3_real_t r[9],
               const se3_real_t t[3], se3_out_t *out, unsigned long calls)
{
  for (unsigned long n = 0UL; n < calls; n++)
  {
    se3_step(planes, stride, cfg, r, t, out);
  }
}

/** `calls` steps under friction from the same pose, the anchors taken with them or not: what a
    bearing body costs a tick. */
void seh_grips(se3_real_t *planes, size_t stride, const se3_cfg_t *cfg,
               const se3_friction_t *friction, const se3_real_t r[9], const se3_real_t t[3],
               unsigned take, se3_out_t *out, unsigned long calls)
{
  for (unsigned long n = 0UL; n < calls; n++)
  {
    se3_grip(planes, stride, cfg, friction, r, t, take, out);
  }
}

/** `calls` returns of the anchors at the same pose: what the contact's memory costs a tick. */
void seh_anchors(se3_real_t *planes, size_t stride, const se3_friction_t *friction,
                 const se3_real_t r[9], const se3_real_t t[3], unsigned long calls)
{
  for (unsigned long n = 0UL; n < calls; n++)
  {
    se3_anchored(planes, stride, friction, r, t);
  }
}

/** `calls` passes of the sums alone: the step's share that grows with its points. */
void seh_sums(se3_real_t *planes, size_t stride, const se3_cfg_t *cfg, const se3_real_t r[9],
              const se3_real_t t[3], se3_out_t *out, unsigned long calls)
{
  for (unsigned long n = 0UL; n < calls; n++)
  {
    se3_summed(planes, stride, cfg->soft, r, t, &out->sums);
  }
}
