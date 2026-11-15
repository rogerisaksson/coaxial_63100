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

/** `calls` passes of the sums alone: the step's share that grows with its points. */
void seh_sums(se3_real_t *planes, size_t stride, const se3_cfg_t *cfg, const se3_real_t r[9],
              const se3_real_t t[3], se3_out_t *out, unsigned long calls)
{
  for (unsigned long n = 0UL; n < calls; n++)
  {
    se3_summed(planes, stride, cfg->soft, r, t, &out->sums);
  }
}
