/** se3_law.c - a bearing point's friction alone, as the core's friction pass takes it on its lanes: no branch */
#include "se3_bits.h"

se3_real_t se3_rubs(se3_real_t rest, se3_real_t slide, se3_real_t fade, se3_real_t speed2)
{
  const se3_real_t pace = ONE + (speed2 * fade);

  return ((slide * pace) + (rest - slide)) / pace;
}

se3_real_t se3_kept(se3_real_t load, se3_real_t speed2, se3_real_t miss2, se3_real_t slide, se3_real_t fall,
                    se3_real_t fade, se3_real_t yield)
{
  return share(load, speed2, miss2, slide, fall, fade, yield);
}
