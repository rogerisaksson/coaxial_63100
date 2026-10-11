/** harness.c - aero/'s sizes and a row of its memory, for the host's ctypes (tools/cores/aero.py). */
#include "aero.h"

#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

/** sizeof the memory, the payload, the frame, a reading, an answer. */
API unsigned aeh_size(int which)
{
  const unsigned sizes[] = {sizeof(aero_t), sizeof(aero_payload_t), sizeof(aero_params_t),
                            sizeof(aero_in_t), sizeof(aero_out_t)};

  return sizes[which];
}

/** A row of the modes as blended (AERO_TARGET ..) - AERO_ROWS: the targets with the
    oscillator on them -, the rate modes' sums after them. */
API void aeh_row(const aero_t *s, int row, float out[AERO_MODES + AERO_AXES])
{
  const float *rows[AERO_ROWS + 1U] = {s->now[0], s->now[1], s->now[2], s->now[3], s->now[4],
                                       s->ask};

  memcpy(out, rows[row], AERO_MODES * sizeof(float));
  memcpy(out + AERO_MODES, s->wound, AERO_AXES * sizeof(float));
}
