/**
  ******************************************************************************
  * @file    board_log.c
  * @brief   One ring for every measurement this board takes, drained over the
  *          wire in bursts.
  *
  * The point is buffered reads. A host asking one question per sample gets
  * one sample per round trip, and a round trip is milliseconds - measured, a
  * 53-byte reply at 115200 is 4.6 ms, so 217 samples per second is the
  * ceiling however fast the board sampled. The ring decouples the two: the
  * board writes at its own rate, the host takes fifteen at a time.
  *
  * Producers are a mix. Board_SyncOnInjected runs in ADC3's interrupt at
  * 50 kHz; the angle and IMU loops run in main(). The consumer is the
  * command layer, also in main(). So the only preemption that can happen is
  * the ISR landing between a main-loop producer's read and its write, and a
  * PRIMASK critical section is exactly the right size for that - there is no
  * RTOS here and a mutex would be a scheduler this board does not have.
  *
  * Full means the newest sample is dropped, not the oldest overwritten. A
  * capture with a hole at a known place beats one that silently slid, and
  * `dropped` says how many went. Nothing here judges a sample: it stores
  * raw codes and a timestamp, and every conversion stays where it was
  * defined (invariant 7).
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"

#include <string.h>

static board_sample_t s_ring[BOARD_LOG_DEPTH];
static volatile uint16_t s_head;        /* next slot to write */
static volatile uint16_t s_tail;        /* next slot to read  */
static volatile uint32_t s_dropped;
static volatile uint8_t  s_sources;     /* bitmask; 0 disables the lot */
static uint8_t  s_seq[BOARD_LOG_SOURCES];


static uint16_t next_of(uint16_t i)
{
  return (uint16_t)((i + 1U) % BOARD_LOG_DEPTH);
}


void Board_LogEnable(uint8_t sources)
{
  /* Reset alongside the mask rather than leaving old samples in front of new
     ones: a burst whose first records predate the run is worse than an empty
     one, and there is no field that would say so. */
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  s_sources = sources;
  s_head = 0U;
  s_tail = 0U;
  s_dropped = 0U;
  memset(s_seq, 0, sizeof(s_seq));
  if (!masked)
  {
    __enable_irq();
  }
}


uint8_t Board_LogSources(void)
{
  return s_sources;
}


void Board_LogPush(uint8_t source, const int16_t *v, uint8_t n)
{
  if ((source >= BOARD_LOG_SOURCES) ||
      ((s_sources & (uint8_t)(1U << source)) == 0U))
  {
    return;
  }

  board_sample_t rec;
  rec.at = Board_Cycles();
  rec.source = source;
  rec.seq = s_seq[source]++;
  rec.v[0] = 0;
  rec.v[1] = 0;
  rec.v[2] = 0;
  rec.v[3] = 0;
  for (uint8_t i = 0U; (i < n) && (i < 4U); i++)
  {
    rec.v[i] = v[i];
  }

  /* Short on purpose. This runs inside ADC3's interrupt at 50 kHz, and the
     window where interrupts are off is one struct copy and one index. */
  const uint32_t masked = __get_PRIMASK();
  __disable_irq();
  const uint16_t next = next_of(s_head);

  if (next == s_tail)
  {
    s_dropped++;
  }
  else
  {
    s_ring[s_head] = rec;
    s_head = next;
  }
  if (!masked)
  {
    __enable_irq();
  }
}


uint16_t Board_LogCount(void)
{
  const uint16_t head = s_head;
  const uint16_t tail = s_tail;

  return (head >= tail) ? (uint16_t)(head - tail)
                        : (uint16_t)(BOARD_LOG_DEPTH - tail + head);
}


uint32_t Board_LogDropped(void)
{
  return s_dropped;
}


uint16_t Board_LogTake(board_sample_t *out, uint16_t max)
{
  uint16_t taken = 0U;

  if (out == NULL)
  {
    return 0U;
  }

  /* The consumer owns the tail and only ever advances it, so a producer
     preempting between these two lines can add a sample but never remove
     one. The copy is outside the critical section for that reason: the slot
     being read cannot be the slot being written unless the ring is empty,
     and then the loop has already stopped. */
  while ((taken < max) && (s_tail != s_head))
  {
    out[taken] = s_ring[s_tail];
    s_tail = next_of(s_tail);
    taken++;
  }
  return taken;
}
