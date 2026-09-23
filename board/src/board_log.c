/** board_log.c - One ring for every measurement, drained over the wire in
    bursts. */
#include "board.h"
#include "board_irq.h"
#include "board_hw.h"

#include <string.h>

/** The log ring's state: the ring, its fill, what has been dropped and
    thinned, and which sources are armed. */
static struct
{
  board_sample_t ring[BOARD_LOG_DEPTH];
  volatile uint16_t head;         /* next slot to write */
  volatile uint16_t tail;         /* next slot to read  */
  volatile uint32_t dropped;
  volatile uint8_t sources;       /* bitmask; 0 disables the lot */
  uint8_t seq[BOARD_LOG_SOURCES];
  volatile uint32_t thinned;      /* pushes refused by the rate limit */
  uint32_t min_gap;               /* cycles a source must leave        */
  uint32_t last_at[BOARD_LOG_SOURCES];
} s;

static uint16_t next_of(uint16_t i)
{
  return (uint16_t)((i + 1U) % BOARD_LOG_DEPTH);
}

void Board_LogEnable(uint8_t sources, uint32_t min_gap_cycles)
{
  /* Reset alongside the mask rather than leaving old samples in front of new
     ones: a burst whose first records predate the run is worse than an empty
     one, and there is no field that would say so. */
  const uint32_t masked = Board_IrqHold();
  s.sources = sources;
  s.min_gap = min_gap_cycles;
  s.head = 0U;
  s.tail = 0U;
  s.dropped = 0U;
  s.thinned = 0U;
  memset(s.seq, 0, sizeof(s.seq));
  /* A whole gap in the past, so every source's first push is free. */
  const uint32_t now = Board_Cycles();
  for (uint8_t i = 0U; i < BOARD_LOG_SOURCES; i++)
  {
    s.last_at[i] = now - min_gap_cycles;
  }
  Board_IrqRelease(masked);
}

uint8_t Board_LogSources(void)
{
  return s.sources;
}

void Board_LogPush(uint8_t source, const int16_t *v, uint8_t n)
{
  if ((source >= BOARD_LOG_SOURCES) ||
      ((s.sources & (uint8_t)(1U << source)) == 0U))
  {
    return;
  }

  /* One source must not crowd out another. */
  const uint32_t now = Board_Cycles();

  if ((s.min_gap != 0U) && ((now - s.last_at[source]) < s.min_gap))
  {
    s.thinned++;
    return;
  }
  s.last_at[source] = now;

  board_sample_t rec;
  rec.at = now;
  rec.source = source;
  rec.seq = s.seq[source]++;
  rec.v[0] = 0;
  rec.v[1] = 0;
  rec.v[2] = 0;
  rec.v[3] = 0;
  for (uint8_t i = 0U; (i < n) && (i < 4U); i++)
  {
    rec.v[i] = v[i];
  }

  /* Short on purpose. */
  const uint32_t masked = Board_IrqHold();
  const uint16_t next = next_of(s.head);

  if (next == s.tail)
  {
    s.dropped++;
  }
  else
  {
    s.ring[s.head] = rec;
    s.head = next;
  }
  Board_IrqRelease(masked);
}

uint16_t Board_LogCount(void)
{
  const uint16_t head = s.head;
  const uint16_t tail = s.tail;

  return (head >= tail) ? (uint16_t)(head - tail)
                        : (uint16_t)(BOARD_LOG_DEPTH - tail + head);
}

uint32_t Board_LogThinned(void)
{
  return s.thinned;
}

uint32_t Board_LogDropped(void)
{
  return s.dropped;
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
     one. */
  while ((taken < max) && (s.tail != s.head))
  {
    out[taken] = s.ring[s.tail];
    s.tail = next_of(s.tail);
    taken++;
  }
  return taken;
}
