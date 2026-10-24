/** wbc_rcu.c - her loop's side of the shared block (wbc_rcu.h, machine.rcu). */
#include "wbc_rcu.h"

#include <stdatomic.h>
#include <string.h>

_Static_assert(sizeof(wbc_rcu_line_t) == WBC_RCU_LINE && _Alignof(wbc_rcu_line_t) == WBC_RCU_LINE,
               "a line is 64 bytes on its boundary");

static _Atomic int64_t *word(void *block, int64_t at)
{
  return (_Atomic int64_t *)block + at;
}

static int64_t got(const void *block, int64_t at)
{
  return atomic_load_explicit((const _Atomic int64_t *)block + at, memory_order_acquire);
}

int64_t wbc_rcu_lines(int64_t words)
{
  return (words + WBC_RCU_PER_LINE - 1) / WBC_RCU_PER_LINE * WBC_RCU_PER_LINE;
}

int64_t wbc_rcu_ring_at(int k)
{
  return wbc_rcu_lines(WBC_RCU_HEADER) + (int64_t)WBC_RCU_PER_LINE * k;
}

int64_t wbc_rcu_slots_at(const void *block, int k)
{
  return got(block, wbc_rcu_ring_at(k) + WBC_RCU_AT);
}

int64_t wbc_rcu_bank_at(const void *block, int k)
{
  const int     last = (int)got(block, WBC_RCU_RINGS) - 1;
  const int64_t r = wbc_rcu_ring_at(last);
  const int64_t end = got(block, r + WBC_RCU_AT) + got(block, r + WBC_RCU_SLOTS) * wbc_rcu_lines(got(block, r + WBC_RCU_WIDTH));

  return end + k * got(block, WBC_RCU_COLS) * wbc_rcu_lines(got(block, WBC_RCU_CAPACITY));
}

int wbc_rcu_tap(void *block, int ring, const double *sample, int n)
{
  const int64_t r = wbc_rcu_ring_at(ring);
  const int64_t head = atomic_load_explicit(word(block, r + WBC_RCU_HEAD), memory_order_relaxed);
  const int64_t slots = got(block, r + WBC_RCU_SLOTS);

  if (head - got(block, r + WBC_RCU_TAIL) >= slots)
  {
    atomic_fetch_add_explicit(word(block, r + WBC_RCU_DROPPED), 1, memory_order_relaxed);
    return 0;
  }
  memcpy((double *)block + got(block, r + WBC_RCU_AT) + (head % slots) * wbc_rcu_lines(got(block, r + WBC_RCU_WIDTH)),
         sample, (size_t)n * sizeof(double));
  atomic_store_explicit(word(block, r + WBC_RCU_HEAD), head + 1, memory_order_release);
  return 1;
}

int64_t wbc_rcu_taken(void *block, double *table, int64_t *rows)
{
  const int64_t seq = got(block, WBC_RCU_SEQ);
  const int64_t cols = got(block, WBC_RCU_COLS), capacity = got(block, WBC_RCU_CAPACITY);
  const double *bank;

  if (seq == got(block, WBC_RCU_TOOK))
  {
    return 0;
  }
  bank = (const double *)block + wbc_rcu_bank_at(block, (int)got(block, WBC_RCU_BANK));
  *rows = got(block, WBC_RCU_ROWS);
  for (int64_t c = 0; c < cols; c++)
  {
    memcpy(table + c * capacity, bank + c * wbc_rcu_lines(capacity), (size_t)*rows * sizeof(double));
  }
  if (got(block, WBC_RCU_SEQ) != seq)
  {
    return 0;
  }
  atomic_store_explicit(word(block, WBC_RCU_TOOK), seq, memory_order_release);
  return seq;
}
