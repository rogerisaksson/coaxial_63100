/** wbc_rcu.h - her loop's side of a tuner apart (machine.rcu): the shared block's layout in C, no
    lock between them. Each section on whole 64-byte lines: the header's words, a line a ring, each
    ring's slots, two banks of a table each, a column's rows contiguous (SoA). A word is written by
    one side alone: her loop's stores of a ring's head and of TOOK release, its loads of the tuner's
    words acquire. Host C11: the board's loop takes its tables over the wire. */
#ifndef WBC_RCU_H
#define WBC_RCU_H

#include <stdint.h>

#define WBC_RCU_LINE     64
#define WBC_RCU_PER_LINE (WBC_RCU_LINE / 8)

/** The header's words, rcu.HEADER's order. */
enum
{
  WBC_RCU_TOOK, WBC_RCU_LAID, WBC_RCU_SEQ, WBC_RCU_BANK, WBC_RCU_ROWS, WBC_RCU_HALT, WBC_RCU_COLS,
  WBC_RCU_CAPACITY, WBC_RCU_RINGS, WBC_RCU_HEADER
};

/** A ring's line's words, rcu.RING's order. */
enum
{
  WBC_RCU_HEAD, WBC_RCU_DROPPED, WBC_RCU_TAIL, WBC_RCU_WIDTH, WBC_RCU_SLOTS, WBC_RCU_AT, WBC_RCU_RING
};

/** A line: eight words on a 64-byte boundary. */
typedef struct
{
  _Alignas(WBC_RCU_LINE) int64_t w[WBC_RCU_PER_LINE];
} wbc_rcu_line_t;

/** `words` rounded up to whole lines. */
int64_t wbc_rcu_lines(int64_t words);

/** Where a section begins, words from the block's start: the header (0), ring `k`'s line, its
    slots, bank `k`. */
int64_t wbc_rcu_ring_at(int k);
int64_t wbc_rcu_slots_at(const void *block, int k);
int64_t wbc_rcu_bank_at(const void *block, int k);

/** Her loop: `n` doubles of `sample` onto `ring`, then its head; a full ring drops it, counted: 0. */
int wbc_rcu_tap(void *block, int ring, const double *sample, int n);

/** Her loop: the table published since her last take copied into `table` (cols x capacity, a
    column's rows contiguous), its rows into `rows`, TOOK said; its sequence, kept only where the
    sequence held through the copy - else 0, nothing taken. */
int64_t wbc_rcu_taken(void *block, double *table, int64_t *rows);

#endif
