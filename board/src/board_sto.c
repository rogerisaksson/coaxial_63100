/**
  ******************************************************************************
  * @file    board_sto.c
  * @brief   What the board can see of the Safe Torque Off chain.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"

#include <string.h>

/** Signal names in the channel table, which is the only place that says
    which ADC and which pin each one is on. */
#define STO_PILOT  "Cinj"
#define STO_LEVEL  "Clevel"


static bool STO_Find(const char *signal, uint8_t *index)
{
  const uint8_t count = Board_AdcCount();
  board_chan_t info;

  for (uint8_t i = 0U; i < count; i++)
  {
    if (Board_AdcChan(i, &info) && info.signal != NULL
        && strcmp(info.signal, signal) == 0)
    {
      *index = i;
      return true;
    }
  }
  return false;
}


static bool STO_ReadOne(const char *signal, int32_t *raw, int32_t *microvolts)
{
  uint8_t index;
  int32_t scaled;               /* Board_AdcRead refuses a NULL, and passing one here made pilot_ok and
     level_ok read false for every call ever made. */

  if (!STO_Find(signal, &index))
  {
    return false;
  }
  return Board_AdcRead(index, raw, microvolts, &scaled);
}


/** The STO chain's state: the keepalive's edges, the worst gap, and the last
    pilot and level readings. */
static struct
{
  uint32_t keepalive;
  uint32_t last_edge;
  uint32_t worst_gap;
} s;


/** Cycles between edges: 200 kHz of edges is the 100 kHz square wave the
    model in electronic_simulations/sto drives MCU_PWM with. */
static uint32_t sto_edge_cycles(void)
{
  static uint32_t cached;

  if (cached == 0U)
  {
    cached = SystemCoreClock / 200000U;
  }
  return cached;
}


void Board_StoKeepalive(void)
{
  /* The longest gap between edges, in raw CYCCNT ticks - invariant 2's rule
     applies here too: dividing cycles down moves the wrap off a power of two
     and the unsigned arithmetic breaks across it. */
  const uint32_t now = Board_Cycles();
  const uint32_t gap = now - s.last_edge;
  const bool pumping = s.keepalive != 0U;

  /* Rate limited, not free-running. */
  if (pumping && (gap < sto_edge_cycles()))
  {
    return;
  }
  if (pumping && (gap > s.worst_gap))
  {
    s.worst_gap = gap;
  }
  s.last_edge = now;

  /* PA10 into R72 330R, C71 100nF and the D10/D14/D15 diodes: a charge pump,
     so only edges deliver anything and a held level is worth exactly as much
     as a stopped CPU. */
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
  s.keepalive++;
}


void Board_StoKeepaliveReset(void)
{
  s.worst_gap = 0U;
}


void Board_StoState(board_sto_state_t *out)
{
  if (out == NULL)
  {
    return;
  }

  memset(out, 0, sizeof(*out));

  /* Both channels come through the AFE's reference, so with AFE_ON low they
     read exact mid-scale and mean nothing - invariant 9. */
  out->afe_on = Board_AfeOn();
  out->pilot_ok = STO_ReadOne(STO_PILOT, &out->pilot_raw,
                              &out->pilot_microvolts);
  out->level_ok = STO_ReadOne(STO_LEVEL, &out->level_raw,
                              &out->level_microvolts);

  /* The one thing the hardware settles by itself. */
  out->stopped = Board_PwmFault();

  /* Reported, not judged: how fast the loop is turning is a fact, and
     whether it is fast enough belongs where the thresholds are. */
  out->keepalive = s.keepalive;
  out->worst_gap = s.worst_gap;
}
