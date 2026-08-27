/**
  ******************************************************************************
  * @file    board_sto.c
  * @brief   What the board can see of the Safe Torque Off chain.
  *
  * Gate driver supply is not the MCU's to switch, and there is no pin for it.
  * A chain on STO.SchDoc releases it, unlocked by a common-mode pilot tone
  * the MASTER injects on the RS485 pair - R36/R37 tap the midpoint, a 1 kHz
  * to 10 kHz band pass and a TLV3492 comparator pair detect it, and leaky
  * integrators turn "the tone is still arriving" into a supply. Stop sending
  * and the level decays. See docs/HARDWARE.md.
  *
  * So this file reads, and that is all it does. Two ADC channels bring the
  * chain back to the MCU:
  *
  *   Cinj    the recovered pilot, off the detector
  *   Clevel  the integrator level - the margin before the chain drops out
  *
  * It does NOT decide whether the chain has released. That would need a
  * threshold on Clevel, and invariant 10 puts thresholds in a test executive
  * beside a calibrated instrument, not in here. The one verdict this board
  * may give is the one it can prove from its own registers: TIM1's break
  * latch, which is nFAULT arriving on PE15 - marked (STOP) on the MCU sheet.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"

#include <string.h>

/** Signal names in the channel table, which is the only place that says
    which ADC and which pin each one is on. Looked up rather than indexed so
    a table that grows a channel does not move these two out from under us. */
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
  int32_t scaled;               /* Board_AdcRead refuses a NULL, and passing
                                   one here made pilot_ok and level_ok read
                                   false for every call ever made. Neither
                                   channel has a cooked unit to collect. */

  if (!STO_Find(signal, &index))
  {
    return false;
  }
  return Board_AdcRead(index, raw, microvolts, &scaled);
}


static uint32_t s_keepalive;


void Board_StoKeepalive(void)
{
  /* PA10 into R72 330R, C71 100nF and the D10/D14/D15 diodes: a charge
     pump, so only edges deliver anything and a held level is worth exactly
     as much as a stopped CPU. That is the point of it - the chain decays
     unless main() keeps turning, and no timer can fake that.

     Measured in electronic_simulations/sto: the model drives this at 100 kHz
     and stops at 18 ms to show the release. */
  HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
  s_keepalive++;
}


void Board_StoState(board_sto_state_t *out)
{
  if (out == NULL)
  {
    return;
  }

  memset(out, 0, sizeof(*out));

  /* Both channels come through the AFE's reference, so with AFE_ON low they
     read exact mid-scale and mean nothing - invariant 9. Reported either
     way, under a flag that cannot be mistaken for one of them. */
  out->afe_on = Board_AfeOn();
  out->pilot_ok = STO_ReadOne(STO_PILOT, &out->pilot_raw,
                              &out->pilot_microvolts);
  out->level_ok = STO_ReadOne(STO_LEVEL, &out->level_raw,
                              &out->level_microvolts);

  /* The one thing the hardware settles by itself. */
  out->stopped = Board_PwmFault();

  /* Reported, not judged: how fast the loop is turning is a fact, and
     whether it is fast enough belongs where the thresholds are. */
  out->keepalive = s_keepalive;
}
