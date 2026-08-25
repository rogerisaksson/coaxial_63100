/**
  ******************************************************************************
  * @file    board_selftest.c
  * @brief   What the board can prove about itself, with nothing attached.
  *
  * The rule for what belongs here: a check is PASS/FAIL only if the board can
  * settle it from its own registers or its own flash. A locked PLL, a
  * calibration that ran, a checksum that matches - those are provable. A voltage
  * being "right" is not, because the board has no calibrated reference and its
  * own ADC is the thing under test.
  *
  * So everything needing an external instrument comes out as INFO with a value
  * attached, and the pass/fail decision belongs to the test executive on the
  * line, next to the DMM and the electronic load. A limit compiled in here would
  * be a limit nobody on the line can see, change, or record against a
  * calibration certificate.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"
#include "modbus_crc.h"

/* End of code and read-only data in flash, from the linker script. The image
   runs from the vector table to here. */
extern uint32_t _etext;

#define FLASH_IMAGE_BASE 0x08000000UL

static void add(board_check_t *out, uint8_t *n, uint8_t capacity,
                const char *name, uint8_t status, int32_t value)
{
  if (*n >= capacity)
  {
    return;
  }

  out[*n].name = name;
  out[*n].status = status;
  out[*n].value = value;
  (*n)++;
}

static uint8_t verdict(bool ok)
{
  return (uint8_t)(ok ? BOARD_CHECK_PASS : BOARD_CHECK_FAIL);
}

/* Every configured channel should leave at most its own bit and, for a
   differential channel, its negative input's bit in PCSEL. More than two bits
   means the accumulation bug is back. */
static bool pcsel_clean(const ADC_TypeDef *adc)
{
  uint32_t bits = adc->PCSEL;
  uint8_t  count = 0U;

  while (bits != 0U)
  {
    bits &= (bits - 1U);
    count++;
  }

  return count <= 2U;
}

/* Differential calibration factor. Reported, never judged: a well matched ADC
   legitimately calibrates to an offset of zero, so a zero factor does NOT mean
   the calibration failed to run - and the registers offer no flag that says it
   did. An earlier version of this file asserted non-zero and failed a perfectly
   healthy board, which is precisely the mistake a limit compiled into firmware
   invites. A line compares these across units instead. */
static int32_t adc_calfact_diff(const ADC_TypeDef *adc)
{
  return (int32_t)((adc->CALFACT & ADC_CALFACT_CALFACT_D) >>
                   ADC_CALFACT_CALFACT_D_Pos);
}

uint8_t Board_SelfTest(board_check_t *out, uint8_t capacity)
{
  uint8_t n = 0U;

  /* Names are kept short deliberately. Each check costs 6 bytes plus its name,
     and the whole reply has to fit one 250-byte RTU payload - an earlier version
     with descriptive names came to 276 and was rejected outright by the writer's
     overflow check rather than truncated, which is the behaviour we want but not
     a thing to rely on. */

  /* ---- clock tree: provable from RCC ---- */
  add(out, &n, capacity, "hse_rdy", verdict((RCC->CR & RCC_CR_HSERDY) != 0U), 0);
  add(out, &n, capacity, "pll1_lock",
      verdict((RCC->CR & RCC_CR_PLL1RDY) != 0U), 0);
  add(out, &n, capacity, "clk_crystal", verdict(Board_SysClkOnCrystal()), 0);

  /* HAL derives this from the RCC registers while SystemCoreClock is a variable
     the startup code set. Disagreement means one of them is stale. */
  const uint32_t derived = HAL_RCC_GetSysClockFreq();
  add(out, &n, capacity, "clk_agrees",
      verdict(derived == SystemCoreClock), (int32_t)derived);

  /* ---- the timebase the protocol depends on ---- */
  const uint32_t first = Board_Cycles();
  for (volatile uint32_t spin = 0U; spin < 1000U; spin++)
  {
    /* long enough for the counter to move at any plausible clock */
  }
  add(out, &n, capacity, "cyccnt_runs", verdict(Board_Cycles() != first), 0);

  /* ---- the ADC reference is external by design ---- */
  const bool vref_external = ((VREFBUF->CSR & VREFBUF_CSR_ENVR) == 0U) &&
                             ((VREFBUF->CSR & VREFBUF_CSR_HIZ) != 0U);
  add(out, &n, capacity, "vref_ext", verdict(vref_external), 0);

  /* ---- ADC state ---- */
  add(out, &n, capacity, "cal_d1", BOARD_CHECK_INFO, adc_calfact_diff(ADC1));
  add(out, &n, capacity, "cal_d2", BOARD_CHECK_INFO, adc_calfact_diff(ADC2));
  add(out, &n, capacity, "cal_d3", BOARD_CHECK_INFO, adc_calfact_diff(ADC3));

  /* This one IS provable: more than two bits in PCSEL means the accumulation
     bug is back, and no reference is needed to say so. Bitmask so a single
     offending unit is identifiable: bit 0 = ADC1, bit 1 = ADC2, bit 2 = ADC3. */
  const int32_t clean = (pcsel_clean(ADC1) ? 1 : 0) |
                        (pcsel_clean(ADC2) ? 2 : 0) |
                        (pcsel_clean(ADC3) ? 4 : 0);
  add(out, &n, capacity, "adc_pcsel", verdict(clean == 7), clean);

  /* ---- firmware integrity ---- */
  const uint8_t *image = (const uint8_t *)FLASH_IMAGE_BASE;
  const uint32_t length = (uint32_t)((const uint8_t *)&_etext - image);

  /* Reported, not judged: the board has nothing to compare these against. A
     line compares them across units and against the build it meant to load. */
  add(out, &n, capacity, "image_len", BOARD_CHECK_INFO, (int32_t)length);
  add(out, &n, capacity, "image_crc", BOARD_CHECK_INFO,
      (int32_t)modbus_crc16(image, length));

  /* ---- values for the executive to judge against its instruments ---- */
  add(out, &n, capacity, "sysclk_hz", BOARD_CHECK_INFO, (int32_t)Board_SysClkHz());
  add(out, &n, capacity, "hclk_hz", BOARD_CHECK_INFO, (int32_t)Board_HclkHz());
  add(out, &n, capacity, "afe_on", BOARD_CHECK_INFO, Board_AfeOn() ? 1 : 0);

  return n;
}
