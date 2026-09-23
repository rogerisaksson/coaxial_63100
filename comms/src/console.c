/**
  ******************************************************************************
  * @file    console.c
  * @brief   printf retarget and the three-key ASCII console. See console.h.
  ******************************************************************************
  */
#include "console.h"
#include "dev_serial.h"
#include "board.h"
#include "board_hw.h"
#include "cmd.h"
#include "link.h"

#include "version.h"

#include <stdio.h>


/* The boot banner. */
void Console_Banner(void)
{
  static const char *const SRC[] = { "HSI", "CSI", "HSE", "PLL1", "unknown" };
  const uint8_t src = Board_SysClkSource();

  printf("\r\n%s  fw %s  proto %u.%u  build %s\r\n",
         Board_Name(), FW_VERSION_STRING,
         (unsigned)CMD_PROTO_MAJOR, (unsigned)CMD_PROTO_MINOR, FW_BUILD_STRING);
  printf("SYSCLK source = %s -> SYSCLK = %lu Hz, HCLK = %lu Hz\r\n",
         SRC[src], (unsigned long)Board_SysClkHz(), (unsigned long)Board_HclkHz());

  /* PLL1 counts as crystal-derived: it is fed from HSE. */
  if (!Board_SysClkOnCrystal())
  {
    printf("WARNING: SYSCLK is not derived from the 25 MHz HSE crystal!\r\n");
  }

  printf("console: 'm' binary mode, 'r' link status, '?' help\r\n");
}

/* Retarget printf() to the USART3 debug port */
int __io_putchar(int ch)
{
  uint8_t c = (uint8_t)ch;
  HAL_UART_Transmit(&huart3, &c, 1, HAL_MAX_DELAY);
  return ch;
}

/* The ASCII console exists for one reason: to get into the binary link and
   back out again by hand. */
void Console_Poll(void)
{
  uint8_t rx;

  /* Through the same ring the binary link reads, because USART3 receives on
     interrupt now. */
  const dev_serial_t *dev = dev_uart(0);
  uint32_t tick = 0U;

  if ((dev == NULL) || !dev->get(dev->ctx, &rx, &tick))
  {
    return;
  }

  if ((rx == 'm') || (rx == 'M'))
  {
    /* Print the way back BEFORE handing over: once the link owns the line,
       nothing may write ASCII to it. */
    printf("%s: binary mode, unit %u. Console output stops here.\r\n"
           "  return with command 0x48 (console), or holding register 0x0001 = 1\r\n",
           link_proto_name(), (unsigned)link_unit_id());
    link_open();
    return;
  }

  if ((rx == 'r') || (rx == 'R'))
  {
    Link_ReportStatus();
    return;
  }

  if (rx == '?')
  {
    printf("commands: 'm' binary mode, 'r' link status, '?' this help\r\n");
    return;
  }

  /* Anything else, including the CR and LF a terminal sends, is ignored. */
}
