/** console.c - the three-key ASCII console, its words written without stdio (say.c). */
#include "console.h"
#include "dev_serial.h"
#include "board.h"
#include "cmd.h"
#include "link.h"
#include "say.h"

#include "version.h"

/* The boot banner. */
void Console_Banner(void)
{
  static const char *const SRC[] = { "HSI", "CSI", "HSE", "PLL1", "unknown" };
  const uint8_t src = Board_SysClkSource();

  say_text("\r\n");
  say_text(Board_Name());
  say_text("  fw ");
  say_text(FW_VERSION_STRING);
  say_text("  proto ");
  say_u32(CMD_PROTO_MAJOR);
  say_text(".");
  say_u32(CMD_PROTO_MINOR);
  say_text("  build ");
  say_text(FW_BUILD_STRING);
  say_text("\r\nSYSCLK source = ");
  say_text(SRC[src]);
  say_text(" -> SYSCLK = ");
  say_u32(Board_SysClkHz());
  say_text(" Hz, HCLK = ");
  say_u32(Board_HclkHz());
  say_text(" Hz\r\n");

  /* PLL1 counts as crystal-derived: it is fed from HSE. */
  if (!Board_SysClkOnCrystal())
  {
    say_text("WARNING: SYSCLK is not derived from the 25 MHz HSE crystal!\r\n");
  }

  say_text("console: 'm' binary mode, 'r' link status, '?' help\r\n");
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
    say_text(link_proto_name());
    say_text(": binary mode, unit ");
    say_u32(link_unit_id());
    say_text(". Console output stops here.\r\n"
             "  return with command 0x48 (console), or holding register 0x0001 = 1\r\n");
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
    say_text("commands: 'm' binary mode, 'r' link status, '?' this help\r\n");
    return;
  }

  /* Anything else, including the CR and LF a terminal sends, is ignored. */
}
