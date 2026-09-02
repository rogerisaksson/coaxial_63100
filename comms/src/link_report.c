/**
  ******************************************************************************
  * @file    link_report.c
  * @brief   Human-readable link status, for the ASCII console only.
  *
  * Kept apart from link.c so the stack itself never depends on printf. This is
  * the console's view of the link, not part of the link.
  ******************************************************************************
  */
#include "link.h"
#include "cmd.h"

#include <stdio.h>

void Link_ReportStatus(void)
{
  link_stats_t s;
  link_stats(&s);

  printf("link: proto=%s  unit=%u  state=%s  t1.5=%lu  t3.5=%lu ticks\r\n",
         link_proto_name(), (unsigned)s.unit_id,
         link_active() ? "BINARY" : "console",
         (unsigned long)s.t15_ticks, (unsigned long)s.t35_ticks);
  printf("  bus_message=%lu  bus_comm_error=%lu  char_overrun=%lu\r\n",
         (unsigned long)s.bus_message, (unsigned long)s.bus_comm_error,
         (unsigned long)s.char_overrun);
  printf("  server_message=%lu  server_exception=%lu  server_no_response=%lu\r\n",
         (unsigned long)s.server_message, (unsigned long)s.server_exception,
         (unsigned long)s.server_no_response);

  const uint16_t n = cmd_count();

  printf("  commands:");
  for (uint16_t i = 0U; i < n; i++)
  {
    const cmd_desc_t *d = cmd_at(i);
    printf(" 0x%02X=%s", (unsigned)d->code, d->name);
  }
  printf("\r\n");
}
