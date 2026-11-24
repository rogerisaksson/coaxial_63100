/** link_report.c - Human-readable link status, for the ASCII console only. */
#include "link.h"
#include "cmd.h"
#include "say.h"

void Link_ReportStatus(void)
{
  link_stats_t s;
  link_stats(&s);

  say_text("link: proto=");
  say_text(link_proto_name());
  say_text("  unit=");
  say_u32(s.unit_id);
  say_text("  state=");
  say_text(link_active() ? "BINARY" : "console");
  say_text("  t1.5=");
  say_u32(s.t15_ticks);
  say_text("  t3.5=");
  say_u32(s.t35_ticks);
  say_text(" ticks\r\n  bus_message=");
  say_u32(s.bus_message);
  say_text("  bus_comm_error=");
  say_u32(s.bus_comm_error);
  say_text("  char_overrun=");
  say_u32(s.char_overrun);
  say_text("\r\n  server_message=");
  say_u32(s.server_message);
  say_text("  server_exception=");
  say_u32(s.server_exception);
  say_text("  server_no_response=");
  say_u32(s.server_no_response);
  say_text("\r\n");

  const uint16_t n = cmd_count();

  say_text("  commands:");
  for (uint16_t i = 0U; i < n; i++)
  {
    const cmd_desc_t *d = cmd_at(i);
    say_text(" 0x");
    say_hex(d->code, 2U);
    say_text("=");
    say_text(d->name);
  }
  say_text("\r\n");
}
