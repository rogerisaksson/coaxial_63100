/**
  ******************************************************************************
  * @file    cmd_board.c
  * @brief   The command table for this board: the old ASCII reports, in binary.
  *
  * Every handler is a flat run of statements. That is not a style preference:
  * the reader and writer in wire.h are total, so there is nothing to branch on
  * between fields, and cmd_dispatch checks their sticky flags once afterwards.
  * A handler only branches where the BOARD can genuinely fail.
  ******************************************************************************
  */
#include <string.h>

#include "cmd.h"
#include "board.h"
#include "board_power.h"
#include "link.h"
#include "version.h"

/* The version query, and the one command whose layout is frozen. The protocol
   major comes FIRST so that a host of any vintage can read two bytes, decide
   whether it understands this device at all, and stop. Fields may only ever be
   appended after that; anything else is a new major by definition. */
static cmd_status_t h_version(rd_t *in, wr_t *out)
{
  (void)in;

  wr_u8(out, CMD_PROTO_MAJOR);
  wr_u8(out, CMD_PROTO_MINOR);
  wr_u8(out, FW_VERSION_MAJOR);
  wr_u8(out, FW_VERSION_MINOR);
  wr_u8(out, FW_VERSION_PATCH);
  wr_str(out, FW_DEVICE_NAME);
  wr_str(out, FW_MCU_NAME);
  wr_str(out, FW_BUILD_STRING);
  wr_u16(out, cmd_count());
  /* Appended, which is the one thing this record allows. An old host stops
     after command_count and never knows it is here. */
  wr_str(out, FW_DEVICE_DESCRIPTION);
  wr_str(out, FW_DEVICE_TYPE);

  return CMD_OK;
}

/* A row costs 17 bytes plus its pin and signal names against MB_MAX_PDU's
   253. Overflowing it is how "+5V sense" and "Gate supply" announced
   themselves - SERVER DEVICE FAILURE from a reply that would not fit -
   and `wr_ok` is checked at the end now, so the next one says so. */
static cmd_status_t h_adc_table(rd_t *in, wr_t *out)
{
  /* Optional start index, so the table is not bounded by one reply: a row
     is 18 bytes plus its names against a 252 cap, and seven channels came to
     197 while nine came to 254. Absent it reads 0, so a host that never
     sends one gets what it always did for as long as that fits. */
  const uint8_t start = (rd_left(in) > 0U) ? rd_u8(in) : 0U;
  const uint8_t total = Board_AdcCount();

  if (start >= total)
  {
    wr_u8(out, 0U);
    wr_u8(out, total);
    return CMD_OK;
  }

  /* Counted first, then written, because the count leads the rows and a
     writer that ran out halfway would have already sent the wrong one. */
  uint8_t n = 0U;
  uint16_t room = (uint16_t)(wr_room(out) - 2U);

  for (uint8_t i = start; i < total; i++)
  {
    board_chan_t c;

    if (!Board_AdcChan(i, &c))
    {
      return CMD_ERR_DEVICE;
    }

    const uint16_t cost = (uint16_t)(18U + (uint16_t)strlen(c.pin)
                                        + (uint16_t)strlen(c.signal));

    if (cost > room)
    {
      break;
    }
    room = (uint16_t)(room - cost);
    n++;
  }

  wr_u8(out, n);

  for (uint8_t i = start; i < (uint8_t)(start + n); i++)
  {
    board_chan_t c;
    int32_t raw = 0;
    int32_t uv = 0;
    int32_t scaled = 0;

    if (!Board_AdcChan(i, &c))
    {
      return CMD_ERR_DEVICE;
    }

    if (!Board_AdcRead(i, &raw, &uv, &scaled))
    {
      return CMD_ERR_DEVICE;
    }

    wr_u8(out, c.adc_index);
    wr_u8(out, c.channel);
    wr_str(out, c.pin);
    wr_u8(out, c.differential ? 1U : 0U);
    wr_str(out, c.signal);
    wr_i32(out, raw);
    wr_i32(out, uv);
    wr_u8(out, c.unit);
    wr_i32(out, scaled);
  }

  /* Appended, so a host that stops at the rows reads what it always did. */
  wr_u8(out, total);

  return wr_ok(out) ? CMD_OK : CMD_ERR_LENGTH;
}

static cmd_status_t h_adc_scan(rd_t *in, wr_t *out)
{
  (void)in;

  int32_t u = 0;
  int32_t v = 0;
  int32_t w = 0;
  int32_t dc_raw = 0;
  int32_t dc_mv = 0;
  int32_t ntc_raw = 0;
  int32_t ntc_cc = 0;

  if (!Board_PhaseRaw(&u, &v, &w))
  {
    return CMD_ERR_DEVICE;
  }

  if (!Board_DcBus(&dc_raw, &dc_mv))
  {
    return CMD_ERR_DEVICE;
  }

  /* The NTC needs the reference, which the AFE powers. Reporting a temperature
     with the AFE off would be reporting exactly 25.00 C every time, because
     mid-scale puts the divider at R25 by definition. Let it fail instead. */
  const bool ntc_ok = Board_Ntc(&ntc_raw, &ntc_cc);

  wr_i32(out, u);
  wr_i32(out, v);
  wr_i32(out, w);
  wr_i32(out, dc_raw);
  wr_i32(out, dc_mv);
  wr_i32(out, ntc_raw);
  wr_i32(out, ntc_ok ? ntc_cc : 0);
  wr_u8(out, Board_AfeOn() ? 1U : 0U);
  wr_u8(out, Board_Pe15() ? 1U : 0U);

  /* WHO HOLDS IT. The rail is reference counted, so `on` after an explicit
     off is not a lie and not a failure - it means somebody else still wants
     it, and without this byte a caller cannot tell that from a write that
     never landed. Appended, so an older host stops reading above it. */
  board_rail_state_t rail;

  wr_u8(out, Board_PowerState(BOARD_RAIL_AFE, &rail) ? rail.users : 0U);

  return CMD_OK;
}

static cmd_status_t h_adc_noise(rd_t *in, wr_t *out)
{
  const uint8_t  adc     = rd_u8(in);
  const uint16_t samples = rd_u16(in);

  if ((adc < 1U) || (adc > 3U))
  {
    return CMD_ERR_VALUE;
  }

  if ((samples < 1U) || (samples > 1000U))
  {
    return CMD_ERR_VALUE;
  }

  int32_t  mean_uv = 0;
  int32_t  min_raw = 0;
  int32_t  max_raw = 0;
  uint32_t span    = 0U;
  uint32_t sd_uv   = 0U;

  if (!Board_AdcNoise(adc, samples, &mean_uv, &min_raw, &max_raw, &span, &sd_uv))
  {
    return CMD_ERR_DEVICE;
  }

  wr_u16(out, samples);
  wr_i32(out, mean_uv);
  wr_i32(out, min_raw);
  wr_i32(out, max_raw);
  wr_u32(out, span);
  wr_u32(out, sd_uv);

  return CMD_OK;
}

static cmd_status_t h_clock(rd_t *in, wr_t *out)
{
  (void)in;

  wr_u32(out, Board_SysClkHz());
  wr_u32(out, Board_HclkHz());
  wr_u32(out, Board_Cycles());
  wr_u32(out, link_ticks_per_us());
  wr_u8(out, Board_SysClkSource());

  return CMD_OK;
}

static cmd_status_t h_afe(rd_t *in, wr_t *out)
{
  const uint8_t action = rd_u8(in);

  if (action > 3U)
  {
    return CMD_ERR_VALUE;
  }

  /* 0 read, 1 off, 2 on, 3 toggle. Flat because the table is the branch. */
  static const int8_t NEXT[4] = { -1, 0, 1, -2 };
  const int8_t want = NEXT[action];

  /* Through the reference count, not the pin. The host is one user among
     several: switching the pin directly here would drop the rail under a
     subsystem that had asked for it, which is the bug board_power.h exists
     to remove. */
  const int on = (want == -2) ? (Board_AfeOn() ? 0 : 1) : want;

  if (on == 0)
  {
    (void)Board_PowerRelease(BOARD_RAIL_AFE, BOARD_USER_HOST);
  }

  if (on == 1)
  {
    (void)Board_PowerAcquire(BOARD_RAIL_AFE, BOARD_USER_HOST);
  }

  wr_u8(out, Board_AfeOn() ? 1U : 0U);
  wr_u8(out, Board_Pe15() ? 1U : 0U);

  /* WHO HOLDS IT. The rail is reference counted, so `on` after an explicit
     off is not a lie and not a failure - it means somebody else still wants
     it, and without this byte a caller cannot tell that from a write that
     never landed. Appended, so an older host stops reading above it. */
  board_rail_state_t rail;

  wr_u8(out, Board_PowerState(BOARD_RAIL_AFE, &rail) ? rail.users : 0U);

  return CMD_OK;
}

static cmd_status_t h_analog_burst(rd_t *in, wr_t *out)
{
  const uint16_t mask     = rd_u16(in);
  const uint16_t samples  = rd_u16(in);
  const uint32_t interval = rd_u32(in);

  /* The three limits Board_AdcBurst enforces, checked here as well, so that
     its own false can mean what h_adc_noise's already does: the device
     failed, not the request. A conversion that times out mid-burst is
     SERVER DEVICE FAILURE, and reporting it as ILLEGAL DATA VALUE sends the
     host looking at arguments that were fine. */
  if ((mask == 0U) || (samples < 1U) || (samples > BOARD_BURST_MAX_SAMPLES) ||
      (((uint64_t)samples * (uint64_t)interval) > (uint64_t)BOARD_BURST_MAX_US))
  {
    return CMD_ERR_VALUE;
  }

  board_burst_t stats[BOARD_BURST_MAX_CHAN];
  uint8_t       count = 0U;
  uint32_t      elapsed = 0U;

  if (!Board_AdcBurst(mask, samples, interval, stats, &count, &elapsed))
  {
    return CMD_ERR_DEVICE;
  }

  wr_u16(out, samples);
  wr_u32(out, elapsed);
  wr_u8(out, count);

  for (uint8_t i = 0U; i < count; i++)
  {
    wr_u8(out, stats[i].index);
    wr_i32(out, stats[i].mean_milliraw);
    wr_i32(out, stats[i].min_raw);
    wr_i32(out, stats[i].max_raw);
    wr_u32(out, stats[i].sd_milliraw);
  }

  return CMD_OK;
}

static cmd_status_t h_self_test(rd_t *in, wr_t *out)
{
  (void)in;

  board_check_t checks[BOARD_SELFTEST_MAX];
  const uint8_t count = Board_SelfTest(checks, BOARD_SELFTEST_MAX);

  wr_u8(out, count);

  for (uint8_t i = 0U; i < count; i++)
  {
    wr_str(out, checks[i].name);
    wr_u8(out, checks[i].status);
    wr_i32(out, checks[i].value);
  }

  return CMD_OK;
}

static cmd_status_t h_link_stats(rd_t *in, wr_t *out)
{
  (void)in;

  link_stats_t s;
  link_stats(&s);

  wr_u8(out, s.unit_id);
  wr_u32(out, s.t15_ticks);
  wr_u32(out, s.t35_ticks);
  wr_u32(out, s.bus_message);
  wr_u32(out, s.bus_comm_error);
  wr_u32(out, s.server_message);
  wr_u32(out, s.server_exception);
  wr_u32(out, s.server_no_response);
  wr_u32(out, s.char_overrun);

  return CMD_OK;
}

static cmd_status_t h_console(rd_t *in, wr_t *out)
{
  (void)in;
  (void)out;

  Board_RequestConsoleMode();
  return CMD_OK;
}

#define CHANNELS_ANALOG   0U
#define CHANNELS_DIGITAL  1U
#define CHANNELS_RESERVED 2U
/* What the board is made of, rather than what it is wired to. The same
   question one level up, and the same rule: the firmware settles it. */
#define CHANNELS_SUBSYSTEMS 3U
/* What is fitted, rather than what it can do. Paged: the parts and their
   strings pass MB_MAX_PDU's 253 - the same overflow that split the analog
   and digital sections apart. */
#define CHANNELS_PARTS 4U

/* Enough room for the longest record the table can hold, checked before each
   one rather than after: wr_t's overflow flag is sticky, and a truncated
   record answered 0x04 instead of the parts that did fit. */
#define PART_RECORD_MAX 96U

/* A pin, its direction and its signal name, with room to spare. */
#define PIN_RECORD_MAX 40U

/** Kind 0: every analog channel, in the channel table's order. */
static cmd_status_t analog_rows(wr_t *out)
{
  const uint8_t analog = Board_AdcCount();

  wr_u8(out, analog);

  for (uint8_t i = 0U; i < analog; i++)
  {
    board_chan_t c;

    if (!Board_AdcChan(i, &c))
    {
      return CMD_ERR_DEVICE;
    }

    wr_u8(out, i);
    wr_u8(out, c.adc_index);
    wr_u8(out, c.channel);
    wr_str(out, c.pin);
    wr_u8(out, BOARD_DIR_IN);       /* an ADC channel is an input, always */
    wr_u8(out, c.differential ? 1U : 0U);
    wr_str(out, c.signal);
    wr_u8(out, c.unit);
  }

  return CMD_OK;
}

/** Kinds 1 and 2, paged from an optional `first`. I/O and infrastructure
  * are two questions kept two answers: kind 1 is what a fixture may set or
  * read without breaking anything; kind 2 is the bus and the debug port,
  * listed only so "why was PB10 refused" has an answer, never a channel to
  * drive. Paged like the parts list: the reserved section went from 7 pins
  * to 19 when SPI2, SPI4 and the IMU's control pins were listed, and 19
  * rows are 418 bytes against MB_MAX_PDU's 253 - the whole section came
  * back as an 0x04 from the writer's overflow flag. */
static cmd_status_t pin_page(rd_t *in, wr_t *out, bool want_io)
{
  const uint8_t rows = Board_DigitalCount();
  const uint8_t first = (rd_left(in) > 0U) ? rd_u8(in) : 0U;
  uint8_t matching = 0U;
  uint8_t sent = 0U;
  uint8_t seen = 0U;

  for (uint8_t i = 0U; i < rows; i++)
  {
    board_dchan_t d;

    if (Board_DigitalChan(i, &d) && (d.usable == want_io))
    {
      matching++;
    }
  }

  if (first > matching)
  {
    return CMD_ERR_VALUE;
  }

  wr_u8(out, matching);
  wr_u8(out, first);
  wr_u8(out, 0U);              /* how many follow - filled in below */

  uint8_t *count_at = &out->buf[out->len - 1U];

  for (uint8_t i = 0U; i < rows; i++)
  {
    board_dchan_t d;

    if (!Board_DigitalChan(i, &d))
    {
      return CMD_ERR_DEVICE;
    }
    if (d.usable != want_io)
    {
      continue;
    }
    if (seen++ < first)
    {
      continue;
    }
    if ((out->len + PIN_RECORD_MAX) > out->cap)
    {
      break;
    }

    wr_str(out, d.pin);
    wr_u8(out, d.dir);
    wr_str(out, d.signal);
    sent++;
  }

  *count_at = sent;
  return CMD_OK;
}

/** Kind 3: the subsystems, one per command table. */
static cmd_status_t subsystem_rows(wr_t *out)
{
  const uint8_t groups = cmd_group_count();

  wr_u8(out, groups);

  for (uint8_t i = 0U; i < groups; i++)
  {
    const cmd_group_t *g = cmd_group(i);

    if (g == NULL)
    {
      return CMD_ERR_DEVICE;
    }

    wr_str(out, g->name);
    wr_str(out, g->what);
    wr_u8(out, g->commands);
  }

  return CMD_OK;
}

/** Kind 4: the fitted parts, paged from an optional `first`. */
static cmd_status_t parts_page(rd_t *in, wr_t *out)
{
  const uint8_t total = Board_PartCount();
  const uint8_t first = (rd_left(in) > 0U) ? rd_u8(in) : 0U;
  uint8_t sent = 0U;

  if (first > total)
  {
    return CMD_ERR_VALUE;
  }

  wr_u8(out, total);
  wr_u8(out, first);
  wr_u8(out, 0U);              /* how many follow - filled in below */

  uint8_t *count_at = &out->buf[out->len - 1U];

  for (uint8_t i = first; i < total; i++)
  {
    board_part_t part;

    if ((out->len + PART_RECORD_MAX) > out->cap)
    {
      break;
    }
    if (!Board_Part(i, &part))
    {
      return CMD_ERR_DEVICE;
    }

    wr_str(out, part.name);
    wr_str(out, part.what);
    wr_str(out, part.where);
    wr_str(out, part.power);
    wr_u8(out, part.state);
    sent++;
  }

  *count_at = sent;
  return CMD_OK;
}

static cmd_status_t h_channels(rd_t *in, wr_t *out)
{
  /* The map, and only the map - no reading. adc_table exists for "what does
     it read now"; this answers "what is there", which is the question a host
     must not answer from a table of its own.

     One section per request: both together came to 273 bytes against
     MB_MAX_PDU's 253, and the writer's overflow flag turned that into an
     0x04 on the first live call. */
  if (rd_left(in) < 1U)
  {
    return CMD_ERR_LENGTH;
  }

  switch (rd_u8(in))
  {
    case CHANNELS_ANALOG:     return analog_rows(out);
    case CHANNELS_DIGITAL:    return pin_page(in, out, true);
    case CHANNELS_RESERVED:   return pin_page(in, out, false);
    case CHANNELS_SUBSYSTEMS: return subsystem_rows(out);
    case CHANNELS_PARTS:      return parts_page(in, out);
    default:                  return CMD_ERR_VALUE;
  }
}


/* The whole command set, in one place. Adding a command is one row and one
   function; there is no switch to keep in step and no registration call. */
static const cmd_desc_t CMD_TABLE[] =
{
  { CMD_VERSION,    "version",    0U,               h_version    },
  /* Variable: an optional start index, so the table is not bounded by
     one reply. Absent reads 0, which is what every host sent before. */
  { CMD_ADC_TABLE,  "adc_table",  CMD_LEN_VARIABLE, h_adc_table  },
  { CMD_ADC_SCAN,   "adc_scan",   0U,               h_adc_scan   },
  { CMD_ADC_NOISE,  "adc_noise",  3U,               h_adc_noise  },
  { CMD_CLOCK,      "clock",      0U,               h_clock      },
  { CMD_AFE,        "afe",        1U,               h_afe        },
  { CMD_LINK_STATS, "link_stats", 0U,               h_link_stats },
  { CMD_ANALOG_BURST, "analog_burst", 8U,          h_analog_burst },
  { CMD_SELF_TEST,  "self_test",  0U,               h_self_test  },
  /* Variable: the kind byte, and a start index behind it for the paged
     sections. */
  { CMD_CHANNELS,   "channels",   CMD_LEN_VARIABLE, h_channels   },
  { CMD_CONSOLE,    "console",    0U,               h_console    },
};

const cmd_desc_t *cmd_board_table(uint8_t *count)
{
  *count = (uint8_t)(sizeof(CMD_TABLE) / sizeof(CMD_TABLE[0]));
  return CMD_TABLE;
}
