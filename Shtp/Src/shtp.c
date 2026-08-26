/**
  ******************************************************************************
  * @file    shtp.c
  * @brief   CEVA SHTP framing and SH-2 report decoding. No hardware.
  ******************************************************************************
  */
#include "shtp.h"

#include <string.h>

/* Little-endian on this wire, unlike Modbus. The BNO08X is byte oriented and
   every multi-byte field in SHTP and SH-2 is LSB first - Figure 1-26 for the
   header, Figure 5-1 for the report interval, Figure 1-34 for the axes. */
static uint16_t rd_u16(const uint8_t *p)
{
  return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t rd_u32(const uint8_t *p)
{
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
         ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
  p[0] = (uint8_t)v;
  p[1] = (uint8_t)(v >> 8);
  p[2] = (uint8_t)(v >> 16);
  p[3] = (uint8_t)(v >> 24);
}

bool shtp_parse_header(const uint8_t *raw, shtp_header_t *out)
{
  if ((raw == NULL) || (out == NULL))
  {
    return false;
  }

  const uint16_t field = rd_u16(raw);

  /* Not a length. The datasheet reserves it precisely because an unpowered or
     wedged peripheral holds MISO high and every read comes back 0xFF. */
  if (field == SHTP_LENGTH_RESERVED)
  {
    return false;
  }

  out->continuation = (field & SHTP_CONTINUATION) != 0U;
  out->length       = (uint16_t)(field & SHTP_LENGTH_MASK);
  out->channel      = raw[2];
  out->seq          = raw[3];

  /* The length counts its own header. Anything below that is not a short
     cargo, it is a header that disagrees with itself. A length of exactly
     zero is what an idle BNO08X clocks out and is left to the caller. */
  return (out->length == 0U) || (out->length >= SHTP_HEADER_LEN);
}

size_t shtp_build(uint8_t *buf, size_t cap, uint8_t channel, uint8_t seq,
                  const uint8_t *payload, size_t len)
{
  const size_t total = SHTP_HEADER_LEN + len;

  if ((buf == NULL) || (total > cap) || (total > SHTP_LENGTH_MASK))
  {
    return 0U;
  }

  buf[0] = (uint8_t)total;
  buf[1] = (uint8_t)(total >> 8);
  buf[2] = channel;
  buf[3] = seq;

  if (len > 0U)
  {
    if (payload == NULL)
    {
      return 0U;
    }
    memcpy(&buf[SHTP_HEADER_LEN], payload, len);
  }

  return total;
}

bool shtp_parse_product_id(const uint8_t *cargo, size_t len,
                           shtp_product_id_t *out)
{
  /* Figure 1-29: sixteen bytes, and every field after the id is fixed. A
     short one is a truncated read, not a device with less to say. */
  if ((cargo == NULL) || (out == NULL) || (len < 16U) ||
      (cargo[0] != SH2_PRODUCT_ID_RESPONSE))
  {
    return false;
  }

  out->reset_cause = cargo[1];
  out->sw_major    = cargo[2];
  out->sw_minor    = cargo[3];
  out->sw_part     = rd_u32(&cargo[4]);
  out->sw_build    = rd_u32(&cargo[8]);
  out->sw_patch    = rd_u16(&cargo[12]);

  return true;
}

size_t shtp_report_len(uint8_t report_id)
{
  switch (report_id)
  {
    /* Figure 5-2, bytes 4..8: id plus a signed 32-bit base delta. */
    case SH2_REPORT_TIMEBASE:
      return 5U;

    /* Ten bytes: id, sequence, status, delay, then three little-endian axes.
       Measured against the datasheet twice over - Figure 1-34 gives the
       calibrated gyroscope report byte by byte, and Figure 5-2 gives the
       accelerometer inside a cargo whose length field (19) only adds up if
       the report is ten bytes. */
    case SH2_REPORT_ACCELEROMETER:
    case SH2_REPORT_GYROSCOPE:
      return 10U;

    /* The same shape: "All input reports have a similar format", section
       1.3.5.2, and these three are three-axis calibrated sensors like the two
       above. Inferred from that sentence rather than tabulated, which is why
       they are listed apart from the two that are. */
    case SH2_REPORT_MAGNETIC_FIELD:
    case SH2_REPORT_LINEAR_ACCEL:
    case SH2_REPORT_GRAVITY:
      return 10U;

    /* The quaternion reports. This datasheet does not tabulate them - it
       refers to the SH-2 Reference Manual - so these come from CEVA's own
       decoder, github.com/ceva-dsp/sh2, sh2_SensorValue.c: the rotation
       vector carries i, j, k, real and an accuracy estimate behind the same
       four-byte header, and the game rotation vector carries the four
       components without it. */
    case SH2_REPORT_ROTATION_VECTOR:
      return 14U;

    case SH2_REPORT_GAME_ROTATION:
      return 12U;

    /* Everything else. Reports are packed back to back and are not
       self-delimiting, so a length nobody checked mis-frames every byte
       after it: the walk stops rather than guessing. */
    default:
      return 0U;
  }
}

size_t shtp_parse_reports(const uint8_t *cargo, size_t len,
                          shtp_report_t *out, size_t max)
{
  size_t at = 0U;
  size_t n = 0U;

  if ((cargo == NULL) || (out == NULL))
  {
    return 0U;
  }

  while ((at < len) && (n < max))
  {
    const uint8_t id = cargo[at];
    const size_t  step = shtp_report_len(id);

    if ((step == 0U) || ((at + step) > len))
    {
      break;
    }

    if (id == SH2_REPORT_TIMEBASE)
    {
      /* Carried once at the head of a cargo and not a sensor reading. The
         caller gets it as a report with no axes so the base delta is not
         silently dropped; see the host for what it is subtracted from. */
      out[n].report_id = id;
      out[n].seq = 0U;
      out[n].status = 0U;
      out[n].delay = 0U;
      out[n].x = 0;
      out[n].y = 0;
      out[n].z = 0;
      out[n].w = 0;
      out[n].count = 0U;
      n++;
      at += step;
      continue;
    }

    out[n].report_id = id;
    out[n].seq       = cargo[at + 1U];
    out[n].status    = cargo[at + 2U];
    out[n].delay     = cargo[at + 3U];
    out[n].x = (int16_t)rd_u16(&cargo[at + 4U]);
    out[n].y = (int16_t)rd_u16(&cargo[at + 6U]);
    out[n].z = (int16_t)rd_u16(&cargo[at + 8U]);
    out[n].w = 0;
    out[n].count = 3U;

    n++;
    at += step;
  }

  return n;
}

size_t shtp_set_feature(uint8_t *buf, size_t cap, uint8_t report_id,
                        uint32_t interval_us)
{
  /* Figure 1-33: seventeen bytes, and the worked example in Figure 5-1 shows
     them under a header whose length field reads 0x15 - four plus seventeen.
     Everything this firmware does not set is zero: no change sensitivity, no
     batching, no sensor-specific word. */
  if ((buf == NULL) || (cap < 17U))
  {
    return 0U;
  }

  memset(buf, 0, 17U);
  buf[0] = SH2_SET_FEATURE_COMMAND;
  buf[1] = report_id;
  wr_u32(&buf[5], interval_us);

  return 17U;
}
