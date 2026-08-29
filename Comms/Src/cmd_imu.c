/**
  ******************************************************************************
  * @file    cmd_imu.c
  * @brief   The BNO08X commands: raw cargo out, and the one request that
  *          proves the link is real.
  *
  * Nothing here decides whether a reading is good, and nothing scales one.
  * The IMU reports fixed-point counts whose Q point belongs to the report id,
  * and turning them into m/s^2 or radians is the host's job - invariant 10,
  * the same rule the ADC channels keep.
  ******************************************************************************
  */
#include <string.h>
#include "board_limits.h"
#include "cmd.h"
#include "board.h"
#include "shtp.h"

/**
  * @brief op 0 - ask the part what it is.
  *
  * Sends a Product ID Request (0xF9) on the SH-2 control channel and reads
  * until the matching 0xF8 comes back. The part answers other things first -
  * after reset it advertises on channel 0 and the executable announces itself
  * on channel 1 (section 5.2.1) - so the read loop skips what it did not ask
  * for rather than failing on it.
  */
static cmd_status_t h_imu_id(rd_t *in, wr_t *out)
{
  (void)in;      /* the operation byte was the whole request */

  if (!Board_ImuReady() && !Board_ImuInit())
  {
    return CMD_ERR_DEVICE;
  }

  const uint8_t request[2] = { SH2_PRODUCT_ID_REQUEST, 0U };

  if (!Board_ImuWrite(SHTP_CH_CONTROL, request, sizeof(request)))
  {
    return CMD_ERR_DEVICE;
  }

  static uint8_t cargo[IMU_CARGO];
  uint8_t  channel = 0U;
  uint16_t len = 0U;
  shtp_product_id_t id;

  /* Bounded: a part that never answers must not hold the link. Eight reads is
     the advertisement, the executable's reset message, SH-2's unsolicited
     initialisation and room to spare.

     WAIT BEFORE EACH ONE. Eight immediate reads take microseconds and the
     part needs milliseconds to produce the answer, so with the queue already
     drained - which is the normal state, the poll loop drains it - all eight
     came back empty and this returned SERVER DEVICE FAILURE on a part that
     was answering perfectly. Measured from the host: the same request sent
     by hand and read 15 ms later got f8 04 03 02, the product id response,
     every time. */
  for (uint8_t tries = 0U; tries < 8U; tries++)
  {
    (void)Board_ImuWaitReady(IMU_ANSWER_WAIT_MS);

    if (!Board_ImuRead(&channel, cargo, sizeof(cargo), &len))
    {
      return CMD_ERR_DEVICE;
    }

    if ((len == 0U) || (channel != SHTP_CH_CONTROL))
    {
      continue;
    }

    if (shtp_parse_product_id(cargo, len, &id))
    {
      wr_u8(out, id.reset_cause);
      wr_u8(out, id.sw_major);
      wr_u8(out, id.sw_minor);
      wr_u32(out, id.sw_part);
      wr_u32(out, id.sw_build);
      wr_u16(out, id.sw_patch);
      return CMD_OK;
    }
  }

  /* Reached the part but never got the answer. A device failure, not a bad
     request: the host asked a well-formed question. */
  return CMD_ERR_DEVICE;
}

/**
  * @brief op 1 - one SHTP cargo, exactly as it arrived.
  *
  * The channel, then the cargo bytes with no header and no interpretation.
  * A length of zero means the part had nothing waiting, which is the normal
  * answer from an IMU nobody has configured yet and is not an error.
  */
static cmd_status_t h_imu_read(rd_t *in, wr_t *out)
{
  (void)in;      /* the operation byte was the whole request */

  if (!Board_ImuReady() && !Board_ImuInit())
  {
    return CMD_ERR_DEVICE;
  }

  static uint8_t cargo[IMU_CARGO];
  uint8_t  channel = 0U;
  uint16_t len = 0U;

  if (!Board_ImuRead(&channel, cargo, sizeof(cargo), &len))
  {
    return CMD_ERR_DEVICE;
  }

  wr_u8(out, channel);
  wr_u8(out, (uint8_t)len);
  wr_bytes(out, cargo, len);

  return CMD_OK;
}

/**
  * @brief op 2 - enable or disable one sensor report.
  *
  * report_id, then the interval in microseconds. Zero disables it, which is
  * what the Set Feature command means by a period of nothing (Figure 1-33).
  * The rate the part actually adopts may differ from the one asked for; it
  * says so in a Get Feature Response, which comes back through imu_read.
  */
static cmd_status_t h_imu_feature(rd_t *in, wr_t *out)
{
  const uint8_t  report_id = rd_u8(in);
  const uint32_t interval  = rd_u32(in);

  (void)out;

  if (!Board_ImuReady() && !Board_ImuInit())
  {
    return CMD_ERR_DEVICE;
  }

  /* Through the board layer, which remembers it: the part forgets on every
     reset and the poll re-applies it. Building the payload here as well
     would be a second answer to what this part is configured to do. */
  if (!Board_ImuSetFeature(report_id, interval))
  {
    return CMD_ERR_DEVICE;
  }

  return CMD_OK;
}

/**
  * @brief op 3 - the four header bytes, unparsed.
  *
  * The bring-up question the parser refuses to answer: FF FF FF FF is a part
  * absent, unpowered or in reset, and 00 00 00 00 one present and idle.
  */
static cmd_status_t h_imu_probe(rd_t *in, wr_t *out)
{
  static uint8_t raw[IMU_CARGO];
  uint8_t len = rd_u8(in);

  if (len == 0U)
  {
    len = 4U;                      /* the header, which is the usual question */
  }

  if (len > (uint8_t)sizeof(raw))
  {
    return CMD_ERR_VALUE;
  }

  const bool select = (rd_left(in) > 0U) ? (rd_u8(in) != 0U) : true;

  if (!Board_ImuProbe(raw, len, select))
  {
    return CMD_ERR_DEVICE;
  }

  uint32_t kernel = 0U;
  uint32_t bitrate = 0U;
  Board_ImuClock(&kernel, &bitrate);

  wr_u32(out, kernel);
  wr_u32(out, bitrate);
  wr_u8(out, len);
  wr_bytes(out, raw, len);
  return CMD_OK;
}

/**
  * @brief op 4 - pulse NRSTN and take what the part says on the way up.
  *
  * The one thing a bring-up cannot do without: a part that has stopped
  * streaming has no other way back, and re-flashing to get a reset is not a
  * diagnostic. Answers with how many cargoes the reset produced, which is
  * the advertisement and the two announcements when it worked.
  */
static cmd_status_t h_imu_reset(rd_t *in, wr_t *out)
{
  (void)in;

  if (!Board_ImuReady() && !Board_ImuInit())
  {
    return CMD_ERR_DEVICE;
  }

  Board_ImuReset();

  /* Enough to clear the advertisement, which is 276 bytes and arrives as
     several cargoes, plus the reset-complete and unsolicited product id
     behind it. Eight was not: measured 2026-08-27, a Set Feature sent
     straight after a reset that had drained eight never took - cargoes
     climbed and no rotation vector ever arrived - and the same write after
     the queue had actually emptied took first time. The part is still
     talking about itself while the write goes out, and a write nobody is
     listening to changes nothing. */
  wr_u8(out, Board_ImuDrain(48U));

  return CMD_OK;
}

/**
  * @brief op 5 - one cargo, exactly as given, on the channel named.
  *
  * The bring-up primitive the parsed operations cannot stand in for. Product
  * Id cannot tell a write that reached the part from one that did not: the
  * part sends an unsolicited Product Id Response after every reset, so the
  * answer is in the queue either way. A Get Feature Request is not - nothing
  * else makes the part emit a 0xFC - and this is what puts one on the wire.
  */
static cmd_status_t h_imu_write(rd_t *in, wr_t *out)
{
  static uint8_t payload[IMU_CARGO];
  const uint8_t channel = rd_u8(in);
  uint16_t len = 0U;

  (void)out;

  while ((rd_left(in) > 0U) && (len < (uint16_t)sizeof(payload)))
  {
    payload[len] = rd_u8(in);
    len++;
  }

  if (!rd_ok(in) || (len == 0U))
  {
    return CMD_ERR_VALUE;
  }

  if (!Board_ImuReady() && !Board_ImuInit())
  {
    return CMD_ERR_DEVICE;
  }

  if (!Board_ImuWrite(channel, payload, len))
  {
    return CMD_ERR_DEVICE;
  }

  return CMD_OK;
}

/**
  * @brief op 6 - drive one of SPI2's pins and read it back.
  *
  * What a write that the part never acts on needs next: reads work, chip
  * select is proven from the inside, so the question is whether anything is
  * holding SPI2's own pins. Answers per pin, not per bus, because the fault
  * this looks for is one net.
  */
static cmd_status_t h_imu_pins(rd_t *in, wr_t *out)
{
  static const uint8_t PINS[4] = { 12U, 13U, 14U, 15U };

  (void)in;

  for (uint8_t i = 0U; i < 4U; i++)
  {
    wr_u8(out, PINS[i]);
    wr_u8(out, Board_ImuPinCheck(PINS[i]));
  }

  return CMD_OK;
}

/**
  * @brief op 7 - does the part answer a wake, and how fast.
  *
  * The measurement that separates a write nobody acts on from a write that
  * never went out. Also reports PS0/WAKE's own readback, so a line the MCU
  * cannot drive is not mistaken for a part that will not answer.
  */
static cmd_status_t h_imu_wake(rd_t *in, wr_t *out)
{
  const uint16_t ms = (rd_left(in) >= 2U) ? rd_u16(in) : 200U;

  wr_u16(out, Board_ImuWakeTest(ms));
  return CMD_OK;
}

/**
  * @brief op 8 - the poll loop's shared record. Touches no SPI: a cargo per
  * request cost 45 ms and caught one frame in eight. `updates` is monotonic,
  * so the same reading read twice is telling. Counts, not radians.
  */
static cmd_status_t h_imu_latest(rd_t *in, wr_t *out)
{
  board_imu_state_t st;

  (void)in;

  Board_ImuState(&st);

  wr_u8(out, st.loop);
  wr_u8(out, st.error);
  wr_u32(out, st.updates);
  wr_u32(out, st.cargoes);
  wr_u32(out, st.errors);
  wr_u8(out, st.have ? 1U : 0U);

  if (st.have)
  {
    wr_u8(out, st.report_id);
    wr_u8(out, st.status);
    wr_u16(out, (uint16_t)st.i);
    wr_u16(out, (uint16_t)st.j);
    wr_u16(out, (uint16_t)st.k);
    wr_u16(out, (uint16_t)st.real);
  }


  /* What the part was asked to report, and whether that request still has
     to be re-made. Appended: a reset throws the feature away and the poll
     re-applies it, and there was no way to see which side of that it was
     on - the loop said `running` with `updates` frozen either way. */
  uint8_t asked_id = 0U;
  uint32_t asked_us = 0U;
  bool asked_pending = false;

  Board_ImuFeatureAsked(&asked_id, &asked_us, &asked_pending);
  wr_u8(out, asked_id);
  wr_u32(out, asked_us);
  wr_u8(out, asked_pending ? 1U : 0U);

  /* Appended, like everything here. `error` above is cleared by the next
     good read, so at 400 reports a second a host polling at 5 Hz sees the
     counter climb and never learns of what. */
  wr_u8(out, st.last_fault);
  wr_u8(out, st.last_fault_id);
  return CMD_OK;
}

/**
  * @brief ops 9 and 10 - stop the poll loop, and start it again.
  *
  * Configuring the part while the loop runs is two masters on one SPI bus.
  * Hold, configure, resume; a resume goes back through init, because the
  * usual reason to have held it was a reset.
  */
static cmd_status_t h_imu_hold(rd_t *in, wr_t *out)
{
  board_imu_state_t st;

  (void)in;

  Board_ImuHold();
  Board_ImuState(&st);
  wr_u8(out, st.loop);

  return CMD_OK;
}

static cmd_status_t h_imu_resume(rd_t *in, wr_t *out)
{
  board_imu_state_t st;

  (void)in;

  Board_ImuResume();
  Board_ImuState(&st);
  wr_u8(out, st.loop);

  return CMD_OK;
}

cmd_status_t cmd_imu_op(uint8_t op, rd_t *in, wr_t *out)
{
  /* Everything below drives SPI2 itself, and the poll loop drives it from
     the main loop: running both is two masters on one bus, and what that
     looks like is a cargo split between them and a stream that stops. Hold
     the loop, configure, resume. Reading the shared record needs no hold,
     which is the whole point of there being one. */
  if ((op != IMU_OP_LATEST) && (op != IMU_OP_HOLD) && (op != IMU_OP_RESUME))
  {
    board_imu_state_t st;

    Board_ImuState(&st);

    if (st.loop != BOARD_IMU_LOOP_HELD)
    {
      return CMD_ERR_DEVICE;
    }
  }

  switch (op)
  {
    case IMU_OP_ID:      return h_imu_id(in, out);
    case IMU_OP_READ:    return h_imu_read(in, out);
    case IMU_OP_FEATURE: return h_imu_feature(in, out);
    case IMU_OP_PROBE:   return h_imu_probe(in, out);
    case IMU_OP_RESET:   return h_imu_reset(in, out);
    case IMU_OP_WRITE:   return h_imu_write(in, out);
    case IMU_OP_PINS:    return h_imu_pins(in, out);
    case IMU_OP_WAKE:    return h_imu_wake(in, out);
    case IMU_OP_LATEST:  return h_imu_latest(in, out);
    case IMU_OP_HOLD:    return h_imu_hold(in, out);
    case IMU_OP_RESUME:  return h_imu_resume(in, out);
    default:             return CMD_ERR_VALUE;
  }
}
