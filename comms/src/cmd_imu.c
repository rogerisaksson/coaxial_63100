/** cmd_imu.c - Device 0: the BNO08X. */
#include <string.h>
#include "comms_limits.h"
#include "cmd.h"
#include "board.h"
#include "shtp.h"

#define ANSWER_TRIES    8U     /* reads given a request before it is called unanswered */
#define DRAIN_LIMIT     48U    /* op 4's drain limit */
#define WAKE_DEFAULT_MS 200U

/** op 0 - ask the part what it is. */
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

  /* Bounded: a part that never answers must not hold the link. */
  for (uint8_t tries = 0U; tries < ANSWER_TRIES; tries++)
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

  /* Reached the part but never got the answer. */
  return CMD_ERR_DEVICE;
}

/** op 1 - one SHTP cargo, exactly as it arrived. */
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

/** op 2 - enable or disable one sensor report. */
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
     reset and the poll re-applies it. */
  if (!Board_ImuSetFeature(report_id, interval))
  {
    return CMD_ERR_DEVICE;
  }

  return CMD_OK;
}

/** op 3 - the four header bytes, unparsed. */
static cmd_status_t h_imu_probe(rd_t *in, wr_t *out)
{
  static uint8_t raw[IMU_CARGO];
  uint8_t len = rd_u8(in);

  if (len == 0U)
  {
    len = SHTP_HEADER_LEN;         /* the usual question */
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

/** op 4 - pulse NRSTN and take what the part says on the way up. */
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
     behind it. */
  wr_u8(out, Board_ImuDrain(DRAIN_LIMIT));

  return CMD_OK;
}

/** op 5 - one cargo, exactly as given, on the channel named. */
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

/** op 6 - drive one of SPI2's pins and read it back. */
static cmd_status_t h_imu_pins(rd_t *in, wr_t *out)
{
  (void)in;

  for (uint8_t i = 0U; i < BOARD_IMU_SPI_PIN_COUNT; i++)
  {
    const uint8_t pin = (uint8_t)(BOARD_IMU_SPI_PIN_FIRST + i);

    wr_u8(out, pin);
    wr_u8(out, Board_ImuPinCheck(pin));
  }

  return CMD_OK;
}

/** op 7 - does the part answer a wake, and how fast. */
static cmd_status_t h_imu_wake(rd_t *in, wr_t *out)
{
  const uint16_t ms = (rd_left(in) >= 2U) ? rd_u16(in) : WAKE_DEFAULT_MS;

  wr_u16(out, Board_ImuWakeTest(ms));
  return CMD_OK;
}

/** op 8 - the poll loop's shared record. Touches no SPI: a cargo per */
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

  /* What the part was asked to report, and whether that request still has to
     be re-made. */
  uint8_t asked_id = 0U;
  uint32_t asked_us = 0U;
  bool asked_pending = false;

  Board_ImuFeatureAsked(&asked_id, &asked_us, &asked_pending);
  wr_u8(out, asked_id);
  wr_u32(out, asked_us);
  wr_u8(out, asked_pending ? 1U : 0U);

  /* Appended, like everything here. */
  wr_u8(out, st.last_fault);
  wr_u8(out, st.last_fault_id);

  /* The three vectors, appended like everything else here. */
  wr_u8(out, st.have_accel ? 1U : 0U);
  wr_u8(out, st.accel_status);
  wr_u16(out, (uint16_t)st.accel[0]);
  wr_u16(out, (uint16_t)st.accel[1]);
  wr_u16(out, (uint16_t)st.accel[2]);

  wr_u8(out, st.have_gyro ? 1U : 0U);
  wr_u8(out, st.gyro_status);
  wr_u16(out, (uint16_t)st.gyro[0]);
  wr_u16(out, (uint16_t)st.gyro[1]);
  wr_u16(out, (uint16_t)st.gyro[2]);

  wr_u8(out, st.have_mag ? 1U : 0U);
  wr_u8(out, st.mag_status);
  wr_u16(out, (uint16_t)st.mag[0]);
  wr_u16(out, (uint16_t)st.mag[1]);
  wr_u16(out, (uint16_t)st.mag[2]);
  return CMD_OK;
}

/** ops 9 and 10 - stop the poll loop, and start it again. */
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

/* Whether the host holds the part's loop - what everything but a read of the
   shared record, a hold and a resume needs. */
static bool imu_held(void)
{
  board_imu_state_t st;

  Board_ImuState(&st);
  return st.loop == BOARD_IMU_LOOP_HELD;
}

cmd_status_t cmd_imu_op(uint8_t op, rd_t *in, wr_t *out)
{
  /* Everything below drives SPI2 itself, and the poll loop drives it from
     the main loop: running both is two masters on one bus, and what that
     looks like is a cargo split between them and a stream that stops. */
  if ((op != IMU_OP_LATEST) && (op != IMU_OP_HOLD) && (op != IMU_OP_RESUME)
      && !imu_held())
  {
    return CMD_ERR_DEVICE;
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
