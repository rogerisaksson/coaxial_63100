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
#include "cmd.h"
#include "board.h"
#include "shtp.h"

/* What one Modbus reply can carry. The part's advertisement is longer than
   this - 276 bytes, measured - so a cargo that big arrives truncated and the
   board layer drops the rest rather than leaving it to desynchronise the
   next read. A bring-up asks for reports, which fit. */
#define IMU_CARGO 200U

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
     initialisation and room to spare. */
  for (uint8_t tries = 0U; tries < 8U; tries++)
  {
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

  uint8_t payload[17];

  if (shtp_set_feature(payload, sizeof(payload), report_id, interval) == 0U)
  {
    return CMD_ERR_DEVICE;
  }

  if (!Board_ImuWrite(SHTP_CH_CONTROL, payload, sizeof(payload)))
  {
    return CMD_ERR_DEVICE;
  }

  return CMD_OK;
}

/**
  * @brief 0x6E - every IMU operation, chosen by the first payload byte.
  *
  * One function code because it is the only one left: the specification's
  * user-defined ranges are 65..72 and 100..110, and this board had spent all
  * but 110. A second code answered ILLEGAL FUNCTION from the protocol layer
  * before dispatch saw it.
  */
/**
  * @brief op 3 - the four header bytes, unparsed.
  *
  * The bring-up question the parser cannot answer, because it refuses both
  * answers: 0xFF FF FF FF is a part that is absent, unpowered or in reset,
  * and 00 00 00 00 is one that is present and idle.
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

  if (!Board_ImuProbe(raw, len))
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
  wr_u8(out, Board_ImuDrain(8U));

  return CMD_OK;
}

static cmd_status_t h_imu(rd_t *in, wr_t *out)
{
  const uint8_t op = rd_u8(in);

  switch (op)
  {
    case IMU_OP_ID:      return h_imu_id(in, out);
    case IMU_OP_READ:    return h_imu_read(in, out);
    case IMU_OP_FEATURE: return h_imu_feature(in, out);
    case IMU_OP_PROBE:   return h_imu_probe(in, out);
    case IMU_OP_RESET:   return h_imu_reset(in, out);
    default:             return CMD_ERR_VALUE;
  }
}

static const cmd_desc_t IMU_TABLE[] =
{
  { CMD_IMU, "imu", CMD_LEN_VARIABLE, h_imu },
};

const cmd_desc_t *cmd_imu_table(uint8_t *count)
{
  *count = (uint8_t)(sizeof(IMU_TABLE) / sizeof(IMU_TABLE[0]));
  return IMU_TABLE;
}
