/**
  ******************************************************************************
  * @file    cmd_test.c
  * @brief   Test fixture commands: link echo and raw pin access.
  *
  * Same shape as cmd_board.c. The reader and writer are total, so each handler
  * is a straight run of statements; the only branches are the ones the rig can
  * genuinely get wrong.
  ******************************************************************************
  */
#include "cmd.h"
#include "testrig.h"

static cmd_status_t h_gate(rd_t *in, wr_t *out)
{
  const uint32_t key  = rd_u32(in);
  const uint8_t  open = rd_u8(in);

  if (open > 1U)
  {
    return CMD_ERR_VALUE;
  }

  if (!testrig_gate(key, open != 0U))
  {
    /* Wrong key. The gate keeps its previous state. */
    return CMD_ERR_VALUE;
  }

  wr_u8(out, testrig_open() ? 1U : 0U);
  return CMD_OK;
}

static cmd_status_t h_echo(rd_t *in, wr_t *out)
{
  const uint16_t n = rd_left(in);

  /* A response payload has 250 bytes of room once the unit id, function code
     and CRC are accounted for. Anything longer could not be echoed. */
  if (n > 250U)
  {
    return CMD_ERR_VALUE;
  }

  for (uint16_t i = 0U; i < n; i++)
  {
    wr_u8(out, rd_u8(in));
  }

  return CMD_OK;
}

static cmd_status_t h_pin_mode(rd_t *in, wr_t *out)
{
  const char    port = (char)rd_u8(in);
  const uint8_t pin  = rd_u8(in);
  const uint8_t mode = rd_u8(in);
  const uint8_t pull = rd_u8(in);

  (void)out;

  if (!testrig_pin_allowed(port, pin))
  {
    return CMD_ERR_VALUE;
  }

  if (!testrig_open())
  {
    return CMD_ERR_DEVICE;
  }

  if (!testrig_pin_mode(port, pin, mode, pull))
  {
    return CMD_ERR_VALUE;
  }

  return CMD_OK;
}

static cmd_status_t h_pin_read(rd_t *in, wr_t *out)
{
  const char    port = (char)rd_u8(in);
  const uint8_t pin  = rd_u8(in);
  bool          level = false;

  if (!testrig_pin_read(port, pin, &level))
  {
    return CMD_ERR_VALUE;
  }

  wr_u8(out, level ? 1U : 0U);
  return CMD_OK;
}

static cmd_status_t h_pin_write(rd_t *in, wr_t *out)
{
  const char    port  = (char)rd_u8(in);
  const uint8_t pin   = rd_u8(in);
  const uint8_t level = rd_u8(in);

  if ((level > 1U) || !testrig_pin_allowed(port, pin))
  {
    return CMD_ERR_VALUE;
  }

  if (!testrig_open())
  {
    return CMD_ERR_DEVICE;
  }

  if (!testrig_pin_write(port, pin, level != 0U))
  {
    return CMD_ERR_VALUE;
  }

  /* Read the pin back rather than echoing what was asked for. On an open-drain
     output or a pin held by the fixture, those differ - and that difference is
     exactly what a test rig is looking for. */
  bool actual = false;
  (void)testrig_pin_read(port, pin, &actual);

  wr_u8(out, actual ? 1U : 0U);
  return CMD_OK;
}

static cmd_status_t h_port_read(rd_t *in, wr_t *out)
{
  const char port = (char)rd_u8(in);
  uint16_t   idr  = 0U;

  if (!testrig_port_read(port, &idr))
  {
    return CMD_ERR_VALUE;
  }

  wr_u16(out, idr);
  return CMD_OK;
}

static cmd_status_t h_port_write(rd_t *in, wr_t *out)
{
  const char     port  = (char)rd_u8(in);
  const uint16_t mask  = rd_u16(in);
  const uint16_t value = rd_u16(in);
  uint16_t       idr   = 0U;

  if (!testrig_open())
  {
    return CMD_ERR_DEVICE;
  }

  if (!testrig_port_write(port, mask, value))
  {
    return CMD_ERR_VALUE;
  }

  (void)testrig_port_read(port, &idr);

  wr_u16(out, idr);
  return CMD_OK;
}

static const cmd_desc_t TEST_TABLE[] =
{
  { CMD_TEST_GATE,  "test_gate",  5U,               h_gate       },
  { CMD_ECHO,       "echo",       CMD_LEN_VARIABLE, h_echo       },
  { CMD_PIN_MODE,   "pin_mode",   4U,               h_pin_mode   },
  { CMD_PIN_READ,   "pin_read",   2U,               h_pin_read   },
  { CMD_PIN_WRITE,  "pin_write",  3U,               h_pin_write  },
  { CMD_PORT_READ,  "port_read",  1U,               h_port_read  },
  { CMD_PORT_WRITE, "port_write", 5U,               h_port_write },
};

const cmd_desc_t *cmd_test_table(uint8_t *count)
{
  *count = (uint8_t)(sizeof(TEST_TABLE) / sizeof(TEST_TABLE[0]));
  return TEST_TABLE;
}
