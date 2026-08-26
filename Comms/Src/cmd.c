/**
  ******************************************************************************
  * @file    cmd.c
  * @brief   Command dispatch. Generic: the table lives in cmd_board.c.
  ******************************************************************************
  */
#include "cmd.h"

/* Every table, in one list. Adding a table is one line here and nothing else:
   lookup, counting and listing all walk this. */
typedef const cmd_desc_t *(*cmd_table_fn)(uint8_t *count);

typedef struct
{
  cmd_table_fn fn;
  const char  *name;
  const char  *what;
} cmd_table_desc_t;

/* Each table is a subsystem, named here so the board can say what it is made
   of. Keep `what` short: the whole list has to fit one PDU beside the rest of
   the channel map. */
static const cmd_table_desc_t TABLES[] =
{
  { cmd_board_table, "board", "ADC channels, digital I/O, clocks, self test" },
  { cmd_test_table,  "testrig", "gated raw pin access for a fixture" },
  { cmd_imu_table,   "imu",   "BNO08X on SPI2 over SHTP" },
};

#define CMD_TABLE_COUNT (sizeof(TABLES) / sizeof(TABLES[0]))

uint8_t cmd_group_count(void)
{
  return (uint8_t)CMD_TABLE_COUNT;
}

const cmd_group_t *cmd_group(uint8_t index)
{
  /* Filled on demand rather than held as a second table: the command count
     is the table's own and would go stale the moment one is added. */
  static cmd_group_t group;

  if (index >= CMD_TABLE_COUNT)
  {
    return NULL;
  }

  uint8_t n = 0U;
  (void)TABLES[index].fn(&n);

  group.name = TABLES[index].name;
  group.what = TABLES[index].what;
  group.commands = n;

  return &group;
}

uint16_t cmd_count(void)
{
  uint16_t total = 0U;

  for (size_t t = 0U; t < CMD_TABLE_COUNT; t++)
  {
    uint8_t n = 0U;
    (void)TABLES[t].fn(&n);
    total = (uint16_t)(total + n);
  }

  return total;
}

const cmd_desc_t *cmd_at(uint16_t index)
{
  for (size_t t = 0U; t < CMD_TABLE_COUNT; t++)
  {
    uint8_t n = 0U;
    const cmd_desc_t *tab = TABLES[t].fn(&n);

    if (index < (uint16_t)n)
    {
      return &tab[index];
    }

    index = (uint16_t)(index - n);
  }

  return NULL;
}

const cmd_desc_t *cmd_find(uint8_t code)
{
  const uint16_t n = cmd_count();

  for (uint16_t i = 0U; i < n; i++)
  {
    const cmd_desc_t *d = cmd_at(i);

    if (d->code == code)
    {
      return d;
    }
  }

  return NULL;
}

/* Guard clauses all the way down, one level of nesting, no else. */
cmd_status_t cmd_dispatch(uint8_t code, const uint8_t *req, uint16_t req_len,
                          uint8_t *rsp, uint16_t rsp_cap, uint16_t *rsp_len)
{
  *rsp_len = 0U;

  const cmd_desc_t *d = cmd_find(code);

  if (d == NULL)
  {
    return CMD_ERR_UNKNOWN;
  }

  if ((d->req_len != CMD_LEN_VARIABLE) && (req_len != (uint16_t)d->req_len))
  {
    return CMD_ERR_LENGTH;
  }

  rd_t in;
  wr_t out;

  rd_init(&in, req, req_len);
  wr_init(&out, rsp, rsp_cap);

  const cmd_status_t st = d->fn(&in, &out);

  if (st != CMD_OK)
  {
    return st;
  }

  /* A handler that read past its request was given a malformed one; a handler
     whose response did not fit is a firmware bug, and both are better reported
     than half answered. */
  if (!rd_ok(&in))
  {
    return CMD_ERR_LENGTH;
  }

  if (!wr_ok(&out))
  {
    return CMD_ERR_DEVICE;
  }

  *rsp_len = wr_len(&out);
  return CMD_OK;
}
