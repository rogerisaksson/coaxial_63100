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

static const cmd_table_fn TABLES[] =
{
  cmd_board_table,
  cmd_test_table,
  cmd_imu_table,
};

#define CMD_TABLE_COUNT (sizeof(TABLES) / sizeof(TABLES[0]))

uint16_t cmd_count(void)
{
  uint16_t total = 0U;

  for (size_t t = 0U; t < CMD_TABLE_COUNT; t++)
  {
    uint8_t n = 0U;
    (void)TABLES[t](&n);
    total = (uint16_t)(total + n);
  }

  return total;
}

const cmd_desc_t *cmd_at(uint16_t index)
{
  for (size_t t = 0U; t < CMD_TABLE_COUNT; t++)
  {
    uint8_t n = 0U;
    const cmd_desc_t *tab = TABLES[t](&n);

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
