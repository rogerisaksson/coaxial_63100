/**
  ******************************************************************************
  * @file    wire.c
  * @brief   Total accessors for binary payloads. See wire.h for the rationale.
  ******************************************************************************
  */
#include "wire.h"

#include <string.h>

void wr_init(wr_t *w, uint8_t *buf, uint16_t cap)
{
  w->buf = buf;
  w->cap = cap;
  w->len = 0U;
  w->bad = false;
}

/* One bounds check, one place. Everything else routes through here. */
void wr_bytes(wr_t *w, const void *src, uint16_t n)
{
  if (w->bad)
  {
    return;
  }

  if ((uint32_t)w->len + (uint32_t)n > (uint32_t)w->cap)
  {
    w->bad = true;
    return;
  }

  memcpy(&w->buf[w->len], src, n);
  w->len = (uint16_t)(w->len + n);
}

void wr_u8(wr_t *w, uint8_t v)
{
  wr_bytes(w, &v, 1U);
}

void wr_u16(wr_t *w, uint16_t v)
{
  const uint8_t b[2] = { (uint8_t)(v >> 8), (uint8_t)v };
  wr_bytes(w, b, 2U);
}

void wr_u32(wr_t *w, uint32_t v)
{
  const uint8_t b[4] = { (uint8_t)(v >> 24), (uint8_t)(v >> 16),
                         (uint8_t)(v >> 8),  (uint8_t)v };
  wr_bytes(w, b, 4U);
}

void wr_i16(wr_t *w, int16_t v)
{
  wr_u16(w, (uint16_t)v);
}

void wr_i32(wr_t *w, int32_t v)
{
  wr_u32(w, (uint32_t)v);
}

void wr_str(wr_t *w, const char *s)
{
  size_t n = (s == NULL) ? 0U : strlen(s);

  if (n > 255U)
  {
    n = 255U;
  }

  wr_u8(w, (uint8_t)n);
  wr_bytes(w, s, (uint16_t)n);
}

void rd_init(rd_t *r, const uint8_t *buf, uint16_t len)
{
  r->buf = buf;
  r->len = len;
  r->pos = 0U;
  r->bad = false;
}

/* Mirror of wr_bytes: one bounds check for the whole reader. */
static bool rd_take(rd_t *r, uint16_t n, const uint8_t **at)
{
  if (r->bad)
  {
    return false;
  }

  if ((uint32_t)r->pos + (uint32_t)n > (uint32_t)r->len)
  {
    r->bad = true;
    return false;
  }

  *at = &r->buf[r->pos];
  r->pos = (uint16_t)(r->pos + n);
  return true;
}

uint8_t rd_u8(rd_t *r)
{
  const uint8_t *p;
  return rd_take(r, 1U, &p) ? p[0] : 0U;
}

uint16_t rd_u16(rd_t *r)
{
  const uint8_t *p;
  return rd_take(r, 2U, &p) ? (uint16_t)(((uint16_t)p[0] << 8) | p[1]) : 0U;
}

uint32_t rd_u32(rd_t *r)
{
  const uint8_t *p;
  if (!rd_take(r, 4U, &p))
  {
    return 0U;
  }
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
         ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}

int32_t rd_i32(rd_t *r)
{
  return (int32_t)rd_u32(r);
}
