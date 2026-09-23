/**
  ******************************************************************************
  * @file    wire.h
  * @brief   Append-only writer and forward-only reader for binary payloads.
  ******************************************************************************
  */
#ifndef WIRE_H
#define WIRE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t *buf;
  uint16_t cap;
  uint16_t len;
  bool     bad;   /**< sticky: a write did not fit */
} wr_t;

void wr_init(wr_t *w, uint8_t *buf, uint16_t cap);
void wr_u8(wr_t *w, uint8_t v);
void wr_i8(wr_t *w, int8_t v);       /**< two's complement, one byte */
void wr_u16(wr_t *w, uint16_t v);
void wr_u32(wr_t *w, uint32_t v);
void wr_i16(wr_t *w, int16_t v);
void wr_i32(wr_t *w, int32_t v);
void wr_bytes(wr_t *w, const void *src, uint16_t n);

/** Length-prefixed ASCII: one length byte then the characters, no terminator. */
void wr_str(wr_t *w, const char *s);
/** The took byte, as every op taking parameters answers: 1, or 0 and the
    board's words for what is wrong and what to do. */
void wr_took(wr_t *out, const char *refusal);

static inline bool wr_ok(const wr_t *w) { return !w->bad; }
static inline uint16_t wr_len(const wr_t *w) { return w->len; }
/** Bytes still free. */
static inline uint16_t wr_room(const wr_t *w)
{ return (w->cap > w->len) ? (uint16_t)(w->cap - w->len) : 0U; }

typedef struct
{
  const uint8_t *buf;
  uint16_t       len;
  uint16_t       pos;
  bool           bad;   /**< sticky: a read ran past the end */
} rd_t;

void     rd_init(rd_t *r, const uint8_t *buf, uint16_t len);
uint8_t  rd_u8(rd_t *r);
int8_t   rd_i8(rd_t *r);              /**< two's complement, one byte */
uint16_t rd_u16(rd_t *r);
uint32_t rd_u32(rd_t *r);
int32_t  rd_i32(rd_t *r);

/** `n` bytes off the reader, or NULL when fewer are left - the raw payload
    an op carries after its fields. */
const uint8_t *rd_bytes(rd_t *r, uint16_t n);
static inline bool rd_ok(const rd_t *r) { return !r->bad; }
static inline uint16_t rd_left(const rd_t *r) { return (uint16_t)(r->len - r->pos); }

#ifdef __cplusplus
}
#endif

#endif /* WIRE_H */
