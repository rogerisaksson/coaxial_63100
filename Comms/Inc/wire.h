/**
  ******************************************************************************
  * @file    wire.h
  * @brief   Append-only writer and forward-only reader for binary payloads.
  *
  * This exists to keep the command handlers flat. Every accessor is total: on
  * overflow the writer sets a sticky flag and drops the write, and on underrun
  * the reader sets a sticky flag and returns zero. Nothing fails mid-sequence,
  * so a handler is a straight run of statements with ONE check at the end
  * instead of an if around every field.
  *
  * Big-endian on the wire, the same order Modbus uses for every field except
  * its CRC. Floats are never transmitted; scaled integers are, in units named
  * by the command that emits them.
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
void wr_u16(wr_t *w, uint16_t v);
void wr_u32(wr_t *w, uint32_t v);
void wr_i16(wr_t *w, int16_t v);
void wr_i32(wr_t *w, int32_t v);
void wr_bytes(wr_t *w, const void *src, uint16_t n);

/** Length-prefixed ASCII: one length byte then the characters, no terminator. */
void wr_str(wr_t *w, const char *s);

static inline bool wr_ok(const wr_t *w) { return !w->bad; }
static inline uint16_t wr_len(const wr_t *w) { return w->len; }
/** Bytes still free. For a writer that must size a list before it
  * writes the count that leads it. */
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
uint16_t rd_u16(rd_t *r);
uint32_t rd_u32(rd_t *r);
int32_t  rd_i32(rd_t *r);

static inline bool rd_ok(const rd_t *r) { return !r->bad; }
static inline uint16_t rd_left(const rd_t *r) { return (uint16_t)(r->len - r->pos); }

#ifdef __cplusplus
}
#endif

#endif /* WIRE_H */
