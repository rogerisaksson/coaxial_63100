/**
  ******************************************************************************
  * @file    wire.h
  * @brief   Append-only writer and forward-only reader for binary payloads.
  *
  * Keeps command handlers flat. Every accessor is total: the writer drops an
  * overflowing write and the reader returns zero on underrun, both setting a
  * sticky flag. So a handler is a straight run of statements with one check at
  * the end, not an if around every field.
  *
  * Big-endian, as Modbus is for every field but its CRC. No floats on the
  * wire; scaled integers, in the units the emitting command names.
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
