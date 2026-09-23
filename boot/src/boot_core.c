/**
  ******************************************************************************
  * @file    boot_core.c
  * @brief   Device 11: the bootloader's ops, over a port of four calls.
  ******************************************************************************
  */
#include "boot.h"

#include <string.h>

/** A chunk's index masks to a byte and a bit of the bitmap. */
#define BITS_PER_BYTE   8U
/** The IEEE polynomial, reflected, bit by bit: a table is 1 K the bootloader
    has no reason to carry for a sum it takes once. */
#define CRC32_POLY      0xEDB88320U
#define CRC32_INIT      0xFFFFFFFFU
/** A thumb address is odd. */
#define THUMB_BIT       1U

static struct
{
  const boot_port_t  *port;
  void               *ctx;
  boot_layout_t       layout;
  boot_state_t        state;
  uint32_t            session;
  uint8_t             unit;
  uint8_t             position;
  uint8_t             flags;
  uint32_t            size;              /**< the image's, from erase       */
  uint32_t            crc;               /**< the image's, from erase       */
  uint16_t            chunks;            /**< the image's, from erase       */
  uint32_t            held;              /**< chunks landed                 */
  uint8_t             bitmap[BOOT_BITMAP_BYTES];
  uint8_t             first[BOOT_WORD_BYTES];   /**< the word written last  */
  bool                first_held;
  uint8_t             record[BOOT_RECORD_MAX];
  uint16_t            record_bytes;
  bool                verified;
  bool                go;
  uint32_t            ignored;           /**< chunks with no erase behind   */
} s;

/* -- the wire's shorthand -------------------------------------------------- */

/* The took byte, as every device answers it: 1, or 0 and the words. */
static void took(wr_t *out, const char *refusal)
{
  wr_took(out, refusal);
}

static bool bit(uint16_t index)
{
  return (s.bitmap[index / BITS_PER_BYTE] & (uint8_t)(1U << (index % BITS_PER_BYTE))) != 0U;
}

static void set_bit(uint16_t index)
{
  s.bitmap[index / BITS_PER_BYTE] |= (uint8_t)(1U << (index % BITS_PER_BYTE));
}

static uint32_t read_u32(uint32_t address)
{
  const uint8_t *p = s.port->read(s.ctx, address);

  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

/* Whether the first `bits` of the request's prefix are this node's. */
static bool prefix_fits(uint8_t bits, const uint8_t *prefix)
{
  const uint8_t whole = bits / BITS_PER_BYTE;
  const uint8_t rest = bits % BITS_PER_BYTE;

  if (bits > BOOT_UID_BITS)
  {
    return false;
  }
  if (memcmp(prefix, s.layout.uid, whole) != 0)
  {
    return false;
  }
  if (rest == 0U)
  {
    return true;
  }
  const uint8_t mask = (uint8_t)(0xFFU << (BITS_PER_BYTE - rest));

  return ((prefix[whole] ^ s.layout.uid[whole]) & mask) == 0U;
}

/* CRC over `size` bytes of the image as it will stand: the first word from
   RAM until it is programmed, the rest from flash. */
static uint32_t image_crc(uint32_t size)
{
  uint32_t crc = CRC32_INIT;

  crc = boot_crc32(crc, s.first_held ? s.first : s.port->read(s.ctx, s.layout.app_base),
                   BOOT_WORD_BYTES);
  crc = boot_crc32(crc, s.port->read(s.ctx, s.layout.app_base + BOOT_WORD_BYTES),
                   size - BOOT_WORD_BYTES);
  return crc ^ CRC32_INIT;
}

/* Whether the image in flash is valid and is, byte for byte, the one an
   erase offers - the header's size and the crc over it. */
static bool image_held(uint32_t size, uint32_t crc)
{
  return boot_app_valid()
         && (read_u32(s.layout.app_base + BOOT_HEADER_OFFSET + 4U) == size)
         && (image_crc(size) == crc);
}

/* One flash word of the record as `seal` would program it: the bytes, then
   0xFF to the word's end. */
static void record_word(uint32_t offset, uint8_t *word)
{
  const uint32_t have = ((s.record_bytes - offset) < BOOT_WORD_BYTES)
                        ? (s.record_bytes - offset) : BOOT_WORD_BYTES;

  memset(word, 0xFF, BOOT_WORD_BYTES);
  memcpy(word, &s.record[offset], have);
}

/* Whether the record's sector already holds every word `seal` would program
   - then it is not erased and not programmed. */
static bool record_held(void)
{
  uint8_t word[BOOT_WORD_BYTES];

  for (uint32_t offset = 0U; offset < s.record_bytes; offset += BOOT_WORD_BYTES)
  {
    record_word(offset, word);
    if (memcmp(s.port->read(s.ctx, s.layout.record_base + offset), word, sizeof(word)) != 0)
    {
      return false;
    }
  }
  return true;
}

/* -- the ops --------------------------------------------------------------- */

static boot_answer_t h_boot_hold(rd_t *in, wr_t *out)
{
  const uint32_t session = rd_u32(in);

  (void)out;
  if (s.state == BOOT_BLANK)
  {
    s.state = BOOT_HELD;
  }
  s.session = session;
  return BOOT_SILENT;
}

static boot_answer_t h_boot_who(rd_t *in, wr_t *out)
{
  const uint8_t bits = rd_u8(in);
  const uint8_t *prefix = rd_bytes(in, (uint16_t)((bits + BITS_PER_BYTE - 1U) / BITS_PER_BYTE));

  if ((prefix == NULL) || !prefix_fits(bits, prefix))
  {
    return BOOT_SILENT;
  }
  wr_bytes(out, s.layout.uid, BOOT_UID_BYTES);
  wr_u8(out, s.layout.type);
  wr_u8(out, (uint8_t)s.state);
  wr_u8(out, s.unit);
  return BOOT_REPLY;
}

static boot_answer_t h_boot_assign(rd_t *in, wr_t *out)
{
  const uint8_t *uid = rd_bytes(in, BOOT_UID_BYTES);
  const uint8_t unit = rd_u8(in);
  const uint8_t position = rd_u8(in);
  const uint8_t flags = rd_u8(in);

  if (!rd_ok(in) || (memcmp(uid, s.layout.uid, BOOT_UID_BYTES) != 0))
  {
    return BOOT_SILENT;
  }
  if ((unit == 0U) || (unit >= BOOT_UNIT))
  {
    took(out, "a unit is 1..246 - 0 is broadcast and 247 is every blank node");
    return BOOT_REPLY;
  }
  s.unit = unit;
  s.position = position;
  s.flags = flags;
  s.state = BOOT_ASSIGNED;
  took(out, NULL);
  return BOOT_REPLY;
}

/* The image an erase offers, taken as the node's: held whole already, or
   erased and waiting for every chunk. */
static void offered(uint32_t size, uint32_t crc, uint16_t chunks, bool held)
{
  s.size = size;
  s.crc = crc;
  s.chunks = chunks;
  s.held = held ? chunks : 0U;
  memset(s.bitmap, held ? 0xFF : 0x00, sizeof(s.bitmap));
  memset(s.first, 0xFF, sizeof(s.first));
  s.first_held = false;
  s.verified = held;
  s.record_bytes = 0U;
  s.state = held ? BOOT_VERIFIED : BOOT_ERASED;
}

static boot_answer_t h_boot_erase(rd_t *in, wr_t *out)
{
  const uint8_t type = rd_u8(in);
  const uint32_t size = rd_u32(in);
  const uint32_t crc = rd_u32(in);
  const uint16_t chunks = rd_u16(in);

  (void)out;
  if ((type != s.layout.type) || (s.state < BOOT_ASSIGNED))
  {
    return BOOT_SILENT;             /* another type's image, or nobody's yet */
  }
  if ((size > s.layout.app_bytes) || (chunks > BOOT_MAX_CHUNKS)
      || (size < BOOT_WORD_BYTES)
      || (chunks != (size + BOOT_CHUNK_BYTES - 1U) / BOOT_CHUNK_BYTES))
  {
    s.port->say(s.ctx, "boot: an erase named an image that does not fit");
    return BOOT_SILENT;
  }
  if (image_held(size, crc))
  {
    /* The image offered is the one in flash: kept whole, every chunk of the
       stream a repeat, and verify will say so. */
    offered(size, crc, chunks, true);
    s.port->say(s.ctx, "boot: the image offered is the one held - kept");
    return BOOT_SILENT;
  }
  if (!s.port->erase(s.ctx, s.layout.app_base, size))
  {
    s.port->say(s.ctx, "boot: the sectors did not erase");
    return BOOT_SILENT;
  }
  offered(size, crc, chunks, false);
  return BOOT_SILENT;
}

/* One chunk's words programmed in place - the image's first word kept in RAM
   instead, and the tail of the last chunk clipped to the size. */
static bool program_chunk(uint16_t index, const uint8_t *data, uint16_t len)
{
  const uint32_t at = (uint32_t)index * BOOT_CHUNK_BYTES;
  uint8_t word[BOOT_WORD_BYTES];

  for (uint32_t offset = 0U; offset < len; offset += BOOT_WORD_BYTES)
  {
    const uint32_t have = ((len - offset) < BOOT_WORD_BYTES) ? (len - offset) : BOOT_WORD_BYTES;

    memset(word, 0xFF, sizeof(word));
    memcpy(word, data + offset, have);
    if ((at + offset) == 0U)
    {
      memcpy(s.first, word, sizeof(s.first));
      s.first_held = true;
      continue;
    }
    if (!s.port->program(s.ctx, s.layout.app_base + at + offset, word))
    {
      return false;
    }
  }
  return true;
}

static boot_answer_t h_boot_chunk(rd_t *in, wr_t *out)
{
  const uint16_t index = rd_u16(in);
  const uint16_t len = rd_left(in);
  const uint8_t *data = rd_bytes(in, len);

  (void)out;
  if ((s.state != BOOT_ERASED) && (s.state != BOOT_VERIFIED))
  {
    s.ignored++;
    return BOOT_SILENT;
  }
  if ((index >= s.chunks) || (len == 0U) || (len > BOOT_CHUNK_BYTES))
  {
    s.ignored++;
    return BOOT_SILENT;
  }
  if (bit(index))
  {
    return BOOT_SILENT;             /* a repeat; flash is not written twice */
  }
  if (!program_chunk(index, data, len))
  {
    s.port->say(s.ctx, "boot: a chunk did not program");
    return BOOT_SILENT;
  }
  set_bit(index);
  s.held++;
  return BOOT_SILENT;
}

static boot_answer_t h_boot_missing(rd_t *in, wr_t *out)
{
  const uint16_t bytes = (uint16_t)((s.chunks + BITS_PER_BYTE - 1U) / BITS_PER_BYTE);

  (void)in;
  wr_u16(out, 0U);
  wr_u16(out, s.chunks);
  wr_bytes(out, s.bitmap, bytes);
  return BOOT_REPLY;
}

static boot_answer_t h_boot_verify(rd_t *in, wr_t *out)
{
  (void)in;
  if ((s.state < BOOT_ERASED) || (s.held != s.chunks))
  {
    wr_u8(out, 0U);
    wr_u32(out, 0U);
    return BOOT_REPLY;
  }
  const uint32_t crc = image_crc(s.size);

  s.verified = (crc == s.crc);
  s.state = s.verified ? BOOT_VERIFIED : s.state;
  wr_u8(out, s.verified ? 1U : 0U);
  wr_u32(out, crc);
  return BOOT_REPLY;
}

static boot_answer_t h_boot_record(rd_t *in, wr_t *out)
{
  const uint16_t offset = rd_u16(in);
  const uint16_t len = rd_left(in);
  const uint8_t *data = rd_bytes(in, len);

  if (s.state < BOOT_ASSIGNED)
  {
    took(out, "a record goes to an assigned node - assign first");
    return BOOT_REPLY;
  }
  if ((uint32_t)offset + len > sizeof(s.record))
  {
    took(out, "the record is at most 2048 bytes");
    return BOOT_REPLY;
  }
  memcpy(&s.record[offset], data, len);
  if (offset + len > s.record_bytes)
  {
    s.record_bytes = (uint16_t)(offset + len);
  }
  took(out, NULL);
  return BOOT_REPLY;
}

/* The record's words programmed into its sector, padded to whole words. */
static bool program_record(void)
{
  uint8_t word[BOOT_WORD_BYTES];

  if (!s.port->erase(s.ctx, s.layout.record_base, s.layout.record_bytes))
  {
    return false;
  }
  for (uint32_t offset = 0U; offset < s.record_bytes; offset += BOOT_WORD_BYTES)
  {
    record_word(offset, word);
    if (!s.port->program(s.ctx, s.layout.record_base + offset, word))
    {
      return false;
    }
  }
  return true;
}

static boot_answer_t h_boot_seal(rd_t *in, wr_t *out)
{
  (void)in;
  if (!s.verified)
  {
    took(out, "the image is not verified - verify first, and it must say ok");
    return BOOT_REPLY;
  }
  if ((s.record_bytes != 0U) && !record_held() && !program_record())
  {
    took(out, "the record's sector did not program");
    return BOOT_REPLY;
  }
  if (s.first_held && !s.port->program(s.ctx, s.layout.app_base, s.first))
  {
    took(out, "the first word did not program - the image stays invalid");
    return BOOT_REPLY;
  }
  s.first_held = false;
  s.state = BOOT_SEALED;
  took(out, NULL);
  return BOOT_REPLY;
}

static boot_answer_t h_boot_go(rd_t *in, wr_t *out)
{
  const uint32_t session = rd_u32(in);

  (void)out;
  if ((session != s.session) || (s.state != BOOT_SEALED))
  {
    s.port->say(s.ctx, "boot: not sealed - staying");
    return BOOT_SILENT;
  }
  s.go = boot_app_valid();
  return BOOT_SILENT;
}

static boot_answer_t h_boot_state(rd_t *in, wr_t *out)
{
  (void)in;
  wr_u8(out, (uint8_t)s.state);
  wr_u8(out, s.layout.type);
  wr_u8(out, s.unit);
  wr_u8(out, s.position);
  wr_u32(out, s.held);
  wr_u32(out, s.chunks);
  wr_u8(out, boot_app_valid() ? 1U : 0U);
  wr_bytes(out, s.layout.uid, BOOT_UID_BYTES);
  return BOOT_REPLY;
}

static boot_answer_t h_boot_dump(rd_t *in, wr_t *out)
{
  const uint16_t offset = rd_u16(in);
  const uint32_t left = (offset < s.layout.record_bytes) ? (s.layout.record_bytes - offset) : 0U;
  const uint16_t page = (uint16_t)((left < BOOT_CHUNK_BYTES) ? left : BOOT_CHUNK_BYTES);

  wr_u16(out, offset);
  wr_bytes(out, s.port->read(s.ctx, s.layout.record_base + offset), page);
  return BOOT_REPLY;
}

static boot_answer_t h_boot_stay(rd_t *in, wr_t *out)
{
  (void)in;
  took(out, "this node is in its bootloader already");
  return BOOT_REPLY;
}

/* -- the seam -------------------------------------------------------------- */

void boot_init(const boot_port_t *port, void *ctx, const boot_layout_t *layout)
{
  memset(&s, 0, sizeof(s));
  s.port = port;
  s.ctx = ctx;
  s.layout = *layout;
  s.unit = BOOT_UNIT;
  memset(s.first, 0xFF, sizeof(s.first));
}

boot_answer_t boot_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case BOOT_OP_HOLD:    return h_boot_hold(in, out);
    case BOOT_OP_WHO:     return h_boot_who(in, out);
    case BOOT_OP_ASSIGN:  return h_boot_assign(in, out);
    case BOOT_OP_ERASE:   return h_boot_erase(in, out);
    case BOOT_OP_CHUNK:   return h_boot_chunk(in, out);
    case BOOT_OP_MISSING: return h_boot_missing(in, out);
    case BOOT_OP_VERIFY:  return h_boot_verify(in, out);
    case BOOT_OP_RECORD:  return h_boot_record(in, out);
    case BOOT_OP_SEAL:    return h_boot_seal(in, out);
    case BOOT_OP_GO:      return h_boot_go(in, out);
    case BOOT_OP_STATE:   return h_boot_state(in, out);
    case BOOT_OP_DUMP:    return h_boot_dump(in, out);
    case BOOT_OP_STAY:    return h_boot_stay(in, out);
    default:
      took(out, "no such boot op - PROTOCOL.md lists 0..12");
      return BOOT_REPLY;
  }
}

bool boot_app_valid(void)
{
  const uint32_t stack = read_u32(s.layout.app_base);
  const uint32_t reset = read_u32(s.layout.app_base + 4U);
  const uint32_t magic = read_u32(s.layout.app_base + BOOT_HEADER_OFFSET);
  const uint32_t size = read_u32(s.layout.app_base + BOOT_HEADER_OFFSET + 4U);
  const uint32_t type = read_u32(s.layout.app_base + BOOT_HEADER_OFFSET + 12U);
  const uint32_t end = s.layout.app_base + s.layout.app_bytes;

  return (stack > BOOT_STACK_BASE) && (stack <= BOOT_STACK_BASE + BOOT_STACK_BYTES)
         && ((reset & THUMB_BIT) != 0U)
         && (reset > s.layout.app_base) && (reset < end)
         && (magic == BOOT_HEADER_MAGIC) && (type == s.layout.type)
         && (size > BOOT_HEADER_OFFSET) && (size <= s.layout.app_bytes);
}

boot_state_t boot_state(void)
{
  return s.state;
}

uint8_t boot_unit(void)
{
  return s.unit;
}

uint8_t boot_position(void)
{
  return s.position;
}

uint8_t boot_flags(void)
{
  return s.flags;
}

bool boot_wants_go(void)
{
  return s.go;
}

bool boot_terminates(void)
{
  return (s.flags & BOOT_FLAG_TERMINATE) != 0U;
}

uint32_t boot_chunks_ignored(void)
{
  return s.ignored;
}

uint32_t boot_crc32(uint32_t crc, const uint8_t *data, size_t len)
{
  for (size_t i = 0U; i < len; i++)
  {
    crc ^= data[i];
    for (uint8_t k = 0U; k < BITS_PER_BYTE; k++)
    {
      crc = (crc >> 1) ^ (((crc & 1U) != 0U) ? CRC32_POLY : 0U);
    }
  }
  return crc;
}
