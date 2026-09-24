/** boot.h - Bootloader state machine, hardware-free. */
#ifndef BOOT_H
#define BOOT_H

#include "wire.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The device under 0x6E, and the unit id a blank node answers to: the last
    legal one, since 248..255 are reserved and 0 is broadcast. */
#define DEVICE_BOOT          11U
#define BOOT_UNIT            247U

/** A chunk of image: 224 bytes is seven flash words, and with the frame
    around it 252 bytes on the wire, inside the 253 a PDU allows. */
#define BOOT_CHUNK_BYTES     224U
#define BOOT_WORD_BYTES      32U      /**< one flash word, 256 bits */
#define BOOT_WORDS_PER_CHUNK (BOOT_CHUNK_BYTES / BOOT_WORD_BYTES)
#define BOOT_UID_BYTES       12U      /**< the MCU's unique id, 96 bits */
#define BOOT_UID_BITS        (BOOT_UID_BYTES * 8U)

/** Where the application runs and where it is kept (docs/BOOT.md). It is
    linked into D2 SRAM, which nothing else uses, and streamed there; flash
    keeps a sealed copy for a power-up with no master - the seal word first,
    the image behind it. */
#define BOOT_RUN_BASE        0x30000000U
#define BOOT_RUN_BYTES       0x48000U      /**< SRAM1..3, 288 K */
#define BOOT_FLASH_BASE      0x08000000U
#define BOOT_SECTOR_BYTES    0x20000U
#define BOOT_STORE_BASE      0x08020000U   /**< sectors 1..14 */
#define BOOT_STORE_BYTES     0x1C0000U
#define BOOT_RECORD_BASE     0x081E0000U   /**< bank 2 sector 7 */
#define BOOT_RECORD_BYTES    0x20000U

/** The largest image in chunks, and the bitmap of those landed: 165 bytes,
    one `missing` reply. */
#define BOOT_MAX_CHUNKS      ((BOOT_RUN_BYTES + BOOT_CHUNK_BYTES - 1U) / BOOT_CHUNK_BYTES)
#define BOOT_BITMAP_BYTES    ((BOOT_MAX_CHUNKS + 7U) / 8U)
/** The record is one flash-word-padded struct; this is room for it. */
#define BOOT_RECORD_MAX      2048U
/** The header behind the application's vector table, and its magic: 'CXAP'
    as the bytes read in flash. */
#define BOOT_HEADER_OFFSET   0x400U
#define BOOT_HEADER_MAGIC    0x50415843U
/** Where a stack pointer must point to be one: DTCM. */
#define BOOT_STACK_BASE      0x20000000U
#define BOOT_STACK_BYTES     0x20000U

/** The image header at BOOT_RUN_BASE + BOOT_HEADER_OFFSET. */
typedef struct
{
  uint32_t magic;
  uint32_t bytes;
  uint32_t version;
  uint32_t type;
} boot_header_t;

/** The board types as `erase`, `who` and the header name them: the two
    inverter types the machine already names. */
#define BOOT_TYPE_COAXIAL_63100  1U
#define BOOT_TYPE_COAXIAL_63020  2U

/** The store's seal: its first flash word, programmed after the image
    behind it, so a store interrupted anywhere holds nothing. */
#define BOOT_SEAL_MAGIC      0x4C414553U   /**< 'SEAL' */

typedef struct
{
  uint32_t magic;
  uint32_t bytes;
  uint32_t crc;
  uint32_t type;
} boot_seal_t;

/** assign's flags. */
#define BOOT_FLAG_TERMINATE  0x01U    /**< the last node on the segment closes the 120 ohm */
/** seal's flags. */
#define BOOT_SEAL_PERSIST    0x01U    /**< keep a copy in the store, if it holds another */

/** The handover slot: the top 32 bytes of DTCM, which both linker scripts
    place at the same address and neither startup zeroes or copies, so it is
    exactly what the last image left. */
#define BOOT_HAND_BYTES      32U
#define BOOT_HAND_MAGIC      0x444E4148U   /**< 'HAND' */
#define BOOT_STAY_MAGIC      0x59415453U   /**< 'STAY' */

typedef struct
{
  uint32_t magic;      /**< BOOT_HAND_MAGIC once a bootloader assigned the rest */
  uint32_t stay;       /**< BOOT_STAY_MAGIC when the application asks back */
  uint8_t  unit;
  uint8_t  position;
  uint8_t  flags;
  uint8_t  reserved;
  uint32_t bytes;      /**< the image the bootloader verified and ran: its size */
  uint32_t crc;        /**< and its CRC-32 - what a warm reset checks RAM against */
} boot_hand_t;

_Static_assert(sizeof(boot_hand_t) <= BOOT_HAND_BYTES, "the handover slot is 32 bytes");

/** The ops, PROTOCOL.md's device 11. */
#define BOOT_OP_HOLD      0U   /**< u32 session -> none; broadcast: stay in the bootloader */
#define BOOT_OP_WHO       1U   /**< u8 bits, bytes prefix -> u8 x12 uid, u8 type, u8 state, u8 unit */
#define BOOT_OP_ASSIGN    2U   /**< u8 x12 uid, u8 unit, u8 position, u8 flags -> u8 took */
#define BOOT_OP_ERASE     3U   /**< u8 type, u32 size, u32 crc, u16 chunks -> none; broadcast */
#define BOOT_OP_CHUNK     4U   /**< u16 index, bytes -> none; broadcast */
#define BOOT_OP_MISSING   5U   /**< -> u16 first, u16 count, bytes bitmap */
#define BOOT_OP_VERIFY    6U   /**< -> u8 ok, u32 crc */
#define BOOT_OP_RECORD    7U   /**< u16 offset, bytes -> u8 took */
#define BOOT_OP_SEAL      8U   /**< [u8 flags] -> u8 took */
#define BOOT_OP_GO        9U   /**< u32 session -> none; broadcast */
#define BOOT_OP_STATE     10U  /**< -> u8 state, u8 type, u8 unit, u8 position, u32 chunks_held, u32 chunks_of, u8 app_valid, u8 x12 uid, u32 image_bytes, u32 image_crc, u8 flags */
#define BOOT_OP_DUMP      11U  /**< u16 offset -> u16 offset, bytes */
#define BOOT_OP_STAY      12U  /**< -> u8 took; the application: back to the bootloader */

/** A node's state, as `state` reports it. */
typedef enum
{
  BOOT_BLANK    = 0,   /**< nothing heard */
  BOOT_HELD     = 1,   /**< a hold heard; staying */
  BOOT_ASSIGNED = 2,   /**< a unit and a position given */
  BOOT_ERASED   = 3,   /**< RAM cleared; chunks landing */
  BOOT_VERIFIED = 4,   /**< the crc matched */
  BOOT_SEALED   = 5,   /**< the record and the first word written; the image valid */
} boot_state_t;

/** What a handler answers with: a reply, silence (not this node's question,
    or a broadcast), or the words are already in `out`. */
typedef enum
{
  BOOT_SILENT = 0,
  BOOT_REPLY  = 1,
} boot_answer_t;

/** The hardware, as four calls. */
typedef struct
{
  bool (*erase)(void *ctx, uint32_t address, uint32_t bytes);
  bool (*program)(void *ctx, uint32_t address, const uint8_t *word);
  const uint8_t *(*read)(void *ctx, uint32_t address);
  void (*say)(void *ctx, const char *line);
} boot_port_t;

/** What this node is and where its memories are. */
typedef struct
{
  uint32_t app_base;       /**< where the image runs: RAM */
  uint32_t app_bytes;
  uint32_t store_base;     /**< where its sealed copy is kept: flash */
  uint32_t store_bytes;
  uint32_t record_base;
  uint32_t record_bytes;
  uint8_t  type;
  uint8_t  uid[BOOT_UID_BYTES];
} boot_layout_t;

/** The node, from nothing. */
void boot_init(const boot_port_t *port, void *ctx, const boot_layout_t *layout);

/** One 0x6E PDU for device 11 - `op` and what follows it in `in` - answered
    into `out`. */
boot_answer_t boot_op(uint8_t op, rd_t *in, wr_t *out);

/** One 0x6E request (device byte, op, fields) answered into `rsp` as the
    application's link answers: the fields alone, no echo. The reply's
    length, or one of these. */
#define BOOT_PDU_SILENT      (-1)   /**< not this node's question, or a broadcast */
#define BOOT_PDU_FOREIGN     (-2)   /**< another device's */
#define BOOT_PDU_OVERFLOW    (-3)   /**< the reply did not fit */
int boot_pdu(const uint8_t *req, size_t req_len, uint8_t *rsp, size_t rsp_cap);

/** The four tests of an image: a stack pointer in DTCM, a thumb reset vector
    in the application, the header's magic and type, the size inside the
    range. */
bool boot_app_valid(void);

/** The gate at reset: RAM still holding, whole, the image the slot names (a
    warm reset), or the store's sealed copy verified and copied into RAM.
    False: nothing may run. */
bool boot_ready(uint32_t hand_bytes, uint32_t hand_crc);

/** The image RAM holds verified - size and CRC, both 0 for none: what the
    slot hands the application. */
void boot_image(uint32_t *bytes, uint32_t *crc);

/** What the hardware layer asks after each frame. */
boot_state_t boot_state(void);
uint8_t      boot_unit(void);             /**< BOOT_UNIT until assigned */
uint8_t      boot_position(void);         /**< 0 until assigned */
uint8_t      boot_flags(void);            /**< assign's flags */
bool         boot_wants_go(void);         /**< a go for a sealed node */
bool         boot_terminates(void);       /**< assign's flag bit 0 */
uint32_t     boot_chunks_ignored(void);   /**< chunks with no erase */

/** CRC-32, IEEE, as every host library computes it. */
uint32_t boot_crc32(uint32_t crc, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_H */
