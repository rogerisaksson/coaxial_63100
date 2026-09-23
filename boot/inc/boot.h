/**
  ******************************************************************************
  * @file    boot.h
  * @brief   The bootloader's state machine: a blank node taking its image and
  *          its record from the master over broadcast, hardware-free.
  *
  * Device 11 under 0x6E, served by a node in its bootloader; the running
  * application serves two of its ops (`state`, `stay`) and refuses the rest
  * in words. docs/BOOT.md is the design: the flash map, the master's
  * sequence, why the first flash word is written last.
  *
  * Portable C11 like modbus/, daq/ and drive/: the flash, the console and
  * the identity arrive through `boot_port_t` and `boot_layout_t`, so
  * `boot/test/harness.c` runs the whole exchange on a RAM image and
  * host/tests/test_boot_core.py drives it through gcc and ctypes before
  * any register is touched. One node per process - the bootloader is one
  * node - so the state is the module's, the way board/ keeps it.
  ******************************************************************************
  */
#ifndef BOOT_H
#define BOOT_H

#include "wire.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The device under 0x6E, and the unit id a blank node answers to: the
    last legal one, since 248..255 are reserved and 0 is broadcast. A
    master reaches every blank node on a segment at once through it. */
#define DEVICE_BOOT          11U
#define BOOT_UNIT            247U

/** A chunk of image: 224 bytes is seven flash words, and with the frame
    around it 252 bytes on the wire, inside the 253 a PDU allows. */
#define BOOT_CHUNK_BYTES     224U
#define BOOT_WORD_BYTES      32U      /**< one flash word, 256 bits          */
#define BOOT_WORDS_PER_CHUNK (BOOT_CHUNK_BYTES / BOOT_WORD_BYTES)
#define BOOT_UID_BYTES       12U      /**< the MCU's unique id, 96 bits      */
#define BOOT_UID_BITS        (BOOT_UID_BYTES * 8U)
/** The application's largest image, 1792 K, in chunks - and the bitmap
    that says which have landed. */
#define BOOT_MAX_CHUNKS      8192U
#define BOOT_BITMAP_BYTES    (BOOT_MAX_CHUNKS / 8U)
/** The record is one flash-word-padded struct; this is room for it. */
#define BOOT_RECORD_MAX      2048U
/** The header behind the application's vector table, and its magic:
    'CXAP' as the bytes read in flash. */
#define BOOT_HEADER_OFFSET   0x400U
#define BOOT_HEADER_MAGIC    0x50415843U
/** Where a stack pointer must point to be one: DTCM. */
#define BOOT_STACK_BASE      0x20000000U
#define BOOT_STACK_BYTES     0x20000U

/** The board types as `erase`, `who` and the header name them: the two
    inverter types the machine already names. A bootloader is built for
    one; an image carries its own; they must agree. */
#define BOOT_TYPE_COAXIAL_63100  1U
#define BOOT_TYPE_COAXIAL_63020  2U

/** assign's flags. */
#define BOOT_FLAG_TERMINATE  0x01U    /**< the last node on the segment closes the 120 ohm */

/** THE HANDOVER SLOT: the top 32 bytes of DTCM, which both linker
    scripts place at the same address and neither startup zeroes or
    copies, so it is exactly what the last image left. The bootloader
    fills the identity before it jumps; the application writes STAY
    before it resets itself; a board with no bootloader finds neither
    magic and is unit 1 as it always was. */
#define BOOT_HAND_BYTES      32U
#define BOOT_HAND_MAGIC      0x444E4148U   /**< 'HAND' */
#define BOOT_STAY_MAGIC      0x59415453U   /**< 'STAY' */

typedef struct
{
  uint32_t magic;      /**< BOOT_HAND_MAGIC once a bootloader assigned the rest */
  uint32_t stay;       /**< BOOT_STAY_MAGIC when the application asks back      */
  uint8_t  unit;
  uint8_t  position;
  uint8_t  flags;
  uint8_t  reserved;
} boot_hand_t;

_Static_assert(sizeof(boot_hand_t) <= BOOT_HAND_BYTES, "the handover slot is 32 bytes");

/** The ops, PROTOCOL.md's device 11. */
#define BOOT_OP_HOLD      0U   /**< u32 session -> none; broadcast: stay in the bootloader */
#define BOOT_OP_WHO       1U   /**< u8 bits, bytes prefix -> u8 x12 uid, u8 type, u8 state, u8 unit; from every node the prefix fits */
#define BOOT_OP_ASSIGN    2U   /**< u8 x12 uid, u8 unit, u8 position, u8 flags -> u8 took; from that node only */
#define BOOT_OP_ERASE     3U   /**< u8 type, u32 size, u32 crc, u16 chunks -> none; broadcast */
#define BOOT_OP_CHUNK     4U   /**< u16 index, bytes -> none; broadcast */
#define BOOT_OP_MISSING   5U   /**< -> u16 first, u16 count, bytes bitmap */
#define BOOT_OP_VERIFY    6U   /**< -> u8 ok, u32 crc */
#define BOOT_OP_RECORD    7U   /**< u16 offset, bytes -> u8 took */
#define BOOT_OP_SEAL      8U   /**< -> u8 took */
#define BOOT_OP_GO        9U   /**< u32 session -> none; broadcast */
#define BOOT_OP_STATE     10U  /**< -> u8 state, u8 type, u8 unit, u8 position, u32 chunks_held, u32 chunks_of, u8 app_valid, u8 x12 uid */
#define BOOT_OP_DUMP      11U  /**< u16 offset -> u16 offset, bytes */
#define BOOT_OP_STAY      12U  /**< -> u8 took; the application: back to the bootloader */

/** A node's state, as `state` reports it. */
typedef enum
{
  BOOT_BLANK    = 0,   /**< nothing heard */
  BOOT_HELD     = 1,   /**< a hold heard; staying */
  BOOT_ASSIGNED = 2,   /**< a unit and a position given */
  BOOT_ERASED   = 3,   /**< the sectors erased; chunks landing */
  BOOT_VERIFIED = 4,   /**< the crc matched */
  BOOT_SEALED   = 5,   /**< the record and the first word programmed */
} boot_state_t;

/** What a handler answers with: a reply, silence (not this node's
    question, or a broadcast), or the words are already in `out`. */
typedef enum
{
  BOOT_SILENT = 0,
  BOOT_REPLY  = 1,
} boot_answer_t;

/** The hardware, as four calls. `erase` clears the sectors covering the
    range; `program` writes one flash word; `read` gives a pointer to
    flash; `say` puts one line on the console. */
typedef struct
{
  bool (*erase)(void *ctx, uint32_t address, uint32_t bytes);
  bool (*program)(void *ctx, uint32_t address, const uint8_t *word);
  const uint8_t *(*read)(void *ctx, uint32_t address);
  void (*say)(void *ctx, const char *line);
} boot_port_t;

/** What this node is and where its flash is. */
typedef struct
{
  uint32_t app_base;
  uint32_t app_bytes;
  uint32_t record_base;
  uint32_t record_bytes;
  uint8_t  type;
  uint8_t  uid[BOOT_UID_BYTES];
} boot_layout_t;

/** The node, from nothing. */
void boot_init(const boot_port_t *port, void *ctx, const boot_layout_t *layout);

/** One 0x6E PDU for device 11 - `op` and what follows it in `in` -
    answered into `out`. BOOT_SILENT means nothing goes on the wire. */
boot_answer_t boot_op(uint8_t op, rd_t *in, wr_t *out);

/** The four tests of an image: a stack pointer in DTCM, a thumb reset
    vector in the application, the header's magic and type, the size
    inside the range. A never-written sector fails the first. */
bool boot_app_valid(void);

/** What the hardware layer asks after each frame. */
boot_state_t boot_state(void);
uint8_t      boot_unit(void);             /**< BOOT_UNIT until assigned  */
uint8_t      boot_position(void);         /**< 0 until assigned          */
uint8_t      boot_flags(void);            /**< assign's flags            */
bool         boot_wants_go(void);         /**< a go for a sealed node    */
bool         boot_terminates(void);       /**< assign's flag bit 0       */
uint32_t     boot_chunks_ignored(void);   /**< chunks with no erase      */

/** CRC-32, IEEE, as every host library computes it. */
uint32_t boot_crc32(uint32_t crc, const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif /* BOOT_H */
