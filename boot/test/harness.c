/**
  * The bootloader's core on a RAM flash: the four port calls over a byte
  * array with 128 K sectors, a fault that can be scripted, the console's
  * last line kept - so host/tests/test_boot_core.py can stream an image,
  * drop chunks, kill the master and read what the node believes. Built
  * with the host gcc like modbus/test/harness.c; nothing here runs on
  * the board.
  */
#include "boot.h"
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

#define SECTOR_BYTES  (128U * 1024U)
#define FLASH_BYTES   (2U * 1024U * 1024U)
#define APP_BASE      0x08020000UL
#define APP_BYTES     (14U * SECTOR_BYTES)
#define RECORD_BASE   0x081E0000UL
#define RECORD_BYTES  SECTOR_BYTES
#define FLASH_BASE    0x08000000UL
#define LINE_MAX      120U

static uint8_t   s_flash[FLASH_BYTES];
static uint32_t  s_erases;
static uint32_t  s_programs;
static uint32_t  s_fail_program_at;      /* an address that refuses, 0 for none */
static int       s_fail_erase;
static char      s_said[LINE_MAX];
static boot_layout_t s_layout;

static uint8_t *at(uint32_t address)
{
  return &s_flash[address - FLASH_BASE];
}

static bool erase(void *ctx, uint32_t address, uint32_t bytes)
{
  (void)ctx;
  if (s_fail_erase)
  {
    return false;
  }
  const uint32_t first = (address - FLASH_BASE) / SECTOR_BYTES;
  const uint32_t last = (address - FLASH_BASE + bytes - 1U) / SECTOR_BYTES;

  memset(&s_flash[first * SECTOR_BYTES], 0xFF, (last - first + 1U) * SECTOR_BYTES);
  s_erases++;
  return true;
}

/* A word programs once: the real controller refuses a word that is not
   erased, and so does this, which is how a test proves nothing is written
   twice. */
static bool program(void *ctx, uint32_t address, const uint8_t *word)
{
  (void)ctx;
  if ((s_fail_program_at != 0U) && (address == s_fail_program_at))
  {
    return false;
  }
  for (uint32_t i = 0U; i < BOOT_WORD_BYTES; i++)
  {
    if (at(address)[i] != 0xFFU)
    {
      return false;
    }
  }
  memcpy(at(address), word, BOOT_WORD_BYTES);
  s_programs++;
  return true;
}

static const uint8_t *read(void *ctx, uint32_t address)
{
  (void)ctx;
  return at(address);
}

static void say(void *ctx, const char *line)
{
  (void)ctx;
  strncpy(s_said, line, sizeof(s_said) - 1U);
  s_said[sizeof(s_said) - 1U] = '\0';
}

static const boot_port_t s_port = { erase, program, read, say };

/* -- what the test drives -------------------------------------------------- */

API void boot_h_reset(uint8_t type, const uint8_t *uid)
{
  memset(s_flash, 0xFF, sizeof(s_flash));
  s_erases = 0U;
  s_programs = 0U;
  s_fail_program_at = 0U;
  s_fail_erase = 0;
  s_said[0] = '\0';
  s_layout.app_base = APP_BASE;
  s_layout.app_bytes = APP_BYTES;
  s_layout.record_base = RECORD_BASE;
  s_layout.record_bytes = RECORD_BYTES;
  s_layout.type = type;
  memcpy(s_layout.uid, uid, BOOT_UID_BYTES);
  boot_init(&s_port, NULL, &s_layout);
}

/* One request - the PDU after the device byte - answered; the reply's
   length, 0 for silence. */
API int boot_h_op(uint8_t op, const uint8_t *req, uint16_t len, uint8_t *rsp, uint16_t cap)
{
  rd_t in;
  wr_t out;

  rd_init(&in, req, len);
  wr_init(&out, rsp, cap);
  if (boot_op(op, &in, &out) == BOOT_SILENT)
  {
    return 0;
  }
  return wr_ok(&out) ? (int)wr_len(&out) : -1;
}

API void boot_h_read(uint32_t address, uint8_t *out, uint32_t n)
{
  memcpy(out, at(address), n);
}

API void boot_h_write(uint32_t address, const uint8_t *in, uint32_t n)
{
  memcpy(at(address), in, n);      /* the debugger's way in: straight to flash */
}

API int      boot_h_state(void)         { return (int)boot_state(); }
API int      boot_h_unit(void)          { return (int)boot_unit(); }
API int      boot_h_go(void)            { return boot_wants_go() ? 1 : 0; }
API int      boot_h_terminates(void)    { return boot_terminates() ? 1 : 0; }
API uint32_t boot_h_ignored(void)       { return boot_chunks_ignored(); }
API int      boot_h_app_valid(void)     { return boot_app_valid() ? 1 : 0; }
API uint32_t boot_h_erases(void)        { return s_erases; }
API uint32_t boot_h_programs(void)      { return s_programs; }
API const char *boot_h_said(void)       { return s_said; }
API void     boot_h_fail_program(uint32_t address) { s_fail_program_at = address; }
API void     boot_h_fail_erase(int on)  { s_fail_erase = on; }
API uint32_t boot_h_crc32(const uint8_t *data, uint32_t n)
{
  return boot_crc32(0xFFFFFFFFUL, data, n) ^ 0xFFFFFFFFUL;
}
