/** cmd_boot.c - Device 11 in the application: state and stay. */
#include "board.h"
#include "boot.h"
#include "cmd.h"
#include "wire.h"

/** The state a running application reports: sealed, since it is the image
    that came through the seal, and valid, since it is running. */
static cmd_status_t h_boot_state(rd_t *in, wr_t *out)
{
  const board_identity_t id = Board_Identity();
  uint8_t uid[BOOT_UID_BYTES];

  (void)in;
  Board_Uid(uid);
  wr_u8(out, (uint8_t)BOOT_SEALED);
  wr_u8(out, id.type);
  wr_u8(out, id.unit);
  wr_u8(out, id.position);
  wr_u32(out, 0U);
  wr_u32(out, 0U);
  wr_u8(out, 1U);
  wr_bytes(out, uid, BOOT_UID_BYTES);
  return CMD_OK;
}

/** The reply goes first; the reset follows once it has left the wire
    (`Board_BootPoll`), and the bootloader finds STAY in the slot. */
static cmd_status_t h_boot_stay(rd_t *in, wr_t *out)
{
  (void)in;
  wr_took(out, NULL);
  Board_BootStay();
  return CMD_OK;
}

cmd_status_t cmd_boot_op(uint8_t op, rd_t *in, wr_t *out)
{
  switch (op)
  {
    case BOOT_OP_STATE: return h_boot_state(in, out);
    case BOOT_OP_STAY:  return h_boot_stay(in, out);
    default:
      wr_took(out, "only the bootloader takes an image - send stay, and the "
                   "master finds this node there");
      return CMD_OK;
  }
}
