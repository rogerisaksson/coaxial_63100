/* Host-build stand-in for cmd_find: the dispatch tables live beside HAL
   handlers and cannot come along, so the table-driven arm answers "not
   found" here and cmd_request_length falls through to the arms this
   suite actually exercises - 0x6E and the standard function codes. The
   table arm is validated by the suite parsing the C tables themselves
   (test_modbus_core, `the fixed dict matches the dispatch tables`). */
#include "cmd.h"

const cmd_desc_t *cmd_find(uint8_t code)
{
  (void)code;
  return (const cmd_desc_t *)0;
}
