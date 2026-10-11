/** board_ctrl.c - The board's loop: ctrl_loop's, its angle the A1335's, its guard the IRQ's. */
#include "board.h"
#include "board_ctrl.h"
#include "board_drive.h"
#include "board_irq.h"

#include <math.h>

#define ANGLE_BITS    0x0FFFU                /* the A1335's twelve */
#define DEG_PER_COUNT (360.0f / 4096.0f)

static ctrl_loop_t s;

void Board_CtrlTick(drive_t *d)
{
  if (!ctrl_loop_due(&s))
  {
    return;
  }
  float angle = NAN;

  if (s.measured == CTRL_LOOP_ANGLE)
  {
    board_angle_state_t a;

    Board_AngleState(&a);
    angle = a.have ? ((float)(a.value & ANGLE_BITS) * DEG_PER_COUNT) : NAN;
  }
  ctrl_loop_step(&s, d, angle);
}

void Board_CtrlState(board_ctrl_state_t *out)
{
  const uint32_t masked = Board_IrqHold();

  ctrl_loop_state(&s, out);
  Board_IrqRelease(masked);
}

const char *Board_CtrlSlot(uint8_t slot, uint8_t kind, const float *params, uint8_t n)
{
  return ctrl_loop_slot(&s, slot, kind, params, n);
}

const char *Board_CtrlWire(uint8_t measured, uint8_t command, uint16_t hz)
{
  return ctrl_loop_wire(&s, Board_DriveTs(), measured, command, hz);
}

const char *Board_CtrlRows(const uint16_t *ms, const float *setpoint, uint8_t n)
{
  return ctrl_loop_rows(&s, ms, setpoint, n);
}

void Board_CtrlClear(void)
{
  const uint32_t masked = Board_IrqHold();

  ctrl_rows_clear(&s.r);
  Board_IrqRelease(masked);
}

const char *Board_CtrlRun(bool on)
{
  const uint32_t masked = Board_IrqHold();
  const char *why = ctrl_loop_run(&s, on);

  Board_IrqRelease(masked);
  return why;
}
