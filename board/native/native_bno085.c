/** native_bno085.c - The Hillcrest BNO085 on SPI2, backed by world/src/world_bno085.c. */
#include "native.h"
#include "world_parts.h"

static world_bno085_t b;

void bno085_open(void)
{
  world_bno085_open(&b);
}

void bno085_pin(uint8_t which, bool level)
{
  world_bno085_pin(&b, which, level, native_powered());
}

uint8_t bno085_transmit(uint8_t byte)
{
  return world_bno085_transmit(&b, byte);
}

bool bno085_interrupt(void)
{
  return world_bno085_interrupt(&b, native_powered());
}

void bno085_tick(void)
{
  world_bno085_tick(&b, native_powered());
}

void native_imu(const double *readings)
{
  world_bno085_imu(&b, readings);
}
