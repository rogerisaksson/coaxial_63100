/** native_a1335.c - The Allegro A1335 on SPI4, backed by world/src/world_parts.c. */
#include "native.h"
#include "world_parts.h"

static world_a1335_t a;

void a1335_open(void)
{
  world_a1335_open(&a);
}

void a1335_select(bool level)
{
  world_a1335_select(&a, level, native_powered(), native_shaft_degrees(), native_angle_celsius());
}

uint8_t a1335_transmit(uint8_t word)
{
  return world_a1335_transmit(&a, word, native_powered());
}
