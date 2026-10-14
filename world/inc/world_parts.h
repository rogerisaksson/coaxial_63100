/** world_parts.h - The board's front end, A1335 and BNO085 across native and emulated boards. */
#ifndef WORLD_PARTS_H
#define WORLD_PARTS_H

#include <stdbool.h>
#include <stdint.h>

/** The front end's ten ADC inputs, in board_adc.c's order. */
typedef enum
{
  WORLD_PIN_NONE = 0,
  WORLD_PIN_U,
  WORLD_PIN_V,
  WORLD_PIN_W,
  WORLD_PIN_DCBUS,
  WORLD_PIN_NTC,
  WORLD_PIN_RAIL5,
  WORLD_PIN_VGATE,
  WORLD_PIN_DIE,
  WORLD_PIN_CLEVEL,
  WORLD_PIN_CINJ
} world_pin_t;

/** The analog front end: phase shunts, dividers, NTC, die thermometer and STO nodes. */
typedef struct
{
  double   ref;
  double   v_per_a, zero_v;
  double   gain_sigma, zero_sigma, bus_sigma;
  double   bus_top, bus_bottom;
  double   ntc_r25, ntc_beta, ntc_fixed;
  double   r5_top, r5_bottom;
  double   gate_top, gate_bottom;
  double   die_at30, die_per_k;
  double   noise;
  uint32_t seed;

  bool     powered;
  double   amps[3], dc, ntc_c, die_c, rail5, gate, clevel, cinj;
  double   gain_err[3], zero_err[3], bus_err;
  uint64_t rng;
} world_afe_t;

void     world_afe_init(world_afe_t *a);
void     world_afe_reset(world_afe_t *a);
void     world_afe_draw(world_afe_t *a, uint32_t seed);
void     world_afe_configure(world_afe_t *a, double v_per_a, double zero_v, double gain_sigma,
                             double zero_sigma, double bus_sigma, uint32_t seed);
double   world_afe_die_volts(const world_afe_t *a, double celsius);
double   world_afe_volts(world_afe_t *a, world_pin_t pin);
uint32_t world_afe_code(world_afe_t *a, world_pin_t pin);

/** The Allegro A1335 on SPI4: four 5-bit words a packet, answered two packets on, CRC-4. */
typedef struct
{
  double   gauss;
  uint32_t words;
  uint32_t command;
  uint32_t reply;
  uint32_t asked;
  bool     any;
} world_a1335_t;

void    world_a1335_open(world_a1335_t *a);
void    world_a1335_reset(world_a1335_t *a);
void    world_a1335_select(world_a1335_t *a, bool level, bool powered, double degrees,
                           double celsius);
uint8_t world_a1335_transmit(world_a1335_t *a, uint8_t word, bool powered);

/** The Hillcrest BNO085 on SPI2: SHTP framing, feature reports and the stand-in's tumble. */
#define WORLD_BNO_QUEUE    32U
#define WORLD_BNO_PACKET   128U
#define WORLD_BNO_INCOMING 64U
#define WORLD_BNO_FEATURES 16U

typedef struct
{
  uint8_t  packets[WORLD_BNO_QUEUE][WORLD_BNO_PACKET];
  uint16_t lengths[WORLD_BNO_QUEUE];
  uint32_t head, tail, count;
  uint8_t  incoming[WORLD_BNO_INCOMING];
  uint32_t in;
  const uint8_t *outgoing;
  uint16_t out_length;
  uint8_t  sequence[6];
  struct
  {
    bool     on;
    uint32_t interval;
    uint32_t elapsed;
    uint8_t  sequence;
  } feature[WORLD_BNO_FEATURES];
  uint32_t features;
  bool     selected, woken, piped;
  double   roll_rate, pitch_rate, yaw_rate;
  double   attitude[4];
  double   body[3];
  double   elapsed;
  double   readings[13];
} world_bno085_t;

void    world_bno085_open(world_bno085_t *b);
void    world_bno085_reset(world_bno085_t *b);
void    world_bno085_pin(world_bno085_t *b, uint8_t which, bool level, bool powered);
uint8_t world_bno085_transmit(world_bno085_t *b, uint8_t byte);
bool    world_bno085_interrupt(const world_bno085_t *b, bool powered);
bool    world_bno085_active(const world_bno085_t *b);
void    world_bno085_tick(world_bno085_t *b, bool powered);
double  world_bno085_read(const world_bno085_t *b, int index);
void    world_bno085_pipe(world_bno085_t *b, int index, double value);
void    world_bno085_imu(world_bno085_t *b, const double *readings);

#endif /* WORLD_PARTS_H */
