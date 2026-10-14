/** world_parts.c - The board's analog front end and A1335 angle sensor in C. */
#include "world_parts.h"

#include <math.h>
#include <string.h>

#define PARTS_FULL_CODE 65535.0
#define A1335_STA       0x8003U
#define A1335_FIELD_DEF 500U

/* xorshift64*, uniform on (0, 1]. */
static double rng_uniform(uint64_t *rng)
{
  *rng ^= *rng >> 12;
  *rng ^= *rng << 25;
  *rng ^= *rng >> 27;
  return ((double)((*rng * 0x2545F4914F6CDD1DULL) >> 11) + 1.0) / 9007199254740992.0;
}

static double rng_gauss(uint64_t *rng)
{
  const double u1 = rng_uniform(rng);
  const double u2 = rng_uniform(rng);

  return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static double afe_mid(const world_afe_t *a)
{
  return a->ref * 32768.0 / PARTS_FULL_CODE;
}

static double afe_phase(const world_afe_t *a, uint8_t leg)
{
  const double gain = a->v_per_a * (1.0 + a->gain_err[leg]);
  const double zero = a->zero_v + a->zero_err[leg];

  return afe_mid(a) + (zero + gain * a->amps[leg]) / 2.0;
}

double world_afe_die_volts(const world_afe_t *a, double celsius)
{
  return a->die_at30 + (celsius - 30.0) * a->die_per_k;
}

static double afe_nominal(const world_afe_t *a, world_pin_t pin)
{
  switch (pin)
  {
    case WORLD_PIN_U:      return afe_phase(a, 0U);
    case WORLD_PIN_V:      return afe_phase(a, 1U);
    case WORLD_PIN_W:      return afe_phase(a, 2U);
    case WORLD_PIN_DCBUS:  return a->dc * (1.0 + a->bus_err) * a->bus_bottom
                                  / (a->bus_top + a->bus_bottom);
    case WORLD_PIN_NTC:
    {
      const double ohms = a->ntc_r25 * exp(a->ntc_beta * (1.0 / (a->ntc_c + 273.15)
                                                          - 1.0 / (25.0 + 273.15)));
      return a->ref * a->ntc_fixed / (ohms + a->ntc_fixed);
    }
    case WORLD_PIN_RAIL5:  return a->rail5 * a->r5_bottom / (a->r5_top + a->r5_bottom);
    case WORLD_PIN_VGATE:  return a->gate * a->gate_bottom / (a->gate_top + a->gate_bottom);
    case WORLD_PIN_DIE:    return world_afe_die_volts(a, a->die_c);
    case WORLD_PIN_CLEVEL: return a->clevel;
    case WORLD_PIN_CINJ:   return a->cinj;
    default:               return 0.0;
  }
}

void world_afe_draw(world_afe_t *a, uint32_t seed)
{
  a->seed = (seed != 0U) ? seed : 1U;
  a->rng = a->seed;
  for (uint8_t leg = 0U; leg < 3U; leg++)
  {
    a->gain_err[leg] = rng_gauss(&a->rng) * a->gain_sigma;
    a->zero_err[leg] = rng_gauss(&a->rng) * a->zero_sigma;
  }
  a->bus_err = rng_gauss(&a->rng) * a->bus_sigma;
}

void world_afe_reset(world_afe_t *a)
{
  a->powered = false;
  a->amps[0] = a->amps[1] = a->amps[2] = 0.0;
  a->dc = a->clevel = a->cinj = a->gate = 0.0;
  a->ntc_c = a->die_c = 25.0;
  a->rail5 = 5.0;
  world_afe_draw(a, a->seed);
}

void world_afe_init(world_afe_t *a)
{
  memset(a, 0, sizeof *a);
  a->ref = 3.3;
  a->v_per_a = 0.025;
  a->bus_top = 49900.0;
  a->bus_bottom = 2200.0;
  a->ntc_r25 = 10000.0;
  a->ntc_beta = 3380.0;
  a->ntc_fixed = 10000.0;
  a->r5_top = 10000.0;
  a->r5_bottom = 10000.0;
  a->gate_top = 57000.0;
  a->gate_bottom = 10000.0;
  a->die_at30 = 0.62;
  a->die_per_k = 0.002;
  a->noise = 0.0001;
  a->seed = 0x63100001U;
  world_afe_reset(a);
}

void world_afe_configure(world_afe_t *a, double v_per_a, double zero_v, double gain_sigma,
                         double zero_sigma, double bus_sigma, uint32_t seed)
{
  a->v_per_a = v_per_a;
  a->zero_v = zero_v;
  a->gain_sigma = gain_sigma;
  a->zero_sigma = zero_sigma;
  a->bus_sigma = bus_sigma;
  world_afe_draw(a, seed);
}

double world_afe_volts(world_afe_t *a, world_pin_t pin)
{
  double volts = afe_mid(a);

  if (a->powered)
  {
    volts = afe_nominal(a, pin) + rng_gauss(&a->rng) * a->noise;
    volts = (volts < 0.0) ? 0.0 : (volts > a->ref) ? a->ref : volts;
  }
  return volts;
}

uint32_t world_afe_code(world_afe_t *a, world_pin_t pin)
{
  const double volts = world_afe_volts(a, pin);
  const double code = floor(volts / a->ref * 65536.0 + 0.5);

  return (code > PARTS_FULL_CODE) ? (uint32_t)PARTS_FULL_CODE : (uint32_t)code;
}

/* ---- Allegro A1335 ------------------------------------------------------------------ */

#define A1335_WORDS         4U
#define A1335_WORD_BITS     5U
#define A1335_WORD_MASK     0x1FU
#define A1335_ADDRESS_SHIFT 12U
#define A1335_DATA_SHIFT    4U
#define A1335_RW_SHIFT      18U
#define A1335_ALL_ONES      0xFFFFFU
#define A1335_COUNTS        4096U
#define A1335_ANG           0x20U
#define A1335_TSEN          0x28U
#define A1335_FIELD         0x2AU
#define A1335_GAUSS_DEF     380.0
#define A1335_ANG_ID        0x5000U
#define A1335_TSEN_ID       0xF000U
#define A1335_FIELD_ID      0xE000U

void world_a1335_reset(world_a1335_t *a)
{
  a->words = 0U;
  a->command = 0U;
  a->reply = A1335_ALL_ONES;
  a->asked = 0U;
  a->any = false;
}

void world_a1335_open(world_a1335_t *a)
{
  memset(a, 0, sizeof *a);
  a->gauss = A1335_GAUSS_DEF;
  world_a1335_reset(a);
}

static uint16_t a1335_register(const world_a1335_t *a, uint32_t reg, double degrees, double celsius)
{
  switch (reg)
  {
    case A1335_ANG:
    {
      const double turns = degrees / 360.0;

      return (uint16_t)(A1335_ANG_ID
                        | ((unsigned)floor((turns - floor(turns)) * (double)A1335_COUNTS)
                           & (A1335_COUNTS - 1U)));
    }
    case A1335_TSEN:
    {
      const double eighths = floor((celsius + 273.15) * 8.0 + 0.5);

      return (uint16_t)(A1335_TSEN_ID
                        | (unsigned)((eighths < 0.0) ? 0.0
                                                     : (eighths > 4095.0) ? 4095.0 : eighths));
    }
    case A1335_FIELD:
    {
      const double g = floor(a->gauss + 0.5);

      return (uint16_t)(A1335_FIELD_ID
                        | (unsigned)((g < 0.0) ? 0.0 : (g > 4095.0) ? 4095.0 : g));
    }
    default:
      return 0U;
  }
}

/* CRC-4, x^4 + x + 1, seed 0xF, over the 16 data bits MSB first. */
static uint32_t a1335_crc4(uint16_t value)
{
  uint32_t crc = 0xFU;

  for (int bit = 15; bit >= 0; bit--)
  {
    const uint32_t top = ((crc >> 3) & 1U) ^ (((uint32_t)value >> (uint32_t)bit) & 1U);

    crc = ((crc << 1) & 0xFU) ^ ((top != 0U) ? 0x3U : 0U);
  }
  return crc;
}

static void a1335_packet(world_a1335_t *a, uint32_t packet, bool powered, double degrees,
                         double celsius)
{
  const uint32_t reg = (packet >> A1335_ADDRESS_SHIFT) & 0x3FU;
  const bool write = ((packet >> A1335_RW_SHIFT) & 1U) != 0U;

  if (write || !powered)
  {
    a->reply = A1335_ALL_ONES;
    return;
  }
  const uint16_t value = a1335_register(a, reg, degrees, celsius);

  a->reply = ((uint32_t)value << A1335_DATA_SHIFT) | a1335_crc4(value);
}

void world_a1335_select(world_a1335_t *a, bool level, bool powered, double degrees, double celsius)
{
  if (!level)
  {
    a->words = 0U;
    a->command = 0U;
    return;
  }
  if (a->words == A1335_WORDS)
  {
    if (a->any)
    {
      a1335_packet(a, a->asked, powered, degrees, celsius);
    }
    a->asked = a->command;
    a->any = true;
  }
  a->words = 0U;
}

uint8_t world_a1335_transmit(world_a1335_t *a, uint8_t word, bool powered)
{
  const uint32_t at = (a->words < A1335_WORDS) ? a->words : (A1335_WORDS - 1U);
  const uint32_t shift = A1335_WORD_BITS * (A1335_WORDS - 1U - at);
  const uint32_t reply = powered ? a->reply : A1335_ALL_ONES;

  a->command = (a->command << A1335_WORD_BITS) | ((uint32_t)word & A1335_WORD_MASK);
  a->words++;
  return (uint8_t)((reply >> shift) & A1335_WORD_MASK);
}

