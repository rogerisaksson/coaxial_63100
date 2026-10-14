/** world_bno085.c - The Hillcrest BNO085 on SPI2: SHTP framing, reports and tumble in C. */
#include "world_parts.h"

#include <math.h>
#include <string.h>

#define BNO_HEADER            4U
#define BNO_TICK_HZ           1000U
#define BNO_GRAVITY           9.80665

#define CHANNEL_EXECUTABLE    1U
#define CHANNEL_CONTROL       2U
#define CHANNEL_INPUT         3U
#define PRODUCT_ID_REQUEST    0xF9U
#define PRODUCT_ID_RESPONSE   0xF8U
#define SET_FEATURE           0xFDU
#define GET_FEATURE           0xFEU
#define GET_FEATURE_RESPONSE  0xFCU
#define COMMAND_REQUEST       0xF2U
#define COMMAND_RESPONSE      0xF1U
#define TIMEBASE              0xFBU
#define ACCELEROMETER         0x01U
#define GYROSCOPE             0x02U
#define MAGNETIC_FIELD        0x03U
#define ROTATION_VECTOR       0x05U
#define GAME_ROTATION_VECTOR  0x08U

static const double FIELD[3] = { 22.0, -3.0, 41.0 };

void world_bno085_reset(world_bno085_t *b)
{
  b->head = b->tail = b->count = 0U;
  b->in = 0U;
  b->outgoing = NULL;
  b->out_length = 0U;
  memset(b->sequence, 0, sizeof b->sequence);
  memset(b->feature, 0, sizeof b->feature);
  b->features = 0U;
  b->selected = false;
  b->woken = false;
}

void world_bno085_open(world_bno085_t *b)
{
  memset(b, 0, sizeof *b);
  b->roll_rate = 2.0 * 3.14159265358979323846 / 25.6;
  b->pitch_rate = 4.0 * 3.14159265358979323846 / 25.6;
  b->attitude[0] = 1.0;
  world_bno085_reset(b);
}

static void bno_send(world_bno085_t *b, uint8_t channel, const uint8_t *cargo, uint32_t length)
{
  const uint32_t total = BNO_HEADER + length;

  if ((b->count >= WORLD_BNO_QUEUE) || (total > WORLD_BNO_PACKET))
  {
    return;
  }
  uint8_t *pkt = b->packets[b->tail];

  pkt[0] = (uint8_t)total;
  pkt[1] = (uint8_t)(total >> 8);
  pkt[2] = channel;
  pkt[3] = b->sequence[channel]++;
  memcpy(&pkt[BNO_HEADER], cargo, length);
  b->lengths[b->tail] = (uint16_t)total;
  b->tail = (b->tail + 1U) % WORLD_BNO_QUEUE;
  b->count++;
}

static void bno_put_u32(uint8_t *at, uint32_t v)
{
  at[0] = (uint8_t)v;
  at[1] = (uint8_t)(v >> 8);
  at[2] = (uint8_t)(v >> 16);
  at[3] = (uint8_t)(v >> 24);
}

static uint32_t bno_get_u32(const uint8_t *at)
{
  return (uint32_t)at[0] | ((uint32_t)at[1] << 8) | ((uint32_t)at[2] << 16)
         | ((uint32_t)at[3] << 24);
}

static void bno_product_id(world_bno085_t *b, uint8_t entry)
{
  uint8_t cargo[16] = { 0 };

  cargo[0] = PRODUCT_ID_RESPONSE;
  cargo[1] = 1U;
  cargo[2] = 3U;
  cargo[3] = 2U;
  bno_put_u32(&cargo[4], 10003606U + entry);
  bno_put_u32(&cargo[8], 324U);
  bno_send(b, CHANNEL_CONTROL, cargo, sizeof cargo);
}

static void bno_feature_response(world_bno085_t *b, uint8_t id, uint32_t interval_us)
{
  uint8_t cargo[17] = { 0 };

  cargo[0] = GET_FEATURE_RESPONSE;
  cargo[1] = id;
  bno_put_u32(&cargo[5], interval_us);
  bno_send(b, CHANNEL_CONTROL, cargo, sizeof cargo);
}

static void bno_command_response(world_bno085_t *b, uint8_t command)
{
  uint8_t cargo[16] = { 0 };

  cargo[0] = COMMAND_RESPONSE;
  cargo[2] = command;
  bno_send(b, CHANNEL_CONTROL, cargo, sizeof cargo);
}

static void bno_boot(world_bno085_t *b, bool powered)
{
  static const uint8_t RESET_COMPLETE[1] = { 0x01U };

  world_bno085_reset(b);
  if (!powered)
  {
    return;
  }
  bno_send(b, CHANNEL_EXECUTABLE, RESET_COMPLETE, sizeof RESET_COMPLETE);
  bno_command_response(b, 0x84U);
}

static void bno_set_feature(world_bno085_t *b, uint8_t id, uint32_t us)
{
  if (id >= WORLD_BNO_FEATURES)
  {
    return;
  }
  const bool was = b->feature[id].on;

  if (us == 0U)
  {
    b->feature[id].on = false;
    b->features -= was ? 1U : 0U;
  }
  else
  {
    const uint32_t ticks = us * BNO_TICK_HZ / 1000000U;

    b->feature[id].on = true;
    b->feature[id].interval = (ticks > 0U) ? ticks : 1U;
    b->feature[id].elapsed = 0U;
    b->features += was ? 0U : 1U;
  }
  bno_feature_response(b, id, us);
}

static void bno_received(world_bno085_t *b, uint8_t channel, const uint8_t *cargo, uint32_t length,
                         bool powered)
{
  if ((channel == CHANNEL_EXECUTABLE) && (length > 0U) && (cargo[0] == 0x01U))
  {
    bno_boot(b, powered);
    return;
  }
  if ((channel != CHANNEL_CONTROL) || (length == 0U))
  {
    return;
  }
  switch (cargo[0])
  {
    case PRODUCT_ID_REQUEST:
      for (uint8_t entry = 0U; entry < 4U; entry++)
      {
        bno_product_id(b, entry);
      }
      break;
    case SET_FEATURE:
      if (length >= 9U)
      {
        bno_set_feature(b, cargo[1], bno_get_u32(&cargo[5]));
      }
      break;
    case GET_FEATURE:
      if (length >= 2U)
      {
        const uint8_t id = cargo[1];
        const uint32_t us = ((id < WORLD_BNO_FEATURES) && b->feature[id].on)
                              ? (b->feature[id].interval * 1000000U / BNO_TICK_HZ) : 0U;

        bno_feature_response(b, id, us);
      }
      break;
    case COMMAND_REQUEST:
      if (length >= 3U)
      {
        bno_command_response(b, cargo[2]);
      }
      break;
    default:
      break;
  }
}

static void bno_select(world_bno085_t *b, bool powered)
{
  b->selected = true;
  b->woken = false;
  b->in = 0U;
  if (powered && (b->count > 0U))
  {
    b->outgoing = b->packets[b->head];
    b->out_length = b->lengths[b->head];
  }
  else
  {
    b->outgoing = NULL;
    b->out_length = 0U;
  }
}

static void bno_deselect(world_bno085_t *b, bool powered)
{
  b->selected = false;
  if ((b->outgoing != NULL) && (b->in >= b->out_length))
  {
    b->head = (b->head + 1U) % WORLD_BNO_QUEUE;
    b->count--;
  }
  b->outgoing = NULL;
  b->out_length = 0U;
  if (powered && (b->in >= BNO_HEADER))
  {
    const uint32_t length = ((uint32_t)b->incoming[0] | ((uint32_t)b->incoming[1] << 8)) & 0x7FFFU;

    if ((length > BNO_HEADER) && (length <= WORLD_BNO_INCOMING) && (b->in >= length))
    {
      bno_received(b, b->incoming[2], &b->incoming[BNO_HEADER], length - BNO_HEADER, powered);
    }
  }
}

void world_bno085_pin(world_bno085_t *b, uint8_t which, bool level, bool powered)
{
  switch (which)
  {
    case 0U:
      if (!level && !b->selected)
      {
        bno_select(b, powered);
      }
      else if (level && b->selected)
      {
        bno_deselect(b, powered);
      }
      break;
    case 1U:
      b->woken = b->woken || (!level && powered);
      break;
    default:
      if (!level)
      {
        world_bno085_reset(b);
      }
      else
      {
        bno_boot(b, powered);
      }
      break;
  }
}

uint8_t world_bno085_transmit(world_bno085_t *b, uint8_t byte)
{
  if (!b->selected)
  {
    return 0U;
  }
  const uint32_t at = b->in;

  if (b->in < WORLD_BNO_INCOMING)
  {
    b->incoming[b->in] = byte;
  }
  b->in++;
  return ((b->outgoing != NULL) && (at < b->out_length)) ? b->outgoing[at] : 0U;
}

bool world_bno085_interrupt(const world_bno085_t *b, bool powered)
{
  return !(powered && !b->selected && (b->woken || (b->count > 0U)));
}

bool world_bno085_active(const world_bno085_t *b)
{
  return b->features > 0U;
}

static void bno_times(const double *p, const double *q, double *out)
{
  out[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
  out[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
  out[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
  out[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
}

static void bno_axis(int axis, double angle, double *q)
{
  q[0] = cos(angle / 2.0);
  q[1] = q[2] = q[3] = 0.0;
  q[axis] = sin(angle / 2.0);
}

static void bno_turn(world_bno085_t *b, double dt)
{
  double was[4], pitch[4], roll[4], yaw[4], ry[4], now[4], turn[4];

  b->elapsed += dt;
  memcpy(was, b->attitude, sizeof was);
  bno_axis(2, b->pitch_rate * b->elapsed, pitch);
  bno_axis(1, b->roll_rate * b->elapsed, roll);
  bno_axis(3, b->yaw_rate * b->elapsed, yaw);
  bno_times(roll, yaw, ry);
  bno_times(pitch, ry, now);
  memcpy(b->attitude, now, sizeof now);
  const double conj[4] = { was[0], -was[1], -was[2], -was[3] };

  bno_times(conj, now, turn);
  const double sign = (turn[0] < 0.0) ? -1.0 : 1.0;

  for (int i = 0; i < 3; i++)
  {
    b->body[i] = 2.0 * sign * turn[i + 1] / dt;
  }
}

static double bno_to_body(const world_bno085_t *b, double ex, double ey, double ez, int index)
{
  const double w = b->attitude[0], x = b->attitude[1], y = b->attitude[2], z = b->attitude[3];

  switch (index)
  {
    case 0:  return (1 - 2 * (y * y + z * z)) * ex + 2 * (x * y + w * z) * ey
                    + 2 * (x * z - w * y) * ez;
    case 1:  return 2 * (x * y - w * z) * ex + (1 - 2 * (x * x + z * z)) * ey
                    + 2 * (y * z + w * x) * ez;
    default: return 2 * (x * z + w * y) * ex + 2 * (y * z - w * x) * ey
                    + (1 - 2 * (x * x + y * y)) * ez;
  }
}

double world_bno085_read(const world_bno085_t *b, int index)
{
  if (b->piped)
  {
    return b->readings[index];
  }
  if (index < 3)
  {
    return bno_to_body(b, 0.0, 0.0, BNO_GRAVITY, index);
  }
  if (index < 6)
  {
    return b->body[index - 3];
  }
  if (index < 9)
  {
    return bno_to_body(b, FIELD[0], FIELD[1], FIELD[2], index - 6);
  }
  return (index == 12) ? b->attitude[0] : b->attitude[index - 8];
}

void world_bno085_pipe(world_bno085_t *b, int index, double value)
{
  if ((index < 0) || (index >= 13))
  {
    return;
  }
  if (!b->piped)
  {
    for (int i = 0; i < 13; i++)
    {
      b->readings[i] = world_bno085_read(b, i);
    }
    b->piped = true;
  }
  b->readings[index] = value;
}

void world_bno085_imu(world_bno085_t *b, const double *readings)
{
  memcpy(b->readings, readings, sizeof b->readings);
  b->piped = true;
}

static uint32_t bno_q(uint8_t *at, double value, double scale)
{
  const double q = floor(value * scale + 0.5);
  const int16_t v = (int16_t)((q < -32768.0) ? -32768.0 : (q > 32767.0) ? 32767.0 : q);

  at[0] = (uint8_t)v;
  at[1] = (uint8_t)((uint16_t)v >> 8);
  return 2U;
}

static uint32_t bno_report(world_bno085_t *b, uint8_t id, uint8_t *out)
{
  uint32_t n = 0U;

  out[n++] = id;
  out[n++] = b->feature[id].sequence++;
  out[n++] = 3U;
  out[n++] = 0U;
  switch (id)
  {
    case ACCELEROMETER:
    case GYROSCOPE:
    case MAGNETIC_FIELD:
    {
      const int first = (id == ACCELEROMETER) ? 0 : (id == GYROSCOPE) ? 3 : 6;
      const double scale = (id == ACCELEROMETER) ? 256.0 : (id == GYROSCOPE) ? 512.0 : 16.0;

      for (int k = 0; k < 3; k++)
      {
        n += bno_q(&out[n], world_bno085_read(b, first + k), scale);
      }
      break;
    }
    case ROTATION_VECTOR:
    case GAME_ROTATION_VECTOR:
      for (int k = 9; k < 13; k++)
      {
        n += bno_q(&out[n], world_bno085_read(b, k), 16384.0);
      }
      if (id == ROTATION_VECTOR)
      {
        n += bno_q(&out[n], 0.05, 4096.0);
      }
      break;
    default:
      for (int k = 0; k < 3; k++)
      {
        n += bno_q(&out[n], 0.0, 1.0);
      }
      break;
  }
  return n;
}

void world_bno085_tick(world_bno085_t *b, bool powered)
{
  uint8_t reports[WORLD_BNO_PACKET - BNO_HEADER];
  uint32_t n = 0U;

  if (b->features == 0U)
  {
    return;
  }
  if (!powered)
  {
    world_bno085_reset(b);
    return;
  }
  bno_turn(b, 1.0 / (double)BNO_TICK_HZ);
  reports[n++] = TIMEBASE;
  reports[n++] = 0U;
  reports[n++] = 0U;
  reports[n++] = 0U;
  reports[n++] = 0U;
  for (uint8_t id = 0U; id < WORLD_BNO_FEATURES; id++)
  {
    if (!b->feature[id].on || (++b->feature[id].elapsed < b->feature[id].interval)
        || ((n + 20U) > sizeof reports))
    {
      continue;
    }
    b->feature[id].elapsed = 0U;
    n += bno_report(b, id, &reports[n]);
  }
  if (n > 5U)
  {
    bno_send(b, CHANNEL_INPUT, reports, n);
  }
}
