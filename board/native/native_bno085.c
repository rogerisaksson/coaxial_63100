/** native_bno085.c - The BNO085 on SPI2 natively: Coaxial63100_BNO085.cs in C. */

/* SHTP packets as board_imu.c drives them: the part's clocked out whole under one chip select
   while the host's goes in; H_INTN low while a packet waits or a WAKE asks, released as the
   chip select falls; NRSTN rising boots it - reset complete on channel 1, the initialize
   response on 2. On channel 2 it answers Product ID (0xF9, four 0xF8s) and Get Feature (0xFE);
   Set Feature (0xFD) answers 0xFC and starts that report on channel 3 at its interval, behind
   a timebase reference: accelerometer Q8 m/s^2, gyroscope Q9 rad/s, magnetic field Q4 uT,
   rotation vectors Q14. It moves: the stand-in's tumble in virtual time, a turn of roll and
   two of pitch in 2.56 s, gravity and the field in the body frame off that attitude; readings
   piped from the host (native_imu) are given instead. AFE_ON powers it. */
#include "native.h"

#include <math.h>
#include <string.h>

#define BNO_HEADER        4U
#define BNO_QUEUE         32U
#define BNO_PACKET        192U
#define BNO_INCOMING      512U
#define BNO_FEATURES      64U
#define BNO_TICK_HZ       1000U
#define BNO_READINGS      13U
#define BNO_TWO_PI        6.283185307179586
#define BNO_GRAVITY       9.80665

#define CHANNEL_EXECUTABLE  1U
#define CHANNEL_CONTROL     2U
#define CHANNEL_INPUT       3U
#define PRODUCT_ID_REQUEST  0xF9U
#define PRODUCT_ID_RESPONSE 0xF8U
#define SET_FEATURE         0xFDU
#define GET_FEATURE         0xFEU
#define GET_FEATURE_RESPONSE 0xFCU
#define COMMAND_REQUEST     0xF2U
#define TIMEBASE            0xFBU
#define ACCELEROMETER       0x01U
#define GYROSCOPE           0x02U
#define MAGNETIC_FIELD      0x03U
#define ROTATION_VECTOR     0x05U
#define GAME_ROTATION_VECTOR 0x08U

/* The earth's field where the stand-in's is, uT. */
static const double FIELD[3] = { 22.0, -3.0, 41.0 };

static struct
{
  uint8_t  queue[BNO_QUEUE][BNO_PACKET];
  uint16_t length[BNO_QUEUE];
  uint32_t head;
  uint32_t count;

  struct
  {
    bool     on;
    uint32_t interval;            /* ticks */
    uint32_t elapsed;
    uint8_t  sequence;
  } feature[BNO_FEATURES];
  uint32_t features;

  uint8_t  incoming[BNO_INCOMING];
  uint32_t in;
  const uint8_t *outgoing;
  uint32_t out_length;
  uint8_t  sequence[6];
  bool     selected;
  bool     woken;

  double   attitude[4];           /* w, x, y, z */
  double   body[3];
  double   elapsed;
  double   roll_rate, pitch_rate, yaw_rate;
  bool     piped;
  double   readings[BNO_READINGS];
} b;

static void bno_reset(void)
{
  b.head = 0U;
  b.count = 0U;
  memset(b.feature, 0, sizeof b.feature);
  b.features = 0U;
  memset(b.sequence, 0, sizeof b.sequence);
  b.selected = false;
  b.woken = false;
  b.outgoing = NULL;
  b.out_length = 0U;
}

void bno085_open(void)
{
  memset(&b, 0, sizeof b);
  b.attitude[0] = 1.0;
  b.roll_rate = BNO_TWO_PI / 2.56;
  b.pitch_rate = 2.0 * BNO_TWO_PI / 2.56;
}

static void bno_send(uint8_t channel, const uint8_t *cargo, uint32_t n)
{
  if ((b.count == BNO_QUEUE) || ((BNO_HEADER + n) > BNO_PACKET))
  {
    return;                       /* the host has stopped reading: the part drops it */
  }
  const uint32_t at = (b.head + b.count) % BNO_QUEUE;
  uint8_t *packet = b.queue[at];
  const uint32_t length = BNO_HEADER + n;

  packet[0] = (uint8_t)length;
  packet[1] = (uint8_t)(length >> 8);
  packet[2] = channel;
  packet[3] = b.sequence[channel]++;
  memcpy(&packet[BNO_HEADER], cargo, n);
  b.length[at] = (uint16_t)length;
  b.count++;
}

static void bno_u32(uint8_t *at, uint32_t value)
{
  at[0] = (uint8_t)value;
  at[1] = (uint8_t)(value >> 8);
  at[2] = (uint8_t)(value >> 16);
  at[3] = (uint8_t)(value >> 24);
}

static void bno_command_response(uint8_t command)
{
  uint8_t cargo[16] = { 0xF1U };

  cargo[2] = command;
  bno_send(CHANNEL_CONTROL, cargo, sizeof cargo);
}

static void bno_feature_response(uint8_t id, uint32_t interval_us)
{
  uint8_t cargo[17] = { GET_FEATURE_RESPONSE };

  cargo[1] = id;
  bno_u32(&cargo[5], interval_us);
  bno_send(CHANNEL_CONTROL, cargo, sizeof cargo);
}

static void bno_boot(void)
{
  static const uint8_t reset_complete[1] = { 0x01U };

  bno_reset();
  if (!native_powered())
  {
    return;
  }
  bno_send(CHANNEL_EXECUTABLE, reset_complete, sizeof reset_complete);
  bno_command_response(0x84U);    /* initialize, unsolicited */
}

static void bno_received(uint8_t channel, const uint8_t *cargo, uint32_t n)
{
  if ((channel == CHANNEL_EXECUTABLE) && (n > 0U) && (cargo[0] == 0x01U))
  {
    bno_boot();
    return;
  }
  if ((channel != CHANNEL_CONTROL) || (n == 0U))
  {
    return;
  }
  switch (cargo[0])
  {
    case PRODUCT_ID_REQUEST:
      for (uint32_t entry = 0U; entry < 4U; entry++)
      {
        /* Figure 1-29: power-on reset, SW 3.2, part 10003606 + entry, build 324. */
        uint8_t id[16] = { PRODUCT_ID_RESPONSE, 1U, 3U, 2U };

        bno_u32(&id[4], 10003606U + entry);
        bno_u32(&id[8], 324U);
        bno_send(CHANNEL_CONTROL, id, sizeof id);
      }
      break;
    case SET_FEATURE:
      if ((n >= 9U) && (cargo[1] < BNO_FEATURES))
      {
        const uint8_t id = cargo[1];
        const uint32_t us = (uint32_t)cargo[5] | ((uint32_t)cargo[6] << 8)
                            | ((uint32_t)cargo[7] << 16) | ((uint32_t)cargo[8] << 24);

        if (us == 0U)
        {
          b.features -= b.feature[id].on ? 1U : 0U;
          b.feature[id].on = false;
        }
        else
        {
          const uint32_t ticks = (uint32_t)(((uint64_t)us * BNO_TICK_HZ) / 1000000U);

          b.features += b.feature[id].on ? 0U : 1U;
          b.feature[id].on = true;
          b.feature[id].interval = (ticks > 1U) ? ticks : 1U;
        }
        bno_feature_response(id, us);
      }
      break;
    case GET_FEATURE:
      if ((n >= 2U) && (cargo[1] < BNO_FEATURES))
      {
        const uint8_t id = cargo[1];

        bno_feature_response(id, b.feature[id].on
                                 ? (uint32_t)(((uint64_t)b.feature[id].interval * 1000000U)
                                              / BNO_TICK_HZ)
                                 : 0U);
      }
      break;
    case COMMAND_REQUEST:
      if (n >= 3U)
      {
        bno_command_response(cargo[2]);
      }
      break;
    default:
      break;
  }
}

static void bno_select(void)
{
  b.selected = true;
  b.woken = false;
  b.in = 0U;
  b.outgoing = (native_powered() && (b.count > 0U)) ? b.queue[b.head] : NULL;
  b.out_length = (b.outgoing != NULL) ? b.length[b.head] : 0U;
}

static void bno_deselect(void)
{
  b.selected = false;
  if ((b.outgoing != NULL) && (b.in >= b.out_length))
  {
    b.head = (b.head + 1U) % BNO_QUEUE;        /* clocked out whole */
    b.count--;
  }
  b.outgoing = NULL;
  if (native_powered() && (b.in >= BNO_HEADER))
  {
    const uint32_t length = ((uint32_t)b.incoming[0] | ((uint32_t)b.incoming[1] << 8)) & 0x7FFFU;

    if ((length > BNO_HEADER) && (b.in >= length))
    {
      bno_received(b.incoming[2], &b.incoming[BNO_HEADER], length - BNO_HEADER);
    }
  }
}

void bno085_pin(uint8_t which, bool level)
{
  switch (which)
  {
    case 0U:
      if (!level && !b.selected)
      {
        bno_select();
      }
      else if (level && b.selected)
      {
        bno_deselect();
      }
      break;
    case 1U:
      b.woken = b.woken || (!level && native_powered());
      break;
    default:
      if (!level)
      {
        bno_reset();
      }
      else
      {
        bno_boot();
      }
      break;
  }
}

uint8_t bno085_transmit(uint8_t byte)
{
  if (!b.selected)
  {
    return 0U;
  }
  const uint32_t at = b.in;

  if (b.in < BNO_INCOMING)
  {
    b.incoming[b.in] = byte;
  }
  b.in++;
  return ((b.outgoing != NULL) && (at < b.out_length)) ? b.outgoing[at] : 0U;
}

/** H_INTN, active low. */
bool bno085_interrupt(void)
{
  return !(native_powered() && !b.selected && (b.woken || (b.count > 0U)));
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

/* The attitude a tick on, and the body rate that turn was: 2 vec(q* q') / dt. */
static void bno_turn(double dt)
{
  double was[4], pitch[4], roll[4], yaw[4], ry[4], now[4], turn[4];

  b.elapsed += dt;
  memcpy(was, b.attitude, sizeof was);
  bno_axis(2, b.pitch_rate * b.elapsed, pitch);
  bno_axis(1, b.roll_rate * b.elapsed, roll);
  bno_axis(3, b.yaw_rate * b.elapsed, yaw);
  bno_times(roll, yaw, ry);
  bno_times(pitch, ry, now);
  memcpy(b.attitude, now, sizeof now);
  const double conj[4] = { was[0], -was[1], -was[2], -was[3] };

  bno_times(conj, now, turn);
  const double sign = (turn[0] < 0.0) ? -1.0 : 1.0;

  for (int i = 0; i < 3; i++)
  {
    b.body[i] = 2.0 * sign * turn[i + 1] / dt;
  }
}

/* An earth-frame vector in the body frame: R(q) transposed. */
static double bno_to_body(double ex, double ey, double ez, int index)
{
  const double w = b.attitude[0], x = b.attitude[1], y = b.attitude[2], z = b.attitude[3];

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

/* Reading `index` - accel xyz, gyro xyz, mag xyz, quaternion i j k real - off the attitude,
   or as piped. */
static double bno_read(int index)
{
  if (b.piped)
  {
    return b.readings[index];
  }
  if (index < 3)
  {
    return bno_to_body(0.0, 0.0, BNO_GRAVITY, index);
  }
  if (index < 6)
  {
    return b.body[index - 3];
  }
  if (index < 9)
  {
    return bno_to_body(FIELD[0], FIELD[1], FIELD[2], index - 6);
  }
  return (index == 12) ? b.attitude[0] : b.attitude[index - 8];
}

static uint32_t bno_q(uint8_t *at, double value, double scale)
{
  const double q = floor(value * scale + 0.5);
  const int16_t v = (int16_t)((q < -32768.0) ? -32768.0 : (q > 32767.0) ? 32767.0 : q);

  at[0] = (uint8_t)v;
  at[1] = (uint8_t)((uint16_t)v >> 8);
  return 2U;
}

static uint32_t bno_report(uint8_t id, uint8_t *out)
{
  uint32_t n = 0U;

  out[n++] = id;
  out[n++] = b.feature[id].sequence++;
  out[n++] = 3U;                  /* status 3: accuracy high */
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
        n += bno_q(&out[n], bno_read(first + k), scale);
      }
      break;
    }
    case ROTATION_VECTOR:
    case GAME_ROTATION_VECTOR:
      for (int k = 9; k < 13; k++)
      {
        n += bno_q(&out[n], bno_read(k), 16384.0);
      }
      if (id == ROTATION_VECTOR)
      {
        n += bno_q(&out[n], 0.05, 4096.0);    /* accuracy, rad Q12 */
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

/** A millisecond: the reports due, behind a timebase reference. */
void bno085_tick(void)
{
  uint8_t reports[BNO_PACKET - BNO_HEADER];
  uint32_t n = 0U;

  if (b.features == 0U)
  {
    return;
  }
  if (!native_powered())
  {
    bno_reset();                  /* its supply gone, what it was told is gone */
    return;
  }
  bno_turn(1.0 / (double)BNO_TICK_HZ);
  reports[n++] = TIMEBASE;
  reports[n++] = 0U;
  reports[n++] = 0U;
  reports[n++] = 0U;
  reports[n++] = 0U;
  for (uint8_t id = 0U; id < BNO_FEATURES; id++)
  {
    if (!b.feature[id].on || (++b.feature[id].elapsed < b.feature[id].interval)
        || ((n + 20U) > sizeof reports))
    {
      continue;
    }
    b.feature[id].elapsed = 0U;
    n += bno_report(id, &reports[n]);
  }
  if (n > 5U)
  {
    bno_send(CHANNEL_INPUT, reports, n);
  }
}

/** The host's readings - accel xyz m/s^2, gyro xyz rad/s, mag xyz uT, quaternion i j k real -
    in place of the tumble: a SIL's body. */
void native_imu(const double *readings)
{
  memcpy(b.readings, readings, sizeof b.readings);
  b.piped = true;
}
