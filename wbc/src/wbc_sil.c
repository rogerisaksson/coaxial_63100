/** wbc_sil.c - her 5 extremity 10 Mbit/s RS485 Modbus RTU buses and 27 SIL coaxial_63100 boards
    (machine.senses.edge.LIMBS, machine.senses.edge.drive), executing firmware modbus_rtu + modbus_slave state
    machines and CRC-16/MODBUS on every frame, through a line that flips bits (`wbc_sil_noise`,
    `wbc_sil_burst`): a frame a board refuses leaves its last setpoint, the next pass sends anew.
    Portable C11. */
#include "wbc_pass.h"
#include "modbus_crc.h"
#include "modbus_rtu.h"
#include "modbus_slave.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define SIL_REG_SETPOINT 0x0100U
#define SIL_REG_KP       0x0140U
#define SIL_REG_FEED     0x0180U
#define SIL_REG_LIMIT    0x01C0U

typedef struct
{
  mb_data_model_t model;
  mb_slave_t      slave;
  mb_rtu_t        rtu;
  int             joint_idx;
  uint8_t         bus_id;
  uint8_t         unit_id;
  uint16_t        hi_word;
  double          target;
  double          feed;
  double          kp;
  double          limit;
} wbc_sil_board_t;

typedef struct
{
  int                count;
  int                joints[8];
  int                board_at[8];
  int                sent_mask;
  int32_t            last_val[4][8];
  unsigned long long tx_bytes;
  unsigned long long frames;
  unsigned long long refused;
  int                stale;
  int                burst;
  double             wire_s;
  uint32_t           ticks;
  unsigned char      last_frame[64];
  int                last_len;
} wbc_sil_bus_t;

static struct
{
  wbc_world_t    *w;
  double          ber;
  uint32_t        draw;
  int             nbuses;
  int             nboards;
  wbc_sil_bus_t   buses[WBC_SIL_BUSES];
  wbc_sil_board_t boards[WBC_PASS_MAX];
} sil_state;

static mb_exception_t sil_validate_range(void *ctx, mb_table_t table, uint16_t addr, uint16_t qty, bool for_write)
{
  const uint16_t bank = (uint16_t)(addr & 0xFFC0U);

  (void)ctx;
  if ((table != MB_TABLE_HOLDING_REG) || !for_write || ((qty & 1U) != 0U))
  {
    return MB_EX_ILLEGAL_DATA_ADDRESS;
  }
  if ((bank != SIL_REG_SETPOINT) && (bank != SIL_REG_KP) && (bank != SIL_REG_FEED) && (bank != SIL_REG_LIMIT))
  {
    return MB_EX_ILLEGAL_DATA_ADDRESS;
  }
  return MB_EX_NONE;
}

static mb_exception_t sil_write_reg(void *ctx, uint16_t addr, uint16_t value)
{
  wbc_sil_board_t *bd = (wbc_sil_board_t *)ctx;
  const uint16_t   bank = (uint16_t)(addr & 0xFFC0U);
  const uint16_t   unit_base = (uint16_t)(bank + (uint16_t)(2U * (uint16_t)(bd->unit_id - 1U)));

  if (addr == unit_base)
  {
    bd->hi_word = value;
  }
  else if (addr == (uint16_t)(unit_base + 1U))
  {
    const uint32_t raw_u = ((uint32_t)bd->hi_word << 16) | (uint32_t)value;
    const int32_t  raw = (int32_t)raw_u;
    wbc_world_t   *w = sil_state.w;
    const int      j = bd->joint_idx;

    if (bank == SIL_REG_SETPOINT)
    {
      bd->target = (double)raw * 1e-6;
      if (w != NULL)
      {
        ((double *)(uintptr_t)w->target)[j] = bd->target;
      }
    }
    else if (bank == SIL_REG_FEED)
    {
      bd->feed = (double)raw * 1e-3;
      if (w != NULL)
      {
        ((double *)(uintptr_t)w->feed)[j] = bd->feed;
      }
    }
    else if (bank == SIL_REG_LIMIT)
    {
      bd->limit = (double)raw * 1e-3;
      if (w != NULL)
      {
        ((double *)(uintptr_t)w->limit)[j] = bd->limit;
      }
    }
    else if (bank == SIL_REG_KP)
    {
      bd->kp = (double)raw * 1e-3;
      if (w != NULL)
      {
        ((double *)(uintptr_t)w->gains)[2 * j] = bd->kp;
      }
    }
  }
  return MB_EX_NONE;
}

/* xorshift32: the line's draw. */
static uint32_t sil_draw(void)
{
  uint32_t x = sil_state.draw;

  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  sil_state.draw = x;
  return x;
}

/* `frame` as the wire carries it into `wire`: a bit flipped in a burst or by the line's noise. */
static void sil_line(wbc_sil_bus_t *bus, const unsigned char *frame, int len, uint8_t *wire)
{
  const double hit = 1.0 - pow(1.0 - sil_state.ber, 10.0 * (double)len);
  const int    flip = (bus->burst > 0) || ((hit > 0.0) && ((double)sil_draw() < hit * 4294967296.0));

  memcpy(wire, frame, (size_t)len);
  bus->burst -= (bus->burst > 0);
  if (flip)
  {
    const uint32_t bit = sil_draw() % (uint32_t)(8 * len);

    wire[bit >> 3] ^= (uint8_t)(1U << (bit & 7U));
  }
}

void wbc_sil_init(wbc_world_t *w, int nbuses, const int *counts, const int *joints)
{
  int at = 0;

  memset(&sil_state, 0, sizeof(sil_state));
  sil_state.w = w;
  sil_state.nbuses = nbuses < WBC_SIL_BUSES ? nbuses : WBC_SIL_BUSES;
  for (int b = 0; b < sil_state.nbuses; b++)
  {
    wbc_sil_bus_t *bus = &sil_state.buses[b];
    const int      cnt = counts[b] < 8 ? counts[b] : 8;

    bus->count = cnt;
    for (int u = 0; u < cnt; u++)
    {
      const int        j = joints[at];
      wbc_sil_board_t *bd = &sil_state.boards[at];

      bus->joints[u] = j;
      bus->board_at[u] = at;
      bd->joint_idx = j;
      bd->bus_id = (uint8_t)b;
      bd->unit_id = (uint8_t)(u + 1);
      if (w != NULL)
      {
        bd->target = w->target[j];
        bd->feed = w->feed[j];
        bd->kp = w->gains[2 * j];
        bd->limit = w->limit[j];
      }
      bd->model.validate_range = sil_validate_range;
      bd->model.write_reg = sil_write_reg;
      bd->model.ctx = bd;
      mb_slave_init(&bd->slave, &bd->model);
      mb_rtu_init(&bd->rtu, &bd->slave, bd->unit_id, WBC_SIL_BAUD, 10U, 1U);
      at++;
    }
  }
  sil_state.nboards = at;
}

int wbc_sil_transceive(wbc_world_t *w, int bus_id, const unsigned char *frame, int len)
{
  wbc_sil_bus_t *bus;
  uint16_t       addr, qty, bank, first_unit;
  uint8_t        wire[64];
  int            refused = 0;

  if ((bus_id < 0) || (bus_id >= sil_state.nbuses) || (len < 9) || (len > 64))
  {
    return -1;
  }
  bus = &sil_state.buses[bus_id];
  if ((frame[0] != MB_RTU_BROADCAST) || (frame[1] != MB_FC_WRITE_MULTIPLE_REGS))
  {
    return -1;
  }
  addr = (uint16_t)(((uint16_t)frame[2] << 8) | (uint16_t)frame[3]);
  qty = (uint16_t)(((uint16_t)frame[4] << 8) | (uint16_t)frame[5]);
  bank = (uint16_t)(addr & 0xFFC0U);
  first_unit = (uint16_t)(((uint16_t)(addr - bank) / 2U) + 1U);
  if ((first_unit != 1U) || ((int)(qty / 2U) != bus->count) || ((int)frame[6] != 4 * bus->count)
      || (len != 9 + 4 * bus->count))
  {
    return -2;
  }
  sil_state.w = w;
  sil_line(bus, frame, len, wire);
  for (int u = 0; u < bus->count; u++)
  {
    wbc_sil_board_t *bd = &sil_state.boards[bus->board_at[u]];
    const uint32_t   err_before = bd->rtu.counters.bus_comm_error;
    const uint32_t   ex_before = bd->rtu.counters.server_exception;
    const uint8_t   *rsp = NULL;

    for (int k = 0; k < len; k++)
    {
      mb_rtu_on_byte(&bd->rtu, wire[k], bus->ticks);
    }
    bus->ticks += bd->rtu.t35_ticks + 1U;
    (void)mb_rtu_service(&bd->rtu, bus->ticks, &rsp);
    refused |= (bd->rtu.counters.bus_comm_error != err_before) || (bd->rtu.counters.server_exception != ex_before);
  }
  bus->tx_bytes += (unsigned long long)len;
  bus->frames += 1ULL;
  bus->refused += (unsigned long long)refused;
  bus->wire_s += (double)len * 10.0 / (double)WBC_SIL_BAUD;
  if (bank == SIL_REG_SETPOINT)
  {
    bus->stale = refused ? bus->stale + 1 : 0;
  }
  memcpy(bus->last_frame, wire, (size_t)len);
  bus->last_len = len;
  return refused ? -3 : 0;
}

void wbc_sil_drive(wbc_world_t *w, int n_idx, const int *idx, const double *target,
                   const double *feed, const double *kp, const double *limit)
{
  const uint16_t regs[4] = {SIL_REG_SETPOINT, SIL_REG_FEED, SIL_REG_LIMIT, SIL_REG_KP};
  const double   scales[4] = {1e6, 1e3, 1e3, 1e3};
  const double  *inputs[4] = {target, feed, limit, kp};
  const double  *curs[4] = {w->target, w->feed, w->limit, w->gains};
  const int      strides[4] = {1, 1, 1, 2};
  int            pos[WBC_PASS_MAX];

  for (int i = 0; i < WBC_PASS_MAX; i++)
  {
    pos[i] = -1;
  }
  for (int k = 0; k < n_idx; k++)
  {
    const int j = idx[k];
    if ((j >= 0) && (j < WBC_PASS_MAX))
    {
      pos[j] = k;
    }
  }
  for (int r = 0; r < 4; r++)
  {
    const double  *arr = inputs[r];
    const double  *cur = curs[r];
    const int      st = strides[r];
    const double   sc = scales[r];
    const uint16_t reg = regs[r];

    if (arr == NULL)
    {
      continue;
    }
    for (int b = 0; b < sil_state.nbuses; b++)
    {
      wbc_sil_bus_t *bus = &sil_state.buses[b];
      const int      cnt = bus->count;
      int            any = 0, changed = (r < 2) || ((bus->sent_mask & (1 << r)) == 0);
      uint8_t        adu[64];

      for (int u = 0; u < cnt; u++)
      {
        if (pos[bus->joints[u]] >= 0)
        {
          any = 1;
          break;
        }
      }
      if (!any)
      {
        continue;
      }
      adu[0] = MB_RTU_BROADCAST;
      adu[1] = MB_FC_WRITE_MULTIPLE_REGS;
      adu[2] = (uint8_t)(reg >> 8);
      adu[3] = (uint8_t)(reg & 0xFFU);
      adu[4] = 0U;
      adu[5] = (uint8_t)(2 * cnt);
      adu[6] = (uint8_t)(4 * cnt);
      for (int u = 0; u < cnt; u++)
      {
        const int      j = bus->joints[u];
        const int      s = pos[j];
        const double   v = (s >= 0) ? arr[s] : cur[st * j];
        const int32_t  iv = (int32_t)llround(v * sc);
        const int32_t  cv = (int32_t)llround(cur[st * j] * sc);
        const uint32_t uv = (uint32_t)iv;

        if (iv != cv)
        {
          changed = 1;
        }
        adu[7 + 4 * u]     = (uint8_t)(uv >> 24);
        adu[7 + 4 * u + 1] = (uint8_t)((uv >> 16) & 0xFFU);
        adu[7 + 4 * u + 2] = (uint8_t)((uv >> 8) & 0xFFU);
        adu[7 + 4 * u + 3] = (uint8_t)(uv & 0xFFU);
      }
      if (!changed)
      {
        continue;
      }
      bus->sent_mask |= (1 << r);
      {
        const size_t flen = modbus_crc_append(adu, (size_t)(7 + 4 * cnt));
        (void)wbc_sil_transceive(w, b, adu, (int)flen);
      }
    }
  }
}

void wbc_sil_stats(int bus_id, unsigned long long *counts, double *wire_s,
                   unsigned char *last_frame, int *last_len)
{
  const wbc_sil_bus_t *bus;
  unsigned long long   msgs = 0ULL;

  if ((bus_id < 0) || (bus_id >= sil_state.nbuses))
  {
    return;
  }
  bus = &sil_state.buses[bus_id];
  for (int u = 0; u < bus->count; u++)
  {
    const wbc_sil_board_t *bd = &sil_state.boards[bus->board_at[u]];
    msgs += (unsigned long long)bd->rtu.counters.server_message;
  }
  counts[0] = bus->tx_bytes;
  counts[1] = bus->frames;
  counts[2] = bus->refused;
  counts[3] = msgs;
  counts[4] = (unsigned long long)bus->stale;
  *wire_s = bus->wire_s;
  if ((last_frame != NULL) && (bus->last_len > 0))
  {
    memcpy(last_frame, bus->last_frame, (size_t)bus->last_len);
  }
  if (last_len != NULL)
  {
    *last_len = bus->last_len;
  }
}

void wbc_sil_noise(double ber, uint32_t seed)
{
  sil_state.ber = ber;
  sil_state.draw = seed | 1U;
}

void wbc_sil_burst(int bus_id, int frames)
{
  if ((bus_id >= 0) && (bus_id < sil_state.nbuses))
  {
    sil_state.buses[bus_id].burst = frames;
  }
}

int wbc_sil_stalest(int *bus_id)
{
  int most = 0;

  *bus_id = -1;
  for (int b = 0; b < sil_state.nbuses; b++)
  {
    if (sil_state.buses[b].stale > most)
    {
      most = sil_state.buses[b].stale;
      *bus_id = b;
    }
  }
  return most;
}

void wbc_sil_off(wbc_world_t *w)
{
  for (int k = 0; k < sil_state.nboards; k++)
  {
    wbc_sil_board_t *bd = &sil_state.boards[k];

    bd->limit = 0.0;
    ((double *)(uintptr_t)w->limit)[bd->joint_idx] = 0.0;
  }
}
