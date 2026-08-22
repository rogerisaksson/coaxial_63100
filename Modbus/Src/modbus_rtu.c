/**
  ******************************************************************************
  * @file    modbus_rtu.c
  * @brief   Modbus RTU framing state machine. No hardware dependencies.
  ******************************************************************************
  */
#include "modbus_rtu.h"
#include "modbus_crc.h"

#include <string.h>

/* Wrap-safe elapsed ticks. Performed in uint32_t so a counter that has wrapped
   once since the reference still yields the correct difference - which holds
   only because the tick counter wraps at exactly 2^32. This is the sole place
   time is compared, deliberately. */
static uint32_t elapsed(uint32_t now, uint32_t then)
{
  return (uint32_t)(now - then);
}

void mb_rtu_init(mb_rtu_t *rtu, mb_slave_t *slave, uint8_t unit_id,
                 uint32_t baud, uint8_t bits_per_char, uint32_t ticks_per_us)
{
  uint32_t t15_us;
  uint32_t t35_us;

  memset(rtu, 0, sizeof(*rtu));

  rtu->slave   = slave;
  rtu->unit_id = unit_id;

  if (baud > 19200U)
  {
    /* V1.02 recommends these fixed values above 19200 baud rather than the
       derived ones, because the derived silences become so short that the
       processing overhead of measuring them dominates. */
    t15_us = 750U;
    t35_us = 1750U;
  }
  else
  {
    /* Character time in microseconds, rounded up so the silence is never
       computed shorter than the specification requires. */
    const uint32_t bits = (bits_per_char == 0U) ? 11U : (uint32_t)bits_per_char;
    const uint32_t char_us = ((bits * 1000000UL) + (baud - 1U)) / baud;

    t15_us = ((char_us * 3U) + 1U) / 2U;   /* 1.5 characters */
    t35_us = ((char_us * 7U) + 1U) / 2U;   /* 3.5 characters */
  }

  if (ticks_per_us == 0U)
  {
    ticks_per_us = 1U;
  }

  rtu->t15_ticks = t15_us * ticks_per_us;
  rtu->t35_ticks = t35_us * ticks_per_us;

  /* Until t3.5 of silence has been observed we do not know where we are in a
     frame, so start out as if a frame were in progress and let the first
     timeout put us into a known idle state. */
  rtu->receiving = false;
  rtu->frame_bad = false;
  rtu->rx_len    = 0U;
}

void mb_rtu_on_byte(mb_rtu_t *rtu, uint8_t byte, uint32_t now_ticks)
{
  if (rtu->receiving)
  {
    /* A gap longer than t1.5 inside a frame means the frame is not a frame.
       It must still be drained and discarded, not truncated and parsed. */
    if (elapsed(now_ticks, rtu->last_event_ticks) > rtu->t15_ticks)
    {
      rtu->frame_bad = true;
    }
  }
  else
  {
    rtu->receiving = true;
    rtu->rx_len    = 0U;
    rtu->frame_bad = false;
  }

  if (rtu->rx_len < MB_RTU_ADU_MAX)
  {
    rtu->rx[rtu->rx_len] = byte;
    rtu->rx_len++;
  }
  else
  {
    /* Longer than any legal ADU. Keep consuming so the line can drain, but
       the frame is already lost. */
    rtu->frame_bad = true;
  }

  rtu->last_event_ticks = now_ticks;
}

void mb_rtu_on_error(mb_rtu_t *rtu, uint32_t now_ticks)
{
  rtu->frame_bad   = true;
  rtu->saw_overrun = true;
  rtu->receiving   = true;   /* force the drain-and-discard path */
  rtu->last_event_ticks = now_ticks;
}

bool mb_rtu_busy(const mb_rtu_t *rtu)
{
  return rtu->receiving;
}

size_t mb_rtu_service(mb_rtu_t *rtu, uint32_t now_ticks, const uint8_t **out)
{
  *out = NULL;

  if (!rtu->receiving)
  {
    return 0U;
  }

  /* A frame ends when, and only when, the line has been quiet for t3.5. */
  if (elapsed(now_ticks, rtu->last_event_ticks) < rtu->t35_ticks)
  {
    return 0U;
  }

  const uint16_t len = rtu->rx_len;
  const bool     bad = rtu->frame_bad;
  const bool     ovr = rtu->saw_overrun;

  rtu->receiving   = false;
  rtu->frame_bad   = false;
  rtu->saw_overrun = false;
  rtu->rx_len      = 0U;

  if (len == 0U)
  {
    return 0U;
  }

  rtu->counters.bus_message++;

  if (ovr)
  {
    rtu->counters.char_overrun++;
  }

  if (bad || (len < MB_RTU_ADU_MIN))
  {
    rtu->counters.bus_comm_error++;
    return 0U;
  }

  if (modbus_crc_check(rtu->rx, len) == 0)
  {
    /* A bad CRC is answered with silence, never with an exception. Replying
       would put a frame on the bus that the master cannot correlate, and on a
       multidrop line it may not even have been addressed to us. */
    rtu->counters.bus_comm_error++;
    return 0U;
  }

  const uint8_t addr = rtu->rx[0];

  if ((addr != rtu->unit_id) && (addr != MB_RTU_BROADCAST))
  {
    /* Intended for another server. Not an error, not ours, not counted. */
    return 0U;
  }

  rtu->counters.server_message++;

  /* PDU is the frame less the unit id and the two CRC bytes. */
  const uint8_t *pdu     = &rtu->rx[1];
  const size_t   pdu_len = (size_t)(len - 3U);

  const size_t rsp_len = mb_slave_execute(rtu->slave, pdu, pdu_len,
                                          &rtu->tx[1], MB_RTU_ADU_MAX - 3U);

  if (rsp_len == 0U)
  {
    rtu->counters.server_no_response++;
    return 0U;
  }

  if ((rtu->tx[1] & 0x80U) != 0U)
  {
    rtu->counters.server_exception++;
  }

  if (addr == MB_RTU_BROADCAST)
  {
    /* The request was executed; a broadcast is never answered. */
    rtu->counters.server_no_response++;
    return 0U;
  }

  rtu->tx[0] = rtu->unit_id;
  rtu->tx_len = (uint16_t)modbus_crc_append(rtu->tx, rsp_len + 1U);

  *out = rtu->tx;
  return rtu->tx_len;
}
