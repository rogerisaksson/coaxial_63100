/**
  ******************************************************************************
  * @file    cmd_length.h
  * @brief   The request-length oracle mb_rtu's early path asks.
  ******************************************************************************
  */
#ifndef CMD_LENGTH_H
#define CMD_LENGTH_H

#include <stdint.h>

/** Full PDU length of the request these bytes begin, or 0 when the bytes
  * so far cannot prove it. See cmd_length.c for the invariant the answer
  * lives under - a wrong non-zero here executes a truncated frame. */
uint16_t cmd_request_length(const uint8_t *pdu, uint16_t have);

#endif
