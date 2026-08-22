/**
  ******************************************************************************
  * @file    testrig.h
  * @brief   Raw pin access for a production test fixture.
  *
  * A test rig needs to drive and sense pins directly, with no application
  * meaning attached. That is genuinely useful and genuinely dangerous, so it
  * lives behind a gate: TEST_MODE must be entered with a key before any pin is
  * reconfigured or driven. Reads are always allowed - they cannot break
  * anything and are what a fixture needs most.
  *
  * Two classes of pin are refused outright, in every mode:
  *
  *   PB10, PB11   USART3. Reconfiguring either one severs the very link the
  *                command arrived on, so the rig would lose the board with no
  *                way back except a power cycle.
  *   PA13..PA15,  The debug port, 5-pin JTAG on this board. Losing it means
  *   PB3, PB4     losing the ability to reflash without a bootloader entry.
  *
  * Everything else is fair game while the gate is open, including the pins the
  * application uses. That is the point of a test mode.
  ******************************************************************************
  */
#ifndef TESTRIG_H
#define TESTRIG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Key that must accompany a request to open the gate: ASCII "TEST". */
#define TESTRIG_KEY 0x54455354UL

/** Pin modes, as they appear on the wire. */
#define TESTRIG_MODE_INPUT      0U
#define TESTRIG_MODE_OUTPUT_PP  1U
#define TESTRIG_MODE_OUTPUT_OD  2U
#define TESTRIG_MODE_ANALOG     3U

/** Pull configuration, as it appears on the wire. */
#define TESTRIG_PULL_NONE 0U
#define TESTRIG_PULL_UP   1U
#define TESTRIG_PULL_DOWN 2U

bool testrig_open(void);

/**
  * @brief  Open or close the gate.
  * @param  key  Must be TESTRIG_KEY, so the mode cannot be entered by a stray
  *              frame or a mistyped command.
  * @return False if the key was wrong; the gate is then left as it was.
  */
bool testrig_gate(uint32_t key, bool open);

/**
  * @brief  Is this pin allowed to be touched at all?
  * @param  port  'A'..'K'.
  * @param  pin   0..15.
  */
bool testrig_pin_allowed(char port, uint8_t pin);

/* Each of these returns false only when the request itself is impossible: an
   unknown port, a pin out of range, a reserved pin, or the gate being shut for
   an operation that needs it. */
bool testrig_pin_mode(char port, uint8_t pin, uint8_t mode, uint8_t pull);
bool testrig_pin_read(char port, uint8_t pin, bool *level);
bool testrig_pin_write(char port, uint8_t pin, bool level);
bool testrig_port_read(char port, uint16_t *value);
bool testrig_port_write(char port, uint16_t mask, uint16_t value);

#ifdef __cplusplus
}
#endif

#endif /* TESTRIG_H */
