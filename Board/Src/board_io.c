/**
  ******************************************************************************
  * @file    board_io.c
  * @brief   Discrete I/O: AFE_ON and PE15, plus the console-mode request.
  ******************************************************************************
  */
#include "board.h"
#include "board_hw.h"

#include "link.h"


/* Every pin this board uses for something, reserved ones included. The
   direction is the MCU's: PB2 drives the AFE switch, PE15 senses it back
   inverted (HARDWARE.md, Discrete I/O). Kept here rather than in testrig.c
   so "what is PB10" has one answer. */
typedef struct
{
  char        port;
  uint8_t     number;
  const char *pin;
  uint8_t     dir;
  const char *signal;
  /** A host may drive it through the test path. FALSE for anything an
      alternate function owns: HAL_GPIO_Init would take the pin off that
      function, and for a gate signal that is one FET latched on against
      one still switching. */
  bool        usable;
  /** It goes in a DAQ record. A DIFFERENT QUESTION - reading a pin costs
      it nothing, so the six gates and the break belong in a measurement
      even though none of them may be written. What stays out is the
      buses and the debug port: sampling SPI or JTAG at the converters'
      rate names a channel nobody asked for, and all twenty-three
      overflowed the layout reply at 312 bytes against MB_MAX_PDU's
      253. */
  bool        sampled;
} DigitalDesc;

static const DigitalDesc s_digital[] =
{
  { 'B',  2U, "PB2",  BOARD_DIR_OUT,   "AFE_ON",              true,  true   },
  /* Still an input carrying nFAULT, and still readable here - IDR reflects
     the pin whatever mode it is in. It now has a second consumer: the .ioc
     routes it to TIM1_BKIN, so the gate drivers stop in hardware rather than
     waiting for anyone to poll this. The signal is FAULTIN from the STO
     chain, not from the drivers - a 2EDL8034 has no fault pin. */
  /* TIM1_BKIN. Not usable for the same reason as the six gate signals, and
     it was missed when they were fixed: the test path calls HAL_GPIO_Init,
     which takes the pin off the alternate function and disconnects the
     break from the timer - silently, and for good until the next reset.
     Measured after a conformance run: MODER read 00 for PE15 with OTYPER
     and PUPDR still carrying the AF_OD setup, so the power stage had no
     hardware break and nothing said so. The fault level is still reported,
     through Board_IoFault() and the gate driver state, which read the pin
     without reconfiguring it. */
  { 'E', 15U, "PE15", BOARD_DIR_IN,    "nFAULT/TIM1_BKIN",    false, true   },
  { 'E', 14U, "PE14", BOARD_DIR_OUT,   "UART5_TERM",          true,  false  },
  /* The STO chain's proof that main() is still turning. Toggled from the
     poll loop, never by a timer - see Board_StoKeepalive(). */
  { 'A', 10U, "PA10", BOARD_DIR_OUT,   "KEEPALIVE",           true,  true   },
  /* The six gate signals. Not usable, and the reason is the whole point of
     the flag: they are TIM1's alternate function, and a host writing one
     through the test path calls HAL_GPIO_Init on it, which takes the pin
     off the timer and leaves it driven by ODR. With the drivers powered
     that is one FET of a half bridge latched on, with the other still
     switching against it - the dead time cannot help, because the pin is
     no longer the timer's to sequence.

     They were absent from this table entirely, so Board_PinUsable fell
     through to its "nothing claims it, a fixture may have it" default and
     answered true for all six. Measured: the reserved list reported 19
     pins and none of them was a gate. */
  { 'E',  8U, "PE8",  BOARD_DIR_OUT,   "TIM1_CH1N/PWMUL",     false, true   },
  { 'E',  9U, "PE9",  BOARD_DIR_OUT,   "TIM1_CH1/PWMUH",      false, true   },
  { 'E', 10U, "PE10", BOARD_DIR_OUT,   "TIM1_CH2N/PWMVL",     false, true   },
  { 'E', 11U, "PE11", BOARD_DIR_OUT,   "TIM1_CH2/PWMVH",      false, true   },
  { 'E', 12U, "PE12", BOARD_DIR_OUT,   "TIM1_CH3N/PWMWL",     false, true   },
  { 'E', 13U, "PE13", BOARD_DIR_OUT,   "TIM1_CH3/PWMWH",      false, true   },
  { 'B', 10U, "PB10", BOARD_DIR_OUT,   "USART3_TX",           false, false  },
  { 'B', 11U, "PB11", BOARD_DIR_IN,    "USART3_RX",           false, false  },
  { 'A', 13U, "PA13", BOARD_DIR_INOUT, "JTMS/SWDIO",          false, false  },
  { 'A', 14U, "PA14", BOARD_DIR_IN,    "JTCK/SWCLK",          false, false  },
  { 'A', 15U, "PA15", BOARD_DIR_IN,    "JTDI",                false, false  },
  { 'B',  3U, "PB3",  BOARD_DIR_OUT,   "JTDO/TRACESWO",       false, false  },
  { 'B',  4U, "PB4",  BOARD_DIR_IN,    "NJTRST",              false, false  },
  { 'B', 12U, "PB12", BOARD_DIR_OUT,   "SPI2_NSS/H_CSN",      false, false  },
  { 'B', 13U, "PB13", BOARD_DIR_OUT,   "SPI2_SCK",            false, false  },
  { 'B', 14U, "PB14", BOARD_DIR_IN,    "SPI2_MISO",           false, false  },
  { 'B', 15U, "PB15", BOARD_DIR_OUT,   "SPI2_MOSI",           false, false  },
  { 'D',  8U, "PD8",  BOARD_DIR_IN,    "IMU H_INTN",          false, false  },
  { 'D',  9U, "PD9",  BOARD_DIR_OUT,   "IMU PS0/WAKE",        false, false  },
  { 'D', 10U, "PD10", BOARD_DIR_OUT,   "IMU NRSTN",           false, false  },
  { 'D', 11U, "PD11", BOARD_DIR_OUT,   "IMU BOOTN",           false, false  },
  { 'E',  2U, "PE2",  BOARD_DIR_OUT,   "SPI4_SCK",            false, false  },
  { 'E',  4U, "PE4",  BOARD_DIR_OUT,   "SPI4_NSS/A1335_CS",   false, false  },
  { 'E',  5U, "PE5",  BOARD_DIR_IN,    "SPI4_MISO",           false, false  },
  { 'E',  6U, "PE6",  BOARD_DIR_OUT,   "SPI4_MOSI",           false, false  },
};

/* What is fitted, as against what it is wired to. One row per part, and the
   whole stack above reads it off the wire: add a part here and board_info,
   the MCP tools and the local model all report it without being told twice.
   `power` names what must be on for the part to work at all - the BNO08X
   answers reads with AFE_ON low and acts on no write, which is the reason
   this column exists. */
typedef struct
{
  const char *name;
  const char *what;
  const char *where;
  const char *power;
  uint8_t     probe;
} PartDesc;

#define PART_PROBE_NONE 0U
#define PART_PROBE_AFE  1U
#define PART_PROBE_IMU  2U
#define PART_PROBE_ANGLE 3U

static const PartDesc s_parts[] =
{
  { "STM32H753VIT6", "the MCU, 475 MHz", "U3", "", PART_PROBE_NONE },
  { "BNO085", "9-axis IMU, SHTP", "SPI2, U13", "AFE_ON", PART_PROBE_IMU },
  { "A1335", "magnetic angle sensor", "SPI4, U14", "AFE_ON",
    PART_PROBE_ANGLE },
  { "AFE", "phase chains + ADC ref", "PB2 switches it", "", PART_PROBE_AFE },
  { "UART5 termination", "120 ohm across the pair", "PE14 switches it", "",
    PART_PROBE_NONE },
  /* The gate_drivers. `power` names the STO chain and not a pin because there is
     no pin: the supply is released by the safety chain on STO.SchDoc when
     the master's RS485 pilot tone keeps arriving. HalfBridge.SchDoc is
     instantiated three times, one per phase, so the BOM carries Altium's
     $ChannelName rather than a designator per half bridge. */
  { "2EDL8034 x3", "half bridge gate drivers", "PE8..PE13, TIM1",
    "STO chain", PART_PROBE_NONE },
  { "IAUCN10S7N021", "bridge FETs, 63 V 100 A", "HalfBridge x3",
    "STO chain", PART_PROBE_NONE },
  { "NTC", "thermistor", "ADC3", "AFE_ON", PART_PROBE_AFE },
  { "DC link divider", "49.9k/2.2k, 78.15 V FS", "ADC", "AFE_ON",
    PART_PROBE_AFE },
  { "USART3", "console or Modbus RTU", "PB10/PB11", "", PART_PROBE_NONE },
};

uint8_t Board_PartCount(void)
{
  return (uint8_t)(sizeof(s_parts) / sizeof(s_parts[0]));
}

bool Board_Part(uint8_t index, board_part_t *info)
{
  if ((index >= Board_PartCount()) || (info == NULL))
  {
    return false;
  }

  const PartDesc *p = &s_parts[index];

  info->name  = p->name;
  info->what  = p->what;
  info->where = p->where;
  info->power = p->power;

  switch (p->probe)
  {
    case PART_PROBE_AFE:
      info->state = Board_AfeOn() ? BOARD_PART_READY : BOARD_PART_UNPOWERED;
      break;

    case PART_PROBE_IMU:
      if (!Board_AfeOn())
      {
        info->state = BOARD_PART_UNPOWERED;
      }
      else
      {
        info->state = Board_ImuReady() ? BOARD_PART_READY : BOARD_PART_SILENT;
      }
      break;

    case PART_PROBE_ANGLE:
      if (!Board_AfeOn())
      {
        info->state = BOARD_PART_UNPOWERED;
      }
      else
      {
        info->state = Board_AngleReady() ? BOARD_PART_READY
                                         : BOARD_PART_SILENT;
      }
      break;

    default:
      /* Nothing here can prove it either way, and inventing an answer is
         what invariant 10 forbids. */
      info->state = BOARD_PART_UNKNOWN;
      break;
  }

  return true;
}

uint8_t Board_DigitalCount(void)
{
  return (uint8_t)(sizeof(s_digital) / sizeof(s_digital[0]));
}

bool Board_DigitalChan(uint8_t index, board_dchan_t *info)
{
  if ((index >= Board_DigitalCount()) || (info == NULL))
  {
    return false;
  }

  const DigitalDesc *d = &s_digital[index];

  info->pin    = d->pin;
  info->dir    = d->dir;
  info->signal = d->signal;
  info->usable = d->usable;

  return true;
}

static GPIO_TypeDef *port_base(char port)
{
  switch (port)
  {
    case 'A': return GPIOA;
    case 'B': return GPIOB;
    case 'C': return GPIOC;
    case 'D': return GPIOD;
    default:  return GPIOE;
  }
}


uint8_t Board_DigitalIoCount(void)
{
  uint8_t n = 0U;

  for (uint8_t i = 0U; i < Board_DigitalCount(); i++)
  {
    if (s_digital[i].usable)
    {
      n++;
    }
  }
  return n;
}


bool Board_DigitalIoChan(uint8_t slot, board_dchan_t *info)
{
  uint8_t n = 0U;

  for (uint8_t i = 0U; i < Board_DigitalCount(); i++)
  {
    if (s_digital[i].usable && (n++ == slot))
    {
      return Board_DigitalChan(i, info);
    }
  }
  return false;
}


uint8_t Board_DigitalSampledCount(void)
{
  uint8_t n = 0U;

  for (uint8_t i = 0U; i < Board_DigitalCount(); i++)
  {
    if (s_digital[i].sampled)
    {
      n++;
    }
  }
  return n;
}


bool Board_DigitalSampledChan(uint8_t slot, board_dchan_t *info)
{
  uint8_t n = 0U;

  for (uint8_t i = 0U; i < Board_DigitalCount(); i++)
  {
    if (s_digital[i].sampled && (n++ == slot))
    {
      return Board_DigitalChan(i, info);
    }
  }
  return false;
}


uint32_t Board_DigitalMask(void)
{
  uint32_t bits = 0U;
  uint8_t slot = 0U;

  /* The SAMPLED rows, not the writable ones. Reading a pin costs it
     nothing, so the six gate signals and the break belong in a
     measurement even though a host may not drive any of them - during
     switching they flicker, and that IS the reading. What stays out is
     the buses and the debug port: sampling JTAG at the converters' rate
     names a channel nobody asked for, and all twenty-three overflowed
     the layout reply at 312 bytes against MB_MAX_PDU's 253.

     Straight off IDR rather than HAL_GPIO_ReadPin per pin: this runs at the
     acquisition task's rate and the function calls buy nothing. */
  for (uint8_t i = 0U; (i < Board_DigitalCount()) && (slot < 32U); i++)
  {
    const DigitalDesc *d = &s_digital[i];

    if (!d->sampled)
    {
      continue;
    }
    if ((port_base(d->port)->IDR & (1UL << d->number)) != 0U)
    {
      bits |= (1UL << slot);
    }
    slot++;
  }
  return bits;
}


bool Board_PinUsable(char port, uint8_t pin)
{
  for (uint8_t i = 0U; i < Board_DigitalCount(); i++)
  {
    if ((s_digital[i].port == port) && (s_digital[i].number == pin))
    {
      return s_digital[i].usable;
    }
  }

  /* Not in the table at all: nothing on this board claims it, so a fixture
     may have it. */
  return true;
}

bool Board_AfeOn(void)
{
  return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2) == GPIO_PIN_SET);
}

void Board_SetAfeOn(bool on)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

bool Board_Pe15(void)
{
  return (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_15) == GPIO_PIN_SET);
}

void Board_RequestConsoleMode(void)
{
  link_request_close();
}
