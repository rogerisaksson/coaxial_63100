/** boot_main.c - Bootloader hardware: pins, clocks, USARTs, flash, RTU loop, jump. */
#include "boot.h"
#include "cmd.h"
#include "modbus_rtu.h"
#include "modbus_slave.h"
#include "stm32h7xx.h"

#include <string.h>

/* -- what this bootloader is built for ------------------------------------- */

#ifndef BOOT_BOARD
#define BOOT_BOARD  BOOT_TYPE_COAXIAL_63100
#endif

/** The clock tree: HSE 25 MHz / 5 = 5 MHz into PLL1, x64 = 320 MHz VCO, / 2
    = 160 MHz for the core; HCLK 80, APB1 80 - so a USART with 8x
    oversampling divides 80 MHz by 16 for exactly 10 Mbit, no fraction. */
#define HSE_HZ                25000000U
#define PLL_M                 5U
#define PLL_N                 64U
#define PLL_P                 2U
#define CORE_HZ               (HSE_HZ / PLL_M * PLL_N / PLL_P)
#define APB1_HZ               (CORE_HZ / 2U)
#define TICKS_PER_US          (CORE_HZ / 1000000U)
#define TICKS_PER_MS          (CORE_HZ / 1000U)

#define BOOT_BAUD             10000000U
#define CONSOLE_BAUD          115200U     /**< the ST-Link's port: Modbus, as the application's */
#define PORTS                 3U          /**< two RS485 segments and the ST-Link's port */

/** The bootloader's own windows. */
#define HOLD_MS               300U    /**< a valid application waits this long for a hold */
#define GO_SETTLE_MS          20U     /**< after go: the last reply out, then the jump */

/** The flash controller: the bank keys, the sector size, the bank split. */
#define FLASH_KEY1            0x45670123U
#define FLASH_KEY2            0xCDEF89ABU
#define FLASH_BANK_BYTES      0x100000U
#define FLASH_SECTOR_BYTES    BOOT_SECTOR_BYTES
#define FLASH_SR_ERRORS       (FLASH_SR_WRPERR | FLASH_SR_PGSERR | FLASH_SR_STRBERR \
                               | FLASH_SR_INCERR | FLASH_SR_OPERR | FLASH_SR_RDPERR \
                               | FLASH_SR_RDSERR | FLASH_SR_SNECCERR | FLASH_SR_DBECCERR)
#define FLASH_WORD_U32        (BOOT_WORD_BYTES / 4U)

/** The MCU's unique id, three words. */
#define UID_WORDS             3U

/* -- the pins -------------------------------------------------------------- */

typedef enum
{
  PIN_OUT_LOW,       /**< an output driven low, and left there */
  PIN_AF,            /**< an alternate function, push-pull, fast */
} pin_mode_t;

typedef struct
{
  GPIO_TypeDef *port;
  uint8_t       pin;
  pin_mode_t    mode;
  uint8_t       af;
} pin_t;

/** The table for one board type: every pin the bootloader drives, in the
    order they are driven - the safe levels first, the USARTs after. */
typedef struct
{
  const pin_t  *pins;
  uint32_t      count;
  USART_TypeDef *rs485[2];        /**< the two segments */
  USART_TypeDef *console;
  GPIO_TypeDef *term_port;        /**< the 120 ohm termination */
  uint8_t       term_pin;
} board_t;

/** coaxial_63100: the six gate inputs low (both FETs of every leg off by a
    driven line - TIM1's idle state is RESET on all six), PA10 low so the STO
    pump is not fed, PB2 low so the AFE and the IMU stay unpowered, PE14 low
    so the termination is open; then USART2 on PA1..3 (DE, TX, RX), UART5 on
    PC8/PC12/PD2 (DE, TX, RX), the console on PB10/PB11. */
static const pin_t PINS_63100[] =
{
  { GPIOE,  8U, PIN_OUT_LOW, 0U },
  { GPIOE,  9U, PIN_OUT_LOW, 0U },
  { GPIOE, 10U, PIN_OUT_LOW, 0U },
  { GPIOE, 11U, PIN_OUT_LOW, 0U },
  { GPIOE, 12U, PIN_OUT_LOW, 0U },
  { GPIOE, 13U, PIN_OUT_LOW, 0U },
  { GPIOA, 10U, PIN_OUT_LOW, 0U },
  { GPIOB,  2U, PIN_OUT_LOW, 0U },
  { GPIOE, 14U, PIN_OUT_LOW, 0U },
  { GPIOA,  1U, PIN_AF,      7U },
  { GPIOA,  2U, PIN_AF,      7U },
  { GPIOA,  3U, PIN_AF,      7U },
  { GPIOC,  8U, PIN_AF,      8U },
  { GPIOC, 12U, PIN_AF,      8U },
  { GPIOD,  2U, PIN_AF,      8U },
  { GPIOB, 10U, PIN_AF,      7U },
  { GPIOB, 11U, PIN_AF,      7U },
};

static const board_t BOARD_63100 =
{
  PINS_63100, sizeof(PINS_63100) / sizeof(PINS_63100[0]),
  { USART2, UART5 }, USART3, GPIOE, 14U,
};

/** The type switch. */
static const board_t *board_of(uint32_t type)
{
  switch (type)
  {
    case BOOT_TYPE_COAXIAL_63100: return &BOARD_63100;
    default:                      return NULL;
  }
}

_Static_assert(BOOT_BOARD == BOOT_TYPE_COAXIAL_63100,
               "boot_main.c has a pin table for coaxial_63100 only");

/* -- the state ------------------------------------------------------------- */

/** The handover slot, shared with the application (boot.h). */
__attribute__((section(".boot_hand")))
boot_hand_t boot_hand;

static struct
{
  const board_t   *board;
  mb_slave_t       slave;
  mb_data_model_t  model;
  mb_rtu_t         rtu[PORTS];
  USART_TypeDef   *usart[PORTS];
  boot_layout_t    layout;
  uint32_t         go_at;
  bool             going;
} s;

/* -- time ------------------------------------------------------------------ */

static uint32_t ticks(void)
{
  return DWT->CYCCNT;
}

static bool elapsed_ms(uint32_t since, uint32_t ms)
{
  return (ticks() - since) >= (ms * TICKS_PER_MS);
}

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->LAR = 0xC5ACCE55U;                  /* the M7's lock on the DWT */
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* -- clocks ---------------------------------------------------------------- */

static void clocks_up(void)
{
  /* The LDO, as reset left it, settled; then VOS1 - one scale up from VOS3,
     so APB may run at 80 MHz. */
  while ((PWR->CSR1 & PWR_CSR1_ACTVOSRDY) == 0U) {}
  PWR->D3CR = (PWR->D3CR & ~PWR_D3CR_VOS_Msk) | PWR_D3CR_VOS_1 | PWR_D3CR_VOS_0;
  while ((PWR->D3CR & PWR_D3CR_VOSRDY) == 0U) {}

  RCC->CR |= RCC_CR_HSEON;
  while ((RCC->CR & RCC_CR_HSERDY) == 0U) {}

  RCC->PLLCKSELR = RCC_PLLCKSELR_PLLSRC_HSE | (PLL_M << RCC_PLLCKSELR_DIVM1_Pos);
  RCC->PLLCFGR = RCC_PLLCFGR_PLL1RGE_2 | RCC_PLLCFGR_DIVP1EN;   /* 4..8 MHz in, wide VCO */
  RCC->PLL1DIVR = ((PLL_N - 1U) << RCC_PLL1DIVR_N1_Pos)
                  | ((PLL_P - 1U) << RCC_PLL1DIVR_P1_Pos)
                  | (1U << RCC_PLL1DIVR_Q1_Pos) | (1U << RCC_PLL1DIVR_R1_Pos);
  RCC->CR |= RCC_CR_PLL1ON;
  while ((RCC->CR & RCC_CR_PLL1RDY) == 0U) {}

  /* Core /1, HCLK /2, APB3 /1, APB1 /1, APB2 /1, APB4 /1: 160, 80, 80. */
  RCC->D1CFGR = RCC_D1CFGR_HPRE_DIV2;
  RCC->D2CFGR = 0U;
  RCC->D3CFGR = 0U;

  /* One wait state at 80 MHz on VOS1, and the programming delay for it. */
  FLASH->ACR = FLASH_ACR_LATENCY_1WS | FLASH_ACR_WRHIGHFREQ_0;

  RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL1;
  while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL1) {}

  RCC->AHB4ENR |= RCC_AHB4ENR_GPIOAEN | RCC_AHB4ENR_GPIOBEN | RCC_AHB4ENR_GPIOCEN
                  | RCC_AHB4ENR_GPIODEN | RCC_AHB4ENR_GPIOEEN;
  /* D2 SRAM, where the application runs - left on through the jump. */
  RCC->AHB2ENR |= RCC_AHB2ENR_SRAM1EN | RCC_AHB2ENR_SRAM2EN | RCC_AHB2ENR_SRAM3EN;
  (void)RCC->AHB2ENR;
  RCC->APB1LENR |= RCC_APB1LENR_USART2EN | RCC_APB1LENR_USART3EN | RCC_APB1LENR_UART5EN;
  (void)RCC->APB1LENR;
}

/** The tree back to reset before the jump: the application's HAL refuses to
    configure a PLL that is the system clock, and expects HSI. */
static void clocks_down(void)
{
  RCC->APB1LRSTR = RCC_APB1LRSTR_USART2RST | RCC_APB1LRSTR_USART3RST | RCC_APB1LRSTR_UART5RST;
  RCC->APB1LRSTR = 0U;
  RCC->APB1LENR &= ~(RCC_APB1LENR_USART2EN | RCC_APB1LENR_USART3EN | RCC_APB1LENR_UART5EN);

  RCC->CFGR &= ~RCC_CFGR_SW_Msk;                          /* HSI */
  while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_HSI) {}
  RCC->CR &= ~RCC_CR_PLL1ON;
  while ((RCC->CR & RCC_CR_PLL1RDY) != 0U) {}
  RCC->CR &= ~RCC_CR_HSEON;
  RCC->PLLCKSELR = 0x02020200U;                           /* the reset values */
  RCC->PLLCFGR = 0x01FF0000U;
  RCC->PLL1DIVR = 0x01010280U;
  RCC->D1CFGR = 0U;
  RCC->D2CFGR = 0U;
  RCC->D3CFGR = 0U;
  FLASH->ACR = FLASH_ACR_LATENCY_7WS | FLASH_ACR_WRHIGHFREQ;    /* the reset value */
  PWR->D3CR = (PWR->D3CR & ~PWR_D3CR_VOS_Msk) | PWR_D3CR_VOS_0;  /* VOS3 */
  while ((PWR->D3CR & PWR_D3CR_VOSRDY) == 0U) {}
}

/* -- pins ------------------------------------------------------------------ */

static void pin_set(GPIO_TypeDef *port, uint8_t pin, bool high)
{
  port->BSRR = high ? (1UL << pin) : (1UL << (pin + 16U));
}

static void pins_up(const board_t *b)
{
  for (uint32_t i = 0U; i < b->count; i++)
  {
    const pin_t *p = &b->pins[i];
    const uint32_t two = 2U * p->pin;

    if (p->mode == PIN_OUT_LOW)
    {
      pin_set(p->port, p->pin, false);       /* the level before the mode */
    }
    p->port->OTYPER &= ~(1UL << p->pin);
    p->port->PUPDR &= ~(3UL << two);
    p->port->OSPEEDR = (p->port->OSPEEDR & ~(3UL << two)) | (2UL << two);
    if (p->mode == PIN_AF)
    {
      const uint32_t four = 4U * (p->pin % 8U);

      p->port->AFR[p->pin / 8U] = (p->port->AFR[p->pin / 8U] & ~(0xFUL << four))
                                   | ((uint32_t)p->af << four);
    }
    p->port->MODER = (p->port->MODER & ~(3UL << two))
                     | ((p->mode == PIN_AF) ? (2UL << two) : (1UL << two));
  }
}

/* -- the USARTs ------------------------------------------------------------ */

/** 8N1, FIFOs on. */
static void usart_up(USART_TypeDef *u, uint32_t baud, bool rs485)
{
  u->CR1 = 0U;
  u->CR2 = 0U;
  u->CR3 = rs485 ? USART_CR3_DEM : 0U;
  if (rs485)
  {
    const uint32_t div = 2U * APB1_HZ / baud;

    u->BRR = (div & 0xFFF0U) | ((div & 0xFU) >> 1);
    u->CR1 = USART_CR1_FIFOEN | USART_CR1_OVER8 | USART_CR1_TE | USART_CR1_RE;
  }
  else
  {
    u->BRR = (APB1_HZ + baud / 2U) / baud;
    u->CR1 = USART_CR1_FIFOEN | USART_CR1_TE | USART_CR1_RE;
  }
  u->ICR = 0xFFFFFFFFU;
  u->CR1 |= USART_CR1_UE;
}

static void usart_send(USART_TypeDef *u, const uint8_t *data, size_t n)
{
  for (size_t i = 0U; i < n; i++)
  {
    while ((u->ISR & USART_ISR_TXE_TXFNF) == 0U) {}
    u->TDR = data[i];
  }
  while ((u->ISR & USART_ISR_TC) == 0U) {}
}

/* -- the console ----------------------------------------------------------- */

/** The ST-Link's port carries Modbus, so a line of text would land inside a
    host's frame: the core's words go nowhere here; `state` says it all. */
static void say(void *ctx, const char *line)
{
  (void)ctx;
  (void)line;
}

/* -- the flash controller -------------------------------------------------- */

typedef struct
{
  volatile uint32_t *keyr;
  volatile uint32_t *cr;
  volatile uint32_t *sr;
  volatile uint32_t *ccr;
  uint32_t           base;
} bank_t;

static bank_t bank_of(uint32_t address)
{
  bank_t b;

  if (address < (FLASH_BANK1_BASE + FLASH_BANK_BYTES))
  {
    b.keyr = &FLASH->KEYR1;
    b.cr = &FLASH->CR1;
    b.sr = &FLASH->SR1;
    b.ccr = &FLASH->CCR1;
    b.base = FLASH_BANK1_BASE;
  }
  else
  {
    b.keyr = &FLASH->KEYR2;
    b.cr = &FLASH->CR2;
    b.sr = &FLASH->SR2;
    b.ccr = &FLASH->CCR2;
    b.base = FLASH_BANK2_BASE;
  }
  return b;
}

static void bank_unlock(const bank_t *b)
{
  if ((*b->cr & FLASH_CR_LOCK) != 0U)
  {
    *b->keyr = FLASH_KEY1;
    *b->keyr = FLASH_KEY2;
  }
}

/** Waits for the bank, then answers whether the operation was clean; every
    error flag is cleared on the way, so the next one starts fresh. */
static bool bank_done(const bank_t *b)
{
  while ((*b->sr & (FLASH_SR_QW | FLASH_SR_BSY)) != 0U) {}
  const bool clean = ((*b->sr & FLASH_SR_ERRORS) == 0U);

  *b->ccr = FLASH_SR_ERRORS | FLASH_SR_EOP;
  return clean;
}

static bool sector_erase(uint32_t address)
{
  const bank_t b = bank_of(address);
  const uint32_t sector = (address - b.base) / FLASH_SECTOR_BYTES;

  bank_unlock(&b);
  if (!bank_done(&b))
  {
    return false;
  }
  *b.cr = FLASH_CR_SER | FLASH_CR_PSIZE_1 | (sector << FLASH_CR_SNB_Pos);
  *b.cr |= FLASH_CR_START;
  const bool ok = bank_done(&b);

  *b.cr = FLASH_CR_LOCK;
  return ok;
}

/** boot_port_t.erase: every sector the range touches. */
static bool flash_erase(void *ctx, uint32_t address, uint32_t bytes)
{
  (void)ctx;
  for (uint32_t at = address & ~(FLASH_SECTOR_BYTES - 1U); at < address + bytes;
       at += FLASH_SECTOR_BYTES)
  {
    if (!sector_erase(at))
    {
      return false;
    }
  }
  return true;
}

/** boot_port_t.program: one 256-bit word, at a word-aligned address that
    reads erased - the controller programs a word once, and the core never
    asks twice, but the harness refuses the second write and so does this. */
static bool flash_program(void *ctx, uint32_t address, const uint8_t *word)
{
  const bank_t b = bank_of(address);
  volatile uint32_t *dst = (volatile uint32_t *)address;
  uint32_t src[FLASH_WORD_U32];

  (void)ctx;
  if ((address % BOOT_WORD_BYTES) != 0U)
  {
    return false;
  }
  for (uint32_t i = 0U; i < FLASH_WORD_U32; i++)
  {
    if (dst[i] != 0xFFFFFFFFU)
    {
      return false;
    }
  }
  memcpy(src, word, sizeof(src));
  bank_unlock(&b);
  if (!bank_done(&b))
  {
    return false;
  }
  *b.cr = FLASH_CR_PG | FLASH_CR_PSIZE_1;
  for (uint32_t i = 0U; i < FLASH_WORD_U32; i++)
  {
    dst[i] = src[i];
  }
  __DSB();
  const bool ok = bank_done(&b);

  *b.cr = FLASH_CR_LOCK;
  return ok;
}

static const uint8_t *flash_read(void *ctx, uint32_t address)
{
  (void)ctx;
  return (const uint8_t *)address;
}

/* -- RAM, where the image runs --------------------------------------------- */

static bool in_run(uint32_t address)
{
  return (address >= BOOT_RUN_BASE) && (address < BOOT_RUN_BASE + BOOT_RUN_BYTES);
}

/** boot_port_t.erase: RAM to 0xFF, whole words; flash by the sector. */
static bool mem_erase(void *ctx, uint32_t address, uint32_t bytes)
{
  if (in_run(address))
  {
    memset((void *)address, 0xFF, (bytes + BOOT_WORD_BYTES - 1U) & ~(BOOT_WORD_BYTES - 1U));
    return true;
  }
  return flash_erase(ctx, address, bytes);
}

static bool mem_program(void *ctx, uint32_t address, const uint8_t *word)
{
  if (in_run(address))
  {
    memcpy((void *)address, word, BOOT_WORD_BYTES);
    return true;
  }
  return flash_program(ctx, address, word);
}

static const boot_port_t PORT = { mem_erase, mem_program, flash_read, say };

/* -- the RTU seam ---------------------------------------------------------- */

/** 0x6E device 11, and nothing else: the shell serves one device. */
static mb_exception_t user_function(void *ctx, uint8_t fc, const uint8_t *req,
                                    size_t req_len, uint8_t *rsp, size_t rsp_cap,
                                    size_t *rsp_len)
{
  (void)ctx;
  const int n = (fc == CMD_DEVICE) ? boot_pdu(req, req_len, rsp, rsp_cap) : BOOT_PDU_FOREIGN;

  if (n == BOOT_PDU_SILENT)
  {
    return MB_NO_REPLY;
  }
  if (n == BOOT_PDU_FOREIGN)
  {
    return MB_EX_ILLEGAL_FUNCTION;
  }
  if (n < 0)
  {
    return MB_EX_SERVER_DEVICE_FAILURE;
  }
  *rsp_len = (size_t)n;
  return MB_EX_NONE;
}

static void rtu_up(void)
{
  memset(&s.model, 0, sizeof(s.model));
  s.model.user_function = user_function;
  mb_slave_init(&s.slave, &s.model);
  s.usart[0] = s.board->rs485[0];
  s.usart[1] = s.board->rs485[1];
  s.usart[2] = s.board->console;
  for (uint32_t i = 0U; i < PORTS; i++)
  {
    mb_rtu_init(&s.rtu[i], &s.slave, BOOT_UNIT, (i < 2U) ? BOOT_BAUD : CONSOLE_BAUD, 10U,
                TICKS_PER_US);
  }
}

/** One pass over one port: the bytes in, the errors, the reply out. */
static void rtu_poll(uint32_t i)
{
  USART_TypeDef *u = s.usart[i];
  mb_rtu_t *rtu = &s.rtu[i];
  const uint8_t *reply;

  while ((u->ISR & USART_ISR_RXNE_RXFNE) != 0U)
  {
    const uint32_t now = ticks();
    const uint32_t errors = u->ISR & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE);
    const uint8_t byte = (uint8_t)u->RDR;

    if (errors != 0U)
    {
      u->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF;
      mb_rtu_on_error(rtu, now);
    }
    else
    {
      mb_rtu_on_byte(rtu, byte, now);
    }
  }
  if ((u->ISR & USART_ISR_ORE) != 0U)
  {
    u->ICR = USART_ICR_ORECF;              /* an overrun with nothing behind it */
    mb_rtu_on_error(rtu, ticks());
  }
  const size_t n = mb_rtu_service(rtu, ticks(), &reply);

  if (n != 0U)
  {
    usart_send(u, reply, n);
  }
}

/** What the core decided this pass: the unit both segments answer to, the
    termination, and whether a go was taken. */
static void follow_core(void)
{
  const uint8_t unit = boot_unit();

  for (uint32_t i = 0U; i < PORTS; i++)
  {
    s.rtu[i].unit_id = unit;
  }
  pin_set(s.board->term_port, s.board->term_pin, boot_terminates());
  if (boot_wants_go() && !s.going)
  {
    s.going = true;
    s.go_at = ticks();
  }
}

/* -- the jump -------------------------------------------------------------- */

/** The pins stay as they are - the safe levels driven, the USART pins at
    their alternate function with the peripherals in reset, so nothing floats
    in the microseconds before the application's init takes them. */
static void jump(void)
{
  const uint32_t *vectors = (const uint32_t *)BOOT_RUN_BASE;

  boot_hand.magic = BOOT_HAND_MAGIC;
  boot_hand.stay = 0U;
  boot_hand.unit = boot_unit();
  boot_hand.position = boot_position();
  boot_hand.flags = boot_flags();
  boot_image(&boot_hand.bytes, &boot_hand.crc);
  clocks_down();
  __DSB();
  __ISB();
  __asm volatile ("msr msp, %0\n\tbx %1" : : "r" (vectors[0]), "r" (vectors[1]) : "memory");
  for (;;) {}
}

/* -- main ------------------------------------------------------------------ */

static void layout_up(void)
{
  const uint32_t *uid = (const uint32_t *)UID_BASE;

  s.layout.app_base = BOOT_RUN_BASE;
  s.layout.app_bytes = BOOT_RUN_BYTES;
  s.layout.store_base = BOOT_STORE_BASE;
  s.layout.store_bytes = BOOT_STORE_BYTES;
  s.layout.record_base = BOOT_RECORD_BASE;
  s.layout.record_bytes = BOOT_RECORD_BYTES;
  s.layout.type = BOOT_BOARD;
  for (uint32_t i = 0U; i < UID_WORDS; i++)
  {
    s.layout.uid[4U * i]      = (uint8_t)(uid[i] & 0xFFU);
    s.layout.uid[4U * i + 1U] = (uint8_t)((uid[i] >> 8) & 0xFFU);
    s.layout.uid[4U * i + 2U] = (uint8_t)((uid[i] >> 16) & 0xFFU);
    s.layout.uid[4U * i + 3U] = (uint8_t)((uid[i] >> 24) & 0xFFU);
  }
}

int main(void)
{
  s.board = board_of(BOOT_BOARD);
  dwt_init();
  clocks_up();
  pins_up(s.board);
  usart_up(s.board->rs485[0], BOOT_BAUD, true);
  usart_up(s.board->rs485[1], BOOT_BAUD, true);
  usart_up(s.board->console, CONSOLE_BAUD, false);
  layout_up();
  boot_init(&PORT, NULL, &s.layout);
  rtu_up();

  /* THE GATE (docs/BOOT.md): RAM still whole with the image the slot names,
     or the store's sealed copy verified into RAM - then a window for a hold;
     asked to stay, or nothing to run, and the node waits for the master. */
  const bool asked_to_stay = (boot_hand.stay == BOOT_STAY_MAGIC);
  const bool warm = (boot_hand.magic == BOOT_HAND_MAGIC);
  const bool ready = boot_ready(warm ? boot_hand.bytes : 0U, warm ? boot_hand.crc : 0U);
  const uint32_t started = ticks();

  boot_hand.stay = 0U;
  for (;;)
  {
    for (uint32_t i = 0U; i < PORTS; i++)
    {
      rtu_poll(i);
    }
    follow_core();

    if (s.going && elapsed_ms(s.go_at, GO_SETTLE_MS))
    {
      jump();
    }
    if (ready && !asked_to_stay && (boot_state() == BOOT_BLANK)
        && elapsed_ms(started, HOLD_MS))
    {
      jump();
    }
  }
}
