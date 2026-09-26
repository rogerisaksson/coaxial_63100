/** native.c - The board's silicon on this host: the clock, TIM1, ADC1-3, the front end. */

/* Over the world core's plant, a board a node of it (native_world). board/src's files run
   on it as built for the part, main()'s loop as main.c's USER CODE has it. TIM1 counts
   centre-aligned on SYSCLK's cycles, its compares preloaded and landed at the updates its
   repetition counter lets through, its update interrupt run there; OC5REF's edge on the
   down-count converts the injected groups and runs ADC3's interrupt; a meter read converts
   its channel. The plant steps between edges on the compares in force. The front end is
   board/emu/Coaxial63100_AFE.cs's - the schematic's networks, LTspice's phase transfer, the
   board's spread from its seed, the converter's noise - and the heat
   Coaxial63100_Plant.cs's; the A1335 reads the plant's shaft, the BNO085 ticks each
   millisecond (native_a1335.c, native_bno085.c, their pins and buses native_io.c's). The
   rest of the board API is the fake board's, weak there. A read of the cycle counter or the
   tick moves the clock, so a spin ends, and runs the interrupts due unless PRIMASK holds
   them. Driven by tools/cores/native.py. */
#include "board.h"
#include "board_drive.h"
#include "board_hw.h"
#include "board_irq.h"
#include "board_power.h"
#include "link.h"
#include "modbus_map.h"
#include "native.h"
#include "world_heat.h"

#include <math.h>
#include <string.h>

/* The world core's flat bridge (world/src/world_emu.c), in a library of its own that a
   limb's boards share. */
typedef void (*native_step_t)(int node, float d0, float d1, float d2, int driven, float ts,
                              float *out);
typedef void (*native_shaft_t)(int node, float *out);

/* TIM1's update interrupt (board/src/board_pwm.c). */
void TIM1_UP_IRQHandler(void);

/* The fake board's stack (board/fake/fake_uart.c). */
void fake_open(void);
void fake_clock_step(uint32_t us);

/* CubeMX's MX_TIM1_Init: ARR 2375, RCR 1, DTG 19, the break on and active low. */
#define NATIVE_ARR         2375U
#define NATIVE_RCR         1U
#define NATIVE_DTG         19U
/* TIM1's kernel is SYSCLK / 2: a tick is two cycles. */
#define NATIVE_TICK_CYCLES 2U
/* A read of the cycle counter, cycles: DWT's load and the loop around it. */
#define NATIVE_READ_CYCLES 8U
/* main()'s loop: a pass at least this often, us. */
#define NATIVE_LOOP_US     20U
/* The A1335's invented turn with no world: one every two virtual seconds, as Renode's. */
#define NATIVE_TURN_S      2.0
#define NATIVE_RAD_TO_DEG  57.29577951308232
#define NATIVE_PI          3.141592653589793
#define NATIVE_ADCS        3U
#define NATIVE_RANKS       2U
#define NATIVE_FULL_CODE   65535.0

/* The heat, Coaxial63100_Plant.cs's: ten steps a second of thermal.c's network as the truth
   (world_heat.c), in a room at 25 C. */
#define HEAT_HZ            10U
#define HEAT_AMBIENT_C     25.0f

TIM_TypeDef  native_tim1;
RCC_TypeDef  native_rcc;
ADC_TypeDef  native_adc3;
static ADC_TypeDef native_adc1;
static ADC_TypeDef native_adc2;

ADC_HandleTypeDef hadc1 = { &native_adc1, { 0U }, 0U };
ADC_HandleTypeDef hadc2 = { &native_adc2, { 0U }, 0U };
ADC_HandleTypeDef hadc3 = { &native_adc3, { 0U }, 0U };

uint16_t native_ts_cal[2];
uint32_t native_primask;

typedef enum
{
  EDGE_UNDERFLOW,
  EDGE_OVERFLOW,
  EDGE_TRIGGER
} native_edge_t;

static struct
{
  uint64_t cycles;                /* SYSCLK's, since native_open */
  uint64_t told_us;               /* how far the fake's microsecond clock has been told */
  bool     in_isr;

  uint64_t irqs;                  /* the NVIC's enables, a bit an IRQn */
  uint64_t tick_at;               /* the next millisecond: SysTick's, the BNO085's */
  uint64_t end;                   /* where the host's step ends: WFI sleeps no further */
  uint32_t sr;                    /* TIM1's SR, its flags rc_w0 */
  uint32_t rep;                   /* the repetition counter */
  uint32_t active[BOARD_PWM_PHASES];  /* the compares' shadows */
  native_edge_t edge;             /* the next edge, and when */
  uint64_t edge_at;
  uint64_t over_at;               /* the last overflow */
  uint64_t plant_at;              /* where the plant stands */

  struct
  {
    uint32_t channel;             /* the regular group's rank 1 */
    uint32_t rank[NATIVE_RANKS];  /* the injected sequence */
    uint32_t ranks;
    bool     injected;
    bool     it;
  } adc[NATIVE_ADCS];

  double   squares[BOARD_PWM_PHASES]; /* A^2 s a leg over the heat's step */
  uint64_t heat_at;
  world_heat_t heat;
  thermal_sense_t seen;
} n;

/* This board's node in its world, and its shaft unwrapped: the electrical angle over the
   pole pairs, as Coaxial63100_Plant.cs turns the A1335. */
static struct
{
  native_step_t  step;
  native_shaft_t shaft;
  int      node;
  double   pole_pairs;
  double   electrical;
  double   mechanical;
  double   speed;          /* rad/s mechanical */
  bool     set;
  double   degrees;
} w;

/* The front end: board/emu/Coaxial63100_AFE.cs's networks, inputs and errors. */
static struct
{
  double ref;
  double bus_top, bus_bottom;
  double ntc_r25, ntc_beta, ntc_fixed;
  double r5_top, r5_bottom;
  double gate_top, gate_bottom;
  double v_per_a, zero_v;         /* LTspice's (coaxial_63100_afe.repl) */
  double gain_sigma, zero_sigma, bus_sigma;
  double die_at30, die_per_k;
  double noise;

  double amps[BOARD_PWM_PHASES], dc, ntc_c, die_c, rail5, gate;
  double gain_err[BOARD_PWM_PHASES], zero_err[BOARD_PWM_PHASES], bus_err;
  uint64_t rng;
} afe =
{
  .ref = 3.3,
  .bus_top = 49900.0, .bus_bottom = 2200.0,
  .ntc_r25 = 10000.0, .ntc_beta = 3380.0, .ntc_fixed = 10000.0,
  .r5_top = 10000.0, .r5_bottom = 10000.0,
  .gate_top = 57000.0, .gate_bottom = 10000.0,
  .die_at30 = 0.62, .die_per_k = 0.002,
  .noise = 0.0001,
  .rng = 0x63100001U,
};

/* ---- the front end ------------------------------------------------------------------ */

typedef enum
{
  PIN_NONE, PIN_U, PIN_V, PIN_W, PIN_DCBUS, PIN_NTC, PIN_RAIL5, PIN_VGATE, PIN_DIE
} native_pin_t;

/* The wiring, ADC1 = 0: board_adc.c's table as the schematic has it. */
static const struct
{
  uint8_t adc;
  uint32_t channel;
  native_pin_t pin;
} WIRED[] =
{
  { 2U, ADC_CHANNEL_1,  PIN_U },     { 0U, ADC_CHANNEL_3,  PIN_V },
  { 1U, ADC_CHANNEL_4,  PIN_W },     { 2U, ADC_CHANNEL_10, PIN_DCBUS },
  { 0U, ADC_CHANNEL_9,  PIN_NTC },   { 0U, ADC_CHANNEL_18, PIN_RAIL5 },
  { 0U, ADC_CHANNEL_19, PIN_VGATE }, { 2U, ADC_CHANNEL_TEMPSENSOR, PIN_DIE },
};

/* xorshift64*, uniform on (0, 1]. */
static double afe_uniform(void)
{
  afe.rng ^= afe.rng >> 12;
  afe.rng ^= afe.rng << 25;
  afe.rng ^= afe.rng >> 27;
  return ((double)((afe.rng * 0x2545F4914F6CDD1DULL) >> 11) + 1.0) / 9007199254740992.0;
}

static double afe_gauss(void)
{
  const double u1 = afe_uniform();
  const double u2 = afe_uniform();

  return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static double afe_mid(void)
{
  return afe.ref * 32768.0 / NATIVE_FULL_CODE;
}

/* One leg at the ADC, as the volts of its code: mid-scale plus half the difference. */
static double afe_phase(uint8_t leg)
{
  const double gain = afe.v_per_a * (1.0 + afe.gain_err[leg]);
  const double zero = afe.zero_v + afe.zero_err[leg];

  return afe_mid() + (zero + gain * afe.amps[leg]) / 2.0;
}

static double afe_die(double celsius)
{
  return afe.die_at30 + (celsius - 30.0) * afe.die_per_k;
}

static double afe_nominal(native_pin_t pin)
{
  switch (pin)
  {
    case PIN_U:     return afe_phase(0U);
    case PIN_V:     return afe_phase(1U);
    case PIN_W:     return afe_phase(2U);
    case PIN_DCBUS: return afe.dc * (1.0 + afe.bus_err) * afe.bus_bottom
                           / (afe.bus_top + afe.bus_bottom);
    case PIN_NTC:
    {
      const double ohms = afe.ntc_r25 * exp(afe.ntc_beta * (1.0 / (afe.ntc_c + 273.15)
                                                            - 1.0 / (25.0 + 273.15)));
      return afe.ref * afe.ntc_fixed / (ohms + afe.ntc_fixed);
    }
    case PIN_RAIL5: return afe.rail5 * afe.r5_bottom / (afe.r5_top + afe.r5_bottom);
    case PIN_VGATE: return afe.gate * afe.gate_bottom / (afe.gate_top + afe.gate_bottom);
    case PIN_DIE:   return afe_die(afe.die_c);
    default:        return 0.0;   /* Clevel, Cinj: not modelled */
  }
}

/* The code `adc` converts on `channel`: AFE_ON off, the reference unpowered, mid-scale. */
static uint32_t afe_code(uint8_t adc, uint32_t channel)
{
  native_pin_t pin = PIN_NONE;
  double volts = afe_mid();

  for (uint32_t k = 0U; k < (sizeof WIRED / sizeof WIRED[0]); k++)
  {
    if ((WIRED[k].adc == adc) && (WIRED[k].channel == channel))
    {
      pin = WIRED[k].pin;
    }
  }
  if (Board_AfeOn())
  {
    volts = afe_nominal(pin) + afe_gauss() * afe.noise;
    volts = (volts < 0.0) ? 0.0 : (volts > afe.ref) ? afe.ref : volts;
  }
  const double code = floor(volts / afe.ref * 65536.0 + 0.5);

  return (code > NATIVE_FULL_CODE) ? (uint32_t)NATIVE_FULL_CODE : (uint32_t)code;
}

/* ---- the clock and the plant -------------------------------------------------------- */

static uint32_t native_cycles_per_us(void)
{
  return SystemCoreClock / 1000000U;
}

static void native_clock_to(uint64_t at)
{
  if (at <= n.cycles)
  {
    return;
  }
  n.cycles = at;
  const uint64_t us = n.cycles / native_cycles_per_us();

  if (us > n.told_us)
  {
    fake_clock_step((uint32_t)(us - n.told_us));
    n.told_us = us;
  }
}

/* The heat's step on the duties, MOE, the legs' mean squares, the link and the shaft: the
   NTC's element and the two dies read off the network. */
static void native_heat(void)
{
  const float dt = 1.0f / (float)HEAT_HZ;
  const float arr = (float)((TIM1->ARR != 0U) ? TIM1->ARR : 1U);
  thermal_load_t load;

  memset(&load, 0, sizeof load);
  load.afe_on = native_powered();
  load.switching = (TIM1->BDTR & TIM_BDTR_MOE) != 0U;
  for (uint8_t leg = 0U; leg < BOARD_PWM_PHASES; leg++)
  {
    load.duty[leg] = (float)n.active[leg] / arr;
    load.phase_sq[leg] = (float)(n.squares[leg] / (double)dt);
    n.squares[leg] = 0.0;
  }
  load.link_volts = (float)afe.dc;
  load.link_amps = -1.0f;
  load.speed_rpm = (float)(fabs(w.speed) * 60.0 / (2.0 * NATIVE_PI));
  world_heat_step(&n.heat, &load, dt, &n.seen);
  afe.ntc_c = n.seen.ntc_c;
  afe.die_c = n.seen.mcu_c;
}

/* The plant on to `at` on the compares in force, the front end's inputs after it. */
static void native_plant_to(uint64_t at)
{
  if (at <= n.plant_at)
  {
    return;
  }
  const float ts = (float)(at - n.plant_at) / (float)SystemCoreClock;
  const float arr = (float)((TIM1->ARR != 0U) ? TIM1->ARR : 1U);
  float got[4] = { 0.0f, 0.0f, 0.0f, 0.0f };

  if (w.step != NULL)
  {
    float shaft[3];

    w.step(w.node, (float)n.active[0] / arr, (float)n.active[1] / arr,
           (float)n.active[2] / arr, ((TIM1->BDTR & TIM_BDTR_MOE) != 0U) ? 1 : 0, ts, got);
    w.shaft(w.node, shaft);
    double turned = (double)shaft[0] - w.electrical;

    turned -= 2.0 * NATIVE_PI * floor(turned / (2.0 * NATIVE_PI) + 0.5);
    w.electrical = (double)shaft[0];
    w.mechanical += turned / w.pole_pairs;
    w.speed = (double)shaft[1];
  }
  for (uint8_t leg = 0U; leg < BOARD_PWM_PHASES; leg++)
  {
    afe.amps[leg] = got[leg];
    n.squares[leg] += (double)got[leg] * (double)got[leg] * (double)ts;
  }
  afe.dc = got[3];
  n.plant_at = at;
  while ((n.plant_at - n.heat_at) >= (SystemCoreClock / HEAT_HZ))
  {
    n.heat_at += SystemCoreClock / HEAT_HZ;
    native_heat();
  }
}

/* ---- TIM1 and the injected groups --------------------------------------------------- */

/* SR's flags are rc_w0: a write of 0 clears one, of 1 leaves it - `SR = ~UIF` clears UIF. */
static void native_reconcile(void)
{
  n.sr &= TIM1->SR;
  TIM1->SR = n.sr;
}

/* An overflow or an underflow: DIR turns, and the update if the repetition counter is out. */
static void native_update(bool overflow)
{
  TIM1->CNT = overflow ? TIM1->ARR : 0U;
  TIM1->CR1 = overflow ? (TIM1->CR1 | TIM_CR1_DIR) : (TIM1->CR1 & ~TIM_CR1_DIR);
  if (n.rep != 0U)
  {
    n.rep--;
    return;
  }
  n.rep = TIM1->RCR;
  n.active[0] = TIM1->CCR1;
  n.active[1] = TIM1->CCR2;
  n.active[2] = TIM1->CCR3;
  native_reconcile();
  n.sr |= TIM_SR_UIF;
  TIM1->SR = n.sr;
  if ((TIM1->DIER & TIM_DIER_UIE) != 0U)
  {
    native_irq(TIM1_UP_IRQn, TIM1_UP_IRQHandler);
  }
}

/* TRGO2: each started injected group converts, then ADC3's interrupt. */
static void native_convert(void)
{
  ADC_HandleTypeDef *const handles[NATIVE_ADCS] = { &hadc1, &hadc2, &hadc3 };

  TIM1->CNT = TIM1->CCR5;
  for (uint8_t k = 0U; k < NATIVE_ADCS; k++)
  {
    if (n.adc[k].injected)
    {
      handles[k]->Instance->JDR1 = afe_code(k, n.adc[k].rank[0]);
      if (n.adc[k].ranks > 1U)
      {
        handles[k]->Instance->JDR2 = afe_code(k, n.adc[k].rank[1]);
      }
    }
  }
  if (n.adc[2].injected && n.adc[2].it)
  {
    ADC3->ISR |= ADC_FLAG_JEOC | ADC_FLAG_JEOS;
    if (Board_SyncIrq())
    {
      ADC3->ISR &= ~(ADC_FLAG_JEOC | ADC_FLAG_JEOS);
    }
  }
}

/* The edge after `done`, which fell at `at`. */
static void native_schedule(native_edge_t done, uint64_t at)
{
  const uint32_t arr = TIM1->ARR;
  const uint64_t half = (uint64_t)arr * NATIVE_TICK_CYCLES;
  const bool trigger = ((TIM1->CR2 & TIM_CR2_MMS2) == TIM_TRGO2_OC5REF)
                       && (TIM1->CCR5 != 0U) && (TIM1->CCR5 <= arr);

  switch (done)
  {
    case EDGE_UNDERFLOW:
      n.edge = EDGE_OVERFLOW;
      n.edge_at = at + half;
      break;
    case EDGE_OVERFLOW:
      n.over_at = at;
      n.edge = trigger ? EDGE_TRIGGER : EDGE_UNDERFLOW;
      n.edge_at = at + (trigger ? (uint64_t)(arr - TIM1->CCR5) * NATIVE_TICK_CYCLES : half);
      break;
    default:
      n.edge = EDGE_UNDERFLOW;
      n.edge_at = n.over_at + half;
      break;
  }
}

static void native_edge(void)
{
  const native_edge_t edge = n.edge;
  const uint64_t at = n.edge_at;

  native_plant_to(at);
  n.in_isr = true;
  if (edge == EDGE_TRIGGER)
  {
    native_convert();
  }
  else
  {
    native_update(edge == EDGE_OVERFLOW);
  }
  n.in_isr = false;
  native_schedule(edge, at);
}

static uint64_t native_cycles_per_ms(void)
{
  return SystemCoreClock / 1000U;
}

/* The clock on to `target`: the timer's edges and the parts' milliseconds before it run. */
static void native_until(uint64_t target)
{
  for (;;)
  {
    const uint64_t edge = ((TIM1->CR1 & TIM_CR1_CEN) != 0U) ? n.edge_at : UINT64_MAX;
    const uint64_t next = (edge < n.tick_at) ? edge : n.tick_at;

    if (next > target)
    {
      break;
    }
    native_clock_to(next);
    if (next == n.tick_at)
    {
      n.tick_at += native_cycles_per_ms();
      bno085_tick();
    }
    else
    {
      native_edge();
    }
  }
  native_clock_to(target);
}

/* ---- what the parts and the board layer call ---------------------------------------- */

uint64_t native_now(void)
{
  return n.cycles;
}

void native_wait(uint64_t cycles)
{
  const uint64_t at = n.cycles + cycles;

  native_reconcile();
  if (n.in_isr || (native_primask != 0U))
  {
    native_clock_to(at);
    return;
  }
  native_until(at);
  native_io_poll();
}

bool native_irq_enabled(int irq)
{
  return (irq >= 0) && (irq < 64) && (((n.irqs >> (unsigned)irq) & 1U) != 0U);
}

void native_irq(int irq, void (*handler)(void))
{
  if (!native_irq_enabled(irq))
  {
    return;
  }
  const bool was = n.in_isr;

  n.in_isr = true;
  handler();
  n.in_isr = was;
  native_reconcile();
}

bool native_powered(void)
{
  return Board_AfeOn();
}

double native_angle_celsius(void)
{
  return n.seen.afe_c;
}

/* The magnet's mechanical angle, degrees: the plant's, or as set, or an invented turn. */
double native_shaft_degrees(void)
{
  if (w.set)
  {
    return w.degrees;
  }
  if (w.step != NULL)
  {
    return w.mechanical * NATIVE_RAD_TO_DEG;
  }
  return 360.0 * ((double)n.cycles / (double)SystemCoreClock) / NATIVE_TURN_S;
}

uint32_t Board_Cycles(void)
{
  native_wait(NATIVE_READ_CYCLES);
  return (uint32_t)n.cycles;
}

/* Asleep until an interrupt: SysTick's millisecond, or the timer's edge when one of its
   interrupts is on - no further than the host's step. The interrupt runs as PRIMASK lets it. */
void native_wfi(void)
{
  uint64_t at = n.tick_at;
  const bool timer = (((TIM1->DIER & TIM_DIER_UIE) != 0U) && native_irq_enabled(TIM1_UP_IRQn))
                     || (n.adc[2].injected && n.adc[2].it);

  if (timer && (n.edge_at < at))
  {
    at = n.edge_at;
  }
  if (n.end < at)
  {
    at = n.end;
  }
  native_clock_to(at);
}

uint32_t HAL_GetTick(void)
{
  native_wait(NATIVE_READ_CYCLES);
  return (uint32_t)(n.cycles / native_cycles_per_ms());
}

void HAL_Delay(uint32_t ms)
{
  native_wait((uint64_t)ms * native_cycles_per_ms());
}

void HAL_NVIC_SetPriority(IRQn_Type irq, uint32_t preempt, uint32_t sub)
{
  (void)irq;
  (void)preempt;
  (void)sub;
}

void HAL_NVIC_EnableIRQ(IRQn_Type irq)
{
  n.irqs |= (uint64_t)1U << (unsigned)irq;
}

void HAL_NVIC_DisableIRQ(IRQn_Type irq)
{
  n.irqs &= ~((uint64_t)1U << (unsigned)irq);
}

static uint8_t native_adc(const ADC_HandleTypeDef *hadc)
{
  return (hadc == &hadc1) ? 0U : (hadc == &hadc2) ? 1U : 2U;
}

HAL_StatusTypeDef HAL_ADC_Init(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_ConfigChannel(ADC_HandleTypeDef *hadc, const ADC_ChannelConfTypeDef *c)
{
  n.adc[native_adc(hadc)].channel = c->Channel;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_Start(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADC_PollForConversion(ADC_HandleTypeDef *hadc, uint32_t timeout_ms)
{
  (void)timeout_ms;
  hadc->Instance->DR = afe_code(native_adc(hadc), n.adc[native_adc(hadc)].channel);
  return HAL_OK;
}

uint32_t HAL_ADC_GetValue(const ADC_HandleTypeDef *hadc)
{
  return hadc->Instance->DR;
}

HAL_StatusTypeDef HAL_ADC_Stop(ADC_HandleTypeDef *hadc)
{
  (void)hadc;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedConfigChannel(ADC_HandleTypeDef *hadc,
                                                  const ADC_InjectionConfTypeDef *c)
{
  const uint8_t k = native_adc(hadc);

  if ((c->InjectedRank < 1U) || (c->InjectedRank > NATIVE_RANKS))
  {
    return HAL_ERROR;
  }
  n.adc[k].rank[c->InjectedRank - 1U] = c->InjectedChannel;
  n.adc[k].ranks = c->InjectedNbrOfConversion;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedStart(ADC_HandleTypeDef *hadc)
{
  n.adc[native_adc(hadc)].injected = true;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedStart_IT(ADC_HandleTypeDef *hadc)
{
  n.adc[native_adc(hadc)].injected = true;
  n.adc[native_adc(hadc)].it = true;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedStop(ADC_HandleTypeDef *hadc)
{
  n.adc[native_adc(hadc)].injected = false;
  return HAL_OK;
}

HAL_StatusTypeDef HAL_ADCEx_InjectedStop_IT(ADC_HandleTypeDef *hadc)
{
  n.adc[native_adc(hadc)].injected = false;
  n.adc[native_adc(hadc)].it = false;
  return HAL_OK;
}

/* One pass of main()'s loop, as main.c's USER CODE runs it. */
static void native_loop(void)
{
  Board_StoKeepalive();
  Board_PowerPoll();
  if (!link_busy())
  {
    Board_ImuPoll();
    Board_AnglePoll();
    Board_DaqPoll();
    Board_ThermalPoll();
  }
  link_poll();
  if (!link_busy())
  {
    Board_StoIdle();
  }
  native_io_poll();
  native_reconcile();
}

/* The exchange's clock `us` on: the edges in it run, and a pass of main()'s loop at least
   every NATIVE_LOOP_US. */
void fake_advance(uint32_t us)
{
  const uint64_t end = n.cycles + (uint64_t)us * native_cycles_per_us();

  n.end = end;
  do
  {
    const uint64_t pass = n.cycles + (uint64_t)NATIVE_LOOP_US * native_cycles_per_us();

    native_until((pass < end) ? pass : end);
    native_loop();
  } while (n.cycles < end);
}

/* ---- the host's side, through ctypes ------------------------------------------------ */

/** The front end's phase transfer and spread (coaxial_63100_afe.repl), this board's errors
    drawn from `seed`. */
void native_afe(double volts_per_amp, double zero_volts, double gain_sigma, double zero_sigma,
                double bus_sigma, uint32_t seed)
{
  afe.v_per_a = volts_per_amp;
  afe.zero_v = zero_volts;
  afe.gain_sigma = gain_sigma;
  afe.zero_sigma = zero_sigma;
  afe.bus_sigma = bus_sigma;
  afe.rng = (seed != 0U) ? seed : 1U;
  for (uint8_t leg = 0U; leg < BOARD_PWM_PHASES; leg++)
  {
    afe.gain_err[leg] = afe_gauss() * gain_sigma;
    afe.zero_err[leg] = afe_gauss() * zero_sigma;
  }
  afe.bus_err = afe_gauss() * bus_sigma;
}

/** This board a node of a world (world/src/world_emu.c's step and shaft), its motor's pole
    pairs to turn the shaft's electrical angle mechanical. */
void native_world(void *step, void *shaft, int node, double pole_pairs)
{
  w.step = (native_step_t)step;
  w.shaft = (native_shaft_t)shaft;
  w.node = node;
  w.pole_pairs = (pole_pairs > 0.0) ? pole_pairs : 1.0;
  w.electrical = 0.0;
  w.mechanical = 0.0;
}

/** The magnet's angle as the host puts it, degrees, the plant's no longer. */
void native_angle(double degrees)
{
  w.degrees = degrees;
  w.set = true;
}

/* The handover slot a bootloader leaves (boot_hand_t), native_hand's; board_boot.c's reading
   of it below, which cannot build here - its image header takes the linker's addresses. */
static struct
{
  bool    assigned;
  uint8_t unit;
  uint8_t position;
  uint8_t flags;
} h;

/** What a bootloader leaves this board in the slot, before native_open: its unit, its
    position down the limb, its flags (BOOT_FLAG_TERMINATE). */
void native_hand(uint8_t unit, uint8_t position, uint8_t flags)
{
  h.assigned = true;
  h.unit = unit;
  h.position = position;
  h.flags = flags;
}

void Board_BootInit(void)
{
  if (!h.assigned)
  {
    return;                          /* no bootloader assigned anything */
  }
  (void)modbus_map_set_unit_id(h.unit);
  Board_SetTermination((h.flags & BOOT_FLAG_TERMINATE) != 0U);
}

board_identity_t Board_Identity(void)
{
  board_identity_t id;

  memset(&id, 0, sizeof id);
  id.assigned = h.assigned;
  id.type = BOARD_BOOT_TYPE;
  id.unit = modbus_map_unit_id();
  id.position = h.assigned ? h.position : 0U;
  id.flags = h.assigned ? h.flags : 0U;
  return id;
}

/** Power on: the fake board's stack, TIM1 as CubeMX leaves it and the update its UG makes,
    then the board's own init in main()'s order - the stage, the record, its dead time, the
    triple and the drive. */
void native_open(void)
{
  memset(&n, 0, sizeof n);
  memset(&w, 0, sizeof w);
  memset(&native_tim1, 0, sizeof native_tim1);
  memset(&native_rcc, 0, sizeof native_rcc);
  memset(&native_adc1, 0, sizeof native_adc1);
  memset(&native_adc2, 0, sizeof native_adc2);
  memset(&native_adc3, 0, sizeof native_adc3);
  native_primask = 0U;
  memset(afe.amps, 0, sizeof afe.amps);
  afe.dc = 0.0;
  world_heat_init(&n.heat, HEAT_AMBIENT_C);
  n.seen.ntc_c = n.seen.mcu_c = n.seen.afe_c = HEAT_AMBIENT_C;
  afe.ntc_c = afe.die_c = HEAT_AMBIENT_C;
  native_ts_cal[0] = (uint16_t)lround(afe_die(30.0) / afe.ref * NATIVE_FULL_CODE);
  native_ts_cal[1] = (uint16_t)lround(afe_die(110.0) / afe.ref * NATIVE_FULL_CODE);
  n.tick_at = native_cycles_per_ms();
  native_io_open();
  a1335_open();
  bno085_open();

  fake_open();
  RCC->APB2ENR = RCC_APB2ENR_TIM1EN;
  TIM1->CR1 = TIM_CR1_CMS | TIM_CR1_ARPE;
  TIM1->ARR = NATIVE_ARR;
  TIM1->RCR = NATIVE_RCR;
  TIM1->BDTR = NATIVE_DTG | TIM_BDTR_BKE;
  n.rep = NATIVE_RCR;
  (void)Board_PwmInit();
  native_reconcile();
  n.edge = EDGE_OVERFLOW;
  n.edge_at = (uint64_t)NATIVE_ARR * NATIVE_TICK_CYCLES;
  (void)Board_PwmSetDeadTime(Board_Cal()->deadtime_ns);
  (void)Board_PwmSetDeadTimeSkew((int8_t)Board_Cal()->deadtime_skew);
  Board_SyncDisarm();
  Board_DriveInit();
}

/** `us` of the board's time. */
void native_run(uint32_t us)
{
  fake_advance(us);
}

/** The board's clock on to `us` since native_open, if it stands short of it: a limb's boards
    run to one time, none drifting by what its own reads added. */
void native_run_to(uint64_t us)
{
  const uint64_t at = us * native_cycles_per_us();

  if (at > n.cycles)
  {
    fake_advance((uint32_t)((at - n.cycles + native_cycles_per_us() - 1U)
                            / native_cycles_per_us()));
  }
}

/** The board's clock since native_open, s. */
double native_seconds(void)
{
  return (double)n.cycles / (double)SystemCoreClock;
}
