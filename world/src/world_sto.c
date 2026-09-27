/** world_sto.c - The STO chain: sto.asc's circuit at 1 us, backward Euler on every capacitor,
    a bracketed Newton on the nodes the diodes make; the schematic's parts, LTspice's models of
    them (sto.asc, TLV3492.LIB, TPS3840PL30.LIB, NL7SZ97.LIB, its SSM6N815R). */
#include "world_sto.h"

#include <math.h>
#include <string.h>

/* kT/q at LTspice's 27 C. */
#define VT          0.025865
#define TWO_PI      6.28318531

/* The master (sto.asc): its amplifier's bias and rails, the coupling caps C4 and C5 in series,
   the loop through R8, R9, its split termination R4 || R5, the far end's R10 || R11 and the
   caps' ESR, closed through the far end's disturbance (sto.asc's CMNOISE, 100 kHz); the pair's
   common mode is that disturbance and the loop's current in R10 || R11. */
#define PILOT_BIAS  1.65
#define PILOT_TOP   5.0
#define LOOP_C      20e-6
#define LOOP_R      68.19
#define FAR_R       30.0
#define NOISE_HZ    100e3

/* The pickup: R36 || R37 and R53 about C75 into TP67, R40 and R89 (from +5), C103 there, the
   clamp D3 and D4; the band-pass C70, R41, R44 || C98, R45 || C99; U16C at R88/R86 of +5, R123
   its hysteresis. */
#define PICK_R      7.2e3
#define C75         33e-9
#define R40         10e3
#define R89         1e6
#define C103        150e-12
#define C70         10e-9
#define R41         3.3e3
#define R44         15e3
#define C98         1.2e-9
#define R45         33e3
#define C99         1.2e-9
#define R123        3.01e6
#define ZERO_CROSS  (1e3 / 101e3)
/* U16B's threshold: R73 from +5, R87 to ground, R122 from its output - 543 or 171 mV. */
#define R73         39e3
#define R87         1.5e3
#define R122        18e3
/* U16, TLV3492 as its macro-model: an input pair's 1 uA tail into 25k, full at 39 mV, 0.2 mA/V
   onto 9 pF and 3.5 Mohm, clamped 2.5 V either side; the output follows that node's sign. */
#define CMP_SLEW    (1e-6 * 25e3 * 0.2e-3 / 9e-12)
#define CMP_FULL    0.039
#define CMP_LEAK    (1.0 / (3.5e6 * 9e-12))
#define CMP_RAIL    2.5
/* The charge path R54, C101, D12 from ground, D11 into Cinj; the discharge path D13, R55. */
#define R54         2.2e3
#define C101        47e-9
#define R55         220.0
/* Cinj, leaky integrator #1: C102, R96, R43 from +3V3D, U4's own 350 nA. */
#define C102        100e-9
#define R96         33e3
#define R43         220e3
#define U4_A        350e-9
/* U4, TPS3840PL30: VIT- 3.0 V, VIT+ 3.1 V, 1.5 V to act at all; CT (its 129 pF and C105's
   220) charged at 1.99 uA to 1.23 V, 216 us; a fall under VIT- acted on 30 us late; RESET at
   0.8 VDD through 1 ohm, or ground through 50. */
#define VIT_LO      3.0
#define VIT_HI      3.1
#define VDD_MIN     1.5
#define CT_SLOPE    (1.98727e-6 / (129.25e-12 + 220e-12))
#define CT_TRIP     1.23
#define FALL_S      30e-6
#define RESET_HIGH  0.8
#define R_UP        1.0
#define R_DOWN      50.0
/* The keepalive's pump: R72, C71, D15 from ground, D14 and R95 into Clevel, D10 onto RESET;
   Clevel, leaky integrator #2: C106 and Q10A's gate, R98. */
#define R72         330.0
#define C71         100e-9
#define R95         27.0
#define C106        10.26e-9
#define R98         18e3
/* Q10 (SSM6N815R: Vto 1.8, Kp 4.5, ksubthres 0.1, Rd + Rs 55 mohm), R93 from +3V3D, R92 on
   +15V7. */
#define VTO         1.8
#define KP          4.5
#define KSUB        0.1
#define R_DS        0.055
#define R93         15e3
#define R92         3.9
/* U11, NL7SZ97: Y = C ? A : B, A grounded, Schmitt inputs at 0.64 and 0.36 VCC, 16 ohm into
   R34's 1.5k. */
#define VCC         3.3
#define SCHMITT_HI  (0.64 * VCC)
#define SCHMITT_LO  (0.36 * VCC)
#define FAULT_HIGH  (VCC * 1.5e3 / (1.5e3 + 16.0))
/* U9 as sto.asc's S1 and V5: 15.5 V through 2 ohm, on over 1.1 V at EN and off under 0.4; C9
   and R28 the rail's capacitance and the drivers' draw. */
#define RAIL15      15.5
#define DCDC_R      2.0
#define EN_ON       1.1
#define EN_OFF      0.4
#define C9          30e-6
#define R28         50.0
/* PGD: the LM5069's UVLO and OVLO pins at 2.5 V on R20, R19, R21 (82k, 8.2k, 3.3k), its 21 uA
   of hysteresis through the divider's top. */
#define UV_ON       20.3
#define UV_OFF      18.6
#define OV_OFF      70.8
#define OV_ON       68.9

/* The hold: a regime's outputs averaged over HOLD_WINDOW s agreeing with the last window's to
   HOLD_V (Cinj, Clevel: the keepalive's ripple sampled at the steps' ends parts two windows by
   1.3 mV) and HOLD_GATE (+15V7), the keepalive's every gap within HOLD_GAP s (5 to 25 us moves
   Clevel 2.86 to 2.72 V) - or the chain tripped, which a pump slower still keeps down. */
#define HOLD_WINDOW 5e-3
#define HOLD_V      5e-3
#define HOLD_GATE   10e-3
#define HOLD_GAP    25e-6

/* Newton: iterations at most, the settled step (V, A). */
#define NEWTON      40
#define SETTLED     1e-9
#define SETTLED_A   1e-12
#define GMIN        1e-12

/* CTS05S40 (Is 10 uA, N 1.05) and 1SS387CT (0.8 nA, 1.8): exponential to about an amp, then on
   that slope. */
typedef struct
{
  double is, nvt, x_max;
} diode_t;

static const diode_t SCHOTTKY = { 10e-6, 1.05 * VT, 11.5 };
static const diode_t CLAMP    = { 0.8e-9, 1.8 * VT, 21.0 };

static inline double lower(double a, double b)
{
  return (a < b) ? a : b;
}

static inline double upper(double a, double b)
{
  return (a > b) ? a : b;
}

/** e^x for x over -700: 2^k in the exponent field, e^r to r^8 (4e-10). */
static double fexp(double x)
{
  if (x < -700.0)
  {
    return 0.0;
  }
  const double k = (double)(int64_t)(x * 1.4426950408889634 + 1024.5) - 1024.0;
  const double r = x - k * 0.6931471805599453;
  union
  {
    double  f;
    int64_t i;
  } u;

  u.f = 1.0 + r * (1.0 + r * (0.5 + r * (1.0 / 6.0 + r * (1.0 / 24.0 + r * (1.0 / 120.0
        + r * (1.0 / 720.0 + r * (1.0 / 5040.0 + r * (1.0 / 40320.0))))))));
  u.i += (int64_t)k << 52;
  return u.f;
}

/** ln x for x over 0: the exponent field's k ln 2, the mantissa's by atanh to s^9 (1e-9). */
static double flog(double x)
{
  union
  {
    double  f;
    int64_t i;
  } u;

  u.f = x;
  int64_t k = ((u.i >> 52) & 0x7FF) - 1023;

  u.i = (u.i & 0x000FFFFFFFFFFFFF) | 0x3FF0000000000000;
  if (u.f > 1.41421356)
  {
    u.f *= 0.5;
    k++;
  }
  const double t = (u.f - 1.0) / (u.f + 1.0);
  const double t2 = t * t;

  return (double)k * 0.6931471805599453
         + 2.0 * t * (1.0 + t2 * (1.0 / 3.0 + t2 * (1.0 / 5.0 + t2 * (1.0 / 7.0 + t2 / 9.0))));
}

/** The current forward through `d` at `v`, its slope into `g`. */
static double diode(const diode_t *d, double v, double *g)
{
  const double x = v / d->nvt;

  if (x > d->x_max)
  {
    const double e = fexp(d->x_max);

    *g = d->is * e / d->nvt;
    return d->is * (e - 1.0) + *g * (v - d->x_max * d->nvt);
  }
  const double e = fexp(x);

  *g = d->is * e / d->nvt + GMIN;
  return d->is * (e - 1.0) + GMIN * v;
}

/** Newton's next guess from `v` for a falling f, the bracket [lo, hi] narrowed by f's sign and
    halved where Newton would leave it. */
static double guard(double v, double f, double df, double *lo, double *hi)
{
  if (f > 0.0)
  {
    *lo = v;
  }
  else
  {
    *hi = v;
  }
  const double next = v - f / df;

  return ((next >= *lo) && (next <= *hi)) ? next : 0.5 * (*lo + *hi);
}

/** `d` in series with `r`, `v` across both: the current, the pair's conductance into `g`; `i`
    the last current, where Newton starts. Reversed, r carries next to nothing; forward, on the
    current r i + n Vt ln(1 + i / Is) = v is concave, and Newton kept over a current under its
    root (r's share once the diode takes the most it could) climbs to it. */
static double series(const diode_t *d, double r, double v, double *i, double *g)
{
  if (v <= 0.0)
  {
    const double e = fexp(v / d->nvt);
    const double gd = d->is * e / d->nvt;

    *i = d->is * (e - 1.0);
    *g = gd / (1.0 + gd * r);
    return *i;
  }
  const double under = upper(0.0, (v - d->nvt * flog(1.0 + v / (r * d->is))) / r);
  double x = upper(*i, under);
  double slope = r + d->nvt / (d->is + x);

  for (int k = 0; k < NEWTON; k++)
  {
    slope = r + d->nvt / (d->is + x);
    const double next = upper(x - (r * x + d->nvt * flog(1.0 + x / d->is) - v) / slope, under);
    const bool settled = fabs(next - x) < SETTLED_A;

    x = next;
    if (settled)
    {
      break;
    }
  }
  *i = x;
  *g = 1.0 / slope;
  return x;
}

/** One of U16's halves: its node moved `dt` on an input `over` its threshold; high while that
    node is. */
static bool compare(double *node, double over, double dt)
{
  const double drive = lower(upper(over / CMP_FULL, -1.0), 1.0);

  *node = lower(upper(*node + dt * (CMP_SLEW * drive - CMP_LEAK * *node), -CMP_RAIL), CMP_RAIL);
  return *node > 0.0;
}

/** A MOSFET's overdrive, V: the square law's, ksubthres smoothing it through Vto. */
static double overdrive(double vgs)
{
  const double x = (vgs - VTO) / KSUB;

  return (x > 20.0) ? (vgs - VTO) : KSUB * log(1.0 + fexp(x));
}

/** Q10A's drain on R93 at Clevel `vgs`: saturated while the drop leaves it over its overdrive,
    else the triode's root. */
static double q10a_drain(double vgs)
{
  const double ov = overdrive(vgs);
  const double g = 1.0 / R93;
  const double sat = 0.5 * KP * ov * ov;

  if ((VCC - sat / g) >= ov)
  {
    return VCC - sat / g;
  }
  const double b = KP * ov + g;

  return (b - sqrt(b * b - 2.0 * KP * g * VCC)) / KP;
}

/** TP34 at Cinj `c`: U16C's pulse train through R54 and C101 (`e`, `r`), D12 from ground, D11
    into Cinj; D11's current, its slope against `c` into `g`. */
static double tp34(world_sto_t *s, double e, double r, double c, double *g)
{
  double lo = lower(e, 0.0) - 0.5;
  double hi = upper(e, c) + 0.5;
  double x = s->n025;
  double g12 = GMIN;
  double g11 = GMIN;
  double i11 = 0.0;

  for (int k = 0; k < NEWTON; k++)
  {
    const double i12 = diode(&SCHOTTKY, -x, &g12);

    i11 = diode(&SCHOTTKY, x - c, &g11);
    const double next = guard(x, (e - x) / r + i12 - i11, -1.0 / r - g12 - g11, &lo, &hi);
    const bool settled = fabs(next - x) < SETTLED;

    x = next;
    if (settled)
    {
      break;
    }
  }
  s->n025 = x;
  *g = g11 * (1.0 / r + g12) / (g11 + 1.0 / r + g12);
  return i11;
}

/** One substep of the whole chain. */
static void substep(world_sto_t *s, const world_sto_in_t *in)
{
  const double dt = WORLD_STO_DT;
  const double v5 = in->rail5 ? 5.0 : 0.0;
  const double u1 = s->u1 ? v5 : 0.0;
  const double u2 = s->u2 ? v5 : 0.0;

  /* The master's amplifier on its coupling caps; the pair's common mode. */
  double drive = PILOT_BIAS;

  if ((in->pilot_volts > 0.0) && (in->pilot_hz > 0.0))
  {
    if (in->pilot_hz != s->hz)
    {
      s->hz = in->pilot_hz;
      s->rc = cos(TWO_PI * s->hz * dt);
      s->rs = sin(TWO_PI * s->hz * dt);
    }
    const double c = s->pc * s->rc - s->ps * s->rs;
    const double sn = s->pc * s->rs + s->ps * s->rc;
    const double norm = 1.5 - 0.5 * (c * c + sn * sn);

    s->pc = c * norm;
    s->ps = sn * norm;
    drive = lower(upper(PILOT_BIAS + in->pilot_volts * s->ps, 0.0), PILOT_TOP);
  }
  double noise = 0.0;

  if (in->noise_volts > 0.0)
  {
    s->noise += NOISE_HZ * dt;
    s->noise -= (s->noise >= 1.0) ? 1.0 : 0.0;
    noise = in->noise_volts * sin(TWO_PI * s->noise);
  }
  const double i_loop = (drive - s->loop - noise) / (LOOP_R + dt / LOOP_C);

  s->loop += dt * i_loop / LOOP_C;
  const double pair = noise + FAR_R * i_loop;

  /* The band-pass as TP67 sees it: C99's node through R45, C98's through R41 and C70, each
     capacitor its companion. */
  const double g99 = C99 / dt;
  const double g98 = C98 / dt;
  const double g70 = C70 / dt;
  const double g103 = C103 / dt;
  const double g10 = g99 + 1.0 / R123;
  const double e10 = (g99 * s->n010 + u1 / R123) / g10;
  const double r45 = R45 + 1.0 / g10;
  const double g7 = g98 + 1.0 / R44 + 1.0 / r45;
  const double e7 = (g98 * s->n007 + e10 / r45) / g7;
  const double r_bp = 1.0 / g7 + R41 + 1.0 / g70;
  const double e_bp = s->c70 + e7;
  const double r_pick = PICK_R + dt / C75;
  const double e_pick = pair - s->c75;

  /* TP67 and its clamp. */
  double lo = lower(lower(e_pick, e_bp), lower(s->n024, 0.0)) - 0.5;
  double hi = upper(upper(e_pick, e_bp), upper(s->n024, 0.0)) + 0.5;
  double v = s->n024;

  for (int k = 0; k < NEWTON; k++)
  {
    double g3;
    double g4;
    const double i3 = diode(&SCHOTTKY, -v, &g3);
    const double i4 = diode(&CLAMP, v, &g4);
    const double f = (e_pick - v) / r_pick + (v5 - v) / R89 - v / R40 - (v - e_bp) / r_bp
                    - g103 * (v - s->n024) + i3 - i4;
    const double df = -1.0 / r_pick - 1.0 / R89 - 1.0 / R40 - 1.0 / r_bp - g103 - g3 - g4;
    const double next = guard(v, f, df, &lo, &hi);
    const bool settled = fabs(next - v) < SETTLED;

    v = next;
    if (settled)
    {
      break;
    }
  }
  s->n024 = v;
  s->c75 += dt * ((e_pick - v) / r_pick) / C75;
  const double i_bp = (v - e_bp) / r_bp;

  s->c70 += i_bp / g70;
  s->n007 = e7 + i_bp / g7;
  s->n010 = e10 + ((s->n007 - e10) / r45) / g10;

  /* Cinj with TP34 on one side and D13 through R55 to U16B on the other: Newton on Cinj, each
     side solved at its every guess. */
  const double r_s = R54 + dt / C101;
  const double e_s = u1 - s->c101;
  const double g_c = C102 / dt + 1.0 / R96 + 1.0 / R43;
  const double e_c = (C102 / dt * s->cinj + VCC / R43 - ((s->cinj > 0.0) ? U4_A : 0.0)) / g_c;
  double c = s->cinj;

  lo = c - 1.0;
  hi = c + 1.0;
  for (int k = 0; k < NEWTON; k++)
  {
    double g11;
    double g13;
    const double i11 = tp34(s, e_s, r_s, c, &g11);
    const double i13 = series(&SCHOTTKY, R55, c - u2, &s->i13, &g13);
    const double next = guard(c, i11 - i13 - (c - e_c) * g_c, -g11 - g13 - g_c, &lo, &hi);
    const bool settled = fabs(next - c) < SETTLED;

    c = next;
    if (settled)
    {
      break;
    }
  }
  s->cinj = c;
  s->c101 += dt * ((e_s - s->n025) / r_s) / C101;

  /* U4 on it. */
  const double vdd = s->cinj;

  if (vdd < VDD_MIN)
  {
    s->sensed = false;
    s->falling = 0.0;
  }
  else if (vdd >= VIT_HI)
  {
    s->sensed = true;
    s->falling = 0.0;
  }
  else if (s->sensed && (vdd < VIT_LO))
  {
    s->falling += dt;
    s->sensed = s->falling < FALL_S;
  }
  else
  {
    s->falling = 0.0;
  }
  s->ct = s->sensed ? lower(s->ct + CT_SLOPE * dt, vdd) : 0.0;
  const bool released = s->sensed && (s->ct >= CT_TRIP);
  const double e_reset = released ? RESET_HIGH * vdd : 0.0;
  const double r_reset = released ? R_UP : R_DOWN;

  /* TP24: the keepalive through R72 and C71, D15 from ground, D14 and R95 into Clevel, D10
     onto RESET; Newton on TP24, each diode's branch solved at its every guess. */
  const double r_src = R72 + dt / C71;
  const double e_src = (s->pin ? VCC : 0.0) - s->c71;
  const double g106 = C106 / dt + 1.0 / R98;
  const double r_th = R95 + 1.0 / g106;
  const double e_th = (C106 / dt) * s->clevel / g106;
  double x = s->n032;
  double i14 = 0.0;
  double i10 = 0.0;

  lo = lower(lower(e_src, e_th), lower(e_reset, 0.0)) - 0.1;
  hi = upper(upper(e_src, e_th), e_reset) + 0.5;
  for (int k = 0; k < NEWTON; k++)
  {
    double g15;
    double g14;
    double g10d;
    const double i15 = diode(&SCHOTTKY, -x, &g15);

    i14 = series(&SCHOTTKY, r_th, x - e_th, &s->i14, &g14);
    i10 = series(&SCHOTTKY, r_reset, x - e_reset, &s->i10, &g10d);
    const double next = guard(x, (e_src - x) / r_src + i15 - i14 - i10,
                              -1.0 / r_src - g15 - g14 - g10d, &lo, &hi);
    const bool settled = fabs(next - x) < SETTLED;

    x = next;
    if (settled)
    {
      break;
    }
  }
  s->n_reset = e_reset + r_reset * i10;
  s->n032 = x;
  s->c71 += dt * ((e_src - x) / r_src) / C71;
  s->clevel = e_th + i14 / g106;

  /* Q10A on Clevel; U11 on its drain and PGD; U9 on U11; +15V7 with Q10B's crowbar. */
  if (fabs(s->clevel - s->gate_at) > 1e-3)
  {
    s->gate_at = s->clevel;
    s->drain = q10a_drain(s->clevel);
    const double ov = overdrive(s->drain);

    s->crowbar = (ov > 1e-6) ? 1.0 / (R92 + 1.0 / (KP * ov) + R_DS) : 0.0;
  }
  s->c_high = s->c_high ? (s->drain > SCHMITT_LO) : (s->drain > SCHMITT_HI);
  s->pgood = s->pgood ? ((in->link_volts > UV_OFF) && (in->link_volts < OV_OFF))
                      : ((in->link_volts > UV_ON) && (in->link_volts < OV_ON));
  s->fault = (!s->c_high && s->pgood) ? FAULT_HIGH : 0.0;
  s->dcdc = s->dcdc ? (s->fault > EN_OFF) : (s->fault > EN_ON);
  const double g_src = s->dcdc ? 1.0 / DCDC_R : 0.0;

  s->gate = ((C9 / dt) * s->gate + g_src * RAIL15)
            / (C9 / dt + g_src + 1.0 / R28 + s->crowbar);

  /* U16C at the zero cross, U16B at its threshold, for the next substep; unpowered, low. */
  const double threshold = (v5 / R73 + u2 / R122) / (1.0 / R73 + 1.0 / R87 + 1.0 / R122);

  s->u1 = compare(&s->cmp1, s->n010 - ZERO_CROSS * v5, dt) && in->rail5;
  s->u2 = compare(&s->cmp2, threshold - s->n024, dt) && in->rail5;
}

void world_sto_init(world_sto_t *s)
{
  memset(s, 0, sizeof(*s));
  s->loop = PILOT_BIAS;
  s->pc = 1.0;
  s->drain = VCC;
  s->gate_at = -1.0;
  s->c_high = true;
  s->cmp1 = -CMP_RAIL;
  s->cmp2 = -CMP_RAIL;
}

/** Whether the keepalive's every gap in the step, from the last edge before it, is within
    HOLD_GAP: PA10 moved between two steps is an edge at this one's start. */
static bool pumping(world_sto_t *s, const world_sto_in_t *in, double dt)
{
  double last = (in->keepalive != s->pin) ? 0.0 : -s->since;
  double worst = 0.0;

  for (uint32_t k = 0U; k < in->edges; k++)
  {
    const double at = (in->at != NULL) ? (double)in->at[k]
                                       : ((double)k + 0.5) * dt / (double)in->edges;

    worst = upper(worst, at - last);
    last = at;
  }
  worst = upper(worst, dt - last);
  s->since = dt - last;
  return worst <= HOLD_GAP;
}

/** Whether the chain may hold in `now`: pumped, or down - FAULTOUT low. */
static bool holdable(const world_sto_t *s, const world_sto_regime_t *now)
{
  return now->pumping || (s->fault <= 0.0);
}

static bool same(const world_sto_regime_t *a, const world_sto_regime_t *b)
{
  return (a->pilot_volts == b->pilot_volts) && (a->pilot_hz == b->pilot_hz)
         && (a->noise_volts == b->noise_volts) && (a->pumping == b->pumping)
         && (a->rail5 == b->rail5) && (a->pgood == b->pgood);
}

void world_sto_step(world_sto_t *s, const world_sto_in_t *in, float dt, world_sto_out_t *out)
{
  const world_sto_regime_t now =
  {
    in->pilot_volts, in->pilot_hz, in->noise_volts, pumping(s, in, dt), in->rail5,
    s->pgood ? ((in->link_volts > UV_OFF) && (in->link_volts < OV_OFF))
             : ((in->link_volts > UV_ON) && (in->link_volts < OV_ON))
  };
  const bool kept = same(&now, &s->regime);

  s->held = s->held && kept && holdable(s, &now);
  if (s->held)
  {
    s->pin = ((in->edges & 1U) != 0U) ? !in->keepalive : in->keepalive;
  }
  else
  {
    const double span = dt + s->owed;
    const uint32_t n = (span > 0.0) ? (uint32_t)(span / WORLD_STO_DT) : 0U;
    uint32_t edge = 0U;

    s->owed = span - (double)n * WORLD_STO_DT;
    s->pin = in->keepalive;
    for (uint32_t k = 0U; k < n; k++)
    {
      const double t = (double)k * WORLD_STO_DT;

      while ((edge < in->edges)
             && (((in->at != NULL) ? in->at[edge]
                                   : ((double)edge + 0.5) * dt / (double)in->edges) <= t))
      {
        s->pin = !s->pin;
        edge++;
      }
      substep(s, in);
    }
    if (((in->edges - edge) & 1U) != 0U)
    {
      s->pin = !s->pin;
    }

    /* The windows: a regime's outputs averaged over HOLD_WINDOW; two in a row agreeing, held. */
    if (!kept)
    {
      memset(s->sums, 0, sizeof(s->sums));
      s->window = 0.0;
      s->settled = false;
      s->regime = now;
    }
    s->sums[0] += s->cinj * dt;
    s->sums[1] += s->clevel * dt;
    s->sums[2] += s->gate * dt;
    s->window += dt;
    if (s->window >= HOLD_WINDOW)
    {
      const double cinj = s->sums[0] / s->window;
      const double clevel = s->sums[1] / s->window;
      const double gate = s->sums[2] / s->window;

      s->held = s->settled && holdable(s, &now) && (fabs(cinj - s->means[0]) < HOLD_V)
                && (fabs(clevel - s->means[1]) < HOLD_V) && (fabs(gate - s->means[2]) < HOLD_GATE);
      s->means[0] = cinj;
      s->means[1] = clevel;
      s->means[2] = gate;
      s->settled = true;
      memset(s->sums, 0, sizeof(s->sums));
      s->window = 0.0;
    }
  }
  /* Held, the windows' means: the state stands at one instant of the ripple. */
  out->cinj = (float)(s->held ? s->means[0] : s->cinj);
  out->clevel = (float)(s->held ? s->means[1] : s->clevel);
  out->reset = (float)s->n_reset;
  out->faultout = (float)s->fault;
  out->vgate = (float)(s->held ? s->means[2] : s->gate);
}
