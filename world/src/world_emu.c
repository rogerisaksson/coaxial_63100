/** world_emu.c - One world and a plant a board, flat, for the emulator's bridge to call. */

/* A process's world: board/emu/Coaxial63100_Plant.cs loads this library once per Renode and
   each emulated board steps its own plant on it, so a limb's boards share one body. Scalars
   in, scalars out: nothing for P/Invoke to marshal but floats. */
#include "world.h"
#include "world_heat.h"
#include "world_parts.h"
#include "world_sto.h"

#include <math.h>
#include <string.h>

static struct
{
  world_t        world;
  world_load_t   load0[WORLD_MOTORS];
  float          j0[WORLD_MOTORS];
  float          b0[WORLD_MOTORS];
  world_plant_t  plant[WORLD_MOTORS];
  double         t[WORLD_MOTORS];     /* each plant's own time, s */
  bool           attached[WORLD_MOTORS];
  world_heat_t   heat[WORLD_MOTORS];
  world_sto_t    sto[WORLD_MOTORS];
  world_afe_t    afe[WORLD_MOTORS];
  world_a1335_t  a1335[WORLD_MOTORS];
  world_bno085_t bno085[WORLD_MOTORS];
} s;

/** A fresh world of `motors` loads, all free, on the ground. */
void emu_world_reset(int motors)
{
  memset(&s.world, 0, sizeof(s.world));
  memset(s.load0, 0, sizeof(s.load0));
  memset(s.j0, 0, sizeof(s.j0));
  memset(s.b0, 0, sizeof(s.b0));
  memset(s.plant, 0, sizeof(s.plant));
  memset(s.t, 0, sizeof(s.t));
  memset(s.attached, 0, sizeof(s.attached));
  s.world.motors = (uint8_t)((motors > (int)WORLD_MOTORS) ? WORLD_MOTORS : (unsigned)motors);
  s.world.body.gravity = 9.81f;
  world_init(&s.world);
}

void emu_world_body(int kind, float mass, float gravity, float slope, float crr, float cda,
                    float rho, float rider, float period, float duty)
{
  world_body_t *b = &s.world.body;

  b->kind = (uint8_t)kind;
  b->mass = mass;
  b->gravity = gravity;
  b->slope = slope;
  b->crr = crr;
  b->cda = cda;
  b->rho = rho;
  b->rider = rider;
  b->period = period;
  b->duty = duty;
}

void emu_world_load(int i, int kind, float gear, float inertia, float mass, float arm,
                    float damping, float k_drag, float k_thrust, float radius, float angle)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  world_load_t *l = &s.world.load[i];

  l->kind = (uint8_t)kind;
  l->gear = gear;
  l->inertia = inertia;
  l->mass = mass;
  l->arm = arm;
  l->damping = damping;
  l->k_drag = k_drag;
  l->k_thrust = k_thrust;
  l->radius = radius;
  l->angle = angle;
  l->torque = 0.0f;
  s.load0[i] = *l;
}

/** Board `i`'s load laid live: its drag, N m per (rad/s)^2, and a torque against its turning,
    N m - a page's propeller and its stage. */
void emu_world_drag(int i, float k_drag, float torque)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  if (k_drag == 0.0f)
  {
    s.world.load[i].mass = 0.0f;
  }
  s.world.load[i].k_drag = k_drag;
  s.world.load[i].torque = torque;
}

/** Board `i`'s model parameters laid live: J, b, and load torque. */
void emu_world_model(int i, float j, float b, float load)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  if ((isnan(j) && isnan(b) && isnan(load)) || (!isnan(j) && (j <= 0.0f)))
  {
    s.world.load[i] = s.load0[i];
    s.plant[i].j_motor = s.j0[i];
    s.plant[i].motor.p.j = s.j0[i];
    s.plant[i].motor.p.b = s.b0[i];
    s.plant[i].motor.p.load = 0.0f;
    return;
  }
  if (!isnan(j) && (j > 0.0f))
  {
    s.plant[i].j_motor = j;
    s.plant[i].motor.p.j = j;
    s.world.load[i].inertia = 0.0f;
  }
  if (!isnan(b) && (b >= 0.0f))
  {
    s.plant[i].motor.p.b = b;
    s.world.load[i].damping = 0.0f;
  }
  if (!isnan(load))
  {
    s.world.load[i].mass = 0.0f;
    s.world.load[i].k_drag = 0.0f;
    s.world.load[i].torque = load;
    s.plant[i].motor.p.load = load;
  }
}

/** Board `i`'s rotor back to rest at theta0. */
void emu_plant_reset(int i)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS) || !s.attached[i])
  {
    return;
  }
  drive_model_init(&s.plant[i].motor);
  s.plant[i].theta_was = s.plant[i].motor.theta;
  s.plant[i].shaft = s.plant[i].motor.theta / s.plant[i].motor.p.pole_pairs;
  world_motor(&s.world, (uint8_t)i, s.plant[i].shaft, 0.0f);
}

/** Board `i`'s motor, in drive_model's terms: its PMSM, the link it runs from. */
void emu_plant_attach(int i, float r, float ld, float lq, float lambda, float pole_pairs,
                      float j, float b, float vdc, float noise)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  drive_model_params_t p;

  drive_model_defaults(&p);
  p.r = r;
  p.ld = ld;
  p.lq = lq;
  p.lambda = lambda;
  p.pole_pairs = pole_pairs;
  p.j = j;
  p.b = b;
  p.vdc = vdc;
  p.noise = noise;
  s.j0[i] = j;
  s.b0[i] = b;
  world_plant_init(&s.plant[i], &s.world, (uint8_t)i, &p);
  s.attached[i] = true;
}

/** Board `i`'s period: the phase currents and the link into `out` (i_a, i_b, i_c, vdc). */
void emu_plant_step(int i, float d0, float d1, float d2, int driven, float ts, float *out)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS) || !s.attached[i])
  {
    out[0] = out[1] = out[2] = out[3] = 0.0f;
    return;
  }
  const float duty[DRIVE_PHASES] = { d0, d1, d2 };
  drive_sample_t sample;

  world_plant_step(&s.plant[i], duty, driven != 0, ts, &sample);
  s.t[i] += (double)ts;
  world_advance(&s.world, s.t[i]);
  out[0] = sample.i[0];
  out[1] = sample.i[1];
  out[2] = sample.i[2];
  out[3] = sample.vdc;
}

/** A load's motion set from outside, a board's plant not stepping it: a test, a script. */
void emu_world_motor(int i, float shaft, float speed)
{
  world_motor(&s.world, (uint8_t)i, shaft, speed);
}

/** The body brought to `t`, s. */
void emu_world_advance(double t)
{
  world_advance(&s.world, t);
}

/** Board `i`'s shaft: [angle rad electrical, speed rad/s mechanical, load's output]. */
void emu_plant_state(int i, float *out)
{
  const world_plant_t *p = &s.plant[(i >= 0 && i < (int)WORLD_MOTORS) ? i : 0];

  out[0] = p->motor.theta;
  out[1] = p->motor.omega / p->motor.p.pole_pairs;
  out[2] = world_output(&s.world, (uint8_t)i);
}

/** The body: [height m, climb m/s, velocity m/s, distance m, time s]. */
void emu_world_state(float *out)
{
  out[0] = s.world.height;
  out[1] = s.world.climb;
  out[2] = s.world.velocity;
  out[3] = s.world.distance;
  out[4] = (float)s.world.t;
}

/** Board `i`'s heat, every node at `ambient`, C. */
void emu_heat_reset(int i, float ambient)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  world_heat_init(&s.heat[i], ambient);
}

/** Board `i`'s room: `ambient`, C, its air path and laminate capacity scaled - a situation's
    (world_heat_room). */
void emu_heat_room(int i, float ambient, float air, float capacity)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  world_heat_room(&s.heat[i], ambient, air, capacity);
}

/** Board `i`'s heat's nodes now, C: THERMAL_NODES into `out`. */
void emu_heat_nodes(int i, float *out)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  memcpy(out, s.heat[i].th.t, sizeof s.heat[i].th.t);
}

/** Board `i`'s heat mounted in `app`, thermal_app_t (world_heat_application). */
void emu_heat_application(int i, int app)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  world_heat_application(&s.heat[i], (thermal_app_t)app);
}

/** Board `i`'s heat on `dt` s. in, its thermal_load_t a row of EMU_HEAT_IN: AFE_ON, MOE, the
    three duties, the legs' mean squares (A^2), the link (V), the shaft (rpm), the frame's airspeed
    (m/s), the link's amps (<0 off the legs'), the dead time (s), the legs' amps (A); out: the
    NTC's element, the MCU's die, the A1335's die (C). */
void emu_heat_step(int i, float dt, const float *in, float *out)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  thermal_load_t load;
  thermal_sense_t seen;

  memset(&load, 0, sizeof(load));
  load.afe_on = in[0] != 0.0f;
  load.switching = in[1] != 0.0f;
  for (int k = 0; k < 3; k++)
  {
    load.duty[k] = in[2 + k];
    load.phase_sq[k] = in[5 + k];
  }
  load.link_volts = in[8];
  load.speed_rpm = in[9];
  load.airspeed_m_s = in[10];
  load.link_amps = in[11];
  load.t_dead_s = in[12];
  for (int k = 0; k < 3; k++)
  {
    load.phase_amps[k] = in[13 + k];
  }
  world_heat_step(&s.heat[i], &load, dt, &seen);
  out[0] = seen.ntc_c;
  out[1] = seen.mcu_c;
  out[2] = seen.afe_c;
}

/** Board `i`'s STO chain at rest: its bus just powered, the master's coupling caps charged. */
void emu_sto_reset(int i)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  world_sto_init(&s.sto[i]);
}

/** Board `i`'s STO chain on `dt` s. in: the master's pilot, amplitude V and Hz, the far end's
    100 kHz common mode (V), the link (V), +5 (AFE_ON), PA10 at the step's start; `edges` PA10's
    edges within the step at `at`, s from its start (NULL: spread); out: Cinj, Clevel, RESET,
    FAULTOUT, +15V7 (V). */
void emu_sto_step(int i, float dt, const float *in, int edges, const float *at, float *out)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  const world_sto_in_t chain =
  {
    in[0], in[1], in[2], in[3], in[4] != 0.0f, in[5] != 0.0f,
    (edges > 0) ? (uint32_t)edges : 0U, at
  };
  world_sto_out_t pins;

  world_sto_step(&s.sto[i], &chain, dt, &pins);
  out[0] = pins.cinj;
  out[1] = pins.clevel;
  out[2] = pins.reset;
  out[3] = pins.faultout;
  out[4] = pins.vgate;
}

/** Board `i`'s analog front end: init, reset, seed draw, field write and pin read. */
void emu_afe_init(int i)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_afe_init(&s.afe[i]);
  }
}

void emu_afe_reset(int i)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_afe_reset(&s.afe[i]);
  }
}

void emu_afe_draw(int i, uint32_t seed)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_afe_draw(&s.afe[i], seed);
  }
}

void emu_afe_set(int i, int field, double value)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return;
  }
  world_afe_t *a = &s.afe[i];

  switch (field)
  {
    case 0:  a->powered = (value != 0.0); break;
    case 1:  a->amps[0] = value; break;
    case 2:  a->amps[1] = value; break;
    case 3:  a->amps[2] = value; break;
    case 4:  a->dc = value; break;
    case 5:  a->ntc_c = value; break;
    case 6:  a->die_c = value; break;
    case 7:  a->rail5 = value; break;
    case 8:  a->gate = value; break;
    case 9:  a->clevel = value; break;
    case 10: a->cinj = value; break;
    case 11: a->ref = value; break;
    case 12: a->v_per_a = value; break;
    case 13: a->zero_v = value; break;
    case 14: a->gain_sigma = value; world_afe_draw(a, a->seed); break;
    case 15: a->zero_sigma = value; world_afe_draw(a, a->seed); break;
    case 16: a->bus_sigma = value; world_afe_draw(a, a->seed); break;
    case 17: a->bus_top = value; break;
    case 18: a->bus_bottom = value; break;
    case 19: a->ntc_r25 = value; break;
    case 20: a->ntc_beta = value; break;
    case 21: a->ntc_fixed = value; break;
    case 22: a->r5_top = value; break;
    case 23: a->r5_bottom = value; break;
    case 24: a->gate_top = value; break;
    case 25: a->gate_bottom = value; break;
    case 26: a->die_at30 = value; break;
    case 27: a->die_per_k = value; break;
    case 28: a->noise = value; break;
    default: break;
  }
}

double emu_afe_volts(int i, int pin)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return 0.0;
  }
  return world_afe_volts(&s.afe[i], (world_pin_t)pin);
}

double emu_afe_die_volts(int i, double celsius)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return 0.0;
  }
  return world_afe_die_volts(&s.afe[i], celsius);
}

/** Board `i`'s A1335 on SPI4. */
void emu_a1335_open(int i)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_a1335_open(&s.a1335[i]);
  }
}

void emu_a1335_reset(int i)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_a1335_reset(&s.a1335[i]);
  }
}

void emu_a1335_select(int i, int level, int powered, double degrees, double celsius)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_a1335_select(&s.a1335[i], level != 0, powered != 0, degrees, celsius);
  }
}

uint8_t emu_a1335_transmit(int i, uint8_t word, int powered)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return 0U;
  }
  return world_a1335_transmit(&s.a1335[i], word, powered != 0);
}

void emu_a1335_gauss(int i, double gauss)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    s.a1335[i].gauss = gauss;
  }
}

/** Board `i`'s BNO085 on SPI2. */
void emu_bno085_open(int i)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_bno085_open(&s.bno085[i]);
  }
}

void emu_bno085_reset(int i)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_bno085_reset(&s.bno085[i]);
  }
}

void emu_bno085_pin(int i, int which, int level, int powered)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_bno085_pin(&s.bno085[i], (uint8_t)which, level != 0, powered != 0);
  }
}

uint8_t emu_bno085_transmit(int i, uint8_t byte)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return 0U;
  }
  return world_bno085_transmit(&s.bno085[i], byte);
}

int emu_bno085_interrupt(int i, int powered)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return 1;
  }
  return world_bno085_interrupt(&s.bno085[i], powered != 0) ? 1 : 0;
}

int emu_bno085_active(int i)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return 0;
  }
  return world_bno085_active(&s.bno085[i]) ? 1 : 0;
}

void emu_bno085_tick(int i, int powered)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_bno085_tick(&s.bno085[i], powered != 0);
  }
}

double emu_bno085_read(int i, int index)
{
  if ((i < 0) || (i >= (int)WORLD_MOTORS))
  {
    return 0.0;
  }
  return world_bno085_read(&s.bno085[i], index);
}

void emu_bno085_pipe(int i, int index, double value)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    world_bno085_pipe(&s.bno085[i], index, value);
  }
}

void emu_bno085_rates(int i, double roll, double pitch, double yaw, int piped)
{
  if ((i >= 0) && (i < (int)WORLD_MOTORS))
  {
    s.bno085[i].roll_rate = roll;
    s.bno085[i].pitch_rate = pitch;
    s.bno085[i].yaw_rate = yaw;
    s.bno085[i].piped = (piped != 0);
  }
}

