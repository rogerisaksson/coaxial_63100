/** world_emu.c - One world and a plant a board, flat, for the emulator's bridge to call. */

/* A process's world: board/emu/Coaxial63100_Plant.cs loads this library once per Renode and
   each emulated board steps its own plant on it, so a limb's boards share one body. Scalars
   in, scalars out: nothing for P/Invoke to marshal but floats. */
#include "world.h"

#include <string.h>

static struct
{
  world_t       world;
  world_plant_t plant[WORLD_MOTORS];
  double        t[WORLD_MOTORS];     /* each plant's own time, s */
  bool          attached[WORLD_MOTORS];
} s;

/** A fresh world of `motors` loads, all free, on the ground. */
void emu_world_reset(int motors)
{
  memset(&s, 0, sizeof(s));
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
