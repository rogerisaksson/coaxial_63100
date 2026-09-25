/** world.c - The loads a machine's motors turn and the body they carry, stepped with them. */
#include "world.h"

#include <math.h>
#include <string.h>

#define PI_F      3.14159265f
#define TWO_PI_F  6.2831853f

/* The body's integration step, s: a lift's height and a vehicle's distance change slowly
   against a PWM period, so they are stepped at most this long at a time. */
#define WORLD_STEP_S 0.001

static float sign_of(float x)
{
  return (x > 0.0f) ? 1.0f : ((x < 0.0f) ? -1.0f : 0.0f);
}

static float gear_of(const world_load_t *l)
{
  return (l->gear > 0.0f) ? l->gear : 1.0f;
}

/* The wheels sharing the vehicle. */
static uint8_t wheels(const world_t *w)
{
  uint8_t n = 0U;

  for (uint8_t i = 0U; i < w->motors; i++)
  {
    n = (uint8_t)(n + ((w->load[i].kind == WORLD_WHEEL) ? 1U : 0U));
  }
  return n;
}

/* The rider's push now: steady, or a kick for `duty` of every `period`. */
static float rider(const world_t *w)
{
  const world_body_t *b = &w->body;

  if (b->period <= 0.0f)
  {
    return b->rider;
  }
  const float into = (float)fmod(w->t, (double)b->period);

  return (into < b->duty * b->period) ? b->rider : 0.0f;
}

/* What holds the vehicle back at `v`, N: the slope, the rolling, the air, less the rider. */
static float resistance(const world_t *w, float v)
{
  const world_body_t *b = &w->body;

  return b->mass * b->gravity * (sinf(b->slope) + b->crr * cosf(b->slope) * sign_of(v))
         + 0.5f * b->rho * b->cda * v * fabsf(v) - rider(w);
}

void world_init(world_t *w)
{
  w->t = 0.0;
  memset(w->shaft, 0, sizeof(w->shaft));
  memset(w->speed, 0, sizeof(w->speed));
  w->height = w->climb = 0.0f;
  w->velocity = w->distance = 0.0f;
}

void world_load(const world_t *w, uint8_t i, float *torque, float *inertia)
{
  const world_load_t *l = &w->load[i];
  const float g = gear_of(l);
  const float wm = w->speed[i];

  *torque = 0.0f;
  *inertia = 0.0f;
  switch (l->kind)
  {
    case WORLD_JOINT:
    {
      /* Gravity turns the link back towards hanging; the gear divides it at the shaft. */
      const float q = l->angle + w->shaft[i] / g;

      *torque = (l->mass * w->body.gravity * l->arm * sinf(q) + l->damping * wm / g) / g;
      *inertia = l->inertia / (g * g);
      break;
    }
    case WORLD_ROTOR:
      *torque = l->k_drag * wm * fabsf(wm) + l->damping * wm;
      *inertia = l->inertia;
      break;
    case WORLD_WHEEL:
    {
      /* Rigid on the road: each wheel carries its share of the vehicle's resistance and of
         its mass, through its gear. */
      const float n = (float)wheels(w);
      const float v = wm / g * l->radius;

      *torque = resistance(w, v) * l->radius / (g * n) + l->damping * wm / (g * g);
      *inertia = (l->inertia + w->body.mass * l->radius * l->radius / n) / (g * g);
      break;
    }
    default:
      break;
  }
}

void world_motor(world_t *w, uint8_t i, float shaft, float speed)
{
  if (i < WORLD_MOTORS)
  {
    w->shaft[i] = shaft;
    w->speed[i] = speed;
  }
}

static float thrust(const world_t *w, uint8_t i)
{
  const world_load_t *l = &w->load[i];

  return (l->kind == WORLD_ROTOR) ? l->k_thrust * w->speed[i] * w->speed[i] : 0.0f;
}

static float vehicle_speed(const world_t *w)
{
  float sum = 0.0f;
  uint8_t n = 0U;

  for (uint8_t i = 0U; i < w->motors; i++)
  {
    if (w->load[i].kind == WORLD_WHEEL)
    {
      sum += w->speed[i] / gear_of(&w->load[i]) * w->load[i].radius;
      n++;
    }
  }
  return (n > 0U) ? sum / (float)n : 0.0f;
}

void world_advance(world_t *w, double t)
{
  while (w->t < t)
  {
    const double left = t - w->t;
    const float dt = (float)((left < WORLD_STEP_S) ? left : WORLD_STEP_S);

    if (w->body.kind == WORLD_LIFT)
    {
      float up = 0.0f;

      for (uint8_t i = 0U; i < w->motors; i++)
      {
        up += thrust(w, i);
      }
      const float a = up / w->body.mass - w->body.gravity;

      w->climb += a * dt;
      w->height += w->climb * dt;
      if (w->height <= 0.0f)
      {
        /* On the ground: it holds the weight until the thrust lifts it. */
        w->height = 0.0f;
        w->climb = (w->climb > 0.0f) ? w->climb : 0.0f;
      }
    }
    else if (w->body.kind == WORLD_VEHICLE)
    {
      w->velocity = vehicle_speed(w);
      w->distance += w->velocity * dt;
    }
    w->t += (double)dt;
  }
}

float world_output(const world_t *w, uint8_t i)
{
  const world_load_t *l = &w->load[i];

  switch (l->kind)
  {
    case WORLD_JOINT: return l->angle + w->shaft[i] / gear_of(l);
    case WORLD_ROTOR: return thrust(w, i);
    case WORLD_WHEEL: return w->speed[i] / gear_of(l) * l->radius;
    default:          return w->shaft[i];
  }
}

/* ---- a board's motor on its load ------------------------------------- */

/* A turn's difference folded into (-pi, pi]: how far the rotor went in one period. */
static float moved(float now, float was)
{
  float d = now - was;

  while (d > PI_F)
  {
    d -= TWO_PI_F;
  }
  while (d <= -PI_F)
  {
    d += TWO_PI_F;
  }
  return d;
}

void world_plant_init(world_plant_t *p, world_t *w, uint8_t index,
                      const drive_model_params_t *motor)
{
  memset(p, 0, sizeof(*p));
  p->motor.p = *motor;
  drive_model_init(&p->motor);
  p->world = w;
  p->index = index;
  p->j_motor = motor->j;
  p->theta_was = p->motor.theta;
  if (index >= w->motors)
  {
    w->motors = (uint8_t)(index + 1U);
  }
}

/* The gates off: the bridge open, no current, the rotor turning on under its load. */
static void coast(drive_model_t *m, float ts)
{
  const uint8_t sub = (m->p.sub == 0U) ? 1U : m->p.sub;
  const float dt = ts / (float)sub;

  m->id = m->iq = 0.0f;
  for (uint8_t k = 0U; k < sub; k++)
  {
    float wm = m->omega / m->p.pole_pairs;

    wm += (-m->p.b * wm - m->p.load) / m->p.j * dt;
    m->omega = wm * m->p.pole_pairs;
    m->theta += m->omega * dt;
  }
  m->theta = drive_wrap(m->theta);
}

void world_plant_step(world_plant_t *p, const float *duty, bool driven, float ts,
                      drive_sample_t *out)
{
  float torque;
  float inertia;

  world_load(p->world, p->index, &torque, &inertia);
  p->motor.p.load = torque;
  p->motor.p.j = p->j_motor + inertia;

  /* The shunts at the top of the period, then the period at what the firmware wrote. */
  drive_model_sample(&p->motor, out);
  if (driven)
  {
    drive_model_advance(&p->motor, duty, ts);
  }
  else
  {
    coast(&p->motor, ts);
  }

  p->shaft += moved(p->motor.theta, p->theta_was) / p->motor.p.pole_pairs;
  p->theta_was = p->motor.theta;
  world_motor(p->world, p->index, p->shaft, p->motor.omega / p->motor.p.pole_pairs);
}
