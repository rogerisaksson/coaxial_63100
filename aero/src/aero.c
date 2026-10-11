/** aero.c - a pass of the frame's core, its steps in table order (aero.h). No branch: a
    choice is a mask (aero_bits.h), a bound a least or a most, a loop its array's length. */
#include "aero_bits.h"

#include <math.h>

#define PI      3.14159265358979f
/** A target is held within this: the blend's arithmetic takes no infinity. */
#define MOST    1e6f
/** No bound on the four's lift, N. */
#define FREE    1e9f
/** A height's constraint is the lift's while the rotors' axis stands this much up. */
#define UPRIGHT 0.05f
/** A torque is short where the thrusts give this much less of it than asked, and N m. */
#define SHORT   1e-3f
/** The lift's weight in the share at the least, of its own: a row of none has no curvature. */
#define FAINT   0.01f

static const float UP[AERO_AXES] = {0.0f, 1.0f, 0.0f};

/** The payload's coming: the rows as they stand are what it blends from; its rows blended as
    far as its time has run, a target between two tracked ones, taken whole from a dropped
    one's side; each target's own rate. */
static void inject(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const uint32_t came = set(p->sequence_id ^ s->sequence), swings = set(p->cpg_enabled);
  const float    fresh = one(came), on = one(swings);
  const float    span = 1e-3f * (float)p->blend_ms;
  const float    dt = most(in->dt, TINY);
  float          share;

  (void)out;
  s->since = (1.0f - fresh) * s->since + dt;
  share = least(1.0f, s->since / most(span, dt));
  s->moved = share - (1.0f - fresh) * s->share;
  s->share = share;
  for (int i = 0; i < (int)AERO_MODES; i++)
  {
    const uint32_t is = known(p->target[i]);
    float          wf, wt;

    for (int k = 0; k < (int)AERO_ROWS; k++)
    {
      s->from[k][i] += fresh * (s->now[k][i] - s->from[k][i]);
    }
    s->to[AERO_TARGET][i] = least(MOST, most(-MOST, pick(is, 0.0f, p->target[i])));
    s->to[AERO_WEIGHT][i] = one(is);
    s->to[AERO_STIFF][i] = clean(p->stiffness[i]);
    s->to[AERO_DAMP][i] = clean(p->damping[i]);
    s->to[AERO_SWING][i] = on * clean(p->cpg_amplitude[i]);
    for (int k = 0; k < (int)AERO_ROWS; k++)
    {
      s->now[k][i] = s->from[k][i] + share * (s->to[k][i] - s->from[k][i]);
    }
    wf = s->from[AERO_WEIGHT][i] * (1.0f - share);
    wt = s->to[AERO_WEIGHT][i] * share;
    s->now[AERO_TARGET][i] = (wf * s->from[AERO_TARGET][i] + wt * s->to[AERO_TARGET][i])
                             / most(wf + wt, TINY);
    s->rate[i] = s->from[AERO_WEIGHT][i] * s->to[AERO_WEIGHT][i]
                 * (s->to[AERO_TARGET][i] - s->from[AERO_TARGET][i]) * s->moved / dt;
  }
  s->base += fresh * (in->height - s->base);
  s->kick = came & swings & ~s->swinging;
  s->sequence = p->sequence_id;
  s->swinging = swings;
}

/** The oscillator a pass on, a point on the circle turned by Cayley's map - no sine taken -
    from (1, 0) where it began; each target with its amplitude of it, and its rate. */
static void swing(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const float dt = most(in->dt, TINY);
  const float half = PI * clean(p->cpg_hz) * dt;
  const float c0 = pick(s->kick, s->wave[0], 1.0f), s0 = pick(s->kick, s->wave[1], 0.0f);
  const float c = ((1.0f - half * half) * c0 - 2.0f * half * s0) / (1.0f + half * half);
  const float w = (2.0f * half * c0 + (1.0f - half * half) * s0) / (1.0f + half * half);
  const float trim = 1.5f - 0.5f * (c * c + w * w);

  (void)out;
  s->wave[0] = c * trim;
  s->wave[1] = w * trim;
  for (int i = 0; i < (int)AERO_MODES; i++)
  {
    s->ask[i] = s->now[AERO_TARGET][i] + s->now[AERO_SWING][i] * s->wave[1];
    s->rate[i] += s->now[AERO_SWING][i] * (2.0f * half / dt) * s->wave[0];
  }
}

/** The observers a pass on: the frame's rates and its speed against what the rotors' thrusts
    as given, its own inertia (spin x J spin) and gravity account for; what is left is theirs
    - the air, the rotors' precession, a weak rotor, the floor. Begun on the first reading. */
static void observe(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const float dt = in->dt, wt = s->p.wo_turn, wp = s->p.wo_pull;
  const float *n = s->p.inertia;
  float       lift = 0.0f;
  float       about[AERO_AXES] = {-(n[2] - n[1]) * in->spin[1] * in->spin[2],
                                  -(n[0] - n[2]) * in->spin[2] * in->spin[0],
                                  -(n[1] - n[0]) * in->spin[0] * in->spin[1]};

  (void)p;
  (void)out;
  for (int i = 0; i < (int)AERO_ROTORS; i++)
  {
    lift += in->given[i];
    about[0] -= in->given[i] * s->p.at_z[i];
    about[1] += in->given[i] * s->p.spin[i] * s->p.kappa;
    about[2] += in->given[i] * s->p.at_x[i];
  }
  for (int a = 0; a < (int)AERO_AXES; a++)
  {
    const float spin = pick(s->begun, in->spin[a], s->spin_hat[a]);
    const float vel = pick(s->begun, in->vel[a], s->vel_hat[a]);
    const float e = in->spin[a] - spin, v = in->vel[a] - vel;

    s->spin_hat[a] = spin + dt * (about[a] / n[a] + s->turn_hat[a] + 2.0f * wt * e);
    s->turn_hat[a] += dt * wt * wt * e;
    s->vel_hat[a] = vel + dt * (lift / s->p.mass * in->r[3 * a + 1] - UP[a] * s->p.gravity
                                + s->push_hat[a] + 2.0f * wp * v);
    s->push_hat[a] += dt * wp * wp * v;
  }
  s->begun = ~0U;
}

/** The lift inside its task constraints: a height held is come to at `rise` from either
    side, a height risen from the payload's coming is a ceiling; neither while the rotors'
    axis lies flat or under. Then as much of it as the axis points where it is asked: none
    while it points away. */
static void pull(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const uint32_t can = lt(UPRIGHT, in->r[4]);
  const uint32_t holds = can & set((p->constraints >> AERO_MAINTAIN) & 1U)
                         & known(p->constraint[AERO_MAINTAIN]);
  const uint32_t caps = can & set((p->constraints >> AERO_ASCENT) & 1U)
                        & known(p->constraint[AERO_ASCENT]);
  const float    rise = s->p.rise, each = s->p.mass / most(in->r[4], UPRIGHT);
  const float    free = s->p.gravity - s->push_hat[1] - 2.0f * rise * in->vel[1];
  const float    held = each * (free - rise * rise
                                * (in->height - clean(p->constraint[AERO_MAINTAIN])));
  const float    capped = each * (free - rise * rise
                                  * (in->height - s->base - clean(p->constraint[AERO_ASCENT])));

  (void)out;
  s->lift = most(s->lift, pick(holds, 0.0f, held));
  s->lift = s->aim * least(s->lift, least(pick(holds, FREE, held), pick(caps, FREE, capped)));
}

/** The thrusts for the lift and the torques inside each rotor's reach (`aero_share`), each
    row asked no further than that reach takes it alone - past it an ask says a way only, and
    a row out of reach took the share from the rows in it; the wrench they give, and where
    it is short of the one asked. */
static void share(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const float asked[AERO_WRENCH] = {s->lift, s->torque[0], s->torque[1], s->torque[2]};
  /* the lift weighs as far as the axis points where it is asked: pointed away, how much is
     no matter, and what the tilt leaves free is the heading's */
  const float weigh[AERO_WRENCH] = {s->p.weigh[0] * most(s->aim, FAINT), s->p.weigh[1],
                                    s->p.weigh[2], s->p.weigh[3]};
  float       low[AERO_WRENCH] = {0.0f, 0.0f, 0.0f, 0.0f};
  float       high[AERO_WRENCH] = {0.0f, 0.0f, 0.0f, 0.0f};
  double      want[AERO_WRENCH], f[AERO_ROTORS];

  (void)p;
  for (int i = 0; i < (int)AERO_ROTORS; i++)
  {
    const float row[AERO_WRENCH] = {1.0f, -s->p.at_z[i], s->p.spin[i] * s->p.kappa,
                                    s->p.at_x[i]};

    for (int k = 0; k < (int)AERO_WRENCH; k++)
    {
      const uint32_t up = lt(0.0f, row[k]);

      high[k] += row[k] * pick(up, in->lo[i], in->hi[i]);
      low[k] += row[k] * pick(up, in->hi[i], in->lo[i]);
    }
  }
  for (int k = 0; k < (int)AERO_WRENCH; k++)
  {
    want[k] = (double)least(high[k], most(low[k], asked[k]));
    out->asked[k] = asked[k];
    out->gave[k] = 0.0f;
  }
  aero_share(s, weigh, want, in->lo, in->hi, f, &out->bound);
  for (int i = 0; i < (int)AERO_ROTORS; i++)
  {
    out->thrust[i] = (float)f[i];
    out->gave[0] += out->thrust[i];
    out->gave[1] -= out->thrust[i] * s->p.at_z[i];
    out->gave[2] += out->thrust[i] * s->p.spin[i] * s->p.kappa;
    out->gave[3] += out->thrust[i] * s->p.at_x[i];
  }
  for (int a = 0; a < (int)AERO_AXES; a++)
  {
    s->held[a] = one(lt(SHORT * (fabsf(asked[a + 1]) + 1.0f),
                        fabsf(out->gave[a + 1] - asked[a + 1])));
    out->up[a] = s->up[a];
    out->miss[a] = s->miss[a];
    out->turned[a] = s->turn_hat[a];
    out->pushed[a] = s->push_hat[a];
  }
}

typedef void (*step_t)(aero_t *, const aero_payload_t *, const aero_in_t *, aero_out_t *);

/** A pass, in order. */
static const step_t STEPS[] = {inject, swing, observe, aero_refer, aero_turn, pull, share};

void aero_init(aero_t *s, const aero_params_t *p)
{
  memset(s, 0, sizeof(*s));
  s->p = *p;
  s->sequence = 0xFFFFFFFFU;
  s->wave[0] = 1.0f;
}

void aero_step(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  for (int k = 0; k < (int)(sizeof(STEPS) / sizeof(STEPS[0])); k++)
  {
    STEPS[k](s, p, in, out);
  }
}
