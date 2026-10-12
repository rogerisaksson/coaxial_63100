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
/** A wind is told from a push where the air's forces turn with it by this much of the
    push's bandwidth, 1/s a rad/s: under it, as much less of it. */
#define CALM    0.013f
/** The observers learn from this far over the floor, m. */
#define ALOFT   0.03f

static const float UP[AERO_AXES] = {0.0f, 1.0f, 0.0f};
/** The axis after an axis, and the one after that. */
static const uint8_t NEXT[AERO_AXES] = {1U, 2U, 0U}, AFTER[AERO_AXES] = {2U, 0U, 1U};

/** The attitude's modes: their targets are one point, blended where it is made. */
static const float ANGLE[AERO_MODES] = {0.0f, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f};

/** The payload's coming: the rows as they stand are what it blends from; its rows blended as
    far as its time has run, a target between two tracked ones, taken whole from a dropped
    one's side. */
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
    s->now[AERO_TARGET][i] += ANGLE[i] * (s->to[AERO_TARGET][i] - s->now[AERO_TARGET][i]);
  }
  s->base += fresh * (in->height - s->base);
  s->came = came;
  s->kick = came & swings & ~s->swinging;
  s->sequence = p->sequence_id;
  s->swinging = swings;
}

/** The oscillator a pass on, a point on the circle turned by Cayley's map - no sine taken -
    from (1, 0) where it began; each target with its amplitude of it. */
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
  }
}

/** The observers a pass on: the frame's rates and its speed against what the rotors' thrusts
    as given, its own inertia (spin x J spin) and gravity account for; what is left is theirs
    - the air, the rotors' precession, a weak rotor, the floor. The speed's holds the air as
    a wind: the frame's drag on its speed through it, squared, and what the rotors lose to
    it along their axis - a thrust's root times that speed -, both the frame's own sizes;
    and a push beside them, the quick one. What the push holds is laid to the wind at
    `wo_wind`, as far as those two turn with a wind - their slope by it M = a (I + u u^T) +
    g n n^T, the wind (M M + r I)^-1 M push: damped least squares, none where no air is -
    and the push gives up what that wind makes: through a bend the drag turns with the
    frame's speed, where a push alone trailed it. Begun on the first reading, and anew each
    pass on the floor: what holds the frame there is the floor's, and a push, a wind or a
    twist held from the air goes with it. */
static void observe(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const float dt = in->dt, wt = s->p.wo_turn, wp = s->p.wo_pull;
  const float *n = s->p.inertia;
  const float axis[AERO_AXES] = {in->r[1], in->r[4], in->r[7]};
  const float u[AERO_AXES] = {in->vel[0] - s->wind[0], in->vel[1] - s->wind[1],
                              in->vel[2] - s->wind[2]};
  const float fast = sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
  const float unit = 1.0f / most(fast, TINY);
  const float through = u[0] * axis[0] + u[1] * axis[1] + u[2] * axis[2];
  const uint32_t live = s->begun & lt(ALOFT, in->height);
  const float floored = 1.0f - one(lt(ALOFT, in->height));
  float       lift = 0.0f, roots = 0.0f, alpha, gamma, det;
  float       m[AERO_AXES][AERO_AXES], sq[AERO_AXES][AERO_AXES], y[AERO_AXES], dw[AERO_AXES];
  float       about[AERO_AXES] = {-(n[2] - n[1]) * in->spin[1] * in->spin[2],
                                  -(n[0] - n[2]) * in->spin[2] * in->spin[0],
                                  -(n[1] - n[0]) * in->spin[0] * in->spin[1]};

  (void)p;
  (void)out;
  for (int i = 0; i < (int)AERO_ROTORS; i++)
  {
    lift += in->given[i];
    roots += sqrtf(most(in->given[i], 0.0f));
    about[0] -= in->given[i] * s->p.at_z[i];
    about[1] += in->given[i] * s->p.spin[i] * s->p.kappa;
    about[2] += in->given[i] * s->p.at_x[i];
  }
  alpha = s->p.drag * fast;
  gamma = s->p.inflow * roots / s->p.mass;
  for (int a = 0; a < (int)AERO_AXES; a++)
  {
    const float spin = pick(live, in->spin[a], s->spin_hat[a]);
    const float vel = pick(live, in->vel[a], s->vel_hat[a]);
    const float e = in->spin[a] - spin, off = in->vel[a] - vel;

    s->spin_hat[a] = spin + dt * (about[a] / n[a] + s->turn_hat[a] + 2.0f * wt * e);
    s->turn_hat[a] += dt * wt * (wt * e - floored * s->turn_hat[a]);
    s->vel_hat[a] = vel + dt * (lift / s->p.mass * axis[a] - UP[a] * s->p.gravity
                                + s->push_hat[a] - alpha * u[a] - gamma * through * axis[a]
                                + 2.0f * wp * off);
    s->push_hat[a] += dt * wp * (wp * off - floored * s->push_hat[a]);
    s->air[a] = s->push_hat[a] - alpha * u[a];
    s->flow[a] = u[a];
  }
  /* the wind that makes the push: M, M push, M M + r I and that by its cofactors */
  for (int a = 0; a < (int)AERO_AXES; a++)
  {
    y[a] = 0.0f;
    for (int b = 0; b < (int)AERO_AXES; b++)
    {
      m[a][b] = alpha * u[a] * unit * u[b] * unit + gamma * axis[a] * axis[b];
    }
    m[a][a] += alpha;
  }
  for (int a = 0; a < (int)AERO_AXES; a++)
  {
    for (int b = 0; b < (int)AERO_AXES; b++)
    {
      y[a] += m[a][b] * s->push_hat[b];
      sq[a][b] = m[a][0] * m[0][b] + m[a][1] * m[1][b] + m[a][2] * m[2][b];
    }
    sq[a][a] += CALM * CALM * wp * wp;
  }
  for (int a = 0; a < (int)AERO_AXES; a++)
  {
    dw[a] = 0.0f;
    for (int b = 0; b < (int)AERO_AXES; b++)
    {
      const float co = sq[NEXT[b]][NEXT[a]] * sq[AFTER[b]][AFTER[a]]
                       - sq[NEXT[b]][AFTER[a]] * sq[AFTER[b]][NEXT[a]];

      dw[a] += co * y[b];
    }
  }
  det = sq[0][0] * (sq[1][1] * sq[2][2] - sq[1][2] * sq[2][1])
        - sq[0][1] * (sq[1][0] * sq[2][2] - sq[1][2] * sq[2][0])
        + sq[0][2] * (sq[1][0] * sq[2][1] - sq[1][1] * sq[2][0]);
  for (int a = 0; a < (int)AERO_AXES; a++)
  {
    dw[a] *= (1.0f - floored) * dt * s->p.wo_wind / most(det, TINY);
  }
  for (int a = 0; a < (int)AERO_AXES; a++)
  {
    s->push_hat[a] -= m[a][0] * dw[0] + m[a][1] * dw[1] + m[a][2] * dw[2];
    s->wind[a] += dw[a] - dt * wp * floored * s->wind[a];
  }
  s->lost = gamma;
  s->begun = ~0U;
}

/** The lift inside its task constraints: a height held is come to at `rise` from either
    side, a height risen from the payload's coming is a ceiling; neither while the rotors'
    axis lies flat or under. Then as much of it as the axis points where it is asked: none
    while it points away; and what the rotors lose to the air along their axis, whatever
    they are asked. */
static void pull(aero_t *s, const aero_payload_t *p, const aero_in_t *in, aero_out_t *out)
{
  const uint32_t can = lt(UPRIGHT, in->r[4]);
  const uint32_t holds = can & set((p->constraints >> AERO_MAINTAIN) & 1U)
                         & known(p->constraint[AERO_MAINTAIN]);
  const uint32_t caps = can & set((p->constraints >> AERO_ASCENT) & 1U)
                        & known(p->constraint[AERO_ASCENT]);
  const float    rise = s->p.rise, each = s->p.mass / most(in->r[4], UPRIGHT);
  const float    through = s->flow[0] * in->r[1] + s->flow[1] * in->r[4] + s->flow[2] * in->r[7];
  const float    free = s->p.gravity - s->air[1] - 2.0f * rise * in->vel[1];
  const float    held = each * (free - rise * rise
                                * (in->height - clean(p->constraint[AERO_MAINTAIN])));
  const float    capped = each * (free - rise * rise
                                  * (in->height - s->base - clean(p->constraint[AERO_ASCENT])));

  (void)out;
  s->lift = most(s->lift, pick(holds, 0.0f, held));
  s->lift = s->aim * least(s->lift, least(pick(holds, FREE, held), pick(caps, FREE, capped)))
            + s->now[AERO_WEIGHT][AERO_THRUST] * s->p.mass * s->lost * through;
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
  const float through = s->flow[0] * in->r[1] + s->flow[1] * in->r[4] + s->flow[2] * in->r[7];

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
  out->head[0] = s->head[0];
  out->head[1] = s->head[1];
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
    out->axis[a] = s->axis[a];
    out->miss[a] = s->miss[a];
    out->turned[a] = s->turn_hat[a];
    out->pushed[a] = s->air[a] - s->lost * through * in->r[3 * a + 1];
    out->wind[a] = s->wind[a];
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
