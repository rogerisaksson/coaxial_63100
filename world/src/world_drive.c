/** world_drive.c - A stand-in board's drive: the board's law and loop on the motor it models. */
#ifndef _WIN32
#define _GNU_SOURCE
#endif

#include "world_drive.h"
#include "board_units.h"

#include <math.h>
#include <string.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif

#define TURN_DEG 360.0
#define COUNTS   4096.0     /* a turn as the A1335 counts it */

static world_drive_t s[WORLD_DRIVES];

static world_drive_t *at(int i)
{
  return ((i >= 0) && (i < WORLD_DRIVES)) ? &s[i] : NULL;
}

void world_drive_init(int i, float ts)
{
  world_drive_t *b = at(i);

  if (b == NULL)
  {
    return;
  }
  memset(b, 0, sizeof(*b));
  b->speed_ask = -1.0f;
  drive_init(&b->law, ts);
  b->law.vdc = b->law.model.p.vdc;
}

drive_t *world_drive_law(int i)
{
  world_drive_t *b = at(i);

  return (b != NULL) ? &b->law : NULL;
}

ctrl_loop_t *world_drive_loop(int i)
{
  world_drive_t *b = at(i);

  return (b != NULL) ? &b->loop : NULL;
}

/** The shaft as the A1335 reads it: a turn in twelve bits, deg. */
static float shaft_deg(const world_drive_t *b)
{
  const double turns = b->shaft / TAU_D;

  return (float)(floor((turns - floor(turns)) * COUNTS) * (TURN_DEG / COUNTS));
}

void world_drive_rest(int i)
{
  world_drive_t *b = at(i);

  if (b != NULL)
  {
    drive_model_init(&b->law.model);
    b->theta_was = b->law.model.theta;
  }
}

/** A period with the converters' bridge open: the law told its MOE, the rotor coasting. */
static bool open_period(drive_t *d, bool moe, drive_out_t *out)
{
  drive_sample_t in;

  drive_model_sample(&d->model, &in);
  const bool trip = drive_step(d, &in, moe, out);

  drive_model_coast(&d->model, d->ts);
  memset(d->model.duty_prev, 0, sizeof(d->model.duty_prev));
  d->model.driven_prev = false;
  return trip;
}

uint32_t world_drive_run(int i, uint32_t periods, bool moe, bool supplied)
{
  world_drive_t *b = at(i);
  uint32_t tripped = 0U;

  if (b == NULL)
  {
    return 0U;
  }
  drive_t *d = &b->law;
  const bool closed = (moe && supplied) || (d->source == DRIVE_SOURCE_MODEL);
  drive_out_t out = { { 0.0f, 0.0f, 0.0f }, false };

  for (uint32_t k = 0U; k < periods; k++)
  {
    if (ctrl_loop_due(&b->loop))
    {
      ctrl_loop_step(&b->loop, d, shaft_deg(b));
    }
    const bool trip = closed ? drive_step_virtual(d, &out) : open_period(d, moe, &out);
    float turned = d->model.theta - b->theta_was;

    /* A period turns the rotor a fraction of a turn: the rest of a gap is its wrap. */
    if (fabsf(turned) > 3.0f)
    {
      turned -= TWO_PI_F * nearbyintf(turned / TWO_PI_F);
    }
    b->shaft += (double)(turned / d->model.p.pole_pairs);
    b->theta_was = d->model.theta;
    tripped += trip ? 1U : 0U;
    if (out.driven)
    {
      for (int p = 0; p < (int)DRIVE_PHASES; p++)
      {
        const double amps = (double)d->model.i_abc[p];

        b->squares[p] += amps * amps;
        b->linked += (double)out.duty[p] * amps;
      }
      b->counted++;
    }
  }
  if (periods > 0U)
  {
    memcpy(b->duty, out.duty, sizeof(b->duty));
    b->driven = out.driven;
  }
  b->trips += tripped;
  return tripped;
}

int world_drive_read(int i, double *out)
{
  const world_drive_t *b = at(i);
  int n = 0;

  if ((b == NULL) || (out == NULL))
  {
    return 0;
  }
  out[n++] = (double)b->law.periods;
  out[n++] = (double)b->trips;
  out[n++] = b->law.inj_valid ? 1.0 : 0.0;
  out[n++] = b->driven ? 1.0 : 0.0;
  for (int k = 0; k < (int)DRIVE_PHASES; k++)
  {
    out[n++] = (double)b->duty[k];
  }
  for (int k = 0; k < (int)DRIVE_PHASES; k++)
  {
    out[n++] = (double)b->law.model.i_abc[k];
  }
  out[n++] = b->shaft;
  return n;
}

int world_drive_window(int i, double *out)
{
  world_drive_t *b = at(i);
  int n = 0;

  if ((b == NULL) || (out == NULL))
  {
    return 0;
  }
  const double periods = (b->counted > 0U) ? (double)b->counted : 1.0;

  for (int p = 0; p < (int)DRIVE_PHASES; p++)
  {
    out[n++] = b->squares[p] / periods;
    b->squares[p] = 0.0;
  }
  out[n++] = b->linked / periods;
  out[n++] = (double)b->counted;
  b->linked = 0.0;
  b->counted = 0U;
  return n;
}

int world_drive_loop_read(int i, double *out)
{
  const world_drive_t *b = at(i);
  ctrl_loop_state_t st;
  int n = 0;

  if ((b == NULL) || (out == NULL))
  {
    return 0;
  }
  ctrl_loop_state(&b->loop, &st);
  out[n++] = st.running ? 1.0 : 0.0;
  out[n++] = st.playing ? 1.0 : 0.0;
  out[n++] = (double)st.measured;
  out[n++] = (double)st.command;
  out[n++] = (double)st.hz;
  out[n++] = (double)st.rows;
  out[n++] = (double)st.free;
  out[n++] = (double)st.queued_s;
  out[n++] = (double)st.played;
  out[n++] = (double)st.idle;
  out[n++] = (double)st.blind;
  out[n++] = (double)st.setpoint;
  out[n++] = (double)st.ref;
  out[n++] = (double)st.value;
  out[n++] = (double)st.estimate;
  out[n++] = (double)st.out;
  return n;
}

uint32_t world_drive_pass(int i, uint32_t periods, bool moe, bool supplied, float speed, double *out)
{
  world_drive_t *b = at(i);

  if (b == NULL)
  {
    return 0U;
  }
  if (!b->loop.r.playing && (b->loop.r.tail != b->loop.r.head))
  {
    b->speed_ask = b->loop.r.row[(b->loop.r.head + CTRL_ROWS - 1U) % CTRL_ROWS].setpoint;
  }
  if ((speed >= 0.0f) && (speed != b->speed_ask))
  {
    const float was = (b->speed_ask >= 0.0f) ? b->speed_ask : speed;
    const float sp[2] = { was + 0.5f * (speed - was), speed };
    const uint16_t ms[2] = { 1U, 1U };

    (void)ctrl_loop_rows(&b->loop, ms, sp, 2U);
    b->speed_ask = speed;
  }
  const uint32_t tripped = world_drive_run(i, periods, moe, supplied);

  if (out != NULL)
  {
    out[0] = (double)b->law.model.omega;
    out[1] = (double)b->law.omega_hat;
    out[2] = (double)b->law.model.id;
    out[3] = (double)b->law.model.iq;
    out[4] = (double)b->law.fault;
  }
  return tripped;
}

#define WORLD_DRIVE_WORKERS 3
#define DRIVE_SPIN_AWAKE    16384
#define DRIVE_SPIN_JOIN     8192

typedef struct
{
  int      slot;
  uint32_t periods;
  bool     moe;
  bool     supplied;
  float    speed;
  double  *out;
  uint32_t tripped;
  int      started;
  int      multi;
  int      parked;
  int      state;
#ifdef _WIN32
  HANDLE asked;
  HANDLE done;
#else
  pthread_mutex_t lock;
  pthread_cond_t  turn;
  pthread_cond_t  done;
#endif
} drive_worker_t;

static drive_worker_t workers[WORLD_DRIVE_WORKERS];

#ifdef _WIN32
static DWORD WINAPI drive_worker_loop(LPVOID arg)
{
  drive_worker_t *w = (drive_worker_t *)arg;

  while (w->started != 0)
  {
    int st = __atomic_load_n(&w->state, __ATOMIC_ACQUIRE);

    if ((w->multi != 0) && (st != 1) && (st != 3))
    {
      for (int spin = 0; spin < DRIVE_SPIN_AWAKE; spin++)
      {
        YieldProcessor();
        st = __atomic_load_n(&w->state, __ATOMIC_ACQUIRE);
        if ((st == 1) || (st == 3))
        {
          break;
        }
      }
    }
    if ((st != 1) && (st != 3))
    {
      __atomic_store_n(&w->parked, 1, __ATOMIC_SEQ_CST);
      st = __atomic_load_n(&w->state, __ATOMIC_SEQ_CST);
      if ((st != 1) && (st != 3))
      {
        (void)WaitForSingleObject(w->asked, INFINITE);
      }
      __atomic_store_n(&w->parked, 0, __ATOMIC_RELEASE);
      st = __atomic_load_n(&w->state, __ATOMIC_ACQUIRE);
      if ((st != 1) && (st != 3))
      {
        continue;
      }
    }
    w->tripped = world_drive_pass(w->slot, w->periods, w->moe, w->supplied, w->speed, w->out);
    {
      int expected = 1;

      if (!__atomic_compare_exchange_n(&w->state, &expected, 2, 0, __ATOMIC_RELEASE, __ATOMIC_ACQUIRE))
      {
        __atomic_store_n(&w->state, 2, __ATOMIC_RELEASE);
        (void)SetEvent(w->done);
      }
    }
  }
  return 0;
}

static DWORD_PTR nth_core_mask(DWORD_PTR rest, int idx)
{
  DWORD_PTR rem = rest;

  for (int c = 0; c < (int)(4U * sizeof(DWORD_PTR)); c++)
  {
    const DWORD_PTR pair = (DWORD_PTR)3U << (unsigned int)(2 * c);
    const DWORD_PTR hit = rem & pair;

    if (hit != (DWORD_PTR)0U)
    {
      if (idx == 0)
      {
        return hit;
      }
      idx--;
      rem &= ~hit;
    }
  }
  return (DWORD_PTR)0U;
}

static DWORD_PTR neighbour_mask(int *narrowed)
{
  DWORD_PTR proc_mask = 0U, sys_mask = 0U;

  *narrowed = 0;
  if (GetProcessAffinityMask(GetCurrentProcess(), &proc_mask, &sys_mask) == 0)
  {
    return 0U;
  }
  *narrowed = (proc_mask != sys_mask);
  const DWORD     cpu = GetCurrentProcessorNumber();
  const DWORD_PTR pair = (cpu < (DWORD)(8U * sizeof(DWORD_PTR)))
                             ? ((DWORD_PTR)3U << (cpu & ~(DWORD)1U))
                             : (DWORD_PTR)0U;

  return proc_mask & ~pair;
}
#else
static int posix_extra_cpus(void)
{
#if defined(__linux__)
  cpu_set_t set;

  if (sched_getaffinity(0, sizeof(set), &set) == 0)
  {
    const int n = CPU_COUNT(&set);

    return (n > 1) ? (n - 1) : 0;
  }
#endif
  const long n = sysconf(_SC_NPROCESSORS_ONLN);

  return (n > 1L) ? (int)(n - 1L) : 0;
}

static void *drive_worker_loop(void *arg)
{
  drive_worker_t *w = (drive_worker_t *)arg;

  while (w->started != 0)
  {
    (void)pthread_mutex_lock(&w->lock);
    while ((__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 1)
           && (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 3))
    {
      (void)pthread_cond_wait(&w->turn, &w->lock);
    }
    (void)pthread_mutex_unlock(&w->lock);
    w->tripped = world_drive_pass(w->slot, w->periods, w->moe, w->supplied, w->speed, w->out);
    {
      int expected = 1;

      if (!__atomic_compare_exchange_n(&w->state, &expected, 2, 0, __ATOMIC_RELEASE, __ATOMIC_ACQUIRE))
      {
        (void)pthread_mutex_lock(&w->lock);
        __atomic_store_n(&w->state, 2, __ATOMIC_RELEASE);
        (void)pthread_cond_signal(&w->done);
        (void)pthread_mutex_unlock(&w->lock);
      }
    }
  }
  return NULL;
}
#endif

static int ensure_drive_worker(int idx)
{
  drive_worker_t *w = &workers[idx];

  if (w->started == 0)
  {
#ifdef _WIN32
    int             narrowed = 0;
    const DWORD_PTR rest = neighbour_mask(&narrowed);
    const DWORD_PTR target = (rest != 0U) ? nth_core_mask(rest, idx) : (DWORD_PTR)0U;

    if (target == (DWORD_PTR)0U)
    {
      w->started = -1;
    }
    else
    {
      HANDLE thread = NULL;

      w->multi = 1;
      w->parked = 1;
      w->state = 0;
      w->asked = CreateEventA(NULL, FALSE, FALSE, NULL);
      w->done = CreateEventA(NULL, FALSE, FALSE, NULL);
      w->started = 1;
      if ((w->asked != NULL) && (w->done != NULL))
      {
        thread = CreateThread(NULL, 0U, drive_worker_loop, w, 0U, NULL);
      }
      if (thread == NULL)
      {
        w->started = -1;
      }
      else
      {
        (void)SetThreadAffinityMask(thread, narrowed ? target : rest);
        (void)CloseHandle(thread);
      }
    }
#else
    if (idx >= posix_extra_cpus())
    {
      w->started = -1;
    }
    else
    {
      pthread_t thread;

      w->multi = 1;
      (void)pthread_mutex_init(&w->lock, NULL);
      (void)pthread_cond_init(&w->turn, NULL);
      (void)pthread_cond_init(&w->done, NULL);
      w->started = 1;
      if (pthread_create(&thread, NULL, drive_worker_loop, w) != 0)
      {
        w->started = -1;
      }
      else
      {
        (void)pthread_detach(thread);
      }
    }
#endif
  }
  return w->started == 1;
}

void world_drives_pass(int count, const int *slots, uint32_t periods, const uint8_t *moe,
                       const uint8_t *supplied, const float *speeds, double *out,
                       uint32_t *tripped)
{
  int distinct = (count > 1) && (count <= (WORLD_DRIVE_WORKERS + 1));

  if (count <= 0)
  {
    return;
  }
  for (int a = 0; distinct && (a < count); a++)
  {
    for (int b = a + 1; b < count; b++)
    {
      if (slots[a] == slots[b])
      {
        distinct = 0;
      }
    }
  }
  if (!distinct)
  {
    for (int r = 0; r < count; r++)
    {
      const uint32_t tr = world_drive_pass(slots[r], periods, moe[r] != 0U, supplied[r] != 0U,
                                           speeds[r], (out != NULL) ? &out[5 * r] : NULL);

      if (tripped != NULL)
      {
        tripped[r] = tr;
      }
    }
    return;
  }
  int launched = 0;

  for (int r = 1; r < count; r++)
  {
    drive_worker_t *w = &workers[r - 1];

    if (!ensure_drive_worker(r - 1))
    {
      break;
    }
    w->slot = slots[r];
    w->periods = periods;
    w->moe = moe[r] != 0U;
    w->supplied = supplied[r] != 0U;
    w->speed = speeds[r];
    w->out = (out != NULL) ? &out[5 * r] : NULL;
#ifdef _WIN32
    __atomic_store_n(&w->state, 1, __ATOMIC_SEQ_CST);
    if (__atomic_load_n(&w->parked, __ATOMIC_SEQ_CST) != 0)
    {
      (void)SetEvent(w->asked);
    }
#else
    (void)pthread_mutex_lock(&w->lock);
    __atomic_store_n(&w->state, 1, __ATOMIC_RELEASE);
    (void)pthread_cond_signal(&w->turn);
    (void)pthread_mutex_unlock(&w->lock);
#endif
    launched++;
  }
  const uint32_t tr0 = world_drive_pass(slots[0], periods, moe[0] != 0U, supplied[0] != 0U,
                                        speeds[0], out);

  if (tripped != NULL)
  {
    tripped[0] = tr0;
  }
  for (int r = 1 + launched; r < count; r++)
  {
    const uint32_t tr = world_drive_pass(slots[r], periods, moe[r] != 0U, supplied[r] != 0U,
                                         speeds[r], (out != NULL) ? &out[5 * r] : NULL);

    if (tripped != NULL)
    {
      tripped[r] = tr;
    }
  }
  for (int idx = 0; idx < launched; idx++)
  {
    drive_worker_t *w = &workers[idx];

#ifdef _WIN32
    if (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2)
    {
      int expected = 1;

      if (w->multi != 0)
      {
        for (int spin = 0; (spin < DRIVE_SPIN_JOIN) && (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2); spin++)
        {
          YieldProcessor();
        }
      }
      if ((__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2)
          && __atomic_compare_exchange_n(&w->state, &expected, 3, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
      {
        (void)WaitForSingleObject(w->done, INFINITE);
      }
    }
#else
    if (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2)
    {
      for (int spin = 0; (spin < 256) && (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2); spin++)
      {
        (void)sched_yield();
      }
      if (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2)
      {
        int expected = 1;

        (void)pthread_mutex_lock(&w->lock);
        if (__atomic_compare_exchange_n(&w->state, &expected, 3, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
        {
          while (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2)
          {
            (void)pthread_cond_wait(&w->done, &w->lock);
          }
        }
        (void)pthread_mutex_unlock(&w->lock);
      }
    }
#endif
    __atomic_store_n(&w->state, 0, __ATOMIC_RELEASE);
    if (tripped != NULL)
    {
      tripped[idx + 1] = w->tripped;
    }
  }
}

