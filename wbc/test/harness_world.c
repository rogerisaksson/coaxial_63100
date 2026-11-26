/** harness_world.c - the robot's world's steps (wbc_world.c) through ctypes beside harness.c: tools.cores.world's
    calls; a stage of MuJoCo's step on a thread of the host's beside its law (machine.world.steps.ahead). */
#ifndef _WIN32
#define _GNU_SOURCE
#endif

#include "wbc_world.h"
#include "wbc_gmo.h"

#include <string.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define API __declspec(dllexport)
#else
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#define API
#endif

#define SPIN_AWAKE 16384
#define SPIN_JOIN  8192

#ifdef _WIN32
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
  return rest;
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
  const DWORD_PTR own = (cpu < (DWORD)(8U * sizeof(DWORD_PTR))) ? ((DWORD_PTR)1U << cpu) : (DWORD_PTR)0U;
  const DWORD_PTR pair = (cpu < (DWORD)(8U * sizeof(DWORD_PTR)))
                             ? ((DWORD_PTR)3U << (cpu & ~(DWORD)1U))
                             : (DWORD_PTR)0U;

  return ((proc_mask & ~pair) != (DWORD_PTR)0U) ? (proc_mask & ~pair) : (proc_mask & ~own);
}
#else
static inline void cpu_relax_once(void)
{
#if defined(__x86_64__) || defined(__i386__)
  __builtin_ia32_pause();
#elif defined(__aarch64__) || defined(__arm__)
  __asm__ volatile("yield" ::: "memory");
#else
  __asm__ volatile("" ::: "memory");
#endif
}

static int posix_multi_core(void)
{
#if defined(__linux__)
  cpu_set_t set;

  if (sched_getaffinity(0, sizeof(set), &set) == 0)
  {
    return CPU_COUNT(&set) > 1;
  }
#endif
  return sysconf(_SC_NPROCESSORS_ONLN) > 1;
}
#endif

/** A stage of MuJoCo's step (mj_step1): its model, its data. */
typedef void (*wbh_stage_t)(const void *model, void *data);

/* the stage asked of the thread beside, what it is asked on, whether one is asked and not yet joined */
static struct
{
  wbh_stage_t stage;
  const void *model;
  void       *data;
  int         pending;
  int         started;
  int         multi;
  int         parked;
  int         state;
#ifdef _WIN32
  HANDLE asked;
  HANDLE done;
#else
  pthread_mutex_t lock;
  pthread_cond_t  turn;
#endif
} beside;

#ifdef _WIN32
static DWORD WINAPI besides(LPVOID none)
{
  (void)none;
  while (beside.started != 0)
  {
    int st = __atomic_load_n(&beside.state, __ATOMIC_ACQUIRE);

    if ((beside.multi != 0) && (st != 1) && (st != 3))
    {
      for (int spin = 0; spin < SPIN_AWAKE; spin++)
      {
        YieldProcessor();
        st = __atomic_load_n(&beside.state, __ATOMIC_ACQUIRE);
        if ((st == 1) || (st == 3))
        {
          break;
        }
      }
    }
    if ((st != 1) && (st != 3))
    {
      __atomic_store_n(&beside.parked, 1, __ATOMIC_SEQ_CST);
      st = __atomic_load_n(&beside.state, __ATOMIC_SEQ_CST);
      if ((st != 1) && (st != 3))
      {
        (void)WaitForSingleObject(beside.asked, INFINITE);
      }
      __atomic_store_n(&beside.parked, 0, __ATOMIC_RELAXED);
      st = __atomic_load_n(&beside.state, __ATOMIC_ACQUIRE);
      if ((st != 1) && (st != 3))
      {
        continue;
      }
    }
    beside.stage(beside.model, beside.data);
    {
      int expected = 1;

      if (!__atomic_compare_exchange_n(&beside.state, &expected, 2, 0, __ATOMIC_RELEASE, __ATOMIC_ACQUIRE))
      {
        __atomic_store_n(&beside.state, 2, __ATOMIC_RELEASE);
        (void)SetEvent(beside.done);
      }
    }
  }
  return 0;
}
#else
static void *besides(void *none)
{
  while (beside.started != 0)
  {
    (void)pthread_mutex_lock(&beside.lock);
    while (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 1)
    {
      (void)pthread_cond_wait(&beside.turn, &beside.lock);
    }
    (void)pthread_mutex_unlock(&beside.lock);
    beside.stage(beside.model, beside.data);
    (void)pthread_mutex_lock(&beside.lock);
    __atomic_store_n(&beside.state, 2, __ATOMIC_RELEASE);
    (void)pthread_cond_broadcast(&beside.turn);
    (void)pthread_mutex_unlock(&beside.lock);
  }
  return none;
}
#endif

/** The stage asked last joined: waited for where it still runs; 1 where one was asked, else 0. */
API int wbh_world_behind(void)
{
  const int was = beside.pending;

  if (was != 0)
  {
#ifdef _WIN32
    if (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2)
    {
      int expected = 1;

      if (beside.multi != 0)
      {
        for (int spin = 0; (spin < SPIN_JOIN) && (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2); spin++)
        {
          YieldProcessor();
        }
      }
      if ((__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2)
          && __atomic_compare_exchange_n(&beside.state, &expected, 3, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
      {
        (void)WaitForSingleObject(beside.done, INFINITE);
      }
    }
#else
    if (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2)
    {
      if (beside.multi != 0)
      {
        for (int spin = 0; (spin < 64) && (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2); spin++)
        {
          cpu_relax_once();
        }
      }
      if (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2)
      {
        (void)pthread_mutex_lock(&beside.lock);
        while (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2)
        {
          (void)pthread_cond_wait(&beside.turn, &beside.lock);
        }
        (void)pthread_mutex_unlock(&beside.lock);
      }
    }
#endif
    __atomic_store_n(&beside.state, 0, __ATOMIC_RELEASE);
    beside.pending = 0;
  }
  return was;
}

/** `stage` of `model` on `data` begun on the thread beside - started at its first -, one asked before joined first:
    1, or 0 where the host gives no thread (nothing begun). */
API int wbh_world_ahead(wbh_stage_t stage, const void *model, void *data)
{
  (void)wbh_world_behind();
  if (beside.started == 0)
  {
#ifdef _WIN32
    int             narrowed = 0;
    const DWORD_PTR rest = neighbour_mask(&narrowed);
    HANDLE          thread = NULL;

    beside.multi = (rest != 0U);
    beside.parked = 1;
    beside.state = 0;
    beside.asked = CreateEventA(NULL, FALSE, FALSE, NULL);
    beside.done = CreateEventA(NULL, FALSE, FALSE, NULL);
    beside.started = 1;     /* before its thread reads it */
    if ((beside.asked != NULL) && (beside.done != NULL))
    {
      thread = CreateThread(NULL, 0U, besides, NULL, 0U, NULL);
    }
    if (thread == NULL)
    {
      beside.started = -1;
    }
    else
    {
      /* Where `placed` narrowed this process to a cache complex, hold the stage to its first neighbour core. */
      if (rest != 0U)
      {
        (void)SetThreadAffinityMask(thread, narrowed ? nth_core_mask(rest, 0) : rest);
      }
      (void)CloseHandle(thread);
    }
#else
    pthread_t thread;

    beside.multi = posix_multi_core();
    (void)pthread_mutex_init(&beside.lock, NULL);
    (void)pthread_cond_init(&beside.turn, NULL);
    beside.started = 1;     /* before its thread reads it */
    if (pthread_create(&thread, NULL, besides, NULL) != 0)
    {
      beside.started = -1;
    }
#endif
  }
  if (beside.started == 1)
  {
    beside.stage = stage;
    beside.model = model;
    beside.data = data;
    beside.pending = 1;
#ifdef _WIN32
    __atomic_store_n(&beside.state, 1, __ATOMIC_SEQ_CST);
    if (__atomic_load_n(&beside.parked, __ATOMIC_SEQ_CST) != 0)
    {
      (void)SetEvent(beside.asked);
    }
#else
    (void)pthread_mutex_lock(&beside.lock);
    __atomic_store_n(&beside.state, 1, __ATOMIC_RELEASE);
    (void)pthread_cond_broadcast(&beside.turn);
    (void)pthread_mutex_unlock(&beside.lock);
#endif
  }
  return beside.pending;
}

API int wbh_steps_size(void)
{
  return (int)sizeof(wbc_steps_t);
}

API int wbh_boxes_size(void)
{
  return (int)sizeof(wbc_boxes_t);
}

API void wbh_world_rated(wbc_steps_t *s, double span)
{
  wbc_world_rated(s, span);
}

API void wbh_world_drives(wbc_steps_t *s, double since)
{
  wbc_world_drives(s, since);
}

API void wbh_world_stuck_step(wbc_steps_t *s)
{
  wbc_world_stuck_step(s);
}

API void wbh_world_before(wbc_steps_t *s, double since)
{
  wbc_world_before(s, since);
}

API void wbh_world_contacts_after(wbc_steps_t *s)
{
  wbc_world_contacts_after(s);
}

API void wbh_world_rest_after(wbc_steps_t *s)
{
  wbc_world_rest_after(s);
}

API void wbh_world_after(wbc_steps_t *s)
{
  wbc_world_after(s);
}

API int wbh_world_paired(const wbc_steps_t *s, const unsigned char *a, const unsigned char *b, double below)
{
  return wbc_world_paired(s, a, b, below);
}

API int wbh_world_contacted(const struct mjData_ *d, wbc_contacts_t *c)
{
  return wbc_world_contacted(d, c);
}

API int wbh_held_size(void)
{
  return (int)sizeof(wbc_held_t);
}

API int wbh_world_held(struct mjData_ *d, const wbc_held_t *h)
{
  return wbc_world_held(d, h);
}

API int wbh_stuck_size(void)
{
  return (int)sizeof(wbc_stuck_t);
}

API void wbh_world_stuck(struct mjData_ *d, wbc_stuck_t *k)
{
  wbc_world_stuck(d, k);
}

API void wbh_world_slipped(const struct mjData_ *d, wbc_stuck_t *k)
{
  wbc_world_slipped(d, k);
}

static struct
{
  wbc_gmo_t  *o;
  wbc_frame_t f;
  double      q[WBC_N - 6];
  double      u[WBC_N];
  double      dt;
  int         pending;
  int         started;
  int         multi;
  int         parked;
  int         state;
#ifdef _WIN32
  HANDLE asked;
  HANDLE done;
#else
  pthread_mutex_t lock;
  pthread_cond_t  turn;
#endif
} gmo_beside;

#ifdef _WIN32
static DWORD WINAPI gmo_besides(LPVOID none)
{
  (void)none;
  while (gmo_beside.started != 0)
  {
    int st = __atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE);

    if ((gmo_beside.multi != 0) && (st != 1) && (st != 3))
    {
      for (int spin = 0; spin < SPIN_AWAKE; spin++)
      {
        YieldProcessor();
        st = __atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE);
        if ((st == 1) || (st == 3))
        {
          break;
        }
      }
    }
    if ((st != 1) && (st != 3))
    {
      __atomic_store_n(&gmo_beside.parked, 1, __ATOMIC_SEQ_CST);
      st = __atomic_load_n(&gmo_beside.state, __ATOMIC_SEQ_CST);
      if ((st != 1) && (st != 3))
      {
        (void)WaitForSingleObject(gmo_beside.asked, INFINITE);
      }
      __atomic_store_n(&gmo_beside.parked, 0, __ATOMIC_RELAXED);
      st = __atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE);
      if ((st != 1) && (st != 3))
      {
        continue;
      }
    }
    wbc_gmo_step(gmo_beside.o, &gmo_beside.f, gmo_beside.q, gmo_beside.u, gmo_beside.dt);
    {
      int expected = 1;

      if (!__atomic_compare_exchange_n(&gmo_beside.state, &expected, 2, 0, __ATOMIC_RELEASE, __ATOMIC_ACQUIRE))
      {
        __atomic_store_n(&gmo_beside.state, 2, __ATOMIC_RELEASE);
        (void)SetEvent(gmo_beside.done);
      }
    }
  }
  return 0;
}
#else
static void *gmo_besides(void *none)
{
  while (gmo_beside.started != 0)
  {
    (void)pthread_mutex_lock(&gmo_beside.lock);
    while (__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 1)
    {
      (void)pthread_cond_wait(&gmo_beside.turn, &gmo_beside.lock);
    }
    (void)pthread_mutex_unlock(&gmo_beside.lock);
    wbc_gmo_step(gmo_beside.o, &gmo_beside.f, gmo_beside.q, gmo_beside.u, gmo_beside.dt);
    (void)pthread_mutex_lock(&gmo_beside.lock);
    __atomic_store_n(&gmo_beside.state, 2, __ATOMIC_RELEASE);
    (void)pthread_cond_broadcast(&gmo_beside.turn);
    (void)pthread_mutex_unlock(&gmo_beside.lock);
  }
  return none;
}
#endif

API int wbh_gmo_behind(void)
{
  const int was = gmo_beside.pending;

  if (was != 0)
  {
#ifdef _WIN32
    if (__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 2)
    {
      int expected = 1;

      if (gmo_beside.multi != 0)
      {
        for (int spin = 0; (spin < SPIN_JOIN) && (__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 2); spin++)
        {
          YieldProcessor();
        }
      }
      if ((__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 2)
          && __atomic_compare_exchange_n(&gmo_beside.state, &expected, 3, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
      {
        (void)WaitForSingleObject(gmo_beside.done, INFINITE);
      }
    }
#else
    if (__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 2)
    {
      if (gmo_beside.multi != 0)
      {
        for (int spin = 0; (spin < 64) && (__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 2); spin++)
        {
          cpu_relax_once();
        }
      }
      if (__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 2)
      {
        (void)pthread_mutex_lock(&gmo_beside.lock);
        while (__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 2)
        {
          (void)pthread_cond_wait(&gmo_beside.turn, &gmo_beside.lock);
        }
        (void)pthread_mutex_unlock(&gmo_beside.lock);
      }
    }
#endif
    __atomic_store_n(&gmo_beside.state, 0, __ATOMIC_RELEASE);
    gmo_beside.pending = 0;
  }
  return was;
}

API int wbh_gmo_ahead(wbc_gmo_t *o, const double *base, const double *q, const double *u, double dt)
{
  (void)wbh_gmo_behind();
  if (gmo_beside.started == 0)
  {
#ifdef _WIN32
    int             narrowed = 0;
    const DWORD_PTR rest = neighbour_mask(&narrowed);

    if (rest == 0U)
    {
      gmo_beside.started = -1;
    }
    else
    {
      HANDLE thread = NULL;

      gmo_beside.multi = 1;
      gmo_beside.parked = 1;
      gmo_beside.state = 0;
      gmo_beside.asked = CreateEventA(NULL, FALSE, FALSE, NULL);
      gmo_beside.done = CreateEventA(NULL, FALSE, FALSE, NULL);
      gmo_beside.started = 1;
      if ((gmo_beside.asked != NULL) && (gmo_beside.done != NULL))
      {
        thread = CreateThread(NULL, 0U, gmo_besides, NULL, 0U, NULL);
      }
      if (thread == NULL)
      {
        gmo_beside.started = -1;
      }
      else
      {
        (void)SetThreadAffinityMask(thread, narrowed ? nth_core_mask(rest, 1) : rest);
        (void)CloseHandle(thread);
      }
    }
#else
    if (!posix_multi_core())
    {
      gmo_beside.started = -1;
    }
    else
    {
      pthread_t thread;

      gmo_beside.multi = 1;
      (void)pthread_mutex_init(&gmo_beside.lock, NULL);
      (void)pthread_cond_init(&gmo_beside.turn, NULL);
      gmo_beside.started = 1;
      if (pthread_create(&thread, NULL, gmo_besides, NULL) != 0)
      {
        gmo_beside.started = -1;
      }
    }
#endif
  }
  if (gmo_beside.started == 1)
  {
    gmo_beside.o = o;
    memcpy(gmo_beside.f.r, base, 9U * sizeof(double));
    memcpy(gmo_beside.f.p, base + 9, 3U * sizeof(double));
    memcpy(gmo_beside.q, q, (size_t)(WBC_N - 6) * sizeof(double));
    memcpy(gmo_beside.u, u, (size_t)WBC_N * sizeof(double));
    gmo_beside.dt = dt;
    gmo_beside.pending = 1;
#ifdef _WIN32
    __atomic_store_n(&gmo_beside.state, 1, __ATOMIC_SEQ_CST);
    if (__atomic_load_n(&gmo_beside.parked, __ATOMIC_SEQ_CST) != 0)
    {
      (void)SetEvent(gmo_beside.asked);
    }
#else
    (void)pthread_mutex_lock(&gmo_beside.lock);
    __atomic_store_n(&gmo_beside.state, 1, __ATOMIC_RELEASE);
    (void)pthread_cond_broadcast(&gmo_beside.turn);
    (void)pthread_mutex_unlock(&gmo_beside.lock);
#endif
  }
  return gmo_beside.pending;
}
