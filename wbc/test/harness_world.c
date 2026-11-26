/** harness_world.c - the robot's world's steps (wbc_world.c) through ctypes beside harness.c: tools.cores.world's
    calls; a stage of MuJoCo's step on a thread of the host's beside its law (machine.world.steps.ahead). */
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
#define API
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
    int expected = 1;

    (void)WaitForSingleObject(beside.asked, INFINITE);
    beside.stage(beside.model, beside.data);
    if (!__atomic_compare_exchange_n(&beside.state, &expected, 2, 0, __ATOMIC_RELEASE, __ATOMIC_ACQUIRE))
    {
      __atomic_store_n(&beside.state, 2, __ATOMIC_RELEASE);
      (void)SetEvent(beside.done);
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

      for (int spin = 0; (spin < 128) && (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2); spin++)
      {
        YieldProcessor();
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
      for (int spin = 0; (spin < 128) && (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2); spin++)
      {
        (void)sched_yield();
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
    __atomic_store_n(&beside.state, 0, __ATOMIC_RELAXED);
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
    HANDLE thread = NULL;

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
      DWORD_PTR proc_mask = 0U, sys_mask = 0U;

      /* Where `placed` narrowed this process to a cache complex, keep the stage off the caller's physical core. */
      if ((GetProcessAffinityMask(GetCurrentProcess(), &proc_mask, &sys_mask) != 0) && (proc_mask != sys_mask))
      {
        const DWORD     cpu = GetCurrentProcessorNumber();
        const DWORD_PTR own = (cpu < (DWORD)(8U * sizeof(DWORD_PTR))) ? ((DWORD_PTR)1U << cpu) : (DWORD_PTR)0U;
        const DWORD_PTR pair = (cpu < (DWORD)(8U * sizeof(DWORD_PTR)))
                                   ? ((DWORD_PTR)3U << (cpu & ~(DWORD)1U))
                                   : (DWORD_PTR)0U;
        const DWORD_PTR rest = ((proc_mask & ~pair) != (DWORD_PTR)0U) ? (proc_mask & ~pair) : (proc_mask & ~own);

        if (rest != (DWORD_PTR)0U)
        {
          (void)SetThreadAffinityMask(thread, rest);
        }
      }
      (void)CloseHandle(thread);
    }
#else
    pthread_t thread;

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
    __atomic_store_n(&beside.state, 1, __ATOMIC_RELEASE);
    (void)SetEvent(beside.asked);
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
    int expected = 1;

    (void)WaitForSingleObject(gmo_beside.asked, INFINITE);
    wbc_gmo_step(gmo_beside.o, &gmo_beside.f, gmo_beside.q, gmo_beside.u, gmo_beside.dt);
    if (!__atomic_compare_exchange_n(&gmo_beside.state, &expected, 2, 0, __ATOMIC_RELEASE, __ATOMIC_ACQUIRE))
    {
      __atomic_store_n(&gmo_beside.state, 2, __ATOMIC_RELEASE);
      (void)SetEvent(gmo_beside.done);
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

      for (int spin = 0; (spin < 256) && (__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 2); spin++)
      {
        YieldProcessor();
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
      for (int spin = 0; (spin < 256) && (__atomic_load_n(&gmo_beside.state, __ATOMIC_ACQUIRE) != 2); spin++)
      {
        (void)sched_yield();
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
    __atomic_store_n(&gmo_beside.state, 0, __ATOMIC_RELAXED);
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
    HANDLE thread = NULL;

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
      DWORD_PTR proc_mask = 0U, sys_mask = 0U;

      if ((GetProcessAffinityMask(GetCurrentProcess(), &proc_mask, &sys_mask) != 0) && (proc_mask != sys_mask))
      {
        const DWORD     cpu = GetCurrentProcessorNumber();
        const DWORD_PTR own = (cpu < (DWORD)(8U * sizeof(DWORD_PTR))) ? ((DWORD_PTR)1U << cpu) : (DWORD_PTR)0U;
        const DWORD_PTR pair = (cpu < (DWORD)(8U * sizeof(DWORD_PTR)))
                                   ? ((DWORD_PTR)3U << (cpu & ~(DWORD)1U))
                                   : (DWORD_PTR)0U;
        const DWORD_PTR rest = ((proc_mask & ~pair) != (DWORD_PTR)0U) ? (proc_mask & ~pair) : (proc_mask & ~own);

        if (rest != (DWORD_PTR)0U)
        {
          (void)SetThreadAffinityMask(thread, rest);
        }
      }
      (void)CloseHandle(thread);
    }
#else
    pthread_t thread;

    (void)pthread_mutex_init(&gmo_beside.lock, NULL);
    (void)pthread_cond_init(&gmo_beside.turn, NULL);
    gmo_beside.started = 1;
    if (pthread_create(&thread, NULL, gmo_besides, NULL) != 0)
    {
      gmo_beside.started = -1;
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
    __atomic_store_n(&gmo_beside.state, 1, __ATOMIC_RELEASE);
    (void)SetEvent(gmo_beside.asked);
#else
    (void)pthread_mutex_lock(&gmo_beside.lock);
    __atomic_store_n(&gmo_beside.state, 1, __ATOMIC_RELEASE);
    (void)pthread_cond_broadcast(&gmo_beside.turn);
    (void)pthread_mutex_unlock(&gmo_beside.lock);
#endif
  }
  return gmo_beside.pending;
}
