/** harness_world.c - the robot's world's steps (wbc_world.c) through ctypes beside harness.c: tools.cores.world's
    calls; a stage of MuJoCo's step on a thread of the host's beside its law (machine.world.steps.ahead). */
#include "wbc_world.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#define API __declspec(dllexport)
#else
#include <pthread.h>
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
      (void)pthread_mutex_lock(&beside.lock);
      while (__atomic_load_n(&beside.state, __ATOMIC_ACQUIRE) != 2)
      {
        (void)pthread_cond_wait(&beside.turn, &beside.lock);
      }
      (void)pthread_mutex_unlock(&beside.lock);
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

      /* Where `placed` narrowed this process to a cache complex, keep the stage off the caller's core. */
      if ((GetProcessAffinityMask(GetCurrentProcess(), &proc_mask, &sys_mask) != 0) && (proc_mask != sys_mask))
      {
        const DWORD     cpu = GetCurrentProcessorNumber();
        const DWORD_PTR own = (cpu < (DWORD)(8U * sizeof(DWORD_PTR))) ? ((DWORD_PTR)1U << cpu) : (DWORD_PTR)0U;
        const DWORD_PTR rest = proc_mask & ~own;

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

API void wbh_world_before(wbc_steps_t *s, double since)
{
  wbc_world_before(s, since);
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
