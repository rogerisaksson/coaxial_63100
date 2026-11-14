/** harness_world.c - her world's steps (wbc_world.c) through ctypes beside harness.c: tools.cores.world's
    calls; a stage of MuJoCo's step on a thread of the host's beside her law (machine.world.steps.ahead). */
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
#ifdef _WIN32
  HANDLE asked;
  HANDLE done;
#else
  pthread_mutex_t lock;
  pthread_cond_t  turn;
  int             state;
#endif
} beside;

#ifdef _WIN32
static DWORD WINAPI besides(LPVOID none)
{
  (void)none;
  while (beside.started != 0)
  {
    (void)WaitForSingleObject(beside.asked, INFINITE);
    beside.stage(beside.model, beside.data);
    (void)SetEvent(beside.done);
  }
  return 0;
}
#else
static void *besides(void *none)
{
  while (beside.started != 0)
  {
    (void)pthread_mutex_lock(&beside.lock);
    while (beside.state != 1)
    {
      (void)pthread_cond_wait(&beside.turn, &beside.lock);
    }
    (void)pthread_mutex_unlock(&beside.lock);
    beside.stage(beside.model, beside.data);
    (void)pthread_mutex_lock(&beside.lock);
    beside.state = 2;
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
    (void)WaitForSingleObject(beside.done, INFINITE);
#else
    (void)pthread_mutex_lock(&beside.lock);
    while (beside.state != 2)
    {
      (void)pthread_cond_wait(&beside.turn, &beside.lock);
    }
    beside.state = 0;
    (void)pthread_mutex_unlock(&beside.lock);
#endif
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
    beside.asked = CreateEventA(NULL, FALSE, FALSE, NULL);
    beside.done = CreateEventA(NULL, FALSE, FALSE, NULL);
    beside.started = 1;     /* before its thread reads it */
    if ((beside.asked == NULL) || (beside.done == NULL) || (CreateThread(NULL, 0, besides, NULL, 0, NULL) == NULL))
    {
      beside.started = -1;
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
    (void)SetEvent(beside.asked);
#else
    (void)pthread_mutex_lock(&beside.lock);
    beside.state = 1;
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
