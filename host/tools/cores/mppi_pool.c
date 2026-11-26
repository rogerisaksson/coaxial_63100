/** mppi_pool.c - worker threads across a plane's rollouts on the host (mppi_plane_costs) */
#ifndef _WIN32
#define _GNU_SOURCE
#endif

#include "mppi.h"

#include <stddef.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif

enum { MPPI_WORKERS = 7, MPPI_CHUNK = 8, MPPI_MIN_BATCH = 64, MPPI_SPIN = 4096 };

void mppi_plane_costs_lanes(const double *x0, const double *u, const double *ref, const double *k,
                            const double *c, int nc, int ns, const double *r, int nr,
                            const double *weight, const double *u0, int n, int h, double dt,
                            double w_p, double w_v, const double *wind, double *cost);

typedef struct
{
  const double *x0, *u, *ref, *k, *c, *r, *weight, *u0, *wind;
  double       *cost;
  double        dt, w_p, w_v;
  int           nc, ns, nr, n, h;
} mppi_job_t;

typedef struct
{
  int started;
  int state;
#ifdef _WIN32
  int       parked;
  HANDLE    thread;
  HANDLE    asked;
  HANDLE    done;
  DWORD_PTR pinned;
#else
  pthread_mutex_t lock;
  pthread_cond_t  turn;
  pthread_cond_t  done;
#endif
} mppi_worker_t;

static mppi_worker_t pool_workers[MPPI_WORKERS];
static mppi_job_t    pool_job;
static int           pool_next;

#ifdef _WIN32
static INIT_ONCE        pool_once = INIT_ONCE_STATIC_INIT;
static CRITICAL_SECTION pool_gate;

static BOOL CALLBACK init_gate(PINIT_ONCE once, PVOID param, PVOID *ctx)
{
  (void)once;
  (void)param;
  (void)ctx;
  InitializeCriticalSection(&pool_gate);
  return TRUE;
}
#else
static pthread_mutex_t pool_gate = PTHREAD_MUTEX_INITIALIZER;
#endif

static void run_chunks(void)
{
  const mppi_job_t j = pool_job;

  for (;;)
  {
    const int base = __atomic_fetch_add(&pool_next, MPPI_CHUNK, __ATOMIC_RELAXED);
    const int stop = ((base + MPPI_CHUNK) < j.n) ? (base + MPPI_CHUNK) : j.n;

    if (base >= j.n)
    {
      break;
    }
    mppi_plane_costs_lanes(j.x0, j.u + (size_t)base * (size_t)j.h * 4U, j.ref, j.k, j.c, j.nc, j.ns,
                           j.r, j.nr, j.weight, j.u0, stop - base, j.h, j.dt, j.w_p, j.w_v, j.wind,
                           j.cost + base);
  }
}

#ifdef _WIN32
static DWORD WINAPI worker_loop(LPVOID arg)
{
  mppi_worker_t *w = (mppi_worker_t *)arg;

  while (w->started != 0)
  {
    int st = __atomic_load_n(&w->state, __ATOMIC_ACQUIRE);

    if ((st != 1) && (st != 3))
    {
      for (int spin = 0; spin < MPPI_SPIN; spin++)
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
    run_chunks();
    {
      int expected = 1;

      if (!__atomic_compare_exchange_n(&w->state, &expected, 2, 0, __ATOMIC_RELEASE, __ATOMIC_ACQUIRE))
      {
        __atomic_store_n(&w->state, 2, __ATOMIC_RELEASE);
        (void)SetEvent(w->done);
      }
    }
  }
  return 0U;
}

static int count_cores(DWORD_PTR mask)
{
  int cores = 0;

  for (int c = 0; c < (int)(4U * sizeof(DWORD_PTR)); c++)
  {
    const DWORD_PTR pair = (DWORD_PTR)3U << (unsigned int)(2 * c);

    if ((mask & pair) != (DWORD_PTR)0U)
    {
      cores++;
    }
  }
  return cores;
}

static DWORD_PTR nth_core_mask(DWORD_PTR mask, int idx)
{
  for (int c = 0; c < (int)(4U * sizeof(DWORD_PTR)); c++)
  {
    const DWORD_PTR hit = mask & ((DWORD_PTR)3U << (unsigned int)(2 * c));

    if (hit != (DWORD_PTR)0U)
    {
      if (idx == 0)
      {
        return hit;
      }
      idx--;
    }
  }
  return (DWORD_PTR)0U;
}

static int available_threads(DWORD_PTR *rest_out)
{
  DWORD_PTR proc_mask = 0U, sys_mask = 0U;

  *rest_out = (DWORD_PTR)0U;
  if (GetProcessAffinityMask(GetCurrentProcess(), &proc_mask, &sys_mask) == 0)
  {
    return 1;
  }
  const int cores = count_cores(proc_mask);

  if (cores <= 1)
  {
    return 1;
  }
  const DWORD     cpu = GetCurrentProcessorNumber();
  const DWORD_PTR own = (cpu < (DWORD)(8U * sizeof(DWORD_PTR)))
                            ? ((DWORD_PTR)3U << (cpu & ~(DWORD)1U))
                            : (DWORD_PTR)0U;

  *rest_out = proc_mask & ~own;
  return (cores > (MPPI_WORKERS + 1)) ? (MPPI_WORKERS + 1) : cores;
}
#else
static void *worker_loop(void *arg)
{
  mppi_worker_t *w = (mppi_worker_t *)arg;

  while (w->started != 0)
  {
    (void)pthread_mutex_lock(&w->lock);
    while ((__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 1)
           && (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 3))
    {
      (void)pthread_cond_wait(&w->turn, &w->lock);
    }
    (void)pthread_mutex_unlock(&w->lock);
    run_chunks();
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

static int available_threads(void)
{
  int bits = 0;

#if defined(__linux__)
  cpu_set_t set;

  if (sched_getaffinity(0, sizeof(set), &set) == 0)
  {
    bits = CPU_COUNT(&set);
  }
#endif
  if (bits <= 0)
  {
    const long onln = sysconf(_SC_NPROCESSORS_ONLN);

    bits = (onln > 1L) ? (int)onln : 1;
  }
  if (bits <= 1)
  {
    return 1;
  }
#if defined(__aarch64__) || defined(__arm__)
  const int cores = bits;
#else
  const int cores = (bits >= 2) ? (bits / 2) : 1;
#endif
  return (cores > (MPPI_WORKERS + 1)) ? (MPPI_WORKERS + 1) : cores;
}
#endif

static int ensure_worker(int idx
#ifdef _WIN32
                         , DWORD_PTR rest
#endif
)
{
  mppi_worker_t *w = &pool_workers[idx];

  if (w->started == 0)
  {
#ifdef _WIN32
    w->parked = 1;
    w->state = 0;
    w->pinned = (DWORD_PTR)0U;
    w->asked = CreateEventA(NULL, FALSE, FALSE, NULL);
    w->done = CreateEventA(NULL, FALSE, FALSE, NULL);
    w->started = 1;
    w->thread = ((w->asked != NULL) && (w->done != NULL))
                    ? CreateThread(NULL, 0U, worker_loop, w, 0U, NULL)
                    : NULL;
    if (w->thread == NULL)
    {
      w->started = -1;
    }
#else
    pthread_t thread;

    w->state = 0;
    (void)pthread_mutex_init(&w->lock, NULL);
    (void)pthread_cond_init(&w->turn, NULL);
    (void)pthread_cond_init(&w->done, NULL);
    w->started = 1;
    if (pthread_create(&thread, NULL, worker_loop, w) != 0)
    {
      w->started = -1;
    }
    else
    {
      (void)pthread_detach(thread);
    }
#endif
  }
#ifdef _WIN32
  if (w->started == 1)
  {
    const DWORD_PTR target = nth_core_mask(rest, idx);

    if ((target != (DWORD_PTR)0U) && (target != w->pinned))
    {
      (void)SetThreadAffinityMask(w->thread, target);
      w->pinned = target;
    }
  }
#endif
  return w->started == 1;
}

void mppi_plane_costs_parallel(const double *x0, const double *u, const double *ref,
                               const double *k, const double *c, int nc, int ns, const double *r,
                               int nr, const double *weight, const double *u0, int n, int h,
                               double dt, double w_p, double w_v, const double *wind, double *cost)
{
  if (n < MPPI_MIN_BATCH)
  {
    mppi_plane_costs_lanes(x0, u, ref, k, c, nc, ns, r, nr, weight, u0, n, h, dt, w_p, w_v, wind, cost);
    return;
  }
#ifdef _WIN32
  DWORD_PTR rest = (DWORD_PTR)0U;
  const int avail = available_threads(&rest);
#else
  const int avail = available_threads();
#endif
  const int by_n = n / 32;
  const int total = (avail < by_n) ? avail : by_n;

  if (total <= 1)
  {
    mppi_plane_costs_lanes(x0, u, ref, k, c, nc, ns, r, nr, weight, u0, n, h, dt, w_p, w_v, wind, cost);
    return;
  }
#ifdef _WIN32
  (void)InitOnceExecuteOnce(&pool_once, init_gate, NULL, NULL);
  EnterCriticalSection(&pool_gate);
#else
  (void)pthread_mutex_lock(&pool_gate);
#endif
  pool_job = (mppi_job_t){x0, u, ref, k, c, r, weight, u0, wind, cost, dt, w_p, w_v, nc, ns, nr, n, h};
  __atomic_store_n(&pool_next, 0, __ATOMIC_RELEASE);
  int launched = 0;

  for (int idx = 0; idx < (total - 1); idx++)
  {
    mppi_worker_t *w = &pool_workers[idx];

#ifdef _WIN32
    if (!ensure_worker(idx, rest))
#else
    if (!ensure_worker(idx))
#endif
    {
      break;
    }
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
  run_chunks();
  for (int idx = 0; idx < launched; idx++)
  {
    mppi_worker_t *w = &pool_workers[idx];

#ifdef _WIN32
    if (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2)
    {
      int expected = 1;

      for (int spin = 0; (spin < MPPI_SPIN) && (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2); spin++)
      {
        YieldProcessor();
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
  }
#ifdef _WIN32
  LeaveCriticalSection(&pool_gate);
#else
  (void)pthread_mutex_unlock(&pool_gate);
#endif
}
