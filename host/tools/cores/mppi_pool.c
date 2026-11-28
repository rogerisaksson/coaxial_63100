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
  const double *white, *nom, *sigma, *lo, *hi, *prior;
  double       *u_mut, *noise, *cost;
  double        dt, w_p, w_v, ar, ar_scale;
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

static inline double clamp_d(double v, double lo, double hi)
{
  return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static void prepare_chunk(const mppi_job_t *j, int base, int stop)
{
  if ((j->h <= 0) || (base >= stop))
  {
    return;
  }
  const size_t  stride = (size_t)j->h * 4U;
  const double  ar = j->ar, ar_s = j->ar_scale;
  const double  s0 = j->sigma[0], s1 = j->sigma[1], s2 = j->sigma[2], s3 = j->sigma[3];
  const double  lo0 = j->lo[0], lo1 = j->lo[1], lo2 = j->lo[2], lo3 = j->lo[3];
  const double  hi0 = j->hi[0], hi1 = j->hi[1], hi2 = j->hi[2], hi3 = j->hi[3];
  const double *nom = j->nom;

  for (int i = base; i < stop; i++)
  {
    const size_t  off = (size_t)i * stride;
    const double *wh = j->white + off;
    double       *u_out = j->u_mut + off;
    double       *ns_out = j->noise + off;

    if ((i == 0) && (j->prior != NULL))
    {
      const double p0 = clamp_d(j->prior[0], lo0, hi0);
      const double p1 = clamp_d(j->prior[1], lo1, hi1);
      const double p2 = clamp_d(j->prior[2], lo2, hi2);
      const double p3 = clamp_d(j->prior[3], lo3, hi3);

      for (int step = 0; step < j->h; step++)
      {
        const size_t o = (size_t)step * 4U;

        u_out[o + 0U] = p0;
        u_out[o + 1U] = p1;
        u_out[o + 2U] = p2;
        u_out[o + 3U] = p3;
        ns_out[o + 0U] = p0 - nom[o + 0U];
        ns_out[o + 1U] = p1 - nom[o + 1U];
        ns_out[o + 2U] = p2 - nom[o + 2U];
        ns_out[o + 3U] = p3 - nom[o + 3U];
      }
      continue;
    }
    double w0 = wh[0], w1 = wh[1], w2 = wh[2], w3 = wh[3];
    double u0 = clamp_d(nom[0] + w0 * s0, lo0, hi0);
    double u1 = clamp_d(nom[1] + w1 * s1, lo1, hi1);
    double u2 = clamp_d(nom[2] + w2 * s2, lo2, hi2);
    double u3 = clamp_d(nom[3] + w3 * s3, lo3, hi3);

    u_out[0] = u0;
    u_out[1] = u1;
    u_out[2] = u2;
    u_out[3] = u3;
    ns_out[0] = u0 - nom[0];
    ns_out[1] = u1 - nom[1];
    ns_out[2] = u2 - nom[2];
    ns_out[3] = u3 - nom[3];
    for (int step = 1; step < j->h; step++)
    {
      const size_t o = (size_t)step * 4U;

      w0 = ar * w0 + ar_s * wh[o + 0U];
      w1 = ar * w1 + ar_s * wh[o + 1U];
      w2 = ar * w2 + ar_s * wh[o + 2U];
      w3 = ar * w3 + ar_s * wh[o + 3U];
      u0 = clamp_d(nom[o + 0U] + w0 * s0, lo0, hi0);
      u1 = clamp_d(nom[o + 1U] + w1 * s1, lo1, hi1);
      u2 = clamp_d(nom[o + 2U] + w2 * s2, lo2, hi2);
      u3 = clamp_d(nom[o + 3U] + w3 * s3, lo3, hi3);
      u_out[o + 0U] = u0;
      u_out[o + 1U] = u1;
      u_out[o + 2U] = u2;
      u_out[o + 3U] = u3;
      ns_out[o + 0U] = u0 - nom[o + 0U];
      ns_out[o + 1U] = u1 - nom[o + 1U];
      ns_out[o + 2U] = u2 - nom[o + 2U];
      ns_out[o + 3U] = u3 - nom[o + 3U];
    }
  }
}

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
    if (j.white != NULL)
    {
      prepare_chunk(&j, base, stop);
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

static void dispatch_job(const mppi_job_t *job)
{
  if ((job->n <= 0) || (job->h <= 0))
  {
    return;
  }
  if (job->n < MPPI_MIN_BATCH)
  {
    if (job->white != NULL)
    {
      prepare_chunk(job, 0, job->n);
    }
    mppi_plane_costs_lanes(job->x0, job->u, job->ref, job->k, job->c, job->nc, job->ns, job->r,
                           job->nr, job->weight, job->u0, job->n, job->h, job->dt, job->w_p,
                           job->w_v, job->wind, job->cost);
    return;
  }
#ifdef _WIN32
  DWORD_PTR rest = (DWORD_PTR)0U;
  const int avail = available_threads(&rest);
#else
  const int avail = available_threads();
#endif
  const int by_n = job->n / 32;
  const int total = (avail < by_n) ? avail : by_n;

  if (total <= 1)
  {
    if (job->white != NULL)
    {
      prepare_chunk(job, 0, job->n);
    }
    mppi_plane_costs_lanes(job->x0, job->u, job->ref, job->k, job->c, job->nc, job->ns, job->r,
                           job->nr, job->weight, job->u0, job->n, job->h, job->dt, job->w_p,
                           job->w_v, job->wind, job->cost);
    return;
  }
#ifdef _WIN32
  (void)InitOnceExecuteOnce(&pool_once, init_gate, NULL, NULL);
  EnterCriticalSection(&pool_gate);
#else
  (void)pthread_mutex_lock(&pool_gate);
#endif
  pool_job = *job;
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

void mppi_plane_costs_parallel(const double *x0, const double *u, const double *ref,
                               const double *k, const double *c, int nc, int ns, const double *r,
                               int nr, const double *weight, const double *u0, int n, int h,
                               double dt, double w_p, double w_v, const double *wind, double *cost)
{
  const mppi_job_t job = {
    .x0 = x0, .u = u, .ref = ref, .k = k, .c = c, .r = r, .weight = weight, .u0 = u0, .wind = wind,
    .cost = cost, .dt = dt, .w_p = w_p, .w_v = w_v, .nc = nc, .ns = ns, .nr = nr, .n = n, .h = h,
  };
  dispatch_job(&job);
}

void mppi_sample_and_costs(const double *x0, const double *white, const double *nom,
                           const double *sigma, const double *lo, const double *hi,
                           const double *prior, double ar, double ar_scale, double *u_out,
                           double *noise_out, const double *ref, const double *k, const double *c,
                           int nc, int ns, const double *r, int nr, const double *weight,
                           const double *u0, int n, int h, double dt, double w_p, double w_v,
                           const double *wind, double *cost)
{
  const mppi_job_t job = {
    .x0 = x0, .u = u_out, .ref = ref, .k = k, .c = c, .r = r, .weight = weight, .u0 = u0,
    .wind = wind, .white = white, .nom = nom, .sigma = sigma, .lo = lo, .hi = hi, .prior = prior,
    .u_mut = u_out, .noise = noise_out, .cost = cost, .dt = dt, .w_p = w_p, .w_v = w_v,
    .ar = ar, .ar_scale = ar_scale, .nc = nc, .ns = ns, .nr = nr, .n = n, .h = h,
  };
  dispatch_job(&job);
}
