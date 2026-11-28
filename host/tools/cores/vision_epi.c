/** vision_epi.c - Affinity-bounded worker pool and epipolar plane/warp C11 kernels. */
#ifndef _WIN32
#define _GNU_SOURCE
#endif

#include "vision.h"

#include <math.h>
#include <stdbool.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif

#define VS_SPIN_AWAKE 2048
#define VS_SPIN_JOIN  2048

typedef struct
{
  vs_task_fn fn;
  void      *ctx;
  int32_t    tid;
  int32_t    nt;
  int        started;
  int        state;
#ifdef _WIN32
  int    parked;
  HANDLE asked;
  HANDLE done;
#else
  pthread_mutex_t lock;
  pthread_cond_t  turn;
  pthread_cond_t  done;
#endif
} vs_worker_t;

static vs_worker_t vs_workers[VS_MAX_THREADS - 1];
static int         vs_pool_busy = 0;

static int32_t vs_avail_cpus(void)
{
#ifdef _WIN32
  DWORD_PTR proc_mask = 0U, sys_mask = 0U;
  if (GetProcessAffinityMask(GetCurrentProcess(), &proc_mask, &sys_mask) != 0 && proc_mask != 0U)
  {
    int32_t n = vs_popcount64((uint64_t)proc_mask);
    return (n > 1) ? ((n < VS_MAX_THREADS) ? n : VS_MAX_THREADS) : 1;
  }
  return 1;
#else
#if defined(__linux__)
  cpu_set_t set;
  if (sched_getaffinity(0, sizeof(set), &set) == 0)
  {
    const int n = CPU_COUNT(&set);
    return (n > 1) ? ((n < VS_MAX_THREADS) ? (int32_t)n : VS_MAX_THREADS) : 1;
  }
#endif
  const long n = sysconf(_SC_NPROCESSORS_ONLN);
  return (n > 1L) ? ((n < (long)VS_MAX_THREADS) ? (int32_t)n : VS_MAX_THREADS) : 1;
#endif
}

#ifdef _WIN32
static DWORD WINAPI vs_worker_loop(LPVOID arg)
{
  vs_worker_t *w = (vs_worker_t *)arg;
  while (w->started != 0)
  {
    int st = __atomic_load_n(&w->state, __ATOMIC_ACQUIRE);
    if ((st != 1) && (st != 3))
    {
      for (int spin = 0; spin < VS_SPIN_AWAKE; spin++)
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
    w->fn(w->ctx, w->tid, w->nt);
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
#else
static void *vs_worker_loop(void *arg)
{
  vs_worker_t *w = (vs_worker_t *)arg;
  while (w->started != 0)
  {
    int st = __atomic_load_n(&w->state, __ATOMIC_ACQUIRE);
    if ((st != 1) && (st != 3))
    {
      for (int spin = 0; spin < 512; spin++)
      {
#if defined(__x86_64__) || defined(__i386__)
        __builtin_ia32_pause();
#elif defined(__aarch64__)
        __asm__ volatile("yield" ::: "memory");
#endif
        st = __atomic_load_n(&w->state, __ATOMIC_ACQUIRE);
        if ((st == 1) || (st == 3))
        {
          break;
        }
      }
    }
    if ((st != 1) && (st != 3))
    {
      (void)pthread_mutex_lock(&w->lock);
      while ((__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 1) &&
             (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 3))
      {
        (void)pthread_cond_wait(&w->turn, &w->lock);
      }
      (void)pthread_mutex_unlock(&w->lock);
    }
    w->fn(w->ctx, w->tid, w->nt);
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

static bool vs_ensure_worker(int32_t idx)
{
  vs_worker_t *w = &vs_workers[idx];
  if (w->started == 0)
  {
    w->tid = idx + 1;
    w->state = 0;
#ifdef _WIN32
    w->parked = 1;
    w->asked = CreateEventA(NULL, FALSE, FALSE, NULL);
    w->done = CreateEventA(NULL, FALSE, FALSE, NULL);
    w->started = 1;
    HANDLE th = ((w->asked != NULL) && (w->done != NULL)) ? CreateThread(NULL, 0U, vs_worker_loop, w, 0U, NULL) : NULL;
    if (th == NULL)
    {
      w->started = -1;
    }
    else
    {
      (void)CloseHandle(th);
    }
#else
    (void)pthread_mutex_init(&w->lock, NULL);
    (void)pthread_cond_init(&w->turn, NULL);
    (void)pthread_cond_init(&w->done, NULL);
    w->started = 1;
    pthread_t th;
    if (pthread_create(&th, NULL, vs_worker_loop, w) != 0)
    {
      w->started = -1;
    }
    else
    {
      (void)pthread_detach(th);
    }
#endif
  }
  return w->started == 1;
}

int32_t vs_threads_acquire(int32_t max_want)
{
  const int32_t avail = vs_avail_cpus();
  int32_t       want = (max_want < avail) ? max_want : avail;
  if (want <= 1)
  {
    return 1;
  }
  if (__atomic_exchange_n(&vs_pool_busy, 1, __ATOMIC_ACQUIRE) != 0)
  {
    return 1;
  }
  int32_t active = 1;
  for (int32_t i = 0; i < want - 1; i++)
  {
    if (!vs_ensure_worker(i))
    {
      break;
    }
    active++;
  }
  if (active <= 1)
  {
    __atomic_store_n(&vs_pool_busy, 0, __ATOMIC_RELEASE);
    return 1;
  }
  return active;
}

void vs_threads_release(int32_t nt)
{
  if (nt > 1)
  {
    __atomic_store_n(&vs_pool_busy, 0, __ATOMIC_RELEASE);
  }
}

void vs_parallel(int32_t nt, vs_task_fn fn, void *ctx)
{
  if (nt <= 1)
  {
    fn(ctx, 0, 1);
    return;
  }
  for (int32_t i = 0; i < nt - 1; i++)
  {
    vs_worker_t *w = &vs_workers[i];
    w->fn = fn;
    w->ctx = ctx;
    w->nt = nt;
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
  }
  fn(ctx, 0, nt);
  for (int32_t i = 0; i < nt - 1; i++)
  {
    vs_worker_t *w = &vs_workers[i];
#ifdef _WIN32
    if (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2)
    {
      for (int spin = 0; (spin < VS_SPIN_JOIN) && (__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2); spin++)
      {
        YieldProcessor();
      }
      int expected = 1;
      if ((__atomic_load_n(&w->state, __ATOMIC_ACQUIRE) != 2) &&
          __atomic_compare_exchange_n(&w->state, &expected, 3, 0, __ATOMIC_ACQ_REL, __ATOMIC_ACQUIRE))
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
}

typedef struct
{
  const uint8_t *src;
  const int32_t *at4;
  const int32_t *wt4;
  const uint8_t *on_mask;
  uint8_t       *dst;
  int32_t        height;
  int32_t        width;
  int32_t        bits;
} vs_warp_ctx_t;

static void vs_warp_task(void *raw, int32_t tid, int32_t nt)
{
  const vs_warp_ctx_t *c = (const vs_warp_ctx_t *)raw;
  const size_t         npix = (size_t)c->height * (size_t)c->width;
  const int32_t        fw = 2 * c->width;
  const int32_t        aw = fw - 1;
  const int64_t        whole = ((int64_t)1 << (2 * c->bits)) * 4;
  const int64_t        half = whole >> 1;
  const size_t         pa = ((size_t)tid * npix) / (size_t)nt;
  const size_t         pb = ((size_t)(tid + 1) * npix) / (size_t)nt;

  for (size_t p = pa; p < pb; p++)
  {
    const size_t o = p * 3U;
    if (c->on_mask[p] == 0U)
    {
      c->dst[o] = 0U;
      c->dst[o + 1U] = 0U;
      c->dst[o + 2U] = 0U;
      continue;
    }
    int64_t s0 = 0, s1 = 0, s2 = 0;
    for (size_t k = 0U; k < 4U; k++)
    {
      const int32_t idx = c->at4[k * npix + p];
      const int64_t w = (int64_t)c->wt4[k * npix + p];
      const int32_t r = idx / aw;
      const int32_t col = idx - r * aw;
      const size_t  p00 = ((size_t)r * (size_t)fw + (size_t)col) * 3U;
      const size_t  p10 = p00 + (size_t)fw * 3U;
      const int64_t a0 =
        (int64_t)c->src[p00] + (int64_t)c->src[p00 + 3U] + (int64_t)c->src[p10] + (int64_t)c->src[p10 + 3U];
      const int64_t a1 =
        (int64_t)c->src[p00 + 1U] + (int64_t)c->src[p00 + 4U] + (int64_t)c->src[p10 + 1U] + (int64_t)c->src[p10 + 4U];
      const int64_t a2 =
        (int64_t)c->src[p00 + 2U] + (int64_t)c->src[p00 + 5U] + (int64_t)c->src[p10 + 2U] + (int64_t)c->src[p10 + 5U];
      s0 += a0 * w;
      s1 += a1 * w;
      s2 += a2 * w;
    }
    c->dst[o] = (uint8_t)((s0 + half) / whole);
    c->dst[o + 1U] = (uint8_t)((s1 + half) / whole);
    c->dst[o + 2U] = (uint8_t)((s2 + half) / whole);
  }
}

void vs_warp_u8(const uint8_t *restrict src, const int32_t *restrict at4, const int32_t *restrict wt4,
                const uint8_t *restrict on_mask, int32_t height, int32_t width, int32_t bits, uint8_t *restrict dst)
{
  if ((height <= 0) || (width <= 0) || (bits < 0) || (bits > 30))
  {
    return;
  }
  vs_warp_ctx_t ctx = {src, at4, wt4, on_mask, dst, height, width, bits};
  const int32_t nt = ((int64_t)height * (int64_t)width >= 16384) ? vs_threads_acquire(VS_MAX_THREADS) : 1;
  vs_parallel(nt, vs_warp_task, &ctx);
  vs_threads_release(nt);
}

typedef struct
{
  const double *d_xyz;
  int64_t      *v_out;
  int64_t      *u_out;
  uint8_t      *on_out;
  int32_t      *at4_out;
  int32_t      *wt4_out;
  double        f_px;
  int32_t       height;
  int32_t       width;
  int32_t       bits;
  int32_t       fine;
} vs_epi_ctx_t;

static void vs_epi_task(void *raw, int32_t tid, int32_t nt)
{
  const vs_epi_ctx_t *c = (const vs_epi_ctx_t *)raw;
  const size_t        npix = (size_t)c->height * (size_t)c->width;
  const size_t        pa = ((size_t)tid * npix) / (size_t)nt;
  const size_t        pb = ((size_t)(tid + 1) * npix) / (size_t)nt;
  const int64_t       q_i = (int64_t)1 << c->bits;
  const double        q_d = (double)q_i;
  const double        half_h = 0.5 * (double)c->height;
  const double        half_w = 0.5 * (double)c->width;
  const double        h_d = (double)c->height;
  const double        w_d = (double)c->width;
  const int64_t       v_max = (int64_t)(c->height - 1) * q_i;
  const int64_t       u_max = (int64_t)(c->width - 1) * q_i;
  const int64_t       area_h = (int64_t)(c->fine * c->height - c->fine + 1);
  const int64_t       area_w = (int64_t)(c->fine * c->width - c->fine + 1);

  for (size_t p = pa; p < pb; p++)
  {
    const double d0 = c->d_xyz[p * 3U + 0U];
    const double d1 = c->d_xyz[p * 3U + 1U];
    const double d2 = c->d_xyz[p * 3U + 2U];
    const double ahead = (d2 < 0.0) ? -d2 : 1.0;
    double       av = half_h - (c->f_px * d1) / ahead;
    double       au = half_w + (c->f_px * d0) / ahead;
    int64_t      V = 0;
    int64_t      U = 0;
    bool         is_on = false;
    if ((d2 < 0.0) && isfinite(av) && isfinite(au))
    {
      av = (av < -1.0) ? -1.0 : ((av > h_d) ? h_d : av);
      au = (au < -1.0) ? -1.0 : ((au > w_d) ? w_d : au);
      V = (int64_t)rint(av * q_d);
      U = (int64_t)rint(au * q_d);
      is_on = (U >= 0) && (U <= u_max) && (V >= 0) && (V <= v_max);
      if (!is_on)
      {
        V = 0;
        U = 0;
      }
    }
    c->on_out[p] = (uint8_t)is_on;
    c->v_out[p] = V;
    c->u_out[p] = U;
    const int64_t u = U * (int64_t)c->fine;
    const int64_t v = V * (int64_t)c->fine;
    const int64_t u0 = u >> c->bits;
    const int64_t v0 = v >> c->bits;
    const int64_t fu = u & (q_i - 1);
    const int64_t fv = v & (q_i - 1);
    const int64_t u1 = (u0 + 1 < area_w - 1) ? (u0 + 1) : (area_w - 1);
    const int64_t v1 = (v0 + 1 < area_h - 1) ? (v0 + 1) : (area_h - 1);
    c->at4_out[0U * npix + p] = (int32_t)(v0 * area_w + u0);
    c->at4_out[1U * npix + p] = (int32_t)(v0 * area_w + u1);
    c->at4_out[2U * npix + p] = (int32_t)(v1 * area_w + u0);
    c->at4_out[3U * npix + p] = (int32_t)(v1 * area_w + u1);
    c->wt4_out[0U * npix + p] = (int32_t)((q_i - fu) * (q_i - fv));
    c->wt4_out[1U * npix + p] = (int32_t)(fu * (q_i - fv));
    c->wt4_out[2U * npix + p] = (int32_t)((q_i - fu) * fv);
    c->wt4_out[3U * npix + p] = (int32_t)(fu * fv);
  }
}

void vs_epi_from_d(const double *restrict d_xyz, int32_t height, int32_t width, double f_px, int32_t bits, int32_t fine,
                   int64_t *restrict v_out, int64_t *restrict u_out, uint8_t *restrict on_out,
                   int32_t *restrict at4_out, int32_t *restrict wt4_out)
{
  if ((height <= 0) || (width <= 0) || (fine <= 0) || (bits < 0) || (bits > 30))
  {
    return;
  }
  vs_epi_ctx_t  ctx = {d_xyz, v_out, u_out, on_out, at4_out, wt4_out, f_px, height, width, bits, fine};
  const int32_t nt = ((int64_t)height * (int64_t)width >= 16384) ? vs_threads_acquire(VS_MAX_THREADS) : 1;
  vs_parallel(nt, vs_epi_task, &ctx);
  vs_threads_release(nt);
}

void vs_epi_kept(const uint8_t *restrict inner0, const uint8_t *restrict inner1, const double *restrict disp,
                 int32_t height, int32_t width, uint8_t *restrict out)
{
  if ((height <= 0) || (width <= 0))
  {
    return;
  }
  const double w_hi = (double)(width - 1);
  for (int32_t y = 0; y < height; y++)
  {
    const size_t rbase = (size_t)y * (size_t)width;
    for (int32_t x = 0; x < width; x++)
    {
      const size_t idx = rbase + (size_t)x;
      if (inner0[idx] == 0U)
      {
        out[idx] = 0U;
        continue;
      }
      const double dv = (double)x - disp[idx];
      int64_t      rx = 0;
      if (!isfinite(dv))
      {
        rx = (dv > 0.0) ? (int64_t)(width - 1) : 0;
      }
      else if (dv >= w_hi)
      {
        rx = (int64_t)(width - 1);
      }
      else if (dv > 0.0)
      {
        rx = (int64_t)rint(dv);
      }
      out[idx] = inner1[rbase + (size_t)rx];
    }
  }
}

void vs_stereo_post(const int64_t *restrict gl, const int64_t *restrict best, const float *restrict tbest,
                    const float *restrict second, const int64_t *restrict seen, const float *restrict c0,
                    const float *restrict lo, const float *restrict hi, int32_t height, int32_t width,
                    int32_t d_count, int32_t rad, double texture, float unique_f, double edge_px,
                    int64_t *restrict box_a, int64_t *restrict box_b, int64_t *restrict box_c,
                    int64_t *restrict vsum_buf, double *restrict disp_out, uint8_t *restrict kept_out)
{
  const size_t  npix = (size_t)height * (size_t)width;
  const int32_t win = 2 * rad + 1;
  const double  w2 = (double)(win * win);
  int64_t      *box_d = vsum_buf + (size_t)width;
  int64_t      *box_e = box_d + npix;

  vs_box_i64(gl, box_a, box_d, vsum_buf, height, width, rad);
  for (size_t i = 0U; i < npix; i++)
  {
    box_d[i] = gl[i] * gl[i];
  }
  vs_box_i64(box_d, box_b, box_e, vsum_buf, height, width, rad);

  for (size_t i = 0U; i < npix; i++)
  {
    const float c0v = c0[i], lov = lo[i], hiv = hi[i];
    const float denom = (lov - 2.0f * c0v) + hiv;
    float       sub = 0.0f;
    if (isfinite(denom) && (denom > 1e-6f))
    {
      sub = (0.5f * (lov - hiv)) / denom;
    }
    const float sub_c = (sub < -0.5f) ? -0.5f : ((sub > 0.5f) ? 0.5f : sub);
    disp_out[i] = (double)best[i] + (double)sub_c;
  }

  for (int32_t y = 0; y < height; y++)
  {
    const size_t rbase = (size_t)y * (size_t)width;
    for (int32_t x = 0; x < width; x++)
    {
      const size_t i = rbase + (size_t)x;
      const double di = disp_out[i];
      int64_t      jmp = 0;
      if (x > 0)
      {
        jmp |= (int64_t)(fabs(di - disp_out[i - 1U]) > edge_px);
      }
      if (y > 0)
      {
        jmp |= (int64_t)(fabs(di - disp_out[i - (size_t)width]) > edge_px);
      }
      box_d[i] = jmp;
    }
  }
  vs_box_i64(box_d, box_c, box_e, vsum_buf, height, width, rad);

  for (int32_t y = 0; y < height; y++)
  {
    const size_t rbase = (size_t)y * (size_t)width;
    for (int32_t x = 0; x < width; x++)
    {
      const size_t  i = rbase + (size_t)x;
      const int64_t b = best[i];
      if (x < d_count)
      {
        kept_out[i] = 0U;
        continue;
      }
      const double  mean = (double)box_a[i] / w2;
      const double  var = (double)box_b[i] / w2 - mean * mean;
      const double  spread = sqrt((var > 0.0) ? var : 0.0);
      const int32_t rx = vs_clamp_i32(x - (int32_t)b, 0, width - 1);
      const int64_t ds = seen[rbase + (size_t)rx] - b;
      const bool    agreed = ((ds < 0) ? -ds : ds) <= 1;
      const float   tb = tbest[i];
      const float   tb1 = (tb != tb) ? tb : ((tb > 1.0f) ? tb : 1.0f);
      kept_out[i] = (uint8_t)((spread > texture) && (second[i] > unique_f * tb1) && (b > 0) &&
                              (b < (int64_t)(d_count - 1)) && isfinite(c0[i]) && agreed && (box_c[i] <= 0));
    }
  }
}
