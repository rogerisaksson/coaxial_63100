/** gmo_probe.c - the robot's observer (wbc_gmo.c and wbc_gmo_factor.c, included whole and unchanged) timed for
    tools.cores.gmo_probe: built apart from the robot's library, with -finstrument-functions over this file alone,
    wbc_gmo.c's calls out of it wrapped here to be timed too - the chain's dynamics, a Jacobian, the split's
    solve, a Givens rotation's hypot. Each function's calls, its ticks with what it calls and without, the
    calls it made and those beneath it, summed until read. Host-only: no robot runs it. */
#include "wbc.h"
#include "wbc_gmo.h"
#include "wbc_libm_arc.h"
#include "wbc_solve.h"

#include <stdint.h>
#include <string.h>

#if defined(__x86_64__) || defined(__i386__)
#include <x86intrin.h>
#else
#include <time.h>
#endif

#ifdef _WIN32
#define API __declspec(dllexport)
#else
#define API
#endif

/** The probe's own functions, never timed: the hooks call them. */
#define QUIET __attribute__((no_instrument_function))

QUIET static uint64_t probe_now(void)
{
#if defined(__x86_64__) || defined(__i386__)
  return (uint64_t)__rdtsc();
#else
  struct timespec t;

  (void)timespec_get(&t, TIME_UTC);
  return (uint64_t)t.tv_sec * 1000000000ULL + (uint64_t)t.tv_nsec;
#endif
}

static void probe_pose(wbc_body_t *b, const wbc_frame_t *base, const double q[WBC_N - 6])
{
  wbc_body_pose(b, base, q);
}

static void probe_mass(wbc_body_t *b)
{
  wbc_body_mass(b);
}

static void probe_bias(wbc_body_t *b, const double u[WBC_N])
{
  wbc_body_bias(b, u);
}

static void probe_jacobian(const wbc_body_t *b, int link, const double r[3], double j[6][WBC_N])
{
  wbc_body_jacobian(b, link, r, j);
}

static void probe_solve(int n, int ld, const double *l, const double *b, double *x)
{
  wbc_chol_solve(n, ld, l, b, x);
}

static void probe_solve_from(int n, int ld, const double *l, const double *b, int from, double *y, double *x)
{
  wbc_chol_solve_from(n, ld, l, b, from, y, x);
}

static double probe_hypot(double x, double y)
{
  return wbc_hypot(x, y);
}

#define wbc_body_pose     probe_pose
#define wbc_body_mass     probe_mass
#define wbc_body_bias     probe_bias
#define wbc_body_jacobian probe_jacobian
#define wbc_chol_solve    probe_solve
#define wbc_chol_solve_from probe_solve_from
#define wbc_hypot         probe_hypot
#include "wbc_gmo.c"
#include "wbc_gmo_factor.c"
#undef wbc_body_pose
#undef wbc_body_mass
#undef wbc_body_bias
#undef wbc_body_jacobian
#undef wbc_chol_solve
#undef wbc_chol_solve_from
#undef wbc_hypot

/** A call's hooks measured: an empty function called n times. */
__attribute__((noinline)) static void probe_empty(void)
{
  __asm__ volatile("");
}

__attribute__((noinline)) static void probe_empties(int n)
{
  for (int k = 0; k < n; k++)
  {
    probe_empty();
  }
}

typedef struct
{
  void (*fn)(void);
  const char *name;
} probe_fn_t;

/** The functions timed, each by its name in wbc_gmo.c or the call it wraps; any other the last slot's. */
static const probe_fn_t probe_fns[] = {
    {(void (*)(void))wbc_gmo_step, "wbc_gmo_step"},   {(void (*)(void))probe_pose, "wbc_body_pose"},
    {(void (*)(void))probe_mass, "wbc_body_mass"},    {(void (*)(void))probe_bias, "wbc_body_bias"},
    {(void (*)(void))accelerations, "accelerations"}, {(void (*)(void))columns, "columns"},
    {(void (*)(void))probe_jacobian, "wbc_body_jacobian"}, {(void (*)(void))expect, "expect"},
    {(void (*)(void))touches, "touches"},             {(void (*)(void))stage, "stage"},
    {(void (*)(void))scoped, "scoped"},               {(void (*)(void))split, "split"},
    {(void (*)(void))pushes, "pushes"},               {(void (*)(void))wbc_gmo_joined, "wbc_gmo_joined"},
    {(void (*)(void))wbc_gmo_left, "wbc_gmo_left"},   {(void (*)(void))probe_solve, "wbc_chol_solve"},
    {(void (*)(void))probe_hypot, "wbc_hypot"},       {(void (*)(void))shared, "shared"},
    {(void (*)(void))wbc_gmo_felt, "wbc_gmo_felt"},   {(void (*)(void))probe_solve_from, "wbc_chol_solve_from"},
    {(void (*)(void))probe_empties, "probe_empties"}, {(void (*)(void))probe_empty, "probe_empty"}};
#define PROBE_FNS   ((int)(sizeof probe_fns / sizeof probe_fns[0]))
#define PROBE_SLOTS (PROBE_FNS + 1)
#define PROBE_DEPTH 32

typedef struct
{
  int      slot;
  uint64_t at;    /**< the call's first tick */
  uint64_t inner; /**< its callees' ticks */
  int64_t  kids;  /**< the calls it made */
  int64_t  below; /**< every call beneath it */
} probe_frame_t;

static probe_frame_t probe_stack[PROBE_DEPTH];
static int           probe_depth;
static int64_t       probe_calls[PROBE_SLOTS], probe_with[PROBE_SLOTS], probe_own[PROBE_SLOTS];
static int64_t       probe_kids[PROBE_SLOTS], probe_below[PROBE_SLOTS];

QUIET static int probe_slot(const void *fn)
{
  int s = PROBE_FNS;

  for (int k = 0; k < PROBE_FNS; k++)
  {
    s = ((uintptr_t)probe_fns[k].fn == (uintptr_t)fn) ? k : s;
  }
  return s;
}

QUIET void __cyg_profile_func_enter(void *fn, void *site);
QUIET void __cyg_profile_func_exit(void *fn, void *site);

QUIET void __cyg_profile_func_enter(void *fn, void *site)
{
  (void)site;
  if (probe_depth < PROBE_DEPTH)
  {
    probe_frame_t *f = &probe_stack[probe_depth];

    f->slot = probe_slot(fn);
    f->inner = 0U;
    f->kids = 0;
    f->below = 0;
    f->at = probe_now();
  }
  probe_depth++;
}

QUIET void __cyg_profile_func_exit(void *fn, void *site)
{
  const uint64_t now = probe_now();

  (void)fn;
  (void)site;
  probe_depth--;
  if (probe_depth < PROBE_DEPTH)
  {
    const probe_frame_t *f = &probe_stack[probe_depth];
    const uint64_t       spent = now - f->at;

    probe_calls[f->slot]++;
    probe_with[f->slot] += (int64_t)spent;
    probe_own[f->slot] += (int64_t)(spent - f->inner);
    probe_kids[f->slot] += f->kids;
    probe_below[f->slot] += f->below;
    if (probe_depth > 0)
    {
      probe_frame_t *up = &probe_stack[probe_depth - 1];

      up->inner += spent;
      up->kids++;
      up->below += f->below + 1;
    }
  }
}

/** sizeof(wbc_gmo_t), held against tools.cores.gmo's mirror. */
API QUIET int probe_gmo_size(void)
{
  return (int)sizeof(wbc_gmo_t);
}

/** Each slot's name into `names` (the last slot's "other"); how many. */
API QUIET int probe_gmo_fns(const char **names)
{
  for (int k = 0; k < PROBE_FNS; k++)
  {
    names[k] = probe_fns[k].name;
  }
  names[PROBE_FNS] = "other";
  return PROBE_SLOTS;
}

/** wbh_gmo_step's step on `o`, timed. */
API QUIET void probe_gmo_step(wbc_gmo_t *o, const double *base, const double *q, const double *u, double dt)
{
  wbc_frame_t f;

  memcpy(f.r, base, 9U * sizeof(double));
  memcpy(f.p, base + 9, 3U * sizeof(double));
  wbc_gmo_step(o, &f, q, u, dt);
}

/** Each slot's calls, ticks with its callees and without, calls it made and calls beneath it, since the last read:
    into `out` (five rows of PROBE_SLOTS), the sums emptied. */
API QUIET void probe_gmo_read(long long *out)
{
  const int64_t *rows[5] = {probe_calls, probe_with, probe_own, probe_kids, probe_below};

  for (int r = 0; r < 5; r++)
  {
    for (int s = 0; s < PROBE_SLOTS; s++)
    {
      out[r * PROBE_SLOTS + s] = (long long)rows[r][s];
    }
  }
  memset(probe_calls, 0, sizeof probe_calls);
  memset(probe_with, 0, sizeof probe_with);
  memset(probe_own, 0, sizeof probe_own);
  memset(probe_kids, 0, sizeof probe_kids);
  memset(probe_below, 0, sizeof probe_below);
}

/** The probe's clock, ticks. */
API QUIET unsigned long long probe_gmo_now(void)
{
  return (unsigned long long)probe_now();
}

/** An empty function called `n` times beneath one: what a timed call's hooks cost its caller and itself. */
API QUIET void probe_gmo_hooks(int n)
{
  probe_empties(n);
}
