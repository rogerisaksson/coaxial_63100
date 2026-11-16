/** wbc_classes.c - what the robot's eyes take a thing for (wbc_classes.h): machine.eyes.classes.of, its tables
    in their order and numpy's arithmetic operation for operation (wbc_np.h), over a look's boxes at once: the
    lodge's 70 things 1.1 ms of numpy a look. Portable C11, the host's alone. */
#include "wbc_classes.h"
#include "wbc_np.h"

#include <math.h>

/** A look's scratch: each thing's numbers (classes.of) - its extents along the room's axes, its top and bottom, its
    long and short sides, its corners, its middle and half along each axis (houses, planar), the longest opaque piece
    under it -, the buildings' corners and labels, its words and its pairs. */
typedef struct
{
  double *e, *top, *bottom, *lng, *shrt, *lo, *hi, *mid, *hlf, *longest, *blo, *bhi;
  int32_t *kcode, *kflags, *tops, *faces, *name, *label, *low, *part, *of;
  unsigned char *raw, *see, *wood, *top_, *surface, *wide, *free_, *steps, *more, *flight, *solid, *seat, *framed;
  unsigned char *walled, *backed, *out, *by, *held_any;
  unsigned char *beside, *within, *held, *rise, *link;
} work_t;

#define WBC_CLASSES_DOUBLES 26
#define WBC_CLASSES_INTS    9
#define WBC_CLASSES_BYTES   19
#define WBC_CLASSES_PAIRS   5

size_t wbc_classes_scratch(int n)
{
  const size_t m = (size_t)(n > 0 ? n : 0);

  return m * (WBC_CLASSES_DOUBLES * sizeof(double) + WBC_CLASSES_INTS * sizeof(int32_t) + WBC_CLASSES_BYTES)
         + WBC_CLASSES_PAIRS * m * m;
}

/** The next `bytes` of the scratch `s`, taken. */
static unsigned char *taken(unsigned char **s, size_t bytes)
{
  unsigned char *at = *s;

  *s += bytes;
  return at;
}

/** The scratch carved for `n` things: its doubles first, its ints, its bytes, its pairs. */
static void carve(work_t *w, int n, unsigned char *s)
{
  const size_t   m = (size_t)n, d = sizeof(double), i = sizeof(int32_t);
  unsigned char *at = s;

  w->e = (double *)taken(&at, 3 * m * d);
  w->top = (double *)taken(&at, m * d);
  w->bottom = (double *)taken(&at, m * d);
  w->lng = (double *)taken(&at, m * d);
  w->shrt = (double *)taken(&at, m * d);
  w->lo = (double *)taken(&at, 3 * m * d);
  w->hi = (double *)taken(&at, 3 * m * d);
  w->mid = (double *)taken(&at, 3 * m * d);
  w->hlf = (double *)taken(&at, 3 * m * d);
  w->longest = (double *)taken(&at, m * d);
  w->blo = (double *)taken(&at, 3 * m * d);
  w->bhi = (double *)taken(&at, 3 * m * d);
  w->kcode = (int32_t *)taken(&at, m * i);
  w->kflags = (int32_t *)taken(&at, m * i);
  w->tops = (int32_t *)taken(&at, m * i);
  w->faces = (int32_t *)taken(&at, m * i);
  w->name = (int32_t *)taken(&at, m * i);
  w->label = (int32_t *)taken(&at, m * i);
  w->low = (int32_t *)taken(&at, m * i);
  w->part = (int32_t *)taken(&at, m * i);
  w->of = (int32_t *)taken(&at, m * i);
  w->raw = taken(&at, m);
  w->see = taken(&at, m);
  w->wood = taken(&at, m);
  w->top_ = taken(&at, m);
  w->surface = taken(&at, m);
  w->wide = taken(&at, m);
  w->free_ = taken(&at, m);
  w->steps = taken(&at, m);
  w->more = taken(&at, m);
  w->flight = taken(&at, m);
  w->solid = taken(&at, m);
  w->seat = taken(&at, m);
  w->framed = taken(&at, m);
  w->walled = taken(&at, m);
  w->backed = taken(&at, m);
  w->out = taken(&at, m);
  w->by = taken(&at, m);
  w->held_any = taken(&at, m);
  w->beside = taken(&at, m * m);
  w->within = taken(&at, m * m);
  w->held = taken(&at, m * m);
  w->rise = taken(&at, m * m);
  w->link = taken(&at, m * m);
}

/** Each thing's numbers and its kind as its box bears it out (classes.BORNE), its flags after. */
static void laid(const wbc_classes_t *k, int n, const double *c, const double *h, const double *R,
                 const double *clear, const int32_t *code, const int32_t *flags, work_t *w)
{
  for (int a = 0; a < n; a++)
  {
    const double *ca = c + 3 * a, *ha = h + 3 * a, *Ra = R + 9 * a;
    double        absR[9];
    int32_t       kc = code[a], kf = flags[a];

    for (int j = 0; j < 9; j++)
    {
      absR[j] = fabs(Ra[j]);
    }
    for (int j = 0; j < 3; j++)
    {
      w->e[3 * a + j] = npsumof(absR + 3 * j, 1, ha, 1, 3, k->lane);
      w->lo[3 * a + j] = ca[j] - w->e[3 * a + j];
      w->hi[3 * a + j] = ca[j] + w->e[3 * a + j];
    }
    w->top[a] = ca[1] + w->e[3 * a + 1];
    w->bottom[a] = ca[1] - w->e[3 * a + 1];
    if ((kf & WBC_CLASSES_DOOR) != 0 && !(w->bottom[a] <= k->over && w->top[a] > k->brace))
    {
      kc = k->timber;
      kf = k->timber_flags;
    }
    if ((kf & WBC_CLASSES_BED) != 0 && !(k->bed0 <= w->top[a] && w->top[a] <= k->bed1))
    {
      kc = k->glass;
      kf = k->glass_flags;
    }
    w->kcode[a] = kc;
    w->kflags[a] = kf;
    w->raw[a] = (kf & WBC_CLASSES_RAW) != 0 ? 1U : 0U;
    w->see[a] = (kf & WBC_CLASSES_SEE) != 0 ? 1U : 0U;
    w->wood[a] = (w->raw[a] != 0U || (kf & WBC_CLASSES_FURN) != 0) ? 1U : 0U;
    w->lng[a] = 2.0 * npmaximum(ha[0], ha[2]);
    w->shrt[a] = 2.0 * npminimum(ha[0], ha[2]);
    w->surface[a] = (w->raw[a] != 0U && clear[a] > 0.0) ? 1U : 0U;
    w->wide[a] = w->shrt[a] > k->thin_eps ? 1U : 0U;
    w->top_[a] = (w->wood[a] != 0U && clear[a] > 0.0 && w->wide[a] != 0U && w->top[a] > k->flush) ? 1U : 0U;
    for (int j = 0; j < 3; j++)
    {
      w->mid[3 * a + j] = 0.5 * (w->lo[3 * a + j] + w->hi[3 * a + j]);
      w->hlf[3 * a + j] = 0.5 * (w->hi[3 * a + j] - w->lo[3 * a + j]);
    }
    w->longest[a] = 0.0;
    w->backed[a] = 0U;
  }
}

/** The pairs: which meet a cell apart (`_beside`), which lie within another's footprint, which are held by a top -
    on its edge, under it, a back at it -, each top's backs and the longest opaque piece under it. */
static void paired(const wbc_classes_t *k, int n, const double *c, work_t *w)
{
  for (int a = 0; a < n; a++)
  {
    const double *ea = w->e + 3 * a;

    w->held_any[a] = 0U;
    for (int b = 0; b < n; b++)
    {
      const double *eb = w->e + 3 * b;
      const double  gx = fabs(c[3 * a] - c[3 * b]), gz = fabs(c[3 * a + 2] - c[3 * b + 2]);
      const size_t  ab = (size_t)a * (size_t)n + (size_t)b;
      unsigned char within, beside, pair, on, under, backs;

      within = (a != b && gx + ea[0] <= (eb[0] + k->cell) + k->eps && gz + ea[2] <= (eb[2] + k->cell) + k->eps)
                   ? 1U : 0U;
      beside = (a != b && gx <= ((ea[0] + eb[0]) + k->cell) + k->eps && gz <= ((ea[2] + eb[2]) + k->cell) + k->eps)
                   ? 1U : 0U;
      w->within[ab] = within;
      w->beside[ab] = beside;
      pair = (w->wood[a] != 0U && w->see[a] == 0U && w->top_[a] == 0U && w->top_[b] != 0U
              && w->kcode[a] == w->kcode[b]) ? 1U : 0U;
      on = (pair != 0U && beside != 0U && w->bottom[a] >= (w->top[b] - k->cell) - k->eps
            && w->bottom[a] <= w->top[b] + k->over) ? 1U : 0U;
      under = (pair != 0U && within != 0U && w->top[a] <= (w->bottom[b] + k->cell) + k->eps) ? 1U : 0U;
      backs = (pair != 0U && beside != 0U && w->bottom[a] <= (w->top[b] + k->cell) + k->eps
               && w->top[a] >= w->top[b] + k->lumbar && w->top[a] <= k->brace_half
               && w->lng[a] <= (w->lng[b] + k->cell) + k->eps) ? 1U : 0U;
      w->held[ab] = (on != 0U || under != 0U || backs != 0U) ? 1U : 0U;
      if (w->held[ab] != 0U)
      {
        w->held_any[a] = 1U;
      }
      if (backs != 0U)
      {
        w->backed[b] = 1U;
      }
      if (under != 0U && w->surface[a] == 0U)
      {
        w->longest[b] = npmaximum(w->longest[b], w->lng[a]);
      }
    }
  }
}

/** The tops the robot steps onto, from the lowest slab up each rise (`steps`), and what each thing's features say. */
static void featured(const wbc_classes_t *k, int n, work_t *w)
{
  int changed = 1;

  for (int a = 0; a < n; a++)
  {
    w->free_[a] = (w->surface[a] != 0U && w->held_any[a] == 0U) ? 1U : 0U;
    w->solid[a] = (w->bottom[a] <= k->flush || w->longest[a] >= 0.5 * w->lng[a]) ? 1U : 0U;
    w->seat[a] = (w->top[a] >= k->seat0 && w->top[a] <= k->seat1) ? 1U : 0U;
  }
  for (int a = 0; a < n; a++)
  {
    w->steps[a] = (w->free_[a] != 0U && w->top[a] <= k->step) ? 1U : 0U;
    w->framed[a] = 0U;
    w->walled[a] = 0U;
    for (int b = 0; b < n; b++)
    {
      const size_t ab = (size_t)a * (size_t)n + (size_t)b;

      w->rise[ab] = (w->beside[ab] != 0U && w->free_[a] != 0U && w->free_[b] != 0U
                     && fabs(w->top[a] - w->top[b]) <= k->step) ? 1U : 0U;
      if (w->beside[ab] != 0U && w->see[b] != 0U)
      {
        w->framed[a] = 1U;
      }
      if (w->beside[ab] != 0U && w->see[b] == 0U && w->lng[b] > k->thin_eps && w->bottom[b] <= k->brace
          && w->top[b] > k->brace)
      {
        w->walled[a] = 1U;
      }
    }
  }
  while (changed != 0)
  {
    changed = 0;
    for (int a = 0; a < n; a++)
    {
      unsigned char more = w->steps[a];

      for (int b = 0; b < n && more == 0U; b++)
      {
        more = (w->rise[(size_t)a * (size_t)n + (size_t)b] != 0U && w->steps[b] != 0U) ? 1U : 0U;
      }
      w->more[a] = more;
    }
    for (int a = 0; a < n; a++)
    {
      if (w->more[a] != w->steps[a])
      {
        changed = 1;
      }
      w->steps[a] = w->more[a];
    }
  }
  for (int a = 0; a < n; a++)
  {
    w->flight[a] = 0U;
    for (int b = 0; b < n; b++)
    {
      if (w->rise[(size_t)a * (size_t)n + (size_t)b] != 0U && w->steps[b] != 0U
          && fabs(w->top[a] - w->top[b]) > k->flush)
      {
        w->flight[a] = 1U;
      }
    }
  }
}

/** classes.TOPS: a top's class, the first that holds. */
static int32_t top_of(const wbc_classes_t *k, const work_t *w, int a)
{
  const double twice = 2.0 * npmaximum(w->shrt[a], k->stance);
  const int    wide = w->wide[a] != 0U, seat = w->seat[a] != 0U, backed = w->backed[a] != 0U;
  const int    steps = w->steps[a] != 0U, solid = w->solid[a] != 0U;

  if (fabs(w->top[a]) <= k->flush)
  {
    return WBC_CLASS_FLOOR;
  }
  if (steps && w->shrt[a] < k->stance && w->flight[a] != 0U)
  {
    return WBC_CLASS_STAIRS;
  }
  if (steps && w->shrt[a] < k->stance)
  {
    return WBC_CLASS_STEP;
  }
  if (steps || (solid && w->shrt[a] >= k->stance && w->top[a] <= k->brace))
  {
    return WBC_CLASS_PLATFORM;
  }
  if (wide && seat && backed && w->lng[a] >= twice)
  {
    return WBC_CLASS_SOFA;
  }
  if (wide && seat && backed)
  {
    return WBC_CLASS_CHAIR;
  }
  if (wide && ((w->top[a] > k->lap && w->top[a] <= k->brace) || (seat && w->shrt[a] >= k->stance)))
  {
    return WBC_CLASS_TABLE;
  }
  if (wide && seat && w->lng[a] >= twice)
  {
    return WBC_CLASS_BENCH;
  }
  if (wide && seat)
  {
    return WBC_CLASS_STOOL;
  }
  if (wide && solid && w->top[a] > k->brace)
  {
    return WBC_CLASS_CABINET;
  }
  if (wide && w->top[a] > k->brace)
  {
    return WBC_CLASS_SHELF;
  }
  return WBC_CLASS_THING;
}

/** classes.FACES: the class of a thing no top, the first that holds. */
static int32_t face_of(const wbc_classes_t *k, const work_t *w, int a)
{
  const int see = w->see[a] != 0U;

  if ((see || (w->framed[a] != 0U && w->lng[a] <= k->thin_eps)) && w->top[a] > k->brace)
  {
    return WBC_CLASS_WINDOW;
  }
  if (see)
  {
    return WBC_CLASS_RAILING;
  }
  if (w->bottom[a] > k->overhead && w->shrt[a] > k->thin_eps)
  {
    return WBC_CLASS_CEILING;
  }
  if (w->top[a] > k->brace && (w->lng[a] > k->thin_eps || w->walled[a] != 0U))
  {
    return WBC_CLASS_WALL;
  }
  if (w->top[a] > k->brace)
  {
    return WBC_CLASS_POST;
  }
  return WBC_CLASS_THING;
}

/** A thing's part in a building (WBC_CLASSES_SHELL, WBC_CLASSES_OPEN) by its told class `cls` (one of the enum) or
    its kind's `flags`: a wall's and a window's, a door's as its look tells it. */
static int32_t housed(int32_t cls, int32_t flags)
{
  if (cls >= 0)
  {
    return (cls == WBC_CLASS_WALL || cls == WBC_CLASS_WINDOW ? WBC_CLASSES_SHELL : 0)
           | (cls == WBC_CLASS_WINDOW ? WBC_CLASSES_OPEN : 0);
  }
  return flags & (WBC_CLASSES_SHELL | WBC_CLASSES_OPEN);
}

int wbc_classes_houses(const wbc_classes_t *k, int n, const double *lo, const double *hi, const double *mid,
                       const double *hlf, const int32_t *part, int32_t *label, int32_t *low, unsigned char *link,
                       double *blo, double *bhi, int32_t *of)
{
  int changed = 1, nb = 0;

  for (int a = 0; a < n; a++)
  {
    label[a] = (part[a] & WBC_CLASSES_SHELL) != 0 ? a : n;
    for (int b = 0; b < n; b++)
    {
      unsigned char near_ = 1U, plane = 0U;

      for (int j = 0; j < 3; j += 2)
      {
        const double ma = mid[3 * a + j], mb = mid[3 * b + j], ha = hlf[3 * a + j], hb = hlf[3 * b + j];

        if (!((fabs(ma - mb) - ha) - hb <= k->far_eps))
        {
          near_ = 0U;
        }
        if (ha <= k->half_thin_eps && hb <= k->half_thin_eps && fabs(ma - mb) <= k->cell_eps)
        {
          plane = 1U;
        }
      }
      link[(size_t)a * (size_t)n + (size_t)b] = ((near_ != 0U || plane != 0U) && (part[b] & WBC_CLASSES_SHELL) != 0)
                                                    ? 1U : 0U;
    }
  }
  while (changed != 0)
  {
    changed = 0;
    for (int a = 0; a < n; a++)
    {
      int32_t least = n;

      if ((part[a] & WBC_CLASSES_SHELL) != 0)
      {
        least = label[a];
        for (int b = 0; b < n; b++)
        {
          if (link[(size_t)a * (size_t)n + (size_t)b] != 0U && label[b] < least)
          {
            least = label[b];
          }
        }
      }
      low[a] = least;
    }
    for (int a = 0; a < n; a++)
    {
      if (low[a] != label[a])
      {
        changed = 1;
      }
      label[a] = low[a];
    }
  }
  for (int32_t g = 0; g < n; g++)
  {
    int     opened = 0, count = 0;
    double *bl = blo + 3 * nb, *bh = bhi + 3 * nb;

    for (int a = 0; a < n && opened == 0; a++)
    {
      opened = (label[a] == g && (part[a] & WBC_CLASSES_OPEN) != 0) ? 1 : 0;
    }
    if (opened == 0)
    {
      continue;
    }
    for (int a = 0; a < n; a++)
    {
      if (label[a] != g)
      {
        continue;
      }
      for (int j = 0; j < 3; j++)
      {
        bl[j] = count == 0 ? lo[3 * a + j] : npminimum(bl[j], lo[3 * a + j]);
        bh[j] = count == 0 ? hi[3 * a + j] : npmaximum(bh[j], hi[3 * a + j]);
      }
      count++;
    }
    if (count >= 2 && bh[0] - bl[0] > k->thin_eps && bh[2] - bl[2] > k->thin_eps)
    {
      of[nb] = g;
      nb++;
    }
  }
  return nb;
}

/** What lies by a building (classes.of's last word): outside every building, by one a floor its veranda, a platform
    its terrace, a floor apart or a platform sunk the ground. */
static void beside_built(const wbc_classes_t *k, int n, int nb, const double *c, const work_t *w, int32_t *cls)
{
  for (int a = 0; a < n; a++)
  {
    const int     floor = cls[a] == WBC_CLASS_FLOOR, platform = cls[a] == WBC_CLASS_PLATFORM;
    const double *ca = c + 3 * a, *ea = w->e + 3 * a;
    unsigned char in = 0U, by = 0U;

    for (int b = 0; b < nb; b++)
    {
      const double *lo = w->blo + 3 * b, *hi = w->bhi + 3 * b;

      if (ca[0] >= lo[0] && ca[0] <= hi[0] && ca[2] >= lo[2] && ca[2] <= hi[2])
      {
        in = 1U;
      }
      if (ca[0] + ea[0] >= (lo[0] - k->cell) - k->eps && ca[0] - ea[0] <= (hi[0] + k->cell) + k->eps
          && ca[2] + ea[2] >= (lo[2] - k->cell) - k->eps && ca[2] - ea[2] <= (hi[2] + k->cell) + k->eps)
      {
        by = 1U;
      }
    }
    w->out[a] = in == 0U ? 1U : 0U;
    w->by[a] = (w->out[a] != 0U && by != 0U) ? 1U : 0U;
    if (w->by[a] != 0U && floor)
    {
      cls[a] = WBC_CLASS_VERANDA;
    }
    else if (w->by[a] != 0U && platform && w->top[a] >= -k->flush)
    {
      cls[a] = WBC_CLASS_TERRACE;
    }
    else if (w->out[a] != 0U && ((floor && w->by[a] == 0U) || (platform && w->top[a] < -k->flush)))
    {
      cls[a] = WBC_CLASS_GROUND;
    }
  }
}

int wbc_classes_of(const wbc_classes_t *k, int n, const double *c, const double *h, const double *R,
                   const double *clear, const int32_t *code, const int32_t *flags, int32_t *cls, int32_t *of,
                   int32_t *whole, unsigned char *w_)
{
  work_t w;
  int    nb;

  if (n <= 0 || n > WBC_CLASSES_BOXES)
  {
    return n == 0 ? 0 : -1;
  }
  carve(&w, n, w_);
  laid(k, n, c, h, R, clear, code, flags, &w);
  paired(k, n, c, &w);
  featured(k, n, &w);
  for (int a = 0; a < n; a++)
  {
    int32_t name = -1;

    w.tops[a] = top_of(k, &w, a);
    w.faces[a] = face_of(k, &w, a);
    if (w.surface[a] != 0U)
    {
      name = w.tops[a];
    }
    if (w.raw[a] != 0U && !(w.surface[a] != 0U && w.tops[a] != WBC_CLASS_THING))
    {
      name = w.faces[a];
    }
    w.name[a] = name;
    whole[a] = a;
    for (int b = 0; b < n && w.held_any[a] != 0U; b++)
    {
      if (w.held[(size_t)a * (size_t)n + (size_t)b] != 0U)
      {
        whole[a] = b;
        break;
      }
    }
  }
  for (int a = 0; a < n; a++)
  {
    cls[a] = w.name[whole[a]];
    of[a] = w.kcode[whole[a]];
    w.part[a] = housed(cls[a], w.kflags[whole[a]]);
  }
  nb = wbc_classes_houses(k, n, w.lo, w.hi, w.mid, w.hlf, w.part, w.label, w.low, w.link, w.blo, w.bhi, w.of);
  if (nb > 0)
  {
    beside_built(k, n, nb, c, &w, cls);
  }
  return 0;
}
