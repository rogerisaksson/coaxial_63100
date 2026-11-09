/** quick_step.c - her step's ask and senses laid into its struct, its C called and its answer made in one C call
 * (tools.cores.step's `_stepped`): its pointed write, her contacts, the ask's fields as numpy's assignments set them
 * (its defaults `_stepped`'s), the call - the interpreter let go, as ctypes lets it go - and its dict, the same objects;
 * a sensing not her compiled sensing's last, a value of another kind or layout its miss, Python's path then. The
 * host's alone. */
#define NO_IMPORT_ARRAY
#include "quick.h"
#include <numpy/arrayscalars.h>

#include <math.h>
#include <string.h>

/* The struct's fields, their offsets laid in this order (step.py's FIELDS). */
enum
{
  F_N, F_ND, F_NC, F_M, F_CJ, F_CP, F_CDJV, F_CF, F_CPATCH, F_CBODY, F_CGIVE, F_STANCE, F_LET_GO, F_COM_ACC, F_NTURNS,
  F_TURN, F_KDOT_GIVEN, F_KDOT, F_SETTLE, F_EDGE, F_MUS, F_RESTS, F_SLIDE, F_FZ_CAP, F_SLID_CAP, F_HAND_CAP, F_LET_CAP,
  F_LET_HOLD, F_OPEN_KP, F_OPEN_KD, F_FREED, F_STRIDES, F_BORNE, F_HOLD, F_SAT, F_CHAIN_VX, F_CHAIN_FREE, F_SWING,
  F_SWING_AT, F_SWING_SPEED, F_SWING_ACC, F_SWING_QUAT, F_FOLD, F_FOLD_KNEE, F_FOLD_RATE, F_FOLD_ACC, F_KNEE_RATE, F_WEP,
  F_POSTURE, F_DERATE, F_CLAMP, F_WEP_NM, F_MASS, F_LO, F_HI, F_TAU, F_QACC, F_BEARS, F_COP, F_COP_N, F_COP_NONE,
  F_OVER, F_FORCES, F_LOAD, F_SCALE, F_RESIDUAL, F_HANG, F_HANG_RATE, F_CN, F_ROWS_FREE, F_SOLES_FREE, F_SOLES, FIELDS
};

/* The keys and attributes it reads and its answer's keys, interned once. */
static const char *const NAMES[] = {
  "M", "contacts", "J", "p", "dJv", "f", "patch", "body", "give", "stance", "let_go", "com_acc", "turns", "kdot",
  "settle", "edge", "mu", "rests", "slide", "fz_cap", "slid_cap", "hand_cap", "let_cap", "let_hold", "open", "strides",
  "borne", "hold", "sat", "twist", "free", "swing", "fold", "knee_rate", "wep", "posture", "derate", "lo", "hi",
  "clamp", "mass", "load", "sensed", "last", "c", "placed", "tau", "qacc", "bears", "cop", "over", "slack", "held",
  "scale", "residual", "hang", "hang_rate", "n", "soles_free", "soles", "rows_free"};
enum
{
  K_M, K_CONTACTS, K_J, K_P, K_DJV, K_F, K_PATCH, K_BODY, K_GIVE, K_STANCE, K_LET_GO, K_COM_ACC, K_TURNS, K_KDOT,
  K_SETTLE, K_EDGE, K_MU, K_RESTS, K_SLIDE, K_FZ_CAP, K_SLID_CAP, K_HAND_CAP, K_LET_CAP, K_LET_HOLD, K_OPEN, K_STRIDES,
  K_BORNE, K_HOLD, K_SAT, K_TWIST, K_FREE, K_SWING, K_FOLD, K_KNEE_RATE, K_WEP, K_POSTURE, K_DERATE, K_LO, K_HI,
  K_CLAMP, K_MASS, K_LOAD, K_SENSED, K_LAST, K_C, K_PLACED, K_TAU, K_QACC, K_BEARS, K_COP, K_OVER, K_SLACK, K_HELD,
  K_SCALE, K_RESIDUAL, K_HANG, K_HANG_RATE, K_N, K_SOLES_FREE, K_SOLES, K_ROWS_FREE, KEYS
};
static PyObject *K[KEYS];

typedef struct
{
  char      *k;                           /* the struct's memory, its step's C */
  int      (*step)(void *);
  Py_ssize_t off[FIELDS], most, pointed, npointed, nd;
  double     mu;                          /* the ask's friction where none is given (machine.stack.wbc.MU) */
  double    *spread;                      /* a derate, lo or hi given as a number, broadcast: three of nd each */
  PyObject  *keys, *miss, *singular;      /* her sensing's keys pointed at (step.SENSED), MISS, LinAlgError */
} step_t;

#define FAT(t, f) ((t)->k + (t)->off[f])
#define FI(t, f) ((int *)FAT(t, f))
#define FD(t, f) ((double *)FAT(t, f))
#define FP(t, f) ((void **)FAT(t, f))

static void dropped(PyObject *capsule)
{
  step_t *t = PyCapsule_GetPointer(capsule, "quick.step");
  if (t == NULL)
    return;
  Py_XDECREF(t->keys);
  Py_XDECREF(t->miss);
  Py_XDECREF(t->singular);
  PyMem_Free(t->spread);
  PyMem_Free(t);
}

/* An ndarray of native float64, C-contiguous and aligned: its data; else NULL. */
static double *f64(PyObject *o)
{
  if (o == NULL || !PyArray_Check(o))
    return NULL;
  PyArrayObject *a = (PyArrayObject *)o;
  if (PyArray_TYPE(a) != NPY_DOUBLE || !PyArray_IS_C_CONTIGUOUS(a) || !PyArray_ISALIGNED(a) || !PyArray_ISNOTSWAPPED(a))
    return NULL;
  return (double *)PyArray_DATA(a);
}

/* `o` as float() reads it where it is a number - Python's, numpy's or a 0-d array -: 1; else 0, no error set. */
static int real(PyObject *o, double *x)
{
  if (o == NULL || !(PyFloat_Check(o) || PyLong_Check(o) || PyArray_IsScalar(o, Number) || PyArray_IsScalar(o, Bool)
                     || (PyArray_Check(o) && PyArray_NDIM((PyArrayObject *)o) == 0)))
    return 0;
  *x = PyFloat_AsDouble(o);
  if (*x == -1.0 && PyErr_Occurred())
  {
    PyErr_Clear();
    return 0;
  }
  return 1;
}

/* `o` into the n doubles at `d` as numpy's `d[:] = o` sets them, `d` laid (n / cols, cols) where `cols`: an ndarray of
 * float64 of that shape, a number everywhere, a sequence of n numbers where `d` is flat; else 0. */
static int fill(PyObject *o, double *d, Py_ssize_t n, Py_ssize_t cols)
{
  double x;

  if (o != NULL && PyArray_Check(o) && PyArray_NDIM((PyArrayObject *)o) > 0)
  {
    PyArrayObject  *a = (PyArrayObject *)o;
    const npy_intp *dims = PyArray_DIMS(a);
    const int       laid = cols ? (PyArray_NDIM(a) == 2 && dims[0] == n / cols && dims[1] == cols)
                                : (PyArray_NDIM(a) == 1 && dims[0] == n);
    const double   *src = f64(o);

    if (!laid || src == NULL)
      return 0;
    memcpy(d, src, (size_t)n * sizeof(double));
    return 1;
  }
  if (real(o, &x))
  {
    for (Py_ssize_t i = 0; i < n; i++)
      d[i] = x;
    return 1;
  }
  if (cols || o == NULL || !(PyTuple_Check(o) || PyList_Check(o)) || PySequence_Fast_GET_SIZE(o) != n)
    return 0;
  for (Py_ssize_t i = 0; i < n; i++)
    if (!real(PySequence_Fast_GET_ITEM(o, i), &d[i]))
      return 0;
  return 1;
}

/* [bool(x) for x in o] into two ints, `o` a tuple or list of two; else 0. */
static int truths(PyObject *o, int *d)
{
  if (o == NULL || !(PyTuple_Check(o) || PyList_Check(o)) || PySequence_Fast_GET_SIZE(o) != 2)
    return 0;
  for (Py_ssize_t i = 0; i < 2; i++)
  {
    const int b = PyObject_IsTrue(PySequence_Fast_GET_ITEM(o, i));
    if (b < 0)
    {
      PyErr_Clear();
      return 0;
    }
    d[i] = b;
  }
  return 1;
}

/* An integer as int() takes it (Python's or numpy's): 1; else 0, no error set. */
static int whole(PyObject *o, int *d)
{
  if (o == NULL || !(PyLong_Check(o) || PyArray_IsScalar(o, Integer)))
    return 0;
  const long v = PyLong_AsLong(o);
  if (v == -1 && PyErr_Occurred())
  {
    PyErr_Clear();
    return 0;
  }
  *d = (int)v;
  return 1;
}

/* `d`'s value for key `k` (borrowed), NULL where it has none - an error raised by the lookup cleared, `*bad` set. */
static PyObject *got(PyObject *d, int k, int *bad)
{
  PyObject *v = PyDict_GetItemWithError(d, K[k]);
  if (v == NULL && PyErr_Occurred())
  {
    PyErr_Clear();
    *bad = 1;
  }
  return v;
}

/* `ask`'s `k` into n doubles (`fill`), `none` everywhere where it has none. */
static int filled(PyObject *ask, int k, double *d, Py_ssize_t n, Py_ssize_t cols, double none, int *bad)
{
  PyObject *o = got(ask, k, bad);
  if (o == NULL)
  {
    for (Py_ssize_t i = 0; i < n; i++)
      d[i] = none;
    return !*bad;
  }
  return fill(o, d, n, cols);
}

/* `ask`'s `k` as float() reads it, `none` where it has none. */
static int number(PyObject *ask, int k, double *x, double none, int *bad)
{
  PyObject *o = got(ask, k, bad);
  if (o == NULL)
    *x = none;
  return o == NULL ? !*bad : real(o, x);
}

/* A derate, lo or hi (`_spread`): an ndarray of nd float64 pointed at, a number broadcast into `buf`; else NULL. */
static double *spread(PyObject *o, double *buf, Py_ssize_t nd, double none)
{
  double x = none;
  if (o != NULL && PyArray_Check(o) && PyArray_NDIM((PyArrayObject *)o) == 1)
    return PyArray_DIMS((PyArrayObject *)o)[0] == nd ? f64(o) : NULL;
  if (o != NULL && !real(o, &x))
    return NULL;
  for (Py_ssize_t i = 0; i < nd; i++)
    buf[i] = x;
  return buf;
}

/* Her sensing pointed at, `_stepped`'s pointed write: her compiled sensing's last, `s`'s the same objects. */
static int pointed(step_t *t, PyObject *kobj, PyObject *s)
{
  PyObject *sensed = PyObject_GetAttr(kobj, K[K_SENSED]), *placed = PyObject_GetAttr(kobj, K[K_PLACED]);
  PyObject *last = sensed != NULL ? PyObject_GetAttr(sensed, K[K_LAST]) : NULL;
  PyObject *c = sensed != NULL ? PyObject_GetAttr(sensed, K[K_C]) : NULL;
  int       ok = last != NULL && c != NULL && placed != NULL && PyTuple_Check(last) && PyTuple_GET_SIZE(last) == 2
           && PyDict_Check(PyTuple_GET_ITEM(last, 0)) && PyTuple_Check(placed) && PyTuple_GET_SIZE(placed) == 2
           && PyTuple_GET_ITEM(placed, 0) == c && PyArray_Check(PyTuple_GET_ITEM(placed, 1));

  if (ok)
  {
    PyObject      *was = PyTuple_GET_ITEM(last, 0);
    PyArrayObject *offs = (PyArrayObject *)PyTuple_GET_ITEM(placed, 1);
    const uint64_t base = PyLong_AsUnsignedLongLong(PyTuple_GET_ITEM(last, 1));

    ok = !PyErr_Occurred() && PyArray_TYPE(offs) == NPY_UINT64 && PyArray_IS_C_CONTIGUOUS(offs)
         && PyArray_SIZE(offs) == t->npointed;
    for (Py_ssize_t i = 0; ok && i < PyTuple_GET_SIZE(t->keys); i++)
    {
      PyObject *key = PyTuple_GET_ITEM(t->keys, i);
      PyObject *mine = PyDict_GetItemWithError(s, key), *theirs = PyDict_GetItemWithError(was, key);
      ok = !PyErr_Occurred() && mine == theirs;
    }
    if (ok)
    {
      uint64_t       *words = (uint64_t *)(void *)t->k + t->pointed;
      const uint64_t *off = (const uint64_t *)PyArray_DATA(offs);
      for (Py_ssize_t i = 0; i < t->npointed; i++)
        words[i] = off[i] + base;
    }
  }
  PyErr_Clear();
  Py_XDECREF(sensed);
  Py_XDECREF(placed);
  Py_XDECREF(last);
  Py_XDECREF(c);
  return ok;
}

/* Her contacts (`s.get('contacts', ())`) pointed at, their patches, bodies and give: 0 where one is not as laid. */
static int touched(step_t *t, PyObject *s)
{
  static const int ptrs[5][2] = {{K_J, F_CJ}, {K_P, F_CP}, {K_DJV, F_CDJV}, {K_F, F_CF}, {K_N, F_CN}};
  int              bad = 0;
  PyObject        *contacts = got(s, K_CONTACTS, &bad);
  Py_ssize_t       nc = 0;
  int              ok = !bad && (contacts == NULL || ((PyList_Check(contacts) || PyTuple_Check(contacts))
                                                       && (nc = PySequence_Fast_GET_SIZE(contacts)) <= t->most));

  for (Py_ssize_t at = 0; ok && at < nc; at++)
  {
    PyObject *c = PySequence_Fast_GET_ITEM(contacts, at), *o;
    double    give = 0.0;
    int       patch = 0, body = 0;

    ok = PyDict_Check(c);
    for (int j = 0; ok && j < 5; j++)
      ok = (FP(t, ptrs[j][1])[at] = f64(got(c, ptrs[j][0], &bad))) != NULL;
    ok = ok && (((o = got(c, K_PATCH, &bad)) == NULL && !bad) || whole(o, &patch));
    ok = ok && (((o = got(c, K_BODY, &bad)) == NULL && !bad) || whole(o, &body));
    ok = ok && (((o = got(c, K_GIVE, &bad)) == NULL && !bad) || real(o, &give));
    FI(t, F_CPATCH)[at] = patch;
    FI(t, F_CBODY)[at] = body;
    FD(t, F_CGIVE)[at] = give;
  }
  *FI(t, F_NC) = (int)nc;
  return ok;
}

/* The ask's swings and folds (`swing`, `fold`: {side: ...}): 0 where one is not as laid. */
static int sides(step_t *t, PyObject *ask)
{
  int       bad = 0;
  PyObject *swing = got(ask, K_SWING, &bad), *fold = got(ask, K_FOLD, &bad);
  int       ok = !bad && (swing == NULL || PyDict_Check(swing)) && (fold == NULL || PyDict_Check(fold));

  for (long side = 0; ok && side < 2; side++)
  {
    PyObject *key = PyLong_FromLong(side);
    PyObject *sw = (key != NULL && swing != NULL) ? PyDict_GetItemWithError(swing, key) : NULL;
    PyObject *fo = (key != NULL && fold != NULL) ? PyDict_GetItemWithError(fold, key) : NULL;
    double    v[3];

    ok = key != NULL && !PyErr_Occurred();
    Py_XDECREF(key);
    FI(t, F_SWING)[side] = sw != NULL;
    FI(t, F_FOLD)[side] = fo != NULL;
    ok = ok && (sw == NULL || (PyTuple_Check(sw) && PyTuple_GET_SIZE(sw) == 4
                               && fill(PyTuple_GET_ITEM(sw, 0), FD(t, F_SWING_AT) + 3 * side, 3, 0)
                               && fill(PyTuple_GET_ITEM(sw, 1), FD(t, F_SWING_SPEED) + 3 * side, 3, 0)
                               && fill(PyTuple_GET_ITEM(sw, 2), FD(t, F_SWING_ACC) + 3 * side, 3, 0)
                               && fill(PyTuple_GET_ITEM(sw, 3), FD(t, F_SWING_QUAT) + 4 * side, 4, 0)));
    ok = ok && (fo == NULL || (PyTuple_Check(fo) && PyTuple_GET_SIZE(fo) == 3 && real(PyTuple_GET_ITEM(fo, 0), &v[0])
                               && real(PyTuple_GET_ITEM(fo, 1), &v[1]) && real(PyTuple_GET_ITEM(fo, 2), &v[2])));
    if (ok && fo != NULL)
    {
      FD(t, F_FOLD_KNEE)[side] = v[0];
      FD(t, F_FOLD_RATE)[side] = v[1];
      FD(t, F_FOLD_ACC)[side] = v[2];
    }
  }
  PyErr_Clear();
  return ok;
}

/* The ask's flags, numbers and small arrays, `_stepped`'s defaults where it has none: 0 where one is not as laid. */
static int asked(step_t *t, PyObject *ask)
{
  int       bad = 0;
  PyObject *turns = got(ask, K_TURNS, &bad), *kdot = got(ask, K_KDOT, &bad), *let_go = got(ask, K_LET_GO, &bad);
  PyObject *open = got(ask, K_OPEN, &bad), *twist = got(ask, K_TWIST, &bad), *free_ = got(ask, K_FREE, &bad);
  PyObject *wep = got(ask, K_WEP, &bad);
  Py_ssize_t nturns = 0;
  int       ok = !bad && truths(got(ask, K_STANCE, &bad), FI(t, F_STANCE))
           && fill(got(ask, K_COM_ACC, &bad), FD(t, F_COM_ACC), 3, 0);

  ok = ok && (turns == NULL || ((PyTuple_Check(turns) || PyList_Check(turns)) && (nturns = PySequence_Fast_GET_SIZE(turns)) <= 2));
  for (Py_ssize_t i = 0; ok && i < nturns; i++)
    ok = fill(PySequence_Fast_GET_ITEM(turns, i), FD(t, F_TURN) + 4 * i, 4, 0);
  *FI(t, F_NTURNS) = (int)nturns;
  *FI(t, F_KDOT_GIVEN) = kdot != NULL && kdot != Py_None;
  ok = ok && (kdot == NULL || kdot == Py_None || fill(kdot, FD(t, F_KDOT), 3, 0));
  if (let_go == NULL)
    FI(t, F_LET_GO)[0] = FI(t, F_LET_GO)[1] = 0;
  ok = ok && (let_go == NULL || truths(let_go, FI(t, F_LET_GO)));
  ok = ok && filled(ask, K_SETTLE, FD(t, F_SETTLE), 12, 6, 0.0, &bad) && filled(ask, K_EDGE, FD(t, F_EDGE), 2, 0, 0.0, &bad)
       && filled(ask, K_MU, FD(t, F_MUS), 2, 0, t->mu, &bad) && filled(ask, K_RESTS, FD(t, F_RESTS), 2, 0, 0.0, &bad)
       && filled(ask, K_SLIDE, FD(t, F_SLIDE), 3, 0, 0.0, &bad)
       && number(ask, K_FZ_CAP, FD(t, F_FZ_CAP), INFINITY, &bad) && number(ask, K_SLID_CAP, FD(t, F_SLID_CAP), INFINITY, &bad)
       && number(ask, K_HAND_CAP, FD(t, F_HAND_CAP), 0.0, &bad)
       && filled(ask, K_LET_CAP, FD(t, F_LET_CAP), 2, 0, INFINITY, &bad)
       && filled(ask, K_LET_HOLD, FD(t, F_LET_HOLD), 2, 0, 0.0, &bad);
  if (open == NULL)
    *FD(t, F_OPEN_KP) = *FD(t, F_OPEN_KD) = FD(t, F_FREED)[0] = FD(t, F_FREED)[1] = 0.0;
  ok = ok && (open == NULL || ((PyTuple_Check(open) || PyList_Check(open)) && PySequence_Fast_GET_SIZE(open) == 3
                               && real(PySequence_Fast_GET_ITEM(open, 0), FD(t, F_OPEN_KP))
                               && real(PySequence_Fast_GET_ITEM(open, 1), FD(t, F_OPEN_KD))
                               && fill(PySequence_Fast_GET_ITEM(open, 2), FD(t, F_FREED), 2, 0)));
  ok = ok && number(ask, K_STRIDES, FD(t, F_STRIDES), 0.0, &bad) && number(ask, K_BORNE, FD(t, F_BORNE), 0.0, &bad)
       && number(ask, K_HOLD, FD(t, F_HOLD), 1.0, &bad) && number(ask, K_SAT, FD(t, F_SAT), 0.0, &bad);
  if (twist == NULL)
    *FD(t, F_CHAIN_VX) = 0.0;
  ok = ok && (twist == NULL || ((PyTuple_Check(twist) || PyList_Check(twist)) && PySequence_Fast_GET_SIZE(twist) > 0
                                && real(PySequence_Fast_GET_ITEM(twist, 0), FD(t, F_CHAIN_VX))));
  if (free_ == NULL)
    FI(t, F_CHAIN_FREE)[0] = FI(t, F_CHAIN_FREE)[1] = 0;
  ok = ok && (free_ == NULL || truths(free_, FI(t, F_CHAIN_FREE)));
  ok = ok && filled(ask, K_KNEE_RATE, FD(t, F_KNEE_RATE), 2, 0, 0.0, &bad);
  ok = ok && filled(ask, K_HANG, FD(t, F_HANG), 2, 0, 0.0, &bad) && number(ask, K_HANG_RATE, FD(t, F_HANG_RATE), 0.0, &bad);
  if (ok)
  {
    const int on = wep == NULL ? 0 : PyObject_IsTrue(wep);
    ok = on >= 0;
    *FI(t, F_WEP) = on > 0;
  }
  PyErr_Clear();
  return ok && !bad;
}

/* Her ask's arrays and her body's (`_stepped`'s): posture, derate, clamp, WEP, mass, lo and hi; 0 where not as laid. */
static int arrays(step_t *t, PyObject *b, PyObject *ask)
{
  int       bad = 0;
  PyObject *clamp = PyObject_GetAttr(b, K[K_CLAMP]), *wep = PyObject_GetAttr(b, K[K_WEP]);
  PyObject *mass = PyObject_GetAttr(b, K[K_MASS]);
  double   *at;
  int       ok = clamp != NULL && wep != NULL && mass != NULL && (*FP(t, F_CLAMP) = f64(clamp)) != NULL
           && (*FP(t, F_WEP_NM) = f64(wep)) != NULL && real(mass, FD(t, F_MASS))
           && (*FP(t, F_POSTURE) = f64(got(ask, K_POSTURE, &bad))) != NULL;

  ok = ok && (at = spread(got(ask, K_DERATE, &bad), t->spread, t->nd, 1.0)) != NULL && !bad;
  if (ok)
    *FP(t, F_DERATE) = at;
  ok = ok && (at = spread(got(ask, K_LO, &bad), t->spread + t->nd, t->nd, -INFINITY)) != NULL && !bad;
  if (ok)
    *FP(t, F_LO) = at;
  ok = ok && (at = spread(got(ask, K_HI, &bad), t->spread + 2 * t->nd, t->nd, INFINITY)) != NULL && !bad;
  if (ok)
    *FP(t, F_HI) = at;
  PyErr_Clear();
  Py_XDECREF(clamp);
  Py_XDECREF(wep);
  Py_XDECREF(mass);
  return ok;
}

static PyObject *copied(const double *d, int nd, npy_intp rows, npy_intp cols)
{
  npy_intp  dims[2] = {rows, cols};
  PyObject *out = PyArray_SimpleNew(nd, dims, NPY_DOUBLE);
  if (out != NULL)
    memcpy(PyArray_DATA((PyArrayObject *)out), d, (size_t)(rows * (nd == 2 ? cols : 1)) * sizeof(double));
  return out;
}

/* The step's answer as `_stepped` makes it, `b.load` set. */
static PyObject *answer(step_t *t, PyObject *b)
{
  const int nd = *FI(t, F_ND), nn = *FI(t, F_N), nc = *FI(t, F_NC), copn = *FI(t, F_COP_N);
  PyObject *load = copied(FD(t, F_LOAD), 1, nd, 0);
  if (load == NULL || PyObject_SetAttr(b, K[K_LOAD], load) < 0)
  {
    Py_XDECREF(load);
    return NULL;
  }
  Py_DECREF(load);
  npy_intp  n_over = nd;
  PyObject *cop = PyList_New(copn), *over = PyArray_SimpleNew(1, &n_over, NPY_BOOL);
  PyObject *bears = Py_BuildValue("[dd]", FD(t, F_BEARS)[0], FD(t, F_BEARS)[1]);
  for (int side = 0; cop != NULL && side < copn; side++)
    PyList_SET_ITEM(cop, side, FI(t, F_COP_NONE)[side] ? Py_NewRef(Py_None) : copied(FD(t, F_COP) + 2 * side, 1, 2, 0));
  if (over != NULL)
    for (int i = 0; i < nd; i++)
      ((npy_bool *)PyArray_DATA((PyArrayObject *)over))[i] = FI(t, F_OVER)[i] != 0;
  PyObject *values[] = {copied(FD(t, F_TAU), 1, nd, 0), copied(FD(t, F_QACC), 1, nn, 0), bears, cop, over, PyList_New(0),
                        PyTuple_New(0), copied(FD(t, F_FORCES), 2, nc, 3), PyFloat_FromDouble(*FD(t, F_SCALE)),
                        PyFloat_FromDouble(*FD(t, F_RESIDUAL)), copied(FD(t, F_SOLES_FREE), 2, 2, 6),
                        copied(FD(t, F_SOLES), 2, 2, 6), copied(FD(t, F_ROWS_FREE), 1, 10, 0)};
  static const int keys[] = {K_TAU,   K_QACC,     K_BEARS,      K_COP,  K_OVER,     K_SLACK,
                             K_HELD,  K_BORNE,    K_SCALE,      K_RESIDUAL, K_SOLES_FREE, K_SOLES,
                             K_ROWS_FREE};
  PyObject        *out = PyDict_New();
  for (size_t i = 0; i < sizeof keys / sizeof keys[0]; i++)
  {
    if (out != NULL && (values[i] == NULL || PyDict_SetItem(out, K[keys[i]], values[i]) < 0))
      Py_CLEAR(out);
    Py_XDECREF(values[i]);
  }
  if (out != NULL && PyErr_Occurred())
    Py_CLEAR(out);
  return out;
}

PyObject *quick_stepper(PyObject *self, PyObject *const *args, Py_ssize_t n)
{
  (void)self;
  if (n != 10 || !PyTuple_Check(args[2]) || PyTuple_GET_SIZE(args[2]) != FIELDS || !PyTuple_Check(args[7]))
  {
    PyErr_SetString(PyExc_TypeError, "stepper(k, step, offsets, most, pointed, npointed, mu, keys, miss, singular)");
    return NULL;
  }
  for (int i = 0; i < KEYS; i++)
    if (K[i] == NULL && (K[i] = PyUnicode_InternFromString(NAMES[i])) == NULL)
      return NULL;
  step_t *t = PyMem_Calloc(1, sizeof(step_t));
  if (t == NULL)
    return PyErr_NoMemory();
  t->k = (char *)PyLong_AsVoidPtr(args[0]);
  t->step = (int (*)(void *))(uintptr_t)PyLong_AsVoidPtr(args[1]);
  for (int f = 0; f < FIELDS; f++)
    t->off[f] = PyLong_AsSsize_t(PyTuple_GET_ITEM(args[2], f));
  t->most = PyLong_AsSsize_t(args[3]);
  t->pointed = PyLong_AsSsize_t(args[4]);
  t->npointed = PyLong_AsSsize_t(args[5]);
  t->mu = PyFloat_AsDouble(args[6]);
  if (PyErr_Occurred())
  {
    PyMem_Free(t);
    return NULL;
  }
  t->nd = *FI(t, F_ND);
  if ((t->spread = PyMem_Calloc(3 * (size_t)t->nd + 1, sizeof(double))) == NULL)
  {
    PyMem_Free(t);
    return PyErr_NoMemory();
  }
  t->keys = Py_NewRef(args[7]);
  t->miss = Py_NewRef(args[8]);
  t->singular = Py_NewRef(args[9]);
  PyObject *capsule = PyCapsule_New(t, "quick.step", dropped);
  if (capsule == NULL)
  {
    Py_DECREF(t->keys);
    Py_DECREF(t->miss);
    Py_DECREF(t->singular);
    PyMem_Free(t->spread);
    PyMem_Free(t);
  }
  return capsule;
}

PyObject *quick_step(PyObject *self, PyObject *const *args, Py_ssize_t n)
{
  (void)self;
  if (n != 5)
  {
    PyErr_SetString(PyExc_TypeError, "step(stepper, k, b, s, ask)");
    return NULL;
  }
  step_t *t = PyCapsule_GetPointer(args[0], "quick.step");
  if (t == NULL)
    return NULL;
  PyObject *kobj = args[1], *b = args[2], *s = args[3], *ask = args[4];
  int       bad = 0;
  if (!PyDict_Check(s) || !PyDict_Check(ask) || !pointed(t, kobj, s)
      || (*FP(t, F_M) = f64(got(s, K_M, &bad))) == NULL || bad || !touched(t, s) || !asked(t, ask) || !sides(t, ask)
      || !arrays(t, b, ask))
    return Py_NewRef(t->miss);
  int r;
  Py_BEGIN_ALLOW_THREADS
  r = t->step(t->k);
  Py_END_ALLOW_THREADS
  if (r)
  {
    PyErr_SetString(t->singular, "Singular matrix");
    return NULL;
  }
  return answer(t, b);
}
