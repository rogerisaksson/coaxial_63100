/** quick_run.c - a compiled part's call in one C call (tools.cores.quick's runner): its head's checks (glue.head),
 * its leaves' walk (quick.c's) - a leaf of another layout handed to its Bound's `leaf` -, its C, and its result's
 * views of its output block (glue.maker), its Bound's `last` set; its miss, or its slow where Python's walk must
 * say why. No float is touched but a bool one's `!= 0.0`: the same objects Python's glue makes. The host's alone. */
#define NO_IMPORT_ARRAY
#include "quick.h"
#include <numpy/arrayscalars.h>

#include <string.h>

/* A check's kind (glue.head): a dict's key there or not, its keys in order, a list's length. */
enum
{
  HAS,
  KEYS,
  LENGTH
};

/* A result's leaf (glue.maker): a view of the block, a numpy float, a bool array, a numpy bool, an argument's own. */
enum
{
  VIEW,
  NUMBER,
  BOOLS,
  BOOL,
  GIVEN
};

/* A result's node: a leaf, a dict, a list, a tuple, a constant. */
enum
{
  LEAF,
  DICT,
  LIST,
  TUPLE,
  CONST
};

typedef struct
{
  int        kind;
  Py_ssize_t a, k;
  int        nd;
  npy_intp   dims[MAXD];
  PyObject  *path;
} leaf_t;

typedef struct
{
  PyObject  *bound, *body, *consts, *pins, *checks, *array_equal, *walk, *seen, *leaf, *places, *shape, *turns;
  PyObject  *blocks[2], *singular, *miss, *slow, *last_name;
  PyObject **made;                                  /* a call's leaves, each made once (glue.maker's l<k>) */
  leaf_t    *leaves;
  void      *fn, *fixed[6], *block_at[2];
  int64_t   *turn;
  Py_ssize_t body_at, total, nleaves;
  int        returns;
} run_t;

static void dropped(PyObject *capsule)
{
  run_t *r = PyCapsule_GetPointer(capsule, "quick.run");
  if (r == NULL)
    return;
  PyObject *held[] = {r->bound, r->body, r->consts, r->pins, r->checks, r->array_equal, r->walk, r->seen, r->leaf,
                      r->places, r->shape, r->turns, r->blocks[0], r->blocks[1], r->singular, r->miss, r->slow,
                      r->last_name};
  for (size_t k = 0; k < sizeof held / sizeof held[0]; k++)
    Py_XDECREF(held[k]);
  for (Py_ssize_t k = 0; k < r->nleaves; k++)
    Py_XDECREF(r->leaves[k].path);
  PyMem_Free(r->leaves);
  PyMem_Free(r->made);
  PyMem_Free(r);
}

/* Whether the exception set is one glue.head's try takes as not traced (KeyError, IndexError, TypeError): cleared. */
static int caught(void)
{
  if (!(PyErr_ExceptionMatches(PyExc_KeyError) || PyErr_ExceptionMatches(PyExc_IndexError)
        || PyErr_ExceptionMatches(PyExc_TypeError)))
    return 0;
  PyErr_Clear();
  return 1;
}

/* args[k][p0][p1]...: a new reference, or NULL with an exception set. */
static PyObject *reached(PyObject *args, Py_ssize_t k, PyObject *path)
{
  if (k >= PyTuple_GET_SIZE(args))
  {
    PyErr_SetString(PyExc_IndexError, "tuple index out of range");
    return NULL;
  }
  PyObject *a = Py_NewRef(PyTuple_GET_ITEM(args, k));
  for (Py_ssize_t p = 0; p < PyTuple_GET_SIZE(path); p++)
  {
    PyObject *x = PyObject_GetItem(a, PyTuple_GET_ITEM(path, p));
    Py_DECREF(a);
    if (x == NULL)
      return NULL;
    a = x;
  }
  return a;
}

/* Whether `args` are as traced (glue.head): 1, 0 not, -1 an error not its try's. */
static int headed(const run_t *r, PyObject *args)
{
  if (r->body_at >= 0)
  {
    if (r->body_at >= PyTuple_GET_SIZE(args) || PyTuple_GET_ITEM(args, r->body_at) != r->body)
      return 0;
    for (Py_ssize_t i = 0; i < PyTuple_GET_SIZE(r->consts); i++)
    {
      PyObject *pair = PyTuple_GET_ITEM(r->consts, i);
      PyObject *got = PyObject_GetAttr(r->body, PyTuple_GET_ITEM(pair, 0));
      if (got == NULL)
        return caught() ? 0 : -1;
      const int same = got == PyTuple_GET_ITEM(pair, 1);
      Py_DECREF(got);
      if (!same)
        return 0;
    }
  }
  for (Py_ssize_t i = 0; i < PyTuple_GET_SIZE(r->pins); i++)
  {
    PyObject *pin = PyTuple_GET_ITEM(r->pins, i), *v = PyTuple_GET_ITEM(pin, 2);
    PyObject *a = reached(args, PyLong_AsSsize_t(PyTuple_GET_ITEM(pin, 0)), PyTuple_GET_ITEM(pin, 1));
    if (a == NULL)
      return caught() ? 0 : -1;
    int differ = 0;
    if (a != v && PyTuple_GET_ITEM(pin, 3) == Py_True)
    {
      PyObject *e = PyObject_CallFunctionObjArgs(r->array_equal, a, v, NULL);
      differ = e == NULL ? -1 : !PyObject_IsTrue(e);
      Py_XDECREF(e);
    }
    else if (a != v)
      differ = Py_TYPE(a) != Py_TYPE(v) ? 1 : PyObject_RichCompareBool(a, v, Py_NE);
    Py_DECREF(a);
    if (differ < 0)
      return caught() ? 0 : -1;
    if (differ)
      return 0;
  }
  for (Py_ssize_t i = 0; i < PyTuple_GET_SIZE(r->checks); i++)
  {
    PyObject  *check = PyTuple_GET_ITEM(r->checks, i), *x = PyTuple_GET_ITEM(check, 3);
    const long kind = PyLong_AsLong(PyTuple_GET_ITEM(check, 0));
    PyObject  *a = reached(args, PyLong_AsSsize_t(PyTuple_GET_ITEM(check, 1)), PyTuple_GET_ITEM(check, 2));
    if (a == NULL)
      return caught() ? 0 : -1;
    int differ;
    if (kind == HAS)
    {
      const int in = PySequence_Contains(a, x), want = PyObject_IsTrue(PyTuple_GET_ITEM(check, 4));
      differ = in < 0 || want < 0 ? -1 : in != want;
    }
    else if (kind == KEYS)
    {
      PyObject *keys = PySequence_Tuple(a);
      differ = keys == NULL ? -1 : PyObject_RichCompareBool(keys, x, Py_NE);
      Py_XDECREF(keys);
    }
    else
    {
      const Py_ssize_t len = PyObject_Length(a);
      differ = len < 0 ? -1 : len != PyLong_AsSsize_t(x);
    }
    Py_DECREF(a);
    if (differ < 0)
      return caught() ? 0 : -1;
    if (differ)
      return 0;
  }
  return 1;
}

/* The walk (quick.walker's): each leaf laid in C, those of another layout by the Bound's `leaf`: 1 all laid, 0 one
 * not (the miss), 2 Python's walk to say why (the slow), -1 an error not its try's. */
static int walked(const run_t *r, PyObject *args)
{
  const walk_t *w = PyCapsule_GetPointer(r->walk, "quick.walk");
  if (w == NULL)
    return -1;
  for (Py_ssize_t at = 0;;)
  {
    const Py_ssize_t got = quick_walked(w, args, at);
    if (got == w->n)
      return 1;
    if (got == -1)
      return 0;
    if (got < -1)
    {
      for (Py_ssize_t j = 0; j < PyList_GET_SIZE(r->seen); j++)
        if (PyList_SetItem(r->seen, j, Py_NewRef(Py_None)) < 0)
          return -1;
      return 2;
    }
    PyObject *place = PyTuple_GET_ITEM(r->places, got);
    PyObject *a = reached(args, PyLong_AsSsize_t(PyTuple_GET_ITEM(place, 1)), PyTuple_GET_ITEM(place, 2));
    if (a == NULL)
      return caught() ? 0 : -1;
    if (PyList_SetItem(r->seen, PyLong_AsSsize_t(PyTuple_GET_ITEM(place, 0)), Py_NewRef(Py_None)) < 0)
    {
      Py_DECREF(a);
      return -1;
    }
    PyObject *ok = PyObject_CallFunctionObjArgs(r->leaf, PyTuple_GET_ITEM(place, 0), a, NULL);
    Py_DECREF(a);
    if (ok == NULL)
      return caught() ? 0 : -1;
    const int fits = PyObject_IsTrue(ok);
    Py_DECREF(ok);
    if (fits <= 0)
      return fits;
    at = got + 1;
  }
}

/* Leaf `k` of the result (glue.maker's l<k>), made: a new reference. */
static PyObject *made(const run_t *r, Py_ssize_t k, PyObject *flat, PyObject *args)
{
  const leaf_t *l = r->leaves + k;
  if (l->kind == GIVEN)
    return reached(args, l->k, l->path);
  const double *at = (const double *)PyArray_DATA((PyArrayObject *)flat) + l->a;
  if (l->kind == NUMBER)
    return PyArray_Scalar((void *)at, quick_f64, flat);
  if (l->kind == BOOL)
    return Py_NewRef(at[0] != 0.0 ? PyArrayScalar_True : PyArrayScalar_False);
  if (l->kind == BOOLS)
  {
    PyObject *b = PyArray_SimpleNew(l->nd, (npy_intp *)l->dims, NPY_BOOL);
    if (b == NULL)
      return NULL;
    npy_bool *o = PyArray_DATA((PyArrayObject *)b);
    for (npy_intp i = 0; i < PyArray_SIZE((PyArrayObject *)b); i++)
      o[i] = at[i] != 0.0;
    return b;
  }
  Py_INCREF(quick_f64);
  PyObject *v = PyArray_NewFromDescr(&PyArray_Type, quick_f64, l->nd, (npy_intp *)l->dims, NULL, (void *)at,
                                     NPY_ARRAY_CARRAY, NULL);
  if (v == NULL)
    return NULL;
  if (PyArray_SetBaseObject((PyArrayObject *)v, Py_NewRef(flat)) < 0)
  {
    Py_DECREF(v);
    return NULL;
  }
  return v;
}

/* Leaf `k`: made once a call, the same object wherever the result holds it - a new reference. */
static PyObject *leaf_of(const run_t *r, Py_ssize_t k, PyObject *flat, PyObject *args)
{
  if (r->made[k] == NULL)
    r->made[k] = made(r, k, flat, args);
  return Py_XNewRef(r->made[k]);
}

/* The result's tree at `node` (glue.maker's `code`): a new reference. */
static PyObject *built(const run_t *r, PyObject *node, PyObject *flat, PyObject *args)
{
  const long kind = PyLong_AsLong(PyTuple_GET_ITEM(node, 0));
  PyObject  *body = PyTuple_GET_ITEM(node, 1);
  if (kind == LEAF)
    return leaf_of(r, PyLong_AsSsize_t(body), flat, args);
  if (kind == CONST)
    return Py_NewRef(body);
  const Py_ssize_t n = PyTuple_GET_SIZE(body);
  PyObject        *got = kind == DICT ? PyDict_New() : kind == LIST ? PyList_New(n) : PyTuple_New(n);
  if (got == NULL)
    return NULL;
  for (Py_ssize_t i = 0; i < n; i++)
  {
    PyObject *item = PyTuple_GET_ITEM(body, i);
    PyObject *v = built(r, kind == DICT ? PyTuple_GET_ITEM(item, 1) : item, flat, args);
    if (v == NULL || (kind == DICT && PyDict_SetItem(got, PyTuple_GET_ITEM(item, 0), v) < 0))
    {
      Py_XDECREF(v);
      Py_DECREF(got);
      return NULL;
    }
    if (kind == DICT)
      Py_DECREF(v);
    else if (kind == LIST)
      PyList_SET_ITEM(got, i, v);
    else
      PyTuple_SET_ITEM(got, i, v);
  }
  return got;
}

static void *pointer(PyObject *x)
{
  return x == Py_None ? NULL : PyLong_AsVoidPtr(x);
}

PyObject *quick_runner(PyObject *self, PyObject *const *args, Py_ssize_t n)
{
  (void)self;
  enum
  {
    BOUND,
    BODY,
    BODY_AT,
    CONSTS,
    PINS,
    CHECKS,
    ARRAY_EQUAL,
    WALK,
    SEEN,
    LEAF_FN,
    PLACES,
    FN,
    RETURNS,
    FIXED,
    BLOCKS,
    TURNS,
    TOTAL,
    LEAVES,
    SHAPE,
    SINGULAR,
    MISS,
    SLOW,
    ARGS
  };
  if (n != ARGS || !PyTuple_Check(args[FIXED]) || PyTuple_GET_SIZE(args[FIXED]) != 6 || !PyTuple_Check(args[BLOCKS])
      || !PyTuple_Check(args[LEAVES]) || !PyArray_Check(args[TURNS]))
  {
    PyErr_SetString(PyExc_TypeError, "runner(bound, body, body_at, consts, pins, checks, array_equal, walk, seen, "
                                     "leaf, places, fn, returns, fixed, blocks, turns, total, leaves, shape, singular, "
                                     "miss, slow)");
    return NULL;
  }
  run_t *r = PyMem_Calloc(1, sizeof *r);
  if (r == NULL)
    return PyErr_NoMemory();
  PyObject *capsule = PyCapsule_New(r, "quick.run", dropped);
  if (capsule == NULL)
  {
    PyMem_Free(r);
    return NULL;
  }
  r->bound = Py_NewRef(args[BOUND]);
  r->body = Py_NewRef(args[BODY]);
  r->consts = Py_NewRef(args[CONSTS]);
  r->pins = Py_NewRef(args[PINS]);
  r->checks = Py_NewRef(args[CHECKS]);
  r->array_equal = Py_NewRef(args[ARRAY_EQUAL]);
  r->walk = Py_NewRef(args[WALK]);
  r->seen = Py_NewRef(args[SEEN]);
  r->leaf = Py_NewRef(args[LEAF_FN]);
  r->places = Py_NewRef(args[PLACES]);
  r->shape = Py_NewRef(args[SHAPE]);
  r->turns = Py_NewRef(args[TURNS]);
  r->singular = Py_NewRef(args[SINGULAR]);
  r->miss = Py_NewRef(args[MISS]);
  r->slow = Py_NewRef(args[SLOW]);
  r->last_name = PyUnicode_InternFromString("last");
  r->body_at = PyLong_AsSsize_t(args[BODY_AT]);
  r->fn = PyLong_AsVoidPtr(args[FN]);
  r->returns = PyObject_IsTrue(args[RETURNS]);
  r->total = PyLong_AsSsize_t(args[TOTAL]);
  r->turn = (int64_t *)PyArray_DATA((PyArrayObject *)args[TURNS]);
  for (int k = 0; k < 6; k++)
    r->fixed[k] = pointer(PyTuple_GET_ITEM(args[FIXED], k));
  for (int k = 0; k < 2 && k < PyTuple_GET_SIZE(args[BLOCKS]); k++)
  {
    r->blocks[k] = Py_NewRef(PyTuple_GET_ITEM(args[BLOCKS], k));
    r->block_at[k] = PyArray_DATA((PyArrayObject *)r->blocks[k]);
  }
  r->nleaves = PyTuple_GET_SIZE(args[LEAVES]);
  r->leaves = PyMem_Calloc((size_t)(r->nleaves > 0 ? r->nleaves : 1), sizeof *r->leaves);
  r->made = PyMem_Calloc((size_t)(r->nleaves > 0 ? r->nleaves : 1), sizeof *r->made);
  if (r->leaves == NULL || r->made == NULL || r->last_name == NULL || PyErr_Occurred())
  {
    const int memory = r->leaves == NULL || r->made == NULL;
    Py_DECREF(capsule);
    return memory ? PyErr_NoMemory() : NULL;
  }
  for (Py_ssize_t k = 0; k < r->nleaves; k++)
  {
    PyObject *spec = PyTuple_GET_ITEM(args[LEAVES], k), *dims = PyTuple_GET_ITEM(spec, 3);
    leaf_t   *l = r->leaves + k;
    l->kind = (int)PyLong_AsLong(PyTuple_GET_ITEM(spec, 0));
    l->a = PyLong_AsSsize_t(PyTuple_GET_ITEM(spec, 1));
    l->k = PyLong_AsSsize_t(PyTuple_GET_ITEM(spec, 2));
    l->nd = (int)PyTuple_GET_SIZE(dims);
    l->path = Py_NewRef(PyTuple_GET_ITEM(spec, 4));
    if (l->nd > MAXD)
    {
      PyErr_SetString(PyExc_ValueError, "runner: a leaf of more dimensions than MAXD");
      Py_DECREF(capsule);
      return NULL;
    }
    for (int d = 0; d < l->nd; d++)
      l->dims[d] = (npy_intp)PyLong_AsSsize_t(PyTuple_GET_ITEM(dims, d));
  }
  if (PyErr_Occurred())
  {
    Py_DECREF(capsule);
    return NULL;
  }
  return capsule;
}

PyObject *quick_run(PyObject *self, PyObject *const *args, Py_ssize_t n)
{
  (void)self;
  if (n != 2 || !PyTuple_Check(args[1]))
  {
    PyErr_SetString(PyExc_TypeError, "run(runner, args - a tuple -)");
    return NULL;
  }
  const run_t *r = PyCapsule_GetPointer(args[0], "quick.run");
  if (r == NULL)
    return NULL;
  PyObject *given = args[1];
  int       fits = headed(r, given);
  if (fits == 1)
    fits = walked(r, given);
  if (fits < 0)
    return NULL;
  if (fits != 1)
    return Py_NewRef(fits == 2 ? r->slow : r->miss);
  PyObject *flat = NULL;
  void     *at = NULL;
  if (r->total)
  {
    const int64_t turn = *r->turn;
    *r->turn = 1 - turn;
    flat = r->blocks[turn];
    at = r->block_at[turn];
  }
  int bad = 0;
  Py_BEGIN_ALLOW_THREADS
  if (r->returns)
    bad = ((part_i)(uintptr_t)r->fn)(NULL, at, r->fixed[0], r->fixed[1], r->fixed[2], r->fixed[3], r->fixed[4],
                                     r->fixed[5]);
  else
    ((part_v)(uintptr_t)r->fn)(NULL, at, r->fixed[0], r->fixed[1], r->fixed[2], r->fixed[3], r->fixed[4], r->fixed[5]);
  Py_END_ALLOW_THREADS
  if (bad)
  {
    PyErr_SetString(r->singular, "Singular matrix");
    return NULL;
  }
  PyObject *got = r->total ? built(r, r->shape, flat, given) : Py_NewRef(Py_None);
  for (Py_ssize_t k = 0; k < r->nleaves; k++)
    Py_CLEAR(r->made[k]);
  if (got == NULL)
    return NULL;
  PyObject *last = r->total ? Py_BuildValue("(ON)", got, PyLong_FromVoidPtr(at)) : Py_BuildValue("(OO)", Py_None,
                                                                                                    Py_None);
  if (last == NULL || PyObject_SetAttr(r->bound, r->last_name, last) < 0)
  {
    Py_XDECREF(last);
    Py_DECREF(got);
    return NULL;
  }
  Py_DECREF(last);
  return got;
}
