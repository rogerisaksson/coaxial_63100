/** quick.c - her compiled parts' glue as an extension of this interpreter (tools.cores.quick): a part's
 * leaves walked and laid in its table, its C called, an array's data read - a call each. The host's
 * alone, against CPython's and numpy's headers: an array's layout read, none of numpy's arithmetic. */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_2_0_API_VERSION
#include <numpy/ndarraytypes.h>

#include <stdint.h>
#include <string.h>

/* A leaf's row (quick.py's WIDTH): the argument it is in, its first key and their count in the walk's
 * keys, its extern's slot, its place among the part's leaves, whether a float is its own scalar, its
 * dimensions' count (-1: Python's always) and each one's length. */
enum
{
  ARG,
  KEY,
  DEPTH,
  SLOT,
  PLACE,
  SCALAR,
  ND,
  DIMS,
  MAXD = 8,
  WIDTH = DIMS + MAXD
};

/* A walk laid once (`context`): its leaves' rows and keys, the table, scalars and own blocks it lays. */
typedef struct
{
  Py_ssize_t      n;
  int64_t        *rows;
  PyObject       *keys;
  uint64_t       *words;
  double         *scalars;
  const uint64_t *own_at;
} walk_t;

static PyTypeObject  *ndarray;
static PyArray_Descr *f64;

static PyObject *laid(PyObject *self, PyObject *const *args, Py_ssize_t n)
{
  (void)self;
  if (n != 2 || !PyType_Check(args[0]))
  {
    PyErr_SetString(PyExc_TypeError, "laid(numpy.ndarray, numpy's float64)");
    return NULL;
  }
  Py_XDECREF(ndarray);
  Py_XDECREF(f64);
  ndarray = (PyTypeObject *)Py_NewRef(args[0]);
  f64 = (PyArray_Descr *)Py_NewRef(args[1]);
  Py_RETURN_NONE;
}

static PyObject *data(PyObject *self, PyObject *a)
{
  (void)self;
  if (ndarray == NULL || !PyObject_TypeCheck(a, ndarray))
  {
    PyErr_SetString(PyExc_TypeError, "data(an ndarray)");
    return NULL;
  }
  return PyLong_FromVoidPtr(((PyArrayObject_fields *)a)->data);
}

static void dropped(PyObject *capsule)
{
  walk_t *w = PyCapsule_GetPointer(capsule, "quick.walk");
  if (w == NULL)
    return;
  Py_XDECREF(w->keys);
  PyMem_Free(w->rows);
  PyMem_Free(w);
}

static PyObject *context(PyObject *self, PyObject *const *args, Py_ssize_t n)
{
  (void)self;
  if (n != 6 || !PyTuple_Check(args[2]))
  {
    PyErr_SetString(PyExc_TypeError, "context(rows, n, keys, words, scalars, own_at)");
    return NULL;
  }
  const int64_t   *rows = PyLong_AsVoidPtr(args[0]);
  const Py_ssize_t count = PyLong_AsSsize_t(args[1]);
  uint64_t        *words = PyLong_AsVoidPtr(args[3]);
  double          *scalars = PyLong_AsVoidPtr(args[4]);
  const uint64_t  *own_at = PyLong_AsVoidPtr(args[5]);
  if (PyErr_Occurred())
    return NULL;
  if (count < 0 || (count > 0 && rows == NULL))
  {
    PyErr_SetString(PyExc_ValueError, "context: a walk's rows, n of them");
    return NULL;
  }
  walk_t *w = PyMem_Calloc(1, sizeof *w);
  if (w == NULL)
    return PyErr_NoMemory();
  w->rows = PyMem_Malloc((size_t)(count > 0 ? count : 1) * WIDTH * sizeof *w->rows);
  if (w->rows == NULL)
  {
    PyMem_Free(w);
    return PyErr_NoMemory();
  }
  if (count > 0)
    memcpy(w->rows, rows, (size_t)count * WIDTH * sizeof *w->rows);
  w->n = count;
  w->keys = Py_NewRef(args[2]);
  w->words = words;
  w->scalars = scalars;
  w->own_at = own_at;
  PyObject *capsule = PyCapsule_New(w, "quick.walk", dropped);
  if (capsule == NULL)
  {
    Py_DECREF(w->keys);
    PyMem_Free(w->rows);
    PyMem_Free(w);
  }
  return capsule;
}

/* Leaf `row`'s `a` laid where it is a float of its own, or a float64 array of its shape in C's order;
 * else 0, the leaf Python's (`Bound.leaf`). */
static int laid_leaf(const walk_t *w, const int64_t *row, PyObject *a)
{
  if (row[SCALAR] && PyFloat_CheckExact(a))
  {
    w->scalars[row[PLACE]] = PyFloat_AS_DOUBLE(a);
    w->words[row[SLOT]] = w->own_at[row[PLACE]];
    return 1;
  }
  if (Py_TYPE(a) != ndarray || row[ND] < 0)
    return 0;
  const PyArrayObject_fields *f = (const PyArrayObject_fields *)a;
  if (f->descr != f64 || f->nd != row[ND] || !(f->flags & NPY_ARRAY_C_CONTIGUOUS))
    return 0;
  for (int d = 0; d < f->nd; d++)
    if (f->dimensions[d] != row[DIMS + d])
      return 0;
  w->words[row[SLOT]] = (uint64_t)(uintptr_t)f->data;
  return 1;
}

static PyObject *walk(PyObject *self, PyObject *const *args, Py_ssize_t n)
{
  (void)self;
  if (n != 3 || !(PyTuple_Check(args[0]) || PyList_Check(args[0])))
  {
    PyErr_SetString(PyExc_TypeError, "walk(args - a tuple or a list -, context, start)");
    return NULL;
  }
  const walk_t    *w = PyCapsule_GetPointer(args[1], "quick.walk");
  const Py_ssize_t start = PyLong_AsSsize_t(args[2]);
  if (w == NULL || (start == -1 && PyErr_Occurred()))
    return NULL;
  PyObject *given = args[0];
  const int listed = PyList_Check(given);
  for (Py_ssize_t r = start; r < w->n; r++)
  {
    const int64_t *row = w->rows + r * WIDTH;
    if (row[ARG] >= (listed ? PyList_GET_SIZE(given) : PyTuple_GET_SIZE(given)))
      return PyLong_FromLong(-1);
    PyObject *a = Py_NewRef(listed ? PyList_GET_ITEM(given, row[ARG]) : PyTuple_GET_ITEM(given, row[ARG]));
    for (int64_t p = 0; p < row[DEPTH]; p++)
    {
      PyObject *x = PyObject_GetItem(a, PyTuple_GET_ITEM(w->keys, row[KEY] + p));
      Py_DECREF(a);
      if (x == NULL)
      {
        const int missing = PyErr_ExceptionMatches(PyExc_KeyError) || PyErr_ExceptionMatches(PyExc_IndexError)
                            || PyErr_ExceptionMatches(PyExc_TypeError);
        PyErr_Clear();
        return PyLong_FromLong(missing ? -1 : -2);
      }
      a = x;
    }
    const int done = laid_leaf(w, row, a);
    Py_DECREF(a);
    if (!done)
      return PyLong_FromSsize_t(r);
  }
  return PyLong_FromSsize_t(w->n);
}

typedef int (*part_i)(void *, void *, void *, void *, void *, void *, void *, void *);
typedef void (*part_v)(void *, void *, void *, void *, void *, void *, void *, void *);

static PyObject *call(PyObject *self, PyObject *const *args, Py_ssize_t n)
{
  (void)self;
  if (n != 10)
  {
    PyErr_SetString(PyExc_TypeError, "call(fn, solves, eight pointers)");
    return NULL;
  }
  void *p[10];
  for (Py_ssize_t k = 0; k < n; k++)
    p[k] = args[k] == Py_None ? NULL : PyLong_AsVoidPtr(args[k]);
  if (PyErr_Occurred())
    return NULL;
  if (p[0] == NULL)
  {
    PyErr_SetString(PyExc_ValueError, "call: no function");
    return NULL;
  }
  int r = 0;
  Py_BEGIN_ALLOW_THREADS
  if (p[1] != NULL)
    r = ((part_i)(uintptr_t)p[0])(p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9]);
  else
    ((part_v)(uintptr_t)p[0])(p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9]);
  Py_END_ALLOW_THREADS
  return PyLong_FromLong(r);
}

static PyMethodDef methods[] = {
  {"laid", (PyCFunction)(void (*)(void))laid, METH_FASTCALL, "laid(numpy.ndarray, its float64): what a leaf is."},
  {"data", data, METH_O, "data(a): an ndarray's first element's address."},
  {"context", (PyCFunction)(void (*)(void))context, METH_FASTCALL,
   "context(rows, n, keys, words, scalars, own_at): a walk laid once."},
  {"walk", (PyCFunction)(void (*)(void))walk, METH_FASTCALL,
   "walk(args, context, start): n all laid, -1 a path not there, -2 another error, else the leaf Python lays."},
  {"call", (PyCFunction)(void (*)(void))call, METH_FASTCALL,
   "call(fn, solves, eight pointers): fn's int - 0 where it returns none -, the interpreter let go."},
  {NULL, NULL, 0, NULL}};

static struct PyModuleDef quick = {PyModuleDef_HEAD_INIT, "_quick", "Her compiled parts' glue (tools.cores.quick).", -1,
                                   methods, NULL, NULL, NULL, NULL};

PyMODINIT_FUNC PyInit__quick(void)
{
  return PyModule_Create(&quick);
}
