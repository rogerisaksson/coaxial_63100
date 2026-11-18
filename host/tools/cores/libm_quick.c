/** libm_quick.c - the robot's own libm (wbc_libm.h) as an extension of this interpreter (tools.cores.libm_quick): a
 * number - a float, numpy's float64, an int - a call inlined, an array of float64 laid in C's order its run
 * (wbc_libm.c: a few a call each, more in vector lanes) into a new array of its shape, anything else handed to
 * Python's (`laid`'s other: an array strided or of another type, two to broadcast). The same bits each way. The
 * host's alone. */
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_2_0_API_VERSION
#include <numpy/arrayobject.h>

#include "wbc_libm_arc.h"

#include <stdint.h>

typedef void (*run1_t)(const double *restrict x, double *restrict y, int32_t n);
typedef void (*run2_t)(const double *restrict a, const double *restrict b, double *restrict y, int32_t n);

/* other(name, *args): Python's way with what is neither a float nor an array laid (`laid`). */
static PyObject *other;

static PyObject *laid(PyObject *self, PyObject *given)
{
  (void)self;
  Py_XSETREF(other, Py_NewRef(given));
  Py_RETURN_NONE;
}

/* `x` as a double: 1 a float (numpy's float64 is one) or an int itself, 0 neither, -1 an int past a double (its
 * error set). */
static int number(PyObject *x, double *v)
{
  if (PyFloat_Check(x))
  {
    *v = PyFloat_AS_DOUBLE(x);
    return 1;
  }
  if (PyLong_CheckExact(x))
  {
    *v = PyLong_AsDouble(x);
    return (*v == -1.0 && PyErr_Occurred() != NULL) ? -1 : 1;
  }
  return 0;
}

/* Whether `x` is an ndarray itself of native float64 in C's order, of a dimension and more and an int32 of elements. */
static int laid_array(PyObject *x)
{
  PyArrayObject *a = (PyArrayObject *)x;

  return PyArray_CheckExact(x) && PyArray_TYPE(a) == NPY_DOUBLE && PyArray_ISNOTSWAPPED(a) && PyArray_IS_C_CONTIGUOUS(a)
         && PyArray_NDIM(a) > 0 && PyArray_SIZE(a) <= INT32_MAX;
}

/* A new float64 array of `a`'s shape, or NULL. */
static PyArrayObject *shaped(PyArrayObject *a)
{
  return (PyArrayObject *)PyArray_SimpleNew(PyArray_NDIM(a), PyArray_DIMS(a), NPY_DOUBLE);
}

static PyObject *elsewhere(const char *name, PyObject *x, PyObject *y)
{
  if (other == NULL)
  {
    PyErr_Format(PyExc_TypeError, "%s: a float or a float64 array laid in C's order (no other way laid)", name);
    return NULL;
  }
  return (y == NULL) ? PyObject_CallFunction(other, "sO", name, x) : PyObject_CallFunction(other, "sOO", name, x, y);
}

static PyObject *run_one(run1_t run, PyObject *x)
{
  PyArrayObject *a = (PyArrayObject *)x, *out = shaped(a);

  if (out != NULL)
  {
    run((const double *)PyArray_DATA(a), (double *)PyArray_DATA(out), (int32_t)PyArray_SIZE(a));
  }
  return (PyObject *)out;
}

#define ONE(NAME)                                                                   \
  static PyObject *lq_##NAME(PyObject *self, PyObject *x)                           \
  {                                                                                 \
    double    v;                                                                    \
    const int is = number(x, &v);                                                   \
                                                                                    \
    (void)self;                                                                     \
    if (is != 0)                                                                    \
    {                                                                               \
      return (is < 0) ? NULL : PyFloat_FromDouble(wbc_##NAME(v));                   \
    }                                                                               \
    return laid_array(x) ? run_one(wbc_libm_##NAME, x) : elsewhere(#NAME, x, NULL); \
  }

ONE(sin)
ONE(cos)
ONE(tan)
ONE(exp)
ONE(expm1)
ONE(log)
ONE(arctan)
ONE(arcsin)
ONE(arccos)

static PyObject *lq_sincos(PyObject *self, PyObject *x)
{
  double    v, s, c;
  const int is = number(x, &v);

  (void)self;
  if (is != 0)
  {
    if (is < 0)
    {
      return NULL;
    }
    wbc_sincos(v, &s, &c);
    return Py_BuildValue("(dd)", s, c);
  }
  if (laid_array(x))
  {
    PyArrayObject *a = (PyArrayObject *)x, *sines = shaped(a), *cosines = shaped(a);

    if (sines == NULL || cosines == NULL)
    {
      Py_XDECREF(sines);
      Py_XDECREF(cosines);
      return NULL;
    }
    wbc_libm_sincos((const double *)PyArray_DATA(a), (double *)PyArray_DATA(sines), (double *)PyArray_DATA(cosines),
                    (int32_t)PyArray_SIZE(a));
    return Py_BuildValue("(NN)", sines, cosines);
  }
  return elsewhere("sincos", x, NULL);
}

/* Whether two laid arrays are of one shape. */
static int alike(PyArrayObject *a, PyArrayObject *b)
{
  return PyArray_NDIM(a) == PyArray_NDIM(b) && PyArray_CompareLists(PyArray_DIMS(a), PyArray_DIMS(b), PyArray_NDIM(a));
}

/* `x` itself where it is laid, a number spread to `like`'s shape (each a new reference); NULL where it is neither -
 * no error set - or its error. */
static PyArrayObject *spread(PyObject *x, PyArrayObject *like)
{
  double         v;
  PyArrayObject *out;

  if (laid_array(x))
  {
    return (PyArrayObject *)Py_NewRef(x);
  }
  if (number(x, &v) <= 0)
  {
    return NULL;
  }
  out = shaped(like);
  if (out != NULL)
  {
    double        *to = (double *)PyArray_DATA(out);
    const npy_intp n = PyArray_SIZE(out);

    for (npy_intp i = 0; i < n; i++)
    {
      to[i] = v;
    }
  }
  return out;
}

/* A run of two: arrays laid alike, or one laid and a number beside it; NULL with no error where it takes neither. */
static PyObject *run_two(run2_t run, PyObject *x, PyObject *y)
{
  PyArrayObject *like = laid_array(x) ? (PyArrayObject *)x : (laid_array(y) ? (PyArrayObject *)y : NULL);
  PyArrayObject *a, *b, *out = NULL;

  if (like == NULL)
  {
    return NULL;
  }
  a = spread(x, like);
  b = (a != NULL) ? spread(y, like) : NULL;
  if (a != NULL && b != NULL && alike(a, b))
  {
    out = shaped(like);
    if (out != NULL)
    {
      run((const double *)PyArray_DATA(a), (const double *)PyArray_DATA(b), (double *)PyArray_DATA(out),
          (int32_t)PyArray_SIZE(like));
    }
  }
  Py_XDECREF(a);
  Py_XDECREF(b);
  return (PyObject *)out;
}

#define TWO(NAME)                                                                                  \
  static PyObject *lq_##NAME(PyObject *self, PyObject *const *args, Py_ssize_t n)                  \
  {                                                                                                \
    double    v = 0.0, w = 0.0;                                                                    \
    int       is;                                                                                  \
    PyObject *got;                                                                                 \
                                                                                                   \
    (void)self;                                                                                    \
    if (n != 2)                                                                                    \
    {                                                                                              \
      PyErr_SetString(PyExc_TypeError, #NAME "(a, b)");                                            \
      return NULL;                                                                                 \
    }                                                                                              \
    is = number(args[0], &v);                                                                      \
    is = (is > 0) ? number(args[1], &w) : is;                                                      \
    if (is != 0)                                                                                   \
    {                                                                                              \
      return (is < 0) ? NULL : PyFloat_FromDouble(wbc_##NAME(v, w));                               \
    }                                                                                              \
    got = run_two(wbc_libm_##NAME, args[0], args[1]);                                              \
    return (got != NULL || PyErr_Occurred() != NULL) ? got : elsewhere(#NAME, args[0], args[1]);   \
  }

TWO(arctan2)
TWO(hypot)

#define SAYS_ONE(NAME) {#NAME, lq_##NAME, METH_O, #NAME "(x): wbc_libm's, a float or an array laid; else Python's."}
#define SAYS_TWO(NAME) \
  {#NAME, (PyCFunction)(void (*)(void))lq_##NAME, METH_FASTCALL, #NAME "(a, b): wbc_libm's, floats or arrays laid alike."}

static PyMethodDef methods[] = {
  {"laid", laid, METH_O, "laid(other): other(name, *args), Python's way with what C does not take."},
  SAYS_ONE(sin),
  SAYS_ONE(cos),
  SAYS_ONE(tan),
  SAYS_ONE(sincos),
  SAYS_ONE(exp),
  SAYS_ONE(expm1),
  SAYS_ONE(log),
  SAYS_ONE(arctan),
  SAYS_ONE(arcsin),
  SAYS_ONE(arccos),
  SAYS_TWO(arctan2),
  SAYS_TWO(hypot),
  {NULL, NULL, 0, NULL}};

static struct PyModuleDef libm_quick = {PyModuleDef_HEAD_INIT, "_libm_quick",
                                        "The robot's own libm, a float or an array a call (tools.cores.libm_quick).",
                                        -1, methods, NULL, NULL, NULL, NULL};

PyMODINIT_FUNC PyInit__libm_quick(void)
{
  import_array();
  return PyModule_Create(&libm_quick);
}
