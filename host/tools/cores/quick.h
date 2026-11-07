/** quick.h - what quick.c's walk and quick_run.c's runner share (tools.cores.quick): a walk's rows and its laying,
 * numpy's ndarray and float64 as this interpreter laid them, a part's C. The host's alone. */
#ifndef QUICK_H
#define QUICK_H

#define PY_SSIZE_T_CLEAN
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_2_0_API_VERSION
#define PY_ARRAY_UNIQUE_SYMBOL quick_ARRAY_API
#include <numpy/arrayobject.h>

#include <stdint.h>

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

/* A part's C: eight pointers, an int back or none. */
typedef int (*part_i)(void *, void *, void *, void *, void *, void *, void *, void *);
typedef void (*part_v)(void *, void *, void *, void *, void *, void *, void *, void *);

/* numpy's ndarray and float64 (`laid`). */
extern PyTypeObject  *quick_ndarray;
extern PyArray_Descr *quick_f64;

/* The walk from row `start` over `given` (a tuple or a list): n all laid, -1 a path not there, -2 another error (its
 * exception cleared), else the row Python lays. */
Py_ssize_t quick_walked(const walk_t *w, PyObject *given, Py_ssize_t start);

/* runner(...) and run(runner, args) (quick_run.c). */
PyObject *quick_runner(PyObject *self, PyObject *const *args, Py_ssize_t n);
PyObject *quick_run(PyObject *self, PyObject *const *args, Py_ssize_t n);

/* stepper(...) and step(stepper, k, b, s, ask) (quick_step.c). */
PyObject *quick_stepper(PyObject *self, PyObject *const *args, Py_ssize_t n);
PyObject *quick_step(PyObject *self, PyObject *const *args, Py_ssize_t n);

/* Part, a compiled part called from C, readied on the module; missing(miss, slow): its runner's misses (quick_part.c). */
int       quick_part_ready(PyObject *module);
PyObject *quick_missing(PyObject *self, PyObject *const *args, Py_ssize_t n);

#endif
