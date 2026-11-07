/** quick_part.c - a compiled part called from C (`Part`, tools.cores.bodied.Compiled's base): its Bound's runner's
 * result where its C is held and its arguments are as traced, its Python (`slow`) else - no frame of Python between
 * a call and its runner. The host's alone. */
#define NO_IMPORT_ARRAY
#include "quick.h"

#include <stddef.h>

/* A part: its Bound, None before its C (`c`). */
typedef struct
{
  PyObject_HEAD
  PyObject *c;
} part_t;

/* What a runner gives back where a call's arguments are not as traced and where Python's walk must say why (quick.MISS,
 * quick.SLOW, held by `missing`); the names a call reads. */
static PyObject *miss, *slow, *runner_name, *slow_name;

PyObject *quick_missing(PyObject *self, PyObject *const *args, Py_ssize_t n)
{
  (void)self;
  if (n != 2)
  {
    PyErr_SetString(PyExc_TypeError, "missing(miss, slow)");
    return NULL;
  }
  Py_XSETREF(miss, Py_NewRef(args[0]));
  Py_XSETREF(slow, Py_NewRef(args[1]));
  Py_RETURN_NONE;
}

/* A call: its runner's result, unless it has none or the runner misses - then its `slow`, its arguments as given. */
static PyObject *called(PyObject *self, PyObject *args, PyObject *kwargs)
{
  PyObject *c = ((part_t *)self)->c;
  if (c != NULL && c != Py_None && miss != NULL)
  {
    Py_INCREF(c);
    PyObject *run = PyObject_GetAttr(c, runner_name);
    Py_DECREF(c);
    if (run == NULL)
      return NULL;
    if (run != Py_None)
    {
      PyObject *got = PyObject_CallOneArg(run, args);
      Py_DECREF(run);
      if (got == NULL || (got != miss && got != slow))
        return got;
      Py_DECREF(got);
    }
    else
      Py_DECREF(run);
  }
  PyObject *fn = PyObject_GetAttr(self, slow_name);
  if (fn == NULL)
    return NULL;
  PyObject *got = PyObject_Call(fn, args, kwargs);
  Py_DECREF(fn);
  return got;
}

static PyObject *made(PyTypeObject *type, PyObject *args, PyObject *kwargs)
{
  (void)args;
  (void)kwargs;
  part_t *self = (part_t *)type->tp_alloc(type, 0);
  if (self != NULL)
    self->c = Py_NewRef(Py_None);
  return (PyObject *)self;
}

static int traversed(PyObject *self, visitproc visit, void *arg)
{
  Py_VISIT(((part_t *)self)->c);
  return 0;
}

static int cleared(PyObject *self)
{
  Py_CLEAR(((part_t *)self)->c);
  return 0;
}

static void dropped(PyObject *self)
{
  PyObject_GC_UnTrack(self);
  cleared(self);
  Py_TYPE(self)->tp_free(self);
}

static PyMemberDef members[] = {{"c", Py_T_OBJECT_EX, offsetof(part_t, c), 0, "its Bound, None before its C"},
                                {NULL, 0, 0, 0, NULL}};

static PyTypeObject part_type = {
  PyVarObject_HEAD_INIT(NULL, 0).tp_name = "_quick.Part",
  .tp_basicsize = sizeof(part_t),
  .tp_dealloc = dropped,
  .tp_call = called,
  .tp_flags = Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE | Py_TPFLAGS_HAVE_GC,
  .tp_doc = "A compiled part called from C: its Bound's runner's result, its `slow` where it has none or misses.",
  .tp_traverse = traversed,
  .tp_clear = cleared,
  .tp_members = members,
  .tp_new = made,
};

int quick_part_ready(PyObject *module)
{
  runner_name = PyUnicode_InternFromString("runner");
  slow_name = PyUnicode_InternFromString("slow");
  if (runner_name == NULL || slow_name == NULL || PyType_Ready(&part_type) < 0)
    return -1;
  return PyModule_AddObjectRef(module, "Part", (PyObject *)&part_type);
}
