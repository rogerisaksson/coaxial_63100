"""A compiled part's glue written out once a build: its result's views, its arguments' checks and walk.

    made = glue.maker(shape, outs)    # (block, args) -> fn's result, each leaf a view of the block
    head = glue.head(bound)           # args -> whether her body, the pins and the checks are as traced
    walk = glue.walk(bound, todo)     # args -> whether each leaf fits, its extern addressed (`Bound.fits`)

Each a Python function of straight lines, its paths, shapes and places constants of the build: a loop
over them a call had walked 1 us a leaf (`tools.cores.bodied.Bound`).
"""
from collections.abc import Callable

import numpy as np

from tools.cores import layout
from tools.cores.wbc import address

#: numpy's float64, the dtype an extern is.
F64 = np.dtype(np.float64)


def whole(r):
    """Whether a Ref is the whole of its buffer in C order."""
    return r.offset == 0 and r.shape == r.buffer.shape and r.strides == tuple(layout.cstrides(r.shape))


def maker(shape, outs):
    """fn's result laid back of an output block and its arguments, a function of them: each leaf a
    view of the block at its span (a 0-d one numpy's scalar, a bool one numpy's bools) - a whole
    argument array the caller's own, as Python returns it -, its dicts, lists and tuples as traced
    (`trace.unflatten`)."""
    consts, lines = [], []
    for k, (a, b, s, dtype, given) in enumerate(outs):
        view = 'f[%d]' % a if not s else 'f[%d:%d]' % (a, b) if len(s) == 1 else 'f[%d:%d].reshape(%r)' % (a, b, tuple(s))
        if given is not None:
            view = 'a[%d]%s' % (given[0], ''.join('[%r]' % p for p in given[1]))
        lines.append('    l%d = %s' % (k, view + ' != 0.0' if dtype == bool and given is None else view))

    def put(x):
        consts.append(x)
        return 'c[%d]' % (len(consts) - 1)

    def code(node):
        kind, body = node
        if kind == 'leaf':
            return 'l%d' % body
        if kind == 'dict':
            return '{%s}' % ', '.join('%s: %s' % (put(k), code(v)) for k, v in body)
        if kind in ('list', 'tuple'):
            items = [code(v) for v in body]
            return '[%s]' % ', '.join(items) if kind == 'list' else '(%s)' % ''.join(x + ', ' for x in items)
        return put(body)
    text = 'def made(f, a):\n%s\n    return %s\n' % ('\n'.join(lines), code(shape))
    space: dict = {'c': consts}
    exec(text, space)
    return space['made']


def _written(name, lines, consts, space) -> Callable[[tuple], bool]:
    """Function `name`(args) of `lines` under a try - a path not there False -, True at their end."""
    text = 'def %s(args):\n    try:\n%s\n    except (KeyError, IndexError, TypeError):\n        return False\n    return True\n' % (
        name, '\n'.join(lines) or '        pass')
    space = dict(space, c=consts)
    exec(text, space)
    return space[name]


def head(b):
    """Bound `b`'s fits before its leaves: her body the one bound and its constants the trace's, each
    pinned value the same, each dict's keys and each list's length as traced."""
    consts, lines = [], []

    def put(x):
        consts.append(x)
        return 'c[%d]' % (len(consts) - 1)

    def at(k, path):
        return 'args[%d]%s' % (k, ''.join('[%s]' % put(p) for p in path))
    if b.at is not None:
        lines += ['        x = args[%d]' % b.at,
                  '        if x is not bound.body%s:' % ''.join(' or getattr(x, %s) is not %s' % (put(k), put(v))
                                                             for k, v in b.consts),
                  '            return False']
    for k, path, v in b.pins:
        p = put(v)
        same = 'not array_equal(a, %s)' % p if isinstance(v, np.ndarray) else '(type(a) is not type(%s) or a != %s)' % (p, p)
        lines += ['        a = %s' % at(k, path), '        if a is not %s and %s:' % (p, same), '            return False']
    for kind, k, path, *rest in b.checks:
        test = {'has': '(%s in a) != %s' % (put(rest[0]), put(rest[-1])), 'keys': 'tuple(a) != %s' % put(rest[0]),
                'length': 'len(a) != %s' % put(rest[0])}[kind]
        lines += ['        a = %s' % at(k, path), '        if %s:' % test, '            return False']
    return _written('head', lines, consts, {'bound': b, 'array_equal': np.array_equal})


def walk(b, todo):
    """Bound `b`'s fits over its leaves `todo`: each found by its path; a float a scalar of its own;
    the array laid last kept, a fresh float array of its shape addressed, else `Bound.leaf`."""
    consts, lines = [], []

    def put(x):
        consts.append(x)
        return 'c[%d]' % (len(consts) - 1)
    for j, i, k, path, written, shape in todo:
        s = put(shape)
        lines.append('        a = args[%d]%s' % (k, ''.join('[%s]' % put(p) for p in path)))
        if not shape and not written:
            lines += ['        if type(a) is float:', '            scalars[%d] = a' % j,
                      '            if seen[%d] is not None:' % j, '                seen[%d] = None' % j,
                      '                words[%d] = own_at[%d]' % (i, j), '        el' + 'if a is not seen[%d] or a.shape != %s:' % (j, s)]
        else:
            lines.append('        if a is not seen[%d] or a.shape != %s:' % (j, s))
        lines += ['            if type(a) is ndarray and a.dtype is F64 and a.shape == %s and a.flags.c_contiguous:' % s,
                  '                seen[%d] = a' % j, '                words[%d] = address(a)' % i,
                  '            elif not leaf(%d, a):' % j, '                return False']
    return _written('walk', lines, consts, {'seen': b.seen, 'words': b.words, 'scalars': b.scalars, 'own_at': b.own_at,
                                            'leaf': b.leaf, 'ndarray': np.ndarray, 'F64': F64, 'address': address})
