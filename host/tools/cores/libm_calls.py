"""The robot's code's calls into the platform's math library, counted by package.

The robot's own elementary functions (`machine.libm`, wbc_libm.h) give one set of bits on every machine. The
platform's - C's libm behind Python's `math`, behind numpy's ufuncs and behind <math.h> - differ in their last bits
from one operating system to the next, and a power (`**`, `pow`) is the platform's pow wherever a float or a numpy
scalar takes it: `x ** 2` is not `x * x` there once in 2000. Counted here, file by file: Python by its syntax tree
(a call or a reference through `math`, through numpy, or by a name imported from either, and every `**` but one of
whole literals), C with its comments and strings taken out. What rounds one way everywhere stays uncounted: sqrt,
fabs, floor, ceil, fmod, copysign, CPython's own hypot and numpy's own tanh.

    python -X utf8 host/tools/cores/libm_calls.py [--files] [PACKAGE ...]
"""
import argparse
import ast
import collections
import os
import re
import sys

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

from tools import REPO

#: `math`'s functions that are the platform's libm's (erf and gamma CPython's own sums over its exp and pow).
MATH = frozenset(('sin cos tan asin acos atan atan2 exp expm1 exp2 log log10 log2 log1p pow sinh cosh tanh asinh acosh '
                  'atanh cbrt erf erfc gamma lgamma').split())
#: numpy's ufuncs that are the platform's libm's on the paths `tools.dev.arithmetic` holds numpy to.
NUMPY = frozenset(('sin cos tan arcsin arccos arctan arctan2 exp expm1 exp2 log log10 log2 log1p hypot sinh cosh arcsinh '
                   'arccosh arctanh cbrt power float_power logaddexp logaddexp2').split())
#: <math.h>'s, each with its float and long double names.
C = frozenset(name + tail for tail in ('', 'f', 'l') for name in (
    'sin cos tan asin acos atan atan2 exp expm1 exp2 log log10 log2 log1p pow sinh cosh tanh asinh acosh atanh cbrt '
    'hypot erf erfc tgamma lgamma').split())
_C_CALL = re.compile(r'(?<![\w.>])(%s)\s*\(' % '|'.join(sorted(C, key=len, reverse=True)))
_C_QUIET = re.compile(r'/\*.*?\*/|//[^\n]*|"(?:\\.|[^"\\\n])*"|\'(?:\\.|[^\'\\\n])*\'', re.S)

#: The robot's Python by package, and its C trees: what `counted` walks.
PYTHON = os.path.join(HOST, 'machine')
C_TREES = ('wbc/src', 'wbc/inc', 'world/src', 'drive/src', 'thermal/src', 'aero/src', 'se3/src', 'mppi/src', 'daq/src')


def python(source):
    """Counter of a Python source's platform calls: 'math.sin', 'np.exp', '**'."""
    found, maths, numpys, names = collections.Counter(), set(), set(), {}
    tree = ast.parse(source)
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            for alias in node.names:
                (maths if alias.name == 'math' else numpys if alias.name == 'numpy' else set()).add(alias.asname or alias.name)
        elif isinstance(node, ast.ImportFrom) and node.module in ('math', 'numpy'):
            listed, kind = (MATH, 'math.') if node.module == 'math' else (NUMPY, 'np.')
            names.update({alias.asname or alias.name: kind + alias.name for alias in node.names if alias.name in listed})
    for node in ast.walk(tree):
        if isinstance(node, ast.Attribute) and isinstance(node.value, ast.Name):
            if node.value.id in maths and node.attr in MATH:
                found['math.' + node.attr] += 1
            elif node.value.id in numpys and node.attr in NUMPY:
                found['np.' + node.attr] += 1
        elif isinstance(node, ast.Name) and isinstance(node.ctx, ast.Load) and node.id in names:
            found[names[node.id]] += 1
        elif isinstance(node, (ast.BinOp, ast.AugAssign)) and isinstance(node.op, ast.Pow) and not _whole(node):
            found['**'] += 1
    return found


def _whole(node):
    """Whether a power is one of whole literals: Python's own exact integers."""
    if not isinstance(node, ast.BinOp):
        return False
    return all(isinstance(side, ast.Constant) and type(side.value) is int and side.value >= 0
               for side in (node.left, node.right))


def c(source):
    """Counter of a C source's <math.h> calls, comments and strings out: 'sin', 'powf'."""
    return collections.Counter(_C_CALL.findall(_C_QUIET.sub(' ', source)))


def counted():
    """{package or C tree: {file: Counter}}: every platform call the robot's code makes, where it stands."""
    out = collections.defaultdict(dict)
    for folder, _, files in os.walk(PYTHON):
        for name in sorted(files):
            if name.endswith('.py'):
                path = os.path.join(folder, name)
                rel = os.path.relpath(path, PYTHON).replace(os.sep, '/')
                with open(path, encoding='utf-8') as f:
                    found = python(f.read())
                if found:
                    out[rel.split('/')[0] if '/' in rel else rel][rel] = found
    for tree in C_TREES:
        folder = os.path.join(REPO, *tree.split('/'))
        for name in sorted(os.listdir(folder)) if os.path.isdir(folder) else ():
            if name.endswith(('.c', '.h')):
                with open(os.path.join(folder, name), encoding='utf-8') as f:
                    found = c(f.read())
                if found:
                    out[tree][name] = found
    return dict(out)


def totals(found=None):
    """{package or C tree: its calls} over `counted`."""
    return {where: sum(sum(n.values()) for n in files.values()) for where, files in (found or counted()).items()}


def main():
    ap = argparse.ArgumentParser(description="The robot's code's calls into the platform's math library, by package.")
    ap.add_argument('only', nargs='*', help='packages or C trees (default: all)')
    ap.add_argument('--files', action='store_true', help='file by file, each call by its name')
    args = ap.parse_args()
    found = counted()
    for where in sorted(found):
        if args.only and where not in args.only:
            continue
        names = collections.Counter()
        for n in found[where].values():
            names.update(n)
        print('%-12s %4d  %s' % (where, sum(names.values()), ' '.join('%s %d' % kv for kv in names.most_common(8))))
        for rel, n in sorted(found[where].items()) if args.files else ():
            print('    %-34s %3d  %s' % (rel, sum(n.values()), ' '.join('%s %d' % kv for kv in n.most_common())))
    print('%-12s %4d' % ('all', sum(totals(found).values())))


if __name__ == '__main__':
    main()
