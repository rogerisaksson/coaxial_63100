#!/usr/bin/env python3
"""The host as a map: each module's brief and each public def and class, one line apiece.

The cheapest way into host/:

    python tools/host_map.py                 # modules and their briefs
    python tools/host_map.py --api           # plus public signatures
    python tools/host_map.py --api coaxial/kalman terminal   # just those
"""
import argparse
import ast
import glob
import io
import os
import sys

TREE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DIRS = ('coaxial', 'coaxial_mcp', 'coaxial_ollama', 'terminal', 'testline', 'tools',
        'examples')
WIDTH = 100


def brief(doc):
    """A docstring's first sentence on one line, cut at WIDTH."""
    first = ' '.join((doc or '').split('\n\n')[0].split())
    end = first.find('. ')
    first = first if end < 0 else first[:end + 1]
    return first if len(first) <= WIDTH else first[:WIDTH - 3] + '...'


def signature(node):
    """`name(args)` for a def, `Name(bases)` for a class."""
    if isinstance(node, ast.ClassDef):
        return '%s(%s)' % (node.name, ', '.join(ast.unparse(b) for b in node.bases))
    return '%s(%s)' % (node.name, ast.unparse(node.args))


def public(tree):
    """The module's public defs and classes, and a class's public methods."""
    for node in tree.body:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)) \
                and not node.name.startswith('_'):
            yield '', node
            if isinstance(node, ast.ClassDef):
                for sub in node.body:
                    if isinstance(sub, (ast.FunctionDef, ast.AsyncFunctionDef)) \
                            and not sub.name.startswith('_'):
                        yield '  ', sub


def main(argv=None):
    parser = argparse.ArgumentParser(description=brief(__doc__))
    parser.add_argument('dirs', nargs='*', default=DIRS)
    parser.add_argument('--api', action='store_true', help='public signatures too')
    args = parser.parse_args(argv)
    sys.stdout.reconfigure(encoding='utf-8')   # a cp1252 pipe cannot carry every brief
    for d in args.dirs:
        for path in sorted(glob.glob(os.path.join(TREE, d, '**', '*.py'), recursive=True)):
            if '__pycache__' in path:
                continue
            tree = ast.parse(io.open(path, encoding='utf-8').read())
            print('%s: %s' % (os.path.relpath(path, TREE).replace(os.sep, '/'),
                              brief(ast.get_docstring(tree))))
            if args.api:
                for indent, node in public(tree):
                    doc = brief(ast.get_docstring(node))
                    print('    %s%s%s' % (indent, signature(node), ' - ' + doc if doc else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
