#!/usr/bin/env python3
"""The host as a map: each module's brief and each public def and class, one line apiece.

The cheapest way into host/:

    python tools/dev/host_map.py                 # modules and their briefs
    python tools/dev/host_map.py --api           # plus public signatures
    python tools/dev/host_map.py --api coaxial/kalman terminal   # just those
    python tools/dev/host_map.py --layers        # which package imports which, counted
    python tools/dev/host_map.py --deps coaxial  # each module's host imports
"""
import argparse
import ast
import glob
import io
import os
import sys
from collections import Counter

TREE = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DIRS = ('coaxial', 'coaxial_mcp', 'coaxial_ollama', 'machine', 'terminal', 'testline', 'tools',
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


def modules(dirs):
    """(path under host/, dotted name, syntax tree) for every module."""
    for d in dirs:
        for path in sorted(glob.glob(os.path.join(TREE, d, '**', '*.py'), recursive=True)):
            if '__pycache__' not in path:
                rel = os.path.relpath(path, TREE).replace(os.sep, '/')
                dotted = rel[:-3].replace('/', '.').replace('.__init__', '')
                yield rel, dotted, ast.parse(io.open(path, encoding='utf-8').read())


def group(rel):
    """A module's package, as a directory: coaxial/graphics, tools, ..."""
    return rel.rsplit('/', 1)[0] if '/' in rel else '.'


def imports(rel, dotted, tree, known):
    """The host modules `tree` imports: relative names against its package,
    bare names against tools/ (the views put it on the path)."""
    package = dotted if rel.endswith('__init__.py') else dotted.rpartition('.')[0]
    found = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            names = [a.name for a in node.names]
        elif isinstance(node, ast.ImportFrom):
            base = node.module or ''
            if node.level:
                parts = package.split('.')[:len(package.split('.')) - node.level + 1]
                base = '.'.join(parts + ([base] if base else []))
            names = [base + '.' + a.name for a in node.names]
        else:
            continue
        for name in names:
            hit = next((n for n in (name, 'tools.' + name) if n in known), None)
            while hit is None and '.' in name:
                name = name.rpartition('.')[0]
                hit = next((n for n in (name, 'tools.' + name) if n in known), None)
            if hit and hit != dotted and hit not in found:
                found.append(hit)
    return found


def show_graph(dirs, by_layer):
    everything = list(modules(DIRS))
    known = {dotted: rel for rel, dotted, _ in everything}
    edges = {}
    for rel, dotted, tree in everything:
        if not any(rel.startswith(d.rstrip('/') + '/') or rel == d for d in dirs):
            continue
        found = imports(rel, dotted, tree, known)
        if not by_layer:
            if found:
                own = dotted.rpartition('.')[0] + '.'
                print('%s: %s' % (rel, ' '.join(d[len(own) - 1:] if d.startswith(own) else d
                                                for d in found)))
            continue
        for dep in found:
            if group(known[dep]) != group(rel):
                edges.setdefault(group(rel), Counter())[group(known[dep])] += 1
    for src in sorted(edges):
        print('%s -> %s' % (src, ', '.join('%s %d' % kv for kv in edges[src].most_common())))


def main(argv=None):
    parser = argparse.ArgumentParser(description=brief(__doc__))
    parser.add_argument('dirs', nargs='*', default=DIRS)
    parser.add_argument('--api', action='store_true', help='public signatures too')
    parser.add_argument('--layers', action='store_true', help='the import graph by package')
    parser.add_argument('--deps', action='store_true', help="each module's host imports")
    args = parser.parse_args(argv)
    sys.stdout.reconfigure(encoding='utf-8')   # a cp1252 pipe cannot carry every brief
    if args.layers or args.deps:
        return show_graph(args.dirs, args.layers)
    for rel, _, tree in modules(args.dirs):
        print('%s: %s' % (rel, brief(ast.get_docstring(tree))))
        if args.api:
            for indent, node in public(tree):
                doc = brief(ast.get_docstring(node))
                print('    %s%s%s' % (indent, signature(node), ' - ' + doc if doc else ''))
    return 0


if __name__ == '__main__':
    sys.exit(main())
