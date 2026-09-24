#!/usr/bin/env python3
"""The firmware as a map: briefs, prototypes, the include graph and what each command reaches.

One line apiece: the cheapest way into the target.

    python tools/dev/target_map.py            # files and their briefs
    python tools/dev/target_map.py --api      # plus every header's prototypes
    python tools/dev/target_map.py --api drive boot   # just those directories
    python tools/dev/target_map.py --layers   # which layer includes which, counted
    python tools/dev/target_map.py --deps comms       # each file's tree headers
    python tools/dev/target_map.py --ops      # command -> handler -> public calls
"""
import argparse
import glob
import io
import os
import re
import sys
from collections import Counter

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))
DIRS = ('board', 'comms', 'boot', 'modbus', 'daq', 'drive', 'thermal', 'filter', 'shtp')
BRIEF = re.compile(r'\A﻿?\s*/\*\*?\s*(\S+?)\s+-\s+(.*?)\*/', re.S)
PROTOTYPE = re.compile(r'^[A-Za-z_][\w \*]*\b\w+\s*\([^;{}]*\)\s*;', re.M)
INCLUDE = re.compile(r'^\s*#\s*include\s+"([^"]+)"', re.M)
DEFINITION = re.compile(r'^(?:static\s+)?[A-Za-z_][\w \*]*?\b(\w+)\s*\([^;{}]*\)\s*\{', re.M)
CALL = re.compile(r'\b([A-Za-z_]\w*)\s*\(')
CASE = re.compile(r'case\s+(\w+?)_OP_(\w+)\s*:\s*return\s+(\w+)\s*\(')
ROW = re.compile(r'\{\s*(CMD_\w+)\s*,\s*"(\w+)"\s*,[^,{}]+,\s*(\w+)\s*\}')


def brief(text):
    m = BRIEF.match(text)
    return ' '.join(m.group(2).split()) if m else ''


def prototypes(text):
    code = re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)
    return [' '.join(p.split()) for p in PROTOTYPE.findall(code)
            if not p.lstrip().startswith(('typedef', 'return', 'static'))]


def files(dirs):
    """(path from the root, text) for every C, header and assembly file."""
    for d in dirs:
        for path in sorted(glob.glob(os.path.join(ROOT, d, '**', '*.[chs]'), recursive=True)):
            rel = os.path.relpath(path, ROOT).replace(os.sep, '/')
            if '/test/' not in rel:
                yield rel, io.open(path, encoding='utf-8', errors='replace').read()


def layer(rel):
    """A file's layer: its top directory; comms/inc/board* is the board's API."""
    return 'board' if rel.startswith('comms/inc/board') else rel.split('/')[0]


def resolver():
    """`#include "x"` -> the tree's header, found as the compiler finds it:
    beside the includer first, then in every inc/ directory."""
    headers = {rel for rel, _ in files(DIRS + ('core',)) if rel.endswith('.h')}
    incs = sorted({h.split('/inc/')[0] + '/inc' for h in headers if '/inc/' in h})

    def resolve(name, includer):
        for base in [includer.rsplit('/', 1)[0]] + incs:
            if base + '/' + name in headers:
                return base + '/' + name
        return None
    return resolve


def deps(rel, text, resolve):
    """The tree headers `rel` includes (the toolchain's are left out)."""
    found = (resolve(name, rel) for name in INCLUDE.findall(text))
    return [h for h in found if h and h != rel]


def short(header):
    return (header.split('/inc/', 1)[1] if '/inc/' in header else header.rsplit('/', 1)[1])[:-2]


def show_layers(dirs):
    resolve, edges = resolver(), {}
    for rel, text in files(dirs):
        for h in deps(rel, text, resolve):
            if layer(h) != layer(rel):
                edges.setdefault(layer(rel), Counter())[layer(h)] += 1
    for src in sorted(edges):
        print('%s -> %s' % (src, ', '.join('%s %d' % kv for kv in edges[src].most_common())))


def show_deps(dirs):
    resolve = resolver()
    for rel, text in files(dirs):
        found = deps(rel, text, resolve)
        if found:
            print('%s: %s' % (rel, ' '.join(short(h) for h in found)))


def bodies(text):
    """{name: body} for every function `text` defines."""
    found = {}
    for m in DEFINITION.finditer(text):
        end = text.find('\n}', m.end())
        found[m.group(1)] = text[m.end():end if end > 0 else len(text)]
    return found


def reached(name, defined, public, seen=None):
    """The public functions `name` calls, through the file's own helpers."""
    seen = set() if seen is None else seen
    seen.add(name)
    out = set()
    for call in CALL.findall(defined.get(name, '')):
        if call in public:
            out.add(call)
        elif call in defined and call not in seen:
            out |= reached(call, defined, public, seen)
    return out


def show_ops():
    public = {re.search(r'(\w+)\s*\(', p).group(1)
              for rel, text in files(DIRS) if rel.endswith('.h') and not rel.endswith('/wire.h')
              for p in prototypes(text)}
    for rel, text in files(('comms',)):
        defined = bodies(text)
        rows = ([(code, what, fn) for code, what, fn in ROW.findall(text)]
                + [('%s.%s' % (dev, op), '', fn) for dev, op, fn in CASE.findall(text)])
        for code, what, fn in rows:
            calls = sorted(reached(fn, defined, public))
            print('%s %s%s: %s' % (code, what + ' ' if what else '', fn, ' '.join(calls)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('dirs', nargs='*')
    parser.add_argument('--api', action='store_true', help="headers' prototypes too")
    parser.add_argument('--layers', action='store_true', help='the include graph by layer')
    parser.add_argument('--deps', action='store_true', help="each file's tree headers")
    parser.add_argument('--ops', action='store_true', help='command -> handler -> public calls')
    args = parser.parse_args(argv)
    sys.stdout.reconfigure(encoding='utf-8')   # a cp1252 pipe cannot carry every brief
    if args.ops:
        return show_ops()
    if args.layers or args.deps:
        show = show_layers if args.layers else show_deps
        return show(args.dirs or DIRS + ('core',))
    for rel, text in files(args.dirs or DIRS):
        print('%s: %s' % (rel, brief(text)))
        if args.api and rel.endswith('.h'):
            for p in prototypes(text):
                print('    ' + p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
