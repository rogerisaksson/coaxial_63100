#!/usr/bin/env python3
"""The firmware as a map: each file's one-line brief and each header's public functions.

One line apiece: the cheapest way into the target.

    python tools/target_map.py            # files and their briefs
    python tools/target_map.py --api      # plus every header's prototypes
    python tools/target_map.py --api drive boot   # just those directories
"""
import argparse
import glob
import io
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
DIRS = ('board', 'comms', 'boot', 'modbus', 'daq', 'drive', 'thermal', 'filter', 'shtp')
BRIEF = re.compile(r'\A﻿?\s*/\*\*?\s*(\S+?)\s+-\s+(.*?)\*/', re.S)
PROTOTYPE = re.compile(r'^[A-Za-z_][\w \*]*\b\w+\s*\([^;{}]*\)\s*;', re.M)


def brief(text):
    m = BRIEF.match(text)
    return ' '.join(m.group(2).split()) if m else ''


def prototypes(text):
    code = re.sub(r'/\*.*?\*/|//[^\n]*', '', text, flags=re.S)
    return [' '.join(p.split()) for p in PROTOTYPE.findall(code)
            if not p.lstrip().startswith(('typedef', 'return', 'static'))]


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('dirs', nargs='*', default=DIRS)
    parser.add_argument('--api', action='store_true', help="headers' prototypes too")
    args = parser.parse_args(argv)
    for d in args.dirs:
        for path in sorted(glob.glob(os.path.join(ROOT, d, '**', '*.[chs]'), recursive=True)):
            rel = os.path.relpath(path, ROOT).replace(os.sep, '/')
            if '/test/' in rel:
                continue
            text = io.open(path, encoding='utf-8', errors='replace').read()
            print('%s: %s' % (rel, brief(text)))
            if args.api and path.endswith('.h'):
                for p in prototypes(text):
                    print('    ' + p)
    return 0


if __name__ == '__main__':
    sys.exit(main())
