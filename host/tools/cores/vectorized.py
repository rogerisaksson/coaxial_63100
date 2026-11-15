#!/usr/bin/env python3
"""The loops of the robot's cores the compiler vectorises, by its own report, a compiler at a time.

    python tools/cores/vectorized.py                    # this host's section: its functions' vectorised loops a target
    python tools/cores/vectorized.py --write            # ... laid into tools/cores/vectorized_rows.py, the others kept
    python tools/cores/vectorized.py --json             # ... as JSON: a container, whose tree is a copy
    python tools/cores/vectorized.py --take ROWS.json   # a section printed elsewhere laid into the table

The cores are portable C11 the compiler vectorises where it can: at -O2 with nothing contracted (`wbc.FLAGS`), a
loop whose lanes are independent. An edit can lose that without a suite going red - a pointer that may alias, a
branch in a loop, a sum carried from pass to pass - and the loop runs on, a lane at a time. So each source is built
for each target its machine's compiler builds (BUILDS: avx, avx2 and avx512 on x86; sve2 and sme2 on ARM, in the
arm64 runner image under QEMU), to assembly and nowhere else, and the compiler's own report read
(-fopt-info-vec-optimized): the loops it vectorised, each once however often it tells it, by the function they
stand in (`spans`). The table holds them a section - a compiler, its major version, its machine: what one compiler
vectorises another may not - and test_vectorized fails where a function holds fewer than its row; more is no
fault. The assembly's fused multiply-adds are counted with them (FUSED): nothing is contracted, so one stands only
where a source asks it by name.
"""
import argparse
import json
import os
import platform
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, HOST)

#: The table, and its first line.
PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'vectorized_rows.py')
NOTICE = "The cores' vectorised loops: written by host/tools/cores/vectorized.py, not by hand."
#: A machine by its names; the targets its compiler builds (`build.TARGETS`), in the table's order.
MACHINES = {'amd64': 'x86_64', 'x86_64': 'x86_64', 'arm64': 'aarch64', 'aarch64': 'aarch64'}
BUILDS = {'x86_64': ('avx', 'avx2', 'avx512'), 'aarch64': ('sve2', 'sme2')}
#: A function's head at column 0; a fused multiply-add's mnemonic, x86's and ARM's.
HEAD = re.compile(r'^(?:static\s+)?(?:inline\s+)?[A-Za-z_][\w\s\*]*?\b(\w+)\s*\([^;]*$')
FUSED = re.compile(r'^\s+(?:vfn?m(?:add|sub)\w*|fn?m(?:add|sub)|fn?ml[as]|fn?m(?:ad|sb))\s')
#: Compiles side by side at most.
AT_ONCE = 16


def machine():
    """This host's machine, as the table names it."""
    name = platform.machine().lower()
    return MACHINES.get(name, name)


def section(cc):
    """This host's section's name: its compiler `cc`, its major version, its machine."""
    said = subprocess.run([cc, '--version'], capture_output=True, text=True).stdout.lower()
    major = subprocess.run([cc, '-dumpversion'], capture_output=True, text=True).stdout.strip().split('.')[0]
    return '%s %s %s' % ('clang' if 'clang' in said else 'gcc', major, machine())


def sources():
    """{name: path} of the robot's cores' own sources: `wbc.SOURCES`' wbc_*.c."""
    from tools.cores import wbc
    return {os.path.basename(s): s for s in wbc.built(wbc.SOURCES) if os.path.basename(s).startswith('wbc_')}


def flags(isa):
    """The cores' flags for target `isa`: `wbc.FLAGS` with that target's in place of this host's, their includes."""
    from tools.cores import build, wbc
    own = set(build.native()) - {'-ffp-contract=off'}
    return (build.FLAGS + [f for f in wbc.FLAGS if f not in own] + list(build.TARGETS[isa])
            + [x for d in wbc.INCLUDES for x in ('-I', d)])


def spans(path):
    """[(function, its first line, its last)] of C source `path`: its heads and closing braces at column 0."""
    out, name, start = [], None, 0
    with open(path, encoding='utf-8') as f:
        for n, line in enumerate(f, 1):
            head = HEAD.match(line)
            if head and name is None and not line.startswith(('typedef', 'struct', 'enum', 'union', '#')):
                name, start = head.group(1), n
            elif line.startswith('}') and name is not None:
                out.append((name, start, n))
                name = None
    return out


def read(cc, isa, path):
    """({function: its vectorised loops}, its assembly's fused multiply-adds) of source `path` built by `cc` for
    target `isa`: the loops the compiler tells vectorised in the source itself, each place once."""
    done = subprocess.run([cc, '-S', '-o', '-'] + flags(isa) + ['-fopt-info-vec-optimized', path],
                          capture_output=True, text=True, encoding='utf-8', errors='replace')
    if done.returncode:
        raise RuntimeError('%s does not build for %s: %s' % (os.path.basename(path), isa, done.stderr[-400:]))
    told = re.findall(re.escape(os.path.basename(path)) + r':(\d+):(\d+): optimized: loop vectorized', done.stderr)
    at = {(int(line), int(col)) for line, col in told}
    loops = {}
    for name, first, last in spans(path):
        held = sum(first <= line <= last for line, _col in at)
        if held:
            loops[name] = held
    return loops, sum(bool(FUSED.match(line)) for line in done.stdout.splitlines())


def report():
    """(this host's section's name, its targets, {source: {function: (its loops a target)}}, {target: {source: its
    fused multiply-adds}}): every source built for every target, AT_ONCE side by side."""
    from tools.cores import build
    cc, targets, files = build.find_cc(), BUILDS[machine()], sources()
    jobs = [(isa, name) for isa in targets for name in sorted(files)]
    with ThreadPoolExecutor(min(AT_ONCE, os.cpu_count() or 2)) as pool:
        got = list(pool.map(lambda job: read(cc, job[0], files[job[1]]), jobs))
    table, fused = {}, {isa: {} for isa in targets}
    for (isa, name), (loops, n) in zip(jobs, got):
        if n:
            fused[isa][name] = n
        for function, held in loops.items():
            table.setdefault(name, {}).setdefault(function, [0] * len(targets))[targets.index(isa)] = held
    return section(cc), targets, {name: {fn: tuple(v) for fn, v in fns.items()} for name, fns in table.items()}, fused


def laid():
    """{section: (its targets, {source: {function: (loops a target)}})} as the table holds it; none where none is."""
    if not os.path.exists(PATH):
        return {}
    from tools.cores import vectorized_rows
    return dict(vectorized_rows.ROWS)


def text(rows):
    """The table's text for `rows` (`laid`'s shape): sections, sources and functions in their names' order."""
    out = ['"""%s"""' % NOTICE,
           '# A section a compiler, its major version and its machine: its targets, then each source\'s functions\'',
           '# vectorised loops a target. A loop lost fails test_vectorized; the tool\'s --write lays a host\'s section.',
           'ROWS = {']
    for key in sorted(rows):
        targets, table = rows[key]
        out.append('    %r: (%r, {' % (key, tuple(targets)))
        for name in sorted(table):
            out.append('        %r: {' % name)
            out += ['            %r: %r,' % (fn, tuple(table[name][fn])) for fn in sorted(table[name])]
            out.append('        },')
        out.append('    }),')
    return '\n'.join(out + ['}']) + '\n'


def written(rows):
    """The table laid for `rows`, LF alone."""
    with open(PATH, 'w', encoding='utf-8', newline='\n') as f:
        f.write(text(rows))


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--write', action='store_true', help="lay this host's section into the table, the others kept")
    p.add_argument('--json', action='store_true', help="this host's section as JSON on stdout")
    p.add_argument('--take', metavar='ROWS.json', help='lay the sections a --json printed elsewhere into the table')
    args = p.parse_args(argv)
    rows = laid()
    if args.take:
        with open(args.take, encoding='utf-8-sig') as f:
            for key, (targets, table) in json.load(f).items():
                rows[key] = (tuple(targets), {name: {fn: tuple(v) for fn, v in fns.items()}
                                              for name, fns in table.items()})
        written(rows)
        print('vectorized: %s laid in %s' % (', '.join(sorted(rows)), os.path.relpath(PATH, HOST)))
        return 0
    key, targets, table, fused = report()
    if args.json:
        print(json.dumps({key: [list(targets), table]}))
        return 0
    if args.write:
        rows[key] = (targets, table)
        written(rows)
    for k, isa in enumerate(targets):
        held = [v[k] for fns in table.values() for v in fns.values() if v[k]]
        print('%-22s %-7s %3d functions hold %3d vectorised loops; fused multiply-adds in %s' % (
            key, isa, len(held), sum(held), ', '.join('%s (%d)' % item for item in sorted(fused[isa].items())) or 'none'))
    return 0


if __name__ == '__main__':
    sys.exit(main())
