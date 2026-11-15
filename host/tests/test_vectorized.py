#!/usr/bin/env python3
"""The compiler vectorises the robot's cores as its table says (`tools/cores/vectorized.py`): no loop is lost.

Each of the cores' sources built for each target this host's compiler builds - avx, avx2 and avx512 on x86; sve2
and sme2 on ARM - and the compiler's own report read: this host's compiler and machine have their section of the
table, every row names a function that stands, and no function holds fewer vectorised loops than its row (more is
no fault). Nothing is contracted: a fused multiply-add stands in a source's assembly only where its text, or an
inline helper it calls, asks one by name. And the reader bites: a loop the compiler vectorises and the same loop
with a dependence carried from pass to pass are told apart.
The cores' arithmetic against numpy's, to the bit: test_wbc_core, test_tick."""
import functools
import json
import os
import re
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A fused multiply-add asked by name; a loop whose lanes are independent, and the same carried from pass to pass.
ASKED = re.compile(r'\bfma\(')
APART = ('void apart(double * restrict a, const double * restrict b, const double * restrict c)\n{\n'
         '  for (int i = 0; i < 1024; i++)\n  {\n    a[i] = b[i] * c[i] + b[i];\n  }\n}\n')
CARRIED = ('void carried(double * restrict a, const double * restrict b, const double * restrict c)\n{\n'
           '  for (int i = 1; i < 1024; i++)\n  {\n    a[i] = a[i - 1] * b[i] + c[i];\n  }\n}\n')


@functools.lru_cache(maxsize=None)
def _told():
    """This host's report (`vectorized.report`), built once a run."""
    from tools.cores import vectorized
    return vectorized.report()


def test_no_function_loses_a_vectorised_loop(report, _c):
    """This host's compiler and machine have their section of the table; every row of it names a function that
    stands in its source; and no function holds fewer vectorised loops than its row, a target at a time."""
    from tools.cores import vectorized
    key, targets, table, _fused = _told()
    rows = vectorized.laid()
    # a host no one reaches by hand (a runner) tells its section whole where it has none, for the tool's --take
    report.check("this host's compiler and machine have their section of the table: %s" % key, key in rows,
                 'lay it: python tools/cores/vectorized.py --write (a container: --json there, --take here)'
                 + ('' if key in rows else '; its section: ' + json.dumps({key: [list(targets), table]})))
    if key not in rows:
        return
    of, laid = rows[key]
    stand = {name: {fn for fn, _first, _last in vectorized.spans(path)} for name, path in vectorized.sources().items()}
    gone = sorted('%s %s' % (name, fn) for name, fns in laid.items() for fn in fns if fn not in stand.get(name, ()))
    report.check('every row names a function that stands, its loops on %s' % ', '.join(targets),
                 tuple(of) == tuple(targets) and not gone and len(laid) > 0, ', '.join(gone[:6]) or '%d sources' % len(laid))
    lost = ['%s %s on %s: %d where %d' % (name, fn, isa, now, was)
            for name, fns in sorted(laid.items()) for fn, counts in sorted(fns.items())
            for isa, was, now in zip(targets, counts, table.get(name, {}).get(fn, (0,) * len(targets))) if now < was]
    held = sum(sum(counts) for fns in laid.values() for counts in fns.values())
    report.check('no function holds fewer vectorised loops than its row', not lost,
                 '; '.join(lost[:6]) or '%d loops held in %d functions' % (held, sum(len(fns) for fns in laid.values())))


def test_nothing_is_contracted(report, _c):
    """A fused multiply-add stands in a source's assembly only where its text asks one by name, or calls an inline
    helper of the cores' headers that does; and the reader sees those that are asked."""
    from tools.cores import vectorized, wbc
    _key, targets, _table, fused = _told()
    helpers, bodies = set(), {}
    for folder in wbc.INCLUDES:
        for name in sorted(os.listdir(folder)):
            if name.endswith('.h'):
                with open(os.path.join(folder, name), encoding='utf-8') as f:
                    lines = f.read().split('\n')
                bodies.update({fn: '\n'.join(lines[first:last]) for fn, first, last in vectorized.spans(os.path.join(folder, name))})
    grew = True
    while grew:
        more = {fn for fn, body in bodies.items() if fn not in helpers
                and (ASKED.search(body) or any(re.search(r'\b%s\(' % h, body) for h in helpers))}
        grew, helpers = bool(more), helpers | more
    asked = set()
    for name, path in vectorized.sources().items():
        with open(path, encoding='utf-8') as f:
            source = f.read()
        if ASKED.search(source) or any(re.search(r'\b%s\(' % h, source) for h in helpers):
            asked.add(name)
    seen = {name for isa in targets for name in fused[isa]}
    stray = sorted('%s on %s (%d)' % (name, isa, n) for isa in targets for name, n in fused[isa].items() if name not in asked)
    report.check('a fused multiply-add stands only where a source asks one by name', not stray and len(seen) > 0,
                 ', '.join(stray[:6]) or 'asked in %s; seen in %s' % (', '.join(sorted(asked)), ', '.join(sorted(seen))))


def test_the_reader_bites(report, _c):
    """A loop whose lanes are independent and the same loop with a dependence carried from pass to pass, each built
    as a core is for each of this host's targets: the reader tells the first vectorised and the second not."""
    from tools.cores import build, vectorized
    cc, told = build.find_cc(), []
    targets = vectorized.targets(cc)
    with tempfile.TemporaryDirectory() as folder:
        for name, source in (('apart.c', APART), ('carried.c', CARRIED)):
            with open(os.path.join(folder, name), 'w', encoding='utf-8', newline='\n') as f:
                f.write(source)
        for isa in targets:
            told.append((isa, vectorized.read(cc, isa, os.path.join(folder, 'apart.c'))[0],
                         vectorized.read(cc, isa, os.path.join(folder, 'carried.c'))[0]))
    report.check('a loop whose lanes are independent is told vectorised, one carried from pass to pass is not',
                 all(apart == {'apart': 1} and not carried for _isa, apart, carried in told),
                 '; '.join('%s: %s, %s' % row for row in told))


ROSTER = (test_no_function_loses_a_vectorised_loop, test_nothing_is_contracted, test_the_reader_bites)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    from tools.cores.build import find_cc
    if find_cc() is None:
        print('  SKIP  no host C compiler; setup.ps1 installs one')
        print('\n0 passed, 0 failed')
        return 0
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report, None)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
