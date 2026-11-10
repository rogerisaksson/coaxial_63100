#!/usr/bin/env python3
"""What CI left red leads the suites that run first (`tools.dev.ci`): a tail read, the tuple laid.

No network: a gate's log tail as its comment holds it, and suites.py's own text. And a run fits the host it
is on (`tools.dev.machine_detector`): a suite's seconds on hosts laid here, this one measured."""
import json
import os
import pathlib
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.dev.focus import chosen
from gynoid_kit import Report

#: A gate's log tail as CI's comment on a commit holds it: a suite red on a check, one crashed, the rest green.
TAIL = ('CI offline (3.14) failed. Log tail:\n\n~~~\n'
        'test_structure.py    2557 passed, 0 failed  41.7s\n'
        'test_wasd.py         22 passed, 1 failed  479.6s\n'
        'test_sight.py        CRASHED exit=3221225477 12.0s\n'
        'test_wasd.py         22 passed, 1 failed  479.6s\n'
        'Total: ~2673  Passed: 2672, Skipped: ~0, Failed: 1, Crashed: 0, (7 of 125 suites ran)\n'
        '  test_wasd.py: no fall and no stumble on the keys from any start (13 spasms)\n~~~\n')


def test_a_log_tail_names_its_red_suites(report):
    """`ci.red`: the suites that failed a check or crashed, in their order, each once; a green tail none."""
    from tools.dev import ci
    report.check('a tail read: the suite red on a check and the one crashed, each once; the green ones not',
                 ci.red(TAIL) == ['test_wasd.py', 'test_sight.py'], ci.red(TAIL))
    report.check('a green tail names none', ci.red('test_structure.py    2557 passed, 0 failed  41.7s\n') == [])


def test_the_red_suites_lead_those_that_run_first(report):
    """`ci.led` on suites.py's own text: CI's red suites at the head of FAILED_FIRST, the rest after in their
    order, none twice, the text about the tuple untouched; nothing laid where they lead already."""
    import ast
    from tools.dev import ci, suites
    with open(os.path.join(os.path.dirname(os.path.abspath(suites.__file__)), 'suites.py'), encoding='utf-8') as f:
        source = f.read()
    was = list(suites.FAILED_FIRST)
    lead = [was[-1], 'test_never_there.py']
    laid = ci.led(source, lead)
    now = next(ast.literal_eval(node.value) for node in ast.parse(laid).body
               if isinstance(node, ast.Assign) and getattr(node.targets[0], 'id', '') == 'FAILED_FIRST')
    report.check("CI's red suites at the head of FAILED_FIRST, the others after in their order, none twice",
                 list(now) == lead + was[:-1], list(now)[:4])
    cut = lambda text: ci.FIRST.sub('', text)
    report.check('the text about the tuple untouched, its lines no wider than before',
                 cut(laid) == cut(source) and max(map(len, laid.splitlines())) <= max(map(len, source.splitlines())))
    report.check('suites that lead already, or none: the same text',
                 ci.led(source, was[:2]) == source and ci.led(source, []) == source and ci.led('x = 1\n', lead) == 'x = 1\n')


def test_a_suite_s_seconds_are_the_host_s(report):
    """`machine_detector.expected` on hosts laid here - a runner of four cores half as fast, a host twice as
    fast -: the tree's record by the pace it was run at, the table's work where it has none; and this host
    measured at two batons."""
    from tools.dev import machine_detector as m
    seconds, paces, work = {'test_own.py': 100.0, 'test_old.py': 30.0}, {'test_own.py': 8.0}, {'test_far.py': 1600.0,
                                                                                               'test_own.py': 1.0}
    slow, fast = (m.expected(seconds, paces, pace, work) for pace in (4.0, 16.0))
    report.check('run at 8 kernels a second, 100 s: 200 s on a host half as fast, 50 s on one twice as fast',
                 (slow['test_own.py'], fast['test_own.py']) == (200.0, 50.0), (slow['test_own.py'], fast['test_own.py']))
    report.check("a suite the tree has no record of: its work in the table over the host's pace; one with no pace "
                 'kept: as it stands', (slow['test_far.py'], fast['test_far.py'], slow['test_old.py']) == (400.0, 100.0, 30.0),
                 (slow['test_far.py'], fast['test_far.py'], slow['test_old.py']))
    new = m.expected(seconds, paces, 4.0, work, suites=('test_new.py', 'test_own.py'))
    report.check('a suite neither knows: as the heaviest they know till its first whole run, 400 s; one known: its own',
                 (new['test_new.py'], new['test_own.py']) == (400.0, 200.0), (new['test_new.py'], new['test_own.py']))
    here = m.host(2)
    report.check('this host measured: its cores, its threads, a pace at two batons',
                 here['cores'] >= 1 and here['threads'] >= here['cores'] and here['width'] == 2 and here['pace'] > 0.0,
                 m.said(here))


def test_the_gate_s_suites_have_their_work(report):
    """tests/work.json holds a row a suite of the offline gate - the gate a host with no record of its own
    runs, GitHub's runner: one without is planned as the heaviest suite, not as itself (laid from a gate
    stopped at its first red suite, the table had none of eleven). A new suite's row: run it whole, then
    `machine_detector.py --keep`."""
    from tools.dev import machine_detector, suites
    have = {*suites.NEEDS_BOARD, *machine_detector.table()}
    lacking = [name for name in suites.DEFAULT_SUITES if name not in have]
    report.check('every suite of the offline gate has its work in the table', not lacking, lacking)


def test_a_table_laid_keeps_what_was_not_run(report):
    """`machine_detector.keep` on a table and a record laid here: a suite the record has not keeps its row -
    read after the file was opened to be written, a run of eighteen suites left a table of 109 those
    eighteen - and a row is raised, never lowered: a tree's later run is lighter than its first."""
    from tools.dev import machine_detector as m
    record = {'seconds': {'test_own.py': 100.0, 'test_warm.py': 10.0, 'test_old.py': 30.0},
              'paces': {'test_own.py': 8.0, 'test_warm.py': 8.0}}
    with tempfile.TemporaryDirectory() as folder:
        path = pathlib.Path(folder, 'work.json')
        path.write_text(json.dumps({'test_far.py': 1600.0, 'test_own.py': 1.0, 'test_warm.py': 900.0}),
                        encoding='utf-8')
        told, held = m.keep(record, path), m.table(path)
    report.check('a suite run whole: its seconds by its pace; one with no pace: not told',
                 told == {'test_own.py': 800.0, 'test_warm.py': 80.0}, told)
    report.check('the table: the suite not run keeps its row, a heavier run raises one, a lighter run lowers none',
                 held == {'test_far.py': 1600.0, 'test_own.py': 800.0, 'test_warm.py': 900.0}, held)


ROSTER = (test_a_log_tail_names_its_red_suites, test_the_red_suites_lead_those_that_run_first,
          test_a_suite_s_seconds_are_the_host_s, test_the_gate_s_suites_have_their_work,
          test_a_table_laid_keeps_what_was_not_run)


def main(argv=None):
    """Every test, or those the command line's words name, or its --shard k/n (tools.dev.focus)."""
    report = Report()
    for test in chosen(ROSTER, sys.argv[1:] if argv is None else argv):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
