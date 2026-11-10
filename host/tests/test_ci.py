#!/usr/bin/env python3
"""What CI left red leads the suites that run first (`tools.dev.ci`): a tail read, the tuple laid.

No network: a gate's log tail as its comment holds it, and suites.py's own text."""
import os
import sys

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


ROSTER = (test_a_log_tail_names_its_red_suites, test_the_red_suites_lead_those_that_run_first)


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
