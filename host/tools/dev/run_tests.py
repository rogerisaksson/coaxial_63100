#!/usr/bin/env python3
"""Run this project's suites and print one deterministic tally.

The numbers each suite already counts itself, never a summary an LLM was
asked to write.

That distinction is the point. A model relaying its own paraphrase of "did
the tests pass" is the failure mode documented across this codebase's own
FINDINGS.md and MODELS.md: a plausible sentence standing in for a fact
nobody actually checked. This script never asks anyone to summarise
anything - it parses the exact "  PASS "/"  FAIL " lines a human reads
running these files directly, and repeats only what it counted.

    python tools/dev/run_tests.py                 # test_ollama, test_mcp, test_simulated
    python tools/dev/run_tests.py --conformance    # + test_conformance.py (needs a real board)
    python tools/dev/run_tests.py --live           # + test_live_model.py (board AND ollama)
    python tools/dev/run_tests.py --file test_mcp.py

Exit code is 0 only if every requested suite ran and nothing in it failed.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tests import counts  # noqa: E402
from tools.dev.runner import _results  # noqa: E402
from tools.dev.scope import _plan, hold_model, release_model  # noqa: E402
from tools.dev.suites import (ALL_SUITES, ALONE, CONFORMANCE, DEFAULT_SUITES,  # noqa: E402
                              FULL_EVERY, LIVE, NEEDS_BOARD, ROOT, STRUCTURE, TIERS)
from tools.target import find_board  # noqa: E402

#: Suites run side by side: their wall time is mostly the stand-ins' sleep
#: (2026-09-21: sensorless 24 s of CPU in 151 s, the DAQ front door 0.9 in
#: 72, the broker 2.2 in 31); one after another was a 400 s gate. Half the
#: cores, at most four: no page file here, and one views page is 14
#: processes.
JOBS = max(1, min(4, (os.cpu_count() or 2) // 2))


def board_note():
    """One line saying whether the board answered, printed only when a suite
    that needs it failed.
    """
    try:
        ports = find_board.list_ports()
    except OSError as exc:                                 # the port listing
        return '  (could not check whether the board is attached: %s)' % exc
    if not ports:
        return ('  NOTE: Windows sees no COM ports at all - every failure '
                'above in %s is the cable, not the code.'
                % ', '.join(NEEDS_BOARD))
    return ('  NOTE: %s need a board answering on COM4. Ports present: %s. '
            'Run with --offline to judge a host change without one.'
            % (', '.join(NEEDS_BOARD), ', '.join(ports)))


def _options(argv):
    """Everything the command line can say. Returns args."""
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--conformance', action='store_true',
                        help='also run test_conformance.py - needs a real '
                             'board on COM4, not just simulated')
    parser.add_argument('--model', default='auto',
                        help="the tag test_live_model.py runs against, or "
                             "'auto' for the tag THIS machine runs - the "
                             "same capability pick board_chat makes. This "
                             "script loads it once and releases it when "
                             "the run ends.")
    parser.add_argument('--smart', action='store_true',
                        help='run what the changes can have broken, and the '
                             'whole lot every %dth commit. --dry-run says '
                             'what it would do without running it.'
                             % FULL_EVERY)
    parser.add_argument('--dry-run', action='store_true',
                        help='with --smart: print the choice and why')
    parser.add_argument('--sections',
                        help='which live sections: tools|sequence|language')
    parser.add_argument('--live', action='store_true',
                        help='also run test_live_model.py - a real ollama '
                             'model against the real board, minutes not '
                             'seconds')
    parser.add_argument('--file', action='append', default=[],
                        help='run only this test file (repeatable), instead '
                             'of the default set')
    parser.add_argument('--coverage', type=int, choices=TIERS,
                        help='run about this percentage of every check there '
                             'is, cheapest-per-check first. Implies --smart.')
    parser.add_argument('--tags', help='subjects inside the ollama suites, '
                                       'instead of asking the model')
    parser.add_argument('--only', help='named tests in the ollama suites, '
                                       'comma-separated: intent,picker')
    parser.add_argument('--structure', action='store_true',
                        help='only the structure suite: imports, cycles, '
                             'duplicate definitions, dead imports, shape. '
                             'Run it after editing anything under host/.')
    parser.add_argument('--match',
                        help='run only the live rows whose question contains '
                             'this text, and nothing else. One changed rule '
                             'is one row, not fifteen minutes.')
    parser.add_argument('--offline', action='store_true',
                        help='skip the suites whose meaning depends on a real '
                             'board (%s). The default set runs either way - '
                             'it falls back to the simulated board and says '
                             'so.' % ', '.join(NEEDS_BOARD))
    parser.add_argument('--jobs', type=int, default=JOBS,
                        help='how many suites run side by side, %d here; 1 '
                             'is one after another. The suites that may '
                             'reach a board or hold the model (%s) run '
                             'alone whatever this says.'
                             % (JOBS, ', '.join(ALONE)))
    return parser.parse_args(argv)


def _run(args, tags, live_sections):
    """Run what the plan chose, and report it."""
    suites = list(args.file) if args.file else list(DEFAULT_SUITES)
    if args.conformance and not args.file:
        suites.append(CONFORMANCE)
    if (args.live and (not args.file or args.smart)
            and LIVE not in suites and not args.match):
        suites.append(LIVE)
    if args.offline:
        suites = [name for name in suites if name not in NEEDS_BOARD]
    if STRUCTURE not in suites and not args.match and not args.only:
        suites.insert(0, STRUCTURE)

    # The model's whole life, in one place.
    holding = LIVE in suites
    if holding:
        held = hold_model(args.model)

    total_pass = total_fail = total_skip = ran = 0
    approx = False
    suite_sizes = {}
    seconds = {}
    failing_lines = []
    ok = True
    try:
        for name, result in _results(suites, args, tags, live_sections):
            if result is None:
                print('%-20s MISSING %s' % (name, ROOT / 'tests' / name))
                ok = False
                continue
            tally, code, failing, elapsed, crash, groups = result
            seconds[name] = round(elapsed, 1)
            if tally is None:
                print('\n'.join(filter(None, [
                    '%-20s CRASHED exit=%s %.1fs' % (name, code, elapsed),
                    crash])))
                ok = False
                continue
            passed, failed, skipped, rough = tally
            total_skip += skipped
            approx = approx or rough
            if groups:
                print('%-20s %s' % ('', groups))
            ran += 1
            total_pass += passed
            total_fail += failed
            suite_sizes[name] = (passed, failed, skipped)
            failing_lines.extend('%s: %s' % (name, x) for x in failing)
            if failed or code != 0:
                ok = False
            print('%-20s %s, %d failed  %.1fs'
                  % (name, '%d passed' % passed, failed, elapsed))

            # Suites that did not run at all, in checks, from what they came to
            # last time.
        sizes = {n: p + f + s for n, (p, f, s) in suite_sizes.items()}
        counts.record('suites', sizes)
        counts.record('seconds', seconds)
        missed, never = counts.missing(
            'suites', [n for n in ALL_SUITES if n not in suite_sizes])
        total_skip += missed
        mark = '~' if approx or never else ''
        print('Total: %s%d  Passed: %d, Skipped: %s%d, Failed: %d, '
              '(%d of %d suites ran)'
              % (mark, total_pass + total_fail + total_skip, total_pass,
                 mark, total_skip, total_fail, ran, len(ALL_SUITES)))
        for line in failing_lines:
            print('  ' + line)
        if total_fail and any(line.split(':')[0] in NEEDS_BOARD
                              for line in failing_lines):
            print(board_note())
        return 0 if ok else 1
    finally:
        if holding:
            release_model(held)


#: What a run that was stopped on purpose exits with. The shell's own
#: convention for it, and distinct from 1 so a caller can tell a suite that
#: failed from a run somebody cut short - which matters when the reason for
#: cutting it short is that it should never have been this long.
STOPPED = 130


def main(argv=None):
    # THE RUNNER MUST SURVIVE WHAT IT REPORTS.
    for stream in (sys.stdout, sys.stderr):
        reconfigure = getattr(stream, 'reconfigure', None)
        if reconfigure is not None:
            reconfigure(errors='replace')
    args = _options(argv)
    try:
        chosen = _plan(args)
        if chosen is None:
            return 0
        return _run(args, *chosen)
    except KeyboardInterrupt:
        # Ctrl+C reaches the child suite too - it is in this process group - so
        # what is left to do here is say so and let the finally release the
        # model.
        print('\nstopped - the suites after this point did not run')
        return STOPPED
    finally:
        # One place, every path.
        release_model()


if __name__ == '__main__':
    sys.exit(main())
