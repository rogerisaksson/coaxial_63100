#!/usr/bin/env python3
"""Run this project's own test suites and print one deterministic
tally - the same numbers each suite already counts itself, never a summary
an LLM was asked to write.

That distinction is the point. A model relaying its own paraphrase of "did
the tests pass" is the failure mode documented across this codebase's own
FINDINGS.md and MODELS.md: a plausible sentence standing in for a fact
nobody actually checked. This script never asks anyone to summarise
anything - it parses the exact "  PASS "/"  FAIL " lines a human reads
running these files directly, and repeats only what it counted.

    python tools/run_tests.py                 # test_ollama, test_mcp, test_simulated
    python tools/run_tests.py --conformance    # + test_conformance.py (needs a real board)
    python tools/run_tests.py --live           # + test_live_model.py (board AND ollama)
    python tools/run_tests.py --file test_mcp.py

Exit code is 0 only if every requested suite ran and nothing in it failed.
"""
import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]           # host/
DEFAULT_SUITES = ('test_ollama.py', 'test_mcp.py', 'test_simulated.py')
CONFORMANCE = 'test_conformance.py'
LIVE = 'test_live_model.py'

# A cable is not a regression. That used to need saying loudly here - an
# unplugged board turned into '22 failed' in a verify loop that was meant to
# be checking a code change - and now it is enforced instead: every suite
# picks its session through coaxial_mcp.session.open_session(), which probes
# the port and falls back to the simulated board, and every one of them
# prints which it got.
#
# CONFORMANCE is the exception and stays listed, because it is the one suite
# a stand-in cannot stand in for: it is an independent byte-level master, and
# a simulated slave would be the shared wrong assumption it exists to rule
# out. With no board it runs its CRC self-test and says what it skipped.
NEEDS_BOARD = (CONFORMANCE,)

TALLY_RE = re.compile(r'^(\d+) passed, (\d+) failed$')
FAIL_RE = re.compile(r'^\s*FAIL\s+(.+?)\s{2,}')


def run_one(path, timeout=300):
    """(tally, returncode, failing_names, elapsed, crash_detail-or-None)."""
    started = time.monotonic()
    try:
        done = subprocess.run([sys.executable, str(path)], cwd=str(ROOT),
                              capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return None, None, [], time.monotonic() - started, 'TIMEOUT after %ss' % timeout

    elapsed = time.monotonic() - started
    lines = (done.stdout or '').splitlines()
    tally = None
    for line in reversed(lines):
        m = TALLY_RE.match(line.strip())
        if m:
            tally = (int(m.group(1)), int(m.group(2)))
            break
    failing = [m.group(1).strip() for m in (FAIL_RE.match(l) for l in lines) if m]

    if tally is None:
        # The suite crashed before printing its own tally - a traceback, an
        # import error. The last of stderr (or stdout, if it wrote nothing
        # to stderr) is what says why; clipped so one runaway crash cannot
        # push this past what a model's context can hold.
        detail = (done.stderr or done.stdout or '').strip()
        return None, done.returncode, failing, elapsed, detail[-1500:]
    return tally, done.returncode, failing, elapsed, None


def board_note():
    """One line saying whether the board answered, printed only when a suite
    that needs it failed.

    Asked, not assumed: 'the board is probably unplugged' is a guess, and this
    script's whole reason for existing is that a guess in place of a fact is
    what goes wrong here. find_board does the same probe link_diagnose does.
    """
    try:
        sys.path.insert(0, str(ROOT / 'tools'))
        import find_board
        ports = find_board.list_ports()
    except Exception as exc:                                  # noqa: BLE001
        return '  (could not check whether the board is attached: %s)' % exc
    if not ports:
        return ('  NOTE: Windows sees no COM ports at all - every failure '
                'above in %s is the cable, not the code.'
                % ', '.join(NEEDS_BOARD))
    return ('  NOTE: %s need a board answering on COM4. Ports present: %s. '
            'Run with --offline to judge a host change without one.'
            % (', '.join(NEEDS_BOARD), ', '.join(ports)))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--conformance', action='store_true',
                        help='also run test_conformance.py - needs a real '
                             'board on COM4, not just simulated')
    parser.add_argument('--live', action='store_true',
                        help='also run test_live_model.py - a real ollama '
                             'model against the real board, minutes not '
                             'seconds')
    parser.add_argument('--file', action='append', default=[],
                        help='run only this test file (repeatable), instead '
                             'of the default set')
    parser.add_argument('--offline', action='store_true',
                        help='skip the suites whose meaning depends on a real '
                             'board (%s). The default set runs either way - '
                             'it falls back to the simulated board and says '
                             'so.' % ', '.join(NEEDS_BOARD))
    args = parser.parse_args(argv)

    suites = list(args.file) if args.file else list(DEFAULT_SUITES)
    if args.conformance and not args.file:
        suites.append(CONFORMANCE)
    if args.live and not args.file:
        suites.append(LIVE)
    if args.offline:
        suites = [name for name in suites if name not in NEEDS_BOARD]

    total_pass = total_fail = 0
    failing_lines = []
    ok = True
    for name in suites:
        path = ROOT / 'tests' / name
        if not path.exists():
            print('%-20s MISSING %s' % (name, path))
            ok = False
            continue
        tally, code, failing, elapsed, crash = run_one(
            path, timeout=1200 if name == LIVE else 300)
        if tally is None:
            print('%-20s CRASHED exit=%s %.1fs' % (name, code, elapsed))
            if crash:
                print(crash)
            ok = False
            continue
        passed, failed = tally
        total_pass += passed
        total_fail += failed
        failing_lines.extend('%s: %s' % (name, x) for x in failing)
        if failed or code != 0:
            ok = False
        print('%-20s %s, %d failed  %.1fs'
              % (name, '%d passed' % passed, failed, elapsed))

    print('TOTAL %d passed, %d failed' % (total_pass, total_fail))
    for line in failing_lines:
        print('  ' + line)
    if total_fail and any(line.split(':')[0] in NEEDS_BOARD
                          for line in failing_lines):
        print(board_note())
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
