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
import os
import re
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]           # host/
sys.path.insert(0, str(ROOT))
from tests import counts                             # noqa: E402
BACKSLASH = chr(92)
DEFAULT_SUITES = ('test_ollama.py', 'test_mcp.py', 'test_simulated.py',
                  'test_parity.py')
CONFORMANCE = 'test_conformance.py'
LIVE = 'test_live_model.py'
ALL_SUITES = DEFAULT_SUITES + (CONFORMANCE, LIVE)

# Coverage tiers, as a plan rather than a ladder of ifs. The percentage is of
# every check this repository has (counts.py knows what each suite and each
# group last came to). Suites join a tier in order of seconds per check, so a
# tier buys the most checks for the least wall time - measured, per check:
# simulated 0.003 s, ollama 0.019, parity 0.13, mcp 0.14, conformance 0.29,
# live 4.6. That last figure is why live joins only at the top tier.
PLAN = {
    25:  (('test_simulated.py',), None),
    50:  (('test_simulated.py', 'test_parity.py', 'test_mcp.py'), None),
    75:  (('test_simulated.py', 'test_parity.py', 'test_mcp.py',
           CONFORMANCE), 'tools'),
}

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
# test_parity.py needs one too, but for the opposite reason: with no board
# both sides of the comparison are the stand-in and it is trivially true, so
# it skips itself rather than passing. Not listed - a cable-less run of it is
# not a failure to explain.
NEEDS_BOARD = (CONFORMANCE,)

TALLY_RE = re.compile(r'^(\d+) passed, (\d+) failed(?:, ~?(\d+) skipped)?$')
FAIL_RE = re.compile(r'^\s*FAIL\s+(.+?)\s{2,}')
# test_ollama.py under --tags says what it left out. Surfaced here, because
# the count that matters to a reader is the one against the whole file.
GROUPS_RE = re.compile(r'^ran \d+ of \d+ groups: .*$')


def run_one(path, timeout=300, extra=()):
    """(tally, code, failing, elapsed, crash-or-None, groups-line-or-None)."""
    started = time.monotonic()
    try:
        # utf-8/replace, not the locale codepage: text=True alone decodes
        # cp1252 here, and a suite printing one character outside it killed
        # the reader thread with UnicodeDecodeError - the run lost, not the
        # character. PYTHONIOENCODING makes the child write what we read.
        env = dict(os.environ, PYTHONIOENCODING='utf-8')
        done = subprocess.run([sys.executable, str(path)] + list(extra),
                              cwd=str(ROOT), env=env, timeout=timeout,
                              capture_output=True, text=True,
                              encoding='utf-8', errors='replace')
    except subprocess.TimeoutExpired:
        return None, None, [], time.monotonic() - started, 'TIMEOUT after %ss' % timeout

    elapsed = time.monotonic() - started
    lines = (done.stdout or '').splitlines()
    tally = None
    for line in reversed(lines):
        m = TALLY_RE.match(line.strip())
        if m:
            tally = (int(m.group(1)), int(m.group(2)),
                     int(m.group(3) or 0), '~' in line)
            break
    failing = [m.group(1).strip() for m in (FAIL_RE.match(l) for l in lines) if m]
    groups = next((l.strip() for l in reversed(lines)
                   if GROUPS_RE.match(l.strip())), None)

    if tally is None:
        # The suite crashed before printing its own tally - a traceback, an
        # import error. The last of stderr (or stdout, if it wrote nothing
        # to stderr) is what says why; clipped so one runaway crash cannot
        # push this past what a model's context can hold.
        detail = (done.stderr or done.stdout or '').strip()
        return None, done.returncode, failing, elapsed, detail[-1500:], groups
    return tally, done.returncode, failing, elapsed, None, groups


def hold_model(tag):
    """Load the model before the first suite that needs it.

    Up front rather than by the first question, so the 7.6 GB wait lands
    where somebody is watching for it instead of inside a turn that then
    looks slow for no reason. Returns None if ollama is not reachable - the
    suite saying it cannot run is a better message than a preload raising
    here.
    """
    try:
        sys.path.insert(0, str(ROOT))
        from coaxial_ollama.client import Ollama
        client = Ollama(tag, keep_alive='30m')
        client.model = client.require_model()
        print('holding %s for the run' % client.model)
        client.preload()
        return client
    except Exception as exc:                                  # noqa: BLE001
        print('could not preload %s: %s' % (tag, exc))
        return None


def release_model(client):
    """Hand the card back, once, when the run is over."""
    if client is None:
        return
    try:
        client.unload()
        print('released %s' % client.model)
    except Exception as exc:                                  # noqa: BLE001
        print('could not release %s: %s' % (client.model, exc))


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


# What a change to each part of the tree can plausibly have broken. Read
# top-down, first match wins, and anything unmatched falls back to the whole
# default set - the safe direction when the map has a hole in it.
#
# `live` is the expensive one: a model load plus a turn per question. It is
# listed only against the files that decide what the model is told and what
# it can call, because that is what a wrong answer there comes from.
TOUCHES = (
    ('host/coaxial_ollama/debug.py',  ('test_ollama.py', 'live:all')),
    ('host/coaxial_ollama/replies.py', ('test_ollama.py', 'live:tools')),
    ('host/coaxial_ollama/language.py', ('test_ollama.py', 'live:language')),
    ('host/coaxial_ollama/',          ('test_ollama.py',)),
    ('host/coaxial_mcp/tools.py',     ('test_mcp.py', 'test_ollama.py',
                                       'test_parity.py', 'live:tools')),
    ('host/coaxial_mcp/render.py',    ('test_mcp.py', 'test_ollama.py',
                                       'test_parity.py')),
    ('host/coaxial_mcp/',             ('test_mcp.py', 'test_parity.py')),
    ('host/coaxial/simulated.py',     ('test_simulated.py', 'test_parity.py',
                                       'test_ollama.py')),
    ('host/coaxial/',                 ('test_simulated.py', 'test_parity.py',
                                       'test_mcp.py')),
    ('host/tools/',                   ('test_ollama.py',)),
    ('host/tests/',                   ()),          # decided by name below
    # Firmware and protocol: the byte-level master is the point of it.
    ('Modbus/',                       (CONFORMANCE, 'test_mcp.py')),
    ('Comms/',                        (CONFORMANCE, 'test_mcp.py')),
    ('Board/',                        (CONFORMANCE, 'test_mcp.py',
                                       'test_parity.py')),
    ('Core/',                         (CONFORMANCE,)),
    # A document can only break the docs index and the phrase table.
    ('docs/',                         ('test_ollama.py',)),
    ('CLAUDE.md',                     ('test_ollama.py',)),
    ('README.md',                     ('test_ollama.py',)),
)

# Every this many commits, run the lot regardless of what changed. A map
# from files to suites is a guess about coupling, and a guess that is never
# checked is one that drifts.
FULL_EVERY = 10


def changed_files(against='HEAD'):
    """Paths touched in the working tree and in the last commit.

    Both, because a suite picked for a change already committed is what a
    pre-push check wants, and one picked for a change not yet staged is
    what an edit-test loop wants.
    """
    import subprocess
    paths = set()
    for args in (['diff', '--name-only', against],
                 ['diff', '--name-only', '--cached'],
                 ['diff', '--name-only', '%s~1' % against, against]):
        try:
            done = subprocess.run(['git'] + args, cwd=str(ROOT.parent),
                                  capture_output=True, text=True,
                                  encoding='utf-8', errors='replace',
                                  timeout=30)
        except Exception:                                     # noqa: BLE001
            continue
        if done.returncode == 0:
            paths |= {line.strip().replace(BACKSLASH, '/')
                      for line in done.stdout.splitlines() if line.strip()}
    return sorted(paths)


def pick(paths):
    """(suites, live_sections, why) for these changed paths."""
    suites, live, why = set(), set(), []
    for path in paths:
        for prefix, wanted in TOUCHES:
            if path.startswith(prefix):
                if not wanted and path.startswith('host/tests/'):
                    name = path.rsplit('/', 1)[-1]
                    if name.startswith('test_'):
                        suites.add(name)
                        why.append('%s -> itself' % path)
                    break
                for item in wanted:
                    if item.startswith('live:'):
                        live.add(item.split(':', 1)[1])
                    else:
                        suites.add(item)
                why.append('%s -> %s' % (path, ', '.join(wanted) or 'itself'))
                break
        else:
            why.append('%s -> unmapped, running everything' % path)
            return set(DEFAULT_SUITES) | {CONFORMANCE}, {'all'}, why
    return suites, live, why


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--conformance', action='store_true',
                        help='also run test_conformance.py - needs a real '
                             'board on COM4, not just simulated')
    parser.add_argument('--model', default='gemma4:12b',
                        help='the tag test_live_model.py runs against. This '
                             'script loads it once and releases it when the '
                             'run ends.')
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
    parser.add_argument('--coverage', type=int, choices=sorted(PLAN),
                        help='run about this percentage of every check there '
                             'is, cheapest-per-check first. Implies --smart.')
    parser.add_argument('--tags', help='subjects inside test_ollama.py, '
                                       'instead of asking the model')
    parser.add_argument('--only', help='named tests in test_ollama.py, '
                                       'comma-separated: intent,picker')
    parser.add_argument('--match',
                        help='run only the live rows whose question contains '
                             'this text, and nothing else. One changed rule '
                             'is one row, not fifteen minutes.')
    parser.add_argument('--offline', action='store_true',
                        help='skip the suites whose meaning depends on a real '
                             'board (%s). The default set runs either way - '
                             'it falls back to the simulated board and says '
                             'so.' % ', '.join(NEEDS_BOARD))
    args = parser.parse_args(argv)

    live_sections = 'all'
    tags = args.tags
    if args.match:
        # One live row and nothing else. A rule that changed is one question,
        # and the whole suite is a model load plus a turn per row.
        args.file, args.smart, args.live = [LIVE], False, True
        live_sections = args.sections or 'all'
    if args.coverage:
        args.smart = True
    if args.only:
        # One file, the named tests, nothing else. The shortest path back
        # after changing one thing, and why this script is the only interface
        # anybody needs to the suites.
        args.file, args.smart, args.live = ['test_ollama.py'], False, False
    if args.smart and not args.file:
        import subprocess
        try:
            count = int(subprocess.run(
                ['git', 'rev-list', '--count', 'HEAD'], cwd=str(ROOT.parent),
                capture_output=True, text=True, encoding='utf-8',
                errors='replace', timeout=30).stdout.strip())
        except Exception:                                     # noqa: BLE001
            count = 0
        paths = changed_files()
        # --minimal skips the sweep on purpose: it is the fix-test cycle's
        # run, and the sweep is the gate's. Anything --minimal misses is
        # what the next unqualified --smart is for.
        # Not on a coverage tier: the sweep exists to catch what narrowing
        # missed, and a tier is narrowing by definition.
        full = bool(count) and count % FULL_EVERY == 0 and not args.coverage
        if full:
            chosen = set(DEFAULT_SUITES) | {CONFORMANCE}
            picked_live, why = {'all'}, ['commit %d is a multiple of %d - '
                                         'everything' % (count, FULL_EVERY)]
        else:
            chosen, picked_live, why = pick(paths)
            if not chosen and not picked_live:
                why.append('nothing changed that any suite covers')
        print('-- smart: %d file%s changed --'
              % (len(paths), '' if len(paths) == 1 else 's'))
        for line in why[:12]:
            print('   ' + line)
        order = list(DEFAULT_SUITES) + [CONFORMANCE]
        # The live suite is not run by name from --file: it is the one with
        # sections, and it is added below. Editing it is a reason to run it.
        if LIVE in chosen:
            picked_live = picked_live or {'all'}
        args.file = [name for name in order if name in chosen]
        live_sections = ','.join(sorted(picked_live)) if picked_live else ''
        if live_sections and 'all' in picked_live:
            live_sections = 'all'
        if live_sections:
            args.live = True
        print('   suites: %s%s' % (', '.join(args.file) or 'none',
                                   '  live: ' + live_sections
                                   if live_sections else ''))
        # The path map above decides which files. Which subjects inside the
        # big one is the judgement call, and it goes to the model - which
        # can only ever cost seconds by over-picking, because every way it
        # fails returns None and this runs the file whole.
        # Not on the full sweep. Narrowing the one run that exists to catch
        # what the narrowing missed is the whole guarantee, spent.
        if args.coverage:
            allowed, sections = PLAN[args.coverage]
            args.file = [f for f in args.file
                         if f == 'test_ollama.py' or f in allowed]
            live_sections = sections or ''
            args.live = bool(sections)
            if sections:
                args.file.append(LIVE)
            print('   %d%% tier: %s%s'
                  % (args.coverage, ', '.join(args.file),
                     ' live:' + sections if sections else ''))
        # The model decides the list, not the path map. The map above is
        # the fallback: coarse by construction - a line moved in
        # coaxial_mcp/tools.py pulls in four suites whatever the line
        # was - and it only stands when there is no model to ask.
        if not tags and not full:
            sys.path.insert(0, str(ROOT / 'tools'))
            import pick_tests
            plan, reason = pick_tests.pick(args.model)
            if plan is None:
                print('   the model picked nothing: %s' % reason)
                print('   falling back to the path map above')
            else:
                args.file = [f for f in plan.suites
                             if f != 'test_live_model.py']
                tags = ','.join(plan.tags) or None
                live_sections = '' if plan.live == 'none' else plan.live
                args.live = bool(live_sections)
                print('   the model picked: %s%s'
                      % (', '.join(args.file) or 'nothing',
                         '  live:' + live_sections if live_sections
                         else ''))
                print('   %s' % (plan.why or 'no reason given'))
        if args.dry_run:
            return 0

    suites = list(args.file) if args.file else list(DEFAULT_SUITES)
    if args.conformance and not args.file:
        suites.append(CONFORMANCE)
    if (args.live and (not args.file or args.smart)
            and LIVE not in suites and not args.match):
        suites.append(LIVE)
    if args.offline:
        suites = [name for name in suites if name not in NEEDS_BOARD]

    # The model's whole life, in one place. It is loaded once before the
    # first suite that needs it, held across every row of every such suite,
    # and handed back when the run is over - not after each suite, and not
    # after each question. Measured: unloading between runs put most of the
    # wall time into loading 7.6 GB again, and holding it after the run put
    # 9.69 GB on the card for 27 minutes at 1 % use. Neither is the bargain.
    holding = LIVE in suites
    if holding:
        held = hold_model(args.model)

    total_pass = total_fail = total_skip = ran = 0
    approx = False
    suite_sizes = {}
    failing_lines = []
    ok = True
    try:
        for name in suites:
            path = ROOT / 'tests' / name
            if not path.exists():
                print('%-20s MISSING %s' % (name, path))
                ok = False
                continue
            # No --release: this script owns the model's life, holds it
            # across every suite that needs it, and hands it back in the
            # `finally` below. The suite releasing it per run was what put
            # most of the wall time into loading 7.6 GB again.
            extra = ['-m', args.model] if name == LIVE else []
            if name == LIVE and live_sections:
                extra += ['--sections', live_sections]
            if name == LIVE and args.match:
                extra += ['--match', args.match]
            if name == 'test_ollama.py':
                if args.only:
                    extra += ['--only', args.only]
                elif tags:
                    extra += ['--tags', tags]
                    if args.coverage:
                        extra += ['--coverage', str(args.coverage)]
            tally, code, failing, elapsed, crash, groups = run_one(
                path, timeout=1200 if name == LIVE else 300, extra=extra)
            if tally is None:
                print('%-20s CRASHED exit=%s %.1fs' % (name, code, elapsed))
                if crash:
                    print(crash)
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

            # Suites that did not run at all, in checks, from what they came
        # to last time. A suite never yet measured makes the total
        # approximate rather than silently short - hence the tilde.
        sizes = {n: p + f + s for n, (p, f, s) in suite_sizes.items()}
        counts.record('suites', sizes)
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


if __name__ == '__main__':
    sys.exit(main())
