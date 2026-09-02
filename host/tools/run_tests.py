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
# Structure first: it answers "does host/ still hold together" in a fifth of
# a second, and every behavioural suite below it assumes the answer is yes.
STRUCTURE = 'test_structure.py'
CORE = 'test_modbus_core.py'
SHTP = 'test_shtp_core.py'
DRIVE = 'test_drive_core.py'
FILTER = 'test_filter_core.py'
SENSORLESS = 'test_sensorless.py'
#: test_ollama.py was 5,496 lines and 733 checks - a third of every check
#: this tree has, in one file, and the reason a tier could not be asked for at
#: any useful resolution. One file per subject now: the largest is 218 checks
#: and the smallest 12, so a budget can actually choose.
OLLAMA = tuple('test_ollama_%s.py' % tag for tag in
               ('tools', 'runner', 'prompt', 'link', 'render', 'bus',
                'board', 'reply', 'language'))
BENCH = 'test_bench.py'
BROKER = 'test_broker.py'
#: The acquisition front door against the stand-in - naming, reading,
#: the record's shape, the buffers. No board and no compiler, so it is
#: one of the cheapest suites here and joins first.
DAQ_API = 'test_daq_api.py'
VIEWS = 'test_views.py'
RENDER = 'test_render.py'
DEFAULT_SUITES = ((STRUCTURE, CORE, SHTP, DRIVE, FILTER, SENSORLESS,
                   BROKER, DAQ_API, VIEWS,
                   RENDER) + OLLAMA
                  + ('test_mcp.py', 'test_simulated.py', 'test_parity.py',
                     BENCH))
CONFORMANCE = 'test_conformance.py'
LIVE = 'test_live_model.py'
ALL_SUITES = DEFAULT_SUITES + (CONFORMANCE, LIVE)

#: Where each suite joins a tier. Cheapest per check first, so a tier buys
#: the most checks for the least wall time. Measured, seconds per check:
#: simulated 0.003, ollama 0.019, core 0.03, parity 0.13, mcp 0.14,
#: conformance 0.29, bench 5.0, live 4.6.
#:
#: The ollama suites are not here: they are in from the first tier and narrow
#: THEMSELVES through their own subject budget, which is where the fine
#: resolution lives.
JOINS = (
    (10, 'test_simulated.py'),
    (12, DAQ_API),
    (15, CORE),
    (20, SHTP),
    # The control law against a motor model, and the commissioning
    # against the stand-in: a compiler and a few seconds, no cable.
    (20, DRIVE),
    # The anti-alias chain against the transfer function it was designed
    # from, and a tone fed through it: a compiler and a second.
    (20, FILTER),
    (20, SENSORLESS),
    (35, 'test_parity.py'),
    (45, 'test_mcp.py'),
    (65, CONFORMANCE),

    # The bench guards the board's loop rates against a recorded baseline.
    # It joins late because its cost is FIXED - four checks and twenty
    # seconds, five seconds a check, dearer than anything but the live model
    # - and it says nothing at all without a board, so a cheap tier would pay
    # for it and get a skip.
    (70, BENCH),
)

#: Where the live suite joins, and where it stops being one section. It is
#: 4.6 seconds per check against conformance's 0.29 and simulated's 0.003,
#: so it is the last thing any budget buys.
LIVE_FROM = 75
LIVE_ALL_FROM = 95

#: The resolution a tier can be named at.
STEP = 5
TIERS = tuple(range(STEP, 101, STEP))


def plan_for(percent):
    """(suites, live sections) a percentage buys.

    STRUCTURE is not in the budget and is never dropped: three seconds, and
    it is the precondition for reading any other result - the behavioural
    suites import what they need and pass while the rest of the package is
    broken.
    """
    suites = [STRUCTURE] + list(OLLAMA)
    suites += [name for at, name in JOINS if percent >= at]

    if percent >= LIVE_ALL_FROM:
        return tuple(suites), 'all'
    if percent >= LIVE_FROM:
        return tuple(suites), 'tools'
    return tuple(suites), None

# A cable is not a regression: every suite opens through open_session(),
# which probes and falls back to the stand-in, and says which it got.
#
# CONFORMANCE is listed because a stand-in cannot stand in for it - it is an
# independent byte-level master, and a simulated slave would be the shared
# wrong assumption it exists to rule out. test_parity is not: with no board
# both sides are the stand-in and it skips itself rather than passing.
NEEDS_BOARD = (CONFORMANCE,)

TALLY_RE = re.compile(r'^(\d+) passed, (\d+) failed(?:, ~?(\d+) skipped)?$')
FAIL_RE = re.compile(r'^\s*FAIL\s+(.+?)\s{2,}')
# The ollama suites under --tags say what they left out. Surfaced here: the
# count that matters is the one against the whole file.
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


# Every tag this run put on the card, so one `finally` can hand them back.
# Measured: the picker loads the model on every --smart run and released
# nothing, so a three-second scoped run left 8.4 GB resident for half an hour.
# Holding it across the suites is the bargain; holding it after the run is not.
_LOADED = []


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
        _LOADED.append(client)
        print('holding %s for the run' % client.model)
        client.preload()
        return client
    except Exception as exc:                                  # noqa: BLE001
        print('could not preload %s: %s' % (tag, exc))
        return None


def _client_for(tag):
    """A handle on a tag, for unloading it. None if ollama is not there."""
    try:
        sys.path.insert(0, str(ROOT))
        from coaxial_ollama.client import Ollama
        return Ollama(tag)
    except Exception:                                         # noqa: BLE001
        return None


def release_model(client=None):
    """Hand the card back, once, when the run is over.

    Called from one `finally` for the whole run, over every client this run
    loaded - a suite's, the picker's, or both. Releasing per suite put most
    of the wall time back into loading 7.6 GB again; releasing nothing left
    it resident long after the run was over. Once, at the end, is the bargain.
    """
    held = [client] if client is not None else list(_LOADED)
    del _LOADED[:]
    done = set()
    for one in held:
        tag = getattr(one, 'model', None)
        if one is None or tag in done:
            continue
        done.add(tag)
        try:
            one.unload()
            print('released %s' % tag)
        except Exception as exc:                              # noqa: BLE001
            print('could not release %s: %s' % (tag, exc))


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
    ('host/coaxial_ollama/debug.py',  OLLAMA + ('live:all',)),
    ('host/coaxial_ollama/replies.py', OLLAMA + ('live:tools',)),
    ('host/coaxial_ollama/language.py', OLLAMA + ('live:language',)),
    ('host/coaxial_ollama/',          OLLAMA),
    ('host/coaxial_mcp/tools.py',     ('test_mcp.py', 'test_parity.py',
                                       'live:tools') + OLLAMA),
    ('host/coaxial_mcp/render.py',    ('test_mcp.py', 'test_parity.py')
                                      + OLLAMA),
    ('host/coaxial_mcp/',             ('test_mcp.py', 'test_parity.py')),
    ('host/coaxial/simulated.py',     ('test_simulated.py',
                                       'test_parity.py') + OLLAMA),
    # The broker is the port itself: every session goes through it when one
    # is up, so its own suite runs whenever it or the two files that reach
    # for it change.
    ('host/coaxial/broker.py',        (BROKER, DAQ_API,
                                      'test_parity.py')),
    ('host/coaxial/rig.py',           (DAQ_API, 'test_simulated.py',
                                      VIEWS)),
    ('host/coaxial/record.py',        (DAQ_API,)),
    ('host/coaxial/fanout.py',        (DAQ_API, BROKER)),
    ('host/coaxial/reader.py',        (DAQ_API,)),
    ('host/coaxial/calibration.py',   (DAQ_API, 'test_simulated.py')),
    ('host/coaxial/board.py',         (BROKER, 'test_simulated.py',
                                       'test_parity.py', 'test_mcp.py')),
    ('host/coaxial_mcp/session.py',   (BROKER, 'test_mcp.py',
                                       'test_parity.py')),
    ('host/tools/session.py',         (BROKER,)),
    # The pure character renderers: a reading in, text out. Nothing reaches a
    # wire, a tool schema or a board when one changes, so the suite that
    # draws them is the whole of it. `orientation` is the exception because
    # coaxial_mcp/tools.py imports it for the tool of the same name.
    ('host/coaxial/orientation.py',   ('test_simulated.py', 'test_mcp.py',
                                       RENDER)),
    ('host/coaxial/engine.py',        (RENDER, VIEWS)),
    ('host/coaxial/wireframe.py',     (RENDER, VIEWS)),
    ('host/coaxial/ascii3d.py',       ('test_simulated.py',)),
    ('host/coaxial/desk.py',          ('test_simulated.py',)),
    ('host/coaxial/dial.py',          ('test_simulated.py',)),
    ('host/coaxial/mesh.py',          ('test_simulated.py',)),
    ('host/coaxial/ansi.py',          ('test_simulated.py',)),
    ('host/coaxial/',                 ('test_simulated.py', 'test_parity.py',
                                       'test_mcp.py')),
    # A live view is a loop, a screen and a cable around a renderer that is
    # tested on its own. What it can break is importing at all, which is the
    # structure suite, plus the drawing it calls.
    # The views run whole - argument parsing, preflight, two frames and
    # the teardown - because four separate restyle breaks were found only
    # by running them by hand.
    ('host/tools/show_',              (STRUCTURE, VIEWS,
                                       'test_simulated.py')),
    ('host/tools/demos.py',           (VIEWS,) + OLLAMA),
    ('host/tools/screen.py',          (STRUCTURE, VIEWS,
                                       'test_simulated.py')),
    # A CACHE THE TOOLS WRITE, not code they read for behaviour. It is
    # tracked, so it turned up in every diff and pulled all nine ollama
    # suites in behind it.
    ('host/tools/.session.json',      ()),
    ('host/tools/',                   OLLAMA),
    ('host/tests/',                   ()),          # decided by name below
    # Firmware and protocol: the byte-level master is the point of it - but
    # the portable core is also compiled and run on this machine, which is
    # the only check on it that does not need a cable.
    ('Modbus/',                       (CORE, CONFORMANCE, 'test_mcp.py')),
    # The SHTP layer is hardware-free like the Modbus core, so the host build
    # is what covers it. Nothing on the Modbus wire changes when it does.
    ('Shtp/',                         (SHTP,)),
    # The decimating filter is hardware-free the same way, and its
    # design lives on the host beside it.
    ('Filter/',                       (FILTER,)),
    ('host/coaxial/bessel.py',        (FILTER, STRUCTURE)),
    # The control law is hardware-free like the SHTP layer, and its suite
    # closes the loop through a motor model - the only check on it that
    # needs no motor. The board glue in Board/ and Comms/ is the bench's.
    ('Drive/',                        (DRIVE,)),
    ('host/coaxial/drive.py',         (SENSORLESS, 'test_simulated.py',
                                       'test_parity.py')),
    ('host/coaxial/sensorless.py',    (SENSORLESS,)),
    ('host/coaxial/commission.py',    (SENSORLESS,)),
    ('host/tools/commission.py',      (STRUCTURE, SENSORLESS)),
    # The stage constants and the host control loops are design arithmetic
    # with closed-form checks; the Monte Carlo drives the compiled law.
    ('host/coaxial/inverter.py',      (SENSORLESS,)),
    ('host/coaxial/loop.py',          (SENSORLESS, DRIVE)),
    ('host/tools/montecarlo.py',      (STRUCTURE, DRIVE)),
    # BENCH is here and not with the host suites: what slows the board down
    # is firmware in the main loop, and the regression it guards against was
    # exactly that - the thermal observer reading two ADC channels and two SPI
    # transactions on every poll, and before that a poll blocking long enough
    # to lose a Modbus character. A host-side edit cannot cause either.
    ('Comms/',                        (CONFORMANCE, 'test_mcp.py', BENCH)),
    ('Board/',                        (CONFORMANCE, 'test_mcp.py',
                                       'test_parity.py', BENCH)),
    ('Core/',                         (CONFORMANCE, BENCH)),
    ('Thermal/',                      (CONFORMANCE, BENCH)),
    # A NOTEBOOK EXAMPLE reaches the library and nothing else reaches it.
    # What it can break is naming a method that does not exist, which is
    # the structure suite's AST pass - measured: it caught a rename that
    # left `print(daq)` behind in two of them.
    ('python_examples/',              (STRUCTURE,)),
    ('notebook_examples/',            (STRUCTURE,)),
    # A document can only break the docs index and the phrase table.
    ('docs/',                         ('test_ollama_runner.py',)),
    ('CLAUDE.md',                     ('test_ollama_runner.py',)),
    ('README.md',                     ('test_ollama_runner.py',)),
    # PowerShell around the Python. None of it is imported by anything under
    # test, so the most it can break is a path - which is what the structure
    # suite's three seconds are for. Listed rather than left unmapped
    # because unmapped means the whole gate, and editing a demo wrapper used
    # to cost seven minutes and a model load.
    ('demos/',                        (STRUCTURE,)),
    ('coaxial_tty.ps1',                      (STRUCTURE,)),
    ('env.ps1',                       (STRUCTURE,)),
    ('host/run_tests.ps1',            (STRUCTURE,)),
    ('setup.ps1',                     (STRUCTURE,)),
    # Neither the CAD export nor the schematic is read by a suite. The parts
    # list and the pin map come off the board, not out of these.
    ('render/',                       ()),
    ('electronics/',                  ()),
    ('datasheets/',                   ()),
    ('.gitignore',                    ()),
    ('.vscode/',                      ()),
)

# Every this many commits, run the lot regardless of what changed. A map
# from files to suites is a guess about coupling, and a guess that is never
# checked is one that drifts.
FULL_EVERY = 10

#: Suites the map may settle alone: no board, no ollama - about 40 s all
#: six together, so asking the model costs a 7.6 GB load longer than the
#: run. Where the map has an explicit rule it is also the better answer,
#: written by someone reading the imports.
CHEAP = frozenset({STRUCTURE, CORE, SHTP, DRIVE, SENSORLESS,
                   'test_simulated.py', VIEWS, RENDER})


def _within_tier(args, live_sections):
    """Hold the model's pick inside the tier's budget.

    A tier is a budget of checks and the model spends inside it. It used to
    be able to spend past it: the tier filtered `args.file` and then the
    model branch assigned straight over the top, live sections included.

    Measured on the 25 % tier with a demo edit in the diff - the tier had
    already dropped the live suite, the model put `live:all` back, and the
    cheapest run there is took 398 s of which 352 were the suite the tier
    exists to leave out.

    The ollama suites are exempt for the same reason the tier exempts them:
    narrowed by tags rather than dropped whole.
    """
    if not args.coverage:
        return live_sections

    allowed, sections = plan_for(args.coverage)
    keep = set(allowed) | {STRUCTURE} | set(OLLAMA)
    dropped = [name for name in args.file if name not in keep]

    args.file = [name for name in args.file if name in keep]
    if not sections and live_sections:
        dropped.append('live:' + live_sections)
        live_sections = ''

    args.live = bool(live_sections)

    if dropped:
        print('   the %d%% tier does not stretch to: %s'
              % (args.coverage, ', '.join(dropped)))
    return live_sections


def _ask_model(args, live_sections):
    """The model's own pick, held inside whatever tier is in force.

    Returns (tags, live_sections). Everything the model can get wrong lands
    on the same answer - run what the path map already chose - because
    running too much costs seconds and running too little hides a regression
    until the next sweep.
    """
    sys.path.insert(0, str(ROOT / 'tools'))
    import pick_tests

    # The picker loads the model too. Registered here so the release at the
    # end of the run covers it, whether or not a suite needs it.
    plan, reason = pick_tests.pick(args.model)
    _LOADED.append(_client_for(args.model))

    if plan is None:
        print('   the model picked nothing: %s' % reason)
        print('   falling back to the path map above')
        return None, live_sections

    # Structure is not the model's to drop. It is three seconds and it is
    # the precondition for every suite below it: they import what they need
    # and pass while the rest of the package is broken.
    args.file = [STRUCTURE] + [f for f in plan.suites
                               if f not in (LIVE, STRUCTURE)]
    tags = ','.join(plan.tags) or None
    live_sections = '' if plan.live == 'none' else plan.live
    args.live = bool(live_sections)

    print('   the model picked: %s%s'
          % (', '.join(args.file) or 'nothing',
             '  live:' + live_sections if live_sections else ''))
    print('   %s' % (plan.why or 'no reason given'))

    return tags, _within_tier(args, live_sections)


def settled(chosen, why):
    """True when the map knew every path and the answer costs seconds.

    Both halves matter. An unmapped path means the map has a hole and the
    fallback is already running everything, which is not something to
    shortcut. A cheap answer means asking cannot pay for itself.
    """
    if any('unmapped' in line for line in why):
        return False

    # An empty pick counts. A change to something no suite reads - the CAD
    # export, a datasheet - is the case where asking the model is most
    # obviously waste: it costs a 7.6 GB load to be told what the map has
    # already said, which is that there is nothing to run.
    return set(chosen) <= CHEAP


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
            paths |= {line.strip().replace('\\', '/')
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
                # An empty entry is a deliberate "nothing under test reads
                # this", not a hole in the map. Saying so is the difference
                # between a rule and an oversight for whoever reads the plan.
                why.append('%s -> %s'
                           % (path, ', '.join(wanted) or 'nothing reads it'))
                break
        else:
            why.append('%s -> unmapped, running everything' % path)
            return set(DEFAULT_SUITES) | {CONFORMANCE}, {'all'}, why
    return suites, live, why


def _options(argv):
    """Everything the command line can say. Returns args."""
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
    return parser.parse_args(argv)


def _plan(args):
    """Which suites, which subjects, which live sections.

    Returns (tags, live_sections) and edits args.file in place -
    the flags below narrow each other, and threading five return
    values through would say less than the names they already have.
    Returns None instead when --dry-run means print and stop.
    """

    live_sections = 'all'
    tags = args.tags
    if args.structure:
        args.file, args.smart, args.live = [STRUCTURE], False, False
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
        # Every subject file is offered the names; the one that owns them
        # runs them and the rest report nothing. Cheaper than asking which
        # file a test lives in, and it cannot go stale.
        args.file, args.smart, args.live = list(OLLAMA), False, False
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
            allowed, sections = plan_for(args.coverage)
            args.file = [f for f in args.file
                         if f in OLLAMA or f in allowed]
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
        if not tags and not full and settled(chosen, why):
            print('   the map knew every path and the answer is seconds - '
                  'not asking the model')
        elif not tags and not full:
            tags, live_sections = _ask_model(args, live_sections)
        if args.dry_run:
            return None            # the plan was the whole point of the run

    # Typed explicitly, so it wins over the 'all' default and over a tier's
    # own pick. It used to be read only inside the --match branch, which
    # meant `--live --sections tools` silently ran all three sections -
    # measured, tools and all coming back with the same 176 checks in the
    # same 255s.
    if args.sections and args.live:
        live_sections = args.sections

    return tags, live_sections


def _extra_for(name, args, tags, live_sections):
    """The flags one suite takes from the plan: the model, its sections
    and match for the live suite; the picked tests or the tags and coverage
    for the ollama ones. No --release: this script owns the model's life,
    holds it across every suite that needs it, and hands it back in _run's
    `finally`. The suite releasing it per run was what put most of the wall
    time into loading 7.6 GB again."""
    extra = ['-m', args.model] if name == LIVE else []
    if name == LIVE and live_sections:
        extra += ['--sections', live_sections]
    if name == LIVE and args.match:
        extra += ['--match', args.match]
    if name not in OLLAMA:
        return extra
    if args.only:
        return extra + ['--only', args.only]
    if tags:
        extra += ['--tags', tags]
        if args.coverage:
            extra += ['--coverage', str(args.coverage)]
    return extra


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
            extra = _extra_for(name, args, tags, live_sections)
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


#: What a run that was stopped on purpose exits with. The shell's own
#: convention for it, and distinct from 1 so a caller can tell a suite that
#: failed from a run somebody cut short - which matters when the reason for
#: cutting it short is that it should never have been this long.
STOPPED = 130


def main(argv=None):
    args = _options(argv)
    try:
        chosen = _plan(args)
        if chosen is None:
            return 0
        return _run(args, *chosen)
    except KeyboardInterrupt:
        # Ctrl+C reaches the child suite too - it is in this process group -
        # so what is left to do here is say so and let the finally release
        # the model. A run abandoned with 7.6 GB still resident is the
        # expensive kind of mistake, and it used to be the default one.
        print('\nstopped - the suites after this point did not run')
        return STOPPED
    finally:
        # One place, every path. The picker loads the model before a single
        # suite runs, and _run's own finally never saw it.
        release_model()


if __name__ == '__main__':
    sys.exit(main())
