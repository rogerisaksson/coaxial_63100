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
import re
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from contextlib import suppress

ROOT = Path(__file__).resolve().parents[2]           # host/
sys.path.insert(0, str(ROOT))
from tests import counts                             # noqa: E402
from tools.target import find_board                                    # noqa: E402
from tools.dev import pick_tests                                    # noqa: E402
from coaxial_ollama import client as clientmod       # noqa: E402
from coaxial_ollama.capability import choose, probe  # noqa: E402
# Structure first: it answers "does host/ still hold together" in a fifth of a
# second, and every behavioural suite below it assumes the answer is yes.
STRUCTURE = 'test_structure.py'
CORE = 'test_modbus_core.py'
SHTP = 'test_shtp_core.py'
DRIVE = 'test_drive_core.py'
FILTER = 'test_filter_core.py'
#: The thermal envelope as the C that will run on the board. The
#: network had `check.c` - the calibration campaign's own report -
#: and the SOA arithmetic that gates a real stage had nothing at all,
#: only a tested Python mirror. That was the wrong way round.
THERMAL = 'test_thermal_core.py'
#: The acquisition engine as the C that will run on the board - the
#: ring, the window, the ladder, the tone, the live accumulator -
#: hardware-free since 2026-09-14 and, until then, tested only by the
#: bench after a flash.
DAQ_CORE = 'test_daq_core.py'
#: The bootloader's state machine as the C that will run - the chunk
#: stream, the bitmap, the seal - on a RAM flash, before any register
#: is touched (docs/BOOT.md).
BOOT_CORE = 'test_boot_core.py'
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
#: The master's side of the bootloader against the stand-in's blank
#: node - no board, no compiler, a second (docs/BOOT.md).
BOOT = 'test_boot.py'
VIEWS = 'test_views.py'
RENDER = 'test_render.py'
DEFAULT_SUITES = ((STRUCTURE, CORE, SHTP, DRIVE, FILTER, THERMAL, DAQ_CORE, BOOT_CORE,
                   SENSORLESS,
                   BROKER, DAQ_API, BOOT, VIEWS,
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
    (12, BOOT),
    (15, CORE),
    (20, SHTP),
    # The control law against a motor model, and the commissioning against the
    # stand-in: a compiler and a few seconds, no cable.
    (20, DRIVE),
    # The anti-alias chain against the transfer function it was designed from,
    # and a tone fed through it: a compiler and a second.
    (20, FILTER),
    # The SOA envelope, same shape and same cost: a compiler and a second.
    (20, THERMAL),
    # The bootloader's core: a compiler and a second, and the one thing that
    # decides whether a blank node ever runs anything.
    (20, BOOT_CORE),
    (20, SENSORLESS),
    (35, 'test_parity.py'),
    (45, 'test_mcp.py'),
    (65, CONFORMANCE),

    # The bench guards the board's loop rates against a recorded baseline.
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
    """(suites, live sections) a percentage buys."""
    suites = [STRUCTURE] + list(OLLAMA)
    suites += [name for at, name in JOINS if percent >= at]

    if percent >= LIVE_ALL_FROM:
        return tuple(suites), 'all'
    if percent >= LIVE_FROM:
        return tuple(suites), 'tools'
    return tuple(suites), None

# A cable is not a regression: every suite opens through open_session(), which
# probes and falls back to the stand-in, and says which it got.
NEEDS_BOARD = (CONFORMANCE,)

#: Suites that may reach the board's port, or hold the model on the card.
#: Each wants the machine to itself - the bench suite measures the link's
#: own rates, conformance its frame gaps - so they run one at a time after
#: the rest. Every other suite opens the stand-in or nothing at all.
ALONE = ('test_mcp.py', 'test_parity.py', BENCH, CONFORMANCE, LIVE)

#: Suites run side by side: their wall time is mostly the stand-ins' sleep
#: (2026-09-21: sensorless 24 s of CPU in 151 s, the DAQ front door 0.9 in
#: 72, the broker 2.2 in 31); one after another was a 400 s gate. Half the
#: cores, at most four: no page file here, and one views page is 14
#: processes.
JOBS = max(1, min(4, (os.cpu_count() or 2) // 2))

TALLY_RE = re.compile(r'^(\d+) passed, (\d+) failed(?:, ~?(\d+) skipped)?$')
# The whole line after FAIL, detail included: a check's detail is the compiler
# warning, the wrong value, the reason - and on a runner the summary (relayed
# as a commit comment) is the only place it surfaces.
FAIL_RE = re.compile(r'^\s{1,8}FAIL\s+(\S.*?)\s*$')
# The ollama suites under --tags say what they left out.
GROUPS_RE = re.compile(r'^ran \d+ of \d+ groups: .*$')


def kill_tree(pid):
    """The process and every descendant, gone."""
    if os.name == 'nt':
        subprocess.run(['taskkill', '/F', '/T', '/PID', str(pid)],
                       capture_output=True)
        return
    with suppress(OSError):
        os.killpg(os.getpgid(pid), signal.SIGKILL)


def run_captured(argv, timeout, cwd=None):
    """`argv` run with its output captured, or None once `timeout` has
    passed - the whole tree killed, not the child alone.
    """
    env = dict(os.environ, PYTHONIOENCODING='utf-8')
    group = {'start_new_session': True} if os.name != 'nt' else {}
    proc = subprocess.Popen(argv, cwd=cwd, env=env, stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE, text=True,
                            encoding='utf-8', errors='replace', **group)
    try:
        out, err = proc.communicate(timeout=timeout)
    except subprocess.TimeoutExpired:
        kill_tree(proc.pid)
        proc.communicate()
        return None
    return subprocess.CompletedProcess(argv, proc.returncode, out, err)


def run_one(path, timeout=300, extra=()):
    """(tally, code, failing, elapsed, crash-or-None, groups-line-or-None)."""
    started = time.monotonic()
    done = run_captured([sys.executable, str(path)] + list(extra), timeout,
                        cwd=str(ROOT))
    if done is None:
        return (None, None, [], time.monotonic() - started,
                'TIMEOUT after %ss' % timeout, None)

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
        # import error.
        detail = (done.stderr or done.stdout or '').strip()
        return None, done.returncode, failing, elapsed, detail[-1500:], groups
    return tally, done.returncode, failing, elapsed, None, groups


# Every tag this run put on the card, so one `finally` can hand them back.
_LOADED = []


def hold_model(tag):
    """Load the model before the first suite that needs it."""
    try:
        client = clientmod.Ollama(tag, keep_alive='30m')
        client.model = client.require_model()
        _LOADED.append(client)
        print('holding %s for the run' % client.model)
        client.preload()
        return client
    except clientmod.FAULTS as exc:
        print('could not preload %s: %s' % (tag, exc))
        return None


def _client_for(tag):
    """A handle on a tag, for unloading it. None if ollama is not there."""
    try:
        return clientmod.Ollama(tag)
    except clientmod.FAULTS:
        return None


def release_model(client=None):
    """Hand the card back, once, when the run is over."""
    held = [client] if client is not None else list(_LOADED)
    del _LOADED[:]
    done = set()
    for one in held:
        if one is None or one.model in done:
            continue
        tag = one.model
        done.add(tag)
        try:
            one.unload()
            print('released %s' % tag)
        except clientmod.FAULTS as exc:
            print('could not release %s: %s' % (tag, exc))


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


# What a change to each part of the tree can plausibly have broken.
TOUCHES = (
    ('host/coaxial_ollama/debug.py',           OLLAMA + ('live:all',)),
    ('host/coaxial_ollama/replies.py',         OLLAMA + ('live:tools',)),
    ('host/coaxial_ollama/language.py',        OLLAMA + ('live:language',)),
    ('host/coaxial_ollama/',                   OLLAMA),
    ('host/coaxial_mcp/tools.py',              ('test_mcp.py', 'test_parity.py',
                                                'live:tools') + OLLAMA),
    ('host/coaxial_mcp/render.py',             ('test_mcp.py', 'test_parity.py')
                                               + OLLAMA),
    ('host/coaxial_mcp/',                      ('test_mcp.py', 'test_parity.py')),
    ('host/coaxial/simulated',                 ('test_simulated.py',
                                                'test_parity.py') + OLLAMA),
    # The broker is the port itself: every session goes through it when one is
    # up, so its own suite runs whenever it or the two files that reach for it
    # change.
    ('host/coaxial/comm/broker.py',            (BROKER, DAQ_API,
                                                'test_parity.py')),
    ('host/coaxial/comm/ports.py',             (BROKER, 'test_mcp.py',
                                                'test_ollama_link.py')),
    ('host/coaxial/rig.py',                    (DAQ_API, 'test_simulated.py',
                                                VIEWS)),
    ('host/coaxial/acquire/record.py',         (DAQ_API,)),
    ('host/coaxial/acquire/fanout.py',         (DAQ_API, BROKER)),
    ('host/coaxial/acquire/reader.py',         (DAQ_API,)),
    ('host/coaxial/devices/calibration.py',    (DAQ_API, 'test_simulated.py')),
    ('host/coaxial/devices/board.py',          (BROKER, 'test_simulated.py',
                                                'test_parity.py', 'test_mcp.py')),
    ('host/coaxial/comm/session.py',           (BROKER, 'test_mcp.py',
                                                'test_parity.py')),
    ('host/tools/target/session.py',           (BROKER,)),
    # The pure character renderers: a reading in, text out.
    ('host/coaxial/draw/orientation.py',       ('test_simulated.py', 'test_mcp.py',
                                                RENDER)),
    ('host/coaxial/graphics/engine.py',        (RENDER, VIEWS)),
    ('host/coaxial/graphics/wireframe.py',     (RENDER, VIEWS)),
    ('host/coaxial/draw/ascii3d.py',           ('test_simulated.py',)),
    ('host/coaxial/draw/desk.py',              ('test_simulated.py',)),
    ('host/coaxial/draw/dial.py',              ('test_simulated.py',)),
    ('host/coaxial/graphics/mesh.py',          ('test_simulated.py',)),
    ('host/coaxial/draw/ansi.py',              ('test_simulated.py',)),
    ('host/coaxial/',                          ('test_simulated.py', 'test_parity.py',
                                                'test_mcp.py')),
    # A live view is a loop, a screen and a cable around a renderer that is
    # tested on its own.
    ('host/terminal/views/',                   (STRUCTURE, VIEWS,
                                                'test_simulated.py')),
    ('host/terminal/views/show_session.py',    (VIEWS,) + OLLAMA),
    ('host/terminal/screen.py',                (STRUCTURE, VIEWS,
                                                'test_simulated.py')),
    ('host/tools/',                            OLLAMA),
    ('host/tests/',                            ()),          # decided by name below
    # Firmware and protocol: the byte-level master is the point of it - but the
    # portable core is also compiled and run on this machine, which is the only
    # check on it that does not need a cable.
    ('modbus/',                                (CORE, CONFORMANCE, 'test_mcp.py')),
    # The SHTP layer is hardware-free like the Modbus core, so the host build
    # is what covers it.
    ('shtp/',                                  (SHTP,)),
    # The decimating filter is hardware-free the same way, and its design lives
    # on the host beside it.
    ('filter/',                                (FILTER,)),
    ('host/coaxial/acquire/bessel.py',         (FILTER, STRUCTURE)),
    # The control law is hardware-free like the SHTP layer, and its suite
    # closes the loop through a motor model - the only check on it that needs
    # no motor.
    ('drive/',                                 (DRIVE,)),
    ('host/coaxial/devices/drive.py',          (SENSORLESS, 'test_simulated.py',
                                                'test_parity.py')),
    ('host/coaxial/model/sensorless.py',       (SENSORLESS,)),
    ('host/coaxial/control/commission.py',     (SENSORLESS,)),
    ('host/tools/bench/commission.py',         (STRUCTURE, SENSORLESS)),
    # The stage constants and the host control loops are design arithmetic with
    # closed-form checks; the Monte Carlo drives the compiled law.
    ('host/coaxial/model/inverter.py',         (SENSORLESS,)),
    ('host/coaxial/control/loop.py',           (SENSORLESS, DRIVE)),
    ('host/coaxial/control/motion.py',         (SENSORLESS, 'test_simulated.py')),
    ('host/tools/sim/montecarlo.py',           (STRUCTURE, DRIVE)),
    # BENCH: firmware in the main loop is what slows the board (the thermal
    # observer's per-poll ADC and SPI reads; a poll that lost a Modbus byte).
    ('comms/',                                 (CONFORMANCE, 'test_mcp.py', BENCH)),
    ('board/',                                 (CONFORMANCE, 'test_mcp.py',
                                                'test_parity.py', BENCH)),
    ('core/',                                  (CONFORMANCE, BENCH)),
    # The observer and its envelope are hardware-free like the filter, so the
    # host build is what covers them; the board glue that acts on the budget
    # lives in board/ and is the bench's.
    ('thermal/',                               (THERMAL, CONFORMANCE, BENCH)),
    # The acquisition engine is hardware-free like the observer, so the host
    # build covers it; the glue that reads the converter is board_daq.c and the
    # bench's, and the record's bytes cross the wire.
    ('boot/',                                  (BOOT_CORE, STRUCTURE)),
    ('host/coaxial/devices/boot.py',           (BOOT, STRUCTURE)),
    ('host/coaxial/simulated/boot.py',         (BOOT, STRUCTURE)),
    ('host/tools/target/flash_nodes.py',       (BOOT, STRUCTURE)),
    ('daq/',                                   (DAQ_CORE, CONFORMANCE, 'test_parity.py',
                                                BENCH)),
    ('host/coaxial/model/thermal.py',          (THERMAL, 'test_sensorless.py',
                                                STRUCTURE)),
    # A NOTEBOOK EXAMPLE reaches the library and nothing else reaches it.
    ('notebook_examples/',                     (STRUCTURE,)),
    # And the file the notebooks are written FROM.
    ('host/tools/notebooks/make_notebooks.py', (STRUCTURE,)),
    # A document can only break the docs index and the phrase table.
    ('docs/',                                  ('test_ollama_runner.py',)),
    ('CLAUDE.md',                              ('test_ollama_runner.py',)),
    ('README.md',                              ('test_ollama_runner.py',)),
    # PowerShell around the Python.
    ('terminal/',                              (STRUCTURE,)),
    ('coaxial_tty.ps1',                        (STRUCTURE,)),
    ('env.ps1',                                (STRUCTURE,)),
    ('host/run_tests.ps1',                     (STRUCTURE,)),
    ('setup.ps1',                              (STRUCTURE,)),
    # Neither the CAD export nor the schematic is read by a suite.
    ('render/',                                ()),
    ('electronics/',                           ()),
    ('datasheets/',                            ()),
    ('.gitignore',                             ()),
    ('.vscode/',                               ()),
)

# Every this many commits, run the lot regardless of what changed.
FULL_EVERY = 10

#: Suites the map may settle alone: no board, no ollama - about 40 s all
#: six together, so asking the model costs a 7.6 GB load longer than the
#: run. Where the map has an explicit rule it is also the better answer,
#: written by someone reading the imports.
CHEAP = frozenset({STRUCTURE, CORE, SHTP, DRIVE, SENSORLESS,
                   'test_simulated.py', VIEWS, RENDER})


def _within_tier(args, live_sections):
    """Hold the model's pick inside the tier's budget."""
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
    """The model's own pick, held inside whatever tier is in force."""
    # The picker loads the model too.
    if args.model == 'auto':
        # The machine's own pick, resolved once - a hardcoded default tag asked
        # a 16 GB bench to test against a model an 8 GB one runs.
        try:
            args.model = choose(probe()).tag
        except (OSError, ValueError, KeyError, AttributeError,
                subprocess.SubprocessError):       # what the probe can meet
            args.model = 'gemma4:12b'
    plan, reason = pick_tests.pick(args.model)
    _LOADED.append(_client_for(args.model))

    if plan is None:
        print('   the model picked nothing: %s' % reason)
        print('   falling back to the path map above')
        return None, live_sections

    # Structure is not the model's to drop.
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
    """True when the map knew every path and the answer costs seconds."""
    if any('unmapped' in line for line in why):
        return False

    # An empty pick counts.
    return set(chosen) <= CHEAP


def changed_files(against='HEAD'):
    """Paths touched in the working tree and in the last commit."""
    paths = set()
    for args in (['diff', '--name-only', against],
                 ['diff', '--name-only', '--cached'],
                 ['diff', '--name-only', '%s~1' % against, against]):
        try:
            done = subprocess.run(['git'] + args, cwd=str(ROOT.parent),
                                  capture_output=True, text=True,
                                  encoding='utf-8', errors='replace',
                                  timeout=30)
        except (OSError, subprocess.SubprocessError):
            continue
        if done.returncode == 0:
            paths |= {line.strip().replace('\\', '/')
                      for line in done.stdout.splitlines() if line.strip()}
    return sorted(paths)


def pick(paths):
    """(suites, live_sections, why) for these changed paths."""
    suites, live, why = set(), set(), []
    for path in paths:
        rule = next((wanted for prefix, wanted in TOUCHES
                     if path.startswith(prefix)), None)
        if rule is None:
            why.append('%s -> unmapped, running everything' % path)
            return set(DEFAULT_SUITES) | {CONFORMANCE}, {'all'}, why
        _touched(path, rule, suites, live, why)
    return suites, live, why


def _touched(path, wanted, suites, live, why):
    """One changed path against its rule."""
    name = path.rsplit('/', 1)[-1]
    own = not wanted and path.startswith('host/tests/')
    if own and name.startswith('test_'):
        suites.add(name)
        why.append('%s -> itself' % path)
    if own:
        return
    live.update(item.split(':', 1)[1] for item in wanted
                if item.startswith('live:'))
    suites.update(item for item in wanted if not item.startswith('live:'))
    why.append('%s -> %s' % (path, ', '.join(wanted) or 'nothing reads it'))


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


def _plan(args):
    """Which suites, which subjects, which live sections."""

    live_sections = 'all'
    tags = args.tags
    if args.structure:
        args.file, args.smart, args.live = [STRUCTURE], False, False
    if args.match:
        # One live row and nothing else.
        args.file, args.smart, args.live = [LIVE], False, True
        live_sections = args.sections or 'all'
    if args.coverage:
        args.smart = True
    if args.only:
        # One file, the named tests, nothing else.
        args.file, args.smart, args.live = list(OLLAMA), False, False
    planned = (_smart(args, tags, live_sections)
               if args.smart and not args.file else (tags, live_sections))
    if planned is None:
        return None                # the plan was the whole point of the run
    tags, live_sections = planned

    # Typed explicitly, so it wins over the 'all' default and over a tier's own
    # pick.
    if args.sections and args.live:
        live_sections = args.sections

    return tags, live_sections


def _commits():
    """How many commits this tree has, or 0 outside git."""
    try:
        return int(subprocess.run(
            ['git', 'rev-list', '--count', 'HEAD'], cwd=str(ROOT.parent),
            capture_output=True, text=True, encoding='utf-8',
            errors='replace', timeout=30).stdout.strip())
    except (OSError, ValueError, subprocess.SubprocessError):
        return 0


def _chosen(paths, full, count):
    """(suites, live sections, why) - everything on the full sweep, the
    path map's pick otherwise."""
    if full:
        return (set(DEFAULT_SUITES) | {CONFORMANCE}, {'all'},
                ['commit %d is a multiple of %d - everything'
                 % (count, FULL_EVERY)])
    chosen, picked_live, why = pick(paths)
    if not chosen and not picked_live:
        why.append('nothing changed that any suite covers')
    return chosen, picked_live, why


def _tiered(args, live_sections):
    """The tier's cut of the smart pick: which subjects inside the big
    suites is the judgement call, and it goes to the model - which can
    only ever cost seconds by over-picking, because every way it fails
    returns None and this runs the file whole.
    """
    allowed, sections = plan_for(args.coverage)
    args.file = [f for f in args.file if f in OLLAMA or f in allowed]
    live_sections = sections or ''
    args.live = bool(sections)
    if sections:
        args.file.append(LIVE)
    print('   %d%% tier: %s%s'
          % (args.coverage, ', '.join(args.file),
             ' live:' + sections if sections else ''))
    return live_sections


def _smart(args, tags, live_sections):
    """The smart plan: the changed files against the path map, the full
    sweep every FULL_EVERY commits, the tier's cut, and the model's pick
    of subjects where the map does not settle it.
    """
    count = _commits()
    paths = changed_files()
    full = bool(count) and count % FULL_EVERY == 0 and not args.coverage
    chosen, picked_live, why = _chosen(paths, full, count)
    print('-- smart: %d file%s changed --'
          % (len(paths), '' if len(paths) == 1 else 's'))
    for line in why[:12]:
        print('   ' + line)
    order = list(DEFAULT_SUITES) + [CONFORMANCE]
    # The live suite is not run by name from --file: it is the one with
    # sections, and it is added below.
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
    if args.coverage:
        live_sections = _tiered(args, live_sections)
    # The model decides the list, not the path map.
    if not tags and not full and settled(chosen, why):
        print('   the map knew every path and the answer is seconds - '
              'not asking the model')
    elif not tags and not full:
        tags, live_sections = _ask_model(args, live_sections)
    if args.dry_run:
        return None
    return tags, live_sections


def _extra_for(name, args, tags, live_sections):
    """The flags one suite takes from the plan: the model, its sections and
    match for the live suite; the picked tests or the tags and coverage
    for the ollama ones.
    """
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
    if tags and args.coverage:
        extra += ['--coverage', str(args.coverage)]
    return extra


def _job(name, args, tags, live_sections):
    """One suite run - `run_one`'s tuple, or None where the file is not."""
    path = ROOT / 'tests' / name
    if not path.exists():
        return None
    return run_one(path, timeout=1200 if name == LIVE else 300,
                   extra=_extra_for(name, args, tags, live_sections))


def _results(suites, args, tags, live_sections):
    """(suite, its result) in the order the report lists them: the suites
    that share the machine, in the plan's order, then the ones that want
    it alone.
    """
    took = counts.load().get('seconds') or {}
    sharing = [name for name in suites if name not in ALONE]
    pool = ThreadPoolExecutor(max_workers=max(1, args.jobs))
    try:
        started = {name: pool.submit(_job, name, args, tags, live_sections)
                   for name in sorted(
                       sharing, key=lambda n: -took.get(n, float('inf')))}
        for name in sharing:
            yield name, started[name].result()
    finally:
        pool.shutdown(wait=True, cancel_futures=True)
    for name in suites:
        if name in ALONE:
            yield name, _job(name, args, tags, live_sections)


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
