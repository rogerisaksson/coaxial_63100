"""The tool surface handed to the model: the MCP set, plus code, shell, report.

The nine board tools are imported from `coaxial_mcp.tools`, not re-declared.
That is deliberate and it is the main structural decision in this package: there
is one description of what a fixture can do to this board, one set of hand-tuned
compact renderers, and one place a new capability gets added. A second, drifting
copy of that surface written for Ollama would be the worst kind of duplication -
the kind that stays plausible while going out of date.

Five tools are added here, and they are what makes this a runner rather than a
chat window:

  run_command    an allowlisted process, for the things that are not the board:
                 a build, a flash, `python -m coaxial`.
  run_python     code against the live `board` object, in a namespace that
                 persists across the whole run.
  build_firmware host/tools/build_and_flash.py, fixed arguments only - the
                 narrow, always-available answer to "build and flash", so a
                 session does not need run_command's wider surface just for
                 that one job.
  run_tests      host/tools/run_tests.py - every offline suite's own tally,
                 parsed by that script rather than summarised by the model.
                 Ungated: it never touches the board's state or its flash.
  link_diagnose  why the board is not answering - OS-level (COM ports
                 present, driver enumeration), not another Modbus call the
                 dead link would fail too. Also ungated, for the same reason.
  report         how a step ends. The model reports a value and a unit; it is
                 never told the limit and never asked for a verdict.

Fifteen tools, against the nine that coaxial_mcp keeps to. The extra cost is
real and it is paid for one thing: a plan step can say "work out which channel
this is" instead of naming a function code.

Note what `report` is not: an assertion. It carries no pass/fail field, because
a field like that is a place for a model to put an opinion, and the runner would
then have to decide whether to believe it. plan.Limit decides, in Python.
"""
import json
import os
import subprocess
import sys
import time

# host/ and host/tools on the path: this file's own directory's parent, so
# it does not matter what the working directory is - dbg.py and the runner
# start from different ones - or what any directory along the way is called.
_HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
_TOOLS = os.path.join(_HOST, 'tools')
sys.path.insert(0, _HOST)
sys.path.insert(0, _TOOLS)

from coaxial.errors import RigError                       # noqa: E402
from coaxial_mcp import detail                            # noqa: E402
from coaxial_mcp import render                            # noqa: E402
from coaxial_mcp.tools import HANDLERS as BOARD_HANDLERS   # noqa: E402
from coaxial_mcp.tools import TOOLS as BOARD_TOOLS         # noqa: E402
from coaxial_mcp.tools import coerce as board_coerce       # noqa: E402
import find_board                                          # noqa: E402

from .sandbox import clip_ends                             # noqa: E402

# The ceiling on anything a tool may put in front of the model, in characters
# - about a thousand tokens. Not a tidiness rule: a tool result goes straight
# into the conversation and is re-sent on every following turn of the same
# question, so one unbounded build log is a prompt that keeps growing until
# the daemon has to allocate for it. `Shell` and `Scope` have clipped their
# own output all along; build_firmware, run_tests and link_diagnose ran their
# subprocesses directly and did not, which is exactly where the largest
# outputs in this package come from. The cap lives at the dispatch point
# instead, so it holds for every handler including the ones not written yet.
TOOL_LIMIT = 4000


def bounded(result, limit=TOOL_LIMIT):
    """One ceiling, at the one place every tool result passes through.

    `Reported` is not text and is never clipped: it carries a value and a
    note the runner judges in Python, not something the model reads back.
    """
    if isinstance(result, str):
        return clip_ends(result, limit)
    return result


_BUILD_AND_FLASH = os.path.join(_TOOLS, 'build_and_flash.py')
_RUN_TESTS = os.path.join(_TOOLS, 'run_tests.py')

EXTRA_TOOLS = [
    {
        'name': 'run_python',
        'description': 'Run Python against the live board. `board`, coaxial, scaling, math, statistics are in scope and persist between calls; the last expression is the result.',
        'description_terse': 'Python against the live board. `board` is in scope and persists; the last expression is the result.',
        'inputSchema': {
            'type': 'object',
            'properties': {'code': {'type': 'string'}},
            'required': ['code'],
        },
    },
    {
        'name': 'run_command',
        'description': 'Run one allowlisted program, argv only - no pipes or redirection. For builds, flashing and CLI tools.',
        'description_terse': 'Run one allowlisted program, argv only - no pipes or redirection.',
        'inputSchema': {
            'type': 'object',
            'properties': {'cmd': {'type': 'string'},
                           'timeout_s': {'type': 'number'}},
            'required': ['cmd'],
        },
    },
    {
        'name': 'build_firmware',
        'description': "Build this firmware and flash it to the board over SWD. Runs host/tools/build_and_flash.py with a fixed build preset and a fixed SWD flash command - nothing about the build or the flash is configurable here beyond which of the two steps to run. 'action': 'build' (compile only), 'flash' (flash the existing build only), or 'both' (default).",
        'description_terse': "Build this firmware and flash it over SWD. action: 'build', 'flash' or 'both' (default). Nothing else is configurable.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'action': {'type': 'string', 'enum': ['build', 'flash', 'both']},
            },
        },
    },
    {
        'name': 'run_tests',
        'description': "Run this project's own offline test suites (test_ollama.py, test_mcp.py, test_simulated.py) and report the exact pass/fail tally each one already counts itself - never a paraphrase. Add 'conformance' to also run test_conformance.py, which needs a real board on COM4.",
        'description_terse': "Run the offline test suites and report each one's own pass/fail tally, never a paraphrase. 'conformance' adds the suite that needs the board.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'conformance': {'type': 'boolean'},
            },
        },
    },
    {
        'name': 'link_diagnose',
        'description': "The board is not answering (ConnectError, NoReplyError, 'link is down'): call this to find out why, instead of just repeating the raw error. Checks in order, most fundamental first, stopping at whichever step actually explains it: 1) target power over SWD via the ST-Link, 2) COM ports Windows sees, 3) whether the configured one is among them, 4) whether the board actually answers on it right now. 'probe_other_ports' adds a step 5, trying every other port for a board that answers somewhere other than where it was told to look.",
        'description_terse': "The board is not answering: call this to find out why instead of repeating the error. Checks target power, the COM ports, and whether the board answers. 'probe_other_ports' tries the others too.",
        'inputSchema': {
            'type': 'object',
            'properties': {
                'probe_other_ports': {'type': 'boolean'},
            },
        },
    },
    {
        'name': 'report',
        'description': 'Finish this step: the value you measured, its unit, and how you got it. Call exactly once, last.',
        'inputSchema': {
            'type': 'object',
            'properties': {
                'value': {'type': 'number'},
                'unit': {'type': 'string'},
                'note': {'type': 'string',
                         'description': 'One or two lines: method and anything odd'},
            },
            'required': ['note'],
        },
    },
]

TOOLS = BOARD_TOOLS + EXTRA_TOOLS

# Calls that change the board's state. The AFE switch is deliberately not here:
# it powers the ADC reference, so a run that could not touch it could not measure
# anything, and every analog step would open with a refusal.
WRITE_CALLS = {
    'gpio_pin': lambda a: a.get('op') in ('write', 'mode'),
    'gpio_port': lambda a: a.get('op') == 'write',
    'test_gate': lambda a: bool(a.get('enable')),
}

# Code and commands are on by default - a runner that cannot run them is not the
# tool that was asked for - and they are what --read-only takes away. Note that
# `allow_writes` cannot police them: run_python holds the same board object the
# gpio tools do, so code is either trusted for this run or it is not available.
# build_firmware is here too: every call in CODE_CALLS is unconditionally a
# write for --confirm purposes (see is_write), on purpose - the risk this
# board carries is in the flash step, not the build, and 'action' is a model
# argument this loop does not get to trust before --confirm has seen it.
CODE_CALLS = ('run_python', 'run_command', 'build_firmware')

# Tools that are neither a board handler nor a CODE_CALLS entry, and need no
# gate at all - see _permit(). Both read local/OS state and never touch the
# board or its flash.
UNGATED_EXTRAS = ('run_tests', 'link_diagnose')

# Calls that actually reach the board - not `docs`, which reads local files and
# proves nothing about a measurement having happened. Shared by debug.py (to
# tell a live link failure from one three turns stale) and runner.py (to tell
# a real report from one nobody measured).
LINK_TOOLS = set(BOARD_HANDLERS) - {'docs'}


def arguments(call):
    """One tool call's arguments, whatever shape Ollama sent them in.

    Usually an object; some builds send a JSON string. A string that will
    not parse is kept under `_unparsed` rather than dropped - `coerce`
    passes unknown keys through and every handler takes `**_`, so it costs
    nothing at the call site and it is the difference between a transcript
    that records what the model actually sent and one that quietly shows
    an empty argument list. debug.py used to return {} here instead, which
    turned a malformed `analog_read` into a silent read of every channel.
    """
    args = (call.get('function') or {}).get('arguments')
    if isinstance(args, str):
        try:
            args = json.loads(args or '{}')
        except ValueError:
            return {'_unparsed': args}
    return args if isinstance(args, dict) else {}


def schemas(tools=TOOLS, level=detail.FULL):
    """MCP tool specs in the shape Ollama's /api/chat wants, at one level of
    detail - `terse` for a model paying for this list out of 8192 tokens,
    `full` for one that is not. See coaxial_mcp/detail.py; nothing here
    decides which, it is passed in from whoever knows the model."""
    return [{'type': 'function',
             'function': {'name': spec['name'],
                          'description': spec['description'],
                          'parameters': spec['inputSchema']}}
            for spec in detail.apply(tools, level)]


class Refused(Exception):
    """A call the operator's flags do not permit. Told to the model, not raised."""


class Reported:
    """Marker: the step is over. Carries what the model said, unjudged."""

    def __init__(self, value=None, unit='', note=''):
        self.value = value
        self.unit = unit
        self.note = note

    def __repr__(self):
        return '<Reported %r %s>' % (self.value, self.unit)


class Toolbox:
    """Dispatch, with the operator's policy in front of it."""

    def __init__(self, session, shell=None, scope=None, allow_writes=False,
                 allow_code=True, confirm=None):
        self.session = session
        self.shell = shell
        self.scope = scope
        self.allow_writes = allow_writes
        self.allow_code = allow_code
        self.confirm = confirm            # callable(name, args) -> bool, or None
        # How much of each tool's documentation this run's model gets - see
        # coaxial_mcp/detail.py. An attribute rather than a constructor
        # argument threaded through every caller: /detail flips it mid
        # session, and every existing caller keeps the full text it has
        # always had.
        self.detail = detail.FULL
        self.log = []
        # Set by the caller before a turn's calls run - True unless the
        # caller actually checked and the current question never said "afe".
        # Defaults permissive so anything that never wires this (every
        # existing test, a plan step, a bare Toolbox in a script) keeps its
        # old behaviour; only debug.py's own repl() and one-shot path set it
        # from the real question text. See _permit() for why it exists.
        self.afe_mentioned = True

    def schemas(self):
        """This run's tool list, at this run's detail level. `detail` is set
        by whoever built the Toolbox and knows which model is reading -
        debug.py from --detail, the runner from the same flag - and defaults
        to the full text for a caller that never said, since that caller is
        not the one short of context."""
        return schemas(TOOLS, self.detail)

    def is_write(self, name, args):
        """Does this call change something, or run something? Both need the
        operator's consent under --confirm, for the same reason."""
        test = WRITE_CALLS.get(name)
        return name in CODE_CALLS or bool(test and test(args or {}))

    def call(self, name, args):
        """Never raises for anything the model did. Returns text, or Reported.

        Model mistakes - a bad channel name, an unknown tool, a refused pin -
        are answers, not exceptions: the model has to read them to correct
        itself, and a traceback in the transcript would only mean the run died
        where it could have recovered.
        """
        args = dict(args or {})
        self.log.append((name, args))

        if name == 'report':
            return Reported(args.get('value'), args.get('unit', ''),
                            args.get('note', ''))

        try:
            self._permit(name, args)
            return bounded(self._dispatch(name, args))
        except Refused as exc:
            return 'ERR %s' % exc
        except (RigError, ValueError, KeyError, TypeError) as exc:
            return render.error(exc)

    def _dispatch(self, name, args):
        """Which handler, once the policy above has allowed the call. Split
        out so every result leaves through exactly one `bounded()` - a new
        tool added to this chain cannot forget the ceiling by being written
        with its own `return`."""
        if name == 'run_python':
            return self._python(args)
        if name == 'run_command':
            return self._command(args)
        if name == 'build_firmware':
            return self._build_firmware(args)
        if name == 'run_tests':
            return self._run_tests(args)
        if name == 'link_diagnose':
            return self._link_diagnose(args)
        return self._board(name, args)

    # ---- policy ------------------------------------------------------------

    def _permit(self, name, args):
        # Neither a board tool nor a CODE_CALLS entry, on purpose: neither
        # touches the board's state or its flash, so neither is gated by
        # --read-only, --allow-writes or --confirm - the same reasoning that
        # leaves `docs` ungated, just for different local actions.
        if (name not in BOARD_HANDLERS and name not in CODE_CALLS
                and name not in UNGATED_EXTRAS):
            raise Refused('unknown tool %r' % name)

        if name in CODE_CALLS and not self.allow_code:
            raise Refused('%s is disabled for this run (--read-only). Use the '
                          'board tools.' % name)

        test = WRITE_CALLS.get(name)
        if test and test(args) and not self.allow_writes:
            raise Refused('%s changes state and this run may only read. The '
                          'operator would have to pass --allow-writes.' % name)

        # afe_power is deliberately not in WRITE_CALLS (see the comment
        # there - a read-only run still has to be able to power the front
        # end it is reading through), which is exactly why it needs a gate
        # of its own: nothing else stops it firing as a precondition for a
        # reading, the one thing the system prompt already says never to do
        # and, measured live, a model did anyway. analog_read works with the
        # AFE either way and reports which - there is never a reading that
        # actually needs this call.
        if (name == 'afe_power' and args.get('action', 'read') != 'read'
                and not self.afe_mentioned):
            raise Refused('not asked for - call analog_read instead, it '
                          'works either way.')

        if self.confirm is not None and self.is_write(name, args):
            if not self.confirm(name, args):
                raise Refused('the operator declined this call. Do not retry '
                              'it; report what you have or explain what is '
                              'missing.')

    # ---- the three that are not the board ---------------------------------

    def _python(self, args):
        if self.scope is None:
            raise Refused('no python scope in this run')
        code = args.get('code')
        if not code:
            raise Refused('run_python needs `code`')
        self.scope.bind(self.session.board)
        return self.scope.run(code)

    def _command(self, args):
        if self.shell is None:
            raise Refused('no shell in this run')
        cmd = args.get('cmd')
        if not cmd:
            raise Refused('run_command needs `cmd`')
        return self.shell.run(cmd, args.get('timeout_s'))

    def _build_firmware(self, args):
        """host/tools/build_and_flash.py, invoked directly - not through
        `self.shell`, so this tool works regardless of what --allow was
        set to. There is nothing here for a model to choose beyond
        `action`: no preset, no elf path, no flash arguments - see that
        script for why those are fixed rather than passed through.
        """
        action = args.get('action') or 'both'
        if action not in ('build', 'flash', 'both'):
            raise Refused("build_firmware: action must be 'build', 'flash' "
                          "or 'both', not %r" % action)
        argv = [sys.executable, _BUILD_AND_FLASH]
        if action != 'both':
            argv.append('--%s-only' % action)
        try:
            done = subprocess.run(argv, capture_output=True, text=True,
                                  timeout=600)
        except subprocess.TimeoutExpired:
            return 'ERR build_firmware timed out after 600s'
        except OSError as exc:
            return 'ERR build_firmware could not start: %s' % exc

        parts = ['exit=%d' % done.returncode]
        if done.stdout.strip():
            parts.append(done.stdout.rstrip())
        if done.stderr.strip():
            parts.append('stderr: ' + done.stderr.rstrip())
        text = '\n'.join(parts)

        if done.returncode == 0 and action != 'build':
            relink = self._relink()
            if relink:
                text += '\n' + relink

        # Prefixed the same way every other failure in this file is, so the
        # code_error backstop in debug.py's Chat.ask() catches a failed build
        # or flash exactly like a declined --confirm call - a fact this loop
        # already has that the model does not get to override with its own
        # summary of what happened.
        return text if done.returncode == 0 else 'ERR %s' % text

    def _relink(self):
        """Reopen the serial link after a flash - not just wait for it.

        `--start` resets the MCU, which reboots into its ASCII console the
        same way it does after any power-up (see coaxial/board.py's
        `open_binary`) - not the binary Modbus mode a cached `Session.board`
        assumes it is still in. Left alone, the very next board tool this
        turn or the next reaches for sends a Modbus frame at a board that is
        listening for console text, gets silence back, and reports the link
        down - measured live: `build_firmware` said FLASH ok and the next
        call was `NoReplyError: ... silence` a moment later, on hardware
        that had, in fact, just come back up.

        `session.reset()` drops the stale handle; the retries are for the
        reboot itself, not the handshake - HAL init after `--start` is
        sub-millisecond on this part, but three tries a third of a second
        apart costs nothing against a flash that just took over a second,
        and buys margin against a slow one.
        """
        if self.session is None or not hasattr(self.session, 'port'):
            # NoBoard (or no session at all) - nothing was ever connected in
            # this run, so there is nothing a flash could have disconnected.
            return ''
        self.session.reset()
        last = None
        for attempt in range(3):
            if attempt:
                time.sleep(0.3)
            try:
                self.session.board
                return 'link re-established'
            except RigError as exc:
                last = exc
        return ('WARNING: link has not answered since the flash (%s) - the '
                'board may still be rebooting. Try a board tool again '
                'before reporting a dead link.' % last)

    def _run_tests(self, args):
        """host/tools/run_tests.py - every suite's own tally, parsed by that
        script, never re-summarised here or by the model. See its docstring
        for why: a model's paraphrase of test output is exactly the kind of
        plausible-but-unverified line this project's own FINDINGS.md warns
        against.
        """
        argv = [sys.executable, _RUN_TESTS]
        if args.get('conformance'):
            argv.append('--conformance')
        try:
            done = subprocess.run(argv, capture_output=True, text=True,
                                  timeout=300)
        except subprocess.TimeoutExpired:
            return 'ERR run_tests timed out after 300s'
        except OSError as exc:
            return 'ERR run_tests could not start: %s' % exc

        text = (done.stdout or '').strip()
        if not text:
            text = (done.stderr or '').strip()
        return text if done.returncode == 0 else 'ERR %s' % text

    def _link_diagnose(self, args):
        """A step-by-step checklist, most fundamental first, stopping at the
        first step that explains the silence rather than running every
        later one regardless - `de mest logiska alternativen`, in the order
        that actually rules each one out or in.

        `host/tools/find_board.py` does the actual work (port listing,
        probing, and the SWD power check) - the same module
        `board_prompt/ComPort.ps1`'s Test-BoardPort/Find-BoardPort call out
        to, as a subprocess, for -AutodetectComport before a Python session
        even exists. One implementation, imported here rather than shelled
        out to since this call is already inside the same process, so "does
        this port answer" cannot drift between what a live session finds
        and what the preflight found.

        Step 1, target power over SWD, is the one this tool could not check
        before and the board's own serial side cannot check at all - see
        find_board.check_power(). Measured live on this bench: an unplugged
        ST-Link cable read `Voltage: 0.00V`, where the serial side alone
        only ever said "silence" - a real fact, but a far less specific one
        pointing at the same actual cause.
        """
        configured = getattr(self.session, 'port', None)
        baud = getattr(self.session, 'baud', 115200)
        unit = getattr(self.session, 'unit', 1)

        if configured is None:
            # Not step 1 - this isn't a rung on the checklist, it's whether
            # there is a real board to run one against at all. A
            # --no-board/--simulated run has no SWD to check power over
            # either, and checking it anyway would spend several real
            # seconds proving nothing about a session that was never going
            # to have a board.
            return ('no configured port to check (--no-board or '
                   '--simulated this run).')

        steps = []
        voltage, detail = find_board.check_power()
        if voltage is None:
            steps.append('1. Target power (ST-Link/SWD): could not check - %s'
                         % detail)
        elif voltage < 1.0:
            steps.append(
                '1. Target power (ST-Link/SWD): %.2fV - no power sensed. '
                'Check the ST-Link USB cable is connected, and that the '
                'board itself is powered. Nothing past this point can work '
                'without it.' % voltage)
            return '\n'.join(steps)
        else:
            steps.append('1. Target power (ST-Link/SWD): %.2fV - powered, '
                         'cable seated.' % voltage)

        ports = find_board.list_ports()
        steps.append('2. COM ports Windows sees: %s' % (', '.join(ports)
                                                         or 'none'))
        if not ports:
            steps.append('   Nothing is enumerating as a serial device - '
                         "check the ST-Link or serial adapter's driver.")
            return '\n'.join(steps)

        if configured not in ports:
            steps.append("3. Configured port %s: not among the ports above "
                         "- the cable may be unplugged from this PC's side, "
                         "or the driver did not enumerate it." % configured)
            return '\n'.join(steps)
        steps.append('3. Configured port %s: present.' % configured)

        if find_board.probe(configured, baud, unit):
            # Measured directly, not inferred from the port merely being
            # present - so this also correctly says "up" when the link had
            # already recovered by the time anything reached for this tool.
            steps.append('4. Board answers on %s right now: yes - the link '
                         'is up.' % configured)
            return '\n'.join(steps)
        steps.append('4. Board answers on %s right now: no.' % configured)
        steps.append('   Powered and the port is right, so check nothing '
                     'else has %s open, and that the last programmer run '
                     'ended with --start, not -hardRst (a halted core '
                     'answers nothing).' % configured)

        if args.get('probe_other_ports'):
            others = [p for p in ports if p != configured]
            found = next((p for p in others
                         if find_board.probe(p, baud, unit)), None)
            if found:
                steps.append('5. Tried every other port: %s answered as '
                             'this board - it may have moved there. '
                             '/reconnect after changing --port to it.'
                             % found)
            elif others:
                steps.append('5. Tried every other port (%s): none '
                             'answered.' % ', '.join(others))

        return '\n'.join(steps)

    def _board(self, name, args):
        # Coerced against the tool's own schema first: see
        # coaxial_mcp.tools.coerce for what a small model sends instead.
        #
        # `detail` rides along with every board call and is not one of the
        # model's arguments - it is this run's, and coerce() would drop it as
        # unknown. Only `docs` reads it (a section clipped for the reader
        # rather than for the document); every other handler takes **_ and
        # ignores it, which is what keeps this one line rather than a
        # per-handler signature change.
        return BOARD_HANDLERS[name](self.session, detail=self.detail,
                                    **board_coerce(name, args))
