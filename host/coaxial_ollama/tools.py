"""The tool surface handed to the model: the MCP set, plus code, shell, report.

The fourteen board tools are imported from `coaxial_mcp.tools`, never re-declared:
one description, one set of renderers, one place a capability is added. A second
copy for Ollama would stay plausible while going out of date.

Six more make this a runner rather than a chat window:

  run_command    an allowlisted process: a build, a flash, `python -m coaxial`.
  run_python     code against the live `board`, in a namespace that persists.
  build_firmware tools/build_and_flash.py, fixed arguments - the narrow answer
                 to "build and flash", so a session needs no wider surface.
  run_tests      tools/run_tests.py - each suite's own tally, parsed by that
                 script. Ungated: it touches neither state nor flash.
  link_diagnose  why the board is silent, OS-level rather than another Modbus
                 call the dead link would fail too. Ungated for the same reason.
  report         how a step ends: a value and a unit, never a verdict.

Twenty against coaxial_mcp's fourteen, and the extra buys one thing: a plan
step can say "work out which channel this is" instead of naming a function
code.

`report` has no pass/fail field - that is where a model would put an opinion the
runner then has to weigh. plan.Limit decides, in Python.
"""
import json
import os
import re
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
# - about a thousand tokens. A result is re-sent on every later turn of the
# same question, so one unbounded build log is a prompt that keeps growing.
# At the dispatch point rather than per handler, so it holds for the ones not
# written yet: Shell and Scope always clipped, build_firmware, run_tests and
# link_diagnose ran subprocesses directly and did not.
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

# On by default, and what --read-only takes away. `allow_writes` cannot police
# them: run_python holds the same board object the gpio tools do, so code is
# either trusted for this run or unavailable. build_firmware counts as a write
# for --confirm whatever `action` says - the risk is the flash step, and
# `action` is a model argument this loop does not trust before --confirm.
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

    Usually an object; some builds send a JSON string. One that will not parse
    is kept under `_unparsed` rather than dropped - returning {} instead
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


def _stand_in(session):
    """Which stand-in a session with no port is: 'simulated' or
    'no board'.

    Asked of the thing that matters rather than the class name: a
    NoBoard refuses to produce a board at all, a SimulatedSession hands
    one over.
    """
    try:
        session.board
    except Exception:                                     # noqa: BLE001
        return 'no board'
    return 'simulated'


def _open_link_answers(session):
    """Whether a link this session already holds open answers now.

    Never opens anything and never raises: a session with no board
    cached returns False, and the caller falls through to the ordinary
    probe that opens the port itself.
    """
    board = getattr(session, '_board', None)
    if board is None:
        return False
    try:
        board.link.echo(b'?')
        return True
    except Exception:                                     # noqa: BLE001
        return False


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
        # The operator's own words this turn, for the one check that needs
        # them - see _wrong_side(). Empty means "not wired", which every
        # existing caller is, and the check then stays out of the way.
        self.asked = ''

    # Which side a word names, in the two languages this loop is spoken
    # in. Not a translation table - the only distinction that matters here
    # is left from right, and it is the one a machine cannot afford to get
    # wrong.
    SIDES = {'left': 'left', 'vänster': 'left', 'vanster': 'left',
             'right': 'right', 'höger': 'right', 'hoger': 'right'}

    def _wrong_side(self, args):
        """A node on the other side of the machine from the one asked for.

        Measured: "kommunicera med vänster knä" was sent as
        `name='right knee'` - the model mistranslated it in the call and
        got it right in the prose that followed. On a humanoid the wrong
        limb moving is the failure that costs something, so the operator's
        own word wins over the model's rendering of it.

        Only fires when both sides are named and they disagree. A question
        that names no side, or a target that names none, is nobody's
        business here.
        """
        words = set(re.findall(r'[^\W\d_]+', (self.asked or '').lower()))
        wanted = {self.SIDES[w] for w in words if w in self.SIDES}
        if len(wanted) != 1:
            return None

        target = str(args.get('name') or args.get('bus') or '').lower()
        got = {self.SIDES[w] for w in re.findall(r'[^\W\d_]+', target)
               if w in self.SIDES}
        if not got and args.get('bus'):
            got = {'left' if str(args['bus']).upper().startswith('L')
                   else 'right'} if str(args['bus']).upper()[:1] in 'LR'                 else set()
        if got and got != wanted:
            return ('ERR you asked for the %s side and this selects the %s '
                    'one (%r). Nothing moved; say it again or give the bus '
                    'and node.'
                    % (wanted.pop(), got.pop(), args.get('name')
                       or args.get('bus')))
        return None

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

        A bad channel name, an unknown tool, a refused pin: answers, not
        exceptions. The model has to read them to correct itself.
        """
        args = dict(args or {})
        self.log.append((name, args))

        if name == 'devices' and args.get('op') == 'use':
            wrong = self._wrong_side(args)
            if wrong:
                return wrong

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
        """tools/build_and_flash.py directly, not through `self.shell`, so
        this works whatever --allow was set to. Nothing here for a model to
        choose but `action`: no preset, no elf path, no flash arguments.
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
                                  encoding='utf-8', errors='replace',
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

        `--start` resets the MCU, which reboots into its ASCII console, not
        the binary Modbus mode a cached `Session.board` assumes. Measured
        live: FLASH ok, then `NoReplyError: ... silence` on hardware that had
        just come back up.

        reset() drops the stale handle; the three retries are for the reboot,
        not the handshake, and cost nothing against a flash that took a second.
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
        """tools/run_tests.py - every suite's own tally, parsed by that
        script, never re-summarised here or by the model. A paraphrase of
        test output is the plausible-but-unverified line FINDINGS warns about.
        """
        argv = [sys.executable, _RUN_TESTS]
        if args.get('conformance'):
            argv.append('--conformance')
        try:
            done = subprocess.run(argv, capture_output=True, text=True,
                                  encoding='utf-8', errors='replace',
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
        """A checklist, most fundamental first, stopping at the step that
        explains the silence rather than running the rest regardless.

        `tools/find_board.py` does the work - the same module
        board_prompt/ComPort.ps1 shells out to, imported here since this call
        is already in-process, so "does this port answer" cannot drift
        between a live session and the preflight.

        Step 1, target power over SWD, is the one the serial side cannot
        check at all: measured, an unplugged ST-Link read `Voltage: 0.00V`
        where serial alone only ever said "silence".
        """
        configured = getattr(self.session, 'port', None)
        baud = getattr(self.session, 'baud', 115200)
        unit = getattr(self.session, 'unit', 1)

        # `simulated` first, then the port - the same order `_interface`
        # asks in, and for the same reason. A stand-in's `port` is a bus
        # label ('AX'), never None, so `configured is None` on its own let
        # a fallen-back session through to a 15s SWD probe and then
        # "Configured port AX: not among the ports above - the cable may
        # be unplugged", about a session that never had a cable. Measured,
        # with the board's JTAG connector pulled.
        if getattr(self.session, 'simulated', False) or configured is None:
            # Not step 1 - this isn't a rung on the checklist, it's whether
            # there is a real board to run one against at all. A stand-in
            # has no SWD to check power over either, and checking it anyway
            # would spend several real seconds proving nothing about a
            # session that was never going to have a board.
            #
            # It does not say "--no-board or --simulated this run" any more:
            # a session that found nothing at startup falls back on its own,
            # and naming two flags the operator never typed is a false
            # statement about how the session was started. Measured - asked
            # "byter du till debugproben" on an auto-fallen-back session,
            # this line was the whole answer on screen, and it named the
            # wrong reason and no way out.
            if _stand_in(self.session) == 'no board':
                return ('--no-board this run: every board tool refuses. '
                        '/board auto looks for a real one.')
            return ('this session is on a simulated board - there is no '
                    'port to check. /board auto looks for a real one, '
                    'debug probe first; /board COM4 tries one by name.')

        steps = []
        voltage, detail = find_board.check_power()
        if voltage is None:
            steps.append('1. Target power (ST-Link/SWD): could not check - %s'
                         % detail)
            # Step 4's closing advice rests on step 1. It used to say
            # "Powered" whatever step 1 concluded, which on a pulled cable
            # asserted the one thing that was false.
            power_says = 'Power unconfirmed, but the port is right'
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
            power_says = 'Powered and the port is right'

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

        # The session's own handle first, and a second open only if it
        # has none. Measured: with the link up and the session holding
        # COM4, find_board.probe opened it a second time, Windows
        # refused, and the checklist printed "4. Board answers on COM4
        # right now: no" one line under "3. Configured port COM4:
        # present." - a false statement about live hardware, produced
        # by the diagnostic itself.
        if (_open_link_answers(self.session)
                or find_board.probe(configured, baud, unit)):
            # Measured directly, not inferred from the port merely being
            # present - so this also correctly says "up" when the link had
            # already recovered by the time anything reached for this tool.
            steps.append('4. Board answers on %s right now: yes - the link '
                         'is up.' % configured)
            return '\n'.join(steps)
        steps.append('4. Board answers on %s right now: no.' % configured)
        # Why it is not answering, not just that it is not. A port another
        # process holds open reads exactly like a board that stopped
        # talking, and this used to guess at the difference in prose -
        # measured, two dbg.py sessions had COM4 open, every probe read
        # silent, and the board was diagnosed as halted, started over SWD
        # and reflashed. None of that was the matter with it.
        if find_board.port_state(configured, baud, unit) == find_board.BUSY:
            steps.append('   %s is open in another process - that is why nothing answers here. Close the other session, or point this one at another port.' % configured)
            return '\n'.join(steps)
        steps.append('   %s, so check nothing else has %s open, and that '
                     'the last programmer run ended with --start, not '
                     '-hardRst (a halted core answers nothing).'
                     % (power_says, configured))

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
        # `detail` is this run's, not the model's - coerce() would drop it as
        # unknown. Only `docs` reads it; every other handler takes **_.
        return BOARD_HANDLERS[name](self.session, detail=self.detail,
                                    **board_coerce(name, args))
