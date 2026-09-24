"""The tool surface handed to the model: the MCP set, plus code, shell,
report.
"""
import json
import os
import re
import subprocess
import sys
import time
from typing import Any

# host/ on the path: this file's own directory's parent, so it does not
# matter what the working directory is - dbg.py and the runner start from
# different ones - or what any directory along the way is called.
_HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, _HOST)

from .sandbox import clip_ends                             # noqa: E402
from coaxial.comm import ports                                  # noqa: E402
from coaxial.errors import LINK_FAULTS, RigError          # noqa: E402
from coaxial_mcp import detail                            # noqa: E402
from coaxial_mcp import render                            # noqa: E402
from coaxial_mcp.schema import TOOLS as BOARD_TOOLS  # noqa: E402
from coaxial_mcp.schema import coerce as board_coerce  # noqa: E402
from coaxial_mcp.tools import HANDLERS as BOARD_HANDLERS   # noqa: E402
from tools.target import find_board                                          # noqa: E402

# The ceiling on anything a tool may put in front of the model, in characters -
# about a thousand tokens.
TOOL_LIMIT = 4000


def bounded(result, limit=TOOL_LIMIT):
    """One ceiling, at the one place every tool result passes through."""
    if isinstance(result, str):
        return clip_ends(result, limit)
    return result


_BUILD_AND_FLASH = os.path.join(_HOST, 'tools', 'target', 'build_and_flash.py')
_RUN_TESTS = os.path.join(_HOST, 'tools', 'dev', 'run_tests.py')

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
        'description': "Build this firmware and flash it to the board over SWD. Runs host/tools/target/build_and_flash.py with a fixed build preset and a fixed SWD flash command - nothing about the build or the flash is configurable here beyond which of the two steps to run. 'action': 'build' (compile only), 'flash' (flash the existing build only), or 'both' (default).",
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
        'description': "Run this project's own offline test suites (the test_ollama_* files, test_mcp.py, test_simulated.py) and report the exact pass/fail tally each one already counts itself - never a paraphrase. Add 'conformance' to also run test_conformance.py, which needs a real board on COM4.",
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

# Calls that change the board's state.
WRITE_CALLS = {
    'gpio_pin': lambda a: a.get('op') in ('write', 'mode'),
    'gpio_port': lambda a: a.get('op') == 'write',
    'test_gate': lambda a: bool(a.get('enable')),
}

# On by default, and what --read-only takes away.
CODE_CALLS = ('run_python', 'run_command', 'build_firmware')

# Tools that are neither a board handler nor a CODE_CALLS entry, and need no
# gate at all - see _permit().
UNGATED_EXTRAS = ('run_tests', 'link_diagnose')

# Calls that actually reach the board - not `docs`, which reads local files and
# proves nothing about a measurement having happened.
LINK_TOOLS = set(BOARD_HANDLERS) - {'docs'}


def arguments(call):
    """One tool call's arguments, whatever shape Ollama sent them in."""
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
    `full` for one that is not.
    """
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
    """Which stand-in a session with no port is: 'simulated' or 'no board'."""
    try:
        session.board
    except LINK_FAULTS:
        return 'no board'
    return 'simulated'


def _open_link_answers(session):
    """Whether a link this session already holds open answers now."""
    board = session.attached
    if board is None:
        return False
    try:
        board.link.echo(b'?')
        return True
    except LINK_FAULTS:
        return False


class Toolbox:
    """Dispatch, with the operator's policy in front of it."""

    def __init__(self, session, shell=None, scope=None, allow_writes=False,
                 allow_code=True, confirm=None):
        self.session: Any = session
        self.shell = shell
        self.scope = scope
        self.allow_writes = allow_writes
        self.allow_code = allow_code
        self.confirm = confirm            # callable(name, args) -> bool, or None
        # How much of each tool's documentation this run's model gets - see
        # coaxial_mcp/detail.py.
        self.detail = detail.FULL
        self.log = []
        # Set by the caller before a turn's calls run - True unless the caller
        # actually checked and the current question never said "afe".
        self.afe_mentioned = True
        # The operator's own words this turn, for the one check that needs them
        # - see _wrong_side().
        self.asked = ''

    # Which side a word names, in the two languages this loop is spoken in.
    SIDES = {'left': 'left', 'vänster': 'left', 'vanster': 'left',
             'right': 'right', 'höger': 'right', 'hoger': 'right'}

    def _wrong_side(self, args):
        """A node on the other side of the machine from the one asked for."""
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
        """This run's tool list, at this run's detail level."""
        return schemas(TOOLS, self.detail)

    def is_write(self, name, args):
        """Does this call change something, or run something? Both need the
        operator's consent under --confirm, for the same reason."""
        test = WRITE_CALLS.get(name)
        return name in CODE_CALLS or bool(test and test(args or {}))

    def call(self, name, args):
        """Never raises for anything the model did."""
        args = dict(args or {})
        self.log.append((name, args))

        wrong = (self._wrong_side(args)
                 if name == 'devices' and args.get('op') == 'use' else None)
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
        """Which handler, once the policy above has allowed the call."""
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

        # afe_power is deliberately not in WRITE_CALLS (see the comment there -
        # a read-only run still has to be able to power the front end it is
        # reading through), which is exactly why it needs a gate of its own:
        # nothing else stops it firing as a precondition for a reading, the one
        # thing the system prompt already says never to do and, measured live,
        # a model did anyway.
        if (name == 'afe_power' and args.get('action', 'read') != 'read'
                and not self.afe_mentioned):
            raise Refused('not asked for - call analog_read instead, it '
                          'works either way.')

        if (self.confirm is not None and self.is_write(name, args)
                and not self.confirm(name, args)):
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
        """tools/target/build_and_flash.py directly, not through `self.shell`, so
        this works whatever --allow was set to.
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

        relink = (self._relink()
                  if done.returncode == 0 and action != 'build' else None)
        if relink:
            text += '\n' + relink

        # Prefixed the same way every other failure in this file is, so the
        # code_error backstop in debug.py's Chat.ask() catches a failed build
        # or flash exactly like a declined --confirm call - a fact this loop
        # already has that the model does not get to override with its own
        # summary of what happened.
        return text if done.returncode == 0 else 'ERR %s' % text

    def _relink(self):
        """Reopen the serial link after a flash - not just wait for it."""
        if self.session is None or self.session.port is None:
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
        """tools/dev/run_tests.py - every suite's own tally, parsed by that
        script, never re-summarised here or by the model.
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

    def _no_board(self):
        """What a diagnosis says on a session that never had a board."""
        if _stand_in(self.session) == 'no board':
            return ('--no-board this run: every board tool refuses. '
                    '/board auto looks for a real one.')
        return ('this session is on a simulated board - there is no '
                'port to check. /board auto looks for a real one, '
                'debug probe first; /board COM4 tries one by name.')

    @staticmethod
    def _other_ports(listed, configured, baud, unit):
        """Step 5, when asked: whether the board answers on any other port."""
        others = [p for p in listed if p != configured]
        found = next((p for p in others
                      if find_board.probe(p, baud, unit)), None)
        if found:
            return ('5. Tried every other port: %s answered as this board - '
                    'it may have moved there. /reconnect after changing '
                    '--port to it.' % found)
        if others:
            return ('5. Tried every other port (%s): none answered.'
                    % ', '.join(others))
        return None

    def _link_diagnose(self, args):
        """A checklist, most fundamental first, stopping at the step that
        explains the silence rather than running the rest regardless.
        """
        # `simulated` first, then the port - the same order `_interface` asks
        # in, and for the same reason.
        if self.session.simulated or self.session.port is None:
            return self._no_board()
        configured, baud, unit = (self.session.port, self.session.baud,
                                  self.session.unit)

        steps = []
        power_says = self._power_step(steps)
        listed = self._ports_step(steps, configured) if power_says else None
        if listed and self._answers_step(steps, configured, baud, unit, power_says):
            if args.get('probe_other_ports'):
                steps.extend(filter(None, [self._other_ports(listed, configured,
                                                             baud, unit)]))
        return '\n'.join(steps)

    @staticmethod
    def _power_step(steps):
        """Step 1, target power over SWD."""
        voltage, detail = find_board.check_power()
        if voltage is None:
            steps.append('1. Target power (ST-Link/SWD): could not check - %s'
                         % detail)
            return 'Power unconfirmed, but the port is right'
        if voltage < 1.0:
            steps.append(
                '1. Target power (ST-Link/SWD): %.2fV - no power sensed. '
                'Check the ST-Link USB cable is connected, and that the '
                'board itself is powered. Nothing past this point can work '
                'without it.' % voltage)
            return None
        steps.append('1. Target power (ST-Link/SWD): %.2fV - powered, '
                     'cable seated.' % voltage)
        return 'Powered and the port is right'

    @staticmethod
    def _ports_step(steps, configured):
        """Steps 2 and 3, the ports Windows sees and whether the configured
        one is among them.
        """
        listed = find_board.list_ports()
        steps.append('2. COM ports Windows sees: %s' % (', '.join(listed)
                                                         or 'none'))
        if not listed:
            steps.append('   Nothing is enumerating as a serial device - '
                         "check the ST-Link or serial adapter's driver.")
            return None
        if configured not in listed:
            steps.append("3. Configured port %s: not among the ports above "
                         "- the cable may be unplugged from this PC's side, "
                         "or the driver did not enumerate it." % configured)
            return None
        steps.append('3. Configured port %s: present.' % configured)
        return listed

    def _answers_step(self, steps, configured, baud, unit, power_says):
        """Step 4, whether the board answers right now, and why not when
        not.
        """
        if (_open_link_answers(self.session)
                or find_board.probe(configured, baud, unit)):
            # Measured directly, not inferred from the port merely being
            # present - so this also correctly says "up" when the link had
            # already recovered by the time anything reached for this tool.
            steps.append('4. Board answers on %s right now: yes - the link '
                         'is up.' % configured)
            return False
        steps.append('4. Board answers on %s right now: no.' % configured)
        if find_board.port_state(configured, baud, unit) == ports.BUSY:
            steps.append('   %s is open in another process - that is why nothing answers here. Close the other session, or point this one at another port.' % configured)
            return False
        steps.append('   %s, so check nothing else has %s open, and that '
                     'the last programmer run ended with --start, not '
                     '-hardRst (a halted core answers nothing).'
                     % (power_says, configured))
        return True

    def _board(self, name, args):
        # Coerced against the tool's own schema first: see
        # coaxial_mcp.tools.coerce for what a small model sends instead.
        return BOARD_HANDLERS[name](self.session, detail=self.detail,
                                    **board_coerce(name, args))
