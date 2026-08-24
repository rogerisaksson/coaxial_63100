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
import os
import subprocess
import sys
import time

sys.path.insert(0, __file__.rsplit('coaxial_ollama', 1)[0])

from coaxial.errors import RigError                       # noqa: E402
from coaxial_mcp import render                            # noqa: E402
from coaxial_mcp.tools import HANDLERS as BOARD_HANDLERS   # noqa: E402
from coaxial_mcp.tools import TOOLS as BOARD_TOOLS         # noqa: E402
from coaxial_mcp.tools import coerce as board_coerce       # noqa: E402

# host/tools/build_and_flash.py and host/tools/run_tests.py, resolved from
# this file's own location rather than the caller's cwd - dbg.py and the
# runner start from different directories, and these tools have to reach the
# same scripts either way.
_BUILD_AND_FLASH = os.path.join(__file__.rsplit('coaxial_ollama', 1)[0],
                                'tools', 'build_and_flash.py')
_RUN_TESTS = os.path.join(__file__.rsplit('coaxial_ollama', 1)[0],
                          'tools', 'run_tests.py')

EXTRA_TOOLS = [
    {
        'name': 'run_python',
        'description': 'Run Python against the live board. `board`, coaxial, scaling, math, statistics are in scope and persist between calls; the last expression is the result.',
        'inputSchema': {
            'type': 'object',
            'properties': {'code': {'type': 'string'}},
            'required': ['code'],
        },
    },
    {
        'name': 'run_command',
        'description': 'Run one allowlisted program, argv only - no pipes or redirection. For builds, flashing and CLI tools.',
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
        'inputSchema': {
            'type': 'object',
            'properties': {
                'conformance': {'type': 'boolean'},
            },
        },
    },
    {
        'name': 'link_diagnose',
        'description': "The board is not answering (ConnectError, NoReplyError, 'link is down'): call this to find out why, instead of just repeating the raw error. Lists the COM ports Windows actually sees right now and whether the configured one is among them - a missing port is a cable or driver problem, a present-but-silent one is a power or wiring problem. 'probe_other_ports' also tries every other port for a few seconds each, for a board that answers somewhere other than where it was told to look.",
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


def schemas(tools=TOOLS):
    """MCP tool specs in the shape Ollama's /api/chat wants."""
    return [{'type': 'function',
             'function': {'name': spec['name'],
                          'description': spec['description'],
                          'parameters': spec['inputSchema']}}
            for spec in tools]


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
        self.log = []
        # Set by the caller before a turn's calls run - True unless the
        # caller actually checked and the current question never said "afe".
        # Defaults permissive so anything that never wires this (every
        # existing test, a plan step, a bare Toolbox in a script) keeps its
        # old behaviour; only debug.py's own repl() and one-shot path set it
        # from the real question text. See _permit() for why it exists.
        self.afe_mentioned = True

    def schemas(self):
        return schemas()

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
        except Refused as exc:
            return 'ERR %s' % exc
        except (RigError, ValueError, KeyError, TypeError) as exc:
            return render.error(exc)

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
        """OS-level, not another Modbus call - the dead link would fail that
        too, and repeating the same failed round trip is not a diagnosis.
        `Test-BoardPort`/`Find-BoardPort` in board_prompt/ComPort.ps1 do the
        same probe for -AutodetectComport before this process even starts;
        this is that logic reachable mid-session, once, so a cable that came
        loose does not need a restart to find out why.
        """
        import serial.tools.list_ports

        ports = [p.device for p in serial.tools.list_ports.comports()]
        configured = getattr(self.session, 'port', None)
        lines = []

        if not ports:
            lines.append('no COM ports at all - nothing is enumerating as a '
                         'serial device. Check the cable, and that the '
                         'ST-Link or serial adapter is plugged in.')
        elif configured is None:
            lines.append('no configured port to check (--no-board or '
                         '--simulated this run).')
        elif configured not in ports:
            lines.append("%s is not among the COM ports Windows currently "
                         "sees (%s) - the cable may be unplugged, or the "
                         "adapter's driver did not enumerate it."
                         % (configured, ', '.join(ports)))
        elif self._probe_board_port(configured):
            # The port exists and the board actually answers on it right
            # now - measured directly, not inferred from the port merely
            # being present. Calling this "not answering" regardless would
            # be wrong exactly when the link had already recovered, or when
            # the model reaches for this tool on a link nobody has actually
            # confirmed is down.
            lines.append('%s is present and the board answers on it right '
                         'now - the link is up.' % configured)
        else:
            lines.append('%s is present in the OS port list (%s), so the '
                         'adapter itself is there - the board is not '
                         'answering on it. Check it is powered, and that '
                         'nothing else has the port open.'
                         % (configured, ', '.join(ports)))

        if args.get('probe_other_ports') and ports:
            others = [p for p in ports if p != configured]
            found = next((p for p in others if self._probe_board_port(p)),
                        None)
            if found:
                lines.append('%s answered as this board - it may have moved '
                             'there. /reconnect after changing --port to it.'
                             % found)
            elif others:
                lines.append('tried %s, none answered as this board.'
                             % ', '.join(others))

        return '\n'.join(lines)

    def _probe_board_port(self, candidate):
        """True if this board answers on `candidate` - the same round trip
        Session.board makes on first use, just against a port that is not
        the configured one and closed again either way. The transport's own
        0.5s read timeout (coaxial/transport.py) is what keeps a silent port
        from hanging this - a handful of candidates costs a few seconds, not
        a stall.
        """
        from coaxial import connect, disconnect

        baud = getattr(self.session, 'baud', 115200)
        unit = getattr(self.session, 'unit', 1)
        try:
            boards = connect([(unit, baud, candidate)])
        except Exception:                                    # noqa: BLE001
            return False
        try:
            disconnect(boards)
        except Exception:                                     # noqa: BLE001
            pass
        return True

    def _board(self, name, args):
        # Coerced against the tool's own schema first: see
        # coaxial_mcp.tools.coerce for what a small model sends instead.
        return BOARD_HANDLERS[name](self.session, **board_coerce(name, args))
