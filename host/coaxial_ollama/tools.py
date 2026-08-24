"""The tool surface handed to the model: the MCP set, plus code, shell, report.

The nine board tools are imported from `coaxial_mcp.tools`, not re-declared.
That is deliberate and it is the main structural decision in this package: there
is one description of what a fixture can do to this board, one set of hand-tuned
compact renderers, and one place a new capability gets added. A second, drifting
copy of that surface written for Ollama would be the worst kind of duplication -
the kind that stays plausible while going out of date.

Four tools are added here, and they are what makes this a runner rather than a
chat window:

  run_command    an allowlisted process, for the things that are not the board:
                 a build, a flash, `python -m coaxial`.
  run_python     code against the live `board` object, in a namespace that
                 persists across the whole run.
  build_firmware host/tools/build_and_flash.py, fixed arguments only - the
                 narrow, always-available answer to "build and flash", so a
                 session does not need run_command's wider surface just for
                 that one job.
  report         how a step ends. The model reports a value and a unit; it is
                 never told the limit and never asked for a verdict.

Thirteen tools, against the nine that coaxial_mcp keeps to. The extra cost is
real and it is paid for one thing: a plan step can say "work out which channel
this is" instead of naming a function code.

Note what `report` is not: an assertion. It carries no pass/fail field, because
a field like that is a place for a model to put an opinion, and the runner would
then have to decide whether to believe it. plan.Limit decides, in Python.
"""
import os
import subprocess
import sys

sys.path.insert(0, __file__.rsplit('coaxial_ollama', 1)[0])

from coaxial.errors import RigError                       # noqa: E402
from coaxial_mcp import render                            # noqa: E402
from coaxial_mcp.tools import HANDLERS as BOARD_HANDLERS   # noqa: E402
from coaxial_mcp.tools import TOOLS as BOARD_TOOLS         # noqa: E402
from coaxial_mcp.tools import coerce as board_coerce       # noqa: E402

# host/tools/build_and_flash.py, resolved from this file's own location
# rather than the caller's cwd - dbg.py and the runner start from different
# directories, and this tool has to reach the same script either way.
_BUILD_AND_FLASH = os.path.join(__file__.rsplit('coaxial_ollama', 1)[0],
                                'tools', 'build_and_flash.py')

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
            return self._board(name, args)
        except Refused as exc:
            return 'ERR %s' % exc
        except (RigError, ValueError, KeyError, TypeError) as exc:
            return render.error(exc)

    # ---- policy ------------------------------------------------------------

    def _permit(self, name, args):
        if name not in BOARD_HANDLERS and name not in CODE_CALLS:
            raise Refused('unknown tool %r' % name)

        if name in CODE_CALLS and not self.allow_code:
            raise Refused('%s is disabled for this run (--read-only). Use the '
                          'board tools.' % name)

        test = WRITE_CALLS.get(name)
        if test and test(args) and not self.allow_writes:
            raise Refused('%s changes state and this run may only read. The '
                          'operator would have to pass --allow-writes.' % name)

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
        # Prefixed the same way every other failure in this file is, so the
        # code_error backstop in debug.py's Chat.ask() catches a failed build
        # or flash exactly like a declined --confirm call - a fact this loop
        # already has that the model does not get to override with its own
        # summary of what happened.
        return text if done.returncode == 0 else 'ERR %s' % text

    def _board(self, name, args):
        # Coerced against the tool's own schema first: see
        # coaxial_mcp.tools.coerce for what a small model sends instead.
        return BOARD_HANDLERS[name](self.session, **board_coerce(name, args))
