"""Where model-authored commands and code run."""
import ast
import contextlib
import importlib.util
import io
import math
import os
import shlex
import statistics
import subprocess
import sys
import time
import traceback

import coaxial
from coaxial.devices import scaling

# Shell punctuation, checked as whole tokens.
_SHELLISM = {'|', '||', '&', '&&', ';', ';;', '>', '>>', '<', '2>', '`'}

# Enough to see what happened, little enough that one runaway command cannot
# push the plan out of the context window.
LIMIT = 4000


def clip(text, limit=LIMIT):
    text = text if isinstance(text, str) else str(text)
    if len(text) <= limit:
        return text
    return text[:limit] + '\n... [%d more characters cut]' % (len(text) - limit)


# How much of a clipped process output is kept from the front.
HEAD_SHARE = 0.35


def clip_ends(text, limit=LIMIT, head_share=HEAD_SHARE):
    """Head and tail of a long output, with the middle cut out."""
    text = text if isinstance(text, str) else str(text)
    if len(text) <= limit:
        return text
    head = max(0, int(limit * head_share))
    tail = max(0, limit - head)
    cut = len(text) - head - tail
    return '%s\n... [%d characters cut from the middle]\n%s' % (
        text[:head], cut, text[len(text) - tail:] if tail else '')


class Shell:
    """Allowlisted process launcher."""

    def __init__(self, allow=(), cwd=None, timeout=120.0):
        self.allow = {self._stem(name) for name in allow}
        self.cwd = cwd or os.getcwd()
        self.timeout = timeout
        self.history = []

    @staticmethod
    def _stem(name):
        base = os.path.basename(str(name).strip().strip('"').lower())
        return base[:-4] if base.endswith('.exe') else base

    def split(self, command):
        """Tokenise for Windows: posix=False keeps backslashes in paths intact."""
        tokens = [t.strip('"') for t in shlex.split(str(command), posix=False)]
        if not tokens:
            raise ValueError('empty command')
        return tokens

    def check(self, command):
        """Raise ValueError with the reason, or return the argv to run."""
        tokens = self.split(command)
        for token in tokens:
            if token in _SHELLISM or token[0] in '<>' or token.startswith('$('):
                raise ValueError(
                    'refused: %r uses %r. This runs as one process, not through '
                    'a shell, so pipes, redirection and chaining do nothing. '
                    'Run the parts as separate calls.' % (command, token))

        program = self._stem(tokens[0])
        if program not in self.allow:
            raise ValueError(
                'refused: %r is not on this run\'s allowlist (%s). The operator '
                'sets it with --allow.'
                % (program, ', '.join(sorted(self.allow)) or 'empty'))
        return tokens

    def run(self, command, timeout=None):
        argv = self.check(command)
        self.history.append(command)
        try:
            done = subprocess.run(argv, cwd=self.cwd, capture_output=True,
                                  text=True, encoding='utf-8',
                                  errors='replace',
                                  timeout=timeout or self.timeout)
        except subprocess.TimeoutExpired:
            return 'TIMEOUT after %.0fs: %s' % (timeout or self.timeout, command)
        except OSError as exc:
            return 'ERR cannot run %s: %s' % (argv[0], exc)

        parts = ['exit=%d' % done.returncode]
        if done.stdout.strip():
            parts.append(done.stdout.rstrip())
        if done.stderr.strip():
            parts.append('stderr: ' + done.stderr.rstrip())
        # clip_ends, not clip: this is a build, a flash or a test run, and
        # every one of those puts its verdict on the last line.
        return clip_ends('\n'.join(parts))


class Scope:
    """A persistent Python namespace with the bench already imported."""

    def __init__(self, board=None, extra=None):


        self.namespace = {
            '__name__': '__bench__',
            'board': board, 'coaxial': coaxial, 'scaling': scaling,
            'math': math, 'statistics': statistics, 'time': time,
        }
        self.namespace.update(extra or {})
        self.runs = 0

    def bind(self, board):
        """Attach the board once the session has opened it."""
        self.namespace['board'] = board

    def available(self):
        """What this namespace holds, for a snippet that reached past it."""
        names = sorted(n for n in self.namespace if not n.startswith('__'))
        # What is importable now, not what was decided once.
        extra = []
        for name in ('pandas', 'numpy'):
            if importlib.util.find_spec(name) is not None:
                extra.append(name)
        have = ('%s can be imported here. ' % ' and '.join(extra)
                if extra else 'No third-party packages are here. ')
        return ('%sThis namespace holds: %s. Means and standard '
                'deviations arrive from board.analog.burst() already '
                'computed on the board; statistics covers the rest.'
                % (have, ', '.join(names)))

    def _board_hint(self):
        """What `board` has, for a model that reached for a tool's name."""
        board = self.namespace.get('board')
        parts = sorted(n for n in dir(board or ())
                       if not n.startswith('_')
                       and not callable(getattr(board, n, None)))
        if not parts:
            return ''
        return ('\nboard has: %s. The tool names are not the method names - '
                'analog_read is a tool, board.analog.read() is the method.'
                % ', '.join('board.' + p for p in parts))

    def run(self, code):
        """Execute `code`, return its output."""

        self.runs += 1
        buffer = io.StringIO()
        source = str(code)
        try:
            tree = ast.parse(source)
        except SyntaxError as first:
            # A snippet that arrived with its newlines still escaped.
            repaired = source.replace('\\n', chr(10))
            if chr(10) in source or repaired == source:
                return 'SyntaxError: %s (line %s)' % (first.msg, first.lineno)
            try:
                tree = ast.parse(repaired)
            except SyntaxError:
                return 'SyntaxError: %s (line %s)' % (first.msg, first.lineno)
            source = repaired

        tail = None
        last = tree.body[-1] if tree.body else None
        if isinstance(last, ast.Expr):
            tree.body.pop()
            tail = ast.Expression(last.value)
        try:
            with contextlib.redirect_stdout(buffer), \
                    contextlib.redirect_stderr(buffer):
                if tree.body:
                    exec(compile(tree, '<bench>', 'exec'), self.namespace)
                value = (None if tail is None else
                         eval(compile(tail, '<bench>', 'eval'), self.namespace))
                if value is not None:
                    print(repr(value), file=buffer)
        except BaseException:
            # The sandbox's edge: whatever model code raised is its output,
            # SystemExit and KeyboardInterrupt included - a sys.exit() in a
            # snippet must not take the runner down mid-plan.
            etype, value, tb = sys.exc_info()
            buffer.write('\n' + ''.join(traceback.format_exception(
                etype, value, tb.tb_next if tb and tb.tb_next else tb,
                limit=4)).strip())
            if isinstance(value, AttributeError) and 'board' in self.namespace:
                buffer.write(self._board_hint())
            if isinstance(value, ImportError):
                # Say what is here, not only what is not.
                buffer.write('\n' + self.available())

        out = buffer.getvalue().strip()
        return clip(out) if out else '(no output)'
