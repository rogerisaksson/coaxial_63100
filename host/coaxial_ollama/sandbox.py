"""Where model-authored commands and code actually run.

Two executors, and the difference between them matters:

  * `Shell` runs one program from an allowlist, as an argv list, never through a
    shell. That is what makes the allowlist mean anything: checking the first
    token would be theatre if the rest of the string could start a second
    process, and with no shell in the path it cannot.

  * `Scope` runs Python in one namespace that persists for the whole run, with
    the live `board` already in it. Persistence is the point: the model reads
    the channel map in one call and computes against it in the next, the same
    way a person at the bench builds up a session.

Neither executor sandboxes in the security sense, and this file should not
pretend otherwise. Anything that can drive a motor controller over a serial port
can do damage with it; the protection here is that the plan says what the run is
for, the transcript says what was actually run, and `--confirm` puts a human in
front of every side effect. On a board rated 63 V and 100 A that is the honest
arrangement: bounded by review, not by a sandbox.

A failure inside either executor is a *result*, not an exception. The model has
to see its own traceback to correct itself, so both return text and let the
runner decide what it means.
"""
import contextlib
import io
import os
import shlex
import subprocess
import sys
import traceback

# Shell punctuation, checked as whole tokens. Nothing here is dangerous once the
# command runs as an argv list - `|` would simply arrive as a literal argument -
# so the refusal is not a security boundary; it is telling the model that its
# command will not do what it thinks. The token-level check is what lets
# `python -c "a; b"` through: the semicolon is inside an argument, not between
# two of them.
_SHELLISM = {'|', '||', '&', '&&', ';', ';;', '>', '>>', '<', '2>', '`'}

# Enough to see what happened, little enough that one runaway command cannot
# push the plan out of the context window.
LIMIT = 4000


def clip(text, limit=LIMIT):
    text = text if isinstance(text, str) else str(text)
    if len(text) <= limit:
        return text
    return text[:limit] + '\n... [%d more characters cut]' % (len(text) - limit)


class Shell:
    """Allowlisted process launcher.

    The allowlist holds program names, not command lines: `python` allows any
    python invocation, because a runner that has to enumerate arguments in
    advance is a runner nobody can write a plan for. The trust boundary is the
    program, and it is chosen by whoever starts the run.
    """

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
                                  text=True, timeout=timeout or self.timeout)
        except subprocess.TimeoutExpired:
            return 'TIMEOUT after %.0fs: %s' % (timeout or self.timeout, command)
        except OSError as exc:
            return 'ERR cannot run %s: %s' % (argv[0], exc)

        parts = ['exit=%d' % done.returncode]
        if done.stdout.strip():
            parts.append(done.stdout.rstrip())
        if done.stderr.strip():
            parts.append('stderr: ' + done.stderr.rstrip())
        return clip('\n'.join(parts))


class Scope:
    """A persistent Python namespace with the bench already imported.

    There is no timeout. A call into the board blocks for as long as the serial
    transport allows and no longer, and interrupting arbitrary model code
    mid-transaction would leave the link in a state the next step inherits -
    worse than waiting.
    """

    def __init__(self, board=None, extra=None):
        import math
        import statistics
        import time

        import coaxial
        from coaxial import scaling

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

    def run(self, code):
        """Execute `code`, return its output.

        The last statement is evaluated as an expression when it is one, so a
        snippet ending in `board.analog.ntc_temperature()` produces a value
        without the model having to remember to print it.
        """
        import ast

        self.runs += 1
        buffer = io.StringIO()
        try:
            tree = ast.parse(str(code))
        except SyntaxError as exc:
            return 'SyntaxError: %s (line %s)' % (exc.msg, exc.lineno)

        tail = None
        if tree.body and isinstance(tree.body[-1], ast.Expr):
            tail = ast.Expression(tree.body.pop().value)

        try:
            with contextlib.redirect_stdout(buffer), \
                    contextlib.redirect_stderr(buffer):
                if tree.body:
                    exec(compile(tree, '<bench>', 'exec'), self.namespace)
                if tail is not None:
                    value = eval(compile(tail, '<bench>', 'eval'),
                                 self.namespace)
                    if value is not None:
                        print(repr(value), file=buffer)
        except BaseException:                       # noqa: BLE001 - see docstring
            # Including KeyboardInterrupt and SystemExit: model code calling
            # sys.exit() must not take the runner down mid-plan.
            #
            # tb_next drops this frame. What is left is the model's own code and
            # the library under it, which is what it has to read to fix the
            # call; a frame pointing into sandbox.py would only suggest the
            # runner was at fault.
            etype, value, tb = sys.exc_info()
            buffer.write('\n' + ''.join(traceback.format_exception(
                etype, value, tb.tb_next if tb and tb.tb_next else tb,
                limit=4)).strip())

        out = buffer.getvalue().strip()
        return clip(out) if out else '(no output)'
