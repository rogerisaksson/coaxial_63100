"""Where model-authored commands and code actually run.

Two executors, and the difference between them matters:

  * `Shell` runs one allowlisted program as an argv list, never through a
    shell. That is what makes the allowlist mean anything: checking the first
    token is theatre if the rest of the string can start a second process.

  * `Scope` runs Python in one namespace that persists for the run, with the
    live `board` in it. Persistence is the point - read the channel map in one
    call, compute against it in the next.

Neither sandboxes in the security sense. Anything that can drive a motor
controller over a serial port can do damage with it; what bounds this is that
the plan says what the run is for, the transcript says what ran, and
`--confirm` puts a human in front of every side effect. On a board rated 63 V
and 100 A that is the honest arrangement: bounded by review, not by a sandbox.

A failure in either is a *result*, not an exception: the model has to see its
own traceback to correct itself.
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


# How much of a clipped process output is kept from the front. The rest comes
# from the end, because the end is where a process says what happened: the
# compiler's error, the linker's summary, the suite's own tally. A head-only
# cut of a long build keeps the banner and drops the answer.
HEAD_SHARE = 0.35


def clip_ends(text, limit=LIMIT, head_share=HEAD_SHARE):
    """Head and tail of a long output, with the middle cut out.

    `clip` keeps the first N characters, which is right for a document and
    wrong for a process. A build that fails prints its command line first and
    its diagnosis last; only one of those two earns a place in a context
    window, and it is not the first one. So both ends are kept and the
    repetitive middle is what goes.

    The notice in the seam says how much was dropped, in the same words `clip`
    uses, so a cut output cannot be read as a short one.
    """
    text = text if isinstance(text, str) else str(text)
    if len(text) <= limit:
        return text
    head = max(0, int(limit * head_share))
    tail = max(0, limit - head)
    cut = len(text) - head - tail
    return '%s\n... [%d characters cut from the middle]\n%s' % (
        text[:head], cut, text[len(text) - tail:] if tail else '')


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

    def available(self):
        """What this namespace holds, for a snippet that reached past it."""
        names = sorted(n for n in self.namespace if not n.startswith('__'))
        return ('This namespace has no third-party packages - no pandas, no '
                'numpy, by decision. It holds: %s. Means and standard '
                'deviations arrive from board.analog.burst() already computed '
                'on the board; statistics covers the rest.'
                % ', '.join(names))

    def run(self, code):
        """Execute `code`, return its output.

        The last statement is evaluated as an expression when it is one, so a
        snippet ending in `board.analog.ntc_temperature()` produces a value
        without the model having to remember to print it.
        """
        import ast

        self.runs += 1
        buffer = io.StringIO()
        source = str(code)
        try:
            tree = ast.parse(source)
        except SyntaxError as first:
            # A snippet that arrived with its newlines still escaped. Seen from
            # the prompt: a whole program on one line, with a literal backslash
            # and n where the line breaks belonged, which python reads as a
            # line continuation followed by something that is not a newline.
            # The model wrote a correct multi-line program; a layer between
            # here and there failed to unescape it.
            #
            # Repaired only after the code has already failed to compile, and
            # only when it holds no real newline - so a working one-liner with
            # an escape inside a string literal is never touched.
            repaired = source.replace('\\n', chr(10))
            if chr(10) in source or repaired == source:
                return 'SyntaxError: %s (line %s)' % (first.msg, first.lineno)
            try:
                tree = ast.parse(repaired)
            except SyntaxError:
                return 'SyntaxError: %s (line %s)' % (first.msg, first.lineno)
            source = repaired

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
            if isinstance(value, AttributeError) and 'board' in self.namespace:
                # The tool names and the library names are not the same words:
                # `analog_read` is a tool, `board.analog.read_all` is the
                # method behind it, and a model that has been calling the first
                # all session reaches for it here too. Seen from the prompt.
                board = self.namespace.get('board')
                parts = sorted(n for n in dir(board or ())
                               if not n.startswith('_')
                               and not callable(getattr(board, n, None)))
                if parts:
                    buffer.write('\nboard has: %s. The tool names are not the '
                                 'method names - analog_read is a tool, '
                                 'board.analog.read_all() is the method.'
                                 % ', '.join('board.' + p for p in parts))
            if isinstance(value, ImportError):
                # Say what is here, not only what is not. pandas and numpy
                # are absent by decision - see host/requirements.txt - and a
                # model that reached for one needs the alternative rather
                # than a refusal.
                buffer.write('\n' + self.available())

        out = buffer.getvalue().strip()
        return clip(out) if out else '(no output)'
