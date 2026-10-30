"""A process beside her loop's: a module of this tree served on its own, asks pickled down its stdin.

Her page's worker is daemonic and may have no children of multiprocessing's (`machine.apart`): what it runs
beside itself is a subprocess, `python -m module`, its stdout its answers' alone. Her planner, her camera's
map and her vision run so (`machine.planner`, `machine.camera`, `machine.vision`).

    aside = Aside('machine.vision', 'her vision')   # spawned at its first ask, or `start` sooner
    aside.send(ask); answer = aside.recv()          # an answer waited for
    aside.heard(answers)                            # or each onto a queue as it comes, None once it ended
    aside.close()
    serve(answer, ready)                            # the module's own loop: `ready` said first where given
"""
import os
import pickle
import subprocess
import sys
import threading

from machine import placed
from machine.errors import MachineError

#: This tree's host directory: where `python -m machine.*` finds its packages.
HOST = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
#: No answer said before the first ask (`serve`).
SILENT = object()


class Aside:

    """`module` on a process of its own (`serve`), spawned at its first `send` or at `start`; `name` its
    errors'."""

    def __init__(self, module, name):
        self.module, self.name, self.process, self.into, self.out = module, name, None, None, None

    def start(self):
        """Its process spawned, where it is not yet; itself."""
        if self.process is None:
            p = subprocess.Popen([sys.executable, '-m', self.module], stdin=subprocess.PIPE,
                                 stdout=subprocess.PIPE, cwd=HOST)
            assert p.stdin is not None and p.stdout is not None
            self.process, self.into, self.out = p, p.stdin, p.stdout
        return self

    def send(self, ask):
        """`ask` down its stdin, pickled whole first: one write, not the pickler's many."""
        self.start()
        try:
            self.into.write(pickle.dumps(ask, protocol=pickle.HIGHEST_PROTOCOL))
            self.into.flush()
        except OSError as e:
            raise MachineError('%s took no ask: %s' % (self.name, e)) from e

    def recv(self):
        """Its next answer, waited for; raises once it has ended."""
        try:
            return pickle.load(self.out)
        except (EOFError, OSError, pickle.UnpicklingError) as e:
            raise MachineError(self.ended()) from e

    def heard(self, answers):
        """Its answers onto the queue `answers` as they come, None once it has ended, by a thread of its own."""
        threading.Thread(target=_heard, args=(self.out, answers), daemon=True).start()

    def ended(self):
        """What its end said: its exit code, waited for."""
        return '%s ended, exit code %s' % (self.name, self.process.wait() if self.process is not None else None)

    def close(self, wait=1.0):
        """Its process ended: a None sent, its stdin closed, `wait` s for it to end, then killed."""
        if self.process is None:
            return
        try:
            pickle.dump(None, self.into)
            self.into.close()
            self.process.wait(timeout=wait)
        except (OSError, subprocess.TimeoutExpired):
            self.process.kill()
            self.process.wait(timeout=wait)
        self.process = None


def _heard(out, answers):
    """`out`'s answers onto `answers` as they come; None once it has ended. Not an Aside's method: a thread
    holding its owner kept its pipe open and its process alive, 105 of her planners beside 28 runs."""
    try:
        while True:
            answers.put(pickle.load(out))
    except (EOFError, OSError, pickle.UnpicklingError):
        answers.put(None)


def serve(answer, ready=SILENT):
    """This process a module served: `ready` said first where given, then each ask off stdin answered on stdout,
    pickled; None, or stdin's end, ends it. Its prints go to stderr: its stdout is its answers'. Placed a core
    apart from her others (`placed`)."""
    placed.placed()
    out, inp, sys.stdout = sys.stdout.buffer, sys.stdin.buffer, sys.stderr
    if ready is not SILENT:
        pickle.dump(ready, out, protocol=pickle.HIGHEST_PROTOCOL)
        out.flush()
    while True:
        try:
            ask = pickle.load(inp)
        except EOFError:
            return
        if ask is None:
            return
        pickle.dump(answer(ask), out, protocol=pickle.HIGHEST_PROTOCOL)
        out.flush()
