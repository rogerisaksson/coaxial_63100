"""Her wear's windows beside her loop: each read and filtered, its load laid LAND passes on.

A window's parts stood her loop's thread 3 to 13 ms each, twenty a window (`wear.wearing`). Here (`Here`, a
run's) they run one a pass as they did; apart (`Apart`, her page's worker's) a process of its own reads the
window off a ring both map, told only where it lies - her loop's share a row a pass. Either way `wear`'s own
arithmetic on the same rows, and her load laid (`wear.landed`) LAND passes after its window shut, not when
its last part ran: the same pass here and apart, so the same bits.

    worn = Here(dt) | Apart(dt).start()     # beside her loop, the process spawned warm
    worn.laid(w)                            # her wear as laid on her body (`wear.laid`), its window empty
    if worn.put(y, tau): worn.shut(at)      # a pass's edge array and drives' torques; a window whole at `at`
    told = worn.due(at, g)                  # a part a pass; at its landing her load laid: `wear.told`, else None
"""
import queue
import sys

from machine.body.figure import JOINTS
from machine.identify import sysid, wear
from machine.process import aside
from machine.senses import edge
from machine.errors import MachineError

#: Passes from a window shut to its load laid on her: its twenty-odd parts here, or its 0.12 s read apart,
#: done well within it flat out or paced.
LAND = 500
#: A ring's row: her edge array, then her drives' torques.
WIDTH = edge.SENSED + len(JOINTS)


def _steps(dt):
    """A window's passes at `dt`, its rows one more: the next window's first is its last."""
    return int(round(wear.WINDOW_S / dt))


class Ring:

    """A window's rows on a ring two windows long - a window apart is read long before its first row comes
    round again -, a pass's edge array and drives' torques a row; in shared memory where `shared` or named
    (`name`, mapped), its own else."""

    def __init__(self, steps, shared=False, name=None):
        import numpy as np
        self.steps, self.cap, self.block, self.name = steps, 2 * (steps + 1), None, None
        if shared or name is not None:
            from multiprocessing import shared_memory
            kept = {'track': False} if name is not None and sys.version_info >= (3, 13) else {}
            self.block = shared_memory.SharedMemory(name=name, create=name is None, size=self.cap * WIDTH * 8, **kept)
            self.name, self.made = self.block.name, name is None
            self.rows = np.ndarray((self.cap, WIDTH), dtype=np.float64, buffer=self.block.buf)
        else:
            self.rows = np.zeros((self.cap, WIDTH))
        self.start, self.n = 0, 0

    def put(self, y, tau):
        """A pass's row onto the window at hand: whether it is whole (its steps and one more)."""
        row = self.rows[(self.start + self.n) % self.cap]
        row[:edge.SENSED], row[edge.SENSED:] = y, tau
        self.n += 1
        return self.n > self.steps

    def window(self, start=None, n=None):
        """(her edge arrays, her drives' torques but the last) of the window at `start`, `n` rows - the one at
        hand where not given -, lists as her worker's were: each a row of a copy of its own."""
        import numpy as np
        start, n = (self.start, self.n) if start is None else (start, n)
        rows = self.rows[(start + np.arange(n)) % self.cap]
        return list(rows[:, :edge.SENSED]), list(rows[:-1, edge.SENSED:])

    def next(self):
        """The next window begun at this one's last row, as her worker's lists began it."""
        self.start, self.n = (self.start + self.n - 1) % self.cap, 1

    def close(self):
        """Its block let go, unlinked where it was made here."""
        if self.block is not None:
            self.rows = None
            self.block.close()
            if self.made:
                self.block.unlink()
            self.block = None


def _read(w, ys, taus, dt):
    """A window's parts as `wear.wearing` ran them, but the laying of its load: rows, filter, boxes; its
    return the load's corrections (`wear.deltas`)."""
    got = yield from wear.windowing(w, ys, taus, dt)
    wear.step(w, got)
    yield
    wear.boxed(w, ys, taus, dt)
    yield
    return wear.deltas(w)


class Here:

    """Her wear's windows in her loop's process: a window's parts one a pass, its load laid at its landing."""

    apart = False

    def __init__(self, dt, ring=None):
        self.dt, self.ring = dt, ring or Ring(_steps(dt))
        self.w = self.parts = self.got = self.due_at = None

    def laid(self, w):
        """Her wear as laid on her body (`wear.laid`) at an epoch: its window empty, none in flight."""
        self.w, self.parts, self.got, self.due_at = w, None, None, None
        self.ring.n = 0

    def put(self, y, tau):
        """A pass's edge array and drives' torques onto the window: whether it is whole."""
        return self.ring.put(y, tau)

    def emptied(self):
        """The window begun anew - she fell, a scene was laid -; one in flight lands as it would."""
        self.ring.n = 0

    def shut(self, at):
        """The window shut at pass `at`: its parts begun, its load due LAND passes on."""
        ys, taus = self.ring.window()
        self.parts, self.got, self.due_at = _read(self.w, ys, taus, self.dt), None, at + LAND
        self.ring.next()

    def due(self, at, g):
        """At pass `at` a part of the window in flight; at its landing - any part left run first - its load
        laid on her (`wear.landed`, `g`): her wear told, else None."""
        if self.due_at is None:
            return None
        while self.got is None:
            try:
                next(self.parts)
            except StopIteration as end:
                self.got = end.value
            if at < self.due_at:
                break
        if at < self.due_at:
            return None
        return self._landed(self.got, g)

    def _landed(self, moved, g):
        wear.landed(self.w, moved, g)
        self.parts = self.got = self.due_at = None
        return wear.told(self.w)

    def close(self, wait=1.0):
        self.ring.close()


class Apart(Here):

    """Her wear's windows on a process of their own (`main`), spawned warm at `start`: a window told where it
    lies on the ring both map, read and filtered there, its load laid here at its landing. Her wear's filter
    is its process's; a lay refused there - a body not hers - or its end said once, her windows are read here
    again (`Here`), on her wear as laid here."""

    apart = True

    def __init__(self, dt):
        super().__init__(dt, Ring(_steps(dt), shared=True))
        self.aside, self.answers, self.asked, self.flight = aside.Aside('machine.process.worn', 'her wear'), None, 0, None

    def start(self):
        """Its process spawned, its answers heard as they come; itself."""
        if self.aside is not None and self.answers is None:
            self.aside.start()
            self.answers = queue.Queue()
            self.aside.heard(self.answers)
        return self

    def laid(self, w):
        super().laid(w)
        if self.flight is not None:
            self._heard(block=True)
            self.flight = None
        if self.aside is not None:
            self._ask(('laid', self.ring.name, self.ring.steps, self.dt, w['nominal']))

    def shut(self, at):
        self._heard(block=False)
        if self.aside is None:
            return super().shut(at)
        self.flight, self.due_at = (self.ring.start, self.ring.n), at + LAND
        self._ask(('read',) + self.flight)
        self.ring.next()
        return None

    def due(self, at, g):
        if self.flight is None:
            return super().due(at, g)
        if at < self.due_at:
            return None
        got, (start, n) = self._heard(block=True), self.flight
        self.flight = None
        if got is None:
            ys, taus = self.ring.window(start, n)
            return self._landed(sysid.done(_read(self.w, ys, taus, self.dt)), g)
        moved, told = got
        wear.landed(self.w, moved, g)
        self.due_at = None
        return told

    def _ask(self, ask):
        try:
            self.start().aside.send(ask)
            self.asked += 1
        except (MachineError, OSError) as e:
            self._ended(e)

    def _heard(self, block):
        """Its answers taken as they come - a lay said, refused, a read, its end -; blocking, up to the read in
        flight's: its (moved, told), None where it was not read there."""
        got = None
        while self.aside is not None and self.asked and (block or not self.answers.empty()):
            answer = self.answers.get()
            if answer is None:
                self._ended('its process ended')
                return None
            self.asked -= 1
            if isinstance(answer, str):
                self._ended(answer)
            elif answer is not True:
                got = answer or None
        return got

    def _ended(self, why):
        """Her windows read here from now on: said once, its process let go."""
        if self.aside is not None:
            print('her wear apart ended, her windows read here: %s' % why, file=sys.stderr)
            self.aside.close(0.2)
            self.aside, self.asked = None, 0

    def close(self, wait=1.0):
        if self.aside is not None:
            self.aside.close(wait)
            self.aside = None
        super().close(wait)


def main():
    """Her wear apart (`aside.serve`): her body built and her wear laid as her loop's - refused where its
    nominal is not hers -, each window read off the ring both map: its load laid on its own body, its
    corrections and her wear told."""
    kept = {}

    def answered(ask):
        if ask[0] == 'laid':
            _kind, name, steps, dt, nominal = ask
            if kept.get('name') != name:
                kept['ring'], kept['name'] = Ring(steps, name=name), name
            from machine.stack import wbc
            w = wear.laid(wbc.Body())
            same = w['nominal'].shape == nominal.shape and w['nominal'].tobytes() == nominal.tobytes()
            kept['w'], kept['dt'] = (w if same else None), dt
            return True if same else 'laid on a body not hers: its nominal %s, hers %s' % (w['nominal'].shape, nominal.shape)
        w = kept.get('w')
        if w is None:
            return False
        ys, taus = kept['ring'].window(ask[1], ask[2])
        moved = sysid.done(_read(w, ys, taus, kept['dt']))
        wear.landed(w, moved)
        return moved, wear.told(w)
    aside.serve(answered)


if __name__ == '__main__':
    main()
