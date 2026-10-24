"""Her loop and a tuner apart with no lock between them: rings of her samples, a double bank of tables.

    link = rcu.Link.made(((width, slots), ..), cols, rows)  # the tuner's; link.name to hand on
    link = rcu.Link.found(name)                    # her loop's, by that name
    link.tap(sample, ring)    # her loop: a sample onto a ring; full, dropped and counted
    got = link.drained(ring)  # the tuner: the ring's samples since its last, (n, width)
    link.published(table)     # the tuner: a table into the bank her loop does not hold, then its sequence
    seq, table = link.taken() # her loop: the table published since her last, copied, else None
    link.said('laid', seq)    # one side's word: her loop's laid, the tuner's halt

One block of shared memory in 64-byte lines: the header's words (HEADER) and a line a ring (RING),
each word written by one side alone; each ring's slots; two banks a table each - a column's
rows contiguous, each column on its own lines (SoA). The tuner writes the bank her loop does not hold,
then BANK, then SEQ; her loop copies the bank SEQ names, keeps it only where SEQ held through the copy
(a seqlock), then writes TOOK: the tuner writes no bank before TOOK says her loop has the last one, so
neither waits on the other. On x86-64 an aligned 8-byte store is atomic and stores are seen in order.
Her loop's side is C (wbc/inc/wbc_rcu.h), the same words; the block locked in memory where the
system grants it (`link.locked`).
"""
import ctypes
import sys
from multiprocessing import shared_memory

from machine.errors import MachineError

#: A line, bytes; a word, bytes; words a line.
LINE, WORD = 64, 8
PER_LINE = LINE // WORD
#: The header's words: the last taken and laid (her loop's); the last publication, its bank and rows,
#: a halt said, the banks' shape, the rings (the tuner's).
HEADER = ('took', 'laid', 'seq', 'bank', 'rows', 'halt', 'cols', 'capacity', 'rings')
#: A ring's line: its count written and its drops (her loop's), its count read, its shape (the tuner's),
#: where its slots begin, words.
RING = ('head', 'dropped', 'tail', 'width', 'slots', 'at')
H = {name: i for i, name in enumerate(HEADER)}
R = {name: i for i, name in enumerate(RING)}


def _lines(words):
    """`words` rounded up to whole lines."""
    return -(-words // PER_LINE) * PER_LINE


def _core():
    """wbc_rcu.c, her loop's side of the block, through the core's harness."""
    from tools.cores import wbc
    return wbc.library()


def _virtual_lock(addr, size):
    k = ctypes.windll.kernel32
    k.GetCurrentProcess.restype = ctypes.c_void_p
    k.GetProcessWorkingSetSize.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_size_t), ctypes.POINTER(ctypes.c_size_t)]
    k.SetProcessWorkingSetSize.argtypes = [ctypes.c_void_p, ctypes.c_size_t, ctypes.c_size_t]
    k.VirtualLock.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
    proc, lo, hi = k.GetCurrentProcess(), ctypes.c_size_t(), ctypes.c_size_t()
    k.GetProcessWorkingSetSize(proc, ctypes.byref(lo), ctypes.byref(hi))
    k.SetProcessWorkingSetSize(proc, lo.value + size, hi.value + size)
    return bool(k.VirtualLock(addr, size))


def _mlock(addr, size):
    libc = ctypes.CDLL(None)
    libc.mlock.argtypes = [ctypes.c_void_p, ctypes.c_size_t]
    return libc.mlock(addr, size) == 0


#: How a block's pages are kept in memory, by platform: past a working set grown by it, or mlock -
#: bounded by RLIMIT_MEMLOCK.
LOCKS = {'win32': _virtual_lock}


class Link:

    """One block of shared memory between her loop and a tuner: its header, rings and banks as views."""

    def __init__(self, shm, owner):
        import numpy as np
        self.shm, self.owner, self.name = shm, owner, shm.name
        top = _lines(len(HEADER))
        rings = int(np.ndarray((1,), np.int64, shm.buf, H['rings'] * WORD)[0])
        self.h = np.ndarray((top + PER_LINE * rings,), np.int64, shm.buf, 0)
        self.r = self.h[top:].reshape(rings, PER_LINE)
        self.rings = [np.ndarray((int(r[R['slots']]), _lines(int(r[R['width']]))), np.float64, shm.buf,
                                 int(r[R['at']]) * WORD) for r in self.r]
        cols, capacity = int(self.h[H['cols']]), int(self.h[H['capacity']])
        at = int(self.r[-1][R['at']]) + self.rings[-1].size
        self.banks = np.ndarray((2, cols, _lines(capacity)), np.float64, shm.buf, at * WORD)
        self.capacity = capacity
        self.base = self.h.ctypes.data
        self.locked = LOCKS.get(sys.platform, _mlock)(self.base, shm.size)
        self.table, self.rows = np.empty((cols, capacity)), ctypes.c_longlong()

    @classmethod
    def made(cls, rings, cols, capacity):
        """The tuner's link: a ring of `slots` samples of `width` each of `rings` ((width, slots), ..),
        two banks of `cols` columns of `capacity` rows, every section on whole lines."""
        import numpy as np
        top = _lines(len(HEADER))
        at, starts = top + PER_LINE * len(rings), []
        for width, slots in rings:
            starts.append(at)
            at += slots * _lines(width)
        shm = shared_memory.SharedMemory(create=True, size=(at + 2 * cols * _lines(capacity)) * WORD)
        h = np.ndarray((top + PER_LINE * len(rings),), np.int64, shm.buf, 0)
        h[:] = 0
        h[[H['cols'], H['capacity'], H['rings']]] = (cols, capacity, len(rings))
        for k, ((width, slots), start) in enumerate(zip(rings, starts)):
            h[top + PER_LINE * k + np.array([R['width'], R['slots'], R['at']])] = (width, slots, start)
        del h
        return cls(shm, True)

    @classmethod
    def found(cls, name):
        """Her loop's link: the block the tuner made, by its name."""
        return cls(shared_memory.SharedMemory(name=name), False)

    def tap(self, sample, ring=0):
        """Her loop: `sample` onto `ring`, then its count; a full ring drops it and counts that."""
        import numpy as np
        sample = np.ascontiguousarray(sample, np.float64)
        return bool(_core().wbh_rcu_tap(self.base, ring, sample.ctypes.data, len(sample)))

    def drained(self, ring=0):
        """The tuner: every sample of `ring` since its last drain, oldest first, (n, width)."""
        import numpy as np
        r, slots = self.r[ring], self.rings[ring].shape[0]
        head, tail = int(r[R['head']]), int(r[R['tail']])
        got = self.rings[ring][(tail + np.arange(head - tail)) % slots, :int(r[R['width']])].copy()
        r[R['tail']] = head
        return got

    def published(self, table):
        """The tuner: `table` (cols, n) into the bank her loop does not hold, then that bank named,
        then the sequence on; False, nothing written, while her loop has not taken the last."""
        import numpy as np
        if self.h[H['took']] != self.h[H['seq']]:
            return False
        table = np.asarray(table, float)
        if table.shape[0] != self.banks.shape[1] or table.shape[1] > self.capacity:
            raise MachineError('a table of %s; the banks hold %d columns of %d rows' % (
                table.shape, self.banks.shape[1], self.capacity))
        bank = (int(self.h[H['seq']]) + 1) % 2
        self.banks[bank, :, :table.shape[1]] = table
        self.h[H['rows']] = table.shape[1]
        self.h[H['bank']] = bank
        self.h[H['seq']] += 1
        return True

    def taken(self):
        """Her loop: (its sequence, the table the tuner published since her last take, copied) - kept
        where the sequence held through the copy -, its taking said; else None."""
        seq = _core().wbh_rcu_taken(self.base, self.table.ctypes.data, ctypes.byref(self.rows))
        return (int(seq), self.table[:, :self.rows.value].copy()) if seq else None

    def said(self, word, value):
        """One side's header word written: her loop's `laid`, the tuner's `halt`."""
        self.h[H[word]] = value

    def word(self, name, ring=None):
        """A header word read, or a ring's."""
        return int(self.h[H[name]] if ring is None else self.r[ring][R[name]])

    def closed(self):
        """The block let go, its views first; its maker unlinks it."""
        del self.h, self.r, self.rings, self.banks, self.table
        self.shm.close()
        if self.owner:
            self.shm.unlink()
