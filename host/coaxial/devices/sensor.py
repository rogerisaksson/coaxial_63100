"""A part the board polls into shared memory, and how a host gets at it."""
import contextlib
import time
from abc import ABC, abstractmethod

from coaxial.comm.wire import Reader
from coaxial.devices.roles import Input


class PolledSensor(Input, ABC):

    """One part, polled by the board's own loop, read through its record."""

    #: What the board's poll loop can be doing, by the byte it reports -
    #: each part's own table, since each has its own states.
    LOOP_STATES: dict = {}

    def __repr__(self):
        return ('<%s - the board polls it; state() reads that record, '
                'hold() takes the bus>' % type(self).__name__)

    def settled(self, seconds=12.0, poll=0.3):
        """Wait for the poll loop to reach 'running', and say whether it did.
        """
        deadline = time.monotonic() + seconds
        while True:
            if self.state()['loop'] == 'running':
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(poll)

    def _loop_state(self, reply):
        """The loop state a hold or resume reports back, as a word."""
        return self.LOOP_STATES.get(Reader(reply).u8(), 'unknown')

    @abstractmethod
    def state(self) -> dict:
        """The poll loop's shared record: the reading, and what went wrong."""

    #: The key of `state()` that is the reading.
    READING: str = ''

    def read(self, count=None, timeout=None):
        """The loop's newest reading; None before it has one."""
        return self.state()[self.READING]

    @abstractmethod
    def peek(self, *args, **kwargs):
        """Take from the part directly. Needs the loop held."""

    @abstractmethod
    def poke(self, *args, **kwargs):
        """Put to the part directly. Needs the loop held."""

    @abstractmethod
    def hold(self):
        """Stop the poll loop so the part can be driven directly."""

    @abstractmethod
    def resume(self):
        """Start the poll loop again."""

    @contextlib.contextmanager
    def configuring(self):
        """Hold the loop for the block, and resume however the block ends."""
        self.hold()
        try:
            yield self
        finally:
            self.resume()
