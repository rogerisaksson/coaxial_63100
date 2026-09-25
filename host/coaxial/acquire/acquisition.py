"""The acquisition surface, named in one place: a Stream of records."""
import time
from abc import ABC, abstractmethod

from coaxial.comm.hostclock import clock_of
from machine.roles import Stream


class Acquisition(Stream, ABC):

    """Configure a task, run it, take records off it."""

    def __repr__(self):
        return ('<%s - configure(), start(), acquire(), stop()>'
                % type(self).__name__)

    @abstractmethod
    def configure(self, *args, **kwargs):
        """Set the task up, replacing whatever was there."""

    @abstractmethod
    def start(self):
        """Begin sampling into the board's buffer."""

    @abstractmethod
    def stop(self):
        """Stop sampling. What is already buffered stays readable."""

    @abstractmethod
    def acquire(self, *args, **kwargs):
        """One block of records, oldest first."""

    @abstractmethod
    def latest(self, *args, **kwargs):
        """The newest reading, taken from the live accumulator."""

    @abstractmethod
    def state(self) -> dict:
        """How the task is doing: rate, what is buffered, what was lost."""

    @abstractmethod
    def shape(self, sections=(), decimate=1):
        """Load an anti-alias chain; none clears it."""

    def sweep_rate(self, channels, records=300, timeout=6.0):
        """Sweeps a second the poll loop manages over `channels`: a finite burst, timed."""
        self.shape()
        self.configure(channels, accumulate=1, digital=True, records=records, interval_us=0)
        clock = clock_of(self)
        began = clock.now()
        self.start()
        while clock.now() - began < timeout and not self.state()['done']:
            time.sleep(0.005)
        span = clock.now() - began
        state = self.state()
        self.stop()
        return (state['produced'] + state['dropped']) / max(span, 1e-6)
