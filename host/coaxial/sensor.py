"""A part the board polls into shared memory, and how a host gets at it.

Both SPI parts work the same way: the firmware's main loop reads the part and
writes a shared record, and a host reads that record rather than driving the
bus. Reading a cargo per request cost 45 ms each and caught one frame in eight,
which is why the loop exists.

That arrangement is what these five names are for. `state` takes the record the
loop wrote. Driving the bus directly needs the loop out of the way first, and a
host that forgets leaves two masters on one segment - so `hold`, `resume` and
the `configuring` block that pairs them are part of the surface, not an extra.

`Imu`, `Angle` and both stand-ins answer it. The stand-ins are the reason it is
declared rather than left as a convention: their methods used to be attached to
the class by a helper after the fact, so what a stand-in did and did not answer
was invisible until a view running -Simulated hit the gap. A name missing here
fails at construction.
"""
from abc import ABC, abstractmethod


class PolledSensor(ABC):

    """One part, polled by the board's own loop, read through its record."""

    def __repr__(self):
        return ('<%s - the board polls it; state() reads that record, '
                'hold() takes the bus>' % type(self).__name__)

    @abstractmethod
    def state(self):
        """The poll loop's shared record: the reading, and what went wrong.

        `updates` is monotonic, so a new reading is told from the same one
        read twice without guessing from the values.
        """

    @abstractmethod
    def read(self, *args, **kwargs):
        """Take from the part directly. Needs the loop held."""

    @abstractmethod
    def write(self, *args, **kwargs):
        """Put to the part directly. Needs the loop held."""

    @abstractmethod
    def hold(self):
        """Stop the poll loop so the part can be driven directly."""

    @abstractmethod
    def resume(self):
        """Start the poll loop again."""

    @abstractmethod
    def configuring(self):
        """Hold the loop for the block, and resume however the block ends.

        The pairing that matters: an exception between `hold` and `resume`
        otherwise leaves the loop stopped and the part silent afterwards.
        """
