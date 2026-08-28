"""The acquisition surface, named in one place.

Three things answer to it: `Daq`, the board's task behind `0x6E` device 6;
`SimulatedDaq`, the stand-in; and `Coaxial63100`, the rig that drives one with
the supply, the clock and the gates around it. All three configure a task, run
it and take records off it, so all three carry the same five names.

That is what this exists to settle. The rig used to prefix its own with the
attribute a caller happens to bind it to - `daq.daq_read()`, `daq.daq_write()`,
`daq.configure_daq()` - which repeats what the object already said and reads as
one name inside another.

It also pins the stand-in to the real one. `SimulatedDaq` is duck-typed against
`Daq`, and a name that drifted between them surfaced as an AttributeError on the
first call that reached for it - late, and only on the path that used it.
A missing method now fails at construction instead.

`write` is deliberately not here: only the rig puts levels out, because only the
rig owns the supply and the gates that a write goes through.
"""
from abc import ABC, abstractmethod


class Acquisition(ABC):

    """Configure a task, run it, take records off it."""

    @abstractmethod
    def configure(self, *args, **kwargs):
        """Set the task up, replacing whatever was there.

        One task, not many - one MCU, three converters, one timer - so this
        replaces rather than adds.
        """

    @abstractmethod
    def start(self):
        """Begin sampling into the board's buffer."""

    @abstractmethod
    def stop(self):
        """Stop sampling. What is already buffered stays readable."""

    @abstractmethod
    def read(self, *args, **kwargs):
        """One block of records, oldest first.

        Empty when nothing has been buffered yet, which is an answer and not
        a failure - call it again.
        """

    @abstractmethod
    def latest(self, *args, **kwargs):
        """The newest reading, taken from the live accumulator.

        The accumulator cannot overflow: a late reader gets a wider averaging
        window rather than a backlog, where `read` drops when the ring fills.
        """

    @abstractmethod
    def state(self):
        """How the task is doing: rate, what is buffered, what was lost."""
