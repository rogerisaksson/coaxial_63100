"""The acquisition surface, named in one place."""
from abc import ABC, abstractmethod


class Acquisition(ABC):

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
    def state(self):
        """How the task is doing: rate, what is buffered, what was lost."""
