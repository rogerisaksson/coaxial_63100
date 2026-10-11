"""The stand-ins' slots: an index into one of world/src's arrays, a board's part its owner."""
import gc
import weakref

from coaxial.errors import RigError


class Slots:

    """The free indices of one array, the lowest first, each back when its owner is gone."""

    def __init__(self, count):
        self.count = count
        self.free = list(range(count))

    def take(self, owner):
        """A free slot, `owner`'s for its life - after a collection, where cycles hold the
        last of them."""
        if not self.free:
            gc.collect()
        if not self.free:
            raise RigError('every one of the %d stand-in boards a process holds is taken'
                           % self.count)
        slot = self.free.pop(0)
        weakref.finalize(owner, self.free.append, slot)
        return slot
