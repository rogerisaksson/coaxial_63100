"""Her fake camera: her eye's frames shot where the world runs, her map kept in a process of its own.

A look's frame (`sight.Eyes.shot`) her left eye's colour and the parallax her stereo would match, from the
renderer's depth on the card; her map and its things (`sight.Keep`) off her loop's GIL, in `python -m
machine.eyes.camera` - a process beside hers (`machine.process.aside`). A look's things reach her a look later, LOOK_S, whatever the wall's pace: her loop
waits on the camera only when it is slower than that.

    cam = Camera(world)                  # the renderer here, her map apart
    cam.look(world, t, head)             # LOOK_S apart: the last look's things taken, this one shot
    things = cam.told()                  # [(kind, centre, half, turn, id, clear)] as `sight.told`'s
    view = cam.view()                    # (thumbnail, hulls) of the last look told (`sight.Keep.view`)
    cam.close()
"""
import queue

from machine.process import aside
from machine.eyes import sight
from machine.errors import MachineError


def main():
    """Her map apart (`aside.serve`): said ready - no things yet -, then each frame into it (`sight.Keep.took`), its
    things, its view and its looks said back."""
    keep = sight.Keep()

    def took(frame):
        keep.took(*frame)
        return sight.told(keep), keep.view(), keep.map.looks
    aside.serve(took, ready=([], None, 0))


class Camera:

    """Her fake camera on a world: its renderer here (`sight.Eyes`), her map apart (`main`, this tree's own);
    `looks` the looks her map has taken, as the last told says."""

    def __init__(self, world):
        self.eyes, self._told_q = sight.Eyes(world), queue.Queue()
        self._aside = aside.Aside('machine.eyes.camera', "her camera's map apart").start()
        self._aside.heard(self._told_q)
        self.looked_at, self.sent, self.things, self.seen, self.looks = None, True, [], None, 0

    def look(self, world, now, head):
        """A look LOOK_S after the last at `now`: the last one's things taken - waited for while her map
        takes it -, this one shot from her `head` {pos, quat} as her estimate has it; whether one was."""
        if self.looked_at is not None and now - self.looked_at < sight.LOOK_S:
            return False
        if self.sent:
            self.things, self.seen, self.looks = self._told()
        seen, disp, kept = self.eyes.shot(world, head)
        self._said((seen, disp.astype('float32'), kept))
        self.looked_at, self.sent = now, True
        return True

    def _told(self):
        """The look in flight's things, view and looks - her map apart's ready first -; raises if it ended
        without them."""
        told = self._told_q.get()
        if told is None:
            raise MachineError(self._aside.ended())
        return told

    def _said(self, frame):
        self._aside.send(frame)

    def told(self):
        return self.things

    def view(self):
        return self.seen

    def close(self, wait=1.0):
        """Her map apart ended (`aside.Aside.close`)."""
        self._aside.close(wait)


if __name__ == '__main__':
    main()
