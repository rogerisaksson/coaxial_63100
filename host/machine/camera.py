"""Her fake camera: her eye's frames shot where the world runs, her map kept in a process of its own.

A look's frame (`sight.Eyes.shot`) her left eye's colour and the parallax her stereo would match, from the
renderer's depth on the card; her map and its things (`sight.Keep`) off her loop's GIL. A look's things
reach her a look later, LOOK_S, whatever the wall's pace: her loop waits on the camera only when it is
slower than that.

    cam = Camera(world)                  # the renderer here, her map apart
    cam.look(world, t, head)             # LOOK_S apart: the last look's things taken, this one shot
    things = cam.told()                  # [(kind, centre, half, turn, id, clear)] as `sight.told`'s
    view = cam.view()                    # (thumbnail, hulls) of the last look told (`sight.Keep.view`)
    cam.close()
"""
from machine import apart, sight
from machine.errors import MachineError


def keeping(conn):
    """Her map apart: said ready - no things yet -, then each frame into it (`sight.Keep.took`), its things,
    its view and its looks said back; None, or her loop's end of the pipe gone, ends it."""
    keep = sight.Keep()
    conn.send(([], None, 0))
    while True:
        try:
            frame = conn.recv()
        except EOFError:
            return
        if frame is None:
            return
        keep.took(*frame)
        conn.send((sight.told(keep), keep.view(), keep.map.looks))


class Camera:

    """Her fake camera on a world: its renderer here (`sight.Eyes`), her map apart (`keeping`); `looks` the
    looks her map has taken, as the last told says."""

    def __init__(self, world):
        self.eyes, (self._conn, child) = sight.Eyes(world), apart.CONTEXT.Pipe()
        self._process = apart.CONTEXT.Process(target=keeping, args=(child,), daemon=True)
        self._process.start()
        child.close()
        self.looked_at, self.sent, self.things, self.seen, self.looks = None, True, [], None, 0

    def look(self, world, now, head):
        """A look LOOK_S after the last at `now`: the last one's things taken - waited for while her map
        takes it -, this one shot from her `head` {pos, quat} as her estimate has it; whether one was."""
        if self.looked_at is not None and now - self.looked_at < sight.LOOK_S:
            return False
        if self.sent:
            self.things, self.seen, self.looks = self._told()
        seen, disp, kept = self.eyes.shot(world, head)
        self._conn.send((seen, disp.astype('float32'), kept))
        self.looked_at, self.sent = now, True
        return True

    def _told(self):
        """The look in flight's things, view and looks - her map apart's ready first -; raises if it ended
        without them: spawned from a script with no main guard it died before it took its end of the pipe,
        the end stayed open here and her loop's first frame waited on it for good."""
        while not self._conn.poll(apart.DRAIN_S):
            if not self._process.is_alive():
                raise MachineError("her camera's map apart ended, exit code %s" % self._process.exitcode)
        return self._conn.recv()

    def told(self):
        return self.things

    def view(self):
        return self.seen

    def close(self, wait=1.0):
        """Her map apart ended: the look in flight taken, a None sent, `wait` s for it to end, then ended."""
        if self._process.is_alive():
            if self.sent and self._conn.poll(wait):
                self._conn.recv()
            self._conn.send(None)
            self._process.join(timeout=wait)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(timeout=wait)
        self._conn.close()
