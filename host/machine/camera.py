"""Her fake camera: her eye's frames shot where the world runs, her map kept in a process of its own.

A look's frame (`sight.Eyes.shot`) her left eye's colour and the parallax her stereo would match, from the
renderer's depth on the card; her map and its things (`sight.Keep`) off her loop's GIL, in `python -m
machine.camera` - a subprocess, not multiprocessing's: her page's worker is daemonic and may have no
children of those. A look's things reach her a look later, LOOK_S, whatever the wall's pace: her loop
waits on the camera only when it is slower than that.

    cam = Camera(world)                  # the renderer here, her map apart
    cam.look(world, t, head)             # LOOK_S apart: the last look's things taken, this one shot
    things = cam.told()                  # [(kind, centre, half, turn, id, clear)] as `sight.told`'s
    view = cam.view()                    # (thumbnail, hulls) of the last look told (`sight.Keep.view`)
    cam.close()
"""
import os
import pickle
import queue
import subprocess
import sys
import threading

from machine import sight
from machine.errors import MachineError


def main():
    """Her map apart: said ready - no things yet -, then each frame off stdin into it (`sight.Keep.took`), its
    things, its view and its looks said back on stdout, pickled; None, or stdin's end, ends it."""
    out, inp, sys.stdout = sys.stdout.buffer, sys.stdin.buffer, sys.stderr
    keep = sight.Keep()
    told = ([], None, 0)
    while True:
        pickle.dump(told, out, protocol=pickle.HIGHEST_PROTOCOL)
        out.flush()
        try:
            frame = pickle.load(inp)
        except EOFError:
            return
        if frame is None:
            return
        keep.took(*frame)
        told = (sight.told(keep), keep.view(), keep.map.looks)


class Camera:

    """Her fake camera on a world: its renderer here (`sight.Eyes`), her map apart (`main`, this tree's own);
    `looks` the looks her map has taken, as the last told says."""

    def __init__(self, world):
        self.eyes, self._told_q = sight.Eyes(world), queue.Queue()
        p = subprocess.Popen([sys.executable, '-m', 'machine.camera'], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             cwd=os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        assert p.stdin is not None and p.stdout is not None
        self._process, self._in, self._out = p, p.stdin, p.stdout
        threading.Thread(target=self._heard, daemon=True).start()
        self.looked_at, self.sent, self.things, self.seen, self.looks = None, True, [], None, 0

    def _heard(self):
        """Her map apart's tellings onto the queue as they come; None once it has ended."""
        try:
            while True:
                self._told_q.put(pickle.load(self._out))
        except (EOFError, OSError, pickle.UnpicklingError):
            self._told_q.put(None)

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
            raise MachineError("her camera's map apart ended, exit code %s" % self._process.wait())
        return told

    def _said(self, frame):
        try:
            pickle.dump(frame, self._in, protocol=pickle.HIGHEST_PROTOCOL)
            self._in.flush()
        except OSError as e:
            raise MachineError("her camera's map apart took no frame: %s" % e) from e

    def told(self):
        return self.things

    def view(self):
        return self.seen

    def close(self, wait=1.0):
        """Her map apart ended: a None sent, its stdin closed, `wait` s for it to end, then killed."""
        try:
            pickle.dump(None, self._in)
            self._in.close()
            self._process.wait(timeout=wait)
        except (OSError, subprocess.TimeoutExpired):
            self._process.kill()
            self._process.wait(timeout=wait)


if __name__ == '__main__':
    main()
