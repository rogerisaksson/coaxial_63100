"""Her fake camera: her eye's frames shot and her map kept in a process of its own.

A look's frame (`sight.Eyes.shot`) her left eye's colour and the parallax her stereo would match, from the
renderer's depth on the card; her map and its things (`sight.Keep`); both off her loop, in `python -m
machine.eyes.camera` - a process beside hers (`machine.process.aside`) -, on her model as her world holds it (MuJoCo's
binary, sent at the first look) posed as her world's last step left it (POSES) and laid as her scene lays it
(PLACED): the same pixels, its renderer drawing the same looks in turn. Shot in her loop, a look's two renders were
17 ms of it and its frame 615 KB down the pipe. A look's things reach her a look later, LOOK_S, whatever the
wall's pace: her loop waits on the camera only when it is slower than that.

    cam = Camera(world)                  # her eyes and her map apart
    cam.look(world, t, head)             # LOOK_S apart: the last look's things taken, this one's world sent
    things = cam.told()                  # [(kind, centre, half, turn, id, clear)] as `sight.told`'s
    view = cam.view()                    # (thumbnail, hulls) of the last look told (`sight.Keep.view`)
    cam.close()
"""
import importlib
import queue

from machine.process import aside
from machine.eyes import sight
from machine.errors import MachineError

#: What MuJoCo's scene reads of her world's data as her eyes draw it (`sight.Eyes`): her bodies', geoms', sites',
#: cameras', lights' and tendons' poses as her world's last step left them - copied into a data of their own, the
#: same pixels.
POSES = ('xpos', 'xquat', 'xmat', 'geom_xpos', 'geom_xmat', 'site_xpos', 'site_xmat', 'cam_xpos', 'cam_xmat',
         'light_xpos', 'light_xdir', 'ten_wrapadr', 'ten_wrapnum', 'wrap_obj', 'wrap_xpos', 'ten_length')
#: What her scene's laying writes into her model as she runs (`floor.lay`): its geoms' sizes and places.
PLACED = ('geom_size', 'geom_pos')


class _Posed:

    """Her model as her world sent it (MuJoCo's binary) and a data of its own, posed as each look says; her eyes on it."""

    def __init__(self, binary):
        mujoco = importlib.import_module('mujoco')
        self.model = mujoco.MjModel.from_binary_path('her.mjb', {'her.mjb': binary})
        self.data = mujoco.MjData(self.model)
        self.eyes = sight.Eyes(self)

    def shot(self, poses, placed, head):
        """A look's frame (`sight.Eyes.shot`) of her world posed and laid so."""
        for name, a in poses.items():
            getattr(self.data, name)[...] = a
        for name, a in placed.items():
            getattr(self.model, name)[...] = a
        return self.eyes.shot(self, head)


def main():
    """Her eyes and her map apart (`aside.serve`): said ready - no things yet -, then each look's world - her model at
    the first, its poses and its laying each - shot (`_Posed.shot`) and taken into her map (`sight.Keep.took`), its
    things, its view and its looks said back."""
    keep, kept = sight.Keep(), {}

    def took(ask):
        binary, poses, placed, head = ask
        if binary is not None:
            kept['world'] = _Posed(binary)
        keep.took(*kept['world'].shot(poses, placed, head))
        return sight.told(keep), keep.view(), keep.map.looks
    aside.serve(took, ready=([], None, 0))


class Camera:

    """Her fake camera on a world: her eyes and her map apart (`main`, this tree's own), her world's poses sent each
    look; `looks` the looks her map has taken, as the last told says."""

    def __init__(self, world):
        self._told_q = queue.Queue()
        self._aside = aside.Aside('machine.eyes.camera', "her camera's map apart").start()
        self._aside.heard(self._told_q)
        self.looked_at, self.sent, self.things, self.seen, self.looks = None, True, [], None, 0
        self._binary_sent = False

    def look(self, world, now, head):
        """A look LOOK_S after the last at `now`: the last one's things taken - waited for while her map
        takes it -, this one's world sent to be shot from her `head` {pos, quat} as her estimate has it -
        her model's binary at the first -; whether one was."""
        if self.looked_at is not None and now - self.looked_at < sight.LOOK_S:
            return False
        if self.sent:
            self.things, self.seen, self.looks = self._told()
        binary = None if self._binary_sent else _binary(world.model)
        self._said((binary, {k: getattr(world.data, k) for k in POSES}, {k: getattr(world.model, k) for k in PLACED},
                    head))
        self.looked_at, self.sent, self._binary_sent = now, True, True
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


def _binary(model):
    """`model` as MuJoCo's binary (mj_saveModel): her model whole, its textures too, as her world holds it now."""
    import numpy as np
    mujoco = importlib.import_module('mujoco')
    buf = np.empty(mujoco.mj_sizeModel(model), np.uint8)
    mujoco.mj_saveModel(model, None, buf)
    return buf.tobytes()


if __name__ == '__main__':
    main()
