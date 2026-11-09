"""Her fake camera: her eye's frames shot and her map kept in a process of its own.

A look's frame (`sight.Eyes.stereo`) her two eyes drawn and matched on the card - her stereo's disparity, not
the renderer's depth (`sight.Eyes.shot`'s) -; her map and its things (`sight.Keep`); both off her loop, in `python -m
machine.eyes.camera` - a process beside hers (`machine.process.aside`) -, on her model as her world holds it (MuJoCo's
binary, sent as her camera starts: its renderer laid beside her laying) posed as her world's last step
left it (POSES) and laid as her scene lays it (PLACED): the same pixels, its renderer drawing the same looks in turn.
Shot in her loop, a look's two renders were 17 ms of it and its frame 615 KB down the pipe. A look's things reach her
a look later, LOOK_S, whatever the wall's pace: her loop waits on the camera only when it is slower than that.

    cam = Camera(world)                  # her eyes and her map apart, her model sent them as they boot
    cam.ready()                          # its renderer laid: waited for once, before her first pass (`worker.readied`)
    cam.look(world, t, head)             # LOOK_S apart: the last look's things taken, this one's world sent
    things = cam.told()                  # [(kind, centre, half, turn, id, clear)] as `sight.told`'s
    t, head, gz = cam.shown              # the time, her head and her gaze they were shot at, a look before
    view = cam.view()                    # (thumbnail, hulls) of the last look told (`sight.Keep.view`)
    cam.close()
"""
import importlib
import queue
import threading

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
#: What her map apart says once warm (`main`): no things, no view, no looks - hers till her first look's reach her.
READY = ([], None, 0)


class _Posed:

    """Her model as her world sent it (MuJoCo's binary) and a data of its own, posed as each look says; her eyes on it."""

    def __init__(self, binary):
        mujoco = importlib.import_module('mujoco')
        self.model = mujoco.MjModel.from_binary_path('her.mjb', {'her.mjb': binary})
        self.data = mujoco.MjData(self.model)
        self.eyes = sight.Eyes(self)

    def shot(self, poses, placed, head, gz=(0.0, 0.0), held=None):
        """A look's frame of her world posed and laid so: her two eyes drawn and matched on the card, laid by her
        servos' gaze `gz` as she holds her eyes at it (`held`, `sight.Eyes.stereo`) - her stereo's own disparity,
        the renderer's depth never hers (`truth`)."""
        for name, a in poses.items():
            getattr(self.data, name)[...] = a
        for name, a in placed.items():
            getattr(self.model, name)[...] = a
        return self.eyes.stereo(self, head, gz, held)


def main():
    """Her eyes and her map apart (`aside.serve`): her model first - its renderer laid, no look - and READY said, then
    each look's world - its poses and its laying - shot (`_Posed.shot`) and taken into her map (`sight.Keep.took`), its
    things, its view and its looks said back. Nothing is drawn or matched before her first look: the card's
    multisampling tells a frame's pixels by the frames drawn before it and the wall's time between them - 6 to 17 of a
    frame's a least bit off, the same scene; none without it -, so a frame drawn here moved her door errand's end, and
    her stereo's card started here, her second look 0.5 s sooner, her bed errand's in 4 runs of 7."""
    keep, kept = sight.Keep(), {}

    def took(ask):
        binary, poses, placed, head, gz, held = ask
        if binary is not None:
            kept['world'] = _Posed(binary)
        if poses is None:
            return READY
        keep.took(*kept['world'].shot(poses, placed, head, gz, held))
        return sight.told(keep), keep.view(), keep.map.looks
    aside.serve(took)


class Camera:

    """Her fake camera on a world: her eyes and her map apart (`main`, this tree's own), her world's poses sent each
    look; `looks` the looks her map has taken, as the last told says."""

    def __init__(self, world):
        self._told_q = queue.Queue()
        self._aside = aside.Aside('machine.eyes.camera', "her camera's map apart").start()
        self._aside.heard(self._told_q)
        # no look in flight, her map apart's READY owed: its things hers till her first look's reach her
        self.looked_at, self.sent, self.things, self.seen, self.looks = None, False, [], None, 0
        self.shot, self.shown, self._ready = None, None, True
        # her model sent by a thread of its own, its write the first down the pipe (`_said`): her map apart booting, its
        # 3 MB written here held her laying till it read them
        warm = (_binary(world.model), None, None, None, None, None)
        self._warming = threading.Thread(target=self._aside.send, args=(warm,), daemon=True)
        self._warming.start()

    def ready(self):
        """Her map apart booted and her model's renderer laid there (`main`), its READY taken: waited for once, before
        her first pass (`worker.readied`) or at her second look; itself."""
        if self._ready:
            self._told()
            self._ready = False
        return self

    def due(self, now):
        """Whether a look is due at `now`: her first, or LOOK_S after her last."""
        return self.looked_at is None or not now - self.looked_at < sight.LOOK_S

    def look(self, world, now, head, gz=(0.0, 0.0), held=None, waited=True):
        """A look LOOK_S after the last at `now`: the last one's things taken - waited for while her map
        takes it, its READY first after her first look, which waits on none: waited on there, her map apart's boot,
        its renderer's laying and its first shot held her first world second 3 s and more -, `shown` the (time, head,
        gaze) they were shot at, none before a look's; this one's world sent to be shot from her `head` {pos, quat}
        as her estimate has it at her gaze `gz`, her gimbals as she holds them (`held`, `gaze.held`); whether one
        was. Not `waited` - her eyes not hers (SIGHT 0: her registry on the world's boxes, her camera her page's view
        alone) -, nothing is waited for: what her map apart has said is taken, and a look still in flight leaves this
        one's world unsent, her looks due as ever (`_drained`)."""
        if not self.due(now):
            return False
        if not waited and self._drained():
            self.looked_at = now
            return True
        if self.sent:
            self.ready()
            self.things, self.seen, self.looks = self._told()
        self._said((None, {k: getattr(world.data, k) for k in POSES}, {k: getattr(world.model, k) for k in PLACED},
                    head, tuple(map(float, gz)), None if held is None else {s: tuple(map(float, a)) for s, a in held.items()}))
        self.shown, self.shot = self.shot, (now, head, tuple(map(float, gz)))
        self.looked_at, self.sent = now, True
        return True

    def _drained(self):
        """Whether a look is still in flight, what her map apart has said taken without a wait - its READY first,
        then a look's things, view and looks."""
        while True:
            try:
                told = self._told_q.get_nowait()
            except queue.Empty:
                return self.sent
            if told is None:
                raise MachineError(self._aside.ended())
            if self._ready:
                self._ready = False
                continue
            self.things, self.seen, self.looks = told
            self.sent = False

    def _told(self):
        """The look in flight's things, view and looks - her map apart's READY first -; raises if it ended
        without them."""
        told = self._told_q.get()
        if told is None:
            raise MachineError(self._aside.ended())
        return told

    def _said(self, frame):
        self._warmed()
        self._aside.send(frame)

    def _warmed(self):
        """Her warm ask written whole (`__init__`): a write after it its own, not amid it."""
        if self._warming is not None:
            self._warming.join()
            self._warming = None

    def told(self):
        return self.things

    def view(self):
        return self.seen

    def close(self, wait=1.0):
        """Her map apart ended (`aside.Aside.close`), her warm ask written first."""
        self._warmed()
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
