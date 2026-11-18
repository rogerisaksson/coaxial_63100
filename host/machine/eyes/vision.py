"""The robot's looks beside its law: each registry pass in a process apart, joined before it is read.

A look at a refresh (`sprung.passed`) is asked before the robot's law's pass and its things joined just before the
robot's wayfinder reads them (`motion.control`), its sensing, latent and legs run meanwhile: 0.99 ms of the
robot's law beside a look's 0.94 ms. The robot's registry, its looks and their arithmetic are the same
either way, so are their bits: in its process (`Here`, a run's), or on its own (`Apart`, the robot's page's
worker's, `machine.process.aside`) - its registry kept there, the laid one sent it at each epoch and read
here no more (`Kept`).

    look(g); ...; joined(g)                    # a look asked at g['t'], the robot's things joined once after it
    looked(g)                                  # both at once: a look between the robot's passes (`sprung.looked`)
    things = g['vision'].boxes(g, least)       # its registry's boxes at `least` salience, between looks
"""
import math

from machine import libm
from machine.process import aside, placed
from machine.senses import inekf
from machine.eyes import gaze, registry
from machine.law import composer
from machine.errors import MachineError

#: A look's things: the world's boxes as the robot's eyes' stand-in, or its eyes' map's (`g['keep']`,
#: `physics.SIGHT`).
LOOKS = (lambda g: g['world'].props(), lambda g: g['keep'].told())
#: What the robot's vision apart says for a look whose boxes are the last one's to the bit (`main`): its things as
#: they were; and before the rows of a look whose things are the last one's but those rows: the rest as they were.
SAME, ROWS = 'its things as the look before', 'its things as the look before but these rows'


# the robot's step's clamps told its drives as zeros it rests limp (`se2.limp`, `motion.drives`): nothing of
# its own to steer, its eyes, its camera and its map rest till a clamp rises (working_envelope_scoper.lisp's
# recumbent hibernation, `composer`)
@composer.capability(lambda g: 'cmd' not in g or not g['drives'][1]['limit'].any() or bool(g['cmd']['limit'].any()),
                     lambda _g: None)
def look(g):
    """A look asked at `g['t']` (LOOKS by the robot's eyes): its camera's shot first, its head as the
    robot's law last sensed it, the time since the robot's last look; with its camera its eyes turned
    by its gaze (`gaze`) - its shot from its head and its gaze, its stereo
    laying its pictures by it. The robot's eyes' things its own once each as they reach it, a
    look later: at the time they were shot, its registry's frustum the head and gaze that shot them
    (`camera.Camera.shown`) - a look's things laid again at each refresh counted its evidence 60 times and its move
    from the look before over a refresh's 8 ms: a thing 'moving' 26 m/s, its wake 13 m, the robot's map barred
    whole."""
    head, t = {'pos': g['s']['head']['pos'].copy(), 'quat': g['s']['head']['quat'].copy()}, g['t']
    if g['keep'] is not None:
        gz = gaze.moved(g)
        gaze.swung(g['world'], gz)
        held = gaze.held(g, gz)
        # a look with the robot's eyes standing on what they look at alone: none in a saccade's blur; the
        # robot's cameras placed on its gimbals and its head read where one is due alone -
        # each law pass they were 140 us of it, the robot's looks LOOK_S apart
        took = gaze.settled(g) and g['keep'].due(g['t'])
        if took:
            gaze.placed(g['world'])
            unfloor, unfloored = g.pop('unfloor', False), g.pop('_unflooring', False)
            g['keep'].look(g['world'], g['t'], inekf.head(g['inekf'], g['body']), gz, held, waited=bool(g['eyes']),
                           unfloor=unfloor)
            if unfloor:
                g['_unflooring'] = True
            if unfloored:
                g['_relocalizing'] = True
            gaze.asked(g, gz)
        if g['eyes'] and (not took or g['keep'].shown is None):
            return
        if g['eyes']:
            t, shot, at = g['keep'].shown
            head = gaze.gazed(shot, at)
            fix = g['keep'].fixed()
            if fix is not None:
                _floored(g, t, fix)
    g['vision'].start(g, LOOKS[g['eyes']](g), head, t - g.get('things_at', t), not g['eyes'], t)
    g['things_at'] = t


def _floored(g, t, fix):
    """The robot's estimate told where its look at `t` lay on its floor (`sight.Keep.took`:
    `fix`, its information, its eye, its grey's fit; `inekf.seen`) - laid by the robot's estimate as it rose,
    the robot's looks raised the bed's top 0.44 -> 0.512 m, its head 0.09-0.41 m high at its sit -;
    what it told the robot and whether it took it kept for the robot's introspection (`g['seen']`).
    Looks the robot's floor told something it could not take in a row longer than chance at the robot's
    own share of them (Laplace's, of every look that told) gives at its gate's confidence
    (`_confidence`) - its light or its floor changed past what a look's gain and offset carry,
    or it was thrown -: its map and its floor laid anew from its next look
    (`g['unfloor']`), its errand to find its thing and its dock again from what the robot sees then
    (`g['relocalize']`, `machine.way.errand.step_errand`). Two in a row let the robot go 4-6 times an errand,
    steady: a quarter of its looks were gated, its registration's errors heavier-tailed than
    it said."""
    from machine.eyes import voxels
    taken = inekf.seen(g['inekf'], *fix[:3])
    g['seen'] = (t, fix, taken)
    if not fix[1].any():
        return
    told, refused, run = g.get('floor_runs', (0, 0, 0))
    told, refused, run = told + 1, refused + (not taken), (run + 1) * (not taken)
    share = (refused + 1.0) / (told + 2.0)
    g['floor_runs'] = (told, refused, run)
    if run >= max(voxels.TOLD, libm.log(1.0 - _confidence()) / libm.log(share)):
        g['floor_runs'], g['unfloor'] = (told, refused, 0), True


def _confidence():
    """The robot's floor's gate's confidence: the chi square with 3 degrees of freedom - a look's turn and place
    along its floor - below `inekf.GATE` (the text's visual gate), in closed form."""
    x = inekf.GATE
    return math.erf(math.sqrt(0.5 * x)) - math.sqrt(2.0 * x / math.pi) * libm.exp(-0.5 * x)


def joined(g):
    """The robot's things after the look asked last (`look`), once; none asked, nothing."""
    if g['vision'].pending:
        g['things'] = g['vision'].join()
        if g.pop('_relocalizing', False):
            g['relocalize'] = True


def looked(g):
    """A look (`look`) and the robot's things after it (`joined`) at once."""
    look(g)
    joined(g)


class Here:

    """The robot's looks in its process, each passed as it is asked."""

    apart = False

    def __init__(self):
        self.pending, self._things, self._boxed = False, None, {}

    def start(self, g, looks, head, dt, everywhere, at=None):
        """The robot's registry (`g['registry']`) `dt` s on and the look `looks` into it from the robot's
        `head`, taken `at` s (`g['t']` without)."""
        registry.passed(g['registry'], head, dt)
        registry.seen(g['registry'], looks, head, g['t'] if at is None else at, everywhere=everywhere)
        self._things, self.pending, self._boxed = registry.boxes(g['registry']), True, {}

    def join(self):
        self.pending = False
        return self._things

    def boxes(self, g, least):
        """The robot's registry's boxes at `least` salience (`registry.boxes`), a look in flight joined first, cached a
        look."""
        joined(g)
        if least not in self._boxed:
            self._boxed[least] = registry.boxes(g['registry'], least)
        return self._boxed[least]

    def ready(self):
        """Nothing to wait for: the robot's looks are its own here."""
        return self

    def close(self, wait=1.0):
        pass


class Apart:

    """The robot's looks on a process of their own (`main`), spawned warm at `start()`: each asked, its things
    joined."""

    apart = True

    def __init__(self):
        self._aside, self.pending, self._looks = aside.Aside('machine.eyes.vision', 'its vision'), False, []
        self._things, self._boxed, self._owed = None, {}, False

    def start(self, g=None, looks=(), head=None, dt=0.0, everywhere=True, at=None):
        """`Here.start` asked of its process - the look in flight before it dropped -; `g` None, its process spawned,
        its ready owed (`ready`)."""
        self._owed = self._owed or self._aside.process is None
        self._aside.start()
        if g is None:
            return
        if self.pending:
            self.join()
        assert head is not None, 'a look is asked from the robot\'s head'
        # the robot's head as floats, its arrays laid anew there (`main`): two arrays pickled were most of an
        # ask's 37 us
        self._aside.send(('look', _laid(g), self._changed(looks), len(looks), (tuple(map(float, head['pos'])),
                          tuple(map(float, head['quat']))), g['t'] if at is None else at, dt, everywhere))
        self.pending, self._boxed = True, {}

    def _changed(self, looks):
        """{place: look} of `looks` not as last sent - a part not the same object, its tuples and floats the same
        values where it is: 63 of the robot's room's 64 are, its door's not -; all of them where their count
        changed. A look the same object is passed at once (the robot's room's still boxes, `boxed.props`): part by
        part, 50 us a look."""
        was, self._looks = self._looks, looks
        if len(was) != len(looks):
            return dict(enumerate(looks))
        return {k: p for k, (p, q) in enumerate(zip(looks, was)) if p is not q and (
            len(p) != len(q) or p[0] != q[0] or any(a is not b for a, b in zip(p[1:], q[1:])))}

    def join(self):
        """The look in flight's things, waited for: those it last laid where its boxes are theirs to the bit (SAME) -
        seated, every look's -, those with the rows told anew where the rest are (ROWS: a chair or a bed creeping
        under the robot, every look's - built whole, 130-150 us of numpy floats in tuples a look)."""
        self.pending = False
        got = self.ready()._aside.recv()
        if isinstance(got, str):
            return self._things
        self._things = registry.patched(self._things, *got[1:]) if got[0] == ROWS else registry.things(*got)
        return self._things

    def ready(self):
        """The process booted - its C warmed (`registry.warm`, `main`) -, its ready taken: waited for once, before the
        robot's first pass (`worker.readied`) or its first answer; itself. Asked at the robot's first pass,
        the robot's vision's boot held it 0.66 s."""
        if self._owed:
            self._owed = False
            self._aside.recv()
        return self

    def boxes(self, g, least):
        """The robot's registry's boxes at `least` salience (`registry.boxed` there), a look in flight joined first:
        asked of its process once a look - its registry moves at a look alone (`start`, its 'look'),
        `boxed` reads it -, the same bits after. Asked each pass, the robot's errand's look for a thing
        (`words._glimpsed`) spent 427 us a pass on the round trip: 23 % of its door errand's wall time, its eyes its
        own."""
        joined(g)
        if least not in self._boxed:
            self._aside.send(('boxes', _laid(g), least))
            self._boxed[least] = registry.things(*self.ready()._aside.recv())
        return self._boxed[least]

    def close(self, wait=1.0):
        self._aside.close(wait)


class Kept(dict):

    """The robot's registry once sent its vision apart (`Apart`): its slots are kept there, read here
    it raises."""

    def __missing__(self, key):
        raise MachineError('the robot\'s registry is its vision\'s apart (`vision.Apart`): its %r is read there' % (key,))


def _laid(g):
    """The robot's registry as laid, where its vision apart has it not yet - `Kept` here once sent -;
    else None."""
    if isinstance(g['registry'], Kept):
        return None
    slots, g['registry'] = g['registry'], Kept()
    return slots


def main():
    """The robot's vision apart (`aside.serve`): its registry as laid, each look - its changes on the last -
    passed into it, its boxes kept and restacked where they changed (`registry.seen`), said; its C warmed first
    (`registry.warm`), its ready said then (`Apart.ready`)."""
    import numpy as np
    placed.placed(cache=True)
    kept: dict = {'looks': []}
    registry.warm()

    def answered(ask):
        if ask[1] is not None:
            kept['slots'] = ask[1]
        slots = kept['slots']
        if ask[0] == 'boxes':
            return registry.boxed(slots, ask[2])
        _kind, _laid_now, changed, n, (pos, quat), t, dt, everywhere = ask
        head = {'pos': np.array(pos), 'quat': np.array(quat)}
        looks = kept['looks'] if len(kept['looks']) == n else [None] * n
        for k, look_ in changed.items():
            looks[k] = look_
        kept['looks'] = looks
        registry.passed(slots, head, dt)
        registry.seen(slots, looks, head, t, everywhere=everywhere, kept=kept, changed=changed)
        boxes, was = registry.boxed(slots), kept.get('said')
        kept['said'] = boxes
        rows = None if was is None else registry.moved(was, boxes)
        if rows is None:
            return boxes
        if not len(rows):
            return SAME
        return (ROWS, rows.tolist(), boxes[1][rows], boxes[2][rows], boxes[3][rows], [boxes[5][k] for k in rows],
                [boxes[6][k] for k in rows])
    aside.serve(answered, ready=True)


if __name__ == '__main__':
    main()
