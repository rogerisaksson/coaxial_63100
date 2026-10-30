"""Her looks beside her law: a look's registry pass on a process of its own, joined before it is read.

A look at a refresh (`sprung.passed`) is asked before her law's pass and its things joined just before her
wayfinder reads them (`motion.control`), her sensing, latent and legs run meanwhile: 0.99 ms of her law
beside a look's 0.94 ms. Her registry, her looks and their arithmetic are the same either way, so are their
bits: in her process (`Here`, a run's), or on its own (`Apart`, her page's worker's, `machine.aside`) - her
registry kept there, the laid one sent it at each epoch and read here no more (`Kept`).

    look(g); ...; joined(g)                    # a look asked at g['t'], her things joined once after it
    looked(g)                                  # both at once: a look between her passes (`sprung.looked`)
    things = g['vision'].boxes(g, least)       # her registry's boxes at `least` salience, between looks
"""
from machine import aside, inekf, registry
from machine.errors import MachineError

#: A look's things: the world's boxes as her eyes' stand-in, or her eyes' map's (`g['keep']`, `physics.SIGHT`).
LOOKS = (lambda g: g['world'].props(), lambda g: g['keep'].told())


def look(g):
    """A look asked at `g['t']` (LOOKS by her eyes): her camera's shot first, her head as her law last sensed it,
    the time since her last look."""
    head = {'pos': g['s']['head']['pos'].copy(), 'quat': g['s']['head']['quat'].copy()}
    if g['keep'] is not None:
        g['keep'].look(g['world'], g['t'], inekf.head(g['inekf'], g['body']))
    g['vision'].start(g, LOOKS[g['eyes']](g), head, g['t'] - g.get('things_at', g['t']), not g['eyes'])
    g['things_at'] = g['t']


def joined(g):
    """Her things after the look asked last (`look`), once; none asked, nothing."""
    if g['vision'].pending:
        g['things'] = g['vision'].join()


def looked(g):
    """A look (`look`) and her things after it (`joined`) at once."""
    look(g)
    joined(g)


class Here:

    """Her looks in her process, each passed as it is asked."""

    apart = False

    def __init__(self):
        self.pending, self._things = False, None

    def start(self, g, looks, head, dt, everywhere):
        """Her registry (`g['registry']`) `dt` s on and the look `looks` into it from her `head` at `g['t']`."""
        registry.passed(g['registry'], head, dt)
        registry.seen(g['registry'], looks, head, g['t'], everywhere=everywhere)
        self._things, self.pending = registry.boxes(g['registry']), True

    def join(self):
        self.pending = False
        return self._things

    def boxes(self, g, least):
        """Her registry's boxes at `least` salience (`registry.boxes`), a look in flight joined first."""
        joined(g)
        return registry.boxes(g['registry'], least)

    def close(self, wait=1.0):
        pass


class Apart:

    """Her looks on a process of their own (`main`), spawned warm at `start()`: each asked, its things joined."""

    apart = True

    def __init__(self):
        self._aside, self.pending, self._looks = aside.Aside('machine.vision', 'her vision'), False, []

    def start(self, g=None, looks=None, head=None, dt=0.0, everywhere=True):
        """`Here.start` asked of its process - the look in flight before it dropped -; `g` None, its process spawned."""
        self._aside.start()
        if g is None:
            return
        if self.pending:
            self.join()
        self._aside.send(('look', _laid(g), self._changed(looks), len(looks), head, g['t'], dt, everywhere))
        self.pending = True

    def _changed(self, looks):
        """{place: look} of `looks` not as last sent - a part not the same object, its tuples and floats the same
        values where it is: 58 of her room's 59 are, its door's not -; all of them where their count changed."""
        was, self._looks = self._looks, looks
        if len(was) != len(looks):
            return dict(enumerate(looks))
        return {k: p for k, (p, q) in enumerate(zip(looks, was))
                if len(p) != len(q) or p[0] != q[0] or any(a is not b for a, b in zip(p[1:], q[1:]))}

    def join(self):
        """The look in flight's things, waited for."""
        self.pending = False
        return registry.things(*self._aside.recv())

    def boxes(self, g, least):
        joined(g)
        self._aside.send(('boxes', _laid(g), least))
        return registry.things(*self._aside.recv())

    def close(self, wait=1.0):
        self._aside.close(wait)


class Kept(dict):

    """Her registry once sent her vision apart (`Apart`): its slots are kept there, read here it raises."""

    def __missing__(self, key):
        raise MachineError('her registry is her vision\'s apart (`vision.Apart`): its %r is read there' % (key,))


def _laid(g):
    """Her registry as laid, where her vision apart has it not yet - `Kept` here once sent -; else None."""
    if isinstance(g['registry'], Kept):
        return None
    slots, g['registry'] = g['registry'], Kept()
    return slots


def main():
    """Her vision apart (`aside.serve`): her registry as laid, each look - its changes on the last - passed into
    it, its boxes said."""
    kept = {'looks': []}

    def answered(ask):
        if ask[1] is not None:
            kept['slots'] = ask[1]
        slots = kept['slots']
        if ask[0] == 'boxes':
            return registry.boxed(slots, ask[2])
        _kind, _laid_now, changed, n, head, t, dt, everywhere = ask
        looks = kept['looks'] if len(kept['looks']) == n else [None] * n
        for k, look_ in changed.items():
            looks[k] = look_
        kept['looks'] = looks
        registry.passed(slots, head, dt)
        registry.seen(slots, looks, head, t, everywhere=everywhere)
        return registry.boxed(slots)
    aside.serve(answered)


if __name__ == '__main__':
    main()
