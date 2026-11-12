"""Her wayfinder apart: her dock and way planned in a process of their own, as her camera's map is.

A peek - a thing's dock foreseen (`machine.way.peek`) - is taken when it is ready, never waited for.
A watch asked at t is answered at t + LATENCY_S (`run-wayfinder-apart`), a dock or a plan she stands for at
once, whatever the wall's pace: her loop waits on one only where the planner is slower than that, and a run
is the same at any pace - a dock told 0.25 s on shifted her walk's onset after her turn in place and she
fell where she had walked on. An ask is a dock - the
face of a thing she docks at and her stage before it (`dock.docked`), then her way there -, a plan - her way
to a place from where she stands among her things (`route.planned` over `route.costs`) - or a watch: her way
left checked against her things as she now holds them (`route.barred`), planned anew where they bar it, else
nothing. A new ask lets the one in flight go. `python -m machine.way.planner`, a process beside hers
(`machine.process.aside`), as her camera's map is.

    planner = Planner()                     # her page's: spawned at its first ask (`start` sooner)
    planner = Here()                        # a run's off her page, in its own process: as Planner answers
    planner.ask(t, ask)                     # {kind, at, yaw, still, to, things, floor, seen, slot, face, dock, warm, passed, way}
    told = planner.answer(t)                # None till LATENCY_S after an ask, then {laid, stops, curve, held}, or {}
    planner.close()
"""
import operator
import queue

from machine.process import aside, placed
from machine.way import obstacles
from machine.errors import MachineError

_APART = obstacles.laws()['wayfinder-apart']
#: A watch's answer reaches her this long after it, s; her way is watched this often as she walks it, s.
LATENCY_S, WATCH_S = _APART['watch-latency-s'], _APART['watch-period-s']
#: What an ask tells of her things where they are the ask before's (`Planner.ask`): those her planner apart kept.
SAME = 'her things as last asked'


def answered(ask):
    """An ask's answer: a dock's {laid, stops, curve, held}, `laid` (`dock.docked`) None alone where no
    face fits her; a plan's {stops, curve, held}; a watch's that where her way left is barred, else {}."""
    import numpy as np
    from machine.way import dock, route
    if ask['kind'] == 'peek':
        return {'laid': dock.docked(ask['at'], ask['things'], ask['floor'], *ask['dock'])}
    laid = dock.docked(ask['at'], ask['things'], ask['floor'], *ask['dock']) if ask['kind'] == 'dock' else {}
    if laid is None:
        return {'laid': None}
    to, face = laid.get('stage', ask['to']), laid.get('face', ask['face'])
    cost = route.costs(ask['things'], ask['floor'], ask['seen'], ask['slot'], face)
    if ask['kind'] == 'watch' and not route.barred(np.isinf(cost), ask['way']):
        return {}
    stops, curve, held = route.planned(ask['at'], ask['yaw'], ask['still'], to, cost, ask['warm'], ask['passed'])
    return dict({'stops': stops, 'curve': curve, 'held': held}, **{'laid': laid} if laid else {})


def main():
    """Her planner apart: each ask answered (`aside.serve`), her things those of the ask before where it tells them
    SAME (`Planner.ask`). On a cache group of its own (`placed`): her page's peeker in it planned docks 0.83 of a
    core over six caches, her vision's among them."""
    placed.placed(cache=True)
    from machine.way import dock, route  # noqa: F401 - warm before the first ask
    kept: dict = {}

    def served(ask):
        if isinstance(ask['things'], str):
            ask = dict(ask, things=kept['things'])
        kept['things'] = ask['things']
        return answered(ask)
    aside.serve(served)


class Planner:

    """Her planner on its process (`main`, this tree's own), spawned at its first ask or `start`."""

    def __init__(self):
        self._aside, self._told_q = aside.Aside('machine.way.planner', 'her planner'), queue.Queue()
        self.asked_at, self._wait, self._things = None, 0.0, None

    def start(self):
        """Its process spawned, where it is not yet."""
        if self._aside.process is not None:
            return
        self._things = None
        self._aside.start().heard(self._told_q)

    def ask(self, now, ask):
        """`ask` sent at `now`, the one in flight let go (its answer read and dropped); her things told SAME where they
        are the very ones of the ask before - kept there (`main`) -: pickled with every ask they were 0.8 to 1.8 ms of
        each, her peeks' one after another, her things new at a look alone."""
        self.start()
        if self.asked_at is not None:
            self._told()
        things, was = ask['things'], self._things
        same = was is not None and len(things) == len(was) and all(map(operator.is_, things, was))
        self._things = things
        self._aside.send(dict(ask, things=SAME) if same else ask)
        self.asked_at, self._wait = now, LATENCY_S * float(ask['kind'] == 'watch')

    def answer(self, now):
        """The ask in flight's answer at `now` - waited for where its process is late -, None before its
        latency after it or with none in flight."""
        if self.asked_at is None or now - self.asked_at < self._wait - 1e-9:
            return None
        self.asked_at = None
        return self._told()

    def ready(self):
        """The ask in flight's answer where its process has given it, None else: never waited for."""
        if self.asked_at is None or self._told_q.empty():
            return None
        self.asked_at = None
        return self._told()

    def _told(self):
        told = self._told_q.get()
        if told is None:
            raise MachineError(self._aside.ended())
        return told

    def close(self, wait=1.0):
        """Its process ended (`aside.Aside.close`)."""
        self._aside.close(wait)
        self.asked_at, self._things = None, None


class Here:

    """Her planner as `Planner` answers it, in the process that asks: a run off her page, one process a
    baton (`focus.relay`)."""

    def __init__(self):
        self.asked_at, self._told, self._wait = None, None, 0.0

    def ask(self, now, ask):
        self._told, self.asked_at, self._wait = answered(ask), now, LATENCY_S * float(ask['kind'] == 'watch')

    def answer(self, now):
        if self.asked_at is None or now - self.asked_at < self._wait - 1e-9:
            return None
        self.asked_at = None
        return self._told

    def ready(self):
        told, self.asked_at = (self._told if self.asked_at is not None else None), None
        return told


if __name__ == '__main__':
    main()
