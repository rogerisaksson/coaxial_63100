"""The robot's wayfinder apart: its dock and way planned in their own process, as its camera's map is.

A peek - a thing's dock foreseen (`machine.way.peek`) - is taken when it is ready, never waited for.
A watch asked at t is answered at t + LATENCY_S (`run-wayfinder-apart`), a dock or a plan the robot stands for
at once, whatever the wall's pace: its loop waits on one only where the planner is slower than that, and a run
is the same at any pace - a dock told 0.25 s on shifted its walk's onset after its turn in place and it fell
where it had walked on. An ask is a dock - the face of a thing the robot docks at and its stage before
it (`dock.docked`), then the robot's way there -, a plan - its way to a place from where the robot
stands among its things (`route.planned` over `route.costs`) - or a watch: its way left
checked against its things as it now holds them (`route.barred`), planned anew where they bar
it, else nothing. A new ask lets the one in flight go. `python -m machine.way.planner`, a process beside the
robot's own (`machine.process.aside`), as its camera's map is.

    planner = Planner()                     # the robot's page's: spawned at its first ask (`start` sooner)
    planner = Here()                        # a run's off the robot's page, in its own process: as Planner answers
    planner.ask(t, ask)                     # {kind, at, yaw, still, to, things, floor, seen, slot, face, dock, warm, passed, way}
    told = planner.answer(t)                # None till LATENCY_S after an ask, then {laid, stops, curve, held}, or {}
    planner.close()
"""
import queue

from machine.process import aside, placed
from machine.way import obstacles
from machine.errors import MachineError

_APART = obstacles.laws()['wayfinder-apart']
#: A watch's answer reaches the robot this long after it, s; the robot's way is watched this often as the
#: robot walks it, s.
LATENCY_S, WATCH_S = _APART['watch-latency-s'], _APART['watch-period-s']
#: What an ask tells of the robot's things where they are the ask before's (`Planner.ask`): those its planner
#: apart kept.
SAME, ROWS = 'its things as last asked', 'its things as last asked but these rows'


def answered(ask):
    """An ask's answer: a dock's {laid, stops, curve, held}, `laid` (`dock.docked`) None alone where no
    face fits the robot - a peek's at its standoff told here (`peek.standoff`); a plan's {stops, curve, held}; a
    watch's that where its way left is barred, else {}."""
    import numpy as np
    from machine.way import dock, peek, route
    if ask['kind'] == 'peek':
        verb, box = ask['dock']
        return {'laid': dock.docked(ask['at'], ask['things'], ask['floor'], verb, box, peek.standoff(ask))}
    laid = dock.docked(ask['at'], ask['things'], ask['floor'], *ask['dock']) if ask['kind'] == 'dock' else {}
    if laid is None:
        return {'laid': None}
    to, face = laid.get('stage', ask['to']), laid.get('face', ask['face'])
    cost = route.costs(ask['things'], ask['floor'], ask['seen'], ask['slot'], face)
    if ask['kind'] == 'watch' and not route.barred(np.isinf(cost), ask['way']):
        return {}
    if ask['kind'] == 'dock' and laid.get('d_stage') is not None:
        corners = route.way(ask['at'], to, cost)
        laid = dock.staged(laid, ([ask['at']] + corners)[-2], ask['yaw'], ask['things'], ask['floor'],
                           np.isinf(cost), *ask['dock'])
        to = laid['stage']
    stops, curve, held = route.planned(ask['at'], ask['yaw'], ask['still'], to, cost, ask['warm'], ask['passed'])
    return dict({'stops': stops, 'curve': curve, 'held': held}, **{'laid': laid} if laid else {})


def main():
    """The robot's planner apart: each ask answered (`aside.serve`), its things those of the ask before
    where it tells them SAME (`Planner.ask`). On a cache group of its own (`placed`): the robot's page's peeker in
    it planned docks 0.83 of a core over six caches, the robot's vision's among them."""
    placed.placed(cache=True)
    from machine.way import dock, route  # noqa: F401 - warm before the first ask
    kept: dict = {}

    def served(ask):
        things = ask['things']
        if isinstance(things, str):
            ask = dict(ask, things=kept['things'])
        elif isinstance(things, tuple) and things[0] == ROWS:
            laid = list(kept['things'])
            for k, row in zip(things[1], things[2]):
                laid[k] = row
            ask = dict(ask, things=laid)
        kept['things'] = ask['things']
        return answered(ask)
    aside.serve(served)


class Planner:

    """The robot's planner on its process (`main`, this tree's own), spawned at its first ask or `start`."""

    def __init__(self):
        self._aside, self._told_q = aside.Aside('machine.way.planner', 'its planner'), queue.Queue()
        self.asked_at, self._wait, self._things = None, 0.0, None

    def start(self):
        """The process spawned, where it is not yet."""
        if self._aside.process is not None:
            return
        self._things = None
        self._aside.start().heard(self._told_q)

    def ask(self, now, ask):
        """`ask` sent at `now`, the one in flight let go (its answer read and dropped); the robot's things
        told SAME where they are the very ones of the ask before - kept there (`main`) -, and those of them not
        the very ones alone where the rest are (ROWS: a look's rows that moved, `vision.ROWS`): pickled with every
        ask they were 0.8 to 1.8 ms of each, its peeks' one after another; a bed creeping under the robot renews
        a row every look, its things whole 1.35 ms an ask."""
        self.start()
        if self.asked_at is not None:
            self._told()
        things, was = ask['things'], self._things
        moved = None if was is None or len(things) != len(was) else [
            k for k, (p, q) in enumerate(zip(things, was)) if p is not q]
        self._things = things
        self._aside.send(ask if moved is None or len(moved) == len(things) else dict(
            ask, things=(ROWS, moved, [things[k] for k in moved]) if moved else SAME))
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
        """The process ended (`aside.Aside.close`)."""
        self._aside.close(wait)
        self.asked_at, self._things = None, None


class Here:

    """The robot's planner as `Planner` answers it, in the process that asks: a run off the robot's page, one
    process a baton (`focus.relay`)."""

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
