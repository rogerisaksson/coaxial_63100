"""A world stepped in a process of its own, paced to the wall and kept to what its page has shown.

Behind the wall by more than a slice, its debt is dropped rather than run off; given what its page
has `shown`, it runs no further than `lead_s` past it; commands in, states said out. Her body
(`machine.worker`) and the crafts' flights (`terminal.views.quad.flying`) run on it.

    apart = Apart(target, *args)       # target(commands, states, shown, *args), a spawned process
    apart.send(**command); newest = apart.latest(into=said); apart.shown(t); apart.close()

    paced(world, commands, states)     # a target's loop on a world: its `t` and `epoch`, `apply`
                                       # (a command), `step` (wall s left out of the pace), `said`
"""
import math
import multiprocessing
import queue
import time

#: Behind the wall by more than SLICE_S, a world runs one slice and drops the rest of its debt;
#: held to its page, it runs LEAD_S of its time past what that has shown at the most, s.
SLICE_S, LEAD_S = 0.05, 0.15

#: The processes run apart are spawned: none inherits the page's threads.
CONTEXT = multiprocessing.get_context('spawn')


def put(states, item):
    """`item` on `states`, the oldest dropped while it is full."""
    while states.full():
        try:
            states.get_nowait()
        except queue.Empty:
            break
    try:
        states.put_nowait(item)
    except queue.Full:
        pass


def paced(world, commands, states, shown=None, lead_s=LEAD_S, realtime=True, say_s=0.0,
          rest_s=0.0, slice_s=SLICE_S):
    """`world` stepped until a None command: paced to the clock (`realtime`, or flat out a slice
    at a time without it), no further than `lead_s` past `shown` (a shared double) given one;
    each command applied before the passes after it; said on `states` first, every `say_s` of
    its time - each pass at none - with its ratio to the wall, and at each new epoch. A pass's
    `step` returns the wall seconds it spent that the pace leaves out; `rest_s` the loop's least
    turn while it waits."""
    put(states, world.said(None))
    wall0 = wall_said = last_wall = time.perf_counter()
    sim0, said_at, spent, ran, ratio = world.t, 0.0, 0.0, 0.0, 1.0
    while True:
        epoch = world.epoch
        try:
            while True:
                command = commands.get_nowait()
                if command is None:
                    return
                world.apply(command)
        except queue.Empty:
            pass
        began = time.perf_counter()
        if world.epoch == epoch:
            if not realtime:
                due = world.t + slice_s
            else:
                if began - last_wall > slice_s:
                    wall0, sim0 = began, world.t
                due = sim0 + (began - wall0)
                if due - world.t > slice_s:
                    due = world.t + slice_s
                    wall0, sim0 = began - slice_s, world.t
            if shown is not None:
                due = min(due, shown.value + lead_s)
            from_t = world.t
            while world.t < due - 1e-9 and world.epoch == epoch:
                t_step = time.perf_counter()
                left_out = world.step()
                wall0 += left_out
                wall_said += left_out
                spent += max(0.0, (time.perf_counter() - t_step) - left_out)
                ran += world.t - from_t
                from_t = world.t
                if world.epoch == epoch and world.t - said_at >= say_s:
                    now_w = time.perf_counter()
                    wall_dt = now_w - wall_said
                    ratio = (min(1.0, ran / wall_dt) if (realtime and wall_dt > 1e-6)
                             else (ran / spent if spent > 1e-6 else ratio))
                    wall_said, spent, ran, said_at = now_w, 0.0, 0.0, world.t
                    put(states, world.said(ratio))
        last_wall = time.perf_counter()
        if world.epoch != epoch:
            wall0 = wall_said = last_wall
            sim0, said_at, spent, ran = world.t, 0.0, 0.0, 0.0
            put(states, world.said(None))
            continue
        if realtime:
            rest = rest_s - (last_wall - began)
            if rest > 0.0:
                time.sleep(rest)
                last_wall = time.perf_counter()


class Apart:

    """`target(commands, states, shown, *args)` in a spawned process: `send` it commands, read its
    `latest` state, tell it what the page has `shown` - none till then, past `shown` -; `kept`
    more of its queues, drained on `close`. `states` the most its queue holds."""

    def __init__(self, target, *args, states=8, kept=(), shown=math.inf):
        self._commands, self._states = CONTEXT.Queue(), CONTEXT.Queue(maxsize=states)
        self._kept = tuple(kept)
        for each in (self._commands, self._states) + self._kept:
            each.cancel_join_thread()
        self._shown = CONTEXT.Value('d', shown, lock=False)
        self._process = CONTEXT.Process(target=target, daemon=True,
                                        args=(self._commands, self._states, self._shown) + args)
        self._process.start()
        self._last = None

    def send(self, **command):
        """`command` to the world, applied before its next pass."""
        self._commands.put(command)

    def latest(self, into=None):
        """The newest state the world has said, or the last one if nothing new; every one said
        since appended to `into` if given."""
        try:
            while True:
                self._last = self._states.get_nowait()
                if into is not None:
                    into.append(self._last)
        except queue.Empty:
            pass
        return self._last

    def shown(self, t):
        """The page has shown the world up to `t`, its time: it runs no further than its lead
        past that."""
        self._shown.value = t

    def alive(self):
        return self._process.is_alive()

    def close(self, wait=1.0):
        """The world stopped: a None command, its queues drained, the process joined - `wait` s
        at the most - or ended."""
        try:
            self._commands.put_nowait(None)
        except queue.Full:
            pass
        for _ in (0, 1):
            for each in (self._states,) + self._kept:
                while True:
                    try:
                        each.get_nowait()
                    except queue.Empty:
                        break
            if not _:
                self._process.join(timeout=wait)
                if self._process.is_alive():
                    self._process.terminate()
