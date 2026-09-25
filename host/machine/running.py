"""The gynoid in a process of her own, paced to the wall clock: whoever draws her never slows her.

    body = Running(cadence=0.85)         # a DYNAMIC gynoid landed in the squat, her director
    body.send(cadence=1.0); body.send(push=(0.0, 0.0, 120.0))
    now = body.latest()                  # {'angles', 'where', 'turn', 'stage', ..}, or None yet
    body.close()

The loop runs at RATE_HZ on simulated time; each wall-clock slice it catches up to real time, at
most SLICE_S of it at once, and says how much faster than real time it can run (`ratio`). What
she does is the director's (`machine.director`): the arrival, the walk, a catch, fallen and up
again.
"""
import multiprocessing
import queue
import time

#: The loop's rate, Hz: at 500 the walker's feedback came too late and she fell (2026-09-25).
RATE_HZ = 1000.0

#: The most simulated time one slice catches up, s; how often the state goes out, s.
SLICE_S, SAY_S = 0.05, 1.0 / 60.0

#: Each drive's torque and power as said: meaned over AVERAGE_S, s.
AVERAGE_S = 0.05


def _run(commands, states, cadence):
    """The worker: the machine, the director, the loop paced to the clock."""
    import numpy as np

    from machine import Machine
    from machine.director import Director
    from machine.figure import JOINTS
    from machine.modes import DYNAMIC
    machine = Machine.discover('gynoid', execution_mode=DYNAMIC)
    machine.arm()
    director = Director(machine, cadence)
    world = machine.nodes['pelvis'].world
    dt = 1.0 / RATE_HZ
    k = dt / AVERAGE_S
    torque, power = np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
    peak = dict(zip(JOINTS, world.peak.tolist()))

    def begin():
        director.begin()
        machine.loop.step(0.0)
    begin()
    bus = machine.loop.bus
    wall0, sim0 = time.perf_counter(), bus['t']
    said, ratio, spent, ran = 0.0, 1.0, 0.0, 0.0
    while True:
        try:
            while True:
                command = commands.get_nowait()
                if command is None:
                    return
                if 'cadence' in command:
                    director.cadence = float(command['cadence'])
                if 'push' in command:
                    world.push(command['push'], command.get('seconds', 0.1))
                if command.get('restart'):
                    begin()
                    wall0, sim0 = time.perf_counter(), bus['t']
        except queue.Empty:
            pass
        began = time.perf_counter()
        due = sim0 + (began - wall0)
        if due - bus['t'] > SLICE_S:
            due = bus['t'] + SLICE_S                     # behind: a slice, and the clock let go
            wall0, sim0 = began, due
        from_t = bus['t']
        while bus['t'] < due - 1e-9:
            machine.loop.write(**director.step(dt))
            machine.loop.step(dt)
            tau = world.data.ctrl
            torque += k * (tau - torque)
            power += k * (tau * world.data.qvel[world.vadr] - power)
        spent += time.perf_counter() - began
        ran += bus['t'] - from_t
        if bus['t'] - said >= SAY_S:
            ratio = ran / spent if spent > 1e-6 else ratio
            spent = ran = 0.0
            said = bus['t']
            state = {'t': bus['t'], 'angles': {j: bus.get(j + '.deg', 0.0) for j in JOINTS},
                     'torque': dict(zip(JOINTS, torque.tolist())),
                     'power': dict(zip(JOINTS, power.tolist())), 'peak': peak,
                     'where': (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z']),
                     'turn': (bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                              bus['pelvis.pose.qz']),
                     'speed': bus['pelvis.pose.vz'], 'phase': director.walker.phase,
                     'cadence': director.cadence, 'stage': director.stage,
                     'fallen': director.stage == 'fallen', 'slips': director.slips,
                     'loads': (bus['pelvis.pose.left_load'], bus['pelvis.pose.right_load']),
                     'ratio': min(ratio, 99.0)}
            try:
                states.put_nowait(state)
            except queue.Full:
                pass
        rest = SAY_S / 4.0 - (time.perf_counter() - began)
        if rest > 0.0:
            time.sleep(rest)


class Running:

    """The gynoid's worker process: `send` it commands, read its `latest` state."""

    def __init__(self, cadence=0.85):
        context = multiprocessing.get_context('spawn')
        self._commands, self._states = context.Queue(), context.Queue(maxsize=8)
        self._process = context.Process(target=_run, args=(self._commands, self._states, cadence),
                                        daemon=True)
        self._process.start()
        self._last = None

    def send(self, **command):
        """{'cadence': strides/s} | {'push': (x, y, z) N, 'seconds': s} | {'restart': True}:
        landed in the squat again."""
        self._commands.put(command)

    def latest(self):
        """The newest state the worker has said, or the last one if nothing new."""
        try:
            while True:
                self._last = self._states.get_nowait()
        except queue.Empty:
            pass
        return self._last

    def close(self):
        self._commands.put(None)
        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
