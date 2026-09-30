"""The gynoid in a process of her own, paced to the wall clock: whoever draws her never slows her.

    body = Running(cadence=0.85)         # a DYNAMIC gynoid landed in the squat, her director
    body.send(cadence=1.0); body.send(push=(0.0, 0.0, 120.0))
    body.send(glitch=('left_knee', 'soa', 0.5))   # a board glitched (`World.glitch`)
    body.send(event='sill')              # laid where her walk meets it (`machine.events`)
    body.send(style=('turn', +1))        # a style knob a step up (`machine.style`), eased in
    now = body.latest()                  # {'angles', 'where', 'turn', 'stage', ..}, or None yet
    body.close()

The loop runs at RATE_HZ on simulated time; each wall-clock slice it catches up to real time, at
most SLICE_S of it at once, and says how much faster than real time it can run (`ratio`). What
she does is the director's (`machine.director`): the arrival, the walk, a catch, fallen and up
by herself.
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

#: An event asked is met LEAD strides on: one on the floor laid that far ahead, one that befalls
#: her where she is counted down a stride at a time - seen coming.
LEAD = 1

#: Fallen, she gets up by herself (`machine.getup`); she is landed in the squat again RECOVER_S
#: after its tries are spent (`director.GETUP_TRIES`), or once down LIE_MAX_S without lying still.
RECOVER_S, LIE_MAX_S = 3.0, 15.0


def _run(commands, states, cadence, local):
    """The worker: the machine, the director, the loop paced to the clock."""
    import numpy as np

    from machine import Machine, events, style
    from machine.director import Director
    from machine.figure import JOINTS
    from machine import heat
    from machine.heat import GATES_ON
    from machine.modes import DYNAMIC
    machine = Machine.discover('gynoid', execution_mode=DYNAMIC)
    machine.arm()
    director = Director(machine, cadence, local=local)
    world = machine.nodes['pelvis'].world
    dt = 1.0 / RATE_HZ
    k = dt / AVERAGE_S
    torque, power = np.zeros(len(JOINTS)), np.zeros(len(JOINTS))
    #: What her drives draw, W: the work they do, their copper's heat, their boards' own
    #: (`machine.heat`: switching and housekeeping) - braking gives nothing back.
    boards, watts = len(JOINTS) * (heat.SWITCHING_W + heat.HOUSEKEEPING_W), 0.0
    peak = dict(zip(JOINTS, world.peak.tolist()))

    def begin():
        director.begin()
        machine.loop.step(0.0)
    begin()
    bus = machine.loop.bus
    wall0, sim0 = time.perf_counter(), bus['t']
    said, ratio, spent, ran, asked = 0.0, 1.0, 0.0, 0.0, {}
    #: An event asked and the strides before it is met (`LEAD`), laid as the left leg's phase
    #: crosses its `events.at` walking.
    event, was = None, 0.0
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
                if 'glitch' in command:
                    world.glitch(*command['glitch'])
                if 'event' in command:
                    event = [command['event'], LEAD]
                if 'style' in command:
                    style.trim(*command['style'])
                if 'sway' in command:
                    style.sway(command['sway'])
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
        if director.stage == 'fallen' and bus['t'] - director.fallen_at >= (
                RECOVER_S if director.given_up else LIE_MAX_S):
            begin()
            wall0, sim0 = time.perf_counter(), bus['t']
        while bus['t'] < due - 1e-9:
            if (event and director.stage == 'walk'
                    and was < events.at(event[0]) <= director.walker.phase):
                if event[0] in events.FLOOR:
                    events.lay(event[0], director, world, strides=event[1])
                    event[1] = 0
                event[1] -= 1
                if event[1] < 0 and event[0] in events.NOW:
                    events.lay(event[0], director, world)
                if event[1] < 0:
                    event = None
            was = director.walker.phase
            asked = director.step(dt)
            machine.loop.write(**asked)
            machine.loop.step(dt)
            tau = world.data.ctrl
            torque += k * (tau - torque)
            power += k * (tau * world.data.qvel[world.vadr] - power)
            drawn = (np.maximum(tau * world.data.qvel[world.vadr], 0.0).sum()
                     + world.loss @ (tau * tau) + boards)
            watts += k * (drawn - watts)
        spent += time.perf_counter() - began
        ran += bus['t'] - from_t
        if bus['t'] - said >= SAY_S:
            ratio = ran / spent if spent > 1e-6 else ratio
            spent = ran = 0.0
            said = bus['t']
            state = {'t': bus['t'], 'angles': dict({j: bus.get(j + '.deg', 0.0) for j in JOINTS},
                                                   **world.loose()),
                     'set': dict(asked),
                     'torque': dict(zip(JOINTS, torque.tolist())),
                     'power': dict(zip(JOINTS, power.tolist())), 'peak': peak,
                     'watts': float(watts),
                     'where': (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z']),
                     'turn': (bus['pelvis.pose.qw'], bus['pelvis.pose.qx'], bus['pelvis.pose.qy'],
                              bus['pelvis.pose.qz']),
                     'speed': bus['pelvis.pose.vz'], 'phase': director.walker.phase,
                     'velocity': (bus['pelvis.pose.vx'], bus['pelvis.pose.vz']),
                     'cadence': director.cadence, 'stage': director.stage,
                     'fallen': director.stage == 'fallen', 'slips': director.slips,
                     'stir': director.pendulum.stir, 'stirs': director.pendulum.stirs,
                     'swing': director.pendulum.swing,
                     'loads': (bus['pelvis.pose.left_load'], bus['pelvis.pose.right_load']),
                     'heat': {j: (bus[n + 'celsius'], bus[n + 'spent'], bus[n + 'derate'],
                                  int(bus[n + 'status']) & GATES_ON)
                              for j, n in director.drives.items()},
                     'props': world.props(), 'armed': tuple(event) if event else None,
                     'buses': ([(tuple(JOINTS[i] for i in b.indices), int(world.block.written[b.link]),
                                 int(world.block.sent[b.link]), b.bad) for b in world.buses.each]
                               if world.buses is not None else []),
                     'recover': (RECOVER_S - (bus['t'] - director.fallen_at)
                                 if director.given_up else None),
                     'style': style.state(), 'sway': style.AXIS, 'ratio': min(ratio, 99.0)}
            try:
                states.put_nowait(state)
            except queue.Full:
                pass
        rest = SAY_S / 4.0 - (time.perf_counter() - began)
        if rest > 0.0:
            time.sleep(rest)


class Running:

    """The gynoid's worker process: `send` it commands, read its `latest` state; `local` the
    model that plans her get-up (`machine.planner`), picklable."""

    def __init__(self, cadence=0.85, local=None):
        context = multiprocessing.get_context('spawn')
        self._commands, self._states = context.Queue(), context.Queue(maxsize=8)
        self._process = context.Process(target=_run, args=(self._commands, self._states, cadence, local),
                                        daemon=True)
        self._process.start()
        self._last = None

    def send(self, **command):
        """{'cadence': strides/s} | {'push': (x, y, z) N, 'seconds': s} | {'glitch': (joint,
        kind, s)} | {'event': one of `events.EVENTS`, laid where her walk meets it} |
        {'style': (knob, steps)}: a knob of `machine.style` trimmed, the walk eased over to it |
        {'sway': s}: every knob at s on `style.SWAY`'s axis, -1 catwalk to 1 swagger |
        {'restart': True}: landed in the squat again."""
        self._commands.put(command)

    def latest(self, into=None):
        """The newest state the worker has said, or the last one if nothing new; every one said
        since appended to `into` if given."""
        try:
            while True:
                self._last = self._states.get_nowait()
                if into is not None:
                    into.append(self._last)
        except queue.Empty:
            pass
        return self._last

    def close(self):
        self._commands.put(None)
        self._process.join(timeout=2.0)
        if self._process.is_alive():
            self._process.terminate()
