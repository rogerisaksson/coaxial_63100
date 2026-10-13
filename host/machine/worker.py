"""The gynoid worker process on her latent loop: paced to the clock, commands heard, state said."""
import multiprocessing
import queue
import time
import numpy as np
from machine import physics, sprung
from machine.errors import MachineError


def run(commands, states, every=sprung.STACK_EVERY, walks=False):
    """The worker: she stands on the loop, paced to the clock; commands heard, her state said.
    Laid, she stands at rest unless `walks`."""
    physics.ROOM, physics.SIGHT = 1.0, 0.0

    def anew():
        g = sprung.laid(every)
        if walks:
            sprung.walked(g)
        return g

    def _put(item):
        while states.full():
            try:
                states.get_nowait()
            except queue.Empty:
                break
        try:
            states.put_nowait(item)
        except queue.Full:
            pass

    gym, g = sprung.entries(), anew()
    says, refused = [sprung.GYMS[0]] + [entry[0] for entry in gym], None
    dt = 1.0 / sprung.RATE_HZ
    k = dt / sprung.AVERAGE_S
    torque, power = np.zeros(len(g['world'].peak)), np.zeros(len(g['world'].peak))
    watts, fallen_at = 0.0, None
    wall0, sim0 = time.perf_counter(), g['t']
    told, ratio, spent, ran = 0.0, 1.0, 0.0, 0.0
    _put(sprung.state(g, torque, power, watts, ratio, fallen_at, says, refused))
    while True:
        try:
            while True:
                command = commands.get_nowait()
                if command is None:
                    return
                if command.get('restart'):
                    g, fallen_at, told = anew(), None, 0.0
                    torque[:], power[:], watts, refused = 0.0, 0.0, 0.0, None
                    wall0, sim0 = time.perf_counter(), g['t']
                    _put(sprung.state(g, torque, power, watts, ratio, fallen_at, says, refused))
                try:
                    if command.get('gym') in sprung.GYMS:
                        gym = sprung.entries(name=command['gym'])
                        says, refused = [command['gym']] + [entry[0] for entry in gym], None
                    if 'text' in command:
                        command, refused = dict(command, entries=sprung.entries(command['text'])), None
                    sprung._heard(g, gym, command)
                except (SyntaxError, MachineError) as error:
                    refused = str(error)
        except queue.Empty:
            pass
        began = time.perf_counter()
        due = sim0 + (began - wall0)
        if due - g['t'] > sprung.SLICE_S:
            due = g['t'] + sprung.SLICE_S
            wall0, sim0 = began, due
        if fallen_at is not None and g['t'] - fallen_at >= sprung.RECOVER_S:
            g, fallen_at, told = anew(), None, 0.0
            torque[:], power[:], watts = 0.0, 0.0, 0.0
            wall0, sim0, due = time.perf_counter(), g['t'], g['t']
            _put(sprung.state(g, torque, power, watts, ratio, fallen_at, says, refused))
        from_t, world = g['t'], g['world']
        while g['t'] < due - 1e-9:
            try:
                if fallen_at is None:
                    sprung.passed(g, dt)
                else:
                    world.feed[:] = 0.0
                    g['t'] += dt
                    world.advance()
            except MachineError:
                fallen_at = g['t']
            if fallen_at is None:
                cur_doing = sprung.doing(g)
                cur_tilt = sprung.tilt(g)
                cur_y = float(world.data.qpos[1])
                is_fallen = False
                if cur_tilt > 75.0 or cur_y < 0.08:
                    is_fallen = True
                elif cur_doing in ('seated', 'approach', 'lower', 'lean', 'thrust'):
                    if cur_tilt > 45.0 or cur_y < 0.28:
                        is_fallen = True
                elif cur_doing in ('bed_rest', 'recline', 'sit_edge', 'sit_up'):
                    if cur_tilt > 45.0 or cur_y < 0.25:
                        is_fallen = True
                elif cur_doing in ('floor_rest', 'kneel', 'cushion', 'land'):
                    if cur_tilt > 75.0 or cur_y < 0.06:
                        is_fallen = True
                elif cur_doing not in sprung.STUNT_PHASES:
                    if cur_y < sprung.DOWN_M or (cur_tilt > sprung.DOWN_DEG and cur_y < 0.65) or cur_tilt > 80.0:
                        is_fallen = True
                if is_fallen:
                    fallen_at = g['t']
            tau = world.data.ctrl
            torque += k * (tau - torque)
            power += k * (tau * world.data.qvel[world.vadr] - power)
            watts += k * (world.drawn() - watts)
        spent += time.perf_counter() - began
        ran += g['t'] - from_t
        if g['t'] - told >= sprung.SAY_S:
            ratio = ran / spent if spent > 1e-6 else ratio
            spent = ran = 0.0
            told = g['t']
            _put(sprung.state(g, torque, power, watts, ratio, fallen_at, says, refused))
        rest = sprung.SAY_S / 4.0 - (time.perf_counter() - began)
        if rest > 0.0:
            time.sleep(rest)


class Running:

    """Her worker process on the loop (`run`): `send` it commands, read its `latest` state;
    laid, she stands at rest unless `walks`."""

    def __init__(self, every=sprung.STACK_EVERY, walks=False):
        context = multiprocessing.get_context('spawn')
        self._commands, self._states = context.Queue(), context.Queue(maxsize=8)
        self._process = context.Process(target=run, daemon=True,
                                        args=(self._commands, self._states, every, walks))
        self._process.start()
        self._last = None

    def send(self, **command):
        """{'say': the gym's entry by its place, in place of what was to come} | {'walk': True}:
        her walk from rest | {'entries': [(label, wire, names)]} | {'gym': one of GYMS} |
        {'text': orthokinematics, begun now} | {'push': (x, y, z) N, 'seconds': s} |
        {'restart': True}: laid anew. Any other key is refused, said with her state."""
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

