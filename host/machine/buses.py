"""Her boards on their buses: a limb's boards a process, the host's Modbus RTU frames real bytes.

    block = Block(joints, buses); buses = Buses(block, limbs)   # World.wire: the processes up
    bus.arm(index); bus.short(index)        # its gates on again; its phases shorted - the next pass
    bus.off(index)                          # its gates off - the next pass
    bus.send(at, now, {index: degrees})     # a pass: the setpoints broadcast, every board polled
    buses.step()                            # a step: every process ticks its boards, in lockstep
    buses.drain(); bus.reading(index)       # the replies read; the board as last heard
    python -m machine.buses NAME J B BAUD TURN_S K=i,i..    # a process: `serve`

The wire is Modbus RTU (`machine.rtu`) on RS485 at BAUD, USART2/UART5's rate (the .ioc), over a
TCP socket a bus - `socket://`, as an emulated limb's port. A pass the host writes one broadcast
of the bus's setpoints and a poll a board, their stamps in the block; a frame lands at the
boards its bytes after its stamp, or after the wire frees, 8N1; a poll is answered TURN_S after
it lands with the board's state then, the reply's bytes on the wire behind it, a gate write with
its echo. Nothing goes on a bus still busy a pass on. A board holds its setpoint by PD (`gains`)
every step, carried on at the rate its last two frames came at, within its clamp (`limit`) as
its envelope derates it and nothing with its gates dropped; its phases shorted, it
gives kt^2/R of the joint's speed back against it (`drives.heat`), within the same clamp - a
short from 1 979 rpm tripped the bench's board (docs/FINDINGS.md), so its current is held to
its amps; its heat (`machine.heat`) steps every THERMAL_S on the currents it gave.

The block (`Block`, FIELDS): the world writes time, q, qd and limit, bumps seq and sends a byte
to each process's stdin; a process takes each bus's new bytes (written - received), ticks its
boards (frames landed, PD to ctrl, polls answered and sent counted) and writes done = seq;
epoch and hold: a reset, every board holding `hold`, its heat at the room's; rotor: the inertia a
board feeds forward on its setpoint's acceleration, its own rotor's; play: half its gearbox's
backlash, rad - its encoder on the motor, a board sees its joint held within it; scale: its
ratio now over its rest's along its rod (`machine.linkage`), its clamp and its amps a N m by it;
emf, ohm, volts: its back-EMF a rad/s, its phase's resistance, the supply over sqrt 3 - its
q current no more than they leave at its speed; air, rds, warm:
a board glitched (`heat.Heat.step`, `heat.Heat.warm`; warm is cleared as taken); envelope: 0
fantasy boards (`heat.Heat.envelope`); drive: what its heat is kept by (`drives.heat`), written
before the processes start. An emulated limb takes a process's place on
the same port and block. A limb a process where the machine has THREADS_A_LIMB hardware threads
a limb, else the limbs shared out by their boards (`share`).
"""
import collections
import math
import os
import socket
import subprocess
import sys
import time
from multiprocessing import resource_tracker, shared_memory
from typing import Any

from machine import heat, rtu
from machine.errors import MachineError

#: The link's rate, bits/s (USART2 and UART5, coaxial_63100.ioc), and a board's turn from a poll
#: landing to its reply, s.
BAUD, TURN_S = 9_216_000, 30e-6

#: The boards' pace and the world's step, s; a board's heat steps every THERMAL_S.
LOCKSTEP_S, THERMAL_S = 0.001, 0.01

#: How long the host spins for the boards' step before yielding, and how long a step may take, s.
SPIN_S, STALL_S = 0.002, 60.0

#: A limb a process where the machine has this many hardware threads a limb.
THREADS_A_LIMB = 2

#: The block's fields: name, struct format, count - a number, or per joint (J) or bus (B).
FIELDS = (('time', 'd', 1), ('seq', 'q', 1), ('epoch', 'q', 1), ('done', 'q', 'B'),
          ('written', 'q', 'B'), ('sent', 'q', 'B'), ('free_at', 'd', 'B'), ('at', 'd', '2B'),
          ('q', 'd', 'J'), ('qd', 'd', 'J'), ('limit', 'd', 'J'), ('ctrl', 'd', 'J'),
          ('hold', 'd', 'J'), ('gains', 'd', '2J'), ('air', 'd', 'J'), ('rds', 'd', 'J'),
          ('warm', 'd', 'J'), ('envelope', 'd', 1), ('drive', 'd', '6J'), ('rotor', 'd', 'J'),
          ('play', 'd', 'J'), ('scale', 'd', 'J'), ('emf', 'd', 'J'), ('ohm', 'd', 'J'),
          ('volts', 'd', 1))

#: A board's setpoint's acceleration, read between frames, filtered over ACCEL_S: mdeg frames a
#: millisecond apart step it by 17 rad/s^2.
ACCEL_S = 0.005

HOST = '127.0.0.1'

#: A board's reading before its first reply: still, at the room's, nothing spent, gates on.
QUIET = (0.0, 0.0, heat.AMBIENT_C, 0.0, 1.0, heat.GATES_ON)


def wire(count, baud=None):
    """Seconds `count` bytes take on the wire, 8N1."""
    return count * 10.0 / (baud or BAUD)


def _count(spec, joints, buses):
    if isinstance(spec, int):
        return spec
    return int(spec[:-1] or 1) * (joints if spec[-1] == 'J' else buses)


def _exactly(sock, count):
    """`count` bytes from the socket."""
    chunks, got = [], 0
    while got < count:
        chunk = sock.recv(count - got)
        if not chunk:
            raise MachineError('the bus closed')
        chunks.append(chunk)
        got += len(chunk)
    return b''.join(chunks)


class Block:

    """The shared block, a memoryview a field: made by the world, opened by name in a process."""

    time: Any
    seq: Any
    epoch: Any
    done: Any
    written: Any
    sent: Any
    free_at: Any
    at: Any
    q: Any
    qd: Any
    limit: Any
    ctrl: Any
    hold: Any
    gains: Any

    def __init__(self, joints, buses, name=None):
        size = 8 * sum(_count(c, joints, buses) for _, _, c in FIELDS)
        if name is None:
            self.shm = shared_memory.SharedMemory(create=True, size=size)
        else:
            try:
                self.shm = shared_memory.SharedMemory(name=name, track=False)
            except TypeError:                 # before 3.13: unregistered by hand, or the
                self.shm = shared_memory.SharedMemory(name=name)   # tracker unlinks at exit
                if os.name != 'nt':
                    resource_tracker.unregister(self.shm._name, 'shared_memory')
        buf: Any = self.shm.buf
        if name is None:
            buf[:size] = bytes(size)
        self.name, self.owner = self.shm.name, name is None
        self.joints, self.buses, self.views = joints, buses, []
        at = 0
        for field, fmt, spec in FIELDS:
            n = 8 * _count(spec, joints, buses)
            view = buf[at:at + n].cast(fmt)
            setattr(self, field, view)
            self.views.append(view)
            at += n

    def close(self):
        for view in self.views:
            view.release()
        self.views = []
        self.shm.close()
        if self.owner:
            self.shm.unlink()


# -- the process ---------------------------------------------------------------------------

class Segment:

    """One bus in its process: its socket, the frames in flight, its boards' loops."""

    def __init__(self, block, link, indices, baud, turn):
        self.block, self.link, self.indices = block, link, list(indices)
        self.baud, self.turn = baud, turn
        n = len(self.indices)
        #: Per board: the setpoint held (rad), its rate (rad/s), when it was set (s), and
        #: whether a frame set it - the rate is read between two frames, never from a hold.
        self.target, self.rate, self.set_at = [0.0] * n, [0.0] * n, [0.0] * n
        self.framed, self.accel = [False] * n, [0.0] * n
        #: The joint as its board sees it through the play, rad (`play`).
        self.play, self.seen = [block.play[i] for i in self.indices], [0.0] * n
        #: Frames landing: (at, {unit: mdeg}); requests to answer: (at, unit, a gate write's
        #: frame or None for a poll).
        self.inbox, self.mail = collections.deque(), collections.deque()
        self.drives = [tuple(block.drive[6 * i:6 * i + 6]) for i in self.indices]
        #: The phases shorted, each joint's torque a rad/s of its speed: kt^2/R, N m s/rad.
        self.damping = [d[0] * d[0] / d[1] for d in self.drives]
        self.heat, self.heat_at = heat.Heat(self.drives), 0.0
        self.free_at, self.received, self.bad = 0.0, 0, 0
        self.epoch = block.epoch[0]
        self.server = socket.socket()
        self.server.bind((HOST, 0))
        self.server.listen(1)
        self.port = self.server.getsockname()[1]
        self.sock: Any = None

    def accept(self):
        self.sock, _ = self.server.accept()
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.server.close()

    def hold(self, now):
        """Every board holding its `hold`, nothing in flight: a reset."""
        b = self.block
        for k, i in enumerate(self.indices):
            self.target[k], self.rate[k], self.set_at[k] = b.hold[i], 0.0, now
            self.framed[k], self.accel[k], self.seen[k] = False, 0.0, b.hold[i]
        self.inbox.clear()
        self.mail.clear()
        self.free_at = now
        self.heat, self.heat_at = heat.Heat(self.drives), now

    def hear(self):
        """The host's new bytes: each frame onto the wire at its stamp, or when the wire frees."""
        b, link = self.block, self.link
        want = b.written[link] - self.received
        if want <= 0:
            return
        data = _exactly(self.sock, want)
        self.received += want
        stamps = (b.at[2 * link], b.at[2 * link + 1])
        frames, bad = rtu.requests(data)
        self.bad += bad
        for unit, fc, body in frames:
            if fc == rtu.WRITE and unit == rtu.BROADCAST:
                self.free_at = max(stamps[0], self.free_at) + wire(len(body) + 4, self.baud)
                self.inbox.append((self.free_at, rtu.setpoints(body)))
            elif fc in (rtu.READ, rtu.WRITE_ONE) and 1 <= unit <= len(self.indices):
                gate = rtu.framed(bytes((unit, fc)) + body) if fc == rtu.WRITE_ONE else None
                start = max(stamps[1], self.free_at) + wire(rtu.POLL_B, self.baud) + self.turn
                self.mail.append((start, unit, gate))
                self.free_at = start + wire(rtu.GATE_B if gate else rtu.REPLY_B, self.baud)

    def tick(self, now):
        """A step at `now`: frames landed set the boards' targets, each board's PD writes its
        torque within its clamp, its heat steps, polls landed are answered with the board's
        state now and gate writes with their echoes."""
        b, h = self.block, self.heat
        if b.epoch[0] != self.epoch:
            self.epoch = b.epoch[0]
            self.hold(now)
        self.hear()
        while self.inbox and self.inbox[0][0] <= now:
            at, setpoints = self.inbox.popleft()
            for unit, mdeg in setpoints.items():
                k = unit - 1
                if not 0 <= k < len(self.indices):
                    continue
                v, span = math.radians(mdeg / 1000.0), at - self.set_at[k]
                two = self.framed[k] and 1e-9 < span < 0.1
                rate = (v - self.target[k]) / span if two else 0.0
                self.accel[k] = (self.accel[k] + ((rate - self.rate[k]) / span - self.accel[k])
                                 * min(1.0, span / ACCEL_S)) if two else 0.0
                self.rate[k] = rate
                self.target[k], self.set_at[k], self.framed[k] = v, at, True
        for k, i in enumerate(self.indices):
            self.seen[k] = min(max(self.seen[k], b.q[i] - self.play[k]), b.q[i] + self.play[k])
            s = b.scale[i]
            if h.shorted[k]:
                # Its braking the world's damping (`physics.World.short`); its windings heat.
                b.ctrl[i] = 0.0
                top = b.limit[i]
                h.load(k, max(-top, min(top, -self.damping[k] * s * b.qd[i])))
            else:
                ref = self.target[k] + self.rate[k] * (now - self.set_at[k])
                tau = (b.gains[2 * i] * (ref - self.seen[k])
                       + b.gains[2 * i + 1] * (self.rate[k] - b.qd[i])
                       + b.rotor[i] * self.accel[k])
                # Motoring, the back-EMF takes from the supply; braking, it adds to it.
                emf = abs(b.emf[i] * s * b.qd[i])
                amps = (b.volts[0] + (-emf if tau * b.qd[i] > 0.0 else emf)) / b.ohm[i]
                top = (s * min(b.limit[i], self.drives[k][0] * max(0.0, amps)) * h.derate[k]
                       if h.gates[k] else 0.0)
                b.ctrl[i] = tau = max(-top, min(top, tau))
                h.load(k, tau / s)
            if b.warm[i] > 0.0:
                h.warm(k, b.warm[i])
                b.warm[i] = 0.0
        if now - self.heat_at >= THERMAL_S - 1e-9:
            h.envelope = b.envelope[0] > 0.0
            h.step(now - self.heat_at, [b.air[i] for i in self.indices],
                   [b.rds[i] for i in self.indices])
            self.heat_at = now
        out = b''
        while self.mail and self.mail[0][0] <= now:
            _, unit, gate = self.mail.popleft()
            i, k = self.indices[unit - 1], unit - 1
            if gate:
                op = rtu.gate_op(gate)
                (h.short if op == rtu.GATE_SHORT else h.off if op == rtu.GATE_OFF else h.arm)(k)
                out += rtu.echo(gate)
                continue
            celsius, spent, derate, status = h.report(k)
            out += rtu.reply(unit, round(math.degrees(self.seen[k]) * 1000.0),
                             round(math.degrees(b.qd[i]) * 1000.0), round(celsius * 100.0),
                             round(spent * 1e4), round(derate * 1e4), status)
        if out:
            self.sock.sendall(out)
            b.sent[self.link] += len(out)
        b.free_at[self.link] = self.free_at


def serve(argv):
    """A process's buses, `python -m machine.buses NAME J B BAUD TURN_S K=i,i..`: the ports said
    on stdout, a step a byte on stdin, over when stdin closes."""
    block = Block(int(argv[2]), int(argv[3]), argv[1])
    baud, turn = float(argv[4]), float(argv[5])
    segments = [Segment(block, int(k), [int(i) for i in spec.split(',')], baud, turn)
                for k, _, spec in (arg.partition('=') for arg in argv[6:])]
    sys.stdout.write(' '.join('%d:%d' % (s.link, s.port) for s in segments) + '\n')
    sys.stdout.flush()
    for segment in segments:
        segment.accept()
    while os.read(0, 1):
        now, seq = block.time[0], block.seq[0]
        for segment in segments:
            segment.tick(now)
        for segment in segments:
            block.done[segment.link] = seq
    block.close()


# -- the host ------------------------------------------------------------------------------

class Bus:

    """One bus as the host has it: its socket, the pass's frames out, the replies in."""

    def __init__(self, block, link, indices, port):
        self.block, self.link, self.indices = block, link, list(indices)
        self.sock = socket.create_connection((HOST, port))
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.polls = b''.join(rtu.poll(unit) for unit in range(1, len(self.indices) + 1))
        self.heard: dict[int, tuple] = {i: QUIET for i in self.indices}
        #: The gate writes for the next pass, {index: rtu op}.
        self.received, self.bad, self.gating = 0, 0, {}

    def send(self, at, now, degrees):
        """The pass: the bus's setpoints {index: degrees} - every board's, none for the polls
        alone - broadcast at `at` s, then every board polled at `now`; a bus still busy a pass
        on gets nothing."""
        b, link = self.block, self.link
        if b.free_at[link] > (at if degrees else now) + LOCKSTEP_S:
            return
        frames = b''.join(rtu.gate(self.indices.index(i) + 1, op)
                          for i, op in self.gating.items()) + self.polls
        self.gating = {}
        if degrees:
            frames = rtu.broadcast(1, [round(degrees[i] * 1000.0) for i in self.indices]) + frames
        b.at[2 * link], b.at[2 * link + 1] = at, now
        self.sock.sendall(frames)
        b.written[link] += len(frames)

    def hold(self, index, degrees):
        """A board holding `degrees` still, as at a reset: what the host has of it."""
        self.heard[index] = (degrees,) + QUIET[1:]
        self.gating = {}

    def arm(self, index):
        """Its board's gates on again, written with the next pass."""
        self.gating[index] = rtu.GATE_ON

    def short(self, index):
        """Its board's phases shorted through the low sides, written with the next pass."""
        self.gating[index] = rtu.GATE_SHORT

    def off(self, index):
        """Its board's gates off - no torque, nothing switched -, written with the next pass."""
        self.gating[index] = rtu.GATE_OFF

    def drain(self):
        """The replies the boards have sent, read: what the host last heard of each."""
        want = self.block.sent[self.link] - self.received
        if want <= 0:
            return
        data = _exactly(self.sock, want)
        self.received += want
        frames, bad = rtu.replies(data)
        self.bad += bad
        for unit, fc, body in frames:
            if fc == rtu.READ and 1 <= unit <= len(self.indices):
                angle, rate, centi_c, spent, derate, status = rtu.state(body)
                self.heard[self.indices[unit - 1]] = (angle / 1000.0, rate / 1000.0,
                                                      centi_c / 100.0, spent / 1e4, derate / 1e4,
                                                      status)

    def reading(self, index):
        """(degrees, deg/s, C, spent, derate, status) of a board as the host last heard it."""
        return self.heard[index]


def share(limbs, threads=None):
    """[[bus, ..], ..]: the limbs by process - one each with THREADS_A_LIMB hardware threads a
    limb, else shared out by their boards, the heaviest first onto the lightest."""
    threads = (os.cpu_count() or 1) if threads is None else threads
    count = max(1, min(len(limbs), threads // THREADS_A_LIMB))
    groups = [[0, []] for _ in range(count)]
    for k in sorted(range(len(limbs)), key=lambda k: -len(limbs[k])):
        lightest = min(groups, key=lambda g: g[0])
        lightest[0] += len(limbs[k])
        lightest[1].append(k)
    return [sorted(g[1]) for g in groups]


class Buses:

    """A world's buses: their processes up and connected, a Bus each - `each` in bus order, `of`
    by joint index."""

    def __init__(self, block, limbs, baud=None, turn=None):
        self.block, self.processes, self.seq = block, [], 0
        root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        for group in share(limbs):
            args = [sys.executable, '-m', 'machine.buses', block.name, str(block.joints),
                    str(block.buses), repr(baud or BAUD), repr(turn or TURN_S)]
            args += ['%d=%s' % (k, ','.join(map(str, limbs[k]))) for k in group]
            self.processes.append(subprocess.Popen(args, cwd=root, stdin=subprocess.PIPE,
                                                   stdout=subprocess.PIPE, bufsize=0))
        ports = {}
        for process in self.processes:
            line = process.stdout.readline().decode()
            if not line:
                raise MachineError('a bus process ended before it served')
            ports.update((int(k), int(p)) for k, _, p in (w.partition(':') for w in line.split()))
        self.each = [Bus(block, k, indices, ports[k]) for k, indices in enumerate(limbs)]
        self.of = {i: bus for bus in self.each for i in bus.indices}

    def step(self):
        """One lockstep: every process told, every bus done."""
        b = self.block
        self.seq += 1
        b.seq[0] = self.seq
        for process in self.processes:
            process.stdin.write(b'\0')
        began = time.perf_counter()
        while min(b.done) < self.seq:
            if time.perf_counter() - began > SPIN_S:
                self._alive(began)
                time.sleep(0)

    def _alive(self, began):
        for k, process in enumerate(self.processes):
            if process.poll() is not None:
                raise MachineError('bus process %d ended (%s)' % (k, process.returncode))
        if time.perf_counter() - began > STALL_S:
            raise MachineError('the buses stalled %.0f s' % STALL_S)

    def drain(self):
        for bus in self.each:
            bus.drain()

    def close(self):
        """The processes over: their stdin closed, then waited for."""
        for process in self.processes:
            for pipe in (process.stdin, process.stdout):
                try:
                    pipe.close()
                except OSError:
                    pass
        for bus in self.each:
            bus.sock.close()
        for process in self.processes:
            try:
                process.wait(2.0)
            except subprocess.TimeoutExpired:
                process.kill()
        self.processes, self.each, self.of = [], [], {}


if __name__ == '__main__':
    serve(sys.argv)
