"""Her boards on their buses, simulated. A thread a limb, its boards' loops at their own pace on
the shared world, the host's frames on the wire between them at the link's rate.

    bus = Bus(world, link, indices)          # a limb's boards, by joint index (`figure.JOINTS`)
    bus.send(at, {index: degrees})           # the host's setpoints, one broadcast, at `at` s
    bus.poll(at)                             # and its polls, a reply a board, landing in turn
    bus.tick(now)                            # a lockstep: frames landed applied, each board's
                                             # PD to the world's ctrl, replies out (`World`)
    bus.reading(index)                       # (degrees, rate, stamp) as the host last heard

The link is Modbus RTU on RS485 at 10 Mbit. The host writes every board's setpoint in one
broadcast a pass (0x10 to address 0: 9 bytes and an i32 a setpoint) and polls each board after
it (0x03, 8 bytes; the reply 13: its angle and rate as i32); a frame lands at the boards its
bytes later at BAUD, 8N1, a reply as many after the poll and the board's turn (TURN_S) - a frame
of known shape dispatches on its CRC, not t3.5 (docs/PROTOCOL.md). Each board runs its PD (`physics.SERVO`) every LOCKSTEP_S on the setpoint
it has, carried on at the rate the last two came at. The threads hold the world's clock in
lockstep, a barrier a step: the emulated boards take the same place later. The host sends
nothing on a bus still busy a pass on: queued without end, the frames fell ever further behind
at 1 Mbit and she fell in 2 s. At 10 Mbit a leg's seven boards hear a pass's setpoints 37 us
after it and the host their state 0.4 ms on: the walk holds as without the wire, the pendulum's
stir 2.8 -> 3.5 mm for the millisecond's lag; at 1 Mbit a leg's polls take 1.7 ms and she falls
within a second. The threads are Python's, the GIL between them: they carry the wire's timing,
not parallel work - a limb a process over a virtual port is the next step (2026-09-27).
"""
import collections
import math
import threading

#: The link's rate, bits/s, and a board's turn from a poll to its reply, s.
BAUD, TURN_S = 10e6, 30e-6

#: A broadcast's bytes over its setpoints (address, function, start, count, byte count, CRC),
#: a setpoint's (i32), a poll's, a reply's (address, function, byte count, angle and rate as
#: i32, CRC).
FRAME_B, SETPOINT_B, POLL_B, REPLY_B = 9, 4, 8, 13

#: The boards' pace and the world's step, s.
LOCKSTEP_S = 0.001


def wire(count):
    """Seconds `count` bytes take on the wire, 8N1."""
    return count * 10.0 / BAUD


class Bus(threading.Thread):

    """One limb's RS485 bus: its boards (joint indices), the frames in flight to and from them,
    and their loops, run a lockstep at a time as the world lets it (`tick`)."""

    def __init__(self, world, link, indices):
        super().__init__(name='bus %d' % link, daemon=True)
        self.world, self.link, self.indices = world, link, list(indices)
        #: Per board: the setpoint held (rad), its rate (rad/s), when it was set (s), and
        #: whether a frame set it - the rate is read between two frames, never from a hold:
        #: from the hold at a reset to the first frame 37 us later it came to 150 rad/s and
        #: every drive slammed to its peak (2026-09-27).
        self.target = {i: 0.0 for i in self.indices}
        self.rate = {i: 0.0 for i in self.indices}
        self.set_at = {i: 0.0 for i in self.indices}
        self.framed = {i: False for i in self.indices}
        #: The host's frames landing: (at, {index: rad}); the replies landing: (at, index,
        #: degrees, rate); what the host has heard: {index: (degrees, rate, at)}.
        self.inbox = collections.deque()
        self.mail = collections.deque()
        self.heard = {i: (0.0, 0.0, 0.0) for i in self.indices}
        #: When the bus is free again, s.
        self.free_at = 0.0
        self.now, self.stop = 0.0, False

    # -- the host's side -----------------------------------------------------------------

    def send(self, at, degrees):
        """The host's setpoints {index: degrees} broadcast at `at` s: they land when the frame
        has gone over the wire, behind the polls out on it; a bus still busy a pass on gets
        nothing (dropped behind the pass's own polls, no setpoint ever reached a board)."""
        if self.free_at > at + LOCKSTEP_S:
            return
        self.free_at = max(at, self.free_at) + wire(FRAME_B + SETPOINT_B * len(degrees))
        self.inbox.append((self.free_at, {i: math.radians(v) for i, v in degrees.items()}))

    def poll(self, at):
        """The host's poll of every board at `at` s, in turn, after the broadcast: each reply
        lands when it has come back over the wire; a bus still busy gets none."""
        if self.free_at > at + LOCKSTEP_S:
            return
        start = max(at, self.free_at)
        for i in self.indices:
            start += wire(POLL_B) + TURN_S
            self.mail.append((start, i))
            start += wire(REPLY_B)
        self.free_at = start

    def reading(self, index):
        """(degrees, rate deg/s, when) of a board as the host last heard it."""
        return self.heard[index]

    # -- the boards' side ----------------------------------------------------------------

    def tick(self, now):
        """A lockstep at `now` s: the frames landed by now set the boards' targets, each
        board's PD writes its torque, the polls landed by now are answered with the board's
        state now."""
        self.now = now
        while self.inbox and self.inbox[0][0] <= now:
            at, targets = self.inbox.popleft()
            for i, v in targets.items():
                span = at - self.set_at[i]
                self.rate[i] = ((v - self.target[i]) / span
                                if self.framed[i] and 1e-9 < span < 0.1 else 0.0)
                self.target[i], self.set_at[i], self.framed[i] = v, at, True
        w = self.world
        for i in self.indices:
            ref = self.target[i] + self.rate[i] * (now - self.set_at[i])
            tau = w.gains[i, 0] * (ref - w.q[i]) + w.gains[i, 1] * (self.rate[i] - w.qd[i])
            w.ctrl[i] = max(-w.limit[i], min(w.limit[i], tau))
        while self.mail and self.mail[0][0] <= now:
            at, i = self.mail.popleft()
            self.heard[i] = (math.degrees(w.q[i]), math.degrees(w.qd[i]), at)

    def hold(self, index, radians):
        """A board holding `radians` still, as at a reset."""
        self.target[index], self.rate[index], self.set_at[index] = radians, 0.0, self.now
        self.framed[index] = False
        self.heard[index] = (math.degrees(radians), 0.0, self.now)

    # -- the thread ----------------------------------------------------------------------

    def run(self):
        """Lockstep with the world: released a step at a time (`World.advance`), the barriers
        broken when the world stops."""
        w = self.world
        try:
            while not self.stop:
                w.begun.wait()
                self.tick(w.data.time)
                w.ended.wait()
        except threading.BrokenBarrierError:
            pass
