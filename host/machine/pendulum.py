"""The virtual pendulum between her inner ears: how smoothly she goes, as one number.

Pivoted between her ears, a bob of her weight hung below, between her legs: a model only, moved by
her head. Walking smoothly it floats along, hanging still and pulling her weight; her head's
surge, sway and bob set it going.

    pendulum = Pendulum()
    pendulum.read(bus, dt)            # each pass, from what the loop read
    pendulum.stir                     # its energy as a height, mm: how rough her walk is, one number
    pendulum.stirs                    # the same fore, across and up
    pendulum.energy                   # the same this pass, unmeaned
    pendulum.swing                    # (fore, side) degrees from plumb, the bob ahead, to her left
    pendulum.felt                     # its pull over her weight: 1 hanging still
    pendulum.off, pendulum.drift      # the bob off its rest and moving from it: (on, across, up)

Its string stretches to its length under her weight - a spring of no length of its own - so the
bob swings with a pendulum's period and bobs with the same one: it hears the head alike every way,
no weights between them. `stir` is its energy about hanging still, meaned over STIR_S, over her
weight: the height it would lift her.
"""
import math

from machine import figure
from machine.gait import HIP_DROP, THIGH

#: The head's chain from the pelvis: the torso, the neck, the head (`figure.SEGMENTS`' first).
CHAIN = figure.SEGMENTS[1:4]

#: Between the inner ears, the head's frame: level with its centre, on its axis.
EARS = (0.0, CHAIN[-1][6][1], 0.0)

#: The string, m: from the ears to her knees' height, standing - the bob between her legs.
LENGTH_M = sum(seg[3][1] for seg in CHAIN) + EARS[1] + HIP_DROP + THIGH

#: From the spine's joint to the ears, m: how far the head goes as the spine turns.
SPINE_TO_EARS_M = CHAIN[1][3][1] + CHAIN[2][3][1] + EARS[1]

#: Its motion dies e-fold in SETTLE_S, s; `stir` is meaned over STIR_S, s.
SETTLE_S, STIR_S = 1.0, 2.0

G = 9.81


def ears(degrees, pelvis, turn):
    """The point between her inner ears, world, for the pelvis at `pelvis` turned `turn` and the
    spine's, neck's and head's joints at {joint: degrees}."""
    at, here = pelvis, turn
    for _name, _parent, joints, offset, _rest, *_mass in CHAIN:
        at = figure.add(at, figure.apply(here, offset))
        for joint, axis, sign in joints:
            here = figure.mul(here, figure.AXES[axis](sign * math.radians(degrees.get(joint, 0.0))))
    return figure.add(at, figure.apply(here, EARS))


class Pendulum:

    """The pendulum, stepped each pass with its pivot where her ears are."""

    def __init__(self, length=LENGTH_M, mass=figure.MASS_KG):
        self.length, self.mass, self.w2 = length, mass, G / length
        self.pivot = self.at = self.velocity = None
        self.swing, self.felt, self.stir, self.stirs = (0.0, 0.0), 1.0, 0.0, (0.0, 0.0, 0.0)
        self.energy = 0.0
        self.off, self.drift = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

    def read(self, bus, dt):
        """On by `dt` with its pivot where the loop read her ears."""
        pivot = ears({j: bus.get(j + '.deg', 0.0) for _n, _p, joints, *_r in CHAIN
                      for j, _a, _s in joints},
                     (bus['pelvis.pose.x'], bus['pelvis.pose.y'], bus['pelvis.pose.z']),
                     figure.quat(bus['pelvis.pose.qw'], bus['pelvis.pose.qx'],
                                 bus['pelvis.pose.qy'], bus['pelvis.pose.qz']))
        self.step(pivot, dt)

    def step(self, pivot, dt):
        """On by `dt`, its pivot now at `pivot`: the string pulls the bob toward it, gravity down,
        its motion about the pivot's damped."""
        if self.pivot is None or dt <= 0.0:
            self.pivot = pivot
            return
        moving = tuple((a - b) / dt for a, b in zip(pivot, self.pivot))
        if self.at is None or self.velocity is None:
            # Hung still under the pivot, moving with it.
            self.at, self.velocity = figure.add(pivot, (0.0, -self.length, 0.0)), moving
            self.pivot = pivot
            return
        damp = 2.0 / SETTLE_S
        pull = tuple(-self.w2 * (b - p) for b, p in zip(self.at, self.pivot))
        self.velocity = tuple(v + dt * (f - damp * (v - m) - (G if k == 1 else 0.0))
                              for k, (v, f, m) in enumerate(zip(self.velocity, pull, moving)))
        self.at = tuple(a + dt * v for a, v in zip(self.at, self.velocity))
        self.pivot = pivot
        hang = figure.sub(self.at, pivot)
        stretch = math.sqrt(sum(c * c for c in hang))
        self.swing = (math.degrees(math.atan2(hang[2], -hang[1])),
                      math.degrees(math.atan2(hang[0], -hang[1])))
        self.felt = stretch / self.length
        # About hanging still: moved off its rest under the pivot, and moving against it.
        off = (hang[0], hang[1] + self.length, hang[2])
        rel = tuple(v - m for v, m in zip(self.velocity, moving))
        self.off, self.drift = (off[2], off[0], off[1]), (rel[2], rel[0], rel[1])
        now = [0.5 * (r * r + self.w2 * o * o) / G for r, o in zip(rel, off)]
        self.energy = 1000.0 * sum(now)
        k = min(1.0, dt / STIR_S)
        self.stirs = tuple(s + (1000.0 * n - s) * k
                           for s, n in zip(self.stirs, (now[2], now[0], now[1])))
        self.stir = sum(self.stirs)

    @property
    def pull(self):
        """The string's pull, N."""
        return self.felt * self.mass * G
