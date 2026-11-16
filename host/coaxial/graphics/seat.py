"""The robot's seat as drawn bare: a plate a side over its buttocks, sliding with the thigh.

    parts += [('left_seat', 'pelvis', (), z0, 0.0, seat.plate(1.0)), ('right_seat', ..., seat.plate(-1.0))]
    bends = drape.table(parts, corners, dense, blends + seat.BEND, given)   # the lower part with the thigh

A plate a side where the robot sits: over the round back of its buttock (`figure.SEAT_R`, `SEAT_X`, `SEAT_Y`,
`SEAT_Z`: the capsules the floor, a chair and a bed meet), under the hip's roll drive and between the pitch
drives. The plate's face stands IN inside the buttock at its outline and FLAT of the way from the buttock's round
to the plane through its outline within it - on the round alone it read as a cup under the drive -, the guards'
foam (`machine.body.guards.THICK_M`) thick; its outline is a box of the power POWER over A deg down from straight
back and B deg out from the buttock's middle, a rim RIM of it in the guards' frame, a hole HOLE of it through it,
DROP of its half height under its middle, and a rivet in each corner; the cleft between the two plates is open.
The plate's lower part goes with the thigh (BEND, `drape`): from BEND's first height down a corner takes its share
of the hip's turns, the whole of them at its second, about the buttock's middle - it slides on the buttock, the
contact it stands in.
"""
import math

from coaxial.graphics.guards import CORNER, PROUD_M, RIVET_R
from coaxial.graphics.livery import BOLT, FOAM, FRAME, PAD
from machine.body import figure
from machine.body.guards import THICK_M

#: Turned about the hip, the plate stood 45 mm out of the robot's capsules at the hip 40 deg back; a hip's yaw to
#: its stops, its roll 30 deg either way and its pitch from 30 deg back to 120 forward keep it 1.4 mm within them
#: and no part nearer than 9.8 mm (16 mm at rest; from 10 deg down its top met the hip's roll ring at 5 mm). Drawn
#: as the buttocks' whole round backs the seat read as two bowls; on struts behind them, as guards, the robot sat on
#: its cheeks in place of its buttocks and tipped onto its back scooting on the floor.
IN, POWER, A, B, RIM, HOLE = 0.004, 3.0, (16.0, 95.0), (-42.0, 36.0), 0.14, 0.3
BEND = (('{side}_seat', '{side}_thigh', -0.09, -0.19, (figure.SEAT_X, figure.SEAT_Y, figure.SEAT_Z[0])),)
AROUND, RINGS, DROP, FLAT = 32, 4, 0.35, 0.65


def plate(side):
    """A side's seat plate's mesh, the pelvis's frame, `side` 1 the left's: its face and its rim out, its back in,
    its walls round its outline and its hole, its rivets proud of its rim; each triangle facing out."""
    from coaxial.model.blocks import numpy as np
    centre = np.array([side * figure.SEAT_X, figure.SEAT_Y, figure.SEAT_Z[0]])
    face = figure.SEAT_R - IN
    t = np.arange(AROUND) * (2.0 * math.pi / AROUND)
    u, v = (np.sign(f(t)) * np.abs(f(t)) ** (2.0 / POWER) for f in (np.cos, np.sin))
    (am, ah), (bm, bh) = ((math.radians(lo + hi) / 2.0, math.radians(hi - lo) / 2.0) for lo, hi in (A, B))

    def way(a, b):
        """The way `a` rad down from straight back and `b` out from the buttock's middle."""
        return np.stack([side * np.sin(b), -np.cos(b) * np.sin(a), -np.cos(b) * np.cos(a)], -1)
    mid = way(np.array(am), np.array(bm))
    edge = float((way(am + ah * v, bm + bh * u) @ mid).min())

    def at(a, b, depth):
        """The plate's surface `depth` under its face (less, proud of it) toward `a`, `b`: FLAT of the way from the
        buttock's round back to the plane through the plate's outline there, each its share."""
        d = way(a, b)
        r = face + FLAT * (face * edge / (d @ mid) - face)
        normal = (1.0 - FLAT) * d + FLAT * mid
        normal = normal / np.linalg.norm(normal, axis=-1)[..., None]
        return centre + r[..., None] * d - depth * normal

    def ring(scale, depth):
        """A ring `scale` of the outline about a middle that sinks from the plate's to its hole's (DROP)."""
        sink = DROP * ah * np.clip((1.0 - RIM - scale) / (1.0 - RIM - HOLE), 0.0, 1.0)
        return at(am + sink + ah * scale * v, bm + bh * scale * u, depth)

    def strip(first, second, ink, away):
        """Triangles between two rings, each facing `away` (a corner's way out of the plate)."""
        k = np.arange(AROUND)
        c = np.vstack([first, second])
        tri = np.vstack([np.stack([k, k + AROUND, (k + 1) % AROUND], 1),
                         np.stack([(k + 1) % AROUND, k + AROUND, (k + 1) % AROUND + AROUND], 1)])
        n = np.cross(c[tri[:, 1]] - c[tri[:, 0]], c[tri[:, 2]] - c[tri[:, 0]])
        flip = (n * away(c[tri].mean(axis=1))).sum(axis=1) < 0.0
        tri[flip] = tri[flip][:, ::-1]
        return c, tri, ink

    middle = at(np.array(am), np.array(bm), 0.0)

    def outward(p):
        return np.broadcast_to(mid, p.shape)

    def aside(p):
        d = p - middle
        return d - (d @ mid)[:, None] * mid
    pad = np.linspace(1.0 - RIM, HOLE, RINGS)
    pieces = [strip(ring(1.0, 0.0), ring(1.0 - RIM, 0.0), FRAME, outward)]
    pieces += [strip(ring(s0, 0.0), ring(s1, 0.0), PAD, outward) for s0, s1 in zip(pad, pad[1:])]
    rows = np.r_[1.0, pad]
    pieces += [strip(ring(s0, THICK_M), ring(s1, THICK_M), FOAM, lambda p: -outward(p)) for s0, s1 in zip(rows, rows[1:])]
    pieces += [strip(ring(1.0, 0.0), ring(1.0, THICK_M), FRAME, aside),
               strip(ring(HOLE, 0.0), ring(HOLE, THICK_M), FRAME, lambda p: -aside(p))]
    cs, ts, ms, base = [], [], [], 0
    for c, tri, ink in pieces:
        cs.append(c)
        ts.append(tri + base)
        ms.append(np.full(len(c), ink))
        base += len(c)
    rim = 1.0 - RIM / 2.0
    turn = np.arange(6) * (math.pi / 3.0)
    for du, dv in ((-1.0, -1.0), (-1.0, 1.0), (1.0, -1.0), (1.0, 1.0)):
        a0, b0 = am + ah * rim * dv * CORNER, bm + bh * rim * du * CORNER
        hub = at(np.array(a0), np.array(b0), -PROUD_M)[None]
        hexagon = at(a0 + RIVET_R / face * np.cos(turn), b0 + RIVET_R / face * np.sin(turn), -PROUD_M)
        c = np.vstack([hub, hexagon])
        tri = np.array([(0, 1 + k, 1 + (k + 1) % 6) for k in range(6)])
        n = np.cross(c[tri[:, 1]] - c[tri[:, 0]], c[tri[:, 2]] - c[tri[:, 0]])
        flip = (n @ mid) < 0.0
        tri[flip] = tri[flip][:, ::-1]
        cs.append(c)
        ts.append(tri + base)
        ms.append(np.full(len(c), BOLT))
        base += len(c)
    corners = np.vstack(cs)
    return corners, np.vstack(ts), np.zeros((len(corners), 2)), np.concatenate(ms)
