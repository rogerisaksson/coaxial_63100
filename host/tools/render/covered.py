"""A drive's board under a printed guard: a black honeycomb cup, or a ring, a lid and a hose's holder.

    tris = covered.cover(face, seat, board_r, band_r, outlet, window)  # the guard, about the drive's axis
    tris = covered.lid(face, board_r, band_r, chain, outlet)           # its lid, where a hose leaves it
    x0, x1, outside = covered.span(face, seat, board_r, band_r)
    pad, marks = covered.across(far, travel, board_r, band_r)          # where the bus's pipe leaves it, its lid's marks
    turn = covered.turned(pad)                                         # the board's turn under it

One print grown from its face across the axis, nothing past 45 degrees, in black - its first layers, FIRST_M of its
face, in the robot's accent (`chain.ACCENT`): a ring of it round every board -: a rim from the face on past the
motor's back, where the board's bare side lies, its wall the cage's honeycomb between solid ends (`clamped.honeycomb`),
onto the band round the motor - a solid 45-degree cone down onto that band where it is narrower than the rim, the
rim's socket ON_M over it - and a solid ring round its face. A board on no chain lies under a face of open honeycomb
inside that ring, one piece with it, turned to lay its heatsink at HEATSINK. Where the bus's hose leaves the guard
(`outlet`), the ring carries a holder for it alone - a solid pad hard by its edge (`edge`), the pipe's root in it -
and the face is a lid of its own (`lid`): an open honeycomb in a solid edge, a notch round the holder, solid over
each of the board's screw terminals (`boards.screws`) - no finger on them -, laid in the ring IN_M under its face
and snapped under NIBS nibs of the ring; the bus's mark on a pad of its own where the bus comes through the
guard's wall (`beside`, `chain.mark`) - a T-cross's body carries its own (`tees`). So the
lid comes off - lifted clear of the nibs, drawn out under the hose - with the hose still in its holder, the ring's
wall is open where the motor's phase leads come in at its side (`window`, `boards.leads`), and the
board's bus's terminals, laid hard beside the holder (`turned`, `boards.bus`), are open to a driver: the chain comes
in and goes on at the terminals themselves, no stub between. Laid with the face at the least x, the motor toward +x.
"""
import math

from tools.render import boards
from tools.render import chain as ch
from tools.render import clamped as cl
from tools.render import knee_loads as kl
from tools.render import knee_mount as km
from tools.render import mounting as m

#: The guard's wall - its face's and its rim's - the least printed wall (`knee_loads.WALL`), how far its socket runs on
#: over the band round the motor, the fit it keeps round that band, the solid ring round its face, the pads' radius
#: round a pipe's root (its collar and a strut), m; its black polymer.
WALL_M, ON_M, FIT_M, RING_M, PAD_M = 1e-3 * kl.WALL, 0.004, 0.0002, 0.004, 0.009
BLACK = (30, 31, 35)
#: The bearing a board on no chain is turned to lay its heatsink at under its guard, rad (+y toward +z): behind the
#: axis; how much of a guard's face - its first layers on the bed - takes the accent, m.
HEATSINK, FIRST_M = -0.5 * math.pi, 0.0006
#: The lid: the gap round it, how far its outside lies under the ring's face - the nibs' thickness over its edge -,
#: its solid edge's width, a nib's reach over that edge and its width, m; how many nibs hold it.
GAP_M, IN_M, EDGE_M, NIB_M, NIBS = 0.0004, 0.0006, 0.002, (0.0012, 0.004), 3


def room(board_r):
    """The guard's bore round a board `board_r` round, m: the board and its leads' room."""
    return board_r + boards.LEAD_M


def span(face, seat, board_r, band_r):
    """(x0, x1, outside) of the guard (`cover`), m."""
    bore = max(room(board_r), band_r + FIT_M)
    return face, seat + max(bore - band_r - FIT_M, 0.0) + ON_M, bore + WALL_M


def edge(board_r, band_r):
    """How far off the axis a pad's middle sits hard by the guard's edge, m: the pad within the guard's outside."""
    return max(room(board_r), band_r + FIT_M) + WALL_M - PAD_M


def across(far, travel, board_r, band_r, entry=None):
    """((y, z) m, [(kind, (y, z) shares of its bore)]): where the bus's pipe leaves the guard - its holder's pad hard
    by the guard's edge (`edge`), across the axis from a fitting `far` rad round it (+y toward +z) with the joint at
    the middle of its `travel` (from, to) deg - and the lid's marks: none where a T-cross stands on the holder, its
    body carrying them (`tees`); the way in's (`beside`) where the trunk comes through the guard's wall at `entry`
    rad and goes on by the holder."""
    turn = far + math.radians(0.5 * (travel[0] + travel[1])) - math.pi
    off = edge(board_r, band_r)
    return (off * math.cos(turn), off * math.sin(turn)), () if entry is None else beside(entry, board_r, band_r)


def beside(bearing, board_r, band_r, kinds=('in',)):
    """[(kind, (y, z) shares of its bore)]: the bus's marks on a lid where the bus passes the guard's wall at
    `bearing` rad (+y toward +z): within the lid's solid edge there, side by side round it where there are two."""
    bore = max(room(board_r), band_r + FIT_M)
    off = bore + WALL_M - RING_M - GAP_M - EDGE_M - ch.RING_M - 0.001
    apart = 2.0 * math.asin((ch.RING_M + 0.001) / off)
    turns = [bearing + (k - 0.5 * (len(kinds) - 1)) * apart for k in range(len(kinds))]
    return tuple((kind, (off * math.cos(t) / bore, off * math.sin(t) / bore)) for kind, t in zip(kinds, turns))


def turned(pad):
    """The turn, rad, that lays the board's bus's terminals (`boards.bus`: from their far end to the end the pad
    stands by) hard beside the holder's pad at `pad` (y, z), clear of the lid's notch's edge round it: open to a
    driver with the lid off."""
    far, near = boards.bus()
    clear = math.asin((PAD_M + GAP_M + EDGE_M) / math.hypot(*pad))
    return math.atan2(pad[1], pad[0]) - near - math.copysign(clear, near - far)


def _arc(inner, outer, x0, x1, at, toward, half, k=180):
    """A band round `at` (y, z) between radii from x0 to x1, open `half` rad either side of the bearing `toward`."""
    import numpy as np
    turn = km._rx(math.degrees(toward) - 180.0)
    return [(tri @ turn.T + np.array([0.0, at[0], at[1]]), colour)
            for tri, colour in m._band_x(inner, outer, x0, x1, BLACK, k=k, gap=(math.pi - half, math.pi + half))]


def _face(x0, x1, within, pads, void=None, ink=BLACK):
    """The honeycomb from x0 to x1: an open net of struts - the cage's cells (`clamped.CELL_M` across,
    `clamped.STRUT_M` struts) -, each strut's middle `within` of the axis and off `void` ((y, z), radius), and a solid
    pad `pads` [((y, z), radius)]; in `ink`."""
    import numpy as np
    tris = []
    for at, r in pads:
        tris += m._cylinder_x(r, x0, x1, ink, k=24, at=tuple(at))
    a = cl.CELL_M
    corner = a / math.sqrt(3.0)                          # a cell's corners off its middle
    turns = [np.array([math.cos(t), math.sin(t)]) * corner for t in np.radians(np.arange(30.0, 390.0, 60.0))]
    rows = int(math.ceil(within / (0.5 * math.sqrt(3.0) * a))) + 1
    for j in range(-rows, rows + 1):
        for i in range(-rows, rows + 1):
            mid = np.array([(i + 0.5 * (j % 2)) * a, j * 0.5 * math.sqrt(3.0) * a])
            for k in range(3):                           # three sides a cell: every side once
                p, q = mid + turns[k], mid + turns[k + 1]
                half = 0.5 * (p + q)
                if (np.hypot(*half) > within or any(np.hypot(*(half - at)) < r for at, r in pads)
                        or (void is not None and np.hypot(*(half - void[0])) < void[1] + 0.5 * corner)):
                    continue
                d = (q - p) / np.linalg.norm(q - p)
                w = np.array([-d[1], d[0]]) * 0.5 * cl.STRUT_M
                tris += m._prism(np.array([p + w, q + w, q - w, p - w]), x0, x1, ink, axis=0)
    return tris


def _notch(inner, outlet):
    """(the half angle a circle `inner` round the axis is open over, the half angle the notch's own edge is open over
    toward the rim): where the lid's notch - GAP_M round the holder's pad at `outlet` - meets that circle."""
    off, round_ = math.hypot(*outlet), PAD_M + GAP_M
    return (math.acos(max(-1.0, min(1.0, (off ** 2 + inner ** 2 - round_ ** 2) / (2.0 * off * inner)))),
            math.acos(max(-1.0, min(1.0, (inner ** 2 - off ** 2 - round_ ** 2) / (2.0 * off * round_)))))


def lid(face, board_r, band_r, chain=(), outlet=(0.0, 0.0), turn=None):
    """[(tri, colour)]: the guard's lid over a board `board_r` round, its outside IN_M under the ring's face at `face`
    (x): an open honeycomb in a solid edge GAP_M inside the ring, a notch GAP_M round the holder's pad at `outlet` (y,
    z) with an edge of its own, the bus's marks `chain` [(kind, (y, z) shares of its bore)] on their pads, and a
    pad over each of the board's screw terminals where the board lies turned as `turned` has it - each as wide as
    PAD_M, the lid's edge and its notch leave. With no `outlet` (None) - the bus through the guard's wall - no notch,
    its edge whole, the board turned `turn` rad."""
    import numpy as np
    bore = max(room(board_r), band_r + FIT_M)
    rim, x0, x1 = bore + WALL_M - RING_M - GAP_M, face + IN_M, face + WALL_M
    marks = [(kind, np.array(share, float) * bore) for kind, share in chain]
    told = [(at, min(PAD_M, rim - float(np.hypot(*at)))) for _kind, at in marks]
    if outlet is None:
        about = km._rx(math.degrees(turn or 0.0))[1:, 1:]
        pads = told + [
            (at, min(PAD_M, rim - float(np.hypot(*at)))) for at in (about @ np.array(q[:2]) for both in boards.screws()[:3]
                                                                    for q in both)]
        tris = m._band_x(rim - EDGE_M, rim, x0, x1, BLACK, k=180) + _face(x0, x1, rim - 0.5 * cl.CELL_M / math.sqrt(3.0), pads)
        return tris + [t for kind, (y, z) in marks for t in ch.mark(kind, (x0, y, z), (-1.0, 0.0, 0.0))]
    toward, void = math.atan2(outlet[1], outlet[0]), PAD_M + GAP_M
    open_, own = _notch(rim - 0.5 * EDGE_M, outlet)
    lain, hole = km._rx(math.degrees(turned(outlet)))[1:, 1:], np.array(outlet, float)
    shut = [lain @ np.array(q[:2]) for both in boards.screws()[:3] for q in both]
    pads = told + [
        (at, min(PAD_M, rim - float(np.hypot(*at)), float(np.hypot(*(at - hole))) - void)) for at in shut]
    tris = (_arc(rim - EDGE_M, rim, x0, x1, (0.0, 0.0), toward, open_)
            + _arc(void, void + EDGE_M, x0, x1, outlet, toward, own)
            + _face(x0, x1, rim - 0.5 * cl.CELL_M / math.sqrt(3.0), pads, (hole, void)))   # no strut's end past the
    # lid's edge or in its notch
    return tris + [t for kind, (y, z) in marks for t in ch.mark(kind, (x0, y, z), (-1.0, 0.0, 0.0))]


def cover(face, seat, board_r, band_r, outlet=None, window=None, lidded=False):
    """[(tri, colour)]: the guard from its face at `face` (x) to the motor's back at `seat`, on over the band round the
    motor `band_r` round outside, over a board `board_r` round, about the x axis: its face an open honeycomb, or -
    where a pipe leaves it at `outlet` (y, z) m - its ring alone, the hose's holder's pad on it and the nibs its lid
    (`lid`) snaps under; its wall under that ring open over `window` (mid, half) rad, where leads come in at its
    side. `lidded` with no outlet: the ring and its lid's nibs alone, the bus through its wall."""
    import numpy as np
    bore = max(room(board_r), band_r + FIT_M)
    outside = bore + WALL_M
    seat_r = band_r + FIT_M
    stations = [(seat, bore, outside)]
    if seat_r < bore - 1e-6:                             # a cone down onto the narrower band, at 45 degrees
        stations.append((seat + bore - seat_r, seat_r, max(seat_r + WALL_M, outside - (bore - seat_r))))
    stations.append((stations[-1][0] + ON_M, seat_r, seat_r + WALL_M))
    if window is None:
        wall = cl.honeycomb(bore, outside, face + WALL_M - cl.RIM_M, seat + cl.RIM_M, BLACK) + cl.turned(stations, None, BLACK)
    else:                                                # laid open about -y, then turned to where the leads are
        mid, half = window
        turn = km._rx(math.degrees(mid) - 180.0)
        wall = [(tri @ turn.T, colour) for tri, colour in cl.honeycomb(
            bore, outside, face + WALL_M - cl.RIM_M, seat + cl.RIM_M, BLACK, (math.pi - half, math.pi + half))
            + cl.turned(stations, math.pi, BLACK, gap=2.0 * half)]
    first = face + FIRST_M                               # its first layers on the bed: the accent's
    tris = wall + m._band_x(outside - RING_M, outside, first, face + WALL_M, BLACK)
    tris += m._band_x(outside - RING_M, outside, face, first, ch.ACCENT)
    if outlet is None and not lidded:
        return (tris + _face(face, first, outside - 0.5 * RING_M, [], ink=ch.ACCENT)
                + _face(first, face + WALL_M, outside - 0.5 * RING_M, []))
    if outlet is not None:                               # the hose's holder
        tris += (m._cylinder_x(PAD_M, face, first, ch.ACCENT, k=24, at=outlet)
                 + m._cylinder_x(PAD_M, first, face + WALL_M, BLACK, k=24, at=outlet))
    outlet = outlet or (-1.0, 0.0)
    reach, wide = NIB_M
    nib = np.array([(outside - RING_M - reach, -0.5 * wide), (outside - RING_M + reach, -0.5 * wide),
                    (outside - RING_M + reach, 0.5 * wide), (outside - RING_M - reach, 0.5 * wide)])
    for k in range(NIBS):                                # the lid's nibs, the first across the axis from the holder
        t = math.atan2(outlet[1], outlet[0]) + math.pi + k * 2.0 * math.pi / NIBS
        turn = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
        tris += m._prism(nib @ turn.T, face, face + IN_M - 0.25 * GAP_M, ch.ACCENT, axis=0)    # in those layers too
    return tris
