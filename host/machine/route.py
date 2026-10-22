"""Her way round the furniture: what stands on her floor grown by her radius, A* through the rest.

    cells = route.blocked(g)                     # her map's cells she may not stand in
    corners = route.way(g, to, cells)            # [(x, z)] from where she stands round them, `to` last
    d = route.staged((x, z), out, most, cells)   # m out along a dock's normal she stands free at
    specs = route.docks()                        # how each errand docks
"""
import math

from machine import figure, room, se2, sight, stride

#: Her map's things grown by her radius and a cell: setting off she walks while her heading is CONE
#: off her leg at most, her walk's arc (WALK_SPEED / YAW_RATE round) a cell off it at most.
GROWN_M = sight.RADIUS_M + sight.CELL_M
CONE = math.acos(1.0 - sight.CELL_M * se2.YAW_RATE / se2.WALK_SPEED)


def docks():
    """How each errand docks, from her figure's, her stride's and the room's dimensions: kind ->
    (the dock on the prop's edge from its centre, m; her heading docked, rad; how far along the
    edge her hips may land off it, m; where she stops before it - her walk's stopping distance
    and the stride a turn in place drifts her -, m; the prop's place failing one found)."""
    d_stage = se2.STOP_M + stride.STRIDE_M
    return {
        'sit_chair': (0.0, -room.SEAT[2], math.pi, room.SEAT[0] - figure.HIP_HALF, d_stage, room.CHAIR_AT),
        'lie_bed': (room.BED[0], 0.0, math.pi / 2.0, room.BED[2] - figure.HIP_HALF, d_stage, room.BED_AT),
        'exit_door': (0.0, 0.0, 0.0, 0.5 * (room.DOOR_X[1] - room.DOOR_X[0]), 4.0 * stride.STRIDE_M,
                      (0.5 * (room.DOOR_X[0] + room.DOOR_X[1]), room.TO)),
    }


def _cell(x, z):
    return (int(math.floor((x - sight.X_M[0]) / sight.CELL_M)), int(math.floor((z - sight.Z_M[0]) / sight.CELL_M)))


def _centre(i, j):
    return (sight.X_M[0] + (i + 0.5) * sight.CELL_M, sight.Z_M[0] + (j + 0.5) * sight.CELL_M)


def _free(cells, cell):
    return 0 <= cell[0] < cells.shape[0] and 0 <= cell[1] < cells.shape[1] and not cells[cell]


def blocked(g):
    """Her map's cells (`sight.Map`'s) she may not stand in: each thing over her ankle and under the
    doorway's top - she steps over lower, walks under higher, pushes the door aside -, GROWN_M round."""
    np = sight._np()
    nx = int(round((sight.X_M[1] - sight.X_M[0]) / sight.CELL_M))
    nz = int(round((sight.Z_M[1] - sight.Z_M[0]) / sight.CELL_M))
    x, z = np.meshgrid(sight.X_M[0] + (np.arange(nx) + 0.5) * sight.CELL_M,
                       sight.Z_M[0] + (np.arange(nz) + 0.5) * sight.CELL_M, indexing='ij')
    out = np.zeros((nx, nz), dtype=bool)
    for p in g['world'].props():
        if len(p) < 4 or p[0] == 'door' or len(p[1]) < 3 or not hasattr(p[2], '__len__') or len(p[2]) < 3:
            continue
        _kind, c, half, turn = p[:4]
        if c[1] + half[1] < figure.ANKLE_H or c[1] - half[1] >= 2.0 * room.DOOR[1]:
            continue
        dx, dz = x - c[0], z - c[2]
        along_x = np.abs(dx * turn[0][0] + dz * turn[2][0])
        along_z = np.abs(dx * turn[0][2] + dz * turn[2][2])
        out |= (along_x <= half[0] + GROWN_M) & (along_z <= half[2] + GROWN_M)
    return out


def way(g, to, cells):
    """[(x, z)] her corners from where she stands to `to` through `cells`' free ones, `to` last; [`to`]
    where no way is found or either end is off her map."""
    d = g['world'].data
    start, goal = _cell(float(d.qpos[0]), float(d.qpos[2])), _cell(*to)
    if not all(0 <= c[0] < cells.shape[0] and 0 <= c[1] < cells.shape[1] for c in (start, goal)):
        return [tuple(to)]
    start = start if not cells[start] else sight._nearest_free(cells, start)
    goal = goal if not cells[goal] else sight._nearest_free(cells, goal)
    path = sight._astar(cells, start, goal)
    return [_centre(*c) for c in sight._pruned(cells, path)[1:-1]] + [tuple(to)] if path else [tuple(to)]


def staged(dock, out, most, cells):
    """m along `out` (a dock's normal, unit (x, z)) from `dock` she stands free at: the middle of the
    first run of free cells along it, `most` at most - searched to twice `most`, so in the open it is
    `most`."""
    steps = [k * sight.CELL_M for k in range(int(round(2.0 * most / sight.CELL_M)) + 1)]
    free = [_free(cells, _cell(dock[0] + s * out[0], dock[1] + s * out[1])) for s in steps]
    k0 = next((k for k, f in enumerate(free) if f), None)
    if k0 is None:
        return most
    k1 = next((k for k in range(k0, len(free)) if not free[k]), len(free)) - 1
    return min(most, 0.5 * (steps[k0] + steps[k1]))
