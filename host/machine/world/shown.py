"""The floor's things as the robot's page shows them: what lies on it, where the mattress gives.

    shown.props(world)          # [(kind, centre, half sizes, turn)] of the floor's events, chairs, bed and slopes
    shown.dents(world, np)      # [(x, y, z, depth)] each contact in the mattress
"""
from machine.world.floor import HOLE_M, PARKED_M, RUG_LONG_M, RUG_M, SHOWN_Y, flight


def _ids(world):
    """The props' ids, looked up once."""
    ids = getattr(world, '_prop_ids', None)
    if ids is None:
        m = world.model
        events = [('stairs' if n.startswith('step') else n.split('_')[0], n, m.body(n).id)
                  for n in ('sill', 'slip', 'brick_left', 'brick_right', 'board', 'wall', 'tree',
                            'ice', 'snow') + tuple(n for n, _h, _a in flight())]
        ids = world._prop_ids = {
            'slabs': (m.geom('slab_a').id, m.geom('slab_b').id),
            'events': [(kind, b, m.body_geomadr[b], name in ('board', 'wall', 'tree') or name.startswith('step'))
                       for kind, name, b in events],
            'rug': m.body('rug').id,
            'chair': (m.body('chair').id, m.geom('chair_seat').id, m.geom('chair_back').id),
            'chair_low': (m.body('chair_low').id, m.geom('chair_low_seat').id, m.geom('chair_low_back').id),
            'bed': (m.body('prop_bed').id, m.geom('bed_mattress').id, m.geom('bed_headboard').id),
            'slopes': [(m.body(n).id, m.geom(n).id) for n in ('slope_ramp', 'slope_top')]}
    return ids


def dents(world, np):
    """[(x, y, z, m)]: each contact in the mattress, world, and how deep."""
    d, mat, ncon = world.data, _ids(world)['bed'][1], world.data.ncon
    if not ncon:
        return []
    geom, dist = d.contact.geom[:ncon], d.contact.dist[:ncon]
    hit = np.flatnonzero((geom == mat).any(axis=1) & (dist < 0.0))
    pos = d.contact.pos[:ncon]
    return [tuple(float(v) for v in pos[i]) + (-float(dist[i]),) for i in hit]


def props(world):
    m, d, out = world.model, world.data, []
    ids = _ids(world)
    a, b = ids['slabs']
    gap = (m.geom_pos[a][2] + m.geom_size[a][2], m.geom_pos[b][2] - m.geom_size[b][2])
    if gap[1] - gap[0] > 1e-6 and getattr(world, 'hole_x', 0.0) is not None:
        out.append(('hole', (getattr(world, 'hole_x', 0.0), -HOLE_M / 2.0, float(sum(gap) / 2.0)),
                    (0.5, HOLE_M / 2.0, float((gap[1] - gap[0]) / 2.0)), ((1, 0, 0), (0, 1, 0), (0, 0, 1))))
    for kind, body, g, whole in ids['events']:
        if d.xpos[body][1] > SHOWN_Y:
            c, half = d.geom_xpos[g].tolist(), m.geom_size[g].tolist()
            top = c[1] + half[1]
            low = c[1] - half[1] if whole else max(0.0, c[1] - half[1])
            out.append((kind, (c[0], (top + low) / 2.0, c[2]), (half[0], (top - low) / 2.0, half[2] or half[0]),
                        tuple(map(tuple, d.xmat[body].reshape(3, 3).tolist()))))
    rug_id = ids['rug']
    if d.xpos[rug_id][2] > PARKED_M + 1.0:
        out.append(('rug', tuple(d.xpos[rug_id].tolist()), (0.3, RUG_M / 2.0, RUG_LONG_M / 2.0),
                    tuple(map(tuple, d.xmat[rug_id].reshape(3, 3).tolist()))))
    for chair_id, seat, back in (ids['chair'], ids['chair_low']):
        if d.xpos[chair_id][1] > SHOWN_Y:
            c_mat = tuple(map(tuple, d.xmat[chair_id].reshape(3, 3).tolist()))
            for g in (seat, back):
                out.append(('chair', tuple(d.geom_xpos[g].tolist()), tuple(m.geom_size[g].tolist()), c_mat))
    bed_id, g_mat, g_hb = ids['bed']
    if d.xpos[bed_id][1] > SHOWN_Y:
        b_mat = tuple(map(tuple, d.xmat[bed_id].reshape(3, 3).tolist()))
        out.append(('bed', tuple(d.geom_xpos[g_mat].tolist()), tuple(m.geom_size[g_mat].tolist()), b_mat))
        if d.xpos[bed_id][1] > -0.1:
            out.append(('bed', tuple(d.geom_xpos[g_hb].tolist()), tuple(m.geom_size[g_hb].tolist()), b_mat))
    for bid, g in ids['slopes']:
        if d.xpos[bid][1] > SHOWN_Y:
            out.append(('slope', tuple(d.geom_xpos[g].tolist()), tuple(m.geom_size[g].tolist()),
                        tuple(map(tuple, d.xmat[bid].reshape(3, 3).tolist()))))
    return out
