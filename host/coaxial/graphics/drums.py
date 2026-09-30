"""Her drives as drawn: each joint's drum, and the patches sewn on the cloth over a drum's ends.

    parts += drums()                               # [(name, parent, offset, mesh)], a joint each
    sewn = patches(parts)                          # {joint: [(part, its corners near an end)]}
    masks = near(sewn, parts, drummed, dense, spans)   # the same patches among the dense dots
"""
from coaxial.graphics.lit import PLATE
from coaxial.graphics.shapes import drum
from machine import drives, figure

#: Where a drive's drum sits on its joint's segment when it is on the joint's axis, m, her left
#: side's (the right's mirrored): the hip's out at the hip's side, its roll's up in the pelvis's
#: socket; the yaw's rides the pelvis over the hip, the spine's pair and the waist's the torso's
#: foot.
DRUM_AT = {'hip': (0.02, 0.0, 0.0), 'hip_roll': (0.0, 0.035, 0.0), 'spine_roll': (0.0, 0.05, 0.0),
           'waist': (0.0, 0.1, -0.01)}
DRUM_ON_PELVIS = {'hip_yaw': 0.09}

#: Each drum's axis (unit, its part's frame) and half its length, m, by its joint - filled as the
#: drums are built.
AXES = {}

#: A patch reaches PATCH_M round a drum's end.
PATCH_M = 0.045


def drums():
    """[(name, parent, offset, mesh)]: each joint's drive's drum."""
    carries = {j: seg for seg in figure.SEGMENTS for j, _axis, _sign in seg[2]}
    axes = {j: axis for seg in figure.SEGMENTS for j, axis, _sign in seg[2]}
    out = []
    for joint, seg in carries.items():
        size = drives.of(joint)[1]
        mesh = drum(size.diameter / 2.0, size.length, axes[joint], PLATE)
        mounted = drives.mount(joint)
        x = -1.0 if joint.startswith('right_') else 1.0
        kind = drives.kind(joint)
        if mounted is not None:
            parent, (ox, oy, oz) = mounted
        elif kind in DRUM_ON_PELVIS:
            parent, (hx, hy, hz) = 'pelvis', seg[3]
            ox, oy, oz = hx * x, hy + DRUM_ON_PELVIS[kind], hz
        else:
            parent, (ox, oy, oz) = seg[0], DRUM_AT.get(kind, (0.0, 0.0, 0.0))
        AXES[joint] = ({'x': (1.0, 0.0, 0.0), 'y': (0.0, 1.0, 0.0),
                        'z': (0.0, 0.0, 1.0)}[axes[joint]], size.length / 2.0)
        out.append(('drive_' + joint, parent, (ox * x, oy, oz), mesh))
    return out


def patches(parts):
    """{joint: [(part index, corner indices)]}: each drum under the cloth, the cloth's corners
    within PATCH_M of the drum's ends, on a cloth riding the drum's own segment."""
    from coaxial.model.blocks import numpy as np      # behind the OpenBLAS cap
    out = {}
    for joint, (axis, half) in AXES.items():
        under = next((p for p in parts if p[0] == 'drive_' + joint), None)
        if under is None:
            continue
        centre = np.asarray(under[3], float)
        ends = [centre + np.asarray(axis) * half, centre - np.asarray(axis) * half]
        for i, (name, parent, _j, offset, _r, mesh) in enumerate(parts):
            if not name.startswith('cloth_') or parent != under[1]:
                continue
            corners = mesh[0] + np.asarray(offset, float)
            close = np.zeros(len(corners), bool)
            for end in ends:
                close |= np.linalg.norm(corners - end, axis=1) < PATCH_M
            if close.any():
                out.setdefault(joint, []).append((i, np.flatnonzero(close)))
    return out


def near(sewn, parts, drummed, dense, spans):
    """{joint: {part: mask}}: the patches `sewn` (`patches`) among the dots drawn without a
    card - `dense` the sampled (points, ..), `spans` each part's of them, `drummed` each drum's
    part by its joint."""
    from coaxial.model.blocks import numpy as np
    out = {}
    for joint, patched in sewn.items():
        axis, half = AXES[joint]
        centre = np.asarray(parts[drummed[joint]][3], float)
        ends = (centre + np.asarray(axis) * half, centre - np.asarray(axis) * half)
        for part, _corners in patched:
            lo, hi = spans[part]
            points = dense[0][lo:hi] + np.asarray(parts[part][3], float)
            out.setdefault(joint, {})[part] = np.min(
                [np.linalg.norm(points - end, axis=1) for end in ends], axis=0) < PATCH_M
    return out
