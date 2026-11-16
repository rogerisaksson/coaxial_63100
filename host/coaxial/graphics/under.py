"""What covers what on the robot drawn: a drive's cover in what it stands in, no shell under clothes.

A drive's cover wears what its middle stands deepest in at rest: a garment's cloth where one is
worn there, else the shell's paint there - a joint's dark, a panel's plate -, so it reads as a
swell of what it stands in. Dressed, a segment a garment rides shows its shell only past a
garment's end that a segment no garment rides hangs beyond - the arm under the sleeve's hem, the
neck's root over the collar. Drawn under bent cloth, the rigid shell stood through it: the
cheeks 145 and the pelvis 105 cm2 at a hip bent 120 deg, the torso 176 at a spine bent 85.

    parts = under.covered(parts, frames)           # a body's, `frames` its parts' at rest
    parts = under.dressed(parts, frames, worn)     # the dressed body's
"""
from coaxial.graphics.shapes import excess, ringed
from machine.body import figure


def _lofts(parts, cloth):
    """[(part's index, its rings)] of the drawn lofts among `parts`: its garments' (`cloth`) or
    its shell's, no drive's cover."""
    return [(k, through) for k, part in enumerate(parts)
            if part[0].startswith('cloth_') == cloth and not part[0].startswith('drive_') and len(part[5][1])
            for through in [ringed(part[5])] if through is not None]


def covered(parts, frames):
    """`parts` [(name, parent, joints, offset, rest, mesh)] with each drive's cover in the paint of
    what its middle stands deepest in, `frames` each part's (turn, spot) at rest: a garment before
    the shell, the shell's paint its corner's nearest that middle; a cover in neither its own."""
    from coaxial.model.blocks import numpy as np
    out, worn_then_shell = list(parts), (_lofts(parts, True), _lofts(parts, False))
    for k, (name, parent, joints, offset, rest, (corners, triangles, uv, materials)) in enumerate(parts):
        if not name.startswith('drive_'):
            continue
        for lofts in worn_then_shell:
            held = [(float(excess(at[None], through)[0]), c, at) for c, through in lofts
                    for at in [(frames[k][1] - frames[c][1]) @ frames[c][0]]]
            reach, c, at = min(held, default=(0.0, k, None))
            if reach < 0.0:
                near = int(np.argmin(np.linalg.norm(parts[c][5][0] - at, axis=1)))
                materials = np.full(len(corners), parts[c][5][3][near])
                break
        out[k] = (name, parent, joints, offset, rest, (corners, triangles, uv, materials))
    return out


def dressed(parts, frames, worn):
    """`parts` as the robot is seen dressed, `worn(name)` whether a part is worn: of a segment a
    garment rides - one hung loose on joints of its own too, the shin stood 70 cm2 through the
    jeans' leg that lies on it - the triangles with a corner past a bare end kept."""
    from coaxial.model.blocks import numpy as np
    names, up = [part[0] for part in parts], {seg[0]: seg[1] for seg in figure.SEGMENTS}
    on = {}
    for c, through in _lofts(parts, True):
        on.setdefault(parts[c][1], []).append((c, through))
    # A segment's joints to the segments no garment rides, where each hangs, at rest.
    bare = {seg: [frames[names.index(n if up[n] == seg else seg)][1] for n in up
                  if n not in on and (up[n] == seg or up[seg] == n)] for seg in on}
    out = list(parts)
    for k, (name, parent, joints, offset, rest, (corners, triangles, uv, materials)) in enumerate(parts):
        seg = name if name in up else parent
        if worn(name) or name.startswith('drive_') or seg not in on or not len(triangles):
            continue
        world, shown = corners @ frames[k][0].T + frames[k][1], np.zeros(len(corners), bool)
        for joint in bare[seg]:
            past = np.ones(len(corners), bool)
            for c, (axis, rows) in on[seg]:
                t, s = frames[c]
                at, hangs = ((world - s) @ t)[:, axis], ((joint - s) @ t)[axis]
                past &= (at > rows[-1][0]) & (hangs > rows[-1][0]) | (at < rows[0][0]) & (hangs < rows[0][0])
            shown |= past
        out[k] = (name, parent, joints, offset, rest, (corners, triangles[shown[triangles].any(axis=1)], uv, materials))
    return out
