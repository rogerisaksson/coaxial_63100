"""The robot's bare look: printable panels over its skeleton, the joints left open.

Each limb's shell a panel of carbon fibre, its ends open and dark at the joints where the
skeleton, the drives and the rods show, the neck and the waist open too.

    meshes = panels.over(meshes)        # {segment: mesh} with their shells' corners re-painted
"""
from coaxial.graphics.lit import paint
from machine.body.figure import FOREARM, SHANK, THIGH, UPPER_ARM

#: The open joints' dark, the skeleton under the panels; a panel's carbon fibre (`lit.TWILL`),
#: graphite under its clear coat.
OPEN, SHELL = paint((38, 40, 46)), paint((84, 92, 112), twill=True)

#: Each limb's length and how far its panel stops short of its joint at the top and the bottom,
#: m - a corner past them is open.
LIMBS = {'thigh': (THIGH, 0.02, 0.05), 'shank': (SHANK, 0.04, 0.05),
         'upper_arm': (UPPER_ARM, 0.03, 0.04), 'forearm': (FOREARM, 0.03, 0.03)}

#: The torso's open waist, its frame's y, m: the spine's drives between the pelvis and the ribs.
WAIST_Y = 0.04


def _limb(corners, length, top, bottom):
    from coaxial.model.blocks import numpy as np
    y = corners[:, 1]
    return np.where((y > -top) | (y < bottom - length), OPEN, SHELL)


def over(meshes):
    """`meshes` {segment: mesh} with their shells as panels: the limbs', the torso's and the
    neck's corners re-painted, the rest as drawn."""
    from coaxial.model.blocks import numpy as np
    out = dict(meshes)
    for name, (corners, tris, uv, materials) in meshes.items():
        part = name.split('_', 1)[1] if name.startswith(('left_', 'right_')) else name
        if part in LIMBS:
            painted = _limb(corners, *LIMBS[part])
        elif part == 'torso':
            painted = np.where(corners[:, 1] < WAIST_Y, OPEN, materials)
        elif part == 'neck':
            painted = np.full(len(corners), OPEN)
        else:
            continue
        out[name] = (corners, tris, uv, painted)
    return out
