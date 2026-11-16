"""A figure's file read: an OBJ's corners and its groups' triangles, a corner's paint off its maps.

    v, groups, at = wavefront.read('FIGURE.obj')        # m, [(names, triangles)], a corner's place on its map
    got = wavefront.painted(path, names, at, triangles)     # (a corner's paint, the triangles its map shows)

A group's map is the picture under the file's folder named as its material and `_diffuse`. Read
MAP a side: a corner's paint, the dots drawn 5-20 mm apart. A map's alpha is its cut-out - a
hair card's margin is clear - and a picture named `_opacity` beside it a copy of it: read as
lightness, hair came out grey.
"""
import os

from coaxial.graphics.lit import PAINTED

#: A map read this many pixels a side.
MAP = 256


def read(path):
    """(corners (n, 3) m, [(names, triangles (k, 3))], each corner's place on its map (n, 2), nan
    with none) of an OBJ: `names` a group's and a material's in lower case, its faces fanned to
    triangles; centimetres read as such."""
    from coaxial.model.blocks import numpy as np
    corners, spots, faces, mapped, group, material = [], [], {}, {}, '', ''
    with open(path, encoding='utf-8', errors='replace') as lines:
        for line in lines:
            if line.startswith('v '):
                corners.append(line.split()[1:4])
            elif line.startswith('vt '):
                spots.append(line.split()[1:3])
            elif line.startswith('g '):
                group = line[2:].strip().lower()
            elif line.startswith('usemtl '):
                material = line[7:].strip().lower()
            elif line.startswith('f '):
                tokens = [(token + '//').split('/') for token in line.split()[1:]]
                ids = [int(token[0]) - 1 for token in tokens]
                mapped.update((i, int(token[1]) - 1) for i, token in zip(ids, tokens) if token[1] and i not in mapped)
                faces.setdefault(group + ' ' + material, []).extend(
                    (ids[0], b, c) for b, c in zip(ids[1:], ids[2:]))
    v, at = np.array(corners, float), np.full((len(corners), 2), np.nan)
    if mapped:
        at[list(mapped)] = np.array(spots, float)[list(mapped.values())]
    return v * (0.01 if np.ptp(v[:, 1]) > 10.0 else 1.0), [(names, np.array(t)) for names, t in faces.items()], at


def painted(path, names, at, triangles):
    """(each corner's paint, the `triangles` there are) off the diffuse map of the group `names` -
    the picture under `path`'s folder named as its material, read MAP a side - at each corner's
    place on it (`at`): what its alpha leaves clear in the rest's mean, a triangle clear at its
    corners and its middle - a hair card's margin - not there; None with no map."""
    from coaxial.model.blocks import numpy as np
    file = next((os.path.join(folder, name) for folder, _dirs, files in os.walk(os.path.dirname(path))
                 for name in files if name.lower().startswith(names.split(' ')[-1] + '_diffuse.')), None)
    if file is None or not np.isfinite(at).any():
        return None
    from PIL import Image
    with Image.open(file) as picture:
        rgba = np.asarray(picture.convert('RGBA').resize((MAP, MAP)), np.int64)
    solid = rgba[..., 3] > 127
    rgb = np.where(solid[..., None], rgba[..., :3], rgba[..., :3][solid].mean(axis=0).astype(np.int64))

    def told(pixels, spot):
        spot = np.nan_to_num(spot) % 1.0
        return pixels[((1.0 - spot[..., 1]) * (MAP - 1)).astype(int), (spot[..., 0] * (MAP - 1)).astype(int)]
    rgb = told(rgb, at)
    there = (told(rgba[..., 3], at[triangles]).sum(axis=1) + told(rgba[..., 3], at[triangles].mean(axis=1))) > 2 * 255
    return PAINTED | rgb[:, 0] << 16 | rgb[:, 1] << 8 | rgb[:, 2], triangles[there]
