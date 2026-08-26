"""three.js AsciiEffect, ported to run without a browser.

From `AndrewSink/STL-to-ASCII-Generator` (script.js) and the AsciiEffect it
drives. What is taken verbatim, because it is what makes the look:

  * the ramp ``' .:-+*=%@#'`` - ten characters, space darkest;
  * ``floor((1 - brightness) * (len - 1))``, then inverted for white ink on a
    black terminal, which is AsciiEffect's own mapping;
  * one white point light, no falloff, up and to the right and in front -
    ``PointLight(0xffffff, 1, 0, 0)`` at ``(100, 100, 400)``;
  * flat shading off the triangle normal, and both faces drawn
    (``material.flatShading = true``, ``material.side = DoubleSide``);
  * a 45 degree perspective camera;
  * two framebuffer rows per character row, which is AsciiEffect's ``y += 2``
    and the reason a character cell being twice as tall as it is wide does
    not squash the model.

What is deliberately NOT taken: the camera position. The reference puts it at
``(bbox.max.x * 4, bbox.max.y, bbox.max.z * 3)`` - a three-quarter view that
flatters a printed model. Here the board's rotation IS the reading, so the
camera stays on +Z and the board turns in front of it. An oblique camera
would make the caption "facing you: component side" a lie at rest.

Draw it big. A board is mostly flat, so the light alone cannot model it -
what shows the components is that each one covers several cells, and at
34x15 none of them does. The reference renders full-window for the same
reason.
"""
import math

#: AsciiEffect's default ramp, darkest first.
CHARACTERS = ' .:-+*=%@#'

#: The key light's direction, from `PointLight.position.set(100, 100, 400)`.
LIGHT_DIRECTION = (100.0, 100.0, 400.0)

#: How far along it the light sits, in model radii - the reference's own
#: distance-to-size ratio. A closer light was tried, to force some modelling
#: onto a flat board seen face-on; it is not needed and it is not what the
#: reference does. What that case actually wanted was RESOLUTION: at 150x60
#: every component covers several cells and supplies its own variation, and
#: at 34x15 nothing can.
LIGHT_DISTANCE = 4.0

#: Framebuffer rows per character row - AsciiEffect's `y += 2`.
ROW_PIXELS = 2

#: The reference camera's field of view, in degrees.
FOV_DEGREES = 45.0

#: Framebuffer pixels per character cell edge. The GPU hands AsciiEffect an
#: already-rasterised canvas, so a cell there is the average over a filled
#: triangle; one sample per cell speckles instead.
#:
#: 3 was measured at 44x19 and is wrong at a size worth drawing: it multiplies
#: the pixel count by nine, and the surface is a point cloud, so past a point
#: it starves the buffer rather than smoothing it. At 100x40 with 120,000
#: points, 2 gives 3.8 points per pixel and eight pixels per cell; 3 gives 1.7
#: and eighteen, and looks worse.
SUPERSAMPLE = 2



def _unit(vector):
    x, y, z = vector
    length = math.sqrt(x * x + y * y + z * z) or 1.0
    return (x / length, y / length, z / length)


def light_position():
    """Where the key light sits, in the model's own units."""
    x, y, z = _unit(LIGHT_DIRECTION)
    return (x * LIGHT_DISTANCE, y * LIGHT_DISTANCE, z * LIGHT_DISTANCE)


def brightness_char(brightness, ramp=CHARACTERS, invert=True):
    """AsciiEffect's mapping, including its inversion for dark mode.

    `invert` is the reference's `isDarkMode`: with it set, bright is heavy
    ink and the background is a space, which is what a terminal wants.
    """
    index = int((1.0 - max(0.0, min(1.0, brightness))) * (len(ramp) - 1))

    if invert:
        index = len(ramp) - 1 - index
    return ramp[index]


def _shade(point, normal, light):
    """Lambert from a point light with no falloff, both faces lit.

    DoubleSide means a triangle facing away is drawn with its normal flipped
    rather than dropped, so the far wall of a bore is lit rather than a hole
    in the picture.
    """
    lx, ly, lz = _unit((light[0] - point[0], light[1] - point[1],
                        light[2] - point[2]))
    return abs(normal[0] * lx + normal[1] * ly + normal[2] * lz)


def fit_distance(width, rows):
    """Where to stand so a unit-radius model just fills the shorter axis.

    The reference fits the camera to the model's bounding box once, on load.
    Here the model turns, so the fit is to its radius and not to a box that
    would breathe as it rotates - a board that grew and shrank while it
    tilted would read as the board moving toward you.
    """
    scale = (rows / 2.0) / math.tan(math.radians(FOV_DEGREES) / 2.0)
    return scale / min(rows / 2.0, width / 2.0) + 1.0


def render(surface, matrix, width, height, distance=None, ramp=CHARACTERS,
           invert=True, supersample=SUPERSAMPLE, zoom=1.0):
    """The surface under `matrix`, as `height` lines of `width` characters.

    `surface` is [(point, normal)] in model units, centred and unit-scaled.
    `matrix` is the nine-element row-major rotation to apply.

    `zoom` above 1.0 comes nearer. It divides the fitted distance rather than
    scaling the projection, because a camera that moves keeps the perspective
    honest - a nearer board is a board seen from nearer, foreshortening and
    all, not one drawn bigger.
    """
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = matrix
    light = light_position()

    cell_rows = ROW_PIXELS * supersample
    rows, cols = height * cell_rows, width * supersample
    if distance is None:
        distance = fit_distance(cols, rows)
    distance = 1.0 + (distance - 1.0) / max(0.05, zoom)

    depth = [[0.0] * cols for _ in range(rows)]
    value = [[0.0] * cols for _ in range(rows)]

    scale = (rows / 2.0) / math.tan(math.radians(FOV_DEGREES) / 2.0)
    cx, cy = (cols - 1) / 2.0, (rows - 1) / 2.0

    for point, normal in surface:
        px, py, pz = point
        x = m0 * px + m1 * py + m2 * pz
        y = m3 * px + m4 * py + m5 * pz
        z = m6 * px + m7 * py + m8 * pz

        ooz = 1.0 / (distance - z)
        col = int(round(cx + scale * ooz * x))
        row = int(round(cy - scale * ooz * y))

        if not (0 <= row < rows and 0 <= col < cols):
            continue
        if ooz <= depth[row][col]:
            continue

        ax, ay, az = normal
        nx = m0 * ax + m1 * ay + m2 * az
        ny = m3 * ax + m4 * ay + m5 * az
        nz = m6 * ax + m7 * ay + m8 * az

        depth[row][col] = ooz
        value[row][col] = _shade((x, y, z), (nx, ny, nz), light)

    return _resolve(depth, value, width, height, cell_rows, supersample,
                    ramp, invert)


def _resolve(depth, value, width, height, cell_rows, supersample, ramp,
             invert):
    """The framebuffer down to characters, averaging each cell.

    Averaging rather than AsciiEffect's single sample per cell: it reads one
    pixel of a canvas the GPU already filled, and this has to do that filling
    itself. Cells with nothing in them stay background - a cell part covered
    averages only what covered it, which is what keeps an edge an edge.
    """
    lines = []
    for r in range(height):
        line = []
        for c in range(width):
            total = seen = 0
            for dr in range(cell_rows):
                buffer_row = depth[r * cell_rows + dr]
                shade_row = value[r * cell_rows + dr]
                for dc in range(supersample):
                    if buffer_row[c * supersample + dc]:
                        total += shade_row[c * supersample + dc]
                        seen += 1
            if seen:
                line.append(brightness_char(total / seen, ramp, invert))
            else:
                line.append(ramp[0] if invert else ramp[-1])
        lines.append(''.join(line).rstrip())
    return '\n'.join(lines)
