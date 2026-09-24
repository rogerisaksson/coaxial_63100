"""The board's X, Y and Z as a gizmo in the frame's upper right."""
import math

from coaxial.draw.ansi import rgb as _rgb
from coaxial.graphics.lines import _trace
from coaxial.graphics.raster import BRAILLE, BRAILLE_BITS


#: THE TRIAD: the board's own X, Y and Z as a small gizmo in the
#: frame's upper right, turning with the board - the reference every
#: CAD view keeps in a corner: braille lines from an origin, the
#: letter one letter past each tip. Each axis in one of the console
#: motif's three roles (terminal/ui/screen.py: NEON teal 44 names things,
#: SODIUM amber 214 is the value that matters, ASH grey 242 the
#: frame) - three colours the theme already owns, not a christmas
#: tree of red, green and blue - X sodium, Y neon, Z ash, asked for as
#: "different colours, chosen by the theme otherwise". An axis toward
#: the camera carries its colour in full, one away TRIAD_DIM of it;
#: the letter is a step brighter than its line (TRIAD_LABEL_LIFT of
#: white mixed in). TRIAD_REACH bounds the length of an axis lying in
#: the screen's plane, in columns (half that in rows), sized from the
#: frame at a sixth of its rows: neither a smudge on a 24-row terminal
#: (four) nor a second subject at 44 (seven).
#:
#: IT IS DRAWN LAST AND CLIPPED BY NOTHING. A gizmo is the frame's
#: own furniture, not something standing in the scene: it reads the
#: rotation, so it has to be legible whatever the board is doing, and
#: a corner the board happens to reach is exactly when the rotation is
#: worth reading. The depth buffer is not consulted.
#:
#: NOTHING IS CLEARED BEHIND IT EITHER. The backdrop, the outline and
#: the axes are all a 2x4 dot matrix, so an axis ORs its dots into
#: whatever the cell already held and the fan, the horizon and the
#: board's own edges run on through the gizmo; the axis takes the
#: cell's colour, being the thing in front. A box blanked behind the
#: triad was a rectangular hole that travelled with the frame. Over a
#: face glyph there is nothing to OR with and the dots replace it,
#: which is what drawing in front means.
TRIAD_REACH = (4, 7)
#: Flush into the corner, no inset. The reach is measured to the TIP
#: and a letter lands one cell past it, so the letter is what the
#: margin ever protected - and it does not need protecting: swept over
#: 60 orientations at 60x20, 80x24, 120x36 and 150x44, every one of X,
#: Y and Z is lettered exactly once at (0, 0). What the inset cost was
#: the corner itself: at 80x24 and zoom 1.6 the board reached into the
#: gizmo's footprint in 17 of 40 frames and 37 cells at the old (2, 1),
#: against 15 and 19 flush; at 150x44, 9 frames and 21 cells against 5
#: and 6.
TRIAD_MARGIN = (0, 0)
TRIAD_DIM = 0.45
TRIAD_LABEL_LIFT = 0.35
TRIAD_AXES = (((1.0, 0.0, 0.0), 'X', 214), ((0.0, 1.0, 0.0), 'Y', 44),
              ((0.0, 0.0, 1.0), 'Z', 242))


def _triad(grid, tone, cam, m, colour):
    """The board's axes as a gizmo in the upper right - see TRIAD_REACH."""
    width, height = cam['width'], cam['height']
    reach = max(TRIAD_REACH[0], min(TRIAD_REACH[1], height // 6))
    half = reach // 2 + 1
    ox = width - 2 - TRIAD_MARGIN[0] - reach
    oy = TRIAD_MARGIN[1] + half
    m0, m1, m2, m3, m4, m5, m6, m7, m8 = m

    def inside(px, py):
        return 0 <= px < width and 0 <= py < height

    def shade(code, cue, lift=0.0):
        r, g, b = _rgb(code)
        return (int(r * cue + (255 - r * cue) * lift + 0.5),
                int(g * cue + (255 - g * cue) * lift + 0.5),
                int(b * cue + (255 - b * cue) * lift + 0.5))

    masks, inks, letters, taken = {}, {}, [], set()
    for (x, y, z), letter, code in TRIAD_AXES:
        vx = m0 * x + m1 * y + m2 * z
        vy = m3 * x + m4 * y + m5 * z
        vz = m6 * x + m7 * y + m8 * z
        cue = TRIAD_DIM + (1.0 - TRIAD_DIM) * 0.5 * (vz + 1.0)
        x0, y0 = ox + 0.5, oy + 0.5
        dx, dy = reach * vx, -0.5 * reach * vy

        def dot(fx, fy, _w, cue=cue, code=code):
            if fx < 0.0 or fy < 0.0:
                return
            px, py = int(fx), int(fy)
            if not inside(px, py):
                return
            at = py * width + px
            col = 1 if fx - px >= 0.5 else 0
            row = min(3, int((fy - py) * 4.0))
            masks[at] = masks.get(at, 0) | BRAILLE_BITS[col][row]
            # Where two axes cross a cell, the nearer one's ink.
            if at not in inks or inks[at][0] < cue:
                inks[at] = (cue, code)

        _trace(x0, y0, 1.0, x0 + dx, y0 + dy, 1.0, dot)
        # The letter: the cell one letter past the tip along the axis, a letter
        # being a column wide and a row tall - the point moves as smoothly as
        # the tip does, so the letter follows a cell at a time.
        run = math.sqrt(dx * dx + 4.0 * dy * dy)
        if run < 0.5:
            ex, ey = 1.0, 0.0
        else:
            ex, ey = dx / run, 2.0 * dy / run
        for out in (1.0, 2.0, 3.0):
            lx, ly = int(x0 + dx + out * ex), int(y0 + dy + out * ey)
            if (lx, ly) not in taken and (lx, ly) != (ox, oy):
                break
        taken.add((lx, ly))
        letters.append((lx, ly, letter, cue, code))

    for at, mask in masks.items():
        r, c = divmod(at, width)
        # OR, not replace: the floor's dots in this cell are part of the same
        # matrix, and an axis crossing them should read as crossing them rather
        # than as a bite taken out of the fan.
        under = ord(grid[r][c]) - BRAILLE
        if 0 <= under <= 0xFF:
            mask |= under
        grid[r][c] = chr(BRAILLE + mask)
        if colour:
            cue, code = inks[at]
            tone[r][c] = shade(code, cue)
    for lx, ly, letter, cue, code in letters:
        if not inside(lx, ly):
            continue
        grid[ly][lx] = letter
        if colour:
            tone[ly][lx] = shade(code, cue, TRIAD_LABEL_LIFT)
    return ox, oy, reach
