"""The QUAD page's wind: a pointer for the way it blows and its size, and its kind in words.

    lines = pointer(air, h, yaw)        # the dial's rows, its words beside them: a hud's

The dial is the floor seen from above as the view has it - up into the screen, the view
`yaw` degrees round, a tick each quarter: its needle the way the wind blows, as long as it is
strong, SPAN_M_S to its whole reach. Beside it the air's kind (`quad.AIR`) and how long it
stays, the wind and what of it is on the frame `h` m up, the gust in it, the kind's own wind
and its eddies.
"""
import math

from coaxial.draw import braille
from coaxial.graphics import shapes
from machine import quad

#: The dial's rows - twice as many cells, a square of dots - the dots round its rim, and the
#: wind its needle is whole at, m/s: REACH of the way to the rim, STUB of it at the least,
#: headed from HEAD of it, its barbs HEAD_DOTS long and HEAD_DEG off its back.
ROWS, RIM, SPAN_M_S = 5, 30, 6.0
REACH, STUB, HEAD, HEAD_DOTS, HEAD_DEG = 0.85, 0.25, 0.5, 2.0, 45.0

#: A gust is said from this much, m/s.
GUST_M_S = 0.3

#: The inks: the rim, and the needle's and its kind's word's by the air's kind. SGR over a
#: string, as the side column takes it.
RESET, RIM_INK = '\x1b[0m', '\x1b[38;5;244m'
INKS = {'calm': '\x1b[38;5;250m', 'constant': '\x1b[38;5;51m', 'gusty': '\x1b[38;5;214m',
        'changing': '\x1b[38;5;227m', 'turbulent': '\x1b[38;5;207m'}


def dial(to, share, rows=ROWS):
    """(rim, needle): two grids of braille bits, `rows` by twice as many cells - the rim's
    dots, and a needle from the middle the way `to`, rad from up toward the right, `share`
    of its whole reach, headed where it is long enough to be."""
    wide, size = 2 * rows, 4 * rows
    rim, needle = ([[0] * wide for _ in range(rows)] for _ in range(2))
    middle = 0.5 * (size - 1)

    def dot(grid, x, y):
        x, y = int(round(x)), int(round(y))
        if 0 <= x < size and 0 <= y < size:
            grid[y // 4][x // 2] |= braille.BIT[x % 2][y % 4]

    for k in range(RIM):
        dot(rim, middle + middle * math.sin(math.tau * k / RIM),
            middle - middle * math.cos(math.tau * k / RIM))
    for x, y in ((0.0, 1.0), (1.0, 0.0), (0.0, -1.0), (-1.0, 0.0)):
        dot(rim, middle + (middle - 1.5) * x, middle - (middle - 1.5) * y)
    reach = REACH * middle * max(STUB, min(1.0, share))
    for step in range(int(2.0 * reach) + 1):
        dot(needle, middle + 0.5 * step * math.sin(to), middle - 0.5 * step * math.cos(to))
    if share >= HEAD:
        for side in (-1.0, 1.0):
            back = to + math.pi + side * math.radians(HEAD_DEG)
            for step in range(1, int(2.0 * HEAD_DOTS) + 1):
                dot(needle, middle + reach * math.sin(to) + 0.5 * step * math.sin(back),
                    middle - reach * math.cos(to) - 0.5 * step * math.cos(back))
    return rim, needle


def seen(wind, yaw):
    """(the way a world wind (x, y, z) blows on the dial, rad from up toward the right, its
    speed along the floor, m/s) for a view `yaw` degrees round: up is into the screen."""
    m = shapes.view(yaw, 0.0)
    right = m[0] * wind[0] + m[2] * wind[2]
    near = m[6] * wind[0] + m[8] * wind[2]
    return math.atan2(right, -near), math.hypot(wind[0], wind[2])


def pointer(air, h, yaw):
    """The dial's rows with their words: the air's kind and how long it stays - or `kept` -,
    the wind, what of it is on the frame `h` m up, the gust in it, the kind's own wind and
    its eddies."""
    to, speed = seen(air['wind'], yaw)
    here = quad.wind_at(air['wind'], h)
    rim, needle = dial(to, speed / SPAN_M_S)
    ink = INKS[air['kind']]
    mean, _veer, gust, rough = air['come']
    words = (ink + air['kind'].upper() + RESET,
             'kept' if air['kept'] else '%.0f s more' % max(0.0, quad.KIND_S - air['for']),
             '%.1f m/s, %.1f on it' % (speed, math.hypot(here[0], here[2])),
             'gust %+.1f of %.1f' % (air['blows'], gust) if air['blows'] >= GUST_M_S
             else 'gusts %.1f' % gust if gust >= GUST_M_S else 'no gusts',
             'mean %.1f, eddies %.1f' % (mean, rough))
    lines = []
    for under, over, word in zip(rim, needle, words):
        cells, was = [], None
        for a, b in zip(under, over):
            want = ink if b else RIM_INK
            if want != was:
                cells.append(want)
                was = want
            cells.append(braille.ALL[a | b])
        lines.append(''.join(cells) + RESET + '  ' + word)
    return lines
