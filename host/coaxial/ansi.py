"""Colour, kept in one place so the renderers stay drawings.

Every renderer here takes a `colour` flag and defaults it to off. That is not
timidity: a picture with escape sequences in it cannot be compared to an
expected string, and the suites compare these pictures. Colour is added at
the edge, by the tool that knows whether it is writing to a terminal, and the
tests see the same characters either way.

256-colour rather than the 16 basic ones. The greys are what make a shaded
surface read as shaded - eight levels of grey against a ramp of ten
characters is most of the picture's information, and the basic set has one.
"""

def utf8_stdout():
    """Make stdout carry the glyphs in this module. Call it at the edge.

    A pipe takes its encoding from the locale, cp1252 here, and every
    ramp below is outside ASCII. Measured: `show_desk.py | anything`
    died with UnicodeEncodeError before the first frame, and
    thermal_model.py died the same way when redirected. `replace`, not
    `strict` - a log is worth reading with glyphs substituted and
    worthless as a traceback.

    Not done on import: a library that reconfigures the caller's
    stdout is a side effect nobody asked for, and this module already
    says colour is added at the edge.
    """
    import sys

    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except (AttributeError, ValueError):   # not a reconfigurable stream
        pass


RESET = '\033[0m'

#: xterm-256 greyscale, black to white. 24 steps, and the ramp characters ride
#: on top of them: a cell is a glyph AND a grey, which is twice the levels
#: either could carry alone.
GREY_FIRST = 232
GREY_STEPS = 24

#: The meter's zones, after the lamp bridge they are drawn from: green below
#: -10 dBFS, amber above it, red at the rail. Numbers are xterm-256.
GREEN = 46
AMBER = 214
RED = 196
DIM = 236
WHITE = 231


def code(number):
    """One 256-colour foreground escape."""
    return '\033[38;5;%dm' % number


def back(number):
    """One 256-colour background escape."""
    return '\033[48;5;%dm' % number


#: Lower half block. A cell drawn with it carries two colours - foreground
#: below, background above - so one character row holds two picture rows.
#: Taken from esimov/ascii-fluid, which uses the same glyph for the same
#: reason: a terminal cell is twice as tall as it is wide, and this is how
#: you get the other half back.
HALF = '▄'


def paint(text, number):
    """`text` in one colour, closed again."""
    return code(number) + text + RESET


def grey(fraction):
    """The grey for a brightness in 0..1."""
    step = int(max(0.0, min(1.0, fraction)) * (GREY_STEPS - 1))
    return GREY_FIRST + step


#: Dark solder mask to lit copper, in the xterm-256 cube. A board drawn in
#: greys is a grey disc; a board drawn in its own colour is recognisably a
#: board, and the components read as the bright end because they are what
#: catches the light. Not decoration - it is the one cue that survives at a
#: cell per two millimetres.
BOARD_RAMP = (22, 22, 28, 34, 40, 46, 47, 83, 119, 155, 191, 227, 231)


#: Colour stops by DEGREES, not by fraction of a span: auto-ranging made a
#: cool board look exactly like a hot one. Spacing is deliberately uneven -
#: the board idles near 30 C, so 20-60 gets the resolution and red starts at
#: 90, where a laminate is in trouble rather than merely working.
THERMAL_STOPS = (
    (-20.0, 17),    # deep blue
    (0.0, 19),
    (15.0, 25),
    (25.0, 31),     # ambient, and where the resolution has to be fine:
    (30.0, 37),     # this board idles near 30 and works between 30 and 60,
    (35.0, 43),     # so the steps are 5 K apart through there. Coarser stops
    (40.0, 44),     # put a 6 K difference inside one colour and hid the hot
    (45.0, 49),     # swap entirely.
    (50.0, 50),
    (55.0, 79),
    (60.0, 83),     # green reaches here
    (65.0, 118),
    (70.0, 154),
    (75.0, 190),
    (80.0, 220),    # yellow
    (85.0, 214),
    (90.0, 208),    # orange - the warm end starts late on purpose
    (95.0, 202),
    (100.0, 196),   # red
)

#: The ends of the scale. What a picture is drawn against, whatever is in it.
THERMAL_MIN = THERMAL_STOPS[0][0]
THERMAL_MAX = THERMAL_STOPS[-1][0]


def thermal(celsius):
    """The colour for an absolute temperature, from THERMAL_STOPS."""
    if celsius <= THERMAL_STOPS[0][0]:
        return THERMAL_STOPS[0][1]
    for (lo_c, lo_n), (hi_c, hi_n) in zip(THERMAL_STOPS, THERMAL_STOPS[1:]):
        if celsius <= hi_c:
            # Nearest stop rather than a blend: the 256-colour cube has no
            # useful intermediates between these, and rounding to one of the
            # two keeps a band readable as a band.
            half = (lo_c + hi_c) / 2.0
            return lo_n if celsius < half else hi_n
    return THERMAL_STOPS[-1][1]


def board(fraction):
    """The board's colour for a brightness in 0..1."""
    step = int(max(0.0, min(1.0, fraction)) * (len(BOARD_RAMP) - 1))
    return BOARD_RAMP[step]


def run(cells):
    """Cells of (text, colour) as one line, changing colour only where it
    changes.

    One escape per run rather than per character: a 44-column picture is 44
    escapes a frame either way, but a board drawn in flat regions is three or
    four, and the difference is what a terminal repainting at 20 Hz notices.
    `colour` None means leave whatever is current alone.
    """
    out = []
    current = None

    for text, number in cells:
        if number is not None and number != current:
            out.append(code(number))
            current = number
        out.append(text)

    if current is not None:
        out.append(RESET)
    return ''.join(out)
