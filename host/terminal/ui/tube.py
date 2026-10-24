"""The tube switched on over the shell it opens on, off from its last frame (`on`, `off`).

    tube.on()                  # the shell's screen, a beam, the band opening, its glow gone: the front page next
    tube.off(lines)            # the last frame (`stage.LAST`) folded to a line, a dot, the shell back

On the alternate screen throughout: the shell underneath is never written. Its text is read off the
console (Win32 `ReadConsoleOutputCharacterW`); where none can be read the tube opens on black.
"""
import ctypes
import os
import shutil
import sys
import time

from coaxial.graphics.approach import DATA

#: The switch on and off, s: a beam, the band, the glow; a fold, a line, a dot.
ON_S, OFF_S = 0.4, 0.3
#: Each stage's end, of the whole: on - the beam drawn across, the band open; off - folded, the line gone.
BEAM, OPEN, FOLDED, GONE = 0.25, 0.65, 0.6, 0.85
#: A frame's period, s: the console's refresh.
FRAME_S = 1.0 / 60.0
#: The shell's text as the tube shows it, grey; the beam's white.
SHELL, WHITE = (150, 150, 150), (255, 255, 255)
ALT, MAIN, HIDE, SHOW, HOME = '\x1b[?1049h', '\x1b[?1049l', '\x1b[?25l', '\x1b[?25h', '\x1b[H'


def seen():
    """The console's visible lines, as text; [] where none can be read."""
    if os.name != 'nt':
        return []
    from ctypes import wintypes

    class Info(ctypes.Structure):
        _fields_ = [('size', wintypes._COORD), ('cursor', wintypes._COORD), ('attributes', wintypes.WORD),
                    ('window', wintypes.SMALL_RECT), ('largest', wintypes._COORD)]
    kernel32 = ctypes.windll.kernel32
    out, info = kernel32.GetStdHandle(-11), Info()
    if not kernel32.GetConsoleScreenBufferInfo(out, ctypes.byref(info)):
        return []
    w = info.window
    width, got = w.Right - w.Left + 1, wintypes.DWORD()
    buf = ctypes.create_unicode_buffer(width)
    lines = []
    for row in range(w.Top, w.Bottom + 1):
        if not kernel32.ReadConsoleOutputCharacterW(out, buf, width, wintypes._COORD(w.Left, row), ctypes.byref(got)):
            return []
        lines.append(buf.value[:got.value].rstrip())
    return lines


def _mix(a, b, u):
    return tuple(round(x + (y - x) * u) for x, y in zip(a, b))


def _bg(rgb, width):
    return '\x1b[48;2;%d;%d;%dm%s\x1b[0m' % (*rgb, ' ' * width)


def _text(line, width, rgb):
    return '\x1b[38;2;%d;%d;%dm%s\x1b[0m' % (*rgb, line[:width].ljust(width))


def _smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def _shown(frame, u, width, height):
    """`frame(u, width, height)`'s rows written over the screen."""
    rows = frame(u, width, height)
    sys.stdout.write(HOME + '\n'.join(rows[:height]))
    sys.stdout.flush()


def _played(frame, seconds):
    width, height = shutil.get_terminal_size()
    began = time.monotonic()
    while (u := (time.monotonic() - began) / seconds) < 1.0:
        _shown(frame, u, width, height)
        time.sleep(FRAME_S)
    _shown(frame, 1.0, width, height)


def _on_frame(shell):
    """The tube warming over `shell`: a beam drawn out from the centre, the band opening from it
    white-hot to the phosphor's glow, the glow gone to black."""
    def frame(u, width, height):
        mid = height // 2
        dim = [_text(shell[r] if r < len(shell) else '', width, SHELL) for r in range(height)]
        if u < BEAM:
            half = round(_smooth(u / BEAM) * width / 2)
            dim[mid] = ' ' * (width // 2 - half) + _bg(WHITE, 2 * half) + ' ' * (width - width // 2 - half)
            return dim
        if u < OPEN:
            s = (u - BEAM) / (OPEN - BEAM)
            half, glow = _smooth(s) * height / 2, _mix(WHITE, DATA, s)
            edge = round(half)
            return [_bg(WHITE if abs(r - mid) == edge else glow, width) if abs(r - mid) <= edge else dim[r]
                    for r in range(height)]
        return [_bg(_mix(DATA, (0, 0, 0), _smooth((u - OPEN) / (1.0 - OPEN))), width)] * height
    return frame


def _off_frame(lines):
    """The tube going dark from `lines` (ANSI rows): folded to a white line at its centre, the
    line drawn in to a dot, the dot gone."""
    def frame(u, width, height):
        mid = height // 2
        blank = ' ' * width
        if u < FOLDED:
            half = (1.0 - _smooth(u / FOLDED)) * height / 2
            edge = round(half)

            def row(off):
                src = mid + round(off * (height / 2) / max(half, 1.0))
                return lines[src] if abs(off) <= edge and 0 <= src < len(lines) else blank
            return [_bg(WHITE, width) if abs(r - mid) == edge < mid else row(r - mid) for r in range(height)]
        rows = [blank] * height
        if u < GONE:
            half = round((1.0 - _smooth((u - FOLDED) / (GONE - FOLDED))) * width / 2)
            rows[mid] = ' ' * (width // 2 - half) + _bg(WHITE, max(1, 2 * half)) + ' ' * max(0, width - width // 2 - half - 1)
        else:
            dot = _mix(WHITE, (0, 0, 0), _smooth((u - GONE) / (1.0 - GONE)))
            rows[mid] = ' ' * (width // 2) + _bg(dot, 1)
        return rows
    return frame


def on():
    """The tube switched on over what the console shows, left on the alternate screen, black."""
    shell = seen()
    sys.stdout.write(ALT + HIDE)
    _played(_on_frame(shell), ON_S)


def off(lines):
    """The tube switched off from `lines`, the last frame's ANSI rows; the shell back."""
    sys.stdout.write(ALT + HIDE)
    try:
        _played(_off_frame(lines), OFF_S)
    finally:
        sys.stdout.write(MAIN + SHOW)
        sys.stdout.flush()
