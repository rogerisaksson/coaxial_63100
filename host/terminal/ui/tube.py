"""The tube switched on over the shell it opens on, off from its last frame (`on`, `off`).

    tube.on()                  # the shell's screen kept, the tube to open on the first frame drawn
    tube.opening(lines)        # that frame's rows: a dot, a line, the picture unfolding over the shell
    tube.off(lines)            # the last frame (`stage.LAST`) folded to a line, a dot, the shell back

A retro set's switch: on, a dot flashed, drawn out to a line, the picture unfolded from it; off, the
picture folded to the line, the line drawn in to a dot, the dot gone. On the alternate screen
throughout: the shell underneath is never written. Its text is read off the console (Win32
`ReadConsoleOutputCharacterW`); where none can be read the tube opens on black.
"""
import ctypes
import os
import shutil
import sys
import time

#: The switch on and off, s.
ON_S, OFF_S = 0.4, 0.3
#: Each stage's end, of the whole: on - the dot, the line drawn out; off - folded, the line drawn in.
DOT, LINE, FOLDED, GONE = 0.15, 0.4, 0.6, 0.85
#: A frame's period, s: the console's refresh.
FRAME_S = 1.0 / 60.0
#: The shell's text as the tube shows it, grey; the beam's white.
SHELL, WHITE, BLACK = (150, 150, 150), (255, 255, 255), (0, 0, 0)
ALT, MAIN, HIDE, SHOW, HOME = '\x1b[?1049h', '\x1b[?1049l', '\x1b[?25l', '\x1b[?25h', '\x1b[H'
#: The shell's rows while the tube waits on its first frame (`on`, `opening`).
OPENING: list = [None]


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


def _smooth(u):
    u = min(1.0, max(0.0, u))
    return u * u * (3.0 - 2.0 * u)


def _grey(shell, width, height):
    """The shell's text as the tube shows it: `height` rows of `width`."""
    return ['\x1b[38;2;%d;%d;%dm%s\x1b[0m' % (*SHELL, (shell[r] if r < len(shell) else '')[:width].ljust(width))
            for r in range(height)]


def _unfolded(lines, under, s, width, height):
    """`lines` unfolded about the centre to `s` of the screen (0 a line, 1 whole), its edges white,
    `under` beyond them."""
    mid, half = height // 2, s * height / 2
    edge = round(half)

    def row(r):
        src = mid + round((r - mid) * (height / 2) / max(half, 1.0))
        return lines[src] if 0 <= src < len(lines) else ' ' * width
    return [_bg(WHITE, width) if abs(r - mid) == edge < mid else row(r) if abs(r - mid) <= edge else under[r]
            for r in range(height)]


def _line(under, s, width, height, ink=WHITE):
    """`under` with a beam across its centre, `s` of the width drawn out - a dot at 0."""
    rows, mid = list(under), height // 2
    half = max(1, round(s * width / 2))
    rows[mid] = ' ' * (width // 2 - half) + _bg(ink, min(width, 2 * half)) + ' ' * max(0, width - width // 2 - half)
    return rows


def _on_frame(shell, lines):
    """The tube warming over `shell` into `lines`: a dot, a line drawn out, the picture unfolding."""
    def frame(u, width, height):
        under = _grey(shell, width, height)
        if u < DOT:
            return _line(under, 0.0, width, height)
        if u < LINE:
            return _line(under, _smooth((u - DOT) / (LINE - DOT)), width, height)
        return _unfolded(lines, under, _smooth((u - LINE) / (1.0 - LINE)), width, height)
    return frame


def _off_frame(lines):
    """The tube going dark from `lines`: folded to a line, the line drawn in to a dot, the dot gone."""
    def frame(u, width, height):
        under = [' ' * width] * height
        if u < FOLDED:
            return _unfolded(lines, under, 1.0 - _smooth(u / FOLDED), width, height)
        if u < GONE:
            return _line(under, 1.0 - _smooth((u - FOLDED) / (GONE - FOLDED)), width, height)
        return _line(under, 0.0, width, height, _mix(WHITE, BLACK, _smooth((u - GONE) / (1.0 - GONE))))
    return frame


def _played(frame, seconds):
    width, height = shutil.get_terminal_size()
    began = time.monotonic()
    while (u := (time.monotonic() - began) / seconds) < 1.0:
        sys.stdout.write(HOME + '\n'.join(frame(u, width, height)[:height]))
        sys.stdout.flush()
        time.sleep(FRAME_S)


def on():
    """What the console shows kept on the alternate screen; the tube opens on the first frame drawn."""
    OPENING[0] = seen()
    width, height = shutil.get_terminal_size()
    sys.stdout.write(ALT + HIDE + HOME + '\n'.join(_grey(OPENING[0], width, height)))
    sys.stdout.flush()


def opening(lines):
    """The tube switched on into `lines`, the first frame's ANSI rows, over the shell `on` kept."""
    shell, OPENING[0] = OPENING[0], None
    if shell is not None:
        _played(_on_frame(shell, lines), ON_S)


def off(lines):
    """The tube switched off from `lines`, the last frame's ANSI rows; the shell back."""
    sys.stdout.write(ALT + HIDE)
    try:
        _played(_off_frame(lines), OFF_S)
    finally:
        sys.stdout.write(MAIN + SHOW)
        sys.stdout.flush()
