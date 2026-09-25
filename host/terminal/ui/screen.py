"""What every live view needs: a preflight line, a flicker-free redraw, a way out that cleans up.

Shared: the structure suite fails a definition that lives in two files.
"""
import os
import re
import sys
import threading
import time

from coaxial import Coaxial63100
from coaxial.devices.thermal import THROTTLE_AT
from coaxial.draw import cross_section, gauges
from coaxial.errors import DeviceStateError, NoReplyError, RigError
from machine.modes import HARDWARE, SIMULATED
from machine import ansi
from terminal.ui.console import Keys, _ignore
from terminal.ui.rate import rate_of
from terminal.ui.scroll import scroll_by, scroll_click, scroll_drag
from terminal.ui.stage import boot, curtain

ansi.utf8_stdout()          # every view draws outside ASCII


ENTER_KEYS = ('\r', '\n')

#: A scroll key's step: the arrows move the instrument column a box.
SCROLL_STEP = {'down': 1, 'up': -1}

#: The probes' control characters, built with chr(): a backslash escape
#: through a shell heredoc arrives as the real character (CLAUDE.md, Traps).
ESC = chr(27)

CR = chr(13)

#: Preflight colours, matching board_chat/Say.ps1 so the two read the
#: same. Green ok, cyan waiting, yellow worth knowing, red stop.
STATES = {'ok': '32', 'wait': '36', 'warn': '33', 'fail': '31'}

#: Whether ok/wait preflight lines print at all. The views turn this off:
#: their boot bar replaced the scroll of green lines on the way in, and
#: the teardown listing on the way out - warn and fail always print.
CHATTER = True

#: The motif's two light sources, wet asphalt between (Blade Runner).
NEON = 44

SODIUM = 214

#: Inline-label ash - the same 66 the theme's `label` style uses, so a
#: tinted string beside a hud row reads as one voice. 242 stays for the
#: gauge rails and key hints, where receding further is the point.
ASH = 242

LABEL = 66

_SGR = re.compile(chr(27) + r'\[[0-9;]*m')


def plain(text):
    """`text` with every SGR escape removed - what the eye would count."""
    return _SGR.sub('', text)


def visible(text):
    return len(plain(text))


def tint(text, colour):
    """`text` in one of the motif's colours. Stripped at the door if piped."""
    return '%s[38;5;%dm%s%s[0m' % (chr(27), colour, text, chr(27))


#: The corner registration cross, three rows tall - the mark the Nostromo
#: screens put in every dead corner of a viewport. Stamped, not drawn: it
#: lands only where the field is empty, so a drawing that reaches a corner
#: simply keeps it.
CROSS = ('  │  ', ' ─┼─ ', '  │  ')


def stamp_crosses(lines, width, inset=2):
    """Registration crosses in the four corners of a field of `lines`."""
    if len(lines) < 8 or width < 24:
        return lines

    out = [line + ' ' * max(0, width - visible(line)) for line in lines]

    tall = len(CROSS)
    for top in (inset - 1, len(out) - inset - tall + 1):
        for left in (inset, width - inset - len(CROSS[0])):
            rows = range(max(0, top), min(len(out), top + tall))
            # A cross that loses a row to the drawing is debris, not a mark:
            # the whole corner yields if any of its cells are taken.
            clear = all(
                out[r][left:left + len(CROSS[0])] == ' ' * len(CROSS[0])
                for r in rows) and len(rows) == tall
            if not clear:
                continue
            for i, bit in enumerate(CROSS):
                line = out[top + i]
                out[top + i] = line[:left] + bit + line[left + len(bit):]
    return [tint(line, NEON) if line.strip() and set(line) <= set(' |-+│─┼')
            else line for line in out]


def panel_width():
    """The window less two, at least 60 - or 100 when this is not a terminal."""
    try:
        return max(60, os.get_terminal_size().columns - 2)
    except OSError:
        return 100


def paced(keys, period, step=0.02):
    """Sleep `period` while polling the keys every `step`."""

    zoom, typed = 0.0, []
    deadline = time.monotonic() + period
    while True:
        got, moved = keys.poll()
        zoom += moved
        typed.extend(keys.taken())
        if got:
            return got, zoom, typed
        now = time.monotonic()
        if now >= deadline:
            return None, zoom, typed
        time.sleep(min(step, deadline - now))


def gauge(fraction, width, hot=THROTTLE_AT):
    """A meter in the motif: the level in dots, the rest of the scale in the
    track's grey, the margin's amber past `hot`.
    """
    fraction = max(0.0, min(1.0, fraction))
    return gauges.gauge(fraction, width,
                        cls=cross_section.SOA_WARN if fraction >= hot
                        else cross_section.SOA_OK)


class Feed:

    """The board read on its own thread, so a frame draws at the screen's
    pace rather than the link's.
    """

    def __init__(self, read, period=0.0):
        self.read = read
        self.period = period
        self.latest = None
        self.error = None
        self.reads = 0
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()
        return self

    def _run(self):

        while not self._stop.is_set():
            try:
                got = self.read()
            except Exception as exc:
                # The thread's edge: whatever `read` raised is kept for the
                # view to show while it keeps drawing - a feed that dies
                # hides its own reason.
                self.error = exc
            else:
                self.error = None
                self.latest = got
                self.reads += 1
            if self.period:
                time.sleep(self.period)

    def stop(self, wait=1.0):
        """Ask it to stop and wait, so nothing touches the link after the
        caller starts putting the board back."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(wait)
        return self


def mode_of(args):
    """The execution mode a view's flags ask for: --simulated, else the board on --port."""
    return SIMULATED if args.simulated else HARDWARE


def open_rig(banner, **kwargs):
    """The rig behind a boot strip, or None with the board's own words said."""

    try:
        with boot(banner):
            return Coaxial63100(**kwargs).open()
    except (RigError, OSError) as exc:
        # OSError as well: a port another process holds, and a broker socket
        # that times out, both arrive here as something other than a RigError
        # and both mean the same thing to a reader - no board.
        say('fail', 'link', str(exc))
        return None


#: The screen's own rate: between a page's draws - its data's rate, 2 Hz on the
#: thermal page - the last frame is shown again UI_HZ times a second, so what moves
#: with the clock (the CRT, the lock, the tags, the clocks) never waits on the data
#: (2026-09-25).
UI_HZ = 15.0


def run_view(board_view, console, period, frames, draw, on_input=None,
             tick=None, mouse=False, on_click=None, on_drag=None,
             scroll_keys=True):
    """The loop every view runs: draw, pace, take keys - until Q, ESC,
    Ctrl+C or `frames` frames. Between draws the frame is shown again at UI_HZ;
    a key cuts the wait and draws at once.
    """

    click, drag = on_click or _ignore, on_drag or _ignore
    count = 0
    try:
        with curtain(board_view) as page, Keys(console, mouse=mouse) as keys:
            while True:
                count += 1
                # The period is frame to frame, measured from this draw's
                # start - not a sleep after it.
                started = time.monotonic()
                shown = draw()
                page.update(shown, refresh=True)
                rate_of(board_view).tick(started)
                if tick is not None and tick():
                    return None
                if frames and count >= frames:
                    return None
                leaving, moved, typed = None, 0.0, []
                while True:
                    left = period - (time.monotonic() - started)
                    leaving, more, got = paced(keys, max(0.0, min(left, 1.0 / UI_HZ)))
                    moved += more
                    typed.extend(got)
                    if leaving or typed or moved or left <= 1.0 / UI_HZ:
                        break
                    page.update(shown, refresh=True)
                if leaving:
                    return leaving
                if scroll_keys:
                    for step in (SCROLL_STEP[key] for key in typed
                                 if key in SCROLL_STEP):
                        scroll_by(board_view, step)
                    typed = [key for key in typed if key not in SCROLL_STEP]
                if on_input is not None:
                    on_input(typed, moved)
                clicks = keys.clicked() if mouse or on_click is not None else ()
                for col, row in clicks:
                    scroll_click(board_view, col, row)
                    click(col, row)
                dx, dy = keys.dragged() if mouse or on_drag is not None else (0, 0)
                if dx or dy:
                    scroll_drag(board_view, dy)
                    drag(dx, dy)
    except KeyboardInterrupt:
        return None


class Freshness:
    """Whether the board's counter moved since the last frame, and how fast."""

    def __init__(self):
        self.seen, self.stale = -1, 0
        self.rate, self._rate_seen, self._rate_at = 0.0, None, time.time()

    def take(self, updates):
        """One frame's counter, or None for no reading."""
        if updates is None or updates == self.seen:
            self.stale += 1
        else:
            self.seen, self.stale = updates, 0
        if updates is None:
            return
        now = time.time()
        if now - self._rate_at < 1.0:
            return
        if self._rate_seen is not None:
            self.rate = (updates - self._rate_seen) / (now - self._rate_at)
        self._rate_seen, self._rate_at = updates, now

    @property
    def note(self):
        return ('stale %d frames' % self.stale) if self.stale else 'live'


def say(state, text, detail=''):
    """One preflight line, the shape board_chat.ps1 prints."""
    if not CHATTER and state in ('ok', 'wait'):
        return
    if not sys.stdout.isatty():
        # Same rule the views follow: colour at the edge, and a pipe is not
        # one.
        sys.stdout.write('  %-6s%-22s %s\n' % (state, text, detail))
        sys.stdout.flush()
        return

    esc = chr(27)
    colour = STATES.get(state, '37')
    sys.stdout.write('  %s[%sm%-6s%s[0m%-22s %s[90m%s%s[0m\n'
                     % (esc, colour, state, esc, text, esc, detail, esc))
    sys.stdout.flush()


#: A board hiccup as a view sees it: one list, for every view and bench tool.
QUIET = (NoReplyError, RigError, DeviceStateError)


def steady(fn, *args, **kwargs):
    """Call it, retrying the link's occasional silence."""

    for _ in range(4):
        try:
            return fn(*args, **kwargs)
        except QUIET:
            time.sleep(0.15)
    return None


def park(rows, console):
    """Put the cursor on the first line below a painted frame."""
    if console:
        sys.stdout.write('%s[%d;1H%s[J' % (chr(27), rows + 1, chr(27)))
        sys.stdout.flush()


def closing(done, console, drawn):
    """List what a view is putting back, under the frame it drew."""
    park(drawn, console)
    say('wait', 'closing', 'putting back what this view changed')
    for name, what in done:
        failed = str(what).startswith('FAILED')
        say('fail' if failed else 'ok', name, what)


def clear(console):
    """Wipe the screen and put the cursor home."""
    if console:
        sys.stdout.write(chr(27) + '[2J' + chr(27) + '[H')
        sys.stdout.flush()
