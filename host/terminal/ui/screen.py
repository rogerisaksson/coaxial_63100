"""What every live view needs: a preflight line, a flicker-free redraw, a way out that cleans up.

Shared: the structure suite fails a definition that lives in two files.
"""
import faulthandler
import gc
import os
import re
import sys
import threading
import time

from rich.segment import Segment

from coaxial import Coaxial63100
from coaxial.comm.session import standing
from coaxial.devices.thermal import THROTTLE_AT
from coaxial.draw import cross_section, gauges
from coaxial.errors import DeviceStateError, NoReplyError, RigError
from machine.modes import HARDWARE, SIMULATED
from machine import ansi
from terminal.ui.console import QUIT_KEYS, SELECT_KEYS, Keys, _ignore
from terminal.ui.rate import rate_of
from terminal.ui.scroll import scroll_by, scroll_click, scroll_drag
from terminal.ui import sto
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


def _mouse_pending(keys):
    return (getattr(keys, '_spun', (0.0, 0.0)) != (0.0, 0.0)
            or getattr(keys, '_panned', (0.0, 0.0)) != (0.0, 0.0)
            or bool(getattr(keys, '_clicked', None)))


def paced(keys, period, step=0.02):
    """Sleep `period` while polling the keys every `step`."""

    zoom, typed = 0.0, []
    deadline = time.monotonic() + period
    while True:
        got, moved = keys.poll()
        zoom += moved
        typed.extend(keys.taken())
        if got or zoom or typed or _mouse_pending(keys):
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


#: Maximum frames per second across views; the most a feed reads its board a second.
FPS_CAP, FEED_HZ = 30.0, 20.0

#: A frame between these, a synchronized update (DEC mode 2026): a terminal that knows it paints
#: the frame whole, not the half of it a write had reached; one that does not ignores them.
SYNC_BEGIN, SYNC_END = '\x1b[?2026h', '\x1b[?2026l'

#: A frame of a page in a console past STALL_S dumps every thread's stack, appended to STALL_LOG:
#: HUMANOID froze as the mouse moved on the bench, and not here in 300 frames under 6 070 sweeps.
STALL_S = 3.0
STALL_LOG = os.path.join(os.environ.get('LOCALAPPDATA') or os.path.expanduser('~/.cache'), 'coaxial_63100',
                         'stalls.txt')

#: Default rig URI when none is passed on the command line.
PORT = 'native://'

#: Minimum sleep between two feed reads, s.
FEED_GAP_S = 0.005


class Feed:

    """The board read on its own thread, so a frame draws at the screen's pace rather than the link's."""

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
            began = time.monotonic()
            try:
                got = self.read()
            except Exception as exc:
                self.error = exc
            else:
                self.error = None
                if got is not None:
                    self.latest = got
                self.reads += 1
            if self.period:
                time.sleep(max(FEED_GAP_S, self.period - (time.monotonic() - began)))

    def stop(self, wait=1.0):
        """Ask it to stop and wait, so nothing touches the link during teardown."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(wait)
        return self


def demo(origin):
    """Whether a page may drive the board to show itself (simulated or emulated, never live)."""
    return standing(origin) != 'live'


def mode_of(args):
    """The execution mode a view's flags ask for: --simulated, else the board on --port."""
    return SIMULATED if args.simulated else HARDWARE


def open_rig(banner, **kwargs):
    """The rig behind a boot strip, or None with the board's own words said."""
    try:
        with boot(banner):
            rig = Coaxial63100(**kwargs).open()
        sto.watch(rig)
        return rig
    except (RigError, OSError) as exc:
        say('fail', 'link', str(exc))
        return None


#: UI refresh rate between data draws so CRT/clock elements stay live, Hz.
UI_HZ = 15.0

#: A frame reaches the terminal its lead after its start: the LEAD_SHARE-th of the work - its draw
#: and rich's update - of the last LEAD_FRAMES frames, LEAD_MARGIN_S over it, s.
LEAD_FRAMES, LEAD_SHARE, LEAD_MARGIN_S = 40, 0.9, 0.002

#: When the frame being drawn reaches the terminal, on time.perf_counter's clock; None between.
_DRAWN_AT: list = [None]


def drawn_at():
    """When the frame being drawn reaches the terminal, s (time.perf_counter): a page playing a
    world back shows it at that instant - its motion in step with the frames as they are seen.
    Now, outside a frame."""
    return time.perf_counter() if _DRAWN_AT[0] is None else _DRAWN_AT[0]


class Beat:

    """A view's frames on an even beat: a frame's slot the last one's write and the interval - the
    view's `period` or the frames' `lead`, the longer -, no sooner than its own start and lead; the
    work of the frames before measured (`took`), each write kept (`wrote`)."""

    def __init__(self, period):
        self.period, self.works, self.last = period, [], None

    def lead(self):
        if not self.works:
            return 0.0
        ordered = sorted(self.works)
        return ordered[int(LEAD_SHARE * (len(ordered) - 1))] + LEAD_MARGIN_S

    def slot(self, began):
        """When the frame begun at `began` is to be written."""
        lead = self.lead()
        due = began + lead
        return due if self.last is None else max(due, self.last + max(self.period, lead))

    def took(self, seconds):
        self.works = (self.works + [seconds])[-LEAD_FRAMES:]

    def wrote(self, at):
        self.last = at


def _wait_frame(keys, page, shown, started, period):
    """Pace until `period` elapses from `started`, refreshing `page` at `UI_HZ` unless input arrives."""
    leaving, moved, typed = None, 0.0, []
    while True:
        left = period - (time.monotonic() - started)
        leaving, more, got = paced(keys, max(0.0, min(left, 1.0 / UI_HZ)))
        moved += more
        typed.extend(got)
        if leaving or typed or moved or _mouse_pending(keys) or left <= 1.0 / UI_HZ:
            return leaving, moved, typed
        page.update(shown, refresh=True)


def run_view(board_view, console, period, frames, draw, on_input=None,
             tick=None, mouse=False, on_click=None, on_drag=None,
             scroll_keys=True, on_pan=None, select=SELECT_KEYS, quits=QUIT_KEYS):
    """Run a view loop: draw, pace at `period` (capped by `FPS_CAP`), and dispatch keys/mouse.
    A frame is held till its beat (`Beat`, `drawn_at`); the heap built before the view is frozen
    out of its collections: a full one in a frame held it 25 ms."""
    period = max(period, 1.0 / FPS_CAP)
    click, drag = on_click or _ignore, on_drag or _ignore
    count, beat = 0, Beat(period)
    gc.collect()
    gc.freeze()
    q_kw = {'quits': quits} if quits != QUIT_KEYS else {}
    stalls = _stall_log() if console else None
    try:
        with curtain(board_view) as page, Keys(console, mouse=mouse, select=select,
                                               pan=on_pan is not None, **q_kw) as keys:
            while True:
                count += 1
                started = time.monotonic()
                if stalls is not None:
                    faulthandler.dump_traceback_later(STALL_S, file=stalls)
                began = time.perf_counter()
                _DRAWN_AT[0] = beat.slot(began)
                with board_view:
                    shown = draw()
                    if console:
                        board_view._buffer.append(Segment(SYNC_BEGIN))
                    page.update(shown, refresh=True)
                    if console:
                        board_view._buffer.append(Segment(SYNC_END))
                    beat.took(time.perf_counter() - began)
                    rest = _DRAWN_AT[0] - time.perf_counter()
                    if rest > 0.0:
                        time.sleep(rest)
                beat.wrote(time.perf_counter())
                _DRAWN_AT[0] = None
                rate_of(board_view).tick(started)
                if (tick is not None and tick()) or (frames and count >= frames):
                    return None
                leaving, moved, typed = _wait_frame(keys, page, shown, started, period)
                if leaving:
                    return leaving
                for step in (SCROLL_STEP[k] for k in typed if scroll_keys and k in SCROLL_STEP):
                    scroll_by(board_view, step)
                if scroll_keys:
                    typed = [k for k in typed if k not in SCROLL_STEP]
                if on_input is not None:
                    on_input(typed, moved)
                for col, row in (keys.clicked() if mouse or on_click is not None else ()):
                    scroll_click(board_view, col, row)
                    click(col, row)
                dx, dy = keys.dragged() if mouse or on_drag is not None else (0, 0)
                if dx or dy:
                    scroll_drag(board_view, dy)
                    drag(dx, dy)
                px, py = keys.panned()
                if on_pan is not None and (px or py):
                    on_pan(px, py)
    except KeyboardInterrupt:
        return None
    finally:
        _DRAWN_AT[0] = None
        gc.unfreeze()
        if stalls is not None:
            faulthandler.cancel_dump_traceback_later()
            stalls.close()


def _stall_log():
    """STALL_LOG open to append, a line naming the page's start; None where it cannot be."""
    try:
        os.makedirs(os.path.dirname(STALL_LOG), exist_ok=True)
        out = open(STALL_LOG, 'a', encoding='utf-8')
        out.write('%s %s\n' % (time.strftime('%Y-%m-%d %H:%M:%S'), ' '.join(sys.argv)))
        out.flush()
        return out
    except OSError:
        return None


class Freshness:
    """Whether the board's counter moved, and how fast.

    A draw faster than the board's readings repeats one, which is not a dead
    link: the reading is called stale only once it has stood still for
    STILL_S. The emulated board repeated 24 of 60 frames at 20 fps, and the
    note flickered live/stale every other frame.
    """

    #: How long the counter stands still before the reading is stale, s.
    STILL_S = 0.75

    def __init__(self):
        self.seen, self.stale = -1, 0
        self.rate, self._rate_seen, self._rate_at = 0.0, None, time.time()
        self._moved_at = time.time()

    def take(self, updates):
        """One frame's counter, or None for no reading."""
        now = time.time()
        if updates is None or updates == self.seen:
            self.stale += 1
        else:
            self.seen, self.stale, self._moved_at = updates, 0, now
        if updates is None:
            return
        if now - self._rate_at < 1.0:
            return
        if self._rate_seen is not None:
            self.rate = (updates - self._rate_seen) / (now - self._rate_at)
        self._rate_seen, self._rate_at = updates, now

    @property
    def still(self):
        """Seconds the counter has stood still."""
        return time.time() - self._moved_at if self.stale else 0.0

    @property
    def note(self):
        still = self.still
        return ('stale %.1f s' % still) if still >= self.STILL_S else 'live'


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
