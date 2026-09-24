"""Pull a tag through the daemon, and draw the download as it comes."""
import argparse
import json
import sys
import time
import urllib.error
import urllib.request

from .client import OllamaError, is_cloud, is_local
from .spinner import _vt

#: The bar's cells, in braille as the rest of the tree draws: a full cell,
#: the left column alone for the half step, and the bottom row for what is
#: still to come - the thermometer tube's grey dots. Half-cell resolution,
#: so WIDTH cells are 2 * WIDTH steps of the layer.
BRAILLE = ('⣿', '⡇', '⣀')
#: For a stream that cannot hold braille (--ascii, or a console whose
#: codec says so): the same three roles.
ASCII = ('=', '>', '-')
WIDTH = 24
#: Off a TTY the row cannot be rewritten, so it is printed whole every
#: this many percent of the layer, and at every change of status.
STEP_PERCENT = 5
#: Seconds the socket may sit silent before the pull is given up on: one
#: read, not the whole download - a layer that stalls for ten minutes is a
#: dead link, and one that takes an hour at a steady rate is a big model.
TIMEOUT_S = 600.0
DEFAULT_HOST = 'http://localhost:11434'

_COLOUR = {'ok': '\x1b[32m', 'wait': '\x1b[36m', 'warn': '\x1b[33m',
           'fail': '\x1b[31m'}
_DIM, _RESET = '\x1b[90m', '\x1b[0m'


def bar(completed, total, width=WIDTH, glyphs=BRAILLE):
    """`width` cells of `completed / total`, at half-cell resolution."""
    full, half, empty = glyphs
    if total <= 0:
        return empty * width
    share = max(0.0, min(1.0, float(completed) / float(total)))
    steps = int(round(2 * width * share))
    cells = full * (steps // 2) + half * (steps % 2)
    return cells + empty * (width - len(cells))


def gigabytes(count):
    """'7.6 GB', '412 MB', '38 kB': the size in the unit that reads."""
    count = float(count)
    if count >= 1e9:
        return '%.1f GB' % (count / 1e9)
    if count >= 1e6:
        return '%.0f MB' % (count / 1e6)
    if count >= 1e3:
        return '%.0f kB' % (count / 1e3)
    return '%d B' % count


def clock(seconds):
    """'2 min 10 s', '48 s', '1 h 5 min'."""
    whole = int(round(seconds))
    if whole >= 3600:
        return '%d h %d min' % (whole // 3600, (whole % 3600) // 60)
    if whole >= 60:
        return '%d min %d s' % (whole // 60, whole % 60)
    return '%d s' % whole


class Progress:
    """The download as it stands, fed the daemon's events one at a time."""

    def __init__(self, tag, now=time.monotonic):
        self.tag = tag
        self.now = now
        self.began = now()
        self.status = ''
        self.digest = None
        self.downloading = False
        self.total = 0
        self.completed = 0
        self.layer_began = self.began
        self.layer_start = 0
        self.done_bytes = 0            # of the layers already finished

    def _next_layer(self, event, digest):
        """A new layer begins: the last one's bytes banked as done."""
        if self.digest is not None:
            self.done_bytes += self.total
        self.digest = digest
        self.total = int(event.get('total') or 0)
        self.completed = int(event.get('completed') or 0)
        self.layer_began, self.layer_start = self.now(), self.completed

    def feed(self, event):
        """One event. True when the status word changed."""
        status = str(event.get('status') or '')
        digest = event.get('digest')
        self.downloading = bool(digest)
        if digest and digest != self.digest:
            self._next_layer(event, digest)
        elif digest:
            self.total = int(event.get('total') or self.total)
            self.completed = int(event.get('completed') or 0)
        changed = status != self.status
        self.status = status
        return changed

    def percent(self):
        return 100.0 * self.completed / self.total if self.total > 0 else 0.0

    def rate(self):
        """Bytes a second over the layer so far; zero until it can be said."""
        elapsed = self.now() - self.layer_began
        moved = self.completed - self.layer_start
        return moved / elapsed if elapsed > 0.5 and moved > 0 else 0.0

    def pulled_bytes(self):
        return self.done_bytes + self.total

    def figures(self):
        """What follows the bar: the percent, the bytes of the bytes, and
        the rate and what is left at it once it can be said.
        """
        text = '%3.0f %%  %s of %s' % (
            self.percent(), gigabytes(self.completed), gigabytes(self.total))
        rate = self.rate()
        if rate > 0.0:
            text += '  %s/s  %s left' % (
                gigabytes(rate), clock((self.total - self.completed) / rate))
        return text

    def row(self, glyphs=BRAILLE):
        """The row's text: the tag, then the bar with its figures while a
        layer is coming down, the daemon's status word otherwise."""
        if not self.downloading:
            return '%s  %s' % (self.tag, self.status)
        return '%s  %s %s' % (
            self.tag, bar(self.completed, self.total, glyphs=glyphs),
            self.figures())


class Rows:
    """Say's columns on a stream: `  state  column  text` - rewritten in
    place on a TTY, in Say's colours, one line each otherwise."""

    def __init__(self, out):
        self.out = out
        self.tty = _vt(out)
        self.widest = 0

    def show(self, state, column, text, final=False, progress=None):
        """One row."""
        row = '  %-6s %-22s %s' % (state, column, text)
        pad = ' ' * max(0, self.widest - len(row))
        self.widest = max(self.widest, len(row))
        if self.tty:
            painted = '  %s%-6s%s %-22s %s%s%s' % (
                _COLOUR.get(state, ''), state, _RESET, column, _DIM, text, _RESET)
            self.out.write('\r' + painted + pad + ('\n' if final else ''))
        else:
            self.out.write(row + '\n')
        self.out.flush()


def carries(out, glyphs=BRAILLE):
    """Whether this stream's codec can hold the glyphs at all."""
    encoding = getattr(out, 'encoding', None) or 'ascii'
    try:
        ''.join(glyphs).encode(encoding)
    except (LookupError, UnicodeEncodeError):
        return False
    return True


def events(tag, host=DEFAULT_HOST, timeout=TIMEOUT_S):
    """The daemon's /api/pull for `tag`, one parsed line at a time."""
    request = urllib.request.Request(
        host.rstrip('/') + '/api/pull',
        data=json.dumps({'model': tag, 'stream': True}).encode('utf-8'),
        headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(request, timeout=timeout) as reply:
            for raw in reply:
                raw = raw.strip()
                if not raw:
                    continue
                try:
                    yield json.loads(raw.decode('utf-8'))
                except ValueError as exc:
                    raise OllamaError('the daemon sent a line that is not '
                                      'JSON: %r' % raw[:80]) from exc
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode('utf-8', 'replace')[:400]
        raise OllamaError('/api/pull %s: %s' % (exc.code, detail)) from exc
    except urllib.error.URLError as exc:
        raise OllamaError('cannot reach ollama at %s (%s). Is `ollama serve` '
                          'running?' % (host, exc.reason)) from exc


def pull(tag, host=DEFAULT_HOST, out=None, source=None, glyphs=None,
         now=time.monotonic, rows=None):
    """Pull `tag` and draw it; the daemon's last status word ('success')."""
    if is_cloud(tag):
        raise OllamaError('%s is a cloud tag: ollama runs those on their '
                          'hardware, and there is nothing to pull' % tag)
    if not is_local(host):
        raise OllamaError('%s is not this host: a pull goes to the local '
                          'daemon only' % host)
    out = out if out is not None else sys.stderr
    glyphs = glyphs or (BRAILLE if carries(out) else ASCII)
    rows = rows if rows is not None else Rows(out)
    state = Progress(tag, now)
    shown = -STEP_PERCENT
    for event in (source if source is not None else events(tag, host)):
        words = event.get('error')
        if words:
            rows.show('fail', 'model', '%s  %s' % (tag, words), final=True,
                      progress=state)
            raise OllamaError('ollama pull %s: %s' % (tag, words))
        changed = state.feed(event)
        step = int(state.percent()) // STEP_PERCENT * STEP_PERCENT
        if rows.tty or changed or step != shown:
            shown = step
            rows.show('wait', 'model', state.row(glyphs), progress=state)
    rows.show('ok', 'model', 'pulled %s, %s in %s' % (
        tag, gigabytes(state.pulled_bytes()), clock(now() - state.began)),
        final=True, progress=state)
    return state.status


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog='python -m coaxial_ollama.pull',
        description='Pull an ollama tag through the local daemon, drawn as '
                    'it comes.')
    parser.add_argument('tag', help='the tag, as `ollama list` would show it')
    parser.add_argument('--host', default=DEFAULT_HOST)
    parser.add_argument('--ascii', action='store_true',
                        help='draw the bar in ASCII whatever the console '
                             'says it can hold')
    args = parser.parse_args(argv)
    try:
        pull(args.tag, host=args.host, out=sys.stdout,
             glyphs=ASCII if args.ascii else None)
    except OllamaError as exc:
        print('ollama: %s' % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        sys.stdout.write('\n')
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
