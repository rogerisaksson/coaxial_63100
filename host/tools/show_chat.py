"""BOARD CHAT: the CCC prompt drawn inside the stage.

The transcript rides a frame under the title band, the input line sits
above the key bar, and the local model answers through the same Chat the
bench prompt (host/board_chat.ps1) drives - a terminal in the terminal.
ESC returns to the menu. Q is a letter here, so the only ways out are
ESC and Ctrl+C. `--frames` draws the page with a canned transcript and
no model at all - the smoke path, like every view.
"""
import argparse
import os
import sys
import threading
import time

sys.path.insert(0, __file__.rsplit('tools', 1)[0])
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rich.layout import Layout                             # noqa: E402
from rich.panel import Panel                               # noqa: E402
from rich.text import Text                                 # noqa: E402
from rich import box                                       # noqa: E402

from screen import (TO_MENU, Keys, curtain, footer,        # noqa: E402
                    header, paced, stage)

#: Rows the page spends outside the transcript: band, input, key bar,
#: and the frame's own two edges.
RESERVE = 5

#: One cell, ten frames: the busy glyph, where the prompt arrow sat.
#: The growing THINKING dots in the key bar wobbled the whole row.
SPIN = '⠋⠙⠹⠸⠼⠴⠦⠧⠇⠏'


class _Origin:
    """What header() asks of a session origin, for a chat that has none."""

    def __init__(self, label, real, port):
        self.label, self.real, self.port = label, real, port


class Script:
    """The transcript: what was said, one styled row per line."""

    def __init__(self):
        self.rows = []
        self.lock = threading.Lock()
        #: None follows the tail; an int is a scrolled-back first row.
        self.pin = None

    def say(self, style, text):
        with self.lock:
            for line in str(text).split('\n'):
                self.rows.append((style, line))

    def window(self, room):
        with self.lock:
            total = len(self.rows)
            start = (max(0, total - room) if self.pin is None
                     else min(self.pin, max(0, total - room)))
            return list(self.rows[start:start + room])


class _Taps:
    """A file for Chat's trace prints: every line lands in the
    transcript, dim, instead of tearing through the frame."""

    encoding = 'utf-8'

    def __init__(self, script):
        self._script, self._part = script, ''

    def write(self, text):
        self._part += text
        while '\n' in self._part:
            line, self._part = self._part.split('\n', 1)
            if line.strip():
                self._script.say('label', '  ' + line.rstrip())
        return len(text)

    def flush(self):
        pass


def open_chat(a, script):
    """The same Chat the bench prompt builds, its prints tapped."""
    from coaxial_ollama import cli, language

    # NOT --quiet: quiet suppresses _trace, and _trace is where a tool
    # result's value grid prints - without it the model's one-line summary
    # is all that reaches the transcript, values nowhere. Measured: "the
    # values are shown on screen", and they were not.
    argv = ['-m', 'auto', '--port', a.port]
    if a.simulated:
        argv.append('--simulated')
    args = cli.parse(argv)
    client, _session, chat = cli.build(args)
    client.model = client.require_model()
    chat.io_log = cli.IOLog()
    chat.compile_intent = True
    chat.out = _Taps(script)
    script.say('name', language.greeting(client.model, chat.language,
                                         'utf-8'))
    return chat


#: What -p is told about this page. CLAUDE.md routes routine board work
#: to the local model and has claude ASK "local model, or here?" - on
#: this page the operator picked Anthropic, so the question is answered.
PAGE = ('You are the ANTHROPIC page of coaxial_tty. The operator chose '
        'you over the local model by opening this page, so the '
        'local-model routing and the "Local model, or here?" question in '
        'CLAUDE.md do not apply: drive the board yourself through the '
        'coaxial MCP tools and answer here, briefly.')


class _Claude:
    """The ANTHROPIC backend: one `claude -p` per turn, continued in the
    repo root. The MCP config is generated with absolute paths and passed
    explicitly: the project .mcp.json sits `pending approval` until an
    interactive run approves it, and -p cannot ask - measured as a page
    that answered nothing at all."""

    def __init__(self, port, script):
        import json
        import tempfile

        self.turns = 0
        self.script = script
        self.proc = None
        self.root = os.path.dirname(os.path.dirname(
            os.path.dirname(os.path.abspath(__file__))))
        host = os.path.join(self.root, 'host')
        spec = {'mcpServers': {'coaxial': {
            'command': sys.executable,
            'args': ['-m', 'coaxial_mcp', '--port', port],
            'cwd': host,
            'env': {'PYTHONPATH': host, 'PYTHONUNBUFFERED': '1'}}}}
        self.config = os.path.join(tempfile.gettempdir(),
                                   'coaxial_mcp_config.json')
        with open(self.config, 'w', encoding='utf-8') as f:
            json.dump(spec, f)

    def command(self, _line):
        return None

    def close(self):
        """A turn left running when the page closes keeps talking into
        the terminal the menu takes back - measured as spam after an
        exit. Kill claude; its MCP server follows when stdin closes."""
        proc = self.proc
        if proc and proc.poll() is None:
            proc.kill()

    def _tell(self, event, answer):
        """One stream-json event: tool calls to the transcript as they
        happen, the result kept for the caller."""
        if event.get('type') == 'assistant':
            content = (event.get('message') or {}).get('content') or ()
            for block in content:
                if isinstance(block, dict) and block.get('type') == 'tool_use':
                    name = str(block.get('name'))
                    self.script.say('label',
                                    '  mcp: %s' % name[len('mcp__coaxial__'):]
                                    if name.startswith('mcp__coaxial__')
                                    else '  tool: %s' % name)
        elif event.get('type') == 'result':
            answer.append(event.get('result') or '')

    def ask(self, line):
        import json
        import subprocess

        cmd = (['claude', '-p'] + (['--continue'] if self.turns else [])
               + [line, '--allowedTools', 'mcp__coaxial',
                  '--mcp-config', self.config, '--strict-mcp-config',
                  '--append-system-prompt', PAGE,
                  '--output-format', 'stream-json', '--verbose'])
        self.turns += 1
        proc = subprocess.Popen(cmd, cwd=self.root, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, text=True,
                                encoding='utf-8', errors='replace')
        self.proc = proc
        answer = []
        for raw in proc.stdout:
            try:
                self._tell(json.loads(raw), answer)
            except ValueError:
                pass
        proc.wait(timeout=60)
        self.proc = None
        if proc.returncode != 0 and not any(answer):
            return 'claude exited %d: %s' % (
                proc.returncode, proc.stderr.read().strip()[:200])
        return '\n'.join(a for a in answer if a) or '(no answer)'


def mcp_ready(chat, port, script, step):
    """Prove the coaxial MCP server starts before the first turn needs it:
    run it once against a closed stdin - a stdio server answers its
    startup line and exits on the EOF. The line lands in the transcript,
    so a dead port or a broken PYTHONPATH shows before anybody types."""
    import subprocess

    host = os.path.join(chat.root, 'host')
    step(0.5, 'MCP SERVER')
    done = subprocess.run(
        [sys.executable, '-m', 'coaxial_mcp', '--port', port],
        cwd=host, stdin=subprocess.DEVNULL, capture_output=True,
        text=True, encoding='utf-8', errors='replace', timeout=30,
        env=dict(os.environ, PYTHONPATH=host, PYTHONUNBUFFERED='1'))
    said = (done.stdout or done.stderr).strip().splitlines()
    if said:
        script.say('label', '  mcp: %s' % said[0])
    if done.returncode != 0:
        script.say('value', 'coaxial_mcp exited %d - the page is up, the '
                            'board tools are not' % done.returncode)
    step(0.9, 'CLAUDE AT THE PROMPT')


def _turn(chat, line, script, state):
    """One question on a worker thread; the frame loop keeps drawing."""
    try:
        box = getattr(chat, 'toolbox', None)
        if box is not None:
            box.afe_mentioned = 'afe' in line.lower()
            box.asked = line
        done = chat.command(line)
        if done is None:
            done = chat.ask(line)
        if done:
            script.say(None, str(done))
    except Exception as exc:                    # noqa: BLE001 - shown, kept
        script.say('value', '%s: %s' % (type(exc).__name__, exc))
    finally:
        state['busy'] = False


def took(entry, key, script, state, chat):
    """The input line after one key; ENTER hands the line to the model."""
    if key in ('\r', '\n'):
        line = entry.strip()
        if line and not state['busy'] and chat is not None:
            script.say('value', '> ' + line)
            script.pin = None
            state['busy'] = True
            threading.Thread(target=_turn, args=(chat, line, script, state),
                             daemon=True).start()
            return ''
        return entry
    if key in ('\x08', '\x7f'):
        return entry[:-1]
    if len(key) == 1 and key.isprintable():
        return entry + key
    return entry


def scrolled(script, zoom, room):
    """A notch - arrow up positive - moves the window three rows; the
    tail resumes at the bottom."""
    with script.lock:
        total = len(script.rows)
    at = max(0, total - room) if script.pin is None else script.pin
    if zoom > 0:
        script.pin = max(0, at - 3)
    elif zoom < 0 and script.pin is not None:
        script.pin = at + 3
        if script.pin >= total - room:
            script.pin = None


def compose(script, entry, state, origin, size, lead, blink):
    height = size.height if size else 24
    room = max(3, height - RESERVE)
    body = Text()
    for style, line in script.window(room):
        body.append(line + '\n', style)
    ask = Text('  %s ' % lead, style='value')
    ask.append(entry)
    ask.append('_' if blink else ' ', style='value')
    keys = (('ENTER', 'SEND'), ('UP DOWN', 'SCROLL'), ('ESC', 'MENU'),
            ('CTRL+C', 'EXIT'))
    log = Panel(body, box=box.HEAVY, border_style='frame',
                title=Text(' %s ' % state['title'], style='name'),
                title_align='left', padding=(0, 1))
    mid = Layout(name='mid')
    if state['tools']:
        from screen import hud
        side = Layout(hud(state['served'], state['tools']),
                      name='tools', size=24)
        mid.split_row(Layout(log, name='log'), side)
    else:
        mid.update(log)
    whole = Layout()
    whole.split_column(
        Layout(header('BOARD CHAT', origin), size=1),
        mid,
        Layout(ask, size=1),
        Layout(footer(keys), size=1))
    return whole


def echo(page, entry, lead, blink):
    """The input row alone, straight to the terminal.

    A keystroke costs ~100 bytes this way instead of a 5-25 kB page:
    even rate-capped, full repaints per key queued a slow terminal
    renderer five seconds behind the fingers. Live repaints the same
    row with the same content whenever the page itself changes."""
    size = page.size
    ask = Text('  %s ' % lead, style='value')
    ask.append(entry)
    ask.append('_' if blink else ' ', style='value')
    line = ''.join(seg.style.render(seg.text) if seg.style else seg.text
                   for seg in page.render(ask) if seg.text != '\n')
    page.file.write('\x1b[%d;1H\x1b[2K%s' % ((size.height if size else 24)
                                             - 1, line))
    page.file.flush()


def canned(script):
    """The smoke transcript: the page with no model behind it."""
    script.say('label', 'SIMULATED SMOKE - no model loaded')
    script.say('value', '> read the NTC')
    script.say('label', '  analog_read: NTC')
    script.say(None, 'NTC: 25.00 C - the AFE is off, that is the label.')


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--simulated', action='store_true')
    p.add_argument('--claude', action='store_true',
                   help='claude -p per turn instead of the local model, '
                        'the coaxial MCP server in the room')
    p.add_argument('--frames', type=int, default=0,
                   help='draw this many with no model and exit - the smoke')
    a = p.parse_args()

    page = stage()
    console = page.is_terminal
    script, chat = Script(), None
    state = {'busy': False, 'tools': (), 'served': 'TOOLS',
             'title': ('CLAUDE - ANTHROPIC OVER MCP' if a.claude
                       else 'CCC - COAXIAL 63100 CHAT CLIENT')}
    if a.frames:
        canned(script)
        origin = _Origin('Simulated', False, a.port)
        state['tools'] = ('board_info', 'analog_read', 'docs')
    elif a.claude:
        import shutil
        if not shutil.which('claude'):
            print('claude is not on PATH - claude.ai/code has the install')
            return 2
        from screen import boot
        with boot('LINKING ANTHROPIC') as step:
            chat = _Claude(a.port, script)
            step(0.3, 'MCP CONFIG')
            mcp_ready(chat, a.port, script, step)
        origin = _Origin('claude + coaxial MCP', True, a.port)
        from coaxial_mcp.tools import TOOLS
        state['tools'] = tuple(spec['name'] for spec in TOOLS)
        state['served'] = 'MCP TOOLS'
        script.say('name', 'ANTHROPIC - one claude -p per turn, continued '
                           'in the repo root; the coaxial MCP tools ride '
                           'along. ESC returns to the menu.')
    else:
        from screen import boot
        with boot('LINKING MODEL'):
            chat = open_chat(a, script)
        origin = _Origin(chat.origin[0], chat.origin[1], a.port)
        state['tools'] = tuple(sorted(chat.tool_names))

    try:
        return _run(a, page, console, script, state, chat, origin)
    finally:
        closer = getattr(chat, 'close', None)
        if closer:
            closer()


def _run(a, page, console, script, state, chat, origin):
    """The frame loop. main() wraps it so a live claude turn dies
    with the page instead of talking into the returned terminal."""
    entry, frame, drawn, painted, face = '', 0, None, 0.0, None
    # NO mouse mode: with reporting on the terminal hands selections to
    # the view and copy stops working - in a chat, the transcript is
    # exactly what gets copied. Arrows scroll instead.
    with curtain(page) as show, Keys(console,
                                     quits=frozenset()) as keys:
        while True:
            frame += 1
            # Painted ONLY when something changed. A flat 20 Hz repaint
            # of the whole page queued the terminal's renderer up and
            # every keypress arrived seconds late - the lag was output
            # pressure, not input.
            blink = frame % 16 < 8
            lead = (SPIN[frame // 2 % len(SPIN)] if state['busy']
                    else '>')
            size = page.size
            # The page's mark carries NO entry: typing never repaints the
            # page, only its own row through echo(). A paint is 14-26 ms
            # and 5-25 kB of ANSI, measured 100x28 to 280x70; ten a
            # second at most even so.
            mark = (len(script.rows), script.pin,
                    size.width if size else 0,
                    size.height if size else 0)
            now = time.monotonic()
            if mark != drawn and now - painted >= 0.1:
                show.update(compose(script, entry, state, origin, size,
                                    lead, blink), refresh=True)
                drawn, painted, face = mark, now, None
            want = (entry, lead, blink)
            if console and want != face:
                echo(page, entry, lead, blink)
                face = want
            if a.frames and frame >= a.frames:
                return 0
            leave, _zoom, typed = paced(keys, 0.05)
            if leave == 'menu':
                return TO_MENU
            if leave:
                return 0
            for key in typed:
                if key in ('up', 'down'):
                    height = page.size.height if page.size else 24
                    scrolled(script, 1 if key == 'up' else -1,
                             max(3, height - RESERVE))
                else:
                    entry = took(entry, key, script, state, chat)


if __name__ == '__main__':
    sys.exit(main())
