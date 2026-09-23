"""BOARD CHAT: the CCC prompt drawn inside the stage.

The transcript rides a frame under the title band, the input line sits
above the key bar, and the local model answers through the same Chat the
bench prompt (host/board_chat.ps1) drives - a terminal in the terminal.
ESC returns to the menu. Q is a letter here, so the only ways out are
ESC and Ctrl+C. `--frames` draws the page with a canned transcript and
no model at all - the smoke path, like every view.
"""
import argparse
import glob
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from contextlib import suppress

sys.path.insert(0, __file__.rsplit('tools', 1)[0])
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from rich.layout import Layout                             # noqa: E402
from rich.panel import Panel                               # noqa: E402
from rich.text import Text                                 # noqa: E402
from rich import box                                       # noqa: E402

from screen import (ENTER_KEYS, TO_MENU, Keys, boot, curtain,  # noqa: E402
                    footer, header, hud, paced, stage)
from coaxial_ollama import cli, language, pull as pulling  # noqa: E402
from coaxial_ollama.client import OllamaError              # noqa: E402
from coaxial_mcp.tools import TOOLS                        # noqa: E402

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


def fit(said, room):
    """`said` within `room` cells, cut at a gap between its figures and
    never through one: the strip's text column wraps past the console's
    width, and a strip on two rows is two strips."""
    if len(said) <= room:
        return said
    gap = said.rfind('  ', 0, room + 1)
    return said[:gap] if gap > 0 else said[:room]


class Strip:
    """pull()'s rows on the boot strip: the bar is the layer's share and the
    bracketed text the pull's own figures, in place of pull's row.
    """

    #: Every event, not one per five percent: the strip is a TTY's.
    tty = True

    def __init__(self, step, room):
        self._step, self._room = step, room

    def show(self, state, column, text, final=False, progress=None):
        if final or progress is None:
            self._step(1.0 if state == 'ok' else 0.0, fit(text, self._room))
            return
        said = progress.figures() if progress.downloading else progress.status
        self._step(progress.percent() / 100.0,
                   fit('PULLING %s  %s' % (progress.tag, said), self._room))


def pulled_on(strip):
    """ensure_pulled's pull, drawn on `strip` - or on pull's own rows
    when the page is not a terminal and there is no strip."""

    def pull_with(tag, host, out):
        return pulling.pull(tag, host=host, out=out, rows=strip)
    return pull_with


def open_chat(a, script, strip=None):
    """The same Chat the bench prompt builds, its prints tapped; the
    picker's tag pulled first when `ollama list` lacks it, on `strip`."""

    # NOT --quiet: quiet suppresses _trace, and _trace is where a tool result's
    # value grid prints - without it the model's one-line summary is all that
    # reaches the transcript, values nowhere.
    argv = ['-m', 'auto', '--port', a.port]
    if a.simulated:
        argv.append('--simulated')
    args = cli.parse(argv)
    client, _session, chat = cli.build(args)
    # Through ensure_pulled, as dbg.py's start and the bench prompt's
    # preflight: the picker names the tag that fits this card, and the day it
    # named one `ollama list` lacked (2026-09-22, llama3.1:8b beside a pulled
    # gemma4:12b that did not fit) this page died in a traceback with the
    # command to type as its last line.
    client.model = cli.ensure_pulled(client, sys.stderr,
                                     pull_with=pulled_on(strip))
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


def find_claude():
    """claude, wherever this machine keeps it - a path, or None."""

    found = shutil.which('claude')
    if found:
        return found
    home = os.path.expanduser('~')
    fixed = [os.path.join(home, '.local', 'bin', 'claude.exe'),
             os.path.join(os.environ.get('APPDATA', ''), 'npm',
                          'claude.cmd')]
    bundled = sorted(glob.glob(os.path.join(
        home, '.vscode', 'extensions', 'anthropic.claude-code-*',
        'resources', 'native-binary', 'claude.exe')), reverse=True)
    for path in fixed + bundled:
        if path and os.path.isfile(path):
            return path
    return None


class _Claude:
    """The ANTHROPIC backend: one `claude -p` per turn, continued in the
    repo root.
    """

    #: No toolbox of its own: the tools are the MCP server's, in claude's
    #: process.
    toolbox = None

    def __init__(self, port, script, exe='claude'):

        self.turns = 0
        self.script = script
        self.proc = None
        self.exe = exe
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
        """A turn left running when the page closes keeps talking into the
        terminal the menu takes back - measured as spam after an exit.
        """
        proc = self.proc
        if proc and proc.poll() is None:
            proc.kill()

    def _tell(self, event, answer):
        """One stream-json event: tool calls to the transcript as they
        happen, the result kept for the caller."""
        if event.get('type') == 'assistant':
            content = (event.get('message') or {}).get('content') or ()
            for block in (b for b in content if isinstance(b, dict)
                          and b.get('type') == 'tool_use'):
                self.script.say('label', _tool_line(str(block.get('name'))))
        elif event.get('type') == 'result':
            answer.append(event.get('result') or '')

    def ask(self, line):

        cmd = ([self.exe, '-p'] + (['--continue'] if self.turns else [])
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
        if proc.stdout is None or proc.stderr is None:
            return 'claude gave no pipes to read'
        for raw in proc.stdout:
            with suppress(ValueError):
                self._tell(json.loads(raw), answer)
        proc.wait(timeout=60)
        self.proc = None
        if proc.returncode != 0 and not any(answer):
            return 'claude exited %d: %s' % (
                proc.returncode, proc.stderr.read().strip()[:200])
        return '\n'.join(a for a in answer if a) or '(no answer)'


def mcp_ready(chat, port, script, step):
    """Prove the coaxial MCP server starts before the first turn needs it:
    run it once against a closed stdin - a stdio server answers its
    startup line and exits on the EOF.
    """

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
        box = chat.toolbox
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


def _sent(line, script, state, chat):
    """ENTER: the line goes to the model on its own thread - unless it is
    empty, a turn is still running, or there is no model.
    """
    if not line or state['busy'] or chat is None:
        return False
    script.say('value', '> ' + line)
    script.pin = None
    state['busy'] = True
    threading.Thread(target=_turn, args=(chat, line, script, state),
                     daemon=True).start()
    return True


def took(entry, key, script, state, chat):
    """The input line after one key; ENTER hands the line to the model."""
    if key in ENTER_KEYS:
        return '' if _sent(entry.strip(), script, state, chat) else entry
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
        script.pin = None if at + 3 >= total - room else at + 3


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
    """The input row alone, straight to the terminal."""
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


MCP_PREFIX = 'mcp__coaxial__'


def _tool_line(name):
    """A tool call as the transcript shows it: the board's tools by their
    own names, anything else as the tool it is."""
    if name.startswith(MCP_PREFIX):
        return '  mcp: %s' % name[len(MCP_PREFIX):]
    return '  tool: %s' % name


def _claude_chat(a, script, state):
    """claude -p per turn with the coaxial MCP server in the room - or
    exit 2, saying where claude comes from."""
    exe = find_claude()
    if exe is None:
        print('claude was not found - claude.ai/code has the install')
        raise SystemExit(2)
    with boot('LINKING ANTHROPIC') as step:
        chat = _Claude(a.port, script, exe)
        step(0.3, 'MCP CONFIG')
        mcp_ready(chat, a.port, script, step)
    state['tools'] = tuple(spec['name'] for spec in TOOLS)
    state['served'] = 'MCP TOOLS'
    script.say('name', 'ANTHROPIC - one claude -p per turn, continued '
                       'in the repo root; the coaxial MCP tools ride '
                       'along. ESC returns to the menu.')
    return chat, _Origin('claude + coaxial MCP', True, a.port)


def main():
    p = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
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
        chat, origin = _claude_chat(a, script, state)
    else:
        try:
            with boot('LINKING MODEL') as step:
                # The strip's bar is 28 cells, its text bracketed a cell on;
                # what is left of the row is the pull's.
                width = page.size.width if page.size else 80
                strip = Strip(step, max(24, width - 34)) if console else None
                chat = open_chat(a, script, strip)
        except OllamaError as exc:
            # The daemon's words on one line and exit 2, as dbg.py's start
            # ends: the chooser prints the code, says the last lines above say
            # why, and waits for a key - so those lines are the words, not
            # forty of traceback over them.
            print('ollama: %s' % exc, file=sys.stderr)
            return 2
        label, real = chat.origin or ('unknown', False)
        origin = _Origin(label, real, a.port)
        state['tools'] = tuple(sorted(chat.tool_names))

    try:
        return _run(a, page, console, script, state, chat, origin)
    finally:
        if chat is not None:
            chat.close()


def _run(a, page, console, script, state, chat, origin):
    """The frame loop."""
    entry, frame, drawn, painted, face = '', 0, None, 0.0, None
    # NO mouse mode: with reporting on the terminal hands selections to the
    # view and copy stops working - in a chat, the transcript is exactly what
    # gets copied.
    with curtain(page) as show, Keys(console,
                                     quits=frozenset()) as keys:
        while True:
            frame += 1
            # Painted ONLY when something changed.
            blink = frame % 16 < 8
            lead = (SPIN[frame // 2 % len(SPIN)] if state['busy']
                    else '>')
            size = page.size
            # The page's mark carries NO entry: typing never repaints the page,
            # only its own row through echo().
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
