#!/usr/bin/env python3
"""What a page makes of the input VS Code itself hands it: a VS Code of its own, driven over CDP.

    python -X utf8 tools/dev/vscode_input.py --raw       # every console record a page would read
    python -X utf8 tools/dev/vscode_input.py             # the terminal app's atlas, cells changed

A VS Code started apart - its own user data, no extensions, the DOM renderer so its terminal reads as
text, the remote debugging port open - opens `--folder` (the tree, its workspace settings in force), and
a terminal in it runs a reader of the console's records (`--raw`) or the terminal app on its atlas.
Chromium's own input pipeline is then handed what a hand does (CDP Input.dispatchMouseEvent): a left
drag, a chord of both buttons, a left drag after it, the wheel; with --raw each record read is printed,
else how many of the tree's cells each step changed against a pause with no input. Windows only.
"""
import argparse
import base64
import json
import os
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request

HOST = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
CODE = os.path.join(os.environ.get('LOCALAPPDATA', ''), 'Programs', 'Microsoft VS Code', 'Code.exe')
#: The instance's user settings: nothing of its own in the way, its terminal drawn in the DOM.
USER = {'workbench.startupEditor': 'none', 'security.workspace.trust.enabled': False,
        'window.restoreWindows': 'none', 'update.mode': 'none', 'telemetry.telemetryLevel': 'off',
        'terminal.integrated.enablePersistentSessions': False, 'terminal.integrated.gpuAcceleration': 'off',
        'terminal.integrated.defaultProfile.windows': 'PowerShell', 'workbench.tips.enabled': False,
        'chat.disableAIFeatures': True, 'workbench.panel.opensMaximized': 'always'}
#: The reader: the page's console modes and its mouse asked for, then every record as a line.
RAW = r'''
import ctypes, json, sys, time
sys.path.insert(0, sys.argv[1])
from terminal.ui import console
from terminal.ui.stage import _vt_on
_vt_on()
was = console._set_console_mode(mouse=True)
sys.stdout.write(console.MOUSE_ON + console.PASTE_ON + 'READING\r\n')
sys.stdout.flush()
k, Record = ctypes.windll.kernel32, console._RECORD
handle, log = k.GetStdHandle(-10), open(sys.argv[2], 'w', encoding='utf-8')
end = time.monotonic() + float(sys.argv[3])
while time.monotonic() < end:
    count = ctypes.c_uint()
    k.GetNumberOfConsoleInputEvents(handle, ctypes.byref(count))
    if count.value:
        buf, got = (Record * count.value)(), ctypes.c_uint()
        k.ReadConsoleInputW(handle, buf, count.value, ctypes.byref(got))
        t = round(time.monotonic(), 3)
        for r in buf[:got.value]:
            if r.type == 1:
                e = r.event.key
                log.write(json.dumps({'t': t, 'ch': e.ch, 'down': e.down, 'vk': e.vk, 'scan': e.scan,
                                      'state': e.state}) + '\n')
            elif r.type == 2:
                m = r.event.mouse
                log.write(json.dumps({'t': t, 'at': [m.pos.X, m.pos.Y], 'buttons': m.buttons,
                                      'flags': m.flags}) + '\n')
            else:
                log.write(json.dumps({'t': t, 'type': r.type}) + '\n')
        log.flush()
    time.sleep(0.005)
sys.stdout.write(console.MOUSE_OFF + console.PASTE_OFF + 'DONE\r\n')
sys.stdout.flush()
console._set_console_mode(was)
'''
#: The window's own view of each mouse, pointer, drag and focus event, caught first (capture), a run of
#: moves told once: what Chromium dispatched whatever the terminal made of it.
EVENTS_JS = ("(() => { window.__told = []; const last = () => window.__told[window.__told.length - 1] || '';"
             " for (const t of ['mousedown', 'mouseup', 'mousemove', 'contextmenu', 'auxclick', 'click', 'dragstart',"
             " 'dragend', 'drop', 'pointerdown', 'pointerup', 'pointercancel', 'gotpointercapture',"
             " 'lostpointercapture', 'selectstart', 'focusin', 'focusout'])"
             " window.addEventListener(t, e => { const s = t + ' button ' + e.button + ' buttons ' + e.buttons"
             " + ' on ' + String(e.target && (e.target.className || e.target.tagName)).slice(0, 48);"
             " if (t === 'mousemove' && last().startsWith(s.slice(0, 30))) return; window.__told.push(s); }, true);"
             " return true; })()")
ROWS_JS = ("(() => { const all = [...document.querySelectorAll('.xterm-rows')].filter(e => e.offsetParent);"
           " const e = all[all.length - 1]; if (!e) return null;"
           " const r = e.closest('.xterm').querySelector('.xterm-screen').getBoundingClientRect();"
           " return {rows: [...e.children].map(d => d.textContent), box: [r.left, r.top, r.width, r.height]}; })()")


class Socket:
    """A WebSocket's client end, what DevTools needs of one: text sent in masked frames, messages read whole."""

    def __init__(self, url):
        address, _, path = url[len('ws://'):].partition('/')
        name, _, port = address.partition(':')
        self.sock, self.rest = socket.create_connection((name, int(port)), timeout=60), b''
        key = base64.b64encode(os.urandom(16)).decode()
        self.sock.sendall(('GET /%s HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n'
                           'Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n' % (path, address, key)).encode())
        while b'\r\n\r\n' not in self.rest:
            self.rest += self._more()
        head, _, self.rest = self.rest.partition(b'\r\n\r\n')
        if b' 101 ' not in head.split(b'\r\n')[0]:
            raise OSError('no websocket at %s: %r' % (url, head[:80]))

    def _more(self):
        got = self.sock.recv(1 << 16)
        if not got:
            raise OSError('the websocket closed')
        return got

    def _take(self, n):
        while len(self.rest) < n:
            self.rest += self._more()
        out, self.rest = self.rest[:n], self.rest[n:]
        return out

    def _frame(self, opcode, data):
        n, mask = len(data), os.urandom(4)
        size = bytes([0x80 | n]) if n < 126 else (b'\xfe' + struct.pack('>H', n) if n < 1 << 16
                                                   else b'\xff' + struct.pack('>Q', n))
        self.sock.sendall(bytes([0x80 | opcode]) + size + mask + bytes(b ^ mask[k & 3] for k, b in enumerate(data)))

    def send(self, text):
        self._frame(1, text.encode('utf-8'))

    def recv(self):
        parts = []
        while True:
            first, second = self._take(2)
            n = second & 0x7F
            n = struct.unpack('>H', self._take(2))[0] if n == 126 else (
                struct.unpack('>Q', self._take(8))[0] if n == 127 else n)
            data, opcode = self._take(n), first & 0x0F
            if opcode == 8:
                raise OSError('the websocket closed')
            if opcode == 9:
                self._frame(10, data)
            elif opcode in (0, 1, 2):
                parts.append(data)
                if first & 0x80:
                    return b''.join(parts).decode('utf-8')


class Cdp:
    """One DevTools session: a call, its result."""

    def __init__(self, url):
        self.ws, self.n = Socket(url), 0

    def __call__(self, method, **params):
        self.n += 1
        self.ws.send(json.dumps({'id': self.n, 'method': method, 'params': params}))
        while True:
            told = json.loads(self.ws.recv())
            if told.get('id') == self.n:
                if 'error' in told:
                    raise RuntimeError('%s: %s' % (method, told['error']))
                return told.get('result', {})

    def js(self, expression):
        return self('Runtime.evaluate', expression=expression, returnByValue=True)['result'].get('value')

    def key(self, key, code, vk, text=''):
        self('Input.dispatchKeyEvent', type='keyDown' if text else 'rawKeyDown', key=key, code=code,
             windowsVirtualKeyCode=vk, text=text)
        self('Input.dispatchKeyEvent', type='keyUp', key=key, code=code, windowsVirtualKeyCode=vk)

    def mouse(self, kind, x, y, button='none', buttons=0, clicks=0):
        self('Input.dispatchMouseEvent', type=kind, x=x, y=y, button=button, buttons=buttons, clickCount=clicks)


def opened(folder, port, home):
    """(the VS Code process, a session on its workbench) - started apart in `home`, `folder` open."""
    data, extensions = os.path.join(home, 'data'), os.path.join(home, 'extensions')
    os.makedirs(os.path.join(data, 'User'), exist_ok=True)
    with open(os.path.join(data, 'User', 'settings.json'), 'w', encoding='utf-8') as f:
        json.dump(USER, f)
    # The extension host this may run under sets ELECTRON_RUN_AS_NODE and its own VSCODE_ hooks.
    env = {k: v for k, v in os.environ.items() if not k.startswith(('ELECTRON_', 'VSCODE_'))}
    proc = subprocess.Popen([CODE, '--user-data-dir', data, '--extensions-dir', extensions,
                             '--remote-debugging-port=%d' % port, '--new-window', '--skip-release-notes',
                             '--disable-workspace-trust', folder], env=env)
    end = time.monotonic() + 120
    while time.monotonic() < end:
        try:
            with urllib.request.urlopen('http://127.0.0.1:%d/json/list' % port, timeout=2) as answer:
                pages = [t for t in json.load(answer) if t.get('type') == 'page' and 'workbench' in t.get('url', '')]
            if pages:
                return proc, Cdp(pages[0]['webSocketDebuggerUrl'])
        except OSError:
            pass
        time.sleep(0.5)
    raise RuntimeError('VS Code never opened its workbench')


def command(cdp, name):
    """A workbench command run from the palette by its title."""
    cdp.key('F1', 'F1', 112)
    time.sleep(1.0)
    cdp('Input.insertText', text=name)
    time.sleep(1.0)
    cdp.key('Enter', 'Enter', 13, '\r')
    time.sleep(1.5)


def screen(cdp):
    """The active terminal's rows and its screen's box, or None before one draws."""
    return cdp.js(ROWS_JS)


def shown(cdp, word, seconds):
    """The terminal's rows once `word` is on them within `seconds`, else None."""
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        got = screen(cdp)
        if got and any(word in row for row in got['rows']):
            return got
        time.sleep(0.5)
    return None


def typed(cdp, text):
    cdp('Input.insertText', text=text)
    cdp.key('Enter', 'Enter', 13, '\r')


def drag(cdp, x, y, dx, steps=8):
    cdp.mouse('mouseMoved', x, y)
    cdp.mouse('mousePressed', x, y, 'left', 1, 1)
    for k in range(1, steps + 1):
        cdp.mouse('mouseMoved', x + dx * k / steps, y, 'left', 1)
        time.sleep(0.03)
    cdp.mouse('mouseReleased', x + dx, y, 'left', 0, 1)


def chord(cdp, x, y):
    """Both buttons as a hand presses them together: left, right, a little move, right up, left up."""
    cdp.mouse('mouseMoved', x, y)
    cdp.mouse('mousePressed', x, y, 'left', 1, 1)
    time.sleep(0.02)
    cdp.mouse('mousePressed', x, y, 'right', 3, 1)
    time.sleep(0.05)
    cdp.mouse('mouseMoved', x + 6, y + 3, 'left', 3)
    time.sleep(0.05)
    cdp.mouse('mouseReleased', x + 6, y + 3, 'right', 1, 1)
    time.sleep(0.02)
    cdp.mouse('mouseReleased', x + 6, y + 3, 'left', 0, 1)


def wheel(cdp, x, y, notches=3):
    for _ in range(notches):
        cdp('Input.dispatchMouseEvent', type='mouseWheel', x=x, y=y, deltaX=0, deltaY=-120)
        time.sleep(0.05)


def changed(before, after):
    """How many of the tree's cells (the left 60 % of the rows between the bands) differ."""
    rows = list(zip(before['rows'], after['rows']))[3:-3]
    width = int(0.6 * max(len(r) for r in before['rows']))
    return sum(a != b for x, y in rows for a, b in zip(x[:width], y[:width]))


def main():
    parser = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    parser.add_argument('--folder', default=os.path.dirname(HOST), help='the workspace VS Code opens')
    parser.add_argument('--raw', action='store_true', help='read the console records, not the app')
    parser.add_argument('--port', type=int, default=9339)
    parser.add_argument('--renderer', default='auto', choices=('auto', 'off'),
                        help="terminal.integrated.gpuAcceleration: auto (VS Code's own) or off (the DOM's, readable)")
    parser.add_argument('--shots', help='a folder for screenshots of the terminal after each step')
    args = parser.parse_args()
    USER['terminal.integrated.gpuAcceleration'] = args.renderer
    if not args.raw and args.renderer != 'off':
        parser.error('the atlas is read off the DOM renderer: --renderer off')
    home = tempfile.mkdtemp(prefix='vscode-input-')
    proc, cdp = opened(args.folder, args.port, home)
    try:
        time.sleep(4.0)
        command(cdp, 'Terminal: Create New Terminal')
        log = os.path.join(home, 'records.jsonl')
        if args.raw:
            script = os.path.join(home, 'raw.py')
            with open(script, 'w', encoding='utf-8') as f:
                f.write(RAW)
            # The shell takes the line when it is ready; the reader's log is there once it reads.
            time.sleep(3.0)
            typed(cdp, 'python -X utf8 "%s" "%s" "%s" 14' % (script, HOST, log))
            end = time.monotonic() + 60
            while not os.path.exists(log) and time.monotonic() < end:
                time.sleep(0.2)
            time.sleep(1.0)
            box = cdp.js("(() => { const r = [...document.querySelectorAll('.xterm-screen')].filter(e => "
                         "e.offsetParent).pop().getBoundingClientRect(); return [r.left, r.top, r.width, r.height]; })()")
            got = {'rows': [], 'box': box} if os.path.exists(log) else None
            cdp.js(EVENTS_JS)
        else:
            if not shown(cdp, 'PS ', 30):
                raise RuntimeError('no shell prompt in the terminal')
            typed(cdp, '.\\system_terminal.ps1')
            if not shown(cdp, 'QUIT', 120):
                raise RuntimeError('the front page never showed')
            cdp('Input.insertText', text='V')
            got = shown(cdp, 'LAYERS', 60)
            time.sleep(3.0)
            got = screen(cdp)
        if not got:
            last = screen(cdp) or {'rows': []}
            raise RuntimeError('the reader or the atlas never showed:\n' +
                               '\n'.join(row.rstrip() for row in last['rows'] if row.strip()))
        left, top, width, height = got['box']
        x, y = left + 0.30 * width, top + 0.5 * height

        def step(name, act):
            before = None if args.raw else screen(cdp)
            if args.raw:
                cdp.js("window.__told.push('-- %s')" % name)
            act()
            time.sleep(1.0)
            if not args.raw:
                print('%-22s %5d cells changed' % (name, changed(before, screen(cdp))))
            if args.shots:
                os.makedirs(args.shots, exist_ok=True)
                png = cdp('Page.captureScreenshot', format='png',
                          clip={'x': left, 'y': top, 'width': width, 'height': height, 'scale': 1})['data']
                with open(os.path.join(args.shots, name.replace(' ', '_') + '.png'), 'wb') as f:
                    f.write(base64.b64decode(png))
        step('no input', lambda: None)
        step('a left drag', lambda: drag(cdp, x, y, 120))
        step('the chord', lambda: chord(cdp, x, y))
        step('a left drag after it', lambda: drag(cdp, x, y, 120))
        step('the wheel', lambda: wheel(cdp, x, y))
        step('a key: z', lambda: cdp('Input.insertText', text='z'))
        if args.raw:
            print('\n'.join(cdp.js('window.__told') or []))
            time.sleep(8.0)
            with open(log, encoding='utf-8') as f:
                records = [json.loads(line) for line in f]
            reads = {}
            for r in records:
                reads.setdefault(r['t'], []).append(r)
            for t, got in reads.items():
                text = ''.join(r['ch'] for r in got if 'ch' in r and r['down'] and r['ch'] not in ('', '\x00'))
                other = [r for r in got if 'ch' not in r]
                print('%.3f  %r%s' % (t, text, '  ' + str(other) if other else ''))
        else:
            cdp('Input.insertText', text='q')
            time.sleep(2.0)
    finally:
        try:
            cdp('Browser.close')
        except (RuntimeError, OSError):
            proc.kill()
        proc.wait(30)
        shutil.rmtree(home, ignore_errors=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
