#!/usr/bin/env python3
"""End-to-end test of the MCP server, plus a token accounting.

Drives the server as a real subprocess over stdio JSON-RPC rather than calling
the handlers directly, so the transport, the schema validation and the framing
are all exercised. The board must be attached: these are live measurements.

Run from the host directory:  python tests/test_mcp.py
"""
import json
import os
import subprocess
import sys
import time

# host/ on the path: this file's own directory's parent, so it does not
# matter what the working directory is.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PROTOCOL_VERSION = '2024-11-05'


def approx_tokens(text):
    """Rough but honest: ~4 characters per token for dense ASCII."""
    return max(1, len(text) // 4)


class ServerProcess:
    """A running MCP server, spoken to in newline-delimited JSON-RPC."""

    def __init__(self, args):
        self.proc = subprocess.Popen(
            [sys.executable, '-u', '-m', 'coaxial_mcp'] + args,
            stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True, bufsize=1)
        self.next_id = 0

    def send(self, method, params=None, notify=False):
        message = {'jsonrpc': '2.0', 'method': method}
        if params is not None:
            message['params'] = params
        if not notify:
            self.next_id += 1
            message['id'] = self.next_id
        self.proc.stdin.write(json.dumps(message) + '\n')
        self.proc.stdin.flush()
        return None if notify else self.next_id

    def read(self, want_id, timeout=30.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            line = self.proc.stdout.readline()
            if not line:
                raise RuntimeError('server closed stdout; stderr:\n%s'
                                   % self.proc.stderr.read())
            message = json.loads(line)
            if message.get('id') == want_id:
                return message
        raise TimeoutError('no reply to request %d in %.0fs' % (want_id, timeout))

    def call(self, method, params=None, timeout=30.0):
        return self.read(self.send(method, params), timeout)

    def tool(self, name, arguments=None, timeout=30.0):
        reply = self.call('tools/call',
                          {'name': name, 'arguments': arguments or {}}, timeout)
        if 'error' in reply:
            raise RuntimeError('%s: %s' % (name, reply['error']))
        blocks = reply['result']['content']
        return '\n'.join(b['text'] for b in blocks if b['type'] == 'text')

    def close(self):
        try:
            self.proc.stdin.close()
        except OSError:
            pass
        try:
            self.proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.proc.kill()


class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.tokens = 0

    def check(self, name, condition, detail=''):
        if condition:
            self.passed += 1
            print('  PASS  %-34s %s' % (name, detail))
        else:
            self.failed += 1
            print('  FAIL  %-34s %s' % (name, detail))

    def result(self, name, text, must_contain=()):
        """Record a tool result, its size, and that it says what it should."""
        cost = approx_tokens(text)
        self.tokens += cost
        missing = [needle for needle in must_contain if needle not in text]
        first = text.splitlines()[0] if text else '<empty>'
        self.check(name, not missing and not text.startswith('ERR'),
                   '%3d tok  %s' % (cost, missing or first[:46]))
        return text


def handshake(server, report):
    reply = server.call('initialize', {
        'protocolVersion': PROTOCOL_VERSION,
        'capabilities': {},
        'clientInfo': {'name': 'test_mcp', 'version': '1'},
    })
    report.check('initialize', 'result' in reply,
                 reply.get('result', {}).get('serverInfo', reply.get('error')))
    server.send('notifications/initialized', notify=True)


def tool_list(server, report):
    reply = server.call('tools/list')
    tools = reply['result']['tools']
    blob = json.dumps(tools, separators=(',', ':'))

    # Compare against what the module declares rather than a number written here,
    # so adding a tool does not fail this test for the wrong reason.
    from coaxial_mcp.tools import HANDLERS
    served = {tool['name'] for tool in tools}
    report.check('tools/list matches the handler table', served == set(HANDLERS),
                 '%d tools, %d chars, ~%d tok for the whole list'
                 % (len(tools), len(blob), approx_tokens(blob)))
    for tool in tools:
        oversize = len(tool['description']) > 140
        report.check('desc fits: ' + tool['name'], not oversize,
                     '%d chars' % len(tool['description']))
    return tools


def exercise(server, report):
    print('\n-- tools, live against the board --')
    report.result('board_info', server.tool('board_info'),
                  ['coaxial_63100', 'sysclk', 'ch adc'])
    report.result('afe_power on', server.tool('afe_power', {'action': 'on'}),
                  ['on=1'])
    report.result('analog_read all', server.tool('analog_read'),
                  ['NTC', 'DCbus', 'C', 'V bus'])
    report.result('analog_read by name',
                  server.tool('analog_read', {'ch': ['NTC', 'DCbus'],
                                              'samples': 32}),
                  ['NTC', 'DCbus'])
    report.result('analog_read custom beta',
                  server.tool('analog_read', {'ch': ['NTC'], 'ntc_beta': 3950}),
                  ['NTC'])
    report.result('test_gate open', server.tool('test_gate', {'enable': True}),
                  ['gate=1'])
    report.result('gpio_pin read',
                  server.tool('gpio_pin', {'op': 'read', 'pin': 'E15'}),
                  ['E15='])
    report.result('gpio_pin write',
                  server.tool('gpio_pin', {'op': 'write', 'pin': 'B2',
                                           'level': True}),
                  ['readback'])
    report.result('gpio_pin mode',
                  server.tool('gpio_pin', {'op': 'mode', 'pin': 'E15',
                                           'mode': 'input', 'pull': 'up'}),
                  ['mode=input'])
    report.result('gpio_port read',
                  server.tool('gpio_port', {'op': 'read', 'port': 'E'}),
                  ['GPIOE=0x'])
    # Writing 0 across all of GPIOB would clear PB10/PB11 and sever the link.
    # The firmware masks those out - and legitimately DOES clear PB2, which is
    # the AFE switch, so the reading afterwards proves both halves at once.
    report.result('gpio_port write masks reserved',
                  server.tool('gpio_port', {'op': 'write', 'port': 'B',
                                            'mask': 0xFFFF, 'value': 0}),
                  ['GPIOB=0x', 'reserved:'])
    report.check('link survived the masked write',
                 'echo ok' in server.tool('link', {'op': 'echo', 'text': 'alive'}),
                 'PB10/PB11 held while the rest of GPIOB went low')
    report.check('unreserved PB2 really was cleared',
                 server.tool('afe_power', {'action': 'read'}).startswith('on=0'),
                 'the AFE switch is not reserved, so the write reached it')
    server.tool('afe_power', {'action': 'on'})
    report.result('link echo', server.tool('link', {'op': 'echo',
                                                    'text': 'hello mcp'}),
                  ['echo ok'])
    report.result('link stats', server.tool('link', {'op': 'stats'}),
                  ['bus_message='])
    report.result('test_gate close', server.tool('test_gate', {'enable': False}),
                  ['gate=0'])


def error_paths(server, report):
    print('\n-- errors are one terse, actionable line --')
    for name, args, expect in [
        ('unknown channel', {'ch': ['Vbat']}, 'unknown channel'),
        ('channel out of range', {'ch': ['99']}, 'out of range'),
    ]:
        text = server.tool('analog_read', args)
        report.check('analog_read ' + name,
                     text.startswith('ERR') and expect in text,
                     '%d tok  %s' % (approx_tokens(text), text[:52]))

    text = server.tool('gpio_pin', {'op': 'read', 'pin': 'ZZ'})
    report.check('bad pin name', text.startswith('ERR'), text[:60])

    text = server.tool('gpio_pin', {'op': 'write', 'pin': 'B10', 'level': True})
    report.check('reserved pin refused with a reason',
                 text.startswith('ERR') and 'USART3' in text, text[:70])

    # Not refused: labelled. Refusing produced a fabricated reading rather
    # than preventing one - asked for the codes with the AFE deliberately off,
    # a model with no numbers wrote "Mid-scale ... 25.00 C" out of the warning
    # text. The codes come back, under a line that cannot be read as one.
    server.tool('afe_power', {'action': 'off'})
    text = server.tool('analog_read')
    report.check('AFE off is labelled, not refused',
                 not text.startswith('ERR') and text.startswith('AFE OFF'),
                 '%d tok  %s' % (approx_tokens(text), text[:56]))
    report.check('and the codes are actually there to read',
                 'NTC' in text and 'smp' in text, text.splitlines()[-1][:56])
    report.check('the label says how to make it a measurement',
                 'afe_power on' in text)
    server.tool('afe_power', {'action': 'on'})
    report.check('with the AFE on there is no banner',
                 not server.tool('analog_read').startswith('AFE OFF'))

    text = server.tool('nonexistent_tool')
    report.check('unknown tool', 'ERR unknown tool' in text, text[:50])


def weak_model_arguments(server, report):
    """An argument of the wrong type never reaches a handler here.

    A smaller model sends ch as a bare string and the numbers as strings -
    measured with llama3.1:8b on this board. On the ollama side that is
    coaxial_mcp.tools.coerce's problem, because nothing sits between the model
    and the handler there. On this side something does: the protocol validates
    against inputSchema first. What matters is that the answer is a refusal
    naming the field, not a TypeError from three frames down - the latter is
    what sends a model off to answer from memory.
    """
    print('\n-- arguments of the wrong type --')
    for name, args in [
        ('a channel as a bare string', {'ch': 'ntc'}),
        ('samples as a string', {'samples': '32'}),
        ('samples as a word', {'samples': 'many'}),
    ]:
        text = server.tool('analog_read', args)
        report.check('analog_read refuses ' + name,
                     'not of type' in text or text.startswith('ERR'),
                     text[:60])


def main():
    # --auto, not --port alone: with no board on COM4 the server serves a
    # stand-in rather than failing every call, and this suite is then
    # testing the MCP layer - the schemas, the JSON-RPC, the argument
    # coercion, the render - which is all of it that does not need
    # firmware. What it is NOT testing then is the firmware, so the tally
    # says which it ran against and never leaves that to be assumed.
    from coaxial_mcp.session import open_session
    session, found = open_session('COM4', simulated=None)
    session.close()
    server = ServerProcess(['--port', found.port or 'COM4']
                           + ([] if found.real else ['--simulated']))
    print('-- against %s --'
          % (found.label if found.real else 'a SIMULATED board: nothing here '
             'says anything about the firmware'))
    report = Report()
    try:
        handshake(server, report)
        tools = tool_list(server, report)
        exercise(server, report)
        weak_model_arguments(server, report)
        error_paths(server, report)
        server.tool('link', {'op': 'release'})
    finally:
        server.close()

    blob = json.dumps(tools, separators=(',', ':'))
    print('\n-- token accounting --')
    print('  tool list          ~%4d tok  (paid once per turn)' % approx_tokens(blob))
    print('  all tool results   ~%4d tok  across %d calls'
          % (report.tokens, report.passed + report.failed))
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
