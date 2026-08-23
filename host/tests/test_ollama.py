#!/usr/bin/env python3
"""Offline test of the Ollama runner: no board, no ollama, no network.

The board and the model are both faked, on purpose. What is under test here is
not whether a language model can read a thermistor - that is what a bench is for
- but whether this runner keeps its promises when the model behaves badly:

  * the limit never reaches the model;
  * the verdict never comes from the model;
  * a model that loops, answers in prose, calls a tool that does not exist or
    writes code that raises still leaves a complete transcript and a recorded
    step;
  * a state change cannot happen without the operator's flag.

Every one of those is a property of this package, so every one of them is
testable on a desk with nothing plugged in.

Run from the host directory:  python tests/test_ollama.py
"""
import io
import json
import os
import sys
import tempfile

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.errors import DeviceStateError                # noqa: E402
from coaxial_ollama import plan as planmod                 # noqa: E402
from coaxial_ollama import runner as runmod                # noqa: E402
from coaxial_ollama import tools as toolmod                # noqa: E402
from coaxial_ollama.sandbox import Scope, Shell            # noqa: E402

BSLASH = chr(92)


# ---- the fake bench --------------------------------------------------------

class FakeLink:
    def __init__(self):
        self.stats_reads = 0

    def echo(self, data):
        return data

    def stats(self):
        self.stats_reads += 1
        return {'unit_id': 1, 'bus_message': 42, 'char_overrun': 0}


class FakeSystem:
    def self_test(self):
        return [{'name': 'PLL lock', 'status': 'pass', 'value': 0},
                {'name': 'ADC calibrated', 'status': 'pass', 'value': 0},
                {'name': 'flash checksum', 'status': 'info', 'value': 0x1234}]

    def clock(self):
        return {'sysclk_hz': 475000000, 'hclk_hz': 237500000, 'source': 'PLL1'}


class FakeAfe:
    def __init__(self):
        self.on = False

    def state(self):
        return {'on': self.on, 'pe15': not self.on}

    def enable(self):
        self.on = True
        return True

    def disable(self):
        self.on = False
        return False

    def toggle(self):
        self.on = not self.on
        return self.on

    def require(self):
        if not self.on:
            raise DeviceStateError('the analog front end is off')


CHANNELS = [
    {'index': 0, 'adc': 1, 'pin': 'PA0', 'differential': True, 'signal': 'Phase U'},
    {'index': 1, 'adc': 1, 'pin': 'PA1', 'differential': True, 'signal': 'Phase V'},
    {'index': 2, 'adc': 3, 'pin': 'PC0', 'differential': False, 'signal': 'NTC'},
    {'index': 3, 'adc': 3, 'pin': 'PC1', 'differential': False, 'signal': 'DC bus'},
]


class FakeAnalog:
    def channels(self, refresh=False):
        return CHANNELS

    def burst(self, mask, samples, rate=None):
        chosen = {}
        for channel in CHANNELS:
            if mask >> channel['index'] & 1:
                chosen[channel['index']] = {'mean_raw': 32768.0 + channel['index'],
                                            'min_raw': 32700, 'max_raw': 32800}
        return {'samples': samples, 'rate_hz': rate or 0.0, 'channels': chosen}


class FakeBoard:
    def __init__(self):
        self.link = FakeLink()
        self.system = FakeSystem()
        self.afe = FakeAfe()
        self.analog = FakeAnalog()

    def close_binary(self):
        pass


class FakeSession:
    def __init__(self):
        self.board = FakeBoard()

    def info(self, refresh=False):
        version = {'device': 'coaxial_63100', 'mcu': 'STM32H753',
                   'firmware': '2.0.0', 'proto_major': 2, 'proto_minor': 1,
                   'build': 'test', 'commands': 21}
        return version, self.board.system.clock(), CHANNELS

    def reset(self):
        pass

    def close(self):
        pass


class ScriptedModel:
    """An Ollama stand-in that replays a list of assistant messages."""

    def __init__(self, turns, model='scripted'):
        self.turns = list(turns)
        self.model = model
        self.prompts = []          # every messages list it was handed

    def chat(self, messages, tools=None):
        self.prompts.append([dict(m) for m in messages])
        self.tools = tools
        if not self.turns:
            return {'role': 'assistant', 'content': 'I have nothing more.'}
        return dict(self.turns.pop(0))

    def usage(self):
        return {'calls': len(self.prompts), 'prompt_tokens': 0,
                'eval_tokens': 0}


def call(name, **args):
    """One tool call in the shape Ollama emits."""
    return {'role': 'assistant', 'content': '',
            'tool_calls': [{'function': {'name': name, 'arguments': args}}]}


class Report:
    def __init__(self):
        self.passed = 0
        self.failed = 0

    def check(self, name, condition, detail=''):
        if condition:
            self.passed += 1
            print('  PASS  %-44s %s' % (name, detail))
        else:
            self.failed += 1
            print('  FAIL  %-44s %s' % (name, detail))


def build(tasks, turns, allow_writes=False, confirm=None, allow=('python',),
          transcript=None, allow_code=True):
    plan = planmod.Plan({
        'product': 'coaxial_63100 BLDC inverter', 'revision': 'test',
        'plan_version': 'test', 'measurement_system_study': 'none, a unit test',
        'tasks': tasks})
    session = FakeSession()
    toolbox = toolmod.Toolbox(session, shell=Shell(allow, timeout=60),
                              scope=Scope(), allow_writes=allow_writes,
                              allow_code=allow_code, confirm=confirm)
    model = ScriptedModel(turns)
    runner = runmod.Runner(plan, model, toolbox,
                           transcript=runmod.Transcript(transcript), echo=False)
    return runner, model, session


# ---- the plan --------------------------------------------------------------

def test_plan(report):
    try:
        planmod.Plan({'product': 'x', 'revision': 'y', 'plan_version': 'z',
                      'tasks': [{'id': 'A', 'ask': 'do it'}]})
        report.check('plan without a study is refused', False, 'it loaded')
    except planmod.PlanError as exc:
        report.check('plan without a study is refused', 'study' in str(exc),
                     str(exc)[:40])

    try:
        planmod.Plan({'product': 'x', 'revision': 'y', 'plan_version': 'z',
                      'measurement_system_study': 'none', 'tasks': []})
        report.check('plan with no tasks is refused', False, 'it loaded')
    except planmod.PlanError:
        report.check('plan with no tasks is refused', True)

    task = planmod.Task('T1', 'n', 'measure it',
                        planmod.Limit(low=1.0, high=2.0, unit='V'))
    brief = task.brief()
    report.check('brief states the unit', 'number in V' in brief)
    report.check('brief hides the limit',
                 '1.0' not in brief and '2.0' not in brief, repr(brief[-30:]))
    report.check('a task with no limit records only',
                 planmod.Task('T2', 'n', 'a').record_only)

    shipped = planmod.Plan.load('coaxial_ollama/plans/bringup.yaml')
    analog = [t for t in shipped.tasks if t.id.startswith('A')]
    report.check('shipped plan judges no analog value',
                 all(t.record_only for t in analog),
                 '%d analog steps, all record_only' % len(analog))


# ---- the verdict -----------------------------------------------------------

def test_verdicts(report):
    limited = [{'id': 'T1', 'name': 'inside', 'ask': 'a',
                'limit': {'low': 0.0, 'high': 10.0, 'unit': 'V'}}]

    runner, _, _ = build(limited, [call('report', value=5.0, unit='V',
                                        note='measured')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('inside the limit passes', record.verdict == 'pass',
                 record.line().strip())

    runner, _, _ = build(limited, [call('report', value=11.0, unit='V',
                                        note='measured')])
    report.check('outside the limit fails',
                 runner.run_task(runner.plan.tasks[0]).verdict == 'fail')

    runner, _, _ = build(limited, [call('report', unit='V', note='no reading')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('a limited step with no number fails',
                 record.verdict == 'fail' and record.warnings,
                 record.warnings[0] if record.warnings else '')

    runner, _, _ = build([{'id': 'T1', 'name': 'record', 'ask': 'a',
                           'unit': 'degC', 'record_only': True}],
                         [call('report', value=1e6, unit='degC', note='odd')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('a record_only step is never failed',
                 record.verdict == 'record' and record.value == 1e6)

    runner, _, _ = build(limited, [call('report', value=5.0, unit='mV',
                                        note='wrong unit')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('a unit mismatch is warned, not silently judged',
                 record.verdict == 'pass' and any('expects' in w
                                                  for w in record.warnings),
                 record.warnings[0] if record.warnings else 'no warning')


def test_model_never_sees_limits(report):
    tasks = [{'id': 'T1', 'name': 'dc link error', 'ask': 'measure the link',
              'limit': {'low': -0.25, 'high': 0.25, 'unit': 'V error'}}]
    runner, model, _ = build(tasks, [call('report', value=0.1, unit='V error',
                                          note='ok')])
    runner.run_task(runner.plan.tasks[0])
    blob = json.dumps(model.prompts)
    report.check('the limit is absent from every prompt',
                 '0.25' not in blob and '-0.25' not in blob)
    report.check('the model is told it is not the judge',
                 'you are not the judge' in blob.lower()
                 and 'never say a value is good' in blob.lower())


# ---- a model behaving badly ------------------------------------------------

def test_misbehaviour(report):
    task = [{'id': 'T1', 'name': 'never reports', 'ask': 'a', 'max_turns': 3,
             'limit': {'high': 1.0, 'unit': 'V'}}]

    prose = {'role': 'assistant', 'content': 'The board looks healthy to me.'}
    runner, _, _ = build(task, [prose, prose, prose, prose])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('prose without a report ends unfinished',
                 record.verdict == 'unfinished', record.warnings[0])

    loop = call('link', op='stats')
    runner, _, session = build(task, [loop] * 10)
    record = runner.run_task(runner.plan.tasks[0])
    report.check('a looping model is stopped by max_turns',
                 record.verdict == 'unfinished' and record.turns == 3,
                 '%d turns, %d calls' % (record.turns, len(record.calls)))

    runner, _, _ = build(task, [call('no_such_tool', x=1),
                                call('report', value=0.5, unit='V', note='n')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('an unknown tool is an answer, not a crash',
                 record.verdict == 'pass' and 'no_such_tool' in record.calls)

    runner, _, _ = build(task, [{'role': 'assistant', 'content': '',
                                 'tool_calls': [{'function': {
                                     'name': 'report',
                                     'arguments': '{"value": 0.4, "unit": "V"}'}}]}])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('arguments as a JSON string are parsed',
                 record.verdict == 'pass' and record.value == 0.4)

    runner, model, _ = build(task, [
        {'role': 'assistant', 'content': 'hm',
         'thinking': 'a long private monologue',
         'tool_calls': [{'function': {'name': 'link',
                                      'arguments': {'op': 'stats'}}}]},
        call('report', value=0.5, unit='V', note='n')])
    record = runner.run_task(runner.plan.tasks[0])
    logged = json.dumps(runner.transcript.events)
    fed_back = json.dumps(model.prompts[1])
    report.check('thinking is logged',
                 'monologue' in logged and record.verdict == 'pass')
    report.check('thinking is not fed back into the next turn',
                 'monologue' not in fed_back and 'thinking' not in fed_back)
    report.check('a tool result is labelled with its tool name',
                 'link: ' in fed_back, fed_back[-90:])


# ---- the board bridge ------------------------------------------------------

def test_board_tools(report):
    tasks = [{'id': 'T1', 'name': 'measure the ntc', 'ask': 'a',
              'unit': 'degC', 'record_only': True, 'max_turns': 8}]
    runner, _, session = build(tasks, [
        call('analog_read', ch=['NTC']),                 # refused: front end off
        call('afe_power', action='on'),
        call('analog_read', ch=['NTC'], samples=128),
        call('self_test'),
        call('report', value=24.9, unit='degC', note='via analog_read'),
    ])
    record = runner.run_task(runner.plan.tasks[0])
    results = [e for e in runner.transcript.events if e['kind'] == 'tool']

    report.check('the MCP board tools are reachable unchanged',
                 record.verdict == 'record' and len(results) == 4,
                 ','.join(record.calls))
    # Labelled rather than refused. A refusal did not stop a fabricated
    # reading, it caused one: with no numbers to report, a model wrote
    # "Mid-scale ... 25.00 C" straight out of the warning text.
    report.check('an analog read with the front end off is labelled, not refused',
                 results[0]['result'].startswith('AFE OFF')
                 and 'afe_power on' in results[0]['result'],
                 results[0]['result'][:52])
    report.check('and it still carries the codes it read',
                 'smp' in results[0]['result'],
                 results[0]['result'].splitlines()[-1][:52])
    report.check('the front end switch is not write-gated',
                 'on=1' in results[1]['result'], results[1]['result'])
    report.check('a reading comes back in the compact renderer',
                 'NTC' in results[2]['result'] and 'smp' in results[2]['result'],
                 results[2]['result'].splitlines()[0])
    report.check('self test reaches the renderer',
                 'PLL lock' in results[3]['result'],
                 results[3]['result'].splitlines()[0])
    report.check('the tool surface is the MCP set plus three',
                 len(toolmod.TOOLS) == 12, '%d tools' % len(toolmod.TOOLS))

    schemas = toolmod.schemas()
    shapes = all(s['type'] == 'function' and s['function']['parameters']['type']
                 == 'object' for s in schemas)
    report.check('every schema is an ollama function schema', shapes)


# ---- code and commands -----------------------------------------------------

def test_scope(report):
    scope = Scope(board=FakeBoard())

    report.check('the last expression is the result',
                 scope.run('2 + 3') == '5')
    scope.run('readings = [1, 2, 3]')
    report.check('the namespace persists between calls',
                 scope.run('sum(readings)') == '6')
    report.check('the board is in scope',
                 'True' in scope.run('board.afe.enable()'))
    report.check('print output is captured',
                 scope.run('print("hello")') == 'hello')
    text = scope.run('round(board.analog.channels(), 2)')
    report.check('a traceback comes back as text, not an exception',
                 'TypeError' in text, text.splitlines()[-1][:60])
    report.check('the traceback blames the model code, not the runner',
                 'sandbox.py' not in text and '<bench>' in text)
    report.check('a syntax error is reported by line',
                 scope.run('def (:').startswith('SyntaxError'))
    report.check('model code cannot exit the runner',
                 'SystemExit' in scope.run('import sys; sys.exit(3)'))
    report.check('a statement with no value says so',
                 scope.run('x = 1') == '(no output)')
    report.check('long output is clipped',
                 'characters cut' in scope.run('print("x" * 20000)'))


def test_scope_repairs(report):
    """Code that arrived mangled, and imports that were never going to be here."""
    scope = Scope(board=FakeBoard())

    # Seen from the prompt: a whole program on one line with a literal
    # backslash-n where the newlines belonged. Python reads that as a line
    # continuation and refuses; the model had written valid code.
    escaped = 'x = 1' + BSLASH + 'ny = 2' + BSLASH + 'nx + y'
    report.check('escaped newlines are repaired, once compilation has failed',
                 scope.run(escaped).strip() == '3', repr(scope.run(escaped)))

    # And never before: a working one-liner with an escape inside a string
    # literal must come out as it went in.
    printed = scope.run('print("a' + BSLASH + 'nb")')
    report.check('an escape inside a string literal is left alone',
                 printed.splitlines()[:2] == ['a', 'b'], repr(printed))
    report.check('a real syntax error is still a syntax error',
                 'SyntaxError' in scope.run('x = ('))

    # pandas is absent by decision, so the failure has to name the alternative
    # rather than only the refusal.
    missing = scope.run('import pandas as pd')
    report.check('a missing package says what is here instead',
                 'no pandas' in missing and 'statistics' in missing,
                 missing.splitlines()[-1][:52])

    # The tool names and the method names are different words, and a model
    # that has called analog_read all session reaches for it here too.
    confused = scope.run('board.analog_read(ch=["ntc"])')
    report.check('a tool name used as a method is corrected',
                 'board.analog' in confused and 'read_all' in confused,
                 confused.splitlines()[-1][:52])
    report.check('and the real method still works',
                 scope.run('board.afe.enable()').strip() == 'True')


def test_prompt(report):
    """The spinner keeps turning while you type, and never over what you typed."""
    import time as clock
    from coaxial_ollama import spinner as spin

    class Tty(io.StringIO):
        encoding = 'cp1252'

        def isatty(self):
            return True

    screen = Tty()
    stop = spin.spinning_prompt('Coaxial_63100', screen, tick=0.01)
    clock.sleep(0.08)
    stop()
    drawn = screen.getvalue()
    header_end = drawn.index('> ') + 2

    report.check('the prompt is written whole, once, glyph flush against it',
                 drawn.startswith('Coaxial_63100' + spin.GREEN)
                 and drawn.count('Coaxial_63100') == 1, repr(drawn[:24]))
    report.check('no space pads the spinner on either side',
                 ' | ' not in drawn and ' > ' not in drawn, repr(drawn[:24]))
    report.check('it keeps painting after the prompt is up',
                 drawn.count(spin.SAVE) >= 3, '%d frames' % drawn.count(spin.SAVE))
    report.check('every frame saves the cursor and puts it back',
                 drawn.count(spin.SAVE) == drawn.count(spin.RESTORE))
    report.check('a working link paints the glyph green',
                 drawn.count(spin.GREEN) >= 3 and spin.RED not in drawn)
    report.check('nothing is redrawn except the spinner column',
                 chr(13) not in drawn and 'Coaxial' not in drawn[header_end:])
    report.check('stopping is final - no frame lands in the next answer',
                 drawn == screen.getvalue())

    down = Tty()
    stop = spin.spinning_prompt('Coaxial_63100', down, tick=0.01, ok=False)
    clock.sleep(0.08)
    stop()
    unwell = down.getvalue()
    report.check('a dead link paints the glyph red, not green',
                 unwell.count(spin.RED) >= 3 and spin.GREEN not in unwell)
    forward_glyphs = [spin.GLYPHS[f % len(spin.GLYPHS)] for f in range(1, 4)]
    backward_glyphs = [spin.GLYPHS[-f % len(spin.GLYPHS)] for f in range(1, 4)]
    report.check('and turns the other way through the same four glyphs',
                 forward_glyphs != backward_glyphs)

    report.check('the bar is an en dash, wider than a hyphen',
                 spin.GLYPHS[2] == chr(0x2013)
                 and spin.GLYPHS[2].encode('cp1252') == bytes([0x96]))
    report.check('a console that cannot hold it gets ASCII, not a question mark',
                 spin._frames(Tty()) == spin.GLYPHS
                 and spin.FALLBACK == ('|', '/', '-', chr(92)))

    class Ascii(Tty):
        encoding = 'ascii'

    report.check('and that fallback is chosen by asking the stream',
                 spin._frames(Ascii()) == spin.FALLBACK)

    # A pipe has no cursor to save: one static prompt, no escapes, no thread.
    piped = io.StringIO()
    noop = spin.spinning_prompt('Coaxial_63100', piped)
    noop()
    report.check('a redirected prompt is static and escape-free',
                 piped.getvalue() ==
                 'Coaxial_63100%s|%s> ' % (spin.GREEN, spin.RESET),
                 repr(piped.getvalue()))


def test_shell(report):
    shell = Shell(['python'], timeout=60)

    out = shell.run('python -c "print(6*7)"')
    report.check('an allowed program runs', 'exit=0' in out and '42' in out,
                 out.replace('\n', ' ')[:40])

    for command, why in (('curl http://example.com', 'not allowed'),
                         ('python -c "print(1)" | findstr 1', 'a pipe'),
                         ('python -c "print(1)" > out.txt', 'a redirect'),
                         ('python x.py && rm -rf .', 'a chain')):
        try:
            shell.check(command)
            report.check('refused: %s' % why, False, command)
        except ValueError as exc:
            report.check('refused: %s' % why, True, str(exc)[:46])

    out = shell.run('python -c "raise SystemExit(4)"')
    report.check('a non-zero exit is reported, not raised', 'exit=4' in out, out)
    out = shell.run('python -c "a = 2; print(a * 3)"')
    report.check('punctuation inside an argument is not shell punctuation',
                 '6' in out, out.replace(chr(10), ' ')[:30])
    report.check('a missing program is an answer',
                 Shell(['nope']).run('nope --version').startswith('ERR'))


def test_policy(report):
    task = [{'id': 'T1', 'name': 'poke a pin', 'ask': 'a', 'max_turns': 4,
             'record_only': True}]

    runner, _, _ = build(task, [call('gpio_pin', op='write', pin='B2',
                                     level=True),
                                call('report', note='refused')])
    runner.run_task(runner.plan.tasks[0])
    result = [e for e in runner.transcript.events if e['kind'] == 'tool'][0]
    report.check('a pin write is refused without --allow-writes',
                 'allow-writes' in result['result'], result['result'][:56])

    runner, _, _ = build(task, [call('run_python', code='board.afe.state()'),
                                call('report', note='ran it')])
    runner.run_task(runner.plan.tasks[0])
    result = [e for e in runner.transcript.events if e['kind'] == 'tool'][0]
    report.check('code runs by default - it is the point of the runner',
                 'pe15' in result['result'], result['result'][:40])

    runner, _, _ = build(task, [call('run_python', code='1+1'),
                                call('report', note='refused')],
                         allow_code=False)
    runner.run_task(runner.plan.tasks[0])
    result = [e for e in runner.transcript.events if e['kind'] == 'tool'][0]
    report.check('--read-only takes code away',
                 'disabled' in result['result'], result['result'][:46])

    asked = []

    def decline(name, args):
        asked.append(name)
        return False

    runner, _, _ = build(task, [call('run_command', cmd='python --version'),
                                call('report', note='declined')],
                         confirm=decline)
    runner.run_task(runner.plan.tasks[0])
    result = [e for e in runner.transcript.events if e['kind'] == 'tool'][0]
    report.check('--confirm can decline a command',
                 asked == ['run_command'] and 'declined' in result['result'],
                 result['result'][:50])

    runner, _, _ = build(task, [call('link', op='stats'),
                                call('report', note='read')],
                         confirm=lambda n, a: False)
    runner.run_task(runner.plan.tasks[0])
    result = [e for e in runner.transcript.events if e['kind'] == 'tool'][0]
    report.check('a read is never put in front of the operator',
                 'bus_message=42' in result['result'], result['result'][:40])

    runner, _, _ = build([{'id': 'T1', 'name': 'needs writes', 'ask': 'a',
                           'needs_writes': True, 'record_only': True}], [])
    summary = runner.run()
    report.check('a step needing writes is skipped loudly',
                 summary['counts'] == {'skipped': 1}
                 and summary['records'][0]['warnings'], summary['counts'])


# ---- the record of it all --------------------------------------------------

def test_transcript(report):
    handle, path = tempfile.mkstemp(suffix='.jsonl')
    os.close(handle)
    try:
        tasks = [{'id': 'T1', 'name': 'first', 'ask': 'a', 'unit': 'V',
                  'limit': {'high': 1.0, 'unit': 'V'}},
                 {'id': 'T2', 'name': 'second', 'ask': 'b', 'record_only': True}]
        runner, _, _ = build(tasks, [call('link', op='stats'),
                                     call('report', value=0.5, unit='V',
                                          note='one'),
                                     call('report', value=2, note='two')],
                             transcript=path)
        summary = runner.run()

        with open(path, encoding='utf-8') as fh:
            events = [json.loads(line) for line in fh]
        kinds = [e['kind'] for e in events]

        report.check('every event is one flushed JSONL line',
                     len(events) == len(runner.transcript.events),
                     '%d events' % len(events))
        report.check('the run is bracketed and every step is bracketed',
                     kinds[0] == 'run_begin' and kinds[-1] == 'run_end'
                     and kinds.count('step_begin') == 2
                     and kinds.count('step_end') == 2, ','.join(kinds))
        report.check('the plan header is in the transcript',
                     events[0]['plan']['measurement_system_study']
                     == 'none, a unit test')
        report.check('a tool call records its arguments and its result',
                     any(e['kind'] == 'tool' and e['args'] == {'op': 'stats'}
                         and 'unit_id' in e['result'] for e in events))
        report.check('both steps are counted',
                     summary['counts'] == {'pass': 1, 'record': 1},
                     summary['counts'])

        text = runmod.report_text(summary)
        report.check('the report shows the limit next to the verdict',
                     '<= 1 V' in text and 'PASS' in text)
        report.check('the report names the study',
                     'a unit test' in text)
        runner.transcript.close()
    finally:
        os.unlink(path)


def test_cli(report):
    from coaxial_ollama.__main__ import main, parse

    args = parse(['--plan', 'p.yaml', '--allow-writes'])
    report.check('the CLI defaults to COM4 and one program on the allowlist',
                 args.port == 'COM4' and args.allow == 'python'
                 and args.allow_writes)
    report.check('--list-tools needs neither board nor ollama',
                 main(['--list-tools']) == 0)

    ad_hoc = planmod.Plan.single('what does the NTC read?', unit='degC')
    report.check('an ad-hoc question can only record',
                 ad_hoc.tasks[0].record_only and 'NONE' in ad_hoc.study)


# ---- the debug CLI: the same board, at a fraction of the tokens ------------

def test_debug(report):
    from coaxial_ollama import debug

    box = toolmod.Toolbox(FakeSession(), shell=Shell(['python']), scope=Scope())

    def chat(turns, **kw):
        kw.setdefault('out', io.StringIO())
        return debug.Chat(ScriptedModel(turns), box, **kw)

    lean = chat([])
    report.check('the debug prompt is a fraction of the runner prompt',
                 debug.approx_tokens(debug.SYSTEM)
                 < debug.approx_tokens(runmod.SYSTEM) / 3,
                 '%d tok against %d' % (debug.approx_tokens(debug.SYSTEM),
                                        debug.approx_tokens(runmod.SYSTEM)))

    costs = {name: chat([], tools=name).tool_cost() for name in debug.SETS}
    report.check('a tool subset costs less per turn than the whole set',
                 costs['read'] < costs['all'] / 1.5 and costs['none'] < 5,
                 ' '.join('%s=%d' % kv for kv in sorted(costs.items())))
    report.check('report is not offered to a debug session',
                 'report' not in chat([], tools='all').tool_names)

    try:
        chat([], tools='no_such_tool')
        report.check('an unknown tool name is refused', False)
    except ValueError as exc:
        report.check('an unknown tool name is refused', 'no such tool' in str(exc))

    # ---- trimming ----
    session = chat([], keep=2)
    session.history = [
        {'role': 'user', 'content': 'old question'},
        {'role': 'tool', 'tool_name': 'analog_read',
         'content': 'analog_read: 128 smp @2000Hz\n' + 'x' * 500},
        {'role': 'assistant', 'content': 'old answer'},
        {'role': 'user', 'content': 'new question'},
    ]
    sent = session.trim()
    blob = json.dumps(sent)
    report.check('the system prompt leads every turn',
                 sent[0]['role'] == 'system'
                 and sent[0]['content'].startswith(debug.SYSTEM),
                 sent[0]['content'][-60:].replace('\n', ' '))
    report.check('an old tool result is stubbed to its first line',
                 '128 smp' in sent[2]['content'] and 'xxxx' not in blob,
                 sent[2]['content'][:44])
    report.check('the recent turns are sent whole',
                 sent[-1]['content'] == 'new question')
    report.check('trimming is what the cost estimate measures',
                 session.context_cost() < debug.approx_tokens(
                     json.dumps(session.history)) + session.tool_cost()
                 + debug.approx_tokens(debug.SYSTEM) + 40)

    # ---- a turn ----
    session = chat([call('afe_power', action='on'),
                    {'role': 'assistant', 'content': 'The front end is on.',
                     'thinking': 'long deliberation'}])
    answer = session.ask('is the front end on?')
    report.check('a question runs its tools and then answers',
                 answer == 'The front end is on.' and session.turn_cost,
                 '%d turns' % len(session.turn_cost))
    report.check('thinking is dropped from the history',
                 'deliberation' not in json.dumps(session.history))

    # ---- the parts that cost nothing ----
    session = chat([])
    report.check('/py runs without asking the model',
                 '6' in session.command('/py 2 * 3')
                 and not session.client.prompts, 'no model call')
    report.check('/sh runs without asking the model',
                 'exit=0' in session.command('/sh python -c "print(1)"'))
    session.link_ok = False
    report.check('/reconnect reports success and flips the spinner back on',
                 session.command('/reconnect') == 'board: link is up'
                 and session.link_ok is True)

    broken = debug.Chat(ScriptedModel([]),
                        toolmod.Toolbox(debug.NoBoard(), scope=Scope()),
                        out=io.StringIO())
    report.check('/reconnect on a dead link says so and keeps the spinner red',
                 broken.command('/reconnect') ==
                 'board: this run was started with --no-board'
                 and broken.link_ok is False,
                 broken.command('/reconnect'))
    report.check('/tools reprices the turn',
                 'tok/turn' in session.command('/tools read')
                 and session.tool_names == debug.SETS['read'])
    report.check('/ctx explains where the tokens go',
                 'of it tools' in session.command('/ctx'))
    report.check('/clear is the cheapest command there is',
                 session.command('/clear') == 'context cleared'
                 and session.history == [])
    report.check('a plain line is not a command',
                 session.command('read the ntc') is None)
    report.check('an unknown command does not become a question',
                 'no such command' in session.command('/nope'))
    try:
        session.command('/q')
        report.check('/q leaves', False)
    except SystemExit:
        report.check('/q leaves', True)

    # ---- budget ----
    spent = chat([{'role': 'assistant', 'content': 'hi'}], budget=1)
    spent.client.usage = lambda: {'calls': 1, 'prompt_tokens': 900,
                                  'eval_tokens': 200}
    report.check('a spent budget refuses instead of asking',
                 'budget' in spent.ask('another question')
                 and not spent.client.prompts)
    report.check('the cost line names the budget',
                 'of 1' in spent.cost_line(), spent.cost_line())

    # ---- no board ----
    offline = toolmod.Toolbox(debug.NoBoard(), scope=Scope())
    text = offline.call('analog_read', {})
    report.check('--no-board answers instead of hanging on a serial port',
                 'no-board' in text, text[:60])

    attached = debug.attach(['coaxial_ollama/plans/bringup.yaml'], 300)
    report.check('an attached file is clipped, and says so',
                 'characters cut' in attached and '300 attached' in attached,
                 attached.splitlines()[0][:60])


# ---- the daemon is on this machine ----------------------------------------

def test_local_only(report):
    """No prompt leaves the bench PC unless somebody asked for that."""
    from coaxial_ollama.client import Ollama, OllamaError, is_cloud, is_local

    report.check('loopback in its usual spellings is local',
                 all(is_local(url) for url in
                     ('http://localhost:11434', 'http://127.0.0.1:11434',
                      'http://[::1]:11434')))
    report.check('another machine is not local',
                 not is_local('http://bench-gpu.lan:11434')
                 and not is_local('https://ollama.com'))
    report.check('a cloud tag is recognised as one',
                 is_cloud('minimax-m3:cloud') and not is_cloud('gemma4:12b'))

    for host in ('http://bench-gpu.lan:11434', 'https://ollama.com'):
        try:
            Ollama('gemma4:12b', host=host)
            report.check('a remote host is refused', False, host)
        except OllamaError as exc:
            report.check('a remote host is refused', 'not this machine' in str(exc))

    try:
        Ollama('minimax-m3:cloud')
        report.check('a cloud tag is refused', False)
    except OllamaError as exc:
        report.check('a cloud tag is refused', 'off this machine' in str(exc))

    remote = Ollama('minimax-m3:cloud', host='https://ollama.com', remote_ok=True)
    report.check('--allow-remote still means yes', remote.host == 'https://ollama.com')

    # require_model resolves a bare stem against `ollama list`. The cloud tag is
    # in that list like any other, so the loose match has to skip it.
    local = Ollama('minimax-m3')
    local.models = lambda: ['gemma4:12b', 'minimax-m3:cloud']
    try:
        local.require_model()
        report.check('stem matching cannot land on a cloud tag', False)
    except OllamaError as exc:
        report.check('stem matching cannot land on a cloud tag',
                     'not pulled' in str(exc))

    picked = Ollama('gemma4')
    picked.models = lambda: ['gemma4:12b', 'minimax-m3:cloud']
    report.check('a local stem still resolves to its tag',
                 picked.require_model() == 'gemma4:12b')

    allowed = Ollama('minimax-m3', remote_ok=True)
    allowed.models = lambda: ['gemma4:12b', 'minimax-m3:cloud']
    report.check('with --allow-remote the cloud tag is a candidate again',
                 allowed.require_model() == 'minimax-m3:cloud')


# ---- the model stays loaded, and so does its prompt cache -----------------

def test_keep_alive(report):
    """A pause between two questions must not cost a reload."""
    from coaxial_ollama.client import Ollama

    sent = []

    def capture(client, reply=None):
        def _post(path, payload):
            sent.append((path, payload))
            return reply or {'message': {'role': 'assistant', 'content': 'ok'}}
        client._post = _post

    client = Ollama('gemma4:12b')
    report.check('keep_alive defaults to something non-zero',
                 client.keep_alive == '30m')

    capture(client)
    client.chat([{'role': 'user', 'content': 'hello'}])
    report.check('every turn re-arms the unload timer',
                 sent[-1][1].get('keep_alive') == '30m')

    # An explicit 0 is how a shared machine gives the VRAM straight back, and
    # None is how a caller says 'do not mention it at all' - the daemon then
    # applies its own default. The two must not collapse into each other.
    zero = Ollama('gemma4:12b', keep_alive=0)
    capture(zero)
    zero.chat([{'role': 'user', 'content': 'hello'}])
    report.check('keep_alive=0 is sent, not dropped as falsy',
                 sent[-1][1].get('keep_alive') == 0)

    absent = Ollama('gemma4:12b', keep_alive=None)
    capture(absent)
    absent.chat([{'role': 'user', 'content': 'hello'}])
    report.check('keep_alive=None leaves the field out',
                 'keep_alive' not in sent[-1][1])

    # The card is not free to sit on. A prompt loop earns a long hold - the KV
    # cache is what makes turn nine quick - and a single question does not:
    # measured here, a one-shot left 9.69 GB resident for another 27 minutes at
    # 1 % utilisation, on a card whose desktop then had 3.8 GB to work in.
    from coaxial_ollama import debug as dbgmod
    report.check('a prompt loop keeps the model, a one-shot does not',
                 dbgmod.keep_alive_for(dbgmod.parse(['--repl'])) == dbgmod.KEEP_ALIVE_REPL
                 and dbgmod.keep_alive_for(dbgmod.parse(['q'])) == dbgmod.KEEP_ALIVE_ONCE)
    report.check('the one-shot hold is the shorter of the two',
                 dbgmod.KEEP_ALIVE_ONCE != dbgmod.KEEP_ALIVE_REPL,
                 '%s against %s' % (dbgmod.KEEP_ALIVE_ONCE, dbgmod.KEEP_ALIVE_REPL))
    report.check('an explicit --keep-alive wins in either mode',
                 dbgmod.keep_alive_for(dbgmod.parse(['--keep-alive', '0', 'q'])) == '0'
                 and dbgmod.keep_alive_for(
                     dbgmod.parse(['--keep-alive', '1h', '--repl'])) == '1h')

    shaped = Ollama('gemma4:12b', fmt='json')
    capture(shaped)
    shaped.chat([{'role': 'user', 'content': 'hello'}])
    report.check("format is sent only when a caller asked for it",
                 sent[-1][1].get('format') == 'json')
    plain = Ollama('gemma4:12b')
    capture(plain)
    plain.chat([{'role': 'user', 'content': 'hello'}])
    report.check('the runner does not turn json mode on',
                 'format' not in sent[-1][1])

    from coaxial_ollama import debug as dbgmod
    report.check('dbg --format reaches the client',
                 dbgmod.parse(['--format', 'json', 'q']).fmt == 'json'
                 and dbgmod.parse(['q']).fmt is None)

    client.preload()
    path, payload = sent[-1]
    report.check('preload loads without generating',
                 path == '/api/chat' and payload['messages'] == []
                 and payload['keep_alive'] == '30m')
    # Measured, not guessed: a preload without num_ctx asks for the model's own
    # default context - 128k on llama3.1 - and the daemon answers 500 trying to
    # allocate 7 GB for the KV cache. When it does fit, it is worse: the model
    # is resident at one context size and the first question arrives at
    # another, so it reloads and the preload has cost a wait rather than saved
    # one.
    report.check('preload loads at the size the questions will use',
                 payload.get('options') is client.options)
    report.check('preload is not counted as a turn', client.calls == 1)

    # Both front ends have to offer it, or the flag exists and nobody can
    # reach it from the bench.
    from coaxial_ollama import debug
    import coaxial_ollama.__main__ as runner
    # Not a constant any more: the parsed value is None and the mode decides,
    # so check what actually reaches the daemon rather than the flag's default.
    report.check('dbg holds the model for a question, briefly',
                 debug.keep_alive_for(debug.parse(['why?'])) == debug.KEEP_ALIVE_ONCE)
    report.check('dbg --keep-alive is settable',
                 debug.parse(['--keep-alive', '1h', 'why?']).keep_alive == '1h')
    report.check('the runner defaults to holding the model',
                 runner.parse(['--plan', 'p.yaml']).keep_alive == '30m')
    report.check('the runner --keep-alive is settable',
                 runner.parse(['--plan', 'p.yaml',
                               '--keep-alive', '0']).keep_alive == '0')


# ---- arguments as a weaker model spells them ------------------------------

def test_coerce(report):
    """A tool that is hard to call is a tool that gets guessed around."""
    from coaxial_mcp.tools import coerce

    got = coerce('analog_read', {'ch': 'ntc', 'samples': '100',
                                 'rate_hz': '1000', 'vref': '3.3'})
    report.check('a bare string channel is one channel', got['ch'] == ['ntc'])
    report.check('a numeric string becomes a number',
                 got == {'ch': ['ntc'], 'samples': 100, 'rate_hz': 1000.0,
                         'vref': 3.3})
    report.check('a list that arrived as text is a list',
                 coerce('analog_read', {'ch': "['NTC']"})['ch'] == ['NTC'])
    # The separator a model puts in a name is not information. board_info
    # prints "DC bus", the short form is "DCbus", and gemma4:12b sent "dc_bus"
    # and was told `unknown channel 'dc_bus'; names are ch3,ch6,dcbus,...` - a
    # refusal listing a name one underscore away from the one it used.
    from coaxial_mcp.tools import _key
    report.check('the separator in a channel name is not information',
                 len({_key(n) for n in ('dc_bus', 'DC bus', 'dc-bus', 'DCbus',
                                        'dcbus', ' DCBUS ')}) == 1)
    report.check('and the same holds for the phases',
                 _key('phase_u') == _key('Phase U') == _key('PHASEU') == 'phaseu')
    report.check('an index is left alone', _key('4') == '4')

    # And the whole path, not just the key: the resolver has to turn every
    # spelling into the same channel index against a real channel table.
    from coaxial_mcp.tools import _resolve
    session = FakeSession()
    wanted = [_resolve(session, [spelling])
              for spelling in ('dc_bus', 'DC bus', 'DCbus', 'dcbus')]
    report.check('every spelling of the DC link resolves to one channel',
                 len({tuple(w) for w in wanted}) == 1, str(wanted[0]))
    # A/B/C and U/V/W are two conventions for the same three phases, and both
    # appear in the same datasheets. Measured: a model asked for
    # ['ntc','dc_bus','phase_a','phase_b','phase_c'] and lost all five readings
    # to the two it spelled the other way.
    class ThreePhase:
        def info(self, refresh=False):
            names = ['Phase U', 'Phase V', 'Phase W', None, 'NTC', 'DC bus', None]
            return None, None, [{'index': i, 'signal': n}
                                for i, n in enumerate(names)]

    board = ThreePhase()
    report.check('A, B and C are U, V and W',
                 _resolve(board, ['phase_a', 'phase_b', 'phase_c'])
                 == _resolve(board, ['phaseu', 'phasev', 'phasew'])
                 == [0, 1, 2])
    report.check('and so are the bare letters, in either convention',
                 _resolve(board, ['A', 'B', 'C']) == _resolve(board, ['u', 'v', 'w'])
                 == [0, 1, 2])
    report.check('the call that failed now resolves whole',
                 _resolve(board, ['ntc', 'dc_bus', 'phase_a', 'phase_b', 'phase_c'])
                 == [4, 5, 0, 1, 2])

    # The alias must never invent a channel: a board without a Phase W has no
    # Phase C either, and saying otherwise would return somebody else's data.
    try:
        _resolve(session, ['phase_c'])       # the fixture has U and V only
        report.check('an alias cannot point at a channel that is absent', False)
    except ValueError as exc:
        report.check('an alias cannot point at a channel that is absent',
                     'unknown channel' in str(exc), str(exc)[:46])

    try:
        _resolve(board, ['dcbusvoltage'])
        report.check('a near miss is named in the error', False)
    except ValueError as exc:
        report.check('a near miss is named in the error',
                     "did you mean 'dcbus'" in str(exc), str(exc)[:56])

    report.check('a mixed list still resolves in order',
                 _resolve(session, ['phase_u', 'NTC', 'dc-bus'])
                 == _resolve(session, ['phaseu', 'ntc', 'dcbus']))
    try:
        _resolve(session, ['not_a_channel'])
        report.check('a name that is not a separator away is still refused', False)
    except ValueError as exc:
        report.check('a name that is not a separator away is still refused',
                     'unknown channel' in str(exc), str(exc)[:50])

    report.check('a comma separated string is several',
                 coerce('analog_read', {'ch': 'ntc,dcbus'})['ch']
                 == ['ntc', 'dcbus'])
    report.check('a boolean spelled as a word',
                 coerce('board_info', {'refresh': 'true'})['refresh'] is True
                 and coerce('board_info', {'refresh': 'no'})['refresh'] is False)
    report.check('null stays null - it means "use the default"',
                 coerce('analog_read', {'ntc_beta': None})['ntc_beta'] is None)
    report.check('what the schema does not name is passed through',
                 coerce('analog_read', {'nonsense': object}) is not None)

    # The point of all of it: what cannot be converted has to say so by name,
    # because the alternative measured here was a TypeError deep in the handler
    # and a model answering from memory.
    for args in ({'samples': 'many'}, {'vref': 'three'}, {'samples': True}):
        try:
            coerce('analog_read', args)
            report.check('a bad argument is refused by name', False, repr(args))
        except ValueError as exc:
            report.check('a bad argument is refused by name',
                         'analog_read' in str(exc) and 'should be' in str(exc),
                         str(exc)[:58])


# ---- the tag follows the machine ------------------------------------------

def test_capability(report):
    """Picked from cores, RAM and VRAM - not from whoever cloned the repo."""
    from coaxial_ollama import capability as cap

    def machine(cores, ram, vram, name='card', used=0.0, free=None, busy=None):
        gpus = ([{'name': name, 'vram_gb': vram, 'used_gb': used, 'via': 'test'}]
                if vram else [])
        return cap.Machine(cores=cores, threads=cores * 2, ram_gb=ram,
                           ram_free_gb=ram if free is None else free,
                           cpu_busy=busy, gpus=gpus, system='test', notes={})

    # A workstation card: the biggest tag that fits WHOLE, never a split one.
    # Measured on the bench: wholly resident is ~5x faster per token than half.
    big = cap.choose(machine(32, 64, 16))
    report.check('16 GB card takes the largest model that fits whole',
                 big.tag == 'qwen2.5:14b' and 'num_gpu' not in big.options,
                 big.tag)

    # A decent laptop: 8 GB card, 2 GB reserve, 6 GB to spend.
    laptop = cap.choose(machine(8, 32, 8))
    report.check('8 GB card drops to a model that fits it',
                 laptop.entry['gb'] <= 6.0 and 'num_gpu' not in laptop.options,
                 laptop.tag)

    # No GPU at all: everything on the CPU, and it says so rather than
    # recommending something that cannot load.
    headless = cap.choose(machine(64, 128, 0))
    report.check('no GPU means num_gpu 0', headless.options.get('num_gpu') == 0,
                 headless.tag)
    report.check('no GPU is explained, not silently slow',
                 'CPU' in headless.why or 'cpu' in headless.why)

    # Under-specified: 4 GB of RAM cannot hold anything in the catalogue.
    tiny = cap.choose(machine(2, 4, 0))
    report.check('a machine too small for any of them says so',
                 any('under-specified' in w for w in tiny.warnings))

    # Capability mode is allowed to spill onto the CPU, and must warn that it
    # costs - otherwise it is a slow default nobody chose.
    step = cap.choose(machine(32, 64, 16), prefer='capability')
    report.check('capability mode reaches for the bigger model',
                 step.tag == 'qwen2.5:32b' and step.options['num_gpu'] > 0,
                 '%s num_gpu=%s' % (step.tag, step.options.get('num_gpu')))
    report.check('and says what it costs',
                 any('slower' in w for w in step.warnings))

    # The reserve is the whole point of the budget: a card must never be
    # filled to the brim, because the desktop lives there too.
    report.check('a quarter of the card is held back, floor 2 GB',
                 cap.reserve_for(16) == 4.0 and cap.reserve_for(4) == 2.0
                 and cap.reserve_for(0) == 0.0)

    # And what the card already holds counts. Measured on this bench: a
    # two-screen desktop was using 2.6 GB before anything of ours ran, so a
    # flat quarter of a 16 GB card left it 1.4 GB to grow into - which is not
    # an error, it is a stutter, which is worse because nobody can read it.
    report.check('what the desktop already uses raises the reserve',
                 cap.reserve_for(16, 2.6) == 2.6 + cap.HEADROOM_GB,
                 '%.1f GB' % cap.reserve_for(16, 2.6))
    report.check('an empty card still gets the flat reserve',
                 cap.reserve_for(16, 0.1) == 4.0)
    report.check('the reserve never exceeds the card',
                 cap.reserve_for(8, 20) <= 8 + cap.HEADROOM_GB)

    import os
    os.environ[cap.RESERVE_ENV] = '8'
    try:
        report.check('a machine can say how much to hold back',
                     cap.reserve_for(16, 2.6) == 8.0)
        stingy = cap.choose(machine(32, 64, 16))
        report.check('and that changes the model, not just the number',
                     stingy.entry['gb'] <= 8.0, stingy.tag)
        os.environ[cap.RESERVE_ENV] = 'nonsense'
        report.check('an unreadable override falls back rather than crashing',
                     cap.reserve_for(16, 2.6) == 2.6 + cap.HEADROOM_GB)
    finally:
        del os.environ[cap.RESERVE_ENV]
    report.check('without the override the measurement decides again',
                 cap.reserve_for(16, 2.6) == 2.6 + cap.HEADROOM_GB)
    fits = [e for e in cap.CATALOGUE if e['gb'] <= 16 - cap.reserve_for(16)]
    report.check('nothing recommended for a 16 GB card fills it',
                 all(e['gb'] < 16 for e in fits))

    # Layer arithmetic: as many as the budget holds, never more than the model
    # has, never negative.
    for vram, ram in ((10, 32), (6, 16), (24, 64), (0, 64)):
        picked = cap.choose(machine(16, ram, vram), prefer='capability')
        layers = picked.options.get('num_gpu')
        if layers is not None:
            report.check('layer count stays inside the model (%d GB card)' % vram,
                         0 <= layers <= picked.entry['layers'],
                         '%s %s/%s' % (picked.tag, layers, picked.entry['layers']))

    # Free RAM, not the sticker. A 64 GB workstation with 8 GB left cannot hold
    # a 42 GB model, and the failure mode is swapping rather than an error.
    squeezed = cap.choose(machine(32, 64, 0, free=8))
    report.check('the choice is made on free RAM, not installed RAM',
                 squeezed.entry['ram_gb'] <= 8, squeezed.tag)
    report.check('a roomy machine still reaches the big models',
                 cap.choose(machine(32, 64, 0, free=64)).entry['gb']
                 >= squeezed.entry['gb'])

    # A busy machine gets a warning, not a different tag: load now says nothing
    # about load in ten minutes, and the tag is chosen for the session.
    busy = cap.choose(machine(32, 64, 0, free=64, busy=90))
    idle = cap.choose(machine(32, 64, 0, free=64, busy=1))
    report.check('a busy machine is warned, not quietly downgraded',
                 busy.tag == idle.tag
                 and any('busy' in w for w in busy.warnings)
                 and not any('busy' in w for w in idle.warnings))

    # The ratchet: our own resident model must not count as somebody's desktop,
    # or every run reserves more than the last and walks the choice downhill.
    real_smi, real_ours = cap._gpus_nvidia_smi, cap._ollama_vram_gb
    try:
        # A 16 GB card reading 10.8 used, of which 7.8 is the model we loaded
        # ourselves. The desktop is 3.0, and that is what the reserve is for.
        cap._gpus_nvidia_smi = lambda: [{'name': 'test', 'vram_gb': 16.0,
                                         'used_gb': 10.8, 'via': 'test'}]
        cap._ollama_vram_gb = lambda host='': 7.8
        probed = cap.probe()
        report.check("ollama's own VRAM is not counted as the desktop's",
                     abs(probed.vram_used_gb - 3.0) < 0.01,
                     '%.1f GB' % probed.vram_used_gb)
        report.check('and the probe says it did that',
                     'minus ollama' in probed.notes['gpu'])
        report.check('so the reserve does not ratchet upwards',
                     cap.reserve_for(16, probed.vram_used_gb)
                     < cap.reserve_for(16, 10.8),
                     '%.1f against %.1f' % (cap.reserve_for(16, probed.vram_used_gb),
                                            cap.reserve_for(16, 10.8)))

        cap._ollama_vram_gb = lambda host='': 0.0
        report.check('a card with nothing of ours on it is unchanged',
                     abs(cap.probe().vram_used_gb - 10.8) < 0.01)
    finally:
        cap._gpus_nvidia_smi, cap._ollama_vram_gb = real_smi, real_ours

    report.check('every candidate is tools-capable by construction',
                 all(entry.get('note') is not None for entry in cap.CATALOGUE))

    # And the probe itself has to survive whatever it finds, on any OS.
    found = cap.probe()
    report.check('probe returns a usable machine',
                 found.threads >= 1 and found.cores >= 1 and found.ram_gb >= 0)
    report.check('probe records how it measured each number',
                 set(found.notes) == set(['cpu', 'ram', 'gpu']), str(found.notes))
    report.check('report is text a human can read',
                 'machine:' in cap.report(found) and 'model:' in cap.report(found))


# ---- the documents are reachable from a prompt ----------------------------

def test_docs(report):
    """The one reader who could not open docs/ was the model at the bench."""
    from coaxial_mcp import docs as docmod
    from coaxial_mcp.tools import HANDLERS, TOOLS

    report.check('docs is a tool the model can call',
                 'docs' in HANDLERS
                 and any(spec['name'] == 'docs' for spec in TOOLS))

    index = docmod.docs()
    for name in ('README', 'CLAUDE', 'ARCHITECTURE', 'PROTOCOL', 'HARDWARE',
                 'FINDINGS', 'MODELS'):
        report.check('the index lists ' + name, name in index)
    report.check('the index is headings, not documents',
                 len(index) < 4000, '%d chars' % len(index))
    report.check('the index says how to go deeper',
                 'section=' in index and 'find=' in index)

    outline = docmod.docs(doc='MODELS')
    report.check('one document lists its own headings',
                 'Threads' in outline and 'Why a local model at all' in outline)

    body = docmod.docs(doc='MODELS', section='Threads')
    report.check('a section returns its text', 'bandwidth-bound' in body)
    report.check('a section stops at the next heading of its level',
                 'Keeping the model loaded' not in body)

    # A parent section keeps its children: asking for the chapter should not
    # silently return only its first paragraph.
    parent = docmod.docs(doc='MODELS', section='Which tag, and who decides')
    report.check('a parent section carries its subsections',
                 'Threads' in parent or 'clipped' in parent)

    report.check('heading matching is loose enough to be usable',
                 'bandwidth-bound' in docmod.docs(doc='MODELS', section='threads'))

    # The AFE number is the one a weak model invents. It has to be findable.
    hits = docmod.docs(find='25.00')
    report.check('search reaches the AFE-off number', 'FINDINGS' in hits
                 or 'HARDWARE' in hits or 'CLAUDE' in hits)
    report.check('search says where it found it, not just that it did',
                 len(hits.splitlines()) >= 2)
    # A hit without its chapter can say the opposite of what the document says:
    # the phase V entry that matches lives under "Refuted", and a model that
    # cannot see that reports a dead end as the explanation. Measured.
    refuted = docmod.docs(find='PCSEL accumulation explains')
    report.check('a search hit carries the chapter it sits under',
                 'Refuted' in refuted, refuted.splitlines()[0][:70])

    report.check('search is capped rather than dumping a document',
                 len(docmod.docs(find='the').splitlines()) <= docmod.FIND_HITS + 1)

    report.check('a long section is clipped and says so',
                 len(docmod.docs(doc='FINDINGS',
                                 section='Confirmed and fixed')) <= docmod.CLIP + 200)

    for bad, expect in ((dict(doc='NOPE'), 'no document'),
                        (dict(doc='MODELS', section='no such heading'), 'no section'),
                        (dict(find='   '), 'something to look for')):
        try:
            docmod.docs(**bad)
            report.check('a bad docs call is refused by name', False, str(bad))
        except ValueError as exc:
            report.check('a bad docs call is refused by name', expect in str(exc),
                         str(exc)[:54])

    report.check('every document named is actually on disk',
                 set(docmod.paths()) == set(docmod.NAMES),
                 ', '.join(sorted(set(docmod.NAMES) - set(docmod.paths()))) or 'all')

    # The bench prompt has to point at the tool, or nothing above matters.
    from coaxial_ollama import debug, runner
    # Not in SYSTEM any more: the language is worked out here and named in the
    # turn's system message, because a model asked to work it out itself
    # answered a European question in Chinese. See language.py.
    from coaxial_ollama import language
    talk = debug.Chat.__new__(debug.Chat)
    talk.keep = 6
    for question, expect in (('Vad är temperaturen på kortet?', 'Swedish'),
                             ('What is the board temperature?', 'English'),
                             ('Was macht der AFE-Schalter?', 'German')):
        talk.history = [{'role': 'user', 'content': question}]
        head = talk.trim()[0]['content']
        report.check('the turn names the language: ' + expect,
                     ('in %s' % expect) in head, head.splitlines()[-1][:56])

    talk.history = [{'role': 'user', 'content': 'status?'}]
    report.check('an undetectable question falls back to mirroring',
                 'language the question was asked in' in talk.trim()[0]['content'])
    report.check('and the board keeps its own words either way',
                 'stay exactly as the board prints them'
                 in language.instruction('Vad är temperaturen?'))

    # A console that cannot encode the answer must not lose it. cp1252 holds
    # Swedish and German; it does not hold a Polish l-stroke or an ohm sign,
    # and the default handler turns that into a UnicodeEncodeError after the
    # measurement has already been taken.
    import io as _io
    raw = _io.BytesIO()
    narrow = _io.TextIOWrapper(raw, encoding='cp1252', newline='')
    debug._printable(narrow)
    try:
        narrow.write('resistans 6487 ohm, Ω och ł')
        narrow.flush()
        report.check('an alphabet the console lacks costs a glyph, not the answer',
                     b'resistans 6487 ohm' in raw.getvalue())
    except UnicodeEncodeError as exc:
        report.check('an alphabet the console lacks costs a glyph, not the answer',
                     False, str(exc)[:50])
    report.check('and a stream that cannot be reconfigured is left alone',
                 debug._printable(object()) is not None)

    report.check('dbg tells the model the documents exist',
                 'docs' in debug.SYSTEM and 'FINDINGS' in debug.SYSTEM)
    report.check('the runner tells it too',
                 'docs' in runner.SYSTEM and 'FINDINGS' in runner.SYSTEM)
    report.check('docs is in the default tool set',
                 'docs' in debug.SETS['read'] and 'docs' in debug.SETS['code'])


def main():
    report = Report()
    for test in (test_plan, test_verdicts, test_model_never_sees_limits,
                 test_misbehaviour, test_board_tools, test_scope, test_shell,
                 test_scope_repairs, test_prompt, test_policy, test_transcript,
                 test_debug, test_cli,
                 test_local_only, test_keep_alive, test_coerce,
                 test_capability, test_docs):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())

