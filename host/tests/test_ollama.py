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
    report.check('an analog read with the front end off is refused with the way out',
                 results[0]['result'].startswith('ERR')
                 and 'afe_power' in results[0]['result'],
                 results[0]['result'][:52])
    report.check('the front end switch is not write-gated',
                 'on=1' in results[1]['result'], results[1]['result'])
    report.check('a reading comes back in the compact renderer',
                 'NTC' in results[2]['result'] and 'smp' in results[2]['result'],
                 results[2]['result'].splitlines()[0])
    report.check('self test reaches the renderer',
                 'PLL lock' in results[3]['result'],
                 results[3]['result'].splitlines()[0])
    report.check('the tool surface is the MCP set plus three',
                 len(toolmod.TOOLS) == 11, '%d tools' % len(toolmod.TOOLS))

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
                 sent[0]['role'] == 'system' and sent[0]['content'] == debug.SYSTEM)
    report.check('an old tool result is stubbed to its first line',
                 '128 smp' in sent[2]['content'] and 'xxxx' not in blob,
                 sent[2]['content'][:44])
    report.check('the recent turns are sent whole',
                 sent[-1]['content'] == 'new question')
    report.check('trimming is what the cost estimate measures',
                 session.context_cost() < debug.approx_tokens(
                     json.dumps(session.history)) + session.tool_cost() + 40)

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


def main():
    report = Report()
    for test in (test_plan, test_verdicts, test_model_never_sees_limits,
                 test_misbehaviour, test_board_tools, test_scope, test_shell,
                 test_policy, test_transcript, test_debug, test_cli,
                 test_local_only):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())

