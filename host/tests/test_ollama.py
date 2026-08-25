#!/usr/bin/env python3
"""Offline test of the Ollama runner: no board, no ollama, no network.

The board and the model are both simulated, on purpose. What is under test
here is not whether a language model can read a thermistor - that is what a
bench is for - but whether this runner keeps its promises when the model
behaves badly:

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
import threading

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.errors import ConnectError, DeviceStateError   # noqa: E402
from coaxial_ollama import plan as planmod                 # noqa: E402
from coaxial_ollama import replies                       # noqa: E402
from coaxial_ollama import runner as runmod                # noqa: E402
from coaxial_ollama import tools as toolmod                # noqa: E402
from coaxial_ollama.sandbox import Scope, Shell            # noqa: E402
from coaxial_mcp import detail                           # noqa: E402

BSLASH = chr(92)


# ---- the simulated bench ---------------------------------------------------

class SimulatedLink:
    def __init__(self, board):
        self.board = board
        self.stats_reads = 0

    def echo(self, data):
        return data

    def stats(self):
        self.stats_reads += 1
        if self.board.broken:
            raise ConnectError('cable pulled')
        if self.board.dead_handle:
            # Distinct from `broken`: a real cable pull can leave the OS
            # handle Session.board cached permanently invalid, since a USB
            # VCP re-enumerates on replug rather than reviving the same
            # handle - measured directly against real hardware. Unlike
            # `broken`, nothing but session.reset() clears this one; the
            # tests using it are the ones proving debug.py actually calls
            # reset() rather than just retrying on the same dead handle.
            raise ConnectError('Attempting to use a port that is not open')
        return {'unit_id': 1, 'bus_message': 42, 'char_overrun': 0}


class SimulatedSystem:
    def self_test(self):
        return [{'name': 'PLL lock', 'status': 'pass', 'value': 0},
                {'name': 'ADC calibrated', 'status': 'pass', 'value': 0},
                {'name': 'flash checksum', 'status': 'info', 'value': 0x1234}]

    def clock(self):
        return {'sysclk_hz': 475000000, 'hclk_hz': 237500000, 'source': 'PLL1'}


class SimulatedAfe:
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


class SimulatedAnalog:
    def __init__(self, board):
        self.board = board

    def channels(self, refresh=False):
        return CHANNELS

    def burst(self, mask, samples, rate=None):
        if self.board.broken:
            raise ConnectError('cable pulled mid-burst')
        if self.board.dead_handle:
            raise ConnectError('Attempting to use a port that is not open')
        chosen = {}
        for channel in CHANNELS:
            if mask >> channel['index'] & 1:
                chosen[channel['index']] = {'mean_raw': 32768.0 + channel['index'],
                                            'min_raw': 32700, 'max_raw': 32800}
        return {'samples': samples, 'rate_hz': rate or 0.0, 'channels': chosen}


class SimulatedBoard:
    def __init__(self):
        # A cable pull is a transport-level failure: it takes down every
        # subsystem's calls at once, not just the one a test happens to be
        # driving - so this lives here, not on SimulatedAnalog alone, and
        # SimulatedLink fails the same way analog does. One flag, shared, the
        # way a real dead Transport actually behaves.
        self.broken = False
        self.dead_handle = False
        self.link = SimulatedLink(self)
        self.system = SimulatedSystem()
        self.afe = SimulatedAfe()
        self.analog = SimulatedAnalog(self)

    def close_binary(self):
        pass


class SimulatedSession:
    def __init__(self):
        self.board = SimulatedBoard()
        self.resets = 0

    def info(self, refresh=False):
        version = {'device': 'coaxial_63100', 'mcu': 'STM32H753',
                   'firmware': '2.0.0', 'proto_major': 2, 'proto_minor': 1,
                   'build': 'test', 'commands': 21}
        return version, self.board.system.clock(), CHANNELS

    def reset(self):
        self.resets += 1
        self.board.dead_handle = False

    def close(self):
        pass


class ScriptedModel:
    """An Ollama stand-in that replays a list of assistant messages."""

    def __init__(self, turns, model='scripted', num_ctx=8192):
        self.turns = list(turns)
        self.model = model
        self.prompts = []          # every messages list it was handed
        # A real window, so every test that drives a Chat or a Runner through
        # this stand-in also proves the prompt those loops build actually fits
        # one - see context.py. A client with no options at all is a separate
        # case and is tested directly.
        self.options = {'num_ctx': num_ctx, 'num_predict': 300}
        self.notes = []

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
    session = SimulatedSession()
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

    # Every scenario below touches the board (link stats) before reporting -
    # a report with nothing behind it is refused, see test_misbehaviour.
    touch = call('link', op='stats')

    runner, _, _ = build(limited, [touch, call('report', value=5.0, unit='V',
                                               note='measured')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('inside the limit passes', record.verdict == 'pass',
                 record.line().strip())

    runner, _, _ = build(limited, [touch, call('report', value=11.0, unit='V',
                                               note='measured')])
    report.check('outside the limit fails',
                 runner.run_task(runner.plan.tasks[0]).verdict == 'fail')

    runner, _, _ = build(limited, [touch,
                                   call('report', unit='V', note='no reading')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('a limited step with no number fails',
                 record.verdict == 'fail' and record.warnings,
                 record.warnings[0] if record.warnings else '')

    runner, _, _ = build([{'id': 'T1', 'name': 'record', 'ask': 'a',
                           'unit': 'degC', 'record_only': True}],
                         [touch,
                          call('report', value=1e6, unit='degC', note='odd')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('a record_only step is never failed',
                 record.verdict == 'record' and record.value == 1e6)

    runner, _, _ = build(limited, [touch, call('report', value=5.0, unit='mV',
                                               note='wrong unit')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('a unit mismatch is warned, not silently judged',
                 record.verdict == 'pass' and any('expects' in w
                                                  for w in record.warnings),
                 record.warnings[0] if record.warnings else 'no warning')


def test_model_never_sees_limits(report):
    tasks = [{'id': 'T1', 'name': 'dc link error', 'ask': 'measure the link',
              'limit': {'low': -0.25, 'high': 0.25, 'unit': 'V error'}}]
    runner, model, _ = build(tasks, [call('link', op='stats'),
                                     call('report', value=0.1, unit='V error',
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
                                call('link', op='stats'),
                                call('report', value=0.5, unit='V', note='n')])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('an unknown tool is an answer, not a crash',
                 record.verdict == 'pass' and 'no_such_tool' in record.calls)

    runner, _, _ = build(task, [
        call('link', op='stats'),
        {'role': 'assistant', 'content': '',
         'tool_calls': [{'function': {
             'name': 'report',
             'arguments': '{"value": 0.4, "unit": "V"}'}}]}])
    record = runner.run_task(runner.plan.tasks[0])
    report.check('arguments as a JSON string are parsed',
                 record.verdict == 'pass' and record.value == 0.4)

    # A report with no board tool called this step is refused, not accepted -
    # the same class of fabrication debug.py hardened against, here gated
    # before it becomes a signed verdict. Three identical attempts (task's own
    # max_turns is 3) so the refusal itself is what ends the step, rather than
    # the model running out of scripted turns and falling into the separate
    # prose-stop path.
    runner, _, _ = build(task, [call('report', value=9.0, unit='V',
                                     note='never measured')] * 3)
    record = runner.run_task(runner.plan.tasks[0])
    report.check('report with nothing behind it is refused, not accepted',
                 record.verdict == 'unfinished'
                 and 'no board measurement' in record.warnings[0],
                 record.warnings[0] if record.warnings else '')

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
                 'samples @' in results[0]['result'],
                 results[0]['result'].splitlines()[-1][:52])
    report.check('the front end switch is not write-gated',
                 'on=1' in results[1]['result'], results[1]['result'])
    report.check('a reading comes back in the compact renderer',
                 'NTC' in results[2]['result']
                 and 'samples @' in results[2]['result'],
                 results[2]['result'].splitlines()[0])
    report.check('self test reaches the renderer',
                 'PLL lock' in results[3]['result'],
                 results[3]['result'].splitlines()[0])
    # Counted against the MCP set rather than a number written here, so
    # adding a tool on that side does not fail this for the wrong reason.
    from coaxial_mcp.tools import TOOLS as MCP_TOOLS
    report.check('the tool surface is the MCP set plus six',
                 len(toolmod.TOOLS) == len(MCP_TOOLS) + 6,
                 '%d tools, MCP has %d' % (len(toolmod.TOOLS),
                                           len(MCP_TOOLS)))

    schemas = toolmod.schemas()
    shapes = all(s['type'] == 'function' and s['function']['parameters']['type']
                 == 'object' for s in schemas)
    report.check('every schema is an ollama function schema', shapes)


# ---- code and commands -----------------------------------------------------

def test_scope(report):
    scope = Scope(board=SimulatedBoard())

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
    scope = Scope(board=SimulatedBoard())

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
    """|robot icon| Coaxial_63<bar>00> - the bar spins in place of the
    text's own '1'; the icon repaints on state changes next to it."""
    import time as clock
    from coaxial_ollama import spinner as spin

    # A real robot ("\U0001F916") is outside cp1252 entirely - not a wide or
    # combining character, just absent - so the fixture used for the "normal"
    # behaviour tests below has to be genuinely Unicode-capable, same as this
    # bench's actual console would need to be to show it for real. cp1252 gets
    # its own fixture further down, where the fallback itself is under test.
    # ascii(), not repr(), in every detail string below: this bench's own
    # console is cp1252 (confirmed further down), and a detail string prints
    # on PASS too - a raw robot in it would crash the report that is supposed
    # to say the check passed.
    class Tty(io.StringIO):
        encoding = 'utf-8'

        def isatty(self):
            return True

    text = 'Coaxial_63100'
    head, tail = 'Coaxial_63', '00'      # text split around its own '1'

    screen = Tty()
    face = spin.prompt(text, screen, tick=10)
    written = screen.getvalue()
    face.stop(True)

    report.check("the bar takes the text's own '1', not a character "
                 'appended after it',
                 written == '%s%s%s%s%s%s%s%s%s>'
                 % (spin.OPEN, spin.ROBOT, spin.ICON_WAIT,
                    spin.CLOSE, head, spin.GREEN, spin.BARS[0], spin.RESET,
                    tail),
                 ascii(written))
    report.check("and it rests on '1' itself, so the name reads normally "
                 'between ticks',
                 spin.BARS[0] == '1')

    no_digit = spin.prompt('no-ones-here', Tty(), tick=10)
    report.check("text with no '1' gets the bar appended after it instead, "
                 'same as the very first version of this',
                 no_digit.out.real.getvalue().endswith(
                     'no-ones-here%s%s%s>' % (spin.GREEN, spin.BARS[0],
                                              spin.RESET)),
                 ascii(no_digit.out.real.getvalue()))
    no_digit.stop(True)

    down = spin.prompt(text, Tty(), tick=10, ok=False)
    down_written = down.out.real.getvalue()
    report.check('a dead link starts with the error icon, red, not waiting',
                 down_written == '%s%s%s%s%s%s%s%s%s>'
                 % (spin.OPEN, spin.ROBOT, spin.ICON_ERROR,
                    spin.CLOSE, head, spin.RED, spin.BARS[0], spin.RESET,
                    tail),
                 ascii(down_written))
    down.stop(False)

    # ---- every repaint rewrites the whole group from column 1, via CR ------
    live = Tty()
    face = spin.prompt(text, live, tick=10, ok=True)
    before = live.getvalue()
    report.check('still on this row while nothing has been submitted',
                 face.rows_up == 0)
    face.busy()
    after_busy = live.getvalue()
    report.check('busy() turns yellow, switches to the busy icon and '
                 'targets one row up',
                 face.color == spin.YELLOW and face.icon == spin.ICON_BUSY
                 and face.rows_up == 1)
    added_busy = after_busy[len(before):]
    report.check('busy() repaints the whole group at once, from column 1 - '
                 'not a jump to a computed column',
                 added_busy == spin.SAVE + (spin.UP % 1) + '\r'
                 + face._prefix() + spin.RESTORE,
                 ascii(added_busy))
    report.check('and the rewritten prefix carries the busy icon and the '
                 "bar's new colour - the icon's own shape is the signal, "
                 'not a colour on top of it',
                 spin.ICON_BUSY in face._prefix()
                 and spin.YELLOW in face._prefix())

    face.stop(True)
    after_stop = live.getvalue()
    added_stop = after_stop[len(after_busy):]
    # tick=10 guarantees the background thread never fires in this test's
    # lifetime, so the bar's frame is still 0 throughout.
    report.check('stop() repaints the same way - one rewrite, back to '
                 'waiting and green',
                 added_stop == spin.SAVE + (spin.UP % 1) + '\r'
                 + face._prefix() + spin.RESTORE,
                 ascii(added_stop))
    report.check('and the rewritten prefix carries the waiting icon and '
                 'green again',
                 spin.ICON_WAIT in face._prefix()
                 and spin.GREEN in face._prefix())

    # Measured live: a real session left the bar frozen on '/' or '-' in
    # scrollback, because stop() reset the icon and colour but never the
    # frame the ticker had drifted to while the operator was still typing
    # the question. tick=10 keeps the background thread from firing here
    # too, so the frame is set by hand to stand in for "several ticks
    # already happened" without a real, timing-dependent sleep.
    drifted = Tty()
    mid_spin = spin.prompt(text, drifted, tick=10, ok=True)
    mid_spin.frame = 2       # as if the bar had ticked to '-' before stop()
    mid_spin.busy()
    mid_spin.stop(True)
    expected = '%s%s%s%s%s%s%s%s' % (spin.OPEN, spin.ROBOT, spin.ICON_WAIT,
                                     spin.CLOSE, head, spin.GREEN,
                                     spin.BARS[0], spin.RESET)
    report.check("stop() rests the bar back on the text's own '1', not "
                 'whatever frame the ticker had drifted to',
                 mid_spin._prefix() == expected, ascii(mid_spin._prefix()))

    same_row = Tty()
    still_waiting = spin.prompt(text, same_row, tick=10, ok=True)
    before2 = same_row.getvalue()
    still_waiting.stop(False)          # never went busy() - still row 0
    added2 = same_row.getvalue()[len(before2):]
    report.check('stopping before busy() repaints this row, not one up - '
                 'no UP sequence at all',
                 added2 == spin.SAVE + '\r' + still_waiting._prefix()
                 + spin.RESTORE,
                 ascii(added2))

    # ---- reported live: a tick landed inside a channel table mid-print,   -
    # ---- because "one row up" was fixed at busy() time, not recomputed ---
    # ---- as _trace() kept printing more of it. -----------------------------
    busy_screen = Tty()
    drifting = spin.prompt(text, busy_screen, tick=10, ok=True)
    drifting.busy()
    report.check('right after busy(), before anything else prints, up is '
                 'exactly one row',
                 drifting.rows_up + drifting.out.lines == 1)
    # Chat.out is repointed at face.out for exactly this reason - whatever
    # _trace() or the final answer prints goes through the same counter.
    drifting.out.write('0  PhaseU  diff    899.2  +0.0906V\n')
    drifting.out.write('1  PhaseV  diff  -8672.5  -0.8734V\n'
                       '2  PhaseW  diff    -65.3  -0.0066V\n')
    report.check('the tracked stream counts every newline written through '
                 'it, from any number of separate writes',
                 drifting.out.lines == 3)
    before3 = busy_screen.getvalue()
    drifting.stop(True)
    added3 = busy_screen.getvalue()[len(before3):]
    report.check("stop() climbs 1 (Enter) + 3 (what printed since) = 4 "
                 'rows, not the 1 it would have used before this fix',
                 added3 == spin.SAVE + (spin.UP % 4) + '\r'
                 + drifting._prefix() + spin.RESTORE,
                 ascii(added3))

    # ---- it actually ticks on a real thread when the stream is a terminal -
    ticking = Tty()
    live_face = spin.prompt(text, ticking, tick=0.01, ok=True)
    clock.sleep(0.08)
    live_face.stop(True)
    report.check('a real terminal gets more than the first frame - it ticks',
                 ticking.getvalue().count(spin.SAVE) >= 3,
                 '%d repaints' % ticking.getvalue().count(spin.SAVE))

    # ---- a shared lock keeps a tick and a trace print from interleaving ---
    # Real Chat._trace() output while the bar is still ticking for the busy
    # phase is exactly the race this lock exists for - proved here by holding
    # it ourselves and showing a tick cannot get past it, not by hoping a
    # timing-based race never happens to interleave in the test run.
    shared = threading.RLock()
    locked = Tty()
    guarded = spin.prompt(text, locked, tick=0.01, ok=True, lock=shared)
    report.check('the caller-supplied lock is the one actually used',
                 guarded.lock is shared)

    before_lock = locked.getvalue()
    shared.acquire()
    try:
        clock.sleep(0.05)          # several tick intervals, all held off
        report.check('a tick blocks on the shared lock instead of racing '
                     'past it',
                     locked.getvalue() == before_lock)
    finally:
        shared.release()
    clock.sleep(0.05)
    guarded.stop(True)
    report.check('and ticking resumes once the lock is free again',
                 locked.getvalue() != before_lock)

    # ---- _Tracked's own lock protects a caller who never wraps their write
    # Reported live: the answer text came back with the prompt group spliced
    # into the middle of a sentence - print(answer, file=face.out) never
    # wrapped itself in the lock, so nothing stopped a tick writing at the
    # same time. _Tracked now holds the lock around every write it is given,
    # whether the caller asked for that or not. Proved with a deliberately
    # slow underlying stream, so a second write demonstrably waits for the
    # first one's write() call to finish rather than hoping a fast race
    # never happens to land badly in the time a test happens to run.
    class SlowReal:
        def __init__(self):
            self.log = []

        def write(self, text):
            self.log.append(('start', text))
            clock.sleep(0.06)
            self.log.append(('end', text))

        def flush(self):
            pass

        def isatty(self):
            return True
        encoding = 'utf-8'

    slow_real = SlowReal()
    tracked = spin._Tracked(slow_real, threading.RLock())

    first = threading.Thread(target=lambda: tracked.write('FIRST'))
    first.start()
    clock.sleep(0.02)                  # first is now inside its slow write()

    second_done = []
    second = threading.Thread(
        target=lambda: (tracked.write('SECOND'), second_done.append(True)))
    second.start()
    clock.sleep(0.02)                  # well within FIRST's write, before it ends

    report.check("a second write through _Tracked - unguarded by the "
                 "caller, exactly how print(answer, file=face.out) is - "
                 "waits for the first one's write() call to finish, not "
                 'just start',
                 not second_done and slow_real.log == [('start', 'FIRST')],
                 slow_real.log)
    first.join()
    second.join()
    report.check('and completes cleanly once the first is done, never '
                 'interleaved',
                 slow_real.log == [('start', 'FIRST'), ('end', 'FIRST'),
                                   ('start', 'SECOND'), ('end', 'SECOND')],
                 slow_real.log)

    report.check('the robot and the two resting icons are real emoji, not '
                 'look-alike runs of ASCII, and need no variation selector '
                 'to render in colour - unlike the pause mark and the '
                 'warning sign that each sat here before',
                 spin.ROBOT == '\U0001F916'
                 and spin.ICON_WAIT == '\U0001F4A4'
                 and spin.ICON_ERROR == '❌'
                 and len(spin.ICON_WAIT) == len(spin.ICON_ERROR) == 1)
    # The gear is the exception, and a deliberate one: asked for by name
    # because it says "working" where an hourglass says "waiting".
    # U+2699 is text-presentation, so it needs the U+FE0F the other three
    # do not - which is the forced-colour case measured as uneven spacing.
    # Asserted as it is so the trade stays visible, not asserted away.
    report.check('the busy icon is the gear, selector and all',
                 spin.ICON_BUSY == '\u2699\ufe0f',
                 spin.ICON_BUSY.encode('unicode_escape').decode())
    report.check('and it is the only one carrying a selector',
                 not any('\ufe0f' in icon for icon in
                         (spin.ROBOT, spin.ICON_WAIT, spin.ICON_ERROR)))
    report.check('none of that matters for positioning any more - every '
                 'repaint rewrites from column 1, not a computed one',
                 not hasattr(face, 'icon_column')
                 and not hasattr(face, 'bar_column'))
    report.check('the bar is plain ASCII - no width question to inherit '
                 'in the first place',
                 all(len(b) == 1 and ord(b) < 128 for b in spin.BARS))
    report.check('the guillemets are cp1252, not something else risking a '
                 'question mark of their own',
                 spin.OPEN.encode('cp1252') == b'\xab'
                 and spin.CLOSE.encode('cp1252') == b'\xbb')

    class Cp1252(io.StringIO):
        encoding = 'cp1252'

    report.check("this bench's own console cannot hold the robot/icons "
                 '- they get ASCII, not a question mark',
                 not spin._capable(Cp1252()))
    report.check('but it can hold the guillemets on their own - a real '
                 'frame around ASCII bookends, not pipes just because the '
                 'robot cannot render',
                 spin._brackets_capable(Cp1252()))

    class Ascii(Cp1252):
        encoding = 'ascii'

    report.check('and the same fallback covers a plain ASCII stream too',
                 not spin._capable(Ascii())
                 and not spin._brackets_capable(Ascii()))
    report.check('a genuinely Unicode-capable stream is the real thing',
                 spin._capable(Tty()) and spin._brackets_capable(Tty()))

    cp1252_face = spin.prompt(text, Cp1252(), tick=10)
    report.check('cp1252 gets real guillemets around ASCII bookends, not '
                 'plain pipes',
                 cp1252_face.open == spin.OPEN
                 and cp1252_face.close == spin.CLOSE
                 and cp1252_face.robot == spin.ROBOT_FALLBACK,
                 ascii(cp1252_face._prefix()))
    cp1252_face.stop(True)

    # A pipe has no cursor to save: one static prompt, no escapes, no thread -
    # busy()/stop() change state but paint nothing further. It has no
    # .encoding either, so it gets the ASCII fallback same as Ascii() does,
    # brackets included.
    piped = io.StringIO()
    quiet = spin.prompt(text, piped, tick=10)
    before_pipe = piped.getvalue()
    quiet.busy()
    quiet.stop(False)
    report.check('a redirected prompt is static and escape-free',
                 before_pipe == '%s%s%s%s%s%s%s%s%s>'
                 % (spin.OPEN_FALLBACK, spin.ROBOT_FALLBACK,
                    spin.ICON_WAIT_FALLBACK,
                    spin.CLOSE_FALLBACK, head, spin.GREEN, spin.BARS[0],
                    spin.RESET, tail),
                 repr(before_pipe))
    report.check('and busy()/stop() on a redirected stream paint nothing '
                 'further',
                 piped.getvalue() == before_pipe)
    report.check('no background thread was even started for it',
                 quiet.thread is None)


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

    # A build prints its command line first and its diagnosis last, so a
    # head-only cut keeps the banner and throws the answer away.
    from coaxial_ollama import sandbox
    long_run = sandbox.clip_ends('START\n' + 'noise\n' * 4000 + 'ERROR: END')
    report.check('a clipped process output keeps the end, where the verdict is',
                 long_run.endswith('ERROR: END') and long_run.startswith('START')
                 and len(long_run) < 4200
                 and 'cut from the middle' in long_run,
                 '%d chars' % len(long_run))
    report.check('a short output is not touched',
                 sandbox.clip_ends('exit=0') == 'exit=0')

    # The ceiling that matters is the one at the dispatch point: every tool
    # goes through it, including the three that run subprocesses of their own
    # and used to return whatever those printed.
    report.check('no tool result may be larger than the ceiling',
                 len(toolmod.bounded('q' * 50000)) < toolmod.TOOL_LIMIT + 120)
    report.check('an ERR prefix survives the ceiling, or the loop stops '
                 'reading a failure as one',
                 toolmod.bounded('ERR ' + 'q' * 50000).startswith('ERR '))
    kept = toolmod.Reported(1.0, 'V', 'note')
    report.check('a report is not text and is not clipped',
                 toolmod.bounded(kept) is kept)


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

    # Measured live: told to turn the AFE off, then asked in a later,
    # unrelated turn for a reading, gemma4:12b turned it back on to "serve"
    # the reading - exactly what the system prompt already says never to do.
    # afe_mentioned defaults True (every existing fixture above never sets
    # it, on purpose - see Toolbox.__init__) so only debug.py's repl() and
    # one-shot path, which set it from the real question text, are covered.
    runner, _, _ = build(task, [])
    toolbox = runner.toolbox
    toolbox.afe_mentioned = False
    refused = toolbox.call('afe_power', {'action': 'on'})
    report.check('afe_power is refused when nothing this turn mentioned the '
                 'AFE', 'ERR' in refused and 'analog_read instead' in refused,
                 refused)
    report.check('a plain state read is never gated by this - it changes '
                 'nothing',
                 not str(toolbox.call('afe_power', {'action': 'read'})
                        ).startswith('ERR'))
    toolbox.afe_mentioned = True
    report.check('and it goes through once the question actually mentioned '
                 'it',
                 not str(toolbox.call('afe_power', {'action': 'on'})
                        ).startswith('ERR'))


def test_power_check_cannot_halt(report):
    """Diagnosing the link must not be able to break it.

    Measured on this bench, and it cost most of a session: `find_board.py
    --power` timed out at 15s, and every serial call after it was silent -
    the console said nothing, a raw Modbus frame said nothing - until
    `-c port=SWD mode=UR --start` brought the board back. The programmer
    had been killed mid-connect-under-reset with NRST asserted, and a
    halted core answers nothing on USART3.

    link_diagnose calls check_power as step 1 and then asks in step 4
    whether the board answers, so the checklist was able to cause the
    silence it went on to report.
    """
    import find_board
    import subprocess

    seen = {}

    class Done:
        stdout, stderr, returncode = 'Voltage     : 3.27V', '', 0

    def spy(argv, **kw):
        seen['argv'] = list(argv)
        seen['timeout'] = kw.get('timeout')
        return Done()

    real_run = subprocess.run
    try:
        subprocess.run = spy
        voltage, _ = find_board.check_power()
    finally:
        subprocess.run = real_run

    argv = seen.get('argv') or []
    report.check('check_power reads the voltage', voltage == 3.27, str(voltage))
    report.check('and never connects under reset',
                 'mode=UR' not in argv, ' '.join(argv[1:]) or 'no call made')
    report.check('it hotplugs instead - reset is never asserted, so a kill '
                 'cannot leave the core halted',
                 'mode=HOTPLUG' in argv, ' '.join(argv[1:]) or 'no call made')
    report.check('and it is still bounded, since that kill is what happens '
                 'on a wedged probe', bool(seen.get('timeout')),
                 str(seen.get('timeout')))


def test_retype_with_the_trace_off(report):
    """A silenced retype must not leave an empty screen.

    SYSTEM says never to restate a tool's own rows, and `is_retype` replaces
    an answer that does it with silence - right, because the trace put the
    table directly above. With `--quiet` there is no trace, and "read every
    analog channel" then answered with nothing at all. The board's own rows
    go out instead: the same table, not the model's typing of it.
    """
    from coaxial_ollama import debug

    retyped = ('PhaseU, PhaseV, PhaseW, Clevel, NTC, DCbus and Cinj were '
               'all read just now.')

    def turn(quiet):
        chat = debug.Chat(ScriptedModel([
            call('analog_read'),
            {'role': 'assistant', 'content': retyped},
        ]), toolmod.Toolbox(SimulatedSession(), scope=Scope()),
            out=io.StringIO(), quiet=quiet)
        return chat.ask('read every analog channel')

    loud = turn(quiet=False)
    report.check('with the trace on, a retyped table is still silenced',
                 loud == '', repr(loud)[:52])

    hushed = turn(quiet=True)
    report.check('with --quiet it is replaced, not silenced',
                 hushed.strip() != '', repr(hushed)[:52])
    report.check('and what goes out is the board rows, not the retyping',
                 'samples @' in hushed and retyped not in hushed,
                 hushed.splitlines()[0][:52] if hushed else '<empty>')


def test_port_state(report):
    """Why a port is not answering, not just that it is not.

    Measured, and it cost most of a session twice over: two `dbg.py`
    sessions had COM4 open, every `probe` read "silent", and the board was
    diagnosed as halted, started over SWD and reflashed - none of which was
    the matter with it. pyserial says exactly what happened; nothing was
    asking.
    """
    import find_board
    import serial

    real_serial = serial.Serial
    try:
        def denied(*a, **kw):
            # The message Windows produced here, Swedish locale and all.
            raise serial.SerialException(
                "could not open port 'COM4': PermissionError(13, "
                "'Åtkomst nekad.', None, 5)")

        serial.Serial = denied
        report.check('a port another process holds is busy, not silent',
                     find_board.port_state('COM4') == find_board.BUSY,
                     find_board.port_state('COM4'))

        def missing(*a, **kw):
            raise serial.SerialException(
                "could not open port 'COM99': FileNotFoundError(2, ...)")

        serial.Serial = missing
        report.check('a port that is not there is absent, not busy',
                     find_board.port_state('COM99') == find_board.ABSENT,
                     find_board.port_state('COM99'))
    finally:
        serial.Serial = real_serial

    # The class name is what decides it: the OS message is localised, and
    # matching "Access is denied" would have read this one as absent.
    report.check('and it is decided on the class name, not the OS wording',
                 'PermissionError' in open(find_board.__file__,
                                           encoding='utf-8').read())

    # The checklist stops guessing at it. Step 4 used to end on "check
    # nothing else has COM4 open" whether or not something did.
    box = toolmod.Toolbox(SimulatedSession())
    real_state = find_board.port_state
    real_power = find_board.check_power
    real_ports = find_board.list_ports
    try:
        find_board.port_state = lambda *a, **kw: find_board.BUSY
        find_board.check_power = lambda *a, **kw: (3.27, 'stubbed')
        find_board.list_ports = lambda: ['COM_TEST']
        box.session = _Held()
        checklist = box.call('link_diagnose', {})
    finally:
        find_board.port_state = real_state
        find_board.check_power = real_power
        find_board.list_ports = real_ports
    report.check('link_diagnose says the port is held, not that it might be',
                 'open in another process' in checklist,
                 checklist.splitlines()[-1][:56])


class _Held:
    """A session on a port something else has open."""
    port, baud, unit = 'COM_TEST', 115200, 1
    _board = None


def test_map_retype(report):
    """One list, not two.

    Measured: "ge mig en lista pa alla analoga kanaler" traced the map -
    seven named rows - and the model then typed the same seven names out
    underneath it. `is_retype` already replaces a retyped *reading* with
    silence because the trace put it on screen; a retyped *map* is the same
    thing, and a map row simply does not look like a reading row, so the
    backstop never saw it.
    """
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_ollama import debug

    # The package's stand-in, not this file's four-channel double: the names
    # below are the seven the board actually reports, and the check is that
    # all of them being typed out again is what the backstop sees.
    listed = ('Här är de analoga kanalerna: PhaseU, PhaseV, PhaseW, Clevel, '
              'NTC, DCbus och Cinj.')

    from coaxial_ollama import replies as repliesmod

    def turn(reply, quiet=False):
        chat = debug.Chat(ScriptedModel([
            call('board_info', kind='analog'),
            {'role': 'assistant', 'content': reply},
        ]), toolmod.Toolbox(Sim(), scope=Scope()),
            out=io.StringIO(), quiet=quiet)
        return chat.ask('ge mig en lista på alla analoga kanaler')

    report.check('a map typed out again under its own trace is silenced',
                 turn(listed) == '', repr(turn(listed))[:52])

    # With no trace the answer is the only copy, so the board's rows go out
    # rather than the model's typing of them - same rule as a reading.
    hushed = turn(listed, quiet=True)
    report.check('with --quiet the map itself goes out instead',
                 'analog:' in hushed and listed not in hushed,
                 hushed.splitlines()[0][:46] if hushed else '<empty>')

    # The bar stays where it was: an answer that says something is untouched.
    finding = 'Sju analoga kanaler, och NTC är den enda med en temperatur.'
    report.check('an answer that is not just the list survives',
                 turn(finding) == finding, repr(turn(finding))[:52])

    # Length is what tells a list from an explanation. Measured: a 43-word
    # answer to "beskriv hardvaran i detta projektet for en novis", naming
    # all seven channels because describing them IS the question, was
    # deleted to an empty screen - and reported as the model failing to
    # answer at all. A description is not a restatement however many names
    # it happens to contain.
    described = ('Kortet är en trefas BLDC-drivare. Framänden matar sju '
                 'ADC-kanaler: PhaseU, PhaseV och PhaseW mäter de tre '
                 'faserna differentiellt bakom okänd förstärkning, NTC är '
                 'termistorn, DCbus är mellanledet genom en spänningsdelare, '
                 'och Clevel och Cinj är två kanaler vars signaler inte är '
                 'dokumenterade här.')
    report.check('a description that names every channel is not a retype',
                 turn(described) == described,
                 '%d words -> %s' % (len(described.split()),
                                     'kept' if turn(described) else 'DELETED'))
    report.check('and the bar is where the measurements put it',
                 repliesmod.RESTATE_MAX_WORDS == 20,
                 str(repliesmod.RESTATE_MAX_WORDS))

    # A markdown table is caught whatever its length - SYSTEM says never to
    # write one, and a long one is worse than a short one.
    wide = ('| ch | name | value |' + chr(10) + '| -- | ---- | ----- |'
            + chr(10) + ('| 0 | PhaseU | 1445.2 |' + chr(10)) * 3
            + ' '.join('word' for _ in range(40)))
    report.check('a long markdown table is still silenced',
                 repliesmod.is_retype(wide, {'phaseu'}, minimum=1),
                 '%d words' % len(wide.split()))

    # The digital blocks, both of them. A digital row names its pin in the
    # first column and never starts with a digit, so MAP_ROW - anchored on
    # the analog shape - never saw one. Measured under the map's own two
    # rows: "De digitala kanalerna ar: PB2 (utgang) for AFE_ON, PE15
    # (ingang) for nFAULT".
    def digital_turn(tool, reply, quiet=False, **args):
        chat = debug.Chat(ScriptedModel([
            call(tool, **args),
            {'role': 'assistant', 'content': reply},
        ]), toolmod.Toolbox(Sim(), scope=Scope()),
            out=io.StringIO(), quiet=quiet)
        return chat.ask('ge mig en lista på de digitala kanalerna')

    listed_pins = ('De digitala kanalerna är:' + chr(10)
                   + 'PB2 (utgång) för AFE_ON' + chr(10)
                   + 'PE15 (ingång) för nFAULT')
    report.check('a retyped digital map is silenced',
                 digital_turn('board_info', listed_pins, kind='digital') == '',
                 repr(digital_turn('board_info', listed_pins,
                                   kind='digital'))[:46])
    report.check('and so is a retyped digital reading',
                 digital_turn('digital_read',
                              'PB2 är 1 och PE15 är 0.') == '')

    # A row can be named back two ways, and the model picks one. Measured:
    # the trace said "PB2 out 1 AFE_ON / PE15 in 0 nFAULT" and the answer
    # said "AFE_ON ar 1 och nFAULT ar 0" - every channel named, not one of
    # them by the pin. Pins and signals are alternatives, not a union: a
    # union would want every name from every column present.
    for reply, silent, why in (
            ('AFE_ON är 1 och nFAULT är 0.', True, 'named by signal'),
            ('PB2 är 1 och PE15 är 0.', True, 'named by pin'),
            ('PB2 (AFE_ON) 1, PE15 (nFAULT) 0.', True, 'named both ways'),
            ('nFAULT är asserterad medan AFE:n är på.', False,
             'one signal, and something to say about it'),
            ('AFE_ON är 1.', False, 'one of two is not the list')):
        got = digital_turn('digital_read', reply)
        report.check('a reading %s -> %s' % (why,
                                             'silent' if silent else 'kept'),
                     (got == '') is silent, repr(got)[:44])

    # The same, off the map rather than a reading.
    for reply, silent in (('AFE_ON och nFAULT.', True),
                          ('Två digitala kanaler.', False)):
        got = digital_turn('board_info', reply, kind='digital')
        report.check('a map named back by signal -> %s'
                     % ('silent' if silent else 'kept'),
                     (got == '') is silent, repr(got)[:44])
    report.check('but a finding that does not name them all survives',
                 digital_turn('board_info', 'nFAULT är asserterad.',
                              kind='digital') == 'nFAULT är asserterad.')
    hushed = digital_turn('digital_read', 'PB2 är 1 och PE15 är 0.',
                          quiet=True)
    report.check('with --quiet the block goes out instead of the retyping',
                 'digital:' in hushed and 'PB2 är 1' not in hushed,
                 hushed.splitlines()[0] if hushed else '<empty>')

    # The two bars are different on purpose. Two channels of a *reading*
    # named together is plausibly synthesis - "NTC and DCbus both read low"
    # is a finding, not a restatement - so a reading still needs three.
    two = {'ntc', 'dcbus'}
    report.check('two channels of a reading are not a restatement',
                 not repliesmod.is_retype('NTC och DCbus ligger båda lågt.',
                                          two))
    report.check('but two of a map are, because a map is the list',
                 repliesmod.is_retype('PB2 och PE15.', {'pb2', 'pe15'},
                                      minimum=2))


def test_reading_block(report):
    """A reading is a headed block, like the map is.

    Asked for both the digital and the analog values, the two arrived as one
    wall: a bare `64 smp @2000Hz` over unlabelled columns, then the digital
    table straight underneath with no gap. The reading is counted and headed
    now, and the trace puts a blank line between blocks.
    """
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_mcp import tools as mcp
    from coaxial_ollama import debug, language

    reading = mcp.HANDLERS['analog_read'](Sim(), samples=64)
    head = [l for l in reading.splitlines() if not l.startswith('AFE')]

    report.check('the reading says how many channels it read',
                 head[0] == 'analog: 7 channels', head[0])
    report.check('and how many samples, spelled out',
                 head[1] == '64 samples @2000Hz', head[1])
    report.check('and names its columns', head[2].split()
                 == ['ch', 'name', 'mode', 'code', 'voltage', 'measure'],
                 head[2])

    # The header is built from the row's own widths, not written out beside
    # them. By hand they drifted: `code` and `voltage` are right-aligned, so
    # a seven-digit number starts a column later than an eight-digit one and
    # a heading placed over the first is wrong for the second.
    from coaxial_mcp import render
    report.check('the header comes from the row format, not from a literal',
                 head[2] == render.ANALOG_HEAD, head[2])

    # Left-aligned columns start where their heading does; right-aligned
    # ones end where theirs does.
    row = next(l for l in head if l.startswith('4 '))
    for word, field in (('name', 'NTC'), ('mode', 'SE')):
        report.check('the %s column starts under its heading' % word,
                     head[2].index(word) == row.index(field),
                     '%d vs %d' % (head[2].index(word), row.index(field)))
    for word, field in (('code', '32768.0'), ('voltage', '+1.6500V')):
        report.check('the %s column ends under its heading' % word,
                     head[2].index(word) + len(word)
                     == row.index(field) + len(field),
                     '%d vs %d' % (head[2].index(word) + len(word),
                                   row.index(field) + len(field)))

    levels = mcp.HANDLERS['digital_read'](Sim())
    report.check('the digital block counts channels, not pins',
                 levels.splitlines()[0] == 'digital: 2 channels',
                 levels.splitlines()[0])
    report.check('and both blocks start their first column the same way',
                 levels.splitlines()[1].split()[0] == 'ch'
                 and head[2].split()[0] == 'ch')
    report.check('while the reserved list stays pins - it is not channels',
                 mcp.HANDLERS['board_info'](Sim(), kind='reserved')
                 .startswith('reserved: 7 pins'))

    # Swedish on screen, English on the wire: the headings turn, the column
    # names do not - they are the board's words, like a channel name.
    turned = language.localise(reading, 'Swedish')
    report.check('the headings turn for a Swedish session',
                 'analog: 7 kanaler' in turned and '64 sampel @' in turned,
                 [l for l in turned.splitlines() if 'kanaler' in l][:1])
    report.check('and the column names do not',
                 render.ANALOG_HEAD in turned)

    # Two blocks on one screen get a line between them.
    screen = io.StringIO()
    chat = debug.Chat(ScriptedModel([
        call('analog_read'), call('digital_read'),
        {'role': 'assistant', 'content': 'Klart.'},
    ]), toolmod.Toolbox(Sim(), scope=Scope()), out=screen)
    chat.ask('ge mig värdena på de digitala och analoga kanalerna')
    printed = screen.getvalue().splitlines()
    at = next(i for i, l in enumerate(printed) if 'digital:' in l)
    report.check('a second block is separated from the first',
                 printed[at - 1].strip() == '', repr(printed[at - 1])[:40])
    report.check('but the first block has no blank line above it',
                 printed[0].strip() != '', repr(printed[0])[:40])


def test_corrections_are_reported(report):
    """A mistake in the question is answered, and said out loud.

    Refusing a misspelt channel is worse than reading the one it meant -
    but reading it silently is worse still: a question about `BUS_VOLT`
    coming back as a DC bus reading, with nothing saying so, is the quiet
    substitution this library exists to prevent.
    """
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_mcp import tools as mcp
    from coaxial_ollama import debug

    session = Sim()
    mcp.HANDLERS['afe_power'](session, action='on')

    for asked, meant, corrected in (
            ('NTC', 'NTC', False),
            ('dc_bus', 'DCbus', False),      # a spelling, not a mistake
            ('temp', 'NTC', True),
            ('BUS_VOLT', 'DCbus', True),
            ('phase_a', 'PhaseU', True)):
        text = mcp.HANDLERS['analog_read'](session, ch=[asked], samples=8)
        head = text.splitlines()[0]
        report.check('ch=[%r] reads %s' % (asked, meant),
                     meant in text, head[:46])
        report.check('   ...and %s'
                     % ('says what it corrected' if corrected
                        else 'has nothing to correct'),
                     head.startswith('read as asked') is corrected, head[:46])

    # More than one, in one line, and the reading still arrives under it.
    both = mcp.HANDLERS['analog_read'](session, ch=['temp', 'bus'], samples=8)
    report.check('two corrections are named together',
                 both.splitlines()[0].count(' read as ') == 2,
                 both.splitlines()[0][:52])
    report.check('and the reading is still there under them',
                 'NTC' in both and 'DCbus' in both and 'samples @' in both)

    # A name that means nothing is still refused - correcting it would be
    # inventing, not reading what was meant.
    try:
        mcp.HANDLERS['analog_read'](session, ch=['not_a_channel'])
        report.check('a name that means nothing is refused, not corrected',
                     False, 'it read something')
    except ValueError as exc:
        report.check('a name that means nothing is refused, not corrected',
                     'unknown channel' in str(exc), str(exc)[:46])

    report.check('and the prompt says the same thing one layer up',
                 'a typo or a wrong fact' in debug.SYSTEM
                 and 'answer what was meant' in debug.SYSTEM,
                 [l for l in debug.SYSTEM.splitlines()
                  if 'typo' in l][:1])


def test_smart_selection(report):
    """Which suites a change can have broken.

    The live suite is a model load plus a turn per question - minutes - so
    running it for a change to a channel renderer is most of a coffee break
    spent proving nothing. `run_tests.py --smart` maps the changed files to
    the suites that cover them, and runs the lot every tenth commit because
    a map from files to suites is a guess about coupling and a guess that
    is never checked is one that drifts.
    """
    import os
    import sys as _sys
    _sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'tools'))
    import run_tests

    for paths, expect_suites, expect_live, why in (
            (['host/coaxial_ollama/language.py'],
             {'test_ollama.py'}, {'language'},
             'the language lock is half the live suite, not all of it'),
            (['host/coaxial_ollama/replies.py'],
             {'test_ollama.py'}, {'tools'},
             'what an answer means is the tool-choice half'),
            (['host/coaxial_mcp/render.py'],
             {'test_mcp.py', 'test_ollama.py', 'test_parity.py'}, set(),
             'a renderer cannot change which tool gets called'),
            (['docs/HARDWARE.md'], {'test_ollama.py'}, set(),
             'a document can only break the docs index'),
            (['Modbus/Src/modbus_rtu.c'],
             {'test_conformance.py', 'test_mcp.py'}, set(),
             'the wire is what the byte-level master is for'),
            (['host/tests/test_parity.py'], {'test_parity.py'}, set(),
             'editing a suite is a reason to run it'),
            (['host/coaxial_ollama/debug.py'],
             {'test_ollama.py'}, {'all'},
             'the prompt and the tool sets are both in there'),
    ):
        suites, live, _ = run_tests.pick(paths)
        report.check('%s -> %s' % (paths[0].rsplit('/', 1)[-1],
                                   ', '.join(sorted(suites)) or 'none'),
                     suites == expect_suites, ', '.join(sorted(suites)))
        report.check('   ...and live: %s (%s)'
                     % (', '.join(sorted(expect_live)) or 'not at all', why),
                     live == expect_live, ', '.join(sorted(live)) or 'none')

    # A path the map does not know is the case to fail safe on: run
    # everything rather than quietly cover nothing.
    suites, live, why = run_tests.pick(['setup.ps1'])
    report.check('an unmapped path runs everything, and says why',
                 run_tests.CONFORMANCE in suites and live == {'all'}
                 and 'unmapped' in why[-1], why[-1][:52])

    report.check('and the whole lot goes every tenth commit',
                 run_tests.FULL_EVERY == 10)


def test_afe_trace(report):
    """A switch that did what it was told needs no line of its own.

    Measured: "sla pa afen" traced `on=1 pe15=0` above an answer that said
    the same thing in words. `on=1` is also the only evidence the write
    landed, so the line goes only when the read-back matches what was
    asked - a request that did not take still prints, loudly.
    """
    from coaxial_ollama import debug

    for action, raw, silent, why in (
            ('on', 'on=1 pe15=0', True, 'did what it was told'),
            ('off', 'on=0 pe15=1', True, 'did what it was told'),
            ('on', 'on=0 pe15=1', False, 'the write did not take'),
            ('off', 'on=1 pe15=0', False, 'the write did not take'),
            ('toggle', 'on=1 pe15=0', False, 'nothing to match it against'),
            ('read', 'on=1 pe15=0', False, 'the state is the answer'),
            ('on', 'ERR ConnectError: cable pulled', False, 'an error'),
            ('on', 'ERR not asked for', True, 'a refusal it recovers from')):
        got = debug._afe_noise('afe_power', {'action': action}, raw)
        report.check('afe_power %s -> %s (%s)'
                     % (action, 'silent' if silent else 'traced', why),
                     got is silent, 'silent' if got else 'traced')

    report.check('and no other tool is quietened by this',
                 not debug._afe_noise('analog_read', {}, 'on=1 pe15=0'))

    # End to end: the turn shows the answer, and nothing above it.
    screen = io.StringIO()
    chat = debug.Chat(ScriptedModel([
        call('afe_power', action='on'),
        {'role': 'assistant', 'content': 'AFE är nu påslagen.'},
    ]), toolmod.Toolbox(SimulatedSession(), scope=Scope()), out=screen)
    chat.toolbox.afe_mentioned = True
    said = chat.ask('slå på afen')
    report.check('a confirmed switch leaves nothing on screen above the '
                 'answer', screen.getvalue().strip() == '',
                 repr(screen.getvalue())[:46])
    report.check('and the answer itself still arrives',
                 said == 'AFE är nu påslagen.', said)


def test_digital_read(report):
    """Listing the channels and reading them are two questions.

    Measured: "ge mig en lista over alla digitala varden" - values - was
    answered with the channel list, because that is all there was. gpio_pin
    reads one pin and gpio_port hands back a register for the model to pick
    bits out of, which is arithmetic this library exists not to hand it.
    """
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_mcp import tools as mcp

    session = Sim()

    mcp.HANDLERS['afe_power'](session, action='on')
    hot = mcp.HANDLERS['digital_read'](session)
    mcp.HANDLERS['afe_power'](session, action='off')
    cold = mcp.HANDLERS['digital_read'](session)

    report.check('it has a level column the map does not',
                 'level' in hot.splitlines()[1]
                 and 'level' not in mcp.HANDLERS['board_info'](
                     session, kind='digital'),
                 hot.splitlines()[1])
    report.check('and the levels move with the board, not with the map',
                 hot != cold, '%r vs %r' % (hot.splitlines()[2][:22],
                                            cold.splitlines()[2][:22]))
    report.check('PB2 follows the AFE switch',
                 'PB2  out   1' in hot and 'PB2  out   0' in cold)
    # nFAULT, and the level is the same measurement it was before the pin
    # had that name - see FINDINGS: 0 with the front end powered reads as a
    # fault asserted, and what drives it is not established. Asserted here
    # because it is what the board does, not because it is understood.
    report.check('and nFAULT reads back inversely, as the board wires it',
                 'PE15 in    0     nFAULT' in hot
                 and 'PE15 in    1     nFAULT' in cold,
                 hot.splitlines()[-1])
    report.check('every pin the map calls digital I/O is read, and only those',
                 len(hot.splitlines()) == 2 + len(
                     Sim().board.system.channel_map()['digital']),
                 '%d rows' % (len(hot.splitlines()) - 2))
    report.check('a bus pin is not among them - it is not a channel',
                 'PB10' not in hot and 'PA13' not in hot)

    # A tool the model cannot call is a tool that does not exist. Measured:
    # digital_read worked and the question still came back with the list,
    # because it was in no named set and the default one is `code`.
    from coaxial_ollama import debug as debugmod
    missing = [name for name in ('read', 'code', 'pins')
               if 'digital_read' not in debugmod.SETS[name]]
    report.check('and the sets a bench session runs actually offer it',
                 not missing, ', '.join(missing) or 'read, code, pins')


def test_map_sections(report):
    """Analog and digital are two lists on the wire and two blocks on screen.

    Measured: asked "ge mig en lista over alla analoga kanaler", the trace
    carried the identity line, the clock line, seven analog rows and two
    digital ones - all under a single `ch adc pin dir mode name` header, with
    the digital rows carrying no index and their columns out of line. Eleven
    lines to answer with seven, and the two kinds mixed into one table.
    """
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_mcp import tools as mcp

    session = Sim()

    whole = mcp.HANDLERS['board_info'](session)
    report.check('the two kinds get their own headed blocks',
                 'analog: 7 channels' in whole
                 and 'digital: 2 channels' in whole,
                 whole.splitlines()[2] if whole else '<empty>')
    report.check('and the digital block has its own columns, not the analog '
                 'ones', 'ch   dir   name' in whole,
                 [l for l in whole.splitlines() if l.startswith('ch ')][:2])

    only = mcp.HANDLERS['board_info'](session, kind='analog')
    report.check('kind=analog is the analog block and nothing else',
                 only.startswith('analog:') and 'digital' not in only
                 and 'sysclk' not in only,
                 '%d lines' % len(only.splitlines()))
    report.check('and it is shorter than the whole thing',
                 len(only.splitlines()) < len(whole.splitlines()),
                 '%d vs %d lines' % (len(only.splitlines()),
                                     len(whole.splitlines())))

    pins = mcp.HANDLERS['board_info'](session, kind='digital')
    report.check('kind=digital is the I/O, and no analog channel',
                 pins.startswith('digital:') and 'PhaseU' not in pins,
                 '%d lines' % len(pins.splitlines()))

    held = mcp.HANDLERS['board_info'](session, kind='reserved')
    report.check('kind=reserved is the bus and the debug port, kept apart',
                 held.startswith('reserved:') and 'PB10' in held
                 and 'PB2 ' not in held, held.splitlines()[0])

    report.check('an unknown kind is refused by name, not guessed at',
                 mcp.HANDLERS['board_info'](session, kind='analogue')
                 .startswith('ERR unknown kind'))


class _NotATty:
    """A pipe, for _printable: records what it was reconfigured to."""
    asked = None

    def isatty(self):
        return False

    def reconfigure(self, **kw):
        _NotATty.asked = kw


def test_channel_map(report):
    """The board describes itself; nothing above it keeps a copy.

    Command 0x6D reports every channel - analog and digital - with the
    direction each one runs. The host used to hold three separate answers to
    "what is PB10": the firmware's testrig table, the firmware's pin table,
    and protocol.RESERVED_PINS. There is one now, and the other two read it.
    """
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial import protocol

    board = Sim().board
    chart = board.system.channel_map()

    report.check('the map has all three lists',
                 chart['analog'] and chart['digital'] and chart['reserved'],
                 '%d analog, %d digital, %d reserved'
                 % (len(chart['analog']), len(chart['digital']),
                    len(chart['reserved'])))
    # The separation is the point, not a flag on a row: what a fixture may
    # set without breaking anything is a different question from what the
    # bus and the debug port sit on, and mixing them invites a pin write
    # that gets refused.
    io_pins = {r['pin'] for r in chart['digital']}
    report.check('digital I/O is only what may be driven',
                 io_pins == {'PB2', 'PE15'}, ', '.join(sorted(io_pins)))
    report.check('and no bus or debug pin is among the channels',
                 not (io_pins & {r['pin'] for r in chart['reserved']}))
    report.check('every analog channel says which way it runs, and it is in',
                 all(row['direction'] == 'in' for row in chart['analog']),
                 ', '.join(sorted({r['direction']
                                   for r in chart['analog']})))
    report.check('every digital channel says which way it runs',
                 all(row['direction'] in protocol.DIRECTIONS.values()
                     for row in chart['digital'] + chart['reserved']),
                 ', '.join(sorted({r['direction']
                                   for r in chart['digital']})))
    report.check('the AFE switch is a digital output',
                 any(r['pin'] == 'PB2' and r['direction'] == 'out'
                     for r in chart['digital']))
    report.check('the link pins are reported, so a refusal can be explained',
                 all(any(r['pin'] == pin for r in chart['reserved'])
                     for pin in ('PB10', 'PB11')))

    # The static copy is the fallback, and it must not drift from the map
    # while it exists. A pin the board calls reserved and this dict does not
    # is the disagreement the whole change exists to prevent.
    refused = {r['pin'] for r in chart['reserved']}
    stale = {'P%s%d' % key for key in protocol.RESERVED_PINS} - refused
    report.check('the static fallback names no pin the board does not',
                 not stale, ', '.join(sorted(stale)) or 'none')

    # The refusal is the board's answer now, so its wording is the board's
    # signal name rather than a string compiled into the host.
    try:
        board.gpio.pin_read('B', 10)
        report.check('a reserved pin is refused by name', False, 'it was read')
    except ValueError as exc:
        report.check('a reserved pin is refused by the name the board gave it',
                     'USART3_TX' in str(exc), str(exc)[:52])


def test_link_recovery(report):
    """One screen, one verdict about the link.

    Measured at the prompt, all of this on a single question: the model
    called link_diagnose, whose step 4 said the board did not answer; the
    turn then ended with no answer, `_probe_link` failed, dropped the dead
    handle, and reported the link down - and `_link_down_message` ran
    link_diagnose a *second* time, which opened the port cleanly and said
    "4. Kortet svarar pa COM4 just nu: ja - lanken ar uppe."

    Two checklists, contradicting each other, seconds apart, with no way
    for the operator to tell which was true. Both halves were host bugs:
    the retry the reset exists for was never taken, and the one call site
    that could not afford to reprint the checklist was the one that never
    passed `shown`.
    """
    from coaxial_ollama import debug

    # Deliberately not the real step-4 wording: that one is in
    # language.PHRASES, so the trace comes back in the session's language
    # while the fake's return value stays English, and the count then
    # measures localisation rather than duplication. What is under test
    # here is only that the diagnosis is not printed twice.
    STEP4_UP = 'DIAGNOSIS MARKER - the link is up'
    DEAD = 'ERR NoReplyError: unit 1, fc 0x47: silence'

    class Flapping:
        """A link that answers only once the stale handle is dropped - the
        shape of a VCP that re-enumerated under a replugged cable. Stands
        in for both the toolbox and its session, which is all `_probe_link`
        and `_link_down_message` touch."""

        def __init__(self, fails=1):
            self.fails, self.resets, self.calls = fails, 0, []
            self.session = self
            self.log = []

        def reset(self):
            self.resets += 1

        def call(self, name, args):
            self.calls.append(name)
            if name == 'link_diagnose':
                return 'a two-line checklist' + chr(10) + STEP4_UP
            if self.fails > 0:
                self.fails -= 1
                return DEAD
            return 'frames rx=12 tx=12'

    def probing(fails):
        chat = debug.Chat.__new__(debug.Chat)
        chat.toolbox = Flapping(fails)
        chat.history, chat.link_ok, chat.quiet = [], True, True
        return chat, chat._probe_link()

    chat, probe = probing(1)
    report.check('a link that answers after the reset is reported up, not '
                 'down', chat.link_ok is True, str(probe)[:40])
    report.check('and the reset happened exactly once',
                 chat.toolbox.resets == 1, str(chat.toolbox.resets))
    report.check('which cost one retry, not a loop',
                 chat.toolbox.calls == ['link', 'link'],
                 ', '.join(chat.toolbox.calls))

    chat, probe = probing(99)
    report.check('a link that stays silent is still reported down',
                 chat.link_ok is False, str(probe)[:40])
    report.check('and is not retried past the one the reset earned',
                 chat.toolbox.calls == ['link', 'link'],
                 ', '.join(chat.toolbox.calls))

    # `shown` means the checklist is already on screen from the trace.
    chat.toolbox.calls = []
    said = chat._link_down_message(DEAD, shown=True)
    report.check('with the checklist already traced, the answer does not '
                 'run link_diagnose again', 'link_diagnose'
                 not in chat.toolbox.calls, ', '.join(chat.toolbox.calls))
    report.check('and does not reprint it either',
                 STEP4_UP not in said, said[:46])
    report.check('but still says the question went unanswered, and by what',
                 said.startswith('link is down, not answered:')
                 and 'NoReplyError' in said, said[:46])

    said = chat._link_down_message(DEAD, shown=False)
    report.check('with nothing on screen above, the checklist comes with '
                 'the answer', STEP4_UP in said, said.splitlines()[-1][:46])

    # End to end, the shape of the transcript: the model calls
    # link_diagnose and then writes nothing, so the turn falls to the
    # stale path. Trace and answer together are the operator's screen.
    screen = io.StringIO()
    box = Flapping(99)
    talk = debug.Chat(ScriptedModel([
        call('link_diagnose'),
        {'role': 'assistant', 'content': ''},
    ]), box, out=screen)
    talk.toolbox = box                      # the fake stands in for both
    # Not "byt till ..." any more: board_switch intercepts that before the
    # model is reached, and this needs a question that actually runs a turn.
    answer = talk.ask('vad läser NTC:n?')
    whole = screen.getvalue() + chr(10) + answer
    report.check('one screen carries the checklist once, not twice',
                 whole.count(STEP4_UP) == 1, '%d copies' % whole.count(STEP4_UP))
    report.check('so it cannot say the link is up and down at once',
                 not (STEP4_UP in answer and 'link is down' in answer),
                 answer.splitlines()[0][:46] if answer else '<empty>')


def test_link_diagnose(report):
    """OS-level, not another board round trip - see tools.py's own docstring
    for why. Ports come from a fake serial.tools.list_ports.comports() here,
    never from real hardware; coaxial.connect and find_board.check_power are
    faked too, for the same reason - both would otherwise probe whatever is
    really plugged into this bench and pass (or fail) for the wrong reason
    on a machine where it happens to answer."""
    import coaxial
    import find_board
    import serial.tools.list_ports as list_ports
    from types import SimpleNamespace

    class FakePort:
        def __init__(self, device):
            self.device = device

    real_comports = list_ports.comports
    real_connect = coaxial.connect
    real_check_power = find_board.check_power
    try:
        list_ports.comports = lambda: [FakePort('COM4'), FakePort('COM7')]
        coaxial.connect = lambda *a, **kw: (_ for _ in ()).throw(
            ConnectError('nothing answered'))
        find_board.check_power = lambda timeout=15: (3.30, 'fake: powered')

        missing = toolmod.Toolbox(SimpleNamespace(port='COM9', baud=115200, unit=1))
        result = missing.call('link_diagnose', {})
        report.check('powered, but a configured port absent from the OS '
                     'list is named as such, not folded into a generic '
                     'error',
                     'COM9' in result and 'not among' in result, result)

        present = toolmod.Toolbox(SimpleNamespace(port='COM4', baud=115200, unit=1))
        result2 = present.call('link_diagnose', {})
        report.check('powered and present, but silent, points at nothing '
                     'else having the port open, not the cable',
                     'COM4' in result2
                     and 'answers on COM4 right now: no' in result2, result2)

        coaxial.connect = lambda *a, **kw: []                # "answers"
        result2b = present.call('link_diagnose', {})
        report.check('and a port that actually answers says the link is '
                     'up, not "silent" just because it exists',
                     'link is up' in result2b, result2b)

        list_ports.comports = lambda: []
        empty = toolmod.Toolbox(SimpleNamespace(port='COM4', baud=115200, unit=1))
        result3 = empty.call('link_diagnose', {})
        report.check('no COM ports at all is named plainly',
                     'Nothing is enumerating' in result3, result3)

        find_board.check_power = lambda timeout=15: (0.0, 'fake: no power')
        unpowered = toolmod.Toolbox(SimpleNamespace(port='COM4', baud=115200, unit=1))
        result4 = unpowered.call('link_diagnose', {})
        report.check('no target power stops the checklist at step 1, before '
                     'even listing COM ports - later steps cannot explain '
                     'more than the first one already does',
                     'no power sensed' in result4
                     and 'COM ports Windows sees' not in result4, result4)
        find_board.check_power = lambda timeout=15: (3.30, 'fake: powered')

        # It names the stand-in it is actually on and the way off it. The
        # line it replaced said "--no-board or --simulated this run" for a
        # session that had been given neither - it fell back on its own -
        # and that was the whole answer on screen to "byter du till
        # debugproben".
        stood_in = toolmod.Toolbox(SimulatedSession()).call('link_diagnose', {})
        report.check('a session with no port names the stand-in it is on, '
                     'and the way off it',
                     'simulated board' in stood_in and '/board auto' in stood_in,
                     stood_in[:52])
        report.check('and never claims a flag the operator did not type',
                     '--simulated' not in stood_in, stood_in[:52])
        from coaxial_ollama.debug import NoBoard
        refused = toolmod.Toolbox(NoBoard()).call('link_diagnose', {})
        report.check('--no-board is the one case that did get the flag',
                     refused.startswith('--no-board this run')
                     and '/board auto' in refused, refused[:52])

        # Ungated: no --allow-writes, no --confirm, no --read-only. It never
        # touches the board's state or its flash, same reasoning as `docs`.
        ro = toolmod.Toolbox(SimulatedSession(), allow_code=False)
        report.check('link_diagnose works even with --read-only',
                     not str(ro.call('link_diagnose', {})).startswith('ERR'))
    finally:
        list_ports.comports = real_comports
        coaxial.connect = real_connect
        find_board.check_power = real_check_power


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
                                     call('link', op='stats'),
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

    box = toolmod.Toolbox(SimulatedSession(), shell=Shell(['python']), scope=Scope())

    def chat(turns, **kw):
        kw.setdefault('out', io.StringIO())
        return debug.Chat(ScriptedModel(turns), box, **kw)

    # Was /3, and SYSTEM sat one token under it. The two hardware facts that
    # stop the model inventing a coaxial cable cost 16 - see the comment on
    # debug.SYSTEM - and none of the rules they sit beside is fat that could
    # pay for them. Still a fraction, which is what this guards: 200 against
    # 556 is 36 %, and a prompt that reached half the runner's would be a
    # bench prompt turning into a test executive.
    report.check('the debug prompt is a fraction of the runner prompt',
                 debug.approx_tokens(debug.SYSTEM)
                 < debug.approx_tokens(runmod.SYSTEM) / 2.5,
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
         'content': 'analog_read: 128 samples @2000Hz\n' + 'x' * 500},
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
                 '128 samples' in sent[2]['content'] and 'xxxx' not in blob,
                 sent[2]['content'][:44])
    report.check('the recent turns are sent whole',
                 sent[-1]['content'] == 'new question')
    # Measured against the system message this turn actually built, not
    # against SYSTEM plus a slack number: the hints, the language line and the
    # model's own tag are all in there, and a magic +40 tips over every time
    # one of them gains a sentence.
    whole = (debug.approx_tokens(json.dumps(session.history))
             + session.tool_cost()
             + debug.approx_tokens(session.trim()[0]['content']))
    report.check('trimming is what the cost estimate measures',
                 session.context_cost() < whole,
                 '%d tok trimmed against %d whole'
                 % (session.context_cost(), whole))

    # `keep` counts messages, and the recent ones are sent whole - so six
    # recent messages is a small prompt right up until one of them is a build
    # log. What bounds it is num_ctx, because that is the number the daemon
    # allocates a KV cache for.
    stuffed = chat([], keep=6, tools='all')
    stuffed.history = [{'role': 'tool', 'tool_name': 'run_command',
                        'content': 'run_command: exit=1\n' + 'e' * 20000}
                       for _ in range(6)]
    stuffed.history.append({'role': 'user', 'content': 'did it build?'})
    report.check('a turn full of build logs still fits the window',
                 stuffed.context_cost() <= stuffed.prompt_budget(),
                 '%d tok of %d' % (stuffed.context_cost(),
                                   stuffed.prompt_budget()))
    report.check('and the question is what survives it',
                 stuffed.trim()[-1]['content'] == 'did it build?')
    report.check('/ctx shows the budget the turn is being fitted to',
                 'of %d' % stuffed.prompt_budget() in stuffed.command('/ctx'),
                 stuffed.command('/ctx'))

    # The client evicts models and shrinks windows in silence, because a
    # library that prints is one nobody can embed. The session it belongs to
    # is what says so.
    noisy = chat([{'role': 'assistant', 'content': 'ok'}])
    noisy.client.notes.append('out of memory: freed qwen2.5:14b (9.0 GB)')
    noisy.ask('read the ntc')
    report.check('what the client did to the machine reaches the operator',
                 'freed qwen2.5:14b' in noisy.out.getvalue()
                 and not noisy.client.notes, noisy.out.getvalue()[:60])

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

    # ---- link status tracks every call, not just /reconnect ----
    flaky_session = SimulatedSession()
    flaky_box = toolmod.Toolbox(flaky_session, shell=Shell(['python']),
                                scope=Scope())
    watch = debug.Chat(ScriptedModel([call('analog_read')]), flaky_box,
                       out=io.StringIO())
    flaky_session.board.broken = True
    down_answer = watch.ask('read everything')
    report.check('a cable pulled mid-turn turns the spinner red on its own',
                 watch.link_ok is False)
    report.check("the link-down answer carries link_diagnose's own finding, "
                 "not just the raw error",
                 'link is down, not answered' in down_answer
                 and len(down_answer.splitlines()) > 1, down_answer)

    flaky_session.board.broken = False
    watch.client.turns = [call('analog_read')]
    watch.ask('try again')
    report.check('and a call that succeeds turns it green again, no /reconnect',
                 watch.link_ok is True)

    # ---- asked again, a stale "it doesn't work" gets a fresh check ----
    # Measured on this bench: told the link was down, gemma4:12b answered
    # "again" with the same sentence from memory and no new tool call at all
    # - an honest answer, not a fabrication, but stale, and the operator had
    # to say "I plugged it back in" before it tried again. A zero-call answer
    # while link_ok is already False now gets a real link check first.
    recover_session = SimulatedSession()
    recover_box = toolmod.Toolbox(recover_session, shell=Shell(['python']),
                                  scope=Scope())
    recovered = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('analog_read')]), recover_box,
        out=io.StringIO())
    recovered.ask('tabellera')            # succeeds first - sets last_channels

    recover_session.board.broken = True
    hits = []
    real_recover_call = recover_box.call

    def recover_call(name, args):
        hits.append(name)
        if name == 'link':
            recover_session.board.broken = False  # reconnected, right now
        return real_recover_call(name, args)
    recover_box.call = recover_call

    recovered.client.turns = [
        call('analog_read'),
        {'role': 'assistant', 'content': 'Jag kan inte lasa av kortet just nu.'},
        call('analog_read')]
    recovered.ask('tabellera igen')
    report.check('a stale refusal triggers a real check instead of repeating',
                 hits == ['analog_read', 'link', 'analog_read'], hits)
    report.check('and the link is marked up again from that check alone',
                 recovered.link_ok is True)

    # ---- ...and if it is still down, the fresh check says so, not the model --
    stuck_recover_session = SimulatedSession()
    stuck_recover_box = toolmod.Toolbox(stuck_recover_session,
                                        shell=Shell(['python']), scope=Scope())
    stuck_recover_session.board.broken = True
    stuck_recovered = debug.Chat(ScriptedModel([
        call('analog_read'),
        {'role': 'assistant', 'content': 'Det gar fortfarande inte att lasa.'}]),
        stuck_recover_box, out=io.StringIO())
    answer = stuck_recovered.ask('tabellera igen')
    report.check('still down: the fresh check speaks, not the stale sentence',
                 answer.startswith('link is down, not answered:')
                 and 'fortfarande' not in answer, answer)

    # ---- a blank answer with no call at all is not taken at face value ----
    # Measured on this bench: the SECOND question asked (the first had
    # already succeeded) after the programmer was unplugged got a blank line
    # and nothing else - link_ok was still True right up to that turn
    # (nothing had failed yet to set it False), so the "stale refusal" check
    # above never triggered, and the model's own empty content, no call,
    # printed as silence with no error in sight.
    blank_session = SimulatedSession()
    blank_box = toolmod.Toolbox(blank_session, shell=Shell(['python']),
                                scope=Scope())
    blank = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('analog_read')]), blank_box,
        out=io.StringIO())
    blank.ask('ge mig en tabell')          # succeeds first - sets last_channels

    blank_session.board.broken = True
    blank.client.turns = [{'role': 'assistant', 'content': ''}]
    answer = blank.ask('ge mig en tabell igen')
    report.check('a blank answer with no call still gets a real check',
                 answer.startswith('link is down, not answered:')
                 and blank.link_ok is False, answer)

    # ---- ...and if the link turns out fine, the model gets a real turn ----
    blank_ok_session = SimulatedSession()
    blank_ok_box = toolmod.Toolbox(blank_ok_session, shell=Shell(['python']),
                                   scope=Scope())
    blank_ok = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('analog_read')]), blank_ok_box,
        out=io.StringIO())
    blank_ok.ask('ge mig en tabell')       # succeeds first - sets last_channels

    ok_hits = []
    real_blank_ok_call = blank_ok_box.call
    blank_ok_box.call = lambda name, args: (
        ok_hits.append(name), real_blank_ok_call(name, args))[1]
    blank_ok.client.turns = [
        {'role': 'assistant', 'content': ''}, call('analog_read')]
    blank_ok.ask('ge mig en tabell igen')
    report.check('a blank answer when the link is fine gets a real turn too',
                 ok_hits == ['link', 'analog_read'] and blank_ok.link_ok is True,
                 ok_hits)

    # ---- ...but an unrelated question with no call is never touched ----
    # Measured on this bench: link_ok can start False from the startup probe
    # alone, before any question at all - and without this gate, a plain "vad
    # ar 2+2" with no call and a perfectly good answer got discarded and
    # replaced with "link is down", because nothing had ever been read
    # successfully to make that check meaningful.
    untouched_session = SimulatedSession()
    untouched_box = toolmod.Toolbox(untouched_session, shell=Shell(['python']),
                                    scope=Scope())
    untouched_session.board.broken = True
    untouched = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content': '2+2 is 4.'}]), untouched_box,
        out=io.StringIO(), link_ok=False)
    answer = untouched.ask('what is 2+2?')
    report.check('an unrelated answer survives even if link_ok started False',
                 answer == '2+2 is 4.', answer)

    # ---- a blank FIRST answer is checked even with nothing to compare it to
    # Measured on this bench: "ge mig en lista over matvardena" as the very
    # first question of a session got a blank line and nothing else - the
    # gate above correctly stayed out of the way (self.last_channels was
    # None, nothing stale to protect against), but a blank answer is never a
    # legitimate answer to anything, so it needs the same check regardless.
    first_blank_session = SimulatedSession()
    first_blank_box = toolmod.Toolbox(first_blank_session,
                                      shell=Shell(['python']), scope=Scope())
    first_blank_hits = []
    real_first_blank_call = first_blank_box.call
    first_blank_box.call = lambda name, args: (
        first_blank_hits.append(name), real_first_blank_call(name, args))[1]
    first_blank_out = io.StringIO()
    first_blank = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content': ''},
        call('afe_power', action='on'), call('analog_read')]),
        first_blank_box, out=first_blank_out)
    first_blank.ask('ge mig en lista over matvardena')
    report.check('a blank first answer is nudged into a real reading',
                 first_blank_hits == ['link', 'afe_power', 'analog_read'],
                 first_blank_hits)
    # The probe itself is plumbing, not a reading - measured here, its raw
    # counters ("unit_id=1 t15_ticks=...") printed on screen for a question
    # that only asked for a list of ADC values, nothing to do with the link.
    # The control is the reading, not the afe_power line: a switch that did
    # what it was told is silenced now (see test_afe_trace), so `on=1` no
    # longer proves the other calls reached the screen.
    report.check('the probe itself is not traced - nobody asked for link stats',
                 'unit_id=' not in first_blank_out.getvalue()
                 and 'samples @' in first_blank_out.getvalue(),
                 first_blank_out.getvalue())

    # ---- ...and if the board really is down from the start, that is what
    # gets reported - not silence.
    first_blank_down_session = SimulatedSession()
    first_blank_down_box = toolmod.Toolbox(first_blank_down_session,
                                           shell=Shell(['python']),
                                           scope=Scope())
    first_blank_down_session.board.broken = True
    first_blank_down = debug.Chat(ScriptedModel(
        [{'role': 'assistant', 'content': ''}]), first_blank_down_box,
        out=io.StringIO())
    answer = first_blank_down.ask('ge mig en lista over matvardena')
    report.check('a blank first answer with the board actually down says so',
                 answer.startswith('link is down, not answered:'), answer)

    # ---- naming a tool instead of calling it gets nudged into calling it ----
    # Measured on this bench: asked for a table, gemma4:12b answered "jeg ma
    # utfore en `analog_read`" and stopped - it named the exact call needed
    # and never made it. There is no fact yet to substitute for a call that
    # never happened, so this nudges rather than overriding.
    narrate_session = SimulatedSession()
    narrate_box = toolmod.Toolbox(narrate_session, shell=Shell(['python']),
                                  scope=Scope())
    narrate_hits = []
    real_narrate_call = narrate_box.call
    narrate_box.call = lambda name, args: (
        narrate_hits.append(name), real_narrate_call(name, args))[1]
    narrate = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content': 'Jeg ma utfore en `analog_read`.'},
        call('afe_power', action='on'), call('analog_read')]), narrate_box,
        out=io.StringIO())
    narrate.ask('ger du mig en tabell over matvarden')
    report.check('a narrated tool name is nudged into an actual call',
                 narrate_hits == ['afe_power', 'analog_read'], narrate_hits)

    # ---- ...but nudging is not an open invitation to loop forever ----
    stuck_narrate_session = SimulatedSession()
    stuck_narrate_box = toolmod.Toolbox(stuck_narrate_session,
                                        shell=Shell(['python']), scope=Scope())
    stuck_narrate = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content': 'Jeg ma utfore en `analog_read`.'}] * 3),
        stuck_narrate_box, out=io.StringIO())
    answer = stuck_narrate.ask('ger du mig en tabell over matvarden')
    report.check('two nudges and no more: the narration is left to stand',
                 answer == 'Jeg ma utfore en `analog_read`.', answer)

    # ---- a failed read is not answered from an old context ----
    stale_session = SimulatedSession()
    stale_box = toolmod.Toolbox(stale_session, shell=Shell(['python']),
                                scope=Scope())
    stale = debug.Chat(ScriptedModel([
        call('analog_read', ch=['NTC']),
        {'role': 'assistant', 'content': 'NTC is 24.9 C.'}]), stale_box,
        out=io.StringIO())
    stale.ask('what is the temperature?')

    stale_session.board.broken = True
    stale.client.turns = [
        call('analog_read', ch=['NTC']),
        # A model is free to write this; the runner does not have to believe
        # it, because the failed read in its own history already says otherwise.
        {'role': 'assistant', 'content': 'NTC is still 24.9 C.'}]
    answer = stale.ask('what is the temperature now?')
    report.check('a link failure this turn overrides a stale-looking answer',
                 answer.startswith('link is down, not answered:')
                 and '24.9' not in answer, answer)
    report.check('and the spinner agrees',
                 stale.link_ok is False)

    # ---- an identical call repeated in one turn does not re-hit the board --
    spam_session = SimulatedSession()
    spam_box = toolmod.Toolbox(spam_session, shell=Shell(['python']),
                               scope=Scope())
    hits = []
    real_call = spam_box.call
    spam_box.call = lambda name, args: (hits.append(name), real_call(name, args))[1]
    spam = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('afe_power', action='on'),
        call('afe_power', action='on'), call('afe_power', action='on'),
        call('analog_read'),
        {'role': 'assistant', 'content': 'table above'}]), spam_box,
        out=io.StringIO())
    spam.ask('tabulate everything')
    report.check('four identical afe_power calls reach the board once',
                 hits.count('afe_power') == 1, hits)
    report.check('a repeatable tool still reaches the board every time',
                 hits.count('analog_read') == 1, hits)
    report.check('the model is told plainly rather than shown the same line',
                 'unchanged this turn' in json.dumps(spam.history))

    # ---- ...but a dedup cannot launder a failure back into a success ------
    stuck_session = SimulatedSession()
    stuck_box = toolmod.Toolbox(stuck_session, shell=Shell(['python']),
                                scope=Scope())
    stuck_session.board.broken = True
    stuck = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content': '',
         'tool_calls': [{'function': {'name': 'analog_read', 'arguments': {}}}]},
        {'role': 'assistant', 'content': '',
         'tool_calls': [{'function': {'name': 'analog_read', 'arguments': {}}}]},
        {'role': 'assistant', 'content': 'reading holds at 24.9 C'}]), stuck_box,
        out=io.StringIO())
    answer = stuck.ask('read it again')
    report.check('a repeated failing call keeps failing, not clean the second time',
                 answer.startswith('link is down, not answered:')
                 and stuck.link_ok is False, answer)

    # ---- a call outside the plain question is not deduped across turns ----
    fresh = debug.Chat(ScriptedModel([call('afe_power', action='on')]), box,
                       out=io.StringIO())
    fresh.ask('turn the front end on')
    fresh.client.turns = [call('afe_power', action='on')]
    hits2 = []
    real2 = fresh.toolbox.call
    fresh.toolbox.call = lambda name, args: (hits2.append(name), real2(name, args))[1]
    fresh.ask('is it still on?')
    report.check('dedup is per question, not across the whole session',
                 hits2 == ['afe_power'], hits2)

    # ---- a table read is not typed out again as the answer -----------------
    report.check('the header row is not mistaken for a channel called smp',
                 'smp' not in replies.READING_ROW.findall(
                     '64 samples @2000Hz\n0  PhaseU  diff   1427.1  +0.1437V'))

    retype_session = SimulatedSession()
    retype_box = toolmod.Toolbox(retype_session, shell=Shell(['python']),
                                 scope=Scope())
    retyped = debug.Chat(ScriptedModel([
        call('afe_power', action='on'),
        call('analog_read', ch=['phA']),          # the bad guess from FINDINGS
        call('analog_read'),
        {'role': 'assistant', 'content':
            'PhaseU: +0.1398V, PhaseV: -0.8226V, '
            'NTC: 2.0567V (38.85C), DCbus: 1.1197V (26.518V)'}]), retype_box,
        out=io.StringIO())
    answer = retyped.ask('tabellera alla AFE-kanaler')
    report.check('a full retype of the table just shown is silenced, not '
                 'echoed back', answer == '', answer)

    # Measured on this bench: cut off by --words before naming every channel,
    # a markdown-table retype used to slip past the all-channels-present check
    # entirely - the very shape SYSTEM already forbids, printed anyway because
    # it never finished. The shape alone is now enough to catch it.
    truncated_session = SimulatedSession()
    truncated_box = toolmod.Toolbox(truncated_session, shell=Shell(['python']),
                                    scope=Scope())
    truncated = debug.Chat(ScriptedModel([
        call('afe_power', action='on'),
        call('analog_read'),
        {'role': 'assistant', 'content':
            '| Kanal | Typ | Raw | Volt |\n'
            '| :--- | :--- | :--- | :--- |\n'
            '| 0 | PhaseU | 911.6 | +0.0918V |\n'
            '| 1 | PhaseV | -8633.4 | -0.8695V |'}]),   # cut off: no NTC, DCbus
        truncated_box, out=io.StringIO())
    answer = truncated.ask('kan du tabellera alla analoga matvarden')
    report.check('a markdown table is silenced even cut off before every '
                 'channel is named', answer == '', answer)

    insight_session = SimulatedSession()
    insight_box = toolmod.Toolbox(insight_session, shell=Shell(['python']),
                                  scope=Scope())
    insight = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('analog_read'),
        {'role': 'assistant', 'content':
            'NTC is running warm at 38.9 C; DCbus looks nominal.'}]),
        insight_box, out=io.StringIO())
    answer = insight.ask('does anything look off?')
    report.check('a real finding that names a couple of channels survives',
                 'running warm' in answer, answer)

    one_session = SimulatedSession()
    one_box = toolmod.Toolbox(one_session, shell=Shell(['python']), scope=Scope())
    one = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('analog_read', ch=['ntc']),
        {'role': 'assistant', 'content': 'NTC is 24.9 C.'}]), one_box,
        out=io.StringIO())
    report.check('a single-channel question keeps its single-channel answer',
                 one.ask('what is the temperature?') == 'NTC is 24.9 C.')

    # ---- a turn that skips the tool call entirely is not taken on faith ----
    skip_session = SimulatedSession()
    skip_box = toolmod.Toolbox(skip_session, shell=Shell(['python']),
                               scope=Scope())
    skip = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('analog_read'),
        {'role': 'assistant', 'content':
            'PhaseU: +0.1398V, PhaseV: -0.8226V, '
            'NTC: 2.0567V (38.85C), DCbus: 1.1197V (26.518V)'}]), skip_box,
        out=io.StringIO())
    skip.ask('tabellera ADC-värdena')
    skip_hits = []
    real_skip_call = skip_box.call
    skip_box.call = lambda name, args: (
        skip_hits.append(name), real_skip_call(name, args))[1]
    skip.client.turns = [
        {'role': 'assistant', 'content':
            # A rewrite, not a copy - a model confabulating a fresh reading
            # does not retype the old numbers verbatim, which is exactly why
            # the verbatim-restate check above cannot be the thing that
            # catches this.
            'PhaseU: +0.1415V, PhaseV: -0.8232V, '
            'NTC: 2.0726V (39.45C), DCbus: 1.1195V (26.511V)'},
        call('analog_read')]     # the nudge, taken - a real reading follows
    skip.ask('tabellera ADC-värdena igen')
    report.check('a fresh-looking table with no tool call this turn is '
                 'nudged into a real one',
                 skip_hits == ['link', 'analog_read'], skip_hits)

    # ...and when the board really is unreachable, that refusal says so -
    # measured here: the generic "ask again" line was itself the complaint,
    # on a bench where the honest answer was "not connected or not powered".
    unplugged_session = SimulatedSession()
    unplugged_box = toolmod.Toolbox(unplugged_session, shell=Shell(['python']),
                                    scope=Scope())
    unplugged = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('analog_read'),
        {'role': 'assistant', 'content':
            'PhaseU: +0.1398V, PhaseV: -0.8226V, '
            'NTC: 2.0567V (38.85C), DCbus: 1.1197V (26.518V)'}]), unplugged_box,
        out=io.StringIO())
    unplugged.ask('tabellera ADC-värdena')
    unplugged_session.board.broken = True
    unplugged.client.turns = [{'role': 'assistant', 'content':
        'PhaseU: +0.1415V, PhaseV: -0.8232V, '
        'NTC: 2.0726V (39.45C), DCbus: 1.1195V (26.511V)'}]
    answer = unplugged.ask('tabellera ADC-värdena igen')
    report.check('a skipped call refuses correctly when the board is really gone',
                 answer.startswith('link is down, not answered:'), answer)

    # ---- the whole reported sequence: works, unplugged, replugged, works
    # again - not stuck repeating "ask again" forever. Measured on this
    # bench: works, unplug, "link is down" (correct) - replug, and it kept
    # answering "no reading taken this turn - ask again" no matter how many
    # times the same question was repeated, because nothing ever prompted the
    # model to actually read again once the link was confirmed back up.
    saga_session = SimulatedSession()
    saga_box = toolmod.Toolbox(saga_session, shell=Shell(['python']),
                               scope=Scope())
    saga_hits = []
    real_saga_call = saga_box.call
    saga_box.call = lambda name, args: (
        saga_hits.append(name), real_saga_call(name, args))[1]

    saga = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('analog_read')]), saga_box,
        out=io.StringIO())
    saga.ask('ger du mig en tabell over de analoga matvardena')  # 1) works

    saga_session.board.broken = True
    saga.client.turns = [call('link', op='stats')]     # 2) it checks, fails
    answer = saga.ask('ger du mig en tabell over de analoga matvardena')
    report.check('unplugged: reported plainly, not guessed',
                 answer.startswith('link is down, not answered:'), answer)

    saga_session.board.broken = False                  # 3) replugged
    saga.client.turns = [
        {'role': 'assistant', 'content': 'Kommunikationen fungerar '
                                         'fortfarande inte.'},
        call('analog_read')]
    saga.ask('ger du mig en tabell over de analoga matvardena')
    report.check('replugged: measures again instead of looping "ask again"',
                 saga_hits[-2:] == ['link', 'analog_read'], saga_hits)

    # ---- a pulled cable can leave the cached handle dead - session.reset()
    # is what actually recovers it, not just retrying. Measured directly
    # against real hardware: unplugging left Session.board's cached serial
    # handle permanently invalid ("Attempting to use a port that is not
    # open"), because a USB VCP re-enumerates on replug rather than reviving
    # the same handle - retrying on the same cached board failed forever,
    # no matter how many times, until something called session.reset(). The
    # saga test above never exercises this: it flips `broken` back to False
    # by hand, which a real replug does not do to a dead handle by itself.
    dead_session = SimulatedSession()
    dead_box = toolmod.Toolbox(dead_session, shell=Shell(['python']),
                               scope=Scope())
    dead_hits = []
    real_dead_call = dead_box.call
    dead_box.call = lambda name, args: (
        dead_hits.append(name), real_dead_call(name, args))[1]

    dead = debug.Chat(ScriptedModel([
        call('afe_power', action='on'), call('analog_read')]), dead_box,
        out=io.StringIO())
    dead.ask('ger du mig en tabell over de analoga matvardena')  # 1) works

    dead_session.board.dead_handle = True                # 2) cable pulled
    dead.client.turns = [call('link', op='stats')]
    answer = dead.ask('ger du mig en tabell over de analoga matvardena')
    report.check('a dead cached handle is reported plainly, same as any '
                 'other lost link',
                 answer.startswith('link is down, not answered:'), answer)
    report.check("debug.py itself calls session.reset() on that failure - "
                 'not left for a human to notice and type /reconnect',
                 dead_session.resets >= 1)
    report.check('and the reset is what actually cleared the dead handle, '
                 'not the cable coming back on its own',
                 dead_session.board.dead_handle is False)

    # 3) no further "replug" step needed - the automatic reset already
    # fixed it, so the very next question measures again on its own.
    dead.client.turns = [call('link', op='stats'), call('analog_read')]
    dead.ask('ger du mig en tabell over de analoga matvardena')
    report.check('so the next question actually reaches analog_read, not '
                 'stuck retrying a dead handle forever',
                 dead_hits[-2:] == ['link', 'analog_read'], dead_hits)

    # ---- a multi-row result prints as rows, not one squashed line ----
    grid = io.StringIO()
    shown = debug.Chat(ScriptedModel([call('board_info')]), box, out=grid)
    shown.ask('what channels does this board have?')
    printed = grid.getvalue()
    report.check('every row of a table lands on its own line',
                 ' | ' not in printed and printed.count(chr(10)) >= len(CHANNELS),
                 printed[:120].replace(chr(10), ' / '))

    # ---- the trace is the table, not a header that clips it apart ----
    long_args = io.StringIO()
    traced = debug.Chat(ScriptedModel([
        call('analog_read', samples=100,
             ch=['dc_bus', 'ntc', 'phase_a', 'phase_b', 'phase_c'])]),
        box, out=long_args)
    traced.ask('read everything')
    printed = long_args.getvalue()
    report.check('a call with long arguments does not print a header at all',
                 'analog_read' not in printed.split(chr(10))[0]
                 and 'more characters cut' not in printed,
                 printed.splitlines()[:2])
    report.check('the result rows print whole, un-clipped by the call header',
                 'samples @' in printed or 'ERR' in printed, printed[:80])

    # ---- a call the model wrote as text is still a call ----
    pasted = ('CallCheckFunction' + chr(10)
              + '{"name": "analog_read", "arguments": {"ch": ["NTC"],'
                ' "rate_hz": 100, "samples": 10}}' + chr(10)
              + 'CallCheckFunction' + chr(10)
              + '{"name": "afe_power", "arguments": {"action": "read"}}')
    calls, rest = replies.salvage_calls(pasted)
    report.check('two calls written as text are both salvaged',
                 [c['function']['name'] for c in calls]
                 == ['analog_read', 'afe_power'] and rest == '',
                 '%d calls' % len(calls))
    report.check('and their nested arguments survive the parse',
                 calls[0]['function']['arguments'].get('samples') == 10,
                 str(calls[0]['function']['arguments']))

    one, rest = replies.salvage_calls(
        '<tool_call>{"name":"analog_read","arguments":{"ch":["NTC"]}}</tool_call>')
    report.check('a single tagged call still works',
                 len(one) == 1 and rest == '')

    for prose in ('The NTC reads 27.4 C.',
                  'I would call {"name": "docs"} but the front end is off.',
                  'Set it with board.afe.on() {no tool needed}.'):
        calls, rest = replies.salvage_calls(prose)
        report.check('an answer is never turned into a board command',
                     not calls and rest == prose, prose[:40])

    report.check('a stray closing tag is not an answer on its own',
                 replies.salvage_calls('done.</tool_call>') == ([], 'done.'))

    spoken = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content':
            'CallFunction' + chr(10)
            + '{"name": "afe_power", "arguments": {"action": "read"}}'},
        {'role': 'assistant', 'content': 'The front end is on.'}]),
        box, out=io.StringIO())
    answer = spoken.ask('is the front end on?')
    report.check('the prompt runs it rather than printing the JSON',
                 answer == 'The front end is on.'
                 and any(m.get('role') == 'tool' for m in spoken.history),
                 answer[:60])

    attached = debug.attach(['coaxial_ollama/plans/bringup.yaml'], 300)
    report.check('an attached file is clipped, and says so',
                 'characters cut' in attached and '300 attached' in attached,
                 attached.splitlines()[0][:60])

    # ---- a crashed model backend loses one turn, not the session ----------
    # Reported live: llama-server hit std::bad_alloc mid-conversation, ollama
    # answered 500, and the whole --repl process died with it - a single
    # OllamaError wasn't in repl()'s per-turn catch, so it rode all the way
    # out to main()'s handler. The board gets a reconnect story; the model
    # backend deserves the same one, since ollama respawns llama-server on
    # the next request same as the board answers again once plugged back in.
    from coaxial_ollama.client import OllamaError

    class Flaky:
        model = 'flaky'

        def __init__(self):
            self.calls = 0

        def chat(self, messages, tools=None):
            self.calls += 1
            if self.calls == 1:
                raise OllamaError('ollama: /api/chat 500: boom')
            return {'role': 'assistant', 'content': 'still here'}

        def usage(self):
            return {'calls': self.calls, 'prompt_tokens': 0, 'eval_tokens': 0}

    crash_out = io.StringIO()
    crashy = debug.Chat(Flaky(), box, out=crash_out)
    old_stdin, old_stdout = sys.stdin, sys.stdout
    sys.stdin = io.StringIO('first question\nsecond question\n')
    sys.stdout = crash_out
    try:
        debug.repl(crashy, hold=True)
    finally:
        sys.stdin, sys.stdout = old_stdin, old_stdout
    transcript = crash_out.getvalue()
    report.check('the crash is reported, not swallowed',
                 'OllamaError' in transcript and 'boom' in transcript)
    report.check('the loop survives it and answers the next question',
                 'still here' in transcript, transcript[-200:])
    report.check('and every question starts fresh - history is cleared '
                 'after each one, growing prompts being what feeds the '
                 'crash just recovered from',
                 crashy.history == [])

    # ---- confirmed on the ordinary path too, not just after a crash -------
    clean = chat([call('afe_power', action='on')])
    clean_out = io.StringIO()
    old_stdin, old_stdout = sys.stdin, sys.stdout
    sys.stdin = io.StringIO('turn one\n/tools\nturn two\n')
    sys.stdout = clean_out
    try:
        debug.repl(clean, hold=True)
    finally:
        sys.stdin, sys.stdout = old_stdin, old_stdout
    report.check('a clean answer clears history too, not only a recovered '
                 'crash, and a slash command in between (/tools) does not '
                 'stop it clearing on the question right after',
                 clean.history == [])


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


# ---- a crashed model runner is not the operator's problem ------------------

def test_runner_crash_retry(report):
    """Measured repeatedly on this bench: llama-server dies with
    std::bad_alloc mid-session and ollama answers 500. The daemon respawns
    it, so asking again works - which used to be the operator's job."""
    from coaxial_ollama import client as clientmod
    from coaxial_ollama.client import Ollama, OllamaError

    crash = OllamaError('/api/chat 500: {"error":"model runner has '
                        'unexpectedly stopped, this may be due to resource '
                        'limitations or an internal error"}')
    report.check('a crashed runner is told apart from a refused request',
                 clientmod._runner_crashed(crash)
                 and not clientmod._runner_crashed(
                     OllamaError('/api/chat 400: invalid tool schema')))

    # No real sleeping in a test suite that runs on every change.
    slept, real_sleep = [], clientmod.time.sleep
    clientmod.time.sleep = slept.append
    try:
        talker = Ollama('gemma4:12b')
        attempts = []

        def flaky(path, payload):
            attempts.append(path)
            if len(attempts) < 3:
                raise crash
            return {'message': {'role': 'assistant', 'content': 'recovered'},
                    'prompt_eval_count': 5, 'eval_count': 2}
        talker._post = flaky
        message = talker.chat([{'role': 'user', 'content': 'hi'}])
        report.check('a crashed runner is retried until it comes back, in '
                     'silence', message['content'] == 'recovered'
                     and len(attempts) == 3, attempts)
        report.check('and it waits between tries rather than collecting the '
                     'same 500 instantly', len(slept) == 2 and all(slept),
                     slept)
        report.check('the token meter counts the reply that arrived, not the '
                     'attempts', talker.calls == 1 and talker.prompt_tokens == 5)

        # A machine genuinely out of memory must still fail, not loop.
        forever = Ollama('gemma4:12b')
        tries = []

        def always_crash(path, payload):
            tries.append(path)
            raise crash
        forever._post = always_crash
        try:
            forever.chat([{'role': 'user', 'content': 'hi'}])
            report.check('a runner that never comes back still raises', False)
        except OllamaError:
            report.check('a runner that never comes back still raises, '
                         'bounded by RUNNER_RETRIES',
                         len(tries) == clientmod.RUNNER_RETRIES + 1, tries)

        # A refused request is a mistake in the request; asking again just
        # makes the same mistake twice.
        picky = Ollama('gemma4:12b')
        refusals = []

        def refuse(path, payload):
            refusals.append(path)
            raise OllamaError('/api/chat 400: invalid tool schema')
        picky._post = refuse
        try:
            picky.chat([{'role': 'user', 'content': 'hi'}])
            report.check('a refused request is not retried', False)
        except OllamaError:
            report.check('a refused request is not retried', len(refusals) == 1)
    finally:
        clientmod.time.sleep = real_sleep


# ---- one screen, one language ---------------------------------------------

def test_screen_language(report):
    """A Swedish question answered in Swedish, under an English warning this
    project wrote itself, is one screen in two languages."""
    import re
    from coaxial_ollama import debug, language

    banner = ('AFE OFF - the ADC reference is unpowered. These are the codes '
              'the converter returned, not measurements: every channel sits '
              'near mid-scale, and the degC and volts below are arithmetic '
              'on that - not a temperature, not a bus voltage. Call '
              'afe_power on to measure.')
    turned = language.localise(banner, 'Swedish')
    report.check('host prose turns into the session language',
                 'AFE AV' in turned and 'unpowered' not in turned,
                 turned[:48])
    report.check('a language with no table is left in English',
                 language.localise(banner, 'German') == banner
                 and language.localise(banner, None) == banner)

    # The values are the board's, and they come back exactly as they went in.
    checklist = ('1. Target power (ST-Link/SWD): 3.27V - powered, cable '
                 'seated.\n2. COM ports Windows sees: COM4\n'
                 '3. Configured port COM4: present.')
    swedish = language.localise(checklist, 'Swedish')
    report.check('the numbers and port names survive being translated',
                 '3.27V' in swedish and swedish.count('COM4') == 2
                 and 'Målspänning' in swedish, swedish.splitlines()[0])
    report.check('and the step numbering is still the step numbering',
                 [line.split('.')[0] for line in swedish.splitlines()]
                 == ['1', '2', '3'])

    table = ('  4  NTC     SE    32768.0  +1.6500V 25.00C\n'
             '  5  DCbus   SE    32768.0  +1.6500V 39.075V bus')
    report.check('a reading is not touched - no channel name is translated',
                 language.localise(table, 'Swedish') == table)

    # Two signals, and they are not the same one. The greeting has no question
    # to read, so it takes the machine's locale; everything after it follows
    # the question. `screen` is the fallback for the case detect() abstains,
    # and it is None here on purpose - a suite that read the Windows locale
    # would pass on one machine and fail on the next.
    box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    talk = debug.Chat(ScriptedModel([], model='gemma4:12b'), box,
                      out=io.StringIO())
    report.check('a test session starts in no language, not the machine one',
                 talk.language is None and talk.screen_language() is None)
    local = debug.Chat(ScriptedModel([], model='gemma4:12b'), box,
                       out=io.StringIO(), session_language='Swedish')
    report.check('a real session starts in the locale build() read',
                 local.screen_language() == 'Swedish')

    # The locale is where it starts, not where it is stuck: a question in
    # another language moves it, and so does asking for one.
    local.history = [{'role': 'user', 'content': 'read all the analog channels'}]
    local.prompt_history = ['read all the analog channels']
    local.trim()
    report.check('a question in another language moves it',
                 local.language == 'English', local.language)
    local.prompt_history.append('svara på svenska')
    local.history = [{'role': 'user', 'content': 'svara på svenska'}]
    local.trim()
    report.check('and asking for one outright moves it too',
                 local.language == 'Swedish', local.language)

    # Every locale this module can name must have a greeting, or a machine
    # set to it opens in English for no reason anyone can see.
    missing = [name for name in set(language._LOCALE_CODES.values())
               if name not in language.GREETINGS]
    report.check('every locale it recognises has a greeting',
                 not missing, ', '.join(sorted(missing)) or 'all present')
    report.check('a console that cannot encode it falls back to English',
                 language.greeting('m', 'Japanese', 'cp1252')
                 == language.greeting('m', 'English')
                 and language.greeting('m', 'Japanese', 'utf-8')
                 != language.greeting('m', 'English'))

    # Every English key must exist verbatim in the source, or a call site has
    # moved on and its translation is dead text nothing will ever match.
    sources = []
    for name in ('coaxial_ollama/debug.py', 'coaxial_ollama/tools.py',
                 'coaxial_mcp/tools.py', 'coaxial_mcp/render.py'):
        with io.open(os.path.join(os.path.dirname(os.path.dirname(
                os.path.abspath(__file__))), name), encoding='utf-8') as handle:
            sources.append(handle.read())
    # Quotes stripped before comparing: the source splits these strings
    # across lines, so the literal run is broken by a `' '` at every wrap.
    joined = ' '.join(_flat(text) for text in sources)
    orphans = [key for key in language.PHRASES['Swedish']
               if _flat(_unformat(key)) not in joined]
    report.check('every translation still matches a line in the source',
                 not orphans, '; '.join(o[:40] for o in orphans) or 'all matched')


def _flat(text):
    """Text with Python's string quotes and line breaks taken out, so a
    literal split across three source lines still reads as one run."""
    for mark in (chr(39), chr(34), chr(92)):      # quotes and a continuation
        text = text.replace(mark, ' ')
    return ' '.join(text.split())


def _unformat(template):
    """The longest literal run of a template - what to look for in the source,
    since the source may split a string across lines and %-specs sit where the
    values go."""
    import re
    return max(re.split(r'%(?:\.\d+)?[a-z]', template), key=len)


# ---- what reaches the screen, and how it reads -----------------------------

def test_fallback(report):
    """No cable is not a failing test suite - it is a different board.

    `open_session` probes the port with the same Modbus round trip a tool
    call makes, and hands back the stand-in when nothing answers. What it
    must never do is leave the caller unable to tell which it got: every
    suite and the prompt itself print it, and that is what these check.
    """
    from coaxial_mcp.session import open_session
    from coaxial_ollama import debug
    from coaxial_ollama import spinner as spin

    session, found = open_session(simulated=True)
    report.check('forced simulated skips the search entirely',
                 type(session).__name__ == 'SimulatedSession'
                 and not found.real, type(session).__name__)
    report.check('and is labelled Simulated, not by a port it never opened',
                 found.label == 'Simulated', found.label)

    # The label names the path, not just the port: a reading over the bench
    # cable and one over the field bus are not the same measurement, and
    # which it was has to be on screen rather than inferred from a COM
    # number. The probe is told apart by its USB VID - measured here, an
    # STLINK-V3SET enumerates 0483:374F - so nothing has to be opened to
    # know which port is the debugger.
    from coaxial_mcp import session as sessionmod
    for real, port, kind, want in ((True, 'COM3', 'probe', 'JTAG and COM3'),
                                   (True, 'COM5', 'serial', 'RS485 at COM5'),
                                   (False, None, None, 'Simulated')):
        report.check('label: %s' % want,
                     sessionmod._label(real, port, kind) == want,
                     sessionmod._label(real, port, kind))
    report.check('and says "simulated" where a firmware version goes',
                 session.board.version_info['firmware'] == 'simulated',
                 session.board.version_info['firmware'])

    # PB2 is the AFE switch, not a spare pin. Measured: writing 0 across
    # GPIOB left the stand-in answering `on=1` to afe_power one call later,
    # because the pin map and the switch were two dictionaries - the one
    # place invariant 9 could be broken by a stand-in with nobody noticing.
    session.board.gpio.test_mode(True)
    session.board.afe.enable()
    session.board.gpio.port_write('B', 0xFFFF, 0)
    report.check('clearing PB2 on the stand-in turns its AFE off',
                 not session.board.afe.state()['on'])
    report.check('and PE15 follows it inversely, as the real one does',
                 session.board.gpio.pin_read('E', 15) is True)
    session.board.gpio.pin_write('B', 2, True)
    report.check('setting PB2 turns it back on',
                 session.board.afe.state()['on']
                 and session.board.gpio.port_read('B') & (1 << 2))

    # link_diagnose's step 4 opened the port a second time to ask whether
    # the board answers - while the session held it open. Measured live,
    # with the link up: "3. Configured port COM4: present." followed by
    # "4. Board answers on COM4 right now: no", a false statement about
    # live hardware produced by the diagnostic itself. The session's own
    # handle is asked first now, and only a session with none falls through
    # to the second open.
    class Held:
        """A session holding an open link, shaped like coaxial_mcp.Session."""
        port, baud, unit = 'COM_TEST', 115200, 1

        def __init__(self, board):
            self._board = board

    live = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    report.check('a session with no board cached falls through to the probe',
                 toolmod._open_link_answers(Held(None)) is False)
    report.check('and one holding an answering link says so without opening '
                 'anything',
                 toolmod._open_link_answers(Held(SimulatedSession().board))
                 is True)

    class Dead:
        def echo(self, data):
            raise ConnectError('cable pulled')

    class DeadBoard:
        link = Dead()

    report.check('a held link that has since died is not counted as up',
                 toolmod._open_link_answers(Held(DeadBoard())) is False)

    # Steps 1 and 2 stubbed: the first shells out to STM32_Programmer_CLI
    # with a 15s timeout, and the second asks Windows what it has plugged
    # in - neither is what step 4 is being checked for, and both make the
    # answer depend on the desk the suite runs on.
    live.session = Held(SimulatedSession().board)
    power, ports = toolmod.find_board.check_power, toolmod.find_board.list_ports
    toolmod.find_board.check_power = lambda *a, **k: (3.27, 'stubbed')
    toolmod.find_board.list_ports = lambda: ['COM_TEST']
    try:
        checklist = live.call('link_diagnose', {})
    finally:
        toolmod.find_board.check_power = power
        toolmod.find_board.list_ports = ports
    report.check('so step 4 reports the link that is actually up',
                 'right now: yes' in checklist,
                 checklist.splitlines()[-1][:52])

    # /board: what the tools talk to, swapped without a restart. Measured at
    # the prompt: asked "byt till en simulerad hardvara", gemma4:12b answered
    # that it could not and was configured for the physical board - true
    # about itself, a dead end for the operator. The swap is the host's.
    swap = debug.Chat.__new__(debug.Chat)
    swap.toolbox = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    swap.origin, swap.link_ok, swap.last_channels = ('Simulated', False), False, {'ntc'}
    said = swap.command('/board simulated')
    report.check('/board simulated takes the stand-in',
                 said == 'board: Simulated'
                 and type(swap.toolbox.session).__name__ == 'SimulatedSession',
                 said)
    report.check('and the prompt tag is rebuilt from the same origin',
                 swap.origin == ('Simulated', False), str(swap.origin))
    report.check('and a table from the board just left is forgotten',
                 swap.last_channels is None, str(swap.last_channels))
    report.check('/board with no argument says what it is on',
                 swap.command('/board').startswith('board: Simulated'),
                 swap.command('/board'))

    # An order to swap the board is the host's to carry out. Measured three
    # times on the same session, and it never once changed board: it refused
    # ("Jag kan inte byta till simulerad hardvara"), then diagnosed the link,
    # then read seven channels and wrote nothing. The operator was giving an
    # order, not asking a question, and the state is the host's either way -
    # the same argument as language.bare_switch.
    for question, want in (
            ('byt till debugproben', 'auto'),
            ('byter du till debugproben', 'auto'),
            ('byt till en simulerad enhet', 'simulated'),
            ('byt till simulerad hårdvara', 'simulated'),
            ('växla till COM4', 'COM4'),
            ('switch to the real board', 'auto'),
            ('byt till RS485', 'rs485'),
            ('byt till fältbussen', 'rs485'),
            # Every one of these lost the order to a single unlisted noun
            # while this required all the words to be known: 'enhet', then
            # 'hardvara', then 'lage'. The rule names what disqualifies an
            # order instead, so a noun nobody thought of costs nothing.
            ('byter du till simulerat läge', 'simulated'),
            ('byt till simulerat läge', 'simulated'),
            ('använd det simulerade kortet', 'simulated'),
            ('koppla om till proben', 'auto'),
            ('byt till den simulerade grejen', 'simulated'),
            # An interrogative disqualifies it.
            ('vad är debugproben?', None),
            ('vilket läge är du i?', None),
            ('vet du om kortet svarar?', None),
            # ...and so does a second request the host cannot carry out.
            ('byt till simulerat läge och läs NTC:n', None),
            ('byt till proben och mät NTC:n', None),
            ('byt språk till svenska', None),
            ('läs NTC:n och DC-länken', None),
            ('beskriv hårdvaran för en novis', None),
            # Names a board, orders nothing.
            ('debugproben är inte inkopplad', None)):
        got = debug.board_switch(question)
        report.check('board order: %s' % question[:34], got == want, str(got))

    # A pipe is not a console. Measured: `printf "byter du till simulerat
    # lage" | dbg --repl` arrived as `lÃ¤ge` under cp1252, which splits into
    # `lã` and `ge` - and `ge` disqualifies a board order as a second
    # request. The order went to the model, which refused it. stdin gets the
    # same treatment as the two outputs now: UTF-8 when it is not a tty.
    mangled = 'byter du till simulerat läge'.encode('utf-8').decode('cp1252')
    report.check('the mangling really does hide the order',
                 debug.board_switch(mangled) is None, mangled[-12:])
    report.check('so a pipe is decoded as UTF-8, like the outputs are',
                 debug._printable(_NotATty()) is not None
                 and _NotATty.asked == {'encoding': 'utf-8',
                                        'errors': 'replace'},
                 str(_NotATty.asked))

    # ...and it reaches the swap, without a model turn.
    ordered = debug.Chat.__new__(debug.Chat)
    ordered.toolbox = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    ordered.origin, ordered.link_ok = ('Simulated', False), False
    ordered.last_channels, ordered.language = None, None
    ordered.io_log = debug.IOLog(enabled=False)
    ordered.out = io.StringIO()
    ordered.client = ScriptedModel([])
    said = ordered.ask('byt till en simulerad enhet')
    report.check('an order to swap the board never reaches the model',
                 not ordered.client.prompts, '%d turns'
                 % len(ordered.client.prompts))
    report.check('and answers with the board it landed on',
                 said == 'board: Simulated', said)
    # Ordered a real board and found none. Patched, not left to whatever is
    # plugged into this bench: the first version of this check passed only
    # while the board happened to be silent, and started failing the moment
    # it answered again.
    import coaxial_mcp.session as sessionmod
    ordered.language = None
    was = sessionmod.open_session
    try:
        sessionmod.open_session = lambda *a, **kw: (
            SimulatedSession(),
            sessionmod.Origin(False, None, 115200, None, 'Simulated'))
        said = ordered.ask('byt till debugproben')
    finally:
        sessionmod.open_session = was
    report.check('a search that found nothing says so, not just where it '
                 'ended up', 'nothing answered' in said, said)

    # An order that cannot be carried out must not cost the board that was
    # working. Measured: "byt till debugproben" twice in a row on a bench
    # whose board had gone silent, both times "inget svarade" - and had the
    # session been on a live probe, the first of those would have dropped it
    # for a stand-in.
    import coaxial_mcp.session as sessionmod

    class Live:
        """A session that is already on a real board."""
        port, baud, unit = 'COM4', 115200, 1
        closed = False

        def close(self):
            self.closed = True

        def reset(self):
            pass

    real = sessionmod.Origin(True, 'COM4', 115200, 'probe', 'JTAG and COM4')
    fake = sessionmod.Origin(False, None, 115200, None, 'Simulated')

    def patched(result):
        def factory(*a, **kw):
            return (SimulatedSession() if not result.real else Live()), result
        return factory

    keeper = debug.Chat.__new__(debug.Chat)
    held = Live()
    keeper.toolbox = toolmod.Toolbox(held, scope=Scope())
    keeper.origin, keeper.link_ok = ('JTAG and COM4', True), True
    keeper.last_channels = {'ntc'}

    original = sessionmod.open_session
    try:
        sessionmod.open_session = patched(fake)
        said = keeper.command('/board rs485')
        report.check('a search that found nothing keeps the working board',
                     keeper.toolbox.session is held and not held.closed,
                     type(keeper.toolbox.session).__name__)
        report.check('and leaves the prompt tag alone',
                     keeper.origin == ('JTAG and COM4', True),
                     str(keeper.origin))
        report.check('and says what it tried, not just where it ended up',
                     said.startswith('board: nothing answered on')
                     and 'JTAG and COM4' in said, said[:56])
        report.check('and does not wipe a reading from a board it kept',
                     keeper.last_channels == {'ntc'},
                     str(keeper.last_channels))

        # Ordering the stand-in never searches, so it always lands.
        report.check('an order for the stand-in always lands',
                     keeper.command('/board simulated') == 'board: Simulated',
                     str(keeper.origin))
        report.check('and that one does swap, and closes what it left',
                     keeper.origin == ('Simulated', False) and held.closed,
                     str(keeper.origin))

        sessionmod.open_session = patched(real)
        report.check('a search that found a board swaps to it',
                     keeper.command('/board auto') == 'board: JTAG and COM4'
                     and keeper.origin == ('JTAG and COM4', True),
                     str(keeper.origin))
        report.check('and forgets a table taken from the board it left',
                     keeper.last_channels is None, str(keeper.last_channels))
    finally:
        sessionmod.open_session = original

    # /model: same idea one layer up. No weights are loaded here - every path
    # below either refuses or is a no-op, which is the whole logic.
    from coaxial_ollama.client import Ollama, OllamaError
    swap.client = Ollama('gemma4:12b', keep_alive=0)
    swap.detail, swap.tool_names = detail.TERSE, ()
    try:
        before = swap.client.model
        said = swap.command('/model no-such-tag:9b')
        report.check('/model refuses a tag that is not pulled, and swaps '
                     'nothing', 'not pulled' in said
                     and swap.client.model == before, said[:52])
        report.check('/model on the tag already running is a no-op',
                     swap.command('/model gemma4:12b') == 'model: gemma4:12b '
                     'already', swap.client.model)
    except OllamaError as exc:
        report.check('/model needs a local ollama daemon', False, str(exc)[:52])

    class VT(io.StringIO):
        encoding = 'utf-8'

        def isatty(self):
            return True

    for tag, ok, colour in (('Simulated', False, '[33m'),
                            ('JTAG and COM3', True, '[32m'),
                            ('RS485 at COM5', True, '[32m')):
        out = VT()
        face = spin.Prompt('Coaxial 63100', out, tick=99, tag=tag, tag_ok=ok)
        face.stop(True)
        painted = out.getvalue()
        report.check('the prompt says (%s)' % tag,
                     '(%s%s[0m)>' % (colour, tag) in painted,
                     'yellow' if ok is False else 'green')
        # stop() repaints the prefix only, and the tag sits after the tail.
        report.check('and the repaint does not eat it: %s' % tag,
                     painted.count(tag) == 1, '%d copies' % painted.count(tag))

    plain = io.StringIO()                      # no isatty, so no VT
    spin.Prompt('Coaxial 63100', plain, tick=99, tag='Simulated', tag_ok=False)
    report.check('a terminal without VT still gets the word, without colour',
                 '(Simulated)>' in plain.getvalue()
                 and '[33m' not in plain.getvalue())


def test_screen(report):
    """Measured with the board unplugged: every board question printed the
    same four-step checklist twice - once clipped as a tool trace, once whole
    as the answer - and paid the ST-Link's fifteen-second timeout twice."""
    from coaxial_ollama import debug

    box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    talk = debug.Chat(ScriptedModel([]), box, tools='code', out=io.StringIO())
    error = ("ERR ConnectError: cannot open COM9 at 115200 baud: could not "
             "open port 'COM9': FileNotFoundError(2, 'no file') -> check the "
             "board is powered, and that a programmer is connected")

    shown = talk._link_down_message(error, shown=True)
    report.check('with the checklist on screen, the answer is one line',
                 shown.count(chr(10)) == 0 and len(shown) < 100, shown)
    report.check('and it keeps the class and what actually failed',
                 'ConnectError' in shown and 'COM9' in shown
                 and 'link is down' in shown, shown)
    report.check('without it, the answer carries the whole error',
                 'FileNotFoundError' in talk._link_down_message(error))

    # -q has no trace, so there is nothing on screen above the answer and the
    # checklist has to travel with it.
    cut = SimulatedSession()
    cut.board.broken = True
    quiet = debug.Chat(ScriptedModel([call('link_diagnose'),
                                      call('analog_read')]),
                       toolmod.Toolbox(cut, scope=Scope()),
                       tools='code', quiet=True, out=io.StringIO())
    answer = quiet.ask('read the NTC')
    report.check('quiet still explains a dead link rather than naming it',
                 'link is down' in answer and len(answer.splitlines()) > 1,
                 answer[:70])

    loud = SimulatedSession()
    loud.board.broken = True
    seen = io.StringIO()
    talkative = debug.Chat(ScriptedModel([call('link_diagnose'),
                                          call('analog_read')]),
                           toolmod.Toolbox(loud, scope=Scope()),
                           tools='code', out=seen)
    answer = talkative.ask('read the NTC')
    report.check('with the checklist traced, the answer does not repeat it',
                 'link is down' in answer and len(answer.splitlines()) == 1,
                 answer[:70])
    # The simulated session has no port, so link_diagnose's own answer is
    # that rather than the four-step checklist - the property under test is
    # the same either way: what the trace printed, the answer does not.
    diagnosis = 'this session is on a simulated board'
    report.check('and the diagnosis was on screen exactly once',
                 seen.getvalue().count(diagnosis) == 1
                 and diagnosis not in answer,
                 '%d times' % seen.getvalue().count(diagnosis))

    # A row that fits is untouched; prose wraps instead of being cut mid-word.
    row = '  4  NTC     SE    32768.0  +1.6500V 25.00C'
    report.check('a reading row is not touched by wrapping',
                 debug._wrapped(row.strip()) == ['  ' + row.strip()])
    prose = ('3. Configured port COM9: not among the ports above - the cable '
             'may be unplugged from this PC\'s side, or the driver did not '
             'enumerate it.')
    wrapped = debug._wrapped(prose)
    report.check('a long line wraps rather than losing its end',
                 len(wrapped) > 1 and wrapped[-1].rstrip().endswith('it.')
                 and all(len(part) <= debug.TRACE_WIDTH for part in wrapped),
                 '%d lines' % len(wrapped))
    report.check('and one row cannot take over the screen',
                 len(debug._wrapped('x ' * 400)) == debug.TRACE_LINES)

    # A capture redirected to a file has no codepage to mismatch, and the
    # locale default here is cp1252 - which turned every Swedish answer in
    # `dbg.py > session.txt` into a row of question marks.
    piped = io.TextIOWrapper(io.BytesIO(), encoding='cp1252')
    debug._printable(piped)
    report.check('a redirected stream is written as UTF-8',
                 piped.encoding.lower().replace('-', '') == 'utf8',
                 piped.encoding)


# ---- the model knows which model it is -------------------------------------

def test_identity(report):
    """Asked what it is, a model with nothing told to it answers out of its
    training - a name, a version and a maker, any of which can be wrong for a
    local tag somebody quantised last week."""
    from coaxial_ollama import debug

    box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    talk = debug.Chat(ScriptedModel([], model='gemma4:12b'), box,
                      tools='code', out=io.StringIO())
    talk.history = [{'role': 'user', 'content': 'which model are you?'}]
    system = talk.trim()[0]['content']
    report.check('the turn says which tag is actually running',
                 'gemma4:12b' in system, system.splitlines()[3][:60])
    report.check('and that it builds and programs this board itself',
                 'build system' in system and 'SWD' in system)

    # A tool set that cannot build must not claim it can - that is the same
    # invention this loop exists to prevent, just in the system prompt.
    reader = debug.Chat(ScriptedModel([], model='gemma4:12b'), box,
                        tools='read', out=io.StringIO())
    reader.history = [{'role': 'user', 'content': 'which model are you?'}]
    read_system = reader.trim()[0]['content']
    report.check('a set without build_firmware claims nothing about building',
                 'gemma4:12b' in read_system and 'build system' not in read_system)

    # One line in, in the operator's own language. Everything else - the tool
    # list, the detail level, the cost - is /help, and printing it on the way
    # in was three lines nobody read twice.
    from coaxial_ollama import language
    hello = language.greeting('gemma4:12b', 'Swedish')
    report.check('the prompt opens with one line, in the machine language',
                 hello.count(chr(10)) == 0 and 'gemma4:12b' in hello
                 and '/help' in hello and 'expert' in hello, hello)
    report.check('a language with no greeting written falls back to English',
                 language.greeting('x', 'Turkish')
                 == language.greeting('x', 'English'))
    report.check('and this machine resolves to a language it can print',
                 language.system_language() in language.LANGUAGE_NAMES,
                 language.system_language())
    report.check('/help carries what the banner no longer does',
                 all(part in talk.command('/help')
                     for part in ('gemma4:12b', 'terse', 'tok/turn',
                                  'analog_read', '/q')),
                 talk.command('/help').splitlines()[2][:60])


# ---- documentation sized for whoever is reading it -------------------------

def test_detail(report):
    """A frontier model over MCP can afford the whole description; gemma4:12b
    pays for it out of the same 8192 tokens the readings come out of. The
    text is picked by code, not written twice."""
    from coaxial_mcp import detail
    from coaxial_ollama import debug

    report.check('a tag that names its size decides on the size',
                 detail.parse_billions('gemma4:12b') == 12.0
                 and detail.parse_billions('llama3.1:8b') == 8.0
                 and detail.parse_billions('qwen2.5:14b') == 14.0)
    report.check('a version number is not a parameter count',
                 detail.parse_billions('qwen3.6:latest') is None
                 and detail.parse_billions('llama3.1') is None)
    report.check('the local sizes get terse, a cloud tag gets full',
                 detail.for_model('gemma4:12b') == detail.TERSE
                 and detail.for_model('qwen2.5:14b') == detail.TERSE
                 and detail.for_model('minimax-m3:cloud') == detail.FULL)
    report.check('an unrecognised tag is assumed small, since the local '
                 'daemon is where unnamed tags live',
                 detail.for_model('qwen3.6:latest') == detail.TERSE)
    report.check('a model at the threshold reads the whole thing',
                 detail.for_model('big:%gb' % detail.FULL_MODEL_B)
                 == detail.FULL)
    report.check('what the operator said beats what the model is',
                 detail.resolve(detail.FULL, model='gemma4:12b') == detail.FULL
                 and detail.resolve(detail.TERSE, model='x:70b')
                 == detail.TERSE)
    report.check('a caller with no model at all gets the full text',
                 detail.resolve() == detail.FULL)

    env, os.environ[detail.ENV] = os.environ.get(detail.ENV), 'terse'
    try:
        report.check('%s decides for the whole machine' % detail.ENV,
                     detail.resolve('auto', model='x:70b') == detail.TERSE)
        report.check('and an explicit flag still beats the environment',
                     detail.resolve('full', model='x:70b') == detail.FULL)
        os.environ[detail.ENV] = 'nonsense'
        report.check('a typo in it is ignored, not raised',
                     detail.resolve('auto', model='gemma4:12b')
                     == detail.TERSE)
    finally:
        if env is None:
            del os.environ[detail.ENV]
        else:
            os.environ[detail.ENV] = env

    specs = [s for s in toolmod.TOOLS if s['name'] in debug.SETS['code']]
    full = json.dumps(toolmod.schemas(specs, detail.FULL))
    terse = json.dumps(toolmod.schemas(specs, detail.TERSE))
    report.check('the default tool set costs meaningfully less terse',
                 len(terse) < len(full) * 0.8,
                 '%d tok against %d' % (len(terse) // 4, len(full) // 4))
    report.check('every tool is still offered, and named the same',
                 [t['function']['name'] for t in json.loads(terse)]
                 == [t['function']['name'] for t in json.loads(full)])
    report.check('no description is emptied by being shortened',
                 all(len(t['function']['description']) > 30
                     for t in json.loads(terse)))
    report.check('a property whose description IS the schema keeps it',
                 'README|CLAUDE' in json.dumps(
                     detail.apply([s for s in toolmod.TOOLS
                                   if s['name'] == 'docs'], detail.TERSE)))
    # Measured: terse dropped analog_read's "omit for all" and the model
    # started naming channels itself, inventing BUS_VOLT and reading five of
    # seven. How to leave a field out is schema, not prose.
    report.check('and so does one that says how to leave the field out',
                 'omit for all' in json.dumps(
                     detail.apply([s for s in toolmod.TOOLS
                                   if s['name'] == 'analog_read'],
                                  detail.TERSE)))
    report.check('shortening never edits the shared TOOLS in place',
                 all('description_terse' in s or len(s['description']) > 0
                     for s in toolmod.TOOLS)
                 and toolmod.TOOLS[0]['description']
                 == detail.apply(toolmod.TOOLS[:1], detail.FULL)[0]['description'])

    # The whole point is that this is decided from the model, with no flag.
    small = debug.Chat(ScriptedModel([], model='gemma4:12b'),
                       toolmod.Toolbox(SimulatedSession(), scope=Scope()),
                       tools='code', out=io.StringIO())
    big = debug.Chat(ScriptedModel([], model='minimax-m3:cloud'),
                     toolmod.Toolbox(SimulatedSession(), scope=Scope()),
                     tools='code', out=io.StringIO())
    report.check('the bench model gets terse without being told',
                 small.detail == detail.TERSE
                 and small.tool_cost() < big.tool_cost(),
                 '%d tok against %d' % (small.tool_cost(), big.tool_cost()))
    report.check('and the toolbox knows too, so `docs` clips for the same '
                 'reader the schemas were shortened for',
                 small.toolbox.detail == detail.TERSE
                 and big.toolbox.detail == detail.FULL)
    report.check('/detail switches it live and reprices the turn',
                 'terse' in small.command('/detail')
                 and 'full' in small.command('/detail full')
                 and small.tool_cost() == big.tool_cost())
    report.check('/detail refuses a level that is not one',
                 'auto' in small.command('/detail sideways')
                 and small.detail == detail.FULL)

    from coaxial_mcp import docs as docsmod
    heading = docsmod._headings(
        docsmod._read(docsmod.paths()['FINDINGS']))[0][1]
    long_section = docsmod.section('FINDINGS', heading, detail.FULL)
    short_section = docsmod.section('FINDINGS', heading, detail.TERSE)
    report.check('a document section is clipped for the reader, not the '
                 'document', len(short_section) < len(long_section) / 2,
                 '%d chars against %d' % (len(short_section),
                                          len(long_section)))
    report.check('and it still says it was clipped, and how to ask for more',
                 'clipped at' in short_section
                 and 'subsection' in short_section)
    report.check('the index keeps the chapter names it is asked by',
                 'FINDINGS' in docsmod.index(detail.TERSE)
                 and len(docsmod.index(detail.TERSE))
                 < len(docsmod.index(detail.FULL)) / 2)


# ---- a prompt that does not fit is a prompt that is not sent ---------------

def test_context_budget(report):
    """What keeps a conversation inside num_ctx, which is the number the
    daemon actually allocates for. Six messages is a small prompt right up
    until one of them is a build log."""
    from coaxial_ollama import context

    report.check('a client with no window to read enforces nothing',
                 context.budget_for(None) == 0
                 and context.budget_for({}) == 0
                 and context.budget_for({'num_ctx': 'nonsense'}) == 0)
    report.check('the reply cap comes out of the same window as the prompt',
                 context.budget_for({'num_ctx': 8192, 'num_predict': 300})
                 == int(8192 * context.CTX_SHARE) - 300)
    report.check('a tiny window still leaves room for a question',
                 context.budget_for({'num_ctx': 256})
                 == context.MIN_PROMPT_TOKENS)

    system = {'role': 'system', 'content': 'be brief'}
    log = 'analog_read: 3 rows\n' + 'x' * 20000

    fitted = context.fit([dict(system),
                          {'role': 'user', 'content': 'read the channels'},
                          {'role': 'tool', 'content': log},
                          {'role': 'assistant', 'content': 'done'},
                          {'role': 'user', 'content': 'and the NTC?'}],
                         budget=400)
    blob = json.dumps(fitted)
    report.check('an oversized tool result is stubbed to its first line',
                 'analog_read: 3 rows' in blob and 'xxxx' not in blob, blob[:70])
    report.check('the system prompt survives being over budget',
                 fitted[0]['content'] == 'be brief')
    report.check('and so does the question just asked',
                 fitted[-1]['content'] == 'and the NTC?')
    report.check('a prompt that fits is left exactly as it was',
                 context.fit([dict(system),
                              {'role': 'user', 'content': 'hi'}],
                             budget=400)
                 == [system, {'role': 'user', 'content': 'hi'}])

    # Mid-turn the last message is a tool result, not the question - so the
    # question is what a naive "drop from the front" would throw away while
    # keeping the reading taken to answer it.
    mid_turn = context.fit(
        [dict(system)]
        + [{'role': 'tool', 'content': 'old: ' + 'y' * 3000} for _ in range(6)]
        + [{'role': 'user', 'content': 'why is the NTC exactly 25.00?'},
           {'role': 'assistant', 'content': ''},
           {'role': 'tool', 'content': 'analog_read: ntc 25.00 C'}],
        budget=120)
    kept = [m['content'] for m in mid_turn]
    report.check('the question outlives the tool results taken to answer it',
                 'why is the NTC exactly 25.00?' in kept, len(mid_turn))
    report.check('and so does the result the model is waiting on',
                 kept[-1] == 'analog_read: ntc 25.00 C')
    report.check('the conversation is actually shortened, not just stubbed',
                 len(mid_turn) < 10, '%d messages' % len(mid_turn))

    # One message larger than the whole window: a pasted log, an attached
    # file. There is no conversation to shorten, so the message itself gives.
    huge = context.fit([dict(system),
                        {'role': 'user', 'content': 'z' * 40000}],
                       budget=600)
    report.check('one message bigger than the window is clipped, not dropped',
                 len(huge) == 2 and len(huge[1]['content']) < 40000
                 and 'more characters cut' in huge[1]['content'],
                 '%d chars' % len(huge[1]['content']))

    report.check('the tool schemas count against the same budget',
                 context.cost([dict(system)], extra_tokens=500)
                 - context.cost([dict(system)]) == 500)


# ---- a full card is not answered by asking again ---------------------------

def test_out_of_memory(report):
    """The other half of the crash above. A runner that died comes back on
    its own; a card that is full stays full, so the retry has to give
    something back before it asks again."""
    from coaxial_ollama import client as clientmod
    from coaxial_ollama.client import Ollama, OllamaError

    oom = OllamaError('/api/chat 500: {"error":"cudaMalloc failed: out of '
                      'memory"}')
    crash = OllamaError('/api/chat 500: {"error":"model runner has '
                        'unexpectedly stopped, this may be due to resource '
                        'limitations"}')
    report.check('running out of memory is told apart from a runner crash',
                 clientmod._out_of_memory(oom)
                 and not clientmod._out_of_memory(crash)
                 and not clientmod._out_of_memory(
                     OllamaError('/api/chat 400: invalid tool schema')))
    report.check('llama.cpp\'s own wording counts too',
                 clientmod._out_of_memory(OllamaError('std::bad_alloc'))
                 and clientmod._out_of_memory(
                     OllamaError('failed to allocate buffer')))

    slept, real_sleep = [], clientmod.time.sleep
    clientmod.time.sleep = slept.append
    try:
        # One other model resident, and the card full. The first rung frees
        # it; the question is then answered without the operator seeing
        # anything but a note.
        talker = Ollama('gemma4:12b', num_ctx=8192)
        posts, gets = [], []

        def one_oom(path, payload):
            posts.append((path, dict(payload)))
            if path == '/api/generate':
                return {}
            if len([p for p in posts if p[0] == '/api/chat']) == 1:
                raise oom
            return {'message': {'role': 'assistant', 'content': 'measured'}}

        def resident(path):
            gets.append(path)
            return {'models': [{'name': 'qwen2.5:14b', 'size_vram': 9 << 30},
                               {'name': 'gemma4:12b', 'size_vram': 8 << 30}]}
        talker._post = one_oom
        talker._get = resident
        message = talker.chat([{'role': 'user', 'content': 'read the ntc'}])
        unloaded = [p[1].get('model') for p in posts
                    if p[0] == '/api/generate']
        report.check('an allocation that failed is retried once room is made',
                     message['content'] == 'measured')
        report.check('the model nobody in this process is using is what goes',
                     unloaded == ['qwen2.5:14b'], unloaded)
        report.check('and this model is dropped too, caches and all',
                     any(p[0] == '/api/chat' and p[1].get('keep_alive') == 0
                         and not p[1].get('messages') for p in posts))
        report.check('what it did to the machine is recorded, not printed',
                     len(talker.notes) == 1 and 'qwen2.5:14b' in talker.notes[0]
                     and '9.0 GB' in talker.notes[0], talker.notes)
        report.check('the window is not shrunk while there is VRAM to free',
                     talker.options['num_ctx'] == 8192)

        # Nothing else resident and still no room: the second rung halves the
        # window, and every turn after this one is asked at the new size.
        stubborn = Ollama('gemma4:12b', num_ctx=8192)
        tries = []

        def two_ooms(path, payload):
            if path != '/api/chat' or not payload.get('messages'):
                return {}
            tries.append(payload['options'].get('num_ctx'))
            if len(tries) < 3:
                raise oom
            return {'message': {'role': 'assistant', 'content': 'measured'}}
        stubborn._post = two_ooms
        stubborn._get = lambda path: {'models': []}
        message = stubborn.chat([{'role': 'user', 'content': 'read the ntc'}])
        report.check('a second failure cuts the context window in half',
                     tries == [8192, 8192, 4096], tries)
        report.check('and the client keeps asking at the size that worked',
                     stubborn.options['num_ctx'] == 4096
                     and message['content'] == 'measured')
        report.check('the operator is told the window moved',
                     any('4096' in note for note in stubborn.notes),
                     stubborn.notes)

        # A machine that cannot hold the model at the floor has a problem no
        # retry solves. It must say so, with the three levers that are the
        # operator's, rather than looping.
        hopeless = Ollama('gemma4:12b', num_ctx=clientmod.MIN_NUM_CTX)
        attempts = []

        def always_oom(path, payload):
            if path == '/api/chat' and payload.get('messages'):
                attempts.append(payload)
                raise oom
            return {}
        hopeless._post = always_oom
        hopeless._get = lambda path: {'models': []}
        try:
            hopeless.chat([{'role': 'user', 'content': 'read the ntc'}])
            report.check('a machine with no room left fails rather than '
                         'looping', False)
        except OllamaError as exc:
            report.check('a machine with no room left fails rather than '
                         'looping', len(attempts) == 2, attempts)
            report.check('and says which lever the operator has',
                         '--num-ctx' in str(exc) and '--num-gpu' in str(exc)
                         and 'capability' in str(exc))

        # The probe is for recovering from an error that already happened.
        # If it fails too, the original error is what has to survive.
        blind = Ollama('gemma4:12b')
        blind._get = lambda path: (_ for _ in ()).throw(
            OllamaError('cannot reach ollama'))
        report.check('a /api/ps that fails does not become the error read',
                     blind.resident() == [])
    finally:
        clientmod.time.sleep = real_sleep


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
    session = SimulatedSession()
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

    # A near miss that can only mean one channel now reads it instead of
    # refusing with the name it meant printed in the refusal. Measured at the
    # prompt with `bus`, which is inside `dcbus` and inside nothing else.
    report.check('a name that can only mean one channel reads that one',
                 _resolve(board, ['dcbusvoltage']) == _resolve(board, ['bus'])
                 == _resolve(board, ['dcbus']))
    report.check('and a word for what it measures resolves too',
                 _resolve(board, ['temp']) == _resolve(board, ['ntc']))
    try:
        _resolve(board, ['phas'])
        report.check('a name that could mean several names them', False)
    except ValueError as exc:
        report.check('a name that could mean several names them',
                     'could be' in str(exc) and 'phaseu' in str(exc),
                     str(exc)[:70])

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
    import os

    from coaxial_ollama import capability as cap

    # This bench itself runs with COAXIAL_VRAM_RESERVE_GB set - see MODELS.md's
    # own `-Reserve 8` example - so a test that leaves the real environment in
    # place fails every unoverridden assertion below on the exact machine these
    # docs were measured on. Cleared for the whole function, not just around the
    # override scenario further down, and restored after.
    had_override = cap.RESERVE_ENV in os.environ
    saved_override = os.environ.pop(cap.RESERVE_ENV, None)
    try:
        _test_capability(report, cap)
    finally:
        if had_override:
            os.environ[cap.RESERVE_ENV] = saved_override


def _test_capability(report, cap):
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
                 len(index) < 4500, '%d chars' % len(index))
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

    # Measured on this bench: this exact question has only one word from the
    # rest of Swedish's list ('over') against two from Dutch's ('en', 'de'),
    # so Dutch outscored Swedish outright and the model answered in a Dutch/
    # Norwegian mix instead. 'en' and 'de' are now in both lists on purpose -
    # see the comment on STOPWORDS - which cancels them as a discriminator
    # rather than leaving them to favour whichever list happened to claim
    # them first.
    report.check('a Swedish question is not lost to Dutch on "en"/"de" alone',
                 language.detect(
                     'ger du mig en tabell över de analoga mätvärdena?')
                 == 'Swedish')

    # Locked at 'German' by the loop above. A message that detects as
    # nothing on its own - "status?" is too short to score any language -
    # must not fall back to mirroring once something has actually been
    # settled: that fallback is a different system-prompt line, a real
    # prefix change, and paying for a KV reload on every one-word follow-up
    # was the whole thing this lock exists to stop.
    talk.history = [{'role': 'user', 'content': 'status?'}]
    report.check('an ambiguous follow-up keeps the session lock, not a '
                 'fallback to mirroring',
                 'in German' in talk.trim()[0]['content'])

    fresh = debug.Chat.__new__(debug.Chat)
    fresh.keep = 6
    fresh.history = [{'role': 'user', 'content': 'status?'}]
    report.check('but with nothing locked yet, the same question falls '
                 'back to mirroring',
                 'language the question was asked in' in fresh.trim()[0]['content'])
    report.check('and the board keeps its own words either way',
                 'stay exactly as the board prints them'
                 in language.instruction('Vad är temperaturen?'))

    talk.history = [{'role': 'user', 'content': 'svara pa engelska tack'}]
    talk.trim()
    report.check('naming a language outright switches the lock even though '
                 'the question itself is written in the locked language',
                 talk.language == 'English', talk.language)

    # Measured at the prompt: "forklara pa japanska ..." named a language
    # next to a verb that was not in the list, so the request was missed and
    # the turn went out under *Answer in Swedish and in no other language* -
    # the host contradicting the operator in the same system prompt. The
    # answer that came back was a channel table.
    talk.history = [{'role': 'user',
                     'content': 'förklara på japanska vad detta '
                                'projektet handlar om'}]
    head = talk.trim()[0]['content']
    report.check('a language asked for with a verb other than "answer" is '
                 'still a request', 'in Japanese' in head, talk.language)

    # ...and it is that turn's request, not a new session language: the next
    # Swedish question takes the lock straight back.
    talk.history = [{'role': 'user', 'content': 'och vad läser NTC:n nu?'}]
    talk.trim()
    report.check('and the next question in the session language takes it back',
                 talk.language == 'Swedish', talk.language)

    for question in ('the German firmware bug is back',
                     'varför är dokumentationen på engelska?',
                     'vad heter skruv på japanska?'):
        got = language.requested_language(question)
        report.check('a language named in passing is not a request: %s'
                     % question[:34], got is None, str(got))

    # A lock with no way out is a trap. Measured: locked to Korean by the
    # question before it, "byt sprak till svenska" matched no verb and
    # detected as no language either - every word in it is outside every
    # stop-word list - so the lock held and the model obeyed it, refusing
    # to switch, in Korean. A name in a message that places in no language
    # is the request: there is nothing else in the message.
    talk.language = 'Korean'
    talk.history = [{'role': 'user', 'content': 'byt språk till svenska'}]
    talk.trim()
    report.check('a locked session can be talked out of its language',
                 talk.language == 'Swedish', talk.language)

    for question, expect in (('svenska tack', 'Swedish'),
                             ('switch to Swedish please', 'Swedish'),
                             ('kan du prata svenska?', 'Swedish'),
                             ('tillbaka till engelska', 'English')):
        got = language.requested_language(question)
        report.check('asking for a language plainly: %s' % question[:30],
                     got == expect, str(got))

    # ...and the model is told the same thing, for the phrasing the host
    # misses next. The refusal above was the host and the operator
    # contradicting each other with the model in the middle.
    report.check('an explicit request overrides the lock in the prompt too',
                 'unless the operator asks for another'
                 in language.instruction_for('Korean'))

    # A message that is nothing but a language request is host business.
    # It used to cost a model turn that answered "Jag har andrat spraket
    # till svenska. Hur kan jag hjalpa dig med din BLDC-inverter?" - and a
    # host note above it saying the same thing again, in a mix of two
    # languages. One word, no round trip, history untouched.
    switch_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    switcher = debug.Chat(ScriptedModel([]), switch_box, out=io.StringIO())
    switcher.language = 'Swedish'
    said = switcher.ask('byt språk till japanska')
    report.check('a bare switch never reaches the model',
                 not switcher.client.prompts and not switcher.history,
                 '%d turns' % len(switcher.client.prompts))
    report.check('and is answered with one word in the language asked for',
                 said == language.OKAY['Japanese'] == 'わかりました',
                 said.encode('unicode_escape').decode())
    report.check('which is what the lock moved to',
                 switcher.language == 'Japanese', switcher.language)

    # The same cp1252 case greeting() documents: Japanese renders as a row
    # of question marks on a console that cannot hold it, and "Okay" beats
    # five of those.
    narrow = io.TextIOWrapper(io.BytesIO(), encoding='cp1252')
    plain = debug.Chat(ScriptedModel([]), switch_box, out=narrow)
    report.check('an alphabet the console lacks falls back to English',
                 plain.ask('byt språk till japanska') == 'Okay')

    # ...but a request with a question attached is still the model's turn,
    # in the new language. This is the line between the two.
    asked_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    with_question = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content': 'このプロジェクトはBLDCインバータです。'}]),
        asked_box, out=io.StringIO())
    with_question.language = 'Swedish'
    said = with_question.ask('förklara på japanska vad detta projektet '
                             'handlar om')
    report.check('a request with a question attached still goes to the model',
                 with_question.client.prompts and said.startswith('この'),
                 said[:12].encode('unicode_escape').decode())

    for question, bare in (('svenska tack', 'Swedish'),
                           ('switch to Swedish please', 'Swedish'),
                           ('kan du prata svenska?', 'Swedish'),
                           ('läs NTC:n', None),
                           ('förklara på japanska vad detta handlar om', None)):
        got = language.bare_switch(question)
        report.check('bare switch or a real question: %s' % question[:32],
                     got == bare, str(got))

    # Measured live: a Swedish question that named a tool without calling it
    # triggered the "call the tool now" nudge - appended to history with
    # role=='user', for the model's benefit - and the language flipped to
    # English on the next trim() because that nudge's own English words were
    # now the last "user" message in history. self.prompt_history exists
    # precisely so this cannot happen: it is appended once, at the top of
    # ask(), and nothing added to history later in the same turn can reach it.
    # Measured from prompt_io.tmp: "Beskriv hardvaran i detta projektet for
    # en novis" logged `analog_read {"ch": ["all"]}`, its table, and then
    # `A:` with nothing after it. The blank answer reached the operator as a
    # blank line under the table - the stale gate is closed by last_channels,
    # which that same call had just set, so nothing caught it. A turn never
    # ends on silence: nudged for words first, then a line saying so.
    blank_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    blank = debug.Chat(ScriptedModel([
        call('analog_read'),
        {'role': 'assistant', 'content': ''},
        {'role': 'assistant', 'content': 'Kortet är en trefas BLDC-drivare.'},
    ]), blank_box, out=io.StringIO())
    said = blank.ask('beskriv hårdvaran för en novis')
    report.check('a reading followed by silence is nudged into an answer',
                 said == 'Kortet är en trefas BLDC-drivare.', repr(said))
    report.check('and the nudge asks for words, not for another reading',
                 any('in words' in (m.get('content') or '')
                     for m in blank.history), 'no such nudge')

    mute_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    mute = debug.Chat(ScriptedModel([
        call('analog_read'),
        {'role': 'assistant', 'content': ''},
        {'role': 'assistant', 'content': ''},
        {'role': 'assistant', 'content': ''},
    ]), mute_box, out=io.StringIO())
    said = mute.ask('beskriv hårdvaran för en novis')
    # Swedish, because the question was: the fallback is host-authored text
    # reaching the screen, so it goes through PHRASES like every other line
    # the operator reads. A blank line was the one thing it must not be.
    report.check('a model that stays silent still ends the turn with words',
                 'fråga igen' in said, repr(said))

    nudge_box = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    nudged = debug.Chat(ScriptedModel([
        {'role': 'assistant', 'content': 'jag ska nu anropa analog_read'},
        call('analog_read')]), nudge_box, out=io.StringIO())
    nudged.ask('vad ar temperaturen pa kortet?')
    report.check("a nudge mid-turn does not steal the language lock from "
                 "what the operator actually typed",
                 nudged.language == 'Swedish', nudged.language)

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

    # The reverse of what this checked before, deliberately: asked to
    # measure, the model called docs and answered with HARDWARE.md's own
    # channel table instead of a reading. SYSTEM says nothing about
    # documents any more, and no default tool set offers them - a session
    # that genuinely wants them asks by name (-t docs), and only then does
    # DOCS_HINT cost a line.
    report.check('the bench prompt does not send the model to the documents',
                 'docs' not in debug.SYSTEM and 'FINDINGS' not in debug.SYSTEM)
    report.check('the docs warning is there for a session that does ask',
                 'never docs' in debug.DOCS_HINT)
    report.check('and never to restate a result already printed above',
                 'restate' in debug.SYSTEM)
    report.check('afe_power is never framed as refusable',
                 'afe_power' in debug.SYSTEM and 'order to do it' in debug.SYSTEM)

    # The line that taught the error. "A table or list means analog_read
    # once" was written about tabulating readings and read as "a list means
    # analog_read", so a question asking for a list of channels fetched a
    # full analog table - measured 6 times out of 24 in test_live_model.py,
    # in both languages. That suite needs ollama and minutes; this catches
    # the wording coming back in seconds.
    report.check('SYSTEM does not tell the model a list is a reading',
                 'table or list' not in debug.SYSTEM,
                 [l for l in debug.SYSTEM.splitlines() if 'analog_read' in l][:1])
    report.check('it names board_info for listing channels',
                 'list of channels is board_info' in debug.SYSTEM)
    report.check('and digital_read for a pin, beside analog_read',
                 'digital_read' in debug.SYSTEM and 'analog_read' in debug.SYSTEM)
    report.check('and a read with AFE off is never framed as impossible',
                 'AFE on or off and reports' in debug.SYSTEM)
    report.check('and afe_power never fires as a side effect of a reading',
                 'never afe_power first' in debug.SYSTEM)
    report.check("and mid-scale is the tool's fact to report, not the prompt's "
                 'to recite',
                 'mid-scale' not in debug.SYSTEM)
    report.check('the runner tells it too',
                 'docs' in runner.SYSTEM and 'FINDINGS' in runner.SYSTEM)
    report.check('no default tool set offers docs - only `docs` and `all`',
                 not any('docs' in debug.SETS[s]
                         for s in ('read', 'code', 'pins', 'build'))
                 and 'docs' in debug.SETS['docs']
                 and 'docs' in debug.SETS['all'])


def main():
    report = Report()
    for test in (test_plan, test_verdicts, test_model_never_sees_limits,
                 test_misbehaviour, test_board_tools, test_scope, test_shell,
                 test_scope_repairs, test_prompt, test_policy,
                 test_link_diagnose, test_link_recovery, test_channel_map, test_corrections_are_reported,
                 test_smart_selection, test_afe_trace, test_reading_block, test_digital_read, test_map_sections, test_map_retype, test_port_state,
                 test_retype_with_the_trace_off,
                 test_power_check_cannot_halt,
                 test_transcript,
                 test_debug, test_cli,
                 test_local_only, test_runner_crash_retry,
                 test_out_of_memory, test_context_budget, test_detail,
                 test_screen, test_screen_language, test_fallback,
                 test_identity,
                 test_keep_alive,
                 test_coerce,
                 test_capability, test_docs):
        print('\n-- %s --' % test.__name__[5:].replace('_', ' '))
        test(report)
    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())

