#!/usr/bin/env python3
"""The plan runner, the sandbox, and the test tooling itself.

Split out of test_ollama.py, which had grown to 5,496 lines and 733 checks in
one file - a third of every check this tree has, and the reason a coverage
tier could not be asked for at any useful resolution. One subject per file
now, so a tier buys them separately and a reader opens the one they meant.

Run from the host directory:  python tests/test_ollama_runner.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (TAGS, select, BSLASH, Scope, Shell, SimulatedBoard, 
    _test_capability, build, call, counts, json, os, planmod, runmod, 
    sys, tempfile, toolmod, types)   # noqa: E402



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

    # A package that is genuinely not here, so the failure has to name
    # the alternative rather than only the refusal. NOT pandas: it is
    # installed now, for `daq.frame()`, and a test that asserted its
    # absence was testing the bench rather than the runner.
    missing = scope.run('import scipy')
    report.check('a missing package says what is here instead',
                 'statistics' in missing and 'namespace holds' in missing,
                 missing.splitlines()[-1][:52])

    # The tool names and the method names are different words, and a model
    # that has called analog_read all session reaches for it here too.
    confused = scope.run('board.analog_read(ch=["ntc"])')
    report.check('a tool name used as a method is corrected',
                 'board.analog' in confused and 'read_all' in confused,
                 confused.splitlines()[-1][:52])
    report.check('and the real method still works',
                 scope.run('board.afe.enable()').strip() == 'True')

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
             set(run_tests.OLLAMA), {'language'},
             'the language lock is half the live suite, not all of it'),
            (['host/coaxial_ollama/replies.py'],
             set(run_tests.OLLAMA), {'tools'},
             'what an answer means is the tool-choice half'),
            (['host/coaxial_mcp/render.py'],
             {'test_mcp.py', 'test_parity.py'} | set(run_tests.OLLAMA), set(),
             'a renderer cannot change which tool gets called'),
            (['docs/HARDWARE.md'], {'test_ollama_runner.py'}, set(),
             'a document can only break the docs index'),
            (['modbus/src/modbus_rtu.c'],
             {'test_modbus_core.py', 'test_conformance.py', 'test_mcp.py'},
             set(),
             'the byte-level master for the wire, and the host build for '
             'the framing itself - which needs no cable'),
            (['shtp/src/shtp.c'], {'test_shtp_core.py'}, set(),
             'the SHTP layer is built and run here, and touches no Modbus '
             'wire'),
            (['host/tests/test_parity.py'], {'test_parity.py'}, set(),
             'editing a suite is a reason to run it'),
            (['host/coaxial_ollama/debug.py'],
             set(run_tests.OLLAMA), {'all'},
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
    #
    # An invented path, not a real one. This check used to name setup.ps1,
    # which meant it failed the day the map grew an entry for setup.ps1 -
    # the test was asserting a hole in the map rather than the behaviour at
    # one, and the map is supposed to grow.
    suites, live, why = run_tests.pick(['nothing/the/map/knows.xyz'])
    report.check('an unmapped path runs everything, and says why',
                 run_tests.CONFORMANCE in suites and live == {'all'}
                 and 'unmapped' in why[-1], why[-1][:52])

    # And the other half of that: an entry deliberately mapped to nothing is
    # a rule, not a hole, and must not fall through to the whole gate.
    suites, live, why = run_tests.pick(['datasheets/imu/UserGuide.pdf'])
    report.check('a path nothing reads runs nothing, and says that instead',
                 not suites and not live and 'nothing reads it' in why[-1],
                 why[-1][:52])

    report.check('and the whole lot goes every tenth commit',
                 run_tests.FULL_EVERY == 10)

    # The map settles the cheap cases on its own. Asking the model there
    # costs a 7.6 GB load to be told what the map already said, and the
    # model's answer can only widen it - which is how editing a demo
    # wrapper used to cost seven minutes.
    for paths, what in (
        (['terminal/adc.ps1'], 'a demo wrapper'),
        (['coaxial_tty.ps1'], 'the demo picker'),
        (['host/coaxial/desk.py'], 'a pure renderer'),
        (['host/tools/show_desk.py'], 'a live view'),
        (['datasheets/imu/UserGuide.pdf'], 'something no suite reads'),
    ):
        suites, _, why = run_tests.pick(paths)
        report.check('%s does not need the model' % what,
                     run_tests.settled(suites, why),
                     ', '.join(sorted(suites)) or 'nothing to run')

    for paths, what in (
        (['host/coaxial/board.py'], 'the library'),
        (['board/src/board_cal.c'], 'firmware'),
        (['nothing/the/map/knows.xyz'], 'an unmapped path'),
    ):
        suites, _, why = run_tests.pick(paths)
        report.check('%s still goes to the model' % what,
                     not run_tests.settled(suites, why),
                     ', '.join(sorted(suites)))

    report.check('and what it settles on runs without a board or a model',
                 run_tests.CONFORMANCE not in run_tests.CHEAP
                 and run_tests.LIVE not in run_tests.CHEAP
                 and not (set(run_tests.OLLAMA) & run_tests.CHEAP),
                 ', '.join(sorted(run_tests.CHEAP)))

    # A tier is a budget and the model spends inside it. Measured: on the
    # 25 % tier the model put back the live suite the tier had dropped, and
    # the cheapest run there is took 398 s of which 352 were that suite.
    class _Args:
        def __init__(self, coverage, files):
            self.coverage, self.file, self.live = coverage, list(files), True

    args = _Args(25, [run_tests.STRUCTURE, 'test_simulated.py',
                      run_tests.CONFORMANCE])
    left = run_tests._within_tier(args, 'all')
    report.check('the 25% tier drops what the model added past it',
                 run_tests.CONFORMANCE not in args.file and left == ''
                 and not args.live, ', '.join(args.file))
    report.check('   ...and keeps what was inside it',
                 args.file == [run_tests.STRUCTURE, 'test_simulated.py'],
                 ', '.join(args.file))

    # 75 % is the tier that does buy conformance and the live tool matrix.
    args = _Args(75, [run_tests.STRUCTURE, run_tests.CONFORMANCE])
    left = run_tests._within_tier(args, 'tools')
    report.check('a tier that does stretch to conformance keeps it',
                 run_tests.CONFORMANCE in args.file and left == 'tools',
                 '%s live:%s' % (', '.join(args.file), left))

    args = _Args(None, [run_tests.CONFORMANCE])
    report.check('and with no tier at all nothing is clamped',
                 run_tests._within_tier(args, 'all') == 'all'
                 and args.file == [run_tests.CONFORMANCE])

    # A tier is arithmetic now, not a table of three, so any 5 % step can be
    # named. What must hold at every one of them: it never shrinks as the
    # percentage rises, structure is always in, and the four named switches
    # still mean exactly what they meant before.
    grew = []
    for percent in run_tests.TIERS:
        suites, sections = run_tests.plan_for(percent)
        grew.append((percent, set(suites), sections))

    report.check('every 5 %% step from %d to 100 is a tier'
                 % run_tests.STEP, len(run_tests.TIERS) == 20
                 and run_tests.TIERS[0] == 5 and run_tests.TIERS[-1] == 100,
                 '%d tiers' % len(run_tests.TIERS))
    report.check('structure is in every one of them - it is the precondition '
                 'for reading any other result',
                 all(run_tests.STRUCTURE in s for _, s, _ in grew))
    report.check('a deeper tier never runs less than a shallower one',
                 all(grew[i][1] <= grew[i + 1][1]
                     for i in range(len(grew) - 1)),
                 ' -> '.join('%d:%d' % (p, len(s)) for p, s, _ in grew[:6]))
    report.check('the live suite is the last thing any budget buys',
                 all(sections is None for p, _, sections in grew
                     if p < run_tests.LIVE_FROM),
                 'joins at %d %%' % run_tests.LIVE_FROM)

    # The four named switches are what everybody types. They have to keep
    # meaning what they did when they were a table.
    # DAQ_API joined at 12 % - the acquisition front door against the
    # stand-in, no board and no compiler - so both tiers below buy it.
    for percent, expect in ((25, {run_tests.STRUCTURE, run_tests.CORE,
                                  run_tests.SHTP, 'test_simulated.py',
                                  run_tests.DRIVE, run_tests.FILTER,
                                  run_tests.SENSORLESS, run_tests.DAQ_API,
                                  } | set(run_tests.OLLAMA)),
                            (75, {run_tests.STRUCTURE, run_tests.CORE,
                                  run_tests.SHTP, 'test_simulated.py',
                                  run_tests.DRIVE, run_tests.FILTER,
                                  run_tests.SENSORLESS, run_tests.DAQ_API,
                                  'test_parity.py', 'test_mcp.py',
                                  run_tests.CONFORMANCE, run_tests.BENCH}
                             | set(run_tests.OLLAMA))):
        suites, _ = run_tests.plan_for(percent)
        report.check('%d %% is what it always was' % percent,
                     set(suites) == expect,
                     ', '.join(sorted(set(suites) ^ expect)) or 'unchanged')



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



# ---- the tag follows the machine ------------------------------------------

def test_chat_hands_the_card_back(report):
    """Chat.close() unloads - the teardown every page reaches for.

    The chooser's chat page ends with `getattr(chat, 'close', None)`;
    only the claude page had one, so leaving BOARD CHAT parked the local
    model for its whole keep_alive. A getattr default of None fails
    SILENTLY, which is why this is a check and not a code review note.
    """
    from coaxial_ollama.client import Ollama
    from coaxial_ollama.debug import Chat

    unloaded = []
    client = Ollama('gemma4:12b')
    client.unload = lambda: unloaded.append(True)
    chat = Chat.__new__(Chat)
    chat.client = client
    chat.io_log = None
    chat.close()
    report.check('close() unloads the model', unloaded == [True])
    closed = []
    chat.io_log = type('L', (), {'close': lambda self: closed.append(True)})()
    chat.close()
    report.check('and closes the io log when there is one',
                 closed == [True])


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

def test_picker(r):
    """tools/pick_tests.py - the model picks subjects, this checks the frame.

    Not what the model answers: that is its job and it changes per model.
    What is checked is every way its answer can be useless, because each one
    has to land on "run everything" and a picker that silently narrows on a
    reply it did not understand is worse than no picker.
    """
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'tools'))
    import pick_tests

    # Not `is`: run as __main__ this file is imported a second time under
    # its real name, so the picker holds an equal dict, not the same one.
    r.check('the catalogue the model is shown is the one the tests carry',
            pick_tests.TAGS == TAGS, repr(sorted(pick_tests.TAGS)))

    j = json.dumps
    # A suite name the picker will recognise. Any of the nine does;
    # what these checks are about is the tags, not the file.
    one = ['test_ollama_render.py']

    def reply(**kw):
        kw.setdefault('suites', one)
        return j(kw)

    for name, text, want, note in (
            ('two known tags', reply(tags=['render', 'reply'], why='x'),
             ['render', 'reply'], None),
            ('one unknown, kept and reported',
             reply(tags=['render', 'nosuch']), ['render'], 'dropped'),
            ('all unknown', reply(tags=['nosuch']), [], None),
            ('every subject is the same as none',
             reply(tags=sorted(TAGS)), [], None),
    ):
        plan, why = pick_tests.parse(text)
        r.check('parse: %s -> %s' % (name, want if want else 'no narrowing'),
                plan is not None and plan.tags == want
                and (note is None or note in why),
                'got %r' % (plan and plan.tags,))

    for name, text, note in (
            ('not JSON at all', 'sure, I would run the render tests',
             'not the JSON'),
            ('no suite named', j({'tags': ['render']}), 'no suite'),
            ('a suite this repository does not have',
             j({'suites': ['test_nope.py']}), 'no suite'),
    ):
        plan, why = pick_tests.parse(text)
        r.check('parse: %s -> run everything' % name,
                plan is None and note in why, 'got %r / %r' % (plan, why))

    # Case and whitespace: a model that answers "Render" has still answered.
    plan, _ = pick_tests.parse(reply(tags=[' Render ', 'REPLY']))
    r.check('parse: a tag is a tag whatever its case or padding',
            plan.tags == ['render', 'reply'], repr(plan.tags))
    r.check('and the suite list comes back as named files',
            plan.suites == one, repr(plan.suites))
    r.check('a live section outside the catalogue falls back to all',
            pick_tests.parse(reply(live='sideways'))[0].live == 'all')
    r.check('and "none" is a real answer, not a missing one',
            pick_tests.parse(reply(live='none'))[0].live == 'none')

    long = ('diff --git a/host/coaxial_ollama/replies.py b/x\n'
            + 'x' * 9000 + '\ndiff --git a/host/tests/test_ollama.py b/y\n')
    cut = pick_tests.clip(long, 200)
    r.check('clip keeps the head, where the file names are',
            cut.startswith('diff --git a/host/coaxial_ollama/replies.py')
            and 'more characters of diff' in cut and len(cut) < 300)
    r.check('a diff that fits is passed through untouched',
            pick_tests.clip('short', 200) == 'short')

    # No ollama, no answer, no narrowing. The import inside pick() is what
    # this substitutes, so the failure lands exactly where a missing daemon
    # would put it.
    real = sys.modules.get('coaxial_ollama.client')
    broken = types.ModuleType('coaxial_ollama.client')

    def explode(*a, **k):
        raise OSError('connection refused')
    broken.Ollama = explode
    sys.modules['coaxial_ollama.client'] = broken
    try:
        plan, why = pick_tests.pick(against='HEAD')
    finally:
        if real is None:
            sys.modules.pop('coaxial_ollama.client', None)
        else:
            sys.modules['coaxial_ollama.client'] = real
    r.check('ollama unreachable runs everything, and says why',
            plan is None and 'connection refused' in why, repr(why))

def _every_roster():
    """(test, tags) from all nine subject files, as one list.

    The roster invariants are about the SET of suites, not about any one of
    them: every catalogue tag has to be on some test somewhere, and the draw
    that covers a picker's blind spot has to be able to reach every subject.
    Neither is checkable inside a file that only owns one subject, which is
    what splitting test_ollama.py cost and what this buys back.
    """
    import importlib

    whole = []
    for tag in sorted(TAGS):
        module = importlib.import_module('tests.test_ollama_%s' % tag)
        whole.extend(module.ROSTER)
    return whole


def test_tag_roster(r):
    """The tags, the roster, and the random draw that covers the pick's gaps."""
    whole = _every_roster()
    named = {t for _, marks in whole for t in marks}

    r.check('every tag on the roster is in the catalogue',
            not (named - set(TAGS)), repr(sorted(named - set(TAGS))))
    r.check('every catalogue tag is on some test',
            not (set(TAGS) - named), repr(sorted(set(TAGS) - named)))
    r.check('there is a file per subject, and it owns its own roster',
            len(TAGS) == 9 and len(whole) >= 40,
            '%d subjects, %d groups' % (len(TAGS), len(whole)))

    listed = [t for t, _ in whole]
    r.check('no test is listed twice', len(listed) == len(set(listed)))

    here = {v for k, v in sorted(globals().items())
            if k.startswith('test_') and callable(v)}
    mine = {t for t, _ in ROSTER}
    r.check('every test in this file is on its own roster (%d)'
            % len(here), not (here - mine),
            repr(sorted(f.__name__ for f in here - mine)))
    r.check('and this roster names none that is gone',
            not (mine - here), repr(sorted(f.__name__ for f in mine - here)))

    # The draw exists because the picker can be wrong the expensive way -
    # by not thinking of a subject at all. One test per uncovered subject,
    # and that floor holds whatever the coverage budget says.
    marks_of = dict(whole)
    chosen = select(whole, {'reply'}, seed=1)
    covered = {t for t, marks in whole if 'reply' in marks}
    r.check('the pick itself is in the run', covered <= chosen)
    reached = {tag for f in chosen for tag in marks_of[f]}
    r.check('and every subject is reached, not just the picked one',
            set(TAGS) <= reached, repr(sorted(set(TAGS) - reached)))
    r.check('the same seed selects the same tests',
            select(whole, {'reply'}, 1) == select(whole, {'reply'}, 1))
    r.check('no pick means no selection - the file runs whole',
            select(whole, set(), 1) == set())

    sizes = counts.load().get('groups') or {}
    if sizes:
        measured = float(sum(sizes.get(t.__name__, 0) for t, _ in whole))
        for pct in (25, 50, 75):
            got = select(whole, {'reply'}, 1, pct)
            share = 100.0 * sum(sizes.get(t.__name__, 0)
                                for t in got) / max(1.0, measured)
            r.check('%d%% asks for %d%% of the checks' % (pct, pct),
                    abs(share - pct) <= 12, '%.0f%%' % share)
        r.check('a budget cuts as well as fills',
                len(select(whole, {'reply'}, 1, 25))
                < len(select(whole, {'reply'}, 1, 75)))
        floor = select(whole, {'reply'}, 1, 1)
        r.check('and never below one test per uncovered subject',
                {tag for f in floor for tag in marks_of[f]} >= set(TAGS),
                repr(sorted(set(TAGS) - {tag for f in floor
                                         for tag in marks_of[f]})))


def test_mouse(r):
    """The wheel and the right-drag, out of a terminal's own reports.

    SGR mouse encoding, because the older one packs each coordinate into a
    single byte and simply stops reporting past column 223 - which on a
    full-screen window is most of it.
    """
    import screen

    keys = screen.Keys(console=True, mouse=True)
    # What the caller DOES with the number, not the sign of the number: it
    # scales zoom by 1 + this, and a bigger zoom stands closer. Checking the
    # sign alone is what let the wheel run backwards under a test named for
    # the behaviour it was not testing.
    def after(report, zoom=1.0):
        keys._buffer = report
        return zoom * (1.0 + keys.poll()[1])

    r.check('a wheel notch up comes nearer',
            after(chr(27) + '[<64;10;10M') > 1.0,
            '%.3f' % after(chr(27) + '[<64;10;10M'))
    r.check('and down goes further',
            after(chr(27) + '[<65;10;10M') < 1.0,
            '%.3f' % after(chr(27) + '[<65;10;10M'))

    # One frame's worth of reports is one change: twenty drag reports applied
    # one at a time would move twenty frames behind the hand.
    keys._buffer = (chr(27) + '[<2;5;20M' + chr(27) + '[<34;5;23M'
                    + chr(27) + '[<34;5;26M')
    leave, zoom = keys.poll()
    r.check('a right-drag pulled down backs away, and adds up over the '
            'frame it arrived in',
            leave is None and abs(zoom + 6 * screen.DRAG_STEP) < 1e-9,
            '%.3f over 6 rows' % zoom)

    keys._buffer = chr(27) + '[<2;5;26m' + chr(27) + '[<34;5;40M'
    r.check('and stops when the button is let go', keys.poll()[1] == 0.0)

    # A key arriving in the same read as a mouse report must survive it.
    keys._buffer = chr(27) + '[<64;1;1M' + 'q'
    r.check('a keystroke mixed in with mouse reports is not eaten',
            keys.poll()[0] == 'quit')

    # Held ONE poll on purpose: the same tail twice is a real lone ESC,
    # once is maybe the front of a split arrow or mouse report.
    keys._buffer = chr(27)
    held = keys.poll()[0]
    r.check('and a bare ESC is still the menu, one poll late, not a '
            'half-read report', held is None and keys.poll()[0] == 'menu')

    r.check('a view with no terminal reads no mouse at all',
            screen.Keys(console=False, mouse=True).poll() == (None, 0.0))

    # A Windows console hands mouse movement over as MOUSE_EVENT records,
    # which msvcrt never shows the program. Without VT input the wheel does
    # nothing at all there, whatever the view prints to ask for reporting -
    # and the checks above could not see it, because they feed the parser
    # bytes directly and never touch the console.
    was = screen.LINE_INPUT | screen.ECHO_INPUT | screen.QUICK_EDIT
    now = screen.console_mode(was)
    r.check('a mouse view asks the console for VT input and takes the mouse',
            now & screen.VT_INPUT and now & screen.MOUSE_INPUT
            and now & screen.EXTENDED_FLAGS, '0x%04X -> 0x%04X' % (was, now))
    r.check('and gives up quick-edit, line mode and echo to get it',
            not (now & (screen.QUICK_EDIT | screen.LINE_INPUT
                        | screen.ECHO_INPUT)))


ROSTER = (
    (test_plan, ('runner',)),
    (test_verdicts, ('runner',)),
    (test_model_never_sees_limits, ('runner', 'prompt')),
    (test_misbehaviour, ('runner', 'reply')),
    (test_scope, ('runner',)),
    (test_scope_repairs, ('runner',)),
    (test_shell, ('runner',)),
    (test_smart_selection, ('runner',)),
    (test_transcript, ('runner',)),
    (test_cli, ('runner',)),
    (test_local_only, ('runner',)),
    (test_runner_crash_retry, ('runner',)),
    (test_out_of_memory, ('runner',)),
    (test_keep_alive, ('runner',)),
    (test_chat_hands_the_card_back, ('runner',)),
    (test_capability, ('runner',)),
    (test_picker, ('runner',)),
    (test_tag_roster, ('runner',)),
    (test_mouse, ('runner',)),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
