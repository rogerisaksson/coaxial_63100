#!/usr/bin/env python3
"""The tool surface: schemas, arguments, which tool answers what.

Split out of test_ollama.py, which had grown to 5,496 lines and 733 checks in
one file - a third of every check this tree has, and the reason a coverage
tier could not be asked for at any useful resolution. One subject per file
now, so a tier buys them separately and a reader opens the one they meant.

Run from the host directory:  python tests/test_ollama_tools.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (CHANNELS, Scope, ScriptedModel, Shell, 
    SimulatedSession, build, call, detail, io, json, os, replies, runmod, 
    sys, toolmod)   # noqa: E402



# ---- the board gate drivers ------------------------------------------------------

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
    # "Mid-scale... 25.00 C" straight out of the warning text.
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

    # ----...and if it is still down, the fresh check says so, not the model --
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

    # ----...and if the link turns out fine, the model gets a real turn ----
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

    # ----...but an unrelated question with no call is never touched ----
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

    # ----...and if the board really is down from the start, that is what
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

    # ----...but nudging is not an open invitation to loop forever ----
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

    # ----...but a dedup cannot launder a failure back into a success ------
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

    #...and when the board really is unreachable, that refusal says so -
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



# ---- documentation sized for whoever is reading it -------------------------

def test_detail(report):
    """A frontier model over MCP can afford the whole description; gemma4:12b
    pays for it out of the same 8192 tokens the readings come out of. The
    text is picked by code, not written twice."""
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

    # The clipper, against text written for the purpose. Measured against a
    # real document instead, this asserted that some chapter of FINDINGS was
    # over 1200 characters - which is a fact about how much prose happens to
    # be in the repository that day, not about the tool. It failed the moment
    # the documents were cut, having found no defect.
    from coaxial_mcp import docs as docsmod
    nl = chr(10)
    made_up = '## Long' + nl + nl + 'x' * (docsmod.CLIP + 500) + nl
    was_read = docsmod._read
    docsmod._read = lambda path: made_up
    try:
        whole = docsmod.section('FINDINGS', 'Long', detail.FULL)
        short = docsmod.section('FINDINGS', 'Long', detail.TERSE)
        docsmod._read = lambda path: ('## Short' + nl + nl
                                      + 'under the limit.' + nl)
        intact = docsmod.section('FINDINGS', 'Short', detail.TERSE)
    finally:
        docsmod._read = was_read
    report.check('a section is clipped for the reader, not the document',
                 len(short) < len(whole),
                 '%d chars against %d' % (len(short), len(whole)))
    report.check('and it still says it was clipped, and how to ask for more',
                 'clipped at' in short and 'subsection' in short,
                 short[-70:])
    report.check('text inside the limit is passed through untouched',
                 'clipped at' not in intact and 'under the limit.' in intact)
    report.check('the index names every document, at either level',
                 all(name in docsmod.index(level)
                     for level in (detail.TERSE, detail.FULL)
                     for name in docsmod.paths()))



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

    # Headings are read out of the document rather than spelled here. Twice
    # now, editing MODELS.md failed this test for renaming a section - which
    # says nothing about the tool, and is exactly what it is not for.
    import re as _re
    text = io.open(os.path.join(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))), 'docs', 'MODELS.md'),
        encoding='utf-8').read()
    heads = [(len(m.group(1)), m.group(2).strip())
             for m in _re.finditer(r'^(#{2,3}) (.+)$', text, _re.M)]
    # The first chapter that actually has a subsection - the first chapter
    # in the file need not, and asserting a parent carries children against
    # one with none proves nothing.
    at = next((i for i, (lvl, _) in enumerate(heads)
               if lvl == 2 and i + 1 < len(heads) and heads[i + 1][0] == 3),
              None)
    if at is None:                    # a document with no subsections at all
        at = next(i for i, (lvl, _) in enumerate(heads) if lvl == 2)
        child = None
    else:
        child = heads[at + 1][1]
    chapter = heads[at][1]
    after = next((h for lvl, h in heads[at + 1:] if lvl == 2), None)

    outline = docmod.docs(doc='MODELS')
    report.check('one document lists its own headings',
                 chapter in outline and (child or chapter) in outline,
                 '%r / %r' % (chapter, child))

    deep = next((h for lvl, h in heads if lvl == 3), chapter)
    body = docmod.docs(doc='MODELS', section=deep)
    report.check('a section returns its text', len(body) > len(deep) + 40)
    later = next((h for lvl, h in heads[[i for i, (_, x)
                                         in enumerate(heads)
                                         if x == deep][0] + 1:] if lvl == 3),
                 None)
    report.check('a section stops at the next heading of its level',
                 later is None or later not in body, '%r' % later)

    # A parent section keeps its children: asking for the chapter should not
    # silently return only its first paragraph.
    parent = docmod.docs(doc='MODELS', section=chapter)
    report.check('a parent section carries its subsections',
                 child is None or child in parent or 'clipped' in parent)
    report.check('and stops at the next chapter',
                 after is None or after not in parent, '%r' % after)

    report.check('heading matching is loose enough to be usable',
                 docmod.docs(doc='MODELS', section=deep.lower()) == body)

    # The AFE number is the one a weak model invents. It has to be findable.
    hits = docmod.docs(find='25.00')
    report.check('search reaches the AFE-off number', 'FINDINGS' in hits
                 or 'HARDWARE' in hits or 'CLAUDE' in hits)
    report.check('search says where it found it, not just that it did',
                 len(hits.splitlines()) >= 2)
    # A hit without its chapter can say the opposite of what the document says:
    # the phase V entry that matches lives under "Ruled Out", and a model that
    # cannot see that reports a dead end as the explanation. Measured.
    refuted = next(head for _, head, _ in docmod._headings(
        docmod._read(docmod.paths()['FINDINGS'])) if 'Ruled Out' in head)
    body = docmod.docs(doc='FINDINGS', section=refuted).splitlines()[2:]
    phrase = next((w.lstrip('*- ') for line in body for w in [line.strip()]
                   if len(w) > 30 and not w.startswith('#')), None)
    report.check('a search hit carries the chapter it sits under',
                 phrase is None
                 or refuted in docmod.docs(find=phrase[:40]),
                 repr(phrase and phrase[:40]))

    report.check('search is capped rather than dumping a document',
                 len(docmod.docs(find='the').splitlines()) <= docmod.FIND_HITS + 1)

    report.check('no section comes back longer than the clip allows',
                 all(len(docmod.docs(doc=name, section=head)) <=
                     docmod.CLIP + 200
                     for name in docmod.paths()
                     for _, head, _ in docmod._headings(
                         docmod._read(docmod.paths()[name]))))

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

    # Measured at the prompt: "forklara pa japanska..." named a language
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

    #...and it is that turn's request, not a new session language: the next
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

    #...and the model is told the same thing, for the phrasing the host
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

    #...but a request with a question attached is still the model's turn,
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

    # Measured live: the "call the tool now" nudge goes into history with
    # role=='user', and its English words then flipped the language on the
    # next trim(). `prompt_history` is appended once at the top of ask() so
    # nothing added later in the turn can reach it.
    #
    # And from prompt_io.tmp: a call, its table, then `A:` with nothing after
    # it - a blank line under the table, past the stale gate because that same
    # call had just set last_channels. A turn never ends on silence.
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
    # The rule, not the sentence: board_info owns the map, and "list" is
    # named as the word that decides nothing. Pinning the old wording
    # instead failed the rewrite that fixed "ge mig en lista over de
    # analoga vardena", which is the opposite of what this is for.
    report.check('it names board_info as the map, and disowns "list"',
                 'board_info' in debug.SYSTEM
                 and 'never "list"' in debug.SYSTEM,
                 [l for l in debug.SYSTEM.splitlines() if 'board_info' in l])
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


ROSTER = (
    (test_board_tools, ('tools', 'board')),
    (test_corrections_are_reported, ('tools', 'prompt')),
    (test_debug, ('tools', 'reply')),
    (test_detail, ('tools', 'prompt')),
    (test_coerce, ('tools',)),
    (test_docs, ('tools', 'runner')),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
