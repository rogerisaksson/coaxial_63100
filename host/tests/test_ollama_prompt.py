#!/usr/bin/env python3
"""SYSTEM, the per-turn hints, and what the model is told.

Split out of test_ollama.py, which had grown to 5,496 lines and 733 checks in
one file - a third of every check this tree has, and the reason a coverage
tier could not be asked for at any useful resolution. One subject per file
now, so a tier buys them separately and a reader opens the one they meant.

Run from the host directory:  python tests/test_ollama_prompt.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (Scope, ScriptedModel, SimulatedSession, 
    build, call, io, json, safe_head, simulated, sys, threading, toolmod, 
    types)   # noqa: E402

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

    screen = Tty()
    face = spin.prompt(text, screen, tick=10)
    written = screen.getvalue()
    face.stop(True)

    report.check('the name is written whole - nothing turns inside it',
                 written == '%s%s%s%s%s%s%s>'
                 % (spin.OPEN, spin.ROBOT, spin.ICON_WAIT, spin.CLOSE,
                    spin.GREEN, text, spin.RESET),
                 ascii(written))
    report.check('and any name at all comes through unsplit',
                 text in written and 'Coaxial_63100' in written)

    # The old design span the '1' in the name. It wrote "Coaxial 63-00" and
    # "Coaxial 63\00" into the operator's transcript, twice, and both times
    # read as a corrupted board name. Nothing may touch the name now.
    plain = spin.prompt('no-ones-here', Tty(), tick=10)
    report.check('a name with no digit is not a special case any more',
                 plain.out.real.getvalue().endswith(
                     '%s%s%sno-ones-here%s>'
                     % (spin.ICON_WAIT, spin.CLOSE, spin.GREEN, spin.RESET)),
                 ascii(plain.out.real.getvalue()))
    plain.stop(True)

    down = spin.prompt(text, Tty(), tick=10, ok=False)
    down_written = down.out.real.getvalue()
    report.check('a dead link starts with the error icon, red, not waiting',
                 down_written == '%s%s%s%s%s%s%s>'
                 % (spin.OPEN, spin.ROBOT, spin.ICON_ERROR, spin.CLOSE,
                    spin.RED, text, spin.RESET),
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
                 face.color == spin.YELLOW and face.busy_now
                 and face.rows_up == 1)
    added_busy = after_busy[len(before):]
    report.check('busy() repaints the whole group at once, from column 1 - '
                 'not a jump to a computed column',
                 added_busy == spin.SAVE + (spin.UP % 1) + '\r'
                 + face._prefix() + spin.RESTORE,
                 ascii(added_busy))
    report.check('and the rewritten prefix turns the icon and goes yellow '
                 "- the glyph's own shape is the signal, not a colour on "
                 'top of it',
                 any(f in face._prefix() for f in spin.SPIN)
                 and spin.YELLOW in face._prefix()
                 and spin.ICON_WAIT not in face._prefix())

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
    expected = '%s%s%s%s%s%s%s' % (spin.OPEN, spin.ROBOT, spin.ICON_WAIT,
                                   spin.CLOSE, spin.GREEN, text, spin.RESET)
    report.check('stop() puts a state icon back, not whatever frame the '
                 'ticker had drifted to',
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
    # No selector anywhere: a forced-colour glyph can sit at a different
    # advance width than a native one, which reads as uneven spacing
    # beside the others. The gear that used to mark "busy" needed one;
    # the moon frames that replaced it do not, and they turn.
    report.check('no glyph here is forced into colour with a selector',
                 not any(chr(0xFE0F) in g for g in
                         (spin.ROBOT, spin.ICON_WAIT, spin.ICON_ERROR)
                         + spin.SPIN))
    report.check('none of that matters for positioning any more - every '
                 'repaint rewrites from column 1, not a computed one',
                 not hasattr(face, 'icon_column')
                 and not hasattr(face, 'bar_column'))
    report.check('every spin frame is one glyph, so the group keeps its '
                 'width while it turns',
                 all(len(f) == 1 for f in spin.SPIN)
                 and all(len(f) == 1 and ord(f) < 128
                         for f in spin.SPIN_FALLBACK))
    report.check('and the turning frames are the same kind of glyph as the '
                 'icon they replace - no width step mid-tick',
                 len(spin.SPIN[0]) == len(spin.ICON_WAIT))
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
    #.encoding either, so it gets the ASCII fallback same as Ascii() does,
    # brackets included.
    piped = io.StringIO()
    quiet = spin.prompt(text, piped, tick=10)
    before_pipe = piped.getvalue()
    quiet.busy()
    quiet.stop(False)
    report.check('a redirected prompt is static and escape-free',
                 before_pipe == '%s%s%s%s%s%s%s>'
                 % (spin.OPEN_FALLBACK, spin.ROBOT_FALLBACK,
                    spin.ICON_WAIT_FALLBACK, spin.CLOSE_FALLBACK,
                    spin.GREEN, text, spin.RESET),
                 repr(before_pipe))
    report.check('and busy()/stop() on a redirected stream paint nothing '
                 'further',
                 piped.getvalue() == before_pipe)
    report.check('no background thread was even started for it',
                 quiet.thread is None)

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

def test_intent(r):
    """The intent pass: classify first, answer second.

    What is checked is the compiler's frame, not the model's judgement -
    every way the extra call can fail has to leave the turn exactly as it
    was before the pass existed, because a hint that is wrong and insistent
    is worse than no hint.
    """
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_mcp import render
    from coaxial_ollama import debug
    from coaxial_ollama import intent

    j = json.dumps
    for name, reply, want in (
            ('an intent and a kind', j({'intent': 'read', 'kind': 'analog',
                                        'why': 'values'}),
             ('read', 'analog')),
            ('a missing kind is none', j({'intent': 'words'}),
             ('words', 'none')),
            ('a kind outside the enum falls back to none',
             j({'intent': 'map', 'kind': 'sideways'}), ('map', 'none')),
            ('an intent this file has no name for',
             j({'intent': 'frobnicate', 'kind': 'analog'}), (None, None)),
            ('not JSON at all', 'I think they want a reading', (None, None)),
            ('nothing at all', j({}), (None, None)),
    ):
        got, kind, why = intent.parse(reply)
        r.check('parse: %s' % name, (got, kind) == want,
                '%r / %r / %r' % (got, kind, why))

    r.check('every intent has a tool or a reason not to',
            all(intent.tool_for(name, 'none') is not None
                or name in ('words', 'control') for name in intent.INTENTS))
    r.check('every intent has a phrase the hint can say',
            set(intent.SAYS) == set(intent.INTENTS))
    # A console that cannot carry the diacritics is this bench's normal
    # case, not an edge one: measured through board_prompt, "matvardena"
    # classified as `map` - "the inputs" - and put the channel map on screen
    # for a question about values. The classifier is told to read Swedish
    # both ways; this is what says the line is still there.
    r.check('the classifier is told Swedish may arrive folded to ASCII',
            'diacritics' in intent.ASK
            and 'matvardena' in intent.ASK and 'vardena' in intent.ASK)

    r.check('the schema offers exactly the intents this file has',
            intent.SCHEMA['properties']['intent']['enum']
            == sorted(intent.INTENTS))
    r.check('and exactly the kinds',
            intent.SCHEMA['properties']['kind']['enum'] == list(intent.KINDS))

    # The axis that caused this module: same intent, different kind, and the
    # kind is what picks the tool.
    r.check('read+analog is analog_read',
            intent.tool_for('read', 'analog') == 'analog_read')
    r.check('read+digital is digital_read',
            intent.tool_for('read', 'digital') == 'digital_read')
    r.check('read+both names both calls',
            intent.tool_for('read', 'both') == 'analog_read and digital_read')
    r.check('map is board_info whatever the kind',
            {intent.tool_for('map', k) for k in intent.KINDS} == {'board_info'})
    r.check('words names no tool at all',
            intent.tool_for('words', 'none') is None)

    # The hint is now for the intents the loop does *not* plan. A planned
    # question makes its calls before the model is asked anything, so there
    # is no tool to name and nothing to hint at.
    r.check('an intent that plans its calls needs no hint at all',
            all(intent.hint(name, kind) == ''
                or 'no board call' in intent.hint(name, kind)
                for name in intent.INTENTS for kind in intent.KINDS
                if intent.plan(name, kind)),
            repr(intent.hint('read', 'analog')))
    r.check('and one that plans nothing still gets its line',
            'afe_power' in intent.hint('power', 'none'))

    r.check('a map question compiles to the map call',
            intent.plan('map', 'analog') == (('board_info',
                                              {'kind': 'analog'}),))
    r.check('a read compiles to the read for its kind',
            intent.plan('read', 'digital') == (('digital_read', {}),))
    r.check('and "both" compiles to both calls, in order',
            [n for n, _ in intent.plan('read', 'both')]
            == ['analog_read', 'digital_read'])
    r.check('words plans nothing - there is no call to make',
            intent.plan('words', 'none') == ())
    r.check('and neither does control',
            intent.plan('control', 'none') == ())
    r.check('every planned call names a tool the toolbox has',
            all(n in debug.SETS['read'] or n in debug.SETS['code']
                for name in intent.INTENTS for kind in intent.KINDS
                for n, _ in intent.plan(name, kind)),
            repr(sorted({n for name in intent.INTENTS
                         for kind in intent.KINDS
                         for n, _ in intent.plan(name, kind)})))
    r.check('and for words it says no call is needed',
            'no board call' in intent.hint('words', 'none')
            and 'board_info' not in intent.hint('words', 'none'))
    r.check('no intent means no hint at all',
            intent.hint(None, 'none') == '')

    # The persistence rule, pinned. ollama keys a loaded runner on num_ctx,
    # so a second client asking for the same tag at a different window
    # unloads and reloads the weights - measured at once per question when
    # this module built its own Ollama. The compile must go through the
    # turn's own client, with only per-request fields overridden.
    class Recorder(object):
        model = 'gemma4:12b'
        options = {'num_ctx': 8192, 'temperature': 0.0}

        def __init__(self):
            self.seen = []

        def chat(self, messages, tools=None, **kw):
            self.seen.append((messages, tools, kw))
            return {'content': json.dumps({'intent': 'read',
                                           'kind': 'analog'})}

    rec = Recorder()
    built = []
    from coaxial_ollama import client as clientmod
    was = clientmod.Ollama

    class Loud(object):
        def __init__(self, *a, **kw):
            built.append((a, kw))
            raise AssertionError('a second client is a model reload')
    clientmod.Ollama = Loud
    try:
        got, kind, _ = intent.compile_intent(rec, 'ge mig de analoga värdena')
    finally:
        clientmod.Ollama = was
    r.check('the compile builds no second client', not built, repr(built))
    r.check("it goes through the turn's own client", len(rec.seen) == 1)
    r.check('and it still classified', (got, kind) == ('read', 'analog'))
    _, tools, kw = rec.seen[0]
    r.check('with the schema, no thinking and a small budget',
            kw.get('fmt') is intent.SCHEMA and kw.get('think') is False
            and 0 < kw.get('num_predict', 0) <= 200, repr(kw))
    r.check('and no tools - it classifies, it does not call',
            not tools)
    r.check('the window is never touched',
            rec.options == {'num_ctx': 8192, 'temperature': 0.0})

    # An unreachable daemon must leave the turn untouched, not raise into it.
    class Dead(object):
        model = 'gemma4:12b'

        def chat(self, *a, **k):
            raise OSError('connection refused')

    real = sys.modules.get('coaxial_ollama.client')
    broken = types.ModuleType('coaxial_ollama.client')

    def explode(*a, **k):
        raise OSError('connection refused')
    broken.Ollama = explode
    sys.modules['coaxial_ollama.client'] = broken
    try:
        got, kind, why = intent.compile_intent(Dead(), 'read the NTC')
    finally:
        if real is None:
            sys.modules.pop('coaxial_ollama.client', None)
        else:
            sys.modules['coaxial_ollama.client'] = real
    r.check('ollama unreachable compiles to nothing, and says why',
            got is None and 'connection refused' in why, repr(why))
    r.check('an empty question is not sent anywhere',
            intent.compile_intent(Dead(), '   ')[0] is None)

    # The executor. What must hold is that the model is asked *after* the
    # calls have run and is offered no tools at all - the turn has no choice
    # left in it to get wrong.
    class Narrator(object):
        model = 'gemma4:12b'
        options = {'num_ctx': 8192}

        def __init__(self, say='NTC 38.5 C.'):
            self.say, self.seen = say, []

        def chat(self, messages, tools=None, **kw):
            self.seen.append((messages, tools))
            return {'content': self.say}

        def usage(self):
            return {'prompt_tokens': 0, 'eval_tokens': 0}

    def planned(say):
        screen = io.StringIO()
        talk = debug.Chat(Narrator(say), toolmod.Toolbox(Sim(), scope=Scope()),
                          tools='code', out=screen, session_language='Swedish')
        talk.compile_intent = True
        was = intent.compile_intent
        intent.compile_intent = lambda client, text: ('read', 'analog', '')
        try:
            answer = talk.ask('ge mig de analoga värdena')
        finally:
            intent.compile_intent = was
        return talk, screen.getvalue(), answer

    talk, screen, answer = planned('NTC läser 38,5 C.')
    ran = [n for n, _ in talk.toolbox.log]
    r.check('a planned question makes its own calls',
            ran == ['analog_read'], ', '.join(ran) or 'none')
    r.check('and the model is offered no tools on that turn',
            all(tools in (None, [], ()) for _, tools in talk.client.seen),
            repr([t for _, t in talk.client.seen]))
    r.check('it is asked once, not in a loop', len(talk.client.seen) == 1)
    r.check('the rows reach the screen', render.ANALOG_HEAD in screen)
    r.check('and the answer is the sentence, not the rows',
            answer == 'NTC läser 38,5 C.', repr(answer[:40]))

    # The retype is the failure this replaced, and it still has to be caught:
    # measured on screen, the whole table written out again as prose under
    # itself, comma decimals and all.
    # Built from the stand-in's table, not typed out: it was typed out and
    # two supply senses were added, so a sentence naming seven of nine
    # stopped being a restatement and the check failed on a working filter.
    _, _, retyped = planned(
        'De analoga värdena är: %s.'
        % ', '.join(c['signal'].replace(' ', '')
                    for c in simulated.CHANNELS))
    r.check('a sentence that just names every channel is silenced',
            retyped == '', repr(retyped[:40]))

    # And a call that failed on the wire never reaches a narrate turn.
    from coaxial.errors import RigError

    class Broken(object):
        simulated = False
        port, baud, unit, bus = 'COM9', 115200, 1, None

        @property
        def board(self):
            raise RigError('cable pulled')

    screen = io.StringIO()
    down = debug.Chat(Narrator(), toolmod.Toolbox(Broken(), scope=Scope()),
                      tools='code', out=screen, session_language='Swedish')
    down.compile_intent = True
    was = intent.compile_intent
    intent.compile_intent = lambda client, text: ('read', 'analog', '')
    try:
        said = down.ask('ge mig de analoga värdena')
    finally:
        intent.compile_intent = was
    r.check('a dead link is reported, not narrated',
            'ERR' in said or 'link is down' in said, repr(said[:60]))
    r.check('and the model was never asked about it',
            not down.client.seen, repr(down.client.seen))

    # And the turn itself: off by default, hint reaches the system message
    # when it is on, and a failed compile changes nothing.
    chat = debug.Chat(ScriptedModel([]),
                      toolmod.Toolbox(Sim(), scope=Scope()),
                      out=io.StringIO())
    r.check('the pass is off unless something turns it on',
            chat.compile_intent is False)
    r.check('off means no second call and no hint',
            chat._compile('ge mig de analoga värdena') == '')

    chat.compile_intent = True
    chat.client = Dead()
    sys.modules['coaxial_ollama.client'] = broken
    try:
        # Through the swap, not just past it: without this the check reaches
        # a running daemon and loads 7.6 GB inside an offline suite.
        blank = chat._compile('ge mig de analoga värdena')
    finally:
        if real is None:
            sys.modules.pop('coaxial_ollama.client', None)
        else:
            sys.modules['coaxial_ollama.client'] = real
    r.check('on, but unreachable, is still no hint', blank == '',
            repr(chat._intent_why))

    # The backstop, offline. A hint saying "this question does not need a
    # reading" did not hold on the real model - measured, board_info followed
    # by analog_read on a map question, one turn after a reading. The loop
    # answers the second call itself, from the fact that the tool the intent
    # named has already succeeded this turn.
    screen = io.StringIO()
    both = debug.Chat(ScriptedModel([
        call('board_info'), call('analog_read'),
        {'role': 'assistant', 'content': 'Kanalerna listade.'},
    ]), toolmod.Toolbox(Sim(), scope=Scope()), out=screen)
    both.compile_intent = True
    was_compile = intent.compile_intent
    intent.compile_intent = lambda client, text: ('map', 'analog', 'stub')
    try:
        both.ask('ge mig en lista på alla analoga kanalerna')
    finally:
        intent.compile_intent = was_compile
    ran = [name for name, _ in both.toolbox.log]
    r.check('the off-axis call never reaches the board',
            ran.count('analog_read') == 0 and 'board_info' in ran,
            ', '.join(ran) or 'none')
    r.check('and the screen shows the map once, not a reading under it',
            screen.getvalue().count(render.ANALOG_HEAD) == 0,
            safe_head(screen.getvalue()))

    # ...and the same loop with no compiled intent must not block anything:
    # every failure in intent.py returns None, and None means old behaviour.
    plain = debug.Chat(ScriptedModel([
        call('board_info'), call('analog_read'),
        {'role': 'assistant', 'content': 'Klart.'},
    ]), toolmod.Toolbox(Sim(), scope=Scope()), out=io.StringIO())
    plain.ask('ge mig kanalerna och deras värden')
    did = [name for name, _ in plain.toolbox.log]
    r.check('with no intent compiled, both calls run as before',
            'board_info' in did and 'analog_read' in did,
            ', '.join(did) or 'none')

    chat.compile_intent = False
    chat.intent = intent.hint('read', 'digital')
    sent = chat.trim()
    r.check('the compiled hint reaches the system message',
            'digital_read' in sent[0]['content'], sent[0]['content'][-160:])
    r.check('and the question itself is never rewritten',
            all('asking for' not in (m.get('content') or '')
                for m in sent[1:]))


ROSTER = (
    (test_prompt, ('prompt', 'tools')),
    (test_policy, ('prompt', 'board')),
    (test_identity, ('prompt',)),
    (test_context_budget, ('prompt',)),
    (test_intent, ('prompt', 'tools')),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
