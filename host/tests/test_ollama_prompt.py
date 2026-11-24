#!/usr/bin/env python3
"""SYSTEM, the per-turn hints, and what the model is told."""
import io
import json
import sys
import threading

from ollama_support import Scope, ScriptedModel, SimulatedSession, build, call, run_file, toolmod
from ollama_intent_kit import test_intent

def test_prompt(report):
    """\u00abrobot icon\u00bb Coaxial_63100> - the name written whole; the icon beside
    it turns while busy and repaints on state changes."""
    import time as clock
    from coaxial_ollama import spinner as spin

    # The robot ("\U0001F916") is absent from cp1252 - not wide, not
    # combining, just absent - so the fixture for the normal-case tests below
    # is UTF-8, as this bench's console must be to show it.
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

    # The bar once spun in place of the name's '1'.
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

    # A live session left the bar frozen on '/' or '-' in scrollback: stop()
    # reset the icon and colour but not the frame the ticker had drifted to
    # while the operator was still typing the question.
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

    # ---- "one row up" is recomputed as _trace() prints, not fixed at busy() -
    # A tick once landed inside a channel table mid-print.
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
    # Chat._trace() output while the bar ticks for the busy phase is the race
    # this lock is for: proved by holding it and showing a tick cannot pass,
    # not by timing a race.
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
    # The prompt group once spliced into an answer mid-sentence:
    # print(answer, file=face.out) takes no lock, so nothing stopped a tick
    # writing at the same time.
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
                 and spin.ICON_ERROR == '\u274c'
                 and len(spin.ICON_WAIT) == len(spin.ICON_ERROR) == 1)
    # No selector anywhere: a forced-colour glyph can sit at a different
    # advance width than a native one, which reads as uneven spacing beside the
    # others.
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
    # busy()/stop() change state but paint nothing further.
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

    runner, _, _ = build(task, [call('link', op='state'),
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

    # Told to turn the AFE off, then asked in a later, unrelated turn for a
    # reading, gemma4:12b turned it back on to "serve" the reading (measured):
    # what the system prompt says never to do.
    runner, _, _ = build(task, [])
    toolbox = runner.toolbox
    toolbox.afe_mentioned = False
    refused = str(toolbox.call('afe_power', {'action': 'on'}))
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

    # One line in, in the operator's own language.
    from coaxial_ollama import language
    hello = language.greeting('gemma4:12b', 'Swedish')
    report.check('the prompt opens with one line, in the host language',
                 hello.count(chr(10)) == 0 and 'gemma4:12b' in hello
                 and '/help' in hello and 'expert' in hello, hello)
    report.check('a language with no greeting written falls back to English',
                 language.greeting('x', 'Turkish')
                 == language.greeting('x', 'English'))
    report.check('and this host resolves to a language it can print',
                 language.system_language() in language.LANGUAGE_NAMES,
                 language.system_language())
    report.check('/help carries what the banner no longer does',
                 all(part in (talk.command('/help') or '')
                     for part in ('gemma4:12b', 'terse', 'tok/turn',
                                  'analog_read', '/q')),
                 (talk.command('/help') or '').splitlines()[2][:60])



# ---- a prompt that does not fit is a prompt that is not sent ---------------

def test_context_budget(report):
    """What keeps a conversation inside num_ctx, which is the number the
    daemon actually allocates for.
    """
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

    # One message larger than the whole window: a pasted log, an attached file.
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

ROSTER = (
    (test_prompt, ('prompt', 'tools')),
    (test_policy, ('prompt', 'board')),
    (test_identity, ('prompt',)),
    (test_context_budget, ('prompt',)),
    (test_intent, ('prompt', 'tools')),
)


if __name__ == '__main__':
    sys.exit(run_file(ROSTER))
