#!/usr/bin/env python3
"""The real model, the real board, one session that changes language.

Every other suite here scripts the model, which proves the host and nothing
about the model. This proves the three things a script cannot:

  * it goes to the board when the question is a reading;
  * it does not when the question is a description;
  * the answer comes back in the language the session is locked to, across a
    switch, in both directions.

What is asserted is *that* `analog_read` was called, never what it returned -
invariant 10. There are no expected values here and there is nothing for one
to be compared against.

Needs ollama. The board is probed and a silent port falls back to the
stand-in, so this runs with or without a cable - the model is the real one
either way, and the model is what is under test. Minutes, not seconds: a
model load plus a turn per question.

Run from the host directory:

    python tests/test_live_model.py
    python tests/test_live_model.py --simulated   # skip the probe
    python tools/run_tests.py --live
"""
import argparse
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial_ollama import debug                            # noqa: E402
from coaxial_ollama import language                         # noqa: E402
from coaxial_ollama import tools as toolmod                 # noqa: E402
from coaxial_ollama.client import Ollama                    # noqa: E402
from coaxial_ollama.sandbox import Scope                    # noqa: E402

# The one tool that reaches the board for a reading. `board_info` and `link`
# reach it too, but a question about what a thing *is* may legitimately want
# the channel map - only a reading is the thing being ruled in and out here.
READING = 'analog_read'

# Whatever this machine is set to would otherwise decide the first turn's
# language, and a suite that passes on a Swedish desktop and fails on an
# English one is testing the desktop. Locked here.
START = 'Swedish'

# question -> the tool it must call, and the tools it must not.
#
# Four questions crossed two ways: list or read, analog or digital. Every
# one of these is a transcript from the bench, and every wrong cell was one
# the operator saw before this suite existed:
#
#   "lista alla digitala kanaler"  -> analog_read, and a full analog table
#   "lista alla analoga kanaler"   -> the map, then the model typing it out
#   "ge mig alla digitala varden"  -> the digital list, no values
#
# `must_not` is the half that matters. Calling nothing is a different
# failure from calling the wrong thing, and the answer being right by luck
# after the wrong call is not this suite passing.
TOOL_CHOICE = (
    # question, must call, must not call
    ('ge mig en lista över alla analoga kanaler', 'board_info',
     ('analog_read', 'digital_read')),
    ('ge mig en lista över alla digitala kanaler', 'board_info',
     ('analog_read', 'digital_read')),
    ('list every analog channel', 'board_info',
     ('analog_read', 'digital_read')),
    ('list every digital channel', 'board_info',
     ('analog_read', 'digital_read')),

    ('läs alla analoga kanaler', 'analog_read', ('digital_read',)),
    ('read every analog channel', 'analog_read', ('digital_read',)),
    ('vad läser NTC:n?', 'analog_read', ('digital_read',)),

    # Every one of these is a phrasing off the bench, kept verbatim rather
    # than paraphrased into one. The matrix had the concept - "ge mig alla
    # digitala varden" - and the model still fetched the analog table for
    # "ge mig vardena fran de digitala kanalerna", on top of the
    # digital_read it had already made correctly. A concept is not a
    # phrasing, and the model answers phrasings.
    ('ge mig alla digitala värden', 'digital_read', ('analog_read',)),
    ('ge mig värdena från de digitala kanalerna', 'digital_read',
     ('analog_read',)),
    ('vad har de digitala kanalerna för värden?', 'digital_read',
     ('analog_read',)),
    ('visa digitala värden', 'digital_read', ('analog_read',)),
    ('read the digital values', 'digital_read', ('analog_read',)),
    ('give me the values from the digital channels', 'digital_read',
     ('analog_read',)),
    # One named pin. digital_read answers it; so would gpio_pin, and the
    # matrix accepts either - board_info answers neither, and that is what
    # it reached for before the values line said "a named pin included".
    ('vilket värde har PB2 nu?', ('digital_read', 'gpio_pin'),
     ('analog_read',)),

    # The mirror, so a fix for one kind cannot quietly break the other.
    ('ge mig värdena från de analoga kanalerna', 'analog_read',
     ('digital_read',)),
    ('vad har de analoga kanalerna för värden?', 'analog_read',
     ('digital_read',)),

    # The bus. One board is a bus of one; a machine is several of it, and
    # "which device" is a question board_info cannot answer.
    ('ge mig en lista på alla enheter du kan kommunicera med', 'devices',
     ('analog_read', 'digital_read')),
    ('list every device you can talk to', 'devices',
     ('analog_read', 'digital_read')),
    ('kommunicera med höger knä', 'devices',
     ('analog_read', 'digital_read')),

    # What a thing IS. board_info is allowed: describing this board from
    # its own map beats describing it from training, and the earlier
    # "no board call" here was the wrong bar - measured, gemma4:12b and
    # qwen2.5:14b both reached for it, which is two families agreeing the
    # expectation was wrong rather than two models being wrong.
    #
    # What must not happen is a measurement, or the front end being
    # switched, or the link being diagnosed. None of those describes
    # anything.
    ('beskriv hårdvaran i detta projektet för en novis', None,
     ('analog_read', 'digital_read', 'afe_power', 'link_diagnose')),
    ('what is this project about', None,
     ('analog_read', 'digital_read', 'afe_power', 'link_diagnose')),
)


# question, must it reach the board, what language the answer is in
TURNS = (
    ('läs NTC:n och DC-länken', True, 'Swedish'),
    ('beskriv hårdvaran i detta projektet för en novis', False, 'Swedish'),
    ('byt språk till engelska', False, 'English'),
    ('read the NTC', True, 'English'),
    ('what is this project about', False, 'English'),
    ('byt språk till svenska', False, 'Swedish'),
)


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


def safe(text, limit=60):
    """One line of a model's answer that any console can print.

    The suite reports in the operator's console encoding, which is cp1252
    here; an answer that is fine on the wire is a UnicodeEncodeError in the
    detail column, and losing the tally to a glyph is the failure this whole
    package keeps running into.
    """
    flat = ' '.join((text or '').split())[:limit]
    encoding = getattr(sys.stdout, 'encoding', None) or 'ascii'
    return flat.encode(encoding, 'replace').decode(encoding, 'replace')


def _twice(answer, results):
    """The names a tool printed this turn, named again by the answer.

    Returns them when the answer is a restatement and '' when it says
    something of its own. The host silences a retype, so a non-empty
    return here means one reached the operator - which is every
    duplication ever reported from this bench.

    Read off the tool results rather than the screen, so it holds with the
    trace off as well.
    """
    from coaxial_ollama import replies

    if not (answer or '').strip():
        return ''
    # Alternatives, not a union: the answer names a row one way, and the
    # union would want every column's name present at once. Measured, the
    # union missing it - "AFE_ON ar 1 och nFAULT ar 0" under a trace whose
    # pins are PB2 and PE15.
    sets = []
    for text in results:
        for pattern in (replies.READING_ROW, replies.MAP_ROW,
                        replies.DIGITAL_ROW, replies.DIGITAL_SIGNAL):
            names = {m.lower() for m in pattern.findall(str(text))}
            if len(names) >= 2:
                sets.append(names)
    for names in sets:
        if replies.is_retype(answer, names, minimum=2):
            return ', '.join(sorted(names))
    return ''


def build(model, port, simulated):
    """(session, chat, real). `simulated=False` probes and falls back."""
    from coaxial_mcp.session import open_session
    session, found = open_session(port, 115200, 1,
                                  simulated=True if simulated else None)
    # Held for the whole run, not unloaded after every request. keep_alive=0
    # is ollama's "hand the VRAM back now", so this suite was reloading 7.6
    # GB for each of its twenty-six questions and spending most of its wall
    # time on that. Released once, in main()'s finally - the same bargain
    # the prompt loop makes, for the same reason.
    client = Ollama(model, keep_alive='30m')
    toolbox = toolmod.Toolbox(session, scope=Scope())
    # `read` rather than the default set: the fewer tools in the schema, the
    # less this measures the model's taste in tools it was never going to
    # need. out is a sink - the trace is noise between PASS lines.
    # quiet=False with the trace pointed at a sink: `out` is what keeps the
    # rows off this suite's own screen, and --quiet is a different mode -
    # it makes the host substitute a silenced block for the answer, which
    # is right at a prompt with no trace and wrong here. Configured the way
    # the operator runs it, or the duplication check measures the mode.
    chat = debug.Chat(client, toolbox, tools='read', quiet=False,
                      out=io.StringIO(), session_language=START)
    return session, chat, found


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('-m', '--model', default='gemma4:12b')
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--sections', default='all',
                        help='tools|language|all. A model load plus a turn '
                             'per question is minutes, and a change to a '
                             'tool description has nothing to do with the '
                             'language lock - run the half that can have '
                             'broken.')
    parser.add_argument('--simulated', action='store_true',
                        help='skip the probe and take the stand-in. Without '
                             'it the port is probed and a silent one falls '
                             'back to the stand-in anyway - the model is the '
                             'real one either way, which is what this tests')
    args = parser.parse_args(argv)

    session, chat, found = build(args.model, args.port, args.simulated)
    report = Report()
    # Which board, said before the first PASS. The tool-choice checks below
    # hold either way - reaching analog_read is the model's decision, not the
    # board's - but "answered 38.53C" from a stand-in is an invented number,
    # and this suite never asserts on one. See invariant 10.
    print('-- %s, %s --'
          % (args.model, found.label if found.real else 'SIMULATED board'))
    try:
        # Tool choice first, and each question from a clean history: what is
        # under test is which tool this question reaches for, not which one
        # the last question left in view.
        # Every tool result this turn produced, for the duplication check.
        results = []
        real_call = chat.toolbox.call

        def recording(name, args):
            outcome = real_call(name, args)
            results.append(outcome)
            return outcome

        chat.toolbox.call = recording

        want = {w.strip() for w in args.sections.split(',') if w.strip()}
        if 'all' in want:
            want = {'tools', 'language'}

        if 'tools' in want:
            print(chr(10) + '-- which tool the question reaches for --')
            for question, must, must_not in TOOL_CHOICE:
                before = len(chat.toolbox.log)
                del results[:]
                answer = chat.ask(question)
                # `link` is dropped: _probe_link makes that call itself
                # when an answer comes back blank, so counting it would
                # measure the host's recovery rather than the model's
                # choice of tool. Measured as "no board call -> link, link,
                # link" on a question the model simply did not answer.
                called = [name for name, _ in chat.toolbox.log[before:]
                          if name != 'link']
                # prompt_history as well as history: trim() puts the last
                # five questions in the system message as "already tried in
                # this conversation", and by the third row this suite had
                # asked for the same list twice in two languages - measured,
                # the model then answered "list every analog channel" from
                # nothing at all, with no call, because it read the question
                # as one it had already done. A row here is one question,
                # not a conversation.
                chat.history = []
                chat.prompt_history = []
                chat.last_channels = None

                # "Answered" is not "wrote a sentence": the host silences a
                # retyped list on purpose, because the trace above it is the
                # answer. Measured, asserting the sentence: twelve of these
                # failed for doing exactly what they should. What must never
                # happen is the operator getting nothing at all - no words
                # and no tool output.
                report.check('%s -> the operator got something'
                             % safe(question, 40),
                             bool(answer.strip()) or bool(results),
                             safe(answer, 40) or '(the trace)')
                if must is None:
                    pass          # only must_not applies - see TOOL_CHOICE
                else:
                    wanted = (must,) if isinstance(must, str) else must
                    report.check('%s -> %s' % (safe(question, 40),
                                               '/'.join(wanted)),
                                 any(w in called for w in wanted),
                                 ', '.join(called) or 'no calls')
                wrong = [name for name in must_not if name in called]
                report.check('%s -> not %s' % (safe(question, 40),
                                               '/'.join(must_not)),
                             not wrong, ', '.join(wrong) or 'none')

                # And the answer is not the trace typed out again. Free: the
                # turn already ran. Every duplication reported from this bench
                # was one question putting the same list on screen twice, and
                # the host's backstop is what has to catch it whatever the
                # model writes - so it is checked on every question, not on
                # the ones a transcript happened to be pasted from.
                said_twice = _twice(answer, results)
                report.check('%s -> said once, not twice' % safe(question, 40),
                             not said_twice, said_twice or 'once')

        if 'language' in want:
            print(chr(10) + '-- language, and reading against describing --')
            for question, needs_board, expect in TURNS:
                before = len(chat.toolbox.log)
                spent = chat.client.usage()['eval_tokens']
                answer = chat.ask(question)
                called = [name for name, _ in chat.toolbox.log[before:]]
                chat.history = []          # every question starts from nothing

                print('\n-- %s --' % safe(question, 70))
                report.check('answered at all', bool(answer.strip()),
                             safe(answer))

                bare = language.bare_switch(question)
                if bare:
                    # No model turn at all, so there is nothing for it to get
                    # wrong: the word, and the lock, are the host's.
                    report.check('a bare switch costs no model tokens',
                                 chat.client.usage()['eval_tokens'] == spent,
                                 '%d eval' % (chat.client.usage()['eval_tokens']
                                              - spent))
                    report.check('answered with the one word for %s' % expect,
                                 answer == language.okay(expect), safe(answer))
                    report.check('and the session moved to %s' % expect,
                                 chat.language == expect, str(chat.language))
                    continue

                report.check('%s the board for a reading'
                             % ('reached' if needs_board
                                else 'did not reach'),
                             (READING in called) == needs_board,
                             ', '.join(called) or 'no calls')
                # detect() is the same judge the session prompt is built from, so
                # a disagreement here is the operator's screen disagreeing too.
                # None passes: measured against the stand-in, "las NTC:n och
                # DC-lanken" was answered "NTC: 25.00C DC-lanken: 39.075V" - terse
                # and no preamble, exactly what SYSTEM asks for, and not one word
                # for a stop-word list to score. An answer with no words in it
                # cannot be in the wrong language. Anything that does detect must
                # be right.
                spoke = language.detect(answer)
                report.check('answered in %s' % expect, spoke in (expect, None),
                             'no words to judge' if spoke is None
                             else '%s: %s' % (spoke, safe(answer, 44)))
                report.check('and the lock is still %s' % expect,
                             chat.language == expect, str(chat.language))
    finally:
        try:
            session.close()
        except Exception:                                     # noqa: BLE001
            pass
        # And the card back, once, however the run ended. A suite that
        # leaves 7.6 GB resident is the failure docs/MODELS.md measured at
        # 9.69 GB for 27 minutes at 1 % use.
        try:
            chat.client.unload()
        except Exception:                                     # noqa: BLE001
            pass

    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
