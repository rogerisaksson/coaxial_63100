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

    ('ge mig alla digitala värden', 'digital_read', ('analog_read',)),
    ('read the digital values', 'digital_read', ('analog_read',)),
    ('vilket värde har PB2 nu?', 'digital_read', ('analog_read',)),

    # Neither: a question about what a thing is, answered in words.
    ('beskriv hårdvaran i detta projektet för en novis', None,
     ('analog_read', 'digital_read')),
    ('what is this project about', None,
     ('analog_read', 'digital_read')),
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


def build(model, port, simulated):
    """(session, chat, real). `simulated=False` probes and falls back."""
    from coaxial_mcp.session import open_session
    session, found = open_session(port, 115200, 1,
                                  simulated=True if simulated else None)
    client = Ollama(model, keep_alive=0)
    toolbox = toolmod.Toolbox(session, scope=Scope())
    # `read` rather than the default set: the fewer tools in the schema, the
    # less this measures the model's taste in tools it was never going to
    # need. out is a sink - the trace is noise between PASS lines.
    chat = debug.Chat(client, toolbox, tools='read', quiet=True,
                      out=io.StringIO(), session_language=START)
    return session, chat, found


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('-m', '--model', default='gemma4:12b')
    parser.add_argument('--port', default='COM4')
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
        print(chr(10) + '-- which tool the question reaches for --')
        for question, must, must_not in TOOL_CHOICE:
            before = len(chat.toolbox.log)
            chat.ask(question)
            called = [name for name, _ in chat.toolbox.log[before:]]
            chat.history = []

            if must is None:
                report.check('%s -> no board call' % safe(question, 40),
                             not called, ', '.join(called) or 'none')
            else:
                report.check('%s -> %s' % (safe(question, 40), must),
                             must in called, ', '.join(called) or 'no calls')
            wrong = [name for name in must_not if name in called]
            report.check('%s -> not %s' % (safe(question, 40),
                                           '/'.join(must_not)),
                         not wrong, ', '.join(wrong) or 'none')

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

            report.check('%s the board' % ('reached' if needs_board
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

    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
