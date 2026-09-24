#!/usr/bin/env python3
"""The real model, the real board, one session that changes language."""
import argparse
import io
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial_ollama import debug, language, tools as toolmod  # noqa: E402
from coaxial_ollama.client import Ollama                    # noqa: E402
from coaxial_ollama.sandbox import Scope                    # noqa: E402

# The one tool that reaches the board for a reading.
READING = 'analog_read'

# Whatever this machine is set to would otherwise decide the first turn's
# language, and a suite that passes on a Swedish desktop and fails on an
# English one is testing the desktop.
START = 'Swedish'

# question -> the tool it must call, and the ones it must not.
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

    # Every one of these is a phrasing off the bench, kept verbatim rather than
    # paraphrased into one.
    ('ge mig alla digitala värden', 'digital_read', ('analog_read',)),
    ('ge mig värdena från de digitala kanalerna', 'digital_read',
     ('analog_read',)),
    ('vad har de digitala kanalerna för värden?', 'digital_read',
     ('analog_read',)),
    ('visa digitala värden', 'digital_read', ('analog_read',)),
    ('read the digital values', 'digital_read', ('analog_read',)),
    ('give me the values from the digital channels', 'digital_read',
     ('analog_read',)),
    # One named pin.
    ('vilket värde har PB2 nu?', ('digital_read', 'gpio_pin'),
     ('analog_read',)),

    # The mirror, so a fix for one kind cannot quietly break the other.
    ('ge mig en lista över de analoga värdena', 'analog_read',
     ('board_info',)),
    ('ge mig en lista över de digitala värdena', 'digital_read',
     ('board_info', 'analog_read')),
    ('list the analog values', 'analog_read', ('board_info',)),
    ('list the digital values', 'digital_read', ('board_info', 'analog_read')),
    ('ge mig värdena från de analoga kanalerna', 'analog_read',
     ('digital_read',)),
    ('vad har de analoga kanalerna för värden?', 'analog_read',
     ('digital_read',)),

    # What the board is made of.
    ('vilka subsystem har kortet?', 'board_info',
     ('analog_read', 'digital_read')),
    ('what subsystems does this board have', 'board_info',
     ('analog_read', 'digital_read')),
    ('vad består kortet av?', 'board_info',
     ('analog_read', 'digital_read')),

    # The IMU is a third kind of reading, and the noun that reaches it is its
    # own name.
    ('lista alla mätvärden från IMU:n', 'imu',
     ('analog_read', 'digital_read')),
    ('list every measurement from the IMU', 'imu',
     ('analog_read', 'digital_read')),
    ('vad säger IMU:n?', 'imu', ('analog_read', 'digital_read')),

    # Orientation is a picture, not a list of numbers, and it has its own tool.
    ('visa mig orienteringen på elektroniken', 'orientation',
     ('analog_read', 'digital_read')),
    ('show me how the board is oriented', 'orientation',
     ('analog_read', 'digital_read')),
    ('hur är kortet vänt?', 'orientation',
     ('analog_read', 'digital_read')),

    # The bus.
    ('ge mig en lista på alla enheter du kan kommunicera med', 'devices',
     ('analog_read', 'digital_read')),
    ('list every device you can talk to', 'devices',
     ('analog_read', 'digital_read')),
    ('kommunicera med höger knä', 'devices',
     ('analog_read', 'digital_read')),

    # The board is silent.
    ('kortet svarar inte, varför?', 'link_diagnose',
     ('analog_read', 'digital_read')),
    ('varför får jag inget svar från kortet?', 'link_diagnose',
     ('analog_read', 'digital_read')),
    ('the board is not answering - why?', 'link_diagnose',
     ('analog_read', 'digital_read')),

    # What a thing IS.
    ('beskriv hårdvaran i detta projektet för en novis', None,
     ('analog_read', 'digital_read', 'afe_power', 'link_diagnose')),
    ('what is this project about', None,
     ('analog_read', 'digital_read', 'afe_power', 'link_diagnose')),
)


# question, must it reach the board, what language the answer is in Two
# questions in one session, history kept - the shape an operator types.
SEQUENCES = (
    (('ge mig en lista på alla analoga kanalerna', 'board_info', 'analog_read'),
     ('ge mig en lista på alla analoga mätvärdena', 'analog_read',
      'board_info')),
    # Verbatim from a transcript, article and all: "den analoga mätvärdena" is
    # not "de", and the wording that was tested is not the wording that was
    # typed.
    (('ge mig en lista över de analoga kanalerna', 'board_info',
      'analog_read'),
     ('ge mig en lista över den analoga mätvärdena', 'analog_read',
      'board_info')),
    (('ge mig en lista över de analoga kanalerna', 'board_info',
      'analog_read'),
     ('ge mig en lista över de analoga värdena', 'analog_read',
      'board_info')),
    (('list every analog channel', 'board_info', 'analog_read'),
     ('list the analog values', 'analog_read', 'board_info')),
    (('ge mig alla analoga mätvärden', 'analog_read', 'board_info'),
     ('ge mig en lista på alla analoga kanalerna', 'board_info',
      'analog_read')),
    (('ge mig en lista på alla digitala kanalerna', 'board_info',
      'digital_read'),
     ('ge mig en lista på alla digitala mätvärdena', 'digital_read',
      'board_info')),
)

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
    """One line of a model's answer that any console can print."""
    flat = ' '.join((text or '').split())[:limit]
    encoding = getattr(sys.stdout, 'encoding', None) or 'ascii'
    return flat.encode(encoding, 'replace').decode(encoding, 'replace')


def _twice(answer, results):
    """The names a tool printed this turn, named again by the answer."""
    from coaxial_ollama import replies

    if not (answer or '').strip():
        return ''
    # Alternatives, not a union: the answer names a row one way, and the union
    # would want every column's name present at once.
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


WORDS = 300        # debug.py's --words default
TOOLS = 'code'     # and its --tools default, which is what board_chat runs


def matching(rows, needle, text):
    """The rows whose question contains `needle`, or all of them for None."""
    if not needle:
        return rows
    def flat(s):
        s = s.lower()
        for a, b in (('å', 'a'), ('ä', 'a'), ('ö', 'o')):
            s = s.replace(a, b)
        return s
    want = flat(needle)
    return [row for row in rows if want in flat(text(row))]


def build(model, port, simulated, compile_intent=True):
    """(session, chat, real). `simulated=False` probes and falls back."""
    from coaxial.comm.session import open_session
    session, found = open_session(port, 115200, 1,
                                  simulated=True if simulated else None)
    # Held for the whole run, not unloaded after every request.
    client = Ollama(model, keep_alive='30m', num_predict=WORDS, think=False)
    toolbox = toolmod.Toolbox(session, scope=Scope())
    # `read` rather than the default set: the fewer tools in the schema, the
    # less this measures the model's taste in tools it was never going to need.
    chat = debug.Chat(client, toolbox, tools=TOOLS, quiet=False,
                      out=io.StringIO(), session_language=START)
    # The intent pass is what the prompt loop runs with, so this measures it by
    # default.
    chat.compile_intent = compile_intent
    return session, chat, found


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('-m', '--model', default='gemma4:12b')
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--no-compile', action='store_true',
                        help='run the matrix without the intent pass, the '
                             'way the loop behaved before it existed')
    parser.add_argument('--match',
                        help='run only the rows whose question contains this '
                             'text. A model load plus a turn per row is '
                             'minutes; one changed rule is one row.')
    parser.add_argument('--release', action='store_true',
                        help='hand the model back when this run ends. Off '
                             'by default: the suite is run again a minute '
                             'later while something is being fixed, and a '
                             'reload costs more than the VRAM does.')
    parser.add_argument('--sections', default='all',
                        help='tools|sequence|language|all. A model load '
                             'plus a turn per question is minutes, and a tool '
                             'description has nothing to do with the language '
                             'lock - run the part that can have broken.')
    parser.add_argument('--simulated', action='store_true',
                        help='skip the probe and take the stand-in. Without '
                             'it the port is probed and a silent one falls '
                             'back to the stand-in anyway - the model is the '
                             'real one either way, which is what this tests')
    args = parser.parse_args(argv)

    release = args.release
    session, chat, found = build(args.model, args.port, args.simulated,
                                 not args.no_compile)
    report = Report()
    # Which board, said before the first PASS.
    print('-- %s, %s --'
          % (args.model, found.label if found.real else 'SIMULATED board'))
    try:
        # Tool choice first, and each question from a clean history: what is
        # under test is which tool this question reaches for, not which one the
        # last question left in view.
        results = []
        real_call = chat.toolbox.call

        def recording(name, args):
            outcome = real_call(name, args)
            results.append(outcome)
            return outcome

        chat.toolbox.call = recording

        want = {w.strip() for w in args.sections.split(',') if w.strip()}
        if 'all' in want:
            want = {'tools', 'sequence', 'language'}

        if 'tools' in want:
            print(chr(10) + '-- which tool the question reaches for --')
            for question, must, must_not in matching(TOOL_CHOICE, args.match,
                                                     lambda row: row[0]):
                before = len(chat.toolbox.log)
                del results[:]
                answer = chat.ask(question)
                # `link` is dropped: _probe_link makes that call itself when an
                # answer comes back blank, so counting it would measure the
                # host's recovery rather than the model's choice of tool.
                called = [name for name, _ in chat.toolbox.log[before:]
                          if name != 'link']
                # prompt_history as well as history: trim() puts the last five
                # questions in the system message as "already tried in this
                # conversation", and by the third row this suite had asked for
                # the same list twice in two languages - measured, the model
                # then answered "list every analog channel" from nothing at
                # all, with no call, because it read the question as one it had
                # already done.
                chat.history = []
                chat.prompt_history = []
                chat.last_channels = None

                # "Answered" is not "wrote a sentence": the host silences a
                # retyped list on purpose, because the trace above it is the
                # answer.
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

                # And the answer is not the trace typed out again.
                said_twice = _twice(answer, results)
                report.check('%s -> said once, not twice' % safe(question, 40),
                             not said_twice, said_twice or 'once')

        if 'sequence' in want:
            print(chr(10) + '-- one question after another, history kept --')
            for pair in matching(SEQUENCES, args.match,
                                 lambda row: ' '.join(q for q, _, _ in row)):
                chat.history = []
                chat.prompt_history = []
                chat.last_channels = None
                seen = []
                for question, must, must_not in pair:
                    before = len(chat.toolbox.log)
                    del results[:]
                    answer = chat.ask(question)
                    called = [name for name, _ in chat.toolbox.log[before:]
                              if name != 'link']
                    shown = ''.join(str(r) for r in results)
                    label = safe(question, 44)
                    report.check('%s -> %s' % (label, must),
                                 must in called,
                                 ', '.join(called) or 'no calls')
                    report.check('%s -> not %s' % (label, must_not),
                                 must_not not in called,
                                 ', '.join(called) or 'none')
                    report.check('%s -> the operator got something' % label,
                                 bool(answer.strip()) or bool(results),
                                 safe(answer, 40) or '(the trace)')
                    # The visible symptom, and the reason this section exists:
                    # the second question put the first question's block on
                    # screen again, character for character.
                    if seen:
                        report.check('%s -> and not the block above it again'
                                     % label,
                                     shown.strip() != seen[-1].strip(),
                                     safe(shown, 44) or '(nothing)')
                    seen.append(shown)

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
                # detect() is the same judge the session prompt is built from,
                # so a disagreement here is the operator's screen disagreeing
                # too.
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
        # The model stays loaded.
        if release:
            try:
                chat.client.unload()
            except Exception:                                 # noqa: BLE001
                pass

    print('\n%d passed, %d failed' % (report.passed, report.failed))
    return 1 if report.failed else 0


if __name__ == '__main__':
    sys.exit(main())
