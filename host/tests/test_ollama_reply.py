#!/usr/bin/env python3
"""What an answer means: retypes, blank answers, nudges.

Split out of test_ollama.py, which had grown to 5,496 lines and 733 checks in
one file - a third of every check this tree has, and the reason a coverage
tier could not be asked for at any useful resolution. One subject per file
now, so a tier buys them separately and a reader opens the one they meant.

Run from the host directory:  python tests/test_ollama_reply.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (Scope, ScriptedModel, SimulatedSession,
    call, io, simulated, toolmod)   # noqa: E402

def test_retype_with_the_trace_off(report):
    """A silenced retype must not leave an empty screen.

    SYSTEM says never to restate a tool's own rows, and `is_retype` replaces
    an answer that does it with silence - right, because the trace put the
    table directly above. With `--quiet` there is no trace, and "read every
    analog channel" then answered with nothing at all. The board's own rows
    go out instead: the same table, not the model's typing of it.
    """
    from coaxial_ollama import debug

    # Every channel the stand-in carries, named. Written out it was seven,
    # and `s_adc` grew to nine.
    retyped = ('%s were all read just now.'
               % ', '.join(c['signal'].replace(' ', '')
                           for c in simulated.CHANNELS))

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
    listed = ('Här är de analoga kanalerna: %s.'
              % ', '.join(c['signal'].replace(' ', '')
                          for c in simulated.CHANNELS))

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

    # Length alone could not tell the two apart, and a bar set on it let a
    # real one through: "Har ar de analoga kanalerna: - PhaseU (kanal 0)"
    # seven times over is 26 words and every one of them is a name, a
    # number or glue. What is counted is the words the table did NOT
    # already contain - 3, 6, 8 and 12 for the restatements measured here,
    # 38 for the description.
    per_channel = ('Här är de analoga kanalerna:' + chr(10)
                   + chr(10).join(
                       '- %s (kanal %d)' % (c['signal'].replace(' ', ''), i)
                       for i, c in enumerate(simulated.CHANNELS)))
    report.check('a list with an index per channel is still a restatement',
                 turn(per_channel) == '',
                 '%d words -> %r' % (len(per_channel.split()),
                                     turn(per_channel)[:20]))
    report.check('and the bar counts what is not in the table, not length',
                 repliesmod.RESTATE_MAX_EXTRA == 15
                 and not hasattr(repliesmod, 'RESTATE_MAX_WORDS'),
                 str(repliesmod.RESTATE_MAX_EXTRA))

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
    # the analog shape - never saw one.
    #
    # Every reply below is built from the board's own digital map rather
    # than typed out. It was typed out, with the two rows the map had then,
    # and adding UART5_TERM and KEEPALIVE to `s_digital` left seven checks
    # failing on a mechanism that was working: `is_retype` wants every
    # channel named, and a fixture naming two of four is not a retype. The
    # channel map is the board's (CLAUDE.md), and so is this.
    def digital_turn(tool, reply, quiet=False, **args):
        chat = debug.Chat(ScriptedModel([
            call(tool, **args),
            {'role': 'assistant', 'content': reply},
        ]), toolmod.Toolbox(Sim(), scope=Scope()),
            out=io.StringIO(), quiet=quiet)
        return chat.ask('ge mig en lista på de digitala kanalerna')

    pins = Sim().board.system.channel_map()['digital']
    signals = [row['signal'] for row in pins]
    both = ['%s (%s)' % (row['pin'], row['signal']) for row in pins]
    one_of = signals[0]
    other = signals[1]

    listed_pins = ('De digitala kanalerna är:' + chr(10)
                   + chr(10).join('%s (%s) för %s'
                                  % (row['pin'],
                                     'utgång' if row['direction'] == 'out'
                                     else 'ingång', row['signal'])
                                  for row in pins))
    report.check('a retyped digital map is silenced',
                 digital_turn('board_info', listed_pins, kind='digital') == '',
                 repr(digital_turn('board_info', listed_pins,
                                   kind='digital'))[:46])
    by_pin = ' och '.join('%s är 1' % row['pin'] for row in pins) + '.'
    report.check('and so is a retyped digital reading',
                 digital_turn('digital_read', by_pin) == '', repr(by_pin)[:44])

    # A row can be named back two ways, and the model picks one. Pins and
    # signals are alternatives, not a union: a union would want every name
    # from every column present.
    by_signal = ' och '.join('%s är 1' % s for s in signals) + '.'
    named_both = ', '.join('%s 1' % b for b in both) + '.'
    for reply, silent, why in (
            (by_signal, True, 'named by signal'),
            (by_pin, True, 'named by pin'),
            (named_both, True, 'named both ways'),
            ('%s är asserterad medan AFE:n är på.' % other, False,
             'one signal, and something to say about it'),
            ('%s är 1.' % one_of, False, 'one of them is not the list')):
        got = digital_turn('digital_read', reply)
        report.check('a reading %s -> %s' % (why,
                                             'silent' if silent else 'kept'),
                     (got == '') is silent, repr(got)[:44])

    # The same, off the map rather than a reading.
    for reply, silent in ((', '.join(signals) + '.', True),
                          ('Fyra digitala kanaler.', False)):
        got = digital_turn('board_info', reply, kind='digital')
        report.check('a map named back by signal -> %s'
                     % ('silent' if silent else 'kept'),
                     (got == '') is silent, repr(got)[:44])
    report.check('but a finding that does not name them all survives',
                 digital_turn('board_info', '%s är asserterad.' % other,
                              kind='digital') == '%s är asserterad.' % other)
    hushed = digital_turn('digital_read', by_pin, quiet=True)
    report.check('with --quiet the block goes out instead of the retyping',
                 'digital:' in hushed and by_pin not in hushed,
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


ROSTER = (
    (test_retype_with_the_trace_off, ('reply',)),
    (test_map_retype, ('reply', 'render')),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
