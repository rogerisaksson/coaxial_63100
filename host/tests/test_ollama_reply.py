#!/usr/bin/env python3
"""What an answer means: retypes, blank answers, nudges."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (Scope, ScriptedModel, SimulatedSession,
    call, io, simulated, toolmod)   # noqa: E402

def test_retype_with_the_trace_off(report):
    """A silenced retype must not leave an empty screen."""
    from coaxial_ollama import debug

    # Every channel the stand-in carries, named.
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
    """One list, not two."""
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_ollama import debug

    # The package's stand-in, not this file's four-channel double: the names
    # below are the seven the board actually reports, and the check is that all
    # of them being typed out again is what the backstop sees.
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

    # Length is what tells a list from an explanation.
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

    # Length alone could not tell the two apart, and a bar set on it let a real
    # one through: "here are the analog channels: - PhaseU (channel 0)" seven
    # times over is 26 words and every one of them is a name, a number or glue.
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

    # A digital row names its pin first and never starts with a digit, so
    # MAP_ROW - anchored on the analog shape - never saw one.
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

    # A row can be named back two ways, and the model picks one.
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

    # The two bars are different on purpose.
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
