#!/usr/bin/env python3
"""How a result reaches the screen: columns, blocks, clipping.

Split out of test_ollama.py, which had grown to 5,496 lines and 733 checks in
one file - a third of every check this tree has, and the reason a coverage
tier could not be asked for at any useful resolution. One subject per file
now, so a tier buys them separately and a reader opens the one they meant.

Run from the host directory:  python tests/test_ollama_render.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (Scope, ScriptedModel, SimulatedSession, 
    call, io, simulated, toolmod)   # noqa: E402

def test_reading_block(report):
    """A reading is a headed block, like the map is.

    Asked for both the digital and the analog values, the two arrived as one
    wall: a bare `64 smp @2000Hz` over unlabelled columns, then the digital
    table straight underneath with no gap. The reading is counted and headed
    now, and the trace puts a blank line between blocks.
    """
    from coaxial import simulated
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_mcp import tools as mcp
    from coaxial_ollama import debug, language

    reading = mcp.HANDLERS['analog_read'](Sim(), samples=64)
    head = [l for l in reading.splitlines() if not l.startswith('AFE')]

    report.check('the reading says how many channels it read',
                 head[0] == 'analog: %d channels'
                 % len(simulated.CHANNELS), head[0])
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
    # Counted off the board, for the reason the reserved check below already
    # gives: a count written here is the second answer to a question the pin
    # table settles. It was 2, `s_digital` grew to 4, and this went red on a
    # renderer that was right.
    usable = len(Sim().board.system.channel_map()['digital'])
    report.check('the digital block counts channels, not pins',
                 levels.splitlines()[0] == 'digital: %d channels' % usable,
                 levels.splitlines()[0])
    report.check('and both blocks start their first column the same way',
                 levels.splitlines()[1].split()[0] == 'ch'
                 and head[2].split()[0] == 'ch')
    # Counted off the stand-in's own list, not written here: the number grew
    # from 7 to 15 when SPI2 and the IMU's control pins were added, and a
    # count in a test is the same second answer a pin table in a document is.
    reserved = mcp.HANDLERS['board_info'](Sim(), kind='reserved')
    expected = 'reserved: %d pins' % len(simulated.RESERVED)
    report.check('while the reserved list stays pins - it is not channels',
                 reserved.startswith(expected),
                 reserved.splitlines()[0])

    # Swedish on screen, English on the wire: the headings turn, the column
    # names do not - they are the board's words, like a channel name.
    turned = language.localise(reading, 'Swedish')
    report.check('the headings turn for a Swedish session',
                 'analog: %d kanaler' % len(simulated.CHANNELS)
                 in turned and '64 sampel @' in turned,
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
    # Both counts off the board. Written here they are a second answer to
    # what `board_adc.c` and `s_digital` already settle, and the digital one
    # was: it said 2, the board grew to 4, and this failed on a renderer
    # that was doing its job.
    shape = session.board.system.channel_map()
    report.check('the two kinds get their own headed blocks',
                 'analog: %d channels' % len(shape['analog']) in whole
                 and 'digital: %d channels' % len(shape['digital']) in whole,
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


ROSTER = (
    (test_reading_block, ('render',)),
    (test_map_sections, ('render', 'board')),
    (test_screen, ('render', 'link')),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
