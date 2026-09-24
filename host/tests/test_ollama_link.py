#!/usr/bin/env python3
"""The serial link: ports, probing, diagnosis, recovery."""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (ConnectError, Scope, ScriptedModel,   # noqa: E402
    SimulatedSession, _Held, _NotATty, call, detail, io, sessionmod, toolmod)
from coaxial.comm import ports                                              # noqa: E402

def test_power_check_cannot_halt(report):
    """Diagnosing the link must not be able to break it."""
    from tools.target import build_and_flash
    from tools.target import find_board
    import subprocess

    seen = {}

    class Done:
        stdout, stderr, returncode = 'Voltage     : 3.27V', '', 0

    def spy(argv, **kw):
        seen['argv'] = list(argv)
        seen['timeout'] = kw.get('timeout')
        return Done()

    # The question here is the shape of the call, not whether this machine has
    # the programmer installed - a runner does not, and without the stub
    # check_power returns early and the spy never sees the argv.
    real_run = subprocess.run
    real_find = build_and_flash.find_programmer
    try:
        subprocess.run = spy
        build_and_flash.find_programmer = lambda path: 'STM32_Programmer_CLI'
        voltage, _ = find_board.check_power()
    finally:
        subprocess.run = real_run
        build_and_flash.find_programmer = real_find

    argv = seen.get('argv') or []
    report.check('check_power reads the voltage', voltage == 3.27, str(voltage))
    report.check('and never connects under reset',
                 'mode=UR' not in argv, ' '.join(argv[1:]) or 'no call made')
    report.check('it hotplugs instead - reset is never asserted, so a kill '
                 'cannot leave the core halted',
                 'mode=HOTPLUG' in argv, ' '.join(argv[1:]) or 'no call made')
    report.check('and it is still bounded, since that kill is what happens '
                 'on a wedged probe', bool(seen.get('timeout')),
                 str(seen.get('timeout')))

def test_port_state(report):
    """Why a port is not answering, not just that it is not."""
    from tools.target import find_board
    import serial

    real_serial = serial.Serial
    try:
        def denied(*a, **kw):
            # The message Windows produced here, Swedish locale and all.
            raise serial.SerialException(
                "could not open port 'COM4': PermissionError(13, "
                "'Åtkomst nekad.', None, 5)")

        serial.Serial = denied
        report.check('a port another process holds is busy, not silent',
                     find_board.port_state('COM4') == ports.BUSY,
                     find_board.port_state('COM4'))

        def missing(*a, **kw):
            raise serial.SerialException(
                "could not open port 'COM99': FileNotFoundError(2, ...)")

        serial.Serial = missing
        report.check('a port that is not there is absent, not busy',
                     find_board.port_state('COM99') == ports.ABSENT,
                     find_board.port_state('COM99'))
    finally:
        serial.Serial = real_serial

    # The class name is what decides it: the OS message is localised, and
    # matching "Access is denied" would have read this one as absent.
    report.check('and it is decided on the class name, not the OS wording',
                 'PermissionError' in open(ports.__file__,
                                           encoding='utf-8').read())

    # The checklist stops guessing at it.
    box = toolmod.Toolbox(SimulatedSession())
    real_state = find_board.port_state
    real_power = find_board.check_power
    real_ports = find_board.list_ports
    try:
        find_board.port_state = lambda *a, **kw: ports.BUSY
        find_board.check_power = lambda *a, **kw: (3.27, 'stubbed')
        find_board.list_ports = lambda: ['COM_TEST']
        box.session = _Held()
        checklist = str(box.call('link_diagnose', {}))
    finally:
        find_board.port_state = real_state
        find_board.check_power = real_power
        find_board.list_ports = real_ports
    report.check('link_diagnose says the port is held, not that it might be',
                 'open in another process' in checklist,
                 checklist.splitlines()[-1][:56])

def test_link_recovery(report):
    """One screen, one verdict about the link."""
    from coaxial_ollama import debug

    # Deliberately not the real step-4 wording: that one is in
    # language.PHRASES, so the trace comes back in the session's language while
    # the fake's return value stays English, and the count then measures
    # localisation rather than duplication.
    STEP4_UP = 'DIAGNOSIS MARKER - the link is up'
    DEAD = 'ERR NoReplyError: unit 1, fc 0x47: silence'

    class Flapping:
        """A link that answers only once the stale handle is dropped - the
        shape of a VCP that re-enumerated under a replugged cable.
        """
        port = bus = unit = attached = None
        baud = 115200
        simulated = False

        def __init__(self, fails=1):
            self.fails, self.resets, self.calls = fails, 0, []
            self.session = self
            self.log = []

        def reset(self):
            self.resets += 1

        def call(self, name, args):
            self.calls.append(name)
            if name == 'link_diagnose':
                return 'a two-line checklist' + chr(10) + STEP4_UP
            if self.fails > 0:
                self.fails -= 1
                return DEAD
            return 'frames rx=12 tx=12'

    def probing(fails):
        chat = debug.Chat.__new__(debug.Chat)
        chat.toolbox = Flapping(fails)
        chat.history, chat.link_ok, chat.quiet = [], True, True
        return chat, chat._probe_link()

    chat, probe = probing(1)
    report.check('a link that answers after the reset is reported up, not '
                 'down', chat.link_ok is True, str(probe)[:40])
    report.check('and the reset happened exactly once',
                 chat.toolbox.resets == 1, str(chat.toolbox.resets))
    report.check('which cost one retry, not a loop',
                 chat.toolbox.calls == ['link', 'link'],
                 ', '.join(chat.toolbox.calls))

    chat, probe = probing(99)
    report.check('a link that stays silent is still reported down',
                 chat.link_ok is False, str(probe)[:40])
    report.check('and is not retried past the one the reset earned',
                 chat.toolbox.calls == ['link', 'link'],
                 ', '.join(chat.toolbox.calls))

    # `shown` means the checklist is already on screen from the trace.
    chat.toolbox.calls = []
    said = chat._link_down_message(DEAD, shown=True)
    report.check('with the checklist already traced, the answer does not '
                 'run link_diagnose again', 'link_diagnose'
                 not in chat.toolbox.calls, ', '.join(chat.toolbox.calls))
    report.check('and does not reprint it either',
                 STEP4_UP not in said, said[:46])
    report.check('but still says the question went unanswered, and by what',
                 said.startswith('link is down, not answered:')
                 and 'NoReplyError' in said, said[:46])

    said = chat._link_down_message(DEAD, shown=False)
    report.check('with nothing on screen above, the checklist comes with '
                 'the answer', STEP4_UP in said, said.splitlines()[-1][:46])

    # End to end, the shape of the transcript: the model calls link_diagnose
    # and then writes nothing, so the turn falls to the stale path.
    screen = io.StringIO()
    box = Flapping(99)
    talk = debug.Chat(ScriptedModel([
        call('link_diagnose'),
        {'role': 'assistant', 'content': ''},
    ]), box, out=screen)
    talk.toolbox = box                      # the fake stands in for both
    # Not a board order any more: board_switch intercepts that before the model
    # is reached, and this needs a question that actually runs a turn.
    answer = talk.ask('vad läser NTC:n?')
    whole = screen.getvalue() + chr(10) + answer
    report.check('one screen carries the checklist once, not twice',
                 whole.count(STEP4_UP) == 1, '%d copies' % whole.count(STEP4_UP))
    report.check('so it cannot say the link is up and down at once',
                 not (STEP4_UP in answer and 'link is down' in answer),
                 answer.splitlines()[0][:46] if answer else '<empty>')

def test_link_diagnose(report):
    """OS-level, not another board round trip - see tools.py's own docstring
    for why.
    """
    import coaxial
    from tools.target import find_board
    import serial.tools.list_ports as list_ports

    class FakePort:
        def __init__(self, device):
            self.device = device

    real_comports = list_ports.comports
    real_connect = coaxial.devices.board.connect
    real_check_power = find_board.check_power
    real_port_state = find_board.port_state
    try:
        # Stubbed for the same reason as the other three: it opens a real port.
        find_board.port_state = lambda *a, **kw: ports.SILENT
        list_ports.comports = lambda: [FakePort('COM4'), FakePort('COM7')]
        coaxial.devices.board.connect = lambda *a, **kw: (_ for _ in ()).throw(
            ConnectError('nothing answered'))
        find_board.check_power = lambda timeout=15: (3.30, 'fake: powered')

        missing = toolmod.Toolbox(sessionmod.Session(port='COM9', baud=115200, unit=1))
        result = str(missing.call('link_diagnose', {}))
        report.check('powered, but a configured port absent from the OS '
                     'list is named as such, not folded into a generic '
                     'error',
                     'COM9' in result and 'not among' in result, result)

        present = toolmod.Toolbox(sessionmod.Session(port='COM4', baud=115200, unit=1))
        result2 = str(present.call('link_diagnose', {}))
        report.check('powered and present, but silent, points at nothing '
                     'else having the port open, not the cable',
                     'COM4' in result2
                     and 'answers on COM4 right now: no' in result2, result2)

        coaxial.devices.board.connect = lambda *a, **kw: []                # "answers"
        result2b = str(present.call('link_diagnose', {}))
        report.check('and a port that actually answers says the link is '
                     'up, not "silent" just because it exists',
                     'link is up' in result2b, result2b)

        list_ports.comports = lambda: []
        empty = toolmod.Toolbox(sessionmod.Session(port='COM4', baud=115200, unit=1))
        result3 = str(empty.call('link_diagnose', {}))
        report.check('no COM ports at all is named plainly',
                     'Nothing is enumerating' in result3, result3)

        find_board.check_power = lambda timeout=15: (0.0, 'fake: no power')
        unpowered = toolmod.Toolbox(sessionmod.Session(port='COM4', baud=115200, unit=1))
        result4 = str(unpowered.call('link_diagnose', {}))
        report.check('no target power stops the checklist at step 1, before '
                     'even listing COM ports - later steps cannot explain '
                     'more than the first one already does',
                     'no power sensed' in result4
                     and 'COM ports Windows sees' not in result4, result4)
        find_board.check_power = lambda timeout=15: (3.30, 'fake: powered')

        # It names the stand-in it is actually on and the way off it.
        stood_in = str(toolmod.Toolbox(SimulatedSession()).call('link_diagnose', {}))
        report.check('a session with no port names the stand-in it is on, '
                     'and the way off it',
                     'simulated board' in stood_in and '/board auto' in stood_in,
                     stood_in[:52])
        report.check('and never claims a flag the operator did not type',
                     '--simulated' not in stood_in, stood_in[:52])
        from coaxial_ollama.debug import NoBoard
        refused = str(toolmod.Toolbox(NoBoard()).call('link_diagnose', {}))
        report.check('--no-board is the one case that did get the flag',
                     refused.startswith('--no-board this run')
                     and '/board auto' in refused, refused[:52])

        # Ungated: no --allow-writes, no --confirm, no --read-only.
        ro = toolmod.Toolbox(SimulatedSession(), allow_code=False)
        report.check('link_diagnose works even with --read-only',
                     not str(ro.call('link_diagnose', {})).startswith('ERR'))

        # The stand-in `open_session` actually returns, not the double at the
        # top of this file - that one has no `port` at all, so it reached the
        # branch above however the branch was written.
        from coaxial.simulated import SimulatedSession as FellBack
        probed = []
        find_board.check_power = lambda timeout=15: (probed.append(1),
                                                     (3.30, 'fake'))[1]
        fell_back = str(toolmod.Toolbox(FellBack()).call('link_diagnose', {}))
        report.check('the stand-in open_session returns is known by its own '
                     'marker, not by a port that happens to be None',
                     'simulated board' in fell_back, fell_back[:52])
        report.check('and no SWD probe is spent on a session with no SWD',
                     not probed, '%d call(s)' % len(probed))

        # Step 4's closing advice used to open with "Powered" whatever step 1
        # concluded - on a pulled cable, asserting the one thing that was false
        # and pointing at a busy port and a halted core instead.
        list_ports.comports = lambda: [FakePort('COM4')]
        coaxial.devices.board.connect = lambda *a, **kw: (_ for _ in ()).throw(
            ConnectError('nothing answered'))
        find_board.check_power = lambda timeout=15: (None, 'fake: unknown')
        unsure = toolmod.Toolbox(sessionmod.Session(port='COM4', baud=115200, unit=1))
        result5 = str(unsure.call('link_diagnose', {}))
        report.check('a step 1 that could not check never closes by '
                     'asserting the board is powered',
                     'Power unconfirmed' in result5
                     and 'Powered and the port is right' not in result5,
                     result5.splitlines()[-1][:60])

        find_board.port_state = lambda *a, **kw: ports.BUSY
        held = toolmod.Toolbox(sessionmod.Session(port='COM4', baud=115200, unit=1))
        result6 = str(held.call('link_diagnose', {}))
        report.check('a port another process holds says so, rather than '
                     'guessing at a halted core',
                     'open in another process' in result6,
                     result6.splitlines()[-1][:58])
        find_board.port_state = lambda *a, **kw: ports.SILENT

        # check_power's own timeout path.
        from tools.target import build_and_flash
        import subprocess
        was = (subprocess.run, build_and_flash.find_programmer,
               build_and_flash.toolchain_path)
        try:
            build_and_flash.toolchain_path = lambda: None
            build_and_flash.find_programmer = lambda _path: 'fake-programmer'

            def killed_at_the_timeout(*_a, **_kw):
                raise subprocess.TimeoutExpired(
                    cmd='fake', timeout=15,
                    output='Board       : STLINK-V3SET' + chr(10)
                           + 'Voltage     : 0.00V' + chr(10)
                           + 'Error: Unable to get core ID')

            subprocess.run = killed_at_the_timeout
            volts, why = real_check_power(15)
            report.check('a programmer killed at its timeout still yields '
                         'the voltage it had already printed',
                         volts == 0.0, '%r - %s' % (volts, why.splitlines()[0]))
        finally:
            (subprocess.run, build_and_flash.find_programmer,
             build_and_flash.toolchain_path) = was
    finally:
        list_ports.comports = real_comports
        coaxial.devices.board.connect = real_connect
        find_board.check_power = real_check_power
        find_board.port_state = real_port_state



# ---- what reaches the screen, and how it reads -----------------------------

def test_fallback(report):
    """No cable is not a failing test suite - it is a different board."""
    from coaxial.comm.session import open_session
    from coaxial_ollama import debug
    from coaxial_ollama import spinner as spin

    session, found = open_session(simulated=True)
    report.check('forced simulated skips the search entirely',
                 type(session).__name__ == 'SimulatedSession'
                 and not found.real, type(session).__name__)
    report.check('and is labelled Simulated, not by a port it never opened',
                 found.label == 'Simulated', found.label)

    # The label names the path, not just the port: a reading over the bench
    # cable and one over the field bus are not the same measurement, and which
    # it was has to be on screen rather than inferred from a COM number.
    for real, port, kind, want in ((True, 'COM3', 'probe', 'JTAG and COM3'),
                                   (True, 'COM5', 'serial', 'RS485 at COM5'),
                                   (False, None, None, 'Simulated')):
        report.check('label: %s' % want,
                     sessionmod._label(real, port, kind) == want,
                     sessionmod._label(real, port, kind))
    report.check('and says "simulated" where a firmware version goes',
                 (session.board.version_info or {})['firmware'] == 'simulated',
                 (session.board.version_info or {})['firmware'])

    # PB2 is the AFE switch, not a spare pin.
    session.board.gpio.test_mode(True)
    session.board.afe.enable()
    session.board.gpio.port_write('B', 0xFFFF, 0)
    report.check('clearing PB2 on the stand-in turns its AFE off',
                 not session.board.afe.state()['on'])
    report.check('and PE15 follows it inversely, as the real one does',
                 session.board.gpio.pin_read('E', 15) is True)
    session.board.gpio.pin_write('B', 2, True)
    report.check('setting PB2 turns it back on',
                 session.board.afe.state()['on']
                 and session.board.gpio.port_read('B') & (1 << 2))

    # link_diagnose's step 4 opened the port a second time to ask whether the
    # board answers - while the session held it open.
    class Held(sessionmod.Session):
        """A session holding an open link - the real class, the link
        handed in."""
        def __init__(self, board):
            super().__init__('COM_TEST', 115200, 1)
            self._board = board

    live = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    report.check('a session with no board cached falls through to the probe',
                 toolmod._open_link_answers(Held(None)) is False)
    report.check('and one holding an answering link says so without opening '
                 'anything',
                 toolmod._open_link_answers(Held(SimulatedSession().board))
                 is True)

    class Dead:
        def echo(self, data):
            raise ConnectError('cable pulled')

    class DeadBoard:
        link = Dead()

    report.check('a held link that has since died is not counted as up',
                 toolmod._open_link_answers(Held(DeadBoard())) is False)

    # Steps 1 and 2 stubbed: the first shells out to STM32_Programmer_CLI with
    # a 15s timeout, and the second asks Windows what it has plugged in -
    # neither is what step 4 is being checked for, and both make the answer
    # depend on the desk the suite runs on.
    live.session = Held(SimulatedSession().board)
    power, ports = toolmod.find_board.check_power, toolmod.find_board.list_ports
    toolmod.find_board.check_power = lambda *a, **k: (3.27, 'stubbed')
    toolmod.find_board.list_ports = lambda: ['COM_TEST']
    try:
        checklist = str(live.call('link_diagnose', {}))
    finally:
        toolmod.find_board.check_power = power
        toolmod.find_board.list_ports = ports
    report.check('so step 4 reports the link that is actually up',
                 'right now: yes' in checklist,
                 checklist.splitlines()[-1][:52])

    # /board: what the tools talk to, swapped without a restart.
    swap = debug.Chat.__new__(debug.Chat)
    swap.toolbox = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    swap.origin, swap.link_ok, swap.last_channels = ('Simulated', False), False, {'ntc'}
    said = swap.command('/board simulated')
    report.check('/board simulated takes the stand-in',
                 said == 'board: Simulated'
                 and type(swap.toolbox.session).__name__ == 'SimulatedSession',
                 said)
    report.check('and the prompt tag is rebuilt from the same origin',
                 swap.origin == ('Simulated', False), str(swap.origin))
    report.check('and a table from the board just left is forgotten',
                 swap.last_channels is None, str(swap.last_channels))
    report.check('/board with no argument says what it is on',
                 (swap.command('/board') or '').startswith('board: Simulated'),
                 swap.command('/board'))

    # An order to swap the board is the host's to carry out.
    for question, want in (
            ('byt till debugproben', 'auto'),
            ('byter du till debugproben', 'auto'),
            ('byt till en simulerad enhet', 'simulated'),
            ('byt till simulerad hårdvara', 'simulated'),
            ('växla till COM4', 'COM4'),
            ('switch to the real board', 'auto'),
            ('byt till RS485', 'rs485'),
            ('byt till fältbussen', 'rs485'),
            # Every one of these lost the order to a single unlisted noun while
            # this required all the words to be known: 'enhet', then
            # 'hardvara', then 'lage'.
            ('byter du till simulerat läge', 'simulated'),
            ('byt till simulerat läge', 'simulated'),
            ('använd det simulerade kortet', 'simulated'),
            ('koppla om till proben', 'auto'),
            ('byt till den simulerade grejen', 'simulated'),
            # An interrogative disqualifies it.
            ('vad är debugproben?', None),
            ('vilket läge är du i?', None),
            ('vet du om kortet svarar?', None),
            #...and so does a second request the host cannot carry out.
            ('byt till simulerat läge och läs NTC:n', None),
            ('byt till proben och mät NTC:n', None),
            ('byt språk till svenska', None),
            ('läs NTC:n och DC-länken', None),
            ('beskriv hårdvaran för en novis', None),
            # Names a board, orders nothing.
            ('debugproben är inte inkopplad', None)):
        got = debug.board_switch(question)
        report.check('board order: %s' % question[:34], got == want, str(got))

    # A pipe is not a console.
    mangled = 'byter du till simulerat läge'.encode('utf-8').decode('cp1252')
    report.check('the mangling really does hide the order',
                 debug.board_switch(mangled) is None, mangled[-12:])
    report.check('so a pipe is decoded as UTF-8, like the outputs are',
                 debug._printable(_NotATty()) is not None
                 and _NotATty.asked == {'encoding': 'utf-8',
                                        'errors': 'replace'},
                 str(_NotATty.asked))

    #...and it reaches the swap, without a model turn.
    ordered = debug.Chat.__new__(debug.Chat)
    ordered.toolbox = toolmod.Toolbox(SimulatedSession(), scope=Scope())
    ordered.origin, ordered.link_ok = ('Simulated', False), False
    ordered.last_channels, ordered.language = None, None
    ordered.io_log = debug.IOLog(enabled=False)
    ordered.out = io.StringIO()
    ordered.client = ScriptedModel([])
    said = ordered.ask('byt till en simulerad enhet')
    report.check('an order to swap the board never reaches the model',
                 not ordered.client.prompts, '%d turns'
                 % len(ordered.client.prompts))
    report.check('and answers with the board it landed on',
                 said == 'board: Simulated', said)
    # Ordered a real board and found none.
    ordered.language = None
    was = sessionmod.open_session
    try:
        sessionmod.open_session = lambda *a, **kw: (
            SimulatedSession(),
            sessionmod.Origin(False, None, 115200, None, 'Simulated',
                              'simulated', 1))
        said = ordered.ask('byt till debugproben')
    finally:
        sessionmod.open_session = was
    report.check('a search that found nothing says so, not just where it '
                 'ended up', 'nothing answered' in said, said)

    # An order that cannot be carried out must not cost the board that was
    # working.
    class Live(sessionmod.Session):
        """A session that is already on a real board."""
        closed = False

        def __init__(self):
            super().__init__('COM4', 115200, 1)

        def close(self):
            self.closed = True

        def reset(self):
            pass

    real = sessionmod.Origin(True, 'COM4', 115200, 'probe',
                             'JTAG and COM4', 'debug probe', 1)
    fake = sessionmod.Origin(False, None, 115200, None, 'Simulated',
                             'simulated', 1)

    def patched(result):
        def factory(*a, **kw):
            return (SimulatedSession() if not result.real else Live()), result
        return factory

    keeper = debug.Chat.__new__(debug.Chat)
    held = Live()
    keeper.toolbox = toolmod.Toolbox(held, scope=Scope())
    keeper.origin, keeper.link_ok = ('JTAG and COM4', True), True
    keeper.last_channels = {'ntc'}

    original = sessionmod.open_session
    try:
        sessionmod.open_session = patched(fake)
        said = keeper.command('/board rs485') or ''
        report.check('a search that found nothing keeps the working board',
                     keeper.toolbox.session is held and not held.closed,
                     type(keeper.toolbox.session).__name__)
        report.check('and leaves the prompt tag alone',
                     keeper.origin == ('JTAG and COM4', True),
                     str(keeper.origin))
        report.check('and says what it tried, not just where it ended up',
                     said.startswith('board: nothing answered on')
                     and 'JTAG and COM4' in said, said[:56])
        report.check('and does not wipe a reading from a board it kept',
                     keeper.last_channels == {'ntc'},
                     str(keeper.last_channels))

        # Ordering the stand-in never searches, so it always lands.
        report.check('an order for the stand-in always lands',
                     keeper.command('/board simulated') == 'board: Simulated',
                     str(keeper.origin))
        report.check('and that one does swap, and closes what it left',
                     keeper.origin == ('Simulated', False) and held.closed,
                     str(keeper.origin))

        sessionmod.open_session = patched(real)
        report.check('a search that found a board swaps to it',
                     keeper.command('/board auto') == 'board: JTAG and COM4'
                     and keeper.origin == ('JTAG and COM4', True),
                     str(keeper.origin))
        report.check('and forgets a table taken from the board it left',
                     keeper.last_channels is None, str(keeper.last_channels))
    finally:
        sessionmod.open_session = original

    # /model: same idea one layer up.
    from coaxial_ollama.client import Ollama
    swap.client = Ollama('gemma4:12b', keep_alive=0)
    swap.detail, swap.tool_names = detail.TERSE, ()
    real_models = Ollama.models
    Ollama.models = lambda self: ['gemma4:12b']
    try:
        before = swap.client.model
        said = swap.command('/model no-such-tag:9b') or ''
        report.check('/model refuses a tag that is not pulled, and swaps '
                     'nothing', 'not pulled' in said
                     and swap.client.model == before, said[:52])
        report.check('/model on the tag already running is a no-op',
                     swap.command('/model gemma4:12b') == 'model: gemma4:12b '
                     'already', swap.client.model)
    finally:
        Ollama.models = real_models

    class VT(io.StringIO):
        encoding = 'utf-8'

        def isatty(self):
            return True

    # 'all' is the broadcast address, and it is the one mode where a command
    # reaches every inverter on the bus and nothing answers to say it landed.
    for tag, ok, colour in (('Simulated', False, '\x1b[33m'),
                            ('JTAG and COM3', True, '\x1b[32m'),
                            ('RS485 at COM5', True, '\x1b[32m'),
                            ('COM4, ALL NODES', 'all', '\x1b[31m')):
        out = VT()
        face = spin.Prompt('Coaxial 63100', out, tick=99, tag=tag, tag_ok=ok)
        face.stop(True)
        painted = out.getvalue()
        report.check('the prompt says (%s)' % tag,
                     '(%s%s[0m)>' % (colour, tag) in painted,
                     {False: 'yellow', 'all': 'RED'}.get(ok, 'green'))
        # stop() repaints the prefix only, and the tag sits after the tail.
        report.check('and the repaint does not eat it: %s' % tag,
                     painted.count(tag) == 1, '%d copies' % painted.count(tag))

    plain = io.StringIO()                      # no isatty, so no VT
    spin.Prompt('Coaxial 63100', plain, tick=99, tag='Simulated', tag_ok=False)
    report.check('a terminal without VT still gets the word, without colour',
                 '(Simulated)>' in plain.getvalue()
                 and '[33m' not in plain.getvalue())


def test_pull_draws_the_daemons_numbers(report):
    """A tag that is not here is pulled, and the download is the daemon's
    own numbers drawn as a bar in Say's columns - one row rewritten in
    place on a TTY, a row every five percent off one.
    """
    from coaxial_ollama import cli, pull
    from coaxial_ollama.client import Ollama, OllamaError

    full, half, empty = pull.BRAILLE
    report.check('the bar is half-cell braille: 0, 52 and 100 % of 24 cells',
                 pull.bar(0, 100) == empty * 24
                 and pull.bar(52, 100) == full * 12 + half + empty * 11
                 and pull.bar(100, 100) == full * 24
                 and pull.bar(52, 100, glyphs=pull.ASCII) == '=' * 12 + '>' + '-' * 11,
                 pull.bar(52, 100))

    total = 4_900_000_000
    script = [{'status': 'pulling manifest'}]
    script += [{'status': 'pulling 667b0c1932bc', 'digest': 'sha256:667b',
                'total': total, 'completed': total * f // 100}
               for f in range(0, 101)]
    script += [{'status': 'pulling 948af2743fc7', 'digest': 'sha256:948a',
                'total': 1481, 'completed': 1481},
               {'status': 'verifying sha256 digest'},
               {'status': 'writing manifest'}, {'status': 'success'}]

    def scripted():
        # A second an EVENT, read as often as the code likes: the clock is how
        # many events have been handed over, so a rate is bytes an event and
        # the estimate follows from it.
        tick = [0.0]

        def stream():
            for event in script:
                tick[0] += 1.0
                yield event
        return stream(), (lambda: tick[0])

    class VT(io.StringIO):
        encoding = 'utf-8'

        def isatty(self):
            return True

    plain = io.StringIO()                       # no isatty, no codec
    source, now = scripted()
    got = pull.pull('llama3.1:8b', out=plain, source=source,
                    glyphs=pull.BRAILLE, now=now)
    rows = plain.getvalue().splitlines()
    waits = [r for r in rows if r.startswith('  wait')]
    report.check('the stream ends in the daemon\'s own last word',
                 got == 'success', got)
    report.check('off a TTY a row at every five percent and every status, '
                 'not every event: %d rows for %d events'
                 % (len(waits), len(script)), 24 <= len(waits) <= 28)
    halfway = [r for r in waits if ' 50 %' in r]
    report.check('a row is Say\'s columns: the tag, the bar, the percent, '
                 'the gigabytes, the rate and what is left',
                 len(halfway) == 1 and 'llama3.1:8b' in halfway[0]
                 and full * 12 in halfway[0] and 'of 4.9 GB' in halfway[0]
                 and '49 MB/s' in halfway[0] and '50 s left' in halfway[0]
                 and halfway[0].startswith('  wait   model '),
                 halfway[0] if halfway else waits[:2])
    report.check('and the last row says what was pulled and how long it took',
                 rows[-1].startswith('  ok     model ')
                 and 'pulled llama3.1:8b, 4.9 GB in 1 min 46 s' in rows[-1],
                 rows[-1])

    ascii_out = io.StringIO()
    source, now = scripted()
    pull.pull('llama3.1:8b', out=ascii_out, source=source, now=now)
    report.check('a stream with no codec for braille gets the ASCII bar',
                 '=' * 12 in ascii_out.getvalue()
                 and full not in ascii_out.getvalue())

    painted = VT()
    source, now = scripted()
    pull.pull('llama3.1:8b', out=painted, source=source, now=now)
    text = painted.getvalue()
    report.check('on a TTY the row is rewritten in place, in Say\'s colours, '
                 'and ends once', text.count('\r') >= len(script)
                 and text.count('\n') == 1 and '\x1b[36m' in text
                 and text.rstrip().endswith('\x1b[0m'),
                 '%d CR, %d LF' % (text.count('\r'), text.count('\n')))

    refused = io.StringIO()
    try:
        pull.pull('no-such:9b', out=refused,
                  source=iter([{'status': 'pulling manifest'},
                               {'error': 'pull model manifest: file does '
                                         'not exist'}]))
        said = ''
    except OllamaError as exc:
        said = str(exc)
    report.check('an error line is the daemon\'s words, raised and shown in '
                 'red', 'file does not exist' in said
                 and refused.getvalue().splitlines()[-1].startswith('  fail'),
                 said)
    try:
        pull.pull('minimax-m3:cloud', source=iter([]), out=io.StringIO())
        said = ''
    except OllamaError as exc:
        said = str(exc)
    report.check('a cloud tag is refused before any request',
                 'cloud' in said, said)

    client = Ollama('gemma4:12b', keep_alive=0)
    listed = [[], ['gemma4:12b']]
    real_models = Ollama.models
    Ollama.models = lambda self: listed.pop(0) if len(listed) > 1 else listed[0]
    pulled = []

    def fake_pull(tag, host, out):
        pulled.append((tag, host))
    try:
        name = cli.ensure_pulled(client, io.StringIO(), pull_with=fake_pull)
        report.check('dbg.py\'s start pulls an absent tag once, then takes the '
                     'name the list gives',
                     pulled == [('gemma4:12b', client.host)]
                     and name == 'gemma4:12b', (pulled, name))

        def unreachable(self):
            raise OllamaError('cannot reach ollama at %s' % self.host)
        Ollama.models = unreachable
        pulled[:] = []
        try:
            cli.ensure_pulled(client, io.StringIO(), pull_with=fake_pull)
            said = ''
        except OllamaError as exc:
            said = str(exc)
        report.check('and a daemon that is not there is not a pull: the '
                     'error as before', 'cannot reach' in said and not pulled,
                     said)
    finally:
        Ollama.models = real_models

    host = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    page = io.open(os.path.join(host, 'board_chat.ps1'),
                   encoding='utf-8-sig').read()
    helpers = io.open(os.path.join(host, 'board_chat', 'Ollama.ps1'),
                      encoding='utf-8-sig').read()
    report.check('the page pulls through the same module, not `ollama pull`',
                 'python -m coaxial_ollama.pull $Model' in page
                 and 'Source pull $Model' not in page)
    report.check('and a failed preload is the daemon\'s words, a missing '
                 'runner named with its fix',
                 'function Get-DaemonWords' in helpers
                 and 'Get-DaemonWords $_' in page
                 and 'llama-server binary not found' in page
                 and 'install.ps1' in page)

    # THE CHOOSER'S CHAT PAGE PULLS ON ITS BOOT STRIP.
    from terminal.views import show_chat
    text = io.open(os.path.join(host, 'terminal', 'views', 'show_chat.py'),
                   encoding='utf-8').read()
    report.check('the chooser\'s chat page pulls through ensure_pulled, '
                 'not require_model',
                 'cli.ensure_pulled(' in text
                 and 'client.require_model()' not in text)
    steps = []
    strip = show_chat.Strip(lambda share, said: steps.append((share, said)),
                            80)
    source, now = scripted()
    got = pull.pull('llama3.1:8b', source=source, now=now, rows=strip)
    report.check('every event lands on the strip and the stream still ends '
                 'in the daemon\'s word',
                 got == 'success' and len(steps) == len(script) + 1,
                 (got, len(steps)))
    halfway = [s for s in steps if ' 50 %' in s[1]]
    report.check('halfway the strip holds the layer\'s share and the figures '
                 'with no bar of their own',
                 len(halfway) == 1 and halfway[0][0] == 0.5
                 and halfway[0][1].startswith('PULLING llama3.1:8b')
                 and full not in halfway[0][1] and '=' not in halfway[0][1]
                 and 'of 4.9 GB' in halfway[0][1]
                 and '49 MB/s' in halfway[0][1]
                 and '50 s left' in halfway[0][1],
                 halfway[0] if halfway else steps[:2])
    report.check('the first row is the daemon\'s status word and the last '
                 'what was pulled, the bar snapped full',
                 steps[0] == (0.0, 'PULLING llama3.1:8b  pulling manifest')
                 and steps[-1] == (1.0, 'pulled llama3.1:8b, 4.9 GB in '
                                        '1 min 46 s'),
                 (steps[0], steps[-1]))
    narrow = show_chat.fit(halfway[0][1], 40) if halfway else ''
    report.check('on a narrow console the text is cut at a gap between '
                 'figures, never through one',
                 0 < len(narrow) <= 40 and narrow.endswith('%')
                 and halfway[0][1].startswith(narrow), narrow)


ROSTER = (
    (test_power_check_cannot_halt, ('link',)),
    (test_port_state, ('link',)),
    (test_link_recovery, ('link', 'reply')),
    (test_link_diagnose, ('link',)),
    (test_fallback, ('link', 'bus')),
    (test_pull_draws_the_daemons_numbers, ('link',)),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
