#!/usr/bin/env python3
"""The serial link: ports, probing, diagnosis, recovery.

Split out of test_ollama.py, which had grown to 5,496 lines and 733 checks in
one file - a third of every check this tree has, and the reason a coverage
tier could not be asked for at any useful resolution. One subject per file
now, so a tier buys them separately and a reader opens the one they meant.

Run from the host directory:  python tests/test_ollama_link.py
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tests.ollama_support import (ConnectError, Scope, ScriptedModel, 
    SimulatedSession, _Held, _NotATty, call, detail, io, toolmod)   # noqa: E402

def test_power_check_cannot_halt(report):
    """Diagnosing the link must not be able to break it.

    Measured on this bench, and it cost most of a session: `find_board.py
    --power` timed out at 15s, and every serial call after it was silent -
    the console said nothing, a raw Modbus frame said nothing - until
    `-c port=SWD mode=UR --start` brought the board back. The programmer
    had been killed mid-connect-under-reset with NRST asserted, and a
    halted core answers nothing on USART3.

    link_diagnose calls check_power as step 1 and then asks in step 4
    whether the board answers, so the checklist was able to cause the
    silence it went on to report.
    """
    import find_board
    import subprocess

    seen = {}

    class Done:
        stdout, stderr, returncode = 'Voltage     : 3.27V', '', 0

    def spy(argv, **kw):
        seen['argv'] = list(argv)
        seen['timeout'] = kw.get('timeout')
        return Done()

    real_run = subprocess.run
    try:
        subprocess.run = spy
        voltage, _ = find_board.check_power()
    finally:
        subprocess.run = real_run

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
    """Why a port is not answering, not just that it is not.

    Measured, and it cost most of a session twice over: two `dbg.py`
    sessions had COM4 open, every `probe` read "silent", and the board was
    diagnosed as halted, started over SWD and reflashed - none of which was
    the matter with it. pyserial says exactly what happened; nothing was
    asking.
    """
    import find_board
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
                     find_board.port_state('COM4') == find_board.BUSY,
                     find_board.port_state('COM4'))

        def missing(*a, **kw):
            raise serial.SerialException(
                "could not open port 'COM99': FileNotFoundError(2, ...)")

        serial.Serial = missing
        report.check('a port that is not there is absent, not busy',
                     find_board.port_state('COM99') == find_board.ABSENT,
                     find_board.port_state('COM99'))
    finally:
        serial.Serial = real_serial

    # The class name is what decides it: the OS message is localised, and
    # matching "Access is denied" would have read this one as absent.
    report.check('and it is decided on the class name, not the OS wording',
                 'PermissionError' in open(find_board.__file__,
                                           encoding='utf-8').read())

    # The checklist stops guessing at it. Step 4 used to end on "check
    # nothing else has COM4 open" whether or not something did.
    box = toolmod.Toolbox(SimulatedSession())
    real_state = find_board.port_state
    real_power = find_board.check_power
    real_ports = find_board.list_ports
    try:
        find_board.port_state = lambda *a, **kw: find_board.BUSY
        find_board.check_power = lambda *a, **kw: (3.27, 'stubbed')
        find_board.list_ports = lambda: ['COM_TEST']
        box.session = _Held()
        checklist = box.call('link_diagnose', {})
    finally:
        find_board.port_state = real_state
        find_board.check_power = real_power
        find_board.list_ports = real_ports
    report.check('link_diagnose says the port is held, not that it might be',
                 'open in another process' in checklist,
                 checklist.splitlines()[-1][:56])

def test_link_recovery(report):
    """One screen, one verdict about the link.

    Measured at the prompt, all of this on a single question: the model
    called link_diagnose, whose step 4 said the board did not answer; the
    turn then ended with no answer, `_probe_link` failed, dropped the dead
    handle, and reported the link down - and `_link_down_message` ran
    link_diagnose a *second* time, which opened the port cleanly and said
    "4. Kortet svarar pa COM4 just nu: ja - lanken ar uppe."

    Two checklists, contradicting each other, seconds apart, with no way
    for the operator to tell which was true. Both halves were host bugs:
    the retry the reset exists for was never taken, and the one call site
    that could not afford to reprint the checklist was the one that never
    passed `shown`.
    """
    from coaxial_ollama import debug

    # Deliberately not the real step-4 wording: that one is in
    # language.PHRASES, so the trace comes back in the session's language
    # while the fake's return value stays English, and the count then
    # measures localisation rather than duplication. What is under test
    # here is only that the diagnosis is not printed twice.
    STEP4_UP = 'DIAGNOSIS MARKER - the link is up'
    DEAD = 'ERR NoReplyError: unit 1, fc 0x47: silence'

    class Flapping:
        """A link that answers only once the stale handle is dropped - the
        shape of a VCP that re-enumerated under a replugged cable. Stands
        in for both the toolbox and its session, which is all `_probe_link`
        and `_link_down_message` touch."""

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

    # End to end, the shape of the transcript: the model calls
    # link_diagnose and then writes nothing, so the turn falls to the
    # stale path. Trace and answer together are the operator's screen.
    screen = io.StringIO()
    box = Flapping(99)
    talk = debug.Chat(ScriptedModel([
        call('link_diagnose'),
        {'role': 'assistant', 'content': ''},
    ]), box, out=screen)
    talk.toolbox = box                      # the fake stands in for both
    # Not "byt till..." any more: board_switch intercepts that before the
    # model is reached, and this needs a question that actually runs a turn.
    answer = talk.ask('vad läser NTC:n?')
    whole = screen.getvalue() + chr(10) + answer
    report.check('one screen carries the checklist once, not twice',
                 whole.count(STEP4_UP) == 1, '%d copies' % whole.count(STEP4_UP))
    report.check('so it cannot say the link is up and down at once',
                 not (STEP4_UP in answer and 'link is down' in answer),
                 answer.splitlines()[0][:46] if answer else '<empty>')

def test_link_diagnose(report):
    """OS-level, not another board round trip - see tools.py's own docstring
    for why. Ports come from a fake serial.tools.list_ports.comports() here,
    never from real hardware; coaxial.connect and find_board.check_power are
    faked too, for the same reason - both would otherwise probe whatever is
    really plugged into this bench and pass (or fail) for the wrong reason
    on a machine where it happens to answer."""
    import coaxial
    import find_board
    import serial.tools.list_ports as list_ports
    from types import SimpleNamespace

    class FakePort:
        def __init__(self, device):
            self.device = device

    real_comports = list_ports.comports
    real_connect = coaxial.connect
    real_check_power = find_board.check_power
    real_port_state = find_board.port_state
    try:
        # Stubbed for the same reason as the other three: it opens a real
        # port. Measured with the debug probe pulled - COM4 still enumerated,
        # opening it raised ACCESS_DENIED, port_state said BUSY, and a check
        # about step 4's closing advice failed because the checklist stopped
        # one step earlier. A suite that passes with a cable in and fails with
        # it out is testing the bench.
        find_board.port_state = lambda *a, **kw: find_board.SILENT
        list_ports.comports = lambda: [FakePort('COM4'), FakePort('COM7')]
        coaxial.connect = lambda *a, **kw: (_ for _ in ()).throw(
            ConnectError('nothing answered'))
        find_board.check_power = lambda timeout=15: (3.30, 'fake: powered')

        missing = toolmod.Toolbox(SimpleNamespace(port='COM9', baud=115200, unit=1))
        result = missing.call('link_diagnose', {})
        report.check('powered, but a configured port absent from the OS '
                     'list is named as such, not folded into a generic '
                     'error',
                     'COM9' in result and 'not among' in result, result)

        present = toolmod.Toolbox(SimpleNamespace(port='COM4', baud=115200, unit=1))
        result2 = present.call('link_diagnose', {})
        report.check('powered and present, but silent, points at nothing '
                     'else having the port open, not the cable',
                     'COM4' in result2
                     and 'answers on COM4 right now: no' in result2, result2)

        coaxial.connect = lambda *a, **kw: []                # "answers"
        result2b = present.call('link_diagnose', {})
        report.check('and a port that actually answers says the link is '
                     'up, not "silent" just because it exists',
                     'link is up' in result2b, result2b)

        list_ports.comports = lambda: []
        empty = toolmod.Toolbox(SimpleNamespace(port='COM4', baud=115200, unit=1))
        result3 = empty.call('link_diagnose', {})
        report.check('no COM ports at all is named plainly',
                     'Nothing is enumerating' in result3, result3)

        find_board.check_power = lambda timeout=15: (0.0, 'fake: no power')
        unpowered = toolmod.Toolbox(SimpleNamespace(port='COM4', baud=115200, unit=1))
        result4 = unpowered.call('link_diagnose', {})
        report.check('no target power stops the checklist at step 1, before '
                     'even listing COM ports - later steps cannot explain '
                     'more than the first one already does',
                     'no power sensed' in result4
                     and 'COM ports Windows sees' not in result4, result4)
        find_board.check_power = lambda timeout=15: (3.30, 'fake: powered')

        # It names the stand-in it is actually on and the way off it. The
        # line it replaced said "--no-board or --simulated this run" for a
        # session that had been given neither - it fell back on its own -
        # and that was the whole answer on screen to "byter du till
        # debugproben".
        stood_in = toolmod.Toolbox(SimulatedSession()).call('link_diagnose', {})
        report.check('a session with no port names the stand-in it is on, '
                     'and the way off it',
                     'simulated board' in stood_in and '/board auto' in stood_in,
                     stood_in[:52])
        report.check('and never claims a flag the operator did not type',
                     '--simulated' not in stood_in, stood_in[:52])
        from coaxial_ollama.debug import NoBoard
        refused = toolmod.Toolbox(NoBoard()).call('link_diagnose', {})
        report.check('--no-board is the one case that did get the flag',
                     refused.startswith('--no-board this run')
                     and '/board auto' in refused, refused[:52])

        # Ungated: no --allow-writes, no --confirm, no --read-only. It never
        # touches the board's state or its flash, same reasoning as `docs`.
        ro = toolmod.Toolbox(SimulatedSession(), allow_code=False)
        report.check('link_diagnose works even with --read-only',
                     not str(ro.call('link_diagnose', {})).startswith('ERR'))

        # The stand-in `open_session` actually returns, not the double at
        # the top of this file - that one has no `port` at all, so it
        # reached the branch above however the branch was written. The real
        # one's `port` is a bus label ('AX'), never None, and asking
        # `configured is None` alone sent a fallen-back session through a
        # 15s SWD probe and then "Configured port AX: not among the ports
        # above - the cable may be unplugged", about a session that never
        # had a cable. Measured with the board's JTAG connector pulled.
        from coaxial.simulated import SimulatedSession as FellBack
        probed = []
        find_board.check_power = lambda timeout=15: (probed.append(1),
                                                     (3.30, 'fake'))[1]
        fell_back = toolmod.Toolbox(FellBack()).call('link_diagnose', {})
        report.check('the stand-in open_session returns is known by its own '
                     'marker, not by a port that happens to be None',
                     'simulated board' in fell_back, fell_back[:52])
        report.check('and no SWD probe is spent on a session with no SWD',
                     not probed, '%d call(s)' % len(probed))

        # Step 4's closing advice used to open with "Powered" whatever step
        # 1 concluded - on a pulled cable, asserting the one thing that was
        # false and pointing at a busy port and a halted core instead.
        list_ports.comports = lambda: [FakePort('COM4')]
        coaxial.connect = lambda *a, **kw: (_ for _ in ()).throw(
            ConnectError('nothing answered'))
        find_board.check_power = lambda timeout=15: (None, 'fake: unknown')
        unsure = toolmod.Toolbox(SimpleNamespace(port='COM4', baud=115200,
                                                 unit=1))
        result5 = unsure.call('link_diagnose', {})
        report.check('a step 1 that could not check never closes by '
                     'asserting the board is powered',
                     'Power unconfirmed' in result5
                     and 'Powered and the port is right' not in result5,
                     result5.splitlines()[-1][:60])

        find_board.port_state = lambda *a, **kw: find_board.BUSY
        held = toolmod.Toolbox(SimpleNamespace(port='COM4', baud=115200,
                                               unit=1))
        result6 = held.call('link_diagnose', {})
        report.check('a port another process holds says so, rather than '
                     'guessing at a halted core',
                     'open in another process' in result6,
                     result6.splitlines()[-1][:58])
        find_board.port_state = lambda *a, **kw: find_board.SILENT

        # check_power's own timeout path. The programmer prints the voltage
        # in its first second, then spends the rest on a second connect at
        # 8MHz - measured with no target, 30.3s against a 15s budget. The
        # reading is in what it wrote before it was killed; returning None
        # reported "unknown" for the one case this check exists to answer.
        import build_and_flash
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
        coaxial.connect = real_connect
        find_board.check_power = real_check_power
        find_board.port_state = real_port_state



# ---- what reaches the screen, and how it reads -----------------------------

def test_fallback(report):
    """No cable is not a failing test suite - it is a different board.

    `open_session` probes the port with the same Modbus round trip a tool
    call makes, and hands back the stand-in when nothing answers. What it
    must never do is leave the caller unable to tell which it got: every
    suite and the prompt itself print it, and that is what these check.
    """
    from coaxial_mcp.session import open_session
    from coaxial_ollama import debug
    from coaxial_ollama import spinner as spin

    session, found = open_session(simulated=True)
    report.check('forced simulated skips the search entirely',
                 type(session).__name__ == 'SimulatedSession'
                 and not found.real, type(session).__name__)
    report.check('and is labelled Simulated, not by a port it never opened',
                 found.label == 'Simulated', found.label)

    # The label names the path, not just the port: a reading over the bench
    # cable and one over the field bus are not the same measurement, and
    # which it was has to be on screen rather than inferred from a COM
    # number. The probe is told apart by its USB VID - measured here, an
    # STLINK-V3SET enumerates 0483:374F - so nothing has to be opened to
    # know which port is the debugger.
    from coaxial_mcp import session as sessionmod
    for real, port, kind, want in ((True, 'COM3', 'probe', 'JTAG and COM3'),
                                   (True, 'COM5', 'serial', 'RS485 at COM5'),
                                   (False, None, None, 'Simulated')):
        report.check('label: %s' % want,
                     sessionmod._label(real, port, kind) == want,
                     sessionmod._label(real, port, kind))
    report.check('and says "simulated" where a firmware version goes',
                 session.board.version_info['firmware'] == 'simulated',
                 session.board.version_info['firmware'])

    # PB2 is the AFE switch, not a spare pin. Measured: writing 0 across
    # GPIOB left the stand-in answering `on=1` to afe_power one call later,
    # because the pin map and the switch were two dictionaries - the one
    # place invariant 9 could be broken by a stand-in with nobody noticing.
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

    # link_diagnose's step 4 opened the port a second time to ask whether
    # the board answers - while the session held it open. Measured live,
    # with the link up: "3. Configured port COM4: present." followed by
    # "4. Board answers on COM4 right now: no", a false statement about
    # live hardware produced by the diagnostic itself. The session's own
    # handle is asked first now, and only a session with none falls through
    # to the second open.
    class Held:
        """A session holding an open link, shaped like coaxial_mcp.Session."""
        port, baud, unit = 'COM_TEST', 115200, 1

        def __init__(self, board):
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

    # Steps 1 and 2 stubbed: the first shells out to STM32_Programmer_CLI
    # with a 15s timeout, and the second asks Windows what it has plugged
    # in - neither is what step 4 is being checked for, and both make the
    # answer depend on the desk the suite runs on.
    live.session = Held(SimulatedSession().board)
    power, ports = toolmod.find_board.check_power, toolmod.find_board.list_ports
    toolmod.find_board.check_power = lambda *a, **k: (3.27, 'stubbed')
    toolmod.find_board.list_ports = lambda: ['COM_TEST']
    try:
        checklist = live.call('link_diagnose', {})
    finally:
        toolmod.find_board.check_power = power
        toolmod.find_board.list_ports = ports
    report.check('so step 4 reports the link that is actually up',
                 'right now: yes' in checklist,
                 checklist.splitlines()[-1][:52])

    # /board: what the tools talk to, swapped without a restart. Measured at
    # the prompt: asked "byt till en simulerad hardvara", gemma4:12b answered
    # that it could not and was configured for the physical board - true
    # about itself, a dead end for the operator. The swap is the host's.
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
                 swap.command('/board').startswith('board: Simulated'),
                 swap.command('/board'))

    # An order to swap the board is the host's to carry out. Measured three
    # times on the same session, and it never once changed board: it refused
    # ("Jag kan inte byta till simulerad hardvara"), then diagnosed the link,
    # then read seven channels and wrote nothing. The operator was giving an
    # order, not asking a question, and the state is the host's either way -
    # the same argument as language.bare_switch.
    for question, want in (
            ('byt till debugproben', 'auto'),
            ('byter du till debugproben', 'auto'),
            ('byt till en simulerad enhet', 'simulated'),
            ('byt till simulerad hårdvara', 'simulated'),
            ('växla till COM4', 'COM4'),
            ('switch to the real board', 'auto'),
            ('byt till RS485', 'rs485'),
            ('byt till fältbussen', 'rs485'),
            # Every one of these lost the order to a single unlisted noun
            # while this required all the words to be known: 'enhet', then
            # 'hardvara', then 'lage'. The rule names what disqualifies an
            # order instead, so a noun nobody thought of costs nothing.
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

    # A pipe is not a console. Measured: `printf "byter du till simulerat
    # lage" | dbg --repl` arrived as `lÃ¤ge` under cp1252, which splits into
    # `lã` and `ge` - and `ge` disqualifies a board order as a second
    # request. The order went to the model, which refused it. stdin gets the
    # same treatment as the two outputs now: UTF-8 when it is not a tty.
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
    # Ordered a real board and found none. Patched, not left to whatever is
    # plugged into this bench: the first version of this check passed only
    # while the board happened to be silent, and started failing the moment
    # it answered again.
    import coaxial_mcp.session as sessionmod
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
    # working. Measured: "byt till debugproben" twice in a row on a bench
    # whose board had gone silent, both times "inget svarade" - and had the
    # session been on a live probe, the first of those would have dropped it
    # for a stand-in.
    import coaxial_mcp.session as sessionmod

    class Live:
        """A session that is already on a real board."""
        port, baud, unit = 'COM4', 115200, 1
        closed = False

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
        said = keeper.command('/board rs485')
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

    # /model: same idea one layer up. No weights are loaded here - every path
    # below either refuses or is a no-op, which is the whole logic.
    from coaxial_ollama.client import Ollama, OllamaError
    swap.client = Ollama('gemma4:12b', keep_alive=0)
    swap.detail, swap.tool_names = detail.TERSE, ()
    try:
        before = swap.client.model
        said = swap.command('/model no-such-tag:9b')
        report.check('/model refuses a tag that is not pulled, and swaps '
                     'nothing', 'not pulled' in said
                     and swap.client.model == before, said[:52])
        report.check('/model on the tag already running is a no-op',
                     swap.command('/model gemma4:12b') == 'model: gemma4:12b '
                     'already', swap.client.model)
    except OllamaError as exc:
        report.check('/model needs a local ollama daemon', False, str(exc)[:52])

    class VT(io.StringIO):
        encoding = 'utf-8'

        def isatty(self):
            return True

    # 'all' is the broadcast address, and it is the one mode where a
    # command reaches every inverter on the bus and nothing answers to
    # say it landed. Red is the colour already spent on 'something is
    # wrong here', which is the right register for it.
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


ROSTER = (
    (test_power_check_cannot_halt, ('link',)),
    (test_port_state, ('link',)),
    (test_link_recovery, ('link', 'reply')),
    (test_link_diagnose, ('link',)),
    (test_fallback, ('link', 'bus')),
)


if __name__ == '__main__':
    from tests.ollama_support import run_file
    sys.exit(run_file(ROSTER))
