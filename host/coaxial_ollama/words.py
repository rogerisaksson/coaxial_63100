"""The prompt loop's fixed words: the system prompt, the hints, the tool sets, the help."""
import re

from . import tools as toolmod
from coaxial.comm import ports


# Every line earns its place, and each one replaced a measured failure: the
# wording history is in docs/MODELS.md, "Measured failure modes".
# The /board and /model line is there because a refusal was measured: asked to
# switch to a simulated board, gemma4:12b answered that it cannot switch
# hardware, being configured to talk to the physical board - accurate about
# itself, a dead end for the operator, and the same shape as
# BUILD_FIRMWARE_HINT below.
SYSTEM = """You are an expert on a coaxial BLDC inverter: the PCB behind an
outrunner's stator, not a cable. Modbus RTU over the probe's COM port or RS485.
Tools for the board, never to guess; off-topic needs none. Answer briefly,
no preamble.
Never a markdown table, never restate a tool's rows.
The noun decides, never "list": channels is the map, board_info. Values, or
a named pin, is a read - analog_read or digital_read, one call per kind.
What a thing IS is words, not a call.
Switching board or model is /board and /model - name it, do not refuse.
A call error, a typo or a wrong fact: say it, answer what was meant, never
hide it behind an old reading.
Any reading: analog_read only, never afe_power first - analog_read works
with the AFE on or off and reports which. Turning the AFE on or off itself
is the order to do it, not to discuss. Phase channels: unknown gain, pin
volts only."""

# Sent only when `docs` is offered, which no default set does.
DOCS_HINT = ("Values come from analog_read, never docs - HARDWARE and "
             "FINDINGS explain what a reading means, they do not produce "
             "one.")

# Sent only when build_firmware is offered.
BUILD_FIRMWARE_HINT = ("A question about building, compiling or flashing "
                       "this board's firmware - including 'can you' or "
                       "'do you' - is answered by calling build_firmware, "
                       "not by explaining that you cannot. You can: that is "
                       "what the tool is for. Never claim you cannot "
                       "compile or program this board.")

# Sent only when run_command is offered.
BUILD_HINT = ("To build or flash: run_command with cmd exactly "
             "'python tools/target/build_and_flash.py' (add --build-only or "
             "--flash-only). Not python3 - only python is allowlisted. "
             "No other command compiles or programs this board.")

# Sent only when link_diagnose is offered.
LINK_DIAGNOSE_HINT = ("A question about why the board is not answering, or "
                      "whether the link is down, is answered by calling "
                      "link_diagnose - not by guessing, not by trying "
                      "build_firmware or anything else. If it has not "
                      "already been called this turn, call it before "
                      "answering - on that question only, and never on any "
                      "other. Then be a troubleshooter, not a reporter: "
                      "turn the checklist into the next concrete thing to "
                      "check or do, in order, one step at a time - not the "
                      "raw step text back at the operator.")

# Named subsets: a debug job knows roughly what it will touch.
SETS = {
    'read': ('board_info', 'devices', 'analog_read', 'digital_read',
             'imu', 'angle', 'thermal', 'orientation', 'self_test',
             'afe_power', 'link', 'link_diagnose'),
    'code': ('board_info', 'devices', 'analog_read', 'digital_read',
             'imu', 'angle', 'thermal', 'orientation', 'self_test',
             'afe_power', 'link', 'run_python', 'build_firmware', 'run_tests',
             'link_diagnose'),
    'pins': ('board_info', 'devices', 'digital_read', 'gpio_pin', 'gpio_port',
             'test_gate', 'afe_power', 'link_diagnose'),
    # run_command, not build_firmware: the wider, allowlisted surface for a
    # session actually about the toolchain, not just build_and_flash.py.
    'build': ('board_info', 'run_command', 'run_tests', 'link_diagnose'),
    # For a question about the documents rather than the hardware.
    'docs': ('board_info', 'analog_read', 'docs', 'link_diagnose'),
    'all': tuple(spec['name'] for spec in toolmod.TOOLS if spec['name'] != 'report'),
    'none': (),
}

HELP = """  /py CODE      run python against the board, no model, no tokens
  /sh CMD       run an allowlisted command, no model
  /reconnect    drop and reopen the board link, no model
  /model [TAG]  swap the model, or auto; bare lists what is pulled
  /board [WHAT] simulated | auto | COM4 - what the tools talk to
  /node [N]     which node on the bus; 0 is every one; bare lists them
  /tools [set]  read|code|pins|build|docs|all|none, or a comma separated list
  /detail [x]   terse|full|auto - how much documentation the tools carry
  /confirm      toggle asking before every write - pin, run_python, run_command
  /lang [NAME]  show, set, or /lang auto to unlock the session's language
  /ctx          what the next turn will cost
  /clear        forget the conversation - the cheapest thing here
  /history      every question asked this session, numbered
  /clear_history   empty that list - separate from /clear, which is the
                    model's own memory, not this
  /cost         tokens so far
  /help  /q"""

# The board's name, not the script's: the useful thing to know in a window
# among several is which bench, not which program.
PROMPT = 'Coaxial 63100'

BLANK_LINE = '\n\n'

# What the operator can call a board, and what open_session takes for it.
BOARD_WORDS = {
    'simulerad': 'simulated', 'simulerat': 'simulated',
    'simulerade': 'simulated', 'simulated': 'simulated', 'sim': 'simulated',
    'stand': 'simulated', 'låtsaskort': 'simulated',

    'debugproben': 'auto', 'debugprobe': 'auto', 'debugprob': 'auto',
    'debugprobben': 'auto', 'proben': 'auto', 'probe': 'auto',
    'debuggern': 'auto', 'debugger': 'auto', 'jtag': 'auto', 'swd': 'auto',
    'stlink': 'auto', 'st': 'auto', 'link': 'auto',
    'riktig': 'auto', 'riktiga': 'auto', 'riktigt': 'auto', 'real': 'auto',
    'auto': 'auto', 'verkliga': 'auto', 'fysiska': 'auto',

    'rs485': 'rs485', 'rs': 'rs485', '485': 'rs485', 'fältbussen': 'rs485',
    'fieldbus': 'rs485',
}

# Verbs that order a swap rather than ask about one.
_BOARD_VERBS = ('byt', 'byta', 'byter', 'växla', 'växlar', 'koppla',
                'använd', 'ta',
                'switch', 'switches', 'change', 'use', 'connect', 'go')

# What stops an order from being one.
_QUESTION_WORDS = ('vad', 'vilken', 'vilket', 'vilka', 'varför', 'hur',
                   'när', 'vem',
                   'what', 'which', 'why', 'how', 'when', 'whether')

# Another thing to do in the same sentence.
_OTHER_ACTIONS = ('läs', 'läser', 'mät', 'mäter', 'visa', 'visar', 'lista',
                  'ge', 'beskriv', 'förklara', 'bygg', 'flasha', 'testa',
                  'kör', 'skriv', 'sätt', 'slå',
                  'read', 'measure', 'show', 'list', 'give', 'describe',
                  'explain', 'build', 'flash', 'test', 'run', 'write',
                  'set', 'turn')

_COM_PORT = re.compile(r'^com\d+$', re.I)


def board_switch(text):
    """The board `text` orders a swap to, when it orders nothing else."""
    words = [w.lower() for w in re.findall(r'[^\W_]+', text or '')]
    if not any(w in _BOARD_VERBS for w in words):
        return None
    ports = [w.upper() for w in words if _COM_PORT.match(w)]
    targets = [BOARD_WORDS[w] for w in words if w in BOARD_WORDS]
    if not targets and not ports:
        return None
    if any(w in _QUESTION_WORDS or w in _OTHER_ACTIONS for w in words):
        return None
    # A named port beats a kind: "switch to COM7" said which one.
    if ports:
        return ports[0]
    if 'simulated' in targets:
        return 'simulated'
    return 'rs485' if 'rs485' in targets else 'auto'


# What /help opens with.
ROLE = 'Senior engineer for this inverter: firmware, AFE, Modbus, live link.'

BUILDS = 'Builds and flashes it too: build_firmware, run_tests.'
