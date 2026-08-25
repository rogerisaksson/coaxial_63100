"""A lean prompt loop for debug jobs: fewest tokens in, fewest tokens out.

    python dbg.py "the NTC reads exactly 25.00 - what is wrong?"
    python dbg.py -q "which channel is the DC link?"
    python dbg.py --repl                 # interactive

runner.py is built for a test plan a person signs; this is the same board and
tools for a question asked sixty times an afternoon. What makes it cheap:

  * ~70 tokens of system prompt, not 350.
  * five tools by default, not eleven - the list is re-sent every turn, so it
    is the cost that scales with the conversation. `--tools` picks the subset.
  * `num_predict` caps the answer, `think` off where supported.
  * old turns stubbed, not resent whole (trim, and context.fit behind it).
  * slash commands run without the model at all: `/py`, `/sh` cost nothing.

Turn cost is tracked, not printed - `/cost`, and `--budget` to stop. Printing
it every turn was noise between the question and its answer.
"""
import json
import os
import re
import sys
import textwrap
import threading
import time

# host/ on the path: this file's own directory's parent, so it does
# not matter what the working directory is or what any directory
# along the way is called.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial.errors import RigError                  # noqa: E402
from coaxial_mcp import detail                       # noqa: E402
from coaxial_mcp import render                       # noqa: E402

from . import context                                # noqa: E402
from . import intent
from . import language                               # noqa: E402
from . import replies                                # noqa: E402
from . import tools as toolmod                       # noqa: E402
from . import spinner as spin                        # noqa: E402
from .context import approx_tokens                   # noqa: E402
from .iolog import IOLog                             # noqa: E402
from .sandbox import clip                            # noqa: E402

# Every line earns its place, and each one replaced a measured failure:
# the wording history is in docs/MODELS.md, "Measured failure modes".
# Two that are easy to undo by accident:
#
#   * "The noun decides, never list" - three earlier wordings lost to
#     "ge mig en lista over de analoga vardena", which has the word for
#     a map and the word for a read in one sentence.
#   * the first line names the PCB and the link, because "an expert with
#     a serial link to a coaxial BLDC inverter" put two words next to
#     each other and the model fused them into a coaxial cable, twice.
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

# The /board and /model line is there because a refusal was measured: asked
# "byt till en simulerad hardvara", gemma4:12b answered "Jag kan inte byta
# till simulerad hardvara. Jag ar konfigurerad for att interagera med den
# fysiska kretskortet" - accurate about itself, a dead end for the operator,
# and the same shape as BUILD_FIRMWARE_HINT below. It cannot do the swap; it
# can say which command does.

# Sent only when `docs` is offered, which no default set does. Measured:
# asked to *measure* the channels, gemma4:12b read HARDWARE.md instead and
# answered with that document's channel table - no measurement in it.
DOCS_HINT = ("Values come from analog_read, never docs - HARDWARE and "
             "FINDINGS explain what a reading means, they do not produce "
             "one.")

# Sent only when build_firmware is offered. The model's training says a chat
# assistant cannot flash hardware, and that belief beat the schema: measured,
# gemma4:12b answered "Nej, jag programmerar inte firmwaren sjalv" with
# build_firmware sitting in its own tool list, never called.
BUILD_FIRMWARE_HINT = ("A question about building, compiling or flashing "
                       "this board's firmware - including 'can you' or "
                       "'do you' - is answered by calling build_firmware, "
                       "not by explaining that you cannot. You can: that is "
                       "what the tool is for. Never claim you cannot "
                       "compile or program this board.")

# Sent only when run_command is offered. Spelled out because guessing failed:
# measured, the first two tries were `python3` (not allowlisted) and
# `python build_and_flash.py`, one directory short of tools/.
BUILD_HINT = ("To build or flash: run_command with cmd exactly "
             "'python tools/build_and_flash.py' (add --build-only or "
             "--flash-only). Not python3 - only python is allowlisted. "
             "No other command compiles or programs this board.")

# Sent only when link_diagnose is offered. A tool in the schema is not a tool
# the model reaches for: asked why the board was silent, it called
# build_firmware - a guess at a fix, not a diagnosis. The automatic path (a
# board call that failed this turn) is handled in ask(); this is for the
# question asked on its own.
#
# "on that question only, and never on any other" is not padding. Without it
# the sentence before it - "if it has not already been called this turn, call
# it before answering" - reads as a standing order, and that is how the model
# read it: measured across three transcripts, link_diagnose ran first on
# "byt till en simulerad enhet", on "byter du till debugproben" and on "vilka
# analoga och digitala kanaler finns", none of which is about the link.
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

# Named subsets: a debug job knows roughly what it will touch. `docs` is in
# none of them but its own and `all` - see DOCS_HINT.
SETS = {
    'read': ('board_info', 'devices', 'analog_read', 'digital_read',
             'self_test', 'afe_power', 'link', 'link_diagnose'),
    'code': ('board_info', 'devices', 'analog_read', 'digital_read',
             'self_test', 'afe_power', 'link', 'run_python', 'build_firmware',
             'run_tests', 'link_diagnose'),
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
# 'simulated' wins wherever it appears: "en simulerad enhet" names both a
# kind and a thing, and the qualifier is the half that decides.
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

# Verbs that order a swap rather than ask about one. Without the verb,
# "vad ar debugproben" reads as an order because it names one.
#
# 'kor' is deliberately absent: "kor mot simulerat" and "kor testerna" are
# the same word doing opposite jobs, and only one of them is this.
_BOARD_VERBS = ('byt', 'byta', 'byter', 'växla', 'växlar', 'koppla',
                'använd', 'ta',
                'switch', 'switches', 'change', 'use', 'connect', 'go')

# What stops an order from being one. This was a list of allowed filler
# words and every word outside it abstained - which meant a noun nobody had
# thought of was enough to lose the order. Measured four times, one word
# each: 'enhet', then 'hardvara', then 'lage'. Naming what disqualifies an
# order is a closed set; naming every noun that does not is not.
# Interrogatives only. 'om' and 'ifall' are subordinating conjunctions, not
# questions, and 'om' is also the particle in "koppla om" - listing it lost
# that order to its own verb.
_QUESTION_WORDS = ('vad', 'vilken', 'vilket', 'vilka', 'varför', 'hur',
                   'när', 'vem',
                   'what', 'which', 'why', 'how', 'when', 'whether')

# Another thing to do in the same sentence. "byt till simulerat lage och
# las NTC:n" is two requests, and the model is the one that can carry out
# both; the host taking the first half silently would drop the second.
_OTHER_ACTIONS = ('läs', 'läser', 'mät', 'mäter', 'visa', 'visar', 'lista',
                  'ge', 'beskriv', 'förklara', 'bygg', 'flasha', 'testa',
                  'kör', 'skriv', 'sätt', 'slå',
                  'read', 'measure', 'show', 'list', 'give', 'describe',
                  'explain', 'build', 'flash', 'test', 'run', 'write',
                  'set', 'turn')

_COM_PORT = re.compile(r'^com\d+$', re.I)


def board_switch(text):
    """The board `text` orders a swap to, when it orders nothing else.

    "byt till debugproben" is the host's to carry out - the same shape as
    a bare language switch, and for the same reason: the session's board
    is host state, and a model asked to change it can only describe or
    refuse. Measured three times, it did both and then read a channel.

    None when the sentence asks rather than orders ("vad ar debugproben"),
    or carries a second request the host cannot do ("byt till simulerat
    lage och las NTC:n"). Anything else with a switch verb and a target is
    an order.

    This used to require every word to be in a list of allowed filler, and
    abstained on anything else. That lost the order to one unlisted noun,
    four times running - 'enhet', 'hardvara', 'lage'. What disqualifies an
    order is a closed set; what may appear in one is not.
    """
    words = [w.lower() for w in re.findall(r'[^\W_]+', text or '')]
    if not any(w in _BOARD_VERBS for w in words):
        return None
    ports = [w.upper() for w in words if _COM_PORT.match(w)]
    targets = [BOARD_WORDS[w] for w in words if w in BOARD_WORDS]
    if not targets and not ports:
        return None
    if any(w in _QUESTION_WORDS or w in _OTHER_ACTIONS for w in words):
        return None
    # A named port beats a kind: "byt till COM7" said which one.
    if ports:
        return ports[0]
    if 'simulated' in targets:
        return 'simulated'
    return 'rs485' if 'rs485' in targets else 'auto'


# What /help opens with. The second line only when the tools behind it are
# loaded: measured, the model denied being able to flash with build_firmware
# sitting in its own list, and an operator who never asks is never told.
ROLE = 'Senior engineer for this inverter: firmware, AFE, Modbus, live link.'
BUILDS = 'Builds and flashes it too: build_firmware, run_tests.'


ERR_CLASS = re.compile(r'^ERR (\w+):')

# Tools that actually reach the board - not 'docs', which reads local files and
# proves nothing about the link either way. Shared with runner.py's own use of
# the same set, for the same reason: a docs call proves nothing either way.
LINK_TOOLS = toolmod.LINK_TOOLS

# render.error's class name, for the calls that report through it. Not every
# RigError means the cable is out: DeviceStateError, UnsupportedProtocolError
# and a Modbus exception all mean the board answered - just refused, or was
# not in a state the request could mean anything in. Only these five mean the
# round trip itself did not happen.
CONTACT_LOST = {'ConnectError', 'NoReplyError', 'CrcError', 'FrameError',
                'PayloadError'}

# Rows of a tool result shown in full - every channel, or a whole self_test,
# without an unbounded dump from run_python.
TRACE_ROWS = 24

# Width before a row continues on the next line, and the lines one row may
# take. Wrapped, not cut: readings are fixed-column and inside this, but
# link_diagnose's prose was being cut mid-word.
TRACE_WIDTH = 96
TRACE_LINES = 3

# Tools where the same call twice means something: a reading changes with
# time, and code knows why it is running again. Everything else repeated in
# one turn is the model not noticing it already has the answer - measured,
# qwen2.5:14b turned the AFE on four times in a row.
REPEATABLE = {'analog_read', 'run_python', 'run_command'}


AFE_STATE = re.compile(r'^on=(\d)')


def _afe_noise(name, args, raw):
    """Whether this afe_power result is worth a line on screen.

    Two are not. A call refused for not having been asked for is a mistake
    the model recovers from one call later - the refusal stays in history
    for it to read, and the operator does not need it.

    And a switch that did what it was told: "sla pa afen" traced `on=1
    pe15=0` above an answer that said the same thing in words. `on=1` is
    also the only evidence the write landed, so the line goes only when the
    read-back **matches what was asked**. A request that did not take, a
    toggle (nothing to match it against), a plain read (the state is the
    answer) and every error all still print.
    """
    if name != 'afe_power':
        return False
    if str(raw).startswith('ERR not asked for'):
        return True
    wanted = str((args or {}).get('action', 'read')).strip().lower()
    if wanted not in ('on', 'off'):
        return False
    got = AFE_STATE.match(str(raw))
    return bool(got) and (got.group(1) == '1') == (wanted == 'on')


def _wrapped(line):
    """One traced row as the lines it takes on screen, indented.

    A row that fits comes back untouched - every reading this board produces
    is one. Only prose wraps, and the continuation keeps the row's own indent
    so a numbered checklist still reads as one.
    """
    body = '  %s' % line.rstrip()
    if len(body) <= TRACE_WIDTH:
        return [body]
    lead = len(line) - len(line.lstrip())
    parts = textwrap.wrap(body, width=TRACE_WIDTH,
                          subsequent_indent=' ' * (4 + lead),
                          break_long_words=False, break_on_hyphens=False)
    if not parts:
        return [body[:TRACE_WIDTH]]
    if len(parts) <= TRACE_LINES:
        return parts
    parts = parts[:TRACE_LINES]
    parts[-1] += ' [...]'
    return parts


def _printable(stream):
    """Make a console survive an alphabet that is not its codepage.

    Applied to stdin as well as the two outputs, and for the same reason in
    reverse. Measured: `printf "byter du till simulerat lage" | dbg --repl`
    arrived as `simulerat lÃ¤ge` under cp1252, which split into `lã` and
    `ge` - and `ge` is one of the verbs that disqualifies a board order, so
    the order went to the model instead of being carried out. A tty is fine
    either way, since the console API hands over real Unicode.

    A console keeps its own codepage and only gains errors='replace': under
    cp1252 an ohm sign or any Cyrillic would otherwise raise
    UnicodeEncodeError and lose a reading already taken. Forcing UTF-8 on a
    console that is not set to it trades that for mojibake in the languages
    this loop actually speaks.

    A file or a pipe is the opposite case and gets UTF-8: no codepage to
    mismatch, and the locale default here turned every Swedish answer in
    `dbg.py > session.txt` into question marks.
    """
    try:
        if stream.isatty():
            stream.reconfigure(errors='replace')
        else:
            stream.reconfigure(encoding='utf-8', errors='replace')
    except (AttributeError, OSError, ValueError):
        pass
    return stream


class Turn:
    """What one question accumulates while it is being answered.

    Eight locals threaded through a 250-line loop is what made that loop five
    branches deep. Named here, each stage of the turn is short enough to read
    on one screen, and what the loop *knows* - as against what the model
    wrote - is a list rather than a habit.
    """

    NUDGE_LIMIT = 2        # then end the turn rather than ask nicely forever

    def __init__(self):
        self.answer = ''
        self.link_error = None      # a call that did not reach the board
        self.code_error = None      # last run_python/run_command that failed
        self.channels = None        # names in this turn's analog_read table
        self.table = None           # and that table, for --quiet
        self.maps = []              # name sets a map or a level read listed
        self.map_text = None        # and that render, for --quiet
        self.diagnosed = False      # link_diagnose ran, and was traced
        self.seen = {}              # (name, args) -> its rendered result
        self.nudges = 0

    def nudge(self, chat, say, giving_up):
        """Ask once more, or give up. None means go round again."""
        if self.nudges < self.NUDGE_LIMIT:
            self.nudges += 1
            chat.history.append({'role': 'user', 'content': say})
            return None
        return giving_up

    def remember_map(self, text):
        """The channel names a map or a level read just put on screen.

        Deliberately not chat.last_channels: that one means "a reading has
        succeeded this session" and gates the answering-from-memory checks.
        A map is not a reading.

        Three ways the same rows can be named back - an analog channel, a
        digital pin, or that pin's signal - kept as alternatives rather than
        one union, because the answer quotes one of them, not all three, and
        a union would need every name from every column present.
        """
        sets = [set(m.lower() for m in pattern.findall(text))
                for pattern in (replies.MAP_ROW, replies.DIGITAL_ROW,
                                replies.DIGITAL_SIGNAL)]
        sets = [names for names in sets if names]
        if sets:
            self.maps = sets
            self.map_text = text


class Chat:
    """One conversation, trimmed on the way out to the model.

    The history is kept whole locally - it costs nothing on this side, and a
    stubbed line is impossible to un-stub. What goes over the wire is the trim.
    """

    def __init__(self, client, toolbox, tools='read', keep=6, budget=0,
                 quiet=False, out=None, link_ok=True, detail_level=detail.AUTO,
                 session_language=None):
        self.client = client
        self.toolbox = toolbox
        # Before set_tools below, which builds the schemas this decides the
        # length of. `auto` reads the model's tag: the tags this loop runs are
        # 8B to 14B and pay for every description out of the same 8192 tokens
        # the readings come out of, so auto lands on terse here and on full
        # for anything big enough not to care.
        self.detail = detail.resolve(detail_level,
                                     model=getattr(client, 'model', None),
                                     default=detail.TERSE)
        toolbox.detail = self.detail
        self.keep = keep
        self.budget = budget
        self.quiet = quiet
        self.out = out or _printable(sys.stdout)
        # Shared with the REPL's spinner: it repaints on its own thread, and
        # unsynchronised writes to one stream interleave into garbage. RLock,
        # not Lock: _trace() holds it across its print()s and
        # spinner._Tracked.write() re-enters it on each one.
        self.print_lock = threading.RLock()
        self.history = []
        self.turn_cost = []
        # What the prompt's spinner shows: read fresh on every prompt, so
        # /reconnect changes it on the very next line rather than needing a
        # restart to notice the cable was plugged back in.
        self.link_ok = link_ok
        # Names from the most recent successful analog_read, kept across turns
        # (unlike the turn-local copy in `ask`) so a later turn that answers
        # with no tool call at all can still be checked against it.
        self.last_channels = None
        # Where the tools are pointed when the session opens, so the first
        # answer does not announce a node nothing moved to. The prompt tag
        # already says where you are; this line is for when that changes.
        self._said_node = (getattr(toolbox.session, 'bus',
                                   getattr(toolbox.session, 'port', None)),
                           getattr(toolbox.session, 'unit', None))
        # The session's language. The machine's locale for a real run - the
        # operator is answered in their own language from the first word,
        # without a question having to prove it first - and None for a test,
        # deliberately: a suite that read the Windows locale would pass on one
        # machine and fail on the next. trim() moves it when a question is
        # actually in another language, or names one outright.
        self.language = session_language
        # Every question typed this session, in order - independent of
        # self.history, which the REPL clears after each answered turn.
        # /history reads it back, /clear_history empties it.
        self.prompt_history = []
        # Off by default: dozens of tests build a Chat and none should touch
        # the filesystem. repl() and main() turn it on for the real one.
        self.io_log = IOLog(enabled=False)
        # Compile the question into an intent before answering it. Off here
        # for the same reason as io_log, and off for a scripted plan: a step
        # written as `analog_read ch=4` has no ambiguity to resolve, and the
        # second model call would be spent on nothing. main() turns it on.
        self.compile_intent = False
        self._intent_why = self._intent_did = None
        self._intent_kind = self._intent_tool = None
        self.set_tools(tools)

    # ---- what the model is allowed to see ----------------------------------

    def set_tools(self, wanted):
        names = SETS.get(wanted)
        if names is None:
            names = tuple(n.strip() for n in str(wanted).split(',') if n.strip())
        known = {spec['name'] for spec in toolmod.TOOLS}
        unknown = [n for n in names if n not in known]
        if unknown:
            raise ValueError('no such tool: %s. Sets: %s'
                             % (', '.join(unknown), ', '.join(SETS)))
        self.tool_names = names
        self.schemas = toolmod.schemas(
            [spec for spec in toolmod.TOOLS if spec['name'] in names],
            self.detail) or None
        return names

    def set_detail(self, wanted=detail.AUTO):
        """How much of each tool's documentation this model gets.

        Resolved against the model's own tag, so `auto` on gemma4:12b is
        terse and `auto` on a frontier model over MCP is full - see
        coaxial_mcp/detail.py. Re-reads the tool list afterwards, because the
        level only exists in what goes over the wire.
        """
        model = getattr(self.client, 'model', None)
        self.detail = detail.resolve(wanted, model=model, default=detail.TERSE)
        self.toolbox.detail = self.detail
        if getattr(self, 'tool_names', None) is not None:
            self.set_tools(','.join(self.tool_names) or 'none')
        return self.detail

    def tool_cost(self):
        """What the tool list alone costs, every turn, before any question."""
        return approx_tokens(json.dumps(getattr(self, 'schemas', None) or []))

    def _lock_language(self, asked):
        """Move the session language, if this question moves it.

        Called from trim() and from a planned turn alike. Measured: a
        planned turn calls no trim(), so a Swedish question answered
        from a compiled plan left the lock on whatever the previous
        question set - the language suite failed only when it ran after
        the others, in one session, which is the only place it shows.
        """
        current = getattr(self, 'language', None)
        requested = language.requested_language(asked)
        detected = language.detect(asked)
        if requested and requested != current:
            current = requested
        elif detected and detected != current:
            current = detected
        self.language = current

    def trim(self):
        """System prompt, stubbed history, recent turns whole.

        Tool results are where the tokens are, so they are what gets stubbed:
        the first line of a channel table is enough to remember that it was
        read, and the model can read it again if it matters.
        """
        head = self.history[:-self.keep] if self.keep else self.history
        tail = self.history[-self.keep:] if self.keep else []

        # The language is named here, not worked out by the model: told to
        # do it itself, qwen2.5:14b answered a European question in Chinese.
        # It locks on the first unambiguous question and stays - a later
        # "tabellera" detects as nothing and would otherwise flip the prompt
        # back, reloading the cached prefix. Only a real switch (detect
        # disagrees) or a named language ("svara pa engelska") moves it.
        #
        # Read from prompt_history, not from the last role=='user' message:
        # a nudge is appended with that role too, and measured live, its
        # English words flipped a Swedish session on the next trim().
        asked = ''
        prompt_history = getattr(self, 'prompt_history', None)
        if prompt_history:
            asked = prompt_history[-1]
        else:
            for message in reversed(self.history):
                if message['role'] == 'user':
                    asked = message.get('content') or ''
                    break
        self._lock_language(asked)
        names = getattr(self, 'tool_names', ())
        hint = ''
        if 'build_firmware' in names:
            hint += '\n' + BUILD_FIRMWARE_HINT
        if 'run_command' in names:
            hint += '\n' + BUILD_HINT
        if 'link_diagnose' in names:
            hint += '\n' + LINK_DIAGNOSE_HINT
        if 'docs' in names:
            hint += '\n' + DOCS_HINT
        # The earlier questions, not the wiped history: "tabellera", then
        # "varfor kan du inte na kortet", then "provade det, fortfarande
        # inget" only reads as a sequence with them in view. Five at most.
        hint += getattr(self, 'intent', '') or ''
        prior = getattr(self, 'prompt_history', [])[-6:-1]
        if prior:
            hint += ('\nEarlier this session, in order: %s. Treat these as '
                     'troubleshooting steps already tried in this '
                     'conversation, not separate unrelated questions.'
                     % '; '.join('"%s"' % clip(q, 60) for q in prior))
        # Which model this is, from the tag the daemon was actually asked for
        # rather than from whatever the weights remember being called. Asked
        # at the prompt, a model with nothing told to it answers out of its
        # training - a name, a version and a maker, all three of which can be
        # wrong for a local tag someone quantised last week. Six tokens buys
        # an answer that matches `ollama ps`.
        # The tag verbatim, and asked for verbatim: told only "you are the
        # local model gemma4:12b", it answered "Jag ar Gemma 4" - aware of
        # what it is, and one paraphrase away from a name that no longer
        # matches `ollama ps` or a bug report.
        who = ('Your model tag is exactly "%s", run locally by ollama on this '
               'bench; give that tag verbatim if asked which model you are.'
               % getattr(getattr(self, 'client', None), 'model', 'unknown'))
        if 'build_firmware' in names:
            # Said as identity, not only as the instruction BUILD_FIRMWARE_HINT
            # carries: "what am I" and "what do I do when asked to flash" are
            # different questions, and the second hint never answered the
            # first. Conditional, because a tool set without build_firmware
            # cannot build anything and a system prompt claiming otherwise is
            # the same invention this loop exists to prevent.
            who += (' You are this board\'s build system too: you compile its'
                    ' firmware and program it over SWD yourself.')
        sent = [{'role': 'system',
                 'content': SYSTEM + '\n' + who + hint + '\n'
                           + language.instruction_for(self.language)}]
        for message in head:
            content = (message.get('content') or '').strip()
            if message['role'] == 'tool':
                first = content.splitlines()[0] if content else ''
                sent.append({'role': 'tool',
                             'content': clip(first, 80) + ' [...]'})
            else:
                # Assistant tool_calls are dropped along with the results they
                # go with: a call whose answer has been stubbed is noise.
                sent.append({'role': message['role'],
                             'content': clip(content, 200)})
        sent.extend(tail)
        return self._fit(sent)

    # ---- what actually fits ------------------------------------------------

    def prompt_budget(self):
        """Tokens the next prompt may take, of the window this model has.
        Zero when the client has no window to read - see context.budget_for.

        getattr for the client itself, not just its options, for the same
        reason trim() reaches for tool_names that way: a Chat built by a test
        to exercise one method has whatever that method needs on it and
        nothing else, and a budget is not what such a test is about.
        """
        client = getattr(self, 'client', None)
        return context.budget_for(getattr(client, 'options', None))

    def _fit(self, sent):
        """Whatever trim() decided to send, cut down to what the model can
        actually be handed. Two different jobs: trim() knows what is worth
        sending, context.fit knows what fits. The tool schemas count - they
        are re-sent every turn and come out of the same window."""
        return context.fit(sent, self.prompt_budget(), self.tool_cost())

    def context_cost(self):
        return context.cost(self.trim(), self.tool_cost())

    # ---- a turn ------------------------------------------------------------

    def _probe_link(self):
        """A live, free-standing check of the link - no AFE, no sample.

        For a turn about to answer from memory: the result is the fact, not
        what the model believes. Updates link_ok and the history exactly as a
        model-issued `link` call would. Not traced - nobody asked for link
        counters, and a failure becomes the turn's own answer anyway.
        """
        probe = self.toolbox.call('link', {'op': 'stats'})
        lost = ERR_CLASS.match(str(probe))
        self.link_ok = not (lost and lost.group(1) in CONTACT_LOST)
        if not self.link_ok:
            # Same reasoning as the main loop's own LINK_TOOLS handling: a
            # cable pulled and replugged can leave the cached board's serial
            # handle permanently dead, since a USB VCP re-enumerates on
            # replug rather than reviving the same handle. Reset here too,
            # or this exact probe - the one this whole recovery path exists
            # for - keeps failing forever on the same dead handle even once
            # the cable is back.
            self.toolbox.session.reset()
            #...and try once more, which is the whole point of the
            # reset. Measured: the handle was dropped, the turn answered
            # "linken ar nere", and link_diagnose one line later opened
            # the port cleanly and said it was up - two verdicts on one
            # screen, the second of them from the retry this path had
            # already earned and did not take.
            probe = self.toolbox.call('link', {'op': 'stats'})
            lost = ERR_CLASS.match(str(probe))
            self.link_ok = not (lost and lost.group(1) in CONTACT_LOST)
        self.history.append({'role': 'tool', 'tool_name': 'link',
                             'name': 'link', 'content': 'link: %s' % probe})
        return probe

    def _link_down_message(self, link_error, shown=False):
        """'link is down, not answered: ...', plus why - run here, by the
        host, every time the link is down, rather than left for the model
        to think to call link_diagnose. Measured: a raw
        ConnectError with a generic hint was read as "a bunch of error
        messages" - the actual, specific reason (which COM ports Windows
        sees right now, whether the configured one is even among them) is a
        fact this loop can just go get, not something worth gambling on the
        model reaching for the right tool.

        `shown` means the model already called link_diagnose this turn and
        its four-step checklist is on screen directly above. Measured with
        the board unplugged: every board question then printed that checklist
        twice - once clipped as a tool trace, once whole as the answer - and
        paid the ST-Link's fifteen-second timeout twice to do it. The same
        rule as the retyped table below: what the trace already shows is not
        worth saying again.

        The error keeps its class and its cause and loses its generic tail
        ('-> check the board is powered, and that a JTAG programmer...'):
        that advice exists for a reader with nothing else, and a reader with
        a specific four-step answer above is not that reader.
        """
        text = str(link_error)
        if shown:
            # The class and its first clause: 'ERR ConnectError: cannot open
            # COM9 at 115200 baud', not that plus the port name again, the
            # OS exception's own repr and the generic advice. All of it is on
            # screen above already; what this line is for is saying that the
            # question went unanswered and by what.
            head = text.split(' -> ')[0]
            return 'link is down, not answered: %s' \
                % clip(': '.join(head.split(': ')[:2]), 120)
        message = 'link is down, not answered: %s' % text
        try:
            diagnosis = self.toolbox.call('link_diagnose', {})
        except Exception:                                     # noqa: BLE001
            return message
        if diagnosis and not str(diagnosis).startswith('ERR'):
            message += '\n' + str(diagnosis)
        return message

    def ask(self, question, max_calls=6):
        """One question, however many tool calls it takes. Returns the
        answer - logs it to IOLog too, from the one place every path
        through _ask_inner's several returns ends up, rather than at each
        of them and risking a future one added without it."""
        # A message that is nothing but a language request never reaches the
        # model: the lock is host state, and the answer is one word. History
        # is left alone too, so the switch costs no prompt prefix either.
        switch = language.bare_switch(question)
        if switch:
            self.language = switch
            answer = language.okay(switch, getattr(self.out, 'encoding', None))
            self.io_log.turn(question)
            self.io_log.answer(answer)
            return answer
        # Same rule one layer out: an order to change the board is the
        # host's to carry out, not a model's to describe. Measured three
        # times on "byt till debugproben" - it refused, then diagnosed the
        # link, then read seven channels, and the board never changed.
        board = board_switch(question)
        if board:
            answer = language.localise(self._switch_board(board),
                                       self.screen_language())
            self.io_log.turn(question)
            self.io_log.answer(answer)
            return answer
        answer = language.localise(self._ask_inner(question, max_calls),
                                   self.screen_language())
        answer = self._say_node(answer)
        self.io_log.answer(answer)
        return answer

    def _say_node(self, answer):
        """Name the node above the answer, on the turn it changed.

        The prompt tag says where the tools are pointed, but a scrolled-back
        answer carries no prompt with it - and on a machine of twenty nodes
        "the NTC is 36.6 C" is not an answer without one. Said once, on the
        turn the node moved, rather than on every turn: a line repeated for
        no reason is the noise the retype backstop exists to cut.
        """
        session = self.toolbox.session
        here = (getattr(session, 'bus', getattr(session, 'port', None)),
                getattr(session, 'unit', None))
        if here[1] is None or here == getattr(self, '_said_node', None):
            return answer
        self._said_node = here
        if here[1] == 0:
            where = 'every node on %s' % (here[0] or 'the bus')
        else:
            where = None
            try:
                from coaxial.simulated import bus_nodes
                if here[0]:
                    where = (bus_nodes(here[0]).get(here[1])
                             or (None, None, None))[2]
            except Exception:                                 # noqa: BLE001
                pass
            where = where or ('%s node %d' % (here[0], here[1]) if here[0]
                              else 'node %d' % here[1])
        line = language.localise('From %s:' % where, self.screen_language())
        return line + '\n' + answer if answer.strip() else line

    def _compile(self, question):
        """The intent hint for this question, or '' when there is none.

        Off by default for anything that is not a real prompt turn: a second
        model call per question is the cost, and a plan runner replaying a
        scripted step already knows what it is asking for.
        """
        self._intent_did = self._intent_kind = self._intent_tool = None
        if not getattr(self, 'compile_intent', False):
            return ''
        got, kind, why = intent.compile_intent(self.client, question)
        self._intent_why = why
        if got is None:
            return ''
        self._intent_did, self._intent_kind = got, kind
        self._intent_tool = intent.tool_for(got, kind)
        return intent.hint(got, kind)

    NARRATE = ("The board answered the operator's question with the output "
               "below. Write one short sentence about it in %s - what it "
               "shows, or that it was read. Never repeat the rows: they are "
               "already on the operator's screen.")

    def _run_plan(self, question, calls):
        """Make the compiled calls, then ask for a sentence about them.

        The model gets no tools on this turn. That is the point: a question
        whose calls have already run has no tool choice left to get wrong, no
        second call to make, and nothing to refuse. Three backstops used to
        police that choice - a SYSTEM rule, a per-turn hint, and a redirect
        that leaked its own text onto the operator's screen.
        """
        self.history.append({'role': 'user', 'content': question})
        self.prompt_history.append(question)
        self.io_log.turn(question)
        # A planned turn never calls trim(), which is where the lock moves.
        # Without this a Swedish question answered from a plan left the
        # language on whatever the last unplanned question set.
        self._lock_language(question)
        self._traced = False
        shown, channels, table = [], None, None
        for name, args in calls:
            try:
                raw = self.toolbox.call(name, args)
            except RigError as exc:
                # A planned call is the host's own, so a raise here would
                # take the turn with it rather than reaching the operator as
                # the link failure it is.
                raw = 'ERR %s: %s' % (type(exc).__name__, exc)
            text = str(raw)
            if str(raw).startswith('ERR '):
                self.io_log.call(name, args, text)
                return self._link_down_message(text, shown=False)
            self._trace(text)
            self.io_log.call(name, args, text)
            shown.append(text)
            found = replies.READING_ROW.findall(text)
            if found:
                channels, table = [m.lower() for m in found], text
                self.last_channels = channels
        self.history.append({'role': 'user',
                             'content': BLANK_LINE.join(shown)})

        answer = ''
        try:
            said = self.client.chat(
                [{'role': 'system',
                  'content': self.NARRATE % (self.language or 'English')},
                 {'role': 'user', 'content': question},
                 {'role': 'user', 'content': BLANK_LINE.join(shown)}])
            answer = (said.get('content') or '').strip()
        except Exception:                                     # noqa: BLE001
            answer = ''                # the rows are the answer either way
        if channels and replies.is_retype(answer, channels):
            answer = table if (self.quiet and table) else ''
        elif self.quiet and not answer:
            answer = BLANK_LINE.join(shown)
        self.history.append({'role': 'assistant', 'content': answer})
        return answer

    def _ask_inner(self, question, max_calls=6):
        if self.over_budget():
            return 'budget of %d tokens is spent; /clear or raise --budget' \
                % self.budget

        # Compile before answering. The sentence still goes to the model
        # verbatim below; what this adds is one line saying which tool the
        # question maps to, worked out by a separate call that has nothing
        # to do but classify. None of it is load-bearing - see intent.py.
        self.intent = self._compile(question)
        planned = intent.plan(self._intent_did, self._intent_kind)
        if planned:
            return self._run_plan(question, planned)

        self.history.append({'role': 'user', 'content': question})
        self.prompt_history.append(question)
        self.io_log.turn(question)
        self._traced = False   # nothing on screen yet, so no leading gap

        turn = Turn()
        for _ in range(max_calls + 1):
            done = self._round(turn)
            if done is not None:
                return done if isinstance(done, str) else self._settle(turn)
        return self._settle(turn)

    def _round(self, turn):
        """One model turn and the calls it asked for.

        Returns None to go round again, a string to end the turn with that
        answer, or True to stop and let the backstops settle it.
        """
        before = self.client.usage()
        message = self.client.chat(self.trim(), self.schemas)
        after = self.client.usage()
        self._meter(after['prompt_tokens'] - before['prompt_tokens'],
                    after['eval_tokens'] - before['eval_tokens'])
        self._notes()

        message.pop('thinking', None)
        calls = message.get('tool_calls') or []
        turn.answer = (message.get('content') or '').strip()

        # A tool call written as prose is still a tool call. qwen2.5 emits
        # one as text often enough to matter - Measured: "vad ar
        # temperaturen" came back as the literal string
        #
        #     {"name": "docs", "arguments": {"find": "temperature"}}
        #     </tool_call>
        #
        # with no tool_calls field, which this loop then printed as the
        # answer. The model was right about what to do; the shape was
        # wrong. Recovering it costs a JSON parse.
        if not calls:
            salvaged, turn.answer = replies.salvage_calls(turn.answer)
            if salvaged:
                calls = salvaged
                message = dict(message, content='', tool_calls=calls)

        self.history.append(message)
        if not calls:
            return self._no_calls(turn)
        for call in calls:
            self._run_call(turn, call)
        return None

    def _no_calls(self, turn):
        """The model wrote instead of calling. None to nudge and go again."""
        # Three shapes of one problem: answering from memory instead of
        # checking, retyping the last reading instead of taking a new one,
        # and answering nothing at all. The first two are gated on
        # turn.channels - a reading having actually succeeded this turn -
        # not on link_ok: without that, a plain "what is 2+2" was discarded
        # on the first question of a session that opened with the board
        # unreachable. A blank answer is never valid and needs no such gate.
        stale = not turn.channels and (
            not turn.answer or (self.last_channels and (
                not self.link_ok
                or replies.is_retype(turn.answer, self.last_channels))))
        if stale:
            probe = self._probe_link()
            if not self.link_ok:
                # `shown` here too: the checklist the model just traced is
                # directly above, and without this the answer printed the
                # whole thing again - the failure the parameter exists for,
                # on the one path that never passed it.
                return self._link_down_message(
                    probe, shown=turn.diagnosed and not self.quiet)
            # Confirmed up. Told only that, the turn still ended on "ask
            # again" and the operator retyped it twice, so the nudge has to
            # be actionable - but not prescriptive. Measured: it named
            # analog_read, and "beskriv hardvaran i detta projektet for en
            # novis" answered blank, got nudged, and came back with a full
            # analog table. The host cannot tell from here whether the
            # question wants a reading; the model can.
            return turn.nudge(
                self, 'The link just answered. Answer the question now - '
                'with a fresh call if it needs one, and in words if it does '
                'not. Never reuse an old reading.',
                'no reading taken this turn - ask again.')
        # A reading did succeed this turn and the model still wrote nothing.
        # Measured: "Beskriv hardvaran i detta projektet for en novis" -
        # gemma4:12b called analog_read, returned empty content, and the
        # operator got the table and a blank line where the answer goes. The
        # gate above cannot catch it: it is closed by turn.channels, which
        # that very call had just set. Nothing about the reading is wrong
        # here, so the nudge asks for the answer rather than a fresh table.
        if not turn.answer:
            return turn.nudge(
                self, 'Answer the question in words now. The tool output is '
                'already on screen - do not repeat it.',
                'the reading above is all that came back - ask again.')
        # It knew exactly what to do and did not do it. Nudged, not silenced
        # or replaced: there is no fact in hand yet to substitute, only a
        # call worth actually making. Bounded the same as the runner's own
        # prose-stop nudge, so a model that keeps narrating instead of
        # calling still ends the turn rather than asking nicely forever.
        if (replies.NAMED_TOOL.search(turn.answer)
                and turn.nudges < Turn.NUDGE_LIMIT):
            turn.nudges += 1
            self.history.append({'role': 'user', 'content':
                                 'Call the tool now - do not describe it.'})
            return None
        return True

    def _run_call(self, turn, call):
        """Make one call the model asked for, and record what it means."""
        name = (call.get('function') or {}).get('name', '?')
        args = toolmod.arguments(call)
        key = (name, json.dumps(args, sort_keys=True, default=str))

        if name not in REPEATABLE and key in turn.seen:
            # Do not spend a board round trip re-asking a question this turn
            # already has the answer to - and say so plainly rather than
            # repeating the same line, which is what asked for the repeat in
            # the first place. `raw` stays the original result so a repeated
            # failure is still read as one below, not laundered into a
            # fresh-looking success by the sentence wrapped around it.
            raw = turn.seen[key]
            result = 'unchanged this turn, already asked: %s' % raw
        else:
            raw = self.toolbox.call(name, args)
            if isinstance(raw, toolmod.Reported):
                raw = 'noted: %s' % raw.note
            turn.seen[key] = raw
            result = raw

        text = str(raw)
        failed = text.startswith('ERR')
        if name in LINK_TOOLS:
            # Every call that actually reaches the board is a live reading on
            # the link itself, not just on this run's question - the spinner
            # is wrong the moment this call's verdict disagrees with what it
            # is currently showing.
            lost = ERR_CLASS.match(text)
            self.link_ok = not (lost and lost.group(1) in CONTACT_LOST)
            # A call that reached the board clears an earlier failure in the
            # same turn; one that did not reach it sets the error that gates
            # the answer below, whatever the model goes on to write about it.
            turn.link_error = text if not self.link_ok else None
            if not self.link_ok:
                # A replugged cable re-enumerates the VCP, so the cached
                # handle stays dead: measured, every retry then fails with
                # "Attempting to use a port that is not open" until
                # session.reset() drops it.
                self.toolbox.session.reset()
        if name == 'link_diagnose' and not failed:
            # Its checklist is on screen from the trace below. What the
            # answer says about a dead link changes accordingly - see
            # _link_down_message.
            turn.diagnosed = True
        if name in toolmod.CODE_CALLS:
            # A failed build, or a --confirm the operator declined, is a fact
            # this loop holds. Measured: refused at the prompt, gemma4:12b
            # still answered "kortet har byggts och flashats" - on the one
            # call that writes to a 63 V board. Cleared by a later success in
            # the same turn.
            turn.code_error = text if failed else None
        if name == 'analog_read' and not failed:
            # A fresh table replaces the last one remembered; an error leaves
            # the previous table in place rather than wiping it, since
            # link_error already takes priority below either way.
            turn.channels = set(m.lower()
                                for m in replies.READING_ROW.findall(text))
            self.last_channels = turn.channels
            turn.table = text
        if name in ('board_info', 'digital_read') and not failed:
            turn.remember_map(text)
        if not _afe_noise(name, args, raw):
            self._trace(result)
        self.io_log.call(name, args, result)         # always - see IOLog
        self.history.append({'role': 'tool', 'tool_name': name, 'name': name,
                             'content': '%s: %s' % (name, result)})

    def _settle(self, turn):
        """The answer the operator gets, after the facts the loop holds."""
        answer = turn.answer
        # A read that failed on the wire is ground truth; the model gets no
        # vote. Measured: with the ST-Link unplugged, qwen2.5:14b answered
        # with an NTC value from three questions earlier. SYSTEM already says
        # not to - this is where saying it was not enough.
        if turn.link_error is not None:
            # `and not self.quiet`: with the trace off there is nothing on
            # screen above this, so the checklist has to come with the answer
            # or the operator is told the link is down and nothing else.
            return self._link_down_message(
                turn.link_error, shown=turn.diagnosed and not self.quiet)
        if turn.code_error is not None:
            return ('the last run_python/run_command call failed, nothing '
                    'was done: %s' % turn.code_error)
        # SYSTEM says not to; qwen2.5:14b did it every time across three
        # sessions. Silence rather than a line saying so - the table is
        # directly above on the same screen. Unless --quiet, where there is
        # no trace and the board's own rows go out instead.
        #
        # Three channels before "all of them named" counts: naming two is
        # plausibly synthesis, and silencing "NTC and DCbus both read low"
        # would cost a finding. A map has nothing to synthesise about, and
        # this board has two digital channels, so listing both IS the map.
        if turn.channels and replies.is_retype(answer, turn.channels):
            return turn.table if (self.quiet and turn.table) else ''
        if any(replies.is_retype(answer, names, minimum=2)
               for names in turn.maps):
            return turn.map_text if (self.quiet and turn.map_text) else ''
        # An answer that hit the token cap stops mid-sentence, and a table
        # that stops mid-row reads as complete to everyone except a reader
        # counting rows. Say so rather than letting the cap look like the end.
        if getattr(self.client, 'truncated', False) and answer:
            answer += ('\n[cut off at --words %s. Ask again with more, or ask '
                       'for fewer channels.]'
                       % self.client.options.get('num_predict', '?'))
        return answer

    # ---- the parts that cost nothing ---------------------------------------

    def command(self, line):
        """A slash command. Returns text, or None if the line was not one.

        These exist because a model is a poor way to run a line of Python you
        already know you want to run - and at a bench, most of the time, you do.
        """
        if not line.startswith('/'):
            return None
        verb, _, rest = line[1:].partition(' ')
        rest = rest.strip()

        if verb in ('q', 'quit', 'exit'):
            raise SystemExit(0)
        if verb in ('help', '?'):
            # Live, not a fixed string: what it can do depends on the tool set
            # this session started with, what it costs on the detail level.
            lines = [ROLE]
            if {'build_firmware', 'run_command'} & set(self.tool_names):
                lines.append(BUILDS)
            lines.append('%s, %s, %d tok/turn: %s'
                         % (getattr(self.client, 'model', '?'), self.detail,
                            self.tool_cost(),
                            ', '.join(self.tool_names) or 'no tools'))
            return '\n'.join(lines + [HELP])
        if verb == 'py':
            return self.toolbox.call('run_python', {'code': rest})
        if verb == 'sh':
            return self.toolbox.call('run_command', {'cmd': rest})
        if verb == 'reconnect':
            return self._reconnect()
        if verb == 'model':
            return self._switch_model(rest)
        if verb == 'board':
            return self._switch_board(rest)
        if verb == 'node':
            return self._switch_node(rest)
        if verb == 'clear':
            self.history = []
            return 'context cleared'
        if verb == 'tools':
            if rest:
                self.set_tools(rest)
            return '%s (%d tok/turn)' % (', '.join(self.tool_names) or 'none',
                                         self.tool_cost())
        if verb == 'detail':
            # Priced, not just named: the whole point of the level is what
            # the tool list costs per turn, and that number is the argument
            # for changing it.
            if rest:
                if rest.lower() not in detail.LEVELS:
                    return 'detail: %s, or auto' % ', '.join(
                        (detail.TERSE, detail.FULL))
                self.set_detail(rest.lower())
            return 'detail: %s (%d tok/turn of tools)' % (self.detail,
                                                          self.tool_cost())
        if verb == 'confirm':
            # /tools build alone hands the model run_command with nothing
            # asking first, unless --confirm was already on the command line
            # that started this session - this is the other half of that
            # switch, reachable without a restart either.
            from .cli import ask_operator     # cli imports Chat: not at top
            self.toolbox.confirm = (None if self.toolbox.confirm
                                    else ask_operator)
            return 'confirm: %s' % ('on - asks before every write'
                                    if self.toolbox.confirm else 'off')
        if verb == 'lang':
            if not rest:
                return ('session language: %s' % self.language
                       if self.language else
                       'not locked yet - mirroring each question')
            if rest.lower() in ('auto', 'off'):
                self.language = None
                return 'language: unlocked - back to mirroring each question'
            named = (language._NAME_TO_LANGUAGE.get(rest.lower())
                    or (rest.title() if rest.title() in language.LANGUAGE_NAMES
                        else None))
            if named is None:
                return ("don't know %r - try an English language name, or "
                        "/lang auto to unlock" % rest)
            self.language = named
            return 'language: %s (locked)' % named
        if verb == 'ctx':
            # The budget is the number that explains the other two once a
            # conversation gets long: a turn that is not growing any more is
            # a turn being trimmed to fit, not a turn that stopped costing.
            budget = self.prompt_budget()
            return '%d messages, next turn about %d tok in%s, %d of it tools' \
                % (len(self.history), self.context_cost(),
                   ' of %d' % budget if budget else '', self.tool_cost())
        if verb == 'cost':
            return self.cost_line()
        if verb == 'history':
            if not self.prompt_history:
                return 'nothing asked yet this session'
            return '\n'.join('%d. %s' % (i, clip(q, 100))
                             for i, q in enumerate(self.prompt_history, 1))
        if verb == 'clear_history':
            n = len(self.prompt_history)
            self.prompt_history = []
            return 'prompt history cleared (%d question%s)' \
                % (n, '' if n == 1 else 's')
        return 'no such command. /help'

    def _switch_model(self, rest):
        """Run this session on another tag, without restarting it.

        The old model's VRAM goes back *before* the new one is asked for.
        On a 16 GB card the other order is a request for two copies of the
        weights, which is the failure docs/MODELS.md spends a section on -
        and the swap is exactly the moment somebody would cause it.

        History goes with it. A prompt prefix cached for one model is worth
        nothing to another, and `detail=auto` is resolved from the tag, so
        the tool schemas are rebuilt too.
        """
        from .client import Ollama, OllamaError

        tag, extra = rest.strip(), {}
        if not tag:
            try:
                have = ', '.join(self.client.models())
            except OllamaError as exc:
                have = str(exc)
            return 'model: %s (%d tok window). Available: %s' % (
                self.client.model, self.client.options.get('num_ctx', 0), have)
        if tag == 'auto':
            from .capability import choose, probe
            picked = choose(probe())
            tag, extra = picked.tag, dict(picked.options or {})
        if tag == self.client.model and not extra:
            return 'model: %s already' % tag

        old = self.client
        fresh = Ollama(tag, host=old.host, remote_ok=old.remote_ok,
                       keep_alive=old.keep_alive, think=old.think,
                       fmt=old.fmt, timeout=old.timeout)
        fresh.options = dict(old.options)
        fresh.options.update(extra)
        try:
            # Its own error text names the tag and how to pull it. Asked here
            # rather than at the next question, so a typo costs a command and
            # not a turn - and so nothing is swapped when it fails.
            fresh.require_model()
        except OllamaError as exc:
            return str(exc)

        try:
            old.unload()
        except Exception as exc:                              # noqa: BLE001
            # Not fatal, and not silent: the session still works, the card is
            # just holding weights nobody is using until keep_alive expires.
            self.io_log.write('  ! could not unload %s: %s%s'
                              % (old.model, exc, '\n'))
        self.client = fresh
        self.history = []
        self.set_detail(self.detail)
        return 'model: %s (was %s), context cleared' % (fresh.model, old.model)

    def _switch_node(self, rest):
        """Which node on the bus the tools talk to. `0` is every one.

        The operator's own route to what `devices op=use` does for the
        model - no model turn, no tokens, same state. Bare, it lists.
        """
        from coaxial_mcp import tools as mcp

        session = self.toolbox.session
        want = rest.strip()
        if not want:
            return mcp.devices(session)
        if want.lower() in ('buses', 'bus'):
            return mcp.devices(session, op='buses')
        # "LL 2" is a bus and a node, which is what a node id needs beside
        # it once there is more than one segment.
        parts = want.split()
        if len(parts) == 2 and parts[1].lstrip('-').isdigit():
            return mcp.devices(session, op='use', bus=parts[0].upper(),
                               unit=int(parts[1]))
        if not want.lstrip('-').isdigit():
            return mcp.devices(session, op='use', name=want)
        return mcp.devices(session, op='use', unit=int(want))

    def prompt_tag(self):
        """(text, ok) for the prompt: the interface, then the node.

        Read fresh every turn rather than stored, because `devices op=use`
        moves the node mid-session and the very next prompt is what should
        show it. `ok` is True for a board, False for a stand-in, and the
        string 'all' for the broadcast address - which the spinner paints
        red, because that is the one mode where a command reaches every
        node and nothing answers.
        """
        label, real = getattr(self, 'origin', None) or (None, True)
        if label is None:
            return None, True
        session = self.toolbox.session
        unit = getattr(session, 'unit', None)
        if unit is None:
            return label, real
        bus = getattr(session, 'bus', None)
        where = None
        try:
            from coaxial.simulated import bus_nodes
            if bus:
                where = (bus_nodes(bus).get(unit) or (None, None, None))[2]
        except Exception:                                     # noqa: BLE001
            pass
        # The bus first, because with five segments a node number alone
        # names nothing: node 2 is a knee on two of them.
        #
        # And the joint without its side, because the bus already carries
        # it: "RL 2 knee", not "RL node 2 right knee". Shorter than the
        # abbreviations that were asked for and needs no key - and `Ra`
        # for ankle would have collided with `RA` for the right arm, on a
        # line whose whole job is to be unambiguous at a glance.
        if where:
            for side in ('left ', 'right '):
                if where.startswith(side):
                    where = where[len(side):]
                    break
        node = 'ALL NODES' if unit == 0 else (
            '%d %s' % (unit, where) if where else 'node %d' % unit)
        if bus:
            node = '%s %s' % (bus, node)
        return '%s, %s' % (label, node), ('all' if unit == 0 else real)

    def _switch_board(self, rest):
        """Point this session at another board, or at a simulated one.

        `simulated` takes the stand-in outright, `auto` looks for a real one
        - debug probe first - and a port name tries that first. The prompt
        tag is rebuilt from the same origin the factory returns, so what the
        screen says and what the tools talk to cannot drift apart.
        """
        from coaxial_mcp.session import open_session

        want = rest.strip().lower()
        if not want:
            label = (getattr(self, 'origin', None) or ('unknown',))[0]
            return ('board: %s. /board simulated | auto | rs485 | COM4'
                    % label)
        if want in ('sim', 'simulated', 'fake'):
            session, found = open_session(simulated=True)
        elif want == 'auto':
            session, found = open_session()
        elif want in ('rs485', 'serial'):
            # The field bus, not the bench cable: probes are excluded, or
            # the probe-first order hands back the one board that was just
            # ruled out.
            import find_board
            session, found = open_session(only=find_board.SERIAL)
        elif want in ('probe', 'jtag', 'swd', 'debugger'):
            import find_board
            session, found = open_session(only=find_board.PROBE)
        else:
            session, found = open_session(rest.strip())

        wanted_real = want not in ('sim', 'simulated', 'fake')
        if wanted_real and not found.real:
            # The search found nothing. Do NOT swap: an order that cannot
            # be carried out must not also cost the board that was working,
            # and "byt till rs485" on a live probe session would otherwise
            # drop it for a stand-in. Say what was tried, so the operator
            # learns something instead of pressing it again - measured, the
            # same order twice in a row, both times "inget svarade", and
            # nothing on screen said the cable and driver were fine.
            try:
                session.close()
            except Exception:                                 # noqa: BLE001
                pass
            import find_board
            seen = ', '.join(find_board.list_ports())
            here = (getattr(self, 'origin', None) or ('unknown',))[0]
            return ('board: nothing answered on %s - still on %s'
                    % (seen or 'no COM port at all', here))

        previous = self.toolbox.session
        if previous is not session:
            try:
                previous.close()
            except Exception:                                 # noqa: BLE001
                pass
        self.toolbox.session = session
        self.origin = (found.label, found.real)
        self.link_ok = True
        # Readings remembered from the board just left are not this one's.
        # Without this, the retype backstop compares an answer against a
        # table taken from different hardware.
        self.last_channels = None
        return 'board: %s' % found.label

    def _reconnect(self):
        """Drop the link and try to reopen it - for a cable that was plugged
        back in without restarting this whole prompt loop.

        Session.reset() forgets the cached board (a no-op on NoBoard); the
        board property that follows is what actually reopens the port. This
        is the one place left that connects eagerly rather than waiting for
        a tool call to need it - the operator asked directly, by name, so
        there is a real question to answer either way.
        """
        session = self.toolbox.session
        session.reset()
        try:
            session.board
        except RigError as exc:
            self.link_ok = False
            return 'board: %s' % exc
        self.link_ok = True
        return 'board: link is up'

    def over_budget(self):
        usage = self.client.usage()
        return bool(self.budget) and \
            usage['prompt_tokens'] + usage['eval_tokens'] >= self.budget

    def cost_line(self):
        usage = self.client.usage()
        total = usage['prompt_tokens'] + usage['eval_tokens']
        text = '%d calls, %d in + %d out = %d tok' % (
            usage['calls'], usage['prompt_tokens'], usage['eval_tokens'], total)
        return text + (' of %d' % self.budget if self.budget else '')

    def _meter(self, prompt_tokens, eval_tokens):
        """Record the cost of one turn. Not printed - see /cost and /ctx."""
        self.turn_cost.append((prompt_tokens, eval_tokens))

    def screen_language(self):
        """Which language this session's own text prints in - the session
        language, which starts as the machine's locale and moves only when a
        question is actually in another one."""
        return self.language

    def _notes(self):
        """Say what the client had to do to the machine to answer at all.

        The Ollama client evicts models and shrinks windows in silence
        because a library that prints is a library nobody can embed - but a
        session whose context window just halved has to be told, or the next
        odd answer looks like the model getting worse for no reason. Traced
        like a tool result, so --quiet stays quiet, and logged unconditionally
        because the log is what a later look at the session reads.
        """
        notes = getattr(self.client, 'notes', None)
        if not notes:
            return
        drained, notes[:] = list(notes), []
        for note in drained:
            self.io_log.write('  ! %s\n' % note)
            self._trace('! ' + note)

    def _trace(self, result):
        """Print what a call returned, as the grid render.py already built it.

        No header, and that is the point of the last revision. It used to print
        the call above the result - name and arguments - and on the call this
        exists for, that header was the worst line on screen:

            analog_read samples=100 ch=['dc_bus', 'ntc', 'phase_a', 'phas
            ... [17 more characters cut] -> 100 smp @2000Hz

        The argument clip put a newline inside the header, which pushed the
        ' -> ' and the first row of the table onto the end of the cut notice.
        And it was restating the result badly while it did it: the table names
        every channel it read, one per row. What is left is the rows.

        Each row is clipped on its own so a long value does not push the row
        after it off screen, with a count for whatever does not fit rather than
        a table that quietly stops.
        """
        if self.quiet:
            return
        # A blank line between blocks, once there is more than one. Asked
        # for both the analog and the digital values, the two tables ran
        # together into one wall - each is headed and counted now, and the
        # gap is what makes the heading read as the start of something.
        # Only before a multi-line result: a one-line answer needs no room
        # around it.
        lead = ''
        if getattr(self, '_traced', False) and '\n' in str(result).strip():
            lead = '\n'
        self._traced = True
        # English stays in the result the model reads, the log keeps and the
        # MCP server serves; the screen gets the operator's language. Only
        # host-authored sentences turn - a channel name, a unit or anything
        # the board said passes through. See language.PHRASES.
        lines = (language.localise(str(result), self.screen_language())
                 .splitlines() or [''])
        with self.print_lock:
            if lead:
                print(file=self.out, flush=True)
            for line in lines[:TRACE_ROWS]:
                for part in _wrapped(line):
                    print(part, file=self.out, flush=True)
            if len(lines) > TRACE_ROWS:
                print('  ... [%d more rows]' % (len(lines) - TRACE_ROWS),
                      file=self.out, flush=True)


if __name__ == '__main__':
    from .cli import main                 # see _ELSEWHERE
    sys.exit(main())


# Moved out, re-exported: dbg.py and two suites reach for these by their old
# names. A module-level __getattr__ keeps that working without importing cli
# at the top, which would be a cycle - cli imports Chat from here.
_ELSEWHERE = {
    'IOLog': 'iolog', 'IO_LOG_PATH': 'iolog',
    'main': 'cli', 'parse': 'cli', 'build': 'cli', 'repl': 'cli',
    'attach': 'cli', 'ask_operator': 'cli', 'NoBoard': 'cli',
    'keep_alive_for': 'cli', 'KEEP_ALIVE_REPL': 'cli',
    'KEEP_ALIVE_ONCE': 'cli', 'INPUT_LIMIT': 'cli', '_printable': 'cli',
}


def __getattr__(name):
    where = _ELSEWHERE.get(name)
    if where is None:
        raise AttributeError(name)
    from importlib import import_module
    return getattr(import_module('.' + where, __package__), name)
