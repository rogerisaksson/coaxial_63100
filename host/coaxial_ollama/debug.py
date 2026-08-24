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
from . import language                               # noqa: E402
from . import replies                                # noqa: E402
from . import tools as toolmod                       # noqa: E402
from . import spinner as spin                        # noqa: E402
from .context import approx_tokens                   # noqa: E402
from .sandbox import Scope, Shell, clip, clip_ends   # noqa: E402

# Deliberately terse, and every line of it earns its place. No restating the
# protocol, no channel map - board_info carries that, once, when asked.
SYSTEM = """You are an expert with a serial link to a coaxial BLDC inverter.
Tools for the board, never to guess; off-topic needs none. Answer briefly,
no preamble.
A table or list means analog_read once - its grid is every channel already.
Never markdown it, never restate a tool's own rows - one line, not two.
Describe, explain or compare means words, from what you know: analog_read
answers what a channel reads now, never what a thing is.
A call error is reported, never guessed or hidden behind an old reading.
Any reading: analog_read only, never afe_power first - analog_read works
with the AFE on or off and reports which. Turning the AFE on or off itself
is the order to do it, not to discuss. Phase channels: unknown gain, pin
volts only."""

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
LINK_DIAGNOSE_HINT = ("A question about why the board is not answering, or "
                      "whether the link is down, is answered by calling "
                      "link_diagnose - not by guessing, not by trying "
                      "build_firmware or anything else. If it has not "
                      "already been called this turn, call it before "
                      "answering. Then be a troubleshooter, not a reporter: "
                      "turn the checklist into the next concrete thing to "
                      "check or do, in order, one step at a time - not the "
                      "raw step text back at the operator.")

# Named subsets: a debug job knows roughly what it will touch. `docs` is in
# none of them but its own and `all` - see DOCS_HINT.
SETS = {
    'read': ('board_info', 'analog_read', 'self_test', 'afe_power', 'link',
             'link_diagnose'),
    'code': ('board_info', 'analog_read', 'self_test', 'afe_power', 'link',
             'run_python', 'build_firmware', 'run_tests', 'link_diagnose'),
    'pins': ('board_info', 'gpio_pin', 'gpio_port', 'test_gate', 'afe_power',
             'link_diagnose'),
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

# What /help opens with. The second line only when the tools behind it are
# loaded: measured, the model denied being able to flash with build_firmware
# sitting in its own list, and an operator who never asks is never told.
ROLE = 'Senior engineer for this inverter: firmware, AFE, Modbus, live link.'
BUILDS = 'Builds and flashes it too: build_firmware, run_tests.'

# Two numbers, because the modes want opposite things. A prompt loop is about
# to be asked again and the cached prefix is worth 8 GB of VRAM; a one-shot is
# not - measured, it left 9.69 GB resident for 27 minutes at 1 % use.
KEEP_ALIVE_REPL = '30m'
KEEP_ALIVE_ONCE = '2m'


def keep_alive_for(args):
    """What the caller asked for, or what the mode implies."""
    if args.keep_alive is not None:
        return args.keep_alive
    return KEEP_ALIVE_REPL if args.repl else KEEP_ALIVE_ONCE


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


def _printable(stream):
    """Make a console survive an answer in somebody else's alphabet.

    A console keeps its own codepage and only gains errors='replace': under
    cp1252 an ohm sign or any Cyrillic would otherwise raise
    UnicodeEncodeError and lose a reading already taken. Forcing UTF-8 on a
    console that is not set to it trades that for mojibake in the languages
    this bench actually speaks.

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


# The most of a piped or attached input that becomes part of a question.
# `sed -n 1,40p log | dbg` is a question about a log; `cat build.log | dbg`
# is the same command with fifty thousand lines behind it, and nothing about
# the pipe says which one arrived.
INPUT_LIMIT = 6000


# host/prompt_io.tmp - resolved from this file's own location, not the
# caller's cwd, so `python dbg.py` from host/ and a task that starts
# somewhere else both land in the same place, at the same fixed name a
# later debugging session can just open without knowing a timestamp.
IO_LOG_PATH = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'prompt_io.tmp')


def _set_attributes(path, value):
    """Windows file attributes, best-effort. Not security - a file with the
    raw questions and answers of a bench session is not secret, it is just
    not something that belongs in an ordinary directory listing next to the
    files this project is actually about."""
    if sys.platform != 'win32':
        return
    try:
        import ctypes
        ctypes.windll.kernel32.SetFileAttributesW(str(path), value)
    except Exception:                                        # noqa: BLE001
        pass


def _unhide(path):
    """Clear the hidden attribute before (re)opening a session's log for
    writing. Measured directly on this bench: `open(path, 'w')` on an
    already-hidden file raised a plain PermissionError, not the OSError
    IOLog already expected and swallowed - the truncate that mode implies
    is what Windows refuses on a hidden file, not the open itself. 0x80 is
    FILE_ATTRIBUTE_NORMAL; nothing to do if the file does not exist yet."""
    if os.path.exists(path):
        _set_attributes(path, 0x80)


def _hide(path):
    _set_attributes(path, 0x02)                     # FILE_ATTRIBUTE_HIDDEN


class IOLog:
    """A hidden per-session log of every question, call and answer - for
    debugging this loop afterwards, not for the operator.

    Overwritten each session, not appended: a log covering three runs ago is
    worse than none when what matters is this one. It keeps more than the
    screen does - a refused afe_power call is hidden from the trace and kept
    here, because that is what answers "why did that turn cost four calls".
    """

    def __init__(self, path=IO_LOG_PATH, enabled=True):
        self.handle = None
        if not enabled:
            return
        _unhide(path)
        try:
            self.handle = open(path, 'w', encoding='utf-8', errors='replace')
            _hide(path)
        except OSError:
            self.handle = None

    def write(self, text):
        if self.handle is None:
            return
        try:
            self.handle.write(text)
            self.handle.flush()
        except OSError:
            self.handle = None

    def turn(self, question):
        self.write('=== %s ===\nQ: %s\n' % (time.strftime('%H:%M:%S'),
                                             question))

    def call(self, name, args, result):
        self.write('  %s %s\n  -> %s\n'
                   % (name, json.dumps(args, default=str)[:300],
                      clip(str(result), 500)))

    def answer(self, text):
        self.write('A: %s\n\n' % text)

    def close(self):
        if self.handle is not None:
            try:
                self.handle.close()
            except OSError:
                pass
            self.handle = None


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
        requested = language.requested_language(asked)
        detected = language.detect(asked)
        current = getattr(self, 'language', None)
        if requested and requested != current:
            current = requested
        elif detected and detected != current:
            current = detected
        self.language = current
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
        self.history.append({'role': 'tool', 'tool_name': 'link',
                             'name': 'link', 'content': 'link: %s' % probe})
        return probe

    def _link_down_message(self, link_error, shown=False):
        """'link is down, not answered: ...', plus why - run here, by the
        host, every time the link is down, rather than left for the model
        to think to call link_diagnose. Measured on this bench: a raw
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
        answer = language.localise(self._ask_inner(question, max_calls),
                                   self.screen_language())
        self.io_log.answer(answer)
        return answer

    def _ask_inner(self, question, max_calls=6):
        if self.over_budget():
            return 'budget of %d tokens is spent; /clear or raise --budget' \
                % self.budget

        self.history.append({'role': 'user', 'content': question})
        self.prompt_history.append(question)
        self.io_log.turn(question)
        answer = ''
        link_error = None
        code_error = None  # last run_python/run_command result, if it failed
        last_channels = None      # names in the most recent analog_read table
        diagnosed = False  # link_diagnose ran this turn, and was traced
        seen = {}          # (name, args) this turn -> its rendered result
        nudges = 0         # times told to call the tool it just named, or to
                           # take a fresh reading instead of an old one

        for _ in range(max_calls + 1):
            before = self.client.usage()
            message = self.client.chat(self.trim(), self.schemas)
            after = self.client.usage()
            self._meter(after['prompt_tokens'] - before['prompt_tokens'],
                        after['eval_tokens'] - before['eval_tokens'])
            self._notes()

            message.pop('thinking', None)
            calls = message.get('tool_calls') or []
            answer = (message.get('content') or '').strip()

            # A tool call written as prose is still a tool call. qwen2.5 emits
            # one as text often enough to matter - measured here, "vad ar
            # temperaturen" came back as the literal string
            #
            #     {"name": "docs", "arguments": {"find": "temperature"}}
            #     </tool_call>
            #
            # with no tool_calls field, which this loop then printed as the
            # answer. The model was right about what to do; the shape was
            # wrong. Recovering it costs a JSON parse.
            if not calls:
                salvaged, answer = replies.salvage_calls(answer)
                if salvaged:
                    calls = salvaged
                    message = dict(message, content='', tool_calls=calls)

            self.history.append(message)
            if not calls:
                # Three shapes of one problem: answering from memory instead
                # of checking, retyping the last reading instead of taking a
                # new one, and answering nothing at all. The first two are
                # gated on last_channels - a reading having actually succeeded
                # this session - not on link_ok: without that, a plain "what
                # is 2+2" was discarded on the first question of a session
                # that opened with the board unreachable. A blank answer is
                # never valid and needs no such gate.
                stale = not last_channels and (
                    not answer or (self.last_channels and (
                        not self.link_ok
                        or replies.is_retype(answer, self.last_channels))))
                if stale:
                    probe = self._probe_link()
                    if not self.link_ok:
                        return self._link_down_message(probe)
                    # Confirmed up. Told only that, the turn still ended on
                    # "ask again" and the operator retyped it twice. A nudge
                    # spends a turn this loop already owns.
                    if nudges < 2:
                        nudges += 1
                        self.history.append({'role': 'user', 'content':
                            'The link just answered - call analog_read for a '
                            'fresh reading, do not reuse the old one.'})
                        continue
                    return 'no reading taken this turn - ask again.'
                # A reading did succeed this turn and the model still wrote
                # nothing. Measured: "Beskriv hardvaran i detta projektet for
                # en novis" - gemma4:12b called analog_read, returned empty
                # content, and the operator got the table and a blank line
                # where the answer goes. The gate above cannot catch it: it
                # is closed by last_channels, which that very call had just
                # set. Nothing about the reading is wrong here, so the nudge
                # asks for the answer rather than for a fresh table.
                if not answer:
                    if nudges < 2:
                        nudges += 1
                        self.history.append({'role': 'user', 'content':
                            'Answer the question in words now. The tool '
                            'output is already on screen - do not repeat '
                            'it.'})
                        continue
                    return 'the reading above is all that came back - ask again.'
                # It knew exactly what to do and did not do it. Nudged, not
                # silenced or replaced: there is no fact in hand yet to
                # substitute, only a call worth actually making. Bounded the
                # same as the runner's own prose-stop nudge, so a model that
                # keeps narrating instead of calling still ends the turn
                # rather than spending it forever asking nicely.
                if answer and replies.NAMED_TOOL.search(answer) and nudges < 2:
                    nudges += 1
                    self.history.append({'role': 'user', 'content':
                        'Call the tool now - do not describe it.'})
                    continue
                break

            for call in calls:
                name = (call.get('function') or {}).get('name', '?')
                args = toolmod.arguments(call)
                key = (name, json.dumps(args, sort_keys=True, default=str))

                if name not in REPEATABLE and key in seen:
                    # Do not spend a board round trip re-asking a question
                    # this turn already has the answer to - and say so plainly
                    # rather than repeating the same line, which is what asked
                    # for the repeat in the first place. `raw` stays the
                    # original result so a repeated failure is still read as
                    # one below, not laundered into a fresh-looking success by
                    # the sentence wrapped around it.
                    raw = seen[key]
                    result = 'unchanged this turn, already asked: %s' % raw
                else:
                    raw = self.toolbox.call(name, args)
                    if isinstance(raw, toolmod.Reported):
                        raw = 'noted: %s' % raw.note
                    seen[key] = raw
                    result = raw
                if name in LINK_TOOLS:
                    # Every call that actually reaches the board is a live
                    # reading on the link itself, not just on this run's
                    # question - the spinner is wrong the moment this call's
                    # verdict disagrees with what it is currently showing.
                    lost = ERR_CLASS.match(str(raw))
                    self.link_ok = not (lost and lost.group(1) in CONTACT_LOST)
                    # A call that reached the board clears an earlier failure
                    # in the same turn; one that did not reach it sets the
                    # error that gates the answer below, whatever the model
                    # goes on to write about it.
                    link_error = str(raw) if not self.link_ok else None
                    if not self.link_ok:
                        # A replugged cable re-enumerates the VCP, so the
                        # cached handle stays dead: measured, every retry then
                        # fails with "Attempting to use a port that is not
                        # open" until session.reset() drops it.
                        self.toolbox.session.reset()
                if name == 'link_diagnose' and not str(raw).startswith('ERR'):
                    # Its checklist is on screen from the trace below. What
                    # the answer says about a dead link changes accordingly -
                    # see _link_down_message.
                    diagnosed = True
                if name in toolmod.CODE_CALLS:
                    # A failed build, or a --confirm the operator declined,
                    # is a fact this loop holds. Measured: refused at the
                    # prompt, gemma4:12b still answered "kortet har byggts och
                    # flashats" - on the one call that writes to a 63 V board.
                    # Cleared by a later success in the same turn.
                    code_error = str(raw) if str(raw).startswith('ERR') else None
                if name == 'analog_read' and not str(raw).startswith('ERR'):
                    # A fresh table replaces the last one remembered; an error
                    # leaves the previous table in place rather than wiping it,
                    # since link_error already takes priority below either way.
                    last_channels = set(m.lower()
                                        for m in replies.READING_ROW.findall(str(raw)))
                    self.last_channels = last_channels
                # An afe_power refused for not being asked for is a mistake
                # the model recovers from one call later. The refusal stays in
                # history for it to read; the operator does not need it on
                # screen. Every other afe_power result traces as before.
                if not (name == 'afe_power'
                       and str(raw).startswith('ERR not asked for')):
                    self._trace(result)
                self.io_log.call(name, args, result)     # always - see IOLog
                self.history.append({'role': 'tool', 'tool_name': name,
                                     'name': name,
                                     'content': '%s: %s' % (name, result)})

        # A read that failed on the wire is ground truth; the model gets no
        # vote. Measured: with the ST-Link unplugged, qwen2.5:14b answered
        # with an NTC value from three questions earlier. SYSTEM already says
        # not to - this is where saying it was not enough.
        if link_error is not None:
            # `and not self.quiet`: with the trace off there is nothing on
            # screen above this, so the checklist has to come with the answer
            # or the operator is told the link is down and nothing else.
            answer = self._link_down_message(link_error,
                                             shown=diagnosed and not self.quiet)
        elif code_error is not None:
            answer = 'the last run_python/run_command call failed, nothing ' \
                     'was done: %s' % code_error
        # Same backstop for a retyped table: SYSTEM says not to, qwen2.5:14b
        # did it every time across three sessions. The bar is narrow - every
        # channel just read, named again by an answer with nothing else in it
        # - so a real one-line finding is untouched. Silence rather than a
        # line saying so: the table is directly above on the same screen.
        elif replies.is_retype(answer, last_channels):
            answer = ''
        # An answer that hit the token cap stops mid-sentence, and a table
        # that stops mid-row reads as complete to everyone except a reader
        # counting rows. Say so rather than letting the cap look like the end.
        elif getattr(self.client, 'truncated', False) and answer:
            answer += ('%s[cut off at --words %s. Ask again with more, or ask '
                       'for fewer channels.]'
                       % (chr(10), self.client.options.get('num_predict', '?')))
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
            self.toolbox.confirm = None if self.toolbox.confirm else ask_operator
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
        # English stays in the result the model reads, the log keeps and the
        # MCP server serves; the screen gets the operator's language. Only
        # host-authored sentences turn - a channel name, a unit or anything
        # the board said passes through. See language.PHRASES.
        lines = (language.localise(str(result), self.screen_language())
                 .splitlines() or [''])
        with self.print_lock:
            for line in lines[:TRACE_ROWS]:
                for part in _wrapped(line):
                    print(part, file=self.out, flush=True)
            if len(lines) > TRACE_ROWS:
                print('  ... [%d more rows]' % (len(lines) - TRACE_ROWS),
                      file=self.out, flush=True)


class NoBoard:
    """Stands in for the session when --no-board is given.

    A question about the code or the build does not need the serial port opened,
    and opening it locks the console for whoever else wants it. Any tool that
    reaches for the board gets a plain answer instead of a timeout.
    """

    board = property(lambda self: self._refuse())
    allow_writes = False

    def _refuse(self):
        raise RigError('this run was started with --no-board')

    def info(self, refresh=False):
        self._refuse()

    def close(self):
        pass

    def reset(self):
        pass


def parse(argv):
    import argparse

    parser = argparse.ArgumentParser(
        prog='dbg', description='Ask a local model about this board, cheaply.')
    parser.add_argument('question', nargs='*', help='ask and exit; omit for a prompt')
    parser.add_argument('--repl', action='store_true',
                        help='force the prompt loop even with piped input')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help='answer only: no tool trace, no token meter')
    parser.add_argument('-t', '--tools', default='code',
                        help='read|code|pins|build|docs|all|none or a comma '
                             'separated list')
    parser.add_argument('-m', '--model', default='gemma4:12b',
                        help="ollama tag, or 'auto' to pick one from this"
                             " machine's cores, RAM and VRAM - see"
                             " coaxial_ollama/capability.py")
    parser.add_argument('--ollama-host', default='http://localhost:11434')
    parser.add_argument('--allow-remote', action='store_true',
                        help='permit a cloud tag or a remote daemon; off by'
                             ' default, because the question carries the board'
                             ' with it')
    parser.add_argument('--words', type=int, default=300,
                        help='cap on generated tokens per turn - 180 clipped '
                             'an open-ended question often enough to be '
                             'annoying; this costs a bit more per turn and '
                             'clips less often')
    parser.add_argument('--format', dest='fmt',
                        help="'json' to make the answer machine readable. The"
                             " board tools are unaffected - they are already"
                             " schema checked - but a model told to answer in"
                             " JSON calls fewer of them, so -t none is usually"
                             " what you want with it")
    parser.add_argument('--keep-alive', default=None,
                        help="how long ollama holds the model, and with it the"
                             " cached prompt prefix. Default depends on the"
                             " mode: %s in a prompt loop, %s for one question,"
                             " because a question already answered is rarely"
                             " followed by another within the half hour. '0'"
                             " hands the VRAM back at once."
                             % (KEEP_ALIVE_REPL, KEEP_ALIVE_ONCE))
    parser.add_argument('--num-gpu', type=int, default=None,
                        help='layers on the GPU; the rest run on the CPU.'
                             ' Set for you by -m auto and by board_prompt.ps1')
    parser.add_argument('--lang',
                        help='answer in this language, whatever the machine '
                             'is set to. Default: the Windows locale, moved '
                             'only by a question in another language or by '
                             'asking for one. /lang changes it mid-session.')
    parser.add_argument('--detail', default=detail.AUTO, choices=detail.LEVELS,
                        help='how much documentation each tool carries into '
                             'every turn. auto reads the model tag: terse for '
                             'the sizes this bench runs locally, full for '
                             'anything with room to read it. %s overrides for '
                             'the whole machine.' % detail.ENV)
    parser.add_argument('--num-ctx', type=int, default=8192)
    parser.add_argument('--keep', type=int, default=6,
                        help='recent messages sent whole; older ones are stubbed')
    parser.add_argument('--budget', type=int, default=0,
                        help='stop asking after this many tokens')
    parser.add_argument('--think', action='store_true',
                        help='let a reasoning model think; costs a lot of tokens')
    parser.add_argument('--file', action='append', default=[],
                        help='attach a clipped file to the first question')
    parser.add_argument('--chars', type=int, default=2000,
                        help='how much of each --file to attach')
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--unit', type=int, default=1)
    board_mode = parser.add_mutually_exclusive_group()
    board_mode.add_argument('--no-board', action='store_true',
                            help='stub the board tools out; every one refuses')
    board_mode.add_argument('--simulated', action='store_true',
                            help='board tools work, against an invented '
                                 'board that never opens a port - see '
                                 'coaxial.simulated')
    parser.add_argument('--allow', default='python',
                        help='programs /sh and run_command may launch. '
                             'Building and flashing does not need anything '
                             'on this list - see the build_firmware tool, '
                             "which is in the default `code` set and always "
                             'runs tools/build_and_flash.py regardless of '
                             '--allow.')
    parser.add_argument('--allow-writes', action='store_true')
    parser.add_argument('--confirm', action='store_true',
                        help='ask before every state change - a pin write, '
                             'run_python, run_command. Off by default, same '
                             'as board_prompt without the flag; the two tools '
                             'this loop is actually built for, analog_read '
                             'and docs, are reads and never ask.')
    return parser.parse_args(argv)


def ask_operator(name, args):
    """The --confirm gate. Anything but y is a no, including a closed stdin."""
    print('\n  %s %s' % (name, json.dumps(args)[:400]))
    try:
        return input('  run it? [y/N] ').strip().lower() in ('y', 'yes')
    except (EOFError, KeyboardInterrupt):
        print('  declined')
        return False


def attach(paths, chars, limit=INPUT_LIMIT):
    """Files as context, clipped. A 3000 line log is not a question.

    Two limits, because --chars only ever bounded one file: ten of them at
    the default 2000 is 20k characters of attachment in front of a one-line
    question, which is the whole window before the board has been asked
    anything. The second bound is on the lot of them together.
    """
    blocks = []
    for path in paths:
        try:
            with open(path, encoding='utf-8', errors='replace') as handle:
                text = handle.read()
        except OSError as exc:
            blocks.append('%s: unreadable (%s)' % (path, exc))
            continue
        blocks.append('--- %s (%d chars, %d attached) ---\n%s'
                      % (path, len(text), min(len(text), chars),
                         clip(text, chars)))
    return clip_ends('\n'.join(blocks), limit)


def build(args):
    from .client import Ollama
    from .tools import Toolbox

    tag, gpu_layers = args.model, args.num_gpu
    if args.model == 'auto':
        from .capability import choose, probe
        picked = choose(probe())
        tag = picked.tag
        if gpu_layers is None:
            gpu_layers = picked.options.get('num_gpu')
        if not args.quiet:
            print('model: %s  (%s)' % (tag, picked.why))

    client = Ollama(tag, host=args.ollama_host,
                    num_ctx=args.num_ctx, num_predict=args.words,
                    think=True if args.think else False,
                    remote_ok=args.allow_remote,
                    keep_alive=keep_alive_for(args), fmt=args.fmt,
                    num_gpu=gpu_layers)
    if args.no_board:
        session = NoBoard()
    elif args.simulated:
        from coaxial.simulated import SimulatedSession
        session = SimulatedSession()
    else:
        from coaxial_mcp.session import Session
        session = Session(args.port, args.baud, args.unit)

    allow = [a for a in args.allow.split(',') if a.strip()]
    toolbox = Toolbox(session, shell=Shell(allow), scope=Scope(),
                      allow_writes=args.allow_writes,
                      confirm=ask_operator if args.confirm else None)
    chat = Chat(client, toolbox, tools=args.tools, keep=args.keep,
                budget=args.budget, quiet=args.quiet,
                detail_level=args.detail,
                session_language=args.lang or language.system_language())
    return client, session, chat


def repl(chat, hold=False):
    from .client import OllamaError

    # One line, in this machine's language. What the tools are, what the
    # detail level is and what a turn costs are all a /help away; printed on
    # the way in they were three lines nobody read twice.
    print(language.greeting(chat.client.model, chat.language,
                            getattr(sys.stdout, 'encoding', None)))
    if not ({'run_command', 'build_firmware'} & set(chat.tool_names)):
        # Printed once, here, by this host - not sent to the model, so it
        # costs nothing per turn. Measured on this bench: asked three times
        # running to build and flash, on the default `code` set - before it
        # carried build_firmware - the model correctly and repeatedly said
        # it could not: accurate, but a dead end with no way out of it short
        # of already knowing this flag. `code` carries build_firmware now;
        # this only still fires for `read`, `pins` or a custom list missing
        # both.
        confirmed = ' and already --confirm' if chat.toolbox.confirm else \
                   ', then /confirm too, or it writes with nobody asking'
        print('  no build_firmware or run_command in this set - it cannot '
              'build or flash. /tools code (or build) switches now, no '
              'restart%s.' % confirmed)
    if not sys.stdin.isatty():
        print('(reading commands from stdin)')
    try:
        while True:
            # Read fresh every time, not captured once: /reconnect flips this
            # mid-loop and the very next prompt is what should show it. The
            # lock is shared with Chat._trace() so a tick and a trace line
            # printed mid-question never interleave on the same stream, and
            # chat.out is pointed at the same tracked stream so the prompt
            # knows how many rows whatever _trace() prints actually add -
            # not a number decided once and trusted for the whole question.
            face = spin.prompt(PROMPT, sys.stdout, lock=chat.print_lock,
                               ok=chat.link_ok)
            chat.out = face.out
            try:
                line = input().strip()
            except (EOFError, KeyboardInterrupt):
                face.stop(chat.link_ok)
                print()
                break
            if not line:
                face.stop(chat.link_ok)
                continue
            face.busy()
            asked = False
            try:
                done = chat.command(line)
                if done is None:
                    asked = True
                    # See tools.py's afe_power gate: set from the real
                    # question text, here rather than inside ask() itself,
                    # so a scripted test driving Chat.ask() directly keeps
                    # its old, permissive default instead of needing "afe"
                    # in every unrelated fixture question.
                    chat.toolbox.afe_mentioned = 'afe' in line.lower()
                    # No note when the lock moves. It used to print
                    # "sprak: bytt till Swedish (last)" above the
                    # answer - a host line, in a mix of two languages,
                    # saying what the answer itself already shows by
                    # being in the new one. A bare switch answers
                    # "Okej" and nothing else, without a model turn.
                    done = chat.ask(line)
                # Stop ticking before the answer prints, not after. stop()'s
                # own repaint climbs back to the prompt row by the same
                # newline count _paint() uses, and a long answer with no
                # embedded '\n' that the terminal itself wraps across two
                # or more rows is invisible to that count either way - the
                # difference is *when* the wrong climb can land on top of
                # the answer. Frozen first, the climb happens while nothing
                # but the prompt's own row exists below it; done after, the
                # same wrong "one row up" lands mid-answer instead, which is
                # exactly what a bench session saw: the prompt group spliced
                # into the middle of a sentence. The exception branch below
                # already stops before it prints - this makes the ordinary
                # answer match it, rather than being the odd one out.
                face.stop(chat.link_ok)
                print(done, file=face.out)
            except SystemExit:
                face.stop(chat.link_ok)
                break
            except (RigError, ValueError, OllamaError) as exc:
                # A dead board and a dead model backend are the same shape of
                # failure here: something the session doesn't own crashed
                # mid-turn. One bad turn is not a reason to lose the rest of
                # the conversation - ollama respawns llama-server on the next
                # request, same as the board answers again once reconnected.
                asked = True
                face.stop(False)
                print('%s: %s%s' % (type(exc).__name__, exc, render.hint(exc)),
                      file=face.out)
            if asked:
                # Every question starts from nothing, on purpose: a growing
                # history is a growing prompt, and a growing prompt is more
                # for llama-server's own prompt cache to hold onto right up
                # to the std::bad_alloc it has crashed with more than once
                # this session. A slash command never touched history in the
                # first place, so it is left alone here.
                chat.history = []
        print(chat.cost_line())
    finally:
        # The 30-minute keep_alive that makes turn nine as quick as turn two
        # is exactly wrong once there is no turn ten coming. Measured on this
        # bench: a session left running unattended held 9.69 GB for another
        # 27 minutes at 1% utilisation. `--keep-alive` is how to say "no,
        # really, leave it" - anything explicit there means the operator
        # already decided, and this leaves that alone.
        if not hold:
            try:
                chat.client.unload()
            except OllamaError:
                pass
        chat.io_log.close()


def main(argv=None):
    from .client import OllamaError

    args = parse(argv)
    # Before anything prints: every path out of here, including the error
    # branches below, goes through a console that may not hold the alphabet
    # the answer arrives in.
    _printable(sys.stdout)
    _printable(sys.stderr)
    question = ' '.join(args.question).strip()
    if not question and not args.repl and not sys.stdin.isatty():
        # `sed -n 1,40p log | dbg` is a question about a log. Draining stdin
        # here would also swallow the prompt loop's input, so --repl skips it.
        #
        # Clipped, and from both ends: the pipe carries whatever the operator
        # aimed at it, and a whole build log or a captured session arrives
        # exactly as easily as forty lines do. Unbounded, it is the largest
        # single thing that can reach the daemon in one go - trim() would
        # have to clip it later anyway, and doing it here means the notice
        # says so before the model ever sees the question.
        question = clip_ends(sys.stdin.read().strip(), INPUT_LIMIT)

    try:
        client, session, chat = build(args)
    except OllamaError as exc:
        # A refused host or a cloud tag is a wiring mistake, not a bench fault:
        # there is no prompt loop worth opening against a model we will not use.
        print('ollama: %s' % exc, file=sys.stderr)
        return 2
    # Real sessions only - build() itself is what dozens of tests call
    # through, and none of them should write a file to do it. See IOLog.
    chat.io_log = IOLog()
    if args.simulated:
        # Loud on purpose, before the model ever answers a thing: board_info
        # says the same ("firmware": "simulated"), but a line here means
        # nobody has to ask a tool first to find out these readings are
        # invented, not measured.
        print('SIMULATED - no port opened, every board reading is invented',
              file=sys.stderr)
    interactive = args.repl or not question
    try:
        client.model = client.require_model()
    except OllamaError as exc:
        # Fatal for one question - there is nothing else to do. Not fatal for the
        # prompt loop: /py and /sh never touch the model, and being unable to
        # reach ollama is no reason to lose the shortest path to the board.
        if not interactive:
            print('ollama: %s' % exc, file=sys.stderr)
            return 2
        print('ollama: %s' % exc, file=sys.stderr)
        print('slash commands still work; questions will not.', file=sys.stderr)

    # Also what the prompt's face shows in the REPL below: green once this is
    # True, red once it is not - --no-board counts as not, since board tools
    # will fail there by design, same as a dead cable.
    #
    # Not probed here any more, on purpose - see FINDINGS/this session's own
    # history: connecting eagerly used to be the only thing standing between
    # a dead link and the model "answering past" it, which is why it moved
    # session.board's own lazy connect up here in the first place. That
    # reason is gone now: link_diagnose and the link_error override in
    # ask() cover it, and cover it better - the model gets a real chance to
    # help troubleshoot a board that never answered instead of the session
    # printing a failure and, for a one-shot question, exiting before it was
    # ever asked anything. The board is touched exactly when a tool call
    # actually needs it, which is what "not per default" means here.
    link_ok = not args.no_board
    chat.link_ok = link_ok

    extra = attach(args.file, args.chars) if args.file else ''
    try:
        if question and not args.repl:
            full_question = '\n'.join(filter(None, (question, extra)))
            chat.toolbox.afe_mentioned = 'afe' in full_question.lower()
            answer = chat.ask(full_question)
            print(answer)
            if not args.quiet:
                print(chat.cost_line(), file=sys.stderr)
        else:
            repl(chat, hold=args.keep_alive is not None)
    except OllamaError as exc:
        print('ollama: %s' % exc, file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        return 130
    finally:
        try:
            session.close()
        except RigError:
            pass
        # Unconditional, and close() is idempotent: repl() closes on its own
        # way out, but a one-shot question never enters repl() at all, and
        # `python dbg.py` with no question enters it despite args.repl being
        # False. Guarding on args.repl got that last case wrong - harmlessly,
        # since the second close is a no-op, but only by accident.
        chat.io_log.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
