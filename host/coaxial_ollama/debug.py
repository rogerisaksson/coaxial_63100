"""A lean prompt loop for debug jobs: fewest tokens in, fewest tokens out.

    python dbg.py "the NTC reads exactly 25.00 - what is wrong?"
    python dbg.py                      # interactive
    python dbg.py -q "which channel is the DC link?"

The runner in runner.py is built for a test plan: a long system prompt, eleven
tools, a fresh conversation per step, a transcript of everything. Every one of
those is right for a report somebody signs and wrong for a question you ask
sixty times an afternoon. This module is the same board and the same tools with
the cost turned down:

  1. A system prompt of about seventy tokens instead of three hundred and fifty.
  2. Five tools by default, not eleven. The tool list is re-sent on every single
     turn, so it is the one cost that scales with turns no matter how short the
     question is - `--tools` picks the subset the job needs.
  3. `num_predict` caps the answer. Debug answers are one or two sentences; a
     reasoning model asked an open question will happily produce eight hundred
     tokens of deliberation instead.
  4. `think` off where the model supports it, which removes that deliberation
     rather than just truncating it.
  5. Old turns are stubbed, not resent whole. A tool result from four questions
     ago is worth its first line, not its forty.
  6. Slash commands run without the model at all. `/py board.afe.state()` and
     `/sh cube-cmake --build --preset Debug` cost zero tokens, and half of what
     one asks a model at a bench is really just "run this and show me".

And it tracks what each turn cost, in and out - `/cost` for the running total,
`--budget` to stop asking once it is spent - without printing a line after
every single turn: measured in daily use, that was screen noise nobody was
reading, sitting between the question and the answer it was about.
"""
import json
import re
import sys
import time

sys.path.insert(0, __file__.rsplit('coaxial_ollama', 1)[0])

from coaxial.errors import RigError                  # noqa: E402
from coaxial_mcp import render                        # noqa: E402

from . import language
from . import tools as toolmod                       # noqa: E402
from .spinner import spinning_prompt                 # noqa: E402
from .sandbox import Scope, Shell, clip              # noqa: E402

# Deliberately terse, and every line of it earns its place. No restating the
# protocol, no channel map - board_info carries that, once, when asked.
SYSTEM = """You are an expert with a serial communication link to a coaxial
BLDC inverter.
Use tools; do not guess. Answer in one or two sentences, no preamble.
Asked for a table or list, call analog_read once - its grid already covers
every channel. Never write one yourself in markdown, and never restate rows a
tool result already printed - one short line, not a second listing.
If a call errors, say so - not a guess, not an old reading.
afe_power also powers the ADC reference: off, every channel reads mid-scale
and the NTC reads exactly 25.00 C - not a measurement. Phase channels sit
behind unknown gain - pin volts only.
Values come from analog_read, never docs - HARDWARE and FINDINGS explain a
reading; they don't produce one."""

# Named subsets, because a debug job knows roughly what it is about to touch.
SETS = {
    'read': ('board_info', 'analog_read', 'self_test', 'afe_power', 'link',
             'docs'),
    'code': ('board_info', 'analog_read', 'self_test', 'afe_power', 'link',
             'docs', 'run_python'),
    'pins': ('board_info', 'gpio_pin', 'gpio_port', 'test_gate', 'afe_power',
             'docs'),
    'all': tuple(spec['name'] for spec in toolmod.TOOLS if spec['name'] != 'report'),
    'none': (),
}

HELP = """  /py CODE      run python against the board, no model, no tokens
  /sh CMD       run an allowlisted command, no model
  /reconnect    drop and reopen the board link, no model
  /tools [set]  read|code|pins|all|none, or a comma separated list
  /ctx          what the next turn will cost
  /clear        forget the conversation - the cheapest thing here
  /cost         tokens so far
  /help  /q"""


# What the prompt loop shows. The board's name rather than the script's: the
# window this appears in is usually one of several, and 'dbg>' says which
# program is running where the useful thing to know is which bench.
PROMPT = 'Coaxial_63100'

# How long ollama holds the weights after the last turn. Two numbers, because
# the two modes want opposite things.
#
# In a prompt loop the model is about to be asked again, and the KV cache of
# the prefix is what makes turn nine as quick as turn two - worth 8 GB of VRAM.
# After a single -q question it is not: measured on this bench, a one-shot left
# 9.69 GB resident and expiring 27 minutes later at 1 % utilisation, on a card
# whose desktop then had 3.8 GB to work in. That is the cost of a cache nobody
# is going to hit.
KEEP_ALIVE_REPL = '30m'
KEEP_ALIVE_ONCE = '2m'


def keep_alive_for(args):
    """What the caller asked for, or what the mode implies."""
    if args.keep_alive is not None:
        return args.keep_alive
    return KEEP_ALIVE_REPL if args.repl else KEEP_ALIVE_ONCE


TOOL_TAG = re.compile(r'</?tool_call>', re.I)
BACKSLASH = chr(92)
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

# Rows of a tool result shown in full. Generous enough for every channel this
# board has, or a full self_test list, without an unbounded dump if run_python
# prints something much longer.
TRACE_ROWS = 24

# Tools where the same call twice is meaningful - a reading changes with time,
# and code is trusted to know why it is running again. Everything else answers
# the same question with the same fact each time, so a repeat within one turn
# is not new information; it is the model unable to tell that it already has
# the answer. Measured on this bench: asked to tabulate the channels,
# qwen2.5:14b turned the front end on, then on again three more times in a
# row, each one a full round trip through the model and the board for a
# result it had already seen.
REPEATABLE = {'analog_read', 'run_python', 'run_command'}

# The channel names off the front of an analog_read row: "0  PhaseU  diff ..."
# -> 'phaseu'. Anchored on the mode column (diff/SE) rather than just a
# leading digit, or this matches render.analog's own header line too - "64
# smp @2000Hz" starts with a number and a word exactly like a row does, and
# without the anchor 'smp' was recognised as a seventh channel of its own.
READING_ROW = re.compile(r'^\d+\s+(\S+)\s+(?:diff|SE)\b', re.M)

# Fewer than this many channels and a short answer naming all of them is
# plausibly synthesis ("NTC and DCbus both read low") rather than a mechanical
# restatement - the case this exists to catch always names a full table's
# worth. Below this the override stays out of the way.
RESTATE_MIN_CHANNELS = 3

# Two or more pipe-delimited lines, the shape of a markdown table row or its
# `| :--- |` header separator. Measured on this bench: asked to "tabellera",
# gemma4:12b wrote the real reading as a markdown table and then hit the
# --words cap partway through the last row - which meant it had not yet named
# every channel, so the all-channels-present check below never matched and
# the cut-off table printed anyway. This checks the SHAPE of a table instead
# of waiting for it to finish naming channels, so a truncated one is caught
# exactly as surely as a complete one.
MARKDOWN_TABLE_ROW = re.compile(r'^\s*\|.*\|\s*$', re.M)


def _is_retype(answer, channels):
    """Whether `answer` is a mechanical restatement of a reading's `channels`.

    Two shapes count, either being enough on its own: every channel named
    (the original catch, for a restatement written out as prose), or the
    answer has the shape of a markdown table at all (which catches one that
    got cut off before naming the last channel - see MARKDOWN_TABLE_ROW).
    A real table is never a legitimate answer here regardless of length,
    since SYSTEM already says not to write one.
    """
    if not (answer and channels):
        return False
    if MARKDOWN_TABLE_ROW.search(answer):
        return True
    return (len(channels) >= RESTATE_MIN_CHANNELS
           and all(re.search(r'\b%s\b' % re.escape(ch), answer, re.I)
                  for ch in channels))


# Words a chat template leaks around a call the model wrote as text instead of
# in the tool_calls field. Measured on this bench: asked "vad ar temperaturen",
# the model answered 'CallCheckFunction' and a JSON object, twice over, and the
# prompt printed all four lines as the answer - which reads as the board having
# stopped giving values. A residue of nothing but these words is still a tool
# call; one word of real prose is not, and vetoes the salvage.
MARKERS = frozenset(('tool', 'tool_call', 'toolcall', 'call', 'calls',
                     'function', 'functions', 'check', 'json', 'assistant',
                     'commentary', 'to', 'and', 'then'))
WORD = re.compile(r'[^\W\d_]+')
# Those words arrive run together as often as spaced - 'CallCheckFunction' is
# one word to any tokeniser and three markers to a reader - so a residue word
# is split at its capitals before being looked up.
CAMEL = re.compile(r'[^\W\d_][a-z]*')


def _is_marker_noise(text):
    """True when nothing in `text` is a word of prose.

    This is the whole safety of the salvage: a message that is a tool call
    wearing a template's clothes has only marker words around the JSON, and a
    message that is an answer has real ones.
    """
    for word in WORD.findall(text):
        parts = CAMEL.findall(word) or [word]
        if any(part.lower() not in MARKERS for part in parts):
            return False
    return True


def _json_objects(text):
    """Every balanced top-level {...} in `text`, as (start, end, parsed).

    Brace counting rather than a regex, because `{.*?}` stops at the first
    closing brace - which in a tool call is the one that ends `arguments`, so
    anything with a nested object in it either fails to parse or parses as half
    of itself. Strings are tracked so a brace inside a value cannot unbalance
    the count. An object that is not JSON is skipped, not fatal.
    """
    found = []
    depth = start = 0
    in_string = escaped = False
    for index, char in enumerate(text):
        if in_string:
            if escaped:
                escaped = False
            elif char == BACKSLASH:
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == '{':
            if not depth:
                start = index
            depth += 1
        elif char == '}' and depth:
            depth -= 1
            if not depth:
                try:
                    parsed = json.loads(text[start:index + 1])
                except ValueError:
                    continue
                found.append((start, index + 1, parsed))
    return found


def _salvage_calls(text):
    """The tool calls a model wrote as text, and what is left of the text.

    Deliberately narrow. A legitimate answer may quote JSON, and turning that
    into a board command would be far worse than printing it - so the test is
    that once the tool tags and the call objects are taken out, no word of
    prose is left, only the marker words above. Returns (calls, remaining
    text): an empty list means the text was an answer after all.

    More than one call in one message is the case that made this a list. The
    single-call version printed a two-call message verbatim, which at the
    prompt looks exactly like the model having stopped taking readings.
    """
    stripped = TOOL_TAG.sub(' ', text)
    if '"name"' not in stripped:
        # No call in it, but a bare `</tool_call>` is not an answer either:
        # hand back what is left once the tag is gone, which may be nothing.
        clean = stripped.strip()
        return [], clean if clean != text.strip() else text

    calls, residue, cursor = [], [], 0
    for start, end, parsed in _json_objects(stripped):
        if not isinstance(parsed, dict) or not parsed.get('name'):
            continue
        calls.append({'function': {'name': parsed['name'],
                                   'arguments': parsed.get('arguments') or {}}})
        residue.append(stripped[cursor:start])
        cursor = end
    residue.append(stripped[cursor:])

    if not calls:
        return [], text
    if not _is_marker_noise(' '.join(residue)):
        # Prose around it: an answer that mentions a call, not a call.
        return [], text
    return calls, ''


def _printable(stream):
    """Make a Windows console survive an answer in somebody else's alphabet.

    The model answers in the language it was asked in, and the console here
    encodes with the locale codepage - cp1252 on this bench. Swedish and German
    are inside it and render correctly; an ohm sign, a Polish l-stroke or any
    Cyrillic is not, and the default error handler turns that into a
    UnicodeEncodeError that kills the answer after the measurement was already
    taken. Replacing the character loses a glyph; raising loses the reading.

    The codepage is left alone on purpose. Forcing UTF-8 would fix the encode
    and hand a legacy console mojibake for the characters it *can* display,
    which is a worse trade for the languages actually spoken at this bench.
    """
    try:
        stream.reconfigure(errors='replace')
    except (AttributeError, OSError, ValueError):
        pass
    return stream


def approx_tokens(text):
    """Rough but honest: about four characters per token for dense ASCII."""
    return max(1, len(str(text)) // 4)


class Chat:
    """One conversation, trimmed on the way out to the model.

    The history is kept whole locally - it costs nothing on this side, and a
    stubbed line is impossible to un-stub. What goes over the wire is the trim.
    """

    def __init__(self, client, toolbox, tools='read', keep=6, budget=0,
                 quiet=False, out=None, link_ok=True):
        self.client = client
        self.toolbox = toolbox
        self.keep = keep
        self.budget = budget
        self.quiet = quiet
        self.out = out or _printable(sys.stdout)
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
            [spec for spec in toolmod.TOOLS if spec['name'] in names]) or None
        return names

    def tool_cost(self):
        """What the tool list alone costs, every turn, before any question."""
        return approx_tokens(json.dumps(self.schemas or []))

    def trim(self):
        """System prompt, stubbed history, recent turns whole.

        Tool results are where the tokens are, so they are what gets stubbed:
        the first line of a channel table is enough to remember that it was
        read, and the model can read it again if it matters.
        """
        head = self.history[:-self.keep] if self.keep else self.history
        tail = self.history[-self.keep:] if self.keep else []

        # The language line is built per turn from the last thing the user
        # actually typed, rather than asked of the model. See language.py: told
        # to work out the language itself, qwen2.5:14b answered a European
        # question in Chinese, Japanese and Thai. Naming it removes the step
        # that was going wrong.
        #
        # It changes only when the user changes language, so the cached prefix
        # survives an ordinary conversation and pays a reload exactly when the
        # conversation genuinely switched.
        asked = ''
        for message in reversed(self.history):
            if message['role'] == 'user':
                asked = message.get('content') or ''
                break
        sent = [{'role': 'system',
                 'content': SYSTEM + '\n' + language.instruction(asked)}]
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
        return sent

    def context_cost(self):
        return approx_tokens(json.dumps(self.trim())) + self.tool_cost()

    # ---- a turn ------------------------------------------------------------

    def ask(self, question, max_calls=6):
        """One question, however many tool calls it takes. Returns the answer."""
        if self.over_budget():
            return 'budget of %d tokens is spent; /clear or raise --budget' \
                % self.budget

        self.history.append({'role': 'user', 'content': question})
        answer = ''
        link_error = None
        last_channels = None      # names in the most recent analog_read table
        read_attempted = False    # analog_read was called this turn, win or lose
        seen = {}          # (name, args) this turn -> its rendered result

        for _ in range(max_calls + 1):
            before = self.client.usage()
            message = self.client.chat(self.trim(), self.schemas)
            after = self.client.usage()
            self._meter(after['prompt_tokens'] - before['prompt_tokens'],
                        after['eval_tokens'] - before['eval_tokens'])

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
                salvaged, answer = _salvage_calls(answer)
                if salvaged:
                    calls = salvaged
                    message = dict(message, content='', tool_calls=calls)

            self.history.append(message)
            if not calls:
                break

            for call in calls:
                name = (call.get('function') or {}).get('name', '?')
                if name == 'analog_read':
                    read_attempted = True
                args = _arguments(call)
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
                if name == 'analog_read' and not str(raw).startswith('ERR'):
                    # A fresh table replaces the last one remembered; an error
                    # leaves the previous table in place rather than wiping it,
                    # since link_error already takes priority below either way.
                    last_channels = set(m.lower()
                                        for m in READING_ROW.findall(str(raw)))
                    self.last_channels = last_channels
                self._trace(result)
                self.history.append({'role': 'tool', 'tool_name': name,
                                     'name': name,
                                     'content': '%s: %s' % (name, result)})

        # A read that failed on the wire is ground truth this loop already
        # has; the model does not get a vote on it. Measured on this bench:
        # asked again after the ST-Link (which carries this board's VCP - see
        # HARDWARE.md) was unplugged, qwen2.5:14b answered with the NTC value
        # from three questions earlier instead of the ConnectError sitting
        # right there in its own context. The system prompt already tells it
        # not to; this is the case where telling it was not enough.
        if link_error is not None:
            answer = 'link is down, not answered: %s' % link_error
        # The system prompt already says not to retype a table just shown -
        # measured here, qwen2.5:14b did it anyway, every time, across three
        # separate bench sessions. Telling it was not enough, so this is the
        # same backstop as the line above: not a smarter prompt, a fact the
        # loop already has that the model does not get a vote on. The bar is
        # deliberately narrow - every channel just read, named again by an
        # answer with nothing else in it - so a real one-line finding ("NTC
        # is running hot") that happens to name a channel or two is untouched.
        #
        # Silence, not a line saying so: the table is the trace directly above
        # this, on the same screen, and a reader looking at it does not need
        # to be told it is not being typed out again.
        elif _is_retype(answer, last_channels):
            answer = ''
        # The two backstops above only look at *this* turn's analog_read calls,
        # so a turn that never calls it at all slips past both. Measured on
        # this bench: asked "tabellera ADC-värdena" again after the link was
        # cut, gemma4:12b answered with a full table of plausible values one
        # round trip later - no analog_read in the trace, just a
        # slightly-perturbed rewrite of the real table from earlier in the
        # conversation. SYSTEM already says never to answer with an older
        # reading; this is the same class of "telling it was not enough" as
        # the two checks above, just for the turn that skips the read rather
        # than one that makes it and gets an error. `self.last_channels`
        # survives across turns for exactly this - the turn-local
        # `last_channels` above is always None here, since a successful
        # analog_read this turn would have set it too.
        elif not read_attempted and _is_retype(answer, self.last_channels):
            answer = 'no reading taken this turn - ask again.'
        # An answer that hit the token cap stops mid-sentence, and a table that
        # stops mid-row reads as complete to everyone except a reader counting
        # rows. Say so rather than letting the cap look like the end.
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
            return HELP
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
        if verb == 'ctx':
            return '%d messages, next turn about %d tok in, %d of it tools' \
                % (len(self.history), self.context_cost(), self.tool_cost())
        if verb == 'cost':
            return self.cost_line()
        return 'no such command. /help'

    def _reconnect(self):
        """Drop the link and try to reopen it - for a cable that was plugged
        back in without restarting this whole prompt loop.

        Session.reset() forgets the cached board (a no-op on NoBoard); the
        board property that follows is what actually reopens the port, and
        catching its RigError here is the same eager check main() runs at
        startup, just runnable again on demand.
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
        lines = str(result).splitlines() or ['']
        for line in lines[:TRACE_ROWS]:
            print('  %s' % line[:96], file=self.out, flush=True)
        if len(lines) > TRACE_ROWS:
            print('  ... [%d more rows]' % (len(lines) - TRACE_ROWS),
                  file=self.out, flush=True)


def _arguments(call):
    args = (call.get('function') or {}).get('arguments')
    if isinstance(args, str):
        try:
            args = json.loads(args or '{}')
        except ValueError:
            return {}
    return args if isinstance(args, dict) else {}


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
                        help='read|code|pins|all|none or a comma separated list')
    parser.add_argument('-m', '--model', default='gemma4:12b',
                        help="ollama tag, or 'auto' to pick one from this"
                             " machine's cores, RAM and VRAM - see"
                             " coaxial_ollama/capability.py")
    parser.add_argument('--ollama-host', default='http://localhost:11434')
    parser.add_argument('--allow-remote', action='store_true',
                        help='permit a cloud tag or a remote daemon; off by'
                             ' default, because the question carries the board'
                             ' with it')
    parser.add_argument('--words', type=int, default=180,
                        help='cap on generated tokens per turn')
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
    parser.add_argument('--no-board', action='store_true')
    parser.add_argument('--allow', default='python,cube-cmake',
                        help='programs /sh and run_command may launch')
    parser.add_argument('--allow-writes', action='store_true')
    return parser.parse_args(argv)


def attach(paths, chars):
    """Files as context, clipped. A 3000 line log is not a question."""
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
    return '\n'.join(blocks)


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
    else:
        from coaxial_mcp.session import Session
        session = Session(args.port, args.baud, args.unit)

    allow = [a for a in args.allow.split(',') if a.strip()]
    toolbox = Toolbox(session, shell=Shell(allow), scope=Scope(),
                      allow_writes=args.allow_writes)
    chat = Chat(client, toolbox, tools=args.tools, keep=args.keep,
                budget=args.budget, quiet=args.quiet)
    return client, session, chat


def repl(chat):
    print('%s, tools: %s (%d tok/turn). /help, /q to leave.'
          % (chat.client.model, ', '.join(chat.tool_names) or 'none',
             chat.tool_cost()))
    if not sys.stdin.isatty():
        print('(reading commands from stdin)')
    while True:
        try:
            # Read fresh every time, not captured once: /reconnect flips this
            # mid-loop and the very next prompt is what should show it.
            stop = spinning_prompt(PROMPT, sys.stdout, ok=chat.link_ok)
            try:
                line = input().strip()
            finally:
                # Before anything else is printed: a frame landing in the
                # middle of an answer puts a glyph inside the text.
                stop()
        except (EOFError, KeyboardInterrupt):
            print()
            break
        if not line:
            continue
        try:
            done = chat.command(line)
            print(done if done is not None else chat.ask(line))
        except SystemExit:
            break
        except (RigError, ValueError) as exc:
            print('%s: %s%s' % (type(exc).__name__, exc, render.hint(exc)))
    print(chat.cost_line())


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
        question = sys.stdin.read().strip()

    try:
        client, session, chat = build(args)
    except OllamaError as exc:
        # A refused host or a cloud tag is a wiring mistake, not a bench fault:
        # there is no prompt loop worth opening against a model we will not use.
        print('ollama: %s' % exc, file=sys.stderr)
        return 2
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

    # Also what the prompt's spinner shows in the REPL below: green and
    # turning forward once this is True, red and turning backward once it
    # is not - --no-board counts as not, since board tools will fail there
    # by design, same as a dead cable.
    link_ok = False
    if not args.no_board:
        # Opened here rather than left to the first board tool call, so a dead
        # link is a clear failure before any tokens are spent - not a tool
        # error the model reads and, with nothing telling it to stop, may
        # answer past anyway. Session.board is lazy, so this is the same
        # connect that would happen on first use, just moved earlier.
        try:
            session.board
        except RigError as exc:
            message = 'board: %s%s' % (exc, render.hint(exc))
            if not interactive:
                print(message, file=sys.stderr)
                return 2
            print(message, file=sys.stderr)
            print('slash commands still work; board tools will fail until the '
                  'link is up.', file=sys.stderr)
        else:
            link_ok = True
    chat.link_ok = link_ok

    extra = attach(args.file, args.chars) if args.file else ''
    try:
        if question and not args.repl:
            answer = chat.ask('\n'.join(filter(None, (question, extra))))
            print(answer)
            if not args.quiet:
                print(chat.cost_line(), file=sys.stderr)
        else:
            repl(chat)
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
    return 0


if __name__ == '__main__':
    sys.exit(main())
