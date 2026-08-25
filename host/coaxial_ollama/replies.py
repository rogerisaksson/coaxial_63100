"""Reading a model's reply: what it meant, as opposed to what it typed.

Everything here is a pure function over text the model produced, with no
board, no client and no conversation state - which is why it is its own
module rather than more of debug.py. Three jobs, each documented at the
function that does it:

  * `is_retype` - is this answer just the tool's own table typed out again?
  * `salvage_calls` - is this "answer" actually a tool call the model wrote
    into content instead of into tool_calls?
  * `is_marker_noise` - the veto that keeps the salvage from turning a
    sentence that merely quotes JSON into a board command.

Every rule in here came from a transcript on this bench, and each one says
which. See docs/MODELS.md for the same failures written up at length.
"""
import json
import re

BACKSLASH = chr(92)
TOOL_TAG = re.compile(r'</?tool_call>', re.I)

# The channel names off the front of an analog_read row: "0  PhaseU  diff ..."
# -> 'phaseu'. Anchored on the mode column (diff/SE) rather than just a
# leading digit, or this matches render.analog's own header line too - "64
# samples @2000Hz" starts with a number and a word exactly like a row does,
# without the anchor 'smp' was recognised as a seventh channel of its own.
READING_ROW = re.compile(r'^\d+\s+(\S+)\s+(?:diff|SE)\b', re.M)

# The same name off a row of board_info's channel map, where it is the
# last field rather than the second:
# "0  3   PC3_C/PC2_C  in    diff PhaseU". Measured: asked for a list of
# the analog channels, the trace printed the map and the model typed the
# seven names out underneath it - two lists where the first was already
# the answer. Same rule as a retyped reading, same reason; a map row just
# does not look like a reading row.
MAP_ROW = re.compile(r'^\d+\s+\d+\s+\S+\s+\S+\s+(?:diff|SE)\s+(\S+)\s*$',
                     re.M)

# A digital row, from the map ("PB2  out   AFE_ON") or from a reading of
# them ("PB2  out   1     AFE_ON"). The pin is what a retyped list names,
# and it is the one field that cannot contain a space.
DIGITAL_ROW = re.compile(r'^(P[A-K]\d+)\s+(?:in|out|inout)\b', re.M)

# ...and the signal off the same row, because a retyped list quotes
# whichever half it read. Measured: the trace said "PB2 out 1 AFE_ON /
# PE15 in 0 nFAULT" and the answer said "AFE_ON ar 1 och nFAULT ar 0" -
# every channel named, and not one of them by the pin the pattern above
# captures. The optional digits eat the level column, which the map has
# and a reading does not.
DIGITAL_SIGNAL = re.compile(r'^P[A-K]\d+\s+(?:in|out|inout)\s+(?:\d+\s+)?(\S.*?)\s*$',
                            re.M)

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

# A tool name distinctive enough that seeing one in prose is real evidence the
# model described the call it should have made instead of making it - not
# 'link' or 'docs', ordinary words that turn up in unrelated sentences too
# often to mean anything. Measured on this bench: asked for a table, gemma4:12b
# answered "jeg ma utfore en `analog_read`" and stopped there - it knew
# exactly what to do and did not do it.
NAMED_TOOL = re.compile(r'\b(analog_read|afe_power|board_info|self_test|'
                        r'gpio_pin|gpio_port|test_gate|run_python|'
                        r'run_command|build_firmware|run_tests|'
                        r'link_diagnose)\b')


def is_retype(answer, channels, minimum=RESTATE_MIN_CHANNELS):
    """Whether `answer` is a mechanical restatement of a reading's `channels`.

    Two shapes count, either being enough on its own: every channel named
    (the original catch, for a restatement written out as prose), or the
    answer has the shape of a markdown table at all (which catches one that
    got cut off before naming the last channel - see MARKDOWN_TABLE_ROW).
    A real table is never a legitimate answer here regardless of length,
    since SYSTEM already says not to write one.

    `minimum` is why the default is not simply 2. On a *reading*, naming
    two channels is plausibly synthesis - "NTC and DCbus both read low" -
    and silencing that would cost a real finding. A *map* has no values
    to synthesise about: listing the channels IS the map, so the caller
    passes 2 there. Measured, under the map's own two digital rows: "De
    digitala kanalerna ar: PB2 (utgang) for AFE_ON, PE15 (ingang) for
    nFAULT".
    """
    if not (answer and channels):
        return False
    if MARKDOWN_TABLE_ROW.search(answer):
        return True
    return (len(channels) >= minimum
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


def is_marker_noise(text):
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


def json_objects(text):
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


def salvage_calls(text):
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
    for start, end, parsed in json_objects(stripped):
        if not isinstance(parsed, dict) or not parsed.get('name'):
            continue
        calls.append({'function': {'name': parsed['name'],
                                   'arguments': parsed.get('arguments') or {}}})
        residue.append(stripped[cursor:start])
        cursor = end
    residue.append(stripped[cursor:])

    if not calls:
        return [], text
    if not is_marker_noise(' '.join(residue)):
        # Prose around it: an answer that mentions a call, not a call.
        return [], text
    return calls, ''
