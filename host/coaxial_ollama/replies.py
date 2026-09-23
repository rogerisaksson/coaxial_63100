"""Reading a model's reply: what it meant, as opposed to what it typed."""
import json
import re

BACKSLASH = chr(92)
TOOL_TAG = re.compile(r'</?tool_call>', re.I)

# The channel names off the front of an analog_read row: "0 PhaseU diff..." ->
# 'phaseu'.
READING_ROW = re.compile(r'^\d+\s+(\S+)\s+(?:diff|SE)\b', re.M)

# The same name off a row of board_info's channel map, where it is the last
# field rather than the second: "0 3 PC3_C/PC2_C in diff PhaseU".
MAP_ROW = re.compile(r'^\d+\s+\d+\s+\S+\s+\S+\s+(?:diff|SE)\s+(\S+)\s*$',
                     re.M)

# A digital row, from the map ("PB2 out AFE_ON") or from a reading of them
# ("PB2 out 1 AFE_ON").
DIGITAL_ROW = re.compile(r'^(P[A-K]\d+)\s+(?:in|out|inout)\b', re.M)

# ...and the signal off the same row, because a retyped list quotes whichever
# half it read.
DIGITAL_SIGNAL = re.compile(r'^P[A-K]\d+\s+(?:in|out|inout)\s+(?:\d+\s+)?(\S.*?)\s*$',
                            re.M)

# Fewer than this many channels and a short answer naming all of them is
# plausibly synthesis ("NTC and DCbus both read low") rather than a mechanical
# restatement - the case this exists to catch always names a full table's
# worth.
RESTATE_MIN_CHANNELS = 3

# ...and this many words beyond the table before it is saying something of its
# own.
RESTATE_MAX_EXTRA = 15

# Per channel, because a restatement of N rows carries N rows' worth of
# connective words - "PB2 (utgang) for AFE_ON" spends three on every row it
# copies.
RESTATE_EXTRA_PER_CHANNEL = 4

WORDS = re.compile(r'[^\W_]+')

# Two or more pipe-delimited lines, the shape of a markdown table row or its `|
# :--- |` header separator.
MARKDOWN_TABLE_ROW = re.compile(r'^\s*\|.*\|\s*$', re.M)

# A tool name distinctive enough that seeing one in prose is real evidence the
# model described the call it should have made instead of making it - not
# 'link' or 'docs', ordinary words that turn up in unrelated sentences too
# often to mean anything.
NAMED_TOOL = re.compile(r'\b(analog_read|afe_power|board_info|self_test|'
                        r'gpio_pin|gpio_port|test_gate|run_python|'
                        r'run_command|build_firmware|run_tests|'
                        r'link_diagnose)\b')


def is_retype(answer, channels, minimum=RESTATE_MIN_CHANNELS):
    """Whether `answer` is a mechanical restatement of a reading's
    `channels`.
    """
    if not (answer and channels):
        return False
    if MARKDOWN_TABLE_ROW.search(answer):
        return True
    # A markdown table is caught above whatever it says: SYSTEM says never to
    # write one, and a long one is worse than a short one.
    words = [w.lower() for w in WORDS.findall(answer)]
    # A channel's own name, and the pieces WORDS splits it into.
    named = set()
    for channel in channels:
        text = str(channel).lower()
        named.add(text)
        named.update(w.lower() for w in WORDS.findall(text))
    extra = [w for w in words if w not in named and not w.isdigit()]
    allowed = max(RESTATE_MAX_EXTRA,
                  RESTATE_EXTRA_PER_CHANNEL * len(named))
    if len(extra) > allowed:
        return False
    # Lookarounds rather than \b: a channel called "+5V" starts with a
    # character that is not a word character, so \b before it can only match
    # after another word character - never after the space it actually follows.
    return (len(channels) >= minimum
           and all(re.search(r'(?<!\w)%s(?!\w)' % re.escape(ch), answer, re.I)
                  for ch in channels))


# Words a chat template leaks around a call the model wrote as text instead of
# in the tool_calls field.
MARKERS = frozenset(('tool', 'tool_call', 'toolcall', 'call', 'calls',
                     'function', 'functions', 'check', 'json', 'assistant',
                     'commentary', 'to', 'and', 'then'))
WORD = re.compile(r'[^\W\d_]+')
# Those words arrive run together as often as spaced - 'CallCheckFunction' is
# one word to any tokeniser and three markers to a reader - so a residue word
# is split at its capitals before being looked up.
CAMEL = re.compile(r'[^\W\d_][a-z]*')


def is_marker_noise(text):
    """True when nothing in `text` is a word of prose."""
    for word in WORD.findall(text):
        parts = CAMEL.findall(word) or [word]
        if any(part.lower() not in MARKERS for part in parts):
            return False
    return True


def json_objects(text):
    """Every balanced top-level {...} in `text`, as (start, end, parsed)."""
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
    """The tool calls a model wrote as text, and what is left of the text."""
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
