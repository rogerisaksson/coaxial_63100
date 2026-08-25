"""Compile the operator's sentence into an intent, before the model answers it.

One turn used to do two jobs at once: work out what was being asked, and
answer it with the right tool. The failures were all the first job showing up
in the second - "ge mig en lista over de analoga vardena" carries the word for
a map and the word for a read in one sentence, and a single pass took the
verb. Split, the ambiguous half is a classification with seven answers, which
is what a 12b model is reliably good at.

The compiled intent is a *hint*, never a substitute. The operator's sentence
still reaches the model verbatim, in their own language, and the answer is
still the model's. What the hint adds is one line naming the tool the intent
maps to.

Every way the compile can fail returns None and the turn proceeds exactly as
it did before this module existed - no hint, one call, old behaviour:

  * ollama unreachable, or the extra call raising for any reason
  * a reply that is not the JSON it was asked for
  * an intent this file has no name for

That is the whole of the error handling, and it is the same shape as
tools/pick_tests.py for the same reason: a compiler that guesses wrong and
insists is worse than one that stays quiet.
"""
import json

# What an operator can be asking for at this bench. Seven, because the axis
# that kept being confused is one line of it - map against read - and a model
# choosing between seven named things is doing something easier than a model
# choosing between fifteen tool schemas.
INTENTS = {
    'map':     'which channels or pins exist - the board layout, no values',
    'read':    'the present value of one or more channels or pins',
    'power':   'turn the analog front end on or off',
    'devices': 'the other nodes: list them, or start talking to one by name',
    'link':    'the link is failing - nothing answers, and why not',
    'words':   'explain, describe, define, compare - an answer in words',
    'control': 'switch which board, model or language the host itself uses',
}

# Measured against gemma4:12b, 12 questions, ~2.75 s each. "kommunicera med
# hoger kna" and "byt till debugproben" both came back 'link' at first,
# because the catalogue called link "the serial link itself: is it up" and
# both questions are about a connection in some sense. Narrowing link to a
# failure, and saying "start talking to one by name" under devices, moved the
# first: 11 of 12.
#
# The twelfth is still wrong and still does not matter: board_switch() in
# debug.py carries "byt till debugproben" out itself, for no model tokens,
# before anything reaches here. A control sentence the host recognises is
# never compiled.

# Which kind of channel, where the intent has one. 'both' is a real answer:
# "read everything" is one question and two calls.
KINDS = ('analog', 'digital', 'both', 'none')

# Intent to tool, for the pairs where it is unambiguous. 'words' and 'control'
# map to nothing on purpose: naming a tool for them is how a request for a
# description turned into a channel table.
TOOL = {
    'map': 'board_info',
    'power': 'afe_power',
    'devices': 'devices',
    'link': 'link_diagnose',
}
READ = {
    'analog': 'analog_read',
    'digital': 'digital_read',
    'both': 'analog_read and digital_read',
    'none': 'analog_read',
}

# What the hint calls each intent. Separate from INTENTS, which is written to
# be chosen between and reads badly in a sentence: "asking for explain,
# describe, define, compare" was the first version of this line.
SAYS = {
    'map':     'which channels or pins exist',
    'read':    'the present value of channels or pins',
    'power':   'the analog front end switched on or off',
    'devices': 'the nodes on the bus, or one of them selected',
    'link':    'the state of the serial link',
    'words':   'an answer in words',
    'control': 'the host to switch board, model, node or language',
}

ASK = """Classify this operator's question. Do not answer it.

Intents:
%s

Kinds: analog, digital, both, none.

The noun decides, never the verb. "List", "give me", "show" say nothing:
channels, pins, inputs is map; values, readings, measurements is read.

The kind is which channels the question is about, and none when it is about
neither. A question naming a pin is the kind that pin is.

JSON only: {"intent": "...", "kind": "...", "why": "a few words"}

Question: %s"""

SCHEMA = {
    'type': 'object',
    'properties': {
        'intent': {'type': 'string', 'enum': sorted(INTENTS)},
        'kind': {'type': 'string', 'enum': list(KINDS)},
        'why': {'type': 'string'},
    },
    'required': ['intent', 'kind'],
}


def tool_for(intent, kind):
    """Which tool answers this intent, or None where naming one would lie."""
    if intent == 'read':
        return READ.get(kind, READ['none'])
    return TOOL.get(intent)


def parse(reply):
    """(intent, kind, why), or (None, None, reason) when it cannot be read."""
    try:
        got = json.loads(reply)
        intent = str(got.get('intent') or '').strip().lower()
        kind = str(got.get('kind') or '').strip().lower() or 'none'
        why = str(got.get('why') or '').strip()
    except Exception:                                         # noqa: BLE001
        return None, None, 'not the JSON it was asked for'
    if intent not in INTENTS:
        return None, None, 'no such intent: %r' % (intent or 'nothing at all')
    if kind not in KINDS:
        kind = 'none'
    return intent, kind, why


# What the *other* half of the axis would be, for the two intents that keep
# swapping. Naming the tool to call was not enough on its own: asked for the
# channel map one turn after a reading, the model called board_info and then
# analog_read as well, because the reading was still in the conversation.
# Caught by test_live_model.py --sections sequence, the only place a second
# question is asked without clearing history first.
NOT_THIS = {
    'map':  'This question does not need a reading.',
    'read': 'This question does not need the channel map.',
}


def hint(intent, kind):
    """The one line a compiled intent adds to the turn, or '' for none."""
    if intent is None:
        return ''
    what = SAYS[intent]
    tool = tool_for(intent, kind)
    if tool:
        line = ('\nThe operator is asking for %s - answered by %s.'
                % (what, tool))
        return line + (' ' + NOT_THIS[intent] if intent in NOT_THIS else '')
    # Saying which call is wrong is worth more here than saying nothing:
    # the measured failure was a description answered with a table.
    return ('\nThe operator is asking for %s. This needs no board '
            'call.' % what)


def compile_intent(client, text):
    """(intent, kind, why). (None, None, reason) when it could not be read.

    Asked through `client` itself, with the schema and a small budget
    overridden for that one call, so the model stays exactly as loaded as it
    was: a short prompt and about forty tokens out, and no reload.
    """
    text = (text or '').strip()
    if not text:
        return None, None, 'nothing was asked'
    try:
        if not getattr(client, 'model', None):
            return None, None, 'no model tag to ask'
        catalogue = '\n'.join('  %-8s %s' % (name, INTENTS[name])
                              for name in sorted(INTENTS))
        # The turn's own client, overridden for one call. A second Ollama
        # was tried first and was the wrong shape: ollama keys a loaded
        # runner on num_ctx, so asking for the same tag at a different
        # window unloaded and reloaded the weights once per question.
        # Same client, same window, same resident model - only the schema
        # and the token budget move.
        #
        # think=False: this is a classification against an enum, and
        # measured on tools/pick_tests.py, thinking spent the whole
        # num_predict budget reasoning and returned an empty content.
        message = client.chat([{'role': 'user',
                                'content': ASK % (catalogue, text)}],
                              fmt=SCHEMA, think=False, num_predict=80)
    except Exception as exc:                                  # noqa: BLE001
        return None, None, 'could not compile: %s' % exc
    return parse((message.get('content') or '').strip())
