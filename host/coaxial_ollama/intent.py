"""Compile the operator's sentence into a plan, before the model sees it."""
import json

from .client import FAULTS

# What an operator can be asking for at this bench.
INTENTS = {
    'map':     'what exists on the board - channels, pins, or which subsystems'
               ' it is made of. No values',
    'read':    'the present value of one or more channels or pins',
    'power':   'turn the analog front end on or off',
    'devices': 'the other nodes: list them, or start talking to one by name',
    'link':    'the link is failing - nothing answers, and why not',
    'words':   'explain, describe, define, compare - an answer in words',
    'control': 'switch which board, model or language the host itself uses',
    'orient':  'how the board is turned or oriented - a picture, not numbers',
}

# Measured against gemma4:12b, 12 questions, ~2.75 s each.

# Which kind of channel, where the intent has one.
KINDS = ('analog', 'digital', 'imu', 'angle', 'subsystems', 'parts',
         'both', 'none')

# Intent to tool, for the pairs where it is unambiguous.
TOOL = {
    'map': 'board_info',
    'power': 'afe_power',
    'devices': 'devices',
    'link': 'link_diagnose',
    'orient': 'orientation',
}
READ = {
    'analog': 'analog_read',
    'digital': 'digital_read',
    'imu': 'imu',
    'both': 'analog_read and digital_read',
    'none': 'analog_read',
}

# What the hint calls each intent.
SAYS = {
    'map':     'what exists on the board',
    'read':    'the present value of channels or pins',
    'power':   'the analog front end switched on or off',
    'devices': 'the nodes on the bus, or one of them selected',
    'link':    'the state of the serial link',
    'words':   'an answer in words',
    'control': 'the host to switch board, model, node or language',
    'orient':  'how the board is turned',
}

ASK = """Classify this operator's question. Do not answer it.

Intents:
%s

Kinds: analog, digital, imu, angle, subsystems, parts, both, none.

The noun decides, never the verb. "List", "give me", "show" say nothing:
channels, pins, inputs is map; values, readings, measurements is read.

The text may arrive with its diacritics stripped by a console that cannot
carry them, so read Swedish both ways: matvarde = matvarden = matvardena =
measurement, varde = varden = vardena = value, kanal = kanaler = channel.

The kind is which channels the question is about, and none when it is about
neither. A question naming a pin is the kind that pin is. A question naming
the IMU, or an accelerometer, gyro or magnetometer, is the imu kind - those
are not the board's ADC channels. A question about what the board is made
of, what it can do or which subsystems it has is the subsystems kind. A
question about what is fitted on it - which components, which parts, what is
mounted, bestyckning, komponenter - is the parts kind. A question naming
the angle sensor, the shaft angle, the rotor position, vinkel or
vinkelgivare is the angle kind - that is the A1335 on SPI4, and it is
neither the IMU nor an ADC channel.

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


def plan(intent, kind):
    """The calls the host makes itself, as ((name, args), ...)."""
    if intent == 'map':
        section = (kind if kind in ('analog', 'digital', 'subsystems',
                                    'parts')
                   else 'all')
        return (('board_info', {'kind': section}),)
    if intent == 'read':
        analog = ('analog_read', {})
        digital = ('digital_read', {})
        return {'analog': (analog,), 'digital': (digital,),
                'imu': (('imu', {'op': 'read'}),),
                'angle': (('angle', {'op': 'read'}),),
                'both': (analog, digital)}.get(kind, (analog,))
    if intent == 'orient':
        return (('orientation', {'op': 'once'}),)
    if intent == 'power':
        return ()                 # on or off is in the sentence, not the kind
    if intent == 'link':
        return (('link_diagnose', {}),)
    return ()


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
    except (ValueError, AttributeError, TypeError):
        return None, None, 'not the JSON it was asked for'
    if intent not in INTENTS:
        return None, None, 'no such intent: %r' % (intent or 'nothing at all')
    if kind not in KINDS:
        kind = 'none'
    return intent, kind, why


def hint(intent, kind):
    """The one line a compiled intent adds to the turn, or '' for none."""
    if intent is None:
        return ''
    what = SAYS[intent]
    tool = None if plan(intent, kind) else tool_for(intent, kind)
    if tool:
        return ('\nThe operator is asking for %s - answered by %s.'
                % (what, tool))
    # Saying which call is wrong is worth more here than saying nothing: the
    # measured failure was a description answered with a table.
    return ('\nThe operator is asking for %s. This needs no board '
            'call.' % what)


def compile_intent(client, text):
    """(intent, kind, why)."""
    text = (text or '').strip()
    if not text:
        return None, None, 'nothing was asked'
    try:
        if not client.model:
            return None, None, 'no model tag to ask'
        catalogue = '\n'.join('  %-8s %s' % (name, INTENTS[name])
                              for name in sorted(INTENTS))
        # The turn's own client, overridden for one call.
        message = client.chat([{'role': 'user',
                                'content': ASK % (catalogue, text)}],
                              fmt=SCHEMA, think=False, num_predict=80)
    except FAULTS as exc:
        return None, None, 'could not compile: %s' % exc
    return parse((message.get('content') or '').strip())
