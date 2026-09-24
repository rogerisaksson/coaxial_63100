"""A channel or pin named loosely - phase A, ch5, PE15 - resolved to the board's own."""
import re

from . import render
from coaxial_mcp.schema import _names


def _key(text):
    """A channel name with an author's punctuation removed."""
    return re.sub(r'[^a-z0-9]', '', str(text).strip().lower())


# The same three phases under the other convention: U/V/W and A/B/C both appear
# in the same datasheets, so `phase_a` is a spelling, not a mistake.
PHASE_ALIASES = {
    'phasea': 'phaseu', 'phaseb': 'phasev', 'phasec': 'phasew',
    'a': 'phaseu', 'b': 'phasev', 'c': 'phasew',
    'u': 'phaseu', 'v': 'phasev', 'w': 'phasew',
}

# A channel by what it measures rather than what it is called.
SIGNAL_ALIASES = {
    'bus': 'dcbus', 'vbus': 'dcbus', 'dc': 'dcbus', 'dclink': 'dcbus',
    'link': 'dcbus', 'busvoltage': 'dcbus', 'voltage': 'dcbus',
    'temp': 'ntc', 'temperature': 'ntc', 'thermistor': 'ntc',
    'thermal': 'ntc', 'degc': 'ntc',
}


def _alias(key, by_name):
    """The name this board knows, for a name somebody else's board uses."""
    target = PHASE_ALIASES.get(key) or SIGNAL_ALIASES.get(key)
    if target and target in by_name:
        return target
    # Not a name this board knows and not an alias either, but it may still
    # single one out - `bus` is inside `dcbus` and inside nothing else.
    found = _matches(key, by_name)
    if len(found) == 1:
        return found[0]
    return key


def _words(text):
    """A name split into the words it was built from."""
    spaced = re.sub(r'(?<=[a-z0-9])(?=[A-Z])|(?<=[A-Z])(?=[A-Z][a-z])',
                    ' ', str(text))
    return [w for w in re.split(r'[^A-Za-z0-9]+', spaced.lower()) if w]


def _matches(key, by_name):
    """Every channel this key could mean, by prefix or containment."""
    if not key:
        return []
    return [name for name in sorted(by_name)
            if name.startswith(key) or key.startswith(name) or key in name]


def _index_of(text, by_name, notes):
    """One requested channel as an index: a number, a name, an alias, a
    spelling of one, or the words it was made of - with what it took to
    get there on `notes`.
    """
    key = _alias(_key(text), by_name)
    if text.isdigit():
        return int(text)
    if key in by_name:
        _note(notes, text, key)
        return by_name[key]
    found = _matches(key, by_name)
    if not found:
        # Not a name, a spelling of one or a substring of one - so try the
        # words it is made of.
        words = _words(text)
        phased = any(w.startswith('phase') for w in words)
        found = sorted({_alias(w, by_name) for w in words
                        if len(w) > 1 or phased} & set(by_name))
    if len(found) > 1:
        raise ValueError('channel %r could be %s - say which'
                         % (text, ' or '.join(found)))
    if not found:
        raise ValueError('unknown channel %r; names are %s'
                         % (text, ','.join(sorted(by_name))))
    _note(notes, text, found[0])
    return by_name[found[0]]


def _note(notes, text, key):
    """Say how a spelling was read, when it was read as something else."""
    if notes is not None and key != _key(text):
        notes.append('%s read as %s' % (text, key))


def _resolve(session, wanted, notes=None):
    """Turn ['4', 'DC bus'] into channel indices."""
    wanted = _names(wanted)
    _, _, channels = session.info()

    # The schema says "omit for all", so a model that wants everything writes
    # ch=['all'] instead - which is the same request in the words the schema
    # used.
    if len(wanted) == 1 and _key(wanted[0]) in ('all', 'every', 'everything'):
        return list(range(len(channels)))
    by_name = {_key(render.short(c['signal'], c['index'])): c['index']
               for c in channels}
    # `ch3` is what an unnamed channel is called, and it stayed addressable by
    # that after PB1 and PC1 were given real names - a caller counting channels
    # should not stop working because somebody named one.
    for channel in channels:
        by_name.setdefault('ch%d' % channel['index'], channel['index'])

    indices = []
    for item in wanted:
        text = str(item).strip()
        index = _index_of(text, by_name, notes)
        if not 0 <= index < len(channels):
            raise ValueError('channel %d out of range 0..%d'
                             % (index, len(channels) - 1))
        indices.append(index)
    return indices


def _split_pin(text):
    text = str(text).strip().upper()
    if len(text) < 2 or not text[0].isalpha() or not text[1:].isdigit():
        raise ValueError('pin %r should look like B2 or E15' % (text,))
    return text[0], int(text[1:])
