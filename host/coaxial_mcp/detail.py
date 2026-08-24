"""How much documentation a reader gets, decided from who is reading.

Every tool description is re-sent every turn. Claude over MCP reads it out of
a window of hundreds of thousands of tokens; gemma4:12b pays for the same text
out of 8192 shared with the conversation and the readings. Writing for the
smaller reader shortchanges the larger one, so the text comes from code: one
spec carries both forms.

    detail.resolve('auto', model='gemma4:12b')         -> 'terse'
    detail.resolve('auto', model='minimax-m3:cloud')   -> 'full'
    detail.resolve('full', model='gemma4:12b')         -> 'full'   (operator said)

`auto` reads the model tag - the one thing every entry point already has. A
parameter count decides on the count, a cloud tag is not short of room, and an
unrecognised tag is assumed small: the local daemon is where unnamed tags
live, and being wrong that way costs a sentence, not a session.
`COAXIAL_DETAIL` overrides for a whole machine.

Deliberately NOT gated on this: the behavioural hints in debug.py. Each exists
because a small model needed telling, so trimming them for small models would
delete them where they earn their place. This shortens documentation, not
instructions.
"""
import os
import re

TERSE = 'terse'
FULL = 'full'
AUTO = 'auto'
LEVELS = (AUTO, TERSE, FULL)

# The environment's way to say it once for every entry point.
ENV = 'COAXIAL_DETAIL'

# Parameters in billions at or above which a reader gets the full text. Set
# between the largest tag run locally here (14B) and the frontier models
# reached over MCP. A judgement about who has room to read, not a benchmark.
FULL_MODEL_B = 30.0

# A parameter count in an ollama tag: gemma4:12b, qwen2.5:14b, llama3.1:8b,
# and the odd 1.5b or 70b. Anchored to the end of a component so a tag like
# `qwen3.6:latest` does not match the 3.6 in its name.
_SIZE = re.compile(r'(?:^|[:\-_])(\d+(?:\.\d+)?)b(?:$|[:\-_])', re.I)


def parse_billions(tag):
    """Parameter count from a tag, or None when it does not say. The last
    match, not the first: `llama3.1:8b` names a version before a size."""
    if not tag:
        return None
    found = _SIZE.findall(str(tag))
    if not found:
        return None
    try:
        return float(found[-1])
    except ValueError:
        return None


def is_cloud(tag):
    """Ollama's marker for a tag that runs on their hardware. Duplicated from
    client.py so coaxial_mcp needs nothing from coaxial_ollama."""
    return bool(tag) and str(tag).split(':')[-1] == 'cloud'


def for_model(tag):
    """The level a model gets when nobody said - see the module docstring."""
    if is_cloud(tag):
        return FULL
    size = parse_billions(tag)
    if size is None:
        return TERSE
    return FULL if size >= FULL_MODEL_B else TERSE


def resolve(level=AUTO, model=None, default=FULL):
    """One level, from the caller, the environment, then the model.

    An explicit terse/full wins, then COAXIAL_DETAIL, then the tag, then
    `default` - FULL, since a caller with no model is the MCP server and its
    reader is not the one short of room. An unrecognised value is ignored:
    this decides how long a sentence is, and a typo in an environment
    variable should not refuse a bench question.
    """
    if level in (TERSE, FULL):
        return level
    from_env = (os.environ.get(ENV) or '').strip().lower()
    if from_env in (TERSE, FULL):
        return from_env
    if model:
        return for_model(model)
    return default if default in (TERSE, FULL) else FULL


def text(spec, level, key='description'):
    """The description this level asks for, falling back to the full one. A
    spec with no terse form is not a mistake - most descriptions here are
    already one line, and a second copy would be two things to keep in step.
    """
    if level == TERSE:
        short = spec.get(key + '_terse')
        if short:
            return short
    return spec.get(key, '')


def _properties(schema, level):
    """Property descriptions are documentation too, and there are more of them
    than tools. Terse drops them, except where the description is the only
    place an allowed spelling appears - dropping that is deleting, not
    shortening."""
    if level != TERSE:
        return schema
    properties = schema.get('properties')
    if not isinstance(properties, dict):
        return schema
    trimmed = {}
    changed = False
    for name, prop in properties.items():
        if isinstance(prop, dict) and 'description' in prop \
                and not _names_values(prop['description']):
            prop = {k: v for k, v in prop.items() if k != 'description'}
            changed = True
        trimmed[name] = prop
    if not changed:
        return schema
    return dict(schema, properties=trimmed)


def _names_values(description):
    """Whether a description carries the values themselves rather than prose
    about them - `README|CLAUDE|...`, `Pin as PORT+NUMBER, e.g. B2`."""
    return '|' in description or 'e.g.' in description


def apply(specs, level):
    """A tool list at one level. Copied, never edited: TOOLS is shared by
    every session in the process, and one terse request must not shorten the
    server for everybody after it."""
    level = level if level in (TERSE, FULL) else FULL
    out = []
    for spec in specs:
        out.append({'name': spec['name'],
                    'description': text(spec, level),
                    'inputSchema': _properties(spec['inputSchema'], level)})
    return out
