"""How much documentation a reader gets, decided from who is reading.

Every tool's description and every schema property description is re-sent on
every single turn. That is the cost this package has always been designed
around - coarse tools, short property names, no examples - but "short" was one
number for every reader, and the readers are not alike:

  * Claude, over MCP, has a context window measured in hundreds of thousands
    of tokens and reads the whole description as background it can afford.
  * gemma4:12b, standing at the bench, pays for the same text out of 8192
    tokens shared with the conversation, the readings and the answer.

The wrong fix is to write the descriptions for the smaller reader and let the
larger one work with less than it could have. The fix here is that the text
comes from code: one spec carries both forms, and whoever assembles the tool
list says which one this run wants.

    detail.resolve('auto', model='gemma4:12b')   -> 'terse'
    detail.resolve('auto', model='minimax-m3:cloud')   -> 'full'
    detail.resolve('full', model='gemma4:12b')   -> 'full'    (the operator said)

`auto` is the default everywhere and reads the model's own tag, because that
is the one piece of information every entry point already has. A tag naming
its parameter count decides on the count; a cloud tag is somebody else's
hardware and by definition not short of room; a tag that says nothing
recognisable is assumed small, since the local daemon is where the unnamed
tags live and being wrong in that direction costs a sentence rather than a
session.

`COAXIAL_DETAIL` overrides the lot, for a machine that wants one answer for
every entry point without a flag on each of them.

What is deliberately NOT gated on this: the behavioural hints in debug.py
(call the tool, do not describe it; python not python3; analog_read works with
the AFE either way). Those exist because a small model needed telling, so
trimming them for small models would remove them exactly where they earn their
place. This module shortens documentation, not instructions.
"""
import os
import re

TERSE = 'terse'
FULL = 'full'
AUTO = 'auto'
LEVELS = (AUTO, TERSE, FULL)

# The environment's way to say it once for every entry point.
ENV = 'COAXIAL_DETAIL'

# Parameters, in billions, at or above which a model gets the full text. Set
# between the largest tag this bench actually runs locally (14B, and 12B by
# default - see capability.py) and the frontier models reached over MCP or a
# cloud tag. It is a judgement about who has room to read, not a benchmark
# result: the local tags are the ones paying for context out of 8 GB of VRAM.
FULL_MODEL_B = 30.0

# A parameter count in an ollama tag: gemma4:12b, qwen2.5:14b, llama3.1:8b,
# and the odd 1.5b or 70b. Anchored to the end of a component so a tag like
# `qwen3.6:latest` does not match the 3.6 in its name.
_SIZE = re.compile(r'(?:^|[:\-_])(\d+(?:\.\d+)?)b(?:$|[:\-_])', re.I)


def parse_billions(tag):
    """Parameter count from a tag, or None when it does not say.

    The last match, not the first: `llama3.1:8b` names a version before it
    names a size, and only one of the two is a parameter count.
    """
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
    """Ollama's marker for a tag that runs on their hardware, not yours -
    duplicated from client.py rather than imported, so this module stays
    importable from coaxial_mcp with no coaxial_ollama underneath it."""
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
    """One level, from what the caller said, the environment, and the model.

    Order: an explicit terse/full wins, then COAXIAL_DETAIL, then the model's
    own tag, then `default` - which is FULL, because a caller with no model to
    read is the MCP server, and the reader there is not the one short of room.

    An unrecognised value is not an error. This decides how long a sentence
    is; refusing to answer a bench question over a typo in an environment
    variable would be a worse trade than ignoring it.
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
    """The description this level asks for, falling back to the full one.

    A spec with no terse form written for it is not a mistake: most of the
    descriptions in this package are already one line, and a second, shorter
    line for those would be two things to keep in step for no saving.
    """
    if level == TERSE:
        short = spec.get(key + '_terse')
        if short:
            return short
    return spec.get(key, '')


def _properties(schema, level):
    """Property descriptions are documentation too, and there are more of them
    than there are tools. In terse they go, except where the description is
    the only place a value's allowed spelling appears - a property whose
    description names the choices keeps it, since dropping that would not be
    shortening the documentation but deleting it."""
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
    """Whether a property description carries the values themselves rather
    than prose about them - `README|CLAUDE|...`, `Pin as PORT+NUMBER, e.g.
    B2`. Those are the schema, spelled in the only place it is spelled."""
    return '|' in description or 'e.g.' in description


def apply(specs, level):
    """A tool list at one level. The specs are copied, never edited: TOOLS is
    module state shared by every session in the process, and a server that
    answered one terse request must not have quietly shortened itself for
    everybody after it."""
    level = level if level in (TERSE, FULL) else FULL
    out = []
    for spec in specs:
        out.append({'name': spec['name'],
                    'description': text(spec, level),
                    'inputSchema': _properties(spec['inputSchema'], level)})
    return out
