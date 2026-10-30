"""Her prompt in the operator's own words: the local chat model picks one row of her catalog.

Any language; the catalog `machine.texts.says` - errands, stunts, behaviours, gaits, takes, the page's
programs, every one generated from the code that plays it -, the answer held to them by a JSON
Schema enum (ollama's structured outputs), null when nothing fits.

    name = gynoid.model()                                          # the model to ask, None with none
    gynoid.warm(name)                                              # loaded before the first prompt
    plan = gynoid.plan('lie down on the floor', 'rest', name)      # {'thought', 'cmds', 'tour'}
    gynoid.release(name)                                           # its memory back now
"""
import json
import urllib.request

HOST = 'http://127.0.0.1:11434'
#: The page's own models, the first pulled asked, else the chat's default (`model`).
MODELS = ('gemma4:e4b',)
#: The warm load and every ask alike - another num_ctx reloads the weights; the prompt 963 tokens.
#: A page killed before `release` frees its model after KEEP; the prompt bar's / warms it anew.
NUM_CTX, KEEP = 4096, '5m'


def _names(path, timeout):
    with urllib.request.urlopen(HOST + path, timeout=timeout) as reply:
        return [m.get('name', '') for m in json.loads(reply.read().decode('utf-8')).get('models', [])]


def model(timeout=0.5):
    """The model to ask: the first of MODELS pulled, else the chat's own default; None with neither.
    One model loaded at a time here (OLLAMA_MAX_LOADED_MODELS 1): the page's evicts the chat's."""
    from coaxial_ollama.__main__ import DEFAULT_MODEL
    try:
        pulled = _names('/api/tags', timeout)
    except (OSError, ValueError):
        return None
    return next((m for m in MODELS + (DEFAULT_MODEL,) if m in pulled), None)


def warm(name):
    """`name` loaded and held KEEP, before a prompt waits on its load."""
    from coaxial_ollama.client import Ollama
    Ollama(name, num_ctx=NUM_CTX, keep_alive=KEEP).preload()


def release(name, timeout=5.0):
    """`name`'s memory handed back now, whatever KEEP was."""
    from coaxial_ollama.client import Ollama
    Ollama(name, num_ctx=NUM_CTX, timeout=timeout).unload()


def choices(rows=None):
    """{op: its row} a prompt may choose: every row of her catalog but the loop's raw words."""
    from machine.texts import says
    return {r['op']: r for r in (says.catalog() if rows is None else rows) if r['kind'] != 'word'}


def command(row):
    """What the loop or the page hears for `row`: a take's rows as the bench says them (`takes.told`),
    else its catalog command."""
    from machine.texts import takes
    return takes.told(row['op']) if row['kind'] == 'take' else dict(row['command'])


def plan(text, doing, name, rows=None, timeout=60.0):
    """The page's plan {'thought', 'cmds', 'tour'} of the operator's `text`, her `doing` now, as the
    model `name` chooses among `choices`; null chosen, nothing moves."""
    from coaxial_ollama.client import Ollama
    rows = choices(rows)
    # null in the op's enum, the sources' own words a line each; a fits flag before the op, the kinds
    # beside the lines, the request put in English first no better. Takes in one list with her own rows
    # stood in for them ('go forward' running_forward, 'stand up from the chair' sit_to_stand): named after.
    schema = {'type': 'object', 'properties': {'op': {'enum': list(rows) + [None]}}, 'required': ['op']}
    own = '; '.join('%s: %s' % (op, r['said']) for op, r in rows.items() if r['kind'] != 'take')
    system = ('You map what an operator tells a humanoid robot, in any language, to exactly one of her '
              'operations. A request for anything she cannot do with her body - cooking, a question, '
              'chatter - is null. Her operations: ' + own + ". Mocap takes, a person's recorded motion "
              'she replays, only when the operator asks for that take: '
              + ', '.join(op for op, r in rows.items() if r['kind'] == 'take') + '. Answer JSON {"op": ...}.')
    reply = Ollama(name, fmt=schema, timeout=timeout, num_ctx=NUM_CTX, think=False, keep_alive=KEEP).chat(
        [{'role': 'system', 'content': system}, {'role': 'user', 'content': '%s (she is now: %s)' % (text, doing)}])
    row = rows.get(json.loads(reply.get('content') or '{}').get('op'))
    if row is None:
        return {'thought': 'Nothing of mine fits that.', 'cmds': [], 'tour': False}
    cmd = command(row)
    tour = bool(cmd.pop('tour', False))
    return {'thought': row['said'], 'cmds': [cmd] if cmd else [], 'tour': tour}
