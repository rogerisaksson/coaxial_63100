"""Her prompt in the operator's own words: the local chat model picks one row of her catalog.

Any language; the catalog `machine.says` - errands, stunts, behaviours, gaits, takes, the page's
programs, every one generated from the code that plays it -, the answer held to them by a JSON
Schema enum (ollama's structured outputs), null when nothing fits.

    name = gynoid.model()                                          # the model to ask, None with none
    plan = gynoid.plan('lie down on the floor', 'rest', name)      # {'thought', 'cmds', 'tour'}
"""
import json
import urllib.request

HOST = 'http://localhost:11434'


def _names(path, timeout):
    with urllib.request.urlopen(HOST + path, timeout=timeout) as reply:
        return [m.get('name', '') for m in json.loads(reply.read().decode('utf-8')).get('models', [])]


def model(timeout=0.5):
    """The model to ask: the one resident (`/api/ps`) - on an 8 GB card a second evicts it and both
    reload, gemma4:12b's cold load over 120 s here -, else the chat's own default where pulled;
    None with neither."""
    from coaxial_ollama.__main__ import DEFAULT_MODEL
    try:
        resident, pulled = _names('/api/ps', timeout), _names('/api/tags', timeout)
    except (OSError, ValueError):
        return None
    return next(iter(resident), None) or (DEFAULT_MODEL if DEFAULT_MODEL in pulled else None)


def choices(rows=None):
    """{op: its row} a prompt may choose: every row of her catalog but the loop's raw words."""
    from machine import says
    return {r['op']: r for r in (says.catalog() if rows is None else rows) if r['kind'] != 'word'}


def command(row):
    """What the loop or the page hears for `row`: a take's rows as the bench says them (`takes.told`),
    else its catalog command."""
    from machine import takes
    return takes.told(row['op']) if row['kind'] == 'take' else dict(row['command'])


def plan(text, doing, name, rows=None, timeout=60.0):
    """The page's plan {'thought', 'cmds', 'tour'} of the operator's `text`, her `doing` now, as the
    model `name` chooses among `choices`; null chosen, nothing moves."""
    from coaxial_ollama.client import Ollama
    rows = choices(rows)
    # null in the op's enum, the sources' own words a line each: 11 of 13 on gemma4:12b warm, 3.5 s; a
    # fits flag before the op 10, after it 9, the kinds beside the lines no better (live gynoid suite).
    schema = {'type': 'object', 'properties': {'op': {'enum': list(rows) + [None]}}, 'required': ['op']}
    system = ('You map what an operator tells a humanoid robot, in any language, to exactly one of her '
              'operations. A request for anything she cannot do with her body - cooking, a question, '
              'chatter - is null. Operations: '
              + '; '.join('%s: %s' % (op, r['said']) for op, r in rows.items()) + '. Answer JSON {"op": ...}.')
    reply = Ollama(name, fmt=schema, timeout=timeout, num_ctx=8192, think=False).chat(
        [{'role': 'system', 'content': system}, {'role': 'user', 'content': '%s (she is now: %s)' % (text, doing)}])
    row = rows.get(json.loads(reply.get('content') or '{}').get('op'))
    if row is None:
        return {'thought': 'Nothing of mine fits that.', 'cmds': [], 'tour': False}
    cmd = command(row)
    tour = bool(cmd.pop('tour', False))
    return {'thought': row['said'], 'cmds': [cmd] if cmd else [], 'tour': tour}
