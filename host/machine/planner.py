"""Her get-up's next steps from what the observer says, a local model's or a server's or DEFAULT's.

A local model writes them, a server once LOCAL_TRIES of its plans failed, DEFAULT's when neither
answers.

    steps, who = planner.plan(observer.status(..), local=model, history=tried)
    stream, marks = planner.stream(steps, now)     # `GetUp.begin(stream, marks)`

A plan is a list of STEPS' names; the model answers JSON held to them (SCHEMA), read back and
checked here - a name it made up, a step from where the one before does not leave her, or a plan
not ending on her feet, falls to DEFAULT's.
"""
import json

from machine import getup

#: The steps a plan is made of: what each does, how she must lie for it, how it leaves her - the
#: words the model reads and the chain a plan is held to. Her knees-together route only: given a
#: sit-up from her back, gemma4:12b planned it face down, a turtle's (2026-10-01).
STEPS = {'straighten out': ('arms and legs straightened, flat',
                            ('face down', 'on her back', 'on a side', 'sitting', 'kneeling'), None),
         'roll onto front': ('rolled over the lower side face down', ('on her back', 'on a side'),
                             'face down'),
         'knees under': ('the knees drawn under her hips, the chest down', ('face down',),
                         'on her knees'),
         'sit back on heels': ('sat back on her heels, the trunk up over her feet, the toes tucked',
                               ('on her knees',), 'kneeling'),
         'onto feet': ('onto her feet on her hands, into a crouch', ('kneeling',), 'crouched')}
#: Straightened out she lies as she did, sat on her back, kneeling face down.
FLAT = {'sitting': 'on her back', 'kneeling': 'face down'}
#: The steps a plan ends on: her feet under her in the crouch the arrival rises from.
UP = ('onto feet',)

#: The house's own plan by how she lies: onto her front, the kneel, her feet - her knees
#: together.
DEFAULT = {'face down': ('straighten out', 'knees under', 'sit back on heels', 'onto feet'),
           'kneeling': ('onto feet',)}
DEFAULT_ELSE = ('straighten out', 'roll onto front', 'knees under', 'sit back on heels',
                'onto feet')

#: The model's answer: the steps, and why.
SCHEMA = {'type': 'object', 'required': ['steps', 'why'],
          'properties': {'steps': {'type': 'array', 'items': {'enum': list(STEPS)}},
                         'why': {'type': 'string'}}}

#: Its failed plans before the server is asked. Thinking, gemma4:12b answered nothing in 120 s;
#: not, a plan in 3.4 (2026-10-01).
LOCAL_TRIES = 2

PROMPT = """You plan how a fallen humanoid robot gets up. She is a 55 kg woman's build with a
woman's strength; her arms cannot push her torso off the floor from face down. Answer JSON:
{"steps": [...], "why": "..."}: steps from this list, in order, the first from how she lies now,
each from where the one before leaves her, the last leaving her on her feet ("%s"):
%s"""


def _read(reply, lying):
    """The steps of a model's JSON answer for her lying so, or None if they are not a plan."""
    try:
        steps = json.loads(reply).get('steps')
    except (ValueError, AttributeError):
        return None
    if not steps or any(s not in STEPS for s in steps) or steps[-1] not in UP:
        return None
    return tuple(steps) if chained(steps, lying) else None


def chained(steps, lying):
    """Whether each of `steps` begins where the one before leaves her, the first as she lies."""
    at = 'on a side' if 'side' in lying else lying
    for step in steps:
        _does, wants, leaves = STEPS[step]
        if at not in wants:
            return False
        at = leaves or FLAT.get(at, at)
    return True


def _lines():
    """STEPS as the model reads them."""
    return '\n'.join('- %s: %s; from %s, leaves her %s' % (
        name, does, ' or '.join(wants), leaves or 'as she lay')
        for name, (does, wants, leaves) in STEPS.items())


def _ask(model, now, history):
    """One model's plan for `now`, None if it gave none."""
    tried = ''.join('\ntried %s: %s' % (', '.join(steps), why) for steps, why in history)
    messages = [{'role': 'system', 'content': PROMPT % ('" or "'.join(UP), _lines())},
                {'role': 'user', 'content': 'now: %s%s' % (json.dumps(now), tried)}]
    try:
        return _read(model.chat(messages, fmt=SCHEMA, think=False, num_predict=300)['content'],
                     now['lying'])
    except Exception:  # noqa: BLE001 - a model failing must not take her down with it
        return None


def plan(now, local=None, server=None, history=()):
    """(steps, who): the next plan for `now` - the local model's, the server's once LOCAL_TRIES
    of the local model's failed (`history`, [(steps, why)]), else DEFAULT's."""
    if server is not None and len(history) >= LOCAL_TRIES:
        steps = _ask(server, now, history)
        if steps:
            return steps, 'server'
    if local is not None:
        steps = _ask(local, now, history)
        if steps:
            return steps, 'local'
    return DEFAULT.get(now['lying'], DEFAULT_ELSE), 'default'


def stream(steps, now):
    """(stream, marks): the steps' getup stream, and [(the index its step ends at, step)]."""
    out, marks = (), []
    for step in steps:
        out += getup.fragment(step, now)
        marks.append((len(out), step))
    return out, marks
