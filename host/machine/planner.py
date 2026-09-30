"""Her get-up's next steps from what the observer says, a local model's or a server's or DEFAULT's.

A local model writes them, a server once LOCAL_TRIES of its plans failed, DEFAULT's when neither
answers.

    steps, who = planner.plan(observer.status(..), local=model, history=tried)
    stream, marks = planner.stream(steps, now)     # `GetUp.begin(stream, marks)`

A plan is a list of STEPS' names; the model answers JSON held to them (SCHEMA), read back and
checked here - a name it made up, or a plan not ending on its feet, falls to DEFAULT's.
"""
import json

from machine import getup

#: The steps a plan is made of, what each wants and what it leaves her in - the words the model
#: reads.
STEPS = {'straighten out': 'lying any way: arms and legs straightened, flat',
         'roll onto back': 'face down or on a side: rolled over the lower side onto her back',
         'roll onto front': 'on her back or a side: rolled over the lower side face down',
         'sit up': 'on her back: sat up on her spine and hips, the heels in',
         'knees under': 'face down: the knees drawn under her hips, the chest down',
         'sit back on heels': 'knees under her: sat back on her heels, the trunk up, the toes '
                              'tucked, the hands to the floor',
         'onto feet': 'on her heels: lifted onto her feet on her hands into a crouch'}
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
{"steps": [...], "why": "..."}, the steps from this list, in order, the last one leaving her on
her feet ("%s"):
%s"""


def _read(reply):
    """The steps of a model's JSON answer, or None if they are not a plan."""
    try:
        steps = json.loads(reply).get('steps')
    except (ValueError, AttributeError):
        return None
    if not steps or any(s not in STEPS for s in steps) or steps[-1] not in UP:
        return None
    return tuple(steps)


def _ask(model, now, history):
    """One model's plan for `now`, None if it gave none."""
    tried = ''.join('\ntried %s: %s' % (', '.join(steps), why) for steps, why in history)
    messages = [{'role': 'system', 'content': PROMPT % ('" or "'.join(UP), '\n'.join(
                    '- %s: %s' % kv for kv in STEPS.items()))},
                {'role': 'user', 'content': 'now: %s%s' % (json.dumps(now), tried)}]
    try:
        return _read(model.chat(messages, fmt=SCHEMA, think=False, num_predict=300)['content'])
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
