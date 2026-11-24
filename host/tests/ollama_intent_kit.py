"""The ollama prompt suite's intent pass: classified first, answered second (`test_intent`)."""
import io
import json
import sys
import types

from ollama_support import Scope, ScriptedModel, call, clientmod, safe_head, simulated, toolmod


def test_intent(r):
    """The intent pass: classify first, answer second."""
    from coaxial.simulated import SimulatedSession as Sim
    from coaxial_mcp import render
    from coaxial_ollama import debug
    from coaxial_ollama import words
    from coaxial_ollama import intent

    j = json.dumps
    for name, reply, want in (
            ('an intent and a kind', j({'intent': 'read', 'kind': 'analog',
                                        'why': 'values'}),
             ('read', 'analog')),
            ('a missing kind is none', j({'intent': 'words'}),
             ('words', 'none')),
            ('a kind outside the enum falls back to none',
             j({'intent': 'map', 'kind': 'sideways'}), ('map', 'none')),
            ('an intent this file has no name for',
             j({'intent': 'frobnicate', 'kind': 'analog'}), (None, None)),
            ('not JSON at all', 'I think they want a reading', (None, None)),
            ('nothing at all', j({}), (None, None)),
    ):
        got, kind, why = intent.parse(reply)
        r.check('parse: %s' % name, (got, kind) == want,
                '%r / %r / %r' % (got, kind, why))

    r.check('every intent has a tool or a reason not to',
            all(intent.tool_for(name, 'none') is not None
                or name in ('words', 'control') for name in intent.INTENTS))
    r.check('every intent has a phrase the hint can say',
            set(intent.SAYS) == set(intent.INTENTS))
    # A console that cannot carry the diacritics is this bench's normal case,
    # not an edge one: measured through board_chat, "matvardena" classified as
    # `map` - "the inputs" - and put the channel map on screen for a question
    # about values.
    r.check('the classifier is told Swedish may arrive folded to ASCII',
            'diacritics' in intent.ASK
            and 'matvardena' in intent.ASK and 'vardena' in intent.ASK)

    r.check('the schema offers exactly the intents this file has',
            intent.SCHEMA['properties']['intent']['enum']
            == sorted(intent.INTENTS))
    r.check('and exactly the kinds',
            intent.SCHEMA['properties']['kind']['enum'] == list(intent.KINDS))

    # Same intent, different kind: the kind picks the tool.
    r.check('read+analog is analog_read',
            intent.tool_for('read', 'analog') == 'analog_read')
    r.check('read+digital is digital_read',
            intent.tool_for('read', 'digital') == 'digital_read')
    r.check('read+both names both calls',
            intent.tool_for('read', 'both') == 'analog_read and digital_read')
    r.check('map is board_info whatever the kind',
            {intent.tool_for('map', k) for k in intent.KINDS} == {'board_info'})
    r.check('words names no tool at all',
            intent.tool_for('words', 'none') is None)

    # The hint is for the intents the loop does not plan.
    r.check('an intent that plans its calls needs no hint at all',
            all(intent.hint(name, kind) == ''
                or 'no board call' in intent.hint(name, kind)
                for name in intent.INTENTS for kind in intent.KINDS
                if intent.plan(name, kind)),
            repr(intent.hint('read', 'analog')))
    r.check('and one that plans nothing still gets its line',
            'afe_power' in intent.hint('power', 'none'))

    r.check('a map question compiles to the map call',
            intent.plan('map', 'analog') == (('board_info',
                                              {'kind': 'analog'}),))
    r.check('a read compiles to the read for its kind',
            intent.plan('read', 'digital') == (('digital_read', {}),))
    r.check('warmth reads the thermal observer',
            intent.plan('read', 'thermal') == (('thermal', {'op': 'state'}),))
    r.check('and "both" compiles to both calls, in order',
            [n for n, _ in intent.plan('read', 'both')]
            == ['analog_read', 'digital_read'])
    r.check('words plans nothing - there is no call to make',
            intent.plan('words', 'none') == ())
    r.check('and neither does control',
            intent.plan('control', 'none') == ())
    r.check('every planned call names a tool the toolbox has',
            all(n in words.SETS['read'] or n in words.SETS['code']
                for name in intent.INTENTS for kind in intent.KINDS
                for n, _ in intent.plan(name, kind)),
            repr(sorted({n for name in intent.INTENTS
                         for kind in intent.KINDS
                         for n, _ in intent.plan(name, kind)})))
    r.check('and for words it says no call is needed',
            'no board call' in intent.hint('words', 'none')
            and 'board_info' not in intent.hint('words', 'none'))
    r.check('no intent means no hint at all',
            intent.hint(None, 'none') == '')

    # The persistence rule, pinned.
    class Recorder(clientmod.Model):
        model = 'gemma4:12b'
        options = {'num_ctx': 8192, 'temperature': 0.0}

        def __init__(self):
            self.seen = []

        def chat(self, messages, tools=None, **kw):
            self.seen.append((messages, tools, kw))
            return {'content': json.dumps({'intent': 'read',
                                           'kind': 'analog'})}

    rec = Recorder()
    built = []
    was = clientmod.Ollama

    class Loud(object):
        def __init__(self, *a, **kw):
            built.append((a, kw))
            raise AssertionError('a second client is a model reload')
    clientmod.Ollama = Loud
    try:
        got, kind, _ = intent.compile_intent(rec, 'ge mig de analoga v\u00e4rdena')
    finally:
        clientmod.Ollama = was
    r.check('the compile builds no second client', not built, repr(built))
    r.check("it goes through the turn's own client", len(rec.seen) == 1)
    r.check('and it still classified', (got, kind) == ('read', 'analog'))
    _, tools, kw = rec.seen[0]
    r.check('with the schema, no thinking and a small budget',
            kw.get('fmt') is intent.SCHEMA and kw.get('think') is False
            and 0 < kw.get('num_predict', 0) <= 200, repr(kw))
    r.check('and no tools - it classifies, it does not call',
            not tools)
    r.check('the window is never touched',
            rec.options == {'num_ctx': 8192, 'temperature': 0.0})

    # An unreachable daemon must leave the turn untouched, not raise into it.
    class Dead(clientmod.Model):
        model = 'gemma4:12b'

        def chat(self, *a, **k):
            raise OSError('connection refused')

    real = sys.modules.get('coaxial_ollama.client')
    broken = types.ModuleType('coaxial_ollama.client')

    def explode(*a, **k):
        raise OSError('connection refused')
    setattr(broken, 'Ollama', explode)
    sys.modules['coaxial_ollama.client'] = broken
    try:
        got, kind, why = intent.compile_intent(Dead(), 'read the NTC')
    finally:
        if real is None:
            sys.modules.pop('coaxial_ollama.client', None)
        else:
            sys.modules['coaxial_ollama.client'] = real
    r.check('ollama unreachable compiles to nothing, and says why',
            got is None and 'connection refused' in why, repr(why))
    r.check('an empty question is not sent anywhere',
            intent.compile_intent(Dead(), '   ')[0] is None)

    # The executor.
    class Narrator(clientmod.Model):
        model = 'gemma4:12b'
        options = {'num_ctx': 8192}

        def __init__(self, say='NTC 38.5 C.'):
            self.say, self.seen = say, []

        def chat(self, messages, tools=None, **kw):
            self.seen.append((messages, tools))
            return {'content': self.say}

        def usage(self):
            return {'prompt_tokens': 0, 'eval_tokens': 0}

    def planned(say):
        screen = io.StringIO()
        talk = debug.Chat(Narrator(say), toolmod.Toolbox(Sim(), scope=Scope()),
                          tools='code', out=screen, session_language='Swedish')
        talk.compile_intent = True
        was = intent.compile_intent
        intent.compile_intent = lambda client, text: ('read', 'analog', '')
        try:
            answer = talk.ask('ge mig de analoga v\u00e4rdena')
        finally:
            intent.compile_intent = was
        return talk, screen.getvalue(), answer

    talk, screen, answer = planned('NTC l\u00e4ser 38,5 C.')
    ran = [n for n, _ in talk.toolbox.log]
    r.check('a planned question makes its own calls',
            ran == ['analog_read'], ', '.join(ran) or 'none')
    r.check('and the model is offered no tools on that turn',
            all(tools in (None, [], ()) for _, tools in talk.client.seen),
            repr([t for _, t in talk.client.seen]))
    r.check('it is asked once, not in a loop', len(talk.client.seen) == 1)
    r.check('the rows reach the screen', render.ANALOG_HEAD in screen)
    r.check('and the answer is the sentence, not the rows',
            answer == 'NTC l\u00e4ser 38,5 C.', repr(answer[:40]))

    # A retype is caught here too: measured on screen, the whole table
    # written out again as prose under itself, comma decimals and all.
    _, _, retyped = planned(
        'De analoga v\u00e4rdena \u00e4r: %s.'
        % ', '.join(c['signal'].replace(' ', '')
                    for c in simulated.CHANNELS))
    r.check('a sentence that just names every channel is silenced',
            retyped == '', repr(retyped[:40]))

    # And a call that failed on the wire never reaches a narrate turn.
    from coaxial.errors import RigError

    class Broken(object):
        simulated = False
        port, baud, unit, bus = 'COM9', 115200, 1, None

        @property
        def board(self):
            raise RigError('cable pulled')

    screen = io.StringIO()
    down = debug.Chat(Narrator(), toolmod.Toolbox(Broken(), scope=Scope()),
                      tools='code', out=screen, session_language='Swedish')
    down.compile_intent = True
    was = intent.compile_intent
    intent.compile_intent = lambda client, text: ('read', 'analog', '')
    try:
        said = down.ask('ge mig de analoga v\u00e4rdena')
    finally:
        intent.compile_intent = was
    r.check('a dead link is reported, not narrated',
            'ERR' in said or 'link is down' in said, repr(said[:60]))
    r.check('and the model was never asked about it',
            not down.client.seen, repr(down.client.seen))

    # And the turn itself: off by default, hint reaches the system message when
    # it is on, and a failed compile changes nothing.
    chat = debug.Chat(ScriptedModel([]),
                      toolmod.Toolbox(Sim(), scope=Scope()),
                      out=io.StringIO())
    r.check('the pass is off unless something turns it on',
            chat.compile_intent is False)
    r.check('off means no second call and no hint',
            chat._compile('ge mig de analoga v\u00e4rdena') == '')

    chat.compile_intent = True
    chat.client = Dead()
    sys.modules['coaxial_ollama.client'] = broken
    try:
        # Through the swap, not just past it: without this the check reaches a
        # running daemon and loads 7.6 GB inside an offline suite.
        blank = chat._compile('ge mig de analoga v\u00e4rdena')
    finally:
        if real is None:
            sys.modules.pop('coaxial_ollama.client', None)
        else:
            sys.modules['coaxial_ollama.client'] = real
    r.check('on, but unreachable, is still no hint', blank == '',
            repr(chat._intent_why))

    # The backstop, offline.
    screen = io.StringIO()
    both = debug.Chat(ScriptedModel([
        call('board_info'), call('analog_read'),
        {'role': 'assistant', 'content': 'Kanalerna listade.'},
    ]), toolmod.Toolbox(Sim(), scope=Scope()), out=screen)
    both.compile_intent = True
    was_compile = intent.compile_intent
    intent.compile_intent = lambda client, text: ('map', 'analog', 'stub')
    try:
        both.ask('ge mig en lista p\u00e5 alla analoga kanalerna')
    finally:
        intent.compile_intent = was_compile
    ran = [name for name, _ in both.toolbox.log]
    r.check('the off-axis call never reaches the board',
            ran.count('analog_read') == 0 and 'board_info' in ran,
            ', '.join(ran) or 'none')
    r.check('and the screen shows the map once, not a reading under it',
            screen.getvalue().count(render.ANALOG_HEAD) == 0,
            safe_head(screen.getvalue()))

    # The same loop with no compiled intent blocks nothing: every failure in
    # intent.py returns None, and None runs the turn without the pass.
    plain = debug.Chat(ScriptedModel([
        call('board_info'), call('analog_read'),
        {'role': 'assistant', 'content': 'Klart.'},
    ]), toolmod.Toolbox(Sim(), scope=Scope()), out=io.StringIO())
    plain.ask('ge mig kanalerna och deras v\u00e4rden')
    did = [name for name, _ in plain.toolbox.log]
    r.check('with no intent compiled, both calls run as before',
            'board_info' in did and 'analog_read' in did,
            ', '.join(did) or 'none')

    chat.compile_intent = False
    chat.intent = intent.hint('read', 'digital')
    sent = chat.trim()
    r.check('the compiled hint reaches the system message',
            'digital_read' in sent[0]['content'], sent[0]['content'][-160:])
    r.check('and the question itself is never rewritten',
            all('asking for' not in (m.get('content') or '')
                for m in sent[1:]))
