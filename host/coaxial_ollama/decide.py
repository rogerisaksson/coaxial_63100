"""A decision model's answers: a state and typed questions in, a probability an option out.

    answers = decide('the record ...', {'suite': choice('which suite next?', {'look': '..', ..})})
    answers['suite']['choice'], answers['suite']['probabilities']

Ollama's `/v1/systemone` (0.35.0 on; Cloudflare's Clef 27B and Clef Flash 9B): no generation,
no tools - one forward pass scores every option of every question against the state (text,
JSON; images from 0.35.1). Three kinds: `choice` of 2-26 named options, `noul` true or false,
`score` on an ordered scale. Nothing here imports the chat client.
"""
import json
import urllib.error
import urllib.request

HOST, TIMEOUT_S = 'http://localhost:11434', 120.0
#: The first tag pulled answers. clef:4k is `ollama create clef:4k -f` a Modelfile of
#: `FROM clef:27b-q4_k_m` and `PARAMETER num_ctx 4096` (docs/MODELS.md): at the model's 16384
#: the runner's 5.4 GB compute buffer does not fit beside 11.7 GB of weights on a 16 GB card
#: (`options` and OLLAMA_CONTEXT_LENGTH ignored); clef-flash refuses every request with
#: "non-finite logit" on 0.35.1 (ollama issue 18769, 2026-10-04).
MODELS = ('clef:4k', 'clef-flash:latest')


def choice(instructions, criteria):
    """A question picking one of `criteria` {option: what it means}."""
    return {'type': 'choice', 'instructions': instructions, 'criteria': dict(criteria)}


def noul(instructions, when_true, when_false):
    """A true-or-false question."""
    return {'type': 'noul', 'instructions': instructions,
            'criteria': {'true': when_true, 'false': when_false}}


def score(instructions, levels):
    """A question scored on `levels`, lowest first."""
    return {'type': 'score', 'instructions': instructions, 'criteria': list(levels)}


def decide(state, questions, model=None, host=HOST, images=()):
    """{key: answer} for `questions` {key: question} over `state` (a string or JSON-able): each
    answer the endpoint's - 'choice' and 'probabilities', 'noul', or 'score'. The model the
    first of MODELS pulled unless given."""
    model = model or available(host=host)
    if not model:
        raise RuntimeError('no decision model pulled: ollama pull %s' % MODELS[0])
    payload = {'model': model, 'state': state if isinstance(state, str) else json.dumps(state),
               'questions': dict(questions)}
    if images:
        payload['images'] = list(images)
    request = urllib.request.Request(host + '/v1/systemone',
                                     data=json.dumps(payload).encode('utf-8'),
                                     headers={'Content-Type': 'application/json'}, method='POST')
    try:
        with urllib.request.urlopen(request, timeout=TIMEOUT_S) as reply:
            return json.loads(reply.read().decode('utf-8'))['answers']
    except urllib.error.HTTPError as exc:
        raise RuntimeError('%s: %s' % (exc.code, exc.read().decode('utf-8', 'replace')[:300]))
    except urllib.error.URLError as exc:
        raise RuntimeError('cannot reach ollama at %s (%s)' % (host, exc.reason))


def available(models=MODELS, host=HOST):
    """The first of `models` pulled here; None when none is."""
    try:
        with urllib.request.urlopen(host + '/api/tags', timeout=5.0) as reply:
            tags = {m.get('name', '') for m in
                    json.loads(reply.read().decode('utf-8')).get('models', [])}
    except (urllib.error.URLError, ValueError):
        return None
    return next((m for m in models if m in tags), None)
