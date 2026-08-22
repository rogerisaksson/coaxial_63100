"""A small Ollama chat client, stdlib only.

No `ollama` package, no `openai` shim, no `requests`. The same reasoning as
testline/pdfwriter.py: this runs on whatever PC is bolted to the bench, and the
whole API surface needed here is one POST to /api/chat plus one GET to /api/tags
to find out whether the model is actually pulled. That is fifty lines of urllib,
against a dependency to keep working through the next OS image.

Two things are deliberate:

  * `stream` is off. Streaming buys a nicer terminal and costs the guarantee
    that a tool call arrives whole; the runner needs the whole message before it
    can dispatch anything, so there is nothing to gain.
  * `temperature` defaults to 0. A test runner that takes a different path
    through the same plan on every invocation cannot be audited. The model still
    is not deterministic across versions or context lengths - but nothing is
    gained by adding sampling noise on top of that.

Model output is never trusted here. This module returns whatever the model said;
deciding what of it is allowed to touch the board is tools.py's problem, and
deciding what counts as a pass is plan.py's.
"""
import json
import urllib.error
import urllib.request


class OllamaError(Exception):
    """Ollama was unreachable, refused the request, or has no such model."""


class Ollama:
    def __init__(self, model, host='http://localhost:11434', temperature=0.0,
                 num_ctx=8192, seed=7, timeout=600.0, num_predict=None,
                 think=None):
        self.model = model
        self.host = host.rstrip('/')
        self.options = {'temperature': temperature, 'num_ctx': num_ctx,
                        'seed': seed}
        if num_predict:
            # A cap on generated tokens. Nothing on this bench needs an essay,
            # and an unbounded reasoning model will write one.
            self.options['num_predict'] = num_predict
        self.think = think
        self.timeout = timeout
        self.calls = 0
        self.eval_tokens = 0
        self.prompt_tokens = 0

    def __repr__(self):
        return '<Ollama %s at %s>' % (self.model, self.host)

    # ---- transport ---------------------------------------------------------

    def _post(self, path, payload):
        request = urllib.request.Request(
            self.host + path,
            data=json.dumps(payload).encode('utf-8'),
            headers={'Content-Type': 'application/json'},
            method='POST')
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as reply:
                return json.loads(reply.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', 'replace')[:400]
            raise OllamaError('%s %s: %s' % (path, exc.code, detail)) from exc
        except urllib.error.URLError as exc:
            raise OllamaError(
                'cannot reach ollama at %s (%s). Is `ollama serve` running?'
                % (self.host, exc.reason)) from exc

    def _get(self, path):
        try:
            with urllib.request.urlopen(self.host + path,
                                        timeout=30.0) as reply:
                return json.loads(reply.read().decode('utf-8'))
        except (urllib.error.URLError, urllib.error.HTTPError) as exc:
            raise OllamaError(
                'cannot reach ollama at %s (%s). Is `ollama serve` running?'
                % (self.host, exc)) from exc

    # ---- what the runner uses ---------------------------------------------

    def models(self):
        return [entry['name'] for entry in self._get('/api/tags').get('models', [])]

    def require_model(self):
        """Fail before the board is touched rather than three steps in.

        Tag matching is loose on purpose: `ollama list` shows qwen3:8b, and a
        plan that says qwen3 should not fail over a missing suffix.
        """
        available = self.models()
        if self.model in available:
            return self.model
        stem = self.model.split(':')[0]
        for name in available:
            if name.split(':')[0] == stem:
                return name
        raise OllamaError('model %r is not pulled. Have: %s. Try: ollama pull %s'
                          % (self.model, ', '.join(available) or '(none)',
                             self.model))

    def chat(self, messages, tools=None):
        """One turn. Returns the assistant message dict, verbatim."""
        payload = {'model': self.model, 'messages': messages,
                   'stream': False, 'options': self.options}
        if tools:
            payload['tools'] = tools
        if self.think is not None:
            payload['think'] = self.think

        try:
            reply = self._post('/api/chat', payload)
        except OllamaError as exc:
            # Models that cannot think refuse the field rather than ignoring it.
            # Drop it once and remember, instead of making every caller know
            # which tags reason and which do not.
            if self.think is None or 'think' not in str(exc).lower():
                raise
            self.think = None
            payload.pop('think')
            reply = self._post('/api/chat', payload)
        self.calls += 1
        self.prompt_tokens += reply.get('prompt_eval_count', 0)
        self.eval_tokens += reply.get('eval_count', 0)

        message = reply.get('message')
        if not isinstance(message, dict):
            raise OllamaError('no message in reply: %r' % (reply,))
        message.setdefault('role', 'assistant')
        message.setdefault('content', '')
        return message

    def usage(self):
        return {'calls': self.calls, 'prompt_tokens': self.prompt_tokens,
                'eval_tokens': self.eval_tokens}
