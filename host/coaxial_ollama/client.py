"""A small Ollama chat client, stdlib only."""
import http.client
import json
import time
import urllib.error
import urllib.parse
import urllib.request

LOOPBACK = ('localhost', '127.0.0.1', '::1')

# How often to re-ask after ollama's runner crashed, and the first backoff.
RUNNER_RETRIES = 2
RUNNER_RETRY_WAIT = 1.5

# What ollama says when llama-server died under it.
_RUNNER_CRASH = ('model runner has unexpectedly stopped',
                 'llama runner process has terminated')

# When the machine, not the request, is the problem.
_OUT_OF_MEMORY = ('out of memory', 'cudamalloc failed', 'std::bad_alloc',
                  'bad_alloc', 'failed to allocate', 'unable to allocate',
                  'cannot allocate memory', 'not enough memory',
                  'insufficient memory', 'no available memory')

# The floor a context window is not shrunk below.
MIN_NUM_CTX = 2048


class OllamaError(Exception):
    """Ollama was unreachable, refused the request, or has no such model."""


#: What a call to the daemon can raise from outside this module: its own
#: refusals, the socket, the HTTP layer under urllib, and a reply that is
#: not the JSON it promised. The guard a caller uses where the model
#: failing must not take the turn down with it.
FAULTS = (OllamaError, OSError, http.client.HTTPException, ValueError,
          KeyError)


# What is left to say once every rung of the ladder has been climbed.
_NO_ROOM_LEFT = (
    'this machine could not fit the model even with the card cleared and '
    'the window at its smallest. Ask for less: --num-ctx smaller, --num-gpu '
    'with fewer layers on the card, or -m with a smaller tag - '
    '`python -m coaxial_ollama.capability` says which one this machine is '
    'actually sized for.')


def _runner_crashed(exc):
    """Whether this error is ollama's runner dying, rather than a request
    it refused - the first is worth asking again, the second never is."""
    text = str(exc).lower()
    return any(marker in text for marker in _RUNNER_CRASH)


def _out_of_memory(exc):
    """Whether the machine ran out of memory, rather than the runner having
    simply fallen over.
    """
    text = str(exc).lower()
    return any(marker in text for marker in _OUT_OF_MEMORY)


def is_local(host):
    """True when this URL can only reach a daemon on this machine."""
    parsed = urllib.parse.urlsplit(host)
    return (parsed.hostname or '') in LOOPBACK


def is_cloud(model):
    """Ollama's own marker for a tag it runs on their hardware, not yours."""
    return model.split(':')[-1] == 'cloud'


def _stay_local(host, model):
    """Refuse a host or a tag that would send the prompt off this machine."""
    if not is_local(host):
        raise OllamaError(
            'host %r is not this machine. The bench runs against a local'
            ' daemon; pass remote_ok=True (--allow-remote) to mean it.'
            % (host,))
    if is_cloud(model):
        raise OllamaError(
            'model %r is an ollama cloud tag: the prompt, and every'
            ' register value in it, would be sent off this machine.'
            ' Pull a local tag, or pass --allow-remote.' % (model,))


def _freed_note(freed, reloaded):
    """What the first retry after an out-of-memory managed to free."""
    if freed:
        return 'out of memory: freed %s, reloading' % ', '.join(freed)
    if reloaded:
        return ('out of memory: nothing else was resident, so this model '
                'was unloaded and its caches with it - reloading')
    return 'out of memory: nothing to free'


class Model:
    """What the runner reads off a model client, whichever it is: the tag
    exactly as the daemon runs it, the options a request carries, whether
    the last answer hit the token cap, and the daemon's notes about the
    last call.
    """
    model = ''
    options = None
    truncated = False
    notes = ()


class Ollama(Model):
    """`/api/chat` over urllib."""
    def __init__(self, model, host='http://localhost:11434', temperature=0.0,
                 num_ctx=8192, seed=7, timeout=600.0, num_predict=None,
                 think=None, remote_ok=False,
                 keep_alive: 'str | int | None' = '30m',
                 fmt=None, num_gpu=None):
        self.remote_ok = remote_ok
        if not remote_ok:
            _stay_local(host, model)
        self.model = model
        self.host = host.rstrip('/')
        self.options = {'temperature': temperature, 'num_ctx': num_ctx,
                        'seed': seed}
        if num_gpu is not None:
            # Layers on the GPU; the rest run on the CPU.
            self.options['num_gpu'] = num_gpu
        if num_predict:
            # A cap on generated tokens.
            self.options['num_predict'] = num_predict
        self.think = think
        # Ollama's own duration syntax: '30m', '1h', 0 to unload at once, -1 to
        # hold forever.
        self.keep_alive = keep_alive
        # 'json', or a JSON Schema as a dict for ollama's structured outputs.
        self.fmt = fmt
        self.timeout = timeout
        self.calls = 0
        self.truncated = False
        self.eval_tokens = 0
        self.prompt_tokens = 0
        # What this client had to do to the machine to keep answering.
        self.notes = []

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
        """Fail before the board is touched rather than three steps in."""
        available = self.models()
        if not self.remote_ok:
            # A cloud tag is in `ollama list` like any other, and stem matching
            # would resolve a bare 'minimax-m3' straight onto it.
            available = [name for name in available if not is_cloud(name)]
        if self.model in available:
            return self.model
        stem = self.model.split(':')[0]
        for name in available:
            if name.split(':')[0] == stem:
                return name
        raise OllamaError('model %r is not pulled. Have: %s. Try: ollama pull %s'
                          % (self.model, ', '.join(available) or '(none)',
                             self.model))

    # ---- making room -------------------------------------------------------

    def resident(self):
        """What ollama is holding right now, from /api/ps."""
        try:
            models = self._get('/api/ps').get('models') or []
        except OllamaError:
            return []
        return [entry for entry in models if isinstance(entry, dict)]

    def free_others(self):
        """Hand back the VRAM held by every model that is not this one."""
        freed = []
        for entry in self.resident():
            name = entry.get('name') or entry.get('model') or ''
            if not name or name == self.model:
                continue
            try:
                # /api/generate, not /api/chat: an empty prompt with keep_alive
                # 0 is ollama's documented unload, and it takes a model name
                # this client was not built for.
                self._post('/api/generate',
                           {'model': name, 'prompt': '', 'keep_alive': 0})
            except OllamaError:
                continue
            size = entry.get('size_vram') or entry.get('size') or 0
            freed.append('%s (%.1f GB)' % (name, size / float(1 << 30))
                         if size else name)
        return freed

    def flush(self):
        """Drop this model too, and its caches with it."""
        try:
            self.unload()
            return True
        except OllamaError:
            return False

    def _shrink_context(self):
        """Halve the window, once, down to the floor."""
        try:
            ctx = int(self.options.get('num_ctx') or 0)
        except (TypeError, ValueError):
            return 0
        if ctx <= MIN_NUM_CTX:
            return 0
        self.options['num_ctx'] = max(MIN_NUM_CTX, ctx // 2)
        return self.options['num_ctx']

    def _make_room(self, attempt):
        """One rung of the out-of-memory ladder."""
        if attempt == 0:
            self.notes.append(_freed_note(self.free_others(), self.flush()))
            return True
        shrunk = self._shrink_context()
        if not shrunk:
            return False
        self.notes.append(
            'out of memory again: context window cut to %d tokens for the '
            'rest of this session' % shrunk)
        return True

    def _recover(self, exc, attempt, payload):
        """Whether a failed POST is worth asking again: a crashed runner the
        daemon respawns, or a full card something can be given back on.
        """
        if attempt >= RUNNER_RETRIES:
            return False
        if not _out_of_memory(exc):
            return _runner_crashed(exc)
        if not self._make_room(attempt):
            raise OllamaError('%s\n%s' % (exc, _NO_ROOM_LEFT))
        payload['options'] = dict(self.options)
        return True

    def _chat_once(self, payload):
        """POST /api/chat, retrying a crashed model runner in silence."""
        for attempt in range(RUNNER_RETRIES + 1):
            try:
                return self._post('/api/chat', payload)
            except OllamaError as exc:
                if not self._recover(exc, attempt, payload):
                    raise
                # The daemon needs a moment to notice its runner is gone and
                # start another; asking again instantly just collects the same
                # 500.
                time.sleep(RUNNER_RETRY_WAIT * (attempt + 1))

    def chat(self, messages, tools=None, fmt=None, think=None,
             num_predict=None):
        """One turn."""
        options = self.options
        if num_predict is not None:
            options = dict(options, num_predict=num_predict)
        payload = {'model': self.model, 'messages': messages,
                   'stream': False, 'options': options}
        if self.keep_alive is not None:
            payload['keep_alive'] = self.keep_alive
        if fmt is not None or self.fmt is not None:
            payload['format'] = self.fmt if fmt is None else fmt
        if tools:
            payload['tools'] = tools
        want_think = self.think if think is None else think
        if want_think is not None:
            payload['think'] = want_think

        try:
            reply = self._chat_once(payload)
        except OllamaError as exc:
            # Models that cannot think refuse the field rather than ignoring
            # it.
            if want_think is None or 'think' not in str(exc).lower():
                raise
            self.think = None
            payload.pop('think')
            reply = self._chat_once(payload)
        if reply is None:
            raise RuntimeError('ollama answered nothing')
        self.calls += 1
        self.prompt_tokens += reply.get('prompt_eval_count', 0)
        self.eval_tokens += reply.get('eval_count', 0)
        # 'length' means num_predict cut the answer off mid-sentence.
        self.truncated = reply.get('done_reason') == 'length'

        message = reply.get('message')
        if not isinstance(message, dict):
            raise OllamaError('no message in reply: %r' % (reply,))
        message.setdefault('role', 'assistant')
        message.setdefault('content', '')
        return message

    def usage(self):
        return {'calls': self.calls, 'prompt_tokens': self.prompt_tokens,
                'eval_tokens': self.eval_tokens}

    def preload(self):
        """Load the model and start the keep_alive clock, before it is
        needed.
        """
        self._post('/api/chat', {'model': self.model, 'messages': [],
                                 'options': self.options,
                                 'keep_alive': self.keep_alive})

    def unload(self):
        """Hand the model's VRAM back at once, whatever keep_alive was."""
        self._post('/api/chat', {'model': self.model, 'messages': [],
                                 'options': self.options, 'keep_alive': 0})
