"""A small Ollama chat client, stdlib only.

No `ollama` package, no `openai` shim, no `requests`. The whole surface needed
is one POST to /api/chat and one GET to /api/tags, and fifty lines of urllib
keep working through the next OS image on the bench PC.

Deliberate:

  * `keep_alive` on every turn. Ollama drops the cached prompt prefix when the
    model unloads, five minutes after the last request by default, and a bench
    session has long gaps. Re-arming each turn keeps a pause from costing an
    8 GB reload. `--keep-alive 0` hands it back.
  * Local daemon, enforced. A `:cloud` tag proxies to somebody else's GPU -
    register dumps and unreleased hardware over TLS. `remote_ok=True` opts in.
  * `format` unset by the runner. json mode constrains `content`, the one part
    this loop does not parse - every number reaching a verdict arrives as a
    `report` argument against a daemon-enforced schema - and a model told to
    answer in JSON describes a tool call instead of making one. Callers outside
    the runner may still set it.
  * `stream` off: the runner needs a whole message before it can dispatch.
  * `temperature` 0: a runner taking a different path through the same plan on
    every invocation cannot be audited.

Out-of-memory is handled here rather than reported - the card is shared with a
desktop. See `_make_room`, and `notes` for what it did.

Model output is never trusted here. What may touch the board is tools.py's
problem; what counts as a pass is plan.py's.
"""
import json
import time
import urllib.error
import urllib.parse
import urllib.request

LOOPBACK = ('localhost', '127.0.0.1', '::1')

# How often to re-ask after ollama's runner crashed, and the first backoff.
# Two is enough for every crash Measured: and small enough that a machine
# genuinely out of memory fails in seconds rather than looping.
RUNNER_RETRIES = 2
RUNNER_RETRY_WAIT = 1.5

# What ollama says when llama-server died under it. The text is the
# daemon's, quoted here because the HTTP status alone does not
# distinguish it from a bad request, and a bad request must not be
# retried.
_RUNNER_CRASH = ('model runner has unexpectedly stopped',
                 'llama runner process has terminated')

# When the machine, not the request, is the problem. A crashed runner comes
# back on its own; a full card stays full, so something has to be given back
# first. Three vocabularies - the driver's, llama.cpp's and ollama's - because
# the same condition surfaces differently depending on which allocation lost.
_OUT_OF_MEMORY = ('out of memory', 'cudamalloc failed', 'std::bad_alloc',
                  'bad_alloc', 'failed to allocate', 'unable to allocate',
                  'cannot allocate memory', 'not enough memory',
                  'insufficient memory', 'no available memory')

# The floor a context window is not shrunk below. Under this the tool schemas
# stop fitting, and a model that cannot be told what its tools are answers
# from memory instead of measuring.
MIN_NUM_CTX = 2048


class OllamaError(Exception):
    """Ollama was unreachable, refused the request, or has no such model."""


# What is left to say once every rung of the ladder has been climbed. The
# three levers are the operator's, not this module's: it cannot decide how
# much of the card the desktop is allowed to keep, and it will not pick a
# smaller model on somebody's behalf mid-question.
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
    simply fallen over. Checked before _runner_crashed, because ollama's
    crash text says 'this may be due to resource limitations' and would
    otherwise swallow the case that needs room made for it."""
    text = str(exc).lower()
    return any(marker in text for marker in _OUT_OF_MEMORY)


def is_local(host):
    """True when this URL can only reach a daemon on this machine."""
    parsed = urllib.parse.urlsplit(host)
    return (parsed.hostname or '') in LOOPBACK


def is_cloud(model):
    """Ollama's own marker for a tag it runs on their hardware, not yours."""
    return model.split(':')[-1] == 'cloud'


class Ollama:
    """`/api/chat` over urllib. Refuses cloud tags and non-loopback hosts,
    retries a crashed runner, and climbs a ladder of its own when the
    card is genuinely full."""
    def __init__(self, model, host='http://localhost:11434', temperature=0.0,
                 num_ctx=8192, seed=7, timeout=600.0, num_predict=None,
                 think=None, remote_ok=False, keep_alive='30m',
                 fmt=None, num_gpu=None):
        self.remote_ok = remote_ok
        if not remote_ok:
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
        self.model = model
        self.host = host.rstrip('/')
        self.options = {'temperature': temperature, 'num_ctx': num_ctx,
                        'seed': seed}
        if num_gpu is not None:
            # Layers on the GPU; the rest run on the CPU. capability.py picks
            # this from the size of the card, and it is an ordinary option
            # rather than a Modelfile - which would be a second tag to keep in
            # step with this one.
            self.options['num_gpu'] = num_gpu
        if num_predict:
            # A cap on generated tokens. Nothing  needs an essay,
            # and an unbounded reasoning model will write one.
            self.options['num_predict'] = num_predict
        self.think = think
        # Ollama's own duration syntax: '30m', '1h', 0 to unload at once, -1 to
        # hold forever. Passed through rather than parsed - the daemon is the
        # authority on what it accepts, and a wrong value should fail loudly at
        # the first request rather than quietly here.
        self.keep_alive = keep_alive
        # 'json', or a JSON Schema as a dict for ollama's structured outputs.
        # Named fmt because `format` is a builtin, and passed through untouched
        # for the same reason keep_alive is: the daemon is the authority on what
        # it accepts.
        self.fmt = fmt
        self.timeout = timeout
        self.calls = 0
        self.truncated = False
        self.eval_tokens = 0
        self.prompt_tokens = 0
        # What this client had to do to the machine to keep answering.
        # Recorded, not printed - a library that writes to a terminal cannot
        # be embedded - and drained by the caller. Empty on a healthy
        # machine, which is the point.
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
        """Fail before the board is touched rather than three steps in.

        Tag matching is loose on purpose: `ollama list` shows gemma4:12b, and
        a plan that says gemma4 should not fail over a missing suffix. Cloud
        tags are not candidates unless remote_ok says so.
        """
        available = self.models()
        if not self.remote_ok:
            # A cloud tag is in `ollama list` like any other, and stem matching
            # would resolve a bare 'minimax-m3' straight onto it. Drop them here
            # so the loose match cannot silently pick one.
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
        """What ollama is holding right now, from /api/ps.

        Never raises: this is only ever called while recovering from an error
        that already happened, and a probe that fails there must not become
        the error the operator reads instead of the real one.
        """
        try:
            models = self._get('/api/ps').get('models') or []
        except OllamaError:
            return []
        return [entry for entry in models if isinstance(entry, dict)]

    def free_others(self):
        """Hand back the VRAM held by every model that is not this one.

        What board_prompt.ps1 does before loading, here for the paths that do
        not go through it. Not done at startup, on purpose: a resident model
        may be the one the operator is talking to in another window. By the
        time this runs the allocation has already failed, so something gives
        either way - and it should be the model nobody here is using.
        """
        freed = []
        for entry in self.resident():
            name = entry.get('name') or entry.get('model') or ''
            if not name or name == self.model:
                continue
            try:
                # /api/generate, not /api/chat: an empty prompt with
                # keep_alive 0 is ollama's documented unload, and it takes a
                # model name this client was not built for.
                self._post('/api/generate',
                           {'model': name, 'prompt': '', 'keep_alive': 0})
            except OllamaError:
                continue
            size = entry.get('size_vram') or entry.get('size') or 0
            freed.append('%s (%.1f GB)' % (name, size / float(1 << 30))
                         if size else name)
        return freed

    def flush(self):
        """Drop this model too, and its caches with it.

        Reloading costs a wait; carrying on against a heap that has already
        refused an allocation costs the session.
        """
        try:
            self.unload()
            return True
        except OllamaError:
            return False

    def _shrink_context(self):
        """Halve the window, once, down to the floor. 0 when there is nothing
        left to give.

        Blunt, and last: the card has already been cleared and the model
        reloaded. The KV cache is the largest thing left that this side
        controls.
        """
        try:
            ctx = int(self.options.get('num_ctx') or 0)
        except (TypeError, ValueError):
            return 0
        if ctx <= MIN_NUM_CTX:
            return 0
        self.options['num_ctx'] = max(MIN_NUM_CTX, ctx // 2)
        return self.options['num_ctx']

    def _make_room(self, attempt):
        """One rung of the out-of-memory ladder. False when there are none
        left, and the original error is then what the caller sees.

        Ordered by what it costs to be wrong: another model reloads, then
        this one reloads, then every later turn is quietly smaller - which is
        why the window is last and why it says so.
        """
        if attempt == 0:
            freed = self.free_others()
            reloaded = self.flush()
            if freed:
                self.notes.append(
                    'out of memory: freed %s, reloading' % ', '.join(freed))
            elif reloaded:
                self.notes.append(
                    'out of memory: nothing else was resident, so this model '
                    'was unloaded and its caches with it - reloading')
            else:
                self.notes.append('out of memory: nothing to free')
            return True
        shrunk = self._shrink_context()
        if not shrunk:
            return False
        self.notes.append(
            'out of memory again: context window cut to %d tokens for the '
            'rest of this session' % shrunk)
        return True

    def _chat_once(self, payload):
        """POST /api/chat, retrying a crashed model runner in silence.

        Measured repeatedly: llama-server dies with `std::bad_alloc` while
        saving its prompt cache, ollama answers 500, and the daemon respawns
        it on the next request - so recovery was always "ask again", by hand.
        Silent because a retry that worked is not news; the counters below
        only count replies that arrived. A retry that fails raises the
        original error rather than looping.
        """
        for attempt in range(RUNNER_RETRIES + 1):
            try:
                return self._post('/api/chat', payload)
            except OllamaError as exc:
                if attempt >= RUNNER_RETRIES:
                    raise
                if _out_of_memory(exc):
                    # A full card does not empty itself between two requests,
                    # so this rung is not a wait - it is giving something
                    # back. Out of rungs, the original error stands: a
                    # machine that cannot hold this model at its smallest
                    # window has a problem no retry is going to solve, and
                    # saying so beats looping.
                    if not self._make_room(attempt):
                        raise OllamaError('%s\n%s' % (exc, _NO_ROOM_LEFT))
                    payload['options'] = dict(self.options)
                elif not _runner_crashed(exc):
                    raise
                # The daemon needs a moment to notice its runner is gone and
                # start another; asking again instantly just collects the
                # same 500. Backs off a little further each time.
                time.sleep(RUNNER_RETRY_WAIT * (attempt + 1))

    def chat(self, messages, tools=None, fmt=None, think=None,
             num_predict=None):
        """One turn. Returns the assistant message dict, verbatim.

        fmt/think/num_predict override this client's own for one call. They
        exist so a caller wanting a schema-constrained classification does
        not build a second client to get it: ollama keys a loaded runner on
        num_ctx, so a second client asking for the same tag with a different
        window evicts the first and reloads 7.6 GB - measured, once per
        question, which is what this parameter list replaced.
        """
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
            # Models that cannot think refuse the field rather than ignoring it.
            # Drop it once and remember, instead of making every caller know
            # which tags reason and which do not.
            if want_think is None or 'think' not in str(exc).lower():
                raise
            self.think = None
            payload.pop('think')
            reply = self._chat_once(payload)
        self.calls += 1
        self.prompt_tokens += reply.get('prompt_eval_count', 0)
        self.eval_tokens += reply.get('eval_count', 0)
        # 'length' means num_predict cut the answer off mid-sentence. The
        # caller has to know: a table that stops in the middle of a row looks
        # like a complete answer to everything except a reader who counts rows.
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
        """Load the model and start the keep_alive clock, before it is needed.

        An empty message list is Ollama's documented "load and do nothing".
        Worth a call so the 8 GB wait lands somewhere visible.

        `options` goes with it: num_ctx sizes the KV cache, and a preload
        without it asks for the model default - 128k and 7 GB for llama3.1,
        which fails here. Worse when it succeeds, since the first question
        then reloads at a different size.
        """
        self._post('/api/chat', {'model': self.model, 'messages': [],
                                 'options': self.options,
                                 'keep_alive': self.keep_alive})

    def unload(self):
        """Hand the model's VRAM back at once, whatever keep_alive was.

        preload()'s empty-messages trick with keep_alive=0. Measured: a
        session left running held 9.69 GB for another 27 minutes at 1 %
        utilisation. Call it leaving anything that set a long keep_alive -
        not after a one-shot, which already asked for a short hold.
        """
        self._post('/api/chat', {'model': self.model, 'messages': [],
                                 'options': self.options, 'keep_alive': 0})
