"""A small Ollama chat client, stdlib only.

No `ollama` package, no `openai` shim, no `requests`. The same reasoning as
testline/pdfwriter.py: this runs on whatever PC is bolted to the bench, and the
whole API surface needed here is one POST to /api/chat plus one GET to /api/tags
to find out whether the model is actually pulled. That is fifty lines of urllib,
against a dependency to keep working through the next OS image.

Four things are deliberate:

  * `keep_alive` is sent on every turn. Ollama caches the KV state of the
    prompt prefix it has already processed, which is what makes turn nine of a
    bench session as quick as turn two - and it throws that cache away the
    moment the model unloads, five minutes after the last request by default.
    A bench session has long gaps in it: you read a number, you move a probe,
    you think. Sending keep_alive on each request restarts that timer, so the
    pause between two questions cannot cost an 8 GB reload and a reprocessed
    context. It buys nothing on a busy machine and costs nothing on an idle
    one, except the VRAM being held - which is what `--keep-alive 0` gives
    back.

  * The daemon is local, and that is enforced rather than assumed. Ollama will
    happily proxy a `:cloud` tag to somebody else's GPU, which on a bench means
    the register dumps, the pin names and whatever a plan says about unreleased
    hardware leave the building over TLS to be logged at the other end. Nothing
    here needs that, so a cloud tag or a non-loopback host raises instead of
    quietly working. `remote_ok=True` is the way to mean it on purpose.

  * `format` is not set by the runner, and that is a decision rather than an
    omission. Ollama's `format='json'` constrains the *content* field, which is
    the one part of a reply this bench does not parse: every number that
    reaches a verdict arrives as an argument to the `report` tool, against a
    JSON Schema the daemon already enforces, and `plan.Limit` judges it in
    Python. Turning on json mode as well would either do nothing or compete
    with the tool path - a model told to answer in JSON tends to describe a
    tool call in the content instead of making one. It is here as a parameter
    because a caller outside the runner may genuinely want machine-readable
    prose, and then it should be one argument rather than a fork of this file.

  * `stream` is off. Streaming buys a nicer terminal and costs the guarantee
    that a tool call arrives whole; the runner needs the whole message before it
    can dispatch anything, so there is nothing to gain.
  * `temperature` defaults to 0. A test runner that takes a different path
    through the same plan on every invocation cannot be audited. The model still
    is not deterministic across versions or context lengths - but nothing is
    gained by adding sampling noise on top of that.

Running out of memory is handled here rather than reported. A bench machine
shares its card with a desktop, and the model is the largest thing on it: the
failure is not rare and it is not the operator's mistake, so an allocation
that loses gets the card cleared of anything else resident, this model's own
caches dropped, and - only if that was not enough - a smaller context window,
in that order. Each rung is recorded in `notes` for the caller to show;
nothing here prints. See `_make_room`.

Model output is never trusted here. This module returns whatever the model said;
deciding what of it is allowed to touch the board is tools.py's problem, and
deciding what counts as a pass is plan.py's.
"""
import json
import time
import urllib.error
import urllib.parse
import urllib.request

LOOPBACK = ('localhost', '127.0.0.1', '::1')

# How often to re-ask after ollama's own model runner has crashed, and how
# long to wait before the first retry (doubling, then tripling). Two is
# enough for every crash measured here - the runner is respawned by the
# time the first retry lands - and small enough that a machine genuinely
# out of memory fails in seconds rather than looping.
RUNNER_RETRIES = 2
RUNNER_RETRY_WAIT = 1.5

# What ollama says when llama-server died under it. The text is the
# daemon's, quoted here because the HTTP status alone does not
# distinguish it from a bad request, and a bad request must not be
# retried.
_RUNNER_CRASH = ('model runner has unexpectedly stopped',
                 'llama runner process has terminated')

# What it says when the machine, rather than the request, is the problem.
# Distinct from the crash above and handled differently: a crashed runner
# comes back on its own and the fix is to ask again, while a card that is
# full stays full - asking again just collects the same error a second and
# third time. Something has to be given back first.
#
# The wording is the daemon's, the driver's and llama.cpp's, in that order,
# which is why the list is longer than one line: the same condition surfaces
# as a CUDA allocation failure, as a C++ bad_alloc from the host side, and
# as ollama's own prose depending on which allocation lost.
_OUT_OF_MEMORY = ('out of memory', 'cudamalloc failed', 'std::bad_alloc',
                  'bad_alloc', 'failed to allocate', 'unable to allocate',
                  'cannot allocate memory', 'not enough memory',
                  'insufficient memory', 'no available memory')

# The floor a context window is not shrunk below when making room. Under
# this the tool schemas alone stop fitting, and a model that cannot be told
# what its tools are is not a smaller session - it is a different, worse one
# that answers from memory instead of measuring.
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
            # A cap on generated tokens. Nothing on this bench needs an essay,
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
        # What this client had to do to the machine to keep answering: VRAM
        # taken back off another model, a context window shrunk. Recorded
        # rather than printed - a library that writes to somebody's terminal
        # is a library that cannot be used quietly - and drained by the
        # caller, which in debug.py's case puts them in the same trace the
        # tool results go to. Empty on a healthy machine, which is the point:
        # a session that answered without any of this leaves no line saying
        # so.
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

        The same job board_prompt.ps1 does before it loads anything, here for
        the paths that never go through it - `dbg.py`, the runner, a plan on
        a machine somebody left an `ollama run` open on. Those weights sit
        there until their own keep_alive expires, and the next load then asks
        a card that is already full.

        Not done at startup, on purpose: a model already resident is one this
        machine can run and possibly the one the operator is talking to in
        another window, and evicting it to load a second copy is how a 16 GB
        card ends up asked for two sets of weights. Here it is different -
        the allocation has already failed, so something is going to give
        either way, and it should be the model nobody in this process is
        using.
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
        """Drop this model too, and with it everything cached around it.

        The KV cache, the prompt cache llama-server writes beside it and
        whatever the runner had fragmented on the card all go with the
        weights. Reloading costs a wait; carrying on against a heap that has
        already refused an allocation costs the session.
        """
        try:
            self.unload()
            return True
        except OllamaError:
            return False

    def _shrink_context(self):
        """Halve the window, once, down to the floor. Returns what it became,
        or 0 when there is nothing left to give.

        Last resort and deliberately blunt: by the time this runs, the card
        has been cleared of everything else and the model reloaded, and the
        allocation still did not fit. The KV cache is the largest thing left
        that this side controls.
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
        left, and then the original error is what the caller sees.

        Ordered by what it costs to be wrong about it. Freeing another
        model's VRAM costs that model a reload and nothing else. Flushing
        this one costs this session a reload. Shrinking the window costs
        every turn after it, quietly, which is why it is last and why it
        says so.
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

        Measured repeatedly on this bench: llama-server dies mid-session
        with `std::bad_alloc` while saving its own prompt cache, and ollama
        answers 500 `model runner has unexpectedly stopped, this may be due
        to resource limitations`. The daemon respawns the runner on the very
        next request, so the recovery was always just "ask again" - which
        until now the operator had to do by hand, retyping a question that
        had already been answered everywhere except in the reply.

        Silent on purpose: a retry that worked is not news, and the token
        counters below only count replies that actually arrived. A retry
        that does not work raises the original error, so a genuinely
        out-of-memory machine still fails loudly rather than looping.
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

    def chat(self, messages, tools=None):
        """One turn. Returns the assistant message dict, verbatim."""
        payload = {'model': self.model, 'messages': messages,
                   'stream': False, 'options': self.options}
        if self.keep_alive is not None:
            payload['keep_alive'] = self.keep_alive
        if self.fmt is not None:
            payload['format'] = self.fmt
        if tools:
            payload['tools'] = tools
        if self.think is not None:
            payload['think'] = self.think

        try:
            reply = self._chat_once(payload)
        except OllamaError as exc:
            # Models that cannot think refuse the field rather than ignoring it.
            # Drop it once and remember, instead of making every caller know
            # which tags reason and which do not.
            if self.think is None or 'think' not in str(exc).lower():
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

        An empty message list is Ollama's documented way to say "load this and
        do nothing" - no generation, no tokens counted, and it returns once the
        weights are resident. Worth a call before the first question so the
        8 GB wait lands somewhere visible instead of inside it.

        `options` goes with it, and that is not decoration. num_ctx sizes the KV
        cache, so a preload without it asks for the model's own default context
        - which for llama3.1 is 128k and 7 GB of buffer, and fails outright on
        this machine. Worse when it succeeds: the model would be resident at one
        context size and the first real question at another, so the daemon
        reloads and the preload has bought a wait rather than saved one.
        """
        self._post('/api/chat', {'model': self.model, 'messages': [],
                                 'options': self.options,
                                 'keep_alive': self.keep_alive})

    def unload(self):
        """Hand the model's VRAM back at once, whatever keep_alive was.

        Same empty-messages trick as preload(), with keep_alive=0 instead:
        the daemon evicts the moment this reply lands rather than waiting out
        the 30 minutes a prompt loop holds it for. Measured on this bench:
        a session left running unattended held 9.69 GB resident for another
        27 minutes at 1% utilisation - a card the desktop needed back, doing
        nothing for anyone. Call this on the way out of anything that set a
        long keep_alive to survive a loop, not after a one-shot question,
        which already asked for a short hold on purpose.
        """
        self._post('/api/chat', {'model': self.model, 'messages': [],
                                 'options': self.options, 'keep_alive': 0})
