"""How much of the model's window a prompt may take, and what goes when it
takes more.

Both loops in this package grow a message list and hand it to the same daemon:
`debug.Chat` for a bench question, `runner.Runner` for a plan step. Both used
to bound it by counting messages - `keep=6` in one, `max_turns` in the other -
and a message count is not a bound on anything. Six messages is a small prompt
right up until one of them is a build log, a document section or a hundred
sample rows.

What it is a bound on is `num_ctx`, because that number is what the daemon
allocates a KV cache for. A prompt that runs past it does not come back as a
polite refusal: it comes back as a 500 with `cudaMalloc failed`, or as
llama-server dying of `std::bad_alloc` while saving its own prompt cache and
ollama respawning it underneath the question. That is the failure this module
exists to make structurally impossible from this side - the client's own
recovery ladder is for when the machine is short of memory for reasons that
have nothing to do with how long the conversation got.

The order things are given up in is the same in both loops, and it is ordered
by what is least missed:

  1. tool results, squeezed to their first line, oldest first. A channel
     table's first row is enough to remember that it was read, and the model
     can read it again for the price of one call.
  2. whole messages from the front - the system prompt and the live question
     excepted. This is the conversation actually being forgotten, which is
     why it is second and not first.
  3. the newest message itself, clipped. Only reachable when one message is
     larger than the whole window: a pasted log, an attached file, a step
     brief somebody wrote at length.

Nothing here is silent. Every cut leaves the notice `clip` writes, so a model
reading a stub can tell that it is one, and `/ctx` shows the budget beside the
cost so an operator can see a turn being trimmed rather than wondering why it
stopped growing.
"""
import json

from .sandbox import clip

# What the prompt may take of the window, before the reply cap comes off the
# top of it. Seven tenths rather than all of it, for two reasons: the estimate
# below is an estimate - four characters per token is right for English prose
# and optimistic for a register dump or a Swedish question - and llama-server
# wants working room beside the context it was asked for.
CTX_SHARE = 0.7

# The floor under that share. A window small enough that seven tenths of it is
# less than this is a window nothing useful was going to fit in anyway, and
# clipping to nothing turns "the answer is short" into "there was no
# question", which is the worse failure of the two.
MIN_PROMPT_TOKENS = 512

# What a stubbed tool result keeps, in characters, before the marker that says
# it is a stub.
STUB_CHARS = 80


def approx_tokens(text):
    """Rough but honest: about four characters per token for dense ASCII."""
    return max(1, len(str(text)) // 4)


def budget_for(options):
    """Tokens a prompt may take, from a client's own options.

    0 means "no window to speak of, enforce nothing" - a scripted stand-in, a
    caller that built a client bare. A guess would be worse than none here,
    since guessing low silently deletes a conversation nobody asked to
    shorten.
    """
    options = options or {}
    try:
        ctx = int(options.get('num_ctx') or 0)
        reply = int(options.get('num_predict') or 0)
    except (TypeError, ValueError):
        return 0
    if ctx <= 0:
        return 0
    return max(MIN_PROMPT_TOKENS, int(ctx * CTX_SHARE) - reply)


def cost(messages, extra_tokens=0):
    """What this exact message list costs, plus whatever else rides with it -
    in practice the tool schemas, which are re-sent on every single turn and
    come out of the same window."""
    return approx_tokens(json.dumps(messages, default=str)) + extra_tokens


def _stub(message):
    """A tool result reduced to the fact that it happened. Returns None when
    there is nothing left to gain - a result already at or below stub length
    would only grow by having the marker added to it."""
    content = (message.get('content') or '').strip()
    first = content.splitlines()[0] if content else ''
    stubbed = clip(first, STUB_CHARS) + ' [...]'
    if len(stubbed) >= len(content):
        return None
    return dict(message, content=stubbed)


def _droppable(messages):
    """The oldest message that may go, or None when nothing may.

    Three are protected: the system prompt, whatever was said last - mid-turn
    a tool result the model is waiting on, between turns the question itself -
    and the most recent user message, which is the question a mid-turn prompt
    would otherwise lose while keeping the tool results taken to answer it.
    """
    last_user = None
    for index, message in enumerate(messages):
        if message.get('role') == 'user':
            last_user = index
    for index in range(1, len(messages) - 1):
        if index != last_user:
            return index
    return None


def fit(messages, budget, extra_tokens=0):
    """Make a message list fit the window. Returns the list, shortened.

    Mutates and returns the same list rather than copying: both callers hand
    over a list they are done deciding about, and a copy would leave the
    original growing behind it - which is the bug this is here to prevent.
    """
    if not budget or cost(messages, extra_tokens) <= budget:
        return messages

    for index in range(1, len(messages) - 1):
        if cost(messages, extra_tokens) <= budget:
            return messages
        if messages[index].get('role') != 'tool':
            continue
        stubbed = _stub(messages[index])
        if stubbed is not None:
            messages[index] = stubbed

    while cost(messages, extra_tokens) > budget and len(messages) > 2:
        index = _droppable(messages)
        if index is None:
            break
        del messages[index]

    if cost(messages, extra_tokens) > budget and len(messages) > 1:
        fixed = extra_tokens + approx_tokens(json.dumps(messages[0],
                                                        default=str))
        room = max(400, (budget - fixed) * 4)
        last = len(messages) - 1
        for index in {last, _droppable_user(messages)} - {None}:
            messages[index] = dict(
                messages[index],
                content=clip(messages[index].get('content') or '', room))
    return messages


def _droppable_user(messages):
    """The last user message, when it is not already the last message - the
    one thing step three above would otherwise leave uncut while clipping a
    tool result beside it."""
    for index in range(len(messages) - 2, -1, -1):
        if messages[index].get('role') == 'user':
            return index
    return None
