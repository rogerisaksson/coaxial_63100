"""How much of the model window a prompt may take, and what goes when it takes
more.

Both loops here grow a message list for the same daemon - `debug.Chat` for a
question, `runner.Runner` for a plan step - and both used to bound it by
counting messages. A message count is not a bound: six messages is a small
prompt right up until one of them is a build log.

`num_ctx` is a bound, because that is what the daemon allocates a KV cache
for. Past it there is no polite refusal - there is a 500, or llama-server
dying while saving its prompt cache. The client's own ladder is for a machine
short of memory; this is for a conversation that simply got long.

What is given up, in order, least missed first:

  1. tool results, squeezed to their first line, oldest first. The model can
     read one again for the price of a call.
  2. whole messages from the front, system prompt and live question aside.
  3. the newest message itself, clipped - only when one message is larger than
     the window: a pasted log, an attached file.

Nothing is silent: every cut leaves the notice `clip` writes, and `/ctx` shows
the budget beside the cost.
"""
import json

from .sandbox import clip

# The prompt's share of the window, before the reply cap comes off the top.
# Seven tenths, not all: the estimate below is an estimate, and llama-server
# wants working room beside the context it was asked for.
CTX_SHARE = 0.7

# The floor under that share. Clipping to nothing turns "the answer is short"
# into "there was no question", which is the worse failure.
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
    bare client. Guessing low would silently delete a conversation nobody
    asked to shorten.
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
    """A tool result reduced to the fact that it happened. None when there is
    nothing to gain - a short result would only grow by being marked."""
    content = (message.get('content') or '').strip()
    first = content.splitlines()[0] if content else ''
    stubbed = clip(first, STUB_CHARS) + ' [...]'
    if len(stubbed) >= len(content):
        return None
    return dict(message, content=stubbed)


def _droppable(messages):
    """The oldest message that may go, or None when nothing may.

    Three are protected: the system prompt, whatever was said last, and the
    most recent user message - which a mid-turn prompt would otherwise lose
    while keeping the tool results taken to answer it.
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

    Mutates rather than copies: a copy would leave the original growing
    behind it, which is the bug this exists to prevent.
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
    """The last user message when it is not already the last message - what
    step three would otherwise leave uncut beside a clipped tool result."""
    for index in range(len(messages) - 2, -1, -1):
        if messages[index].get('role') == 'user':
            return index
    return None
