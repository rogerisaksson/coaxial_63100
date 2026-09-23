"""How much of the model window a prompt may take, and what goes when it
takes more.
"""
import json

from .sandbox import clip

# The prompt's share of the window, before the reply cap comes off the top.
CTX_SHARE = 0.7

# The floor under that share.
MIN_PROMPT_TOKENS = 512

# What a stubbed tool result keeps, in characters, before the marker that says
# it is a stub.
STUB_CHARS = 80


def approx_tokens(text):
    """Rough but honest: about four characters per token for dense ASCII."""
    return max(1, len(str(text)) // 4)


def budget_for(options):
    """Tokens a prompt may take, from a client's own options."""
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
    """A tool result reduced to the fact that it happened."""
    content = (message.get('content') or '').strip()
    first = content.splitlines()[0] if content else ''
    stubbed = clip(first, STUB_CHARS) + ' [...]'
    if len(stubbed) >= len(content):
        return None
    return dict(message, content=stubbed)


def _droppable(messages):
    """The oldest message that may go, or None when nothing may."""
    last_user = None
    for index, message in enumerate(messages):
        if message.get('role') == 'user':
            last_user = index
    for index in range(1, len(messages) - 1):
        if index != last_user:
            return index
    return None


def fit(messages, budget, extra_tokens=0):
    """Make a message list fit the window."""
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
