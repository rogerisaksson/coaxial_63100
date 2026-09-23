"""BOARD CHAT: the local llm, or claude with the board over MCP - the
second question."""
from terminal.loader import call, common

HEADLINE, KEY, WHAT = 'BOARD CHAT', 'C', 'prompt the local llm, or claude'
ORDER, NAME = 70, 'chat'
ITEMS = ('who answers', (
    ('C', 'CCC', 'coaxial 63100 chat client - the local llm', 'chat'),
    ('A', 'ANTHROPIC', 'claude, the board tools over MCP', 'claude')))


def run(args, name):
    import show_chat
    argv = common(args) + (['--claude'] if name == 'claude' else [])
    return call(show_chat.main, argv)
