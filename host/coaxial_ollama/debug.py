"""A lean prompt loop for debug jobs: fewest tokens in, fewest tokens out."""
import os
import sys
import threading
from contextlib import suppress
from importlib import import_module

from .client import OllamaError

# host/ on the path: this file's own directory's parent, so it does not matter
# what the working directory is or what any directory along the way is called.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from typing import Any

from . import intent
from . import language                               # noqa: E402
from .iolog import IOLog                             # noqa: E402
from coaxial_mcp import detail                       # noqa: E402
from coaxial_ollama.budget import ChatBudget
from coaxial_ollama.commands import ChatCommands
from coaxial_ollama.turn import _printable, ChatTurn


class Chat(ChatBudget, ChatTurn, ChatCommands):
    """One conversation, trimmed on the way out to the model."""

    #: WHAT A CHAT HOLDS, declared. A suite builds one bare with __new__
    #: and sets what the method under test reads; the rest are these,
    #: and a method reads an attribute rather than asking getattr whether
    #: it exists. `client` and `out` are Any: a scripted model and a
    #: StringIO stand in for both.
    client: Any = None
    toolbox: Any = None
    out: Any = None
    io_log: Any = None         # the CLI's IOLog once it opened one
    language = None
    tool_names: tuple = ()     # set_tools() fills it
    schemas: 'list | None' = None   # the specs, None with no tools
    prompt_history: tuple = ()
    intent = ''
    compile_intent = False
    _said_node = None

    def __init__(self, client, toolbox, tools='read', keep=6, budget=0,
                 quiet=False, out=None, link_ok=True, detail_level=detail.AUTO,
                 session_language=None):
        self.client: Any = client
        self.toolbox = toolbox
        # Before set_tools below, which builds the schemas this decides the
        # length of.
        self.detail = detail.resolve(detail_level,
                                     model=client.model,
                                     default=detail.TERSE)
        toolbox.detail = self.detail
        self.keep = keep
        self.budget = budget
        self.quiet = quiet
        self.out: Any = out or _printable(sys.stdout)
        # Shared with the REPL's spinner: it repaints on its own thread, and
        # unsynchronised writes to one stream interleave into garbage.
        self.print_lock = threading.RLock()
        self.history = []
        self.turn_cost = []
        # What the prompt's spinner shows: read fresh on every prompt, so
        # /reconnect changes it on the very next line rather than needing a
        # restart to notice the cable was plugged back in.
        self.link_ok = link_ok
        # Names from the most recent successful analog_read, kept across turns
        # (unlike the turn-local copy in `ask`) so a later turn that answers
        # with no tool call at all can still be checked against it.
        self.last_channels = None
        # Where the tools are pointed when the session opens, so the first
        # answer does not announce a node nothing moved to.
        self._said_node = (toolbox.session.bus, toolbox.session.unit)
        # The session's language.
        self.language = session_language
        # Every question typed this session, in order - independent of
        # self.history, which the REPL clears after each answered turn.
        self.prompt_history = ()
        # Off by default: dozens of tests build a Chat and none should touch
        # the filesystem.
        self.io_log: Any = IOLog(enabled=False)
        # Compile the question into an intent before answering it.
        self.compile_intent = False
        self._intent_why = self._intent_did = None
        self._intent_kind = self._intent_tool = None
        self.set_tools(tools)

    def close(self):
        """Hand the card back and close the log - every page's way out."""
        with suppress(OllamaError):
            self.client.unload()
        if self.io_log is not None:
            self.io_log.close()


if __name__ == '__main__':
    from .cli import main                 # see _ELSEWHERE
    sys.exit(main())


# Moved out, re-exported: dbg.py and two suites reach for these by their old
# names.
_ELSEWHERE = {
    'IOLog': 'iolog', 'IO_LOG_PATH': 'iolog',
    'main': 'cli', 'parse': 'cli', 'build': 'cli', 'repl': 'cli',
    'attach': 'cli', 'ask_operator': 'cli', 'NoBoard': 'cli',
    'keep_alive_for': 'cli', 'KEEP_ALIVE_REPL': 'cli',
    'KEEP_ALIVE_ONCE': 'cli', 'INPUT_LIMIT': 'cli', '_printable': 'cli',
}


def __getattr__(name):
    where = _ELSEWHERE.get(name)
    if where is None:
        raise AttributeError(name)
    return getattr(import_module('.' + where, __package__), name)
