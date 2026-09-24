"""A lean prompt loop for debug jobs: fewest tokens in, fewest tokens out."""
import sys
import threading
from contextlib import suppress
from importlib import import_module
from typing import Any

from .client import OllamaError
from .iolog import IOLog
from coaxial_mcp import detail
from coaxial_ollama.budget import ChatBudget
from coaxial_ollama.commands import ChatCommands
from coaxial_ollama.turn import ChatTurn, _printable


class Chat(ChatBudget, ChatTurn, ChatCommands):
    """One conversation, trimmed on the way out to the model."""

    #: What a chat holds, declared. A suite builds one bare with __new__
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
        # What the prompt's spinner shows, read per prompt: /reconnect
        # changes it on the next line.
        self.link_ok = link_ok
        # Names from the last successful analog_read, kept across turns,
        # unlike the turn-local copy in `ask`: a later turn with no tool call
        # is checked against it.
        self.last_channels: set | None = None     # what the last turn read
        # Where the tools are pointed when the session opens, so the first
        # answer does not announce a node nothing moved to.
        self._said_node = (toolbox.session.bus, toolbox.session.unit)
        # The session's language.
        self.language = session_language
        # Every question typed this session, in order - independent of
        # self.history, which the REPL clears after each answered turn.
        self.prompt_history = ()
        # Off by default: suites build a Chat and write no file.
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
