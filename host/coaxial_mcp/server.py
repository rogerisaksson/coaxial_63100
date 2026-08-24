"""MCP wiring: the low-level Server over stdio.

The low-level API rather than FastMCP on purpose. FastMCP derives schemas from
type hints and docstrings, which is convenient and produces a bigger tool list
than necessary; here the schemas are hand-written because their size is the
thing being optimised.
"""
import sys

import anyio
import mcp.types as types
from mcp.server.lowlevel import Server
from mcp.server.stdio import stdio_server

from coaxial.errors import RigError

from . import detail as detailmod
from . import render
from .session import Session, open_session
from .tools import HANDLERS, TOOLS

SERVER_NAME = 'coaxial-63100'


def build(session, level=detailmod.FULL):
    """The server, at one level of documentation.

    Full by default, and that is the right default here rather than in the
    ollama package: the reader on the other end of an MCP pipe has a context
    window measured in hundreds of thousands of tokens, and shortening the
    tool list for it buys nothing it needed. `--detail terse` is for the case
    this server was not built for and now supports anyway - a small local
    model driving it through an MCP client of its own.
    """
    server = Server(SERVER_NAME)

    @server.list_tools()
    async def list_tools():
        return [types.Tool(name=spec['name'],
                           description=spec['description'],
                           inputSchema=spec['inputSchema'])
                for spec in detailmod.apply(TOOLS, level)]

    @server.call_tool()
    async def call_tool(name, arguments):
        handler = HANDLERS.get(name)
        if handler is None:
            return [types.TextContent(type='text',
                                      text='ERR unknown tool %r' % name)]

        # The board is a serial port: every handler blocks. Run it off the event
        # loop so a long burst cannot stall the protocol side of the server.
        def run():
            try:
                # `detail` is this server's, not the caller's: it decides how
                # much of a document `docs` hands back. Every handler takes
                # **_, so the ones that do not read it are unaffected.
                return handler(session, detail=level, **(arguments or {}))
            except (RigError, ValueError, KeyError) as exc:
                # Expected and actionable, so answer compactly instead of
                # letting the SDK wrap a traceback the model has to read.
                return render.error(exc)

        text = await anyio.to_thread.run_sync(run)
        return [types.TextContent(type='text', text=text)]

    return server


async def serve(port='COM4', baud=115200, unit=1, level=detailmod.FULL,
                simulated=False):
    session, found = open_session(port, baud, unit, simulated=simulated)
    # stderr, not stdout: stdout is the JSON-RPC pipe. board_info says
    # "simulated" in the version record either way - this is for whoever
    # started the process and would otherwise not know which they got.
    print('serving: %s' % found.label, file=sys.stderr, flush=True)
    server = build(session, level)
    try:
        async with stdio_server() as (read_stream, write_stream):
            await server.run(read_stream, write_stream,
                             server.create_initialization_options())
    finally:
        # Hand the UART back, or the board looks dead to anyone with a terminal.
        session.close()


def main(argv=None):
    import argparse

    parser = argparse.ArgumentParser(prog='python -m coaxial_mcp')
    parser.add_argument('--port', default='COM4')
    parser.add_argument('--simulated', dest='simulated', action='store_const',
                        const=True, default=False,
                        help='serve a stand-in board instead of opening the '
                             'port. board_info reports "simulated" for the '
                             'firmware either way')
    parser.add_argument('--auto', dest='simulated', action='store_const',
                        const=None,
                        help='probe --port first and fall back to the '
                             'stand-in only if nothing answers')
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--unit', type=int, default=1)
    parser.add_argument('--detail', default=detailmod.FULL,
                        choices=detailmod.LEVELS,
                        help='how much documentation the tools and `docs` '
                             'carry. Full here by default - the reader on an '
                             'MCP pipe has room for it; terse is for a small '
                             'local model driving this server. %s overrides.'
                             % detailmod.ENV)
    args = parser.parse_args(argv)

    anyio.run(serve, args.port, args.baud, args.unit,
              detailmod.resolve(args.detail), args.simulated)
    return 0
