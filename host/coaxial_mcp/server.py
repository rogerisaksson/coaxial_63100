"""MCP wiring: the low-level Server over stdio.

The low-level API rather than FastMCP on purpose. FastMCP derives schemas from
type hints and docstrings, which is convenient and produces a bigger tool list
than necessary; here the schemas are hand-written because their size is the
thing being optimised.
"""
import anyio
import mcp.types as types
from mcp.server.lowlevel import Server
from mcp.server.stdio import stdio_server

from coaxial.errors import RigError

from . import render
from .session import Session
from .tools import HANDLERS, TOOLS

SERVER_NAME = 'coaxial-63100'


def build(session):
    server = Server(SERVER_NAME)

    @server.list_tools()
    async def list_tools():
        return [types.Tool(name=spec['name'],
                           description=spec['description'],
                           inputSchema=spec['inputSchema'])
                for spec in TOOLS]

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
                return handler(session, **(arguments or {}))
            except (RigError, ValueError, KeyError) as exc:
                # Expected and actionable, so answer compactly instead of
                # letting the SDK wrap a traceback the model has to read.
                return render.error(exc)

        text = await anyio.to_thread.run_sync(run)
        return [types.TextContent(type='text', text=text)]

    return server


async def serve(port='COM4', baud=115200, unit=1):
    session = Session(port, baud, unit)
    server = build(session)
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
    parser.add_argument('--baud', type=int, default=115200)
    parser.add_argument('--unit', type=int, default=1)
    args = parser.parse_args(argv)

    anyio.run(serve, args.port, args.baud, args.unit)
    return 0
