"""MCP server for the coaxial_63100 board.

Built for token economy, so a small model can run a long test sequence without
its context filling up with protocol noise:

  * seven coarse tools rather than one per firmware command, because the whole
    tool list is re-read on every turn;
  * one-line descriptions and short property names;
  * dense text results - fixed columns, no JSON punctuation, numbers rounded to
    what the hardware resolves;
  * the channel map returned once by board_info and referred to by name after
    that, instead of repeated inside every reading;
  * errors as a single ERR line carrying the way out.

The board connection opens lazily on the first tool call and is not a tool of
its own: a forgotten connect would cost a whole round trip.
"""
__all__ = ['render', 'session', 'tools', 'server']
