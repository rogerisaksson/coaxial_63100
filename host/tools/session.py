#!/usr/bin/env python3
"""Hold the board's port so more than one thing can use it.

    python tools/session.py                 # serve COM4 until Ctrl+C
    python tools/session.py --port COM7
    python tools/session.py --status        # what is serving, if anything

Everything else - the views, the session dashboard, switch.py, the MCP
server, a one-off script - reaches the broker on its own once it is up, and
opens the port directly when it is not. Nothing has to be told which.

The board is still one slave on one wire: requests are serialised, so two
clients interleave whole transactions and never a frame. What this removes
is the exclusive OWNERSHIP, not the exclusivity of the wire.
"""
import argparse
import os
import signal
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import broker                                # noqa: E402
from screen import say                                    # noqa: E402


def status():
    """What is serving, and whether it actually answers."""
    said = broker.serving()
    if not said:
        say('ok', 'session', 'nothing is serving - every tool opens its own')
        return 0

    reached = broker.attach((said.get('host', broker.HOST),
                             said.get('tcp', broker.PORT)))
    if reached is None:
        # The file outlives a process that was killed, and a stale address
        # reads exactly like a live one until the connect fails.
        say('warn', 'session', 'a stale address for %s, pid %s - nothing '
                               'answers on it' % (said.get('serial'),
                                                  said.get('pid')))
        return 1

    say('ok', 'session', '%s, pid %s, %d baud'
        % (reached.port, said.get('pid'), reached.baud))
    reached.close()
    return 0


def force():
    """Stop a serving broker outright, sessions or not.

    The polite way refuses while sessions hold the port, which is right: it
    is theirs until they let go. It stops being right when the LINK is dead
    - a board that reset under the broker answers nothing, and the sessions
    being protected cannot talk either. Then the refusal is the only thing
    standing between the bench and a working port.

    Asks first, so the ordinary case still goes through the front door.
    """
    said = broker.serving()
    if not said:
        say('ok', 'session', 'nothing is serving')
        return 0

    where = (said.get('host', broker.HOST), said.get('tcp', broker.PORT))
    if broker.stand_down(where):
        say('ok', 'session', 'it stood down on its own - nobody was using it')
        return 0

    pid = said.get('pid')
    say('warn', 'session', 'stopping pid %s, and every session on it' % pid)
    try:
        os.kill(int(pid), signal.SIGTERM)
    except (OSError, TypeError, ValueError) as exc:
        say('fail', 'session', 'could not stop pid %s: %s' % (pid, exc))
        return 1

    try:
        os.remove(broker.WHERE)
    except OSError:
        pass
    say('ok', 'session', 'port given back - the next session opens its own')
    return 0


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--port', default='COM4')
    p.add_argument('--baud', type=int, default=115200)
    p.add_argument('--hold', action='store_true',
                   help='stay up with no clients; otherwise the last session out takes it down')
    p.add_argument('--status', action='store_true')
    p.add_argument('--force', action='store_true',
                   help='stop a broker even though sessions hold it - for one '
                        'whose link is dead, where the refusal protects '
                        'sessions that cannot talk anyway')
    a = p.parse_args()

    if a.status:
        return status()

    if a.force:
        return force()

    if broker.attach() is not None:
        say('fail', 'session', 'one is already serving - see --status')
        return 1

    say('ok', 'serving', '%s at %d baud on %s:%d'
        % (a.port, a.baud, broker.HOST, broker.PORT))
    say('wait', 'holding',
        'Ctrl+C to give the port back' if a.hold
        else 'until the last session goes, or Ctrl+C')
    try:
        broker.serve(a.port, a.baud, until_idle=not a.hold)
    except KeyboardInterrupt:
        pass
    say('ok', 'session', 'port given back')
    return 0


if __name__ == '__main__':
    sys.exit(main())
