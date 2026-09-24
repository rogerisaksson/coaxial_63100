"""The bus tools: which interfaces answer, which nodes are on them, and the link's health."""
from . import render
from coaxial.comm import ports, protocol
from coaxial_mcp.names import _key


def _interface(session):
    """How the host reaches this bus, for the list's own header."""
    if session.simulated or session.port is None:
        return 'Simulated'
    port = session.port
    try:
        kind = ports.kind_of(port)
    except OSError:                                    # the port listing
        return str(port)
    return '%s at %s' % ('debug probe' if kind == ports.PROBE
                         else 'RS485', port)


def _buses(session):
    """[(label, serves)] for this session, however it is wired."""
    return list(session.buses())


def _sweep(session, first, last, bus=None):
    """[(bus, unit, version)] across one segment or all of them."""
    units = range(int(first), int(last) + 1)
    labels = [bus] if bus else [label for label, _ in _buses(session)]
    found = []
    for label in labels:
        for unit, version in session.scan(units, bus=label):
            found.append((label, unit, version))
    return found


def devices(session, op='list', unit=None, name=None, bus=None,
            first=1, last=16, **_):
    """The other units on this bus, and which one the tools talk to."""
    here = (session.bus, session.unit)
    if op == 'buses':
        counts = [(label, serves,
                   len(_sweep(session, first, last, label)))
                  for label, serves in _buses(session)]
        return render.buses(counts, here[0])
    if op == 'list':
        return render.devices(_sweep(session, first, last, bus), here,
                              _interface(session))
    if op == 'use':
        return _use(session, here, unit, name, bus, first, last)
    return 'ERR unknown op %r; list or use' % (op,)


def _named(found, name):
    """The nodes `name` picks out by what they call themselves, across every
    segment: "the right knee" is one node on one bus, and the operator
    should not have to know which.
    """
    key = _key(name)
    return [(b, u) for b, u, v in found
            if key in _key(v.get('where', ''))
            or key in _key(v.get('description', ''))]


def _use(session, here, unit, name, bus, first, last):
    """Point the session at one node, by unit or by name - or at every node at
    once, which Modbus spells 0.
    """
    if unit is not None and int(unit) == protocol.BROADCAST:
        # Never in the scan, and never will be: nothing answers at 0.
        session.use(protocol.BROADCAST)
        return ('multicast: every node on the bus acts, none answers. '
                'Reads are refused here; an order still goes out. '
                'devices op=use unit=N picks one node again.')
    found = _sweep(session, first, last, bus)
    if unit is None and not name:
        return ('ERR use needs unit= or name=. On the bus: %s'
                % '; '.join('%s %d %s' % (b, u, v.get('where', ''))
                            for b, u, v in found))
    hit = _named(found, name) if unit is None else []
    if unit is None and len(hit) != 1:
        return ('ERR %r matches %d nodes: %s'
                % (name, len(hit),
                   ', '.join('%s %d' % pair for pair in hit) or 'none'))
    if unit is None:
        bus, unit = hit[0]
    if (bus or here[0], int(unit)) not in [(b, u) for b, u, _ in found]:
        # Not a refusal for its own sake: pointing the session at a unit nobody
        # is at makes every later call time out, and the operator reads that as
        # the board having died.
        return ('ERR no node at %s %s; answering: %s'
                % (bus or here[0], unit,
                   ', '.join('%s %d' % (b, u) for b, u, _ in found)
                   or 'none'))
    session.use(int(unit), bus=bus)
    return render.devices(found, (session.bus, session.unit),
                          _interface(session))


def link(session, op='stats', text='ping', **_):
    board = session.board
    if op == 'echo':
        return 'echo ok %r' % board.link.echo(text)
    if op == 'release':
        board.close_binary()
        session.reset()
        return 'console released'
    return render.kv(board.link.stats())
