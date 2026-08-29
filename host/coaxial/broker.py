"""One process owns the serial port; everything else asks it.

WHY. The link is one wire and one slave, so two masters split a frame - the
same lesson SPI2 taught the IMU. Every tool here used to own the port
outright, which made switching the gate drivers and watching the heat two
processes and one port, and made looking at a running test impossible.

WHAT CROSSES. Modbus requests, unchanged: unit, function code, payload. The
broker does not interpret any of it and holds no state of its own, so it
cannot become a second protocol to keep in step with the first. It serialises
access and nothing else.

ERRORS CROSS AS THEMSELVES. A refusal from the board arrives at the client as
the same `coaxial.errors` class it would have raised in-process, carrying the
board's own sentence. Invariant 8 does not stop at a socket.
"""
import json
import os
import socket
import socketserver
import threading

from . import errors
from .transport import Transport

#: Loopback only. The board is a bench instrument on somebody's desk, and a
#: broker on 0.0.0.0 is that desk's power stage on the network.
HOST = '127.0.0.1'
PORT = 8763

#: Where the broker says what it is serving, so a client can name the port it
#: ended up on rather than guessing. Beside the session snapshot, and removed
#: on the way out.
WHERE = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                     os.pardir, 'tools', '.session.addr')


def _line(payload):
    return (json.dumps(payload) + '\n').encode('utf-8')


def _rebuild(answer):
    """The exception the far side raised, as itself.

    Not every class takes one string: ModbusException is built from the
    unit, the function and the reason code, and calling it with the message
    raised a TypeError that hid the board's actual refusal - measured, a
    whole switching run lost to `missing 2 required positional arguments`.
    The arguments cross with it, and the message is the fallback.
    """
    kind = getattr(errors, answer['error'], errors.RigError)

    # By FIELD, not by args: ModbusException formats its message in
    # __init__, so `args` is that one string and rebuilding from it gets
    # `missing 2 required positional arguments`. Its attributes are named
    # exactly like its parameters - unit, function, code - which is the
    # whole trick and holds for any error class written the same way.
    fields = answer.get('fields') or {}
    if fields:
        try:
            return kind(**fields)
        except TypeError:
            pass
    try:
        return kind(answer['message'])
    except TypeError:
        # A class this host cannot rebuild is still a refusal, and losing
        # the sentence is worse than losing the class.
        return errors.RigError('%s: %s' % (answer['error'], answer['message']))


class BrokerTransport:

    """A Transport that forwards instead of driving a UART.

    Duck-typed against `Transport`: `Board` takes either and cannot tell.
    Only what Board actually calls is here, for the reason the stand-ins give
    - a surface copied wholesale is a surface nobody checks.
    """

    def __init__(self, address=(HOST, PORT), timeout=10.0):
        self.address = address
        self.port = '?'
        self._lock = threading.Lock()
        self._sock = socket.create_connection(address, timeout=timeout)
        self._file = self._sock.makefile('rwb')
        self.baud = self._ask({'op': 'baud'})['baud']
        self.port = self._ask({'op': 'port'})['port']

    def _ask(self, message):
        with self._lock:
            self._file.write(_line(message))
            self._file.flush()
            raw = self._file.readline()

        if not raw:
            raise errors.NoReplyError(
                'the session broker closed the connection - it was serving '
                '%s and is not there now' % self.port)

        answer = json.loads(raw.decode('utf-8'))
        if 'error' in answer:
            raise _rebuild(answer)
        return answer

    def request(self, unit, function, payload=b'', exact_payload=None,
                timeout=None):
        got = self._ask({'op': 'request', 'unit': unit, 'function': function,
                         'payload': bytes(payload).hex(),
                         'exact_payload': exact_payload, 'timeout': timeout})
        return bytes.fromhex(got['payload'])

    def broadcast(self, function, payload=b'', settle=0.05):
        self._ask({'op': 'broadcast', 'function': function,
                   'payload': bytes(payload).hex(), 'settle': settle})
        return None

    @property
    def is_open(self):
        """Whether this CLIENT is still connected. Always False here.

        A teardown checks this to decide whether to hand the line back to the
        text console, and a client must never do that: the port is not its
        to give, and the console would take it from everyone else still
        attached. The broker hands it back when it stops.
        """
        return False

    def close(self):
        """Drop this client. The broker and the board carry on."""
        try:
            self._file.close()
            self._sock.close()
        except OSError:
            pass


class _Handler(socketserver.StreamRequestHandler):

    """One client, one line at a time. The lock is the whole design."""

    def setup(self):
        socketserver.StreamRequestHandler.setup(self)
        # A LOOK IS NOT A USE. `--status` and the staleness check attach to
        # prove the broker answers and close again; counting those would
        # have the last one out be whoever asked whether anybody was in.
        self.uses = False

    def finish(self):
        socketserver.StreamRequestHandler.finish(self)
        if not self.uses:
            return
        with self.server.lock:
            self.server.clients -= 1
            last = self.server.clients == 0
        # THE LAST SESSION TAKES IT DOWN. Refcounted like the rails on the
        # board, and for the same reason: a broker nobody is using is a port
        # nobody else can open, and one somebody has to remember to stop is
        # one that gets left running.
        if last and self.server.until_idle:
            threading.Thread(target=self.server.shutdown, daemon=True).start()

    def handle(self):
        while True:
            raw = self.rfile.readline()
            if not raw:
                return
            try:
                message = json.loads(raw.decode('utf-8'))
            except ValueError:
                message = None
            if message is None:
                answer = {'error': 'RigError', 'message': 'not a request'}
            else:
                answer = self._answer(message)
            self.wfile.write(_line(answer))
            self.wfile.flush()

    def _answer(self, message):
        served = self.server
        op = message.get('op')
        if op in ('request', 'broadcast') and not self.uses:
            self.uses = True
            with served.lock:
                served.clients += 1
        try:
            return self._do(served, op, message)
        except errors.RigError as exc:
            # The class name AND its arguments, so the client raises what it
            # would have raised in-process. A code the client maps back is
            # exactly the thing this tree refuses to have.
            return {'error': type(exc).__name__, 'message': str(exc),
                    'fields': {k: v for k, v in vars(exc).items()
                               if isinstance(v, (int, str))}}
        except Exception as exc:                      # noqa: BLE001
            return {'error': 'RigError',
                    'message': '%s: %s' % (type(exc).__name__, exc)}

    @staticmethod
    def _do(served, op, message):
        if op == 'baud':
            return {'baud': served.transport.baud}
        if op == 'port':
            return {'port': served.serial_port}
        if op == 'clients':
            return {'clients': served.clients}
        if op == 'stand_down':
            # Only with nobody using it. A suite that needs the port raw -
            # conformance sends deliberately malformed frames, which is the
            # one thing a broker cannot forward - takes it when it is free
            # and is told who has it when it is not.
            with served.lock:
                busy = served.clients
            if busy:
                return {'error': 'DeviceStateError',
                        'message': '%d session%s still using %s'
                                   % (busy, '' if busy == 1 else 's',
                                      served.serial_port)}
            threading.Thread(target=served.shutdown, daemon=True).start()
            return {'payload': ''}
        if op == 'request':
            with served.lock:
                got = served.transport.request(
                    message['unit'], message['function'],
                    bytes.fromhex(message['payload']),
                    message['exact_payload'], message['timeout'])
            return {'payload': bytes(got).hex()}
        if op == 'broadcast':
            with served.lock:
                served.transport.broadcast(
                    message['function'], bytes.fromhex(message['payload']),
                    message['settle'])
            return {'payload': ''}
        return {'error': 'RigError', 'message': 'unknown op %r' % (op,)}


class _Server(socketserver.ThreadingTCPServer):
    daemon_threads = True
    allow_reuse_address = True
    clients = 0
    until_idle = True


def serving():
    """What a running broker says it is serving, or None. Does not connect.

    The file alone is not the answer - it outlives a process that was killed,
    and a stale address reads exactly like a live one until the connect
    fails. This is the cheap check; `attach` is the real one.
    """
    try:
        with open(WHERE, encoding='utf-8') as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def clients(address=(HOST, PORT)):
    """How many sessions are using the broker. None if none is serving.

    Asking is not using: this attaches and closes, and the count it reports
    does not include itself.
    """
    reached = attach(address, timeout=2.0)
    if reached is None:
        return None
    try:
        return reached._ask({'op': 'clients'})['clients']   # noqa: SLF001
    finally:
        reached.close()


def stand_down(address=(HOST, PORT), wait=5.0):
    """Ask a broker to give the port back. True once nothing answers.

    Refuses while sessions are using it, and says how many - the port is
    theirs until they let go.
    """
    import time

    reached = attach(address, timeout=2.0)
    if reached is None:
        return True
    try:
        reached._ask({'op': 'stand_down'})                  # noqa: SLF001
    finally:
        reached.close()

    until = time.time() + wait
    while time.time() < until:
        if attach(address, timeout=0.5) is None:
            return True
        time.sleep(0.05)
    return False


def attach(address=(HOST, PORT), timeout=10.0):
    """A BrokerTransport, or None if nothing is serving.

    None here is not a status code standing in for a failure - it answers
    `is one running`, which invariant 8 has nothing to say about.
    """
    try:
        return BrokerTransport(address, timeout)
    except OSError:
        return None


def _kind(port):
    """`debug probe` or `RS485`, off the port listing. None if it cannot say."""
    import sys

    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.join(here, os.pardir, 'tools'))
    try:
        import find_board
        return find_board.kind_of(port)
    except Exception:                       # noqa: BLE001 - not fatal
        return None


def spawn(port, baud=115200, wait=8.0):
    """Start a broker for `port` in its own process. True if it came up.

    ITS OWN PROCESS, not a thread here: a broker inside the first session
    would die with it and take the port from everyone else still attached.
    This way the last session out is what stops it, whichever one that is.
    """
    import subprocess
    import sys
    import time

    here = os.path.dirname(os.path.abspath(__file__))
    script = os.path.join(here, os.pardir, 'tools', 'session.py')
    try:
        subprocess.Popen(                                # noqa: S603
            [sys.executable, script, '--port', port, '--baud', str(baud)],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            cwd=os.path.join(here, os.pardir))
    except OSError:
        return False

    until = time.time() + wait
    while time.time() < until:
        said = serving()
        if said and said.get('serial') == port:
            return True
        time.sleep(0.05)
    return False


def serve(port, baud=115200, address=(HOST, PORT), transport=None,
          until_idle=True):
    """Own the port and answer for it until interrupted.

    The console handover happens HERE, once, because owning the port is what
    it is for: the board boots into its text console, and a client reaching
    the broker cannot do it - the escape would go into a link that is
    already framed, which the board answers by handing the line back to the
    console for everybody.

    `transport` takes the place of the UART, which is how this is tested
    without one. It is a seam and not a stand-in: there is no simulated
    broker, because the stand-in has no port for two processes to contend
    over and speaks methods rather than frames.

    `until_idle` stops when the last client goes, which is what makes this
    something nobody has to remember to start or stop. `session.py --hold`
    is the other case: a bench where the port should stay taken between
    runs.
    """
    from .board import Board          # here: board.py reaches for this one

    handed = transport is not None
    if not handed:
        transport = Transport(port, baud)
        Board(transport, 1).open_binary()
    server = _Server(address, _Handler)
    server.transport = transport
    server.serial_port = port
    server.lock = threading.Lock()
    server.until_idle = until_idle

    # The KIND too - debug probe or RS485. A client reaching the broker
    # cannot work it out: that answer comes from the Windows port listing,
    # and without it every shared session called itself RS485.
    with open(WHERE, 'w', encoding='utf-8') as handle:
        json.dump({'serial': port, 'pid': os.getpid(), 'kind': _kind(port),
                   'host': address[0], 'tcp': address[1]}, handle)
    try:
        server.serve_forever()
    finally:
        server.server_close()
        if not handed:
            transport.close()
        try:
            os.remove(WHERE)
        except OSError:
            pass
