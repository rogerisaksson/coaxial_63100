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
import time

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
                timeout=None, reply_shape=None):
        # `reply_shape` is a plain dict for this reason: the saving it buys
        # is on the OTHER side of this socket, where the serial port is, so
        # it has to survive the trip as JSON.
        got = self._ask({'op': 'request', 'unit': unit, 'function': function,
                         'payload': bytes(payload).hex(),
                         'exact_payload': exact_payload, 'timeout': timeout,
                         'reply_shape': reply_shape})
        return bytes.fromhex(got['payload'])

    # -- the shared ring --------------------------------------------------

    def stream(self, stride, records):
        """Ask the broker to drain the task into a ring of `records`."""
        return self._ask({'op': 'daq_stream', 'stride': stride,
                          'records': records})

    def unstream(self):
        return self._ask({'op': 'daq_unstream'})

    def stream_state(self):
        return self._ask({'op': 'daq_state'})

    def take(self, cursor, most=0):
        """Records from `cursor`. (blob, first, lost, next).

        `lost` is what the writer overwrote before this cursor reached
        them - reported, never hidden, because a gap nobody counted is the
        one failure a shared ring must not have.
        """
        got = self._ask({'op': 'daq_take', 'from': cursor, 'max': most})
        return (bytes.fromhex(got['blob']), got['first'], got['lost'],
                got['next'])

    def broadcast(self, function, payload=b'', settle=0.05):
        self._ask({'op': 'broadcast', 'function': function,
                   'payload': bytes(payload).hex(), 'settle': settle})
        return None

    def answers(self, unit=1):
        """Whether the BOARD behind the broker replies. A look, not a use:
        it does not make this client one of the sessions holding the port."""
        return bool(self._ask({'op': 'answers', 'unit': unit})['answers'])

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
        # THE COUNT COMES DOWN FIRST. The base finish() flushes and closes,
        # which raises on a peer that was killed - and the decrement after it
        # then never ran, so the count stuck at one and the broker could
        # never take itself down. Measured: a session killed mid-run left it
        # holding the port with nobody on it.
        try:
            last = self._release()
        finally:
            try:
                socketserver.StreamRequestHandler.finish(self)
            except OSError:
                pass
        # THE LAST SESSION TAKES IT DOWN - after a linger. Refcounted like
        # the rails on the board: a broker nobody is using is a port nobody
        # else can open. The linger is what makes the MENU fast: spawning a
        # broker and handing the console over costs ~5 s, and going down
        # the instant a view closed meant every hop between views paid it
        # again - measured, open() 5.85 s against 0.05 through a live one.
        # A new client landing inside the linger cancels it.
        if last and self.server.until_idle:
            threading.Thread(target=self._stand_down_when_idle,
                             daemon=True).start()

    def _stand_down_when_idle(self):
        deadline = time.monotonic() + self.server.linger
        while time.monotonic() < deadline:
            time.sleep(0.5)
            with self.server.lock:
                if self.server.clients:
                    return                     # somebody came back - stay up
        with self.server.lock:
            if self.server.clients:
                return
        self.server.shutdown()

    def _release(self):
        """Give up this client's use, if it had one. True if it was the last."""
        if not self.uses:
            return False
        self.uses = False
        with self.server.lock:
            self.server.clients -= 1
            return self.server.clients == 0

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
        if op == 'answers':
            # A LOOK, NOT A USE - the staleness check asks this before it
            # commits `auto` to a real port, and whoever asks whether the
            # board is there must not become the last one out. The same
            # read the keepalive makes, under the same lock.
            from . import protocol
            try:
                with served.lock:
                    served.transport.request(
                        message.get('unit', 1), protocol.VERSION, b'',
                        None, 1.0)
            except (errors.RigError, OSError):
                return {'answers': False}
            served.spoke()
            return {'answers': True}
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
                got = served.retrying(
                    message['unit'], message['function'],
                    bytes.fromhex(message['payload']),
                    message['exact_payload'], message['timeout'],
                    message.get('reply_shape'))
            served.spoke()
            return {'payload': bytes(got).hex()}
        if op == 'daq_stream':
            served.stream(int(message['stride']), int(message['records']))
            return served.fanout.state()
        if op == 'daq_unstream':
            served.unstream()
            return {'streaming': False}
        if op == 'daq_state':
            if served.fanout is None:
                return {'streaming': False}
            got = served.fanout.state()
            got['streaming'] = served.streaming
            return got
        if op == 'daq_take':
            if served.fanout is None:
                raise errors.RigError(
                    'nothing is streaming - daq_stream first, and the '
                    'broker will keep the ring for every client on it')
            blob, first, lost, nxt = served.fanout.take(
                int(message['from']), int(message.get('max') or 0))
            return {'blob': blob.hex(), 'first': first, 'lost': lost,
                    'next': nxt}
        if op == 'broadcast':
            with served.lock:
                served.transport.broadcast(
                    message['function'], bytes.fromhex(message['payload']),
                    message['settle'])
            served.spoke()
            return {'payload': ''}
        return {'error': 'RigError', 'message': 'unknown op %r' % (op,)}


class _Server(socketserver.ThreadingTCPServer):

    """The broker's own state: the transport, the lock, and who is using it."""

    daemon_threads = True
    allow_reuse_address = True
    clients = 0
    until_idle = True
    #: How long an idle broker waits for the next client, seconds. Long
    #: enough to hop between menu views; short enough that the port frees
    #: itself within a minute of real abandonment. Zero in the tests, so
    #: one test's broker cannot linger on the port and answer the next
    #: test's clients - which it did, and the suite said so. stand_down
    #: stays immediate for whoever asks for the port by name.
    linger = 45.0

    #: The shared ring, and the thread that fills it. ONE READER OF THE
    #: BOARD, many readers of the ring: the link is a single wire and a
    #: second drainer would take records the first never sees, so the
    #: broker drains it once and every client reads its own way through
    #: `coaxial.fanout` from its own cursor.
    fanout = None
    streaming = False
    _streamer = None
    _stop_stream = None

    #: Records a client asks for in one go. Whole replies only - the board
    #: fits four or so in a PDU, and asking for more is a loop here.
    STREAM_BATCH = 0

    #: When a board frame last went out on anybody's behalf - clients and
    #: the keepalive both stamp it.
    heard = 0.0
    #: Seconds of client silence before the broker speaks for them. The
    #: firmware drops a silent host's rail claims after 10 s
    #: (BOARD_POWER_HOST_QUIET_MS) - right for a killed script, wrong for
    #: an operator thinking between chat turns. The margin is 3x.
    KEEPALIVE = 3.0

    def stream(self, stride, records):
        """Start draining the task into a ring of `records`, or resize it.

        Resizing makes a NEW ring, so every cursor is stale - clients are
        told where the new one starts by `daq_state` and take from there.
        Sizing a live ring in place would move records under readers that
        were counting on their sequence numbers meaning something.
        """
        from .fanout import Fanout

        with self.lock:
            same = (self.fanout is not None
                    and self.fanout.stride == stride
                    and self.fanout.capacity == records)
        if same and self.streaming:
            return
        self.unstream()
        self.fanout = Fanout(stride, records)
        self._stop_stream = threading.Event()
        self._streamer = threading.Thread(
            target=_stream_loop, args=(self, self._stop_stream),
            name='broker-daq', daemon=True)
        self.streaming = True
        self._streamer.start()

    def unstream(self):
        """Stop draining. What the ring holds stays readable."""
        if self._stop_stream is not None:
            self._stop_stream.set()
        if self._streamer is not None:
            self._streamer.join(2.0)
        self._streamer = None
        self._stop_stream = None
        self.streaming = False

    def spoke(self):
        import time
        self.heard = time.monotonic()

    def tick(self, stop):
        """One version read per KEEPALIVE of quiet, only while somebody
        is attached. The broker knows a thinking session from a dead one:
        it counts clients. With none attached nothing ticks, and the
        firmware's deadman does exactly its job."""
        import time

        from . import protocol
        from .errors import RigError

        while not stop.wait(0.5):
            if self.clients <= 0:
                continue
            if time.monotonic() - self.heard < self.KEEPALIVE:
                continue
            try:
                with self.lock:
                    self.transport.request(1, protocol.VERSION, b'')
                self.spoke()
            except (RigError, OSError):
                # The next real request finds out properly; a keepalive
                # never owns an error.
                pass

    def retrying(self, unit, function, payload, exact_payload,
                 timeout, reply_shape=None):
        """One request, and one re-open if the board went quiet.

        A RESET PUTS THE BOARD BACK IN ITS TEXT CONSOLE. The handover happens
        once, when the broker takes the port - so a board that resets under
        it answers nothing ever again, and takes every session with it.
        Measured: a live session went silent on 0x41 and stayed there.

        Silence only, and once: a refusal is an answer and must not be
        retried, and a board that is simply gone should say so rather than
        double every timeout.
        """
        from .errors import NoReplyError

        try:
            return self.transport.request(unit, function, payload,
                                          exact_payload, timeout, reply_shape)
        except NoReplyError:
            pass

        from .board import Board
        Board(self.transport, unit).open_binary()
        return self.transport.request(unit, function, payload,
                                      exact_payload, timeout, reply_shape)


def _stream_loop(served, stop):
    """Drain the board into the ring until told to stop.

    The broker's own reader, and the only one: a client that also read the
    board would take records this never sees. It costs the link exactly
    what one reader costs, however many clients are attached, which is the
    reason to put it here rather than in each of them.
    """
    from . import protocol

    payload = bytes([protocol.DEVICE_DAQ, 4, 0])
    stride = served.fanout.stride
    idle = 0.002
    while not stop.is_set():
        try:
            with served.lock:
                reply = served.transport.request(1, protocol.DEVICE, payload)
            served.spoke()
        except Exception:                              # noqa: BLE001
            # A quiet board is not a reason to stop streaming: the task may
            # be between configurations, and the next turn asks again.
            stop.wait(0.05)
            continue
        got = reply[0] if reply else 0
        if got:
            served.fanout.put(reply[1:1 + got * stride])
        else:
            stop.wait(idle)


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
    except errors.RigError:
        # It says no by refusing, and this function answers `did it`. A
        # caller asking whether the port came free should not have to catch
        # the sentence explaining that it did not.
        return False
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
          until_idle=True, linger=45.0):
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
    server.linger = linger
    quiet = threading.Event()
    threading.Thread(target=server.tick, args=(quiet,), daemon=True).start()

    # The KIND too - debug probe or RS485. A client reaching the broker
    # cannot work it out: that answer comes from the Windows port listing,
    # and without it every shared session called itself RS485.
    with open(WHERE, 'w', encoding='utf-8') as handle:
        json.dump({'serial': port, 'pid': os.getpid(), 'kind': _kind(port),
                   'host': address[0], 'tcp': address[1]}, handle)
    try:
        server.serve_forever()
    finally:
        quiet.set()
        server.server_close()
        if not handed:
            transport.close()
        try:
            os.remove(WHERE)
        except OSError:
            pass
