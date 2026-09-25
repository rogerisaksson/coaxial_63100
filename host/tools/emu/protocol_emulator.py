"""`emulator://` as pyserial opens it: the image on Renode, started on first open, stopped at exit.

    Coaxial63100(port='emulator://').open()                  # one board, its console
    Coaxial63100(port='emulator://?nodes=4', unit=3).open()  # a limb: the bus, a unit on it
    Coaxial63100(port='emulator://?world=quad&nodes=4').open()  # on the quad's rotors
    Coaxial63100(port='emulator://?nodes=2&baud=10000000', unit=2).open()  # 10 Mbit
    Coaxial63100(port='emulator://?mips=100').open()         # Renode's own speed, 4.75 x faster
    Coaxial63100(port='emulator://?body=humanoid&bus=LL', unit=2).open()  # the left knee
    Coaxial63100(port='emulator://?nodes=1&boot=1').open()   # blank: the host loads its build
    .\\coaxial_tty.ps1 -Port emulator://

One emulator per URL a process (tools.emu.emulator); every open of the URL is a connection
to it, its `time_scale` measured on the open and the Transport's, its `units` what a scan
probes, `console` False on a limb's bus. COAXIAL_ELF picks the image.
"""
import atexit
import urllib.parse

import serial
from serial.serialutil import SerialBase

from tools.emu.emulator import FAITHFUL_MIPS, Body, Emulator, Limb

_RUNNING = {}

#: A body's buses and the world each limb turns: the stand-in's fleet (coaxial.simulated.link)
#: emulated, a Renode process a limb.
BODIES = {'humanoid': {'AX': 'humanoid_axis', 'LA': 'humanoid_arm', 'LL': 'humanoid_leg',
                       'RA': 'humanoid_arm', 'RL': 'humanoid_leg'}}


def _query(url):
    return urllib.parse.parse_qs(urllib.parse.urlsplit(url).query)


def buses(url):
    """[(a bus's URL, what it serves)] for a body's URL, else None: the session's segments."""
    from coaxial.simulated.link import SIMULATED_BUSES

    query = _query(url)
    if 'body' not in query or 'bus' in query:
        return None
    return [('%s&bus=%s' % (url, name), SIMULATED_BUSES[name][0])
            for name in sorted(BODIES[query['body'][0]])]


def _body(url, query):
    """The limb a body's URL names, its first without `bus`: the body started once."""
    from coaxial.simulated.link import bus_nodes

    kind = query['body'][0]
    key = 'emulator://?body=%s' % kind
    if key not in _RUNNING:
        mips = int(query['mips'][0]) if 'mips' in query else FAITHFUL_MIPS
        body = Body({name: len(bus_nodes(name)) for name in BODIES[kind]}, worlds=BODIES[kind],
                    mips=mips).start()
        atexit.register(body.stop)
        _RUNNING[key] = body
    body = _RUNNING[key]
    return body.limbs[query.get('bus', [sorted(body.limbs)[0]])[0]]


def release(url):
    """The URL's emulator stopped, if one runs."""
    emu = _RUNNING.pop(url, None)
    if emu is not None:
        emu.stop()


def emulator_for(url):
    """The running emulator a URL names, started if it is not yet."""
    query = _query(url)
    if 'body' in query:
        return _body(url, query)
    if url not in _RUNNING:
        nodes = int(query.get('nodes', ['0'])[0])
        world = query.get('world', [None])[0]
        mips = int(query['mips'][0]) if 'mips' in query else FAITHFUL_MIPS
        baud = int(query['baud'][0]) if 'baud' in query else None
        boot = query.get('boot', ['0'])[0] not in ('0', '')
        emu = (Limb(nodes, world=world, mips=mips, baud=baud, boot=boot) if nodes
               else Emulator(world=world, mips=mips, boot=boot)).start()
        atexit.register(emu.stop)
        _RUNNING[url] = emu
    return _RUNNING[url]


class Serial(SerialBase):
    """A connection to the emulated board's console, or to a limb's bus."""

    def open(self):
        if self.port is None:
            raise serial.SerialException('no URL to open')
        try:
            emu = emulator_for(self.port)
        except RuntimeError as exc:           # no Renode, no image: said as a port that fails
            raise serial.SerialException(str(exc)) from exc
        self._inner = serial.serial_for_url(emu.url, self.baudrate, timeout=self.timeout)
        self.time_scale = emu.measure()
        self.time_scale_source = emu.load
        self.units = emu.units
        self.console = not isinstance(emu, Limb)
        self.is_open = True

    def close(self):
        if self.is_open:
            self._inner.close()
        self.is_open = False

    def _reconfigure_port(self, force_update=False):
        if self.is_open:
            self._inner.timeout = self.timeout

    def from_url(self, url):
        return url

    @property
    def in_waiting(self):
        return self._inner.in_waiting

    def read(self, size=1):
        return self._inner.read(size)

    def write(self, data):
        return self._inner.write(data)

    def flush(self):
        self._inner.flush()

    def reset_input_buffer(self):
        self._inner.reset_input_buffer()

    def reset_output_buffer(self):
        self._inner.reset_output_buffer()
