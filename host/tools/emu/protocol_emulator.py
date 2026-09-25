"""`emulator://` as pyserial opens it: the image on Renode, started on first open, stopped at exit.

    Coaxial63100(port='emulator://').open()                  # one board, its console
    Coaxial63100(port='emulator://?nodes=4', unit=3).open()  # a limb: the bus, a unit on it
    Coaxial63100(port='emulator://?world=quad&nodes=4').open()  # on the quad's rotors
    Coaxial63100(port='emulator://?nodes=2&mips=475&baud=10000000', unit=2).open()  # 10 Mbit
    .\\coaxial_tty.ps1 -Port emulator://

One emulator per URL a process (tools.emu.emulator); every open of the URL is a connection
to it. COAXIAL_ELF picks the image.
"""
import atexit
import urllib.parse

import serial
from serial.serialutil import SerialBase

from tools.emu.emulator import Emulator, Limb

_RUNNING = {}


def emulator_for(url):
    """The running emulator a URL names, started if it is not yet."""
    if url not in _RUNNING:
        query = urllib.parse.parse_qs(urllib.parse.urlsplit(url).query)
        nodes = int(query.get('nodes', ['0'])[0])
        world = query.get('world', [None])[0]
        mips = int(query['mips'][0]) if 'mips' in query else None
        baud = int(query['baud'][0]) if 'baud' in query else None
        emu = (Limb(nodes, world=world, mips=mips, baud=baud) if nodes
               else Emulator(world=world, mips=mips)).start()
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
