"""pyserial's `native://` scheme: tools.cores.native's line as a port (`Serial`), emulator://'s buses."""
import time

import serial
from serial.serialutil import SerialBase

from tools.cores import native
from tools.emu import protocol_emulator

buses = protocol_emulator.buses


class Serial(SerialBase):
    """`native://` as pyserial opens it, a UART's line: what is written every board on it
    hears at the line's pace; a read takes what they said, waiting its timeout while the limb
    runs on the wall's time."""

    #: Wall s a virtual s: the board keeps real time.
    time_scale = 1.0

    def open(self):
        try:
            self._limb = native.limb_for(self.portstr or 'native://')
        except RuntimeError as exc:           # no compiler: said as a port that fails
            raise serial.SerialException(str(exc)) from exc
        self.heat_clock = self._limb.heat_clock
        self.room = self._limb.room
        self.mount, self.mounted = self._limb.mount, self._limb.mounted()
        self.world_drive = self._limb.world_drive
        self.drag = self._limb.drag
        self.model = self._limb.model
        self.model_reset = self._limb.model_reset
        self.pilot = self._limb.pilot
        self.time_scale_source = self._limb.load
        #: The units a scan probes, and whether the port is a board's console.
        self.units = self._limb.units
        self.console = self._limb.console
        self._answer = bytearray()
        self.is_open = True

    def virtual_seconds(self):
        """The limb's clock, s."""
        return self._limb.seconds()

    def close(self):
        self.is_open = False

    def _reconfigure_port(self, force_update=False):
        pass

    @property
    def in_waiting(self):
        self._answer += self._limb.take()
        return len(self._answer)

    def read(self, size=1):
        deadline = time.monotonic() + (self.timeout if self.timeout is not None else 1e9)
        while True:
            self._answer += self._limb.take()
            if len(self._answer) >= size or time.monotonic() >= deadline:
                got, self._answer = bytes(self._answer[:size]), self._answer[size:]
                return got
            time.sleep(native.PACE_S)

    def write(self, data):
        data = bytes(data)
        self._limb.send(data)
        return len(data)

    def reset_input_buffer(self):
        self._limb.take()
        self._answer = bytearray()

    def reset_output_buffer(self):
        pass

    def flush(self):
        pass
