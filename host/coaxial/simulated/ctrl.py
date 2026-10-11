"""The board's loop without a board: ctrl_loop.c in the stand-in drive's slot, this its edge."""
import ctypes
from typing import Any

from coaxial.devices.ctrl import COMMANDS, KINDS, MEASURED, SLOTS, _index, _rows
from coaxial.errors import RigError
from machine.roles import Controller
from tools.cores import stand

#: Rows a call carries to the ring at once (a u8), each call all or none.
ROWS_A_CALL = 255


class SimulatedCtrl(Controller):

    """Device 12 on the stand-in: the board's loop ticked in its drive's sample (world_drive.c),
    rows played on that drive's clock, the shaft its angle."""

    def __init__(self):
        #: The drive whose slot carries the loop; the board wires it.
        self.drive: Any = None

    def _loop(self, call, *args):
        """ctrl_loop's `call` on the drive's loop, the drive brought to now first; a refusal
        raised in the board's words."""
        drive, lib = self.drive, stand.library()
        with drive._lock:
            drive._advance()
            why = getattr(lib, call)(lib.world_drive_loop(drive._slot), *args)
        if why:
            raise RigError('%s (simulated)' % why.decode())
        return True

    def state(self):
        drive = self.drive
        with drive._lock:
            drive._advance()
            row = stand.loop(drive._slot)
        counts = {name: int(row[name]) for name in ('hz', 'rows', 'free', 'played', 'idle',
                                                    'blind')}
        return dict({name: row[name] for name in ('queued_s', 'setpoint', 'ref', 'value',
                                                  'estimate', 'out')},
                    running=bool(row['running']), playing=bool(row['playing']),
                    measured=MEASURED[int(row['measured'])],
                    command=COMMANDS[int(row['command'])], **counts)

    def read(self, count=None, timeout=None):
        return self.state()

    def slot(self, slot, part=None):
        index = _index(SLOTS, slot, 'slot')
        kind = _index(KINDS, type(part).__name__, 'kind') if part is not None else 0
        values = [float(getattr(part, name)) for name in part.PARAMS] if part is not None else []
        return self._loop('ctrl_loop_slot', index, kind, stand.floats(values or [0.0]),
                          len(values))

    def load(self, feedback):
        for name in SLOTS:
            self.slot(name, getattr(feedback, name))
        return True

    def wire(self, measured, command, hz):
        if not 0 < float(hz) <= 0xFFFF:
            raise RigError('hz is 1 up to the PWM rate (simulated)')
        return self._loop('ctrl_loop_wire', self.drive.TS, _index(MEASURED, measured, 'measured'),
                          _index(COMMANDS, command, 'command'), int(hz))

    def rows(self, rows):
        wire = _rows(rows)
        for at in range(0, len(wire), ROWS_A_CALL):
            chunk = wire[at:at + ROWS_A_CALL]
            self._loop('ctrl_loop_rows', (ctypes.c_uint16 * len(chunk))(*[ms for ms, _ in chunk]),
                       stand.floats([milli / 1e3 for _, milli in chunk]), len(chunk))
        return len(wire)

    def drop(self):
        drive, lib = self.drive, stand.library()
        with drive._lock:
            drive._advance()
            lib.ctrl_rows_clear(lib.world_drive_loop(drive._slot))
        return True

    #: The setpoint last written: a loop writes every pass, the ring only a change.
    _written = None

    def write(self, setpoint):
        if setpoint != self._written:
            self.rows([(0.0, setpoint)])
            self._written = setpoint
        return True

    def on(self):
        return self._loop('ctrl_loop_run', True)

    def off(self):
        return self._loop('ctrl_loop_run', False)

    def is_on(self):
        return self.state()['running']
