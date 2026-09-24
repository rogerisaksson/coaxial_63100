"""The robot's other boards, stood in: a battery pack and a head camera."""
import time

from machine.nodes import Module, Node
from machine.roles import Output


class _Contactor(Output):

    WRITES = ('contactor',)

    def __init__(self, bms):
        self.bms = bms

    def write(self, **values):
        self.bms.closed = values.get('contactor', 1.0) >= 0.5
        return values

    def off(self):
        self.bms.closed = False


class SimulatedBms(Node):

    """A 12S pack: each cell 3.3 V empty to 4.2 V full, 4 mohm; `draw()` amps while the
    contactor is closed. Trips a run below 3.3 V a cell or above 60 C."""

    CELLS = 12
    UNITS = dict(Node.UNITS, cell_min='V', contactor='0/1')

    def __init__(self, name='battery', ah=5.0, soc=0.9, draw=None, clock=time.monotonic):
        self.ah, self.soc, self.closed = float(ah), float(soc), True
        self.draw = draw or (lambda: 2.0)
        self._clock = clock
        self._last = clock()
        super().__init__(name, {'type': 'bms', 'device': 'simulated_bms', 'where': 'torso',
                                'link': 'simulated', 'unit': 0},
                         {'pack': Module(self._read, _Contactor(self), ('contactor',),
                                         {'contactor': (0.0, 1.0)},
                                         {'volts': {'LL': 3.3 * self.CELLS},
                                          'temp': {'HH': 60.0}})})

    def _read(self):
        now = self._clock()
        amps = self.draw() if self.closed else 0.0
        self.soc = max(0.0, self.soc - amps * (now - self._last) / 3600.0 / self.ah)
        self._last = now
        cell = 3.3 + 0.9 * self.soc - 0.004 * amps
        return {'volts': cell * self.CELLS, 'amps': amps, 'soc': self.soc, 'cell_min': cell,
                'temp': 25.0 + 0.5 * amps, 'contactor': self.closed}

    def couple(self, machine):
        """2 A, and a quarter amp an actuator."""
        self.draw = lambda: 2.0 + 0.25 * len(machine.actuators)


class SimulatedCamera(Node):

    """A camera and one target at `target_deg`: where it sits in the frame, -1..1 across the
    field of view (`identity['fov']`, deg), from where the actuator named `where` points
    (`yaw()`, deg); 0 unseen."""

    UNITS = dict(Node.UNITS, x='1', y='1', seen='0/1')

    def __init__(self, name='head_camera', target_deg=40.0, yaw=None, where='head'):
        self.target_deg, self.yaw = float(target_deg), yaw or (lambda: 0.0)
        super().__init__(name, {'type': 'camera', 'device': 'simulated_camera', 'where': where,
                                'link': 'simulated', 'unit': 0, 'fov': 60.0},
                         {'vision': Module(self._read)})

    def _read(self):
        off = (self.target_deg - self.yaw() + 180.0) % 360.0 - 180.0
        half = self.identity['fov'] / 2.0
        seen = abs(off) <= half
        return {'target': {'x': off / half if seen else 0.0, 'y': 0.0,
                           'seen': seen}}

    def couple(self, machine):
        """Turned by the actuator named where it sits, if the machine has one."""
        where = self.identity['where']
        if where in machine.actuators:
            self.yaw = lambda: machine.loop.bus.get(where + '.deg', 0.0)
