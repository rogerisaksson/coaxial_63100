"""The assembly: SimulatedBoard wires the devices; SimulatedSession answers like session.Session.
"""
from typing import Any

from coaxial.comm.protocol import BROADCAST
from coaxial.errors import DeviceStateError
from coaxial.kalman.observer import Observer
from coaxial.simulated.acquire.capture import SimulatedCapture
from coaxial.simulated.acquire.clock import SimulatedClock
from coaxial.simulated.acquire.daq import SimulatedDaq
from coaxial.simulated.analog import SimulatedAfe, SimulatedAnalog, SimulatedCalibration
from coaxial.simulated.boot import SimulatedBoot
from coaxial.simulated.drive.device import SimulatedDrive
from coaxial.simulated.link import (DEFAULT_BUS, SIMULATED_BUSES, SimulatedLink,
                                    _BroadcastRefuses, bus_nodes)
from coaxial.simulated.power import SimulatedGateDrivers, SimulatedPower
from coaxial.simulated.sensors import SimulatedAngle, SimulatedImu
from coaxial.simulated.system import SimulatedGpio, SimulatedSystem
from coaxial.simulated.thermal.observer import SimulatedThermal


class SimulatedBoard:
    """A whole board without a board."""

    #: The rig that opened this board - `Board.rig`'s twin. And no wire:
    #: nothing streams from a stand-in but its own reader thread.
    rig: Any = None
    transport = None
    #: What it answers when asked its bitrate. There is no wire, so this is
    #: the rate it pretends to run at - enough for arithmetic about a link,
    #: and it is `origin.interface` that says the link is not real.
    baud = 115200

    def __init__(self, unit=1, bus=DEFAULT_BUS):
        self.unit = int(unit)
        self.bus = bus
        name, kind, where = bus_nodes(bus).get(
            self.unit,
            ('coaxial_63100', 'bldc_inverter',
             'unassigned unit %d on %s' % (self.unit, bus)))
        self.version_info = {
            # The MINOR is the one this stand-in actually implements - sensor
            # fields in records (7) and the counted duty (8) - so a host gating
            # a feature on the version exercises the same gate here that it
            # will at the bench.
            'proto_major': 2, 'proto_minor': 8, 'firmware': 'simulated',
            'device': name, 'mcu': 'STM32H753 (simulated)',
            'build': 'simulated', 'commands': 21, 'type': kind,
            # Says what it is AND that it is invented, in the same line, so a
            # list of five devices cannot be read as five real ones.
            'description': 'SIMULATED three-phase BLDC inverter at the %s'
                           % where,
            'where': where,
        }
        # The broadcast node wires nothing: every subsystem name refuses,
        # through __getattr__ - see BROADCAST_REFUSAL.
        if self.unit != BROADCAST:
            self._wire()

    def __getattr__(self, name):
        """On the broadcast unit every subsystem refuses, whatever it is
        called - a list of names here was the copy that went stale, and
        left `thermal` and `power` answering with AttributeError instead.
        """
        if name.startswith('_') or vars(self).get('unit') != BROADCAST:
            raise AttributeError(name)
        return _BroadcastRefuses()

    def _wire(self):
        """One of every subsystem, and the cross-links that make them one board
        rather than a bag of stand-ins.
        """
        self.system = SimulatedSystem(self.version_info)
        self.link = SimulatedLink()
        self.afe = SimulatedAfe()
        self.analog = SimulatedAnalog(self.afe)
        self.gpio = SimulatedGpio(self.afe)
        self.calibration = SimulatedCalibration()
        self.imu = SimulatedImu()
        self.angle = SimulatedAngle()
        self.gate_drivers = SimulatedGateDrivers()
        self.thermal = SimulatedThermal()
        # THE SAME CLASS AS THE BOARD'S, not a stand-in for it.
        self.observer = Observer(self)
        self.power = SimulatedPower()
        self.boot = SimulatedBoot()
        self.capture = SimulatedCapture()
        self.clock = SimulatedClock()
        self.daq = SimulatedDaq()
        # ONE TIMEBASE, as the board has one.
        self.daq.clock = self.clock
        self.drive = SimulatedDrive()
        # WHERE IT LOOKS.
        self.thermal._sample = self.drive.sample
        # AND WHAT IT DROPS.
        self.thermal._gate = self._drop_stage
        # And what the drive reports as switching: the bridge, so a dropped
        # stage stops making current in the model too.
        self.drive._switching = lambda: self.gate_drivers._enabled
        # AND THE HAND ON THE THROTTLE.
        self.thermal._derate_to = self._derate_drive
        self.thermal._duty = self._effective_duty
        # The drive is what the phases and the gates FOLLOW: a record and the
        # modulation that produced it come from one electrical angle, or they
        # are two inventions that happen to be printed together.
        self.gate_drivers._drive = self.drive
        self.daq.drive = self.drive
        # And the analog reads see the same current on the phases, so a tare
        # through them zeroes the records (values.py).
        self.analog.drive = self.drive
        # The shaft sensor reads the SAME rotor: a servo closed over the A1335
        # moves what the drive torques, or the loop it closes is between two
        # inventions.
        self.angle.drive = self.drive
        # And the A1335's die is as warm as the board it sits on.
        self.angle.thermal = self.thermal
        self.daq.angle = self.angle
        self.daq.imu = self.imu
        # `zero()` reads a channel, so it needs the board that has them.
        self.calibration.board = self

    def __repr__(self):
        return '<SimulatedBoard - no port, no cable, invented values>'

    def _derate_drive(self, factor):
        """Scale the drive's current clamp. `Board_DriveDerate`'s twin."""
        drive = self.drive
        if isinstance(drive, SimulatedDrive):
            drive._derate = max(0.0, min(1.0, factor))

    def _effective_duty(self):
        """What the compares hold, as a fraction of the period."""
        gates = self.gate_drivers
        if not isinstance(gates, SimulatedGateDrivers) or not gates._enabled:
            return (0.0, 0.0, 0.0)
        period = float(gates.PERIOD or 1)
        return tuple(t / period for t in gates._compares)

    def _drop_stage(self):
        """Drop the gates for the thermal envelope. True if it did."""
        if not self.gate_drivers._enabled:
            return False
        self.gate_drivers.off()
        return True

    def close_binary(self):
        pass

    def broadcast(self, function, payload=b''):
        """Acted on by every simulated node, answered by none."""
        return None

    def request(self, *_a, **_k):
        if self.unit == BROADCAST:
            raise DeviceStateError(
                'unit 0 is the broadcast address: every node acts on a '
                'broadcast and none answers it, so there is nothing to '
                'read back.')
        raise DeviceStateError('the simulated board answers through its '
                               'subsystems, not raw requests')


# Several of this board on one bus, which is what a machine built out of them
# looks like: same firmware, same commands, different unit id and a different
# thing bolted to the shaft.
class SimulatedSession:
    """Drop-in for `coaxial.comm.session.Session` that never opens a port."""

    # Read by anything that must not mistake this for a board - see
    # `coaxial_mcp.tools._interface`, which used to decide from the port and
    # started calling a bus label an RS485 segment.
    simulated = True

    def __init__(self, port=None, baud=115200, unit=1, bus=DEFAULT_BUS,
                 **_kwargs):
        # Takes what a real Session takes, so a caller that always builds "the
        # session" the same way is one fewer branch to keep in step.
        self.baud = baud
        self.bus = port if port in SIMULATED_BUSES else bus
        self.port = self.bus
        self.unit = int(unit)
        self._board = SimulatedBoard(self.unit, self.bus)
        # THE SESSION'S LINE REACHES THE BOARD.
        self._board.baud = self.baud
        if isinstance(self._board.daq, SimulatedDaq):
            self._board.daq.baud = self.baud
        self._info = None

    def buses(self):
        """[(label, what it serves)] - every segment on this machine."""
        return [(label, serves)
                for label, (serves, _) in sorted(SIMULATED_BUSES.items())]

    def scan(self, units=range(1, 17), bus=None):
        """[(unit, version)] for the nodes in `units` on one bus."""
        label = bus or self.bus
        nodes = bus_nodes(label)
        return [(unit, SimulatedBoard(unit, label).version_info)
                for unit in units if unit in nodes]

    def use(self, unit, bus=None):
        """Point this session at another node, and another bus with it."""
        if bus is not None:
            self.bus = self.port = bus
        self.unit = int(unit)
        self._board = SimulatedBoard(self.unit, self.bus)
        self._info = None
        return self.unit

    def broadcast(self, function, payload=b''):
        """Acted on by every simulated node, answered by none."""
        return None

    @property
    def board(self):
        return self._board

    @property
    def attached(self):
        """The board, which is always there: a stand-in opens nothing."""
        return self._board

    def info(self, refresh=False):
        if self._info is None or refresh:
            board = self._board
            self._info = (board.version_info, board.system.clock(),
                          board.analog.channels())
        return self._info

    def close(self):
        pass

    def reset(self):
        pass
