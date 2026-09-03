"""The assembly: SimulatedBoard wires every device together and
SimulatedSession answers like coaxial_mcp.session.Session."""
from ..errors import DeviceStateError
from .link import DEFAULT_BUS, SIMULATED_BUSES, SimulatedLink, _BroadcastRefuses, bus_nodes
from .system import SimulatedGpio, SimulatedSystem
from .analog import SimulatedAfe, SimulatedAnalog, SimulatedCalibration
from .sensors import SimulatedAngle, SimulatedImu
from .power import SimulatedGateDrivers, SimulatedPower, SimulatedThermal
from .daq import SimulatedCapture, SimulatedClock, SimulatedDaq
from ..observer import Observer
from .drive import SimulatedDrive


class SimulatedBoard:
    """A whole board without a board. Duck-typed against the real one, so
    the tools above cannot tell which they are holding - except that
    every touchpoint labels itself."""
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
            # The MINOR is the one this stand-in actually implements -
            # sensor fields in records (7) and the counted duty (8) - so a
            # host gating a feature on the version exercises the same gate
            # here that it will at the bench. It sat at 0 while the
            # firmware moved to 2.8, and pulse.py's counted path would
            # have silently link-timed every simulated rehearsal.
            'proto_major': 2, 'proto_minor': 8, 'firmware': 'simulated',
            'device': name, 'mcu': 'STM32H753 (simulated)',
            'build': 'simulated', 'commands': 21, 'type': kind,
            # Says what it is AND that it is invented, in the same line, so
            # a list of five devices cannot be read as five real ones.
            'description': 'SIMULATED three-phase BLDC inverter at the %s'
                           % where,
            'where': where,
        }
        if self.unit == 0:
            refuse = _BroadcastRefuses()   # see BROADCAST_REFUSAL
            self.system = self.link = self.afe = refuse
            self.analog = self.gpio = self.imu = refuse
            self.calibration = refuse
            self.angle = self.gate_drivers = self.capture = self.daq = refuse
            self.drive = self.observer = refuse
            self.clock = refuse
        else:
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
            # `Observer` is composed out of ops that already exist -
            # the drive's, the shaft sensor's - so there is nothing
            # here for it to talk to that differs, and a second
            # implementation would only be a second thing to drift.
            self.observer = Observer(self)
            self.power = SimulatedPower()
            self.capture = SimulatedCapture()
            self.clock = SimulatedClock()
            self.daq = SimulatedDaq()
            # ONE TIMEBASE, as the board has one. A record's `at` and
            # the clock's cycles both come off DWT->CYCCNT there; here
            # they were two counters that started at zero at different
            # moments, so `sync.to_host()` mapped a DAQ stamp through a
            # calibration made for the other - measured, a DataFrame
            # indexed in the year 2083.
            self.daq.clock = self.clock
            self.drive = SimulatedDrive()
            # WHERE IT LOOKS. The observer is given a SAMPLER - the
            # phase currents and whether the bridge is switching - and
            # works the dissipation out itself. It was handed a finished
            # power budget, which made it a formality: it was being told
            # the answer by the thing it was watching. On the board it is
            # this shape too - the firmware samples currents in the
            # control interrupt and the observer integrates what it saw,
            # and neither of them knows where the current came from.
            self.thermal._sample = self.drive.sample
            # AND WHAT IT DROPS. `board_thermal.c` calls
            # `Board_PwmDisable()` after every step where the budget says
            # tripped, guarded by `Board_PwmIsEnabled()` so a stage that
            # is already down is not dropped again - that guard is what
            # makes the trip counter count TRIPS and not steps. The same
            # guard is here.
            self.thermal._gate = self._drop_stage
            # And what the drive reports as switching: the bridge, so a
            # dropped stage stops making current in the model too.
            self.drive._switching = lambda: self.gate_drivers._enabled
            # AND THE HAND ON THE THROTTLE. `Board_DriveDerate()` there;
            # a scaling on the drive's own clamp here.
            self.thermal._derate_to = self._derate_drive
            self.thermal._duty = self._effective_duty
            # The sample point is one register: moving it through the
            # gate drivers moves the drive's moments too.
            self.gate_drivers._drive = self.drive
            # The drive is what the phases and the gates FOLLOW: a
            # record and the modulation that produced it come from
            # one electrical angle, or they are two inventions that
            # happen to be printed together.
            self.daq.drive = self.drive
            # The shaft sensor reads the SAME rotor: a servo closed over
            # the A1335 moves what the drive torques, or the loop it
            # closes is between two inventions. The DAQ's sensor fields
            # read the same parts the subsystems answer for.
            self.angle.drive = self.drive
            self.daq.angle = self.angle
            self.daq.imu = self.imu
            # `zero()` reads a channel, so it needs the board that
            # has them.
            self.calibration.board = self

    def __repr__(self):
        return '<SimulatedBoard - no port, no cable, invented values>'

    def _derate_drive(self, factor):
        """Scale the drive's current clamp. `Board_DriveDerate`'s twin."""
        self.drive._derate = max(0.0, min(1.0, factor))

    def _effective_duty(self):
        """What the compares hold, as a fraction of the period.

        The EFFECTIVE duty: what the clamp and the derate left, not what
        anything asked for. Zero with the stage down, because that is
        what the compares are worth then.
        """
        if not self.gate_drivers._enabled:
            return (0.0, 0.0, 0.0)
        period = float(self.gate_drivers.PERIOD or 1)
        return tuple(t / period for t in self.gate_drivers._duty)

    def _drop_stage(self):
        """Drop the gates for the thermal envelope. True if it did.

        The board's own `Board_PwmDisable()` behind the same
        `Board_PwmIsEnabled()` guard: a stage already down is not dropped
        again, so what the counter counts is trips.
        """
        if not self.gate_drivers._enabled:
            return False
        self.gate_drivers.disable()
        return True

    def close_binary(self):
        pass

    def broadcast(self, function, payload=b''):
        """Acted on by every simulated node, answered by none."""
        return None

    def request(self, *_a, **_k):
        from ..errors import DeviceStateError
        if getattr(self, 'unit', 1) == 0:
            raise DeviceStateError(
                'unit 0 is the broadcast address: every node acts on a '
                'broadcast and none answers it, so there is nothing to '
                'read back.')
        raise DeviceStateError('the simulated board answers through its '
                               'subsystems, not raw requests')


# Several of this board on one bus, which is what a machine built out of
# them looks like: same firmware, same commands, different unit id and a
# different thing bolted to the shaft. Non-contiguous on purpose - a scan
# that assumes 1..n is one that stops at the first gap.
#
# The joints are invented, like every other value in this file, and each
# one says so in its own description. What is not invented is the shape:
# one unit id per device, and identity is how a host tells them apart.
class SimulatedSession:
    """Drop-in for `coaxial_mcp.session.Session` that never opens a port.

    Same public shape - `.board`, `.info()`, `.close()`, `.reset()` - so
    `Toolbox` and every handler in `coaxial_mcp/tools.py` work against it
    without knowing the difference. `--simulated` on dbg.py is the only
    thing that decides which one gets built.
    """

    # Read by anything that must not mistake this for a board - see
    # `coaxial_mcp.tools._interface`, which used to decide from the port
    # and started calling a bus label an RS485 segment.
    simulated = True

    def __init__(self, port=None, baud=115200, unit=1, bus=DEFAULT_BUS,
                 **_kwargs):
        # Takes what a real Session takes, so a caller that always builds
        # "the session" the same way is one fewer branch to keep in step.
        # A real bus is a serial segment and its label is its port, so a
        # `port` that names a bus is taken as one.
        self.baud = baud
        self.bus = port if port in SIMULATED_BUSES else bus
        self.port = self.bus
        self.unit = int(unit)
        self._board = SimulatedBoard(self.unit, self.bus)
        # THE SESSION'S LINE REACHES THE BOARD. Without it a stand-in
        # asked to emulate a 10 Mbit/s segment still charged 115200
        # for every reply, and every throughput number off it was the
        # debug probe's whatever the caller asked for.
        self._board.baud = self.baud
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
