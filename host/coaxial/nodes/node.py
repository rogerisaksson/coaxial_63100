"""A board as an IO node: modules of float channels, asked for, then configured into loops."""
from collections import namedtuple

from coaxial.control.controller import Polled, flat
from coaxial.devices.roles import Endpoint

#: One channel a node offers: its name ('<node>.<module>.<key>'), 'in' or 'out', its unit,
#: its range (None where the board sets none).
Channel = namedtuple('Channel', 'name direction unit low high')

#: Units by key, where the key says it.
UNITS = {'omega_hat': 'rad/s', 'omega_cmd': 'rad/s', 'omega_target': 'rad/s', 'accel': 'rad/s2',
         'theta_hat': 'rad', 'theta_cmd': 'rad', 'theta': 'rad', 'eps': 'rad',
         'id': 'A', 'iq': 'A', 'ih': 'A', 'eps_amps': 'A', 'id_ref': 'A', 'iq_ref': 'A',
         'pol_pos': 'A', 'pol_neg': 'A', 'vd': 'V', 'vq': 'V', 'vdc': 'V', 'e_bemf': 'V',
         'pol_volts': 'V', 'pol_periods': 'periods', 'pol_gap': 'periods', 'ts': 's',
         'degrees': 'deg', 'ntc': 'C', 'afe': 'C', 'mcu': 'C', 'ambient': 'C',
         'expected_ntc': 'C', 'error': 'K', 'seconds': 's'}


class Module:

    """One of a node's IO modules: a read, and the writer and keys it takes, if any."""

    def __init__(self, read=None, writer=None, writes=()):
        self.read, self.writer, self.writes = read, writer, tuple(writes)


class Node(Endpoint):

    """One board: `capabilities()` first, then `source(module)` and `sink(module)` for a
    loop. Modules: drive (state in, setpoints out), angle, imu, thermal, power."""

    def __init__(self, rig, name=None):
        self.rig = rig
        identity = rig.board.system.version()
        self.name = name or (identity.get('where') or 'unit%d' % rig.origin.unit).replace(' ', '_')
        self.identity = {'device': identity.get('device'), 'where': identity.get('where'),
                         'link': rig.origin.port, 'unit': rig.origin.unit}
        board = rig.board
        self.modules = {
            'drive': Module(rig.drive.state, rig.drive, rig.drive.WRITES),
            'angle': Module(board.angle.state),
            'imu': Module(board.imu.state),
            'thermal': Module(board.thermal.state),
            'power': Module(board.power.state),
        }

    def state(self):
        return dict(self.identity, name=self.name, modules=list(self.modules))

    def _limits(self):
        """What the record bounds: the currents at the drive's i_max."""
        i_max = self.rig.drive.params().get('drv_i_max_ma')
        return {'id_ref': (-i_max, i_max), 'iq_ref': (-i_max, i_max)} if i_max else {}

    def capabilities(self, *modules):
        """Every channel of the named modules (all by default), each read once."""
        limits = self._limits()
        out = []
        for name in modules or self.modules:
            module = self.modules[name]
            prefix = '%s.%s.' % (self.name, name)
            if module.read is not None:
                out += [Channel(prefix + key, 'in', UNITS.get(key.rpartition('.')[2], ''),
                                None, None) for key in sorted(flat(module.read() or {}))]
            out += [Channel(prefix + key, 'out', UNITS.get(key, ''), *limits.get(key, (None, None)))
                    for key in module.writes]
        return out

    def source(self, module):
        """The module as a loop's source."""
        return Polled(self.modules[module].read)

    def sink(self, module):
        """The module as a loop's sink."""
        return self.modules[module].writer

    def close(self):
        self.rig.close()
