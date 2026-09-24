"""The Coaxial63100 family for `machine`: a board as a node, the actuators it can be, discovery.

    from machine import Machine
    Machine.discover('humanoid', simulated=True)     # loads this module: a joint per Coaxial

A joint (deg, the drive holding an angle), a surface (a joint of narrow span), a rotor
(rpm, a speed loop), a torque (A, the current itself).
"""
import math

from coaxial.model.sensorless import RAD_S_PER_RPM
from machine.controller import Feedback
from machine.machine import Actuator
from machine.nodes import Module, Node
from machine.parts import AngleHold, Direct, Gain, Slew, SpeedPI, Wrap

#: The stand-in's joint damping, N.m.s: a gearbox and a limb, zeta ~0.5 on a 2 A hold of the
#: bench motor (k 0.735 N.m/rad, J 2e-5). The bare rotor's 1e-5 (zeta 0.0013) rings at 30 Hz
#: for seconds, and a 25 Hz loop pumps it until a pole slips (2026-09-24).
JOINT_B = 4e-3


def discover(port='COM4', simulated=False, units=range(1, 17), **kw):
    """Every Coaxial answering on every bus this host reaches, each opened as a node."""
    from coaxial import Coaxial63100
    first = Coaxial63100(port=port, simulated=simulated, **kw).open()
    found = [(bus, unit) for bus, _ in first.session.buses()
             for unit, _ in first.session.scan(units, bus)]
    first.close()
    return [Coaxial(Coaxial63100(port=bus, unit=unit, simulated=simulated, **kw).open())
            for bus, unit in found]


class _OnDrive(Actuator):

    """A feedback through the drive: armed through the gates, the drive and gates off at
    disarm. On the stand-in the gates arm past the STO chain and the interlock."""

    READS = DRIVES = 'drive'

    def _gates(self, arming):
        rig = self.node.rig
        if arming is None:
            arming = {'bypass_sto': True, 'ignore_interlock': True} if rig.simulated else {}
        rig.gates.on(**arming)

    def disarm(self):
        rig = self.node.rig
        rig.drive.off()
        rig.gates.off()
        if rig.simulated:
            rig.drive.configure(source='adc')

    @property
    def poles(self):
        return self.node.rig.drive.params().get('motor_pole_pairs') or 7


class Joint(_OnDrive):

    """An angle, deg from where it detented at arm: the drive's HOLD drags the rotor there."""

    UNIT, READS, BACK, ALIGNS = 'deg', 'angle', 'deg', True

    def __init__(self, node, span=90.0, deg_s=90.0, amps=2.0):
        super().__init__(node)
        self.half, self.deg_s, self.amps = float(span), float(deg_s), float(amps)

    def span(self):
        return (-self.half, self.half)

    def feedback(self, name):
        return Feedback(AngleHold(self.poles), setpoint=name,
                        measured=self.node.name + '.angle.degrees', command=name + '.theta',
                        sink='%s.drive.theta' % self.node.name, prefilter=Slew(self.deg_s),
                        measure=Wrap(), ref=name + '.ref', value=name + '.deg')

    def arm(self, f, arming=None):
        """Held at +90 deg electrical on a sixth of its amps: `ramp`, then `align`."""
        rig, drive = self.node.rig, self.node.rig.drive
        if rig.simulated:
            drive.configure(source='model')
            drive.model.configure(j=2e-5, b=JOINT_B, load=0.0)
        self._gates(arming)
        f.regulator.configure(theta0=drive.state()['theta_hat'])
        drive.write(id_ref=self.amps / 6.0, iq_ref=0.0, theta=f.regulator.theta0 + math.pi / 2,
                    omega_target=0.0)
        drive.hold()

    def ramp(self, k, steps):
        self.node.rig.drive.write(id_ref=self.amps * k / steps)

    def align(self, f):
        self.node.rig.drive.write(theta=f.regulator.theta0)

    def zero(self):
        return self.node.rig.board.angle.state()['degrees']


class Surface(Joint):

    """A control surface: a joint of narrow span."""

    def __init__(self, node, span=25.0, deg_s=120.0, amps=2.0):
        super().__init__(node, span, deg_s, amps)


class Rotor(_OnDrive):

    """A speed, rpm: a speed loop over the drive's observer, iq out."""

    UNIT, BACK = 'rpm', 'rpm'

    def __init__(self, node, rpm_max=6000.0, rpm_s=3000.0, amps=5.0, hz=3.0):
        super().__init__(node)
        self.rpm_max, self.rpm_s, self.amps, self.hz = (float(v) for v in
                                                         (rpm_max, rpm_s, amps, hz))

    def span(self):
        return (0.0, self.rpm_max)

    def feedback(self, name):
        p = self.node.rig.drive.params()
        kt = 1.5 * self.poles * p.get('motor_lambda_uvs', 0.005)
        return Feedback(SpeedPI(self.hz, self.amps, kt, 2e-5, 1e-5, scale=RAD_S_PER_RPM),
                        setpoint=name, measured=self.node.name + '.drive.omega_hat',
                        command=name + '.iq', sink='%s.drive.iq_ref' % self.node.name,
                        prefilter=Slew(self.rpm_s), measure=Gain(1.0 / (self.poles * RAD_S_PER_RPM)),
                        ref=name + '.ref', value=name + '.rpm')

    def arm(self, f, arming=None):
        rig, drive = self.node.rig, self.node.rig.drive
        if rig.simulated:
            drive.configure(source='model')
            drive.model.configure(j=2e-5, b=1e-5, load=0.0)
        self._gates(arming)
        drive.write(id_ref=0.0, iq_ref=0.0)
        drive.on('sensorless')


class Torque(Rotor):

    """A current, A: the setpoint straight to iq, the speed read back as rpm too."""

    UNIT, BACK = 'A', 'amps'

    def __init__(self, node, amps=5.0, a_s=10.0):
        super().__init__(node, amps=amps)
        self.a_s = float(a_s)

    def span(self):
        return (0.0, self.amps)

    def feedback(self, name):
        return Feedback(Direct(self.amps), setpoint=name,
                        measured=self.node.name + '.drive.iq', command=name + '.iq',
                        sink='%s.drive.iq_ref' % self.node.name, prefilter=Slew(self.a_s),
                        measure=Gain(1.0), ref=name + '.ref', value=name + '.amps')


class Coaxial(Node):

    """An opened Coaxial63100: modules drive (state in, setpoints out), angle, imu,
    thermal, power; named where it sits unless named."""

    UNITS = dict(Node.UNITS, **{
        'omega_hat': 'rad/s', 'omega_cmd': 'rad/s', 'omega_target': 'rad/s', 'accel': 'rad/s2',
        'theta_hat': 'rad', 'theta_cmd': 'rad', 'theta': 'rad', 'eps': 'rad',
        'id': 'A', 'iq': 'A', 'ih': 'A', 'eps_amps': 'A', 'id_ref': 'A', 'iq_ref': 'A',
        'pol_pos': 'A', 'pol_neg': 'A', 'vd': 'V', 'vq': 'V', 'vdc': 'V', 'e_bemf': 'V',
        'pol_volts': 'V', 'pol_periods': 'periods', 'pol_gap': 'periods', 'ts': 's',
        'ntc': 'C', 'afe': 'C', 'mcu': 'C', 'ambient': 'C', 'expected_ntc': 'C', 'error': 'K'})
    ACTUATORS = {'joint': Joint, 'surface': Surface, 'rotor': Rotor, 'torque': Torque}

    def __init__(self, rig, name=None):
        identity = rig.board.system.version()
        i_max = rig.drive.params().get('drv_i_max_ma')
        currents = {'id_ref': (-i_max, i_max), 'iq_ref': (-i_max, i_max)} if i_max else {}
        board = rig.board
        super().__init__(
            name or (identity.get('where') or 'unit%d' % rig.origin.unit).replace(' ', '_'),
            {'type': identity.get('type'), 'device': identity.get('device'),
             'where': identity.get('where'), 'link': rig.origin.port, 'unit': rig.origin.unit},
            {'drive': Module(rig.drive.state, rig.drive, rig.drive.WRITES, currents),
             'angle': Module(board.angle.state),
             'imu': Module(board.imu.state),
             'thermal': Module(board.thermal.state),
             'power': Module(board.power.state)})
        self.rig = rig

    def close(self):
        self.rig.close()
