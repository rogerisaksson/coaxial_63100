"""A body: one joint feedback per node, the drives holding angles, driven by a sequence.

    body = Body(Nodes.discover(device=True))       # every node a joint, named where it sits
    print(body.prompt())                           # what a model is told
    out = body.run(program)                        # armed, run, disarmed; out.summary()

A joint `knee`: setpoint `knee` (deg from where it detented at arm) -> Slew -> AngleHold ->
`knee.theta` -> the drive's HOLD angle; back from `knee.angle.degrees` through Wrap as
`knee.deg`, the joint angle a program tests. A target outside +/-`span` is refused before
anything moves; a joint 10 deg past it trips the run.
"""
import time

from coaxial.control.controller import Feedback
from coaxial.control.parts import AngleHold, Slew, Wrap
from coaxial.control.sequencer import Sequencer, prompt


class Body:

    """Joints over nodes: a Loop with one Feedback each, armed and disarmed as one."""

    def __init__(self, nodes, joints=None, amps=2.0, deg_s=90.0, ki=0.0, rate_hz=25.0,
                 arming=None, span=90.0):
        self.nodes, self.amps = nodes, float(amps)
        self.joints = list(joints or [node.name for node in nodes])
        self.ranges = {j: (-span, span) for j in self.joints}
        self.limits = {j + '.deg': {'LL': -span - 10.0, 'HH': span + 10.0} for j in self.joints}
        self.arming = arming if arming is not None else (
            {'bypass_sto': True, 'ignore_interlock': True} if self._simulated() else {})
        self.loop = nodes.loop(inputs=['%s.angle' % j for j in self.joints],
                               outputs=['%s.drive' % j for j in self.joints], rate_hz=rate_hz)
        for j in self.joints:
            poles = nodes[j].rig.drive.params().get('motor_pole_pairs') or 7
            self.loop.add(j, Feedback(AngleHold(poles, ki=ki), setpoint=j,
                                      measured=j + '.angle.degrees', command=j + '.theta',
                                      sink='%s.drive.theta' % j, prefilter=Slew(deg_s),
                                      measure=Wrap(), ref=j + '.ref', value=j + '.deg'))

    def _simulated(self):
        return all(node.rig.simulated for node in self.nodes)

    def arm(self, loop=None, steps=6, settle=0.05):
        """Every joint: the stage on, HOLD with the current ramped, zero where it detents."""
        drives = [self.nodes[j].rig.drive for j in self.joints]
        for j, drive in zip(self.joints, drives):
            rig = self.nodes[j].rig
            if rig.simulated:
                drive.configure(source='model')
                drive.model.configure(j=2e-5, b=1e-5, load=0.0)
            rig.gates.on(**self.arming)
            f = self.loop.feedbacks[j]
            f.regulator.configure(theta0=drive.state()['theta_hat'])
            drive.write(id_ref=self.amps / steps, iq_ref=0.0, theta=f.regulator.theta0,
                        omega_target=0.0)
            drive.hold()
        for k in range(2, steps + 1):
            time.sleep(settle)
            for drive in drives:
                drive.write(id_ref=self.amps * k / steps)
        time.sleep(2.0 * settle)
        for j in self.joints:
            f = self.loop.feedbacks[j]
            f.measure.configure(zero=self.nodes[j].rig.board.angle.state()['degrees'])
            f.prefilter.reset()
            f.regulator.reset()
        self.loop.write(**{j: 0.0 for j in self.joints})
        return self.joints

    def disarm(self, loop=None):
        """Every drive off, every stage down, a stand-in back on its converters."""
        for j in self.joints:
            rig = self.nodes[j].rig
            rig.drive.off()
            rig.gates.off()
            if rig.simulated:
                rig.drive.configure(source='adc')
        return self.joints

    def prompt(self):
        """The grammar and this body's joints: what a model is told."""
        return prompt(self.loop, {j: 'deg' for j in self.joints}, self.ranges)

    def run(self, text, **kw):
        """A program as text, on this body: checked, armed, run, disarmed however it ends."""
        return Sequencer.parse(text, ranges=self.ranges, limits=self.limits, init=self.arm,
                               cleanup=self.disarm, **kw).run(self.loop)

    def pose(self):
        """Every joint's angle now, deg from its zero."""
        bus = self.loop.read()
        return {j: bus.get(j + '.deg') for j in self.joints}
