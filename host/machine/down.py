"""Down: the fall braked, her drives cut to zero and checked before she gets up.

Lain still, every drive is cut, then a bus's boards check themselves one after another
(autodetect); given up, she is cut while still and braked while she moves.

    d = down.Down(world)               # landed: braked as the fall left her
    d.step(dt, bus)                    # True while she is cut; d.mode
"""
import math

from machine import drives
from machine.figure import JOINTS

#: Still STILL_S - the pelvis under STILL_M_S and turning under STILL_DEG_S - every drive is cut:
#: its gates off, no torque, nothing switched (`World.off`); moving again, braked - its phases
#: shorted - till still once more. Lying, the page drew 120 W: 1 s after her landing her tuck
#: (`falls.TUCKED`) armed every drive and held her curled, 136-163 W; given up she stayed
#: falling, holding it, 157 W (2026-10-02).
STILL_S, STILL_M_S, STILL_DEG_S = 0.3, 0.05, 10.0

#: Cut, she settles - her limbs come to rest as their drives let go -, braked again only past
#: TUMBLE_M_S or TUMBLE_DEG_S: braked at stillness's own 0.05 m/s, on a lace she was cut and
#: braked again 18 times in 7 s (2026-10-02).
TUMBLE_M_S, TUMBLE_DEG_S = 0.2, 45.0

#: Cut, each bus's boards check themselves one after another, CHECK_S each, the buses at once: the
#: winding and the angle sensor on a d-axis current of CHECK_SHARE of the board's amps - no torque,
#: its switching and copper drawn (`World.test`). The world breaks nothing; by their peaks
#: (`World.geared`) six falls of six put a gearbox past its momentary rating (`drives.shock`),
#: a knee 3.4 times and an ankle 3.5 on a lace (2026-10-02).
CHECK_S, CHECK_SHARE = 0.2, 0.2


class Down:

    """Her drives while she is down: 'braking' as the fall left them, 'checking' each, then
    'checked'; not to be checked, 'cut' - still, cut; moving, braked."""

    def __init__(self, world, check=True):
        self.world, self.check, self.mode, self.still, self.at = world, check, 'braking', 0.0, 0.0
        driven = [i for i, j in enumerate(JOINTS) if not drives.passive(j)]
        each = world.buses.each if world.buses is not None else ()
        self.order = [[i for i in b.indices if i in driven] for b in each] or [[i] for i in driven]

    def step(self, dt, bus):
        """The pass, the pelvis's speed and turn as `bus` reads them: still, every drive cut and
        checked in turn; moving, every drive braked. True while she is cut."""
        w = self.world
        moving = math.sqrt(sum(bus['pelvis.pose.v' + a] ** 2 for a in 'xyz'))
        turning = math.degrees(math.sqrt(sum(bus['pelvis.pose.w' + a] ** 2 for a in 'xyz')))
        self.still = self.still + dt if moving < STILL_M_S and turning < STILL_DEG_S else 0.0
        if self.mode != 'braking' and (moving > TUMBLE_M_S or turning > TUMBLE_DEG_S):
            for i in range(len(JOINTS)):
                w.short(i)
            self.mode, self.still = 'braking', 0.0
        if self.mode == 'braking' and self.still < STILL_S:
            return False
        if self.mode == 'braking':
            for i in range(len(JOINTS)):
                w.off(i)
            self.mode, self.at = 'checking' if self.check else 'cut', 0.0
        if self.mode == 'checking':
            slot = int(self.at / CHECK_S)
            for on in self.order:
                for k, i in enumerate(on):
                    w.test(i, CHECK_SHARE * drives.of(JOINTS[i])[1].amps if k == slot else 0.0)
            self.at += dt
            if slot >= max(len(on) for on in self.order):
                self.check, self.mode = False, 'checked'
        return True
