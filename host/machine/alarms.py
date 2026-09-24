"""Alarms over a loop's channels: levels, a log, a stop - beside the sequencer, not in it.

    alarms = Alarms({'iq_ref': {'HH': 1.9}})       # the levels in force from the start
    Sequencer.parse(text, alarms=alarms).run(loop)  # it calls the hooks below every pass
    alarms.log                                      # a line an alarm
    alarms.stop('operator')                         # from any thread: the run trips

L and H alarm: logged when one comes and when it goes (`ok`), the run going on; it goes
only once the value is back past its bound by DEADBAND of it, so a value on the bound
cannot chatter; `active` holds the ones that are on. LL and HH trip: `check` raises
Tripped and the sequencer goes to cleanup. A row's level columns arrive through `set`
and hold from there (inf clears one); a step's timeout through `timeout`. Whatever else
watches - a pack, a camera, an operator - ends a run the same way: `stop(why)`. A
handler is any object with these hooks; one of your own subclasses Alarms and calls
`super().check` first.
"""
from machine.errors import MachineError

LEVELS = ('LL', 'L', 'H', 'HH')

#: How far back past its bound a value must come for its alarm to go: this share of the
#: bound, at least DEADBAND_FLOOR.
DEADBAND, DEADBAND_FLOOR = 0.01, 1e-6


class Tripped(MachineError):
    """A channel past its LL or HH, or a stop."""


def crossed(value, level, bound):
    """Whether `value` is past `level` (LL L below, H HH above; GE LE as tests)."""
    return value >= bound if level in ('H', 'HH', 'GE') else value <= bound


class Alarms:

    """Levels by channel ({channel: {'HH': 1.9}}), the log, the active alarms, a stop; the
    sequencer's hooks: `begin` a run, `step` a step, `set` a row's levels, `check` a pass,
    `timeout`."""

    def __init__(self, levels=None):
        #: The levels a run begins with - the caller's own dict, read at each `begin`.
        self.base = levels if levels is not None else {}
        self.levels, self.log = {}, []
        #: {(channel, level): the value it came at} - the alarms on now.
        self.active = {}
        self._where, self._stop = '', None
        self.begin()

    def begin(self):
        """A run from the start: the base levels, an empty log, no stop."""
        self.levels = {ch: dict(lv) for ch, lv in self.base.items()}
        self.log, self.active, self._stop = [], {}, None
        return 0

    def step(self, where):
        """A step begins: what the log says it happened in."""
        self._where = where

    def set(self, levels):
        """A row's levels, in force from here: {channel: {level: bound}}."""
        for channel, lv in levels.items():
            self.levels.setdefault(channel, {}).update(lv)

    def check(self, bus, trips=True):
        """Every level against `bus`: an alarm logged as it comes and as it goes; LL, HH
        and a stop raise Tripped when `trips` (cleanup logs them, never trips)."""
        if self._stop is not None and trips:
            raise Tripped(self._stop)
        for channel, lv in self.levels.items():
            value = bus.get(channel)
            if value is None:
                continue
            for level, bound in lv.items():
                key = (channel, level)
                if crossed(value, level, bound):
                    said = '%s %s %.4g past %.4g' % (level, channel, value, bound)
                    if key not in self.active:
                        self.active[key] = value
                        self._say(said)
                    if trips and len(level) == 2:
                        raise Tripped(said)
                elif key in self.active:
                    band = max(DEADBAND_FLOOR, DEADBAND * abs(bound))
                    back = bound - band if level in ('H', 'HH') else bound + band
                    if not crossed(value, level, back):
                        del self.active[key]
                        self._say('ok %s %s %.4g' % (level, channel, value))

    def timeout(self, what):
        """A step waited out its time: logged; the sequencer goes on."""
        self._say('timeout ' + what)

    def stop(self, why):
        """End the run at its next check, from any thread: to cleanup. The first stop is the
        one kept and logged; a watcher that keeps asking changes nothing."""
        if self._stop is None:
            self._stop = 'stop: %s' % why
            self._say(self._stop)

    def _say(self, what):
        self.log.append('%s: %s' % (self._where, what) if self._where else what)
