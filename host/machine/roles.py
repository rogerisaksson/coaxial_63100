"""What every device is - an input, an output, or a controller of both - and its verbs.

    Endpoint    state()  configure(**settings)  fault()  clear()  hold()  resume()
    Input       + read(count, timeout)  trigger()
    Stream      Input that runs once started: + start()  stop()  collect(count, timeout)
    Output      + write(**values)  on()  off()  is_on()  trip()
    Controller  Input + Output, + move(**targets)

The parts a controller is wired from (`machine.controller.Loop`), every port a float:

    Part        INPUTS, OUTPUTS, PARAMS; step(dt, **inputs) -> {output: float}; reset();
                params(); configure(**params)
    Filter      x -> y                                a prefilter, a gain
    Estimator   measured, command -> estimate
    Regulator   setpoint, measured -> command

One verb, one meaning, on every device. A verb a device cannot do refuses
in words (MachineError); what a device adds beyond the verbs is a setting of
`configure`, or a named method of its own.
"""
import time
from collections import namedtuple

from machine.errors import MachineError

#: One run of a stream: what it read, its state at the end, how long it took.
Run = namedtuple('Run', 'records state seconds')


class Endpoint:

    """Anything on the board or the host that runs."""

    def state(self) -> dict:
        """Everything it knows about itself, in one read."""
        raise NotImplementedError

    def configure(self, **settings):
        """Replace its settings."""
        raise MachineError('%s takes no settings' % type(self).__name__)

    def fault(self):
        """What stops it, in its own words; None while well."""
        return self.state().get('fault') or None

    def clear(self):
        """Clear a latched fault."""
        raise MachineError('%s latches no fault' % type(self).__name__)

    def hold(self):
        """Freeze where it is: a sensor's poll loop, an output's value, a shaft."""
        raise MachineError('%s has no hold' % type(self).__name__)

    def resume(self):
        """Carry on after `hold`."""
        raise MachineError('%s has no hold' % type(self).__name__)


class Input(Endpoint):

    """Something read: a channel, a sensor, a stream of records."""

    def read(self, count=None, timeout=None):
        """Its value now, or `count` values within `timeout` seconds."""
        raise NotImplementedError

    def trigger(self):
        """Take one acquisition now."""
        raise MachineError('%s takes no trigger' % type(self).__name__)


class Stream(Input):

    """An input that runs on its own once started, into a ring."""

    def start(self, *args, **kwargs):
        raise NotImplementedError

    def stop(self):
        raise NotImplementedError

    def collect(self, count=-1, timeout=None):
        """Start, read `count` within `timeout`, stop: one `Run`."""
        began = time.time()
        self.start()
        try:
            records = self.read(count, timeout)
            state = self.state()
        finally:
            self.stop()
        return Run(records, state, time.time() - began)


class Output(Endpoint):

    """Something written: a rail, a pin, a stage. `WRITES`: the keys write() takes, when fixed."""

    WRITES = ()

    def write(self, **values):
        """Put values out."""
        raise NotImplementedError

    def on(self):
        """To its working state."""
        raise MachineError('%s has no on' % type(self).__name__)

    def is_on(self):
        return bool(self.state().get('on'))

    def off(self):
        """To its safe state."""
        raise MachineError('%s has no off' % type(self).__name__)

    def trip(self):
        """To its safe state now, latched as a fault."""
        raise MachineError('%s has no trip' % type(self).__name__)


class Controller(Input, Output):

    """Reads its plant, writes its actuator, closes a loop between them."""

    def move(self, **targets):
        """Go to the targets: a position, a speed, a current."""
        raise MachineError('%s does not move' % type(self).__name__)


class Part:

    """A block of floats: named ports, float parameters, one step. A subclass that steps
    is a kind (`KINDS`) a panel offers and a saved loop is rebuilt from, its constructor
    taking its PARAMS by name, each with a default."""

    INPUTS, OUTPUTS, PARAMS = (), (), ()
    KINDS = {}

    def __init_subclass__(cls, **kw):
        super().__init_subclass__(**kw)
        if cls.step is not Part.step:
            Part.KINDS[cls.__name__] = cls

    def step(self, dt, **inputs):
        raise NotImplementedError

    def reset(self):
        pass

    def params(self):
        return {name: float(getattr(self, name)) for name in self.PARAMS}

    def configure(self, **params):
        unknown = sorted(set(params) - set(self.PARAMS))
        if unknown:
            raise MachineError('%s has no %s - its parameters are %s' % (
                type(self).__name__, ', '.join(unknown), ', '.join(self.PARAMS) or 'none'))
        for name, value in params.items():
            setattr(self, name, float(value))
        return self.params()


class Filter(Part):

    """x in, y out: a setpoint shaped, a measurement scaled."""

    INPUTS, OUTPUTS = ('x',), ('y',)


class Estimator(Part):

    """The measurement and the last command in, the estimate out."""

    INPUTS, OUTPUTS = ('measured', 'command'), ('estimate',)


class Regulator(Part):

    """The setpoint and the measurement (or its estimate) in, the command out."""

    INPUTS, OUTPUTS = ('setpoint', 'measured'), ('command',)
