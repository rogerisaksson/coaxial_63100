"""What every device is - an input, an output, or a controller of both - and its verbs.

    Endpoint    state()  configure(**settings)  fault()  clear()  hold()  resume()
    Input       + read(count, timeout)  trigger()
    Stream      Input that runs once started: + start()  stop()  collect(count, timeout)
    Output      + write(**values)  on()  off()  is_on()  trip()
    Controller  Input + Output, + move(**targets)
    Regulator   Controller on the host: step(measured) -> command, reset()
    Estimator   Controller on the host: step(measured) -> estimate, reset()

One verb, one meaning, on every device. A verb a device cannot do refuses
in words (RigError); what a device adds beyond the verbs is a setting of
`configure`, or a named method of its own.
"""
import time
from collections import namedtuple

from coaxial.errors import RigError

#: One run of a stream: what it read, its state at the end, how long it took.
Run = namedtuple('Run', 'records state seconds')


class Endpoint:

    """Anything on the board or the host that runs."""

    def state(self) -> dict:
        """Everything it knows about itself, in one read."""
        raise NotImplementedError

    def configure(self, **settings):
        """Replace its settings."""
        raise RigError('%s takes no settings' % type(self).__name__)

    def fault(self):
        """What stops it, in its own words; None while well."""
        return self.state().get('fault') or None

    def clear(self):
        """Clear a latched fault."""
        raise RigError('%s latches no fault' % type(self).__name__)

    def hold(self):
        """Freeze where it is: a sensor's poll loop, an output's value, a shaft."""
        raise RigError('%s has no hold' % type(self).__name__)

    def resume(self):
        """Carry on after `hold`."""
        raise RigError('%s has no hold' % type(self).__name__)


class Input(Endpoint):

    """Something read: a channel, a sensor, a stream of records."""

    def read(self, count=None, timeout=None):
        """Its value now, or `count` values within `timeout` seconds."""
        raise NotImplementedError

    def trigger(self):
        """Take one acquisition now."""
        raise RigError('%s takes no trigger' % type(self).__name__)


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

    """Something written: a rail, a pin, a stage."""

    def write(self, **values):
        """Put values out."""
        raise NotImplementedError

    def on(self):
        """To its working state."""
        raise RigError('%s has no on' % type(self).__name__)

    def is_on(self):
        return bool(self.state().get('on'))

    def off(self):
        """To its safe state."""
        raise NotImplementedError

    def trip(self):
        """To its safe state now, latched as a fault."""
        raise RigError('%s has no trip' % type(self).__name__)


class Controller(Input, Output):

    """Reads its plant, writes its actuator, closes a loop between them."""

    def move(self, **targets):
        """Go to the targets: a position, a speed, a current."""
        raise RigError('%s does not move' % type(self).__name__)


class Regulator(Controller):

    """A loop computed on the host: a measurement in, a command out."""

    def step(self, *measured):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError


class Estimator(Controller):

    """A filter computed on the host: a measurement in, an estimate out."""

    def step(self, *measured):
        raise NotImplementedError

    def reset(self):
        raise NotImplementedError
