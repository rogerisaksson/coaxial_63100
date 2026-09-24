"""What every device is - an input, an output, or a controller of both - and its verbs.

    Endpoint    state()  configure(**settings)  fault()  clear()
    Input       + read(count, timeout)  trigger()
    Output      + write(**values)  on()  off()  is_on()  trip()  hold()  resume()
    Controller  Input + Output, + move(**targets)
    Regulator   Controller on the host: step(measured) -> command, reset()
    Estimator   Controller on the host: step(measured) -> estimate, reset()

One verb, one meaning, on every device. A verb a device cannot do refuses
in words (RigError); what a device adds beyond the verbs is a setting of
`configure`, or a named method of its own.
"""
from coaxial.errors import RigError


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


class Input(Endpoint):

    """Something read: a channel, a sensor, a stream of records."""

    def read(self, count=None, timeout=None):
        """Its value now, or `count` values within `timeout` seconds."""
        raise NotImplementedError

    def trigger(self):
        """Take one acquisition now."""
        raise RigError('%s takes no trigger' % type(self).__name__)


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

    def hold(self):
        """Keep what it is doing; take no new writes."""
        raise RigError('%s has no hold' % type(self).__name__)

    def resume(self):
        """Take writes again after `hold`."""
        raise RigError('%s has no hold' % type(self).__name__)


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
