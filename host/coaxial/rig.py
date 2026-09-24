"""One board behind one class: connect, configure, trigger, read."""
import sys
import time
import zlib
from contextlib import suppress

from coaxial.acquire.acquisition import Acquisition
from coaxial.acquire.clock import NTP_SERVER
from coaxial.acquire.stream import TaskStream
from coaxial.acquire.task import Task
from coaxial.comm import broker, session as sessionmod
from coaxial.control.motion import Motion
from coaxial.devices import boot as bootmod
from coaxial.devices.board import Board
from coaxial.devices.gates import GateStage
from coaxial.errors import LINK_FAULTS, RigError


def _subsystem_names():
    """The board's subsystem names, off its declaration, plus `gates` and
    `daq`.
    """
    return frozenset(Board.parts()) | {'gates', 'daq'}


class Later:

    """A subsystem named before its session is open."""

    def __init__(self, device, name):
        self._device, self._name = device, name

    def open(self):
        """Open the device this handle belongs to. Returns the live handle."""
        self._device.open()
        return getattr(self._device, self._name)

    def _live(self):
        if self._device.board is None:
            raise RigError(
                '%s is a handle on a session that is not open yet - '
                'open() on it, or on the device, is what makes it live'
                % self._name)
        return getattr(self._device, self._name)

    def __getattr__(self, attr):
        return getattr(self._live(), attr)

    def __repr__(self):
        if self._device.board is None:
            return ('<%s of a session not yet open - open() opens it>'
                    % self._name)
        return repr(self._live())


#: What the acquisition front door answers. A whitelist, not everything:
#: `daq.write` reaching the pin writer would put the device vocabulary
#: behind the wrong name.
DAQ_DOOR = ('configure', 'shape', 'ladder', 'tone', 'start', 'stop', 'collect', 'sweep_rate',
            'state', 'acquire', 'latest', 'blocks', 'read_buffer',
            'buffered', 'channels', 'outputs', 'catalogue', 'pick', 'read',
            'configure_buffer', 'capture', 'enable', 'disable',
            'channel_names', 'columns', 'series', 'frame', 'frames',
            'history')


class DaqView:

    """The acquisition front door, as its own handle."""

    def __init__(self, device):
        self._device = device

    def open(self):
        """Open the device this handle belongs to."""
        self._device.open()
        return self

    def close(self):
        """End the acquisition: the task stopped, what it buffered still
        readable.
        """
        self._device.stop()
        return self

    @property
    def layout(self):
        return self._device.layout

    def __getattr__(self, name):
        if name in DAQ_DOOR:
            return getattr(self._device, name)
        raise AttributeError(
            '%r is not part of the acquisition front door. It has: open, '
            'close, layout, %s' % (name, ', '.join(DAQ_DOOR)))

    def __enter__(self):
        """Start the task, and stop it on the way out however that goes."""
        self._device.start()
        return self

    def __exit__(self, *_):
        self._device.stop()
        return False

    def __repr__(self):
        return ('<the acquisition front door - configure(), then `with` it '
                'or start(); read()/series()/columns(); stop(), close()>')


class Coaxial63100(Task, TaskStream, Acquisition):

    """One board, one acquisition task, one clock."""

    def __init__(self, port='COM4', baud=115200, unit=1, link='auto',
                 simulated_device=False, power_afe=False, own_image=True):
        """Say where the board is. Nothing is opened until `open()`, which
        makes a real board run this host's own build (`own_image`)."""
        self.port = port
        self.baud = baud
        self.unit = unit
        self.link = link
        self.simulated_device = simulated_device
        self.power_afe = power_afe
        self.own_image = own_image
        #: (path of this host's build, whether open() loaded it), or None.
        self.image_loaded = None

        self.session = None
        self._board = None
        # No `self.gates = None` here: before open() the name goes through
        # __getattr__ like every subsystem, so `stage = device.gates` binds a
        # Later whose open() opens the device.
        self.daq = DaqView(self)
        # Motion, the same way: bound before open() so `device.motion` reads
        # like `device.daq`, opened lazily by its factories.
        self.motion = Motion(self)
        self._origin = None
        self.simulated = simulated_device
        self.layout = None
        self.sync = None
        # The stamps' wrap count, carried from block to block - `_epoch` - and
        # the last stamp itself, for a block of one record's dt.
        self._last_raw = None
        self._epoch = 0
        self._last_stamp = None
        self._afe_was_on = None
        #: Whether this session holds a reference on the AFE rail, so
        #: close() releases exactly what it took and no more.
        self._afe_held = False
        # The host-side reader, alive only between start() and stop().
        self._reader = None
        self._buffer_records = self.BUFFER_RECORDS
        self._history = []
        self._asked_done = 0.0
        self._done_seen = 0
        self._cursor = 0
        self._lost = 0

    # -- opening and closing --------------------------------------------

    def open(self):
        """Open the link, bring the supply up, and hand back self."""
        if self.session is not None:
            return self

        simulated = True if self.simulated_device else (
            None if self.link == 'auto' else False)

        self.session, self._origin = sessionmod.open_session(
            self.port, baud=self.baud, unit=self.unit, simulated=simulated)
        self._board = self.session.board
        self.gates = GateStage(self.board)
        # THE WAY BACK.
        self.board.rig = self
        self.simulated = not self.origin.real
        if self.own_image and not self.simulated:
            self._own_image()

        if self.power_afe:
            self._take_afe()
        return self

    def _own_image(self):
        """THE HOST'S OWN BUILD ON THE BOARD (docs/BOOT.md): a board running
        another image takes this host's through its bootloader, into RAM and
        its store - once per build - so nothing is asked of a firmware this
        host was not built with. A board others share is not reset under
        them: that is refused in words."""
        found = bootmod.host_image()
        if found is None:
            return                        # no build at hand: nothing to compare
        path, image = found
        want = (len(image), zlib.crc32(image))
        running = bootmod.stale(self.board, image)
        if running is None:
            self.image_loaded = (path, False)
            return
        if self.origin.label.endswith('- shared'):
            raise RigError('unit %d runs another image (%s) than this host\'s build %s '
                           '(%d B, crc %08x), and other sessions share the board - close '
                           'them, and the next open loads it'
                           % (self.unit, running, path, want[0], want[1]))
        print('coaxial: unit %d runs image %s; loading this host\'s build %s (%d B, crc '
              '%08x) through its bootloader' % ((self.unit, running, path) + want),
              file=sys.stderr)
        bootmod.load(self.board, image)
        self.board.probe()
        self.image_loaded = (path, True)

    #: What the parts need after their rail comes up before anything talks
    #: to them. Enabling and configuring in the same breath answered SERVER
    #: DEVICE FAILURE.
    AFE_SETTLE = 0.3

    def _take_afe(self):
        """This session's own reference on the rail, and the parts' settle when
        it was this that switched the rail on.
        """
        already = self.board.afe.is_on()
        self.board.afe.on()
        self._afe_held = True
        if not already:
            time.sleep(self.AFE_SETTLE)

    def __getattr__(self, name):
        """`device.imu` is `device.board.imu`, and it can be NAMED early."""
        if name.startswith('_') or name in ('board', 'session'):
            raise AttributeError(name)

        board = self.__dict__.get('_board')
        if board is None:
            return self._later(name)
        try:
            return getattr(board, name)
        except AttributeError:
            raise AttributeError(
                '%r is not a subsystem of this board. It has: %s'
                % (name, ', '.join(sorted(
                    n for n in vars(board) if not n.startswith('_')))))

    def _later(self, name):
        """A handle on a subsystem named before open(), checked against the
        board's declaration so a typo fails at the binding.
        """
        if name in _subsystem_names():
            return Later(self, name)
        raise AttributeError(
            '%r is not a subsystem of this board. It has: %s'
            % (name, ', '.join(sorted(_subsystem_names()))))

    def _others_here(self):
        """Whether another session is on this board. False if unknowable."""
        if self.simulated:
            return False
        try:
            count = broker.clients()          # None: nobody is serving
        except LINK_FAULTS + (ValueError,):   # the socket, the address file
            return False
        return count is not None and count > 1

    def close(self):
        """Everything this session started, undone."""
        if self._board is not None:
            # One try per step.
            for step in (self.stop, self._release_stage, self._release_afe):
                with suppress(RigError):
                    step()
        if self.session is not None:
            self.session.close()
        self.session = self._board = None
        vars(self).pop('gates', None)      # back to a Later, reopenable

    def _release_stage(self):
        """Disarm on the way out - the safety net for a run that was killed."""
        if (self.gates is None or self.gates.armed_here
                or not self._others_here()):
            self.board.gate_drivers.off()

    def _release_afe(self):
        """Release OUR reference; the refcount keeps the rail up for whoever
        else holds it.
        """
        if not self._afe_held:
            return
        self.board.afe.off()
        self._afe_held = False

    def __enter__(self):
        return self.open()

    def __exit__(self, *_):
        self.close()

    @property
    def board(self):
        """The board behind the session: what every subsystem holds and
        every page reads.
        """
        if self._board is None:
            raise RigError('the rig is not open - open() first, or use it '
                           'as a context manager')
        return self._board

    @property
    def origin(self):
        """Where the board was reached: `open_session`'s Origin, with its
        `interface`, `label` and `real`.
        """
        if self._origin is None:
            raise RigError('the rig is not open - open() first, or use it '
                           'as a context manager')
        return self._origin

    def __repr__(self):
        where = self._origin.label if self._origin else 'not open'
        return '<Coaxial63100 %s%s>' % (
            where, ' SIMULATED' if self.simulated else '')

    # -- the clock -------------------------------------------------------

    def set_time_from_pc(self, seconds=3.0, reference='utc',
                         ntp_server=NTP_SERVER):
        """Tie the board's cycle counter to a real clock."""
        self.sync = self.board.clock.sync(
            seconds=seconds, reference=reference,
            ntp_server=ntp_server)
        return self.sync

    #: Records the host keeps when nobody has said. Ten thousand at a
    #: fifty-byte stride is half a megabyte, which is nothing at this end
    #: of the link and several seconds of headroom at the other.
    BUFFER_RECORDS = 10000

