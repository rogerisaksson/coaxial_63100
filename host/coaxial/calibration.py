"""The board's calibration record: the scaling parameters and the per-channel
corrections, read and written where they live.

This module holds no numbers. The defaults are the firmware's, the stored
values are the board's, and a host that kept its own copy would be a host that
answers for the wrong board the moment it is pointed at a second one.

Two ways to correct a channel, and they are not interchangeable:

  * `zero()` - input at zero, the code it reads becomes the origin;
  * `span()` - a known reference applied, told to the board in the channel's
    own unit, and the gain follows.

Zero first. Spanning against an un-zeroed channel folds the offset into the
gain, which then looks right at the reference point and nowhere else.
"""
import re

from . import protocol
from .afe import powered
from .errors import DeviceStateError, ModbusException, RigError
from .protocol import CalOp
from .subsystem import Device, forgetting, remembered
from .wire import Reader, label, pack, pages
from typing import Any


class CalibrationOps:
    """Zero and span by name, over whatever answers `set_channel`.

    A MIXIN AND NOT A SECOND COPY. The real record and the stand-in
    both have to answer these, and the parity suite fails a name
    that exists on one side only - so the arithmetic lives once and
    both inherit it. What differs between them is `set_channel`,
    `read` and `zero`, which is exactly the wire.
    """

    #: What the concrete class brings: the board the channels are read
    #: through, and the record's own read, write and save. Declared so
    #: the mixin's methods read as calls on something that exists.
    board: Any

    def read(self, *args, **kwargs):
        raise NotImplementedError

    def set_channel(self, index, offset_raw, gain_ppm):
        raise NotImplementedError

    def zero(self, index):
        raise NotImplementedError

    def save(self):
        raise NotImplementedError

    def compensate(self, name, gain=None, offset=None, save=True):
        """Write one channel's gain and offset, by name.

            board.calibration.compensate('phaseU', gain=1.002, offset=-7155)

        `set_channel()` takes an index and both numbers; this takes the
        board's own channel name and lets either stand, so a span does not
        have to restate a zero it must not disturb.

        CLASSIC OFFSET AND GAIN, in the order the board applies them:
        `(code - offset) * gain`. `gain` is a plain multiplier here and
        parts per million on the wire, because 1.002 is what an operator
        means and 2000 is what the record stores.

        Returns `{'offset_raw', 'gain_ppm'}` as stored.
        """
        index = self._index_of(name)
        was = {c['index']: c for c in self.read()['channels']}.get(index, {})
        offset_raw = (was.get('offset_raw') or 0 if offset is None
                      else int(round(offset)))
        gain_ppm = (was.get('gain_ppm') or 0 if gain is None
                    else int(round((float(gain) - 1.0) * 1e6)))
        self.set_channel(index, offset_raw, gain_ppm)
        if save:
            self.save()
        return {'offset_raw': offset_raw, 'gain_ppm': gain_ppm}

    @powered
    def tare(self, *names, **kw):
        """Zero channels: what they read now becomes zero.

            board.calibration.tare('phaseU', auto=True, save=False)
            board.calibration.tare()      # every current channel, saved

        A MEASUREMENT AND THEN A `compensate()`. With `auto` it reads a
        BURST here - a zero taken from one conversion carries that
        conversion's noise into every reading after it - and writes the
        mean as the offset, leaving the gain alone. With `auto=False` it
        asks the board to do both in one op, which is what `zero()` is:
        the same answer in one round trip, and no window in which the host
        holds a number the board has not agreed to.

        NOTHING HERE KNOWS WHAT IS ON THE INPUT. Taring a live channel
        stores a live reading as zero, which is the operator's mistake to
        make; the codes returned are what make it visible.

        Refused with the AFE off (`powered`): it powers the converter's
        reference, so every channel reads exact mid-scale and a tare
        against that writes a plausible number that means nothing
        (invariant 9).

        Returns `{name: code}`.
        """
        auto = kw.pop('auto', True)
        save = kw.pop('save', True)
        if kw:
            raise TypeError('tare() got %s' % ', '.join(sorted(kw)))

        rows = self.board.system.channel_map()['analog']
        wanted = ([self._spell(n) for n in names] if names
                  else [r['signal'] for r in rows if r.get('unit') == 'mA'])
        take = self._tare_here if auto else self._tare_there
        got = {name: take(name) for name in wanted}
        if save:
            self.save()
        return got

    def _tare_here(self, name):
        """A burst's mean, measured on this side and written as the offset."""
        code = self._burst_mean(name)
        self.compensate(name, offset=code, save=False)
        return code

    def _tare_there(self, name):
        """The board's own zero, one round trip."""
        return self.zero(self._index_of(name))

    # -- naming, which is the board's ------------------------------------

    def _rows(self):
        return self.board.system.channel_map()['analog']

    @staticmethod
    def _match(name):
        return re.sub(r'[^a-z0-9]', '', str(name).lower())

    def _spell(self, name):
        """The board's own spelling of `name`, or a raise naming its list."""
        want = self._match(name)
        spelled = next((row['signal'] for row in self._rows()
                        if self._match(row['signal']) == want), None)
        if spelled is None:
            raise RigError('no channel called %r. This board has: %s'
                           % (name, ', '.join(r['signal']
                                              for r in self._rows())))
        return spelled

    def _index_of(self, name):
        spelling = self._spell(name)
        return next(row['index'] for row in self._rows()
                    if row['signal'] == spelling)

    def _burst_mean(self, name):
        """One channel's code, meaned over a burst, for a tare to keep."""
        code = next((row['mean_raw']
                     for row in self.board.analog.read_all()['channels']
                     if row['signal'] == name), None)
        if code is None:
            raise RigError('%r is not a channel this board reads' % name)
        return int(round(code))


class Calibration(CalibrationOps, Device, device=protocol.DEVICE_CAL):

    """Device 3 behind 0x6E. Edits are volatile until save()."""

    # A 128 KB sector erase is specified at up to 4 s on this silicon, and the
    # board answers save() only once it has erased, reprogrammed and read
    # back. The default 0.5 s budget would time out on a save that worked.
    SAVE_TIMEOUT = 6.0

    @remembered
    def read(self):
        """The whole record, plus whether flash holds one.

        Cached, and every writer below forgets it - a cache the writers do
        not clear is worse than no cache: it hands back a record the board
        no longer holds, and nothing says the two have parted company.
        `refresh=True` goes to the board regardless. MEASURED: uncached, a
        scaled frame asked for the channel trims and paid a FULL ROUND TRIP
        per frame for a record nobody had touched - eight a second in a live
        plot, on the link this tree spent a day widening. Invisible against
        the stand-in, where the call is local.

        `stored` false means these are the firmware's compiled-in defaults -
        the schematic's arithmetic, never measured. It is the difference
        between a calibrated board and an uncalibrated one, and nothing else
        in the reply shows it.
        """
        reader = Reader(self._op(CalOp.GET))
        stored = bool(reader.u8())
        version = reader.u16()
        # Consume what the BOARD said it sent, not what this list happens to
        # name. A count read off the wire and then iterated over a local list
        # is how the reader fell four fields behind and stayed there.
        params = {self._name(i): reader.u32() for i in range(reader.u8())}
        channels = [{'index': index, 'offset_raw': reader.i32(),
                     'gain_ppm': reader.i32()}
                    for index in range(reader.u8())]
        # The thermal envelope. Centi-degrees per node, in thermal_node_t
        # order; zero means that node has no ceiling, which is what a node
        # with no measurement behind it should carry rather than a guess.
        limits = [reader.centi() for _ in range(reader.u8())]
        throttle = reader.micro('u32')
        params.update(self._paged(len(params)))
        return {'stored': stored, 'version': version,
                'params': params, 'channels': channels,
                'soa_limit_c': limits, 'soa_throttle_at': throttle}

    @staticmethod
    def _name(ident):
        return label(protocol.CAL_PARAMS, ident, 'param')

    def _paged(self, first):
        """The parameters past what op 0 carries, through op 8.

        Op 0 kept its first fifteen when the record grew to forty-five -
        the whole reply was 310 bytes against a 253-byte PDU. A firmware
        without op 8 answers ILLEGAL DATA VALUE and has nothing past the
        fifteen anyway.
        """
        return {self._name(i): page.u32()
                for page in pages(self._params_from, absent=ModbusException,
                                  first=first)
                for i in page.indices()}

    def _params_from(self, first):
        return self._op(CalOp.PARAMS, pack(('u8', first)))

    def _ident(self, name):
        """The record id of a parameter, or a raise naming them all."""
        if name not in protocol.CAL_PARAMS:
            raise DeviceStateError(
                '%r is not a calibration parameter. There are %d: %s'
                % (name, len(protocol.CAL_PARAMS),
                   ', '.join(protocol.CAL_PARAMS)))
        return protocol.CAL_PARAMS.index(name)

    @forgetting('read')
    def set_param(self, name, value):
        """One scalar, by the name read() returns it under."""
        self._op(CalOp.SET_PARAM,
                 pack(('u8', self._ident(name)), ('u32', int(value))))

    @forgetting('read')
    def set_channel(self, index, offset_raw, gain_ppm):
        """Both corrections for one channel, together.

        Together because they are applied together - offset first, then gain -
        and setting one while guessing the other is how a half-applied
        calibration happens.
        """
        self._op(CalOp.SET_CHANNEL,
                 pack(('u8', index), ('i32', int(offset_raw)),
                      ('i32', int(gain_ppm))))

    @forgetting('read')
    def zero(self, index):
        """Measure the channel now and keep the reading as its offset.

        Returns the code that was stored. The board does not know what is on
        the input - pointing it at a live one is the operator's mistake to
        make, and the returned code is what makes it visible.
        """
        return Reader(self._op(CalOp.ZERO, pack(('u8', index)))).i32()

    @forgetting('read')
    def span(self, index, reference):
        """Trim the gain so the channel reports `reference`.

        The reference is in the channel's own unit - milliamperes for a phase,
        millivolts for the DC link. Refused for the thermistor, whose
        conversion is logarithmic and has no scale factor, and for a channel
        reading zero, which no finite gain turns into something.
        """
        return Reader(self._op(CalOp.SPAN, pack(('u8', index),
                                                ('i32', int(reference))))).i32()

    @forgetting('read')
    def save(self):
        """Commit to flash. Erases and rewrites one sector, then reads back.

        Returns what the board said it did. It used to drop the reply and
        return None, which invariant 8 exists to forbid: every call produces
        its result or raises, and a caller cannot tell None from a habit.
        """
        return bool(Reader(self._op(CalOp.SAVE,
                                    timeout=self.SAVE_TIMEOUT)).u8())

    @forgetting('read')
    def load(self):
        """Re-read flash, discarding uncommitted edits."""
        return bool(Reader(self._op(CalOp.LOAD)).u8())

    @forgetting('read')
    def defaults(self):
        """Back to the firmware's compiled-in numbers. RAM only until save."""
        return bool(Reader(self._op(CalOp.DEFAULTS)).u8())
