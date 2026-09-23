"""The board's calibration record: the scaling parameters and the per-channel
corrections, read and written where they live.
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
    """Zero and span by name, over whatever answers `set_channel`."""

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
        """Write one channel's gain and offset, by name."""
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
        """Zero channels: what they read now becomes zero."""
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
    # back.
    SAVE_TIMEOUT = 6.0

    @remembered
    def read(self):
        """The whole record, plus whether flash holds one."""
        reader = Reader(self._op(CalOp.GET))
        stored = bool(reader.u8())
        version = reader.u16()
        # Consume what the BOARD said it sent, not what this list happens to
        # name.
        params = {self._name(i): reader.u32() for i in range(reader.u8())}
        channels = [{'index': index, 'offset_raw': reader.i32(),
                     'gain_ppm': reader.i32()}
                    for index in range(reader.u8())]
        # The thermal envelope.
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
        """The parameters past what op 0 carries, through op 8."""
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
        """Both corrections for one channel, together."""
        self._op(CalOp.SET_CHANNEL,
                 pack(('u8', index), ('i32', int(offset_raw)),
                      ('i32', int(gain_ppm))))

    @forgetting('read')
    def zero(self, index):
        """Measure the channel now and keep the reading as its offset."""
        return Reader(self._op(CalOp.ZERO, pack(('u8', index)))).i32()

    @forgetting('read')
    def span(self, index, reference):
        """Trim the gain so the channel reports `reference`."""
        return Reader(self._op(CalOp.SPAN, pack(('u8', index),
                                                ('i32', int(reference))))).i32()

    @forgetting('read')
    def save(self):
        """Commit to flash. Erases and rewrites one sector, then reads back."""
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
