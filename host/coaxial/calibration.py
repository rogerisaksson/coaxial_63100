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
from . import protocol
from .errors import DeviceStateError
from .subsystem import Subsystem
from .wire import Reader


class Calibration(Subsystem):

    """Device 3 behind 0x6E. Edits are volatile until save()."""

    # A 128 KB sector erase is specified at up to 4 s on this silicon, and the
    # board answers save() only once it has erased, reprogrammed and read
    # back. The default 0.5 s budget would time out on a save that worked.
    SAVE_TIMEOUT = 6.0

    def _op(self, op, payload=b'', timeout=None):
        return self.request(protocol.DEVICE,
                            bytes([protocol.DEVICE_CAL, op]) + bytes(payload),
                            timeout=timeout)

    def read(self):
        """The whole record, plus whether flash holds one.

        `stored` false means these are the firmware's compiled-in defaults -
        the schematic's arithmetic, never measured. It is the difference
        between a calibrated board and an uncalibrated one, and nothing else
        in the reply shows it.
        """
        reader = Reader(self._op(protocol.CAL_OP_GET))
        stored = bool(reader.u8())
        version = reader.u16()
        params = {}
        for name in protocol.CAL_PARAMS[:reader.u8()]:
            params[name] = reader.u32()

        channels = []
        for index in range(reader.u8()):
            channels.append({'index': index,
                             'offset_raw': reader.i32(),
                             'gain_ppm': reader.i32()})

        return {'stored': stored, 'version': version,
                'params': params, 'channels': channels}

    def set_param(self, name, value):
        """One scalar, by the name read() returns it under."""
        try:
            ident = protocol.CAL_PARAMS.index(name)
        except ValueError:
            raise DeviceStateError(
                '%r is not a calibration parameter. There are %d: %s'
                % (name, len(protocol.CAL_PARAMS),
                   ', '.join(protocol.CAL_PARAMS)))

        self._op(protocol.CAL_OP_SET_PARAM,
                 bytes([ident]) + int(value).to_bytes(4, 'big'))

    def set_channel(self, index, offset_raw, gain_ppm):
        """Both corrections for one channel, together.

        Together because they are applied together - offset first, then gain -
        and setting one while guessing the other is how a half-applied
        calibration happens.
        """
        self._op(protocol.CAL_OP_SET_CHANNEL,
                 bytes([index]) +
                 int(offset_raw).to_bytes(4, 'big', signed=True) +
                 int(gain_ppm).to_bytes(4, 'big', signed=True))

    def zero(self, index):
        """Measure the channel now and keep the reading as its offset.

        Returns the code that was stored. The board does not know what is on
        the input - pointing it at a live one is the operator's mistake to
        make, and the returned code is what makes it visible.
        """
        return Reader(self._op(protocol.CAL_OP_ZERO, bytes([index]))).i32()

    def span(self, index, reference):
        """Trim the gain so the channel reports `reference`.

        The reference is in the channel's own unit - milliamperes for a phase,
        millivolts for the DC link. Refused for the thermistor, whose
        conversion is logarithmic and has no scale factor, and for a channel
        reading zero, which no finite gain turns into something.
        """
        return Reader(self._op(protocol.CAL_OP_SPAN,
                               bytes([index]) +
                               int(reference).to_bytes(4, 'big',
                                                       signed=True))).i32()

    def save(self):
        """Commit to flash. Erases and rewrites one sector, then reads back."""
        self._op(protocol.CAL_OP_SAVE, timeout=self.SAVE_TIMEOUT)

    def load(self):
        """Re-read flash, discarding uncommitted edits."""
        self._op(protocol.CAL_OP_LOAD)

    def defaults(self):
        """Back to the firmware's compiled-in numbers. RAM only until save."""
        self._op(protocol.CAL_OP_DEFAULTS)
