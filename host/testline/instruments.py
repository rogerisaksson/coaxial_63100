"""Instruments, and why they are the only source of truth on the line.

The board under test reports raw ADC codes. It cannot say whether they are
right, because its reference is a rail it cannot measure and its own converter is
part of what is being tested. Truth therefore comes from instruments that carry a
calibration certificate, and every reading below arrives with the asset id and
calibration due date of whatever produced it.

That metadata is not decoration. It is what makes a test record defensible: a
measurement whose instrument was out of calibration on the day is not a
measurement, and a line that cannot demonstrate otherwise cannot ship.

The implementations here are SIMULATED. Replacing one means implementing the same
three or four methods against VISA, a serial protocol, or a vendor SDK - nothing
above this module knows the difference, which is the point.
"""
import datetime
import random


class InstrumentError(Exception):
    """The instrument could not produce a reading."""


class CalibrationExpired(InstrumentError):
    """Refuses to measure. A reading from an out-of-calibration instrument is
    worse than no reading, because it looks like data in the report."""


class Instrument:
    """Base class carrying the traceability every reading needs."""

    kind = 'instrument'

    def __init__(self, model, asset_id, calibration_due, simulated=True):
        self.model = model
        self.asset_id = asset_id
        self.calibration_due = calibration_due       # datetime.date
        self.simulated = simulated

    def __repr__(self):
        return '<%s %s asset=%s>' % (self.kind, self.model, self.asset_id)

    def check_calibration(self, today=None):
        today = today or datetime.date.today()
        if self.calibration_due < today:
            raise CalibrationExpired(
                '%s (asset %s) calibration expired %s'
                % (self.model, self.asset_id, self.calibration_due))

    def provenance(self):
        return {
            'kind': self.kind,
            'model': self.model,
            'asset_id': self.asset_id,
            'calibration_due': self.calibration_due.isoformat(),
            'simulated': self.simulated,
        }


class Dmm(Instrument):
    """Bench multimeter across whatever node the fixture routes to it."""

    kind = 'DMM'

    def __init__(self, model='Keysight 34465A', asset_id='DMM-0001',
                 calibration_due=None, simulated=True):
        super().__init__(model, asset_id,
                         calibration_due or datetime.date(2027, 3, 31), simulated)
        self._nodes = {}

    def set_simulated_node(self, node, volts, noise_v=0.0008):
        """Only meaningful for the simulation. A real DMM has no such method,
        which is exactly why the sequence never calls it."""
        self._nodes[node] = (volts, noise_v)

    def read_dc_volts(self, node):
        self.check_calibration()
        if node not in self._nodes:
            raise InstrumentError('no simulated source configured for node %r'
                                  % (node,))
        volts, noise = self._nodes[node]
        return random.gauss(volts, noise)


class SignalGenerator(Instrument):
    """Drives a stimulus into a test point. Here it sets a DC level, which is
    what a fixture needs to exercise an analog input at a known value."""

    kind = 'signal generator'

    def __init__(self, model='Rigol DG1022Z', asset_id='SIG-0007',
                 calibration_due=None, simulated=True):
        super().__init__(model, asset_id,
                         calibration_due or datetime.date(2027, 1, 15), simulated)
        self.output = {}

    def set_dc(self, channel, volts):
        self.check_calibration()
        self.output[channel] = volts
        return volts

    def off(self, channel):
        self.output.pop(channel, None)


class Oscilloscope(Instrument):
    """Captures a waveform at a test point. Used here to characterise ripple,
    which a DMM averages away and the board cannot see at all."""

    kind = 'oscilloscope'

    def __init__(self, model='Tektronix MSO44', asset_id='SCP-0003',
                 calibration_due=None, simulated=True):
        super().__init__(model, asset_id,
                         calibration_due or datetime.date(2026, 11, 30), simulated)
        self._sim = {}

    def set_simulated_channel(self, channel, mean_v, ripple_vpp):
        self._sim[channel] = (mean_v, ripple_vpp)

    def measure_ripple_vpp(self, channel):
        self.check_calibration()
        if channel not in self._sim:
            raise InstrumentError('no simulated waveform on channel %r'
                                  % (channel,))
        _, ripple = self._sim[channel]
        return abs(random.gauss(ripple, ripple * 0.05))


class BarcodeScanner(Instrument):
    """Reads the serial number off the PCBA.

    Not a measuring instrument, so it carries no calibration date that matters -
    but it is the single most important device on the bench, because a report
    without a serial number cannot be traced back to a board and is therefore
    worthless as a record.
    """

    kind = 'barcode scanner'

    def __init__(self, model='Zebra DS2208', asset_id='BCR-0011',
                 simulated=True, queue=None):
        super().__init__(model, asset_id, datetime.date(2099, 1, 1), simulated)
        self.queue = list(queue or [])

    def check_calibration(self, today=None):
        return None                 # a scanner reads or it does not

    def scan(self):
        """Return the next barcode. Blocks on a real scanner; here it pops a
        queue so a sequence can be run unattended in CI."""
        if not self.queue:
            raise InstrumentError('no barcode presented to the scanner')
        return self.queue.pop(0)


class Bench:
    """Everything on the bench, so a sequence takes one argument.

    The set of instruments IS the measurement system. Swap a DMM and the GRR
    study that established the limits no longer applies to this bench, which is
    why the provenance of every one of them lands in the report.
    """

    def __init__(self, dmm=None, siggen=None, scope=None, scanner=None):
        self.dmm = dmm or Dmm()
        self.siggen = siggen or SignalGenerator()
        self.scope = scope or Oscilloscope()
        self.scanner = scanner or BarcodeScanner()

    def all(self):
        return [self.dmm, self.siggen, self.scope, self.scanner]

    def provenance(self):
        return [instrument.provenance() for instrument in self.all()]

    def check_calibration(self, today=None):
        """Fail before the first measurement, not after the last."""
        for instrument in self.all():
            instrument.check_calibration(today)
