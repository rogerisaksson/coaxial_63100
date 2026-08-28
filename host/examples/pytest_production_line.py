"""Template: how a production line consumes this board.

    cd host && python -m pytest examples/pytest_production_line.py -v

A pattern rather than a working line test: the interesting part is the boundary
it draws. The board is a dumb slave - it measures and reports, holds no limits,
and judges nothing that needs a reference. So:

  * **Limits live in the test plan**, here the `limits` fixture. On a real line
    that is a TestStand sequence, a pytest ini, a YAML per variant - anywhere a
    process engineer can read it, change it under revision control and show it
    to an auditor. A limit compiled into firmware is one nobody on the line can
    see.

  * **Truth lives in calibrated instruments.** The board's numbers are
    uncalibrated by construction: its reference is a rail it cannot measure and
    its ADC is part of what is under test. A real line compares its reading
    against a DMM on the same node, a load at a known current, a chamber at a
    known setpoint. `reference_instruments` is where those attach; here it
    skips.

  * **The board judges only itself.** `system.self_test()` returns pass/fail
    for what its own registers and flash prove - a locked PLL, a calibration
    that ran, a checksum. No external reference, so no external limit. It is
    asserted below with no numbers in this file.
"""
import os
import sys

import pytest

# host/ on the path: this file's own directory's parent. Was '..' and '.',
# which only worked when run from host/ or host/examples - pytest collecting
# this file from the repository root failed outright.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import connect, disconnect


# ---- fixtures: the line's knowledge, not the board's ------------------------

@pytest.fixture(scope='module')
def board():
    boards = connect([(1, 115200)])
    boards[0].afe.enable()
    yield boards[0]
    disconnect(boards)


@pytest.fixture(scope='module')
def limits():
    """Stand-in for the test plan.

    Deliberately empty. Filling this in with numbers taken from one board on a
    bench is how a sample becomes a specification by accident. Real limits come
    from the design margin and the process capability, and they arrive from
    outside this file.
    """
    return {}


@pytest.fixture(scope='module')
def reference_instruments():
    """Stand-in for the calibrated instruments on the line.

    A real implementation returns handles to a DMM across the DC link, an
    electronic load on the phases, a chamber or a probe near the thermistor -
    each with a valid calibration date. Without them there is nothing to compare
    the board's readings against, so the value tests below skip rather than
    pretend.
    """
    return None


# ---- what the board can settle by itself -----------------------------------

def test_self_test_reports_no_failures(board):
    """The only firmware-side verdict, and it needs no limits from us."""
    failures = board.system.self_test_failures()
    assert not failures, 'board self test failed: %s' % (
        ', '.join(check['name'] for check in failures),)


def test_link_is_clean(board):
    """Echo proves framing, checksum and both codecs without touching state."""
    payload = 'production line ' + 'x' * 200
    assert board.link.echo(payload) == payload

    stats = board.link.stats()
    assert stats['char_overrun'] == 0, 'the receiver overran; the link is unhealthy'


def test_every_channel_responds(board):
    """Structural, not numeric: each configured channel returns a reading.

    A stuck channel shows up as identical min and max across a burst, which is
    detectable without knowing what the value should be.
    """
    reading = board.analog.read_all(nr_of_samples=64, sample_rate=2000.0)

    assert len(reading['channels']) == len(board.analog.channels())

    for channel in reading['channels']:
        assert channel['max_raw'] != channel['min_raw'], (
            'channel %d (%s) returned an identical value for every sample'
            % (channel['index'], channel['signal'] or channel['pin']))


def test_afe_switch_reaches_the_pin(board):
    """A physical witness for a logical write, needing no calibration.

    PE15 tracks AFE_ON inversely on this design, so toggling the coil and reading
    the discrete input proves the write reached the pin and not merely a
    register. Structural, so it belongs here rather than in the test plan.
    """
    board.gpio.test_mode(True)
    try:
        board.gpio.pin_write('B', 2, False)
        with_afe_off = board.gpio.pin_read('E', 15)

        board.gpio.pin_write('B', 2, True)
        with_afe_on = board.gpio.pin_read('E', 15)
    finally:
        board.gpio.test_mode(False)
        board.afe.enable()

    assert with_afe_off and not with_afe_on, (
        'PE15 did not follow AFE_ON: off -> %s, on -> %s'
        % (with_afe_off, with_afe_on))


# ---- what needs an instrument ----------------------------------------------

def test_dc_link_against_a_meter(board, limits, reference_instruments):
    """Compare the board against a DMM on the same node.

    The board's DC link reading is absolute: it scales with a reference rail the
    board cannot measure. Only a calibrated meter can say whether it is right,
    which is why this test needs an instrument and not a number.
    """
    if reference_instruments is None:
        pytest.skip('no calibrated meter attached; nothing to compare against')

    measured = board.analog.dcbus_voltage()['volts']
    reference = reference_instruments.dmm.read_volts()
    tolerance = limits['dc_link_tolerance_volts']

    assert abs(measured - reference) <= tolerance


def test_thermistor_against_a_known_temperature(board, limits,
                                                reference_instruments):
    """The thermistor conversion is ratiometric, so it survives a reference
    error - but the nameplate B and R25 still carry tolerance, and self-heating
    biases it high. Only a known temperature settles it."""
    if reference_instruments is None:
        pytest.skip('no temperature reference attached')

    measured = board.analog.ntc_temperature()['celsius']
    reference = reference_instruments.chamber.setpoint_celsius()

    assert abs(measured - reference) <= limits['temperature_tolerance_celsius']


def test_phase_sense_against_a_load(board, limits, reference_instruments):
    """The phase channels sit behind AFE gain neither side knows, so the host
    reports volts at the ADC pin and nothing further. Turning that into amperes
    is a calibration step: drive a known current and fit the gain."""
    if reference_instruments is None:
        pytest.skip('no electronic load attached')

    pytest.skip('gain calibration procedure not defined yet')
