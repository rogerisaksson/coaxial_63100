"""Template: how a production line consumes this board."""
import os
import sys

import pytest

# host/ on the path: this file's own directory's parent.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from coaxial import connect, disconnect


# ---- fixtures: the line's knowledge, not the board's ------------------------

@pytest.fixture(scope='module')
def board():
    boards = connect([(1, 115200)])
    boards[0].afe.on()
    yield boards[0]
    disconnect(boards)


@pytest.fixture(scope='module')
def limits():
    """Stand-in for the test plan."""
    return {}


@pytest.fixture(scope='module')
def reference_instruments():
    """Stand-in for the calibrated instruments on the line."""
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
    """Structural, not numeric: each configured channel returns a reading."""
    reading = board.analog.read(samples=64, sample_rate=2000.0)

    assert len(reading['channels']) == len(board.analog.channels())

    for channel in reading['channels']:
        assert channel['max_raw'] != channel['min_raw'], (
            'channel %d (%s) returned an identical value for every sample'
            % (channel['index'], channel['signal'] or channel['pin']))


def test_afe_switch_reaches_the_pin(board):
    """A physical witness for a logical write, needing no calibration."""
    board.gpio.on()
    try:
        board.gpio.write('B', 2, False)
        with_afe_off = board.gpio.read('E', 15)

        board.gpio.write('B', 2, True)
        with_afe_on = board.gpio.read('E', 15)
    finally:
        board.gpio.off()
        board.afe.on()

    assert with_afe_off and not with_afe_on, (
        'PE15 did not follow AFE_ON: off -> %s, on -> %s'
        % (with_afe_off, with_afe_on))


# ---- what needs an instrument ----------------------------------------------

def test_dc_link_against_a_meter(board, limits, reference_instruments):
    """Compare the board against a DMM on the same node."""
    if reference_instruments is None:
        pytest.skip('no calibrated meter attached; nothing to compare against')

    measured = board.analog.dcbus_voltage()['volts']
    reference = reference_instruments.dmm.read_volts()
    tolerance = limits['dc_link_tolerance_volts']

    assert abs(measured - reference) <= tolerance


def test_thermistor_against_a_known_temperature(board, limits,
                                                reference_instruments):
    """The thermistor conversion is ratiometric, so it survives a reference
    error - but the nameplate B and R25 still carry tolerance, and
    self-heating biases it high.
    """
    if reference_instruments is None:
        pytest.skip('no temperature reference attached')

    measured = board.analog.ntc_temperature()['celsius']
    reference = reference_instruments.chamber.setpoint_celsius()

    assert abs(measured - reference) <= limits['temperature_tolerance_celsius']


def test_phase_sense_against_a_load(board, limits, reference_instruments):
    """The phase channels sit behind AFE gain neither side knows, so the
    host reports volts at the ADC pin and nothing further.
    """
    if reference_instruments is None:
        pytest.skip('no electronic load attached')

    pytest.skip('gain calibration procedure not defined yet')
