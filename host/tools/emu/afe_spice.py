#!/usr/bin/env python3
"""The front end the emulator models, from LTspice runs of the AFE: transfer, spread, drift.

    python tools/emu/afe_spice.py                  # run LTspice twice, fit, write the repl
    python tools/emu/afe_spice.py --keep DIR       # and keep the runs in DIR
    python tools/emu/afe_spice.py --from DIR       # fit runs already kept

electronic_simulations/afe/amplifiers.asc (the submodule), twice, each run saving only what
the fit reads:

- spread: `.step param run 0 N 1` in place of the schematic's 0..10 - run 0 nominal, 1..N the
  parts drawn inside the tolerances given in its `val(nom, tol)`s (`--runs`, RUNS: a run is
  ~5 min on the laptop);
- drift: nominal parts, `.step temp` over TEMPERATURES.

At every ADC sample instant - the end of each window the sample switch (V(s1+)) is closed -
the held V(v_diff) against I(RSHUNT) is fitted to zero + k * I, and V(vdcbus)/V(adc_vbus)
taken as the bus ratio. The result is board/emu/coaxial_63100_afe.repl: the nominal transfer,
the spread between boards (each emulated board draws its own, seeded by its UID) and the
drift with the board's temperature.
"""
import argparse
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402

SIMULATIONS = os.path.join(REPO, 'electronic_simulations')
SCHEMATIC = os.path.join(SIMULATIONS, 'afe', 'amplifiers.asc')
MODELS = os.path.join(SIMULATIONS, 'motor_inverters', 'half_bridge')
OUT = os.path.join(REPO, 'board', 'emu', 'coaxial_63100_afe.repl')
LTSPICE = os.path.join(os.environ.get('ProgramFiles', r'C:\Program Files'), 'ADI', 'LTspice',
                       'LTspice.exe')
STEP = '!.step param run 0 10 1'
TRACES = ('V(s1+)', 'V(v_diff)', 'I(RSHUNT)', 'V(vdcbus)', 'V(adc_vbus)')
#: Boards drawn by default: enough for a sigma, few for a run of 5 min each (2026-09-25).
RUNS = 4
#: The drift run's board temperatures, C: its ends, the slope through them.
TEMPERATURES = (0, 85)
#: The sample switch's drive threshold, the ADCSW model's Vt.
SWITCH_VT = 0.5
#: Where a directive lands on the sheet: below everything drawn there.
DIRECTIVE_AT = 'TEXT 288 1900 Left 2 !%s\n'


def run_ltspice(workdir, name, step):
    """amplifiers.asc as `name`.asc in `workdir`, its step replaced by `step`, only TRACES
    saved; the .raw's path."""
    shutil.copytree(os.path.dirname(SCHEMATIC), workdir, dirs_exist_ok=True)
    for lib in os.listdir(MODELS):
        if lib.lower().endswith('.lib'):
            shutil.copy(os.path.join(MODELS, lib), workdir)
    with open(SCHEMATIC, encoding='latin-1') as f:
        text = f.read()
    text = text.replace(STEP, ';' + STEP[1:]) + DIRECTIVE_AT % step
    text += DIRECTIVE_AT % ('.save ' + ' '.join(TRACES))
    asc = os.path.join(workdir, name + '.asc')
    with open(asc, 'w', encoding='latin-1') as f:
        f.write(text)
    subprocess.run([LTSPICE, '-b', asc], cwd=workdir, check=True)
    return asc[:-4] + '.raw'


def steps(path):
    """[{name: array}] a step, out of an LTspice binary .raw (time double, the rest float32),
    read through a memory map; the steps split where time starts over."""
    with open(path, 'rb') as f:
        head = f.read(1 << 20)
    marker = 'Binary:\n'.encode('utf-16-le')
    at = head.index(marker)
    lines = head[:at].decode('utf-16-le').split('\n')
    points = int(next(l for l in lines if l.startswith('No. Points:')).split(':')[1])
    names = [l.split('\t')[2] for l in lines[lines.index('Variables:') + 1:] if l.startswith('\t')]
    record = np.dtype([('time', '<f8'), ('rest', '<f4', (len(names) - 1,))])
    data = np.memmap(path, dtype=record, mode='r', offset=at + len(marker), shape=(points,))
    time = np.abs(np.asarray(data['time']))
    starts = [0] + list(np.flatnonzero(np.diff(time) < 0) + 1) + [points]
    columns = {name: np.asarray(data['rest'][:, names.index(name) - 1], dtype=np.float64)
               for name in TRACES}
    return [{name: column[a:b] for name, column in columns.items()}
            for a, b in zip(starts, starts[1:])]


def fit(t):
    """One step's transfer at the sample instants."""
    closed = t['V(s1+)'] > SWITCH_VT
    held = np.flatnonzero(closed[:-1] & ~closed[1:])      # the last point of each window
    volts, amps = t['V(v_diff)'][held], t['I(RSHUNT)'][held]
    slope, zero = np.polyfit(amps, volts, 1)
    return {'samples': len(held), 'volts_per_amp': slope, 'zero_volts': zero,
            'residual_volts': float(np.std(volts - (zero + slope * amps))),
            'amps': (float(amps.min()), float(amps.max())),
            'bus_ratio': float(np.median(t['V(vdcbus)'][held] / t['V(adc_vbus)'][held]))}


def summary(spread, drift):
    """The repl's numbers from the two runs' fits."""
    nominal, boards = spread[0], spread[1:]
    gain = np.array([b['volts_per_amp'] for b in boards]) / nominal['volts_per_amp'] - 1.0
    zero = np.array([b['zero_volts'] for b in boards]) - nominal['zero_volts']
    bus = np.array([b['bus_ratio'] for b in boards]) / nominal['bus_ratio'] - 1.0
    kelvin = np.array(TEMPERATURES[:len(drift)], dtype=float)
    return {'nominal': nominal, 'boards': len(boards),
            'gain_sigma': float(np.std(gain)), 'zero_sigma': float(np.std(zero)),
            'bus_sigma': float(np.std(bus)),
            'gain_per_kelvin': float(np.polyfit(kelvin, [d['volts_per_amp'] for d in drift], 1)[0]
                                     / nominal['volts_per_amp']),
            'zero_per_kelvin': float(np.polyfit(kelvin, [d['zero_volts'] for d in drift], 1)[0])}


def repl(got, source):
    n = got['nominal']
    return (
        '// coaxial_63100_afe.repl - Written by host/tools/emu/afe_spice.py from\r\n'
        '// %s. Nominal: %d ADC samples of V(v_diff)\r\n'
        '// against I(RSHUNT) over %.1f .. %.1f A, fitted; the residual, %.2f mV sigma, is the\r\n'
        '// current moving within the sample window, not converter noise. Spread: %d boards\r\n'
        '// drawn inside the schematic\'s tolerances. Drift: nominal parts over %s C.\r\n'
        'afe:\r\n'
        '    PhaseVoltsPerAmp: %.6f\r\n'
        '    PhaseZeroVolts: %.6f\r\n'
        '    PhaseGainSigma: %.6f\r\n'
        '    PhaseZeroSigmaVolts: %.6f\r\n'
        '    PhaseGainPerKelvin: %.8f\r\n'
        '    PhaseZeroVoltsPerKelvin: %.8f\r\n'
        '    BusGainSigma: %.6f\r\n'
        % (source, n['samples'], n['amps'][0], n['amps'][1], n['residual_volts'] * 1e3,
           got['boards'], ', '.join(str(t) for t in TEMPERATURES),
           n['volts_per_amp'], n['zero_volts'], got['gain_sigma'], got['zero_sigma'],
           got['gain_per_kelvin'], got['zero_per_kelvin'], got['bus_sigma']))


def main():
    parser = argparse.ArgumentParser(description='The AFE the emulator models, from LTspice.')
    parser.add_argument('--keep', help='run into this directory and keep the runs')
    parser.add_argument('--from', dest='source', help='fit the runs kept in this directory')
    parser.add_argument('--runs', type=int, default=RUNS, help='boards drawn (default %d)' % RUNS)
    args = parser.parse_args()
    head = subprocess.run(['git', '-C', SIMULATIONS, 'rev-parse', '--short', 'HEAD'],
                          capture_output=True, text=True).stdout.strip()
    source = 'electronic_simulations/afe/amplifiers.asc (%s)' % head
    with tempfile.TemporaryDirectory() as scratch:
        work = args.source or args.keep or scratch
        if args.source:
            raws = [os.path.join(work, name + '.raw') for name in ('spread', 'drift')]
        else:
            os.makedirs(work, exist_ok=True)
            raws = [run_ltspice(work, 'spread', '.step param run 0 %d 1' % args.runs),
                    run_ltspice(work, 'drift', '.step temp list %s'
                                % ' '.join(str(t) for t in TEMPERATURES))]
        spread, drift = ([fit(t) for t in steps(raw)] for raw in raws)
    got = summary(spread, drift)
    for key, value in got.items():
        print('%-16s %s' % (key, value))
    with open(OUT, 'w', encoding='utf-8', newline='') as f:
        f.write(repl(got, source))
    print('wrote', os.path.relpath(OUT, REPO))
    return 0


if __name__ == '__main__':
    sys.exit(main())
