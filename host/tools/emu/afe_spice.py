#!/usr/bin/env python3
"""The phase transfer the emulator's front end uses, from the LTspice run of the AFE.

    python tools/emu/afe_spice.py                 # run LTspice, fit, write the repl
    python tools/emu/afe_spice.py --raw RUN.raw   # fit a run already made

electronic_simulations/afe/amplifiers.asc (the submodule) runs with its Monte Carlo step off:
nominal parts. At every ADC sample instant - the end of each window the sample switch
(V(s1+)) is closed - the held V(v_diff) against I(RSHUNT) there is fitted to zero + k * I,
the residual's sigma is its spread there, and V(adc_vbus) against V(vdcbus) gives the bus ratio.
The result is board/emu/coaxial_63100_afe.repl, which coaxial_63100.resc loads over the
front end's defaults.
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
#: The sample switch's drive threshold, the ADCSW model's Vt.
SWITCH_VT = 0.5


def run_ltspice(workdir):
    """amplifiers.asc run once, nominal, in `workdir`; the .raw's path."""
    shutil.copytree(os.path.dirname(SCHEMATIC), workdir, dirs_exist_ok=True)
    for lib in os.listdir(MODELS):
        if lib.lower().endswith('.lib'):
            shutil.copy(os.path.join(MODELS, lib), workdir)
    asc = os.path.join(workdir, os.path.basename(SCHEMATIC))
    with open(asc, encoding='latin-1') as f:
        text = f.read()
    with open(asc, 'w', encoding='latin-1') as f:
        f.write(text.replace(STEP, ';' + STEP[1:]))
    subprocess.run([LTSPICE, '-b', asc], cwd=workdir, check=True)
    return asc[:-4] + '.raw'


def traces(path, wanted):
    """{name: array} for `wanted` out of an LTspice binary .raw (time double, the rest
    float32), read through a memory map."""
    with open(path, 'rb') as f:
        head = f.read(1 << 20)
    marker = 'Binary:\n'.encode('utf-16-le')
    at = head.index(marker)
    lines = head[:at].decode('utf-16-le').split('\n')
    points = int(next(l for l in lines if l.startswith('No. Points:')).split(':')[1])
    names = [l.split('\t')[2] for l in lines[lines.index('Variables:') + 1:] if l.startswith('\t')]
    record = np.dtype([('time', '<f8'), ('rest', '<f4', (len(names) - 1,))])
    data = np.memmap(path, dtype=record, mode='r', offset=at + len(marker), shape=(points,))
    out = {'time': np.abs(np.asarray(data['time']))}
    for name in wanted:
        out[name] = np.asarray(data['rest'][:, names.index(name) - 1], dtype=np.float64)
    return out


def fit(t):
    """The transfer at the sample instants."""
    closed = t['V(s1+)'] > SWITCH_VT
    held = np.flatnonzero(closed[:-1] & ~closed[1:])      # the last point of each window
    volts, amps = t['V(v_diff)'][held], t['I(RSHUNT)'][held]
    slope, zero = np.polyfit(amps, volts, 1)
    residual = volts - (zero + slope * amps)
    ratio = np.median(t['V(vdcbus)'][held] / t['V(adc_vbus)'][held])
    return {'samples': len(held), 'volts_per_amp': slope, 'zero_volts': zero,
            'noise_volts': float(np.std(residual)), 'amps': (amps.min(), amps.max()),
            'bus_ratio': ratio}


def repl(got, source):
    return (
        '// coaxial_63100_afe.repl - Written by host/tools/emu/afe_spice.py from\r\n'
        '// %s: %d ADC samples of V(v_diff) against\r\n'
        '// I(RSHUNT) over %.1f .. %.1f A, fitted. The residual, %.2f mV sigma, is the current\r\n'
        '// moving within the sample window, not converter noise; the bus ratio\r\n'
        '// V(vdcbus)/V(adc_vbus) there is %.4f.\r\n'
        'afe:\r\n'
        '    PhaseVoltsPerAmp: %.6f\r\n'
        '    PhaseZeroVolts: %.6f\r\n'
        % (source, got['samples'], got['amps'][0], got['amps'][1], got['noise_volts'] * 1e3,
           got['bus_ratio'], got['volts_per_amp'], got['zero_volts']))


def main():
    parser = argparse.ArgumentParser(description='The AFE transfer for the emulator, from LTspice.')
    parser.add_argument('--raw', help='a run already made (default: run LTspice now)')
    args = parser.parse_args()
    head = subprocess.run(['git', '-C', SIMULATIONS, 'rev-parse', '--short', 'HEAD'],
                          capture_output=True, text=True).stdout.strip()
    source = 'electronic_simulations/afe/amplifiers.asc (%s), nominal' % head
    with tempfile.TemporaryDirectory() as work:
        raw = args.raw or run_ltspice(work)
        got = fit(traces(raw, ('V(s1+)', 'V(v_diff)', 'I(RSHUNT)', 'V(vdcbus)', 'V(adc_vbus)')))
    for key, value in got.items():
        print('%-14s %s' % (key, value))
    with open(OUT, 'w', encoding='utf-8', newline='') as f:
        f.write(repl(got, source))
    print('wrote', os.path.relpath(OUT, REPO))
    return 0


if __name__ == '__main__':
    sys.exit(main())
