#!/usr/bin/env python3
"""The front end the emulator models: the transfer off a nominal LTspice run, the spread by arithmetic.

    python tools/emu/afe_spice.py                  # the spread from the tolerances, the transfer kept
    python tools/emu/afe_spice.py --raw RUN.raw    # and the transfer refitted off a nominal run

Nothing here runs LTspice: a run of electronic_simulations/afe/amplifiers.asc is minutes and
its Monte Carlo the schematic's owner's to run. A run given (`--raw`, nominal, `.step` off) is
fitted at every ADC sample instant - the end of each window the sample switch (V(s1+)) is
closed - as the held V(v_diff) against I(RSHUNT), zero + k * I, and V(vdcbus)/V(adc_vbus) as
the bus ratio. The spread between boards is the schematic's own tolerances, its `val(nom, tol)`
read off the .asc: a part's sigma tol/3 (the schematic's gauss(tol/3)), the gain's sigma the
root sum of squares over the parts that set it (PHASE_GAIN), the zero's the Rf/Rg sides'
mismatch on the common mode (PHASE_ZERO), the bus divider's its two resistors (BUS). Drift:
nothing in the schematic gives a tempco, so none. The result is
board/emu/coaxial_63100_afe.repl.
"""
import argparse
import math
import os
import re
import subprocess
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from tools import REPO  # noqa: E402

SIMULATIONS = os.path.join(REPO, 'electronic_simulations')
SCHEMATIC = os.path.join(SIMULATIONS, 'afe', 'amplifiers.asc')
OUT = os.path.join(REPO, 'board', 'emu', 'coaxial_63100_afe.repl')
TRACES = ('V(s1+)', 'V(v_diff)', 'I(RSHUNT)', 'V(vdcbus)', 'V(adc_vbus)')
#: The sample switch's drive threshold, the ADCSW model's Vt.
SWITCH_VT = 0.5

#: What sets the phase gain (the design notes' chain): the shunt, the divider into the FDA, the
#: THS4551's Rf and Rg on each side.
PHASE_GAIN = ('RSHUNT', 'R12', 'R17', 'R1', 'R19', 'R5', 'R44')
#: The two sides whose Rf/Rg mismatch turns the common mode into a zero, and that mode, V.
PHASE_ZERO = ('R1', 'R19', 'R5', 'R44')
COMMON_MODE = 1.65
#: The DC link divider, 49.9k over 2.2k.
BUS = ('R23', 'R37')


def tolerances(path=SCHEMATIC):
    """{part: tolerance} off the schematic's `val(nom, tol)`s."""
    with open(path, encoding='latin-1') as f:
        text = f.read()
    found = {}
    for block in text.split('SYMBOL ')[1:]:
        name = re.search(r'SYMATTR InstName (\S+)', block)
        value = re.search(r'val\([^,]+,\s*([0-9.eE+-]+)\)', block)
        if name and value:
            found[name.group(1)] = float(value.group(1))
    return found


def spread(tol):
    """The sigmas the repl carries, from the tolerances."""
    sigma = {part: t / 3.0 for part, t in tol.items()}

    def rss(parts):
        return math.sqrt(sum(sigma[p] ** 2 for p in parts))

    return {'gain': rss(PHASE_GAIN), 'zero': COMMON_MODE * rss(PHASE_ZERO), 'bus': rss(BUS)}


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
    """A nominal run's transfer at the sample instants."""
    closed = t['V(s1+)'] > SWITCH_VT
    held = np.flatnonzero(closed[:-1] & ~closed[1:])      # the last point of each window
    volts, amps = t['V(v_diff)'][held], t['I(RSHUNT)'][held]
    slope, zero = np.polyfit(amps, volts, 1)
    return {'samples': len(held), 'volts_per_amp': float(slope), 'zero_volts': float(zero),
            'residual_volts': float(np.std(volts - (zero + slope * amps))),
            'amps': (float(amps.min()), float(amps.max())),
            'bus_ratio': float(np.median(t['V(vdcbus)'][held] / t['V(adc_vbus)'][held]))}


def kept():
    """The transfer the repl carries now, and its lines on where it came from."""
    with open(OUT, encoding='utf-8') as f:
        text = f.read()

    def number(key):
        found = re.search(r'%s: ([0-9.eE+-]+)' % key, text)
        if found is None:
            raise SystemExit('%s carries no %s - refit it: --raw RUN.raw' % (OUT, key))
        return float(found.group(1))

    said = [l for l in text.splitlines() if l.startswith('// ') and 'afe_spice' not in l
            and not l.startswith(('// Spread', '// the zero', '// is given'))]
    return {'volts_per_amp': number('PhaseVoltsPerAmp'), 'zero_volts': number('PhaseZeroVolts'),
            'said': said}


def repl(transfer, sig, source):
    return ''.join(line + '\r\n' for line in (
        ['// coaxial_63100_afe.repl - Written by host/tools/emu/afe_spice.py.']
        + transfer['said']
        + ['// Spread: %s\'s tolerances, sigma tol/3, root sum of squares - the gain over %s,'
           % (source, ', '.join(PHASE_GAIN)),
           '// the zero %.2f V of common mode on %s\'s mismatch, the bus divider %s. No tempco'
           % (COMMON_MODE, '/'.join(PHASE_ZERO), '/'.join(BUS)),
           '// is given in the schematic, so no drift.',
           'afe:',
           '    PhaseVoltsPerAmp: %.6f' % transfer['volts_per_amp'],
           '    PhaseZeroVolts: %.6f' % transfer['zero_volts'],
           '    PhaseGainSigma: %.6f' % sig['gain'],
           '    PhaseZeroSigmaVolts: %.6f' % sig['zero'],
           '    BusGainSigma: %.6f' % sig['bus']]))


def main():
    parser = argparse.ArgumentParser(description='The AFE the emulator models: no LTspice run.')
    parser.add_argument('--raw', help='a nominal run of amplifiers.asc to refit the transfer from')
    args = parser.parse_args()
    head = subprocess.run(['git', '-C', SIMULATIONS, 'rev-parse', '--short', 'HEAD'],
                          capture_output=True, text=True).stdout.strip()
    source = 'amplifiers.asc (%s)' % head
    if args.raw:
        n = fit(steps(args.raw)[0])
        transfer = dict(n, said=[
            '// %s, nominal: %d ADC samples of V(v_diff) against I(RSHUNT) over %.1f .. %.1f A,'
            % (source, n['samples'], n['amps'][0], n['amps'][1]),
            '// fitted; the residual, %.2f mV sigma, is the current moving within the sample'
            % (n['residual_volts'] * 1e3),
            '// window, not converter noise; the bus ratio V(vdcbus)/V(adc_vbus) there %.4f.'
            % n['bus_ratio']])
    else:
        transfer = kept()
    sig = spread(tolerances())
    for key, value in sig.items():
        print('%-6s sigma %.3g' % (key, value))
    with open(OUT, 'w', encoding='utf-8', newline='') as f:
        f.write(repl(transfer, sig, source))
    print('wrote', os.path.relpath(OUT, REPO))
    return 0


if __name__ == '__main__':
    sys.exit(main())
