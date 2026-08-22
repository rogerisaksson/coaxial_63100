#!/usr/bin/env python3
"""Scale a phase_log CSV to physical units and report stability.

Every channel is converted through its own full chain. The point of this
script is that the scaling lives in one place: the earlier hand-rolled
comparison reported DC bus figures at the ADC pin and silently dropped the
23.7x divider, which made the absolute numbers meaningless.
"""
import csv, math, sys

VREF   = 3.3
N_SE   = 65536.0          # single-ended: 0..65535, 0 = 0 V
N_DIFF = 32768.0          # differential: offset binary, code already centred
DIV_DCBUS = (49900.0 + 2200.0) / 2200.0

# name -> (full-scale denominator, gain from ADC pin to the physical quantity,
#          unit, note). gain None = unknown analog gain ahead of the pin, so
#          the pin voltage is all we can honestly report.
CHANNELS = {
    'U':     (N_DIFF, None,      'V', 'AFE gain unknown - value is at the ADC pin'),
    'V':     (N_DIFF, None,      'V', 'AFE gain unknown - value is at the ADC pin'),
    'W':     (N_DIFF, None,      'V', 'AFE gain unknown - value is at the ADC pin'),
    'DCbus': (N_SE,   DIV_DCBUS, 'V', '49.9k/2.2k divider, absolute - depends on VREF'),
}

def lsb_volts(full_scale, gain):
    return VREF / full_scale * (gain if gain else 1.0)

def fit(t, a):
    """mean, peak-to-peak, slope per minute, sd, sd after removing the trend."""
    n = len(a)
    m = sum(a) / n
    tb = sum(t) / n
    sxx = sum((x - tb) ** 2 for x in t)
    slope = sum((t[i] - tb) * (a[i] - m) for i in range(n)) / sxx
    res = [a[i] - (m + slope * (t[i] - tb)) for i in range(n)]
    sd = math.sqrt(sum((x - m) ** 2 for x in a) / (n - 1))
    rsd = math.sqrt(sum(x * x for x in res) / (n - 1))
    return m, max(a) - min(a), slope * 60000.0, sd, rsd

def report(path):
    rows = list(csv.DictReader(open(path)))
    t = [float(r['t_ms']) for r in rows]
    print('=== %s : n=%d over %.0f s ===' % (path, len(rows), (t[-1] - t[0]) / 1000))
    period = (t[-1] - t[0]) / (len(t) - 1)
    print('heartbeat %.3f ms (%+.0f ppm vs host clock)\n' % (period, (period - 1000) / 1000 * 1e6))
    for name, (fs, gain, unit, note) in CHANNELS.items():
        if name not in rows[0]:
            continue
        a = [float(r[name]) for r in rows]
        m, p2p, drift, sd, rsd = fit(t, a)
        k = lsb_volts(fs, gain)
        print('%-6s mean %+10.4f %s   p2p %8.2f m%s   drift %+7.2f m%s/min' % (
            name, m * k, unit, p2p * k * 1000, unit, drift * k * 1000, unit))
        print('       sd %8.3f m%s   sd_detrended %8.3f m%s   (1 LSB = %.4f m%s)' % (
            sd * k * 1000, unit, rsd * k * 1000, unit, k * 1000, unit))
        print('       %s\n' % note)

if __name__ == '__main__':
    for p in (sys.argv[1:] or ['phase_log.csv']):
        report(p)
