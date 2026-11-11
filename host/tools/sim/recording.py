#!/usr/bin/env python3
"""A recording of her loop told (`watch.recorded`): its blocks, its events, each window a table.

    python tools/sim/recording.py --last                  # the newest recording, every window
    python tools/sim/recording.py recordings/x.jsonl --window 2   # its second window alone
    python tools/sim/recording.py --last --overview 0.5   # the whole of it a row each 0.5 s, then its windows

Its blocks in time - 60 Hz between, a window at her streams' own clocks about each capture her
analyzers mark -; every event her analyzers told, in time; each window a row at each offset from
its trigger (ROWS_S) and each STEP_S across it, within it: her surety, each sole's load (N) as the row's time finds
them; her drives' largest torque (N m) and her joints' fastest rate (deg/s) since the row before,
each by its joint - a transient between rows is in the row it ends at -; the events since. The
overview's rows, 60 Hz and windows alike, are the same over its step.
"""
import argparse
import glob
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine.process.introspect import RECORDINGS

#: A window's rows: these offsets from its trigger, s, where the window holds them.
ROWS_S = (-0.6, -0.3, -0.15, -0.1, -0.05, -0.02, 0.0, 0.02, 0.05, 0.1, 0.15, 0.3, 0.6, 0.85)
#: Her watch's own events, not hers: left out.
QUIET = ('loaded', 'reloaded', 'dropped', 'armed', 'gated', 'every')
#: And a row each STEP_S across a window, from its start: one drawn on past its trigger is told whole.
STEP_S = 0.1


def joined(gates):
    """{stream: (times, samples)} of every block of `gates` (`gate.read`) in time; a vector's
    samples an array, its fields and their names beside it ({stream: (fields, names)})."""
    import numpy as np
    out, lay = {}, {}
    for g in gates:
        for n, (ts, vs) in g['streams'].items():
            if len(ts):
                out.setdefault(n, []).append((np.asarray(ts, float), vs))
            m = g['meta']['streams'].get(n, {})
            lay.setdefault(n, (m.get('fields'), m.get('names', {})) if m.get('kind') == 'vector' else None)
    joined_ = {}
    for n, parts in out.items():
        ts = np.concatenate([p[0] for p in parts])
        o = np.argsort(ts, kind='stable')
        vs = np.concatenate([p[1] for p in parts])[o] if lay.get(n) else [x for p in parts for x in p[1]]
        joined_[n] = (ts[o], vs if lay.get(n) else [vs[i] for i in o])
    return joined_, lay


def _field(series, lay, stream, field):
    """(times, block, entry names) of `field` of the vector `stream`."""
    import numpy as np
    if stream not in series or not lay.get(stream):
        return np.zeros(0), np.zeros((0, 0)), []
    fields, names = lay[stream]
    f = fields[field]
    ts, vs = series[stream]
    return ts, vs[:, f['at']:f['at'] + f['n']], names.get(f.get('of'), [])


def _at(ts, t):
    """The index of the last of `ts` at or before `t`, else None."""
    import numpy as np
    i = int(np.searchsorted(ts, t, side='right')) - 1
    return i if i >= 0 else None


def _peak(ts, block, names, lo, hi, scale=1.0):
    """'value joint' of the largest entry of `block` over (lo, hi], '-' none there."""
    import numpy as np
    sel = (ts > lo) & (ts <= hi)
    if not sel.any():
        return '-'
    b = np.abs(block[sel]) * scale
    i, j = np.unravel_index(int(np.argmax(b)), b.shape)
    return '%7.1f %-15s' % (b[i, j], names[j] if j < len(names) else j)


def rows(series, lay, events, times, start=None):
    """A row at each of `times`: (t, surety, soles L, soles R, tau peak, qd peak, events) since the one
    before - the first's since `start`, else a ms -."""
    import numpy as np
    st, sv = series.get('surety', (np.zeros(0), []))
    tl, load, _ = _field(series, lay, 'read', 'LOAD')
    tt, tau, tn = _field(series, lay, 'felt', 'TAU')
    tq, qd, qn = _field(series, lay, 'felt', 'QD')
    out, last = [], (times[0] - 1e-3 if times else 0.0) if start is None else start
    for t in times:
        i, k = _at(st, t), _at(tl, t)
        told = [e['analyzer'] + '.' + e['event'] for e in events if last < e['t'] <= t and e['event'] not in QUIET]
        out.append((t, '%.3f' % sv[i]['surety'] if i is not None else '  -  ',
                    '%5.0f' % load[k][0] if k is not None else '  -', '%5.0f' % load[k][1] if k is not None else '  -',
                    _peak(tt, tau, tn, last, t), _peak(tq, qd, qn, last, t, math.degrees(1.0)), ' '.join(told)))
        last = t
    return out


def printed(table, t0=None):
    """`rows`' rows as text, each at its time - or its offset from `t0` -."""
    head = '  %8s | surety | sole L N | sole R N | |tau| N m  joint        | |qd| deg/s joint       | events'
    lines = [head % ('dt s' if t0 is not None else 't s')]
    for t, s, l, r, tau, qd, ev in table:
        lines.append('  %8s | %s  | %8s | %8s | %-23s | %-23s | %s' % (
            '%+.3f' % (t - t0) if t0 is not None else '%.3f' % t, s, l, r, tau, qd, ev))
    return '\n'.join(lines)


def fielded(series, lay, specs, times):
    """A row at each of `times` of the fields `specs` - 'stream.FIELD' of a vector stream (impedance.COM_ACC,
    latent.NOW.Z, read.LOAD), 'stream.key.key' into a plain one's sample (wrench.got.bears; nan where the sample
    lacks the key, wrench.ask.swing.1.0 while that sole stands) -, each entry '%.3g', the sample at or before the
    row's time: her law's layers as the recording has them, beside the table."""
    import numpy as np
    cols = []
    for spec in specs:
        stream, _, path = spec.partition('.')
        if lay.get(stream):
            ts, block, _names = _field(series, lay, stream, path)
            cols.append((spec, ts, lambda i, block=block: np.atleast_1d(block[i])))
        else:
            ts, vs = series.get(stream, (np.zeros(0), []))

            def pick(i, vs=vs, path=path):
                v = vs[i]
                for key in path.split('.'):
                    if isinstance(v, dict) and key not in v:
                        return np.full(1, np.nan)
                    v = v[key] if isinstance(v, dict) else v[int(key)]
                return np.atleast_1d(np.asarray(v, float)).ravel()
            cols.append((spec, ts, pick))
    lines = ['  %8s | %s' % ('t s', ' | '.join(spec for spec, _, _ in cols))]
    for t in times:
        cells = [' '.join('%.3g' % x for x in pick(i)) if (i := _at(ts, t)) is not None else '-' for _spec, ts, pick in cols]
        lines.append('  %8.3f | %s' % (t, ' | '.join(cells)))
    return '\n'.join(lines)


def _state_at(series, lay, t):
    """`(y, fr, a, out)` reconstructed from recorded vector streams at or before `t`, or `None`."""
    from machine.law import latent
    from machine.process import kinematics
    for name in ('modal', 'frame', 'impedance', 'latent'):
        if name not in series:
            return None
    ym, fm, am, lm = (_at(series[n][0], t) for n in ('modal', 'frame', 'impedance', 'latent'))
    if any(idx is None for idx in (ym, fm, am, lm)):
        return None
    fr_fields = (lay.get('frame') or (None,))[0]
    fr = kinematics.unpack_frame(series['frame'][1][fm], fr_fields)
    return series['modal'][1][ym], fr, series['impedance'][1][am], series['latent'][1][lm][latent.STATE:]


def moded(series, lay, times):
    """Modal table (`z/ref` for locomotion modes and worst mode tracking error) across `times`."""
    from machine.process import kinematics
    keys = (('ax', 'axial_compression'), ('lat', 'lateral_shift'), ('drv', 'sagittal_drive'),
            ('pit', 'pitch_attitude'), ('sag', 'sagittal_shift'), ('trn', 'pelvis_turn'), ('rck', 'foot_rock'))
    lines = ['  %8s | %s | worst err' % ('t s', ' | '.join('%-13s' % k for k, _m in keys))]
    for t in times:
        st = _state_at(series, lay, t)
        if st is None:
            lines.append('  %8.3f | -' % t)
            continue
        md = kinematics.modes(*st)
        cells = ['%+5.2f/%+5.2f  ' % (md['z'][m], md['ref'][m]) for _k, m in keys]
        lines.append('  %8.3f | %s | %+.3f %s' % (t, ' | '.join(cells), md['worst_err'], md['worst_mode']))
    return '\n'.join(lines)


def posed(series, lay, times):
    """Way-frame kinematic table (CoM speed, capture point, soles, legs and knees) across `times`."""
    from machine.process import kinematics
    tk_l, kl, _ = _field(series, lay, 'world', 'qpos.left_knee')
    tk_r, kr, _ = _field(series, lay, 'world', 'qpos.right_knee')
    lines = ['  %8s | v_way (al,lt,up)  | xi (al,lt)    | sole_al L/R   | meet_al L/R   | tilt L/R deg | leg L/R m   | knee L/R deg' % 't s']
    for t in times:
        st = _state_at(series, lay, t)
        if st is None:
            lines.append('  %8.3f | -' % t)
            continue
        wy = kinematics.way(*st[:3])
        sl, sr = wy['soles']['left'], wy['soles']['right']
        ll, lr = wy['legs']['left'], wy['legs']['right']
        il, ir = _at(tk_l, t), _at(tk_r, t)
        kn = ('%5.1f %5.1f' % (abs(math.degrees(float(kl[il][0]))), abs(math.degrees(float(kr[ir][0]))))
              if il is not None and ir is not None else '    -     -')
        lines.append('  %8.3f | %+5.2f %+5.2f %+5.2f | %+5.2f %+5.2f   | %+6.3f %+6.3f | %+6.3f %+6.3f | %+5.1f %+5.1f  | %.3f %.3f | %s' % (
            t, wy['v_way_m_s'][0], wy['v_way_m_s'][1], wy['v_way_m_s'][2],
            wy['capture_way_m'][0], wy['capture_way_m'][1],
            sl['place_way_m'][0], sr['place_way_m'][0], sl['meet_way_m'][0], sr['meet_way_m'][0],
            sl['tilt_deg'], sr['tilt_deg'], ll['length_m'], lr['length_m'], kn))
    return '\n'.join(lines)


def told(path, window=None, overview=None, fields=(), modes=False, kinematics=False):
    """The recording at `path` told as text: its blocks, its events, its windows' tables (`window`:
    the one numbered so), the whole a row each `overview` s first where asked; `fields`, `modes` and
    `kinematics`, her layers, modes and way-frame kinematics beside each table."""
    from machine.rig import gate
    gates = gate.read(path)
    series, lay = joined(gates)
    events = sorted(series.get('events', ((), []))[1], key=lambda e: e['t'])
    wins = [g for g in gates if g['meta']['name'] != 'every']
    numbered = {id(g): k + 1 for k, g in enumerate(wins)}  # list.index would compare the blocks' arrays
    span = [min(g['meta']['window'][0] for g in gates), max(g['meta']['window'][1] for g in gates)]
    out = ['%s: %.2f..%.2f s of her, %d blocks: %d windows, the rest 60 Hz' % (
        os.path.basename(path), span[0], span[1], len(gates), len(wins))]
    for g in gates:
        m, tr = g['meta'], g['trailer'] or {}
        what = '60 Hz' if m['name'] == 'every' else 'window %d  %s%s' % (
            numbered[id(g)], m['name'], ' (%s)' % tr['cut'] if tr.get('cut') else '')
        out.append('  %8.3f..%8.3f  %s' % (m['window'][0], m['window'][1], what))
    out.append('events:')
    out.extend('  %8.3f  %s.%s  %s' % (e['t'], e['analyzer'], e['event'], {k: v for k, v in e.items()
               if k not in ('t', 'analyzer', 'event', 'capture')}) for e in events if e['event'] not in QUIET)
    if overview:
        n = int(math.floor((span[1] - span[0]) / overview))
        times = [span[0] + overview * (k + 1) for k in range(n)]
        out += ['', 'overview, a row each %.2f s:' % overview, printed(rows(series, lay, events, times))]
        out += [fielded(series, lay, fields, times)] if fields else []
        out += [moded(series, lay, times)] if modes else []
        out += [posed(series, lay, times)] if kinematics else []
    for k, g in enumerate(wins):
        if window is not None and k + 1 != window:
            continue
        m = g['meta']
        t0, (lo, hi) = m['t'], m['window']
        ev = {a: b for a, b in (m['event'] or {}).items() if a not in ('t', 'analyzer', 'event', 'capture')}
        held = [(h['name'], round(h['t'], 3)) for h in (g['trailer'] or {}).get('held_off') or []]
        times = sorted({t0 + d for d in ROWS_S if lo <= t0 + d <= hi}
                       | {lo + STEP_S * (j + 1) for j in range(int((hi - lo) / STEP_S))} | {hi})
        out += ['', 'window %d: %s at %.3f s %s, %.3f..%.3f%s' % (k + 1, m['name'], t0, ev, lo, hi,
                                                              ', held off %s' % held if held else ''),
                printed(rows(series, lay, events, times, lo), t0)]
        out += [fielded(series, lay, fields, times)] if fields else []
        out += [moded(series, lay, times)] if modes else []
        out += [posed(series, lay, times)] if kinematics else []
    return '\n'.join(out)


def main(argv=None):
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('path', nargs='?', help='a recording (recordings/*.jsonl)')
    parser.add_argument('--last', action='store_true', help='the newest recording')
    parser.add_argument('--window', type=int, help='its window numbered so alone')
    parser.add_argument('--overview', type=float, help='the whole of it a row each this many s first')
    parser.add_argument('--fields', default='', help="her layers' fields beside each table's rows, comma-separated: "
                                                     "impedance.COM_ACC,latent.NOW.Z,read.LOAD,wrench.got.bears")
    parser.add_argument('--modes', action='store_true', help='her measured modes beside references and worst error')
    parser.add_argument('--kinematics', action='store_true', help='her way-frame CoM, capture point, soles, legs and knees')
    args = parser.parse_args(argv)
    found = glob.glob(os.path.join(RECORDINGS, '*.jsonl')) if args.last else []
    path = args.path or (max(found, key=os.path.getmtime) if found else None)
    if path is None:
        parser.error('a recording, or --last')
    print(told(path, args.window, args.overview, tuple(f for f in args.fields.split(',') if f),
               args.modes, args.kinematics))
    return 0


if __name__ == '__main__':
    sys.exit(main())
