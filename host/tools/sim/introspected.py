#!/usr/bin/env python3
"""The robot's law's layers read as it runs: each pass's snapshot by name and its analyzers' events.

    python tools/sim/introspected.py                                # the robot's walk into a turn: events, a row at 4.5 s
    python tools/sim/introspected.py --play 'w3 -3' --at 3.5 4.2    # rows at those seconds
    python tools/sim/introspected.py --scenario stairs_down --at 2.5 3.5
    python tools/sim/introspected.py --errand chair --at 1.6 3.9 9.6 13.2
    python tools/sim/introspected.py --fields kinematics.v_way_m_s modes.z.lateral_shift drives.knees_deg

A player's keys as a play (`player.played`) on a scene (`scenarios.ran`), a named scenario (`--scenario`), or a
page errand (`--errand`, `tools.sim.told`), read by the hook the robot's loop calls
each pass (`laid`) - read only: its code is never wrapped or patched. `introspect.Tap` hands a law pass's snapshot,
each layer named from the robot's code's own layouts as it is first read, in that pass (latent, impedance, wrench,
observer, contacts, modes, kinematics, drives, body); `watch.Watch` turns the snapshots into the robot's analyzers'
events (a stumble, a fall, a spasm, a phase), its own left out (`recording.QUIET`). A row at each `--at` second:
what the robot is doing and each field asked for, a dotted path into the snapshot.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The fields a row shows unless asked others: the robot's speed and its capture point on its way, which soles
#: stand, its surety.
FIELDS = ('kinematics.v_way_m_s', 'kinematics.capture_way_m', 'impedance.ask.STANCE', 'body.surety')


def field(snap, path):
    """The value at `path` ('layer.key..') in `snap`, its numbers to four places."""
    value = snap
    for key in path.split('.'):
        value = value[key]
    if isinstance(value, float):
        return round(value, 4)
    return [round(v, 4) if isinstance(v, float) else v for v in value] if isinstance(value, list) else value


def ran(play, scene, at, fields, scenario=None, errand=None):
    """(a row at each second of `at`, the robot's analyzers' events, when it fell) of `play`, `scenario` or `errand`."""
    from machine.process import introspect, watch
    from tools.sim import scenarios
    from tools.sim.player import played
    from tools.sim.recording import QUIET
    tap, judge, rows, told_ev, due = introspect.Tap(), watch.Watch(), [], [], sorted(at)

    def laid(s):
        snap = tap.passed(s)
        if snap is None:
            return
        told_ev.extend(e for e in judge.step(snap) if e['event'] not in QUIET)
        while due and snap['t'] >= due[0]:
            rows.append(dict(t=round(snap['t'], 3), doing=snap['doing'], **{f: field(snap, f) for f in fields}))
            due.pop(0)

    if errand is not None:
        from tools.sim import told as sim_told
        _line, r = sim_told.ran(errand, laid=laid)
        return rows, told_ev, r.s['fallen_at']
    if scenario is not None:
        return rows, told_ev, scenarios.ran(scenario, laid=laid)['fell_at']
    plan, secs = played(play)
    scenarios.SCENARIOS['introspected'] = (scene, plan, secs, 'stands', None)
    return rows, told_ev, scenarios.ran('introspected', laid=laid)['fell_at']


def main(argv=None):
    p = argparse.ArgumentParser(description=(__doc__ or '').split('\n\n')[0])
    p.add_argument('--play', default='w3 wa2 -3', help="the player's keys, held their seconds (`player.played`)")
    p.add_argument('--scene', default='flat', help='the scene laid round the robot (`scenarios`)')
    p.add_argument('--scenario', default=None, help='a named scenario (`scenarios.SCENARIOS`)')
    p.add_argument('--errand', default=None, help='a named page errand (`tools.sim.told.ERRANDS`)')
    p.add_argument('--at', type=float, nargs='*', default=[4.5], help='the seconds a row is read at')
    p.add_argument('--fields', nargs='*', default=list(FIELDS), help="each a dotted path into the snapshot")
    args = p.parse_args(argv)
    rows, told, fell = ran(args.play, args.scene, args.at, args.fields, args.scenario, args.errand)
    for e in told:
        print('%8.3f  %s.%s  %s' % (e['t'], e['analyzer'], e['event'],
                                    {k: v for k, v in e.items() if k not in ('t', 'analyzer', 'event', 'capture')}))
    for row in rows:
        print(json.dumps(row))
    print(json.dumps({'fell': fell}))
    return 0


if __name__ == '__main__':
    sys.exit(main())
