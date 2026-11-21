"""Sketch work on the relay, side by side: shapes by their loads, sheets drawn, members checked.

    COAXIAL_ROBOT=gynoid_mesomorph_medium_coax python -X utf8 tools/render/batch.py shape check   # every one
    ... batch.py shape:hood check:shoulder-bus                     # these alone
    ... batch.py whole                                             # each case's room, all solid: does it carry?
    ... batch.py sheet --out DIR                                   # every sketch's sheet, an image each in DIR
    ... batch.py --list                                            # what there is

The tests' relay (`tools.dev.focus.relay`) runs the jobs: each a process of its own on a core of its own, booked
in the machine's ledger beside every relay's and container's - a company of agents shares the desk -, OpenBLAS
on one thread, killed with its tree at its time or where the relay is stopped. A job is a case for `topo` to shape
(CASES: the expression that makes it - a variant its keyword arguments) or to read whole (`topo.whole`: its room
all solid, minutes where a shape takes an hour), a sketch's sheet (SHEETS: its `main` with an image's path) or a
sketch's checks (CHECKS: its `checked`); each prints its result as JSON on its last line, and the table tells each
as it ends - the shapes first, the longest. `--json` keeps every result.
"""
import argparse
import json
import os
import sys

if __package__ in (None, ''):
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

#: The cases shaped: {name: the expression that makes it, the sketches' modules by their own names}.
CASES = {
    'trial': 'topo.trial()',
    'hood': 'shoulder_shaped.case()',
    'hood-0.2': 'shoulder_shaped.case(share=0.2)',
    'hood-0.3': 'shoulder_shaped.case(share=0.3)',
    'hood-behind': 'shoulder_shaped.case(up=-1.0)',
    'trunk-yoke': 'trunk_shaped.cases()["yoke"]',
    'trunk-lever': 'trunk_shaped.cases()["lever"]',
    'thigh-arm': 'leg_shaped.cases()["thigh arm"]',
    'thigh-knuckle': 'leg_shaped.cases()["thigh knuckle"]',
    'hip-yoke': 'leg_shaped.cases()["yoke"]',
    'hip-block': 'leg_shaped.cases()["block"]',
    'stator-clamp': 'leg_shaped.cases()["stator clamp"]',
}
#: The sketches whose sheets are drawn; the checks told: {name: the expression that tells them}.
SHEETS = ('shoulder_rolled', 'shoulder_hosed', 'shoulder_mount', 'shoulder_shaped', 'arm_worn', 'trunk_shaped')
CHECKS = {'shoulder-held': 'shoulder_laid.held()', 'shoulder': 'shoulder_mount.checked()',
          'abduction': 'shoulder_rolled.checked()', 'shoulder-bus': 'shoulder_hosed.checked()',
          'neck': 'neck_mount.checked()', 'neck-hose': 'neck_hosed.checked()', 'hip': 'hip_laid.checks()'}
#: Each kind's commit, GB, and time, s: a shape the longest - the hood's room, a kilogram solid on 3 mm voxels, ran
#: past two hours.
KINDS = {'shape': (2.5, 21600.0), 'whole': (2.5, 3600.0), 'sheet': (2.5, 3600.0), 'check': (1.5, 3600.0)}


def _code(kind, name, out):
    """The Python a job runs: its result as JSON on its last line - an expression's module imported by its name."""
    if kind == 'whole':
        return ('import json; from tools.render import topo, %s; w = topo.whole(%s); '
                'print(json.dumps({k: round(v, 3) for k, v in w.items()}))' % (CASES[name].split('.')[0], CASES[name]))
    if kind == 'shape':
        return ('import json; from tools.render import topo, %s; got = topo.shaped(%s, log=print); '
                'layers = got["solid"].transpose(1, 0, 2); print(json.dumps({"grams": round(float(got["grams"]), 1), '
                '"room": round(float(got["room"]), 1), "margin": round(float(got["margin"]), 3), '
                '"level": float(got["level"]), "lost": int(got["lost"]), '
                '"hung": int((layers & ~topo.stood(layers)).sum())}))' % (CASES[name].split('.')[0], CASES[name]))
    if kind == 'sheet':
        image = os.path.join(out, name + '.png')
        return ('import json; from tools.render import %s as s; s.main([%r]); print(json.dumps({"sheet": %r}))'
                % (name, image, image))
    return ('import json; from tools.render import %s; print(json.dumps(%s, default=str))'
            % (CHECKS[name].split('.')[0], CHECKS[name]))


def jobs(asked, out):
    """[focus.Job]: what `asked` names - a kind (every one of it) or 'kind:name' -, the shapes first."""
    from tools.dev import focus
    lists = {'shape': list(CASES), 'whole': list(CASES), 'sheet': list(SHEETS), 'check': list(CHECKS)}
    picked = []
    for word in asked:
        kind, _, name = word.partition(':')
        if kind not in lists or (name and name not in lists[kind]):
            raise SystemExit('no such job: %s (--list tells them)' % word)
        picked += [(kind, n) for n in ([name] if name else lists[kind]) if (kind, n) not in picked]
    picked.sort(key=lambda kn: list(KINDS).index(kn[0]))
    return [focus.Job('%s:%s' % (kind, name), [sys.executable, '-X', 'utf8', '-c', _code(kind, name, out)],
                      KINDS[kind][0], KINDS[kind][1]) for kind, name in picked]


def _told(output):
    """A job's result: the JSON on its output's last such line, else its last lines."""
    for line in reversed(output.strip().splitlines()):
        if line.startswith('{'):
            try:
                return json.loads(line)
            except ValueError:
                break
    return {'tail': output.strip().splitlines()[-3:]}


def main(argv=None):
    from tools.dev import focus
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('asked', nargs='*', help="a kind - shape, whole, sheet, check - or 'kind:name'")
    parser.add_argument('--out', default=os.path.join(os.getcwd(), 'sheets'), help='where the sheets go')
    parser.add_argument('--batons', type=int, default=None, help='the most jobs at once (the relay: every core)')
    parser.add_argument('--json', default=None, help='every result kept here')
    parser.add_argument('--list', action='store_true')
    args = parser.parse_args(argv)
    if args.list or not args.asked:
        for kind, names in (('shape', CASES), ('whole', CASES), ('sheet', SHEETS), ('check', CHECKS)):
            print('%s: %s' % (kind, ', '.join(names)))
        return 0
    os.makedirs(args.out, exist_ok=True)
    work, results, failed = jobs(args.asked, os.path.abspath(args.out)), {}, 0
    print('%d jobs on the relay, %s' % (len(work), os.environ.get('COAXIAL_ROBOT', 'the default build')), flush=True)
    for job, output, code, seconds in focus.relay(work, args.batons):
        told = _told(output)
        results[job.name] = dict(told, seconds=round(seconds, 1), code=code)
        failed += code != 0
        print('%-34s %6.0f s  %s' % (job.name, seconds, json.dumps(told) if code == 0 else
                                       'FAILED (%s): %s' % (code, told)), flush=True)
    if args.json:
        with open(args.json, 'w', encoding='utf-8', newline='') as fh:
            json.dump(results, fh, indent=1)
    print('%d of %d done, %d failed' % (len(results) - failed, len(results), failed))
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
