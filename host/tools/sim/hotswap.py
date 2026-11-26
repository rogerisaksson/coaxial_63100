"""CLI for compiling, live hot-swapping, and checkpoint-iterating LLM-generated cores and engrams."""
import argparse
import json
import os
import sys
from typing import Any

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from machine.process import hotswap, worker
from machine.world import physics


def _parse_floats(raw: str | None):
    if raw is None:
        return None
    return [float(x.strip()) for x in raw.split(',') if x.strip()]


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=(__doc__ or '').splitlines()[0])
    parser.add_argument('--compile', metavar='SPEC', help='compile and validate core/engram JSON file or string')
    parser.add_argument('--attach', type=int, metavar='PORT', help='attach to live `--telemetry PORT` robot')
    parser.add_argument('--swap', metavar='SPEC', help='hot-swap core/engram JSON spec on attached session')
    parser.add_argument('--unload', nargs='?', const='*', metavar='TARGET',
                        help='unload named core/engram slot (or all when omitted)')
    parser.add_argument('--save', metavar='NAME', help='save named checkpoint on attached session')
    parser.add_argument('--reload', metavar='NAME', help='reload named checkpoint on attached session')
    parser.add_argument('--checkpoint-file', metavar='PATH', help='checkpoint `.pkl` file path for save/reload/iterate')
    parser.add_argument('--where', help='comma-separated pelvis coordinates `x,z` or `x,y,z` in meters')
    parser.add_argument('--yaw-deg', type=float, help='robot heading override in degrees on reload')
    parser.add_argument('--zero-vel', action='store_true', help='zero generalized velocities on reload')
    parser.add_argument('--say', help='say command triggered immediately after reload')
    parser.add_argument('--iterate', metavar='SPEC',
                        help='JSON file/string listing candidate bundles to evaluate from a MuJoCo state')
    parser.add_argument('--scene', default='flat', help='floor scenario when running `--iterate` without `--checkpoint-file`')
    parser.add_argument('--settle-steps', type=int, default=300, help='initial settle steps before capturing checkpoint')
    parser.add_argument('--steps', type=int, default=500, help='simulation passes per candidate during `--iterate`')
    parser.add_argument('--jobs', type=int, default=0, help='parallel worker processes for `--iterate` (0 = all cores)')
    args = parser.parse_args(argv)

    if args.compile:
        built = hotswap.compile_batch(args.compile)
        summary = [{'type': r['type'], 'kind': r['kind'], 'name': r['name'],
                    'compiled_c': r.get('compiled_c', False), 'warnings': r.get('warnings', []),
                    'compile_ms': r['compile_ms']} for r in built]
        sys.stdout.write(json.dumps(summary, indent=2) + '\n')
        return 0

    if args.attach is not None:
        payload: dict[str, Any] = {}
        if args.save:
            payload['checkpoint'] = {'action': 'save', 'name': args.save, 'path': args.checkpoint_file}
        elif args.reload or args.checkpoint_file:
            ck: dict[str, Any] = {'action': 'reload', 'zero_vel': bool(args.zero_vel)}
            if args.reload:
                ck['name'] = args.reload
            if args.checkpoint_file:
                ck['path'] = args.checkpoint_file
            if args.where:
                ck['where'] = _parse_floats(args.where)
            if args.yaw_deg is not None:
                ck['yaw_deg'] = args.yaw_deg
            if args.say:
                ck['say'] = args.say
            payload['checkpoint'] = ck
        if args.unload is not None:
            payload['hotswap'] = ({'action': 'unload', 'all': True} if args.unload == '*'
                                  else {'action': 'unload', 'target': args.unload})
        elif args.swap:
            payload['hotswap'] = hotswap.normalize_specs(args.swap)
        if not payload:
            payload['hotswap'] = {'action': 'status'}
        reply = hotswap.remote(args.attach, payload)
        sys.stdout.write(json.dumps(reply, indent=2) + '\n')
        return 0 if reply.get('ok', True) else 1

    if args.iterate:
        if args.iterate.strip().startswith(('[', '{')):
            raw = json.loads(args.iterate)
        else:
            with open(args.iterate, encoding='utf-8') as f:
                raw = json.load(f)
        candidates = raw['candidates'] if isinstance(raw, dict) and 'candidates' in raw else (
            raw if isinstance(raw, list) else [raw])
        prev = (physics.ROOM, physics.CAMERA, physics.SIGHT)
        physics.ROOM = physics.CAMERA = physics.SIGHT = 0.0
        try:
            s = None
            if not args.checkpoint_file:
                s = worker.session()
                if args.scene != 'flat':
                    worker.apply(s, {'scenario': args.scene})
                for _ in range(max(0, args.settle_steps)):
                    worker.stepped(s)
                hotswap.save_state(s, 'base')
            rel_kw: dict[str, Any] = {'zero_vel': bool(args.zero_vel)}
            if args.where:
                rel_kw['where'] = _parse_floats(args.where)
            if args.yaw_deg is not None:
                rel_kw['yaw_deg'] = args.yaw_deg
            if args.say:
                rel_kw['say'] = args.say
            results = hotswap.iterate(candidates, s=s, checkpoint_name='base',
                                      path=args.checkpoint_file, steps=args.steps, jobs=args.jobs, **rel_kw)
            sys.stdout.write(json.dumps(results, indent=2) + '\n')
            return 0
        finally:
            physics.ROOM, physics.CAMERA, physics.SIGHT = prev

    parser.print_help()
    return 2


if __name__ == '__main__':
    sys.exit(main())
